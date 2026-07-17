#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// OPEN REVOLUTION BAND: a partial wrap bounded by two full-height u-iso
// side edges (a castellated barrel wall). The construction is a
// revolution grid FOLLOWED by a boolean cut, never a loft between the
// rims — index-pairing a feature-clustered castellated chain against a
// uniformly sampled circle shears every column diagonally:
//   - nu+1 STRAIGHT columns at fixed azimuths. When a rim is a lone
//     flat full-span edge solved at nu, its contract samples ARE that
//     row (columns end ON rim verts, curve-sampled bit-identically);
//     otherwise a thin transition strip absorbs the count mismatch
//     without re-spacing any border (quads + n-gons, monotone in u).
//   - nv row bands from the sides' solved count; the side columns
//     sample the side curves at uniform steps — their contract.
//   - Contiguous castellation runs (notches, scallops) get their
//     lattice cells deleted up to a local row just past the feature;
//     one ear-clip web per run weaves the run's exact contract samples
//     into the staircase, the cavity opening at the rim.
// Everything is built in w = |v - vCut| space so a band castellated at
// its HIGH rim reuses the same code mirrored (emitted winding flips).
bool meshRevolutionOpenBand(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model,
                            const std::vector<int>& rimEdges,
                            const std::vector<int>& solvedEdge, int faceId,
                            int nu, int nv, const std::vector<int>& sides,
                            MeshBuilder& out,
                            const PinnedEdges* pins,
                            const std::vector<std::vector<int>>*
                                insertWires) {
    if (sides.size() != 2 || surf.IsVClosed() || surf.IsUClosed()) {
        return false;
    }
    // Respect the solve: nu == the plain rim's count is what lets the
    // columns pass THROUGH the rim samples (passPlain/passCut below,
    // a bridge-free 1:1 weld). The old max(3,...) floor broke exactly
    // that on small drum segments — a quarter chamfer ring solved 2/2
    // got 3 columns and a hair-thin hug-row bridge that folded.
    nu = std::max(2, nu);
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double uspan = std::max(1e-12, u1 - u0);
    const double wspan = std::max(1e-12, v1 - v0);

    // Interior cutout wires (a slot/hole through the wall) become
    // boolean-cut boxes: lattice cells they cover are deleted and the
    // cavity is webbed to the wire's exact contract samples, so the
    // columns stay straight and full-height (a cut FOLLOWS the
    // primitive; it never drives rows across it). Boxes must sit
    // strictly inside the band or the cavity would eat a border.
    struct IBox {
        double bu0 = 1e300, bu1 = -1e300;
        double bv0 = 1e300, bv1 = -1e300;
        const std::vector<int>* wire = nullptr;
        int rowLo = -1, rowHi = -1;  // slot-extent row keys
        int colL = -1, colR = -1;    // kept columns bracketing the cut
    };
    std::vector<IBox> iboxes;
    if (insertWires) {
        for (const auto& w : *insertWires) {
            if (w.empty()) return false;
            IBox b;
            b.wire = &w;
            for (int eid : w) {
                if (eid < 1 || eid > model.edges.Extent()) return false;
                const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
                double f2, l2;
                Handle(Geom2d_Curve) pc =
                    BRep_Tool::CurveOnSurface(edge, face, f2, l2);
                if (pc.IsNull()) return false;
                const int n = std::max(
                    16, eid < int(solvedEdge.size()) ? solvedEdge[eid] : 1);
                for (int k = 0; k <= n; ++k) {
                    gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * k / double(n));
                    b.bu0 = std::min(b.bu0, uv.X());
                    b.bu1 = std::max(b.bu1, uv.X());
                    b.bv0 = std::min(b.bv0, uv.Y());
                    b.bv1 = std::max(b.bv1, uv.Y());
                }
            }
            if (!(b.bu1 > b.bu0) || !(b.bv1 > b.bv0)) return false;
            if (b.bu0 <= u0 + 0.02 * uspan || b.bu1 >= u1 - 0.02 * uspan ||
                b.bv0 <= v0 + 0.02 * wspan || b.bv1 >= v1 - 0.02 * wspan) {
                return false;  // not strictly interior
            }
            iboxes.push_back(b);
        }
    }

    // The outer wire's cycle with face-local edge instances (the model
    // map's copies lose the orientation the sample direction needs).
    // With interior cutout wires present, the outer wire is the one
    // carrying the two side edges.
    std::vector<std::pair<int, TopoDS_Edge>> order;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        std::vector<std::pair<int, TopoDS_Edge>> cand;
        bool hasSide = false;
        for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face);
             we.More(); we.Next()) {
            if (BRep_Tool::Degenerated(we.Current())) continue;
            const int eid = model.edges.FindIndex(we.Current());
            if (eid < 1) continue;
            cand.push_back({eid, we.Current()});
            if (eid == sides[0] || eid == sides[1]) hasSide = true;
        }
        if (hasSide) {
            order = std::move(cand);
            break;
        }
    }
    const int sA = sides[0], sB = sides[1];
    int ia = -1, ib = -1;
    for (size_t k = 0; k < order.size(); ++k) {
        if (order[k].first == sA) ia = int(k);
        if (order[k].first == sB) ib = int(k);
    }
    // A sloppy wire the explorer walked short would blend the chains —
    // refuse and let the face take the contract floor.
    const std::set<int> want(rimEdges.begin(), rimEdges.end());
    if (ia < 0 || ib < 0 || order.size() != want.size() + 2) return false;
    auto runOf = [&](int from, int to) {
        std::vector<std::pair<int, TopoDS_Edge>> r;
        const int n = int(order.size());
        for (int k = (from + 1) % n; k != to; k = (k + 1) % n) {
            r.push_back(order[k]);
        }
        return r;
    };
    std::vector<std::pair<int, TopoDS_Edge>> runs[2] = {runOf(ia, ib),
                                                        runOf(ib, ia)};
    if (runs[0].empty() || runs[1].empty()) return false;
    for (const auto& r : runs) {
        for (const auto& [eid, e] : r) {
            if (!want.count(eid)) return false;
        }
    }

    // Contract samples per edge, chained in wire order. Each edge keeps
    // its identity (a piece) so castellation runs can be told from the
    // base arcs that hug the rim level.
    struct BandPt {
        double u, v;
        gp_Pnt p;
    };
    struct Piece {
        int eid = 0;
        bool hug = false;
        double bu0 = 1e300, bu1 = -1e300;  // pcurve box
        double bv0 = 1e300, bv1 = -1e300;
        std::vector<BandPt> pts;
    };
    auto samplePieces = [&](const std::vector<std::pair<int, TopoDS_Edge>>&
                                run,
                            std::vector<Piece>& pieces) {
        for (const auto& [eid, edge] : run) {
            double f2, l2, f3, l3;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f2, l2);
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
            if (pc.IsNull() || c3.IsNull()) return false;
            int n = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
            if (n < 1) n = 1;
            const bool rev = edge.Orientation() == TopAbs_REVERSED;
            Piece piece;
            piece.eid = eid;
            // A pinned far-rim arc carries the band's column azimuths, so
            // the rim samples land ON the columns and the bottom transition
            // strip collapses to quads (columns run straight to the rim).
            for (double t : edgeSampleFractions(eid, n, 0.0, rev,
                                                /*includeLast=*/true, pins,
                                                &model)) {
                gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
                piece.pts.push_back(
                    {uv.X(), uv.Y(), c3->Value(f3 + (l3 - f3) * t)});
            }
            // The box bounds the true curve, not just the samples: the
            // deletion pads and web guards measure against it.
            for (int k = 0; k <= 16; ++k) {
                gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * k / 16.0);
                piece.bu0 = std::min(piece.bu0, uv.X());
                piece.bu1 = std::max(piece.bu1, uv.X());
                piece.bv0 = std::min(piece.bv0, uv.Y());
                piece.bv1 = std::max(piece.bv1, uv.Y());
            }
            pieces.push_back(std::move(piece));
        }
        return true;
    };
    std::vector<Piece> chainP[2];
    if (!samplePieces(runs[0], chainP[0]) ||
        !samplePieces(runs[1], chainP[1])) {
        return false;
    }
    // Rim assignment by mean v (castellation pulls the mean inward but
    // never past the middle on anything loftable).
    double rimV[2];
    for (int c = 0; c < 2; ++c) {
        double mean = 0;
        int cnt = 0;
        for (const Piece& p : chainP[c]) {
            for (const BandPt& b : p.pts) {
                mean += b.v;
                ++cnt;
            }
        }
        mean /= std::max(1, cnt);
        rimV[c] = std::abs(mean - v0) <= std::abs(mean - v1) ? v0 : v1;
    }
    if (rimV[0] == rimV[1]) {
        dbg("openband face %d: chains claim one rim", faceId);
        return false;
    }
    // hug = the piece stays at its chain's rim level; anything that
    // leaves the rim is castellation.
    bool feature[2] = {false, false};
    for (int c = 0; c < 2; ++c) {
        for (Piece& p : chainP[c]) {
            p.hug = std::abs(p.bv0 - rimV[c]) < 0.02 * wspan &&
                    std::abs(p.bv1 - rimV[c]) < 0.02 * wspan;
            feature[c] |= !p.hug;
        }
    }
    // A chain whose castellation reaches its ends (a wavy gear-flank
    // rim, no base arc against the side) cannot cut regions — its
    // strip row rises past the whole wave instead (WAVE mode). Only
    // strictly interior runs cut. Two chains both wanting regions
    // cannot share one lattice.
    bool chainTouch[2], interior[2];
    double chainDev[2] = {0, 0};
    for (int c = 0; c < 2; ++c) {
        chainTouch[c] = !chainP[c].front().hug || !chainP[c].back().hug;
        interior[c] = feature[c] && !chainTouch[c];
        for (const Piece& p : chainP[c]) {
            if (p.hug) continue;
            chainDev[c] = std::max({chainDev[c], std::abs(p.bv0 - rimV[c]),
                                    std::abs(p.bv1 - rimV[c])});
        }
        // A wave the strip cannot clear without eating the band.
        if (feature[c] && chainTouch[c] && chainDev[c] > 0.35 * wspan) {
            dbg("openband face %d: side-touching castellation too tall",
                faceId);
            return false;
        }
    }
    if (interior[0] && interior[1]) {
        dbg("openband face %d: both rims castellated", faceId);
        return false;
    }
    // The CUT chain sits at w=0; the PLAIN chain at w=wspan.
    const int cutIdx = interior[0]     ? 0
                       : interior[1]   ? 1
                       : feature[0]    ? 0
                       : feature[1]    ? 1
                                       : (rimV[0] == v0 ? 0 : 1);
    const int plainIdx = 1 - cutIdx;
    const double vCut = rimV[cutIdx];
    const double sign = vCut == v0 ? 1.0 : -1.0;
    auto wOf = [&](double v) { return sign * (v - vCut); };
    auto vOf = [&](double w) { return vCut + sign * w; };

    // Flatten each chain ascending in u, joints deduplicated.
    struct ChainFlat {
        std::vector<BandPt> s;
        std::vector<int> pieceFirst, pieceLast;  // sample index ranges
        std::vector<bool> hug;
    };
    auto flatten = [&](std::vector<Piece>& pieces, ChainFlat& cf) {
        const double uF = pieces.front().pts.front().u;
        const double uL = pieces.back().pts.back().u;
        if (uL < uF) {
            std::reverse(pieces.begin(), pieces.end());
            for (Piece& p : pieces) {
                std::reverse(p.pts.begin(), p.pts.end());
            }
        }
        for (Piece& p : pieces) {
            cf.pieceFirst.push_back(
                cf.s.empty() ? 0 : int(cf.s.size()) - 1);
            const size_t skip = cf.s.empty() ? 0 : 1;
            cf.s.insert(cf.s.end(), p.pts.begin() + skip, p.pts.end());
            cf.pieceLast.push_back(int(cf.s.size()) - 1);
            cf.hug.push_back(p.hug);
        }
        return cf.s.size() >= 2;
    };
    ChainFlat cut, plain;
    if (!flatten(chainP[cutIdx], cut) || !flatten(chainP[plainIdx], plain)) {
        return false;
    }

    // Column azimuths. A plain rim solved at exactly nu passes through:
    // its samples ARE the columns' ends, so anchor the columns at the
    // sample azimuths (uniform for circles by construction).
    const bool passPlain =
        plain.hug.size() == 1 && int(plain.s.size()) == nu + 1 &&
        plain.s.front().u <= u0 + 0.02 * uspan &&
        plain.s.back().u >= u1 - 0.02 * uspan;
    std::vector<double> uk(nu + 1);
    for (int k = 0; k <= nu; ++k) {
        uk[k] = passPlain ? plain.s[k].u : u0 + k * uspan / nu;
        if (k && uk[k] <= uk[k - 1]) {
            dbg("openband face %d: non-monotone rim", faceId);
            return false;
        }
    }
    const int M = int(cut.s.size()) - 1;
    bool passCut = !feature[cutIdx] && cut.hug.size() == 1 && M == nu;
    if (passCut) {
        for (int k = 0; k <= nu && passCut; ++k) {
            passCut = std::abs(cut.s[k].u - uk[k]) < 1e-7 * uspan;
        }
    }

    // Castellation runs -> regions: column-aligned boxes whose lattice
    // cells are never emitted; the web weaves the run's samples.
    struct Region {
        int iA, iB;    // cut-chain sample range, corner samples included
        int colL, colR;  // bounding KEPT columns
        double wTop = 0;
        double rowfW = 0;
        int rowKey = -1;
        double slotU0 = 0, slotU1 = 0;  // the notch's true pcurve u-span
    };
    std::vector<Region> regions;
    // WAVE mode: castellation the lattice cannot cut (it reaches the
    // sides) is absorbed whole by a strip row above it instead.
    bool waveCut = feature[cutIdx] && !interior[cutIdx];
    if (interior[cutIdx]) {
        size_t p = 0;
        while (p < cut.hug.size()) {
            if (cut.hug[p]) {
                ++p;
                continue;
            }
            Region r;
            r.iA = cut.pieceFirst[p];
            double bu0 = 1e300, bu1 = -1e300;
            while (p < cut.hug.size() && !cut.hug[p]) {
                bu0 = std::min(bu0, chainP[cutIdx][p].bu0);
                bu1 = std::max(bu1, chainP[cutIdx][p].bu1);
                const double wA = wOf(chainP[cutIdx][p].bv0);
                const double wB = wOf(chainP[cutIdx][p].bv1);
                r.wTop = std::max({r.wTop, wA, wB});
                r.iB = cut.pieceLast[p];
                ++p;
            }
            // Bounding kept columns, sliver cells pushed out with the
            // notch (a wall grazing a column would web a needle).
            int cL = int(std::upper_bound(uk.begin(), uk.end(), bu0) -
                         uk.begin()) -
                     1;
            cL = std::clamp(cL, 0, nu - 1);
            if (cL > 0 && bu0 - uk[cL] < 0.3 * (uk[cL + 1] - uk[cL])) --cL;
            int cR = int(std::lower_bound(uk.begin(), uk.end(), bu1) -
                         uk.begin());
            cR = std::clamp(cR, 1, nu);
            if (cR < nu && uk[cR] - bu1 < 0.3 * (uk[cR] - uk[cR - 1])) ++cR;
            r.colL = cL;
            r.colR = cR;
            r.slotU0 = bu0;
            r.slotU1 = bu1;
            regions.push_back(r);
        }
    }
    if (!regions.empty()) {
        // Merge runs whose DELETED cells overlap; the merged web weaves
        // the in-between samples too. Runs that merely touch at one
        // column stay separate — a merged web would ear-clip long
        // chords across the wrap (observed: a fold spanning a scallop
        // AND the next notch) — and the gap arc fans from the shared
        // column instead of stripping.
        std::vector<Region> merged;
        for (const Region& r : regions) {
            if (!merged.empty() && r.colL < merged.back().colR) {
                merged.back().colR = std::max(merged.back().colR, r.colR);
                merged.back().iB = r.iB;
                merged.back().wTop = std::max(merged.back().wTop, r.wTop);
                merged.back().slotU1 = std::max(merged.back().slotU1, r.slotU1);
            } else {
                merged.push_back(r);
            }
        }
        regions = std::move(merged);
        for (const Region& r : regions) {
            // A region may reach the first/last interior column (the
            // flanking arc then fans from it) but never a side column:
            // sides carry the row contract, not staircases. A coarse
            // lattice that leaves no room falls back to WAVE mode.
            if (r.colL < 1 || r.colR > nu - 1) {
                dbg("openband face %d: regions crowd the sides -> wave",
                    faceId);
                regions.clear();
                waveCut = true;
                break;
            }
        }
    }
    if (waveCut && chainDev[cutIdx] > 0.35 * wspan) {
        dbg("openband face %d: wave too tall to absorb (%.0f%%)", faceId,
            100.0 * chainDev[cutIdx] / wspan);
        return false;
    }

    // Sides: rows are their solved count, sampled at uniform curve
    // steps — the border contract with the faces across the band ends.
    auto sideAt = [&](int eid) {
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(
            TopoDS::Edge(model.edges(eid)), face, f, l);
        return pc.IsNull() ? 1e300 : pc->Value(0.5 * (f + l)).X();
    };
    const double uSa = sideAt(sA), uSb = sideAt(sB);
    if (uSa > 1e299 || uSb > 1e299) return false;
    const int sideLo = uSa <= uSb ? sA : sB;
    const int sideHi = uSa <= uSb ? sB : sA;
    auto sideCount = [&](int eid) {
        const int n = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
        return n < 1 ? std::max(1, nv) : n;
    };
    if (sideCount(sideLo) != sideCount(sideHi)) {
        dbg("openband face %d: side counts %d/%d differ", faceId,
            sideCount(sideLo), sideCount(sideHi));
        return false;
    }
    nv = sideCount(sideLo);

    // Partial passCut ("rim-grounded"): the fillet flow-through usually pins
    // the CUT rim's CLEAN arcs (between notches) to the column azimuths, so
    // away from a notch the columns already land on the rim. When every
    // non-notch column has an aligned cut sample, the bottom transition strip
    // is a redundant ring wrapped across the whole primitive — exactly the
    // user's "a cut driving edges across the primitive". Ground the columns
    // straight on the rim and drop the strip; only the notch mouths keep their
    // local webs, so away from a notch every column is one unbroken span.
    // A column is IN a notch when its azimuth falls in that notch's true u-span
    // (slotU0..slotU1) — those never ground, they belong to the web. Every
    // OTHER column is on the clean rim and grounds onto its nearest CLEAN-RIM
    // (hug) sample. Using the true span (not a distance tolerance) is what lets
    // the match be loose enough to absorb the fillet flow-through's rounding
    // without a column snapping onto the notch-edge sample next door.
    auto notchInterior = [&](int c) {
        const double m = 0.1 * uspan / std::max(3, nu);
        for (const Region& r : regions) {
            if (uk[c] > r.slotU0 - m && uk[c] < r.slotU1 + m) return true;
        }
        return false;
    };
    std::vector<int> colToCut(nu + 1, -1);
    {
        std::vector<char> cutHug(cut.s.size(), 0);
        for (size_t p = 0; p < cut.hug.size(); ++p) {
            if (!cut.hug[p]) continue;
            for (int i = cut.pieceFirst[p]; i <= cut.pieceLast[p]; ++i) {
                if (i >= 0 && i < int(cutHug.size())) cutHug[i] = 1;
            }
        }
        const double uTol = 0.3 * uspan / std::max(3, nu);
        for (int c = 1; c < nu; ++c) {
            if (notchInterior(c)) continue;
            double best = uTol;
            for (size_t i = 0; i < cut.s.size(); ++i) {
                if (!cutHug[i]) continue;
                const double d = std::abs(cut.s[i].u - uk[c]);
                if (d < best) {
                    best = d;
                    colToCut[c] = int(i);
                }
            }
        }
    }
    auto colInterior = [&](int c) {
        for (const Region& r : regions) {
            if (r.colL < c && c < r.colR) return true;
        }
        return false;
    };
    bool rimGrounded = !passCut && !waveCut && !getenv("WEFT_NO_GROUND");
    for (int c = 1; c < nu && rimGrounded; ++c) {
        if (!colInterior(c) && colToCut[c] < 0) rimGrounded = false;
    }
    if (rimGrounded) {
        // The sliver-expansion pulled some region boundaries onto GROUNDED
        // columns; tighten each region back off them so it bounds only the
        // truly-cut (un-grounded) interior, and the grounded columns run
        // full-height to the rim instead of being webbed as notch interior.
        for (Region& r : regions) {
            while (r.colL + 1 < r.colR && colToCut[r.colL + 1] >= 0) ++r.colL;
            while (r.colR - 1 > r.colL && colToCut[r.colR - 1] >= 0) --r.colR;
        }
    }

    // Row table (w space). Feature rows sit just past each castellation
    // top; the strip rows hug the rims so the columns stay straight for
    // (nearly) the whole height.
    const bool cutStrip = !passCut && !rimGrounded;
    const bool plainStrip = !passPlain;
    double minRowf = 1e300;
    for (Region& r : regions) {
        r.rowfW = r.wTop + std::max(0.04 * r.wTop, 0.005 * wspan);
        minRowf = std::min(minRowf, r.rowfW);
    }
    double wBot = 0.0;
    if (cutStrip) {
        wBot = std::min({0.25 * wspan / nv, 0.05 * wspan, 0.4 * minRowf});
        if (waveCut) {
            // The strip row must clear the whole wave.
            wBot = std::max(wBot,
                            1.04 * chainDev[cutIdx] + 0.005 * wspan);
        }
        if (wBot < 1e-3 * wspan) {
            dbg("openband face %d: no room for the rim strip", faceId);
            return false;
        }
    }
    double wTopRow = wspan;
    if (plainStrip) {
        double clear = std::min(0.25 * wspan / nv, 0.05 * wspan);
        if (feature[plainIdx]) {  // wavy far rim: clear its wave too
            clear = std::max(clear,
                             1.04 * chainDev[plainIdx] + 0.005 * wspan);
        }
        wTopRow = wspan - clear;
    }
    if (wBot > wTopRow - 0.05 * wspan) {
        dbg("openband face %d: strip rows collide", faceId);
        return false;
    }
    for (const Region& r : regions) {
        if (r.rowfW > wTopRow - 0.02 * wspan) {
            dbg("openband face %d: castellation reaches the far rim",
                faceId);
            return false;  // reaches rim
        }
        if (r.rowfW < wBot + 0.01 * wspan) {
            dbg("openband face %d: castellation under the strip row",
                faceId);
            return false;  // under strip
        }
    }
    std::vector<double> rowW;
    auto addRow = [&](double w) {
        for (size_t i = 0; i < rowW.size(); ++i) {
            if (std::abs(rowW[i] - w) < 0.008 * wspan) return int(i);
        }
        rowW.push_back(w);
        return int(rowW.size()) - 1;
    };
    const int keyBot = addRow(wBot);
    const int keyTop = addRow(wTopRow);
    if (keyBot == keyTop) return false;
    std::vector<int> keyAx;
    for (int j = 1; j < nv; ++j) keyAx.push_back(addRow(j * wspan / nv));
    for (Region& r : regions) r.rowKey = addRow(r.rowfW);
    // Interior cutouts: rows exactly at each box's v-extents (carried
    // only by the columns the box touches — no full-width band across
    // the primitive), and the kept columns bracketing the cut. The cut
    // must stay clear of the rim strips, the sides, and any
    // castellation region — anything more entangled falls back.
    if (!iboxes.empty() && (!regions.empty() || waveCut)) return false;
    for (IBox& b : iboxes) {
        const double wA = wOf(b.bv0), wB = wOf(b.bv1);
        b.rowLo = addRow(std::min(wA, wB));
        b.rowHi = addRow(std::max(wA, wB));
        if (b.rowLo == b.rowHi) return false;
        if (rowW[b.rowLo] < rowW[keyBot] - 1e-12 ||
            rowW[b.rowHi] > rowW[keyTop] + 1e-12) {
            return false;  // cut reaches into a rim strip
        }
        b.colL = -1;
        for (int c = 0; c < nu; ++c) {
            const bool covers = uk[c + 1] > b.bu0 + 1e-12 * uspan &&
                                uk[c] < b.bu1 - 1e-12 * uspan;
            if (covers && b.colL < 0) b.colL = c;
            if (covers) b.colR = c + 1;
        }
        if (b.colL < 1 || b.colR > nu - 1) return false;
    }

    // Per-column row keys. Columns strictly inside a region start at its
    // feature row (the cells below are the boolean cut); its bounding
    // columns carry the feature row as an extra vertex their outward
    // cells absorb as n-gons.
    std::vector<std::vector<int>> colKeys(nu + 1);
    for (int c = 1; c < nu; ++c) {
        int floorKey = keyBot;
        // Region boundary columns suppress axial rows under their
        // feature row when the rows form a LONG comb: collinear
        // staircase verts dead-end the web's ear clip into zero-area
        // slivers (observed at axial=100), and the outward cell absorbs
        // the whole span as one n-gon instead. SHORT combs stay — the
        // side bands anchor on their natural axial rows and the web
        // staircase handles a few steps fine.
        double axFloor = rowW[keyBot];
        double axCand = axFloor;
        for (const Region& r : regions) {
            if (r.colL < c && c < r.colR) floorKey = r.rowKey;
            if (c == r.colL || c == r.colR) {
                axCand = std::max(axCand, rowW[r.rowKey]);
            }
        }
        if (axCand > axFloor) {
            int comb = 0;
            for (int k : keyAx) {
                if (rowW[k] > rowW[keyBot] + 1e-12 &&
                    rowW[k] < axCand - 1e-12) {
                    ++comb;
                }
            }
            if (comb > 8) axFloor = axCand;
        }
        std::vector<int> ks{floorKey, keyTop};
        for (int k : keyAx) {
            if (rowW[k] > rowW[floorKey] + 1e-12 &&
                rowW[k] > axFloor + 1e-12 &&
                rowW[k] < rowW[keyTop] - 1e-12) {
                ks.push_back(k);
            }
        }
        for (const Region& r : regions) {
            if (c == r.colL || c == r.colR) ks.push_back(r.rowKey);
        }
        // Columns touched by an interior cutout carry its extent rows;
        // the first column outside the cut absorbs them as n-gon
        // corners — the local collar.
        for (const IBox& b : iboxes) {
            if (c >= b.colL && c <= b.colR) {
                ks.push_back(b.rowLo);
                ks.push_back(b.rowHi);
            }
        }
        std::sort(ks.begin(), ks.end(),
                  [&](int a, int b) { return rowW[a] < rowW[b]; });
        ks.erase(std::unique(ks.begin(), ks.end()), ks.end());
        colKeys[c] = std::move(ks);
    }

    // Build locally; a failed web must leave nothing emitted.
    PolyMesh local;
    MeshBuilder wb(local);
    const bool flip = (face.Orientation() == TopAbs_REVERSED) ^ (sign < 0);
    auto chainIds = [&](const ChainFlat& cf) {
        std::vector<uint32_t> ids(cf.s.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            ids[i] = wb.addVertex(cf.s[i].p,
                                  {faceId, cf.s[i].u, cf.s[i].v});
        }
        return ids;
    };
    const std::vector<uint32_t> cutIds = chainIds(cut);
    const std::vector<uint32_t> plainIds = chainIds(plain);
    std::vector<std::map<int, uint32_t>> vid(nu + 1);
    for (int c = 1; c < nu; ++c) {
        for (int key : colKeys[c]) {
            if (key == keyTop && passPlain) {
                vid[c][key] = plainIds[c];
            } else if (key == keyBot && passCut) {
                vid[c][key] = cutIds[c];
            } else if (key == keyBot && rimGrounded && colToCut[c] >= 0) {
                // Reuse the aligned cut-rim sample so the column welds to the
                // rim with no strip and no duplicate vertex.
                vid[c][key] = cutIds[colToCut[c]];
            } else {
                const double vv = vOf(rowW[key]);
                vid[c][key] =
                    wb.addVertex(surf.Value(uk[c], vv), {faceId, uk[c], vv});
            }
        }
    }
    // Side columns: uniform curve steps, corner verts shared with the
    // chains so the band welds to itself without tolerance games.
    auto sampleSide = [&](int eid, std::vector<uint32_t>& ids,
                          std::vector<double>& ws, uint32_t idW0,
                          uint32_t idW1) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        double f2, l2, f3, l3;
        Handle(Geom2d_Curve) pc =
            BRep_Tool::CurveOnSurface(edge, face, f2, l2);
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        if (pc.IsNull() || c3.IsNull()) return false;
        // Row j must land on sample j whichever way the curve runs; the
        // sample SET {i/n} is direction-independent, so the contract
        // holds either way.
        const bool up = wOf(pc->Value(f2).Y()) <= wOf(pc->Value(l2).Y());
        ids.resize(nv + 1);
        ws.assign(nv + 1, 0.0);
        ws[nv] = wspan;
        for (int j = 0; j <= nv; ++j) {
            if (j == 0) {
                ids[j] = idW0;
                continue;
            }
            if (j == nv) {
                ids[j] = idW1;
                continue;
            }
            const double t = up ? double(j) / nv : 1.0 - double(j) / nv;
            gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
            ws[j] = wOf(uv.Y());
            ids[j] = wb.addVertex(c3->Value(f3 + (l3 - f3) * t),
                                  {faceId, uv.X(), uv.Y()});
        }
        return true;
    };
    std::vector<uint32_t> sideLoIds, sideHiIds;
    std::vector<double> sideLoW, sideHiW;
    if (!sampleSide(sideLo, sideLoIds, sideLoW, cutIds.front(),
                    plainIds.front()) ||
        !sampleSide(sideHi, sideHiIds, sideHiW, cutIds.back(),
                    plainIds.back())) {
        return false;
    }

    auto emitRing = [&](std::vector<uint32_t> ring) {
        ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
        while (ring.size() > 1 && ring.front() == ring.back()) {
            ring.pop_back();
        }
        if (ring.size() < 3) return;
        wb.addPolygon(std::move(ring), faceId, flip);
    };

    // Lattice cells: per column pair, bands at the rows BOTH columns
    // carry; one-sided feature rows ride along as extra ring verts (the
    // n-gon absorbers). Cells inside a region's box below its feature
    // row simply never exist — that is the boolean cut. Cells covered
    // by an interior cutout box are cut the same way; the cavity webs
    // to the wire's contract samples afterwards.
    auto cellCut = [&](int c, double wA, double wB) {
        for (const IBox& b : iboxes) {
            if (c >= b.colL && c + 1 <= b.colR &&
                wA >= rowW[b.rowLo] - 1e-12 &&
                wB <= rowW[b.rowHi] + 1e-12) {
                return true;
            }
        }
        return false;
    };
    for (int c = 1; c + 1 < nu; ++c) {
        const std::vector<int>& L = colKeys[c];
        const std::vector<int>& R = colKeys[c + 1];
        std::vector<int> common;
        for (int k : L) {
            if (std::find(R.begin(), R.end(), k) != R.end()) {
                common.push_back(k);
            }
        }
        for (size_t b = 0; b + 1 < common.size(); ++b) {
            const double wA = rowW[common[b]], wB = rowW[common[b + 1]];
            if (cellCut(c, wA, wB)) continue;
            std::vector<uint32_t> ring{vid[c][common[b]],
                                       vid[c + 1][common[b]]};
            for (int k : R) {
                if (rowW[k] > wA + 1e-12 && rowW[k] < wB - 1e-12) {
                    ring.push_back(vid[c + 1][k]);
                }
            }
            ring.push_back(vid[c + 1][common[b + 1]]);
            ring.push_back(vid[c][common[b + 1]]);
            for (auto it = L.rbegin(); it != L.rend(); ++it) {
                if (rowW[*it] > wA + 1e-12 && rowW[*it] < wB - 1e-12) {
                    ring.push_back(vid[c][*it]);
                }
            }
            emitRing(std::move(ring));
        }
    }
    // Side cells: one band per side segment (the side's contract steps),
    // the inner column's rows absorbed as ring verts. The bottom/top
    // bands close through the strip rows' diagonals.
    {
        // Each side sample anchors on the inner column's nearest EXISTING
        // key (monotone), and consecutive bands SHARE their boundary
        // anchor — so the inner column's vertical edges are covered
        // gapless whatever keys the column actually carries. Anchoring
        // on nominal axial heights left holes: a region boundary column
        // suppresses axial rows under its feature row, two bands then
        // met at different keys, and the segment between them faced the
        // notch web on one side and nothing on the other (observed at
        // radial 12 x axial 3).
        auto sideAnchors = [&](const std::vector<int>& ks,
                               const std::vector<double>& ws) {
            std::vector<int> A(nv + 1);
            A[0] = 0;
            A[nv] = int(ks.size()) - 1;
            for (int j = 1; j < nv; ++j) {
                int best = A[j - 1];
                double bd = std::abs(rowW[ks[best]] - ws[j]);
                for (int k = A[j - 1] + 1; k + 1 < int(ks.size()); ++k) {
                    const double d = std::abs(rowW[ks[k]] - ws[j]);
                    if (d < bd) {
                        bd = d;
                        best = k;
                    }
                }
                A[j] = best;
            }
            return A;
        };
        const std::vector<int> anchL = sideAnchors(colKeys[1], sideLoW);
        const std::vector<int> anchR =
            sideAnchors(colKeys[nu - 1], sideHiW);
        for (int j = 0; j < nv; ++j) {
            {
                std::vector<uint32_t> ring{sideLoIds[j]};
                for (int k = anchL[j]; k <= anchL[j + 1]; ++k) {
                    ring.push_back(vid[1][colKeys[1][k]]);
                }
                ring.push_back(sideLoIds[j + 1]);
                emitRing(std::move(ring));
            }
            {
                std::vector<uint32_t> ring{
                    vid[nu - 1][colKeys[nu - 1][anchR[j]]], sideHiIds[j],
                    sideHiIds[j + 1]};
                for (int k = anchR[j + 1]; k > anchR[j]; --k) {
                    ring.push_back(vid[nu - 1][colKeys[nu - 1][k]]);
                }
                emitRing(std::move(ring));
            }
        }
    }

    // Transition strip: natural border samples (low w side) bridged to
    // the first lattice row by a monotone u map — quads where the
    // counts advance together, an n-gon absorbing each extra point.
    // Low rail forward, high rail backward = lattice winding.
    auto emitStrip = [&](const std::vector<uint32_t>& lowIds,
                         const std::vector<double>& lowU,
                         const std::vector<uint32_t>& highIds,
                         const std::vector<double>& highU) {
        const int nL = int(lowIds.size()) - 1, nH = int(highIds.size()) - 1;
        if (nL < 1 || nH < 1) return false;
        const bool lowSparse = nL <= nH;
        const std::vector<uint32_t>& S = lowSparse ? lowIds : highIds;
        const std::vector<double>& sU = lowSparse ? lowU : highU;
        const std::vector<uint32_t>& D = lowSparse ? highIds : lowIds;
        const std::vector<double>& dU = lowSparse ? highU : lowU;
        const int m = int(S.size()) - 1, n = int(D.size()) - 1;
        std::vector<int> mp(m + 1);
        mp[0] = 0;
        mp[m] = n;
        for (int k = 1; k < m; ++k) {
            int j = mp[k - 1];
            while (j + 1 < n && std::abs(dU[j + 1] - sU[k]) <=
                                    std::abs(dU[j] - sU[k])) {
                ++j;
            }
            mp[k] = j;
        }
        for (int k = 0; k < m; ++k) {
            std::vector<uint32_t> ring;
            if (lowSparse) {
                ring = {S[k], S[k + 1]};
                for (int t = mp[k + 1]; t >= mp[k]; --t) {
                    ring.push_back(D[t]);
                }
            } else {
                for (int t = mp[k]; t <= mp[k + 1]; ++t) {
                    ring.push_back(D[t]);
                }
                ring.push_back(S[k + 1]);
                ring.push_back(S[k]);
            }
            emitRing(std::move(ring));
        }
        return true;
    };
    if (cutStrip) {
        // One strip piece per base-arc span between regions; the pieces'
        // end diagonals close against the side cells and the web rings.
        struct Gap {
            int i0, i1, c0, c1;
        };
        std::vector<Gap> gaps;
        int i0 = 0, c0 = 1;
        for (const Region& r : regions) {
            gaps.push_back({i0, r.iA, c0, r.colL});
            i0 = r.iB;
            c0 = r.colR;
        }
        gaps.push_back({i0, M, c0, nu - 1});
        for (const Gap& g : gaps) {
            if (g.i1 <= g.i0 || g.c1 < g.c0) {
                dbg("openband face %d: degenerate strip gap", faceId);
                return false;
            }
            if (g.c1 == g.c0) {
                // Two regions touching at one column: only one strip-row
                // vertex sits above the base arc, so the arc cannot ladder
                // between columns. Gather it as ONE grouped n-gon fanned
                // from that vertex instead of a tri fan — same boundary,
                // one clean polygon rather than a run of triangles.
                std::vector<uint32_t> ring;
                for (int i = g.i0; i <= g.i1; ++i) ring.push_back(cutIds[i]);
                ring.push_back(vid[g.c0][keyBot]);
                emitRing(std::move(ring));
                continue;
            }
            std::vector<uint32_t> lowIds, highIds;
            std::vector<double> lowU, highU;
            for (int i = g.i0; i <= g.i1; ++i) {
                lowIds.push_back(cutIds[i]);
                lowU.push_back(cut.s[i].u);
            }
            for (int c = g.c0; c <= g.c1; ++c) {
                highIds.push_back(vid[c][keyBot]);
                highU.push_back(uk[c]);
            }
            if (!emitStrip(lowIds, lowU, highIds, highU)) return false;
        }
    }
    if (plainStrip) {
        std::vector<uint32_t> lowIds, highIds;
        std::vector<double> lowU, highU;
        for (int c = 1; c < nu; ++c) {
            lowIds.push_back(vid[c][keyTop]);
            lowU.push_back(uk[c]);
        }
        for (size_t i = 0; i < plain.s.size(); ++i) {
            highIds.push_back(plainIds[i]);
            highU.push_back(plain.s[i].u);
        }
        if (!emitStrip(lowIds, lowU, highIds, highU)) return false;
    }

    // Notch webs: one simple ring per region — staircase up the right
    // bounding column, across the feature row, down the left, then the
    // castellation chain's exact samples. Ear-clipped in (u*r, w).
    const double rScale = std::max(
        1e-6,
        surf.Value((u0 + u1) / 2, (v0 + v1) / 2)
                .Distance(surf.Value((u0 + u1) / 2 + 1e-3, (v0 + v1) / 2)) /
            1e-3);
    // Normalized cumulative arc-length (in the same (u*r, w) metric the web
    // is measured in) — a rail's fraction parameter for laddering.
    auto arcFrac = [](const std::vector<std::array<double, 2>>& p) {
        std::vector<double> f(p.size(), 0.0);
        double L = 0.0;
        for (size_t i = 1; i < p.size(); ++i) {
            L += std::hypot(p[i][0] - p[i - 1][0], p[i][1] - p[i - 1][1]);
            f[i] = L;
        }
        if (L > 1e-12) {
            for (double& x : f) x /= L;
        }
        return f;
    };
    for (const Region& r : regions) {
        // Web as a QUAD RIBBON, not a tri fan. Two roughly-parallel rails
        // bound it: the OUTER boundary (up the left bounding column, across
        // the feature row, down the right) and the INNER cut chain. Both
        // trace the notch's up/across/down profile from the low-u foot to
        // the high-u foot, so laddering them by surface arc-fraction lays
        // quads along the walls and only a few clean grouped n-gons over
        // the rounded/scalloped top. The ribbon's boundary loop is edge-
        // identical to the ear-clip's ring, so watertightness is unchanged.
        std::vector<uint32_t> outer;
        std::vector<std::array<double, 2>> outerUW;
        auto pushOut = [&](uint32_t id, double u, double w) {
            outer.push_back(id);
            outerUW.push_back({u * rScale, w});
        };
        for (int k : colKeys[r.colL]) {
            if (rowW[k] <= rowW[r.rowKey] + 1e-12) {
                pushOut(vid[r.colL][k], uk[r.colL], rowW[k]);
            }
        }
        for (int c = r.colL + 1; c <= r.colR - 1; ++c) {
            pushOut(vid[c][r.rowKey], uk[c], rowW[r.rowKey]);
        }
        for (auto it = colKeys[r.colR].rbegin();
             it != colKeys[r.colR].rend(); ++it) {
            if (rowW[*it] <= rowW[r.rowKey] + 1e-12) {
                pushOut(vid[r.colR][*it], uk[r.colR], rowW[*it]);
            }
        }
        std::vector<uint32_t> inner;
        std::vector<std::array<double, 2>> innerUW;
        for (int i = r.iA; i <= r.iB; ++i) {
            inner.push_back(cutIds[i]);
            innerUW.push_back({cut.s[i].u * rScale, wOf(cut.s[i].v)});
        }
        // Cut chain is LOW (the notch floor), outer boundary is HIGH. Ladder
        // by arc-fraction: attach each sparse outer vertex to the nearest
        // dense cut sample (monotone), then emit one cell per span in which
        // the cut chain advances — so every cell carries at least two cut
        // samples and is a quad (or, where several cut samples or a shared-
        // column vertex pile up, a clean grouped n-gon), never a tri fan.
        if (outer.size() >= 3 && inner.size() >= 2) {
            const std::vector<double> of = arcFrac(outerUW);
            const std::vector<double> inf = arcFrac(innerUW);
            const int m = int(outer.size()) - 1, n = int(inner.size()) - 1;
            std::vector<int> mp(m + 1);
            mp[0] = 0;
            mp[m] = n;
            for (int k = 1; k < m; ++k) {
                int j = mp[k - 1];
                while (j + 1 < n &&
                       std::abs(inf[j + 1] - of[k]) <=
                           std::abs(inf[j] - of[k])) {
                    ++j;
                }
                mp[k] = j;
            }
            // Cell breakpoints: an outer vertex only closes a cell where the
            // cut chain has advanced past the last break. Outer vertices that
            // map to the same cut sample (a shared-column split, a compressed
            // top) fold into the running cell as extra n-gon corners. The
            // trailing run merges back so no leftover tri escapes.
            std::vector<int> bp{0};
            for (int b = 1; b <= m; ++b) {
                if (mp[b] > mp[bp.back()]) bp.push_back(b);
            }
            if (bp.back() != m) bp.back() = m;
            for (size_t g = 0; g + 1 < bp.size(); ++g) {
                const int a = bp[g], b = bp[g + 1];
                std::vector<uint32_t> ring;
                for (int t = mp[a]; t <= mp[b]; ++t) ring.push_back(inner[t]);
                for (int c = b; c >= a; --c) ring.push_back(outer[c]);
                emitRing(std::move(ring));
            }
            continue;
        }
        // Degenerate region (pinched to a single column): fall back to the
        // ear-clip, which tiles the identical ring either way.
        std::vector<uint32_t> ring;
        std::vector<std::array<double, 3>> pts;
        auto push = [&](uint32_t id, double u, double w) {
            ring.push_back(id);
            pts.push_back({u * rScale, w, 0.0});
        };
        for (int k : colKeys[r.colR]) {
            if (rowW[k] <= rowW[r.rowKey] + 1e-12) {
                push(vid[r.colR][k], uk[r.colR], rowW[k]);
            }
        }
        for (int c = r.colR - 1; c >= r.colL; --c) {
            push(vid[c][r.rowKey], uk[c], rowW[r.rowKey]);
        }
        for (auto it = colKeys[r.colL].rbegin();
             it != colKeys[r.colL].rend(); ++it) {
            if (rowW[*it] < rowW[r.rowKey] - 1e-12) {
                push(vid[r.colL][*it], uk[r.colL], rowW[*it]);
            }
        }
        for (int i = r.iA; i <= r.iB; ++i) {
            push(cutIds[i], cut.s[i].u, wOf(cut.s[i].v));
        }
        if (ring.size() < 3) return false;
        std::vector<uint32_t> idx(ring.size());
        std::iota(idx.begin(), idx.end(), 0u);
        size_t emitted = 0;
        for (const auto& t : triangulatePoly(pts, idx)) {
            const uint32_t a = ring[t[0]], b = ring[t[1]], c = ring[t[2]];
            if (a == b || b == c || a == c) continue;
            wb.addPolygon({a, b, c}, faceId, flip);
            ++emitted;
        }
        // A complete ear-clip yields exactly V-2 triangles; anything
        // less means the web has a hole — fail the face un-emitted.
        if (emitted + 2 < ring.size()) return false;
    }

    // Interior cutout webs: the deleted cells leave one open directed
    // ring per box inside the local build; splice the wire's exact
    // contract samples into it as a keyhole and ear-clip. The web is the
    // LOCAL collar — the bridge edges are interior (used twice, once per
    // flanking triangle), and the wire samples weld to the slot wall
    // faces by construction (they sample the same 3D curves at the same
    // solved counts).
    if (!iboxes.empty()) {
        std::map<std::pair<uint32_t, uint32_t>, int> dir;
        for (const auto& poly : local.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                ++dir[{poly[i], poly[(i + 1) % poly.size()]}];
            }
        }
        std::map<uint32_t, uint32_t> next;
        for (const auto& [e, n] : dir) {
            if (n != 1 || dir.count({e.second, e.first})) continue;
            if (next.count(e.first)) return false;  // ambiguous boundary
            next[e.first] = e.second;
        }
        std::vector<std::vector<uint32_t>> loops;
        {
            std::set<uint32_t> visited;
            for (const auto& [a, b] : next) {
                if (visited.count(a)) continue;
                std::vector<uint32_t> loop{a};
                visited.insert(a);
                uint32_t cur = b;
                bool closed = false;
                for (size_t guard = 0; guard <= next.size(); ++guard) {
                    if (cur == a) {
                        closed = true;
                        break;
                    }
                    auto it = next.find(cur);
                    if (it == next.end()) break;
                    loop.push_back(cur);
                    visited.insert(cur);
                    cur = it->second;
                }
                if (closed && loop.size() >= 3) {
                    loops.push_back(std::move(loop));
                }
            }
        }
        const double du = uspan / nu;
        for (const IBox& b : iboxes) {
            // The cavity loop: the closed boundary ring whose UV bbox
            // fits inside the cut's column/row bracket (the band's own
            // outer boundary spans the whole face and never matches).
            const double lu0 = uk[b.colL] - 0.5 * du;
            const double lu1 = uk[b.colR] + 0.5 * du;
            const double bvA =
                std::min(vOf(rowW[b.rowLo]), vOf(rowW[b.rowHi]));
            const double bvB =
                std::max(vOf(rowW[b.rowLo]), vOf(rowW[b.rowHi]));
            const double vPad = 0.02 * wspan;
            int li = -1;
            for (size_t i = 0; i < loops.size(); ++i) {
                double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
                for (uint32_t v : loops[i]) {
                    const Anchor& an = local.anchors[v];
                    x0 = std::min(x0, an.u);
                    x1 = std::max(x1, an.u);
                    y0 = std::min(y0, an.v);
                    y1 = std::max(y1, an.v);
                }
                if (x0 >= lu0 && x1 <= lu1 && y0 >= bvA - vPad &&
                    y1 <= bvB + vPad) {
                    if (li >= 0) return false;  // two rings in one box
                    li = int(i);
                }
            }
            if (li < 0) return false;
            std::vector<uint32_t> cav = loops[li];
            // The boundary chain (walked in the survivors' stored
            // direction) reversed is the direction the collar must
            // traverse the cavity edges — each shared edge then carries
            // exactly one collar polygon against one lattice polygon.
            std::reverse(cav.begin(), cav.end());

            // The wire's contract samples, chained into one closed ring.
            struct HP {
                gp_Pnt p;
                double u, v;
            };
            std::vector<std::vector<HP>> pieces;
            for (int eid : *b.wire) {
                const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
                double f2, l2, f3, l3;
                Handle(Geom2d_Curve) pc =
                    BRep_Tool::CurveOnSurface(edge, face, f2, l2);
                Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
                if (pc.IsNull() || c3.IsNull()) return false;
                const int n =
                    eid < int(solvedEdge.size())
                        ? std::max(1, solvedEdge[eid])
                        : 1;
                std::vector<HP> pts;
                for (double tt : edgeSampleFractions(eid, n, 0.0, false,
                                                     /*includeLast=*/true,
                                                     pins, &model)) {
                    gp_Pnt2d uv = pc->Value(f2 + tt * (l2 - f2));
                    pts.push_back({c3->Value(f3 + tt * (l3 - f3)), uv.X(),
                                   uv.Y()});
                }
                if (pts.size() < 2) return false;
                pieces.push_back(std::move(pts));
            }
            std::vector<HP> hole = pieces[0];
            {
                std::vector<char> used(pieces.size(), 0);
                used[0] = 1;
                for (size_t step = 1; step < pieces.size(); ++step) {
                    const gp_Pnt cur = hole.back().p;
                    double best = 1e300;
                    size_t bj = 0;
                    bool rev = false;
                    for (size_t j = 0; j < pieces.size(); ++j) {
                        if (used[j]) continue;
                        const double dF =
                            cur.Distance(pieces[j].front().p);
                        const double dB = cur.Distance(pieces[j].back().p);
                        if (dF < best) {
                            best = dF;
                            bj = j;
                            rev = false;
                        }
                        if (dB < best) {
                            best = dB;
                            bj = j;
                            rev = true;
                        }
                    }
                    std::vector<HP>& pj = pieces[bj];
                    if (rev) std::reverse(pj.begin(), pj.end());
                    hole.insert(hole.end(), pj.begin() + 1, pj.end());
                    used[bj] = 1;
                }
                double perim = 0;
                for (size_t i = 1; i < hole.size(); ++i) {
                    perim += hole[i - 1].p.Distance(hole[i].p);
                }
                if (hole.size() < 4 ||
                    hole.front().p.Distance(hole.back().p) >
                        0.05 * std::max(perim, 1e-9)) {
                    return false;  // the wire didn't close
                }
                hole.pop_back();
            }
            // Collar ladder between the two closed rings: both must wind
            // the SAME way — a rung then traverses the cavity edge
            // forward and the hole edge backward, manifold on both.
            auto areaCav = [&]() {
                double a = 0;
                for (size_t i = 0; i < cav.size(); ++i) {
                    const Anchor& p = local.anchors[cav[i]];
                    const Anchor& q =
                        local.anchors[cav[(i + 1) % cav.size()]];
                    a += p.u * rScale * q.v - q.u * rScale * p.v;
                }
                return a;
            };
            auto areaHole = [&]() {
                double a = 0;
                for (size_t i = 0; i < hole.size(); ++i) {
                    const HP& p = hole[i];
                    const HP& q = hole[(i + 1) % hole.size()];
                    a += p.u * rScale * q.v - q.u * rScale * p.v;
                }
                return a;
            };
            if (areaCav() * areaHole() < 0) {
                std::reverse(hole.begin(), hole.end());
            }
            std::vector<uint32_t> hid(hole.size());
            for (size_t i = 0; i < hole.size(); ++i) {
                hid[i] = wb.addVertex(hole[i].p,
                                      {faceId, hole[i].u, hole[i].v});
            }
            // Align the ring starts at the nearest pair, then zip by
            // arc fraction — quads where the two rings advance together,
            // triangles where one is denser (the ring counts are
            // independent: the cavity follows the lattice, the hole
            // follows the wire's solved counts).
            size_t ci = 0, hj = 0;
            {
                double best = 1e300;
                for (size_t i = 0; i < cav.size(); ++i) {
                    const Anchor& an = local.anchors[cav[i]];
                    for (size_t j = 0; j < hole.size(); ++j) {
                        const double dx = (an.u - hole[j].u) * rScale;
                        const double dy = an.v - hole[j].v;
                        const double d2 = dx * dx + dy * dy;
                        if (d2 < best) {
                            best = d2;
                            ci = i;
                            hj = j;
                        }
                    }
                }
            }
            const size_t nc = cav.size(), nh = hole.size();
            auto C = [&](size_t k) { return cav[(ci + k) % nc]; };
            auto H = [&](size_t k) { return hid[(hj + k) % nh]; };
            // Pair the rings by POLAR ANGLE around the cutout centroid —
            // both encircle it in the same direction, so angle pairing
            // is monotone and twist-free where arc-length fractions
            // distort (a rectangle cavity ring against a round hole
            // spends very different fractions per turn).
            double cx = 0, cy = 0;
            for (const HP& h : hole) {
                cx += h.u * rScale;
                cy += h.v;
            }
            cx /= double(nh);
            cy /= double(nh);
            auto unwrap = [&](std::vector<double>& a) {
                for (size_t k = 1; k < a.size(); ++k) {
                    while (a[k] - a[k - 1] > M_PI) a[k] -= 2.0 * M_PI;
                    while (a[k] - a[k - 1] < -M_PI) a[k] += 2.0 * M_PI;
                }
            };
            std::vector<double> tc(nc + 1), th(nh + 1);
            for (size_t k = 0; k <= nc; ++k) {
                const Anchor& a = local.anchors[C(k % nc)];
                tc[k] = std::atan2(a.v - cy, a.u * rScale - cx);
            }
            for (size_t k = 0; k <= nh; ++k) {
                const HP& h = hole[(hj + k) % nh];
                th[k] = std::atan2(h.v - cy, h.u * rScale - cx);
            }
            unwrap(tc);
            unwrap(th);
            // Both rings sweep one full turn the same way; anything else
            // is a geometry the collar cannot express.
            if (std::abs(std::abs(tc[nc] - tc[0]) - 2.0 * M_PI) > 0.5 ||
                std::abs(std::abs(th[nh] - th[0]) - 2.0 * M_PI) > 0.5 ||
                (tc[nc] - tc[0]) * (th[nh] - th[0]) < 0) {
                return false;
            }
            // Attach each sparse cavity vertex to the nearest dense hole
            // sample (monotone by unwrapped angle), then emit ONE cell
            // per cavity segment carrying every hole sample in its span
            // — a quad or a grouped n-gon, never a triangle fan (the
            // notch webs' absorption pattern).
            const double angularDirection = tc[nc] > tc[0] ? 1.0 : -1.0;
            auto off = [&](double h, double c) {
                // angular offset h-c, wrapped to the nearest turn
                double d = (h - c) * angularDirection;
                while (d > M_PI) d -= 2.0 * M_PI;
                while (d < -M_PI) d += 2.0 * M_PI;
                return std::abs(d);
            };
            std::vector<size_t> mp(nc + 1);
            mp[0] = 0;
            mp[nc] = nh;
            for (size_t k = 1; k < nc; ++k) {
                size_t j = mp[k - 1];
                while (j + 1 < nh &&
                       off(th[j + 1], tc[k]) <= off(th[j], tc[k])) {
                    ++j;
                }
                mp[k] = j;
            }
            for (size_t k = 0; k < nc; ++k) {
                std::vector<uint32_t> cell{C(k), C(k + 1)};
                for (size_t j = mp[k + 1]; j-- > mp[k];) {
                    cell.push_back(H(j + 1));
                }
                cell.push_back(H(mp[k]));
                cell.erase(std::unique(cell.begin(), cell.end()),
                           cell.end());
                while (cell.size() > 1 && cell.front() == cell.back()) {
                    cell.pop_back();
                }
                if (cell.size() < 3) continue;
                wb.addPolygon(std::move(cell), faceId, false);
            }
            dbg("openband face %d insert collar: cav=%zu hole=%zu", faceId,
                nc, nh);
        }
    }

    dbg("openband face %d: cols=%d rows=%d regions=%zu passCut=%d "
        "passPlain=%d inserts=%zu polys=%zu",
        faceId, nu, nv, regions.size(), passCut ? 1 : 0, passPlain ? 1 : 0,
        iboxes.size(), local.polygons.size());
    // Everything validated: splat the local result into the builder.
    std::vector<uint32_t> outMap(local.vertices.size());
    for (uint32_t i = 0; i < local.vertices.size(); ++i) {
        outMap[i] = out.addVertex(
            gp_Pnt(local.vertices[i][0], local.vertices[i][1],
                   local.vertices[i][2]),
            local.anchors[i]);
    }
    for (const auto& poly : local.polygons) {
        std::vector<uint32_t> mapped;
        mapped.reserve(poly.size());
        for (uint32_t idx2 : poly) mapped.push_back(outMap[idx2]);
        out.addPolygon(std::move(mapped), faceId, false);
    }
    return true;
}

// Full-wrap castellated rim (see castellatedRimBand): a u-closed
// cylinder/cone whose one rim is a plain full circle and the other is cut
// through by a notch — two walls dropping from the rim to an interior
// floor. The plain rim drives a STRAIGHT uniform column lattice (columns at
// fixed azimuths are exact rulings on a cylinder/cone), and its own samples
// ARE the columns, so it welds bit-identically to its neighbour. The
// notched rim's base arcs weld through a thin transition strip (which does
// not disturb the straight columns), and the notch itself is boolean-cut,
// its walls/floor webbed to their exact edge samples (the same contract the
// notch's wall/floor faces sample). This is the full-wrap sibling of
// meshRevolutionOpenBand: identical cut/strip/web, but the lattice wraps
// periodically instead of running between two side edges.
bool meshRevolutionRimNotch(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model,
                            const std::vector<int>& rimLow,
                            const std::vector<int>& rimHigh, int plainRimEdge,
                            const std::vector<int>& solvedEdge, int faceId,
                            int nu, int nv, MeshBuilder& out,
                            const PinnedEdges* pins,
                            // Interior row LEVELS (surface v): the insert
                            // composition subdivides every column at these
                            // heights so slot rows exist for the carve.
                            // Only the pinned boolean-cut path supports
                            // them; the strip path bails so the caller's
                            // floor still catches the face.
                            const std::vector<double>* levelsOpt) {
    if (!surf.IsUClosed() || surf.IsVClosed()) return false;
    if (rimLow.empty() || rimHigh.empty()) return false;
    nu = std::max(3, nu);
    nv = std::max(1, nv);
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double period = std::max(1e-12, u1 - u0);
    const double vspan = std::max(1e-12, v1 - v0);

    const bool lowPlain = rimLow.size() == 1 && rimLow[0] == plainRimEdge;
    const bool highPlain = rimHigh.size() == 1 && rimHigh[0] == plainRimEdge;
    if (lowPlain == highPlain) return false;  // exactly one plain rim
    const std::vector<int>& plainChain = lowPlain ? rimLow : rimHigh;
    const std::vector<int>& cutChain = lowPlain ? rimHigh : rimLow;

    struct SPt {
        double u, v;
        gp_Pnt p;
    };

    // ---- Plain rim: sample the full circle at nu; its samples ARE the
    // columns (uniform azimuths = straight rulings) and weld to whatever
    // shares this circle.
    std::vector<SPt> plainS;
    {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(plainChain[0]));
        double f2, l2, f3, l3;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f2, l2);
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        if (pc.IsNull() || c3.IsNull()) return false;
        const bool rev = edge.Orientation() == TopAbs_REVERSED;
        const double ph = closedEdgePhase(edge, model);
        for (int i = 0; i < nu; ++i) {
            const double t = phasedT(i, nu, ph, rev);
            gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
            double u = uv.X();
            u -= period * std::floor((u - u0) / period);
            plainS.push_back({u, uv.Y(), c3->Value(f3 + (l3 - f3) * t)});
        }
    }
    if (int(plainS.size()) != nu) return false;
    std::sort(plainS.begin(), plainS.end(),
              [](const SPt& a, const SPt& b) { return a.u < b.u; });
    std::vector<double> uk(nu);
    for (int c = 0; c < nu; ++c) {
        uk[c] = plainS[c].u;
        if (c && uk[c] <= uk[c - 1] + 1e-9 * period) return false;
    }
    double vPlain = 0;
    for (const SPt& s : plainS) vPlain += s.v;
    vPlain /= nu;
    const double vCut =
        std::abs(vPlain - v0) < std::abs(vPlain - v1) ? v1 : v0;
    const double sign = vPlain > vCut ? 1.0 : -1.0;
    auto wOf = [&](double v) { return sign * (v - vCut); };
    auto vOf = [&](double w) { return vCut + sign * w; };
    const double wspan = wOf(vPlain);
    if (wspan <= 1e-9) return false;

    // ---- Cut rim: sample each edge at its solved count; a piece that
    // stays at the cut rim level is a base arc (hug), anything that drops
    // away is the notch (walls + floor).
    struct Piece {
        std::vector<SPt> pts;
        bool hug = false;
    };
    std::vector<Piece> pieces;
    bool basePinned = false;
    for (int eid : cutChain) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        double f2, l2, f3, l3;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f2, l2);
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        if (pc.IsNull() || c3.IsNull()) return false;
        int n = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
        if (n < 1) n = 1;
        const bool rev = edge.Orientation() == TopAbs_REVERSED;
        const double ph = closedEdgePhase(edge, model);
        Piece piece;
        double bv0 = 1e300, bv1 = -1e300;
        for (int k = 0; k <= 16; ++k) {
            gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * k / 16.0);
            bv0 = std::min(bv0, uv.Y());
            bv1 = std::max(bv1, uv.Y());
        }
        piece.hug = std::abs(bv0 - vCut) < 0.02 * vspan &&
                    std::abs(bv1 - vCut) < 0.02 * vspan;
        // Pinned cut-rim edges carry the column azimuths (base arc + floor)
        // or a single flat span (walls); both faces read them, so the notch
        // is a clean boolean cut of the uniform column lattice with no
        // strip and no wall ladders. basePinned flips on when the base arc
        // (a hug piece) is pinned — that's the reframed model.
        const bool usePins = edgeIsPinned(eid, pins);
        if (usePins && piece.hug) basePinned = true;
        const std::vector<double> fr =
            edgeSampleFractions(eid, n, ph, rev, /*includeLast=*/true,
                                usePins ? pins : nullptr, &model);
        for (double t : fr) {
            gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
            piece.pts.push_back({uv.X(), uv.Y(), c3->Value(f3 + (l3 - f3) * t)});
        }
        pieces.push_back(std::move(piece));
    }

    // Chain the pieces into one loop by nearest 3D endpoints; each sample
    // inherits its piece's hug flag.
    std::vector<SPt> S;
    std::vector<char> hug;
    {
        auto append = [&](const Piece& pc2, bool rev) {
            const auto& p = pc2.pts;
            for (size_t k = 0; k < p.size(); ++k) {
                const SPt& s = rev ? p[p.size() - 1 - k] : p[k];
                if (!S.empty() && S.back().p.Distance(s.p) < 1e-9) continue;
                S.push_back(s);
                hug.push_back(pc2.hug ? 1 : 0);
            }
        };
        std::vector<char> used(pieces.size(), 0);
        append(pieces[0], false);
        used[0] = 1;
        for (size_t step = 1; step < pieces.size(); ++step) {
            double bd = 1e300;
            size_t bi = 0;
            bool rev = false;
            for (size_t k = 0; k < pieces.size(); ++k) {
                if (used[k]) continue;
                const double dF = S.back().p.Distance(pieces[k].pts.front().p);
                const double dB = S.back().p.Distance(pieces[k].pts.back().p);
                if (dF < bd) { bd = dF; bi = k; rev = false; }
                if (dB < bd) { bd = dB; bi = k; rev = true; }
            }
            used[bi] = 1;
            append(pieces[bi], rev);
        }
        if (S.size() > 1 && S.front().p.Distance(S.back().p) < 1e-9) {
            S.pop_back();
            hug.pop_back();
        }
    }
    const int SN = int(S.size());
    if (SN < 4) return false;

    // Orient ascending in u and rotate to start just past the seam so the
    // notch (feature run) sits strictly interior.
    for (SPt& s : S) s.u -= period * std::floor((s.u - u0) / period);
    double turn = 0;
    for (int i = 0; i < SN; ++i) {
        double d = S[(i + 1) % SN].u - S[i].u;
        d -= period * std::round(d / period);
        turn += d;
    }
    if (turn < 0) {
        std::reverse(S.begin(), S.end());
        std::reverse(hug.begin(), hug.end());
    }
    int startIdx = 0;
    for (int i = 0; i < SN; ++i) {
        if (S[i].u < S[(i - 1 + SN) % SN].u - 1e-9 * period) {
            startIdx = i;
            break;
        }
    }
    std::rotate(S.begin(), S.begin() + startIdx, S.end());
    std::rotate(hug.begin(), hug.begin() + startIdx, hug.end());

    // Single contiguous feature run (one notch), strictly interior.
    int iFa = -1, iFb = -1;
    for (int i = 0; i < SN; ++i) {
        if (!hug[i]) {
            if (iFa < 0) iFa = i;
            iFb = i;
        }
    }
    if (iFa <= 0 || iFb >= SN - 1) return false;  // no notch / straddles seam
    for (int i = iFa; i <= iFb; ++i) {
        if (hug[i]) return false;  // more than one notch: not handled here
    }

    double nu0 = 1e300, nu1 = -1e300, wTop = 0;
    for (int i = iFa; i <= iFb; ++i) {
        nu0 = std::min(nu0, S[i].u);
        nu1 = std::max(nu1, S[i].u);
        wTop = std::max(wTop, wOf(S[i].v));
    }
    if (wTop <= 1e-6 * wspan || wTop > 0.9 * wspan) return false;

    // Bounding KEPT columns; sliver cells are pushed out with the notch.
    int colL = 0;
    while (colL + 1 < nu && uk[colL + 1] <= nu0) ++colL;
    if (colL > 0 && colL + 1 < nu &&
        nu0 - uk[colL] < 0.3 * (uk[colL + 1] - uk[colL])) {
        --colL;
    }
    int colR = nu - 1;
    for (int c = 0; c < nu; ++c) {
        if (uk[c] >= nu1) { colR = c; break; }
    }
    if (colR > 0 && colR < nu - 1 &&
        uk[colR] - nu1 < 0.3 * (uk[colR] - uk[colR - 1])) {
        ++colR;
    }
    if (colL < 1 || colR > nu - 1 || colR <= colL) return false;

    // ===== PINNED BOOLEAN-CUT NOTCH (the reframed model) =================
    // The end geometry is a perfect cylinder with a rectangular bite taken
    // out. Lay a UNIFORM full-cylinder column lattice (columns at the plain
    // rim's own azimuths, driven by radial/adaptive as if there were no
    // notch), run every column straight from the plain rim to the cut rim,
    // and boolean-cut the notch: columns in the notch u-range stop at the
    // floor. No wall ladders, no feature columns, no inset strip — the two
    // notch corners each fold into ONE cap n-gon; everywhere else is quads.
    // The cut-rim base arcs and the notch floor carry the column azimuths
    // (pinned), and the single-span walls carry only the two corner edges,
    // so the neighbour annulus / floor / wall faces weld bit-identically.
    if (basePinned) {
        // Attempted as a transaction: a count whose columns miss the
        // notch corners cannot take the clean cut, and that must NOT
        // fail the whole face (radial 23/46/51/69 on the notched
        // fixture demoted exactly this way) — the generic strip path
        // below absorbs any misalignment, and the pieces already sample
        // pinned edges at their pin positions so borders stay exact.
        const int colL0 = colL, colR0 = colR;
        const bool cleanCut = [&]() -> bool {
        const double snapU = 0.02 * (period / nu);
        const double tolV2 = 0.05 * vspan;
        const double vFloor = vOf(wTop);
        std::vector<int> baseArcS(nu, -1);
        std::vector<int> floorS(nu, -1);
        std::vector<int> topCorner, botCorner;  // S indices (az_L / az_R)
        for (int i = 0; i < SN; ++i) {
            const bool top = std::abs(S[i].v - vCut) < tolV2;
            const bool bot = std::abs(S[i].v - vFloor) < tolV2;
            if (top == bot) continue;  // wall interior (neither rim level)
            int mc = -1;
            for (int c = 0; c < nu; ++c) {
                double d = S[i].u - uk[c];
                d -= period * std::round(d / period);
                if (std::abs(d) < snapU) { mc = c; break; }
            }
            if (top) {
                if (mc >= 0) baseArcS[mc] = i;
                else topCorner.push_back(i);
            } else {
                if (mc >= 0) floorS[mc] = i;
                else botCorner.push_back(i);
            }
        }
        // Derive the notch column span from the base-arc coverage gap.
        int lo = nu, hi = -1;
        for (int c = 0; c < nu; ++c) {
            if (baseArcS[c] < 0) { lo = std::min(lo, c); hi = std::max(hi, c); }
        }
        if (hi < 0 || lo < 1 || hi > nu - 2) return false;  // interior notch
        for (int c = lo; c <= hi; ++c) {
            if (baseArcS[c] >= 0 || floorS[c] < 0) return false;
        }
        colL = lo - 1;  // last non-notch column left of the notch
        colR = hi + 1;  // first non-notch column right of the notch
        if (topCorner.size() != 2 || botCorner.size() != 2) return false;

        // Corner azimuths sit in the two coverage gaps; pair top<->bottom
        // by azimuth and label left (colL..colL+1) / right (colR-1..colR).
        auto uNorm = [&](double u) {
            u -= period * std::floor((u - u0) / period);
            return u;
        };
        auto inGap = [&](double u, int a, int b) {
            double lo2 = uNorm(uk[a]), hi2 = uNorm(uk[b]);
            double uu = uNorm(u);
            if (hi2 < lo2) hi2 += period;
            if (uu < lo2) uu += period;
            return uu > lo2 - 1e-9 && uu < hi2 + 1e-9;
        };
        int TL = -1, TR = -1, BL = -1, BR = -1;
        for (int i : topCorner) {
            if (inGap(S[i].u, colL, colL + 1)) TL = i;
            else if (inGap(S[i].u, colR - 1, colR)) TR = i;
        }
        for (int i : botCorner) {
            if (inGap(S[i].u, colL, colL + 1)) BL = i;
            else if (inGap(S[i].u, colR - 1, colR)) BR = i;
        }
        if (TL < 0 || TR < 0 || BL < 0 || BR < 0) return false;

        PolyMesh local;
        MeshBuilder wb(local);
        const bool flip = (face.Orientation() == TopAbs_REVERSED) ^ (sign < 0);
        std::vector<uint32_t> cutIds(SN);
        for (int i = 0; i < SN; ++i) {
            cutIds[i] = wb.addVertex(S[i].p, {faceId, S[i].u, S[i].v});
        }
        std::vector<uint32_t> plainIds(nu);
        for (int c = 0; c < nu; ++c) {
            plainIds[c] =
                wb.addVertex(plainS[c].p, {faceId, plainS[c].u, plainS[c].v});
        }
        // Interior row levels (the insert composition): every column line
        // subdivides at the requested surface-v heights, ordered from the
        // plain rim toward the cut end. Levels apply only strictly inside
        // a column's own span — the notch's columns end at the FLOOR, so
        // a level above it simply doesn't exist there (the caps absorb
        // the difference as n-gon side verts).
        std::vector<std::vector<uint32_t>> colChain(nu);
        if (levelsOpt) {
            // Margin matches the insert's own plan gate (1% rim
            // clearance): a slot row 1.5% above the rim is a thin but
            // VALID band, and the staircase needs it — a fatter margin
            // silently dropped it and left the carve's loop open.
            const double margin = 0.005 * vspan;
            for (int c = 0; c < nu; ++c) {
                const bool notchCol = c > colL && c < colR;
                const double vEnd = notchCol ? vFloor : vCut;
                const double lo3 = std::min(vPlain, vEnd) + margin;
                const double hi3 = std::max(vPlain, vEnd) - margin;
                std::vector<double> lv;
                for (double v : *levelsOpt) {
                    if (v > lo3 && v < hi3) lv.push_back(v);
                }
                std::sort(lv.begin(), lv.end());
                if (vPlain > vEnd) std::reverse(lv.begin(), lv.end());
                for (double v : lv) {
                    colChain[c].push_back(wb.addVertex(
                        surf.Value(uk[c], v), {faceId, uk[c], v}));
                }
            }
        }
        // Each column's cut-rim end: base-arc vertex outside the notch,
        // floor vertex inside it.
        auto topOf = [&](int c) {
            return baseArcS[c] >= 0 ? cutIds[baseArcS[c]] : cutIds[floorS[c]];
        };
        auto emit = [&](std::vector<uint32_t> ring) {
            ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
            while (ring.size() > 1 && ring.front() == ring.back()) {
                ring.pop_back();
            }
            if (ring.size() >= 3) wb.addPolygon(std::move(ring), faceId, flip);
        };
        for (int c = 0; c < nu; ++c) {
            const int cp = (c + 1) % nu;
            const bool cNotch = c > colL && c < colR;
            const bool pNotch = cp > colL && cp < colR;
            if (c == colL) {
                // Left cap: base arc -> wall (TL,BL) -> floor, folded
                // once; level verts ride the two side chains.
                std::vector<uint32_t> ring{topOf(c), cutIds[TL],
                                           cutIds[BL], topOf(cp)};
                for (auto it = colChain[cp].rbegin();
                     it != colChain[cp].rend(); ++it) {
                    ring.push_back(*it);
                }
                ring.push_back(plainIds[cp]);
                ring.push_back(plainIds[c]);
                for (uint32_t v : colChain[c]) ring.push_back(v);
                emit(std::move(ring));
            } else if (c == colR - 1) {
                // Right cap.
                std::vector<uint32_t> ring{topOf(c), cutIds[BR],
                                           cutIds[TR], topOf(cp)};
                for (auto it = colChain[cp].rbegin();
                     it != colChain[cp].rend(); ++it) {
                    ring.push_back(*it);
                }
                ring.push_back(plainIds[cp]);
                ring.push_back(plainIds[c]);
                for (uint32_t v : colChain[c]) ring.push_back(v);
                emit(std::move(ring));
            } else if (colChain[c].empty() && colChain[cp].empty()) {
                (void)cNotch;
                (void)pNotch;
                emit({topOf(c), topOf(cp), plainIds[cp], plainIds[c]});
            } else {
                // Banded column pair: both lines carry the same level
                // set (non-cap pairs are both full-height or both
                // notch-floor columns); a mismatch means a level fell
                // inside one column's end margin only — bail to the
                // caller's floor rather than emit a cracked band.
                if (colChain[c].size() != colChain[cp].size()) {
                    return false;
                }
                std::vector<uint32_t> lowC{plainIds[c]};
                std::vector<uint32_t> lowP{plainIds[cp]};
                for (size_t k = 0; k < colChain[c].size(); ++k) {
                    lowC.push_back(colChain[c][k]);
                    lowP.push_back(colChain[cp][k]);
                }
                lowC.push_back(topOf(c));
                lowP.push_back(topOf(cp));
                for (size_t b = 0; b + 1 < lowC.size(); ++b) {
                    emit({lowC[b + 1], lowP[b + 1], lowP[b], lowC[b]});
                }
            }
        }
        dbg("rimnotch face %d: PINNED nu=%d cols[%d,%d] polys=%zu", faceId,
            nu, colL, colR, local.polygons.size());
        std::vector<uint32_t> outMap(local.vertices.size());
        for (uint32_t i = 0; i < local.vertices.size(); ++i) {
            outMap[i] = out.addVertex(
                gp_Pnt(local.vertices[i][0], local.vertices[i][1],
                       local.vertices[i][2]),
                local.anchors[i]);
        }
        for (const auto& poly : local.polygons) {
            std::vector<uint32_t> mapped;
            mapped.reserve(poly.size());
            for (uint32_t idx2 : poly) mapped.push_back(outMap[idx2]);
            out.addPolygon(std::move(mapped), faceId, false);
        }
        return true;
        }();
        if (cleanCut) return true;
        colL = colL0;
        colR = colR0;
        dbg("rimnotch face %d: pinned cut misaligned at nu=%d, strip path",
            faceId, nu);
    }

    // The strip path below cannot honor explicit interior levels — the
    // insert composition only rides the pinned boolean-cut lattice.
    if (levelsOpt) return false;

    // ---- Rows in w. Feature row just past the notch depth; a thin strip
    // row hugs the cut rim so the columns stay straight for (nearly) the
    // whole height. The plain rim is exact at w = wspan (no strip there).
    // (Pinned castellated rims never reach here — they emit the clean
    // boolean-cut lattice above and return.)
    const double wFeat = wTop + std::max(0.04 * wTop, 0.005 * wspan);
    const double wBot = std::min({0.25 * wspan / nv, 0.05 * wspan, 0.4 * wFeat});
    if (wBot < 1e-3 * wspan) return false;
    if (wFeat > wspan - 0.02 * wspan || wFeat < wBot + 0.01 * wspan) {
        return false;
    }
    std::vector<double> rowW;
    auto addRow = [&](double w) {
        for (size_t i = 0; i < rowW.size(); ++i) {
            if (std::abs(rowW[i] - w) < 0.008 * wspan) return int(i);
        }
        rowW.push_back(w);
        return int(rowW.size()) - 1;
    };
    const int keyBot = addRow(wBot);
    const int keyTop = addRow(wspan);
    if (keyBot == keyTop) return false;
    std::vector<int> keyAx;
    for (int j = 1; j < nv; ++j) keyAx.push_back(addRow(j * wspan / nv));
    const int keyFeat = addRow(wFeat);

    // Identify the notch outline GEOMETRICALLY (coarse sampling can class
    // a wall's rim corner as a hug arc endpoint, so the hug flag alone
    // misses it): the floor is the deepest-v run, each wall the constant-u
    // run rising from a floor end back to the rim. Ordered top -> bottom.
    const double vFloorLvl = vOf(wTop);
    const double tolV = 0.03 * vspan;
    const double tolU = 0.15 * std::max(1e-9, nu1 - nu0);
    int fL = iFa, fR = iFb;
    {
        bool found = false;
        for (int i = iFa; i <= iFb; ++i) {
            if (std::abs(S[i].v - vFloorLvl) < tolV) {
                if (!found) { fL = i; found = true; }
                fR = i;
            }
        }
        if (!found) return false;
    }
    auto climbWall = [&](int from, int dir) {
        const double uW = S[from].u;
        std::vector<int> w;
        for (int step = 0, i = from; step < SN;
             ++step, i = (i + dir + SN) % SN) {
            if (std::abs(S[i].u - uW) > tolU) break;
            w.push_back(i);
            if (std::abs(S[i].v - vCut) < tolV) break;
        }
        std::reverse(w.begin(), w.end());  // top (rim) -> bottom (floor)
        return w;
    };
    const std::vector<int> leftWall = climbWall(fL, -1);
    const std::vector<int> rightWall = climbWall(fR, +1);
    if (leftWall.size() < 2 || rightWall.size() < 2) return false;
    if (std::abs(S[leftWall.front()].v - vCut) > tolV ||
        std::abs(S[rightWall.front()].v - vCut) > tolV) {
        return false;  // a wall that never reaches the rim: malformed
    }
    // The bounding columns carry a row at each wall sample's depth so the
    // wall strip is a clean 1:1 ladder (vertical rulings, dAz = 0) — a
    // coarse column would slant a diagonal across the wall instead. Only
    // the notch-depth band (below keyBot, above keyFeat) is populated.
    std::vector<int> wallRowsL, wallRowsR;
    auto collectWallRows = [&](const std::vector<int>& wall,
                               std::vector<int>& out) {
        for (int i : wall) {
            const double w = wOf(S[i].v);
            if (w <= rowW[keyBot] + 1e-9 || w >= rowW[keyFeat] - 1e-9) continue;
            out.push_back(addRow(w));
        }
    };
    collectWallRows(leftWall, wallRowsL);
    collectWallRows(rightWall, wallRowsR);

    std::vector<std::vector<int>> colKeys(nu);
    for (int c = 0; c < nu; ++c) {
        const int floorKey = (colL < c && c < colR) ? keyFeat : keyBot;
        std::vector<int> ks{floorKey, keyTop};
        for (int k : keyAx) {
            if (rowW[k] > rowW[floorKey] + 1e-12 &&
                rowW[k] < rowW[keyTop] - 1e-12) {
                ks.push_back(k);
            }
        }
        if (c == colL || c == colR) ks.push_back(keyFeat);
        if (c == colL) ks.insert(ks.end(), wallRowsL.begin(), wallRowsL.end());
        if (c == colR) ks.insert(ks.end(), wallRowsR.begin(), wallRowsR.end());
        std::sort(ks.begin(), ks.end(),
                  [&](int a, int b) { return rowW[a] < rowW[b]; });
        ks.erase(std::unique(ks.begin(), ks.end()), ks.end());
        colKeys[c] = std::move(ks);
    }

    // ---- Build locally; a failed web must leave nothing emitted.
    PolyMesh local;
    MeshBuilder wb(local);
    const bool flip = (face.Orientation() == TopAbs_REVERSED) ^ (sign < 0);
    std::vector<uint32_t> cutIds(SN);
    for (int i = 0; i < SN; ++i) {
        cutIds[i] = wb.addVertex(S[i].p, {faceId, S[i].u, S[i].v});
    }
    std::vector<uint32_t> plainIds(nu);
    for (int c = 0; c < nu; ++c) {
        plainIds[c] =
            wb.addVertex(plainS[c].p, {faceId, plainS[c].u, plainS[c].v});
    }
    std::vector<std::map<int, uint32_t>> vid(nu);
    for (int c = 0; c < nu; ++c) {
        for (int key : colKeys[c]) {
            if (key == keyTop) {
                vid[c][key] = plainIds[c];
                continue;
            }
            const double vv = vOf(rowW[key]);
            vid[c][key] =
                wb.addVertex(surf.Value(uk[c], vv), {faceId, uk[c], vv});
        }
    }

    auto emitRing = [&](std::vector<uint32_t> ring) {
        ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
        while (ring.size() > 1 && ring.front() == ring.back()) ring.pop_back();
        if (ring.size() < 3) return;
        wb.addPolygon(std::move(ring), faceId, flip);
    };

    // Lattice cells, wrapping periodically; region cells below the feature
    // row simply never exist (the boolean cut).
    for (int c = 0; c < nu; ++c) {
        const int cp = (c + 1) % nu;
        const std::vector<int>& L = colKeys[c];
        const std::vector<int>& R = colKeys[cp];
        std::vector<int> common;
        for (int k : L) {
            if (std::find(R.begin(), R.end(), k) != R.end()) common.push_back(k);
        }
        for (size_t b = 0; b + 1 < common.size(); ++b) {
            const double wA = rowW[common[b]], wB = rowW[common[b + 1]];
            std::vector<uint32_t> ring{vid[c][common[b]], vid[cp][common[b]]};
            for (int k : R) {
                if (rowW[k] > wA + 1e-12 && rowW[k] < wB - 1e-12) {
                    ring.push_back(vid[cp][k]);
                }
            }
            ring.push_back(vid[cp][common[b + 1]]);
            ring.push_back(vid[c][common[b + 1]]);
            for (auto it = L.rbegin(); it != L.rend(); ++it) {
                if (rowW[*it] > wA + 1e-12 && rowW[*it] < wB - 1e-12) {
                    ring.push_back(vid[c][*it]);
                }
            }
            emitRing(std::move(ring));
        }
    }

    // Transition strip: the cut rim's base-arc samples (exact, welded to
    // the neighbour) bridged to the keyBot uniform row by a monotone u map
    // — quads where the counts advance together, an n-gon absorbing each
    // extra point. Wraps once around through the seam.
    auto emitStrip = [&](const std::vector<uint32_t>& lowIds,
                         const std::vector<double>& lowU,
                         const std::vector<uint32_t>& highIds,
                         const std::vector<double>& highU) {
        const int nL = int(lowIds.size()) - 1, nH = int(highIds.size()) - 1;
        if (nL < 1 || nH < 1) return false;
        const bool lowSparse = nL <= nH;
        const std::vector<uint32_t>& Sp = lowSparse ? lowIds : highIds;
        const std::vector<double>& sU = lowSparse ? lowU : highU;
        const std::vector<uint32_t>& D = lowSparse ? highIds : lowIds;
        const std::vector<double>& dU = lowSparse ? highU : lowU;
        const int m = int(Sp.size()) - 1, n = int(D.size()) - 1;
        std::vector<int> mp(m + 1);
        mp[0] = 0;
        mp[m] = n;
        for (int k = 1; k < m; ++k) {
            int j = mp[k - 1];
            while (j + 1 < n && std::abs(dU[j + 1] - sU[k]) <=
                                    std::abs(dU[j] - sU[k])) {
                ++j;
            }
            mp[k] = j;
        }
        for (int k = 0; k < m; ++k) {
            std::vector<uint32_t> ring;
            if (lowSparse) {
                ring = {Sp[k], Sp[k + 1]};
                for (int t = mp[k + 1]; t >= mp[k]; --t) ring.push_back(D[t]);
            } else {
                for (int t = mp[k]; t <= mp[k + 1]; ++t) ring.push_back(D[t]);
                ring.push_back(Sp[k + 1]);
                ring.push_back(Sp[k]);
            }
            emitRing(std::move(ring));
        }
        return true;
    };
    auto unwrapAsc = [&](std::vector<double>& us) {
        for (size_t i = 1; i < us.size(); ++i) {
            while (us[i] < us[i - 1]) us[i] += period;
        }
    };
    // Cut-rim strip: the hug ARC (right wall top, around outside the
    // notch through the seam, to left wall top) bridged to the keyBot
    // uniform row. Wraps once around.
    {
        std::vector<uint32_t> lowIds, highIds;
        std::vector<double> lowU, highU;
        const int arcStart = rightWall.front(), arcEnd = leftWall.front();
        for (int step = 0, i = arcStart; step < SN; ++step, i = (i + 1) % SN) {
            lowIds.push_back(cutIds[i]);
            lowU.push_back(S[i].u);
            if (i == arcEnd) break;
        }
        for (int step = 0, c = colR; step < nu; ++step, c = (c + 1) % nu) {
            highIds.push_back(vid[c][keyBot]);
            highU.push_back(uk[c]);
            if (c == colL) break;
        }
        unwrapAsc(lowU);
        unwrapAsc(highU);
        if (!emitStrip(lowIds, lowU, highIds, highU)) return false;
    }

    // Notch region: welded through STRUCTURED strips so the columns stay
    // straight (an ear-clip fan would slant across the notch). The two
    // walls are u-iso (constant azimuth = vertical rulings): a vertical
    // strip v-matches each bounding column to its wall samples, so every
    // large-dz edge is a vertical ruling (dAz = 0). The floor is v-iso: a
    // horizontal strip u-matches the bounding-column feature row to the
    // floor samples. Corners are shared edges with the cut-rim strip
    // (top) and the below-feature lattice (bottom).
    // Left wall: wall samples <-> colL column (cut rim .. floor). Wall is
    // the low-u rail here so the shared colL edges wind opposite the
    // lattice cell on colL's left (manifold).
    {
        std::vector<uint32_t> colIds, wallIds;
        std::vector<double> colW, wallW;
        for (int k : colKeys[colL]) {
            if (rowW[k] <= rowW[keyFeat] + 1e-9) {
                colIds.push_back(vid[colL][k]);
                colW.push_back(rowW[k]);
            }
        }
        for (int i : leftWall) {
            wallIds.push_back(cutIds[i]);
            wallW.push_back(wOf(S[i].v));
        }
        if (!emitStrip(wallIds, wallW, colIds, colW)) return false;
    }
    // Right wall: colR column <-> wall samples (colR is the low-u rail so
    // its shared edges wind opposite the lattice cell on colR's right).
    {
        std::vector<uint32_t> colIds, wallIds;
        std::vector<double> colW, wallW;
        for (int i : rightWall) {
            wallIds.push_back(cutIds[i]);
            wallW.push_back(wOf(S[i].v));
        }
        for (int k : colKeys[colR]) {
            if (rowW[k] <= rowW[keyFeat] + 1e-9) {
                colIds.push_back(vid[colR][k]);
                colW.push_back(rowW[k]);
            }
        }
        if (!emitStrip(colIds, colW, wallIds, wallW)) return false;
    }
    // Floor: the bounding columns' feature row (colL..colR) <-> the floor
    // edge samples (the wall bottoms are the shared end corners).
    {
        std::vector<uint32_t> gridIds, floorIds;
        std::vector<double> gridU, floorU;
        for (int c = colL; c <= colR; ++c) {
            gridIds.push_back(vid[c][keyFeat]);
            gridU.push_back(uk[c]);
        }
        for (int i = fL; i <= fR; ++i) {
            floorIds.push_back(cutIds[i]);
            floorU.push_back(S[i].u);
        }
        if (!emitStrip(floorIds, floorU, gridIds, gridU)) return false;
    }

    dbg("rimnotch face %d: nu=%d nv=%d cols[%d,%d] wTop=%.3f polys=%zu",
        faceId, nu, nv, colL, colR, wTop / wspan, local.polygons.size());
    std::vector<uint32_t> outMap(local.vertices.size());
    for (uint32_t i = 0; i < local.vertices.size(); ++i) {
        outMap[i] = out.addVertex(
            gp_Pnt(local.vertices[i][0], local.vertices[i][1],
                   local.vertices[i][2]),
            local.anchors[i]);
    }
    for (const auto& poly : local.polygons) {
        std::vector<uint32_t> mapped;
        mapped.reserve(poly.size());
        for (uint32_t idx2 : poly) mapped.push_back(outMap[idx2]);
        out.addPolygon(std::move(mapped), faceId, false);
    }
    return true;
}


}  // namespace weft::mesher_impl
