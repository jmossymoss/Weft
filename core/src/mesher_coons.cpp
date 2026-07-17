#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// ---------------------------------------------------------------------------
// Coons patch for four-sided freeform faces (plan §3.5): a structured grid
// blended between the four boundary pcurves in UV. Turns bspline strips
// into flowing quads instead of chord triangles, and its opposite sides
// take part in density matching like any grid.


// Build the patch from the face's outer wire: four sides chained
// head-to-tail with pcurves, interior probes inside the face. Tolerated
// wire noise: ONE degenerate edge (a bspline pole — becomes the
// collapsed side, its pcurve covering the UV gap) or, among five edges,
// ONE corner stub far shorter than the sides (skipped in the chain and
// stitched into the corner polygon at mesh time). `rotate` shifts which
// edge becomes side 0 — it picks the corner the grid anchors to and, on
// triangular patches, which corner the fan terminates in.
bool makeCoonsPatch(const TopoDS_Face& face, const Model& model,
                    CoonsPatch& patch, int rotate,
                    const char** why,
                    bool* reflexPlanar) {
    auto reject = [&](const char* r) {
        if (why) *why = r;
        return false;
    };
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return reject("no outer wire");
    // Extra wires are HOLES: fine on CURVED charts as long as each sits
    // strictly inside the outer wire's UV box — the grid meshes whole,
    // the covered cells are cut out and webbed to the hole's exact
    // border after. Planar plates with holes stay with quad-fill /
    // plate-web, which own flat topology.
    if (BRepAdaptor_Surface(face).GetType() == GeomAbs_Plane) {
        TopExp_Explorer wx(face, TopAbs_WIRE);
        if (wx.More()) {
            wx.Next();
            if (wx.More()) return reject("face has holes");
        }
    }
    {
        double ou0 = 1e300, ou1 = -1e300, ov0 = 1e300, ov1 = -1e300;
        auto wireBox = [&](const TopoDS_Shape& w, double& u0, double& u1,
                           double& v0, double& v1) {
            u0 = 1e300; u1 = -1e300; v0 = 1e300; v1 = -1e300;
            for (TopExp_Explorer ex(w, TopAbs_EDGE); ex.More(); ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                if (BRep_Tool::Degenerated(e)) continue;
                double f, l;
                Handle(Geom2d_Curve) pc =
                    BRep_Tool::CurveOnSurface(e, face, f, l);
                if (pc.IsNull()) continue;
                for (int k = 0; k <= 12; ++k) {
                    gp_Pnt2d uv = pc->Value(f + (l - f) * k / 12.0);
                    u0 = std::min(u0, uv.X()); u1 = std::max(u1, uv.X());
                    v0 = std::min(v0, uv.Y()); v1 = std::max(v1, uv.Y());
                }
            }
        };
        wireBox(outer, ou0, ou1, ov0, ov1);
        patch.outerBox = {ou0, ou1, ov0, ov1};
        const double mu = 0.03 * std::max(1e-12, ou1 - ou0);
        const double mv = 0.03 * std::max(1e-12, ov1 - ov0);
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            if (wx.Current().IsSame(outer)) continue;
            double u0, u1, v0, v1;
            wireBox(wx.Current(), u0, u1, v0, v1);
            if (u0 <= ou0 + mu || u1 >= ou1 - mu || v0 <= ov0 + mv ||
                v1 >= ov1 - mv) {
                return reject("face has holes");  // rim-touching hole
            }
            std::vector<int> ids;
            for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More();
                 ex.Next()) {
                int eid = model.edges.FindIndex(ex.Current());
                if (eid > 0) ids.push_back(eid);
            }
            if (ids.empty()) return reject("face has holes");
            patch.holeWires.push_back(std::move(ids));
            patch.holeBoxes.push_back({u0, u1, v0, v1});
        }
    }
    struct WireEdge {
        TopoDS_Edge edge;
        bool degenerate;
        double len;
    };
    std::vector<WireEdge> all;
    for (BRepTools_WireExplorer wx(outer, face); wx.More(); wx.Next()) {
        if (all.size() >= 24) return reject("more than 24 edges");
        const TopoDS_Edge edge = wx.Current();
        double f, l;
        Handle(Geom2d_Curve) pcurve =
            BRep_Tool::CurveOnSurface(edge, face, f, l);
        if (pcurve.IsNull()) return reject("pcurve missing");
        const bool degen = BRep_Tool::Degenerated(edge);
        double len = 0.0;
        if (!degen) {
            BRepAdaptor_Curve c(edge);
            len = GCPnts_AbscissaPoint::Length(c);
        }
        all.push_back({edge, degen, len});
    }
    // The "gap": a degenerate pole edge, or a corner stub (five edges,
    // shortest under 2% of the perimeter).
    int gap = -1;
    for (size_t i = 0; i < all.size(); ++i) {
        if (all[i].degenerate) {
            if (gap >= 0) return reject("two pole edges");
            gap = int(i);
        }
    }
    if (gap < 0 && all.size() == 5) {
        double perim = 0.0;
        int shortest = 0;
        for (size_t i = 0; i < all.size(); ++i) {
            perim += all[i].len;
            if (all[i].len < all[shortest].len) shortest = int(i);
        }
        // No short stub: fall through to corner-angle chaining below.
        if (all[shortest].len < 0.02 * perim) gap = shortest;
    }
    // A pole edge can be accompanied by one microscopic REAL edge at either
    // end (the revolved profile missed the axis by STEP tolerance).  That
    // sliver is not a fourth patch side: retain it as a local corner stitch
    // while the remaining three sides use the collapsed-pole grid.  Both
    // adjacent edges must not be tiny, otherwise the pole layout is
    // ambiguous and keeps the old conservative path.
    int poleStub = -1;
    if (gap >= 0 && all[gap].degenerate && all.size() == 5) {
        double perim = 0.0;
        for (const auto& e : all) perim += e.len;
        const int before = (gap + int(all.size()) - 1) % int(all.size());
        const int after = (gap + 1) % int(all.size());
        const bool beforeTiny = all[before].len < 0.02 * perim;
        const bool afterTiny = all[after].len < 0.02 * perim;
        if (beforeTiny != afterTiny) poleStub = beforeTiny ? before : after;
    }
    const int nReal = int(all.size()) - (gap >= 0 ? 1 : 0) -
                      (poleStub >= 0 ? 1 : 0);
    if (nReal < 3) return reject("under 3 real edges");

    // Order the real sides starting AFTER the gap, so the gap sits
    // between side (nReal-1)'s end and side 0's start. Plain patches
    // (no gap) honour the requested rotation instead.
    std::vector<int> order;
    const int start = gap >= 0 ? (gap + 1) % int(all.size()) : 0;
    for (int k = 0; k < int(all.size()); ++k) {
        int i = (start + k) % int(all.size());
        if (i != gap && i != poleStub) order.push_back(i);
    }

    // More than four sides: group consecutive edges into FOUR sides at
    // the sharpest wire corners (a curved band whose rail is split by a
    // T-junction is still a four-sided patch). Turn angle at each joint
    // comes from the 3D end tangents; the gap (pole/stub) is always a
    // corner.
    // Tangents and SIGNED turns at wire joints. The sign comes from the
    // oriented surface normal: positive = convex (interior < 180 deg),
    // negative = reflex. A reflex corner breaks the whole four-sided
    // abstraction — transfinite interpolation over such a domain must
    // fold — so it rejects the patch instead of shipping folded cells.
    auto tangentAt = [&](int src, bool atEnd) -> gp_Vec {
        BRepAdaptor_Curve c(all[src].edge);
        const bool rev = all[src].edge.Orientation() == TopAbs_REVERSED;
        const double par = (atEnd != rev) ? c.LastParameter()
                                          : c.FirstParameter();
        gp_Pnt pp;
        gp_Vec d;
        c.D1(par, pp, d);
        if (rev) d.Reverse();
        return d;
    };
    BRepAdaptor_Surface signSurf(face);
    auto signedTurnAt = [&](size_t k, const std::vector<int>& ord) {
        int prev = ord[(k + ord.size() - 1) % ord.size()];
        gp_Vec a = tangentAt(prev, true);
        gp_Vec b = tangentAt(ord[k], false);
        if (a.Magnitude() < 1e-12 || b.Magnitude() < 1e-12) return 0.0;
        double f2, l2;
        Handle(Geom2d_Curve) pc =
            BRep_Tool::CurveOnSurface(all[ord[k]].edge, face, f2, l2);
        if (pc.IsNull()) return double(a.Angle(b));
        const bool rev = all[ord[k]].edge.Orientation() == TopAbs_REVERSED;
        gp_Pnt2d uv = pc->Value(rev ? l2 : f2);
        gp_Pnt sp;
        gp_Vec du, dv;
        signSurf.D1(uv.X(), uv.Y(), sp, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
        if (n.Magnitude() < 1e-12) return double(a.Angle(b));
        return std::atan2(a.Crossed(b).Dot(n) / n.Magnitude(), a.Dot(b));
    };

    std::vector<size_t> sideStart;  // indices into `order` that begin sides
    if (nReal > 4) {
        // Joint k sits BEFORE order[k] (between order[k-1] and order[k]).
        // With a gap (pole/stub) joint 0 is FORCED to be a corner; on a
        // plain closed wire it gets its real turn like every other joint,
        // or a smooth wire-start would steal a real corner's slot.
        std::vector<double> turn(order.size(), M_PI);
        std::vector<double> signedT(order.size(), M_PI);
        for (size_t k = gap >= 0 ? 1 : 0; k < order.size(); ++k) {
            signedT[k] = signedTurnAt(k, order);
            turn[k] = std::abs(signedT[k]);
        }
        std::vector<size_t> byTurn(order.size());
        std::iota(byTurn.begin(), byTurn.end(), 0);
        std::sort(byTurn.begin(), byTurn.end(),
                  [&](size_t x, size_t y) { return turn[x] > turn[y]; });
        // Need four clear corners; a fuzzy fourth means this isn't a
        // four-sided patch.
        if (turn[byTurn[3]] < 20.0 * M_PI / 180.0) return reject("no clear fourth corner");
        sideStart = {byTurn[0], byTurn[1], byTurn[2], byTurn[3]};
        std::sort(sideStart.begin(), sideStart.end());
        if (gap >= 0 && sideStart[0] != 0) return reject("gap not at a corner");
        // Reflex screening is DETECTION only, and PLANAR only: the
        // caller may prefer quad-fill for a flat chevron (transfinite
        // interpolation over a reflex domain must fold), but the patch
        // itself stays valid — in modes without a better planar mesher
        // the untangler and the fold overlay handle the outcome.
        if (reflexPlanar && signSurf.GetType() == GeomAbs_Plane) {
            for (size_t k = 0; k < order.size(); ++k) {
                if (gap >= 0 && k == 0) continue;  // forced gap corner
                const bool isCorner = std::find(sideStart.begin(),
                                                sideStart.end(),
                                                k) != sideStart.end();
                if (isCorner && signedT[k] < -45.0 * M_PI / 180.0) {
                    *reflexPlanar = true;
                }
                if (!isCorner && signedT[k] < -60.0 * M_PI / 180.0) {
                    *reflexPlanar = true;
                }
            }
        }
        // Rotate `order` so a corner is first, keeping the gap corner
        // first when there is one.
        if (gap < 0) {
            size_t shift =
                sideStart[size_t(rotate) % sideStart.size()] % order.size();
            if (shift) {
                std::rotate(order.begin(), order.begin() + shift,
                            order.end());
                for (size_t& v : sideStart) {
                    v = (v + order.size() - shift) % order.size();
                }
                std::sort(sideStart.begin(), sideStart.end());
            }
        }
    } else {
        if (gap < 0 && rotate > 0) {
            std::rotate(order.begin(),
                        order.begin() + (rotate % order.size()),
                        order.end());
        }
        for (size_t k = 0; k < std::min<size_t>(4, order.size()); ++k) {
            sideStart.push_back(k);
        }
        // Four plain sides: every joint is a corner, and a reflex one
        // (a dart-shaped face) folds the grid just like the chained
        // case. Detection only, planar only, same reasoning as above.
        if (reflexPlanar && nReal == 4 &&
            signSurf.GetType() == GeomAbs_Plane) {
            for (size_t k = gap >= 0 ? 1 : 0; k < order.size(); ++k) {
                if (signedTurnAt(k, order) < -45.0 * M_PI / 180.0) {
                    *reflexPlanar = true;
                }
            }
        }
    }

    auto pieceOf = [&](int src) {
        CoonsPatch::SidePiece pce;
        const TopoDS_Edge& edge = all[src].edge;
        pce.pc = BRep_Tool::CurveOnSurface(edge, face, pce.f, pce.l);
        pce.rev = edge.Orientation() == TopAbs_REVERSED;
        pce.edgeId = model.edges.FindIndex(edge);
        pce.len = all[src].len;
        return pce;
    };
    // Fill the four sides (legacy arrays mirror each side's first piece).
    const int nSides = nReal == 3 ? 3 : 4;
    for (int sIdx = 0; sIdx < nSides; ++sIdx) {
        size_t from = sideStart[sIdx];
        size_t to = sIdx + 1 < int(sideStart.size())
                        ? sideStart[sIdx + 1]
                        : order.size();
        for (size_t k = from; k < to; ++k) {
            patch.chain[sIdx].push_back(pieceOf(order[k]));
            if (patch.chain[sIdx].front().edgeId < 1) return reject("side has invalid edge");
        }
        // A "side" that swallowed a third of the outline isn't a side —
        // the face isn't four-cornered, and forcing a transfinite grid
        // through it slivers and folds (the flaregun underside panel:
        // one side chained ELEVEN edges). Quad-fill owns those.
        if (patch.chain[sIdx].size() > 8) {
            return reject("side chains too many edges");
        }
        const auto& p0 = patch.chain[sIdx].front();
        patch.pc[sIdx] = p0.pc;
        patch.first[sIdx] = p0.f;
        patch.last[sIdx] = p0.l;
        patch.rev[sIdx] = p0.rev;
        patch.edgeIds[sIdx] = p0.edgeId;
    }
    auto fill = [&](int slot, int src) {
        const TopoDS_Edge& edge = all[src].edge;
        double f, l;
        patch.pc[slot] = BRep_Tool::CurveOnSurface(edge, face, f, l);
        patch.first[slot] = f;
        patch.last[slot] = l;
        patch.rev[slot] = edge.Orientation() == TopAbs_REVERSED;
        patch.edgeIds[slot] = model.edges.FindIndex(edge);
    };
    if (nReal == 3) {
        patch.collapsedLast = true;
        if (gap >= 0) {
            // Pole from a degenerate edge: its pcurve spans the UV gap.
            fill(3, gap);
            patch.poleCurve = true;
            patch.edgeIds[3] = patch.edgeIds[1];  // density: v follows side 1
        } else {
            // Plain triangle: side 3 is the sides-0/2 corner point.
            patch.edgeIds[3] = patch.edgeIds[2];
            patch.pc[3] = patch.pc[2];
            patch.first[3] = patch.last[3] = 0.0;
            patch.rev[3] = false;
        }
    }
    auto rememberStub = [&](int src, int afterSide) {
        const TopoDS_Edge& stub = all[src].edge;
        patch.stubEdgeId = model.edges.FindIndex(stub);
        patch.stubAfterSide = afterSide;
        patch.stubRev = stub.Orientation() == TopAbs_REVERSED;
        patch.stubPc = BRep_Tool::CurveOnSurface(stub, face,
                                                 patch.stubFirst,
                                                 patch.stubLast);
        return patch.stubEdgeId >= 1 && !patch.stubPc.IsNull();
    };
    if (poleStub >= 0) {
        const int before = (gap + int(all.size()) - 1) % int(all.size());
        // Before the degenerate edge: side2 -> stub -> pole side.
        // After it: pole side -> stub -> side0.
        if (!rememberStub(poleStub, poleStub == before ? 2 : 3)) {
            return reject("pole stub edge unknown");
        }
    } else if (gap >= 0 && nReal != 3) {
        // Four real sides + corner stub: remember it for stitching.
        if (!rememberStub(gap, 3)) return reject("stub edge unknown");
    }
    const int sides = patch.collapsedLast && !patch.poleCurve ? 3 : 4;
    for (int i = 0; i < sides; ++i) {
        if (patch.edgeIds[i] < 1) return reject("side edge unknown");
    }
    // Head-to-tail continuity in UV (a seam on a periodic surface breaks
    // the chain; such faces are not Coons candidates).
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double span = std::max(umax - umin, vmax - vmin);
    for (int i = 0; i < sides; ++i) {
        gp_Pnt2d a = patch.side(i, 1.0);
        gp_Pnt2d b = patch.side((i + 1) % 4, 0.0);
        if (patch.collapsedLast && !patch.poleCurve && i == 2) {
            b = patch.side(0, 0.0);
        }
        // Tolerance noise on exported pcurves reaches ~1e-3 of the span;
        // an actual seam jump is on the order of the span itself. The
        // corner carrying a stub legitimately jumps by the stub's length.
        double allow = 0.02 * span;
        if (patch.stubEdgeId > 0 && i == patch.stubAfterSide &&
            !patch.stubPc.IsNull()) {
            allow += patch.stubPc->Value(patch.stubFirst)
                         .Distance(patch.stubPc->Value(patch.stubLast));
        }
        if (a.Distance(b) > allow) return reject("sides not head-to-tail in UV");
    }
    // Interior probes must land inside the face.
    const double tol = BRep_Tool::Tolerance(face);
    for (int j = 1; j < 4; ++j) {
        for (int i = 1; i < 4; ++i) {
            gp_Pnt2d p = patch.uv(i / 4.0, j / 4.0);
            bool inHole = false;
            for (const auto& b : patch.holeBoxes) {
                if (p.X() >= b[0] && p.X() <= b[1] && p.Y() >= b[2] &&
                    p.Y() <= b[3]) {
                    inHole = true;
                    break;
                }
            }
            if (inHole) continue;  // the cutout web covers that region
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), p,
                                         tol);
            if (cls.State() == TopAbs_OUT) return reject("interior probe outside face");
        }
    }
    return true;
}

// Emit the Coons grid. Single-edge sides sample at uParams/vParams (the
// 0..1 splits, clustered for fillet strips); CHAINED sides sample each
// piece at its own solved count, so the border matches every neighbour
// vertex-for-vertex and the grid gains a column at each T-junction.
// Lattice count floors for a holed patch: each hole should span roughly
// HALF a cell, so the cutout takes 1-2 cells and the collar + one ring
// of webbing absorbs it — the reference absorption pattern — without a
// global density explosion. Structural (holes demand lattice lines even
// across dead-straight borders), so it survives the density dial.
// Measured PHYSICALLY (3D lengths), not in uv — bspline parameter space
// compresses and a small hole can read as a third of the domain.
void insertCountFloors(const TopoDS_Face& face, const CoonsPatch& patch,
                       int& minU, int& minV) {
    minU = minV = 0;
    if (patch.holeWires.empty()) return;
    Handle(Geom_Surface) S = BRep_Tool::Surface(face);
    if (S.IsNull()) return;
    const int kT = 64;
    std::array<std::vector<gp_Pnt2d>, 2> tab;
    double len[2] = {0, 0};
    for (int d = 0; d < 2; ++d) {
        gp_Pnt prev;
        for (int k = 0; k <= kT; ++k) {
            const double f = double(k) / kT;
            const gp_Pnt2d uv =
                d == 0 ? patch.uv(f, 0.5) : patch.uv(0.5, f);
            tab[d].push_back(uv);
            const gp_Pnt p = S->Value(uv.X(), uv.Y());
            if (k) len[d] += prev.Distance(p);
            prev = p;
        }
    }
    for (const auto& b : patch.holeBoxes) {
        const double ucm = 0.5 * (b[0] + b[1]);
        const double vcm = 0.5 * (b[2] + b[3]);
        const double hx =
            S->Value(b[0], vcm).Distance(S->Value(b[1], vcm));
        const double hy =
            S->Value(ucm, b[2]).Distance(S->Value(ucm, b[3]));
        for (int d = 0; d < 2; ++d) {
            double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
            for (const auto& q : tab[d]) {
                x0 = std::min(x0, q.X());
                x1 = std::max(x1, q.X());
                y0 = std::min(y0, q.Y());
                y1 = std::max(y1, q.Y());
            }
            const bool sweepsX = (x1 - x0) >= (y1 - y0);
            const double frac =
                (sweepsX ? hx : hy) / std::max(1e-12, len[d]);
            const int c = std::clamp(
                int(std::ceil(0.5 / std::max(0.02, frac))), 2, 16);
            int& slot = d == 0 ? minU : minV;
            slot = std::max(slot, c);
        }
    }
    dbg("insert floors: minU=%d minV=%d (mid-iso len %.3f / %.3f)", minU,
        minV, len[0], len[1]);
}

bool meshCoonsGridBody(const TopoDS_Face& face, const Model& model,
                       int faceId, const std::vector<double>& uParams,
                       const std::vector<double>& vParams, int rotate,
                       const std::vector<int>& solvedEdge, MeshBuilder& out,
                       const std::vector<double>* uScaffold = nullptr,
                       const std::vector<double>* vScaffold = nullptr,
                       const PinnedEdges* pins = nullptr,
                       bool decoupleSeams = false) {
    CoonsPatch patch;
    if (!makeCoonsPatch(face, model, patch, rotate)) return false;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    // Border vertices evaluate on the shared 3D edge curves, not through
    // this face's pcurve: both faces of an edge then produce bit-identical
    // points and the weld is exact.
    struct BPt {
        gp_Pnt p;
        gp_Pnt2d uv;
    };
    // One side sampled in wire direction. Single-piece sides honour the
    // given 0..1 splits; chains take each piece at its solved count.
    auto sampleSide = [&](int i,
                          const std::vector<double>& params)
        -> std::vector<BPt> {
        std::vector<BPt> row;
        const auto& ch = patch.chain[i];
        if (ch.size() <= 1) {
            const int eid = patch.edgeIds[i];
            BRepAdaptor_Curve c(TopoDS::Edge(model.edges(eid)));
            const double f3 = c.FirstParameter(), l3 = c.LastParameter();
            // A pinned rail carries explicit column-azimuth fractions
            // (forward-param) that both faces emit; the fraction maps the
            // edge directly (no rev flip — rev only reorders the list).
            if (edgeIsPinned(eid, pins)) {
                for (double tt : edgeSampleFractions(eid, 0, 0.0,
                                                     patch.rev[i], true, pins,
                                                     &model)) {
                    row.push_back(
                        {c.Value(f3 + tt * (l3 - f3)),
                         patch.pc[i]->Value(patch.first[i] +
                                            tt * (patch.last[i] -
                                                  patch.first[i]))});
                }
                return row;
            }
            // A UNIFORM request goes through the shared even-arc sampler so
            // a freeform single side spaces its divisions evenly in 3D and
            // welds bit-identically to whatever samples the other face of
            // this edge (analytic edges come back byte-identical). Only
            // deliberately CLUSTERED params (a fillet hold) are kept
            // verbatim — those faces are exempt from the border check.
            bool uniform = params.size() >= 2;
            for (size_t k = 0; k < params.size() && uniform; ++k) {
                uniform = std::abs(params[k] -
                                   double(k) / double(params.size() - 1)) <
                          1e-9;
            }
            const std::vector<double> frac =
                uniform ? edgeSampleFractions(eid, int(params.size()) - 1, 0.0,
                                              patch.rev[i], true, pins, &model)
                        : std::vector<double>{};
            if (uniform) {
                for (double tt : frac) {
                    row.push_back(
                        {c.Value(f3 + tt * (l3 - f3)),
                         patch.pc[i]->Value(patch.first[i] +
                                            tt * (patch.last[i] -
                                                  patch.first[i]))});
                }
                return row;
            }
            for (double t : params) {
                double tt = patch.rev[i] ? 1.0 - t : t;
                row.push_back(
                    {c.Value(f3 + tt * (l3 - f3)),
                     patch.pc[i]->Value(patch.first[i] +
                                        tt * (patch.last[i] -
                                              patch.first[i]))});
            }
            return row;
        }
        for (size_t k = 0; k < ch.size(); ++k) {
            const auto& pce = ch[k];
            int n = 1;
            if (pce.edgeId > 0 && pce.edgeId < int(solvedEdge.size()) &&
                solvedEdge[pce.edgeId] > 0) {
                n = solvedEdge[pce.edgeId];
            }
            BRepAdaptor_Curve c(TopoDS::Edge(model.edges(pce.edgeId)));
            const double f3 = c.FirstParameter(), l3 = c.LastParameter();
            if (edgeIsPinned(pce.edgeId, pins)) {
                for (double tt : edgeSampleFractions(
                         pce.edgeId, 0, 0.0, pce.rev,
                         /*includeLast=*/k + 1 == ch.size(), pins, &model)) {
                    row.push_back(
                        {c.Value(f3 + tt * (l3 - f3)),
                         pce.pc->Value(pce.f + tt * (pce.l - pce.f))});
                }
                continue;
            }
            // Each chain piece samples its own edge at even 3D arc length
            // (uniform for analytic edges — byte-identical), so a chained
            // side welds to the same shared curve the neighbour meshes.
            for (double tt : edgeSampleFractions(
                     pce.edgeId, n, 0.0, pce.rev,
                     /*includeLast=*/k + 1 == ch.size(), pins, &model)) {
                row.push_back(
                    {c.Value(f3 + tt * (l3 - f3)),
                     pce.pc->Value(pce.f + tt * (pce.l - pce.f))});
            }
        }
        return row;
    };
    auto uniformParams = [](int n) {
        std::vector<double> ps(n + 1);
        for (int i = 0; i <= n; ++i) ps[i] = double(i) / n;
        return ps;
    };

    // Bottom = side0 (wire dir), Top = side2 reversed, Right = side1,
    // Left = side3 reversed — so Bottom[i] pairs Top[i] and Left[j]
    // pairs Right[j], with (0,0) at side0's start.
    //
    // On a CHAINED patch every single side samples at its own SOLVED
    // count (its group never united with the opposite side); when the
    // chain pass couldn't reconcile opposite totals (shared rails,
    // cascades, user pins) the sizes differ and the face falls back
    // visibly instead of breaking the seam. Plain patches keep the
    // given (possibly clustered) splits.
    auto solvedCount = [&](int eid) {
        return eid > 0 && eid < int(solvedEdge.size()) && solvedEdge[eid] > 0
                   ? solvedEdge[eid]
                   : 1;
    };
    auto paramsFor = [&](int i,
                         const std::vector<double>& plain)
        -> std::vector<double> {
        if (patch.chain[i].size() > 1) return {};  // chain: per-piece
        if (patch.chained()) {
            return uniformParams(solvedCount(patch.edgeIds[i]));
        }
        // Plain patches: a UNIFORM request still samples at the edge's
        // OWN solved count — the border contract — with any rail
        // mismatch absorbed by the transition strips. Only deliberately
        // clustered splits (fillet holds) keep the given params; those
        // faces are exempt from the border check.
        bool uniform = true;
        for (size_t k = 0; k < plain.size() && uniform; ++k) {
            uniform = std::abs(plain[k] - double(k) /
                                              double(plain.size() - 1)) <
                      1e-9;
        }
        const int eid = patch.edgeIds[i];
        const int sc = eid > 0 && eid < int(solvedEdge.size())
                           ? solvedEdge[eid]
                           : 0;
        if (uniform && sc >= 1) return uniformParams(sc);
        return plain;
    };
    // (Tried and reverted: sampling a single side at the opposite
    // chain's arc fractions to kill rung skew — every single edge is
    // ALSO someone else's uniformly-sampled seam, and the sweep's opens
    // exploded 50x. Border positions are a shared contract; rung
    // alignment has to come from somewhere else.)
    std::vector<BPt> bottom = sampleSide(0, paramsFor(0, uParams));
    std::vector<BPt> top = sampleSide(2, paramsFor(2, uParams));
    std::reverse(top.begin(), top.end());

    // Decoupled rails: opposite totals may disagree now that the solver
    // no longer grows chains into equality. The deficit rail's NATURAL
    // points stay the emitted border — they are the neighbours' weld
    // contract and are never re-spaced (doctrine). The lattice itself
    // gets an arc-fraction resampling of that rail purely as blending
    // scaffold; at emission a transition strip of quads (plus a 5-gon
    // wherever a count is absorbed — the Plasticity pattern) stitches
    // the natural rail to the first interior grid line, and the
    // scaffold row is not emitted at all.
    // Scaffold rows resample the NATURAL rail by ARC LENGTH, not by
    // chain fraction: side(i,t) walks chains piece-by-piece, so a
    // T-junction with unequal piece densities shears every interior
    // column diagonally (the flaregun jacket rungs). Arc-uniform
    // scaffolds keep columns upright; UV interpolates within a natural
    // segment and re-evaluates on the surface.
    auto resampleAt = [&](const std::vector<BPt>& nat,
                          const std::vector<double>& fracs) {
        std::vector<double> arc(nat.size(), 0.0);
        for (size_t k = 1; k < nat.size(); ++k) {
            arc[k] = arc[k - 1] + nat[k].p.Distance(nat[k - 1].p);
        }
        const double total = arc.back() > 1e-12 ? arc.back() : 1.0;
        std::vector<BPt> row(fracs.size());
        size_t j = 0;
        for (size_t k = 0; k < fracs.size(); ++k) {
            const double sTarget = total * fracs[k];
            while (j + 2 < nat.size() && arc[j + 1] < sTarget) ++j;
            const double seg = std::max(1e-12, arc[j + 1] - arc[j]);
            const double t = std::clamp((sTarget - arc[j]) / seg, 0.0, 1.0);
            gp_Pnt2d uv(
                nat[j].uv.X() + t * (nat[j + 1].uv.X() - nat[j].uv.X()),
                nat[j].uv.Y() + t * (nat[j + 1].uv.Y() - nat[j].uv.Y()));
            row[k] = {surface->Value(uv.X(), uv.Y()), uv};
        }
        return row;
    };
    auto resample = [&](const std::vector<BPt>& nat, size_t n) {
        std::vector<double> fr(n);
        for (size_t k = 0; k < n; ++k) fr[k] = double(k) / double(n - 1);
        return resampleAt(nat, fr);
    };
    std::vector<BPt> natBottom, natTop;  // natural deficit rails
    if (bottom.size() != top.size() && bottom.size() >= 2 &&
        top.size() >= 2) {
        if (decoupleSeams) {
            // Decoupled seams: NO transition strip — the lattice's own
            // arc-uniform resampling of the deficit rail IS the emitted
            // border (points on the rail curve via surface eval). The
            // neighbour samples the shared edge at its own count; the
            // post-weld splice reconciles the two as seam n-gons.
            if (bottom.size() < top.size()) {
                bottom = resample(bottom, top.size());
            } else {
                top = resample(top, bottom.size());
            }
        } else if (bottom.size() < top.size()) {
            natBottom = bottom;
            bottom = resample(natBottom, top.size());
        } else {
            natTop = top;
            top = resample(natTop, bottom.size());
        }
    }
    // Interior densification (hole cutouts): the borders are a fixed
    // contract, so extra lattice lines come from scaffold rails at the
    // CALLER'S fractions — natural rails keep their own points (shared
    // fractions line up, so those columns still reach the border) and
    // the transition strips absorb only the added lines.
    if (uScaffold && uScaffold->size() > bottom.size() &&
        bottom.size() == top.size() && bottom.size() >= 2) {
        if (natBottom.empty()) natBottom = bottom;
        if (natTop.empty()) natTop = top;
        bottom = resampleAt(natBottom, *uScaffold);
        top = resampleAt(natTop, *uScaffold);
    }
    if (bottom.size() != top.size() || bottom.size() < 2) return false;
    const int nu = int(bottom.size()) - 1;

    std::vector<BPt> right = sampleSide(1, paramsFor(1, vParams));
    std::vector<BPt> left;
    if (patch.collapsedLast) {
        // Pole: the whole left column is one point (fan rows).
        left.assign(right.size(), {bottom.front().p,
                                   patch.side(3, 0.5)});
        for (size_t j = 0; j < left.size(); ++j) {
            left[j].uv = patch.side(3, 1.0 - double(j) / (right.size() - 1));
        }
    } else {
        left = sampleSide(3, paramsFor(3, vParams));
        std::reverse(left.begin(), left.end());
    }
    std::vector<BPt> natLeft, natRight;  // natural deficit rails
    if (right.size() != left.size() && right.size() >= 2 &&
        left.size() >= 2 && !patch.collapsedLast) {
        if (right.size() < left.size()) {
            natRight = right;
            right = resample(natRight, left.size());
        } else {
            natLeft = left;
            left = resample(natLeft, right.size());
        }
    }
    if (patch.collapsedLast && left.size() != right.size()) {
        left.assign(right.size(), left.empty() ? BPt{bottom.front().p,
                                                     patch.side(3, 0.5)}
                                               : left.front());
        for (size_t j = 0; j < left.size(); ++j) {
            left[j].uv = patch.side(3, 1.0 - double(j) / (right.size() - 1));
        }
    }
    if (vScaffold && vScaffold->size() > right.size() &&
        !patch.collapsedLast && right.size() == left.size() &&
        right.size() >= 2) {
        if (natLeft.empty()) natLeft = left;
        if (natRight.empty()) natRight = right;
        left = resampleAt(natLeft, *vScaffold);
        right = resampleAt(natRight, *vScaffold);
    }
    if (right.size() != left.size() || right.size() < 2) return false;
    const int nv = int(right.size()) - 1;

    // The (a,b) lattice follows the wire, whose UV handedness varies; the
    // Jacobian sign decides the polygon winding.
    gp_Pnt2d c0 = patch.uv(0.5, 0.5);
    gp_Pnt2d ca = patch.uv(0.55, 0.5);
    gp_Pnt2d cb = patch.uv(0.5, 0.55);
    const double jac = (ca.X() - c0.X()) * (cb.Y() - c0.Y()) -
                       (ca.Y() - c0.Y()) * (cb.X() - c0.X());
    const bool flip = (face.Orientation() == TopAbs_REVERSED) != (jac < 0);

    // Blend weights follow the borders' normalized arc positions so
    // clustered fillet rows stay clustered inside.
    auto arcWeights = [](const std::vector<BPt>& row) {
        std::vector<double> w(row.size(), 0.0);
        for (size_t i = 1; i < row.size(); ++i) {
            w[i] = w[i - 1] + row[i].p.Distance(row[i - 1].p);
        }
        double total = w.back() > 1e-12 ? w.back() : 1.0;
        for (double& x : w) x /= total;
        return w;
    };
    const std::vector<double> awB = arcWeights(bottom);
    const std::vector<double> awT = arcWeights(top);
    const std::vector<double> bwL = arcWeights(left);
    const std::vector<double> bwR = arcWeights(right);

    // Interior verts: discrete Coons blend of the border SAMPLES in 3D,
    // projected onto the surface. Blending in UV folds wherever a band's
    // pcurves bend tighter than the band is wide — EXCEPT on developable
    // charts (cylinders, cones), where the UV blend IS the ruling and
    // the 3D blend+projection is what wobbles (crumpled fillet bands).
    const GeomAbs_SurfaceType chartType =
        BRepAdaptor_Surface(face).GetType();
    const bool ruledChart = chartType == GeomAbs_Cylinder ||
                            chartType == GeomAbs_Cone;
    GeomAPI_ProjectPointOnSurf proj;
    proj.Init(gp_Pnt(0, 0, 0), surface);
    const gp_Pnt c00 = bottom.front().p, c10 = bottom.back().p;
    const gp_Pnt c11 = top.back().p, c01 = top.front().p;
    std::vector<BPt> gpts((nu + 1) * (nv + 1));
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            BPt bp;
            if (j == 0) bp = bottom[i];
            else if (j == nv) bp = top[i];
            else if (i == 0) bp = left[j];
            else if (i == nu) bp = right[j];
            else {
                // Bilinearly blended weights: rows near the top follow the
                // TOP border's spacing, not the bottom's. With a chained
                // side whose spacing drifts a step against the opposite
                // rail, single-border weights skew every interior rung the
                // same way until the last row folds over (bowtie cells).
                const double b0 = 0.5 * (bwL[j] + bwR[j]);
                const double a = (1.0 - b0) * awB[i] + b0 * awT[i];
                const double b = (1.0 - a) * bwL[j] + a * bwR[j];
                gp_XYZ blend =
                    bottom[i].p.XYZ() * (1 - b) + top[i].p.XYZ() * b +
                    left[j].p.XYZ() * (1 - a) + right[j].p.XYZ() * a -
                    (c00.XYZ() * ((1 - a) * (1 - b)) +
                     c10.XYZ() * (a * (1 - b)) + c11.XYZ() * (a * b) +
                     c01.XYZ() * ((1 - a) * b));
                gp_Pnt2d seed = patch.uv(a, b);
                bp.p = surface->Value(seed.X(), seed.Y());
                bp.uv = seed;
                if (!ruledChart) {
                    proj.Perform(gp_Pnt(blend));
                    if (proj.IsDone() && proj.NbPoints() > 0) {
                        bp.p = proj.NearestPoint();
                        double pu, pv;
                        proj.LowerDistanceParameters(pu, pv);
                        bp.uv.SetX(pu);
                        bp.uv.SetY(pv);
                    }
                }
            }
            gpts[j * (nu + 1) + i] = bp;
        }
    }

    // Untangle folded interiors: transfinite blending of a strongly
    // non-convex outline (a chevron plane, a chained strip drifting
    // against its rail) can cross its own rungs. Pinned-border Laplace
    // passes in UV pull interior points back inside the domain; a pass
    // is kept only when it strictly reduces the number of inverted
    // cells, so a wrapped periodic chart can never make things worse.
    if (nu > 1 && nv > 1) {
        auto countFlips = [&](const std::vector<BPt>& g) {
            double total = 0, meanAbs = 0;
            std::vector<double> areas;
            areas.reserve(size_t(nu) * nv);
            for (int j = 0; j < nv; ++j) {
                for (int i = 0; i < nu; ++i) {
                    const gp_Pnt2d& q00 = g[j * (nu + 1) + i].uv;
                    const gp_Pnt2d& q10 = g[j * (nu + 1) + i + 1].uv;
                    const gp_Pnt2d& q11 = g[(j + 1) * (nu + 1) + i + 1].uv;
                    const gp_Pnt2d& q01 = g[(j + 1) * (nu + 1) + i].uv;
                    double a2 =
                        (q10.X() - q00.X()) * (q11.Y() - q00.Y()) -
                        (q11.X() - q00.X()) * (q10.Y() - q00.Y()) +
                        (q11.X() - q00.X()) * (q01.Y() - q00.Y()) -
                        (q01.X() - q00.X()) * (q11.Y() - q00.Y());
                    areas.push_back(a2);
                    total += a2;
                    meanAbs += std::abs(a2);
                }
            }
            meanAbs /= double(std::max<size_t>(1, areas.size()));
            int flips = 0;
            for (double a2 : areas) {
                if (a2 * total < 0 && std::abs(a2) > 1e-3 * meanAbs) {
                    ++flips;
                }
            }
            return flips;
        };
        int bestFlips = countFlips(gpts);
        if (bestFlips > 0) {
            // In-place Gauss-Seidel, alternating sweep direction: roughly
            // twice Jacobi's convergence per pass, no directional bias.
            // Two schemes, each accepted pass-by-pass only on improvement:
            // plain Laplace handles extremely anisotropic charts (thin
            // strips), the Winslow stencil handles non-convex outlines
            // where a harmonic map itself must fold. Whatever survives is
            // the best grid either scheme reached.
            auto sweep = [&](std::vector<BPt>& sm, bool winslow, bool fwd) {
                for (int jj = 1; jj < nv; ++jj) {
                    const int j = fwd ? jj : nv - jj;
                    for (int ii = 1; ii < nu; ++ii) {
                        const int i = fwd ? ii : nu - ii;
                        const gp_Pnt2d& le = sm[j * (nu + 1) + i - 1].uv;
                        const gp_Pnt2d& ri = sm[j * (nu + 1) + i + 1].uv;
                        const gp_Pnt2d& dn = sm[(j - 1) * (nu + 1) + i].uv;
                        const gp_Pnt2d& up = sm[(j + 1) * (nu + 1) + i].uv;
                        BPt& b = sm[j * (nu + 1) + i];
                        if (!winslow) {
                            b.uv = gp_Pnt2d(0.25 * (le.X() + ri.X() +
                                                    dn.X() + up.X()),
                                            0.25 * (le.Y() + ri.Y() +
                                                    dn.Y() + up.Y()));
                            b.p = surface->Value(b.uv.X(), b.uv.Y());
                            continue;
                        }
                        const gp_Pnt2d& pp =
                            sm[(j + 1) * (nu + 1) + i + 1].uv;
                        const gp_Pnt2d& pm =
                            sm[(j - 1) * (nu + 1) + i + 1].uv;
                        const gp_Pnt2d& mp =
                            sm[(j + 1) * (nu + 1) + i - 1].uv;
                        const gp_Pnt2d& mm =
                            sm[(j - 1) * (nu + 1) + i - 1].uv;
                        const double xu = 0.5 * (ri.X() - le.X());
                        const double yu = 0.5 * (ri.Y() - le.Y());
                        const double xv = 0.5 * (up.X() - dn.X());
                        const double yv = 0.5 * (up.Y() - dn.Y());
                        const double al = xv * xv + yv * yv;
                        const double be = xu * xv + yu * yv;
                        const double ga = xu * xu + yu * yu;
                        const double den = 2.0 * (al + ga);
                        if (den < 1e-30) continue;
                        b.uv = gp_Pnt2d(
                            (al * (ri.X() + le.X()) +
                             ga * (up.X() + dn.X()) -
                             0.5 * be *
                                 (pp.X() - mp.X() - pm.X() + mm.X())) /
                                den,
                            (al * (ri.Y() + le.Y()) +
                             ga * (up.Y() + dn.Y()) -
                             0.5 * be *
                                 (pp.Y() - mp.Y() - pm.Y() + mm.Y())) /
                                den);
                        b.p = surface->Value(b.uv.X(), b.uv.Y());
                    }
                }
            };
            for (int scheme = 0; scheme < 2 && bestFlips > 0; ++scheme) {
                std::vector<BPt> sm = gpts;  // start from the best so far
                for (int pass = 0; pass < 40 && bestFlips > 0; ++pass) {
                    sweep(sm, scheme == 1, (pass & 1) == 0);
                    int flips = countFlips(sm);
                    if (flips < bestFlips) {
                        gpts = sm;
                        bestFlips = flips;
                    }
                }
            }
            if (bestFlips > 0) {
                dbg("coons: face %d still has %d inverted cell(s) after "
                    "untangling",
                    faceId, bestFlips);
            }
        }
    }

    // A deficit rail's scaffold row is NOT emitted — its natural points
    // are the border, bridged to the first interior line below. The
    // emitted grid shrinks by one row/column on each such side.
    const int j0 = natBottom.empty() ? 0 : 1;
    const int j1 = natTop.empty() ? nv : nv - 1;
    const int i0 = natLeft.empty() ? 0 : 1;
    const int i1 = natRight.empty() ? nu : nu - 1;

    std::vector<uint32_t> grid((nu + 1) * (nv + 1), 0);
    for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
            const BPt& bp = gpts[j * (nu + 1) + i];
            grid[j * (nu + 1) + i] =
                out.addVertex(bp.p, {faceId, bp.uv.X(), bp.uv.Y()});
        }
    }

    // Corner stub: sample its 3D curve at the solved count and stitch the
    // samples into the (0,0) corner polygon — the chain put the stub
    // between side 3's end and side 0's start, i.e. right there.
    std::vector<uint32_t> stubVerts;  // near end + interiors, wire order
    if (patch.stubEdgeId > 0) {
        int segs = patch.stubEdgeId < int(solvedEdge.size())
                       ? std::max(1, solvedEdge[patch.stubEdgeId])
                       : 1;
        BRepAdaptor_Curve sc(TopoDS::Edge(model.edges(patch.stubEdgeId)));
        const double f = sc.FirstParameter(), l = sc.LastParameter();
        // Even 3D arc on the stub too (near end + interiors, far endpoint
        // owned by the corner): a freeform stub then welds to whatever
        // samples this shared edge on the other face. Analytic stubs come
        // back byte-identical.
        for (double tt : edgeSampleFractions(patch.stubEdgeId, segs, 0.0,
                                             patch.stubRev,
                                             /*includeLast=*/false, pins,
                                             &model)) {
            gp_Pnt pos = sc.Value(f + (l - f) * tt);
            gp_Pnt2d p2 = patch.stubPc->Value(patch.stubFirst +
                                              tt * (patch.stubLast -
                                                    patch.stubFirst));
            stubVerts.push_back(
                out.addVertex(pos, {faceId, p2.X(), p2.Y()}));
        }
    }

    // A collapsed patch side (makeCoonsPatch's pole, or a rim that sampled
    // a near-zero corner edge) yields a COLUMN of degenerate cells: two of
    // the four corners land on the same point, so the weld later shears
    // each into a bare triangle -- the tri sunburst a coons pole sheds.
    // Emit the honest quad cells straight away; gather each degenerate cell
    // under its pole apex, then ladder the fan into quads (with one pentagon
    // when the fan has an odd number of rungs) so the pole caps
    // quad-dominant. Non-degenerate faces take exactly the old path.
    // A corner pair this close collapses under the global weld (default
    // 1e-6 mm) into the shared apex that a tri fan pivots on -- an order of
    // magnitude above that catches near-degenerate poles too, and stays far
    // below any legitimate cell edge, so honest faces never trip it.
    auto samePos = [&](uint32_t a, uint32_t b) {
        const auto& A = out.mesh().vertices[a];
        const auto& B = out.mesh().vertices[b];
        const double dx = A[0] - B[0], dy = A[1] - B[1], dz = A[2] - B[2];
        return dx * dx + dy * dy + dz * dz <= 1e-10;  // ~1e-5 mm
    };
    struct PoleFan {
        uint32_t apex;
        std::vector<std::array<uint32_t, 2>> segs;  // rim rungs, in cell order
    };
    std::vector<PoleFan> fans;
    for (int j = j0; j < j1; ++j) {
        for (int i = i0; i < i1; ++i) {
            std::array<uint32_t, 4> ring = {grid[j * (nu + 1) + i],
                                            grid[j * (nu + 1) + i + 1],
                                            grid[(j + 1) * (nu + 1) + i + 1],
                                            grid[(j + 1) * (nu + 1) + i]};
            const bool stubCorner =
                !stubVerts.empty() && i == 0 &&
                ((patch.stubAfterSide == 3 && j == 0) ||
                 (patch.stubAfterSide == 2 && j + 1 == j1));
            if (stubCorner) {
                std::vector<uint32_t> withStub(ring.begin(), ring.end());
                withStub.insert(withStub.end(), stubVerts.begin(),
                                stubVerts.end());
                out.addPolygon(std::move(withStub), faceId, flip);
                continue;
            }
            int deg = -1;
            for (int k = 0; k < 4; ++k) {
                if (samePos(ring[k], ring[(k + 1) % 4])) { deg = k; break; }
            }
            if (deg < 0) {
                out.addPolygon({ring[0], ring[1], ring[2], ring[3]}, faceId,
                               flip);
                continue;
            }
            // CCW triangle after the collapse: apex = the coincident corner,
            // the two rim verts are the far pair in winding order.
            const uint32_t apex = ring[deg];
            const std::array<uint32_t, 2> seg = {ring[(deg + 2) % 4],
                                                 ring[(deg + 3) % 4]};
            int fi = -1;
            for (size_t f = 0; f < fans.size(); ++f) {
                if (samePos(fans[f].apex, apex)) { fi = int(f); break; }
            }
            if (fi < 0) {
                fans.push_back({apex, {}});
                fi = int(fans.size()) - 1;
            }
            fans[fi].segs.push_back(seg);
        }
    }
    // Ladder every pole fan. The rungs are directed apex->a->b triangles;
    // chained end-to-start they give the rim in winding order (a fan on the
    // opposite side sweeps in reverse cell order, so chain by endpoints,
    // not by loop order). Pair rungs into apex quads, folding the odd tail
    // into a pentagon so no triangle survives. If the rungs don't form one
    // clean path (never seen on a real pole), emit them raw -- watertight.
    for (const PoleFan& fan : fans) {
        if (patch.poleCurve) {
            // A genuine analytic pole is the rounded end of a half-capsule.
            // Keep the standard capsule topology: fixed meridian count,
            // evenly spaced profile rings, and one honest triangle per
            // meridian only in the pole row. Merging pairs into apex quads
            // creates the large asymmetric wedges visible on MP9 and buys no
            // fidelity; here even span flow is more important than forcing
            // every cell to be a quad.
            for (const auto& s : fan.segs) {
                out.addPolygon({fan.apex,s[0],s[1]},faceId,flip);
            }
            continue;
        }
        std::map<uint32_t, std::array<uint32_t, 2>> byStart;
        std::set<uint32_t> isEnd;
        bool simple = true;
        for (const auto& s : fan.segs) {
            if (!byStart.emplace(s[0], s).second) simple = false;
            isEnd.insert(s[1]);
        }
        uint32_t head = fan.segs.front()[0];
        for (const auto& s : fan.segs) {
            if (!isEnd.count(s[0])) { head = s[0]; break; }
        }
        std::vector<uint32_t> rim{head};
        uint32_t cur = head;
        while (byStart.count(cur) && rim.size() <= fan.segs.size()) {
            cur = byStart[cur][1];
            rim.push_back(cur);
        }
        if (!simple || int(rim.size()) != int(fan.segs.size()) + 1) {
            for (const auto& s : fan.segs) {
                out.addPolygon({fan.apex, s[0], s[1]}, faceId, flip);
            }
            continue;
        }
        // Two rungs merge into an apex quad only where the pole surface
        // stays flat across them -- a crease (the tork jacket seam) would
        // fold the merged cell over the CAD normal. Gauge it by the two
        // rung triangles' normals: if they oppose, leave the rungs as
        // triangles rather than ship a fold.
        auto rungNormal = [&](uint32_t b, uint32_t c) {
            const auto& A = out.mesh().vertices[fan.apex];
            const auto& B = out.mesh().vertices[b];
            const auto& C = out.mesh().vertices[c];
            const double ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2];
            const double vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
            return gp_Vec(uy * vz - uz * vy, uz * vx - ux * vz,
                          ux * vy - uy * vx);
        };
        auto flat = [&](int i) {  // rungs i and i+1 fold-free as one quad
            const gp_Vec n1 = rungNormal(rim[i], rim[i + 1]);
            const gp_Vec n2 = rungNormal(rim[i + 1], rim[i + 2]);
            return n1.Magnitude() > 1e-20 && n2.Magnitude() > 1e-20 &&
                   n1.Dot(n2) > 0.0;
        };
        const int segCount = int(rim.size()) - 1;
        int t = 0;
        while (t < segCount) {
            const int left = segCount - t;
            if (left == 3 && flat(t) && flat(t + 1)) {
                // Pentagon soaks up the odd tail across three flat rungs.
                out.addPolygon({fan.apex, rim[t], rim[t + 1], rim[t + 2],
                                rim[t + 3]},
                               faceId, flip);
                t += 3;
            } else if (left >= 2 && flat(t)) {
                out.addPolygon({fan.apex, rim[t], rim[t + 1], rim[t + 2]},
                               faceId, flip);
                t += 2;
            } else {  // a crease or a lone tail rung: an honest triangle
                out.addPolygon({fan.apex, rim[t], rim[t + 1]}, faceId, flip);
                t += 1;
            }
        }
    }

    // Transition strips for the deficit rails: natural border points
    // bridge to the first interior grid line with a monotone index map —
    // quads where the counts advance together, a 5-gon wherever the
    // dense line contributes an extra point. Lattice-CCW is low line
    // forward then high line backward, matching the grid cells' winding.
    auto railIds = [&](const std::vector<BPt>& row) {
        std::vector<uint32_t> ids(row.size());
        for (size_t k = 0; k < row.size(); ++k) {
            ids[k] = out.addVertex(row[k].p, {faceId, row[k].uv.X(),
                                              row[k].uv.Y()});
        }
        return ids;
    };
    auto arcFractions = [&](const std::vector<uint32_t>& ids) {
        // Normalized cumulative arc of an emitted vertex sequence.
        std::vector<double> arc(ids.size(), 0.0);
        for (size_t k = 1; k < ids.size(); ++k) {
            const auto& A = out.mesh().vertices[ids[k - 1]];
            const auto& B = out.mesh().vertices[ids[k]];
            arc[k] = arc[k - 1] + std::sqrt((B[0] - A[0]) * (B[0] - A[0]) +
                                            (B[1] - A[1]) * (B[1] - A[1]) +
                                            (B[2] - A[2]) * (B[2] - A[2]));
        }
        const double total = arc.back() > 1e-12 ? arc.back() : 1.0;
        for (double& x : arc) x /= total;
        return arc;
    };
    // stubTail (with skipFirst / cornerAfter) reattaches the corner stub
    // when the cell that used to carry it was replaced by a strip.
    auto emitStrip = [&](const std::vector<uint32_t>& low,
                         const std::vector<uint32_t>& high,
                         const std::vector<uint32_t>* stubTail,
                         bool skipFirstStub, uint32_t cornerAfter) {
        const int nLow = int(low.size()) - 1;
        const int nHigh = int(high.size()) - 1;
        if (nLow < 1 || nHigh < 1) return;
        const bool lowSparse = nLow <= nHigh;
        const std::vector<uint32_t>& S = lowSparse ? low : high;
        const std::vector<uint32_t>& D = lowSparse ? high : low;
        // Chained rails are piecewise-nonuniform: map by ARC fraction,
        // not index, or the bridge crosses arc positions into long
        // diagonal slivers (the zig-zag band).
        const std::vector<double> sArc = arcFractions(S);
        const std::vector<double> dArc = arcFractions(D);
        const int m = int(S.size()) - 1;
        const int n = int(D.size()) - 1;
        std::vector<int> mp(m + 1);
        mp[0] = 0;
        mp[m] = n;
        for (int k = 1; k < m; ++k) {
            int j = mp[k - 1];
            while (j + 1 < n && std::abs(dArc[j + 1] - sArc[k]) <=
                                    std::abs(dArc[j] - sArc[k])) {
                ++j;
            }
            mp[k] = j;
        }
        for (int k = 0; k < m; ++k) {
            const int a = mp[k];
            const int b = mp[k + 1];
            std::vector<uint32_t> ring;
            if (lowSparse) {
                ring = {S[k], S[k + 1]};
                for (int t = b; t >= a; --t) ring.push_back(D[t]);
            } else {
                for (int t = a; t <= b; ++t) ring.push_back(D[t]);
                ring.push_back(S[k + 1]);
                ring.push_back(S[k]);
            }
            ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
            if (ring.size() > 1 && ring.front() == ring.back()) {
                ring.pop_back();
            }
            if (k == 0 && stubTail && !stubTail->empty()) {
                ring.insert(ring.end(),
                            stubTail->begin() + (skipFirstStub ? 1 : 0),
                            stubTail->end());
                if (cornerAfter != UINT32_MAX) ring.push_back(cornerAfter);
            }
            if (ring.size() < 3) continue;
            out.addPolygon(std::move(ring), faceId, flip);
        }
    };
    if (!natBottom.empty()) {
        std::vector<uint32_t> high;
        for (int i = i0; i <= i1; ++i) high.push_back(grid[j0 * (nu + 1) + i]);
        emitStrip(railIds(natBottom), high,
                  stubVerts.empty() || patch.stubAfterSide != 3
                      ? nullptr
                      : &stubVerts,
                  false,
                  UINT32_MAX);
    }
    if (!natTop.empty()) {
        std::vector<uint32_t> low;
        for (int i = i0; i <= i1; ++i) low.push_back(grid[j1 * (nu + 1) + i]);
        emitStrip(low, railIds(natTop), nullptr, false, UINT32_MAX);
    }
    if (!natLeft.empty()) {
        std::vector<uint32_t> low;
        for (int j = j0; j <= j1; ++j) low.push_back(grid[j * (nu + 1) + i0]);
        // The stub's far end is side0's start — a contract point that
        // lives at lattice (0,0), outside the emitted grid here. Close
        // the strip through it. (stubVerts[0] coincides with the natural
        // left rail's first point, so it is skipped.)
        const bool stubHere = !stubVerts.empty() && natBottom.empty() &&
                              patch.stubAfterSide == 3;
        uint32_t corner00 = UINT32_MAX;
        if (stubHere) {
            const BPt& c = gpts[0];
            corner00 = out.addVertex(c.p, {faceId, c.uv.X(), c.uv.Y()});
        }
        emitStrip(low, railIds(natLeft),
                  stubHere ? &stubVerts : nullptr, true, corner00);
    }
    if (!natRight.empty()) {
        std::vector<uint32_t> high;
        for (int j = j0; j <= j1; ++j) high.push_back(grid[j * (nu + 1) + i1]);
        emitStrip(railIds(natRight), high, nullptr, false, UINT32_MAX);
    }
    return true;
}


// Interior trim wires on a coons face (an angled hole through a curved
// top): mesh the grid whole, DELETE the covered cells, and web the
// staircase to the wires' exact border sampling — the same two-step
// the revolution bands use, so the hole welds watertight to the bore.
// The interior refines (borders stay at solved counts) until every
// hole is strictly interior to the lattice.
bool meshCoonsGrid(const TopoDS_Face& face, const Model& model, int faceId,
                   const std::vector<double>& uParams,
                   const std::vector<double>& vParams, int rotate,
                   const std::vector<int>& solvedEdge, MeshBuilder& out,
                   const std::vector<std::vector<int>>* inserts,
                   int collarRings,
                   const PinnedEdges* pins, int cellCap,
                   bool decoupleSeams) {
    if (!inserts || inserts->empty()) {
        return meshCoonsGridBody(face, model, faceId, uParams, vParams,
                                 rotate, solvedEdge, out, nullptr, nullptr,
                                 pins, decoupleSeams);
    }
    dbg("coons cutout %d: %zu insert wire(s)", faceId, inserts->size());
    // Hole rings: 3D edge curves at solved counts (the bore wall's own
    // contract), uv through the pcurves, pieces chained by endpoints.
    struct HPt {
        gp_Pnt p;
        double u, v;
    };
    std::vector<std::vector<HPt>> rings;
    std::vector<std::array<double, 4>> boxes;
    for (const auto& wire : *inserts) {
        std::vector<std::vector<HPt>> pieces;
        for (int eid : wire) {
            if (eid < 1 || eid > model.edgeCount()) continue;
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(edge)) continue;
            int n = eid < int(solvedEdge.size()) && solvedEdge[eid] > 0
                        ? solvedEdge[eid]
                        : 8;
            double f2, l2, f3, l3;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f2, l2);
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
            if (pc.IsNull() || c3.IsNull()) return false;
            const double ph = closedEdgePhase(edge, model);
            std::vector<HPt> piece;
            for (double t : edgeSampleFractions(
                     eid, n, ph, false, /*includeLast=*/true, nullptr,
                     &model)) {
                gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
                piece.push_back(
                    {c3->Value(f3 + (l3 - f3) * t), uv.X(), uv.Y()});
            }
            pieces.push_back(std::move(piece));
        }
        if (pieces.empty()) return false;
        std::vector<HPt> ring = std::move(pieces[0]);
        std::vector<char> used(pieces.size(), 0);
        used[0] = 1;
        for (size_t step = 1; step < pieces.size(); ++step) {
            double bd = 1e300;
            size_t bi = 0;
            bool rev = false;
            for (size_t k = 0; k < pieces.size(); ++k) {
                if (used[k]) continue;
                double dF = ring.back().p.Distance(pieces[k].front().p);
                double dB = ring.back().p.Distance(pieces[k].back().p);
                if (dF < bd) { bd = dF; bi = k; rev = false; }
                if (dB < bd) { bd = dB; bi = k; rev = true; }
            }
            used[bi] = 1;
            std::vector<HPt> pc2 = std::move(pieces[bi]);
            if (rev) std::reverse(pc2.begin(), pc2.end());
            ring.insert(ring.end(), pc2.begin() + 1, pc2.end());
        }
        if (ring.size() > 1 &&
            ring.front().p.Distance(ring.back().p) < 1e-9) {
            ring.pop_back();
        }
        if (ring.size() < 3) return false;
        std::array<double, 4> b{1e300, -1e300, 1e300, -1e300};
        for (const HPt& q : ring) {
            b[0] = std::min(b[0], q.u);
            b[1] = std::max(b[1], q.u);
            b[2] = std::min(b[2], q.v);
            b[3] = std::max(b[3], q.v);
        }
        rings.push_back(std::move(ring));
        boxes.push_back(b);
    }
    double uScale = 1.0;
    try {
        BRepAdaptor_Surface sa(face);
        const double um = (sa.FirstUParameter() + sa.LastUParameter()) / 2;
        const double vm = (sa.FirstVParameter() + sa.LastVParameter()) / 2;
        const double su = std::max(
            1e-9, sa.Value(um, vm).Distance(sa.Value(um + 1e-3, vm)) / 1e-3);
        const double sv = std::max(
            1e-9, sa.Value(um, vm).Distance(sa.Value(um, vm + 1e-3)) / 1e-3);
        uScale = su / sv;
    } catch (const Standard_Failure&) {
    }
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    // The border params are a fixed contract (paramsFor pins them to the
    // solved counts), so extra resolution comes from scaffold lines — and
    // ONLY where geometry demands them. Curvature already lives in the
    // solved border counts (adaptive), so each direction keeps its
    // natural lines (those columns run border to border) and gains just
    // a snug BRACKET pair around every hole. A flat direction never
    // sprouts rows with nothing to follow; escalation midpoints the
    // hole band first and only then doubles everything.
    CoonsPatch cpatch;
    if (!makeCoonsPatch(face, model, cpatch, rotate)) return false;
    auto sideCount = [&](int i) {
        const auto& ch = cpatch.chain[i];
        if (ch.size() > 1) {
            int total = 0;
            for (const auto& pce : ch) {
                int n = 1;
                if (pce.edgeId > 0 && pce.edgeId < int(solvedEdge.size()) &&
                    solvedEdge[pce.edgeId] > 0) {
                    n = solvedEdge[pce.edgeId];
                }
                total += n;
            }
            return std::max(1, total);
        }
        const int eid = cpatch.edgeIds[i];
        return eid > 0 && eid < int(solvedEdge.size()) && solvedEdge[eid] > 0
                   ? solvedEdge[eid]
                   : 1;
    };
    const int nU = std::max(sideCount(0), sideCount(2));
    const int nV = std::max(sideCount(1), sideCount(3));
    // Lattice-fraction -> surface-uv tables along the two mid-isolines,
    // for placing hole brackets in fraction space. Each direction sweeps
    // ONE uv axis dominantly; measure against that one.
    const int kTab = 128;
    std::vector<gp_Pnt2d> uTab(kTab + 1), vTab(kTab + 1);
    for (int k = 0; k <= kTab; ++k) {
        const double fr = double(k) / kTab;
        uTab[k] = cpatch.uv(fr, 0.5);
        vTab[k] = cpatch.uv(0.5, fr);
    }
    auto bracket = [&](const std::vector<gp_Pnt2d>& tab,
                       const std::array<double, 4>& b,
                       double& fa, double& fb) {
        double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
        for (const auto& q : tab) {
            x0 = std::min(x0, q.X());
            x1 = std::max(x1, q.X());
            y0 = std::min(y0, q.Y());
            y1 = std::max(y1, q.Y());
        }
        const bool useX = (x1 - x0) >= (y1 - y0);
        const double lo = useX ? b[0] : b[2];
        const double hi = useX ? b[1] : b[3];
        fa = 2.0;
        fb = -1.0;
        for (size_t k = 0; k < tab.size(); ++k) {
            const double val = useX ? tab[k].X() : tab[k].Y();
            if (val >= lo && val <= hi) {
                const double fr = double(k) / double(tab.size() - 1);
                fa = std::min(fa, fr);
                fb = std::max(fb, fr);
            }
        }
        return fb >= fa;
    };
    auto mergeIn = [](std::vector<double>& fr, double x) {
        x = std::clamp(x, 0.01, 0.99);
        for (double e : fr) {
            if (std::abs(e - x) < 5e-3) return;
        }
        fr.push_back(x);
    };
    auto scaffoldFor = [&](int attempt, bool uDir) {
        const int nat = uDir ? nU : nV;
        std::vector<double> fr(nat + 1);
        for (int k = 0; k <= nat; ++k) fr[k] = double(k) / nat;
        for (const auto& b : boxes) {
            double fa, fb;
            if (!bracket(uDir ? uTab : vTab, b, fa, fb)) continue;
            // The plan's insert floors normally make the natural lattice
            // fine enough (hole spans about half a cell) — then NO
            // brackets, so every line runs border to border and the
            // borders carry the counts. Brackets return only when a pin
            // or override starved the direction below the hole's need.
            const int need =
                int(std::ceil(0.5 / std::max(0.01, fb - fa)));
            if (nat >= need && attempt == 0) continue;
            const double m = std::max(0.25 * (fb - fa), 0.02);
            mergeIn(fr, fa - m);
            mergeIn(fr, fb + m);
            if (attempt >= 1) mergeIn(fr, 0.5 * (fa + fb));
        }
        std::sort(fr.begin(), fr.end());
        for (int d = 2; d <= attempt; ++d) {  // last-resort doubling
            std::vector<double> dense;
            dense.reserve(fr.size() * 2);
            for (size_t k = 0; k + 1 < fr.size(); ++k) {
                dense.push_back(fr[k]);
                dense.push_back(0.5 * (fr[k] + fr[k + 1]));
            }
            dense.push_back(fr.back());
            fr = std::move(dense);
        }
        return fr;
    };
    for (int attempt = 0; attempt < 5; ++attempt) {
        std::vector<double> uFr = scaffoldFor(attempt, true);
        std::vector<double> vFr = scaffoldFor(attempt, false);
        // Pathology guard: the scaffold doubling that separates close bores
        // grows geometrically. Once it would blow the face's area-share
        // cell budget it has stopped being real detail — stop densifying
        // and let the face demote to the border-exact contract floor
        // rather than ship tens of thousands of cells one face wide.
        if (cellCap > 0 &&
            double(uFr.size() - 1) * double(vFr.size() - 1) >
                double(cellCap)) {
            dbg("coons cutout %d: attempt %d over budget (%zux%zu > %d)",
                faceId, attempt, uFr.size() - 1, vFr.size() - 1, cellCap);
            break;
        }
        const std::vector<double>* uSc =
            int(uFr.size()) > nU + 1 ? &uFr : nullptr;
        const std::vector<double>* vSc =
            int(vFr.size()) > nV + 1 ? &vFr : nullptr;
        PolyMesh grid;
        {
            MeshBuilder tmp(grid);
            if (!meshCoonsGridBody(face, model, faceId, uParams, vParams,
                                   rotate, solvedEdge, tmp, uSc, vSc, pins,
                                   decoupleSeams)) {
                dbg("coons cutout %d: body failed (attempt %d)", faceId,
                    attempt);
                return false;
            }
        }
        if (grid.polygons.empty()) continue;  // degenerate lattice level
        auto ekey = [](uint32_t a, uint32_t b) {
            return (uint64_t(std::min(a, b)) << 32) | std::max(a, b);
        };
        std::map<uint64_t, int> use;
        for (const auto& poly : grid.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                ++use[ekey(poly[i], poly[(i + 1) % poly.size()])];
            }
        }
        std::set<uint32_t> borderVert;
        for (const auto& poly : grid.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                if (use[ekey(a, b)] == 1) {
                    borderVert.insert(a);
                    borderVert.insert(b);
                }
            }
        }
        std::vector<char> keep(grid.polygons.size(), 1);
        std::vector<int> deletedPer(rings.size(), 0);
        bool coarse = false;
        bool touchedBorder = false;
        for (size_t pi = 0; pi < grid.polygons.size() && !coarse; ++pi) {
            double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
            int cn = 0;
            for (uint32_t vi : grid.polygons[pi]) {
                const Anchor& an = grid.anchors[vi];
                if (an.faceId != faceId) { cn = 0; break; }
                u0 = std::min(u0, an.u);
                u1 = std::max(u1, an.u);
                v0 = std::min(v0, an.v);
                v1 = std::max(v1, an.v);
                ++cn;
            }
            if (!cn) continue;
            // Delete on uv-RECT overlap, not centroid membership: a kept
            // cell that the ring still crosses self-intersects the web.
            // Over-deletion only widens the web; a deletion reaching the
            // border rails refines instead.
            for (size_t w = 0; w < boxes.size(); ++w) {
                const auto& b = boxes[w];
                if (u1 >= b[0] && u0 <= b[1] && v1 >= b[2] && v0 <= b[3]) {
                    for (uint32_t vi : grid.polygons[pi]) {
                        if (borderVert.count(vi)) {
                            if (!touchedBorder) {
                                dbg("coons cutout %d: %zu-gon rect "
                                    "[%.3f..%.3f]x[%.3f..%.3f] hits box %zu",
                                    faceId, grid.polygons[pi].size(), u0,
                                    u1, v0, v1, w);
                            }
                            coarse = true;
                            touchedBorder = true;
                        }
                    }
                    keep[pi] = 0;
                    ++deletedPer[w];
                    break;
                }
            }
        }
        for (int d : deletedPer) {
            if (d < 1) coarse = true;
        }
        if (coarse) {
            dbg("coons cutout %d: attempt %d too coarse (polys=%zu, "
                "deleted[0]=%d, borderTouch=%d, box=[%.3f..%.3f]x"
                "[%.3f..%.3f])",
                faceId, attempt, grid.polygons.size(), deletedPer[0],
                touchedBorder ? 1 : 0, boxes[0][0], boxes[0][1],
                boxes[0][2], boxes[0][3]);
            continue;
        }
        std::map<uint64_t, std::array<int, 2>> sideUse;
        for (size_t pi = 0; pi < grid.polygons.size(); ++pi) {
            const auto& poly = grid.polygons[pi];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint64_t k = ekey(poly[i], poly[(i + 1) % poly.size()]);
                auto& s2 =
                    sideUse.emplace(k, std::array<int, 2>{0, 0})
                        .first->second;
                ++s2[keep[pi] ? 0 : 1];
            }
        }
        std::map<uint32_t, std::vector<uint32_t>> adj;
        for (const auto& [k, s2] : sideUse) {
            if (s2[0] == 1 && s2[1] == 1) {
                uint32_t a = uint32_t(k >> 32), b = uint32_t(k);
                adj[a].push_back(b);
                adj[b].push_back(a);
            }
        }
        bool bad = false;
        for (const auto& [v, ns] : adj) {
            if (ns.size() != 2) bad = true;
        }
        if (bad) {
            dbg("coons cutout %d: attempt %d staircase not a loop", faceId,
                attempt);
            continue;
        }
        std::vector<std::vector<uint32_t>> loops;
        {
            std::set<uint32_t> seen;
            for (const auto& [v0, ns] : adj) {
                if (seen.count(v0)) continue;
                std::vector<uint32_t> loop{v0};
                seen.insert(v0);
                uint32_t prev = v0, cur = ns[0];
                while (cur != v0) {
                    loop.push_back(cur);
                    seen.insert(cur);
                    const auto& nn = adj[cur];
                    uint32_t nxt = nn[0] == prev ? nn[1] : nn[0];
                    prev = cur;
                    cur = nxt;
                    if (loop.size() > adj.size()) { bad = true; break; }
                }
                if (bad) break;
                loops.push_back(std::move(loop));
            }
        }
        if (bad || loops.size() != rings.size()) {
            dbg("coons cutout %d: attempt %d loops %zu != holes %zu", faceId,
                attempt, loops.size(), rings.size());
            continue;
        }
        // Assemble locally; `out` receives the part only when every web
        // proves complete (a partial face is a guaranteed leak).
        PolyMesh assembled;
        MeshBuilder ab(assembled);
        std::vector<uint32_t> remap(grid.vertices.size(), UINT32_MAX);
        auto emitVert = [&](uint32_t i) {
            if (remap[i] == UINT32_MAX) {
                remap[i] = ab.addVertex(
                    gp_Pnt(grid.vertices[i][0], grid.vertices[i][1],
                           grid.vertices[i][2]),
                    grid.anchors[i]);
            }
            return remap[i];
        };
        for (size_t pi = 0; pi < grid.polygons.size(); ++pi) {
            if (!keep[pi]) continue;
            std::vector<uint32_t> poly;
            poly.reserve(grid.polygons[pi].size());
            for (uint32_t vi : grid.polygons[pi]) {
                poly.push_back(emitVert(vi));
            }
            ab.addPolygon(std::move(poly), faceId, false);
        }
        std::vector<char> ringUsed(rings.size(), 0);
        bool webbed = true;
        for (const auto& loop : loops) {
            double cu = 0, cv = 0;
            for (uint32_t vi : loop) {
                cu += grid.anchors[vi].u;
                cv += grid.anchors[vi].v;
            }
            cu /= double(loop.size());
            cv /= double(loop.size());
            int w = -1;
            for (size_t k = 0; k < rings.size(); ++k) {
                const auto& b = boxes[k];
                if (!ringUsed[k] && cu >= b[0] && cu <= b[1] &&
                    cv >= b[2] && cv <= b[3]) {
                    w = int(k);
                    break;
                }
            }
            if (w < 0) { webbed = false; break; }
            ringUsed[w] = 1;
            std::vector<WebPoint> outerRing;
            for (uint32_t vi : loop) {
                outerRing.push_back(
                    {gp_Pnt2d(grid.anchors[vi].u * uScale,
                              grid.anchors[vi].v),
                     emitVert(vi)});
            }
            auto area = [](const std::vector<WebPoint>& r) {
                double a = 0;
                for (size_t i = 0; i < r.size(); ++i) {
                    const auto& p1 = r[i].uv;
                    const auto& p2 = r[(i + 1) % r.size()].uv;
                    a += p1.X() * p2.Y() - p2.X() * p1.Y();
                }
                return a / 2;
            };
            if (area(outerRing) < 0) {
                std::reverse(outerRing.begin(), outerRing.end());
            }
            std::vector<WebPoint> holeRing;
            for (const HPt& q : rings[w]) {
                holeRing.push_back(
                    {gp_Pnt2d(q.u * uScale, q.v),
                     ab.addVertex(q.p, {faceId, q.u, q.v})});
            }
            // Concentric quad collar between the bore ring and the
            // staircase (junctionRings, same knob as the planar ring
            // junction): the ring's density gets absorbed by clean
            // loops instead of one wide fan web. Scaled about the
            // ring's own centroid, capped safely inside the staircase.
            std::vector<WebPoint> webInner = holeRing;
            if (collarRings > 0 && rings[w].size() >= 3) {
                double rcU = 0, rcV = 0;
                for (const HPt& q : rings[w]) {
                    rcU += q.u;
                    rcV += q.v;
                }
                rcU /= double(rings[w].size());
                rcV /= double(rings[w].size());
                // Max uniform scale before any ring vertex crosses the
                // staircase: cast centroid->vertex rays against every
                // staircase segment (scaled uv keeps the metric honest).
                const gp_Pnt2d C(rcU * uScale, rcV);
                double smax = 1e300;
                for (const HPt& q : rings[w]) {
                    const double dx = q.u * uScale - C.X();
                    const double dy = q.v - C.Y();
                    const double dlen = std::hypot(dx, dy);
                    if (dlen < 1e-12) { smax = 0; break; }
                    for (size_t si = 0; si < outerRing.size(); ++si) {
                        const gp_Pnt2d& A = outerRing[si].uv;
                        const gp_Pnt2d& B =
                            outerRing[(si + 1) % outerRing.size()].uv;
                        const double ex = B.X() - A.X();
                        const double ey = B.Y() - A.Y();
                        const double den = dx * ey - dy * ex;
                        if (std::abs(den) < 1e-18) continue;
                        const double t =
                            ((A.X() - C.X()) * ey - (A.Y() - C.Y()) * ex) /
                            den;
                        const double uu =
                            ((A.X() - C.X()) * dy - (A.Y() - C.Y()) * dx) /
                            den;
                        if (t > 0 && uu >= 0 && uu <= 1) {
                            smax = std::min(smax, t);
                        }
                    }
                }
                if (smax > 1.25 && smax < 1e300) {
                    Handle(Geom_Surface) cs = BRep_Tool::Surface(face);
                    const double sTop = 1.0 + (smax * 0.85 - 1.0);
                    std::vector<WebPoint> prev = holeRing;
                    for (int k = 1; k <= collarRings && !cs.IsNull();
                         ++k) {
                        const double sk =
                            1.0 + (sTop - 1.0) * double(k) /
                                      double(collarRings);
                        std::vector<WebPoint> loopK;
                        for (const HPt& q : rings[w]) {
                            const double lu = rcU + (q.u - rcU) * sk;
                            const double lv = rcV + (q.v - rcV) * sk;
                            gp_Pnt lp = cs->Value(lu, lv);
                            loopK.push_back(
                                {gp_Pnt2d(lu * uScale, lv),
                                 ab.addVertex(lp, {faceId, lu, lv})});
                        }
                        const size_t n = prev.size();
                        for (size_t i = 0; i < n; ++i) {
                            std::vector<WebPoint> quad = {
                                prev[i], prev[(i + 1) % n],
                                loopK[(i + 1) % n], loopK[i]};
                            std::vector<uint32_t> ids;
                            double a2 = 0;
                            for (size_t j = 0; j < 4; ++j) {
                                const auto& p1 = quad[j].uv;
                                const auto& p2 = quad[(j + 1) % 4].uv;
                                a2 += p1.X() * p2.Y() - p2.X() * p1.Y();
                                ids.push_back(quad[j].vert);
                            }
                            if (a2 < 0) {
                                std::reverse(ids.begin(), ids.end());
                            }
                            ab.addPolygon(std::move(ids), faceId, flip);
                        }
                        prev = std::move(loopK);
                    }
                    webInner = std::move(prev);
                }
            }
            if (area(webInner) > 0) {
                std::reverse(webInner.begin(), webInner.end());
            }
            if (!triangulateWeb(std::move(outerRing),
                                {std::move(webInner)}, faceId, flip, ab)) {
                webbed = false;
                break;
            }
        }
        if (!webbed) {
            dbg("coons cutout %d: attempt %d web failed", faceId, attempt);
            continue;
        }
        for (size_t vi = 0; vi < assembled.vertices.size(); ++vi) {
            out.addVertex(gp_Pnt(assembled.vertices[vi][0],
                                 assembled.vertices[vi][1],
                                 assembled.vertices[vi][2]),
                          assembled.anchors[vi]);
        }
        // addVertex ids are sequential from the part's current size —
        // the part is always empty here (planned meshers own it), so
        // polygon indices carry over unchanged.
        for (size_t pi = 0; pi < assembled.polygons.size(); ++pi) {
            out.addPolygon(assembled.polygons[pi], faceId, false);
        }
        dbg("coons cutout %d: %zu hole(s), %zu polys after %d refine(s)",
            faceId, rings.size(), assembled.polygons.size(), attempt);
        return true;
    }
    return false;
}


}  // namespace weft::mesher_impl
