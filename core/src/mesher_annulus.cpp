#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// A face bounded by exactly two closed loops — the flat ring between two
// revolution rims. Meshes as one zippered band: equal loop counts give
// pure quads, unequal a clean taper. Its borders sample the 3D edge
// curves, and the conformity pass then snaps them onto whatever the
// neighbours generated (phase-exact welds).
//
// `requireRing` (the automatic path) gates on actual ring geometry: the
// two loops must be roughly concentric and of comparable size. "Two wires"
// alone also matches a big plate with one small slot, and zippering a tiny
// loop against a huge boundary makes a fan mess — those faces belong to
// the fallback/plate meshers unless the user forces the band.

bool planAnnulus(const TopoDS_Face& face, const Model& model, FacePlan& plan,
                 bool requireRing) {
    int wires = 0;
    std::array<std::vector<int>, 2> loop;
    std::array<TopoDS_Wire, 2> wire2;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    int outerIdx = -1;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (wires >= 2) return false;
        const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
        // Wire order + orientation matter for chain sampling.
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            if (BRep_Tool::Degenerated(edge)) return false;
            double f, l;
            if (BRep_Tool::Curve(edge, f, l).IsNull()) return false;
            int eid = model.edges.FindIndex(edge);
            if (eid < 1) return false;
            // A repeated edge (a closed-surface seam walked twice)
            // would sample its border twice — non-manifold after weld.
            for (int prev : loop[wires]) {
                if (prev == eid) return false;
            }
            loop[wires].push_back(eid);
        }
        if (loop[wires].empty() || loop[wires].size() > 24) return false;
        if (!outer.IsNull() && wire.IsSame(outer)) outerIdx = wires;
        wire2[wires] = wire;
        ++wires;
    }
    if (wires != 2) return false;
    if (requireRing) {
        GProp_GProps a, b;
        BRepGProp::LinearProperties(wire2[0], a);
        BRepGProp::LinearProperties(wire2[1], b);
        double pa = a.Mass(), pb = b.Mass();
        if (pa < 1e-12 || pb < 1e-12) return false;
        double ratio = std::min(pa, pb) / std::max(pa, pb);
        // Concentric within a third of the bigger loop's equivalent
        // radius, neither loop dwarfing the other, and both loops round
        // (a centered slot in a long plate is concentric — not a ring).
        double eqRadius = std::max(pa, pb) / (2.0 * M_PI);
        double apart = a.CentreOfMass().Distance(b.CentreOfMass());
        if (ratio < 0.25 || apart > 0.35 * eqRadius) return false;
        if (wireElongation(wire2[0]) > 2.2 ||
            wireElongation(wire2[1]) > 2.2) {
            return false;
        }
        // Only actually-flat faces: a domed two-wire panel meshed as a
        // straight-railed ring ignores the surface between its loops.
        BRepAdaptor_Surface flatProbe(face);
        if (!isGeometricallyFlat(face, flatProbe)) return false;
    }
    if (outerIdx == 1) std::swap(loop[0], loop[1]);
    plan.kind = MesherKind::AnnulusRing;
    plan.uEdges = loop[0];
    plan.vEdges = loop[1];
    plan.constrains = true;
    return true;
}

bool meshAnnulusRing(const TopoDS_Face& face, const Model& model, int faceId,
                     const std::vector<int>& outerLoop,
                     const std::vector<int>& innerLoop,
                     const std::vector<int>& solvedEdge, int radialDefault,
                     MeshBuilder& out) {
    // A ring = the wire's edges chained in order, each sampled at its own
    // solved count (endpoints shared with the next edge, so a loop of K
    // edges at counts c_k has sum(c_k) vertices). The wire is walked ON
    // THE FACE: the model's stored edge orientation can differ per edge,
    // and sampling with it zigzags multi-edge loops into folded rings.
    auto sampleRing = [&](const std::vector<int>& loop) {
        std::vector<gp_Pnt> pts;
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
            bool mine = false;
            for (BRepTools_WireExplorer we(wire, face); we.More();
                 we.Next()) {
                if (model.edges.FindIndex(we.Current()) == loop[0]) {
                    mine = true;
                    break;
                }
            }
            if (!mine) continue;
            for (BRepTools_WireExplorer we(wire, face); we.More();
                 we.Next()) {
                const TopoDS_Edge edge = we.Current();
                int eid = model.edges.FindIndex(edge);
                int n = eid >= 1 && eid < int(solvedEdge.size())
                            ? solvedEdge[eid]
                            : 0;
                if (n < 1) n = std::max(3, radialDefault) / int(loop.size());
                n = std::max(1, n);
                BRepAdaptor_Curve c(edge);
                double f = c.FirstParameter(), l = c.LastParameter();
                const bool rev = edge.Orientation() == TopAbs_REVERSED;
                const double ph = closedEdgePhase(edge, model);
                // endpoint owned by next edge; even 3D arc for freeform
                for (double t : edgeSampleFractions(
                         eid, n, ph, rev, /*includeLast=*/false, nullptr,
                         &model)) {
                    pts.push_back(c.Value(f + (l - f) * t));
                }
            }
            break;
        }
        return pts;
    };
    std::vector<gp_Pnt> A = sampleRing(outerLoop);
    std::vector<gp_Pnt> B = sampleRing(innerLoop);
    if (A.size() < 3 || B.size() < 3) return false;
    const int nOut = int(A.size());
    const int nIn = int(B.size());

    // Direction + start alignment: try B forward and reversed at every
    // offset, keep the pairing with the shortest total rails.
    auto pairingCost = [&](const std::vector<gp_Pnt>& b, int off) {
        double sum = 0;
        for (int i = 0; i < nOut; ++i) {
            int j = (off + i * nIn / nOut) % nIn;
            sum += A[i].Distance(b[j]);
        }
        return sum;
    };
    std::vector<gp_Pnt> Brev(B.rbegin(), B.rend());
    double best = 1e300;
    int bestOff = 0;
    bool rev = false;
    for (int off = 0; off < nIn; ++off) {
        double c1 = pairingCost(B, off);
        if (c1 < best) { best = c1; bestOff = off; rev = false; }
        double c2 = pairingCost(Brev, off);
        if (c2 < best) { best = c2; bestOff = off; rev = true; }
    }
    if (rev) B = Brev;

    std::vector<uint32_t> av(nOut), bv(nIn);
    for (int i = 0; i < nOut; ++i) av[i] = out.addVertex(A[i], {});
    for (int j = 0; j < nIn; ++j) {
        bv[j] = out.addVertex(B[(bestOff + j) % nIn], {});
    }

    // Equal counts zip to pure quads; mismatched counts bridge by ARC
    // fraction with the extra dense points grouped into 5-gons (the
    // alternating-triangle zipper drew a WWWW sliver band around every
    // count-mismatched disc rim). Winding is fixed afterwards against
    // the surface normal at the first polygon.
    std::vector<std::vector<uint32_t>> polys;
    if (nOut == nIn) {
        for (int i = 0; i < nOut; ++i) {
            polys.push_back({av[i], av[(i + 1) % nOut],
                             bv[(i + 1) % nIn], bv[i]});
        }
    } else {
        const bool aSparse = nOut <= nIn;
        const std::vector<uint32_t>& S = aSparse ? av : bv;
        const std::vector<uint32_t>& D = aSparse ? bv : av;
        const std::vector<gp_Pnt>* Sp = aSparse ? &A : &B;
        const std::vector<gp_Pnt>* Dp = aSparse ? &B : &A;
        const int ns = int(S.size()), nd = int(D.size());
        // Normalized cumulative arcs. av pairs with A directly; bv was
        // built offset-aligned, so its geometric order is
        // B[(bestOff + j) % nIn].
        auto fractionsOf = [&](const std::vector<gp_Pnt>& pts, int n,
                               bool useOff) {
            std::vector<double> f(n + 1, 0.0);
            for (int i = 1; i <= n; ++i) {
                const gp_Pnt& p0 =
                    pts[useOff ? (bestOff + i - 1) % n : (i - 1)];
                const gp_Pnt& p1 = pts[useOff ? (bestOff + i) % n : i % n];
                f[i] = f[i - 1] + p0.Distance(p1);
            }
            const double t = f[n] > 1e-12 ? f[n] : 1.0;
            for (double& x : f) x /= t;
            return f;
        };
        const std::vector<double> sf = fractionsOf(*Sp, ns, !aSparse);
        const std::vector<double> df = fractionsOf(*Dp, nd, aSparse);
        std::vector<int> mp(ns + 1);
        mp[0] = 0;
        mp[ns] = nd;
        for (int k = 1; k < ns; ++k) {
            int j = mp[k - 1];
            while (j + 1 < nd && std::abs(df[j + 1] - sf[k]) <=
                                     std::abs(df[j] - sf[k])) {
                ++j;
            }
            mp[k] = j;
        }
        for (int k = 0; k < ns; ++k) {
            std::vector<uint32_t> ring2;
            if (aSparse) {
                ring2 = {S[k], S[(k + 1) % ns]};
                for (int t = mp[k + 1]; t >= mp[k]; --t) {
                    ring2.push_back(D[t % nd]);
                }
            } else {
                for (int t = mp[k]; t <= mp[k + 1]; ++t) {
                    ring2.push_back(D[t % nd]);
                }
                ring2.push_back(S[(k + 1) % ns]);
                ring2.push_back(S[k]);
            }
            ring2.erase(std::unique(ring2.begin(), ring2.end()),
                        ring2.end());
            if (ring2.size() > 1 && ring2.front() == ring2.back()) {
                ring2.pop_back();
            }
            if (ring2.size() < 3) continue;
            polys.push_back(std::move(ring2));
        }
    }

    // Face normal at the ring midpoint decides the winding.
    BRepAdaptor_Surface surf(face);
    double um = (surf.FirstUParameter() + surf.LastUParameter()) / 2;
    double vm = (surf.FirstVParameter() + surf.LastVParameter()) / 2;
    gp_Pnt sp;
    gp_Vec du, dv;
    surf.D1(um, vm, sp, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    bool flip = false;
    if (!polys.empty() && n.Magnitude() > 1e-12) {
        // Newell normal of the first polygon via the builder's positions
        // is awkward pre-build; use the sampled points directly.
        gp_Pnt p0 = A[0], p1 = A[1 % nOut], p2 = B[bestOff % nIn];
        gp_Vec pn = gp_Vec(p0, p1).Crossed(gp_Vec(p0, p2));
        flip = pn.Dot(n) < 0;
    }
    for (auto& poly : polys) out.addPolygon(std::move(poly), faceId, flip);
    return true;
}

// OPEN (C-shaped) annulus band. A notch cut clean through a flat ring leaves
// a SINGLE wire — two concentric arc rails (an inner circle and an outer
// circle, same centre/axis, each interrupted by the notch) joined by two
// radial walls — instead of the two closed loops planAnnulus wants. A full
// washer keeps two wires and never reaches here. On success: uEdges = the
// outer arc(s), vEdges = the inner arc(s), cWalls = the two joining walls.
bool planAnnulusCRing(const TopoDS_Face& face, const Model& model,
                      FacePlan& plan) {
    BRepAdaptor_Surface surf(face);
    if (!isGeometricallyFlat(face, surf)) return false;
    int wires = 0;
    TopoDS_Wire theWire;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
        theWire = TopoDS::Wire(wx.Current());
    }
    if (wires != 1) return false;

    struct ArcE {
        int eid;
        double rad;
        double span;
        gp_Pnt ctr;
        gp_Dir axis;
    };
    std::vector<ArcE> arcs;
    std::vector<int> walls;
    for (BRepTools_WireExplorer we(theWire, face); we.More(); we.Next()) {
        const TopoDS_Edge e = we.Current();
        if (BRep_Tool::Degenerated(e)) return false;
        double f, l;
        if (BRep_Tool::Curve(e, f, l).IsNull()) return false;
        const int eid = model.edges.FindIndex(e);
        if (eid < 1) return false;
        BRepAdaptor_Curve c(e);
        if (c.GetType() == GeomAbs_Circle) {
            const gp_Circ ci = c.Circle();
            arcs.push_back({eid, ci.Radius(),
                            std::abs(c.LastParameter() - c.FirstParameter()),
                            ci.Location(), ci.Axis().Direction()});
        } else {
            walls.push_back(eid);
        }
    }
    // Two arc rails + exactly two connecting walls.
    if (arcs.size() < 2 || walls.size() != 2) return false;
    double rMin = 1e300, rMax = 0;
    for (const ArcE& a : arcs) {
        rMin = std::min(rMin, a.rad);
        rMax = std::max(rMax, a.rad);
    }
    if (rMax - rMin < 1e-4) return false;  // one radius: not a ring
    const gp_Pnt ctr = arcs[0].ctr;
    const gp_Dir axis = arcs[0].axis;
    std::vector<int> outer, inner;
    double outerSpan = 0, innerSpan = 0;
    for (const ArcE& a : arcs) {
        if (a.ctr.Distance(ctr) > 1e-4) return false;      // one centre
        if (std::abs(a.axis.Dot(axis)) < 0.999) return false;  // one axis
        const double dOut = std::abs(a.rad - rMax);
        const double dIn = std::abs(a.rad - rMin);
        if (std::min(dOut, dIn) > 1e-4) return false;  // exactly two radii
        if (dOut < dIn) { outer.push_back(a.eid); outerSpan += a.span; }
        else { inner.push_back(a.eid); innerSpan += a.span; }
    }
    if (outer.empty() || inner.empty()) return false;
    // The rails must be the RING (most of the circle present, cut open by
    // a small notch) — not a thin annular sector whose short arcs are ends
    // and long straight sides are the "walls". A ring-with-a-notch keeps
    // well over half the circle on each rail; a sector spans a sliver.
    if (outerSpan < M_PI || innerSpan < M_PI) return false;
    // Each wall spans from the inner radius to the outer radius.
    auto radiusOf = [&](const gp_Pnt& p) {
        gp_Vec v(ctr, p);
        const gp_Vec ax(axis);
        return (v - ax.Multiplied(v.Dot(ax))).Magnitude();
    };
    for (int weid : walls) {
        BRepAdaptor_Curve c(TopoDS::Edge(model.edges(weid)));
        const double ra = radiusOf(c.Value(c.FirstParameter()));
        const double rb = radiusOf(c.Value(c.LastParameter()));
        const double lo = std::min(ra, rb), hi = std::max(ra, rb);
        if (std::abs(lo - rMin) > 0.05 * rMax ||
            std::abs(hi - rMax) > 0.05 * rMax) {
            return false;
        }
    }
    plan.kind = MesherKind::AnnulusRing;
    plan.uEdges = outer;
    plan.vEdges = inner;
    plan.cWalls = walls;
    plan.cRing = true;
    plan.constrains = true;
    return true;
}

// Mesh the C-ring as radial quad spokes. Both rails are sampled at their
// pinned column azimuths (shared with the cylinder wall / bore rims, so
// they weld bit-identically and every column meets a spoke). The two rails
// pair by arc fraction into radial quads; a count difference near the notch
// (a column present on the outer rail but inside the inner rail's wider
// notch gap) is absorbed by one grouped n-gon at that wall end. The walls
// carry the across-ring row count: >1 gives interior rows (uniform radial
// spokes), 1 gives a single quad band.
bool meshAnnulusCRing(const TopoDS_Face& face, const Model& model, int faceId,
                      const std::vector<int>& outerEdges,
                      const std::vector<int>& innerEdges,
                      const std::vector<int>& wallEdges,
                      const std::vector<int>& solvedEdge, int radialDefault,
                      MeshBuilder& out, const PinnedEdges* pins) {
    const double weld = 1e-4 + BRep_Tool::Tolerance(face);
    // Sample an open edge chain end-to-end, honouring per-edge pins.
    auto sampleChain = [&](const std::vector<int>& edges,
                           int fallbackN) -> std::vector<gp_Pnt> {
        std::vector<std::vector<gp_Pnt>> pieces;
        for (int eid : edges) {
            const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
            double f, l;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
            if (c3.IsNull()) return {};
            int n = (eid >= 1 && eid < int(solvedEdge.size())) ? solvedEdge[eid]
                                                               : 0;
            if (n < 1) n = std::max(1, fallbackN);
            const bool rev = e.Orientation() == TopAbs_REVERSED;
            const double ph = closedEdgePhase(e, model);
            std::vector<gp_Pnt> pc;
            for (double t : edgeSampleFractions(eid, n, ph, rev,
                                                /*includeLast=*/true, pins,
                                                &model)) {
                pc.push_back(c3->Value(f + (l - f) * t));
            }
            if (pc.size() < 2) return {};
            pieces.push_back(std::move(pc));
        }
        if (pieces.empty()) return {};
        std::vector<gp_Pnt> chain = pieces[0];
        std::vector<char> used(pieces.size(), 0);
        used[0] = 1;
        bool progress = true;
        while (progress) {
            progress = false;
            for (size_t k = 0; k < pieces.size(); ++k) {
                if (used[k]) continue;
                std::vector<gp_Pnt> pv = pieces[k];
                if (chain.back().Distance(pv.front()) < weld) {
                    chain.insert(chain.end(), pv.begin() + 1, pv.end());
                } else if (chain.back().Distance(pv.back()) < weld) {
                    std::reverse(pv.begin(), pv.end());
                    chain.insert(chain.end(), pv.begin() + 1, pv.end());
                } else if (chain.front().Distance(pv.back()) < weld) {
                    chain.insert(chain.begin(), pv.begin(), pv.end() - 1);
                } else if (chain.front().Distance(pv.front()) < weld) {
                    std::reverse(pv.begin(), pv.end());
                    chain.insert(chain.begin(), pv.begin(), pv.end() - 1);
                } else {
                    continue;
                }
                used[k] = 1;
                progress = true;
            }
        }
        for (char u : used) {
            if (!u) return {};  // disconnected chain
        }
        return chain;
    };

    std::vector<gp_Pnt> O = sampleChain(outerEdges, std::max(3, radialDefault));
    std::vector<gp_Pnt> I = sampleChain(innerEdges, std::max(3, radialDefault));
    if (O.size() < 2 || I.size() < 2) return false;
    // Run both rails the same rotational sense: inner end 0 near outer end 0.
    if (O.front().Distance(I.back()) < O.front().Distance(I.front())) {
        std::reverse(I.begin(), I.end());
    }

    // The two walls carry the across-ring row count. Sample each from its
    // outer end to its inner end; match to the left (O.front/I.front) and
    // right (O.back/I.back) corners.
    auto sampleWall = [&](int eid) -> std::vector<gp_Pnt> {
        const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
        double f, l;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
        if (c3.IsNull()) return {};
        int n = (eid >= 1 && eid < int(solvedEdge.size())) ? solvedEdge[eid]
                                                           : 0;
        if (n < 1) n = 1;
        const bool rev = e.Orientation() == TopAbs_REVERSED;
        const double ph = closedEdgePhase(e, model);
        std::vector<gp_Pnt> pc;
        for (double t : edgeSampleFractions(eid, n, ph, rev,
                                            /*includeLast=*/true, pins,
                                            &model)) {
            pc.push_back(c3->Value(f + (l - f) * t));
        }
        return pc;
    };
    std::vector<gp_Pnt> wA = sampleWall(wallEdges[0]);
    std::vector<gp_Pnt> wB = sampleWall(wallEdges[1]);
    if (wA.size() < 2 || wB.size() < 2) return false;
    // Left wall joins O.front to I.front; right joins O.back to I.back.
    auto orientWall = [&](std::vector<gp_Pnt> w, const gp_Pnt& outEnd,
                          const gp_Pnt& inEnd) {
        if (w.front().Distance(outEnd) > w.back().Distance(outEnd)) {
            std::reverse(w.begin(), w.end());
        }
        (void)inEnd;
        return w;
    };
    std::vector<gp_Pnt> Lw, Rw;
    if (wA.front().Distance(O.front()) + wA.back().Distance(O.front()) <
        wB.front().Distance(O.front()) + wB.back().Distance(O.front())) {
        Lw = orientWall(wA, O.front(), I.front());
        Rw = orientWall(wB, O.back(), I.back());
    } else {
        Lw = orientWall(wB, O.front(), I.front());
        Rw = orientWall(wA, O.back(), I.back());
    }
    int R = int(Lw.size()) - 1;
    if (int(Rw.size()) - 1 != R) R = std::min(R, int(Rw.size()) - 1);
    if (R < 1) R = 1;

    // Ring centre/axis + radii for polar interior-row placement.
    gp_Pnt ctr;
    gp_Dir axis(0, 0, 1);
    double rOut = 0, rIn = 0;
    {
        BRepAdaptor_Curve oc(TopoDS::Edge(model.edges(outerEdges[0])));
        BRepAdaptor_Curve ic(TopoDS::Edge(model.edges(innerEdges[0])));
        const gp_Circ oci = oc.Circle();
        ctr = oci.Location();
        axis = oci.Axis().Direction();
        rOut = oci.Radius();
        rIn = ic.Circle().Radius();
    }
    const gp_Vec ax(axis);
    auto polar = [&](const gp_Pnt& p, double radius) {
        gp_Vec v(ctr, p);
        gp_Vec radial = v - ax.Multiplied(v.Dot(ax));
        if (radial.Magnitude() < 1e-12) return p;
        radial.Normalize();
        return gp_Pnt(ctr.XYZ() + radial.Multiplied(radius).XYZ());
    };

    const int M = int(O.size());
    // Build one id array per radial row. Rows 0..R-1 carry M columns; row R
    // is the inner rail (its own count). Adjacent rows share their vertices.
    std::vector<std::vector<uint32_t>> rowId(R + 1);
    std::vector<std::vector<gp_Pnt>> rowPt(R + 1);
    rowPt[0] = O;
    rowPt[R] = I;
    for (int r = 1; r < R; ++r) {
        const double t = double(r) / R;
        const double rad = rOut + (rIn - rOut) * t;
        rowPt[r].resize(M);
        for (int c = 0; c < M; ++c) {
            if (c == 0) rowPt[r][c] = Lw[std::min(r, int(Lw.size()) - 1)];
            else if (c == M - 1) rowPt[r][c] = Rw[std::min(r, int(Rw.size()) - 1)];
            else rowPt[r][c] = polar(O[c], rad);
        }
    }
    for (int r = 0; r <= R; ++r) {
        rowId[r].resize(rowPt[r].size());
        for (size_t c = 0; c < rowPt[r].size(); ++c) {
            rowId[r][c] = out.addVertex(rowPt[r][c], {});
        }
    }

    // Winding: compare a sample quad's normal to the face normal.
    BRepAdaptor_Surface surf(face);
    double um = (surf.FirstUParameter() + surf.LastUParameter()) / 2;
    double vm = (surf.FirstVParameter() + surf.LastVParameter()) / 2;
    gp_Pnt sp;
    gp_Vec du, dv;
    surf.D1(um, vm, sp, du, dv);
    gp_Vec fn = du.Crossed(dv);
    if (face.Orientation() == TopAbs_REVERSED) fn.Reverse();
    bool flip = false;
    if (fn.Magnitude() > 1e-12) {
        gp_Vec pn = gp_Vec(O[0], O[1 % M]).Crossed(gp_Vec(O[0], I[0]));
        flip = pn.Dot(fn) < 0;
    }

    // Interior quad bands (both rows M-wide).
    for (int r = 1; r < R; ++r) {
        for (int c = 0; c + 1 < M; ++c) {
            out.addPolygon({rowId[r - 1][c], rowId[r - 1][c + 1],
                            rowId[r][c + 1], rowId[r][c]},
                           faceId, flip);
        }
    }

    // Final band: arc-fraction ladder between the last M-wide row and the
    // inner rail — quads with grouped n-gons absorbing the count mismatch.
    const std::vector<uint32_t>& A = rowId[R - 1];
    const std::vector<gp_Pnt>& Ap = rowPt[R - 1];
    const std::vector<uint32_t>& B = rowId[R];
    const std::vector<gp_Pnt>& Bp = rowPt[R];
    auto arcFrac = [](const std::vector<gp_Pnt>& pts) {
        std::vector<double> f(pts.size(), 0.0);
        for (size_t i = 1; i < pts.size(); ++i) {
            f[i] = f[i - 1] + pts[i].Distance(pts[i - 1]);
        }
        const double t = f.back() > 1e-12 ? f.back() : 1.0;
        for (double& x : f) x /= t;
        return f;
    };
    const bool aSparse = A.size() <= B.size();
    const std::vector<uint32_t>& S = aSparse ? A : B;
    const std::vector<uint32_t>& D = aSparse ? B : A;
    const std::vector<double> sf = arcFrac(aSparse ? Ap : Bp);
    const std::vector<double> df = arcFrac(aSparse ? Bp : Ap);
    const int m = int(S.size()) - 1;
    const int nd = int(D.size()) - 1;
    std::vector<int> mp(m + 1);
    mp[0] = 0;
    mp[m] = nd;
    for (int k = 1; k < m; ++k) {
        int j = mp[k - 1];
        while (j + 1 < nd &&
               std::abs(df[j + 1] - sf[k]) <= std::abs(df[j] - sf[k])) {
            ++j;
        }
        mp[k] = j;
    }
    for (int k = 0; k < m; ++k) {
        std::vector<uint32_t> ring2;
        if (aSparse) {
            ring2 = {S[k], S[k + 1]};
            for (int t = mp[k + 1]; t >= mp[k]; --t) ring2.push_back(D[t]);
        } else {
            for (int t = mp[k]; t <= mp[k + 1]; ++t) ring2.push_back(D[t]);
            ring2.push_back(S[k + 1]);
            ring2.push_back(S[k]);
        }
        ring2.erase(std::unique(ring2.begin(), ring2.end()), ring2.end());
        if (ring2.size() > 1 && ring2.front() == ring2.back()) ring2.pop_back();
        if (ring2.size() < 3) continue;
        out.addPolygon(std::move(ring2), faceId, flip);
    }
    return true;
}


}  // namespace weft::mesher_impl
