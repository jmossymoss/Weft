#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// ---------------------------------------------------------------------------
// Execution.

// A revolution band whose two rims carry DIFFERENT counts (linkRims off):
// one ring per rim, zippered with triangles by angular fraction. Rim
// vertices evaluate exactly like the quad band's, so caps still weld.
void meshRevolutionTaper(const TopoDS_Face& face,
                         const BRepAdaptor_Surface& surf, int faceId, int nA,
                         int nB, double phaseV0, double phaseV1,
                         MeshBuilder& out) {
    nA = std::max(3, nA);
    nB = std::max(3, nB);
    const double uRange = surf.LastUParameter() - surf.FirstUParameter();
    const double v0 = surf.FirstVParameter();
    const double v1 = surf.LastVParameter();
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    std::vector<uint32_t> A(nA), B(nB);
    for (int i = 0; i < nA; ++i) {
        double u = phaseV0 + uRange * i / nA;
        A[i] = out.addVertex(surf.Value(u, v0), {faceId, u, v0});
    }
    for (int j = 0; j < nB; ++j) {
        double u = phaseV1 + uRange * j / nB;
        B[j] = out.addVertex(surf.Value(u, v1), {faceId, u, v1});
    }
    int ia = 0, ib = 0;
    while (ia < nA || ib < nB) {
        double fa = double(ia + 1) / nA, fb = double(ib + 1) / nB;
        bool stepA = ib >= nB || (ia < nA && fa <= fb);
        if (stepA) {
            out.addPolygon({A[ia % nA], A[(ia + 1) % nA], B[ib % nB]},
                           faceId, flip);
            ++ia;
        } else {
            out.addPolygon({A[ia % nA], B[(ib + 1) % nB], B[ib % nB]},
                           faceId, flip);
            ++ib;
        }
    }
}

// Quad grid over a closed-in-u surface of revolution. Handles a closed v
// (torus) by wrapping rows, and degenerate rows (cone apex, sphere poles)
// by collapsing them to one vertex — the weld pass then turns the adjacent
// quads into triangles.
// Phase-align a revolution grid's u sampling to a rim EDGE's curve start:
// two analytic faces sharing that circle then sample the identical points
// (each surface's own u origin can be rotated arbitrarily — torus vs
// cylinder — which used to leave every shared rim vertex slightly off).
double revolutionUPhase(const BRepAdaptor_Surface& surf,
                        const Model& model, int rimEdgeId,
                        bool* atLastV) {
    const double u0 = surf.FirstUParameter();
    if (atLastV) *atLastV = false;
    if (rimEdgeId < 1 || rimEdgeId > model.edgeCount()) return u0;
    const TopoDS_Edge edge = TopoDS::Edge(model.edges(rimEdgeId));
    if (BRep_Tool::Degenerated(edge)) return u0;
    double f, l;
    if (BRep_Tool::Curve(edge, f, l).IsNull()) return u0;
    BRepAdaptor_Curve c(edge);
    // The taper's ring must start where the PHASED first border sample
    // sits, not at the curve origin.
    const gp_Pnt p0 = c.Value(
        c.FirstParameter() + (c.LastParameter() - c.FirstParameter()) *
                                 closedEdgePhase(edge, model));
    const double range = surf.LastUParameter() - u0;
    // Which v end the rim lives at.
    double dFirst = 1e300, dLast = 1e300;
    for (int k = 0; k < 8; ++k) {
        double u = u0 + range * k / 8.0;
        dFirst = std::min(dFirst,
                          p0.Distance(surf.Value(u, surf.FirstVParameter())));
        dLast = std::min(dLast,
                         p0.Distance(surf.Value(u, surf.LastVParameter())));
    }
    const bool last = dFirst > dLast;
    if (atLastV) *atLastV = last;
    const double v =
        last ? surf.LastVParameter() : surf.FirstVParameter();
    // Coarse scan + a few bisection refinements onto the edge start.
    double best = u0, bestD = 1e300;
    const int kCoarse = 64;
    for (int k = 0; k < kCoarse; ++k) {
        double u = u0 + range * k / kCoarse;
        double d = p0.Distance(surf.Value(u, v));
        if (d < bestD) { bestD = d; best = u; }
    }
    double step = range / kCoarse;
    for (int it = 0; it < 24; ++it) {
        step /= 2;
        for (double u : {best - step, best + step}) {
            double d = p0.Distance(surf.Value(u, v));
            if (d < bestD) { bestD = d; best = u; }
        }
    }
    return best;
}


// Returns false when the two rims carry irreconcilably different totals
// (multi-edge chains on both sides that the density sum constraint could
// not equalize): the closed transition strip that would bridge them
// degenerates into folded lunes on thin bands, so the face takes the
// contract floor instead.
// Mesh a spherical / dome cap (planDomeCap) as a UV hemisphere: concentric
// latitude rings from the base loop to the pole, straight meridians at even
// azimuth aligned to the base samples, closed at the crown by a triangle fan
// into a single apex vertex. The base ring samples the shared base edges at
// their solved counts (so it welds to the neighbours bit-for-bit — the
// watertight contract); interior rings and meridians walk the true surface at
// constant azimuth param, so they follow a squashed/ellipsoidal dome exactly.
bool meshDomeCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const Model& model, const FacePlan& plan,
                 const std::vector<int>& solvedEdge, int faceId, int nLat,
                 MeshBuilder& out, const PinnedEdges* pins) {
    const bool azimIsV = plan.domeAzimIsV;
    const double polarBase = plan.domePolarBase;
    const double polarPole = plan.domePolarPole;
    nLat = std::max(2, nLat);
    auto surfAt = [&](double polar, double azim) {
        return azimIsV ? surf.Value(polar, azim) : surf.Value(azim, polar);
    };

    // Base ring: sample the shared base edges at their solved counts, walked
    // in outer-wire order so azimuth is monotone around the loop. Each sample
    // keeps its exact 3D point (welds to the neighbour) and its surface (u,v)
    // — the polar coordinate is the axis climb, the azimuth coordinate drives
    // the meridian above it.
    struct BasePt { gp_Pnt p; double u, v, azim; };
    std::vector<BasePt> ring0;
    const std::set<int> baseSet(plan.uEdges.begin(), plan.uEdges.end());
    std::set<int> seen;
    auto sampleBaseEdge = [&](const TopoDS_Edge& e, int eid) {
        double f2, l2, f3, l3;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e, face, f2, l2);
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f3, l3);
        if (pc.IsNull() || c3.IsNull()) return;
        seen.insert(eid);
        int n = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
        if (n < 1) n = 3;
        const bool rev = e.Orientation() == TopAbs_REVERSED;
        const double ph = closedEdgePhase(e, model);
        for (double t : edgeSampleFractions(eid, n, ph, rev,
                                            /*includeLast=*/false, pins,
                                            &model)) {
            const gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
            const gp_Pnt p = c3->Value(f3 + (l3 - f3) * t);
            ring0.push_back({p, uv.X(), uv.Y(),
                             azimIsV ? uv.Y() : uv.X()});
        }
    };
    for (BRepTools_WireExplorer we(BRepTools::OuterWire(face), face); we.More();
         we.Next()) {
        const TopoDS_Edge e = we.Current();
        if (BRep_Tool::Degenerated(e)) continue;
        const int eid = model.edges.FindIndex(e);
        if (eid < 1 || !baseSet.count(eid) || seen.count(eid)) continue;
        sampleBaseEdge(e, eid);
    }
    for (int eid : plan.uEdges) {  // WireExplorer can silently drop edges
        if (eid < 1 || eid > model.edgeCount() || seen.count(eid)) continue;
        sampleBaseEdge(TopoDS::Edge(model.edges(eid)), eid);
    }
    const int nAz = int(ring0.size());
    if (nAz < 3) return false;

    // Normalize the azimuth direction: walk the frame and reverse if the loop
    // runs clockwise, so latitude rings and meridians share one handedness.
    {
        const double aSpan = azimIsV ? (surf.LastVParameter() - surf.FirstVParameter())
                                     : (surf.LastUParameter() - surf.FirstUParameter());
        double turn = 0;
        for (int k = 0; k < nAz; ++k) {
            double d = ring0[(k + 1) % nAz].azim - ring0[k].azim;
            d -= aSpan * std::round(d / aSpan);
            turn += d;
        }
        if (turn < 0) std::reverse(ring0.begin(), ring0.end());
    }

    // Latitude ring vertices. Row 0 is the exact base samples; interior rows
    // walk the surface at even polar fractions, constant azimuth per column
    // (straight meridians); the last row is the single apex vertex, repeated
    // per column so the shared quad loop collapses it into a clean pole fan.
    std::vector<std::vector<uint32_t>> ring(nLat + 1);
    ring[0].resize(nAz);
    for (int k = 0; k < nAz; ++k) {
        ring[0][k] = out.addVertex(ring0[k].p,
                                   {faceId, ring0[k].u, ring0[k].v});
    }
    for (int j = 1; j < nLat; ++j) {
        const double polar =
            polarBase + (polarPole - polarBase) * (double(j) / nLat);
        ring[j].resize(nAz);
        for (int k = 0; k < nAz; ++k) {
            const double azim = ring0[k].azim;
            const gp_Pnt p = surfAt(polar, azim);
            const double u = azimIsV ? polar : azim;
            const double v = azimIsV ? azim : polar;
            ring[j][k] = out.addVertex(p, {faceId, u, v});
        }
    }
    const gp_Pnt apex = surfAt(polarPole, ring0[0].azim);
    const uint32_t apexIdx = out.addVertex(
        apex, {faceId, azimIsV ? polarPole : ring0[0].azim,
                       azimIsV ? ring0[0].azim : polarPole});
    ring[nLat].assign(nAz, apexIdx);

    // Winding: pick the single flip that makes the emitted cells agree with
    // orient * (du x dv) (the same rule foldedPolys judges by), sampled at an
    // interior ring vertex where the surface normal is well defined.
    bool flip = false;
    {
        const double orient = face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
        const int jt = 1;  // first interior ring
        const double polar =
            polarBase + (polarPole - polarBase) * (double(jt) / nLat);
        gp_Pnt sp; gp_Vec du, dv;
        const double au = azimIsV ? polar : ring0[0].azim;
        const double av = azimIsV ? ring0[0].azim : polar;
        surf.D1(au, av, sp, du, dv);
        gp_Vec sn = du.Crossed(dv);
        if (sn.Magnitude() > 1e-14) {
            sn *= orient;
            // Newell normal of the test quad in default (unflipped) order.
            const std::array<uint32_t, 4> q{ring[0][0], ring[0][1], ring[1][1],
                                            ring[1][0]};
            gp_Vec nw(0, 0, 0);
            for (int i = 0; i < 4; ++i) {
                const auto& A = out.mesh().vertices[q[i]];
                const auto& B = out.mesh().vertices[q[(i + 1) % 4]];
                nw += gp_Vec((A[1] - B[1]) * (A[2] + B[2]),
                             (A[2] - B[2]) * (A[0] + B[0]),
                             (A[0] - B[0]) * (A[1] + B[1]));
            }
            if (nw.Dot(sn) < 0) flip = true;
        }
    }

    // Emit the lattice; the pole row (all apex) collapses each top quad into a
    // fan triangle via the consecutive-duplicate squeeze.
    for (int j = 0; j < nLat; ++j) {
        for (int k = 0; k < nAz; ++k) {
            const int k2 = (k + 1) % nAz;
            std::vector<uint32_t> poly{ring[j][k], ring[j][k2], ring[j + 1][k2],
                                       ring[j + 1][k]};
            poly.erase(std::unique(poly.begin(), poly.end()), poly.end());
            if (poly.size() > 1 && poly.front() == poly.back()) poly.pop_back();
            if (poly.size() < 3) continue;
            out.addPolygon(std::move(poly), faceId, flip);
        }
    }
    dbg("domecap face %d: nAz=%d nLat=%d azimV=%d flip=%d", faceId, nAz, nLat,
        azimIsV ? 1 : 0, flip ? 1 : 0);
    return true;
}


}  // namespace weft::mesher_impl
