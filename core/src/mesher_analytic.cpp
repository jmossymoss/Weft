#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// Outward normal of a planar face (accounts for face orientation).
gp_Vec planarFaceNormal(const TopoDS_Face& face, const BRepAdaptor_Surface& surf) {
    gp_Vec n(surf.Plane().Axis().Direction());
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    return n;
}

void meshDiskCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const gp_Circ& circ, int faceId, int n, CapStyle cap,
                 MeshBuilder& out, double startAngle) {
    n = std::max(3, n);
    const gp_Pln pln = surf.Plane();
    auto planeAnchor = [&](const gp_Pnt& p) {
        Anchor a{faceId, 0.0, 0.0};
        ElSLib::Parameters(pln, p, a.u, a.v);
        return a;
    };
    // Ring points come from the circle's own parametrization so they land
    // on the same positions as an adjacent revolution side sharing this
    // circle (startAngle carries the phase anchor when the border edge is
    // phased); the weld pass then stitches the two faces watertight.
    std::vector<uint32_t> ring(n);
    std::vector<gp_Pnt> pts(n);
    for (int i = 0; i < n; ++i) {
        pts[i] = ElCLib::Value(startAngle + i * 2.0 * M_PI / n, circ);
        ring[i] = out.addVertex(pts[i], planeAnchor(pts[i]));
    }

    // Ring order follows circle parametrization, which is unrelated to the
    // face's outward side; orient by comparing the ring's normal to the face's.
    gp_Vec ringNormal = gp_Vec(pts[0], pts[1]).Crossed(gp_Vec(pts[1], pts[2]));
    const bool flip = ringNormal.Dot(planarFaceNormal(face, surf)) < 0;

    if (cap == CapStyle::NGon) {
        out.addPolygon(ring, faceId, flip);
    } else {
        uint32_t center =
            out.addVertex(circ.Location(), planeAnchor(circ.Location()));
        for (int i = 0; i < n; ++i) {
            out.addPolygon({center, ring[i], ring[(i + 1) % n]}, faceId, flip);
        }
    }
}

void meshParametricGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        int faceId, const std::vector<double>& uParams,
                        const std::vector<double>& vParams, MeshBuilder& out) {
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const int nu = static_cast<int>(uParams.size()) - 1;
    const int nv = static_cast<int>(vParams.size()) - 1;
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    std::vector<uint32_t> grid((nu + 1) * (nv + 1));
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            double u = umin + uParams[i] * (umax - umin);
            double v = vmin + vParams[j] * (vmax - vmin);
            grid[j * (nu + 1) + i] =
                out.addVertex(surf.Value(u, v), {faceId, u, v});
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            out.addPolygon({grid[j * (nu + 1) + i], grid[j * (nu + 1) + i + 1],
                            grid[(j + 1) * (nu + 1) + i + 1],
                            grid[(j + 1) * (nu + 1) + i]},
                           faceId, flip);
        }
    }
}

// A one-direction-closed blend ring: the chart is closed in v (the ring)
// and open in u (the profile); the boundary is exactly two closed rims
// (one B-rep edge each, at the profile extremes) joined by a seam. The
// demo strut skirts are the diagnosed case — fillet bands around tilted
// cylinders, where the coons patch's 3D transfinite blend fights the
// strongly non-arclength chart, folds, and loses the self-heal
// tournament to the contract floor (the "dissolving" collar).
//
// This lattice never interpolates in 3D. Rim rows are the rims' exact
// contract samples; every interior vertex is evaluated DIRECTLY on the
// surface at station parameters lerped between the paired rim stations
// (wrap-shortest in v, seam-arc-fraction weights in u), so it cannot
// fold from transfinite warp by construction. Station pairing between
// the rims takes the least-twist rotation (min total wrapped v
// distance) — per-station, which is what the coons rotate knob could
// never express. The ring is emitted CLOSED (no duplicated seam
// column); the seam edge is internal to the face and exempt from the
// border contract. Callers gate the result transactionally (contract +
// fold census) and fall back to the historic coons byte-identically.
bool meshRingLattice(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                     const Model& model, int faceId,
                     const std::vector<int>& solvedEdge,
                     const PinnedEdges* pins, MeshBuilder& out) {
    if (!surf.IsVClosed() || surf.IsUClosed()) return false;
    const double vLo = surf.FirstVParameter();
    const double period = surf.IsVPeriodic()
                              ? surf.VPeriod()
                              : surf.LastVParameter() - vLo;
    if (!(period > 0)) return false;

    // Boundary census: exactly one seam (used twice by the face) and two
    // one-sided rim edges. Anything richer (multi-edge rims, extra
    // borders, holes) stays on the coons path.
    std::map<int, int> occur;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        if (BRep_Tool::Degenerated(TopoDS::Edge(ex.Current()))) continue;
        const int eid = model.edges.FindIndex(ex.Current());
        if (eid >= 1) ++occur[eid];
    }
    int seamEid = 0;
    std::vector<int> rimEids;
    for (const auto& [eid, cnt] : occur) {
        if (cnt == 2) {
            if (seamEid) return false;
            seamEid = eid;
        } else if (cnt == 1) {
            rimEids.push_back(eid);
        } else {
            return false;
        }
    }
    if (!seamEid || rimEids.size() != 2) return false;

    struct Rim {
        std::vector<gp_Pnt> p;     // M ring stations (closed, no repeat)
        std::vector<double> u, v;  // pcurve uv per station
        double uMean = 0;
    };
    Rim rim[2];
    for (int k = 0; k < 2; ++k) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(rimEids[k]));
        double f2, l2, f3, l3;
        Handle(Geom2d_Curve) pc =
            BRep_Tool::CurveOnSurface(edge, face, f2, l2);
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        if (pc.IsNull() || c3.IsNull()) return false;
        // The rim must itself be a closed ring, not an open border arc.
        if (c3->Value(f3).Distance(c3->Value(l3)) >
            std::max(1e-6, BRep_Tool::Tolerance(edge))) {
            return false;
        }
        const int n = rimEids[k] < int(solvedEdge.size())
                          ? solvedEdge[rimEids[k]]
                          : 0;
        if (!edgeIsPinned(rimEids[k], pins) && n < 3) return false;
        const double ph = closedEdgePhase(edge, model);
        for (double t : edgeSampleFractions(rimEids[k], n, ph, false,
                                            /*includeLast=*/false, pins,
                                            &model)) {
            const gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
            rim[k].p.push_back(c3->Value(f3 + (l3 - f3) * t));
            rim[k].u.push_back(uv.X());
            rim[k].v.push_back(uv.Y());
        }
    }
    // Equal station counts or no lattice (the density solve's rail
    // alignment usually delivers this; a mismatch means transition
    // topology this mesher does not attempt).
    const int M = int(rim[0].p.size());
    if (M < 3 || int(rim[1].p.size()) != M) return false;

    // Normalize both rims to ascend v (wrap-aware), then place the
    // low-profile rim first.
    for (int k = 0; k < 2; ++k) {
        Rim& R = rim[k];
        double turn = 0;
        for (int i = 0; i + 1 < M; ++i) {
            double d = R.v[i + 1] - R.v[i];
            d -= period * std::round(d / period);
            turn += d;
        }
        if (turn < 0) {
            std::reverse(R.p.begin(), R.p.end());
            std::reverse(R.u.begin(), R.u.end());
            std::reverse(R.v.begin(), R.v.end());
        }
        for (double u : R.u) R.uMean += u;
        R.uMean /= M;
    }
    Rim& A = rim[rim[0].uMean <= rim[1].uMean ? 0 : 1];
    Rim& B = rim[rim[0].uMean <= rim[1].uMean ? 1 : 0];
    if (!(B.uMean - A.uMean >
          1e-6 * (surf.LastUParameter() - surf.FirstUParameter()))) {
        return false;
    }

    // Profile rows at the seam's solved stations: the seam samples are
    // even in 3D arc length, so their u parameters give well-spaced rows
    // even on wildly non-arclength charts (the skirt's u range is 12x
    // its profile length).
    const TopoDS_Edge seam = TopoDS::Edge(model.edges(seamEid));
    double sf2, sl2;
    Handle(Geom2d_Curve) spc = BRep_Tool::CurveOnSurface(seam, face, sf2, sl2);
    if (spc.IsNull()) return false;
    const int np =
        seamEid < int(solvedEdge.size()) ? solvedEdge[seamEid] : 0;
    if (!edgeIsPinned(seamEid, pins) && np < 1) return false;
    std::vector<double> su;
    for (double t : edgeSampleFractions(seamEid, np, 0.0, false,
                                        /*includeLast=*/true, pins, &model)) {
        su.push_back(spc->Value(sf2 + (sl2 - sf2) * t).X());
    }
    if (su.size() < 2) return false;
    if (su.front() > su.back()) std::reverse(su.begin(), su.end());
    for (size_t i = 0; i + 1 < su.size(); ++i) {
        if (su[i + 1] <= su[i]) return false;  // rows must advance
    }
    const int NP = int(su.size()) - 1;
    const double span = su[NP] - su[0];
    if (!(span > 0)) return false;

    // Least-twist station pairing: the rotation of B against A that
    // minimizes total wrapped v distance.
    int bestR = 0;
    double bestCost = 1e300;
    for (int r = 0; r < M; ++r) {
        double c = 0;
        for (int j = 0; j < M; ++j) {
            double d = B.v[(j + r) % M] - A.v[j];
            d -= period * std::round(d / period);
            c += std::abs(d);
        }
        if (c < bestCost) {
            bestCost = c;
            bestR = r;
        }
    }

    std::vector<uint32_t> grid((NP + 1) * M);
    for (int j = 0; j < M; ++j) {
        grid[j] = out.addVertex(A.p[j], {faceId, A.u[j], A.v[j]});
    }
    for (int j = 0; j < M; ++j) {
        const int k = (j + bestR) % M;
        grid[NP * M + j] = out.addVertex(B.p[k], {faceId, B.u[k], B.v[k]});
    }
    for (int r = 1; r < NP; ++r) {
        const double w = (su[r] - su[0]) / span;
        for (int j = 0; j < M; ++j) {
            const int k = (j + bestR) % M;
            double dv = B.v[k] - A.v[j];
            dv -= period * std::round(dv / period);
            double vv = A.v[j] + w * dv;
            vv -= period * std::floor((vv - vLo) / period);
            const double uu = A.u[j] + w * (B.u[k] - A.u[j]);
            grid[r * M + j] =
                out.addVertex(surf.Value(uu, vv), {faceId, uu, vv});
        }
    }
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    for (int r = 0; r < NP; ++r) {
        for (int j = 0; j < M; ++j) {
            const int j1 = (j + 1) % M;
            out.addPolygon({grid[r * M + j], grid[(r + 1) * M + j],
                            grid[(r + 1) * M + j1], grid[r * M + j1]},
                           faceId, flip);
        }
    }
    return true;
}

// Concentric quad rings between a hole circle and the rectangular border
// of a planar face. The circle takes one vertex per boundary vertex (the
// solver guarantees ring count == 2*(nu+nv)); circle vertices reuse the
// circle's own parametrization so the adjacent boss/bore welds watertight,
// and border vertices sit on the same grid nodes as the neighbouring faces.
void meshRingJunction(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                      const gp_Circ& circ, int faceId, int nu, int nv,
                      int loops, MeshBuilder& out, double startAngle) {
    const int n = 2 * (nu + nv);
    loops = std::max(1, loops);

    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double du = (umax - umin) / nu;
    const double dv = (vmax - vmin) / nv;

    // Border vertices, walking the rectangle perimeter cyclically.
    std::vector<gp_Pnt> border;
    border.reserve(n);
    for (int i = 0; i < nu; ++i) border.push_back(surf.Value(umin + i * du, vmin));
    for (int j = 0; j < nv; ++j) border.push_back(surf.Value(umax, vmin + j * dv));
    for (int i = nu; i > 0; --i) border.push_back(surf.Value(umin + i * du, vmax));
    for (int j = nv; j > 0; --j) border.push_back(surf.Value(umin, vmin + j * dv));

    std::vector<gp_Pnt> ring(n);
    for (int k = 0; k < n; ++k) {
        ring[k] = ElCLib::Value(startAngle + k * 2.0 * M_PI / n, circ);
    }

    // Pair border and ring vertices by angle around the circle center: make
    // the border loop run the same way as the circle parametrization, then
    // rotate it so border[0] sits nearest ring[0]'s angle.
    const gp_Pnt center = circ.Location();
    const gp_Vec X(circ.Position().XDirection());
    const gp_Vec Y(circ.Position().YDirection());
    auto angleOf = [&](const gp_Pnt& p) {
        gp_Vec d(center, p);
        return std::atan2(d.Dot(Y), d.Dot(X));
    };
    auto wrap = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a <= -M_PI) a += 2.0 * M_PI;
        return a;
    };
    double turn = 0.0;
    for (int k = 0; k < n; ++k) {
        turn += wrap(angleOf(border[(k + 1) % n]) - angleOf(border[k]));
    }
    if (turn < 0) std::reverse(border.begin(), border.end());

    int best = 0;
    double bestDist = 1e30;
    for (int k = 0; k < n; ++k) {
        double d = std::abs(wrap(angleOf(border[k])));
        if (d < bestDist) { bestDist = d; best = k; }
    }
    std::rotate(border.begin(), border.begin() + best, border.end());

    // Concentric rings: r=0 is the circle, r=loops is the border.
    const gp_Pln pln = surf.Plane();
    auto planeAnchor = [&](const gp_Pnt& p) {
        Anchor a{faceId, 0.0, 0.0};
        ElSLib::Parameters(pln, p, a.u, a.v);
        return a;
    };
    std::vector<std::vector<uint32_t>> rows(loops + 1, std::vector<uint32_t>(n));
    for (int k = 0; k < n; ++k) {
        rows[0][k] = out.addVertex(ring[k], planeAnchor(ring[k]));
    }
    for (int r = 1; r < loops; ++r) {
        double t = double(r) / loops;
        for (int k = 0; k < n; ++k) {
            gp_Pnt p(ring[k].X() + t * (border[k].X() - ring[k].X()),
                     ring[k].Y() + t * (border[k].Y() - ring[k].Y()),
                     ring[k].Z() + t * (border[k].Z() - ring[k].Z()));
            rows[r][k] = out.addVertex(p, planeAnchor(p));
        }
    }
    for (int k = 0; k < n; ++k) {
        rows[loops][k] = out.addVertex(border[k], planeAnchor(border[k]));
    }

    // Winding: the ring runs counter-clockwise around the circle axis; flip
    // if that disagrees with the face's outward normal.
    const bool flip =
        gp_Vec(circ.Position().Direction()).Dot(planarFaceNormal(face, surf)) > 0;
    for (int r = 0; r < loops; ++r) {
        for (int k = 0; k < n; ++k) {
            int k2 = (k + 1) % n;
            out.addPolygon({rows[r][k], rows[r][k2], rows[r + 1][k2],
                            rows[r + 1][k]},
                           faceId, flip);
        }
    }
}

// A planar face as one boundary n-gon: perimeter walk over the solved
// border subdivisions. Interior topology is the engine's problem.
void meshMinimalNGon(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                     int faceId, int nu, int nv, MeshBuilder& out) {
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double du = (umax - umin) / nu;
    const double dv = (vmax - vmin) / nv;

    std::vector<uint32_t> ring;
    ring.reserve(2 * (nu + nv));
    auto add = [&](double u, double v) {
        ring.push_back(out.addVertex(surf.Value(u, v), {faceId, u, v}));
    };
    for (int i = 0; i < nu; ++i) add(umin + i * du, vmin);
    for (int j = 0; j < nv; ++j) add(umax, vmin + j * dv);
    for (int i = nu; i > 0; --i) add(umin + i * du, vmax);
    for (int j = nv; j > 0; --j) add(umin, vmin + j * dv);

    out.addPolygon(ring, faceId, face.Orientation() == TopAbs_REVERSED);
}

// Corner-angle quality of a polygon: total deviation from 90-degree
// corners, or a large penalty when a corner is degenerate/reflex.
double quadAngleCost(const std::array<gp_Pnt, 4>& q) {
    double cost = 0;
    for (int i = 0; i < 4; ++i) {
        gp_Vec e1(q[i], q[(i + 1) % 4]);
        gp_Vec e2(q[i], q[(i + 3) % 4]);
        if (e1.Magnitude() < 1e-12 || e2.Magnitude() < 1e-12) return 1e9;
        double deg = e1.Angle(e2) * 180.0 / M_PI;
        if (deg < 20.0 || deg > 160.0) return 1e9;
        cost += std::abs(deg - 90.0);
    }
    return cost;
}

// Pair adjacent triangles of a part into quads (the same greedy-by-angle
// move the fallback floor uses). Only interior diagonals merge — a border
// edge is on the outline and belongs to ONE triangle, so it is never a
// merge candidate, and the outline plus every weld contract stay bit-
// identical. A reflex or badly skewed merge is rejected by the cost
// cutoff, so no folded quad ships. This lifts quad-fill's CDT rim from a
// triangle fan to quad-dominant flow without touching the interior grid
// (already quads) or moving a single vertex.
void pairPartTris(PolyMesh& part) {
    auto p3 = [&](uint32_t v) {
        return gp_Pnt(part.vertices[v][0], part.vertices[v][1],
                      part.vertices[v][2]);
    };
    auto ekey = [](uint32_t a, uint32_t b) {
        return (uint64_t(std::min(a, b)) << 32) | std::max(a, b);
    };
    std::vector<std::array<uint32_t, 3>> tris;
    std::vector<int> triFace;
    std::vector<std::vector<uint32_t>> keep;
    std::vector<int> keepFace;
    for (size_t i = 0; i < part.polygons.size(); ++i) {
        const int fid =
            i < part.polygonFaceId.size() ? part.polygonFaceId[i] : -1;
        if (part.polygons[i].size() == 3) {
            tris.push_back({part.polygons[i][0], part.polygons[i][1],
                            part.polygons[i][2]});
            triFace.push_back(fid);
        } else {
            keep.push_back(part.polygons[i]);
            keepFace.push_back(fid);
        }
    }
    if (tris.size() < 2) return;
    // Edge -> the (up to two) triangles that share it. An edge touched by
    // three triangles is non-manifold input; leave those out of pairing.
    std::map<uint64_t, std::array<int, 2>> em;
    std::set<uint64_t> tooMany;
    for (size_t t = 0; t < tris.size(); ++t) {
        for (int i = 0; i < 3; ++i) {
            const uint64_t k = ekey(tris[t][i], tris[t][(i + 1) % 3]);
            auto& e = em.emplace(k, std::array<int, 2>{-1, -1}).first->second;
            if (e[0] < 0) e[0] = int(t);
            else if (e[1] < 0) e[1] = int(t);
            else tooMany.insert(k);
        }
    }
    struct Cand {
        double cost;
        int t1, t2;
        std::array<uint32_t, 4> ring;
    };
    std::vector<Cand> cands;
    for (const auto& [key, e] : em) {
        if (e[1] < 0 || tooMany.count(key)) continue;
        const int t1 = e[0], t2 = e[1];
        if (triFace[t1] != triFace[t2]) continue;
        const uint32_t p = uint32_t(key >> 32), q = uint32_t(key);
        uint32_t P = p, Q = q;
        bool fwd = false;
        for (int i = 0; i < 3; ++i) {
            if (tris[t1][i] == p && tris[t1][(i + 1) % 3] == q) fwd = true;
        }
        if (!fwd) std::swap(P, Q);
        uint32_t c = 0, d = 0;
        for (uint32_t v : tris[t1]) if (v != p && v != q) c = v;
        for (uint32_t v : tris[t2]) if (v != p && v != q) d = v;
        const std::array<uint32_t, 4> ring{P, d, Q, c};
        const double cost = quadAngleCost(
            {p3(ring[0]), p3(ring[1]), p3(ring[2]), p3(ring[3])});
        if (cost > 1e8) continue;
        cands.push_back({cost, t1, t2, ring});
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.cost < b.cost; });
    std::vector<char> used(tris.size(), 0);
    std::vector<std::vector<uint32_t>> polys;
    std::vector<int> polyFace;
    for (const Cand& cd : cands) {
        if (used[cd.t1] || used[cd.t2]) continue;
        used[cd.t1] = used[cd.t2] = 1;
        polys.push_back(
            {cd.ring[0], cd.ring[1], cd.ring[2], cd.ring[3]});
        polyFace.push_back(triFace[cd.t1]);
    }
    for (size_t t = 0; t < tris.size(); ++t) {
        if (used[t]) continue;
        polys.push_back({tris[t][0], tris[t][1], tris[t][2]});
        polyFace.push_back(triFace[t]);
    }
    for (size_t i = 0; i < keep.size(); ++i) {
        polys.push_back(std::move(keep[i]));
        polyFace.push_back(keepFace[i]);
    }
    part.polygons = std::move(polys);
    part.polygonFaceId = std::move(polyFace);
}


}  // namespace weft::mesher_impl
