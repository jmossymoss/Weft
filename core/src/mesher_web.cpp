#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// A planar face with hole loops that the simpler patterns can't take
// (three or more wires, or two wires too edge-rich for the annulus band):
// the bolt-hole plate. Each hole gets a quad collar, the rest is an
// ear-clipped triangle web — every boundary vertex sits on its B-rep edge
// curve at the solved count, so all neighbours weld watertight.
// Collect a planar face's wires as per-edge loop chains, outer wire first.
// Shared by the plate-web planner and the generalized minimal-ngon.
// Geometric flatness: CAD kernels routinely carry visually flat regions
// as bsplines, and "minimal n-gon" is about the GEOMETRY being flat, not
// the surface type. Sample a grid over the UV bounds and measure the
// spread along the average normal.
bool isGeometricallyFlat(const TopoDS_Face& face,
                         const BRepAdaptor_Surface& surf, double flatFrac) {
    if (surf.GetType() == GeomAbs_Plane) return true;
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    const int N = 5;
    gp_XYZ c(0, 0, 0);
    std::array<gp_Pnt, N * N> pts;
    for (int j = 0; j < N; ++j) {
        for (int i = 0; i < N; ++i) {
            pts[j * N + i] = surf.Value(u0 + (u1 - u0) * i / (N - 1),
                                        v0 + (v1 - v0) * j / (N - 1));
            c += pts[j * N + i].XYZ();
        }
    }
    c /= double(N * N);
    // Newell-style normal over the sample grid diagonals.
    gp_XYZ n(0, 0, 0);
    for (int j = 0; j + 1 < N; ++j) {
        for (int i = 0; i + 1 < N; ++i) {
            gp_XYZ d1 = pts[(j + 1) * N + i + 1].XYZ() - pts[j * N + i].XYZ();
            gp_XYZ d2 = pts[(j + 1) * N + i].XYZ() - pts[j * N + i + 1].XYZ();
            n += d1.Crossed(d2);
        }
    }
    if (n.Modulus() < 1e-12) return false;
    n.Normalize();
    // diag is the face's own diameter — NEVER the distance from the
    // world origin: seeding it with |p| made flatness origin-dependent,
    // so a curved 1mm sliver 87mm out measured against an 87mm
    // yardstick and passed as "flat" (weldment pipe-end corner, faces
    // 71/72 — their n-gons tore off the neighbouring walls).
    double lo = 1e300, hi = -1e300, diag = 0;
    for (const gp_Pnt& p : pts) {
        double d = (p.XYZ() - c).Dot(n);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    }
    for (const gp_Pnt& p : pts) {
        for (const gp_Pnt& q : pts) {
            diag = std::max(diag, p.Distance(q));
        }
    }
    if (getenv("WEFT_FLAT_DEBUG")) {
        dbg("flat? dev=%g diag=%g frac=%.4f -> %d", hi - lo, diag,
            diag > 1e-9 ? (hi - lo) / diag : 0.0,
            hi - lo < std::max(1e-6, flatFrac * diag) ? 1 : 0);
    }
    return hi - lo < std::max(1e-6, flatFrac * diag);
}


// A dead-end conical cap so shallow it reads as a flat panel — the top of
// a bevelled spray-can disk (foam faces 325/366): a cone whose taper is a
// small fraction of its width, a single round boundary (no holes), and no
// neighbour to tear. isGeometricallyFlat rejects it (its 3% dish is far
// past the 0.1% plane tolerance), so it would otherwise tri-fan; a single
// boundary n-gon is the clean flat-panel result the CAD n-gon policy wants.
// Deliberately narrow: only genuine caps (round, single loop, barely
// dished) qualify, never a structural cone wall or a strip.
bool isShallowCapCone(const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf) {
    if (surf.GetType() != GeomAbs_Cone) return false;
    // Exactly one wire (a hole would need a web, not a single n-gon).
    int wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
    }
    if (wires != 1) return false;
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    // Sample the surface; measure dish (deviation from the boundary's mean
    // plane) and the in-plane diameter. A cap dishes by a small fraction of
    // its width; a cone WALL runs far in v and dishes as much as it spans.
    const int N = 6;
    std::vector<gp_Pnt> pts;
    gp_XYZ c(0, 0, 0);
    for (int j = 0; j < N; ++j) {
        for (int i = 0; i < N; ++i) {
            gp_Pnt p = surf.Value(u0 + (u1 - u0) * i / (N - 1),
                                  v0 + (v1 - v0) * j / (N - 1));
            pts.push_back(p);
            c += p.XYZ();
        }
    }
    c /= double(pts.size());
    gp_XYZ n(0, 0, 0);
    for (int j = 0; j + 1 < N; ++j) {
        for (int i = 0; i + 1 < N; ++i) {
            gp_XYZ d1 = pts[(j + 1) * N + i + 1].XYZ() - pts[j * N + i].XYZ();
            gp_XYZ d2 = pts[(j + 1) * N + i].XYZ() - pts[j * N + i + 1].XYZ();
            n += d1.Crossed(d2);
        }
    }
    if (n.Modulus() < 1e-12) return false;
    n.Normalize();
    double lo = 1e300, hi = -1e300, diam = 0;
    for (const gp_Pnt& p : pts) {
        double d = (p.XYZ() - c).Dot(n);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    }
    for (const gp_Pnt& p : pts)
        for (const gp_Pnt& q : pts) diam = std::max(diam, p.Distance(q));
    if (diam < 1e-9) return false;
    const double dish = (hi - lo) / diam;  // 0 = flat panel, big = a wall
    if (dish > 0.10) return false;
    // Round, compact cap: the boundary's radius barely varies (a strip or a
    // gouged outline swings wide). Reuse the wire-elongation probe.
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull() || wireElongation(outer) > 1.6) return false;
    return true;
}

bool collectPlanarLoops(const TopoDS_Face& face,
                        const BRepAdaptor_Surface& surf, const Model& model,
                        FacePlan& plan, bool requirePlane,
                        bool tolerateDegenerate) {
    if (requirePlane && !isGeometricallyFlat(face, surf)) return false;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    std::vector<std::vector<int>> loops;
    int outerIdx = -1, wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
        std::vector<int> loop;
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            // A degenerate edge is a POLE: it collapses to a single 3D
            // point that its neighbour edges already reach, so on a
            // pole-tolerant collect it contributes no boundary segment
            // and is simply skipped (the loop's real edges stay a closed
            // 3D ring through the pole). Callers that grid in UV must not
            // rely on the collapsed seam edge existing.
            if (BRep_Tool::Degenerated(edge)) {
                if (tolerateDegenerate) continue;
                return false;
            }
            double f, l;
            if (BRep_Tool::Curve(edge, f, l).IsNull()) return false;
            if (BRep_Tool::CurveOnSurface(edge, face, f, l).IsNull()) {
                return false;
            }
            int eid = model.edges.FindIndex(edge);
            if (eid < 1) return false;
            loop.push_back(eid);
        }
        // Real CAD outlines run to dozens of arcs (rounded-corner
        // brackets); the sampler handles any count, so the cap is only
        // a pathological-input guard.
        if (loop.empty() || loop.size() > 512) return false;
        if (wire.IsSame(outer)) outerIdx = wires;
        loops.push_back(std::move(loop));
        ++wires;
    }
    if (wires < 1 || outerIdx < 0) return false;
    if (outerIdx != 0) std::swap(loops[0], loops[outerIdx]);
    plan.loops = std::move(loops);
    // Flattened list for reporting/conformity; densities stay per-edge
    // (solveDensity never unites these loops' edges with each other).
    plan.uEdges.clear();
    for (const auto& loop : plan.loops) {
        plan.uEdges.insert(plan.uEdges.end(), loop.begin(), loop.end());
    }
    plan.constrains = true;
    return true;
}

// How slot-shaped a hole wire is: max/min distance from the wire's sample
// centroid. A circle is ~1; a 4:1 slot is ~4.
double wireElongation(const TopoDS_Wire& wire) {
    std::vector<gp_Pnt> pts;
    for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        double f, l;
        Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
        if (c.IsNull()) continue;
        for (int i = 0; i < 8; ++i) {
            pts.push_back(c->Value(f + (l - f) * (i + 0.5) / 8.0));
        }
    }
    if (pts.size() < 4) return 1.0;
    gp_XYZ c(0, 0, 0);
    for (const gp_Pnt& p : pts) c += p.XYZ();
    c /= double(pts.size());
    double rMin = 1e300, rMax = 0;
    for (const gp_Pnt& p : pts) {
        double r = p.XYZ().Subtracted(c).Modulus();
        rMin = std::min(rMin, r);
        rMax = std::max(rMax, r);
    }
    return rMin > 1e-12 ? rMax / rMin : 1e300;
}

bool planPlateWeb(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, FacePlan& plan,
                  bool requireRoundHoles) {
    // Probe locally: a rejected plan must not leak loop state into the
    // caller's plan (the auto chain keeps trying other meshers with it).
    FacePlan probe;
    if (!collectPlanarLoops(face, surf, model, probe)) return false;
    if (probe.loops.size() < 2) return false;
    if (requireRoundHoles) {
        // The automatic path only takes plates whose holes are compact
        // (bolt circles and the like) — the radial collar is built for
        // those. Slots and keyways read badly under a collar+fan web, so
        // they stay with the fallback unless the user forces plate-web.
        TopoDS_Wire outer = BRepTools::OuterWire(face);
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
            if (wire.IsSame(outer)) continue;
            if (wireElongation(wire) > 2.2) return false;
        }
    }
    plan.loops = std::move(probe.loops);
    plan.uEdges = std::move(probe.uEdges);
    plan.constrains = true;
    plan.kind = MesherKind::PlateWeb;
    return true;
}

// Generalized minimal n-gon (game topology, plan §1/§4.1): ANY planar face
// can go minimal. A single-wire face becomes one boundary n-gon on the
// exact solved border; a holed face becomes a hole-bridged ear-clip web
// with ZERO interior vertices — the flattest topology that still welds.
bool planMinimalPlanar(const TopoDS_Face& face,
                       const BRepAdaptor_Surface& surf, const Model& model,
                       FacePlan& plan, bool requirePlane) {
    FacePlan probe;
    if (!collectPlanarLoops(face, surf, model, probe, requirePlane)) {
        return false;
    }
    plan.loops = std::move(probe.loops);
    plan.uEdges = std::move(probe.uEdges);
    plan.constrains = true;
    plan.kind = MesherKind::MinimalNGon;
    return true;
}

// --- Plate-web execution: 2D machinery -------------------------------------
// The plate is planar, so its UV space is an isometric chart — offsets and
// intersection tests run there and map straight back to 3D.


double loopSignedArea(const std::vector<WebPoint>& pts) {
    double a = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const gp_Pnt2d& p = pts[i].uv;
        const gp_Pnt2d& q = pts[(i + 1) % pts.size()].uv;
        a += p.X() * q.Y() - q.X() * p.Y();
    }
    return a / 2;
}

double webCross(const gp_Pnt2d& o, const gp_Pnt2d& a, const gp_Pnt2d& b) {
    return (a.X() - o.X()) * (b.Y() - o.Y()) -
           (a.Y() - o.Y()) * (b.X() - o.X());
}

// Proper segment intersection (shared endpoints don't count): used to keep
// hole-to-outer bridges from crossing any boundary edge.
bool webSegmentsCross(const gp_Pnt2d& a, const gp_Pnt2d& b, const gp_Pnt2d& c,
                      const gp_Pnt2d& d) {
    const double eps = 1e-12;
    auto near2 = [&](const gp_Pnt2d& p, const gp_Pnt2d& q) {
        return p.SquareDistance(q) < eps;
    };
    if (near2(a, c) || near2(a, d) || near2(b, c) || near2(b, d)) return false;
    double d1 = webCross(c, d, a), d2 = webCross(c, d, b);
    double d3 = webCross(a, b, c), d4 = webCross(a, b, d);
    return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) &&
           std::abs(d1 - d2) > eps && std::abs(d3 - d4) > eps;
}

// Ear clipping over a CCW polygon (may contain coincident bridge vertex
// pairs from hole merging — they share `vert`, so the doubled bridge edges
// cancel and the result stays watertight).
bool earClip(std::vector<WebPoint> poly, int faceId, bool flip,
             MeshBuilder& out) {
    const size_t n = poly.size();
    if (n < 3) return false;
    // Scale-free epsilon for convexity/containment decisions.
    double span = 0;
    for (const WebPoint& p : poly) {
        span = std::max({span, std::abs(p.uv.X()), std::abs(p.uv.Y())});
    }
    const double eps = 1e-12 * std::max(1.0, span * span);

    std::vector<size_t> idx(n);
    std::iota(idx.begin(), idx.end(), 0);
    auto insideTri = [&](const gp_Pnt2d& a, const gp_Pnt2d& b,
                         const gp_Pnt2d& c, const gp_Pnt2d& p) {
        return webCross(a, b, p) > eps && webCross(b, c, p) > eps &&
               webCross(c, a, p) > eps;
    };
    // Shape quality: normalized so an equilateral triangle scores 1 and
    // slivers approach 0. Clipping the BEST valid ear each round (instead
    // of the first found) keeps fans from piling onto one vertex.
    auto quality = [](const gp_Pnt2d& a, const gp_Pnt2d& b,
                      const gp_Pnt2d& c) {
        double area = std::abs((b.X() - a.X()) * (c.Y() - a.Y()) -
                               (c.X() - a.X()) * (b.Y() - a.Y())) / 2;
        double s = a.SquareDistance(b) + b.SquareDistance(c) +
                   c.SquareDistance(a);
        return s > 1e-300 ? 4.0 * std::sqrt(3.0) * area / s : 0.0;
    };
    size_t guard = 3 * n * n + 16;
    while (idx.size() > 3 && guard-- > 0) {
        bool clipped = false;
        size_t bestK = idx.size();
        double bestQ = -1.0;
        for (size_t k = 0; k < idx.size(); ++k) {
            size_t ip = idx[(k + idx.size() - 1) % idx.size()];
            size_t ic = idx[k];
            size_t in = idx[(k + 1) % idx.size()];
            const gp_Pnt2d &a = poly[ip].uv, &b = poly[ic].uv,
                           &c = poly[in].uv;
            if (webCross(a, b, c) <= eps) continue;  // reflex or collinear
            double q = quality(a, b, c);
            if (q <= bestQ) continue;  // can't beat the current best
            bool blocked = false;
            for (size_t other : idx) {
                if (other == ip || other == ic || other == in) continue;
                const gp_Pnt2d& p = poly[other].uv;
                // Coincident duplicates (bridge twins) never block an ear.
                if (p.SquareDistance(a) < eps || p.SquareDistance(b) < eps ||
                    p.SquareDistance(c) < eps) {
                    continue;
                }
                if (insideTri(a, b, c, p)) {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
            bestK = k;
            bestQ = q;
        }
        if (bestK < idx.size()) {
            size_t ip = idx[(bestK + idx.size() - 1) % idx.size()];
            size_t ic = idx[bestK];
            size_t in = idx[(bestK + 1) % idx.size()];
            out.addPolygon({poly[ip].vert, poly[ic].vert, poly[in].vert},
                           faceId, flip);
            idx.erase(idx.begin() + bestK);
            clipped = true;
        }
        if (!clipped) {
            // Numerical dead end: the old fan-close swept folded
            // triangles across hole regions. Fail honestly — the caller
            // demotes the face and the contract floor (or OCCT) takes
            // over with the borders intact.
            return false;
        }
    }
    if (idx.size() == 3) {
        out.addPolygon({poly[idx[0]].vert, poly[idx[1]].vert,
                        poly[idx[2]].vert},
                       faceId, flip);
    }
    return true;
}

// Merge hole rings into the outer ring via non-crossing bridges (doubled
// bridge vertices), rightmost holes first, then ear-clip the result.
// Merge every hole ring into the outer ring with non-crossing bridges
// (doubled bridge verts share ids, so the bridge edges cancel pairwise
// and the result stays watertight). Returns one simple "keyhole" ring.
std::vector<WebPoint> mergeHolesIntoRing(
    std::vector<WebPoint> outer, std::vector<std::vector<WebPoint>> holes,
    int faceId, bool flip, MeshBuilder& out) {
    auto maxX = [](const std::vector<WebPoint>& ring) {
        size_t best = 0;
        for (size_t i = 1; i < ring.size(); ++i) {
            if (ring[i].uv.X() > ring[best].uv.X()) best = i;
        }
        return best;
    };
    std::sort(holes.begin(), holes.end(),
              [&](const std::vector<WebPoint>& a,
                  const std::vector<WebPoint>& b) {
                  return a[maxX(a)].uv.X() > b[maxX(b)].uv.X();
              });

    for (size_t h = 0; h < holes.size(); ++h) {
        const std::vector<WebPoint>& hole = holes[h];
        const size_t m = maxX(hole);
        const gp_Pnt2d& M = hole[m].uv;
        // Candidate bridge target: nearest outer vertex whose connecting
        // segment crosses no boundary (outer so far, this hole, or any
        // hole still waiting to merge). Non-crossing => inside the domain.
        auto crossesAny = [&](const gp_Pnt2d& from, const gp_Pnt2d& to) {
            auto crossesRing = [&](const std::vector<WebPoint>& ring) {
                for (size_t i = 0; i < ring.size(); ++i) {
                    if (webSegmentsCross(from, to, ring[i].uv,
                                         ring[(i + 1) % ring.size()].uv)) {
                        return true;
                    }
                }
                return false;
            };
            if (crossesRing(outer) || crossesRing(hole)) return true;
            for (size_t j = h + 1; j < holes.size(); ++j) {
                if (crossesRing(holes[j])) return true;
            }
            return false;
        };
        size_t bestP = outer.size();
        double bestD = 1e300;
        for (size_t p = 0; p < outer.size(); ++p) {
            double d = M.SquareDistance(outer[p].uv);
            if (d >= bestD) continue;
            if (crossesAny(M, outer[p].uv)) continue;
            bestD = d;
            bestP = p;
        }
        if (bestP == outer.size()) {
            // No visible vertex (pathological): the old fan sealed the
            // hole with a membrane, silently covering a real opening.
            // Return empty so the caller fails the face instead.
            return {};
        }
        // Splice: ...P, M, M+1, ..., M-1, M, P, ... — P and M appear twice
        // sharing their vertex ids, so the bridge edges cancel pairwise.
        std::vector<WebPoint> merged;
        merged.reserve(outer.size() + hole.size() + 2);
        merged.insert(merged.end(), outer.begin(),
                      outer.begin() + bestP + 1);
        for (size_t k = 0; k <= hole.size(); ++k) {
            merged.push_back(hole[(m + k) % hole.size()]);
        }
        merged.insert(merged.end(), outer.begin() + bestP, outer.end());
        outer = std::move(merged);
    }
    return outer;
}

// Web triangulation via a real CDT: build a Z=0 planar face whose wires
// are the region's UV segments, let OCCT mesh it (a plane needs no
// interior refinement and straight edges never split), and harvest the
// triangles. Strict validation — every node must land exactly on an
// input ring vertex and the triangulated area must match the region —
// rejects anything suspicious back to the ear-clip path. Unlike ear
// clipping of a keyhole-merged ring, a CDT cannot fold, so the dead-end
// fan that used to sweep across complex plates is gone where this runs.
bool delaunayWeb(const std::vector<WebPoint>& outer,
                 const std::vector<std::vector<WebPoint>>& holes,
                 int faceId, bool flip, MeshBuilder& out) {
    if (outer.size() < 3) return false;
    double span = 0, regionArea = loopSignedArea(outer);
    if (regionArea <= 0) return false;  // outer must be CCW
    for (const std::vector<WebPoint>& h : holes) {
        if (h.size() < 3) return false;
        double a = loopSignedArea(h);
        if (a >= 0) return false;  // holes must be CW
        regionArea += a;
    }
    if (regionArea <= 0) return false;
    for (const WebPoint& p : outer) {
        span = std::max({span, std::abs(p.uv.X()), std::abs(p.uv.Y())});
    }
    const double tol = 1e-9 * std::max(1.0, span);

    // Ring vertex lookup by quantized UV. Ambiguous keys (two ring points
    // sharing a position but not a vertex) cannot be mapped back safely.
    std::map<std::pair<int64_t, int64_t>, uint32_t> vertByUv;
    auto keyOf = [&](double x, double y) {
        return std::make_pair(int64_t(std::llround(x / tol)),
                              int64_t(std::llround(y / tol)));
    };
    auto addRing = [&](const std::vector<WebPoint>& ring) {
        for (size_t i = 0; i < ring.size(); ++i) {
            if (ring[i].uv.SquareDistance(
                    ring[(i + 1) % ring.size()].uv) < tol * tol) {
                return false;  // zero-length segment: MakePolygon drops it
            }
            auto [it, fresh] = vertByUv.try_emplace(
                keyOf(ring[i].uv.X(), ring[i].uv.Y()), ring[i].vert);
            if (!fresh && it->second != ring[i].vert) return false;
        }
        return true;
    };
    if (!addRing(outer)) return false;
    for (const std::vector<WebPoint>& h : holes) {
        if (!addRing(h)) return false;
    }

    try {
        auto makeWire = [&](const std::vector<WebPoint>& ring,
                            TopoDS_Wire& wire) {
            BRepBuilderAPI_MakePolygon mp;
            for (const WebPoint& p : ring) {
                mp.Add(gp_Pnt(p.uv.X(), p.uv.Y(), 0.0));
            }
            mp.Close();
            if (!mp.IsDone()) return false;
            wire = mp.Wire();
            return true;
        };
        TopoDS_Wire ow;
        if (!makeWire(outer, ow)) return false;
        BRepBuilderAPI_MakeFace mf(gp_Pln(), ow, true);
        for (const std::vector<WebPoint>& h : holes) {
            TopoDS_Wire hw;
            if (!makeWire(h, hw)) return false;
            mf.Add(hw);
        }
        if (!mf.IsDone()) return false;
        TopoDS_Face f = mf.Face();
        // Huge deflection: straight edges never split, and a plane never
        // needs interior refinement, so the nodes are exactly our points.
        IMeshTools_Parameters mp;
        mp.Deflection = 1e9;
        mp.Angle = 1.0;
        mp.InParallel = false;
        BRepMesh_IncrementalMesh mesher(f, mp);
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(f, loc);
        if (tri.IsNull()) return false;

        std::vector<uint32_t> nodeVert(tri->NbNodes() + 1, UINT32_MAX);
        for (int n = 1; n <= tri->NbNodes(); ++n) {
            gp_Pnt p = tri->Node(n);
            auto it = vertByUv.find(keyOf(p.X(), p.Y()));
            if (it == vertByUv.end()) return false;  // Steiner/split node
            nodeVert[n] = it->second;
        }
        // Collect with winding + coverage validation before emitting.
        std::vector<std::array<uint32_t, 3>> tris;
        auto nodeUv = [&](int n) {
            gp_Pnt p = tri->Node(n);
            return gp_Pnt2d(p.X(), p.Y());
        };
        double covered = 0;
        for (int t = 1; t <= tri->NbTriangles(); ++t) {
            int n1, n2, n3;
            tri->Triangle(t).Get(n1, n2, n3);
            double a2 = webCross(nodeUv(n1), nodeUv(n2), nodeUv(n3));
            if (a2 < 0) {
                std::swap(n2, n3);
                a2 = -a2;
            }
            covered += a2 / 2;
            tris.push_back({nodeVert[n1], nodeVert[n2], nodeVert[n3]});
        }
        if (std::abs(covered - regionArea) > 0.005 * regionArea) {
            return false;  // covered a hole or leaked past the boundary
        }
        for (const auto& t : tris) {
            out.addPolygon({t[0], t[1], t[2]}, faceId, flip);
        }
        return true;
    } catch (const Standard_Failure&) {
        return false;
    }
}

bool triangulateWeb(std::vector<WebPoint> outer,
                    std::vector<std::vector<WebPoint>> holes, int faceId,
                    bool flip, MeshBuilder& out) {
    if (delaunayWeb(outer, holes, faceId, flip, out)) return true;
    std::vector<WebPoint> ring = mergeHolesIntoRing(
        std::move(outer), std::move(holes), faceId, flip, out);
    if (ring.size() < 3) return false;
    return earClip(std::move(ring), faceId, flip, out);
}



// A planar face's wire sampled as one chained ring: each edge at its own
// solved count, positions on the 3D edge curve (weld-exact), UV from the
// pcurve (the plate's isometric chart).

double planarRingArea(const PlanarRing& r) {
    double a = 0;
    for (size_t i = 0; i < r.uv.size(); ++i) {
        const gp_Pnt2d& p = r.uv[i];
        const gp_Pnt2d& q = r.uv[(i + 1) % r.uv.size()];
        a += p.X() * q.Y() - q.X() * p.Y();
    }
    return a / 2;
}

// Sample every wire, then normalize the winding in UV: outer CCW, holes
// CW — triangulation then emits CCW in UV, and one global flip against
// the face orientation fixes 3D winding.
bool samplePlanarRings(const TopoDS_Face& face, const Model& model,
                       const std::vector<int>& solvedEdge, int radialDefault,
                       std::vector<PlanarRing>& rings,
                       const PinnedEdges* pins) {
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
        PlanarRing ring;
        ring.isOuter = wire.IsSame(outerWire);
        int wireEdges = 0;
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            ++wireEdges;
        }
        // Sloppy wires defeat BRepTools_WireExplorer (it silently DROPS
        // edges it cannot chain within tolerance) — every dropped edge
        // is a missing border. Detect the drop and assemble the ring by
        // hand: sample each edge, then chain pieces by nearest
        // endpoints, exactly like the insert webs do.
        int rawEdges = 0;
        for (TopoDS_Iterator it(wire); it.More(); it.Next()) {
            if (it.Value().ShapeType() == TopAbs_EDGE &&
                !BRep_Tool::Degenerated(TopoDS::Edge(it.Value()))) {
                ++rawEdges;
            }
        }
        if (rawEdges > wireEdges) {
            struct Piece {
                std::vector<gp_Pnt2d> uv;
                std::vector<gp_Pnt> p;
            };
            std::vector<Piece> pieces;
            for (TopoDS_Iterator it(wire); it.More(); it.Next()) {
                if (it.Value().ShapeType() != TopAbs_EDGE) continue;
                const TopoDS_Edge edge = TopoDS::Edge(it.Value());
                if (BRep_Tool::Degenerated(edge)) continue;
                int eid = model.edges.FindIndex(edge);
                int n = (eid >= 1 && eid < int(solvedEdge.size()))
                            ? solvedEdge[eid]
                            : 0;
                if (n < 1) {
                    n = std::max(1, std::max(3, radialDefault) /
                                        std::max(1, rawEdges));
                }
                double f3, l3, f2, l2;
                Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
                Handle(Geom2d_Curve) c2 =
                    BRep_Tool::CurveOnSurface(edge, face, f2, l2);
                if (c3.IsNull() || c2.IsNull()) {
                    dbg("planar rings: edge %d missing %s%s", eid,
                        c3.IsNull() ? "3D curve" : "",
                        c2.IsNull() ? " pcurve" : "");
                    return false;
                }
                const bool rev = edge.Orientation() == TopAbs_REVERSED;
                const double ph = closedEdgePhase(edge, model);
                Piece pc;
                for (double t : edgeSampleFractions(eid, n, ph, rev,
                                                    /*includeLast=*/true,
                                                    pins, &model)) {
                    pc.uv.push_back(c2->Value(f2 + (l2 - f2) * t));
                    pc.p.push_back(c3->Value(f3 + (l3 - f3) * t));
                }
                pieces.push_back(std::move(pc));
            }
            if (pieces.empty()) return false;
            Piece chain = std::move(pieces[0]);
            std::vector<char> used(pieces.size(), 1);
            used[0] = 1;
            for (size_t k = 1; k < pieces.size(); ++k) used[k] = 0;
            for (size_t step = 1; step < pieces.size(); ++step) {
                double bd = 1e300;
                size_t bi = 0;
                bool rev2 = false;
                for (size_t k = 0; k < pieces.size(); ++k) {
                    if (used[k]) continue;
                    double dF = chain.p.back().Distance(pieces[k].p.front());
                    double dB = chain.p.back().Distance(pieces[k].p.back());
                    if (dF < bd) { bd = dF; bi = k; rev2 = false; }
                    if (dB < bd) { bd = dB; bi = k; rev2 = true; }
                }
                used[bi] = 1;
                Piece pc = std::move(pieces[bi]);
                if (rev2) {
                    std::reverse(pc.uv.begin(), pc.uv.end());
                    std::reverse(pc.p.begin(), pc.p.end());
                }
                chain.uv.insert(chain.uv.end(), pc.uv.begin() + 1,
                                pc.uv.end());
                chain.p.insert(chain.p.end(), pc.p.begin() + 1, pc.p.end());
            }
            // Drop the closing duplicate.
            if (chain.p.size() > 1 &&
                chain.p.front().Distance(chain.p.back()) <
                    1e-6 + BRep_Tool::Tolerance(face)) {
                chain.uv.pop_back();
                chain.p.pop_back();
            }
            ring.uv = std::move(chain.uv);
            ring.p = std::move(chain.p);
            if (ring.uv.size() < 3) {
                dbg("planar rings: hand-chained ring only %zu verts",
                    ring.uv.size());
                return false;
            }
            rings.push_back(std::move(ring));
            continue;
        }
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            // Degenerate edges (pole collapses) carry no border contract
            // — skip them rather than refusing the whole face (freeform
            // pocket walls often carry one, and refusing sent those
            // faces to raw OCCT triangulation).
            if (BRep_Tool::Degenerated(edge)) continue;
            int eid = model.edges.FindIndex(edge);
            int n = (eid >= 1 && eid < int(solvedEdge.size()))
                        ? solvedEdge[eid]
                        : 0;
            if (n < 1) {
                n = std::max(1, std::max(3, radialDefault) /
                                    std::max(1, wireEdges));
            }
            double f3, l3, f2, l2;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
            Handle(Geom2d_Curve) c2 =
                BRep_Tool::CurveOnSurface(edge, face, f2, l2);
            if (c3.IsNull() || c2.IsNull()) {
                dbg("planar rings: edge %d missing %s%s", eid,
                    c3.IsNull() ? "3D curve" : "",
                    c2.IsNull() ? " pcurve" : "");
                return false;
            }
            const bool rev = edge.Orientation() == TopAbs_REVERSED;
            const double ph = closedEdgePhase(edge, model);
            // endpoint owned by the next edge
            for (double t : edgeSampleFractions(eid, n, ph, rev,
                                                /*includeLast=*/false, pins,
                                                &model)) {
                ring.uv.push_back(c2->Value(f2 + (l2 - f2) * t));
                ring.p.push_back(c3->Value(f3 + (l3 - f3) * t));
            }
        }
        if (ring.uv.size() < 3) {
            dbg("planar rings: wire ring only %zu verts", ring.uv.size());
            for (BRepTools_WireExplorer we(wire, face); we.More();
                 we.Next()) {
                const int eid = model.edges.FindIndex(we.Current());
                const int solved =
                    eid >= 1 && eid < int(solvedEdge.size())
                        ? solvedEdge[eid]
                        : 0;
                const size_t pinCount =
                    edgeIsPinned(eid, pins) ? (*pins)[eid].size() : 0;
                dbg("planar rings: edge %d solved=%d pin-points=%zu", eid,
                    solved, pinCount);
            }
            return false;
        }
        rings.push_back(std::move(ring));
    }
    for (PlanarRing& r : rings) {
        double a = planarRingArea(r);
        if (std::abs(a) < 1e-14) {
            dbg("planar rings: ring area %.3g degenerate (%zu verts)", a,
                r.uv.size());
            return false;
        }
        if (r.isOuter != (a > 0)) {
            std::reverse(r.uv.begin(), r.uv.end());
            std::reverse(r.p.begin(), r.p.end());
        }
    }
    return true;
}

bool meshPlateWeb(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, int faceId,
                  const std::vector<int>& solvedEdge, int radialDefault,
                  int collarRings, bool squareCollar, MeshBuilder& out) {
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
        return false;
    }
    if (rings.size() < 2) return false;

    // Boundary vertices (anchorless: they live on shared B-rep edges).
    std::vector<std::vector<uint32_t>> ringVerts(rings.size());
    for (size_t r = 0; r < rings.size(); ++r) {
        for (const gp_Pnt& p : rings[r].p) {
            ringVerts[r].push_back(out.addVertex(p, {}));
        }
    }

    // A UV-CCW polygon's 3D normal equals du×dv on a plane chart, so the
    // face orientation alone decides the global flip.
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    // Even-odd containment against the sampled rings: cheap, and exactly
    // consistent with the polygon domain the web is triangulated over
    // (the true-face classifier costs ~1ms per probe on plates this size).
    auto insideDomain = [&](const gp_Pnt2d& p) {
        int crossings = 0;
        for (const PlanarRing& r : rings) {
            for (size_t i = 0; i < r.uv.size(); ++i) {
                const gp_Pnt2d& a = r.uv[i];
                const gp_Pnt2d& b = r.uv[(i + 1) % r.uv.size()];
                if ((a.Y() > p.Y()) == (b.Y() > p.Y())) continue;
                double x = a.X() + (p.Y() - a.Y()) / (b.Y() - a.Y()) *
                                       (b.X() - a.X());
                if (x > p.X()) ++crossings;
            }
        }
        return (crossings & 1) != 0;
    };

    // Quad collar around every hole: radial offset in UV away from the
    // hole's centroid, clamped so it can't reach any other loop, dropped
    // if the offset points leave the face or the ring degenerates.
    const size_t outerIdx = [&] {
        for (size_t r = 0; r < rings.size(); ++r) {
            if (rings[r].isOuter) return r;
        }
        return size_t(0);
    }();
    std::vector<WebPoint> webOuter;
    for (size_t i = 0; i < rings[outerIdx].uv.size(); ++i) {
        webOuter.push_back({rings[outerIdx].uv[i], ringVerts[outerIdx][i]});
    }
    std::vector<std::vector<WebPoint>> webHoles;
    for (size_t r = 0; r < rings.size(); ++r) {
        if (r == outerIdx) continue;
        const PlanarRing& hole = rings[r];
        const size_t n = hole.uv.size();
        gp_XY centroid(0, 0);
        double perimeter = 0;
        for (size_t i = 0; i < n; ++i) {
            centroid += hole.uv[i].XY();
            perimeter += hole.uv[i].Distance(hole.uv[(i + 1) % n]);
        }
        centroid /= double(n);
        double d = 1.2 * perimeter / double(n);
        // Clearance to every other loop's vertices caps the collar depth.
        double clearance = 1e300;
        for (size_t o = 0; o < rings.size(); ++o) {
            if (o == r) continue;
            for (const gp_Pnt2d& q : rings[o].uv) {
                for (const gp_Pnt2d& p : hole.uv) {
                    clearance = std::min(clearance, p.Distance(q));
                }
            }
        }
        // Several concentric rings ("junction rings") share the clearance
        // budget: an even radial fan around the hole instead of one thin
        // band + a long web reach.
        const int wantRings = std::max(1, collarRings);
        double dStep =
            std::min(d, 0.35 * clearance / double(wantRings));

        std::vector<WebPoint> boundary;  // what the web sees for this hole
        for (size_t i = 0; i < n; ++i) {
            boundary.push_back({hole.uv[i], ringVerts[r][i]});
        }
        const double holeA = planarRingArea(hole);

        // Square collars collapse the ring to exactly FOUR corner verts:
        // every hole vertex fans into its quadrant's corner, transitions
        // become quads, and the web onward sees a clean 4-gon — the
        // classic game pattern for a round hole in a plate.
        if (squareCollar && n >= 8) {
            double off = dStep * wantRings;
            bool built = false;
            while (off > 1e-9 * (1.0 + perimeter) && !built) {
                double bx0 = 1e300, bx1 = -1e300, by0 = 1e300, by1 = -1e300;
                for (const gp_Pnt2d& p : hole.uv) {
                    bx0 = std::min(bx0, p.X());
                    bx1 = std::max(bx1, p.X());
                    by0 = std::min(by0, p.Y());
                    by1 = std::max(by1, p.Y());
                }
                bx0 -= off; bx1 += off;
                by0 -= off; by1 += off;
                // CW to match the hole's winding.
                const gp_Pnt2d corner[4] = {{bx0, by0}, {bx0, by1},
                                            {bx1, by1}, {bx1, by0}};
                bool ok = true;
                for (int k = 0; k < 4 && ok; ++k) {
                    ok = insideDomain(corner[k]);
                }
                // Quadrant of every hole vertex (nearest corner by angle);
                // must step by at most one corner between neighbours.
                std::vector<int> sect(n);
                if (ok) {
                    for (size_t i = 0; i < n; ++i) {
                        double best = -1e300;
                        for (int k = 0; k < 4; ++k) {
                            gp_XY a = hole.uv[i].XY() - centroid;
                            gp_XY b = corner[k].XY() - centroid;
                            double dot =
                                (a * b) / std::max(1e-12, a.Modulus() *
                                                              b.Modulus());
                            if (dot > best) {
                                best = dot;
                                sect[i] = k;
                            }
                        }
                    }
                    for (size_t i = 0; i < n && ok; ++i) {
                        int a = sect[i], b = sect[(i + 1) % n];
                        int step = ((b - a) % 4 + 4) % 4;
                        if (step > 1) ok = false;  // empty quadrant
                    }
                }
                if (!ok) {
                    off /= 2;
                    continue;
                }
                std::array<uint32_t, 4> cv;
                std::array<WebPoint, 4> cw;
                for (int k = 0; k < 4; ++k) {
                    gp_Pnt cp = surf.Value(corner[k].X(), corner[k].Y());
                    cv[k] = out.addVertex(cp, {faceId, corner[k].X(),
                                               corner[k].Y()});
                    cw[k] = {corner[k], cv[k]};
                }
                for (size_t i = 0; i < n; ++i) {
                    size_t j = (i + 1) % n;
                    if (sect[i] == sect[j]) {
                        out.addPolygon({ringVerts[r][i], ringVerts[r][j],
                                        cv[sect[i]]},
                                       faceId, flip);
                    } else {  // quadrant transition: one quad
                        out.addPolygon({ringVerts[r][i], ringVerts[r][j],
                                        cv[sect[j]], cv[sect[i]]},
                                       faceId, flip);
                    }
                }
                boundary.assign(cw.begin(), cw.end());
                built = true;
            }
            if (built) {
                webHoles.push_back(std::move(boundary));
                continue;
            }
            // No room for the square: fall through to the radial rings.
        }
        while (dStep > 1e-9 * (1.0 + perimeter)) {
            // Grow ring by ring; stop at the first one that leaves the
            // face or degenerates (keeping what fit so far).
            std::vector<WebPoint> prev = boundary;
            double prevAbsA = std::abs(holeA);
            int built = 0;
            for (int ring = 1; ring <= wantRings; ++ring) {
                std::vector<gp_Pnt2d> collar(n);
                bool ok = true;
                double off = dStep * ring;
                // Square borders: the ring lies on the hole's expanded
                // bounding rectangle, each vertex placed where its ray
                // from the centroid meets the rectangle.
                double bx0 = 1e300, bx1 = -1e300, by0 = 1e300,
                       by1 = -1e300;
                if (squareCollar) {
                    for (const gp_Pnt2d& p : hole.uv) {
                        bx0 = std::min(bx0, p.X());
                        bx1 = std::max(bx1, p.X());
                        by0 = std::min(by0, p.Y());
                        by1 = std::max(by1, p.Y());
                    }
                    bx0 -= off; bx1 += off;
                    by0 -= off; by1 += off;
                }
                for (size_t i = 0; i < n && ok; ++i) {
                    gp_XY dir = hole.uv[i].XY() - centroid;
                    double len = dir.Modulus();
                    if (len < 1e-12) { ok = false; break; }
                    if (squareCollar) {
                        double t = 1e300;
                        if (dir.X() > 1e-12)
                            t = std::min(t, (bx1 - centroid.X()) / dir.X());
                        if (dir.X() < -1e-12)
                            t = std::min(t, (bx0 - centroid.X()) / dir.X());
                        if (dir.Y() > 1e-12)
                            t = std::min(t, (by1 - centroid.Y()) / dir.Y());
                        if (dir.Y() < -1e-12)
                            t = std::min(t, (by0 - centroid.Y()) / dir.Y());
                        if (t > 1e200) { ok = false; break; }
                        collar[i] = gp_Pnt2d(centroid + dir * t);
                    } else {
                        collar[i] =
                            gp_Pnt2d(hole.uv[i].XY() + dir * (off / len));
                    }
                    if (!insideDomain(collar[i])) ok = false;
                }
                if (ok) {
                    // Same orientation as the hole and strictly growing.
                    double collarA = 0;
                    for (size_t i = 0; i < n; ++i) {
                        const gp_Pnt2d& p = collar[i];
                        const gp_Pnt2d& q = collar[(i + 1) % n];
                        collarA += p.X() * q.Y() - q.X() * p.Y();
                    }
                    collarA /= 2;
                    ok = (collarA < 0) == (holeA < 0) &&
                         std::abs(collarA) > prevAbsA;
                    if (ok) prevAbsA = std::abs(collarA);
                }
                if (!ok) break;
                std::vector<WebPoint> collarPts(n);
                for (size_t i = 0; i < n; ++i) {
                    gp_Pnt cp = surf.Value(collar[i].X(), collar[i].Y());
                    collarPts[i] = {collar[i],
                                    out.addVertex(cp, {faceId, collar[i].X(),
                                                       collar[i].Y()})};
                }
                for (size_t i = 0; i < n; ++i) {
                    size_t j = (i + 1) % n;
                    out.addPolygon({prev[i].vert, prev[j].vert,
                                    collarPts[j].vert, collarPts[i].vert},
                                   faceId, flip);
                }
                prev = std::move(collarPts);
                ++built;
            }
            if (built > 0) {
                boundary = std::move(prev);
                break;
            }
            dStep /= 2;  // even the first ring didn't fit: pull in, retry
        }
        webHoles.push_back(std::move(boundary));
    }

    return triangulateWeb(std::move(webOuter), std::move(webHoles), faceId,
                          flip, out);
}

// Generalized minimal n-gon: the flattest topology a planar face can
// carry. One wire -> a single boundary n-gon on the exact solved border;
// holes -> the hole-bridged ear-clip web with zero interior vertices.
// Split a holed panel into SIMPLE n-gons: two non-crossing bridges per
// hole become real shared edges dividing the region, so every emitted
// polygon is simple (no doubled keyhole edges). Keyhole rings are legal
// topology but no importer triangulates them reliably — they render and
// export as membranes sealing the holes. Returns false when a hole
// cannot see two distinct targets (caller falls back).
bool splitIntoSimplePolys(std::vector<WebPoint> outer,
                          std::vector<std::vector<WebPoint>> holes,
                          std::vector<std::vector<WebPoint>>& polysOut) {
    auto maxX = [](const std::vector<WebPoint>& ring) {
        size_t best = 0;
        for (size_t i = 1; i < ring.size(); ++i) {
            if (ring[i].uv.X() > ring[best].uv.X()) best = i;
        }
        return best;
    };
    std::sort(holes.begin(), holes.end(),
              [&](const std::vector<WebPoint>& a,
                  const std::vector<WebPoint>& b) {
                  return a[maxX(a)].uv.X() > b[maxX(b)].uv.X();
              });
    auto inside = [](const std::vector<WebPoint>& ring, const gp_Pnt2d& p) {
        int c = 0;
        for (size_t i = 0; i < ring.size(); ++i) {
            const gp_Pnt2d& a = ring[i].uv;
            const gp_Pnt2d& b = ring[(i + 1) % ring.size()].uv;
            if ((a.Y() > p.Y()) == (b.Y() > p.Y())) continue;
            double x = a.X() +
                       (p.Y() - a.Y()) / (b.Y() - a.Y()) * (b.X() - a.X());
            if (x > p.X()) ++c;
        }
        return (c & 1) != 0;
    };
    auto signedArea = [](const std::vector<WebPoint>& ring) {
        double a = 0;
        for (size_t i = 0; i < ring.size(); ++i) {
            const gp_Pnt2d& p = ring[i].uv;
            const gp_Pnt2d& q = ring[(i + 1) % ring.size()].uv;
            a += p.X() * q.Y() - q.X() * p.Y();
        }
        return a / 2;
    };

    polysOut.clear();
    polysOut.push_back(std::move(outer));
    for (size_t h = 0; h < holes.size(); ++h) {
        const std::vector<WebPoint>& H = holes[h];
        // The (unique) current region that contains this hole.
        size_t ri = polysOut.size();
        for (size_t r = 0; r < polysOut.size(); ++r) {
            if (inside(polysOut[r], H[0].uv)) {
                ri = r;
                break;
            }
        }
        if (ri == polysOut.size()) return false;
        const std::vector<WebPoint>& R = polysOut[ri];
        auto crossesAny = [&](const gp_Pnt2d& a, const gp_Pnt2d& b,
                              const gp_Pnt2d* alsoA,
                              const gp_Pnt2d* alsoB) {
            auto crossesRing = [&](const std::vector<WebPoint>& ring) {
                for (size_t i = 0; i < ring.size(); ++i) {
                    if (webSegmentsCross(a, b, ring[i].uv,
                                         ring[(i + 1) % ring.size()].uv)) {
                        return true;
                    }
                }
                return false;
            };
            if (crossesRing(R) || crossesRing(H)) return true;
            for (size_t j = h + 1; j < holes.size(); ++j) {
                if (crossesRing(holes[j])) return true;
            }
            if (alsoA && webSegmentsCross(a, b, *alsoA, *alsoB)) return true;
            return false;
        };
        // Bridge 1 from the hole's rightmost vertex; bridge 2 from near
        // its antipode (scanning on from there if occluded).
        const size_t a1 = maxX(H);
        size_t b1 = R.size();
        double bd = 1e300;
        for (size_t p = 0; p < R.size(); ++p) {
            double d = H[a1].uv.SquareDistance(R[p].uv);
            if (d >= bd) continue;
            if (crossesAny(H[a1].uv, R[p].uv, nullptr, nullptr)) continue;
            bd = d;
            b1 = p;
        }
        if (b1 == R.size()) return false;
        size_t a2 = H.size(), b2 = R.size();
        for (size_t off = 0; off < H.size() && a2 == H.size(); ++off) {
            const size_t cand = (a1 + H.size() / 2 + off) % H.size();
            if (cand == a1) continue;
            double bd2 = 1e300;
            for (size_t p = 0; p < R.size(); ++p) {
                if (p == b1) continue;
                double d = H[cand].uv.SquareDistance(R[p].uv);
                if (d >= bd2) continue;
                if (crossesAny(H[cand].uv, R[p].uv, &H[a1].uv,
                               &R[b1].uv)) {
                    continue;
                }
                bd2 = d;
                b2 = p;
            }
            if (b2 != R.size()) a2 = cand;
        }
        if (a2 == H.size() || b2 == R.size()) return false;
        // Split: region boundary arcs stay in wire order (R is CCW, H is
        // CW as sampled), the bridges become the shared closing edges.
        auto walkR = [&](size_t from, size_t to) {
            std::vector<WebPoint> arc;
            for (size_t i = from;; i = (i + 1) % R.size()) {
                arc.push_back(R[i]);
                if (i == to) break;
            }
            return arc;
        };
        auto walkH = [&](size_t from, size_t to) {
            std::vector<WebPoint> arc;
            for (size_t i = from;; i = (i + 1) % H.size()) {
                arc.push_back(H[i]);
                if (i == to) break;
            }
            return arc;
        };
        std::vector<WebPoint> ring1 = walkR(b1, b2);
        {
            std::vector<WebPoint> harc = walkH(a2, a1);
            ring1.insert(ring1.end(), harc.begin(), harc.end());
        }
        std::vector<WebPoint> ring2 = walkR(b2, b1);
        {
            std::vector<WebPoint> harc = walkH(a1, a2);
            ring2.insert(ring2.end(), harc.begin(), harc.end());
        }
        if (ring1.size() < 3 || ring2.size() < 3) return false;
        if (signedArea(ring1) <= 0 || signedArea(ring2) <= 0) {
            return false;  // bad split (occlusion edge case): fall back
        }
        polysOut[ri] = std::move(ring1);
        polysOut.push_back(std::move(ring2));
    }
    return true;
}

bool meshMinimalPlanar(const TopoDS_Face& face, const Model& model,
                       int faceId, const std::vector<int>& solvedEdge,
                       int radialDefault, MeshBuilder& out,
                       const PinnedEdges* pins) {
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings,
                           pins)) {
        return false;
    }
    if (rings.empty()) return false;
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    if (rings.size() == 1) {
        std::vector<uint32_t> poly;
        for (const gp_Pnt& p : rings[0].p) poly.push_back(out.addVertex(p, {}));
        if (poly.size() < 3) return false;
        out.addPolygon(std::move(poly), faceId, flip);  // UV-CCW already
        return true;
    }

    std::vector<WebPoint> webOuter;
    std::vector<std::vector<WebPoint>> webHoles;
    for (const PlanarRing& r : rings) {
        std::vector<WebPoint> ring;
        for (size_t i = 0; i < r.uv.size(); ++i) {
            ring.push_back({r.uv[i], out.addVertex(r.p[i], {})});
        }
        if (r.isOuter) webOuter = std::move(ring);
        else webHoles.push_back(std::move(ring));
    }
    if (webOuter.size() < 3) return false;
    // Minimal means minimal AND simple: two real bridges per hole split
    // the panel into k+1 simple n-gons (shared edges, no doubled
    // keyhole slits — those render and import as hole membranes).
    std::vector<std::vector<WebPoint>> simple;
    if (splitIntoSimplePolys(webOuter, webHoles, simple)) {
        for (const auto& ring : simple) {
            std::vector<uint32_t> poly;
            poly.reserve(ring.size());
            for (const WebPoint& w : ring) poly.push_back(w.vert);
            out.addPolygon(std::move(poly), faceId, flip);
        }
        return true;
    }
    // Pathological visibility: keep the keyhole as a last resort.
    std::vector<WebPoint> ring = mergeHolesIntoRing(
        std::move(webOuter), std::move(webHoles), faceId, flip, out);
    std::vector<uint32_t> poly;
    poly.reserve(ring.size());
    for (const WebPoint& w : ring) poly.push_back(w.vert);
    if (poly.size() < 3) return false;
    out.addPolygon(std::move(poly), faceId, flip);
    return true;
}


}  // namespace weft::mesher_impl
