// Decoupled mesher (rewrite/decoupled-core). See core/include/weft/decoupled.hpp
// for the architecture and core/spike/{seam_bridge,decoupled_face}.cpp for the
// de-risk that proves the loop bridge is watertight/manifold/fold-free.
//
// The whole point: faces couple ONLY through per-edge sample counts. This file
//   (1) assigns every B-rep edge a count (a pure input: per-edge pin, per-face
//       radial/axial proposal, or a geometry default) and samples it ONCE into
//       a shared array — both incident faces read the same 3D points, so they
//       weld with no global density solve and no ripple;
//   (2) meshes each face's interior at its OWN count (revolution grid / planar
//       n-gon / boundary floor) and, where the interior count differs from a
//       border count, absorbs the gap with the loop bridge;
//   (3) concatenates the per-face parts and welds per solid.
//
// Increment 1 handles the analytic backbone (full cylinders/cones as quad
// grids, planar faces as boundary n-gons) with a boundary-triangulation floor
// for everything else; later increments plug in UV-coons, poles, and holes.

#include "weft/decoupled.hpp"

#include "weft/mesh.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pnt2d.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace weft {
namespace {

using P3 = std::array<double, 3>;

P3 toP3(const gp_Pnt& p) { return {p.X(), p.Y(), p.Z()}; }
P3 sub(const P3& a, const P3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
P3 add(const P3& a, const P3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
P3 mul(const P3& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
P3 cross(const P3& a, const P3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}
double dot(const P3& a, const P3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
double len(const P3& a) { return std::sqrt(dot(a, a)); }
P3 lerp(const P3& a, const P3& b, double t) { return add(mul(a, 1 - t), mul(b, t)); }

// Newell normal of a polygon (indices into verts). Robust for n-gons.
P3 newell(const std::vector<P3>& verts, const std::vector<uint32_t>& poly) {
    P3 n{0, 0, 0};
    for (size_t k = 0; k < poly.size(); ++k) {
        const P3& a = verts[poly[k]];
        const P3& b = verts[poly[(k + 1) % poly.size()]];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    return n;
}

// ---- world-anchored phase for closed circles ------------------------------
// A full circle has no natural sample start; anchor every coaxial circle's
// first sample to one fixed world direction projected into its plane, so rings
// stacked on the same axis share column angles (mirrors meshers.cpp's
// ringAnchorAngle). Returns an angle in the circle's own (X,Y) frame.
double ringAnchorAngle(const gp_Circ& circ) {
    const gp_Dir d = circ.Axis().Direction();
    gp_XYZ g(0.7548776662466927, 0.5698402909980532, 0.3247179572447461);
    gp_XYZ pr = g - d.XYZ() * g.Dot(d.XYZ());
    if (pr.SquareModulus() < 1e-18) pr = gp_XYZ(1, 0, 0) - d.XYZ() * d.X();
    const double x = pr.Dot(circ.Position().XDirection().XYZ());
    const double y = pr.Dot(circ.Position().YDirection().XYZ());
    double a = std::atan2(y, x);
    if (a < 0) a += 2.0 * M_PI;
    return a;
}

// ---- edge sampling (the border contract) ----------------------------------
struct EdgeSamples {
    std::vector<P3> pts;        // forward order (param f->l)
    std::vector<double> param;  // the edge parameter at each pt (for pcurve UV)
    bool closed = false;        // full loop by itself (a circle): pts is a ring
    bool valid = false;
};

// Sample edge `eid`'s own 3D curve at `n` segments. A full circle returns n
// ring points (no duplicate endpoint); an open edge returns n+1 points
// including both endpoints. Freeform curves are spaced by arc length; lines and
// arcs by uniform parameter (already even). Closed circles are phase-anchored.
EdgeSamples sampleEdge(const Model& model, int eid, int n) {
    EdgeSamples s;
    if (eid < 1 || eid > model.edgeCount() || n < 1) return s;
    const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
    if (BRep_Tool::Degenerated(e)) return s;
    double f = 0, l = 0;
    Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
    if (c3.IsNull()) return s;
    const double span = l - f;
    if (span <= 1e-12) return s;
    GeomAdaptor_Curve gac(c3, f, l);
    const GeomAbs_CurveType ct = gac.GetType();
    const bool closedGeom = c3->Value(f).Distance(c3->Value(l)) < 1e-7;

    if (ct == GeomAbs_Circle && closedGeom) {
        // Full circle: n phase-anchored ring points. Circle parameter == angle
        // in its own frame, so param = anchor + k/n * 2pi.
        const gp_Circ circ = gac.Circle();
        const double a0 = ringAnchorAngle(circ);
        s.pts.reserve(n);
        s.param.reserve(n);
        for (int k = 0; k < n; ++k) {
            const double t = a0 + 2.0 * M_PI * k / n;
            s.pts.push_back(toP3(c3->Value(t)));
            s.param.push_back(t);
        }
        s.closed = true;
        s.valid = true;
        return s;
    }

    std::vector<double> frac;
    frac.reserve(n + 1);
    if (ct == GeomAbs_Line || ct == GeomAbs_Circle) {
        for (int i = 0; i <= n; ++i) frac.push_back(double(i) / n);
    } else {
        GCPnts_UniformAbscissa algo(gac, n + 1);
        if (algo.IsDone() && algo.NbPoints() == n + 1) {
            for (int i = 1; i <= n + 1; ++i)
                frac.push_back(std::clamp((algo.Parameter(i) - f) / span, 0.0, 1.0));
            frac.front() = 0.0;
            frac.back() = 1.0;
        } else {
            for (int i = 0; i <= n; ++i) frac.push_back(double(i) / n);
        }
    }
    s.pts.reserve(frac.size());
    s.param.reserve(frac.size());
    for (double t : frac) {
        s.pts.push_back(toP3(c3->Value(f + span * t)));
        s.param.push_back(f + span * t);
    }
    s.closed = false;
    s.valid = true;
    return s;
}

// ---- per-edge count (pure inputs) -----------------------------------------
// A single forward pass: geometry sets a default, revolution faces propose
// radial on their rims and axial on their side edges, per-edge pins win.
struct EdgeCounts {
    std::vector<int> count;  // index = eid; 0 = unused
};

// Curvature default for a freeform edge: total turning / angle tolerance.
int freeformDefault(const Model& model, int eid, double angleTolDeg) {
    const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
    double f = 0, l = 0;
    Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
    if (c3.IsNull()) return 1;
    const int probes = 24;
    double turn = 0;
    P3 prev{0, 0, 0};
    bool have = false;
    P3 lastDir{0, 0, 0};
    for (int i = 0; i <= probes; ++i) {
        P3 p = toP3(c3->Value(f + (l - f) * i / probes));
        if (have) {
            P3 d = sub(p, prev);
            double dl = len(d);
            if (dl > 1e-9) {
                d = mul(d, 1.0 / dl);
                if (i > 1) {
                    double c = std::clamp(dot(d, lastDir), -1.0, 1.0);
                    turn += std::acos(c);
                }
                lastDir = d;
            }
        }
        prev = p;
        have = true;
    }
    const double tol = std::max(5.0, angleTolDeg) * M_PI / 180.0;
    return std::clamp((int)std::ceil(turn / tol), 1, 512);
}

EdgeCounts solveEdgeCounts(const Model& model, const Analysis& analysis,
                           const GenerationSettings& settings) {
    EdgeCounts ec;
    ec.count.assign(model.edgeCount() + 1, 0);
    const auto& d = settings.defaults;
    const double scale = std::max(0.05, settings.densityScale);
    auto scl = [&](int n) { return std::max(1, (int)std::lround(n * scale)); };

    // Geometry default per edge.
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
        double f = 0, l = 0;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
        if (c3.IsNull()) { ec.count[eid] = 1; continue; }
        GeomAdaptor_Curve gac(c3, f, l);
        const GeomAbs_CurveType ct = gac.GetType();
        const bool closed = c3->Value(f).Distance(c3->Value(l)) < 1e-7;
        int n = 1;
        if (ct == GeomAbs_Circle) {
            if (closed) {
                n = scl(d.radial);
            } else {
                const double span = std::abs(l - f);  // arc angle
                n = std::max(1, (int)std::ceil(span / (2 * M_PI) * scl(d.radial)));
            }
        } else if (ct == GeomAbs_Line) {
            n = 1;
        } else {
            n = scl(freeformDefault(model, eid, d.angleToleranceDeg));
        }
        ec.count[eid] = n;
    }

    // Revolution faces propose radial on closed-circle rims, axial on straight
    // side edges (max-wins). The interior is NOT forced to match — the bridge
    // absorbs any remaining mismatch.
    for (const FaceInfo& fi : analysis.faces) {
        const bool rev = fi.type == SurfaceType::Cylinder ||
                         fi.type == SurfaceType::Cone ||
                         fi.type == SurfaceType::Sphere ||
                         fi.type == SurfaceType::Torus;
        if (!rev) continue;
        const FaceMeshSettings& fs = settings.forFace(fi.id);
        for (int eid : fi.edgeIds) {
            if (eid < 1 || eid > model.edgeCount()) continue;
            const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
            double f = 0, l = 0;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
            if (c3.IsNull()) continue;
            GeomAdaptor_Curve gac(c3, f, l);
            if (gac.GetType() == GeomAbs_Circle) {
                const bool closed = c3->Value(f).Distance(c3->Value(l)) < 1e-7;
                if (closed) ec.count[eid] = std::max(ec.count[eid], scl(fs.radial));
            } else if (gac.GetType() == GeomAbs_Line) {
                ec.count[eid] = std::max(ec.count[eid], scl(fs.axial));
            }
        }
    }

    // Per-edge pins are exact user inputs (no scale).
    for (const auto& [eid, n] : settings.perEdge)
        if (eid >= 1 && eid <= model.edgeCount() && n >= 1) ec.count[eid] = n;

    for (int& n : ec.count) n = std::max(0, n);
    return ec;
}

// A per-face mesh being built; vertices are local (folded into the model at the
// end) and each carries a surface anchor. orient() winds every polygon to the
// B-rep material normal using the SAME per-vertex-anchor vote foldedPolys judges
// by, so the output is fold-free by that detector's own definition (a bowtie the
// vote can't fix means the interior grid itself twisted — handled upstream by
// azimuth-aligning revolution rings, so orient() only ever flips whole cells).
struct FacePart {
    const Model* model;
    TopoDS_Face face;
    int faceId;
    Handle(Geom_Surface) surf;
    std::vector<P3> verts;
    std::vector<Anchor> anchors;  // parallel to verts
    std::vector<std::vector<uint32_t>> polys;

    FacePart(const Model& m, const TopoDS_Face& f, int id)
        : model(&m), face(f), faceId(id), surf(BRep_Tool::Surface(f)) {}

    uint32_t addV(const P3& p) {
        Anchor a;
        a.faceId = faceId;
        if (!surf.IsNull()) {
            GeomAPI_ProjectPointOnSurf proj(gp_Pnt(p[0], p[1], p[2]), surf);
            if (proj.IsDone() && proj.NbPoints() >= 1)
                proj.LowerDistanceParameters(a.u, a.v);
        }
        verts.push_back(p);
        anchors.push_back(a);
        return uint32_t(verts.size() - 1);
    }
    void addPoly(std::vector<uint32_t> p) {
        std::vector<uint32_t> c;
        for (uint32_t i : p)
            if (c.empty() || c.back() != i) c.push_back(i);
        if (c.size() > 1 && c.front() == c.back()) c.pop_back();
        if (c.size() >= 3) polys.push_back(std::move(c));
    }
    void orient() {
        if (surf.IsNull()) return;
        const double os = face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
        for (auto& poly : polys) {
            P3 n = newell(verts, poly);
            const double nl = len(n);
            if (nl < 1e-14) continue;
            int votes = 0;
            for (uint32_t vi : poly) {
                const Anchor& a = anchors[vi];
                gp_Pnt sp;
                gp_Vec du, dv;
                surf->D1(a.u, a.v, sp, du, dv);
                gp_Vec sn = du.Crossed(dv);
                const double sl = sn.Magnitude();
                if (sl < 1e-14) continue;  // pole: normal undefined
                const double d =
                    os * (sn.X() * n[0] + sn.Y() * n[1] + sn.Z() * n[2]) /
                    (sl * nl);
                if (d > 0.1) ++votes;
                else if (d < -0.1) --votes;
            }
            if (votes < 0) std::reverse(poly.begin(), poly.end());
        }
    }
};

// ---- loop bridge (from core/spike/seam_bridge.cpp, proven) -----------------
// Bridge two ordered loops of local vertex indices into one ring of cells:
// quads where counts line up, grouped n-gons where they don't. `closed` wraps
// the loops (revolution rims); open loops (a face side) share endpoints.
void bridgeLoops(FacePart& part, const std::vector<uint32_t>& O,
                 const std::vector<uint32_t>& I, bool closed) {
    const int N = (int)O.size(), M = (int)I.size();
    if (N < 2 || M < 2) return;
    if (closed) {
        if (M <= N) {
            for (int c = 0; c < M; ++c) {
                const int oa = (int)std::llround((long double)c * N / M);
                const int ob = (int)std::llround((long double)(c + 1) * N / M);
                std::vector<uint32_t> cell;
                for (int k = oa; k <= ob; ++k) cell.push_back(O[k % N]);
                cell.push_back(I[(c + 1) % M]);
                cell.push_back(I[c % M]);
                part.addPoly(std::move(cell));
            }
        } else {
            for (int c = 0; c < N; ++c) {
                const int ia = (int)std::llround((long double)c * M / N);
                const int ib = (int)std::llround((long double)(c + 1) * M / N);
                std::vector<uint32_t> cell;
                cell.push_back(O[c % N]);
                cell.push_back(O[(c + 1) % N]);
                for (int k = ib; k >= ia; --k) cell.push_back(I[k % M]);
                part.addPoly(std::move(cell));
            }
        }
    } else {
        const int a = N - 1, b = M - 1;  // edge counts; endpoints shared
        if (a < 1 || b < 1) return;
        if (b <= a) {
            for (int c = 0; c < b; ++c) {
                int ia = (int)std::llround((long double)c * a / b);
                int ib = (int)std::llround((long double)(c + 1) * a / b);
                std::vector<uint32_t> cell;
                cell.push_back(I[c]);
                cell.push_back(I[c + 1]);
                for (int k = ib; k >= ia; --k) cell.push_back(O[k]);
                part.addPoly(std::move(cell));
            }
        } else {
            for (int c = 0; c < a; ++c) {
                int ia = (int)std::llround((long double)c * b / a);
                int ib = (int)std::llround((long double)(c + 1) * b / a);
                std::vector<uint32_t> cell;
                for (int k = ia; k <= ib; ++k) cell.push_back(I[k]);
                cell.push_back(O[c + 1]);
                cell.push_back(O[c]);
                part.addPoly(std::move(cell));
            }
        }
    }
}

// ---- face boundary loops (outer + holes) ----------------------------------
// Walk each wire in face-local order, sampling every edge at its solved count
// from the shared cache; concatenate into an ordered ring of local vertices.
struct Loop {
    std::vector<uint32_t> verts;               // local indices into part.verts
    std::vector<std::array<double, 2>> uv;     // pcurve (u,v) parallel to verts;
                                               // NaN where no pcurve exists
};

// Cache: eid -> forward samples at the solved count. Shared so both faces
// produce identical border points.
using SampleCache = std::vector<EdgeSamples>;

std::vector<Loop> faceLoops(FacePart& part, const Model& model,
                            const EdgeCounts& ec, const SampleCache& cache,
                            std::map<std::array<double, 3>, uint32_t>* dedup) {
    std::vector<Loop> loops;
    // Weld coincident border verts WITHIN this part (shared edge endpoints) via
    // a spatial key, so a loop closes and adjacent edges share their corner.
    auto keyOf = [](const P3& p) {
        return std::array<double, 3>{std::round(p[0] * 1e6) / 1e6,
                                     std::round(p[1] * 1e6) / 1e6,
                                     std::round(p[2] * 1e6) / 1e6};
    };
    auto vertFor = [&](const P3& p) -> uint32_t {
        auto k = keyOf(p);
        auto it = dedup->find(k);
        if (it != dedup->end()) return it->second;
        uint32_t id = part.addV(p);
        (*dedup)[k] = id;
        return id;
    };
    for (TopExp_Explorer wx(part.face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
        Loop loop;
        for (BRepTools_WireExplorer we(wire, part.face); we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            int eid = model.edges.FindIndex(edge);
            if (eid < 1 || eid >= (int)cache.size()) continue;
            const EdgeSamples& es = cache[eid];
            if (!es.valid || es.pts.empty()) continue;
            std::vector<P3> pts = es.pts;
            std::vector<double> prm = es.param;
            if (we.Orientation() == TopAbs_REVERSED) {
                std::reverse(pts.begin(), pts.end());
                std::reverse(prm.begin(), prm.end());
            }
            // The edge's 2D pcurve ON THIS FACE gives the EXACT (u,v) of each
            // sample (it shares the edge parameter with the 3D curve) — the
            // true trim boundary, unlike an ambiguous point projection.
            double pf, pl;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, part.face, pf, pl);
            auto uvAt = [&](size_t i) -> std::array<double, 2> {
                if (!pc.IsNull() && i < prm.size()) {
                    gp_Pnt2d q = pc->Value(prm[i]);
                    return {q.X(), q.Y()};
                }
                return {std::nan(""), std::nan("")};
            };
            const size_t start = (es.closed || loop.verts.empty()) ? 0 : 1;
            for (size_t i = start; i < pts.size(); ++i) {
                loop.verts.push_back(vertFor(pts[i]));
                loop.uv.push_back(uvAt(i));
            }
        }
        // Close: drop trailing vert if it equals the first (open-edge loop).
        while (loop.verts.size() > 1 && loop.verts.front() == loop.verts.back()) {
            loop.verts.pop_back();
            loop.uv.pop_back();
        }
        if (loop.verts.size() >= 3) loops.push_back(std::move(loop));
    }
    return loops;
}

// ---- revolution wall (full cylinder / cone) -------------------------------
// Two closed-circle rims + a periodic seam. Interior is a (nu x nv) grid. When
// the rims share a count we lerp between them (rulings lie exactly on a
// cylinder/cone); the top/bottom rows ARE the rim samples so caps weld.
bool meshRevolutionWall(FacePart& part, const Model& model, const Analysis& an,
                        const EdgeCounts& ec, const SampleCache& cache,
                        const FaceMeshSettings& fs, MesherKind& kind) {
    // Collect closed rim edges of this face (the insert runs first and claims
    // any wall that actually has interior holes, so a plain 2-rim grid here can
    // never cover a hole).
    std::vector<int> rims;
    const FaceInfo& fi = an.faces[part.faceId - 1];
    for (int eid : fi.edgeIds) {
        if (eid < 1 || eid >= (int)cache.size()) continue;
        if (cache[eid].valid && cache[eid].closed) rims.push_back(eid);
    }
    // One rim + an apex (a full cone / sphere cap): collapse to the pole.
    // Rulings lerp(rim, apex) lie exactly on a cone, so the last grid ring
    // collapses to the apex and the tip row is triangles.
    if (rims.size() == 1) {
        BRepAdaptor_Surface surf(part.face);
        if (surf.GetType() != GeomAbs_Cone) return false;
        // A true cone reaches its apex; a FRUSTUM with a subdivided far rim has
        // extra curved edges (arcs). Only the rim + straight seams (+ the
        // degenerate apex) may be present -- else bail so the seam band
        // reconstructs both rims from their arcs.
        for (int eid : fi.edgeIds) {
            if (eid == rims[0] || eid < 1 || eid > model.edgeCount()) continue;
            const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(e)) continue;
            double f, l;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
            if (c3.IsNull()) continue;
            GeomAdaptor_Curve gac(c3, f, l);
            if (gac.GetType() != GeomAbs_Line) return false;
        }
        const EdgeSamples& r = cache[rims[0]];
        const int nu = (int)r.pts.size();
        if (nu < 3) return false;
        gp_Pnt ap = surf.Cone().Apex();
        P3 apex{ap.X(), ap.Y(), ap.Z()};
        int nv = std::max(1, fs.axial);
        std::vector<std::vector<uint32_t>> rings(nv + 1);
        for (int rr = 0; rr <= nv; ++rr) {
            double t = double(rr) / nv;
            if (rr == nv) {
                rings[rr].assign(1, part.addV(apex));  // collapsed pole
            } else {
                rings[rr].resize(nu);
                for (int i = 0; i < nu; ++i)
                    rings[rr][i] = part.addV(lerp(r.pts[i], apex, t));
            }
        }
        for (int rr = 0; rr < nv; ++rr) {
            const bool tip = (rr + 1 == nv);
            for (int i = 0; i < nu; ++i) {
                int j = (i + 1) % nu;
                if (tip)
                    part.addPoly({rings[rr][i], rings[rr][j], rings[nv][0]});
                else
                    part.addPoly({rings[rr][i], rings[rr][j], rings[rr + 1][j],
                                  rings[rr + 1][i]});
            }
        }
        kind = MesherKind::RevolutionGrid;
        return true;
    }
    if (rims.size() != 2) return false;  // route others to the floor for now

    const EdgeSamples& a = cache[rims[0]];
    const EdgeSamples& b = cache[rims[1]];
    if (a.pts.size() < 3 || b.pts.size() < 3) return false;

    // Unequal rim counts (e.g. a cone/frustum with one rim pinned): the rims
    // can't lerp 1:1, so bridge them into one absorbing band — the decoupled
    // thesis (the loop bridge proven fold-free by the spike). Orient both rings
    // CCW about the axis first so the fraction bridge pairs by azimuth.
    if (a.pts.size() != b.pts.size()) {
        BRepAdaptor_Surface bs(part.face);
        gp_Ax1 bx;
        if (bs.GetType() == GeomAbs_Cylinder) bx = bs.Cylinder().Axis();
        else if (bs.GetType() == GeomAbs_Cone) bx = bs.Cone().Axis();
        else return false;
        const P3 bo{bx.Location().X(), bx.Location().Y(), bx.Location().Z()};
        const P3 bd{bx.Direction().X(), bx.Direction().Y(), bx.Direction().Z()};
        P3 br = std::abs(bd[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
        P3 bX = cross(br, bd);
        bX = mul(bX, 1.0 / std::max(1e-12, len(bX)));
        P3 bY = cross(bd, bX);
        auto ccw = [&](std::vector<P3> r) {
            if (r.size() < 2) return r;
            auto az = [&](const P3& p) {
                P3 q = sub(p, bo);
                return std::atan2(dot(q, bY), dot(q, bX));
            };
            double d = az(r[1]) - az(r[0]);
            while (d > M_PI) d -= 2 * M_PI;
            while (d < -M_PI) d += 2 * M_PI;
            if (d < 0) std::reverse(r.begin(), r.end());
            return r;
        };
        std::vector<uint32_t> loR, hiR;
        for (const P3& p : ccw(a.pts)) loR.push_back(part.addV(p));
        for (const P3& p : ccw(b.pts)) hiR.push_back(part.addV(p));
        bridgeLoops(part, loR, hiR, true);
        kind = MesherKind::RevolutionGrid;
        return true;
    }
    const int nu = (int)a.pts.size();
    if (nu < 3) return false;

    std::vector<P3> lo = a.pts, hi = b.pts;

    // Axis for the azimuth alignment (the rims' local circle frames may differ
    // in X-direction / axis sign, so a phase-anchored index i does NOT share a
    // world azimuth between the two rims — pairing by index twists the rulings
    // and folds cells). Realign hi so hi[i] is the sample nearest lo[i]'s
    // azimuth about the surface axis; the points are unchanged (they still weld
    // to the caps), only the interior pairing is fixed.
    BRepAdaptor_Surface surf(part.face);
    gp_Pnt aO;
    gp_Dir aD;
    if (surf.GetType() == GeomAbs_Cylinder) {
        aO = surf.Cylinder().Axis().Location();
        aD = surf.Cylinder().Axis().Direction();
    } else if (surf.GetType() == GeomAbs_Cone) {
        aO = surf.Cone().Axis().Location();
        aD = surf.Cone().Axis().Direction();
    } else {
        return false;
    }
    const P3 O{aO.X(), aO.Y(), aO.Z()}, D{aD.X(), aD.Y(), aD.Z()};
    P3 ref = std::abs(D[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
    P3 X0 = cross(ref, D);
    X0 = mul(X0, 1.0 / std::max(1e-12, len(X0)));
    P3 Y0 = cross(D, X0);
    auto azim = [&](const P3& p) {
        P3 r = sub(p, O);
        return std::atan2(dot(r, Y0), dot(r, X0));
    };
    std::vector<double> aL(nu), aH(nu);
    for (int i = 0; i < nu; ++i) { aL[i] = azim(lo[i]); aH[i] = azim(hi[i]); }
    std::vector<P3> hiAligned(nu);
    auto angDist = [](double x, double y) {
        double d = std::fmod(std::abs(x - y), 2 * M_PI);
        return d > M_PI ? 2 * M_PI - d : d;
    };
    std::vector<char> used(nu, 0);
    for (int i = 0; i < nu; ++i) {
        int best = -1;
        double bd = 1e300;
        for (int j = 0; j < nu; ++j) {
            if (used[j]) continue;
            double d = angDist(aL[i], aH[j]);
            if (d < bd) { bd = d; best = j; }
        }
        used[best] = 1;
        hiAligned[i] = hi[best];
    }
    hi.swap(hiAligned);

    const int nv = std::max(1, fs.axial);
    std::vector<std::vector<uint32_t>> rings(nv + 1, std::vector<uint32_t>(nu));
    for (int i = 0; i < nu; ++i) {
        rings[0][i] = part.addV(lo[i]);
        rings[nv][i] = part.addV(hi[i]);
    }
    for (int r = 1; r < nv; ++r) {
        double t = double(r) / nv;
        for (int i = 0; i < nu; ++i)
            rings[r][i] = part.addV(lerp(lo[i], hi[i], t));
    }
    for (int r = 0; r < nv; ++r)
        for (int i = 0; i < nu; ++i) {
            int j = (i + 1) % nu;
            part.addPoly({rings[r][i], rings[r][j], rings[r + 1][j], rings[r + 1][i]});
        }
    kind = MesherKind::RevolutionGrid;
    return true;
}

// ---- partial revolution wall (open-u cylinder / cone band) ----------------
// A partial wrap (< 360 deg) has an OPEN boundary: two rim ARCS (top + bottom)
// and two straight SIDE lines. Rulings between the two arcs lie exactly on a
// cylinder/cone, and the straight sides are uniform lines that coincide with
// those rulings, so a structured grid (arcs azimuth-aligned then lerped, side
// columns taken from the side edges' shared samples) welds on all four borders.
// Requires exactly 2 arcs + 2 lines with matched counts; anything else (slots,
// notches, curved sides, tori) bails to the floor.
bool meshPartialRevolutionWall(FacePart& part, const Model& model,
                               const EdgeCounts& ec, const SampleCache& cache,
                               const FaceInfo& fi, MesherKind& kind) {
    BRepAdaptor_Surface surf(part.face);
    const GeomAbs_SurfaceType st = surf.GetType();
    gp_Ax1 ax;
    if (st == GeomAbs_Cylinder) ax = surf.Cylinder().Axis();
    else if (st == GeomAbs_Cone) ax = surf.Cone().Axis();
    else return false;
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(part.face, umin, umax, vmin, vmax);
    if (umax - umin >= 2 * M_PI - 1e-6) return false;  // full wrap: closed path

    // Classify boundary edges by curve type: open arcs vs straight lines.
    std::vector<int> arcs, lines;
    for (int eid : fi.edgeIds) {
        if (eid < 1 || eid >= (int)cache.size() || !cache[eid].valid) continue;
        const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
        double f, l;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
        if (c3.IsNull()) continue;
        GeomAdaptor_Curve gac(c3, f, l);
        if (gac.GetType() == GeomAbs_Circle && !cache[eid].closed)
            arcs.push_back(eid);
        else if (gac.GetType() == GeomAbs_Line)
            lines.push_back(eid);
        else
            return false;  // unclassifiable border -> floor
    }
    if (arcs.size() != 2 || lines.size() != 2) return false;

    const P3 O{ax.Location().X(), ax.Location().Y(), ax.Location().Z()};
    const P3 D{ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()};
    P3 ref = std::abs(D[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
    P3 X0 = cross(ref, D);
    X0 = mul(X0, 1.0 / std::max(1e-12, len(X0)));
    P3 Y0 = cross(D, X0);
    auto azim = [&](const P3& p) {
        P3 r = sub(p, O);
        return std::atan2(dot(r, Y0), dot(r, X0));
    };
    auto height = [&](const std::vector<P3>& pts) {
        double h = 0;
        for (const P3& p : pts) h += dot(sub(p, O), D);
        return h / pts.size();
    };
    // Orient an arc so index 0->1 goes CCW (increasing azimuth): both arcs then
    // start at the same (umin) end and share azimuth per index.
    auto orientCCW = [&](std::vector<P3> pts) {
        if (pts.size() < 2) return pts;
        double d = azim(pts[1]) - azim(pts[0]);
        while (d > M_PI) d -= 2 * M_PI;
        while (d < -M_PI) d += 2 * M_PI;
        if (d < 0) std::reverse(pts.begin(), pts.end());
        return pts;
    };
    std::vector<P3> a0 = orientCCW(cache[arcs[0]].pts);
    std::vector<P3> a1 = orientCCW(cache[arcs[1]].pts);
    std::vector<P3> bot = height(a0) <= height(a1) ? a0 : a1;
    std::vector<P3> top = height(a0) <= height(a1) ? a1 : a0;
    const int nu = (int)bot.size() - 1;
    if ((int)top.size() - 1 != nu || nu < 1) return false;

    // Side edges: match each to the arc end whose corner it shares.
    auto d2 = [&](const P3& a, const P3& b) { return dot(sub(a, b), sub(a, b)); };
    auto sideFrom = [&](const P3& lo, const P3& hi) -> std::vector<P3> {
        for (int eid : lines) {
            std::vector<P3> s = cache[eid].pts;
            if (s.size() < 2) continue;
            bool fwd = d2(s.front(), lo) < 1e-10 && d2(s.back(), hi) < 1e-10;
            bool rev = d2(s.back(), lo) < 1e-10 && d2(s.front(), hi) < 1e-10;
            if (rev) std::reverse(s.begin(), s.end());
            if (fwd || rev) return s;
        }
        return {};
    };
    std::vector<P3> left = sideFrom(bot.front(), top.front());
    std::vector<P3> right = sideFrom(bot.back(), top.back());
    const int nv = (int)left.size() - 1;
    if (nv < 1 || (int)right.size() - 1 != nv) return false;

    std::vector<std::vector<uint32_t>> g(nv + 1, std::vector<uint32_t>(nu + 1));
    for (int iv = 0; iv <= nv; ++iv)
        for (int iu = 0; iu <= nu; ++iu) {
            P3 p;
            if (iv == 0) p = bot[iu];
            else if (iv == nv) p = top[iu];
            else if (iu == 0) p = left[iv];
            else if (iu == nu) p = right[iv];
            else p = lerp(bot[iu], top[iu], double(iv) / nv);
            g[iv][iu] = part.addV(p);
        }
    for (int iv = 0; iv < nv; ++iv)
        for (int iu = 0; iu < nu; ++iu)
            part.addPoly({g[iv][iu], g[iv][iu + 1], g[iv + 1][iu + 1],
                          g[iv + 1][iu]});
    kind = MesherKind::RevolutionGrid;
    return true;
}

// Walk each wire of a face into an ordered ring of 3D points from the shared
// cache (no part mutation).
std::vector<std::vector<P3>> wireLoopsP3(const Model& model,
                                         const TopoDS_Face& face,
                                         const SampleCache& cache) {
    std::vector<std::vector<P3>> out;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        std::vector<P3> loop;
        for (BRepTools_WireExplorer we(w, face); we.More(); we.Next()) {
            int eid = model.edges.FindIndex(we.Current());
            if (eid < 1 || eid >= (int)cache.size() || !cache[eid].valid) continue;
            std::vector<P3> pts = cache[eid].pts;
            if (we.Orientation() == TopAbs_REVERSED)
                std::reverse(pts.begin(), pts.end());
            size_t start = loop.empty() ? 0 : 1;
            for (size_t k = start; k < pts.size(); ++k) loop.push_back(pts[k]);
        }
        while (loop.size() > 1 && len(sub(loop.front(), loop.back())) < 1e-7)
            loop.pop_back();
        if (loop.size() >= 3) out.push_back(std::move(loop));
    }
    return out;
}

// A cyl/cone band whose two rims are multi-edge closed LOOPS (a boolean-cut
// bore, whose rims are intersection curves rather than single circle edges).
// Both rims must encircle the axis; align their start azimuths and bridge them
// into one band (quads where the counts line up, grouped n-gons where they
// don't) — the rulings lie on the surface, so no twist.
bool meshRevolutionBandLoops(FacePart& part, const Model& model,
                             const SampleCache& cache, MesherKind& kind) {
    BRepAdaptor_Surface surf(part.face);
    gp_Ax1 ax;
    switch (surf.GetType()) {
        case GeomAbs_Cylinder: ax = surf.Cylinder().Axis(); break;
        case GeomAbs_Cone: ax = surf.Cone().Axis(); break;
        case GeomAbs_Torus: ax = surf.Torus().Axis(); break;
        case GeomAbs_Sphere: ax = surf.Sphere().Position().Axis(); break;
        default: return false;
    }
    std::vector<std::vector<P3>> loops = wireLoopsP3(model, part.face, cache);
    if (loops.size() != 2) return false;
    const P3 O{ax.Location().X(), ax.Location().Y(), ax.Location().Z()};
    const P3 D{ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()};
    P3 ref = std::abs(D[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
    P3 X0 = cross(ref, D);
    X0 = mul(X0, 1.0 / std::max(1e-12, len(X0)));
    P3 Y0 = cross(D, X0);
    auto azim = [&](const P3& p) {
        P3 r = sub(p, O);
        return std::atan2(dot(r, Y0), dot(r, X0));
    };
    // Total signed azimuth swept ~ +/-2pi means the loop encircles the axis.
    auto sweep = [&](const std::vector<P3>& r) {
        double t = 0;
        for (size_t i = 0; i < r.size(); ++i) {
            double d = azim(r[(i + 1) % r.size()]) - azim(r[i]);
            while (d > M_PI) d -= 2 * M_PI;
            while (d < -M_PI) d += 2 * M_PI;
            t += d;
        }
        return t;
    };
    double sa = sweep(loops[0]), sb = sweep(loops[1]);
    if (std::abs(std::abs(sa) - 2 * M_PI) > 0.6 ||
        std::abs(std::abs(sb) - 2 * M_PI) > 0.6)
        return false;  // not both full rims
    // Drop consecutive near-duplicate points (a subdivided rim can leave a tiny
    // segment); otherwise bridgeLoops emits a degenerate cell that addPoly
    // drops, leaving a gap in the band.
    auto dedup = [](std::vector<P3> r) {
        std::vector<P3> o;
        for (const P3& p : r)
            if (o.empty() || len(sub(o.back(), p)) > 1e-7) o.push_back(p);
        while (o.size() > 1 && len(sub(o.front(), o.back())) < 1e-7) o.pop_back();
        return o;
    };
    std::vector<P3> A = dedup(loops[0]), B = dedup(loops[1]);
    if (A.size() < 3 || B.size() < 3) return false;
    if (sa < 0) std::reverse(A.begin(), A.end());   // both CCW
    if (sb < 0) std::reverse(B.begin(), B.end());
    // Rotate B so B[0] shares A[0]'s azimuth -> fraction pairing follows azimuth.
    double a0 = azim(A[0]);
    int rot = 0;
    double bd = 1e300;
    for (int i = 0; i < (int)B.size(); ++i) {
        double d = std::fmod(std::abs(azim(B[i]) - a0), 2 * M_PI);
        if (d > M_PI) d = 2 * M_PI - d;
        if (d < bd) { bd = d; rot = i; }
    }
    std::rotate(B.begin(), B.begin() + rot, B.end());
    std::vector<uint32_t> ra, rb;
    for (const P3& p : A) ra.push_back(part.addV(p));
    for (const P3& p : B) rb.push_back(part.addV(p));
    bridgeLoops(part, ra, rb, true);
    kind = MesherKind::RevolutionGrid;
    return true;
}

// A periodic surface (torus fillet ring, boolean-cut cylinder) whose boundary is
// ONE wire that encircles the seam is an annular band, not a disk. Reconstruct
// its two rims from the EDGE structure — a rim edge's pcurve runs along the
// encircling axis A, a seam edge runs across it — sampling each rim from the
// shared cache and ordering it by parametric azimuth (uv[A] mod period, since
// the two rims are the same circle at different pcurve u-offsets). Bridge the
// rims closed, GATED on a watertight self-check so a face that isn't a clean
// two-rim band rolls back and falls through to the floor.
bool meshSeamBand(FacePart& part, const Model& model, const SampleCache& cache,
                  const FaceInfo& fi) {
    BRepAdaptor_Surface surf(part.face);
    const double uPer = surf.IsUPeriodic() ? surf.UPeriod() : 0.0;
    const double vPer = surf.IsVPeriodic() ? surf.VPeriod() : 0.0;
    if (uPer <= 0 && vPer <= 0) return false;

    struct EInfo { int eid; double da, db, bmid; };
    // First pass with A=u to measure spans, decide the encircling axis.
    auto pcOf = [&](int eid, double& f, double& l) {
        const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
        return BRep_Tool::CurveOnSurface(e, part.face, f, l);
    };
    double sumdu = 0, sumdv = 0;
    std::vector<int> eids;
    for (int eid : fi.edgeIds) {
        if (eid < 1 || eid >= (int)cache.size() || !cache[eid].valid) continue;
        double f, l;
        Handle(Geom2d_Curve) pc = pcOf(eid, f, l);
        if (pc.IsNull()) return false;
        gp_Pnt2d p0 = pc->Value(f), p1 = pc->Value(l);
        sumdu += std::abs(p1.X() - p0.X());
        sumdv += std::abs(p1.Y() - p0.Y());
        eids.push_back(eid);
    }
    if (eids.size() < 2) return false;
    const int A = sumdu >= sumdv ? 0 : 1;      // encircling axis (0=u,1=v)
    const double per = A == 0 ? uPer : vPer;
    if (per <= 0) return false;

    // Rim edges span primarily along A; group them into two clusters by their
    // midpoint in the other coordinate B.
    double maxA = 0, bmin = 1e300, bmax = -1e300;
    std::vector<EInfo> rims;
    for (int eid : eids) {
        double f, l;
        Handle(Geom2d_Curve) pc = pcOf(eid, f, l);
        if (pc.IsNull()) continue;
        gp_Pnt2d p0 = pc->Value(f), p1 = pc->Value(l), pm = pc->Value(0.5 * (f + l));
        double da = A == 0 ? std::abs(p1.X() - p0.X()) : std::abs(p1.Y() - p0.Y());
        double db = A == 0 ? std::abs(p1.Y() - p0.Y()) : std::abs(p1.X() - p0.X());
        double bmid = A == 0 ? pm.Y() : pm.X();
        maxA = std::max(maxA, da);
        if (da > db) {  // a rim (runs along A), not a seam
            rims.push_back({eid, da, db, bmid});
            bmin = std::min(bmin, bmid);
            bmax = std::max(bmax, bmid);
        }
    }
    if (rims.size() < 2 || bmax - bmin < 1e-6) return false;
    // Only a genuine full-ring encircler qualifies: a rim must wind most of the
    // period. A segment (rails + end-caps) has short rim spans and would leak
    // its end-caps here, so it belongs on the floor.
    if (maxA < 0.75 * per) return false;
    const double bmed = 0.5 * (bmin + bmax);

    // Sample a rim cluster (edges with bmid on one side) into a ring. Each
    // edge's samples keep their ON-CURVE order (so every ring edge is a real
    // B-rep segment that welds with the neighbour); only the EDGES are ordered
    // relative to each other by azimuth. Shared endpoints dedup by position.
    auto buildRim = [&](bool hiSide) -> std::vector<uint32_t> {
        struct RE { double azi; std::vector<uint32_t> vids; };
        std::vector<RE> res;
        std::map<std::array<long long, 3>, uint32_t> dedup;
        auto vfor = [&](const P3& p) {
            std::array<long long, 3> key{llround(p[0] * 1e5), llround(p[1] * 1e5),
                                         llround(p[2] * 1e5)};
            auto it = dedup.find(key);
            if (it != dedup.end()) return it->second;
            uint32_t v = part.addV(p);
            dedup[key] = v;
            return v;
        };
        auto aziAt = [&](Handle(Geom2d_Curve) pc, double param) {
            gp_Pnt2d q = pc->Value(param);
            double a = std::fmod(A == 0 ? q.X() : q.Y(), per);
            return a < 0 ? a + per : a;
        };
        for (const EInfo& r : rims) {
            if ((r.bmid >= bmed) != hiSide) continue;
            double f, l;
            Handle(Geom2d_Curve) pc = pcOf(r.eid, f, l);
            if (pc.IsNull()) continue;
            const EdgeSamples& s = cache[r.eid];
            RE re;
            for (const P3& p : s.pts) re.vids.push_back(vfor(p));
            double a0 = aziAt(pc, s.param.front()), a1 = aziAt(pc, s.param.back());
            double d = a1 - a0;
            while (d > per / 2) d -= per;
            while (d < -per / 2) d += per;
            if (d < 0) { std::reverse(re.vids.begin(), re.vids.end()); a0 = a1; }
            re.azi = a0;
            res.push_back(std::move(re));
        }
        std::sort(res.begin(), res.end(),
                  [](const RE& x, const RE& y) { return x.azi < y.azi; });
        std::vector<uint32_t> ring;
        for (const RE& re : res)
            for (uint32_t v : re.vids)
                if (ring.empty() || ring.back() != v) ring.push_back(v);
        while (ring.size() > 1 && ring.front() == ring.back()) ring.pop_back();
        return ring;
    };
    std::vector<uint32_t> ra = buildRim(false), rb = buildRim(true);
    if (ra.size() < 3 || rb.size() < 3) return false;

    // Coverage check: every SHARED boundary edge must be fully represented in
    // the two rims (its samples all land on rim vertices). If the classification
    // dropped a shared edge (a segment's end-cap mistaken for a seam), reject so
    // the face falls to the floor -- this keeps the band a pure improvement.
    auto pkey = [](const P3& p) {
        return std::array<long long, 3>{llround(p[0] * 1e5), llround(p[1] * 1e5),
                                        llround(p[2] * 1e5)};
    };
    std::set<std::array<long long, 3>> rimPos;
    for (uint32_t v : ra) rimPos.insert(pkey(part.verts[v]));
    for (uint32_t v : rb) rimPos.insert(pkey(part.verts[v]));
    for (int eid : fi.edgeIds) {
        if (eid < 1 || eid >= (int)cache.size() || !cache[eid].valid) continue;
        const TopoDS_Shape& e = model.edges(eid);
        int nf = 0;
        if (model.edgeToFaces.Contains(e))
            for (const TopoDS_Shape& sh : model.edgeToFaces.FindFromKey(e)) {
                int f2 = model.faces.FindIndex(sh);
                if (f2 >= 1 && f2 != part.faceId) ++nf;
            }
        if (nf == 0) continue;  // internal seam edge: not required in the rims
        for (const P3& p : cache[eid].pts)
            if (!rimPos.count(pkey(p))) return false;  // shared edge dropped
    }

    const size_t pBase = part.polys.size();
    bridgeLoops(part, ra, rb, /*closed=*/true);
    // Watertight self-check: band cells must be 2-manifold with exactly the two
    // rim rings as boundary; otherwise roll back and let the floor try.
    std::map<std::pair<uint32_t, uint32_t>, int> use;
    for (size_t p = pBase; p < part.polys.size(); ++p) {
        const auto& poly = part.polys[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
            use[a < b ? std::make_pair(a, b) : std::make_pair(b, a)]++;
        }
    }
    int once = 0;
    bool bad = part.polys.size() == pBase;
    for (const auto& [e, c] : use) {
        if (c > 2) bad = true;
        if (c == 1) ++once;
    }
    if (bad || once != (int)(ra.size() + rb.size())) {
        part.polys.resize(pBase);
        return false;
    }
    return true;
}

// ---- full revolution wall with interior bore holes ------------------------
// A full cylinder/cone wall whose two main rims (top/bottom) enclose K interior
// closed-circle holes (bores punched through). Grid the wall as u-wrapped rings
// with v-rows placed to BRACKET each hole, punch the cells each hole covers, and
// bridge the resulting staircase to the hole's exact rim samples (from the
// cache, so the bore wall welds). Requires equal-count main rims and holes that
// occupy disjoint cell blocks; otherwise bails to the floor.
bool meshRevolutionWallInsert(FacePart& part, const Model& model,
                              const SampleCache& cache, const FaceInfo& fi,
                              MesherKind& kind) {
    BRepAdaptor_Surface surf(part.face);
    const GeomAbs_SurfaceType st = surf.GetType();
    gp_Ax1 ax;
    if (st == GeomAbs_Cylinder) ax = surf.Cylinder().Axis();
    else if (st == GeomAbs_Cone) ax = surf.Cone().Axis();
    else return false;
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(part.face, umin, umax, vmin, vmax);
    if (umax - umin < 2 * M_PI - 1e-6) return false;  // partial handled elsewhere

    const P3 O{ax.Location().X(), ax.Location().Y(), ax.Location().Z()};
    const P3 D{ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()};
    P3 ref = std::abs(D[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
    P3 X0 = cross(ref, D);
    X0 = mul(X0, 1.0 / std::max(1e-12, len(X0)));
    P3 Y0 = cross(D, X0);
    auto azim = [&](const P3& p) {
        P3 r = sub(p, O);
        return std::atan2(dot(r, Y0), dot(r, X0));
    };
    auto hgt = [&](const P3& p) { return dot(sub(p, O), D); };

    auto ccw = [&](std::vector<P3> r) {
        if (r.size() < 2) return r;
        double d = azim(r[1]) - azim(r[0]);
        while (d > M_PI) d -= 2 * M_PI;
        while (d < -M_PI) d += 2 * M_PI;
        if (d < 0) std::reverse(r.begin(), r.end());
        return r;
    };
    // Main rims = the two closed-CIRCLE edges at extreme mean height (bore rims
    // are intersection curves, not circles, so this picks the true top/bottom).
    struct Ring { std::vector<P3> pts; double h; };
    std::vector<Ring> circles;
    for (int eid : fi.edgeIds) {
        if (eid < 1 || eid >= (int)cache.size() || !cache[eid].valid) continue;
        if (!cache[eid].closed) continue;
        const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
        double f, l;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
        if (c3.IsNull()) continue;
        GeomAdaptor_Curve gac(c3, f, l);
        if (gac.GetType() != GeomAbs_Circle) continue;
        double hm = 0;
        for (const P3& p : cache[eid].pts) hm += hgt(p);
        circles.push_back({cache[eid].pts, hm / cache[eid].pts.size()});
    }
    if ((int)circles.size() < 2) return false;
    int loI = 0, hiI = 0;
    for (int i = 1; i < (int)circles.size(); ++i) {
        if (circles[i].h < circles[loI].h) loI = i;
        if (circles[i].h > circles[hiI].h) hiI = i;
    }
    if (loI == hiI) return false;
    std::vector<P3> mLo = ccw(circles[loI].pts), mHi = ccw(circles[hiI].pts);
    const int nu = (int)mLo.size();
    if (nu < 6 || (int)mHi.size() != nu) return false;
    const double hLo = circles[loI].h, hHi = circles[hiI].h;
    if (hHi - hLo < 1e-9) return false;

    // Hole loops = every wire except the outer, sampled from the shared cache
    // (so the bore wall welds). Circular closed edges at interior height (not a
    // main rim) also count.
    std::vector<std::vector<P3>> holeLoops;
    const TopoDS_Wire outerW = BRepTools::OuterWire(part.face);
    for (TopExp_Explorer wx(part.face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        if (w.IsSame(outerW)) continue;
        std::vector<P3> loop;
        for (BRepTools_WireExplorer we(w, part.face); we.More(); we.Next()) {
            int eid = model.edges.FindIndex(we.Current());
            if (eid < 1 || eid >= (int)cache.size() || !cache[eid].valid) continue;
            std::vector<P3> pts = cache[eid].pts;
            if (we.Orientation() == TopAbs_REVERSED)
                std::reverse(pts.begin(), pts.end());
            size_t start = loop.empty() ? 0 : 1;
            for (size_t k = start; k < pts.size(); ++k) loop.push_back(pts[k]);
        }
        while (loop.size() > 1 && len(sub(loop.front(), loop.back())) < 1e-9)
            loop.pop_back();
        if (loop.size() >= 3) holeLoops.push_back(std::move(loop));
    }
    if (holeLoops.empty()) return false;

    // Holes and their (column-range, height-range) footprints.
    struct Hole { std::vector<P3> ring; double h0, h1; int c0, c1; };
    std::vector<Hole> holes;
    // Grid column azimuths from the bottom main rim.
    std::vector<double> colAz(nu);
    for (int i = 0; i < nu; ++i) colAz[i] = azim(mLo[i]);
    auto nearestCol = [&](double az) {
        int best = 0;
        double bd = 1e300;
        for (int i = 0; i < nu; ++i) {
            double d = std::fmod(std::abs(az - colAz[i]), 2 * M_PI);
            if (d > M_PI) d = 2 * M_PI - d;
            if (d < bd) { bd = d; best = i; }
        }
        return best;
    };
    std::vector<double> rowH{hLo, hHi};
    for (auto& loop : holeLoops) {
        Hole h;
        h.ring = loop;
        h.h0 = 1e300; h.h1 = -1e300;
        std::vector<int> cols;
        for (const P3& p : h.ring) {
            double z = hgt(p);
            h.h0 = std::min(h.h0, z); h.h1 = std::max(h.h1, z);
            cols.push_back(nearestCol(azim(p)));
        }
        if (h.h0 <= hLo + 1e-6 || h.h1 >= hHi - 1e-6) return false;  // touches a rim
        // Contiguous column span (mod nu): pick the rotation with the smallest
        // spread so a seam-straddling hole stays contiguous.
        std::sort(cols.begin(), cols.end());
        cols.erase(std::unique(cols.begin(), cols.end()), cols.end());
        int bestStart = 0, bestSpan = nu;
        for (int s = 0; s < (int)cols.size(); ++s) {
            int lo = cols[s], hi = cols[(s + (int)cols.size() - 1) % cols.size()];
            int span = (hi - lo + nu) % nu;
            if (span < bestSpan) { bestSpan = span; bestStart = s; }
        }
        h.c0 = cols[bestStart];
        h.c1 = (h.c0 + bestSpan) % nu;
        rowH.push_back(h.h0);
        rowH.push_back(h.h1);
        holes.push_back(std::move(h));
    }
    if (holes.empty()) return false;

    std::sort(rowH.begin(), rowH.end());
    rowH.erase(std::unique(rowH.begin(), rowH.end(),
                           [](double a, double b) { return std::abs(a - b) < 1e-6; }),
               rowH.end());
    const int nv = (int)rowH.size() - 1;
    if (nv < 1) return false;
    auto rowAt = [&](double h) {
        int best = 0;
        for (int j = 0; j < (int)rowH.size(); ++j)
            if (std::abs(rowH[j] - h) < std::abs(rowH[best] - h)) best = j;
        return best;
    };

    // Grid rings by height fraction; rulings lie on the cyl/cone.
    std::vector<std::vector<uint32_t>> g(nv + 1, std::vector<uint32_t>(nu));
    for (int j = 0; j <= nv; ++j) {
        double t = (rowH[j] - hLo) / (hHi - hLo);
        for (int i = 0; i < nu; ++i)
            g[j][i] = part.addV(lerp(mLo[i], mHi[i], t));
    }
    // Mark deleted cells (cell (j,i) spans rows j..j+1, cols i..i+1 mod nu).
    std::vector<std::vector<char>> dead(nv, std::vector<char>(nu, 0));
    for (const Hole& h : holes) {
        int r0 = rowAt(h.h0), r1 = rowAt(h.h1);
        if (r1 <= r0) return false;
        for (int j = r0; j < r1; ++j)
            for (int c = h.c0;; c = (c + 1) % nu) {
                if (dead[j][c]) return false;  // holes overlap -> bail
                dead[j][c] = 1;
                if (c == h.c1) break;
            }
    }
    for (int j = 0; j < nv; ++j)
        for (int i = 0; i < nu; ++i) {
            if (dead[j][i]) continue;
            int ni = (i + 1) % nu;
            part.addPoly({g[j][i], g[j][ni], g[j + 1][ni], g[j + 1][i]});
        }
    // Web each hole: the perimeter of its deleted block bridges to the ring.
    for (const Hole& h : holes) {
        int r0 = rowAt(h.h0), r1 = rowAt(h.h1);
        std::vector<uint32_t> stair;
        // bottom edge L->R
        for (int c = h.c0;; c = (c + 1) % nu) {
            stair.push_back(g[r0][c]);
            if (c == h.c1) break;
        }
        stair.push_back(g[r0][(h.c1 + 1) % nu]);
        // right edge up
        for (int j = r0 + 1; j <= r1; ++j) stair.push_back(g[j][(h.c1 + 1) % nu]);
        // top edge R->L
        for (int c = (h.c1 + 1) % nu;; c = (c + nu - 1) % nu) {
            stair.push_back(g[r1][c]);
            if (c == h.c0) break;
        }
        // left edge down
        for (int j = r1 - 1; j > r0; --j) stair.push_back(g[j][h.c0]);
        // dedup consecutive
        std::vector<uint32_t> s2;
        for (uint32_t v : stair)
            if (s2.empty() || s2.back() != v) s2.push_back(v);
        while (s2.size() > 1 && s2.front() == s2.back()) s2.pop_back();
        std::vector<uint32_t> ring;
        for (const P3& p : ccw(h.ring)) ring.push_back(part.addV(p));
        if (s2.size() >= 3 && ring.size() >= 3) bridgeLoops(part, s2, ring, true);
    }
    kind = MesherKind::RevolutionGrid;
    return true;
}

// ---- planar plate with K holes: bridge decomposition ----------------------
// A flat face with holes. Cutting each hole with TWO non-crossing bridges turns
// a polygon-with-one-hole into two SIMPLE (hole-free) polygons; with several
// holes, each hole splits the polygon that contains it and the remaining holes
// are redistributed by containment, so a K-holed plate becomes K+1 simple
// n-gons with real shared edges (the codebase's doctrine — no ear-clip-over-
// keyhole to fold; watertight for any count ratio). All-or-nothing: if any hole
// can't find a clear pair of bridges the whole face bails to the floor.
// `geom` supplies the 2D projection coordinates (part.verts for a flat face,
// per-vertex UV for a curved face). With `triangulate`, each simple output
// polygon is ear-clipped (reliable — they carry no holes) instead of emitted as
// an n-gon: that makes this a robust curved-face FLOOR (bridge-decompose, then
// triangulate) with none of the keyhole ear-clip's self-overlap.
bool meshPlanarMultiHole(FacePart& part, std::vector<uint32_t> O,
                         std::vector<std::vector<uint32_t>> holes,
                         const std::vector<P3>& geom, bool triangulate) {
    if (O.size() < 3 || holes.empty()) return false;
    P3 nrm = newell(geom, O);
    if (len(nrm) < 1e-14) return false;
    nrm = mul(nrm, 1.0 / len(nrm));
    P3 refA = std::abs(nrm[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
    P3 ex = cross(refA, nrm);
    ex = mul(ex, 1.0 / std::max(1e-12, len(ex)));
    P3 ey = cross(nrm, ex);
    auto uv = [&](uint32_t v) {
        return std::array<double, 2>{dot(geom[v], ex), dot(geom[v], ey)};
    };
    auto sArea = [&](const std::vector<uint32_t>& L) {
        double s = 0;
        for (size_t k = 0; k < L.size(); ++k) {
            auto a = uv(L[k]), b = uv(L[(k + 1) % L.size()]);
            s += a[0] * b[1] - b[0] * a[1];
        }
        return s;
    };
    if (sArea(O) < 0) std::reverse(O.begin(), O.end());  // outer CCW
    for (auto& H : holes)
        if (sArea(H) > 0) std::reverse(H.begin(), H.end());  // holes CW
    auto seg = [&](std::array<double, 2> a, std::array<double, 2> b,
                   std::array<double, 2> c, std::array<double, 2> d) {
        auto o = [](std::array<double, 2> p, std::array<double, 2> q,
                    std::array<double, 2> r) {
            double v = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
            return v > 1e-12 ? 1 : (v < -1e-12 ? -1 : 0);
        };
        int o1 = o(a, b, c), o2 = o(a, b, d), o3 = o(c, d, a), o4 = o(c, d, b);
        return o1 != o2 && o3 != o4 && o1 && o2 && o3 && o4;
    };
    auto hitsLoop = [&](std::array<double, 2> A, std::array<double, 2> B,
                        uint32_t sa, uint32_t sb, const std::vector<uint32_t>& L) {
        for (size_t k = 0; k < L.size(); ++k) {
            uint32_t p = L[k], q = L[(k + 1) % L.size()];
            if (p == sa || q == sa || p == sb || q == sb) continue;
            if (seg(A, B, uv(p), uv(q))) return true;
        }
        return false;
    };
    auto centroidUV = [&](const std::vector<uint32_t>& L) {
        std::array<double, 2> c{0, 0};
        for (uint32_t v : L) { auto p = uv(v); c[0] += p[0]; c[1] += p[1]; }
        c[0] /= L.size(); c[1] /= L.size();
        return c;
    };
    auto inPoly = [&](std::array<double, 2> pt, const std::vector<uint32_t>& L) {
        bool in = false;
        for (size_t i = 0, j = L.size() - 1; i < L.size(); j = i++) {
            auto a = uv(L[i]), b = uv(L[j]);
            if (((a[1] > pt[1]) != (b[1] > pt[1])) &&
                (pt[0] < (b[0] - a[0]) * (pt[1] - a[1]) / (b[1] - a[1]) + a[0]))
                in = !in;
        }
        return in;
    };
    auto arcOf = [&](const std::vector<uint32_t>& L, int from, int to) {
        std::vector<uint32_t> s;
        int n = (int)L.size();
        for (int k = from;; k = (k + 1) % n) {
            s.push_back(L[k]);
            if (k == to) break;
        }
        return s;
    };
    auto d2 = [&](uint32_t a, uint32_t b) {
        return dot(sub(part.verts[a], part.verts[b]),
                   sub(part.verts[a], part.verts[b]));
    };

    struct Work { std::vector<uint32_t> poly; std::vector<int> hs; };
    std::vector<int> all(holes.size());
    for (int i = 0; i < (int)holes.size(); ++i) all[i] = i;
    std::vector<Work> stack{{O, all}};
    std::vector<std::vector<uint32_t>> out;
    int guard = 0;
    while (!stack.empty()) {
        if (guard++ > 4 * (int)holes.size() + 8) return false;
        Work w = std::move(stack.back());
        stack.pop_back();
        if (w.hs.empty()) { out.push_back(std::move(w.poly)); continue; }
        const int hidx = w.hs[0];
        std::vector<uint32_t>& H = holes[hidx];
        const int N = (int)w.poly.size(), M = (int)H.size();
        // Bridges must clear the polygon, this hole, AND every other hole in w.
        auto clear = [&](int pi, int hi, int pi2, int hi2) {
            auto A = uv(w.poly[pi]), B = uv(H[hi]);
            if (hitsLoop(A, B, w.poly[pi], H[hi], w.poly)) return false;
            for (int oh : w.hs)
                if (hitsLoop(A, B, w.poly[pi], H[hi], holes[oh])) return false;
            if (pi2 >= 0 && seg(A, B, uv(w.poly[pi2]), uv(H[hi2]))) return false;
            return true;
        };
        int pA = -1, hA = -1;
        double bd = 1e300;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j) {
                double d = d2(w.poly[i], H[j]);
                if (d < bd && clear(i, j, -1, -1)) { bd = d; pA = i; hA = j; }
            }
        if (pA < 0) return false;
        int pB = -1, hB = -1;
        bd = 1e300;
        for (int dj = M / 4; dj <= 3 * M / 4; ++dj) {
            int j = (hA + dj) % M;
            if (j == hA) continue;
            for (int i = 0; i < N; ++i) {
                if (i == pA) continue;
                double d = d2(w.poly[i], H[j]);
                if (d < bd && clear(i, j, pA, hA)) { bd = d; pB = i; hB = j; }
            }
        }
        if (pB < 0) return false;
        std::vector<uint32_t> p1 = arcOf(w.poly, pA, pB);
        for (uint32_t v : arcOf(H, hB, hA)) p1.push_back(v);
        std::vector<uint32_t> p2 = arcOf(w.poly, pB, pA);
        for (uint32_t v : arcOf(H, hA, hB)) p2.push_back(v);
        Work w1{std::move(p1), {}}, w2{std::move(p2), {}};
        for (size_t k = 1; k < w.hs.size(); ++k) {
            int oh = w.hs[k];
            if (inPoly(centroidUV(holes[oh]), w1.poly)) w1.hs.push_back(oh);
            else w2.hs.push_back(oh);
        }
        stack.push_back(std::move(w1));
        stack.push_back(std::move(w2));
    }
    for (auto& p : out) {
        if (triangulate)
            for (const auto& t : triangulatePoly(geom, p))
                part.addPoly({p[t[0]], p[t[1]], p[t[2]]});
        else
            part.addPoly(std::move(p));
    }
    return true;
}

// ---- full closed periodic surface (sphere / torus) ------------------------
// A complete sphere or torus is one closed face with no shared trim borders, so
// it grids straight from its own (u,v) parametrization: one parametrization for
// every ring means adjacent rings share u by construction (no twist). u wraps;
// v wraps on a torus and collapses to poles on a sphere. Trimmed patches
// (u-span < full period) fall through to the floor.
bool meshFullPeriodic(FacePart& part, const Model& model, const FaceInfo& fi,
                      const FaceMeshSettings& fs, MesherKind& kind) {
    BRepAdaptor_Surface surf(part.face);
    const GeomAbs_SurfaceType st = surf.GetType();
    if (st != GeomAbs_Sphere && st != GeomAbs_Torus) return false;
    // Only a COMPLETE closed surface grids straight from surf.Value; a patch
    // (sphere zone, torus fillet band) has shared boundary edges whose samples
    // must come from the cache, so route it to the band mesher instead.
    for (int eid : fi.edgeIds) {
        if (eid < 1 || eid > model.edgeCount()) continue;
        const TopoDS_Shape& e = model.edges(eid);
        if (!model.edgeToFaces.Contains(e)) continue;
        for (const TopoDS_Shape& s : model.edgeToFaces.FindFromKey(e)) {
            int f2 = model.faces.FindIndex(s);
            if (f2 >= 1 && f2 != part.faceId) return false;  // shared -> patch
        }
    }
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(part.face, umin, umax, vmin, vmax);
    if ((umax - umin) < 2 * M_PI - 1e-6) return false;  // trimmed -> floor
    const bool vFull = (vmax - vmin) >= 2 * M_PI - 1e-6;
    const int nu = std::max(3, fs.radial);
    // A v-periodic tube (torus) needs >= 3 cross-section rings to wrap without
    // doubling cells; a sphere only needs 2 (equator + poles).
    const int nv = std::max(vFull ? 3 : 2, fs.axial);
    Handle(Geom_Surface) gs = part.surf;
    if (gs.IsNull()) return false;
    auto S = [&](double u, double v) {
        gp_Pnt p = gs->Value(u, v);
        return P3{p.X(), p.Y(), p.Z()};
    };
    const int rowsN = vFull ? nv : nv + 1;  // distinct rings (torus wraps v)
    std::vector<std::vector<uint32_t>> rings(rowsN);
    for (int j = 0; j < rowsN; ++j) {
        double v = vmin + (vmax - vmin) * j / nv;
        std::vector<P3> pts(nu);
        for (int i = 0; i < nu; ++i)
            pts[i] = S(umin + (umax - umin) * i / nu, v);
        bool pole = true;
        for (int i = 1; i < nu; ++i)
            if (len(sub(pts[i], pts[0])) > 1e-7) { pole = false; break; }
        if (pole) {
            rings[j].assign(1, part.addV(pts[0]));
        } else {
            rings[j].resize(nu);
            for (int i = 0; i < nu; ++i) rings[j][i] = part.addV(pts[i]);
        }
    }
    const int bands = vFull ? rowsN : rowsN - 1;
    for (int r = 0; r < bands; ++r) {
        const std::vector<uint32_t>& A = rings[r];
        const std::vector<uint32_t>& B = rings[(r + 1) % rowsN];
        const bool aPole = A.size() == 1, bPole = B.size() == 1;
        for (int i = 0; i < nu; ++i) {
            int j = (i + 1) % nu;
            if (aPole && bPole) continue;
            if (bPole)
                part.addPoly({A[i], A[j], B[0]});
            else if (aPole)
                part.addPoly({A[0], B[j], B[i]});
            else
                part.addPoly({A[i], A[j], B[j], B[i]});
        }
    }
    kind = MesherKind::RevolutionGrid;
    return true;
}

// ---- boundary floor (keyhole-bridged loops -> triangulatePoly) ------------
// Watertight for any face: uses ONLY the shared border samples. Bridges every
// inner (hole) loop into the outer loop with a doubled-vertex keyhole, then
// hands the single ring to the proven ear-clipper (mesh.cpp). Quality is a
// floor; real interior meshers (UV-coons) replace it in later increments.
// `geom` supplies the coordinates for all geometric decisions (outer pick,
// keyhole visibility, triangulation) while polygons emit the REAL vertex ids.
// Pass part.verts to triangulate in a 3D plane (flat faces) or per-vertex UV
// (from the anchors) to triangulate in the surface parametrization (curved
// faces / walls with holes), which a 3D-plane projection would mangle.
void meshFloor(FacePart& part, const std::vector<Loop>& loops,
               const std::vector<P3>& geom) {
    if (loops.empty()) return;
    // Outer loop = the one enclosing the most area (Newell magnitude), NOT the
    // most vertices: a plate's rectangular border has few samples (straight
    // edges) while its round hole has many, so vertex count picks the hole.
    size_t outer = 0;
    double outerA = -1;
    for (size_t i = 0; i < loops.size(); ++i) {
        double a = len(newell(geom, loops[i].verts));
        if (a > outerA) { outerA = a; outer = i; }
    }
    std::vector<uint32_t> ring = loops[outer].verts;
    if (ring.size() < 3) return;

    // Project everything to the outer loop's dominant plane once, so the
    // visibility test that keeps a keyhole bridge from crossing a loop edge
    // (and folding the ear-clip) is a clean 2D check.
    P3 nrm = newell(geom, ring);
    if (len(nrm) < 1e-14) return;
    nrm = mul(nrm, 1.0 / len(nrm));
    P3 refA = std::abs(nrm[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
    P3 ex = cross(refA, nrm);
    ex = mul(ex, 1.0 / std::max(1e-12, len(ex)));
    P3 ey = cross(nrm, ex);
    auto uv = [&](uint32_t v) {
        return std::array<double, 2>{dot(geom[v], ex), dot(geom[v], ey)};
    };
    // Proper segment intersection (open segments; shared endpoints allowed).
    auto crosses = [&](std::array<double, 2> a, std::array<double, 2> b,
                       std::array<double, 2> c, std::array<double, 2> dd) {
        auto o = [](std::array<double, 2> p, std::array<double, 2> q,
                    std::array<double, 2> r) {
            double val = (q[0] - p[0]) * (r[1] - p[1]) -
                         (q[1] - p[1]) * (r[0] - p[0]);
            return val > 1e-12 ? 1 : (val < -1e-12 ? -1 : 0);
        };
        int o1 = o(a, b, c), o2 = o(a, b, dd), o3 = o(c, dd, a), o4 = o(c, dd, b);
        return o1 != o2 && o3 != o4 && o1 && o2 && o3 && o4;
    };
    auto bridgeHits = [&](uint32_t rv, uint32_t hv) {
        auto A = uv(rv), B = uv(hv);
        auto scan = [&](const std::vector<uint32_t>& L) {
            for (size_t k = 0; k < L.size(); ++k) {
                uint32_t p = L[k], q = L[(k + 1) % L.size()];
                if (p == rv || q == rv || p == hv || q == hv) continue;
                if (crosses(A, B, uv(p), uv(q))) return true;
            }
            return false;
        };
        if (scan(ring)) return true;
        for (size_t li = 0; li < loops.size(); ++li)
            if (li != outer && scan(loops[li].verts)) return true;
        return false;
    };
    auto d2 = [&](uint32_t a, uint32_t b) {
        return dot(sub(geom[a], geom[b]), sub(geom[a], geom[b]));
    };
    for (size_t li = 0; li < loops.size(); ++li) {
        if (li == outer) continue;
        const std::vector<uint32_t>& h = loops[li].verts;
        if (h.size() < 3) continue;
        // Shortest MUTUALLY-VISIBLE pair (bridge segment crosses no loop edge):
        // the doubled slit then ear-clips as a clean hole.
        size_t bri = 0, bhi = 0;
        double bd = 1e300;
        bool found = false;
        for (size_t ri = 0; ri < ring.size(); ++ri)
            for (size_t hi = 0; hi < h.size(); ++hi) {
                double d = d2(ring[ri], h[hi]);
                if (d >= bd) continue;
                if (bridgeHits(ring[ri], h[hi])) continue;
                bd = d; bri = ri; bhi = hi; found = true;
            }
        if (!found)  // fall back to nearest even if it grazes
            for (size_t ri = 0; ri < ring.size(); ++ri)
                for (size_t hi = 0; hi < h.size(); ++hi) {
                    double d = d2(ring[ri], h[hi]);
                    if (d < bd) { bd = d; bri = ri; bhi = hi; }
                }
        // Splice: outer[0..ri] + hole[hi..around..hi] + outer[ri..]. The
        // doubled outer[ri] and hole[hi] form the zero-width bridge slit that
        // triangulatePoly tessellates as a hole (inner loops wound opposite to
        // the outer by wire order).
        std::vector<uint32_t> nr;
        nr.reserve(ring.size() + h.size() + 2);
        for (size_t k = 0; k <= bri; ++k) nr.push_back(ring[k]);
        for (size_t k = 0; k <= h.size(); ++k) nr.push_back(h[(bhi + k) % h.size()]);
        for (size_t k = bri; k < ring.size(); ++k) nr.push_back(ring[k]);
        ring.swap(nr);
    }
    for (const auto& t : triangulatePoly(geom, ring))
        part.addPoly({ring[t[0]], ring[t[1]], ring[t[2]]});
}

// Freeform faces (bspline / bezier / general revolution / extrusion / offset)
// have no periodic seam and a strongly non-planar boundary, so triangulating in
// their surface UV beats a 3D-plane projection. Analytic types keep the plane
// (flats are planar; cyl/cone/sphere/torus are periodic and their UV seam would
// break the ear-clip).
bool freeformFloor(SurfaceType t) {
    return t == SurfaceType::BSpline || t == SurfaceType::Bezier ||
           t == SurfaceType::Revolution || t == SurfaceType::Extrusion ||
           t == SurfaceType::Offset || t == SurfaceType::Other;
}

// Choose the floor's working coordinates. Freeform faces triangulate in their
// surface UV — taken from the loops' exact pcurve (u,v) — so a bspline patch
// conforms to its true trim boundary instead of collapsing under a 3D-plane
// projection (or a self-intersecting point-projection). Bail to the 3D vertices
// when the UV is unusable: missing pcurves, or a genuine annulus (a single loop
// encircling the seam), which is bridged as a band instead.
void meshFloorAuto(FacePart& part, const std::vector<Loop>& loops,
                   bool useUV) {
    if (loops.empty()) return;
    std::vector<P3> uvp;
    bool wantUV = useUV;
    if (useUV) {
        uvp.assign(part.verts.size(), P3{0, 0, 0});
        BRepAdaptor_Surface bs(part.face);
        const double uPer = bs.IsUPeriodic() ? bs.UPeriod() : 0.0;
        const double vPer = bs.IsVPeriodic() ? bs.VPeriod() : 0.0;
        // Unwrap each loop so a boundary that crosses the periodic seam stays
        // continuous (a torus fillet segment straddling u=0 becomes a simple
        // loop instead of a torn one), then shift each hole loop by whole
        // periods to sit near the outer loop.
        auto unwrap = [](double x, double ref, double per) {
            if (per <= 0) return x;
            while (x - ref > per * 0.5) x -= per;
            while (ref - x > per * 0.5) x += per;
            return x;
        };
        struct UL { std::vector<std::array<double, 2>> uv; std::array<double, 2> mean; double area; };
        std::vector<UL> uls;
        bool haveUV = true;
        for (const Loop& lp : loops) {
            UL ul;
            ul.uv.resize(lp.verts.size());
            double su = 0, sv = 0;
            for (size_t k = 0; k < lp.verts.size(); ++k) {
                auto q = lp.uv[k];
                if (std::isnan(q[0]) || std::isnan(q[1])) { haveUV = false; break; }
                if (k > 0) {
                    q[0] = unwrap(q[0], ul.uv[k - 1][0], uPer);
                    q[1] = unwrap(q[1], ul.uv[k - 1][1], vPer);
                }
                ul.uv[k] = q;
                su += q[0];
                sv += q[1];
            }
            if (!haveUV) break;
            ul.mean = {su / ul.uv.size(), sv / ul.uv.size()};
            double a = 0;
            const int n = (int)ul.uv.size();
            for (int i = 0; i < n; ++i) {
                const auto& p = ul.uv[i];
                const auto& r = ul.uv[(i + 1) % n];
                a += p[0] * r[1] - r[0] * p[1];
            }
            ul.area = a;
            uls.push_back(std::move(ul));
        }
        if (haveUV) {
            size_t oi = 0;
            for (size_t i = 1; i < uls.size(); ++i)
                if (std::abs(uls[i].area) > std::abs(uls[oi].area)) oi = i;
            double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
            for (size_t i = 0; i < uls.size(); ++i) {
                double du = 0, dv = 0;
                if (i != oi) {
                    du = unwrap(uls[i].mean[0], uls[oi].mean[0], uPer) - uls[i].mean[0];
                    dv = unwrap(uls[i].mean[1], uls[oi].mean[1], vPer) - uls[i].mean[1];
                }
                for (size_t k = 0; k < uls[i].uv.size(); ++k) {
                    double u = uls[i].uv[k][0] + du, v = uls[i].uv[k][1] + dv;
                    uvp[loops[i].verts[k]] = {u, v, 0};
                    umin = std::min(umin, u); umax = std::max(umax, u);
                    vmin = std::min(vmin, v); vmax = std::max(vmax, v);
                }
            }
            // If even after unwrapping the outer spans a whole period, it truly
            // encircles the seam (an annulus, not a disk) -> the flat UV can't be
            // one simple polygon; fall back to the 3D projection.
            const bool encU = uPer > 0 && umax - umin > uPer * 0.9;
            const bool encV = vPer > 0 && vmax - vmin > vPer * 0.9;
            // A single loop encircling the seam is a genuine annulus (not a flat
            // disk); the flat UV can't be one simple polygon, so fall back to the
            // 3D projection. (A dedicated band mesher for these is the next piece
            // -- see docs/HANDOFF.md.)
            if (umax - umin < 1e-9 || vmax - vmin < 1e-9 || encU || encV)
                haveUV = false;
        }
        if (!haveUV) wantUV = false;
    }
    const std::vector<P3>& geom = wantUV ? uvp : part.verts;
    // Outer = the max-area loop; a single loop triangulates directly (reliable),
    // multiple loops bridge-decompose into simple polygons then triangulate —
    // no keyhole ear-clip, so no self-overlap.
    size_t o = 0;
    double oa = -1;
    for (size_t i = 0; i < loops.size(); ++i) {
        double a = len(newell(geom, loops[i].verts));
        if (a > oa) { oa = a; o = i; }
    }
    if (loops[o].verts.size() < 3) return;
    if (loops.size() == 1) {
        const auto& r = loops[o].verts;
        for (const auto& t : triangulatePoly(geom, r))
            part.addPoly({r[t[0]], r[t[1]], r[t[2]]});
        return;
    }
    std::vector<std::vector<uint32_t>> hs;
    for (size_t i = 0; i < loops.size(); ++i)
        if (i != o) hs.push_back(loops[i].verts);
    if (!meshPlanarMultiHole(part, loops[o].verts, hs, geom, /*triangulate=*/true))
        meshFloor(part, loops, geom);  // keyhole fallback if bridges fail
}

// ---- global winding consistency -------------------------------------------
// Per-face orient() winds each cell to its own material normal, which is
// correct in isolation but does not by itself guarantee that adjacent faces
// traverse a shared edge in OPPOSITE directions (the watertight-manifold
// invariant a game engine needs for consistent normals). This flood-fill makes
// the whole mesh consistent: BFS over shared edges, flipping any neighbour that
// traverses a shared edge the SAME way; then each connected component's global
// sign is fixed against the B-rep material normal so it isn't inside-out.
void orientMeshConsistent(PolyMesh& mesh, const Model& model) {
    const size_t nP = mesh.polygons.size();
    if (!nP) return;
    // Undirected edge -> polygons that use it (manifold edges only).
    std::map<std::pair<uint32_t, uint32_t>, std::vector<int>> edgePolys;
    for (size_t p = 0; p < nP; ++p) {
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            edgePolys[key].push_back((int)p);
        }
    }
    auto traverses = [&](int p, uint32_t a, uint32_t b) {  // does p have a->b?
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i)
            if (poly[i] == a && poly[(i + 1) % poly.size()] == b) return true;
        return false;
    };
    std::vector<char> seen(nP, 0);
    std::vector<int> comp;
    for (size_t s = 0; s < nP; ++s) {
        if (seen[s]) continue;
        // BFS this component into consistency.
        comp.clear();
        std::vector<int> stack{(int)s};
        seen[s] = 1;
        while (!stack.empty()) {
            int p = stack.back();
            stack.pop_back();
            comp.push_back(p);
            const auto poly = mesh.polygons[p];  // copy (we may flip neighbours)
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
                const auto& users = edgePolys[key];
                if (users.size() != 2) continue;  // boundary / non-manifold
                int q = users[0] == p ? users[1] : users[0];
                if (seen[q]) continue;
                // Neighbour must traverse the shared edge the OTHER way (b->a);
                // if it also has a->b, it's wound the same -> flip it.
                if (traverses(q, a, b))
                    std::reverse(mesh.polygons[q].begin(), mesh.polygons[q].end());
                seen[q] = 1;
                stack.push_back(q);
            }
        }
        // Fix the component's global sign against the B-rep: the largest cell
        // votes its Newell normal against its face's material normal.
        int ref = comp[0];
        double refArea = -1;
        for (int p : comp) {
            P3 n = newell(mesh.vertices, mesh.polygons[p]);
            double a = len(n);
            if (a > refArea) { refArea = a; ref = p; }
        }
        const int fid = ref < (int)mesh.polygonFaceId.size()
                            ? mesh.polygonFaceId[ref] : 0;
        if (fid >= 1 && fid <= model.faceCount()) {
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
            const auto& poly = mesh.polygons[ref];
            P3 c{0, 0, 0};
            for (uint32_t v : poly) c = add(c, mesh.vertices[v]);
            c = mul(c, 1.0 / poly.size());
            if (!surf.IsNull()) {
                GeomAPI_ProjectPointOnSurf proj(gp_Pnt(c[0], c[1], c[2]), surf);
                if (proj.IsDone() && proj.NbPoints() >= 1) {
                    double u, v;
                    proj.LowerDistanceParameters(u, v);
                    gp_Pnt sp;
                    gp_Vec du, dv;
                    surf->D1(u, v, sp, du, dv);
                    gp_Vec sn = du.Crossed(dv);
                    if (face.Orientation() == TopAbs_REVERSED) sn.Reverse();
                    P3 nn = newell(mesh.vertices, poly);
                    if (sn.Magnitude() > 1e-14 &&
                        (sn.X() * nn[0] + sn.Y() * nn[1] + sn.Z() * nn[2]) < 0)
                        for (int p : comp)
                            std::reverse(mesh.polygons[p].begin(),
                                         mesh.polygons[p].end());
                }
            }
        }
    }
}

}  // namespace

PolyMesh meshDecoupled(const Model& model, const Analysis& analysis,
                       const GenerationSettings& settings,
                       GenerationReport* report) {
    const int faceN = model.faceCount();
    EdgeCounts ec = solveEdgeCounts(model, analysis, settings);

    // Sample every edge ONCE at its solved count -> the shared border array.
    SampleCache cache(model.edgeCount() + 1);
    for (int eid = 1; eid <= model.edgeCount(); ++eid)
        if (ec.count[eid] >= 1) cache[eid] = sampleEdge(model, eid, ec.count[eid]);

    // Mesh each face independently.
    std::vector<FacePart> parts;
    parts.reserve(faceN);
    std::map<int, MesherKind> kinds;
    for (int fid = 1; fid <= faceN; ++fid) {
        const FaceMeshSettings& fs = settings.forFace(fid);
        if (fs.exclude) continue;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        parts.emplace_back(model, face, fid);
        FacePart& part = parts.back();
        MesherKind kind = MesherKind::Fallback;
        const FaceInfo& fi = analysis.faces[fid - 1];

        bool done = false;
        if (fi.type == SurfaceType::Cylinder || fi.type == SurfaceType::Cone) {
            // Insert first: it only succeeds when the wall really has interior
            // holes, so plain walls/bores fall through to the grid meshers.
            done = meshRevolutionWallInsert(part, model, cache, fi, kind);
            if (!done)
                done = meshRevolutionWall(part, model, analysis, ec, cache, fs, kind);
            if (!done)
                done = meshPartialRevolutionWall(part, model, ec, cache, fi, kind);
            if (!done)
                done = meshRevolutionBandLoops(part, model, cache, kind);
            if (!done && (done = meshSeamBand(part, model, cache, fi)))
                kind = MesherKind::RevolutionGrid;
        } else if (fi.type == SurfaceType::Sphere ||
                   fi.type == SurfaceType::Torus) {
            done = meshFullPeriodic(part, model, fi, fs, kind);
            if (!done)  // a band/zone patch: bridge its two shared rim loops
                done = meshRevolutionBandLoops(part, model, cache, kind);
            if (!done && (done = meshSeamBand(part, model, cache, fi)))
                kind = MesherKind::RevolutionGrid;
        }
        if (!done) {
            std::map<std::array<double, 3>, uint32_t> dedup;
            std::vector<Loop> loops = faceLoops(part, model, ec, cache, &dedup);
            if (loops.empty()) {
                kind = MesherKind::Fallback;
            } else if (fi.type == SurfaceType::Plane && loops.size() == 1) {
                // Flat single-loop face: one boundary n-gon (game-minimal).
                part.addPoly(loops[0].verts);
                kind = loops[0].verts.size() > 4 && fi.edgeIds.size() == 1
                           ? MesherKind::DiskCap
                           : MesherKind::MinimalNGon;
            } else if (fi.type == SurfaceType::Plane && loops.size() >= 2) {
                // Flat plate with K holes: bridge-decompose into K+1 simple
                // n-gons; fall back to the floor if a hole can't be bridged.
                size_t o = 0;
                double oa = -1;
                for (size_t i = 0; i < loops.size(); ++i) {
                    double a = len(newell(part.verts, loops[i].verts));
                    if (a > oa) { oa = a; o = i; }
                }
                std::vector<std::vector<uint32_t>> hs;
                for (size_t i = 0; i < loops.size(); ++i)
                    if (i != o) hs.push_back(loops[i].verts);
                if (meshPlanarMultiHole(part, loops[o].verts, hs, part.verts,
                                        /*triangulate=*/false))
                    kind = MesherKind::PlateWeb;
                else {
                    meshFloorAuto(part, loops, false);
                    kind = MesherKind::Fallback;
                }
            } else if (loops.size() == 2) {
                // Non-planar annular band (washer / fillet ring): the proven
                // fraction bridge gives clean quads.
                size_t o = len(newell(part.verts, loops[0].verts)) >=
                                   len(newell(part.verts, loops[1].verts))
                               ? 0 : 1;
                bridgeLoops(part, loops[o].verts, loops[1 - o].verts, true);
                kind = MesherKind::AnnulusRing;
            } else {
                // Walls with bore holes, curved patches, and everything else:
                // the floor, triangulated in exact pcurve UV (a valid face's UV
                // trim boundary is simple, so no self-overlap) for any non-flat
                // surface; flats stay in their exact 3D plane.
                meshFloorAuto(part, loops, fi.type != SurfaceType::Plane);
                kind = MesherKind::Fallback;
            }
        }
        part.orient();
        kinds[fid] = kind;
    }

    // ---- assemble ----
    PolyMesh mesh;
    std::vector<std::array<size_t, 2>> range(faceN + 1, {0, 0});
    for (FacePart& part : parts) {
        uint32_t base = uint32_t(mesh.vertices.size());
        range[part.faceId] = {base, base + part.verts.size()};
        for (size_t i = 0; i < part.verts.size(); ++i) {
            mesh.vertices.push_back(part.verts[i]);
            mesh.anchors.push_back(part.anchors[i]);
        }
        for (auto& poly : part.polys) {
            for (uint32_t& v : poly) v += base;
            mesh.polygons.push_back(std::move(poly));
            mesh.polygonFaceId.push_back(part.faceId);
        }
    }

    // ---- weld per solid ----
    std::vector<int> faceSolid(faceN + 1, 0);
    int solidId = 0;
    auto assign = [&](const TopoDS_Shape& obj) {
        ++solidId;
        for (TopExp_Explorer fx(obj, TopAbs_FACE); fx.More(); fx.Next()) {
            int f2 = model.faces.FindIndex(fx.Current());
            if (f2 > 0 && faceSolid[f2] == 0) faceSolid[f2] = solidId;
        }
    };
    for (TopExp_Explorer sx(model.shape, TopAbs_SOLID); sx.More(); sx.Next())
        assign(sx.Current());
    for (TopExp_Explorer sx(model.shape, TopAbs_SHELL, TopAbs_SOLID); sx.More();
         sx.Next())
        assign(sx.Current());
    std::vector<int> weldGroup;
    if (solidId > 1) {
        weldGroup.assign(mesh.vertices.size(), 0);
        for (int fid = 1; fid <= faceN; ++fid)
            for (size_t v = range[fid][0]; v < range[fid][1]; ++v)
                weldGroup[v] = faceSolid[fid];
    }
    weldVertices(mesh, std::max(settings.weldTolerance, 1e-6),
                 weldGroup.empty() ? nullptr : &weldGroup, nullptr);

    // Consistent winding across faces (game-engine normals): the per-face
    // material-normal orient() is a good seed but not a global guarantee.
    orientMeshConsistent(mesh, model);

    // Border-contract verifier (env-gated): every sample of an edge shared by
    // two real faces must land on a welded vertex used by >= 2 faces. Reports
    // which mesher pairs fail to weld.
    if (std::getenv("WEFT_DC_CONTRACT")) {
        auto key = [](const P3& p) {
            return std::array<long long, 3>{llround(p[0] * 1e5),
                                            llround(p[1] * 1e5),
                                            llround(p[2] * 1e5)};
        };
        std::map<std::array<long long, 3>, uint32_t> pos;
        for (uint32_t v = 0; v < mesh.vertices.size(); ++v)
            pos.emplace(key(mesh.vertices[v]), v);
        std::vector<std::set<int>> vf(mesh.vertices.size());
        for (size_t p = 0; p < mesh.polygons.size(); ++p)
            for (uint32_t v : mesh.polygons[p]) vf[v].insert(mesh.polygonFaceId[p]);
        int bad = 0;
        std::map<std::pair<std::string, std::string>, int> pairs;
        for (int eid = 1; eid <= model.edgeCount(); ++eid) {
            if (eid >= (int)cache.size() || !cache[eid].valid) continue;
            const TopoDS_Shape& e = model.edges(eid);
            if (!model.edgeToFaces.Contains(e)) continue;
            std::set<int> fset;
            for (const TopoDS_Shape& s : model.edgeToFaces.FindFromKey(e)) {
                int f2 = model.faces.FindIndex(s);
                if (f2 >= 1 && !settings.forFace(f2).exclude) fset.insert(f2);
            }
            if (fset.size() < 2) continue;  // input boundary or self-seam
            std::vector<int> fs(fset.begin(), fset.end());
            for (const P3& sp : cache[eid].pts) {
                auto it = pos.find(key(sp));
                int uses = it == pos.end() ? 0 : (int)vf[it->second].size();
                if (uses < 2) {
                    ++bad;
                    auto nm = [&](int f) {
                        return kinds.count(f) ? mesherKindName(kinds[f]) : "?";
                    };
                    std::string k0 = nm(fs[0]), k1 = nm(fs[1]);
                    if (k0 > k1) std::swap(k0, k1);
                    if (bad <= 5 && std::getenv("WEFT_DC_CONTRACT2")) {
                        std::string ub;
                        if (it != pos.end())
                            for (int f : vf[it->second]) ub += " " + std::to_string(f);
                        std::fprintf(stderr,
                                     "  eid=%d f=%d,%d(%s,%s) sample(%.2f,%.2f,%.2f) "
                                     "vert=%s usedBy:%s\n",
                                     eid, fs[0], fs[1], nm(fs[0]), nm(fs[1]),
                                     sp[0], sp[1], sp[2],
                                     it == pos.end() ? "MISSING" : "found", ub.c_str());
                    }
                    pairs[{k0, k1}]++;
                    break;
                }
            }
        }
        std::fprintf(stderr, "[contract] %d shared edges not fully welded\n", bad);
        for (auto& [k, c] : pairs)
            std::fprintf(stderr, "  %-16s <-> %-16s : %d\n", k.first.c_str(),
                         k.second.c_str(), c);
    }

    if (report) {
        report->faceMesher = kinds;
        for (int eid = 1; eid <= model.edgeCount(); ++eid)
            if (ec.count[eid] >= 1) report->edgeDivisions[eid] = ec.count[eid];
    }
    return mesh;
}

}  // namespace weft
