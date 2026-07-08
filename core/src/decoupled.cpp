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
#include <GeomAdaptor_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
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
#include <map>
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
    std::vector<P3> pts;  // forward order (param f->l)
    bool closed = false;  // full loop by itself (a circle): pts is a ring
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
        for (int k = 0; k < n; ++k)
            s.pts.push_back(toP3(c3->Value(a0 + 2.0 * M_PI * k / n)));
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
    for (double t : frac) s.pts.push_back(toP3(c3->Value(f + span * t)));
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
    std::vector<uint32_t> verts;  // local indices into part.verts
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
            if (we.Orientation() == TopAbs_REVERSED)
                std::reverse(pts.begin(), pts.end());
            if (es.closed) {
                // Whole loop is this one circle.
                for (const P3& p : pts) loop.verts.push_back(vertFor(p));
            } else {
                // Append, dropping the shared first endpoint (previous edge's
                // last). vertFor dedups it to the same id regardless.
                size_t start = loop.verts.empty() ? 0 : 1;
                for (size_t i = start; i < pts.size(); ++i)
                    loop.verts.push_back(vertFor(pts[i]));
            }
        }
        // Close: drop trailing vert if it equals the first (open-edge loop).
        while (loop.verts.size() > 1 && loop.verts.front() == loop.verts.back())
            loop.verts.pop_back();
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
    // Collect closed-circle rim edges of this face.
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
    if (a.pts.size() != b.pts.size()) return false;  // unequal rims -> floor
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

// ---- planar plate with K holes: bridge decomposition ----------------------
// A flat face with holes. Cutting each hole with TWO non-crossing bridges turns
// a polygon-with-one-hole into two SIMPLE (hole-free) polygons; with several
// holes, each hole splits the polygon that contains it and the remaining holes
// are redistributed by containment, so a K-holed plate becomes K+1 simple
// n-gons with real shared edges (the codebase's doctrine — no ear-clip-over-
// keyhole to fold; watertight for any count ratio). All-or-nothing: if any hole
// can't find a clear pair of bridges the whole face bails to the floor.
bool meshPlanarMultiHole(FacePart& part, std::vector<uint32_t> O,
                         std::vector<std::vector<uint32_t>> holes) {
    if (O.size() < 3 || holes.empty()) return false;
    P3 nrm = newell(part.verts, O);
    if (len(nrm) < 1e-14) return false;
    nrm = mul(nrm, 1.0 / len(nrm));
    P3 refA = std::abs(nrm[2]) < 0.9 ? P3{0, 0, 1} : P3{1, 0, 0};
    P3 ex = cross(refA, nrm);
    ex = mul(ex, 1.0 / std::max(1e-12, len(ex)));
    P3 ey = cross(nrm, ex);
    auto uv = [&](uint32_t v) {
        return std::array<double, 2>{dot(part.verts[v], ex),
                                     dot(part.verts[v], ey)};
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
    for (auto& p : out) part.addPoly(std::move(p));
    return true;
}

// ---- full closed periodic surface (sphere / torus) ------------------------
// A complete sphere or torus is one closed face with no shared trim borders, so
// it grids straight from its own (u,v) parametrization: one parametrization for
// every ring means adjacent rings share u by construction (no twist). u wraps;
// v wraps on a torus and collapses to poles on a sphere. Trimmed patches
// (u-span < full period) fall through to the floor.
bool meshFullPeriodic(FacePart& part, const FaceMeshSettings& fs,
                      MesherKind& kind) {
    BRepAdaptor_Surface surf(part.face);
    const GeomAbs_SurfaceType st = surf.GetType();
    if (st != GeomAbs_Sphere && st != GeomAbs_Torus) return false;
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
// surface UV — taken from the per-vertex anchors — so a bspline patch conforms
// instead of collapsing under a 3D-plane projection. Bail to the 3D vertices
// when the UV is unusable: a full-period seam wrap (the loop spans ~2pi in u, so
// the flat UV is an annulus, not a disk) or missing/degenerate anchors.
void meshFloorAuto(FacePart& part, const std::vector<Loop>& loops,
                   bool useUV) {
    if (loops.empty()) return;
    if (!useUV) { meshFloor(part, loops, part.verts); return; }
    std::vector<P3> uvp(part.verts.size(), P3{0, 0, 0});
    double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
    bool haveUV = part.anchors.size() == part.verts.size();
    for (size_t i = 0; i < part.anchors.size() && haveUV; ++i) {
        uvp[i] = {part.anchors[i].u, part.anchors[i].v, 0};
        umin = std::min(umin, part.anchors[i].u);
        umax = std::max(umax, part.anchors[i].u);
        vmin = std::min(vmin, part.anchors[i].v);
        vmax = std::max(vmax, part.anchors[i].v);
    }
    const bool degenerate = !haveUV || (umax - umin) < 1e-9 ||
                            (vmax - vmin) < 1e-9;
    const bool seamWrap = (umax - umin) > 1.9 * M_PI;  // annulus, not a disk
    meshFloor(part, loops, (degenerate || seamWrap) ? part.verts : uvp);
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
            done = meshRevolutionWall(part, model, analysis, ec, cache, fs, kind);
            if (!done)
                done = meshPartialRevolutionWall(part, model, ec, cache, fi, kind);
        } else if (fi.type == SurfaceType::Sphere ||
                   fi.type == SurfaceType::Torus) {
            done = meshFullPeriodic(part, fs, kind);
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
                if (meshPlanarMultiHole(part, loops[o].verts, hs))
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
                // the floor, triangulated in surface UV for freeform faces
                // (a bspline patch) and in a 3D plane otherwise.
                meshFloorAuto(part, loops, freeformFloor(fi.type));
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

    if (report) {
        report->faceMesher = kinds;
        for (int eid = 1; eid <= model.edgeCount(); ++eid)
            if (ec.count[eid] >= 1) report->edgeDivisions[eid] = ec.count[eid];
    }
    return mesh;
}

}  // namespace weft
