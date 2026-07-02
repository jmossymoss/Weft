#include "weft/meshers.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <ElCLib.hxx>
#include <ElSLib.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace weft {

std::vector<double> clusteredParams(int divisions, double hold) {
    int n = std::max(1, divisions);
    hold = std::min(0.95, std::max(0.0, hold));
    std::vector<double> t(n + 1);
    for (int i = 0; i <= n; ++i) {
        double x = double(i) / n;
        // Monotonic for hold < 1: slope 1-hold at the ends, 1+hold mid-span,
        // so intervals shrink near the creases and grow in the middle.
        t[i] = x - hold * std::sin(2.0 * M_PI * x) / (2.0 * M_PI);
    }
    t.front() = 0.0;
    t.back() = 1.0;
    return t;
}

const char* mesherKindName(MesherKind k) {
    switch (k) {
        case MesherKind::RevolutionGrid: return "revolution-grid";
        case MesherKind::DiskCap: return "disk-cap";
        case MesherKind::PlanarGrid: return "parametric-grid";
        case MesherKind::RingJunction: return "ring-junction";
        case MesherKind::Fallback: return "fallback-tri";
    }
    return "fallback-tri";
}

namespace {

class MeshBuilder {
public:
    explicit MeshBuilder(PolyMesh& mesh) : mesh_(mesh) {}

    uint32_t addVertex(const gp_Pnt& p, const Anchor& anchor) {
        mesh_.vertices.push_back({p.X(), p.Y(), p.Z()});
        mesh_.anchors.push_back(anchor);
        return static_cast<uint32_t>(mesh_.vertices.size() - 1);
    }

    void addPolygon(std::vector<uint32_t> indices, int faceId, bool flip) {
        if (flip) std::reverse(indices.begin(), indices.end());
        mesh_.polygons.push_back(std::move(indices));
        mesh_.polygonFaceId.push_back(faceId);
    }

private:
    PolyMesh& mesh_;
};

// ---------------------------------------------------------------------------
// Planning: decide a strategy per face and collect the edges whose
// subdivision counts that strategy consumes, split by parametric direction.

struct FacePlan {
    MesherKind kind = MesherKind::Fallback;
    // Edges the mesher subdivides `radial`/`gridU` times (u-iso boundary
    // curves) and `axial`/`gridV` times respectively. Empty for Fallback.
    std::vector<int> uEdges;
    std::vector<int> vEdges;
    // Whether this plan's edge lists participate in density matching.
    bool constrains = false;
    // Fillet strips get support loops across the blend instead of plain
    // grid divisions; `acrossIsU` says which parametric direction spans it.
    bool isFillet = false;
    bool acrossIsU = true;
    gp_Circ circ;          // DiskCap and RingJunction
    int circleEdgeId = 0;  // RingJunction: the hole's edge
};

bool isClosedRevolution(const BRepAdaptor_Surface& surf) {
    switch (surf.GetType()) {
        case GeomAbs_Cylinder:
        case GeomAbs_Cone:
        case GeomAbs_Sphere:
        case GeomAbs_Torus:
        case GeomAbs_SurfaceOfRevolution: break;
        default: return false;
    }
    return surf.IsUClosed();
}

// A planar face bounded by exactly one full-circle edge (a cylinder cap).
bool boundingCircle(const TopoDS_Face& face, gp_Circ& circOut, int& edgeIdOut,
                    const Model& model) {
    int edgeCount = 0;
    TopoDS_Edge only;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        ++edgeCount;
        only = TopoDS::Edge(ex.Current());
    }
    if (edgeCount != 1) return false;

    double f = 0, l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(only, f, l);
    Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
    if (circle.IsNull()) return false;
    if (std::abs((l - f) - 2.0 * M_PI) > 1e-7) return false;
    circOut = circle->Circ();
    edgeIdOut = model.edges.FindIndex(only);
    return true;
}

enum class EdgeIso { UAligned, VAligned, Neither };

// Classify a boundary edge by its pcurve: does it run along u (constant v)
// or along v (constant u)? Density matching only binds iso-aligned edges,
// where "edge subdivisions" and "grid divisions" are the same thing.
EdgeIso edgeIsoDirection(const TopoDS_Edge& edge, const TopoDS_Face& face,
                         double uRange, double vRange) {
    double f = 0, l = 0;
    Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, f, l);
    if (pcurve.IsNull()) return EdgeIso::Neither;
    gp_Pnt2d a = pcurve->Value(f);
    gp_Pnt2d b = pcurve->Value(l);
    gp_Pnt2d m = pcurve->Value(0.5 * (f + l));
    double du = std::max(std::abs(b.X() - a.X()), std::abs(m.X() - a.X())) /
                std::max(uRange, 1e-12);
    double dv = std::max(std::abs(b.Y() - a.Y()), std::abs(m.Y() - a.Y())) /
                std::max(vRange, 1e-12);
    const double kIsoTol = 1e-6;
    if (dv < kIsoTol && du > kIsoTol) return EdgeIso::UAligned;
    if (du < kIsoTol && dv > kIsoTol) return EdgeIso::VAligned;
    return EdgeIso::Neither;
}

void collectIsoEdges(const TopoDS_Face& face, const Model& model,
                     const std::vector<int>& edgeIds, FacePlan& plan) {
    BRepAdaptor_Surface surf(face);
    double uRange = surf.LastUParameter() - surf.FirstUParameter();
    double vRange = surf.LastVParameter() - surf.FirstVParameter();
    for (int eid : edgeIds) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;  // apex/pole edges
        switch (edgeIsoDirection(edge, face, uRange, vRange)) {
            case EdgeIso::UAligned: plan.uEdges.push_back(eid); break;
            case EdgeIso::VAligned: plan.vEdges.push_back(eid); break;
            case EdgeIso::Neither: plan.constrains = false; return;
        }
    }
    plan.constrains = true;
}

// A planar face with a rectangular (2u+2v iso-edge) outer wire and exactly
// one full-circle inner wire: the cylinder-to-plane junction case. Fills
// plan.circ / circleEdgeId / uEdges / vEdges on success.
bool planRingJunction(const TopoDS_Face& face, const Model& model,
                      FacePlan& plan) {
    BRepAdaptor_Surface surf(face);
    if (surf.GetType() != GeomAbs_Plane) return false;

    TopoDS_Wire outer = BRepTools::OuterWire(face);
    TopoDS_Wire inner;
    int wireCount = 0;
    for (TopExp_Explorer ex(face, TopAbs_WIRE); ex.More(); ex.Next()) {
        ++wireCount;
        if (!ex.Current().IsSame(outer)) inner = TopoDS::Wire(ex.Current());
    }
    if (wireCount != 2 || inner.IsNull()) return false;

    int innerEdgeCount = 0;
    TopoDS_Edge circleEdge;
    for (TopExp_Explorer ex(inner, TopAbs_EDGE); ex.More(); ex.Next()) {
        ++innerEdgeCount;
        circleEdge = TopoDS::Edge(ex.Current());
    }
    if (innerEdgeCount != 1) return false;
    double f = 0, l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(circleEdge, f, l);
    Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
    if (circle.IsNull() || std::abs((l - f) - 2.0 * M_PI) > 1e-7) return false;

    std::vector<int> outerIds;
    for (TopExp_Explorer ex(outer, TopAbs_EDGE); ex.More(); ex.Next()) {
        int eid = model.edges.FindIndex(ex.Current());
        if (eid > 0 && std::find(outerIds.begin(), outerIds.end(), eid) ==
                           outerIds.end()) {
            outerIds.push_back(eid);
        }
    }
    if (outerIds.size() != 4) return false;
    collectIsoEdges(face, model, outerIds, plan);
    if (!plan.constrains || plan.uEdges.size() != 2 || plan.vEdges.size() != 2) {
        plan.uEdges.clear();
        plan.vEdges.clear();
        plan.constrains = false;
        return false;
    }

    plan.kind = MesherKind::RingJunction;
    plan.circ = circle->Circ();
    plan.circleEdgeId = model.edges.FindIndex(circleEdge);
    return true;
}

// UV grid feasibility: the grid must lie inside the trim boundary, verified
// by classifying every node and cell center. Conforming a grid to arbitrary
// trim curves is the hard problem (plan §7.1) and stays out of scope.
bool parametricGridFits(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        int nu, int nv) {
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    // A face spanning a full period needs wrap handling this mesher doesn't
    // do; closed revolutions are handled by RevolutionGrid instead.
    if (surf.IsUPeriodic() && umax - umin > surf.UPeriod() - 1e-9) return false;
    if (surf.IsVPeriodic() && vmax - vmin > surf.VPeriod() - 1e-9) return false;

    const double du = (umax - umin) / nu;
    const double dv = (vmax - vmin) / nv;
    const double tol = BRep_Tool::Tolerance(face);

    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            gp_Pnt2d node(umin + i * du, vmin + j * dv);
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), node, tol);
            if (cls.State() == TopAbs_OUT) return false;
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            gp_Pnt2d center(umin + (i + 0.5) * du, vmin + (j + 0.5) * dv);
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), center,
                                         tol);
            if (cls.State() != TopAbs_IN) return false;
        }
    }
    return true;
}

FacePlan planFace(int fid, const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings) {
    const TopoDS_Face face = TopoDS::Face(model.faces(fid));
    const FaceMeshSettings& s = settings.forFace(fid);
    const FaceInfo& info = analysis.faces[fid - 1];
    BRepAdaptor_Surface surf(face);
    FacePlan plan;

    if (isClosedRevolution(surf)) {
        plan.kind = MesherKind::RevolutionGrid;
        collectIsoEdges(face, model, info.edgeIds, plan);
        return plan;
    }

    int capEdgeId = 0;
    if (surf.GetType() == GeomAbs_Plane &&
        boundingCircle(face, plan.circ, capEdgeId, model)) {
        plan.kind = MesherKind::DiskCap;
        plan.uEdges.push_back(capEdgeId);  // ring count = edge subdivisions
        plan.constrains = true;
        return plan;
    }

    if (planRingJunction(face, model, plan)) return plan;

    if (parametricGridFits(face, surf, std::max(1, s.gridU),
                           std::max(1, s.gridV))) {
        plan.kind = MesherKind::PlanarGrid;
        if (info.isFillet) {
            plan.isFillet = true;
            // The blend arc runs along u for a cylinder strip and along the
            // minor circle (v) for a toroidal corner patch.
            plan.acrossIsU = surf.GetType() == GeomAbs_Cylinder;
        }
        // Only a plain 2u+2v rectangle ties its grid to its edges; anything
        // else meshes with its own settings, unconstrained.
        collectIsoEdges(face, model, info.edgeIds, plan);
        if (plan.uEdges.size() != 2 || plan.vEdges.size() != 2) {
            plan.constrains = false;
        }
        return plan;
    }

    plan.kind = MesherKind::Fallback;
    return plan;
}

// ---------------------------------------------------------------------------
// Density matching: a union-find over edges. Each parametric face requires
// all its u-edges to share one subdivision count (and v-edges another), and
// shared edges tie neighbouring faces' groups together. Every face proposes
// its own settings; a group resolves to the max proposal unless an explicit
// per-edge override pins it.

class EdgeGroups {
public:
    explicit EdgeGroups(int edgeCount) : parent_(edgeCount + 1) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    int find(int e) {
        while (parent_[e] != e) e = parent_[e] = parent_[parent_[e]];
        return e;
    }

    void unite(const std::vector<int>& edges) {
        for (size_t i = 1; i < edges.size(); ++i) {
            parent_[find(edges[i])] = find(edges[0]);
        }
    }

private:
    std::vector<int> parent_;
};

struct DensitySolution {
    EdgeGroups groups;
    std::map<int, int> groupCount;  // root edge -> solved divisions

    explicit DensitySolution(int edgeCount) : groups(edgeCount) {}

    // Solved count for an edge, or `fallback` if it never got a proposal.
    int countFor(int edgeId, int fallback) {
        auto it = groupCount.find(groups.find(edgeId));
        return it == groupCount.end() ? fallback : it->second;
    }
};

DensitySolution solveDensity(const Model& model, std::map<int, FacePlan>& plans,
                             const GenerationSettings& settings) {
    DensitySolution sol(model.edgeCount());

    for (const auto& [fid, plan] : plans) {
        if (!plan.constrains) continue;
        sol.groups.unite(plan.uEdges);
        sol.groups.unite(plan.vEdges);
    }

    auto propose = [&](const std::vector<int>& edges, int count) {
        if (edges.empty()) return;
        int root = sol.groups.find(edges[0]);
        auto [it, inserted] = sol.groupCount.try_emplace(root, count);
        if (!inserted) it->second = std::max(it->second, count);
    };
    for (const auto& [fid, plan] : plans) {
        if (!plan.constrains) continue;
        const FaceMeshSettings& s = settings.forFace(fid);
        if (plan.kind == MesherKind::PlanarGrid ||
            plan.kind == MesherKind::RingJunction) {
            int nu = std::max(1, plan.isFillet && plan.acrossIsU
                                     ? s.filletLoops : s.gridU);
            int nv = std::max(1, plan.isFillet && !plan.acrossIsU
                                     ? s.filletLoops : s.gridV);
            propose(plan.uEdges, nu);
            propose(plan.vEdges, nv);
        } else {  // revolution sides and disk caps subdivide rings radially
            propose(plan.uEdges, std::max(3, s.radial));
            propose(plan.vEdges, std::max(1, s.axial));
        }
    }

    // Explicit per-edge overrides pin their whole group (max if several).
    std::map<int, int> pinned;
    for (const auto& [eid, count] : settings.perEdge) {
        if (eid < 1 || eid > model.edgeCount()) continue;
        int root = sol.groups.find(eid);
        auto [it, inserted] = pinned.try_emplace(root, count);
        if (!inserted) it->second = std::max(it->second, count);
    }
    for (const auto& [root, count] : pinned) sol.groupCount[root] = count;

    // Ring junctions close the loop: the circle must take exactly one ring
    // vertex per boundary vertex, so its group count is DERIVED from the
    // boundary — 2*(nu+nv) — and propagates through the group to whatever
    // boss/bore shares that circle ("the plate drives the boss"). A face
    // whose circle is pinned to an incompatible count can't form the
    // pattern and demotes to fallback triangulation.
    for (auto& [fid, plan] : plans) {
        if (plan.kind != MesherKind::RingJunction) continue;
        int nu = sol.countFor(plan.uEdges[0], 1);
        int nv = sol.countFor(plan.vEdges[0], 1);
        int derived = 2 * (nu + nv);
        int root = sol.groups.find(plan.circleEdgeId);
        auto pin = pinned.find(root);
        if (pin != pinned.end() && pin->second != derived) {
            plan.kind = MesherKind::Fallback;
            plan.constrains = false;
            continue;
        }
        sol.groupCount[root] = derived;
    }

    return sol;
}

// ---------------------------------------------------------------------------
// Execution.

// Quad grid over a closed-in-u surface of revolution. Handles a closed v
// (torus) by wrapping rows, and degenerate rows (cone apex, sphere poles)
// by collapsing them to one vertex — the weld pass then turns the adjacent
// quads into triangles.
void meshRevolutionGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        int faceId, int nu, int nv, MeshBuilder& out) {
    nu = std::max(3, nu);
    nv = std::max(1, nv);
    const double u0 = surf.FirstUParameter();
    const double v0 = surf.FirstVParameter();
    const double du = (surf.LastUParameter() - u0) / nu;
    const bool vWrap = surf.IsVClosed();
    const double dv = (surf.LastVParameter() - v0) / nv;
    const int rows = vWrap ? nv : nv + 1;
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    std::vector<std::vector<uint32_t>> ring(rows);
    for (int j = 0; j < rows; ++j) {
        double v = v0 + j * dv;
        std::vector<gp_Pnt> pts(nu);
        bool degenerate = true;
        for (int i = 0; i < nu; ++i) {
            pts[i] = surf.Value(u0 + i * du, v);
            if (i > 0 && pts[i].Distance(pts[0]) > 1e-9) degenerate = false;
        }
        if (degenerate) {
            ring[j].assign(nu, out.addVertex(pts[0], {faceId, u0, v}));
        } else {
            ring[j].resize(nu);
            for (int i = 0; i < nu; ++i) {
                ring[j][i] = out.addVertex(pts[i], {faceId, u0 + i * du, v});
            }
        }
    }

    for (int j = 0; j < nv; ++j) {
        const std::vector<uint32_t>& lo = ring[j];
        const std::vector<uint32_t>& hi = ring[(j + 1) % rows];
        for (int i = 0; i < nu; ++i) {
            int i2 = (i + 1) % nu;
            std::vector<uint32_t> quad{lo[i], lo[i2], hi[i2], hi[i]};
            // Collapse repeats now so degenerate rows emit clean triangles.
            quad.erase(std::unique(quad.begin(), quad.end()), quad.end());
            if (quad.size() > 1 && quad.front() == quad.back()) quad.pop_back();
            if (quad.size() < 3) continue;
            out.addPolygon(std::move(quad), faceId, flip);
        }
    }
}

// Outward normal of a planar face (accounts for face orientation).
gp_Vec planarFaceNormal(const TopoDS_Face& face, const BRepAdaptor_Surface& surf) {
    gp_Vec n(surf.Plane().Axis().Direction());
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    return n;
}

void meshDiskCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const gp_Circ& circ, int faceId, int n, CapStyle cap,
                 MeshBuilder& out) {
    n = std::max(3, n);
    const gp_Pln pln = surf.Plane();
    auto planeAnchor = [&](const gp_Pnt& p) {
        Anchor a{faceId, 0.0, 0.0};
        ElSLib::Parameters(pln, p, a.u, a.v);
        return a;
    };
    // Ring points come from the circle's own parametrization so they land on
    // the same positions as an adjacent revolution side sharing this circle;
    // the weld pass then stitches the two faces watertight.
    std::vector<uint32_t> ring(n);
    std::vector<gp_Pnt> pts(n);
    for (int i = 0; i < n; ++i) {
        pts[i] = ElCLib::Value(i * 2.0 * M_PI / n, circ);
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

// Concentric quad rings between a hole circle and the rectangular border
// of a planar face. The circle takes one vertex per boundary vertex (the
// solver guarantees ring count == 2*(nu+nv)); circle vertices reuse the
// circle's own parametrization so the adjacent boss/bore welds watertight,
// and border vertices sit on the same grid nodes as the neighbouring faces.
void meshRingJunction(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                      const gp_Circ& circ, int faceId, int nu, int nv,
                      int loops, MeshBuilder& out) {
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
    for (int k = 0; k < n; ++k) ring[k] = ElCLib::Value(k * 2.0 * M_PI / n, circ);

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

// Last resort: OCCT chord-tolerance triangulation of the single face.
void meshFallback(const TopoDS_Face& face, int faceId, const FaceMeshSettings& s,
                  MeshBuilder& out) {
    BRepMesh_IncrementalMesh mesher(face, s.chordTolerance);
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) return;

    const bool flip = face.Orientation() == TopAbs_REVERSED;
    const bool hasUV = tri->HasUVNodes();
    std::vector<uint32_t> verts(tri->NbNodes());
    for (int i = 1; i <= tri->NbNodes(); ++i) {
        Anchor a;
        if (hasUV) {
            gp_Pnt2d uv = tri->UVNode(i);
            a = {faceId, uv.X(), uv.Y()};
        }
        verts[i - 1] = out.addVertex(
            tri->Node(i).Transformed(loc.Transformation()), a);
    }
    for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int a, b, c;
        tri->Triangle(i).Get(a, b, c);
        out.addPolygon({verts[a - 1], verts[b - 1], verts[c - 1]}, faceId, flip);
    }
}

}  // namespace

PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings, GenerationReport* report) {
    std::map<int, FacePlan> plans;
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        plans.emplace(fid, planFace(fid, model, analysis, settings));
    }

    DensitySolution density = solveDensity(model, plans, settings);

    PolyMesh mesh;
    MeshBuilder out(mesh);
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FaceMeshSettings& s = settings.forFace(fid);
        const FacePlan& plan = plans.at(fid);
        BRepAdaptor_Surface surf(face);

        auto solved = [&](const std::vector<int>& edges, int fallback) {
            return edges.empty() ? fallback
                                 : density.countFor(edges[0], fallback);
        };

        switch (plan.kind) {
            case MesherKind::RevolutionGrid:
                meshRevolutionGrid(face, surf, fid, solved(plan.uEdges, s.radial),
                                   solved(plan.vEdges, s.axial), out);
                break;
            case MesherKind::DiskCap:
                meshDiskCap(face, surf, plan.circ, fid,
                            solved(plan.uEdges, s.radial), s.cap, out);
                break;
            case MesherKind::PlanarGrid: {
                int defU = plan.isFillet && plan.acrossIsU ? s.filletLoops
                                                           : s.gridU;
                int defV = plan.isFillet && !plan.acrossIsU ? s.filletLoops
                                                            : s.gridV;
                int nu = plan.constrains ? solved(plan.uEdges, defU)
                                         : std::max(1, defU);
                int nv = plan.constrains ? solved(plan.vEdges, defV)
                                         : std::max(1, defV);
                // Support loops hug the creases on fillet strips.
                double holdU = plan.isFillet && plan.acrossIsU ? s.filletHold : 0;
                double holdV = plan.isFillet && !plan.acrossIsU ? s.filletHold : 0;
                meshParametricGrid(face, surf, fid, clusteredParams(nu, holdU),
                                   clusteredParams(nv, holdV), out);
                break;
            }
            case MesherKind::RingJunction:
                meshRingJunction(face, surf, plan.circ, fid,
                                 solved(plan.uEdges, s.gridU),
                                 solved(plan.vEdges, s.gridV), s.junctionRings,
                                 out);
                break;
            case MesherKind::Fallback:
                meshFallback(face, fid, s, out);
                break;
        }

        if (report) {
            report->faceMesher[fid] = plan.kind;
            if (plan.constrains) {
                for (int eid : plan.uEdges) {
                    report->edgeDivisions[eid] = density.countFor(eid, 0);
                }
                for (int eid : plan.vEdges) {
                    report->edgeDivisions[eid] = density.countFor(eid, 0);
                }
                if (plan.circleEdgeId > 0) {
                    report->edgeDivisions[plan.circleEdgeId] =
                        density.countFor(plan.circleEdgeId, 0);
                }
            }
        }
    }

    weldVertices(mesh, settings.weldTolerance);
    return mesh;
}

}  // namespace weft
