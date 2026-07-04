#include "weft/meshers.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <ElCLib.hxx>
#include <ElSLib.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_HArray1OfReal.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <numeric>
#include <set>
#include <unordered_map>
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
        case MesherKind::QuadDominant: return "quad-dominant";
        case MesherKind::MinimalNGon: return "minimal-ngon";
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
        groups_.push_back(currentGroup_);
        return static_cast<uint32_t>(mesh_.vertices.size() - 1);
    }

    void addPolygon(std::vector<uint32_t> indices, int faceId, bool flip) {
        if (flip) std::reverse(indices.begin(), indices.end());
        mesh_.polygons.push_back(std::move(indices));
        mesh_.polygonFaceId.push_back(faceId);
        mesh_.polygonPartId.push_back(currentPart_);
    }

    // Which solid the vertices being emitted belong to; welding never
    // merges across groups, so touching assembly parts stay separate.
    void setGroup(int g) { currentGroup_ = g; }
    // Dense 1-based part id stamped on emitted polygons (exporters split
    // assemblies by part).
    void setPart(int p) { currentPart_ = p; }
    const std::vector<int>& groups() const { return groups_; }

    size_t vertexCount() const { return mesh_.vertices.size(); }
    gp_Pnt vertex(size_t i) const {
        const auto& v = mesh_.vertices[i];
        return gp_Pnt(v[0], v[1], v[2]);
    }

private:
    PolyMesh& mesh_;
    std::vector<int> groups_;
    int currentGroup_ = 0;
    int currentPart_ = 1;
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
    // Angular span (degrees) of each parametric direction when the surface
    // curves along it (cylinder/cone/sphere/torus strips), else 0. Floors
    // the default grid density so a 180-degree bend never meshes as a
    // handful of flat quads (curvature-adaptive default, plan §4.2).
    double uSpanDeg = 0.0;
    double vSpanDeg = 0.0;
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

// Endpoints of an edge's pcurve on a face (plus its midpoint, so curved
// pcurves can't fake a straight span).
bool pcurveSpan(const TopoDS_Edge& edge, const TopoDS_Face& face,
                gp_Pnt2d& a, gp_Pnt2d& b) {
    double f = 0, l = 0;
    Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, f, l);
    if (pcurve.IsNull()) return false;
    a = pcurve->Value(f);
    b = pcurve->Value(l);
    return true;
}

// A grid mesher's boundary contract: its u-edges must be the full bottom
// and top of the face's UV rectangle and its v-edges the full left and
// right. Anything short of that (split sides, partial rims, trims that
// stop early) means the uniform grid would NOT land its border vertices
// on the actual boundary — the face must fall back to conformal
// triangulation instead of silently meshing over the mismatch.
bool isoEdgesFormRectangle(const TopoDS_Face& face, const Model& model,
                           const FacePlan& plan, bool uClosed) {
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double uTol = std::max(1e-12, (umax - umin) * 1e-4);
    const double vTol = std::max(1e-12, (vmax - vmin) * 1e-4);

    bool haveVMin = false, haveVMax = false;
    for (int eid : plan.uEdges) {
        gp_Pnt2d a, b;
        if (!pcurveSpan(TopoDS::Edge(model.edges(eid)), face, a, b)) return false;
        double lo = std::min(a.X(), b.X()), hi = std::max(a.X(), b.X());
        if (std::abs(lo - umin) > uTol || std::abs(hi - umax) > uTol) return false;
        double v = 0.5 * (a.Y() + b.Y());
        if (std::abs(v - vmin) <= vTol) haveVMin = true;
        else if (std::abs(v - vmax) <= vTol) haveVMax = true;
        else return false;
    }
    bool haveUMin = false, haveUMax = false;
    for (int eid : plan.vEdges) {
        gp_Pnt2d a, b;
        if (!pcurveSpan(TopoDS::Edge(model.edges(eid)), face, a, b)) return false;
        double lo = std::min(a.Y(), b.Y()), hi = std::max(a.Y(), b.Y());
        if (std::abs(lo - vmin) > vTol || std::abs(hi - vmax) > vTol) return false;
        double u = 0.5 * (a.X() + b.X());
        if (std::abs(u - umin) <= uTol) haveUMin = true;
        else if (std::abs(u - umax) <= uTol) haveUMax = true;
        else return false;
    }
    if (uClosed) return true;  // revolutions: rims/seams may legitimately
                               // be absent (poles, closed v); presence of a
                               // partial rim was already rejected above
    return haveVMin && haveVMax && haveUMin && haveUMax &&
           plan.uEdges.size() == 2 && plan.vEdges.size() == 2;
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
    if (!plan.constrains || plan.uEdges.size() != 2 || plan.vEdges.size() != 2 ||
        !isoEdgesFormRectangle(face, model, plan, /*uClosed=*/false)) {
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
        // A revolution face whose boundary is anything but full rims (and
        // an optional seam) — a slanted trim, a hole through the wall, a
        // split rim — cannot be covered by the uniform grid without meshing
        // over its trims. Fall back to conformal triangulation.
        if (!plan.constrains ||
            !isoEdgesFormRectangle(face, model, plan, /*uClosed=*/true)) {
            plan = FacePlan{};
            plan.kind = MesherKind::Fallback;
        }
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
        // Flat panels collapse to one boundary n-gon on request; the border
        // still carries the density-matched vertices, so neighbours weld.
        if (s.minimal && surf.GetType() == GeomAbs_Plane) {
            plan.kind = MesherKind::MinimalNGon;
        }
        if (info.isFillet) {
            plan.isFillet = true;
            // The blend arc runs along u for a cylinder strip and along the
            // minor circle (v) for a toroidal corner patch.
            plan.acrossIsU = surf.GetType() == GeomAbs_Cylinder;
        }
        {
            double umin, umax, vmin, vmax;
            BRepTools::UVBounds(face, umin, umax, vmin, vmax);
            const double toDeg = 180.0 / M_PI;
            switch (surf.GetType()) {
                case GeomAbs_Cylinder:
                case GeomAbs_Cone:
                    plan.uSpanDeg = (umax - umin) * toDeg;
                    break;
                case GeomAbs_Sphere:
                case GeomAbs_Torus:
                    plan.uSpanDeg = (umax - umin) * toDeg;
                    plan.vSpanDeg = (vmax - vmin) * toDeg;
                    break;
                default:
                    break;
            }
        }
        // Only a plain 2u+2v rectangle ties its grid to its edges. A face
        // that passed the containment probe but whose boundary is NOT the
        // exact UV rectangle (split sides, slanted edges) would mesh its
        // bounding rectangle at private divisions and crack against every
        // neighbour — demote it to conformal triangulation instead.
        collectIsoEdges(face, model, info.edgeIds, plan);
        if (plan.uEdges.size() != 2 || plan.vEdges.size() != 2 ||
            !isoEdgesFormRectangle(face, model, plan, /*uClosed=*/false)) {
            plan.constrains = false;
        }
        if (!plan.constrains) {
            plan = FacePlan{};
            plan.kind = MesherKind::Fallback;
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

    // A face with an explicit per-face override PINS its groups: the user
    // asked for that density by name, so it must not be silently outvoted
    // by neighbours' defaults. Groups touched only by defaulted faces
    // resolve to the max proposal as before; several overrides sharing a
    // group still resolve by max among themselves.
    std::map<int, int> facePinned;
    auto propose = [&](const std::vector<int>& edges, int count,
                       bool overridden) {
        if (edges.empty()) return;
        int root = sol.groups.find(edges[0]);
        auto [it, inserted] = sol.groupCount.try_emplace(root, count);
        if (!inserted) it->second = std::max(it->second, count);
        if (overridden) {
            auto [pit, pIns] = facePinned.try_emplace(root, count);
            if (!pIns) pit->second = std::max(pit->second, count);
        }
    };
    for (const auto& [fid, plan] : plans) {
        if (!plan.constrains) continue;
        const FaceMeshSettings& s = settings.forFace(fid);
        const bool overridden = settings.perFace.count(fid) > 0;
        if (plan.kind == MesherKind::PlanarGrid ||
            plan.kind == MesherKind::MinimalNGon ||
            plan.kind == MesherKind::RingJunction) {
            int nu = std::max(1, plan.isFillet && plan.acrossIsU
                                     ? s.filletLoops : s.gridU);
            int nv = std::max(1, plan.isFillet && !plan.acrossIsU
                                     ? s.filletLoops : s.gridV);
            // Defaults are generic; a 4-division grid says nothing about a
            // 180-degree bend. Floor default densities by the direction's
            // angular span so curved strips honour the angle tolerance.
            // Explicit per-face overrides mean exactly what they say.
            if (!overridden) {
                const double tol = std::max(5.0, s.angleToleranceDeg);
                if (plan.uSpanDeg > 0) {
                    nu = std::max(nu, (int)std::ceil(plan.uSpanDeg / tol));
                }
                if (plan.vSpanDeg > 0) {
                    nv = std::max(nv, (int)std::ceil(plan.vSpanDeg / tol));
                }
            }
            propose(plan.uEdges, nu, overridden);
            propose(plan.vEdges, nv, overridden);
        } else {  // revolution sides and disk caps subdivide rings radially
            propose(plan.uEdges, std::max(3, s.radial), overridden);
            propose(plan.vEdges, std::max(1, s.axial), overridden);
        }
    }
    for (const auto& [root, count] : facePinned) sol.groupCount[root] = count;

    // Explicit per-edge overrides pin their whole group (max if several),
    // winning over both defaults and per-face overrides.
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

// ---------------------------------------------------------------------------
// Conformal boundaries for triangulated faces (plan §7.1, first real slice).
//
// The whole shape is triangulated ONCE, so OCCT discretizes every B-rep
// edge once and both adjacent triangulations share its polyline — trimmed
// faces meet vertex-for-vertex out of the box. Edges that a parametric
// mesher controls (a cylinder rim next to a trimmed plate) instead carry a
// *canonical polyline* — the exact vertices the parametric side emitted —
// and the triangulated side's boundary is surgically conformed to it:
// canonical points are inserted (triangle splits), leftover triangulation
// boundary points are collapsed into them. The result is watertight
// without giving up exact division control.

struct EdgePolyline {
    std::vector<double> params;  // ascending along the edge curve
    std::vector<gp_Pnt> pts;     // exact positions the neighbour emitted
    // True when a parametric mesher authored this polyline: the fallback
    // side must then keep these segments unsplit during quad subdivision
    // (the parametric side has no midpoints to meet).
    bool fromParametric = false;
    // Closed edge whose canonical points don't include the curve's seam
    // parameter (the parametric side's u-origin is phase-shifted from the
    // curve origin): the polyline is an open run of interior points and
    // the chain's own seam node must be collapsed away, closing the loop
    // through the wrap segment between last and first canonical points.
    bool closedLoop = false;
};

// Project emitted vertices of a parametric face onto one of its edges and
// order them along it. Returns false when the collection doesn't look like
// a full chain (endpoints missing), in which case no surgery happens.
bool collectEmittedPolyline(const TopoDS_Edge& edge, MeshBuilder& out,
                            size_t vBegin, size_t vEnd, EdgePolyline& poly) {
    double f = 0, l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
    if (curve.IsNull()) return false;
    const gp_Pnt pf = curve->Value(f);
    const gp_Pnt pl = curve->Value(l);
    const gp_Pnt pm = curve->Value(0.5 * (f + l));
    // Real-world edges sit within their *edge tolerance* of the adjacent
    // surfaces — that is what the tolerance means — and the parametric
    // side's vertices are evaluated ON its surface. So the acceptance
    // window must scale with the edge tolerance and the edge size, not
    // with Precision::Confusion(); a Confusion-sized window silently
    // rejects every emitted vertex on dirty CAD and leaves the crack in
    // place. Interior vertices sit a full grid cell away, orders of
    // magnitude beyond this window.
    const double approxLen = pf.Distance(pm) + pm.Distance(pl);
    const double tol = std::max({BRep_Tool::Tolerance(edge) * 10.0,
                                 approxLen * 1e-4, 1e-6});

    const bool closed = pf.Distance(pl) <= tol;
    struct Hit {
        double param;
        gp_Pnt pnt;
        double dist;
    };
    std::vector<Hit> hits;
    for (size_t i = vBegin; i < vEnd; ++i) {
        gp_Pnt p = out.vertex(i);
        // Endpoints explicitly: the projector reports true extrema only
        // and can miss the curve ends entirely.
        const double df = p.Distance(pf);
        if (df <= tol) {
            hits.push_back({f, p, df});
            if (closed) hits.push_back({l, p, df});
            continue;
        }
        const double dl = p.Distance(pl);
        if (dl <= tol) {
            hits.push_back({l, p, dl});
            continue;
        }
        GeomAPI_ProjectPointOnCurve proj(p, curve, f, l);
        if (proj.NbPoints() < 1 || proj.LowerDistance() > tol) continue;
        double t = proj.LowerDistanceParameter();
        hits.push_back({std::min(std::max(t, f), l), p, proj.LowerDistance()});
    }
    if (hits.size() < 2) return false;
    std::sort(hits.begin(), hits.end(),
              [](const Hit& a, const Hit& b) { return a.param < b.param; });
    // One point per parameter cluster. Duplicates come from the seam
    // column of a closed grid (same point twice — either survives) and,
    // on narrow strips whose width is inside the dirty-CAD tolerance
    // window, from the NEXT row of the grid projecting onto the edge —
    // there the truly-on-curve vertex must win, so keep the closest.
    std::vector<std::pair<double, gp_Pnt>> unique;
    double bestDist = 0.0;
    for (const Hit& h : hits) {
        if (!unique.empty() &&
            (h.param - unique.back().first) < (l - f) * 1e-3) {
            if (h.dist < bestDist) {
                unique.back() = {unique.back().first, h.pnt};
                bestDist = h.dist;
            }
            continue;
        }
        unique.push_back({h.param, h.pnt});
        bestDist = h.dist;
    }

    const double endTol = (l - f) * 1e-5;
    const bool haveF = std::abs(unique.front().first - f) <= endTol;
    const bool haveL = std::abs(unique.back().first - l) <= endTol;
    if (closed) {
        // A closed rim: when the chain starts at f it must close back at l
        // with the same point. Whichever end the seam point landed on,
        // mirror it to the other. When the parametric side's u-origin is
        // phase-shifted from the curve origin (neither end matches), keep
        // the open run of interior points and let conformChain collapse
        // the triangulation's own seam node — the loop then closes through
        // the wrap segment between last and first canonical points.
        if (haveF && !haveL) {
            unique.push_back({l, unique.front().second});
        } else if (haveL && !haveF) {
            unique.insert(unique.begin(), {f, unique.back().second});
        } else if (!haveF && !haveL) {
            poly.closedLoop = true;
        }
    } else {
        if (!haveF || !haveL) return false;
    }
    if (unique.size() < 2) return false;
    poly.params.clear();
    poly.pts.clear();
    for (const auto& [t, p] : unique) {
        poly.params.push_back(t);
        poly.pts.push_back(p);
    }
    poly.fromParametric = true;
    return true;
}

// Local, mutable copy of a face triangulation.
struct TriSoup {
    std::vector<gp_Pnt> pts;
    std::vector<gp_Pnt2d> uvs;
    bool hasUV = false;
    std::vector<std::array<int, 3>> tris;
};

struct BoundaryChain {
    int edgeId = 0;
    std::vector<int> nodes;      // soup node ids, ordered along the edge
    std::vector<double> params;  // ascending edge-curve parameters
    bool fromParametric = false;
    // Conformed to a phase-shifted closed rim: the chain is the open run
    // of canonical points and the boundary closes through the wrap
    // segment (nodes.back() -> nodes.front()).
    bool closedLoop = false;
};

// The single triangle owning a boundary segment (a,b), with the segment's
// direction inside it. Returns -1 when not found.
int findBoundaryTriangle(const TriSoup& s, int a, int b, int& ia, int& ib) {
    for (size_t t = 0; t < s.tris.size(); ++t) {
        const auto& tr = s.tris[t];
        for (int i = 0; i < 3; ++i) {
            int x = tr[i], y = tr[(i + 1) % 3];
            if ((x == a && y == b) || (x == b && y == a)) {
                ia = x;
                ib = y;
                return static_cast<int>(t);
            }
        }
    }
    return -1;
}

// Make `chain` reproduce `target` exactly: snap coincident points to the
// canonical positions, split triangles to insert missing canonical points,
// collapse leftover boundary points into their nearest canonical neighbour.
void conformChain(TriSoup& s, BoundaryChain& chain, const EdgePolyline& target,
                  const Handle(Geom2d_Curve)& pcurve) {
    if (chain.nodes.size() < 2 || target.params.size() < 2) return;
    const bool loop = target.closedLoop;
    const double range = std::max(1e-12, chain.params.back() - chain.params.front());
    const double epsP = range * 1e-5;

    // Fast path: same discretization — just adopt the canonical positions.
    if (!loop && chain.nodes.size() == target.params.size()) {
        bool same = true;
        for (size_t i = 0; i < chain.params.size(); ++i) {
            if (std::abs(chain.params[i] - target.params[i]) > epsP) {
                same = false;
                break;
            }
        }
        if (same) {
            for (size_t i = 0; i < chain.nodes.size(); ++i) {
                s.pts[chain.nodes[i]] = target.pts[i];
                chain.params[i] = target.params[i];
            }
            return;
        }
    }

    std::vector<bool> keep(chain.nodes.size(), false);
    if (!loop) {
        // Open (or seam-aligned closed) edge: endpoints correspond.
        s.pts[chain.nodes.front()] = target.pts.front();
        s.pts[chain.nodes.back()] = target.pts.back();
        keep.front() = keep.back() = true;
    }
    // Phase-shifted closed rim: every target point is interior; the
    // chain's own seam node gets collapsed once the run is in place.

    // Insert (or claim) every interior canonical point.
    for (size_t j = loop ? 0 : 1;
         j + (loop ? 0 : 1) < target.params.size(); ++j) {
        const double t = target.params[j];
        size_t k = 0;
        while (k + 1 < chain.params.size() && chain.params[k + 1] < t - epsP) ++k;
        if (k + 1 < chain.params.size() &&
            std::abs(chain.params[k + 1] - t) <= epsP &&
            k + 2 < chain.params.size() + 1) {
            // Claim an existing boundary node.
            size_t idx = k + 1;
            if (idx + 1 < chain.nodes.size()) {  // interior only
                s.pts[chain.nodes[idx]] = target.pts[j];
                chain.params[idx] = t;
                if (s.hasUV && !pcurve.IsNull()) {
                    s.uvs[chain.nodes[idx]] = pcurve->Value(t);
                }
                keep[idx] = true;
                continue;
            }
        }
        // Split the bracketing segment's triangle.
        int a = chain.nodes[k], b = chain.nodes[k + 1];
        int ia = 0, ib = 0;
        int tIdx = findBoundaryTriangle(s, a, b, ia, ib);
        if (tIdx < 0) continue;  // torn triangulation; skip this point
        int q = static_cast<int>(s.pts.size());
        s.pts.push_back(target.pts[j]);
        s.uvs.push_back(s.hasUV && !pcurve.IsNull() ? pcurve->Value(t)
                                                    : gp_Pnt2d(0, 0));
        std::array<int, 3> old = s.tris[tIdx];
        int c = old[0] + old[1] + old[2] - ia - ib;
        // Preserve winding: (ia, ib, c) -> (ia, q, c) + (q, ib, c).
        s.tris[tIdx] = {ia, q, c};
        s.tris.push_back({q, ib, c});
        chain.nodes.insert(chain.nodes.begin() + k + 1, q);
        chain.params.insert(chain.params.begin() + k + 1, t);
        keep.insert(keep.begin() + k + 1, true);
    }

    // Collapse boundary nodes that aren't canonical into the nearest kept
    // neighbour along the chain.
    for (size_t k = 1; k + 1 < chain.nodes.size();) {
        if (keep[k]) { ++k; continue; }
        size_t left = k - 1;
        size_t right = k + 1;
        while (right + 1 < chain.nodes.size() && !keep[right]) ++right;
        size_t into = (chain.params[k] - chain.params[left] <=
                       chain.params[right] - chain.params[k])
                          ? left
                          : right;
        int dead = chain.nodes[k], live = chain.nodes[into];
        for (auto& tr : s.tris) {
            for (int& v : tr) {
                if (v == dead) v = live;
            }
        }
        chain.nodes.erase(chain.nodes.begin() + k);
        chain.params.erase(chain.params.begin() + k);
        keep.erase(keep.begin() + k);
        if (into == left) { /* k now points at the next candidate */ }
    }

    // Phase-shifted closed rim: retire the triangulation's seam node —
    // the boundary then closes through the wrap segment between the last
    // and first canonical points, exactly like the parametric side's own
    // quads across its seam.
    if (loop && chain.nodes.size() >= 4) {
        const int seamA = chain.nodes.front();
        const int seamB = chain.nodes.back();
        const double toFirst = chain.params[1] - chain.params.front();
        const double toLast = chain.params.back() -
                              chain.params[chain.params.size() - 2];
        const int live = toFirst <= toLast
                             ? chain.nodes[1]
                             : chain.nodes[chain.nodes.size() - 2];
        for (auto& tr : s.tris) {
            for (int& v : tr) {
                if (v == seamA || v == seamB) v = live;
            }
        }
        chain.nodes.erase(chain.nodes.begin());
        chain.params.erase(chain.params.begin());
        chain.nodes.pop_back();
        chain.params.pop_back();
        chain.closedLoop = true;
    }

    // Drop triangles the collapses degenerated.
    s.tris.erase(std::remove_if(s.tris.begin(), s.tris.end(),
                                [](const std::array<int, 3>& tr) {
                                    return tr[0] == tr[1] || tr[1] == tr[2] ||
                                           tr[0] == tr[2];
                                }),
                 s.tris.end());
}

struct SegInfo {
    int edgeId = 0;
    double t0 = 0, t1 = 0;  // edge-curve parameters of the segment ends
    bool noSplit = false;   // parametric neighbour: leave the segment whole
};

double minAngle3d(const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c) {
    gp_Vec ab(a, b), bc(b, c), ca(c, a);
    if (ab.Magnitude() < 1e-15 || bc.Magnitude() < 1e-15 ||
        ca.Magnitude() < 1e-15) {
        return 0.0;
    }
    double m = ab.Angle(ca.Reversed());
    m = std::min(m, bc.Angle(ab.Reversed()));
    m = std::min(m, ca.Angle(bc.Reversed()));
    return m;
}

double cross2d(const gp_Pnt2d& o, const gp_Pnt2d& p, const gp_Pnt2d& q) {
    return (p.X() - o.X()) * (q.Y() - o.Y()) -
           (p.Y() - o.Y()) * (q.X() - o.X());
}

// Lawson-style edge flips toward better-shaped triangles. Boundary
// conformity surgery splits whatever triangle happens to own each border
// segment, which piles up sliver fans; flipping interior diagonals (where
// the UV quad is convex and the 3D minimum angle improves) restores a
// near-Delaunay interior that the quad pairing can work with. Boundary
// segments have a single owner and are never candidates.
void flipToDelaunay(TriSoup& s) {
    if (!s.hasUV) return;
    for (int pass = 0; pass < 10; ++pass) {
        std::map<std::pair<int, int>, std::vector<int>> owners;
        for (size_t t = 0; t < s.tris.size(); ++t) {
            for (int i = 0; i < 3; ++i) {
                int a = s.tris[t][i], b = s.tris[t][(i + 1) % 3];
                owners[a < b ? std::make_pair(a, b) : std::make_pair(b, a)]
                    .push_back(static_cast<int>(t));
            }
        }
        bool flipped = false;
        for (const auto& [seg, ts] : owners) {
            if (ts.size() != 2) continue;
            auto& T1 = s.tris[ts[0]];
            auto& T2 = s.tris[ts[1]];
            // Earlier flips this pass may have retired the segment from
            // either triangle (the owners map is rebuilt per pass, not per
            // flip) — both must still hold it.
            auto holds = [&](const std::array<int, 3>& tr) {
                int have = 0;
                for (int v : tr) {
                    if (v == seg.first || v == seg.second) ++have;
                }
                return have == 2;
            };
            if (!holds(T1) || !holds(T2)) continue;
            // Orient: T1 traverses a->b, T2 traverses b->a.
            int a = -1, b = -1;
            for (int i = 0; i < 3; ++i) {
                int x = T1[i], y = T1[(i + 1) % 3];
                if ((x == seg.first && y == seg.second) ||
                    (x == seg.second && y == seg.first)) {
                    a = x;
                    b = y;
                    break;
                }
            }
            if (a < 0) continue;
            int c = T1[0] + T1[1] + T1[2] - a - b;
            int d = T2[0] + T2[1] + T2[2] - a - b;
            if (c == d || c == a || c == b || d == a || d == b) continue;

            // The UV quad a-d-b-c must be strictly convex or the flip
            // would fold the parametrization.
            const gp_Pnt2d &ua = s.uvs[a], &ub = s.uvs[b], &uc = s.uvs[c],
                           &ud = s.uvs[d];
            double x1 = cross2d(ua, ud, ub), x2 = cross2d(ud, ub, uc),
                   x3 = cross2d(ub, uc, ua), x4 = cross2d(uc, ua, ud);
            if (!((x1 > 0 && x2 > 0 && x3 > 0 && x4 > 0) ||
                  (x1 < 0 && x2 < 0 && x3 < 0 && x4 < 0))) {
                continue;
            }

            double before = std::min(minAngle3d(s.pts[a], s.pts[b], s.pts[c]),
                                     minAngle3d(s.pts[b], s.pts[a], s.pts[d]));
            double after = std::min(minAngle3d(s.pts[a], s.pts[d], s.pts[c]),
                                    minAngle3d(s.pts[d], s.pts[b], s.pts[c]));
            if (after <= before + 1e-12) continue;

            T1 = {a, d, c};
            T2 = {d, b, c};
            flipped = true;
        }
        if (!flipped) break;
    }
}

// Interior Steiner refinement. Boundary surgery often leaves a face whose
// border is dense (pinned to parametric neighbours) but whose interior is
// the raw boundary-only CDT — long triangles spanning the face that the
// quad pairing can't do anything with. Split interior edges much longer
// than the boundary's own median spacing (midpoints evaluated ON the
// surface through averaged UVs) and re-flip, until edge lengths are
// commensurate. Boundary segments (single-owner) are never touched, so
// conformity — and watertightness — is preserved by construction.
void refineInterior(TriSoup& s, const std::vector<BoundaryChain>& chains,
                    const BRepAdaptor_Surface& surf) {
    if (!s.hasUV) return;
    std::vector<double> blens;
    for (const BoundaryChain& chain : chains) {
        for (size_t i = 0; i + 1 < chain.nodes.size(); ++i) {
            blens.push_back(
                s.pts[chain.nodes[i]].Distance(s.pts[chain.nodes[i + 1]]));
        }
    }
    if (blens.size() < 4) return;
    std::nth_element(blens.begin(), blens.begin() + blens.size() / 2,
                     blens.end());
    const double h = blens[blens.size() / 2];
    if (h <= 1e-12) return;
    const double hi = 1.6 * h;
    const size_t budget = s.pts.size() * 4 + 2048;

    for (int pass = 0; pass < 8 && s.pts.size() < budget; ++pass) {
        std::map<std::pair<int, int>, std::vector<int>> owners;
        for (size_t t = 0; t < s.tris.size(); ++t) {
            for (int i = 0; i < 3; ++i) {
                int a = s.tris[t][i], b = s.tris[t][(i + 1) % 3];
                owners[a < b ? std::make_pair(a, b) : std::make_pair(b, a)]
                    .push_back(static_cast<int>(t));
            }
        }
        bool split = false;
        for (const auto& [seg, ts] : owners) {
            if (ts.size() != 2) continue;  // boundary or non-manifold: skip
            if (s.pts[seg.first].Distance(s.pts[seg.second]) <= hi) continue;
            auto holds = [&](const std::array<int, 3>& tr) {
                int have = 0;
                for (int v : tr) {
                    if (v == seg.first || v == seg.second) ++have;
                }
                return have == 2;
            };
            if (!holds(s.tris[ts[0]]) || !holds(s.tris[ts[1]])) {
                continue;  // stale this pass
            }

            gp_Pnt2d uv(0.5 * (s.uvs[seg.first].X() + s.uvs[seg.second].X()),
                        0.5 * (s.uvs[seg.first].Y() + s.uvs[seg.second].Y()));
            const int m = static_cast<int>(s.pts.size());
            s.pts.push_back(surf.Value(uv.X(), uv.Y()));
            s.uvs.push_back(uv);

            // By index, never by reference: the push_back inside can
            // reallocate s.tris and dangle a second-triangle reference.
            auto splitTri = [&](int tIdx) {
                const std::array<int, 3> tr = s.tris[tIdx];
                for (int i = 0; i < 3; ++i) {
                    int a = tr[i], b = tr[(i + 1) % 3];
                    if ((a == seg.first && b == seg.second) ||
                        (a == seg.second && b == seg.first)) {
                        int c = tr[(i + 2) % 3];
                        s.tris[tIdx] = {a, m, c};
                        s.tris.push_back({m, b, c});
                        return;
                    }
                }
            };
            splitTri(ts[0]);
            splitTri(ts[1]);
            split = true;
            if (s.pts.size() >= budget) break;
        }
        if (!split) break;
        flipToDelaunay(s);
    }
}

// Last resort for trimmed/freeform faces: chord-tolerance triangulation
// (shared across the whole shape), boundary conformed to canonical edge
// polylines, optionally paired into quads. Pairing is greedy over a
// quality score that prefers near-rectangular quads whose edges follow the
// surface's parametric directions — the seed of the plan's guided quad
// flow (§3.5); a real cross-field solver replaces the guidance later.
void meshFallback(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  int faceId, const FaceMeshSettings& s, const Model& model,
                  const std::map<int, EdgePolyline>& canonical,
                  MeshBuilder& out) {
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) {
        BRepMesh_IncrementalMesh mesher(face, s.chordTolerance, Standard_False,
                                        s.angleToleranceDeg * M_PI / 180.0);
        tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) return;
    }

    const bool flip = face.Orientation() == TopAbs_REVERSED;

    TriSoup soup;
    soup.hasUV = tri->HasUVNodes();
    soup.pts.resize(tri->NbNodes());
    soup.uvs.resize(tri->NbNodes(), gp_Pnt2d(0, 0));
    for (int i = 1; i <= tri->NbNodes(); ++i) {
        soup.pts[i - 1] = tri->Node(i).Transformed(loc.Transformation());
        if (soup.hasUV) soup.uvs[i - 1] = tri->UVNode(i);
    }
    soup.tris.resize(tri->NbTriangles());
    for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int a, b, c;
        tri->Triangle(i).Get(a, b, c);
        soup.tris[i - 1] = {a - 1, b - 1, c - 1};
    }

    // Boundary chains per B-rep edge, conformed to canonical polylines.
    std::vector<BoundaryChain> chains;
    std::map<int, Handle(Geom_Curve)> edgeCurve;
    std::set<int> seenEdges;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        const int eid = model.edges.FindIndex(edge);
        if (eid < 1 || !seenEdges.insert(eid).second) continue;
        if (BRep_Tool::Degenerated(edge)) continue;

        // A closed surface's seam edge appears twice in the face and its
        // two polygons live in ONE PolygonOnClosedTriangulation
        // representation, dispatched by edge orientation. Fetch both, or
        // the second seam side gets subdivided as if it were interior and
        // the seam cracks open (observed as exactly one open edge per
        // seam segment, both sides).
        std::vector<Handle(Poly_PolygonOnTriangulation)> reps;
        Handle(Poly_PolygonOnTriangulation) p1 = BRep_Tool::PolygonOnTriangulation(
            TopoDS::Edge(edge.Oriented(TopAbs_FORWARD)), tri, loc);
        if (!p1.IsNull() && p1->NbNodes() >= 2) reps.push_back(p1);
        if (BRep_Tool::IsClosed(edge, face)) {
            Handle(Poly_PolygonOnTriangulation) p2 =
                BRep_Tool::PolygonOnTriangulation(
                    TopoDS::Edge(edge.Oriented(TopAbs_REVERSED)), tri, loc);
            if (!p2.IsNull() && p2 != p1 && p2->NbNodes() >= 2) {
                reps.push_back(p2);
            }
        }
        if (reps.empty()) continue;

        double f = 0, l = 0;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
        edgeCurve[eid] = curve;

        for (const auto& polyOnTri : reps) {
            BoundaryChain chain;
            chain.edgeId = eid;
            const TColStd_Array1OfInteger& nodes = polyOnTri->Nodes();
            for (int i = nodes.Lower(); i <= nodes.Upper(); ++i) {
                chain.nodes.push_back(nodes(i) - 1);
            }
            if (polyOnTri->HasParameters()) {
                const TColStd_Array1OfReal& ps =
                    polyOnTri->Parameters()->Array1();
                for (int i = ps.Lower(); i <= ps.Upper(); ++i) {
                    chain.params.push_back(ps(i));
                }
            } else if (!curve.IsNull()) {
                for (int n : chain.nodes) {
                    GeomAPI_ProjectPointOnCurve proj(soup.pts[n], curve, f, l);
                    chain.params.push_back(
                        proj.NbPoints() ? proj.LowerDistanceParameter() : f);
                }
            } else {
                continue;
            }
            if (chain.params.size() != chain.nodes.size()) continue;
            if (chain.params.front() > chain.params.back()) {
                std::reverse(chain.nodes.begin(), chain.nodes.end());
                std::reverse(chain.params.begin(), chain.params.end());
            }

            auto canIt = canonical.find(eid);
            if (canIt != canonical.end()) {
                double pf = 0, pl = 0;
                Handle(Geom2d_Curve) pcurve =
                    BRep_Tool::CurveOnSurface(edge, face, pf, pl);
                conformChain(soup, chain, canIt->second, pcurve);
                chain.fromParametric = canIt->second.fromParametric;
            }
            chains.push_back(std::move(chain));
        }
    }

    flipToDelaunay(soup);
    if (s.quadDominant && s.interiorRefine) refineInterior(soup, chains, surf);

    // Segment lookup for the subdivision pass: which node pairs lie on a
    // B-rep edge, and whether they may be split.
    std::map<std::pair<int, int>, SegInfo> boundarySeg;
    for (const BoundaryChain& chain : chains) {
        for (size_t i = 0; i + 1 < chain.nodes.size(); ++i) {
            int a = chain.nodes[i], b = chain.nodes[i + 1];
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            boundarySeg[key] = {chain.edgeId, chain.params[i],
                                chain.params[i + 1], chain.fromParametric};
        }
        if (chain.closedLoop && chain.nodes.size() >= 2) {
            // The wrap segment of a phase-shifted rim. Always authored by
            // a parametric neighbour, so it is never split — the params
            // are only bookkeeping.
            int a = chain.nodes.back(), b = chain.nodes.front();
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            boundarySeg[key] = {chain.edgeId, chain.params.back(),
                                chain.params.front(), true};
        }
    }

    // Lazy node -> output vertex mapping (surgery may have orphaned nodes).
    std::vector<int64_t> globalOf(soup.pts.size(), -1);
    auto globalVert = [&](int n) {
        if (globalOf[n] < 0) {
            Anchor a = soup.hasUV
                           ? Anchor{faceId, soup.uvs[n].X(), soup.uvs[n].Y()}
                           : Anchor{};
            globalOf[n] = out.addVertex(soup.pts[n], a);
        }
        return static_cast<uint32_t>(globalOf[n]);
    };

    if (!s.quadDominant) {
        for (const auto& t : soup.tris) {
            out.addPolygon({globalVert(t[0]), globalVert(t[1]), globalVert(t[2])},
                           faceId, flip);
        }
        return;
    }

    const std::vector<gp_Pnt>& pts = soup.pts;
    const std::vector<std::array<int, 3>>& tris = soup.tris;
    const bool hasUV = soup.hasUV;

    // A face bounded entirely by parametric neighbours gets NO midpoint
    // subdivision: none of its border segments may split, so subdividing
    // would only turn the whole border ring into n-gon fans. Plain guided
    // pairing gives cleaner cells, and the choice is safely per-face —
    // any fallback-fallback border segment forces subdivision on BOTH
    // sides (it is non-canonical for both), keeping midpoints paired.
    bool pureParametricBorder = true;
    for (const BoundaryChain& chain : chains) {
        if (!chain.fromParametric) { pureParametricBorder = false; break; }
    }
    if (chains.empty()) pureParametricBorder = false;

    std::vector<std::vector<int>> paired;  // local rings, tris and quads

    // Candidate merges: two triangles sharing an edge form the quad
    // (opp1, a, opp2, b) with the shared diagonal (a,b) removed.
    struct Candidate {
        double cost;
        int t1, t2;
        std::array<int, 4> ring;
    };
    std::map<std::pair<int, int>, std::pair<int, int>> edgeUse;  // edge -> tris
    for (size_t t = 0; t < tris.size(); ++t) {
        for (int i = 0; i < 3; ++i) {
            int a = tris[t][i], b = tris[t][(i + 1) % 3];
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            auto it = edgeUse.find(key);
            if (it == edgeUse.end()) edgeUse[key] = {static_cast<int>(t), -1};
            else it->second.second = static_cast<int>(t);
        }
    }

    auto thirdVertex = [&](int t, int a, int b) {
        for (int v : tris[t]) {
            if (v != a && v != b) return v;
        }
        return -1;
    };

    std::vector<Candidate> candidates;
    for (const auto& [key, owners] : edgeUse) {
        if (owners.second < 0) continue;
        int a = key.first, b = key.second;
        int c1 = thirdVertex(owners.first, a, b);
        int c2 = thirdVertex(owners.second, a, b);
        // Orient the ring with t1's winding: when t1 traverses the shared
        // edge a->b, the merged boundary cycle is c1->a->c2->b; reversed
        // when t1 runs b->a.
        std::array<int, 4> ring{c1, a, c2, b};
        for (int i = 0; i < 3; ++i) {
            if (tris[owners.first][i] == b &&
                tris[owners.first][(i + 1) % 3] == a) {
                ring = {c1, b, c2, a};
                break;
            }
        }
        double cost =
            quadAngleCost({pts[ring[0]], pts[ring[1]], pts[ring[2]],
                           pts[ring[3]]});
        if (cost > 1e8) continue;

        // Guidance: reward quads whose edges follow the parametric
        // directions at the quad center (trivial direction field).
        if (hasUV) {
            gp_Pnt2d uv0 = soup.uvs[ring[0]];
            gp_Pnt2d uv2 = soup.uvs[ring[2]];
            gp_Pnt p;
            gp_Vec du, dv;
            surf.D1(0.5 * (uv0.X() + uv2.X()), 0.5 * (uv0.Y() + uv2.Y()), p,
                    du, dv);
            if (du.Magnitude() > 1e-9 && dv.Magnitude() > 1e-9) {
                gp_Vec e(pts[ring[0]], pts[ring[1]]);
                if (e.Magnitude() > 1e-12) {
                    double alignU = std::abs(e.Normalized().Dot(du.Normalized()));
                    double alignV = std::abs(e.Normalized().Dot(dv.Normalized()));
                    // 0 when aligned with u or v, up to ~20 when diagonal.
                    cost += 20.0 * std::min(1.0, 1.0 - std::max(alignU, alignV));
                }
            }
        }
        candidates.push_back({cost, owners.first, owners.second, ring});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& x, const Candidate& y) {
                  return x.cost < y.cost;
              });

    std::vector<bool> used(tris.size(), false);
    for (const Candidate& c : candidates) {
        if (used[c.t1] || used[c.t2]) continue;
        used[c.t1] = used[c.t2] = true;
        paired.push_back({c.ring[0], c.ring[1], c.ring[2], c.ring[3]});
    }
    for (size_t t = 0; t < tris.size(); ++t) {
        if (used[t]) continue;
        paired.push_back({tris[t][0], tris[t][1], tris[t][2]});
    }

    if (pureParametricBorder) {
        for (const auto& ring : paired) {
            std::vector<uint32_t> poly;
            poly.reserve(ring.size());
            for (int v : ring) poly.push_back(globalVert(v));
            out.addPolygon(std::move(poly), faceId, flip);
        }
        return;
    }

    // One midpoint (Catmull-Clark-style) subdivision turns the paired mesh
    // into (mostly) pure quads: each tri becomes 3, each quad 4. New
    // vertices are evaluated on the surface through averaged UVs, so they
    // sit exactly on the B-rep, not on the chord. Two boundary rules keep
    // the borders watertight:
    //  - segments on a B-rep edge take their midpoint ON the edge curve at
    //    the parameter midpoint, so both adjacent faces create the exact
    //    same vertex;
    //  - segments a parametric mesher authored are never split (it has no
    //    midpoints to meet); the touching cells become n-gons instead.
    auto emitVertex = [&](double u, double v, const gp_Pnt& fallbackPnt) {
        if (!hasUV) return out.addVertex(fallbackPnt, {});
        gp_Pnt p = surf.Value(u, v);
        return out.addVertex(p, {faceId, u, v});
    };
    const std::vector<gp_Pnt2d>& uvs = soup.uvs;
    std::map<std::pair<int, int>, uint32_t> midOf;
    auto segKey = [](int a, int b) {
        return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
    };
    auto splittable = [&](int a, int b) {
        auto it = boundarySeg.find(segKey(a, b));
        return it == boundarySeg.end() || !it->second.noSplit;
    };
    auto midpoint = [&](int a, int b) {
        auto key = segKey(a, b);
        auto it = midOf.find(key);
        if (it != midOf.end()) return it->second;

        uint32_t idx;
        auto seg = boundarySeg.find(key);
        Handle(Geom_Curve) curve;
        if (seg != boundarySeg.end()) {
            auto cIt = edgeCurve.find(seg->second.edgeId);
            if (cIt != edgeCurve.end()) curve = cIt->second;
        }
        if (!curve.IsNull()) {
            // Boundary midpoint: same edge curve, same parameter midpoint
            // from both sides — bitwise-identical, so the weld closes it.
            double tm = 0.5 * (seg->second.t0 + seg->second.t1);
            gp_Pnt p = curve->Value(tm);
            Anchor anchor = hasUV ? Anchor{faceId,
                                           0.5 * (uvs[a].X() + uvs[b].X()),
                                           0.5 * (uvs[a].Y() + uvs[b].Y())}
                                  : Anchor{};
            idx = out.addVertex(p, anchor);
        } else {
            gp_Pnt mid(0.5 * (pts[a].X() + pts[b].X()),
                       0.5 * (pts[a].Y() + pts[b].Y()),
                       0.5 * (pts[a].Z() + pts[b].Z()));
            idx = emitVertex(0.5 * (uvs[a].X() + uvs[b].X()),
                             0.5 * (uvs[a].Y() + uvs[b].Y()), mid);
        }
        midOf[key] = idx;
        return idx;
    };

    for (const auto& ring : paired) {
        const int n = static_cast<int>(ring.size());
        int splits = 0;
        std::vector<bool> split(n);
        for (int i = 0; i < n; ++i) {
            split[i] = splittable(ring[i], ring[(i + 1) % n]);
            if (split[i]) ++splits;
        }

        if (splits == 0) {  // wedged between parametric borders: emit as-is
            std::vector<uint32_t> poly;
            poly.reserve(n);
            for (int v : ring) poly.push_back(globalVert(v));
            out.addPolygon(std::move(poly), faceId, flip);
            continue;
        }
        if (splits == 1) {
            // A single midpoint can't meet a center vertex cleanly; split
            // the cell toward the opposite corner instead.
            int e = 0;
            while (!split[e]) ++e;
            uint32_t m = midpoint(ring[e], ring[(e + 1) % n]);
            int far = (e + 1 + n / 2) % n;
            std::vector<uint32_t> a{m};
            for (int i = (e + 1) % n; i != far; i = (i + 1) % n) {
                a.push_back(globalVert(ring[i]));
            }
            a.push_back(globalVert(ring[far]));
            std::vector<uint32_t> b{m, globalVert(ring[far])};
            for (int i = (far + 1) % n; i != (e + 1) % n; i = (i + 1) % n) {
                b.push_back(globalVert(ring[i]));
            }
            if (a.size() >= 3) out.addPolygon(std::move(a), faceId, flip);
            if (b.size() >= 3) out.addPolygon(std::move(b), faceId, flip);
            continue;
        }

        double cu = 0, cv = 0, cx = 0, cy = 0, cz = 0;
        for (int v : ring) {
            cu += uvs[v].X();
            cv += uvs[v].Y();
            cx += pts[v].X();
            cy += pts[v].Y();
            cz += pts[v].Z();
        }
        uint32_t center =
            emitVertex(cu / n, cv / n, gp_Pnt(cx / n, cy / n, cz / n));

        // Walk the expanded ring from midpoint to midpoint; every arc plus
        // the center is one cell (a quad when the arc holds one corner).
        int start = 0;
        while (!split[start]) ++start;  // splits >= 2 guarantees one
        std::vector<uint32_t> cell{midpoint(ring[start], ring[(start + 1) % n])};
        for (int step = 1; step <= n; ++step) {
            int i = (start + step) % n;
            cell.push_back(globalVert(ring[i]));
            if (split[i]) {
                uint32_t m = midpoint(ring[i], ring[(i + 1) % n]);
                cell.push_back(m);
                cell.push_back(center);
                out.addPolygon(std::move(cell), faceId, flip);
                cell = {m};
            }
        }
        // The loop closes exactly at the starting midpoint.
    }
}

// FaceId -> welding group: connected components of the face-adjacency
// graph (faces sharing a B-rep edge). Faces the topology joins must weld
// — including across solids that share edges in dirty CAD — while parts
// that merely TOUCH share no edges (loadStep does not sew) and stay in
// separate groups, so contact surfaces never fuse into non-manifold
// shells.
std::vector<int> faceWeldGroups(const Model& model) {
    std::vector<int> parent(model.faceCount() + 1);
    std::iota(parent.begin(), parent.end(), 0);
    std::function<int(int)> find = [&](int x) {
        while (parent[x] != x) x = parent[x] = parent[parent[x]];
        return x;
    };
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopTools_ListOfShape& adj = model.edgeToFaces.FindFromIndex(eid);
        int first = 0;
        for (TopTools_ListOfShape::Iterator it(adj); it.More(); it.Next()) {
            int fid = model.faces.FindIndex(it.Value());
            if (fid < 1) continue;
            if (!first) first = fid;
            else parent[find(fid)] = find(first);
        }
    }
    std::vector<int> group(model.faceCount() + 1, 0);
    for (int fid = 1; fid <= model.faceCount(); ++fid) group[fid] = find(fid);
    return group;
}

}  // namespace

PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings, GenerationReport* report) {
    std::map<int, FacePlan> plans;
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        plans.emplace(fid, planFace(fid, model, analysis, settings));
    }

    DensitySolution density = solveDensity(model, plans, settings);

    const std::vector<int> weldGroup = faceWeldGroups(model);
    // Dense 1-based part numbering from the adjacency components.
    std::map<int, int> partOfGroup;
    std::vector<int> partOfFace(weldGroup.size(), 1);
    for (int fid = 1; fid < (int)weldGroup.size(); ++fid) {
        auto [it, inserted] =
            partOfGroup.try_emplace(weldGroup[fid],
                                    (int)partOfGroup.size() + 1);
        partOfFace[fid] = it->second;
    }

    // Triangulate the whole shape ONCE, so OCCT discretizes each B-rep edge
    // once and neighbouring trimmed faces share their border polylines.
    bool anyFallback = false;
    for (const auto& [fid, plan] : plans) {
        if (plan.kind == MesherKind::Fallback) anyFallback = true;
    }
    std::map<int, EdgePolyline> canonical;
    if (anyFallback) {
        const FaceMeshSettings& d = settings.defaults;
        BRepTools::Clean(model.shape);  // stale caches would pin densities
        BRepMesh_IncrementalMesh mesher(model.shape, d.chordTolerance,
                                        Standard_False,
                                        d.angleToleranceDeg * M_PI / 180.0,
                                        Standard_True);

        // Per-face chord/angle overrides re-triangulate just that face —
        // capture its edges' shared polylines first, so the override face
        // can be conformed back onto the borders its neighbours still use.
        std::vector<int> overridden;
        for (const auto& [fid, s] : settings.perFace) {
            auto it = plans.find(fid);
            if (it == plans.end() || it->second.kind != MesherKind::Fallback) {
                continue;
            }
            if (s.chordTolerance == d.chordTolerance &&
                s.angleToleranceDeg == d.angleToleranceDeg) {
                continue;
            }
            overridden.push_back(fid);
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            TopLoc_Location loc;
            Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
            if (tri.IsNull()) continue;
            for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
                int eid = model.edges.FindIndex(edge);
                if (eid < 1 || canonical.count(eid)) continue;
                if (BRep_Tool::Degenerated(edge)) continue;
                Handle(Poly_PolygonOnTriangulation) p =
                    BRep_Tool::PolygonOnTriangulation(edge, tri, loc);
                if (p.IsNull() || p->NbNodes() < 2 || !p->HasParameters()) {
                    continue;
                }
                EdgePolyline poly;
                const TColStd_Array1OfInteger& nodes = p->Nodes();
                const TColStd_Array1OfReal& ps = p->Parameters()->Array1();
                for (int i = nodes.Lower(); i <= nodes.Upper(); ++i) {
                    poly.pts.push_back(
                        tri->Node(nodes(i)).Transformed(loc.Transformation()));
                }
                for (int i = ps.Lower(); i <= ps.Upper(); ++i) {
                    poly.params.push_back(ps(i));
                }
                if (poly.params.front() > poly.params.back()) {
                    std::reverse(poly.params.begin(), poly.params.end());
                    std::reverse(poly.pts.begin(), poly.pts.end());
                }
                canonical[eid] = std::move(poly);
            }
        }
        for (int fid : overridden) {
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            const FaceMeshSettings& s = settings.forFace(fid);
            BRepTools::Clean(face);
            BRepMesh_IncrementalMesh remesh(face, s.chordTolerance,
                                            Standard_False,
                                            s.angleToleranceDeg * M_PI / 180.0);
        }
    }

    PolyMesh mesh;
    MeshBuilder out(mesh);

    // Phase 1: parametric meshers. Their emitted border vertices are the
    // canonical polylines the triangulated faces must conform to.
    std::map<int, std::pair<size_t, size_t>> emitted;  // fid -> vertex range
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FaceMeshSettings& s = settings.forFace(fid);
        const FacePlan& plan = plans.at(fid);
        if (plan.kind == MesherKind::Fallback) continue;
        BRepAdaptor_Surface surf(face);
        out.setGroup(weldGroup[fid]);
        out.setPart(partOfFace[fid]);
        const size_t vBegin = out.vertexCount();

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
                int nu = solved(plan.uEdges, defU);
                int nv = solved(plan.vEdges, defV);
                // Support loops hug the creases on fillet strips.
                double holdU = plan.isFillet && plan.acrossIsU ? s.filletHold : 0;
                double holdV = plan.isFillet && !plan.acrossIsU ? s.filletHold : 0;
                meshParametricGrid(face, surf, fid, clusteredParams(nu, holdU),
                                   clusteredParams(nv, holdV), out);
                break;
            }
            case MesherKind::MinimalNGon:
                meshMinimalNGon(face, surf, fid, solved(plan.uEdges, s.gridU),
                                solved(plan.vEdges, s.gridV), out);
                break;
            case MesherKind::RingJunction:
                meshRingJunction(face, surf, plan.circ, fid,
                                 solved(plan.uEdges, s.gridU),
                                 solved(plan.vEdges, s.gridV), s.junctionRings,
                                 out);
                break;
            case MesherKind::QuadDominant:
            case MesherKind::Fallback:
                break;  // phase 2
        }
        emitted[fid] = {vBegin, out.vertexCount()};

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

    // Phase 1.5: canonical polylines for every edge where a parametric face
    // meets a triangulated one — collected from what the parametric side
    // actually emitted, so the conformed border welds exactly.
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        int parametricFid = 0;
        bool hasFallback = false;
        const TopTools_ListOfShape& adj =
            model.edgeToFaces.FindFromKey(model.edges(eid));
        for (TopTools_ListOfShape::Iterator it(adj); it.More(); it.Next()) {
            int fid = model.faces.FindIndex(it.Value());
            if (fid < 1) continue;
            if (plans.at(fid).kind == MesherKind::Fallback) hasFallback = true;
            else if (!parametricFid) parametricFid = fid;
        }
        if (!hasFallback || !parametricFid) continue;
        auto range = emitted.find(parametricFid);
        if (range == emitted.end()) continue;
        EdgePolyline poly;
        if (collectEmittedPolyline(edge, out, range->second.first,
                                   range->second.second, poly)) {
            canonical[eid] = std::move(poly);  // wins over captured polylines
        }
    }

    // Phase 2: triangulated faces, borders conformed to the canonical
    // polylines (both the parametric ones and the whole-shape ones).
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const FacePlan& plan = plans.at(fid);
        if (plan.kind != MesherKind::Fallback) continue;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FaceMeshSettings& s = settings.forFace(fid);
        BRepAdaptor_Surface surf(face);
        out.setGroup(weldGroup[fid]);
        out.setPart(partOfFace[fid]);
        meshFallback(face, surf, fid, s, model, canonical, out);
        if (report) {
            report->faceMesher[fid] = s.quadDominant ? MesherKind::QuadDominant
                                                     : MesherKind::Fallback;
        }
    }

    weldVertices(mesh, settings.weldTolerance, &out.groups());
    return mesh;
}

}  // namespace weft
