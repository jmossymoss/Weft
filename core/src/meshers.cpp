#include "weft/meshers.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
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
#include <array>
#include <cmath>
#include <atomic>
#include <cstdarg>
#include <map>
#include <mutex>
#include <numeric>
#include <set>
#include <thread>
#include <vector>

namespace weft {

// Stage-by-stage debug trace. Guarded by a mutex, flushed per line, so a
// crash log's last line names the exact face/edge/stage that died.
static std::FILE* gDebugLog = nullptr;
static std::mutex gDebugMutex;

void setGenerateDebugLog(std::FILE* f) { gDebugLog = f; }

static void dbg(const char* fmt, ...) {
    if (!gDebugLog) return;
    std::lock_guard<std::mutex> lock(gDebugMutex);
    va_list args;
    va_start(args, fmt);
    std::fprintf(gDebugLog, "[core] ");
    std::vfprintf(gDebugLog, fmt, args);
    std::fputc('\n', gDebugLog);
    std::fflush(gDebugLog);
    va_end(args);
}

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
        case MesherKind::CoonsGrid: return "coons-grid";
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
    // Revolution bands: false lets the two rims solve independently and
    // the band meshes as a triangulated taper between them.
    bool linkRims = true;
    // Forced fallback flavour: -1 = follow settings, 0 = pure tris,
    // 1 = quad-dominant (used when the user forces a mesher).
    int forceFallbackQuads = -1;
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

// A closed revolution face is only a clean band if it actually covers its
// surface's full parametric rectangle — a cylinder with pockets trimmed
// into it must NOT mesh as an untrimmed band overlapping its neighbours.
bool revolutionCovers(const TopoDS_Face& face) {
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double tol = BRep_Tool::Tolerance(face);
    for (int j = 1; j < 4; ++j) {
        for (int i = 0; i < 8; ++i) {
            gp_Pnt2d p(umin + (i + 0.5) / 8.0 * (umax - umin),
                       vmin + j / 4.0 * (vmax - vmin));
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), p,
                                         tol);
            if (cls.State() == TopAbs_OUT) return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Coons patch for four-sided freeform faces (plan §3.5): a structured grid
// blended between the four boundary pcurves in UV. Turns bspline strips
// into flowing quads instead of chord triangles, and its opposite sides
// take part in density matching like any grid.

struct CoonsPatch {
    std::array<int, 4> edgeIds{};
    std::array<Handle(Geom2d_Curve), 4> pc;
    std::array<double, 4> first{}, last{};
    std::array<bool, 4> rev{};

    // Point along side i at t in [0,1], walking the wire direction.
    gp_Pnt2d side(int i, double t) const {
        double tt = rev[i] ? 1.0 - t : t;
        return pc[i]->Value(first[i] + tt * (last[i] - first[i]));
    }

    // Bilinearly blended interior: a along side0->side2, b across.
    gp_Pnt2d uv(double a, double b) const {
        gp_Pnt2d bo = side(0, a), ri = side(1, b);
        gp_Pnt2d to = side(2, 1.0 - a), le = side(3, 1.0 - b);
        gp_Pnt2d c00 = side(0, 0.0), c10 = side(0, 1.0);
        gp_Pnt2d c11 = side(1, 1.0), c01 = side(3, 0.0);
        double x = (1 - b) * bo.X() + b * to.X() + (1 - a) * le.X() +
                   a * ri.X() -
                   ((1 - a) * (1 - b) * c00.X() + a * (1 - b) * c10.X() +
                    a * b * c11.X() + (1 - a) * b * c01.X());
        double y = (1 - b) * bo.Y() + b * to.Y() + (1 - a) * le.Y() +
                   a * ri.Y() -
                   ((1 - a) * (1 - b) * c00.Y() + a * (1 - b) * c10.Y() +
                    a * b * c11.Y() + (1 - a) * b * c01.Y());
        return {x, y};
    }
};

// Build the patch from the face's outer wire: exactly four non-degenerate
// edges with pcurves, chained head-to-tail, interior probes inside the
// face. Anything else returns false and the face meshes another way.
bool makeCoonsPatch(const TopoDS_Face& face, const Model& model,
                    CoonsPatch& patch) {
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    int n = 0;
    for (BRepTools_WireExplorer wx(outer, face); wx.More(); wx.Next()) {
        if (n >= 4) return false;
        const TopoDS_Edge edge = wx.Current();
        if (BRep_Tool::Degenerated(edge)) return false;
        double f, l;
        Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, f, l);
        if (pcurve.IsNull()) return false;
        patch.edgeIds[n] = model.edges.FindIndex(edge);
        patch.pc[n] = pcurve;
        patch.first[n] = f;
        patch.last[n] = l;
        patch.rev[n] = edge.Orientation() == TopAbs_REVERSED;
        ++n;
    }
    if (n != 4) return false;
    for (int i = 0; i < 4; ++i) {
        if (patch.edgeIds[i] < 1) return false;
    }
    // Head-to-tail continuity in UV (a seam on a periodic surface breaks
    // the chain; such faces are not Coons candidates).
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double span = std::max(umax - umin, vmax - vmin);
    for (int i = 0; i < 4; ++i) {
        gp_Pnt2d a = patch.side(i, 1.0);
        gp_Pnt2d b = patch.side((i + 1) % 4, 0.0);
        if (a.Distance(b) > 1e-4 * span) return false;
    }
    // Interior probes must land inside the face.
    const double tol = BRep_Tool::Tolerance(face);
    for (int j = 1; j < 4; ++j) {
        for (int i = 1; i < 4; ++i) {
            gp_Pnt2d p = patch.uv(i / 4.0, j / 4.0);
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), p,
                                         tol);
            if (cls.State() == TopAbs_OUT) return false;
        }
    }
    return true;
}

// Emit the Coons grid. uParams/vParams are the 0..1 splits (clustered for
// fillet strips); vertices evaluate exactly on the surface.
bool meshCoonsGrid(const TopoDS_Face& face, const Model& model, int faceId,
                   const std::vector<double>& uParams,
                   const std::vector<double>& vParams, MeshBuilder& out) {
    CoonsPatch patch;
    if (!makeCoonsPatch(face, model, patch)) return false;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    // Border vertices evaluate on the shared 3D edge curves, not through
    // this face's pcurve: both faces of an edge then produce bit-identical
    // points and the weld is exact (pcurves only agree with the curve to
    // the edge tolerance, which exceeds the weld tolerance).
    std::array<BRepAdaptor_Curve, 4> c3d;
    for (int i = 0; i < 4; ++i) {
        c3d[i].Initialize(TopoDS::Edge(model.edges(patch.edgeIds[i])));
    }
    auto sidePnt = [&](int i, double t) {
        double tt = patch.rev[i] ? 1.0 - t : t;
        double f = c3d[i].FirstParameter(), l = c3d[i].LastParameter();
        return c3d[i].Value(f + tt * (l - f));
    };
    const int nu = int(uParams.size()) - 1;
    const int nv = int(vParams.size()) - 1;
    // The (a,b) lattice follows the wire, whose UV handedness varies; the
    // Jacobian sign says whether the grid is CCW in UV, and combined with
    // the face orientation that decides the polygon winding.
    gp_Pnt2d c0 = patch.uv(0.5, 0.5);
    gp_Pnt2d ca = patch.uv(0.55, 0.5);
    gp_Pnt2d cb = patch.uv(0.5, 0.55);
    const double jac = (ca.X() - c0.X()) * (cb.Y() - c0.Y()) -
                       (ca.Y() - c0.Y()) * (cb.X() - c0.X());
    const bool flip = (face.Orientation() == TopAbs_REVERSED) != (jac < 0);

    std::vector<uint32_t> grid((nu + 1) * (nv + 1));
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            gp_Pnt2d p = patch.uv(uParams[i], vParams[j]);
            gp_Pnt pos;
            if (j == 0) pos = sidePnt(0, uParams[i]);
            else if (j == nv) pos = sidePnt(2, 1.0 - uParams[i]);
            else if (i == nu) pos = sidePnt(1, vParams[j]);
            else if (i == 0) pos = sidePnt(3, 1.0 - vParams[j]);
            else pos = surface->Value(p.X(), p.Y());
            grid[j * (nu + 1) + i] =
                out.addVertex(pos, {faceId, p.X(), p.Y()});
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
    return true;
}

FacePlan planFace(int fid, const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings) {
    const TopoDS_Face face = TopoDS::Face(model.faces(fid));
    const FaceMeshSettings& s = settings.forFace(fid);
    const FaceInfo& info = analysis.faces[fid - 1];
    BRepAdaptor_Surface surf(face);
    FacePlan plan;

    auto finishRevolution = [&]() {
        plan.kind = MesherKind::RevolutionGrid;
        collectIsoEdges(face, model, info.edgeIds, plan);
        if (!s.linkRims && plan.uEdges.size() == 2) plan.linkRims = false;
    };

    // The user can force a strategy; if it can't build on this face the
    // plan degrades to plain triangulation so the choice is visible.
    if (s.forceMesher > 0) {
        MesherKind want = MesherKind(s.forceMesher - 1);
        switch (want) {
            case MesherKind::RevolutionGrid:
                if (isClosedRevolution(surf)) {
                    finishRevolution();
                    return plan;
                }
                break;
            case MesherKind::DiskCap: {
                int capEdgeId = 0;
                if (surf.GetType() == GeomAbs_Plane &&
                    boundingCircle(face, plan.circ, capEdgeId, model)) {
                    plan.kind = MesherKind::DiskCap;
                    plan.uEdges.push_back(capEdgeId);
                    plan.constrains = true;
                    return plan;
                }
                break;
            }
            case MesherKind::RingJunction:
                if (planRingJunction(face, model, plan)) return plan;
                break;
            case MesherKind::PlanarGrid:
            case MesherKind::MinimalNGon:
                if (parametricGridFits(face, surf, std::max(1, s.gridU),
                                       std::max(1, s.gridV))) {
                    plan.kind = want;
                    collectIsoEdges(face, model, info.edgeIds, plan);
                    if (plan.uEdges.size() != 2 || plan.vEdges.size() != 2) {
                        plan.constrains = false;
                        if (want == MesherKind::MinimalNGon) {
                            plan.kind = MesherKind::PlanarGrid;
                        }
                    }
                    return plan;
                }
                break;
            case MesherKind::CoonsGrid: {
                CoonsPatch patch;
                if (makeCoonsPatch(face, model, patch)) {
                    plan.kind = MesherKind::CoonsGrid;
                    plan.uEdges = {patch.edgeIds[0], patch.edgeIds[2]};
                    plan.vEdges = {patch.edgeIds[1], patch.edgeIds[3]};
                    plan.constrains = true;
                    return plan;
                }
                break;
            }
            case MesherKind::QuadDominant:
                plan.kind = MesherKind::Fallback;
                plan.forceFallbackQuads = 1;
                return plan;
            case MesherKind::Fallback:
                plan.kind = MesherKind::Fallback;
                plan.forceFallbackQuads = 0;
                return plan;
        }
        plan.kind = MesherKind::Fallback;
        return plan;
    }

    if (isClosedRevolution(surf) && revolutionCovers(face)) {
        finishRevolution();
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
        // Only a plain 2u+2v rectangle ties its grid to its edges; anything
        // else meshes with its own settings, unconstrained.
        collectIsoEdges(face, model, info.edgeIds, plan);
        if (plan.uEdges.size() != 2 || plan.vEdges.size() != 2) {
            plan.constrains = false;
        }
        // The n-gon's ring is a rectangle perimeter walk; without the 2u+2v
        // structure there is nothing reliable to walk.
        if (plan.kind == MesherKind::MinimalNGon && !plan.constrains) {
            plan.kind = MesherKind::PlanarGrid;
        }
        return plan;
    }

    // Four-sided freeform/trimmed faces get a structured Coons grid; the
    // across-the-blend direction of a fillet strip is whichever side pair
    // is shorter in 3D.
    {
        CoonsPatch patch;
        if (makeCoonsPatch(face, model, patch)) {
            plan.kind = MesherKind::CoonsGrid;
            plan.uEdges = {patch.edgeIds[0], patch.edgeIds[2]};
            plan.vEdges = {patch.edgeIds[1], patch.edgeIds[3]};
            plan.constrains = true;
            if (info.isFillet) {
                plan.isFillet = true;
                auto sideLen = [&](int i) {
                    BRepAdaptor_Curve c(
                        TopoDS::Edge(model.edges(patch.edgeIds[i])));
                    return GCPnts_AbscissaPoint::Length(c);
                };
                plan.acrossIsU =
                    sideLen(0) + sideLen(2) < sideLen(1) + sideLen(3);
            }
            return plan;
        }
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
        if (plan.linkRims) sol.groups.unite(plan.uEdges);
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
            plan.kind == MesherKind::CoonsGrid ||
            plan.kind == MesherKind::MinimalNGon ||
            plan.kind == MesherKind::RingJunction) {
            int nu = std::max(1, plan.isFillet && plan.acrossIsU
                                     ? s.filletLoops : s.gridU);
            int nv = std::max(1, plan.isFillet && !plan.acrossIsU
                                     ? s.filletLoops : s.gridV);
            propose(plan.uEdges, nu, overridden);
            propose(plan.vEdges, nv, overridden);
        } else {  // revolution sides and disk caps subdivide rings radially
            if (!plan.linkRims && plan.uEdges.size() == 2) {
                // Unlinked rims: each ring solves on its own (pin per-edge
                // or via the rim fields to make them differ).
                propose({plan.uEdges[0]}, std::max(3, s.radial), overridden);
                propose({plan.uEdges[1]}, std::max(3, s.radial), overridden);
            } else {
                propose(plan.uEdges, std::max(3, s.radial), overridden);
            }
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

// A revolution band whose two rims carry DIFFERENT counts (linkRims off):
// one ring per rim, zippered with triangles by angular fraction. Rim
// vertices evaluate exactly like the quad band's, so caps still weld.
void meshRevolutionTaper(const TopoDS_Face& face,
                         const BRepAdaptor_Surface& surf, int faceId, int nA,
                         int nB, MeshBuilder& out) {
    nA = std::max(3, nA);
    nB = std::max(3, nB);
    const double u0 = surf.FirstUParameter();
    const double uRange = surf.LastUParameter() - u0;
    const double v0 = surf.FirstVParameter();
    const double v1 = surf.LastVParameter();
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    std::vector<uint32_t> A(nA), B(nB);
    for (int i = 0; i < nA; ++i) {
        double u = u0 + uRange * i / nA;
        A[i] = out.addVertex(surf.Value(u, v0), {faceId, u, v0});
    }
    for (int j = 0; j < nB; ++j) {
        double u = u0 + uRange * j / nB;
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

// Last resort for trimmed/freeform faces: OCCT chord-tolerance
// triangulation, optionally paired into quads. Pairing is greedy over a
// quality score that prefers near-rectangular quads whose edges follow the
// surface's parametric directions — the seed of the plan's guided quad
// flow (§3.5); a real cross-field solver replaces the guidance later.
void meshFallback(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  int faceId, const FaceMeshSettings& s, MeshBuilder& out) {
    // Faces mesh on worker threads, but OCCT triangulation writes shared
    // per-EDGE data (adjacent faces touch the same TEdge), so the OCCT
    // calls serialize; the heavy per-node work below stays parallel.
    // Clean first so a loosened deviation actually re-coarsens instead of
    // keeping the cached finer triangulation.
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri;
    {
        static std::mutex occtMeshMutex;
        std::lock_guard<std::mutex> lock(occtMeshMutex);
        BRepTools::Clean(face);
        IMeshTools_Parameters mp;
        mp.Deflection = s.chordTolerance;
        mp.Angle = s.angleToleranceDeg * M_PI / 180.0;
        mp.Relative = s.relativeDeviation;
        if (s.minSize > 0) mp.MinSize = s.minSize;
        mp.InParallel = Standard_True;
        BRepMesh_IncrementalMesh mesher(face, mp);
        tri = BRep_Tool::Triangulation(face, loc);
    }
    if (tri.IsNull()) return;

    const bool flip = face.Orientation() == TopAbs_REVERSED;
    const bool hasUV = tri->HasUVNodes();
    std::vector<uint32_t> verts(tri->NbNodes());
    std::vector<gp_Pnt> pts(tri->NbNodes());
    for (int i = 1; i <= tri->NbNodes(); ++i) {
        Anchor a;
        if (hasUV) {
            gp_Pnt2d uv = tri->UVNode(i);
            a = {faceId, uv.X(), uv.Y()};
        }
        pts[i - 1] = tri->Node(i).Transformed(loc.Transformation());
        verts[i - 1] = out.addVertex(pts[i - 1], a);
    }

    std::vector<std::array<int, 3>> tris(tri->NbTriangles());
    for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int a, b, c;
        tri->Triangle(i).Get(a, b, c);
        tris[i - 1] = {a - 1, b - 1, c - 1};
    }

    if (!s.quadDominant) {
        for (const auto& t : tris) {
            out.addPolygon({verts[t[0]], verts[t[1]], verts[t[2]]}, faceId, flip);
        }
        return;
    }
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
            gp_Pnt2d uv0 = tri->UVNode(ring[0] + 1);
            gp_Pnt2d uv2 = tri->UVNode(ring[2] + 1);
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

    // One midpoint (Catmull-Clark-style) subdivision turns the paired mesh
    // into pure quads: each tri becomes 3, each quad 4. New vertices are
    // evaluated on the surface through averaged UVs, so they sit exactly on
    // the B-rep, not on the chord.
    auto emitVertex = [&](double u, double v, const gp_Pnt& fallbackPnt) {
        if (!hasUV) return out.addVertex(fallbackPnt, {});
        gp_Pnt p = surf.Value(u, v);
        return out.addVertex(p, {faceId, u, v});
    };
    std::vector<gp_Pnt2d> uvs(pts.size());
    if (hasUV) {
        for (size_t i = 0; i < pts.size(); ++i) uvs[i] = tri->UVNode(i + 1);
    }
    std::map<std::pair<int, int>, uint32_t> midOf;
    auto midpoint = [&](int a, int b) {
        auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
        auto it = midOf.find(key);
        if (it != midOf.end()) return it->second;
        gp_Pnt mid(0.5 * (pts[a].X() + pts[b].X()),
                   0.5 * (pts[a].Y() + pts[b].Y()),
                   0.5 * (pts[a].Z() + pts[b].Z()));
        uint32_t idx = emitVertex(0.5 * (uvs[a].X() + uvs[b].X()),
                                  0.5 * (uvs[a].Y() + uvs[b].Y()), mid);
        midOf[key] = idx;
        return idx;
    };

    for (const auto& ring : paired) {
        const int n = static_cast<int>(ring.size());
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
        for (int i = 0; i < n; ++i) {
            out.addPolygon({verts[ring[i]], midpoint(ring[i], ring[(i + 1) % n]),
                            center, midpoint(ring[(i + n - 1) % n], ring[i])},
                           faceId, flip);
        }
    }
}

// ---------------------------------------------------------------------------
// Border conformity (plan §7.1, first bite): freeform faces triangulate to
// chord tolerance, so their borders never agree with an analytic
// neighbour's solved divisions — T-junctions along every shared edge. Fix
// after meshing: for each edge where a fallback face meets a constraining
// analytic face, snap the fallback border chain onto the analytic vertex
// chain and insert any analytic verts the chain skips; the weld then fuses
// the seam exactly.

struct EdgeParamPoint {
    uint32_t vert;
    double param;
};

void conformFallbackBorders(PolyMesh& mesh, const Model& model,
                            const std::map<int, FacePlan>& plans,
                            const GenerationSettings& settings,
                            const std::vector<std::array<size_t, 2>>& range) {
    auto isFreeform = [&](int fid) {
        MesherKind k = plans.at(fid).kind;
        return (k == MesherKind::Fallback || k == MesherKind::QuadDominant) &&
               !settings.forFace(fid).exclude;
    };
    auto isAnalytic = [&](int fid) {
        MesherKind k = plans.at(fid).kind;
        return plans.at(fid).constrains && k != MesherKind::Fallback &&
               k != MesherKind::QuadDominant && !settings.forFace(fid).exclude;
    };

    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        if (!isFreeform(fid)) continue;

        // Topological border vertices of this face's sub-mesh.
        std::map<std::pair<uint32_t, uint32_t>, int> use;
        std::vector<size_t> facePolys;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            facePolys.push_back(p);
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                ++use[a < b ? std::make_pair(a, b) : std::make_pair(b, a)];
            }
        }
        std::set<uint32_t> borderVerts;
        for (const auto& [e, count] : use) {
            if (count == 1) {
                borderVerts.insert(e.first);
                borderVerts.insert(e.second);
            }
        }
        if (borderVerts.empty()) continue;

        for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE); ex.More();
             ex.Next()) {
            int eid = model.edges.FindIndex(ex.Current());
            if (eid < 1) continue;
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            // Degenerate edges (poles, apexes) carry no 3D curve; seam and
            // curveless edges can't anchor a chain either.
            if (BRep_Tool::Degenerated(edge)) continue;
            {
                double cf, cl;
                if (BRep_Tool::Curve(edge, cf, cl).IsNull()) continue;
            }
            int nfid = 0;
            if (model.edgeToFaces.Contains(ex.Current())) {
                for (const TopoDS_Shape& s :
                     model.edgeToFaces.FindFromKey(ex.Current())) {
                    int f2 = model.faces.FindIndex(s);
                    if (f2 != fid) nfid = f2;
                }
            }
            if (nfid < 1 || !isAnalytic(nfid)) continue;
            dbg("conform: face %d edge %d (analytic neighbour %d)", fid, eid,
                nfid);

            BRepAdaptor_Curve curve(edge);
            const double f = curve.FirstParameter(), l = curve.LastParameter();
            const bool closed = curve.IsClosed();
            const double period = l - f;
            GCPnts_AbscissaPoint lenTool;
            double edgeLen = GCPnts_AbscissaPoint::Length(curve);
            (void)lenTool;

            // Exact distance/parameter on the curve for a mesh vertex.
            auto project = [&](uint32_t v, double tol,
                               double* paramOut) -> bool {
                gp_Pnt p(mesh.vertices[v][0], mesh.vertices[v][1],
                         mesh.vertices[v][2]);
                double bestD = p.Distance(curve.Value(f));
                double bestT = f;
                double dl = p.Distance(curve.Value(l));
                if (dl < bestD) { bestD = dl; bestT = l; }
                try {
                    Extrema_ExtPC ext(p, curve);
                    if (ext.IsDone()) {
                        for (int i = 1; i <= ext.NbExt(); ++i) {
                            double d = std::sqrt(ext.SquareDistance(i));
                            if (d < bestD) {
                                bestD = d;
                                bestT = ext.Point(i).Parameter();
                            }
                        }
                    }
                } catch (...) {
                    // Extrema can fail on exotic curves; endpoint distances
                    // computed above still stand.
                }
                if (bestD > tol) return false;
                *paramOut = bestT;
                return true;
            };

            // The analytic side's verts on this edge: the authoritative
            // chain. Coons verts evaluate through the pcurve, which is
            // only guaranteed to agree with the 3D curve to the edge
            // tolerance, so include it.
            const double tolTarget =
                std::max(1e-6 * (1.0 + edgeLen),
                         10.0 * BRep_Tool::Tolerance(edge));
            std::vector<EdgeParamPoint> targets;
            for (size_t v = range[nfid][0]; v < range[nfid][1]; ++v) {
                double t;
                if (project(uint32_t(v), tolTarget, &t)) {
                    targets.push_back({uint32_t(v), t});
                }
            }
            if (targets.size() < 2) continue;
            std::sort(targets.begin(), targets.end(),
                      [](const EdgeParamPoint& a, const EdgeParamPoint& b) {
                          return a.param < b.param;
                      });

            // The freeform side's border verts near this edge. Subdivision
            // midpoints sit on the surface but off the curve by up to the
            // chord sagitta, so the tolerance is the face's deviation.
            const double tolMover = std::max(
                1e-6 * (1.0 + edgeLen),
                settings.forFace(fid).chordTolerance * 1.2);
            std::map<uint32_t, double> movers;  // vert -> snapped param
            for (uint32_t v : borderVerts) {
                double t;
                if (project(v, tolMover, &t)) movers[v] = t;
            }
            if (movers.empty()) continue;

            auto paramGap = [&](double a, double b) {  // |a-b| wrap-aware
                double d = std::abs(a - b);
                return closed ? std::min(d, period - d) : d;
            };
            // Snap every mover to the nearest target (position + param).
            for (auto& [v, t] : movers) {
                const EdgeParamPoint* best = &targets[0];
                for (const EdgeParamPoint& cand : targets) {
                    if (paramGap(cand.param, t) < paramGap(best->param, t)) {
                        best = &cand;
                    }
                }
                mesh.vertices[v] = mesh.vertices[best->vert];
                t = best->param;
            }

            // Insert targets skipped between consecutive border movers so
            // the chains agree vertex-for-vertex.
            for (size_t p : facePolys) {
                std::vector<uint32_t>& poly = mesh.polygons[p];
                std::vector<uint32_t> ring;
                ring.reserve(poly.size() + 4);
                for (size_t i = 0; i < poly.size(); ++i) {
                    uint32_t u = poly[i], w = poly[(i + 1) % poly.size()];
                    ring.push_back(u);
                    auto mu = movers.find(u), mw = movers.find(w);
                    if (mu == movers.end() || mw == movers.end()) continue;
                    // Only true border segments take insertions — a
                    // triangulation also has interior chords whose both
                    // ends sit on the curve, and inserting into those
                    // duplicates the chain.
                    auto useIt = use.find(
                        u < w ? std::make_pair(u, w) : std::make_pair(w, u));
                    if (useIt == use.end() || useIt->second != 1) continue;
                    // Border segment on the edge: walk the shorter param
                    // arc from u to w, inserting the targets inside it.
                    double pu = mu->second, pw = mw->second;
                    if (paramGap(pu, pw) < 1e-12) continue;
                    bool forward = closed
                        ? std::fmod(pw - pu + period, period) <= period * 0.5
                        : pw > pu;
                    std::vector<const EdgeParamPoint*> between;
                    for (const EdgeParamPoint& cand : targets) {
                        double rel = closed
                            ? std::fmod((forward ? cand.param - pu
                                                 : pu - cand.param) + period,
                                        period)
                            : (forward ? cand.param - pu : pu - cand.param);
                        double span = closed
                            ? std::fmod((forward ? pw - pu : pu - pw) + period,
                                        period)
                            : std::abs(pw - pu);
                        if (rel > 1e-12 && rel < span - 1e-12) {
                            between.push_back(&cand);
                        }
                    }
                    std::sort(between.begin(), between.end(),
                              [&](const EdgeParamPoint* a,
                                  const EdgeParamPoint* b) {
                                  auto key = [&](double t) {
                                      return closed
                                          ? std::fmod((forward ? t - pu
                                                               : pu - t) +
                                                          period, period)
                                          : (forward ? t - pu : pu - t);
                                  };
                                  return key(a->param) < key(b->param);
                              });
                    for (const EdgeParamPoint* c : between) {
                        ring.push_back(c->vert);
                    }
                }
                poly = std::move(ring);
            }
        }
    }
}

}  // namespace

PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings, GenerationReport* report) {
    dbg("generate: begin (%d faces, %d edges, parallel=%d, conform=%d)",
        model.faceCount(), model.edgeCount(), settings.parallelMeshing ? 1 : 0,
        settings.conformBorders ? 1 : 0);
    std::map<int, FacePlan> plans;
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        FacePlan plan = planFace(fid, model, analysis, settings);
        if (settings.forFace(fid).exclude) {
            // Deleted faces neither mesh nor constrain their neighbours'
            // densities — their borders become free boundary loops.
            plan.kind = MesherKind::Fallback;
            plan.constrains = false;
        }
        plans.emplace(fid, std::move(plan));
    }
    dbg("generate: plans done");

    DensitySolution density = solveDensity(model, plans, settings);
    dbg("generate: density solved");

    // Resolve every face's division counts up front (union-find lookups
    // path-compress, so they must not run concurrently) — after this the
    // per-face meshing is embarrassingly parallel.
    const int faceN = model.faceCount();
    std::vector<std::array<int, 3>> counts(faceN + 1, {0, 0, 0});
    for (int fid = 1; fid <= faceN; ++fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        const FacePlan& plan = plans.at(fid);
        auto solved = [&](const std::vector<int>& edges, int fallback) {
            return edges.empty() ? fallback
                                 : density.countFor(edges[0], fallback);
        };
        switch (plan.kind) {
            case MesherKind::RevolutionGrid:
            case MesherKind::DiskCap: {
                int nuA = solved(plan.uEdges, s.radial);
                int nuB = nuA;
                if (!plan.linkRims && plan.uEdges.size() == 2) {
                    nuB = density.countFor(plan.uEdges[1], s.radial);
                }
                counts[fid] = {nuA, solved(plan.vEdges, s.axial), nuB};
                break;
            }
            case MesherKind::PlanarGrid:
            case MesherKind::CoonsGrid: {
                int defU = plan.isFillet && plan.acrossIsU ? s.filletLoops
                                                           : s.gridU;
                int defV = plan.isFillet && !plan.acrossIsU ? s.filletLoops
                                                            : s.gridV;
                counts[fid] = {plan.constrains ? solved(plan.uEdges, defU)
                                               : std::max(1, defU),
                               plan.constrains ? solved(plan.vEdges, defV)
                                               : std::max(1, defV),
                               0};
                break;
            }
            case MesherKind::MinimalNGon:
            case MesherKind::RingJunction:
                counts[fid] = {solved(plan.uEdges, s.gridU),
                               solved(plan.vEdges, s.gridV), 0};
                break;
            default:
                break;
        }
    }

    // Mesh every face into its own part, in parallel, then merge in face
    // order so the output is deterministic (identical to the serial order).
    std::vector<PolyMesh> parts(faceN + 1);
    auto meshFace = [&](int fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        if (s.exclude) return;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FacePlan& plan = plans.at(fid);
        dbg("mesh face %d: %s", fid, mesherKindName(plan.kind));
        BRepAdaptor_Surface surf(face);
        MeshBuilder out(parts[fid]);
        const int nu = counts[fid][0], nv = counts[fid][1];

        switch (plan.kind) {
            case MesherKind::RevolutionGrid:
                if (!plan.linkRims && counts[fid][2] > 0 &&
                    counts[fid][2] != nu && !surf.IsVClosed()) {
                    // counts[0] belongs to uEdges[0]; find which v-end that
                    // rim sits at so the taper's rings land on their caps.
                    int nA = nu, nB = counts[fid][2];
                    BRepAdaptor_Curve rim(
                        TopoDS::Edge(model.edges(plan.uEdges[0])));
                    gp_Pnt pm = rim.Value(
                        (rim.FirstParameter() + rim.LastParameter()) / 2);
                    double d0 = 1e300, d1 = 1e300;
                    const double u0 = surf.FirstUParameter();
                    const double du =
                        (surf.LastUParameter() - u0) / 16.0;
                    for (int k = 0; k < 16; ++k) {
                        d0 = std::min(d0, pm.Distance(surf.Value(
                                              u0 + k * du,
                                              surf.FirstVParameter())));
                        d1 = std::min(d1, pm.Distance(surf.Value(
                                              u0 + k * du,
                                              surf.LastVParameter())));
                    }
                    if (d1 < d0) std::swap(nA, nB);
                    meshRevolutionTaper(face, surf, fid, nA, nB, out);
                } else {
                    meshRevolutionGrid(face, surf, fid, nu, nv, out);
                }
                break;
            case MesherKind::DiskCap:
                meshDiskCap(face, surf, plan.circ, fid, nu, s.cap, out);
                break;
            case MesherKind::PlanarGrid: {
                // Support loops hug the creases on fillet strips.
                double holdU = plan.isFillet && plan.acrossIsU ? s.filletHold : 0;
                double holdV = plan.isFillet && !plan.acrossIsU ? s.filletHold : 0;
                meshParametricGrid(face, surf, fid, clusteredParams(nu, holdU),
                                   clusteredParams(nv, holdV), out);
                break;
            }
            case MesherKind::CoonsGrid: {
                double holdU = plan.isFillet && plan.acrossIsU ? s.filletHold : 0;
                double holdV = plan.isFillet && !plan.acrossIsU ? s.filletHold : 0;
                if (!meshCoonsGrid(face, model, fid, clusteredParams(nu, holdU),
                                   clusteredParams(nv, holdV), out)) {
                    meshFallback(face, surf, fid, s, out);
                }
                break;
            }
            case MesherKind::MinimalNGon:
                meshMinimalNGon(face, surf, fid, nu, nv, out);
                break;
            case MesherKind::RingJunction:
                meshRingJunction(face, surf, plan.circ, fid, nu, nv,
                                 s.junctionRings, out);
                break;
            case MesherKind::QuadDominant:
            case MesherKind::Fallback: {
                FaceMeshSettings fs = s;
                if (plan.forceFallbackQuads >= 0) {
                    fs.quadDominant = plan.forceFallbackQuads != 0;
                }
                meshFallback(face, surf, fid, fs, out);
                break;
            }
        }
    };

    unsigned threads = std::min<unsigned>(
        std::max(1u, std::thread::hardware_concurrency()), unsigned(faceN));
    if (!settings.parallelMeshing) threads = 1;
    dbg("generate: meshing on %u thread(s)", threads);
    if (threads <= 1) {
        for (int fid = 1; fid <= faceN; ++fid) meshFace(fid);
    } else {
        std::atomic<int> nextFace{1};
        std::exception_ptr firstError;
        std::mutex errorMutex;
        std::vector<std::thread> pool;
        for (unsigned t = 0; t < threads; ++t) {
            pool.emplace_back([&] {
                try {
                    for (int fid = nextFace.fetch_add(1); fid <= faceN;
                         fid = nextFace.fetch_add(1)) {
                        meshFace(fid);
                    }
                } catch (...) {
                    std::lock_guard<std::mutex> lock(errorMutex);
                    if (!firstError) firstError = std::current_exception();
                }
            });
        }
        for (std::thread& th : pool) th.join();
        if (firstError) std::rethrow_exception(firstError);
    }

    PolyMesh mesh;
    std::vector<std::array<size_t, 2>> range(faceN + 1, {0, 0});
    for (int fid = 1; fid <= faceN; ++fid) {
        PolyMesh& part = parts[fid];
        uint32_t base = uint32_t(mesh.vertices.size());
        range[fid] = {size_t(base), size_t(base) + part.vertices.size()};
        mesh.vertices.insert(mesh.vertices.end(), part.vertices.begin(),
                             part.vertices.end());
        mesh.anchors.insert(mesh.anchors.end(), part.anchors.begin(),
                            part.anchors.end());
        for (auto& poly : part.polygons) {
            for (uint32_t& v : poly) v += base;
            mesh.polygons.push_back(std::move(poly));
        }
        mesh.polygonFaceId.insert(mesh.polygonFaceId.end(),
                                  part.polygonFaceId.begin(),
                                  part.polygonFaceId.end());
    }

    dbg("generate: merged (%zu verts, %zu polys)", mesh.vertexCount(),
        mesh.polygonCount());
    if (settings.conformBorders) {
        conformFallbackBorders(mesh, model, plans, settings, range);
        dbg("generate: borders conformed");
    }

    if (report) {
        for (int fid = 1; fid <= faceN; ++fid) {
            const FaceMeshSettings& s = settings.forFace(fid);
            const FacePlan& plan = plans.at(fid);
            bool fallbackQuads = plan.forceFallbackQuads >= 0
                                     ? plan.forceFallbackQuads != 0
                                     : s.quadDominant;
            report->faceMesher[fid] =
                plan.kind == MesherKind::Fallback && fallbackQuads
                    ? MesherKind::QuadDominant
                    : plan.kind;
            if (plan.kind == MesherKind::RevolutionGrid &&
                plan.uEdges.size() == 2) {
                report->faceRims[fid] = {plan.uEdges[0], plan.uEdges[1]};
            }
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

    dbg("generate: welding");
    weldVertices(mesh, settings.weldTolerance);
    dbg("generate: done (%zu verts, %zu polys)", mesh.vertexCount(),
        mesh.polygonCount());
    return mesh;
}

}  // namespace weft
