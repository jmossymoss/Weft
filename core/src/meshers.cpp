#include "weft/meshers.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <Standard_Failure.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GProp_GProps.hxx>
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
#include <tuple>
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
        case MesherKind::AnnulusRing: return "annulus-ring";
        case MesherKind::PlateWeb: return "plate-web";
        case MesherKind::QuadFill: return "quad-fill";
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
    // PlateWeb: every boundary wire's edge chain (loops[0] = outer wire).
    // Each edge solves independently — a bore drives its own hole loop.
    std::vector<std::vector<int>> loops;
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
                     const std::vector<int>& edgeIds, FacePlan& plan,
                     bool skipNonIso = false) {
    BRepAdaptor_Surface surf(face);
    double uRange = surf.LastUParameter() - surf.FirstUParameter();
    double vRange = surf.LastVParameter() - surf.FirstVParameter();
    for (int eid : edgeIds) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;  // apex/pole edges
        switch (edgeIsoDirection(edge, face, uRange, vRange)) {
            case EdgeIso::UAligned: plan.uEdges.push_back(eid); break;
            case EdgeIso::VAligned: plan.vEdges.push_back(eid); break;
            case EdgeIso::Neither:
                // Revolution bands tolerate non-iso edges (pocket cuts,
                // forced full bands): the rims still constrain. Grids
                // need the full 2u+2v structure and bail instead.
                if (skipNonIso) continue;
                plan.constrains = false;
                return;
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
double wireElongation(const TopoDS_Wire& wire);  // defined with plate-web

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
            loop[wires].push_back(eid);
        }
        if (loop[wires].empty() || loop[wires].size() > 8) return false;
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
    }
    if (outerIdx == 1) std::swap(loop[0], loop[1]);
    plan.kind = MesherKind::AnnulusRing;
    plan.uEdges = loop[0];
    plan.vEdges = loop[1];
    plan.constrains = true;
    return true;
}

void meshAnnulusRing(const TopoDS_Face& face, const Model& model, int faceId,
                     const std::vector<int>& outerLoop,
                     const std::vector<int>& innerLoop,
                     const std::vector<int>& solvedEdge, int radialDefault,
                     MeshBuilder& out) {
    // A ring = the wire's edges chained in order, each sampled at its own
    // solved count (endpoints shared with the next edge, so a loop of K
    // edges at counts c_k has sum(c_k) vertices).
    auto sampleRing = [&](const std::vector<int>& loop) {
        std::vector<gp_Pnt> pts;
        for (int eid : loop) {
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            int n = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
            if (n < 1) n = std::max(3, radialDefault) / int(loop.size());
            n = std::max(1, n);
            BRepAdaptor_Curve c(edge);
            double f = c.FirstParameter(), l = c.LastParameter();
            const bool rev = edge.Orientation() == TopAbs_REVERSED;
            for (int i = 0; i < n; ++i) {  // skip the shared endpoint
                double t = rev ? 1.0 - double(i) / n : double(i) / n;
                pts.push_back(c.Value(f + (l - f) * t));
            }
        }
        return pts;
    };
    std::vector<gp_Pnt> A = sampleRing(outerLoop);
    std::vector<gp_Pnt> B = sampleRing(innerLoop);
    if (A.size() < 3 || B.size() < 3) return;
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

    // Zipper by fraction (equal counts -> pure quads). Winding is fixed
    // afterwards against the surface normal at the first polygon.
    struct Poly { std::vector<uint32_t> ring; };
    std::vector<std::vector<uint32_t>> polys;
    int ia = 0, ib = 0;
    while (ia < nOut && ib < nIn &&
           nOut == nIn) {  // quad ring fast path
        polys.push_back({av[ia % nOut], av[(ia + 1) % nOut],
                         bv[(ib + 1) % nIn], bv[ib % nIn]});
        ++ia;
        ++ib;
    }
    while (ia < nOut || ib < nIn) {
        double fa = double(ia + 1) / nOut, fb = double(ib + 1) / nIn;
        bool stepA = ib >= nIn || (ia < nOut && fa <= fb);
        if (stepA) {
            polys.push_back({av[ia % nOut], av[(ia + 1) % nOut],
                             bv[ib % nIn]});
            ++ia;
        } else {
            polys.push_back({av[ia % nOut], bv[(ib + 1) % nIn],
                             bv[ib % nIn]});
            ++ib;
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
}

// A planar face with hole loops that the simpler patterns can't take
// (three or more wires, or two wires too edge-rich for the annulus band):
// the bolt-hole plate. Each hole gets a quad collar, the rest is an
// ear-clipped triangle web — every boundary vertex sits on its B-rep edge
// curve at the solved count, so all neighbours weld watertight.
// Collect a planar face's wires as per-edge loop chains, outer wire first.
// Shared by the plate-web planner and the generalized minimal-ngon.
bool collectPlanarLoops(const TopoDS_Face& face,
                        const BRepAdaptor_Surface& surf, const Model& model,
                        FacePlan& plan, bool requirePlane = true) {
    if (requirePlane && surf.GetType() != GeomAbs_Plane) return false;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    std::vector<std::vector<int>> loops;
    int outerIdx = -1, wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
        std::vector<int> loop;
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            if (BRep_Tool::Degenerated(edge)) return false;
            double f, l;
            if (BRep_Tool::Curve(edge, f, l).IsNull()) return false;
            if (BRep_Tool::CurveOnSurface(edge, face, f, l).IsNull()) {
                return false;
            }
            int eid = model.edges.FindIndex(edge);
            if (eid < 1) return false;
            loop.push_back(eid);
        }
        if (loop.empty() || loop.size() > 24) return false;
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
                       FacePlan& plan) {
    FacePlan probe;
    if (!collectPlanarLoops(face, surf, model, probe)) return false;
    plan.loops = std::move(probe.loops);
    plan.uEdges = std::move(probe.uEdges);
    plan.constrains = true;
    plan.kind = MesherKind::MinimalNGon;
    return true;
}

// --- Plate-web execution: 2D machinery -------------------------------------
// The plate is planar, so its UV space is an isometric chart — offsets and
// intersection tests run there and map straight back to 3D.

struct WebPoint {
    gp_Pnt2d uv;
    uint32_t vert;  // index in the MeshBuilder
};

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
void earClip(std::vector<WebPoint> poly, int faceId, bool flip,
             MeshBuilder& out) {
    const size_t n = poly.size();
    if (n < 3) return;
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
            // Numerical dead end (should not happen on sane plates): close
            // the rest as a fan so the face at least stays connected.
            for (size_t k = 1; k + 1 < idx.size(); ++k) {
                out.addPolygon({poly[idx[0]].vert, poly[idx[k]].vert,
                                poly[idx[k + 1]].vert},
                               faceId, flip);
            }
            return;
        }
    }
    if (idx.size() == 3) {
        out.addPolygon({poly[idx[0]].vert, poly[idx[1]].vert,
                        poly[idx[2]].vert},
                       faceId, flip);
    }
}

// Merge hole rings into the outer ring via non-crossing bridges (doubled
// bridge vertices), rightmost holes first, then ear-clip the result.
void triangulateWeb(std::vector<WebPoint> outer,
                    std::vector<std::vector<WebPoint>> holes, int faceId,
                    bool flip, MeshBuilder& out) {
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
            // No visible vertex (pathological): mesh the hole ring away as
            // its own fan so we don't lose the boundary vertices entirely.
            for (size_t k = 1; k + 1 < hole.size(); ++k) {
                out.addPolygon(
                    {hole[0].vert, hole[k + 1].vert, hole[k].vert}, faceId,
                    flip);
            }
            continue;
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
    earClip(std::move(outer), faceId, flip, out);
}

// A planar face's wire sampled as one chained ring: each edge at its own
// solved count, positions on the 3D edge curve (weld-exact), UV from the
// pcurve (the plate's isometric chart).
struct PlanarRing {
    std::vector<gp_Pnt2d> uv;
    std::vector<gp_Pnt> p;
    bool isOuter = false;
};

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
                       std::vector<PlanarRing>& rings) {
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
        PlanarRing ring;
        ring.isOuter = wire.IsSame(outerWire);
        int wireEdges = 0;
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            ++wireEdges;
        }
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
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
            if (c3.IsNull() || c2.IsNull()) return false;
            const bool rev = edge.Orientation() == TopAbs_REVERSED;
            for (int i = 0; i < n; ++i) {  // endpoint owned by the next edge
                double t = rev ? 1.0 - double(i) / n : double(i) / n;
                ring.uv.push_back(c2->Value(f2 + (l2 - f2) * t));
                ring.p.push_back(c3->Value(f3 + (l3 - f3) * t));
            }
        }
        if (ring.uv.size() < 3) return false;
        rings.push_back(std::move(ring));
    }
    for (PlanarRing& r : rings) {
        double a = planarRingArea(r);
        if (std::abs(a) < 1e-14) return false;
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

    triangulateWeb(std::move(webOuter), std::move(webHoles), faceId, flip,
                   out);
    return true;
}

// Generalized minimal n-gon: the flattest topology a planar face can
// carry. One wire -> a single boundary n-gon on the exact solved border;
// holes -> the hole-bridged ear-clip web with zero interior vertices.
bool meshMinimalPlanar(const TopoDS_Face& face, const Model& model,
                       int faceId, const std::vector<int>& solvedEdge,
                       int radialDefault, MeshBuilder& out) {
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
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
    triangulateWeb(std::move(webOuter), std::move(webHoles), faceId, flip,
                   out);
    return true;
}

bool planQuadFill(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, FacePlan& plan) {
    // Any trimmed surface patch works — the grid lives in UV and maps
    // through the surface — except closed ones (the seam would need a
    // wrapped grid; revolution grids own those).
    if (surf.IsUClosed() || surf.IsVClosed()) return false;
    FacePlan probe;
    if (!collectPlanarLoops(face, surf, model, probe,
                            /*requirePlane=*/false)) {
        return false;
    }
    plan.loops = std::move(probe.loops);
    plan.uEdges = std::move(probe.uEdges);
    plan.constrains = true;
    plan.kind = MesherKind::QuadFill;
    return true;
}

// Quad-fill: a planar face of any outline gets an interior quad grid sized
// from its border density, and the gap between the grid and the exact
// boundary closes with the hole-bridged ear-clip web. Large clean quad
// flow on plates instead of fan triangulations.
bool meshQuadFill(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, int faceId,
                  const std::vector<int>& solvedEdge, int radialDefault,
                  double minSize, MeshBuilder& out) {
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
        return false;
    }
    if (rings.empty()) return false;
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    // Boundary segments (for spacing, containment, and clearance tests).
    struct Seg {
        gp_Pnt2d a, b;
    };
    std::vector<Seg> segs;
    std::vector<double> lens;
    double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
    for (const PlanarRing& r : rings) {
        for (size_t i = 0; i < r.uv.size(); ++i) {
            const gp_Pnt2d& a = r.uv[i];
            const gp_Pnt2d& b = r.uv[(i + 1) % r.uv.size()];
            segs.push_back({a, b});
            lens.push_back(a.Distance(b));
            umin = std::min(umin, a.X());
            umax = std::max(umax, a.X());
            vmin = std::min(vmin, a.Y());
            vmax = std::max(vmax, a.Y());
        }
    }
    if (segs.size() < 3) return false;
    std::sort(lens.begin(), lens.end());
    // Slightly finer than the border spacing: the rim web needs a cell of
    // clearance, so a coarser grid would waste most of the face on rim.
    // NOTE lens are UV distances; on a curved surface the metric differs
    // per direction, so the cell size splits into hu/hv using the surface
    // derivatives at the patch centre (a curved patch's UV chart can be
    // arbitrarily anisotropic — a cylinder's u is an angle).
    double h3 = std::max(0.55 * lens[lens.size() / 2], minSize);
    if (h3 < 1e-12) return false;
    double su = 1.0, sv = 1.0;
    {
        gp_Pnt sp;
        gp_Vec du, dv;
        surf.D1((umin + umax) / 2, (vmin + vmax) / 2, sp, du, dv);
        su = std::max(1e-9, du.Magnitude());
        sv = std::max(1e-9, dv.Magnitude());
    }
    // lens were measured in UV; estimate the 3D border spacing and derive
    // per-direction UV cell sizes from it.
    const double uvToWorld = (su + sv) / 2;
    double hWorld = h3 * uvToWorld;
    double hu = hWorld / su, hv = hWorld / sv;
    // Cap the grid size; a tiny median segment on a huge plate would
    // otherwise explode the cell count.
    while ((umax - umin) / hu * ((vmax - vmin) / hv) > 20000.0) {
        hu *= 1.5;
        hv *= 1.5;
    }

    auto insideDomain = [&](const gp_Pnt2d& p) {
        int crossings = 0;
        for (const Seg& s : segs) {
            if ((s.a.Y() > p.Y()) == (s.b.Y() > p.Y())) continue;
            double x = s.a.X() + (p.Y() - s.a.Y()) / (s.b.Y() - s.a.Y()) *
                                     (s.b.X() - s.a.X());
            if (x > p.X()) ++crossings;
        }
        return (crossings & 1) != 0;
    };

    const int nx = std::max(1, int((umax - umin) / hu));
    const int ny = std::max(1, int((vmax - vmin) / hv));
    // Center the grid in the bbox so border cells get equal clearance on
    // both sides instead of sitting flush against one edge.
    const double u0 = umin + 0.5 * ((umax - umin) - nx * hu);
    const double v0 = vmin + 0.5 * ((vmax - vmin) - ny * hv);
    auto cornerUV = [&](int i, int j) {
        return gp_Pnt2d(u0 + i * hu, v0 + j * hv);
    };

    // Bin boundary segments by grid row so the per-cell clearance test
    // only looks at nearby geometry.
    std::vector<std::vector<int>> rowSegs(ny + 1);
    for (int si = 0; si < int(segs.size()); ++si) {
        double y0 = std::min(segs[si].a.Y(), segs[si].b.Y()) - hv;
        double y1 = std::max(segs[si].a.Y(), segs[si].b.Y()) + hv;
        int j0 = std::max(0, int(std::floor((y0 - v0) / hv)));
        int j1 = std::min(ny, int(std::floor((y1 - v0) / hv)) + 1);
        for (int j = j0; j <= j1; ++j) rowSegs[j].push_back(si);
    }

    // A cell is kept when its four corners are inside the domain and no
    // boundary segment comes near its (slightly inflated) box — the rim
    // web needs breathing room to stay well-shaped.
    const double marginU = 0.30 * hu;
    const double marginV = 0.30 * hv;
    std::vector<char> keep(size_t(nx) * ny, 0);
    auto keepAt = [&](int i, int j) -> char& {
        return keep[size_t(j) * nx + i];
    };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            bool ok = true;
            for (int c = 0; c < 4 && ok; ++c) {
                ok = insideDomain(cornerUV(i + (c & 1), j + (c >> 1)));
            }
            if (!ok) continue;
            double x0 = u0 + i * hu - marginU, x1 = x0 + hu + 2 * marginU;
            double y0 = v0 + j * hv - marginV, y1 = y0 + hv + 2 * marginV;
            for (int si : rowSegs[j]) {
                const Seg& s = segs[si];
                // Conservative: reject when the segment's box overlaps the
                // inflated cell box (exact seg/box adds little here).
                if (std::max(s.a.X(), s.b.X()) < x0 ||
                    std::min(s.a.X(), s.b.X()) > x1 ||
                    std::max(s.a.Y(), s.b.Y()) < y0 ||
                    std::min(s.a.Y(), s.b.Y()) > y1) {
                    continue;
                }
                ok = false;
                break;
            }
            if (ok) keepAt(i, j) = 1;
        }
    }

    // Diagonal pinches (two kept cells touching only at a corner) would
    // give that corner four frontier edges; drop one cell until clean.
    for (bool changed = true; changed;) {
        changed = false;
        for (int j = 0; j + 1 < ny; ++j) {
            for (int i = 0; i + 1 < nx; ++i) {
                char& a = keepAt(i, j);
                char& b = keepAt(i + 1, j + 1);
                char& c = keepAt(i + 1, j);
                char& d = keepAt(i, j + 1);
                if (a && b && !c && !d) { a = 0; changed = true; }
                else if (c && d && !a && !b) { c = 0; changed = true; }
            }
        }
    }

    auto kept = [&](int i, int j) {
        return i >= 0 && j >= 0 && i < nx && j < ny && keepAt(i, j);
    };

    // Interior quad grid. Corner vertices are created on demand and shared
    // with the frontier loops, so the rim web welds to the grid exactly.
    std::map<std::pair<int, int>, uint32_t> cornerVert;
    auto vertAt = [&](int i, int j) {
        auto it = cornerVert.find({i, j});
        if (it != cornerVert.end()) return it->second;
        gp_Pnt2d uv = cornerUV(i, j);
        uint32_t v = out.addVertex(surf.Value(uv.X(), uv.Y()),
                                   {faceId, uv.X(), uv.Y()});
        cornerVert[{i, j}] = v;
        return v;
    };
    bool anyCell = false;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (!keepAt(i, j)) continue;
            anyCell = true;
            out.addPolygon({vertAt(i, j), vertAt(i + 1, j),
                            vertAt(i + 1, j + 1), vertAt(i, j + 1)},
                           faceId, flip);
        }
    }

    // Ring -> web points, and boundary ring vertices (anchorless).
    auto ringWeb = [&](const PlanarRing& r) {
        std::vector<WebPoint> web;
        for (size_t i = 0; i < r.uv.size(); ++i) {
            web.push_back({r.uv[i], out.addVertex(r.p[i], {})});
        }
        return web;
    };
    std::vector<WebPoint> faceOuter;
    std::vector<std::vector<WebPoint>> faceHoles;
    for (const PlanarRing& r : rings) {
        if (r.isOuter) faceOuter = ringWeb(r);
        else faceHoles.push_back(ringWeb(r));
    }
    if (faceOuter.size() < 3) return false;

    if (!anyCell) {  // no room for a grid: plain web over the whole face
        triangulateWeb(std::move(faceOuter), std::move(faceHoles), faceId,
                       flip, out);
        return true;
    }

    // Frontier: kept-region boundary edges, traced into closed loops on
    // the grid corners (pinch removal guarantees two frontier edges per
    // frontier corner).
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> adj;
    auto frontierEdge = [&](int i0, int j0, int i1, int j1) {
        adj[{i0, j0}].push_back({i1, j1});
        adj[{i1, j1}].push_back({i0, j0});
    };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (!keepAt(i, j)) continue;
            if (!kept(i, j - 1)) frontierEdge(i, j, i + 1, j);
            if (!kept(i, j + 1)) frontierEdge(i, j + 1, i + 1, j + 1);
            if (!kept(i - 1, j)) frontierEdge(i, j, i, j + 1);
            if (!kept(i + 1, j)) frontierEdge(i + 1, j, i + 1, j + 1);
        }
    }
    std::vector<std::vector<std::pair<int, int>>> frontierLoops;
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> used;
    for (const auto& [start, nbrs] : adj) {
        for (const auto& first : nbrs) {
            if (used.count({start, first})) continue;
            std::vector<std::pair<int, int>> loop{start};
            std::pair<int, int> prev = start, cur = first;
            used.insert({start, first});
            used.insert({first, start});  // one traversal per edge
            bool closed = false;
            for (size_t guard = 0; guard < adj.size() * 4 + 4; ++guard) {
                if (cur == start) { closed = true; break; }
                loop.push_back(cur);
                const auto& next = adj.at(cur);
                std::pair<int, int> step{-1, -1};
                for (const auto& n : next) {
                    if (n != prev && !used.count({cur, n})) {
                        step = n;
                        break;
                    }
                }
                if (step.first < 0) break;
                used.insert({cur, step});
                used.insert({step, cur});
                prev = cur;
                cur = step;
            }
            if (closed && loop.size() >= 4) {
                frontierLoops.push_back(std::move(loop));
            }
        }
    }
    if (frontierLoops.empty()) return false;  // shouldn't happen with cells

    // Every frontier loop either encloses a web POCKET (its inside is not
    // kept: it acts as that pocket's outer ring) or wraps a kept ISLAND
    // (it is a hole of the enclosing web region).
    struct Region {
        std::vector<WebPoint> outer;
        std::vector<std::vector<WebPoint>> holes;
        double area = 0;  // |signed| of the outer, for nesting
    };
    std::vector<Region> regions;
    regions.push_back({std::move(faceOuter), {}, 1e300});

    auto loopWeb = [&](const std::vector<std::pair<int, int>>& loop) {
        std::vector<WebPoint> web;
        for (const auto& [i, j] : loop) web.push_back({cornerUV(i, j),
                                                       vertAt(i, j)});
        return web;
    };
    auto signedAreaOf = [&](const std::vector<WebPoint>& web) {
        double a = 0;
        for (size_t i = 0; i < web.size(); ++i) {
            const gp_Pnt2d& p = web[i].uv;
            const gp_Pnt2d& q = web[(i + 1) % web.size()].uv;
            a += p.X() * q.Y() - q.X() * p.Y();
        }
        return a / 2;
    };
    std::vector<std::vector<WebPoint>> pendingHoles;
    for (const auto& loop : frontierLoops) {
        std::vector<WebPoint> web = loopWeb(loop);
        double area = signedAreaOf(web);
        // A point just inside the loop: offset from the first edge's
        // midpoint toward the interior; kept there => island (hole).
        gp_Pnt2d m((web[0].uv.X() + web[1].uv.X()) / 2,
                   (web[0].uv.Y() + web[1].uv.Y()) / 2);
        gp_Pnt2d dir(web[1].uv.X() - web[0].uv.X(),
                     web[1].uv.Y() - web[0].uv.Y());
        double side = area > 0 ? 1.0 : -1.0;  // interior is left of CCW
        gp_Pnt2d probe(m.X() - side * dir.Y() * 0.25,
                       m.Y() + side * dir.X() * 0.25);
        int pi = int(std::floor((probe.X() - u0) / hu));
        int pj = int(std::floor((probe.Y() - v0) / hv));
        if (kept(pi, pj)) {
            // Island: a hole of whichever region contains it.
            if (area > 0) {
                std::reverse(web.begin(), web.end());  // holes wind CW
            }
            pendingHoles.push_back(std::move(web));
        } else {
            // Pocket: its own web region, outer CCW.
            if (area < 0) std::reverse(web.begin(), web.end());
            regions.push_back({std::move(web), {}, std::abs(area)});
        }
    }
    for (auto& hole : faceHoles) pendingHoles.push_back(std::move(hole));

    // Assign each hole to the smallest region whose outer contains it.
    auto containsPoint = [&](const std::vector<WebPoint>& ring,
                             const gp_Pnt2d& p) {
        int crossings = 0;
        for (size_t i = 0; i < ring.size(); ++i) {
            const gp_Pnt2d& a = ring[i].uv;
            const gp_Pnt2d& b = ring[(i + 1) % ring.size()].uv;
            if ((a.Y() > p.Y()) == (b.Y() > p.Y())) continue;
            double x = a.X() + (p.Y() - a.Y()) / (b.Y() - a.Y()) *
                                   (b.X() - a.X());
            if (x > p.X()) ++crossings;
        }
        return (crossings & 1) != 0;
    };
    for (auto& hole : pendingHoles) {
        int best = 0;
        double bestArea = 1e300;
        for (size_t r = 0; r < regions.size(); ++r) {
            if (regions[r].area >= bestArea) continue;
            if (r == 0 || containsPoint(regions[r].outer, hole[0].uv)) {
                best = int(r);
                bestArea = regions[r].area;
            }
        }
        regions[best].holes.push_back(std::move(hole));
    }
    for (Region& r : regions) {
        triangulateWeb(std::move(r.outer), std::move(r.holes), faceId, flip,
                       out);
    }
    return true;
}

FacePlan planFace(int fid, const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings,
                  GenerationCache* cache) {
    const TopoDS_Face face = TopoDS::Face(model.faces(fid));
    const FaceMeshSettings& s = settings.forFace(fid);
    const FaceInfo& info = analysis.faces[fid - 1];
    BRepAdaptor_Surface surf(face);
    FacePlan plan;

    // Geometry-only probe results memoize in the cache (the classifier
    // calls dominate planning cost and never change for a model).
    auto revCovers = [&] {
        if (cache) {
            auto it = cache->revolutionCovers.find(fid);
            if (it != cache->revolutionCovers.end()) return it->second;
        }
        bool v = revolutionCovers(face);
        if (cache) cache->revolutionCovers[fid] = v;
        return v;
    };
    auto coonsOk = [&](CoonsPatch& patch) {
        // Patch construction is cheap; a memoized NEGATIVE skips it (and
        // the probes); a positive still rebuilds the (cheap) patch data.
        if (cache) {
            auto it = cache->coonsValid.find(fid);
            if (it != cache->coonsValid.end() && !it->second) return false;
        }
        bool v = makeCoonsPatch(face, model, patch);
        if (cache) cache->coonsValid[fid] = v;
        return v;
    };

    auto finishRevolution = [&]() {
        plan.kind = MesherKind::RevolutionGrid;
        collectIsoEdges(face, model, info.edgeIds, plan,
                        /*skipNonIso=*/true);
        if (plan.uEdges.empty()) plan.constrains = false;
        if (!s.linkRims && plan.uEdges.size() == 2) plan.linkRims = false;
    };

    // The user can force a strategy; if it can't build on this face the
    // plan degrades to plain triangulation so the choice is visible.
    if (s.forceMesher > 0) {
        MesherKind want = MesherKind(s.forceMesher - 1);
        switch (want) {
            case MesherKind::RevolutionGrid:
                // Forced: also accept u-closed freeform surfaces (revolved
                // bsplines and the like) that the auto path won't touch.
                if (isClosedRevolution(surf) || surf.IsUClosed()) {
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
                if (parametricGridFits(face, surf, std::max(1, s.gridU),
                                       std::max(1, s.gridV))) {
                    plan.kind = want;
                    collectIsoEdges(face, model, info.edgeIds, plan);
                    if (plan.uEdges.size() != 2 || plan.vEdges.size() != 2) {
                        plan.constrains = false;
                    }
                    return plan;
                }
                break;
            case MesherKind::MinimalNGon:
                // Any planar face can go minimal: single wire -> one exact
                // boundary n-gon, holes -> bridged web, zero interior verts.
                if (planMinimalPlanar(face, surf, model, plan)) return plan;
                break;
            case MesherKind::CoonsGrid: {
                CoonsPatch patch;
                if (coonsOk(patch)) {
                    plan.kind = MesherKind::CoonsGrid;
                    plan.uEdges = {patch.edgeIds[0], patch.edgeIds[2]};
                    plan.vEdges = {patch.edgeIds[1], patch.edgeIds[3]};
                    plan.constrains = true;
                    return plan;
                }
                break;
            }
            case MesherKind::AnnulusRing:
                // Forced: no ring-shape gate — the user asked for the band.
                if (planAnnulus(face, model, plan, /*requireRing=*/false)) {
                    return plan;
                }
                break;
            case MesherKind::PlateWeb:
                if (planPlateWeb(face, surf, model, plan,
                                 /*requireRoundHoles=*/false)) {
                    return plan;
                }
                break;
            case MesherKind::QuadFill:
                if (planQuadFill(face, surf, model, plan)) return plan;
                break;
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

    if (isClosedRevolution(surf) && revCovers()) {
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

    // Game-topology minimal (the per-face/default flag): any flat face
    // collapses to its boundary — one n-gon, or a hole-bridged flat web.
    if (s.minimal && planMinimalPlanar(face, surf, model, plan)) return plan;

    if (planRingJunction(face, model, plan)) return plan;

    // Auto picks stay conservative: the annulus band only for actual
    // concentric rings, the plate web only for compact (bolt-style) holes.
    // Everything else keeps the fallback unless the user forces a mesher.
    if (planAnnulus(face, model, plan, /*requireRing=*/true)) return plan;

    if (planPlateWeb(face, surf, model, plan, /*requireRoundHoles=*/true)) {
        return plan;
    }

    // Curved surfaces skip the parametric grid on auto: its border rows
    // sample the SURFACE uniformly, which never lands vertex-for-vertex on
    // a neighbour's sampling of the shared edge. Four-sided curved faces
    // fall through to the Coons patch below, whose border rows evaluate on
    // the 3D edge curves (weld-exact); the rest conform as freeform.
    if (surf.GetType() == GeomAbs_Plane &&
        parametricGridFits(face, surf, std::max(1, s.gridU),
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

    // Four-sided freeform/trimmed faces get a structured Coons grid; the
    // across-the-blend direction of a fillet strip is whichever side pair
    // is shorter in 3D.
    {
        CoonsPatch patch;
        if (coonsOk(patch)) {
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

    // Flat faces with quad-dominant set get the structured grid + rim
    // fill instead of OCCT triangulation + pairing.
    if (s.quadDominant && planQuadFill(face, surf, model, plan)) return plan;

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
        // Loop-based plans (plate webs, minimal planar) never tie their
        // edges together: each hole/border edge solves on its own (the
        // bore through a hole drives that hole).
        if (!plan.loops.empty()) continue;
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
    // Curvature-adaptive proposals: an edge's count comes from tangential-
    // deflection sampling of its curve under the proposing face's chord +
    // angle tolerances — big arcs get more segments than small ones,
    // straight edges get 1. Memoized per (edge, tolerances) for this solve.
    std::map<std::tuple<int, long long, long long>, int> adCache;
    auto adaptiveCount = [&](int eid, const FaceMeshSettings& s) {
        auto key = std::make_tuple(eid, (long long)(s.chordTolerance * 1e9),
                                   (long long)(s.angleToleranceDeg * 1e6));
        auto it = adCache.find(key);
        if (it != adCache.end()) return it->second;
        int n = 1;
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (!BRep_Tool::Degenerated(edge)) {
            double f, l;
            if (!BRep_Tool::Curve(edge, f, l).IsNull()) {
                BRepAdaptor_Curve c(edge);
                double ang =
                    std::max(1.0, s.angleToleranceDeg) * M_PI / 180.0;
                double chord = std::max(1e-9, s.chordTolerance);
                try {
                    GCPnts_TangentialDeflection td(c, ang, chord, 2);
                    n = std::clamp(td.NbPoints() - 1, 1, 256);
                } catch (const Standard_Failure&) {
                }
                // Closed edges (full circles) keep a sane ring floor.
                if (c.Value(c.FirstParameter())
                        .Distance(c.Value(c.LastParameter())) < 1e-9) {
                    n = std::max(n, 6);
                }
            }
        }
        adCache[key] = n;
        return n;
    };
    // Propose `flat` onto a set, or — adaptive — each edge's own
    // curvature count with `floorA` as the minimum.
    auto proposeSet = [&](const std::vector<int>& edges, int flat,
                          int floorA, bool adaptive,
                          const FaceMeshSettings& s, bool overridden) {
        if (!adaptive) {
            propose(edges, flat, overridden);
            return;
        }
        for (int eid : edges) {
            propose({eid}, std::max(floorA, adaptiveCount(eid, s)),
                    overridden);
        }
    };

    for (const auto& [fid, plan] : plans) {
        if (!plan.constrains) continue;
        const FaceMeshSettings& s = settings.forFace(fid);
        const bool overridden = settings.perFace.count(fid) > 0;
        if (!plan.loops.empty()) {
            // Explicit boundary control: a TOTAL vertex count around the
            // outer loop, distributed across its edges by arc length and
            // pinned — it drives the neighbouring walls' shared edges too.
            const bool boundarySet = s.boundary > 0 && !plan.loops[0].empty();
            if (boundarySet) {
                const std::vector<int>& outer = plan.loops[0];
                std::vector<double> lens(outer.size(), 1.0);
                double sum = 0;
                for (size_t i = 0; i < outer.size(); ++i) {
                    BRepAdaptor_Curve c(
                        TopoDS::Edge(model.edges(outer[i])));
                    lens[i] = std::max(1e-12,
                                       GCPnts_AbscissaPoint::Length(c));
                    sum += lens[i];
                }
                int total = std::max(int(outer.size()), s.boundary);
                int assigned = 0;
                for (size_t i = 0; i < outer.size(); ++i) {
                    int share =
                        i + 1 == outer.size()
                            ? std::max(1, total - assigned)
                            : std::max(1,
                                       int(std::floor(total * lens[i] / sum +
                                                      0.5)));
                    assigned += share;
                    propose({outer[i]}, share, /*overridden=*/true);
                }
            }
            // Every other border edge proposes independently. Plate webs
            // share the radial default out per loop (a one-edge hole
            // circle gets all of it; a bore's larger proposal still wins).
            // Minimal planar proposes the floor — flattest possible — and
            // lets the neighbours drive any edge that needs more; it must
            // never PIN a shared edge down, even as an explicit override.
            for (size_t li = boundarySet ? 1 : 0; li < plan.loops.size();
                 ++li) {
                const auto& loop = plan.loops[li];
                bool minimal = plan.kind == MesherKind::MinimalNGon;
                int per = minimal ? 1
                                  : std::max(1, std::max(3, s.radial) /
                                                    int(loop.size()));
                for (int eid : loop) {
                    if (minimal) {
                        propose({eid}, 1, false);
                    } else {
                        proposeSet({eid}, per, 1, s.adaptive, s, overridden);
                    }
                }
            }
        } else if (plan.kind == MesherKind::PlanarGrid ||
            plan.kind == MesherKind::CoonsGrid ||
            plan.kind == MesherKind::MinimalNGon ||
            plan.kind == MesherKind::RingJunction) {
            int nu = std::max(1, plan.isFillet && plan.acrossIsU
                                     ? s.filletLoops : s.gridU);
            int nv = std::max(1, plan.isFillet && !plan.acrossIsU
                                     ? s.filletLoops : s.gridV);
            // Support loops across a blend stay a deliberate choice; the
            // other directions adapt to their edges' curvature.
            bool adU = s.adaptive && !(plan.isFillet && plan.acrossIsU);
            bool adV = s.adaptive && !(plan.isFillet && !plan.acrossIsU);
            proposeSet(plan.uEdges, nu, nu, adU, s, overridden);
            proposeSet(plan.vEdges, nv, nv, adV, s, overridden);
        } else if (plan.kind == MesherKind::AnnulusRing) {
            // Both loops are rings; they solve independently (their own
            // neighbours usually drive them).
            proposeSet(plan.uEdges, std::max(3, s.radial), 3, s.adaptive, s,
                       overridden);
            proposeSet(plan.vEdges, std::max(3, s.radial), 3, s.adaptive, s,
                       overridden);
        } else {  // revolution sides and disk caps subdivide rings radially
            if (!plan.linkRims && plan.uEdges.size() == 2) {
                // Unlinked rims: each ring solves on its own (pin per-edge
                // or via the rim fields to make them differ).
                proposeSet({plan.uEdges[0]}, std::max(3, s.radial), 3,
                           s.adaptive, s, overridden);
                proposeSet({plan.uEdges[1]}, std::max(3, s.radial), 3,
                           s.adaptive, s, overridden);
            } else {
                proposeSet(plan.uEdges, std::max(3, s.radial), 3, s.adaptive,
                           s, overridden);
            }
            // Explicit axial acts as the floor along the axis; profile
            // curvature (a vase wall) adds what it needs.
            proposeSet(plan.vEdges, std::max(1, s.axial),
                       std::max(1, s.axial), s.adaptive, s, overridden);
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
                         int nB, double uPhase, MeshBuilder& out) {
    nA = std::max(3, nA);
    nB = std::max(3, nB);
    const double u0 = uPhase;
    const double uRange = surf.LastUParameter() - surf.FirstUParameter();
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
// Phase-align a revolution grid's u sampling to a rim EDGE's curve start:
// two analytic faces sharing that circle then sample the identical points
// (each surface's own u origin can be rotated arbitrarily — torus vs
// cylinder — which used to leave every shared rim vertex slightly off).
double revolutionUPhase(const BRepAdaptor_Surface& surf,
                        const Model& model, int rimEdgeId) {
    const double u0 = surf.FirstUParameter();
    if (rimEdgeId < 1 || rimEdgeId > model.edgeCount()) return u0;
    const TopoDS_Edge edge = TopoDS::Edge(model.edges(rimEdgeId));
    if (BRep_Tool::Degenerated(edge)) return u0;
    double f, l;
    if (BRep_Tool::Curve(edge, f, l).IsNull()) return u0;
    BRepAdaptor_Curve c(edge);
    const gp_Pnt p0 = c.Value(c.FirstParameter());
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
    const double v = dFirst <= dLast ? surf.FirstVParameter()
                                     : surf.LastVParameter();
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

void meshRevolutionGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        int faceId, int nu, int nv, double uPhase,
                        MeshBuilder& out) {
    nu = std::max(3, nu);
    nv = std::max(1, nv);
    const double u0 = uPhase;
    const double v0 = surf.FirstVParameter();
    const double du = (surf.LastUParameter() - surf.FirstUParameter()) / nu;
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
        return (k == MesherKind::Fallback || k == MesherKind::QuadDominant ||
                k == MesherKind::AnnulusRing ||
                !plans.at(fid).loops.empty() ||
                (k == MesherKind::PlanarGrid &&
                 !plans.at(fid).constrains)) &&
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
            const bool analyticNb = nfid >= 1 && isAnalytic(nfid);
            // A pinned edge with no analytic driver (both sides freeform,
            // or the neighbour deleted) resamples this border to exactly
            // the pinned count with vertices ON the curve — count control
            // that keeps the curvature.
            auto pinIt = settings.perEdge.find(eid);
            const bool pinnedResample =
                !analyticNb && pinIt != settings.perEdge.end() &&
                pinIt->second >= 2;
            // Freeform-to-freeform seams: the DENSER side is the
            // authority (ties: lower id) and only the sparser side moves,
            // exactly like the analytic case — snapping never collapses
            // because the target chain has at least as many verts.
            const bool freeformSeam = !analyticNb && !pinnedResample &&
                                      nfid >= 1 && isFreeform(nfid);
            if (!analyticNb && !pinnedResample && !freeformSeam) continue;
            dbg("conform: face %d edge %d (%s)", fid, eid,
                analyticNb ? "analytic neighbour"
                : pinnedResample ? "pinned resample"
                                 : "freeform seam");

            BRepAdaptor_Curve curve(edge);
            const double f = curve.FirstParameter(), l = curve.LastParameter();
            const bool closed = curve.IsClosed();
            const double period = l - f;
            GCPnts_AbscissaPoint lenTool;
            double edgeLen = GCPnts_AbscissaPoint::Length(curve);
            (void)lenTool;

            // Coarse polyline of the edge: candidates farther from it
            // than the tolerance (plus the sampling slack) can't project
            // onto the curve, so the expensive Extrema never runs for the
            // bulk of a neighbour's vertices.
            std::array<gp_Pnt, 33> coarse;
            for (int i = 0; i < 33; ++i) {
                coarse[i] = curve.Value(f + (l - f) * i / 32.0);
            }
            const double slack = edgeLen / 16.0;

            // Exact distance/parameter on the curve for a mesh vertex.
            auto project = [&](uint32_t v, double tol,
                               double* paramOut) -> bool {
                gp_Pnt p(mesh.vertices[v][0], mesh.vertices[v][1],
                         mesh.vertices[v][2]);
                double quick = 1e300;
                for (const gp_Pnt& c : coarse) {
                    quick = std::min(quick, p.SquareDistance(c));
                }
                if (quick > (tol + slack) * (tol + slack)) return false;
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

            // The authoritative chain: the analytic side's verts on this
            // edge, or — for a pinned resample — fresh uniform samples on
            // the curve itself. Coons verts evaluate through the pcurve,
            // which only agrees with the 3D curve to the edge tolerance,
            // so include it.
            const double tolTarget =
                std::max(1e-6 * (1.0 + edgeLen),
                         10.0 * BRep_Tool::Tolerance(edge));
            // My border verts on this edge (needed up front: seams pick
            // the denser side as authority before any vertex moves).
            const double tolMoverPre = std::max(
                1e-6 * (1.0 + edgeLen),
                std::max(settings.forFace(fid).chordTolerance,
                         nfid >= 1 ? settings.forFace(nfid).chordTolerance
                                   : 0.0) *
                    1.2);
            std::map<uint32_t, double> movers;  // vert -> snapped param
            for (uint32_t v : borderVerts) {
                double t;
                if (project(v, tolMoverPre, &t)) movers[v] = t;
            }
            if (movers.empty()) continue;

            std::vector<EdgeParamPoint> targets;
            if (analyticNb || freeformSeam) {
                // A freeform authority's border verts sit off the curve by
                // up to its chord sagitta, so accept a looser projection.
                const double tolT =
                    freeformSeam
                        ? std::max(tolTarget,
                                   settings.forFace(nfid).chordTolerance *
                                       1.2)
                        : tolTarget;
                for (size_t v = range[nfid][0]; v < range[nfid][1]; ++v) {
                    double t;
                    if (project(uint32_t(v), tolT, &t)) {
                        targets.push_back({uint32_t(v), t});
                    }
                }
            } else {
                const int n = pinIt->second;
                const int count = closed ? n : n + 1;
                for (int i = 0; i < count; ++i) {
                    double t = f + (l - f) * i / double(n);
                    gp_Pnt q = curve.Value(t);
                    uint32_t nv = uint32_t(mesh.vertices.size());
                    mesh.vertices.push_back({q.X(), q.Y(), q.Z()});
                    mesh.anchors.push_back({});
                    targets.push_back({nv, t});
                }
            }
            if (targets.size() < 2) continue;
            // Seam authority: only the sparser side conforms; the denser
            // (or equal-count lower-id) side keeps its chain.
            if (freeformSeam &&
                (targets.size() < movers.size() ||
                 (targets.size() == movers.size() && fid < nfid))) {
                continue;
            }
            std::sort(targets.begin(), targets.end(),
                      [](const EdgeParamPoint& a, const EdgeParamPoint& b) {
                          return a.param < b.param;
                      });

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
                  const GenerationSettings& settings, GenerationReport* report,
                  GenerationCache* cache) {
    dbg("generate: begin (%d faces, %d edges, parallel=%d, conform=%d)",
        model.faceCount(), model.edgeCount(), settings.parallelMeshing ? 1 : 0,
        settings.conformBorders ? 1 : 0);
    std::map<int, FacePlan> plans;
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        FacePlan plan = planFace(fid, model, analysis, settings, cache);
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
    // Flat per-edge count table: lets meshers consume per-edge counts from
    // worker threads (union-find lookups path-compress, so countFor can't
    // run concurrently).
    std::vector<int> solvedEdge(model.edgeCount() + 1, 0);
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        solvedEdge[eid] = density.countFor(eid, 0);
    }
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
            case MesherKind::AnnulusRing:
                counts[fid] = {solved(plan.uEdges, s.radial),
                               solved(plan.vEdges, s.radial), 0};
                break;
            default:
                break;
        }
    }

    // Cache keys: everything that shapes a face's part. A hit skips the
    // (expensive) meshing entirely and reuses the stored part.
    std::vector<std::string> cacheKey(faceN + 1);
    std::vector<bool> cached(faceN + 1, false);
    for (int fid = 1; fid <= faceN; ++fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        const FacePlan& plan = plans.at(fid);
        char key[320];
        std::snprintf(
            key, sizeof key,
            "k%d c%d f%d a%d l%d q%d|%d,%d,%d|r%d x%d u%d v%d cap%d ch%.6g "
            "an%.6g fl%d fh%.6g jr%d qd%d mn%d ex%d ms%.6g rd%d sq%d",
            int(plan.kind), plan.constrains ? 1 : 0, plan.isFillet ? 1 : 0,
            plan.acrossIsU ? 1 : 0, plan.linkRims ? 1 : 0,
            plan.forceFallbackQuads, counts[fid][0], counts[fid][1],
            counts[fid][2], s.radial, s.axial, s.gridU, s.gridV, int(s.cap),
            s.chordTolerance, s.angleToleranceDeg, s.filletLoops,
            s.filletHold, s.junctionRings, s.quadDominant ? 1 : 0,
            s.minimal ? 1 : 0, s.exclude ? 1 : 0, s.minSize,
            s.relativeDeviation ? 1 : 0, s.squareCollar ? 1 : 0);
        cacheKey[fid] = key;
        if (plan.kind == MesherKind::AnnulusRing || !plan.loops.empty()) {
            for (int eid : plan.uEdges) {
                cacheKey[fid] += "u" + std::to_string(solvedEdge[eid]);
            }
            for (int eid : plan.vEdges) {
                cacheKey[fid] += "v" + std::to_string(solvedEdge[eid]);
            }
        }
    }

    // Mesh every face into its own part, in parallel, then merge in face
    // order so the output is deterministic (identical to the serial order).
    std::vector<PolyMesh> parts(faceN + 1);
    int cacheHits = 0;
    if (cache) {
        for (int fid = 1; fid <= faceN; ++fid) {
            auto it = cache->faces.find(fid);
            if (it != cache->faces.end() &&
                it->second.first == cacheKey[fid]) {
                parts[fid] = it->second.second;  // copy: merge mutates
                cached[fid] = true;
                ++cacheHits;
            }
        }
    }
    auto meshFace = [&](int fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        if (s.exclude) return;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FacePlan& plan = plans.at(fid);
        dbg("mesh face %d: %s", fid, mesherKindName(plan.kind));
        BRepAdaptor_Surface surf(face);
        MeshBuilder out(parts[fid]);
        const int nu = counts[fid][0], nv = counts[fid][1];

        auto revPhase = [&]() {
            return plan.uEdges.empty()
                       ? surf.FirstUParameter()
                       : revolutionUPhase(surf, model, plan.uEdges[0]);
        };
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
                    meshRevolutionTaper(face, surf, fid, nA, nB,
                                        revPhase(), out);
                } else {
                    meshRevolutionGrid(face, surf, fid, nu, nv, revPhase(),
                                       out);
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
                if (!plan.loops.empty()) {
                    if (!meshMinimalPlanar(face, model, fid, solvedEdge,
                                           s.radial, out)) {
                        meshFallback(face, surf, fid, s, out);
                    }
                } else {
                    meshMinimalNGon(face, surf, fid, nu, nv, out);
                }
                break;
            case MesherKind::RingJunction:
                meshRingJunction(face, surf, plan.circ, fid, nu, nv,
                                 s.junctionRings, out);
                break;
            case MesherKind::AnnulusRing:
                meshAnnulusRing(face, model, fid, plan.uEdges, plan.vEdges,
                                solvedEdge, s.radial, out);
                break;
            case MesherKind::PlateWeb:
                if (!meshPlateWeb(face, surf, model, fid, solvedEdge,
                                  s.radial, s.junctionRings, s.squareCollar,
                                  out)) {
                    meshFallback(face, surf, fid, s, out);
                }
                break;
            case MesherKind::QuadFill:
                if (!meshQuadFill(face, surf, model, fid, solvedEdge,
                                  s.radial, s.minSize, out)) {
                    meshFallback(face, surf, fid, s, out);
                }
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
        // Safety net: any directed edge repeated inside one face's part is
        // degenerate topology (it would leak non-manifold edges into the
        // weld). Throw the part away and triangulate honestly instead.
        if (plan.kind != MesherKind::Fallback &&
            plan.kind != MesherKind::QuadDominant) {
            std::set<std::pair<uint32_t, uint32_t>> seen;
            bool sane = true;
            for (const auto& poly : parts[fid].polygons) {
                for (size_t i = 0; i < poly.size() && sane; ++i) {
                    if (!seen.insert({poly[i],
                                      poly[(i + 1) % poly.size()]})
                             .second) {
                        sane = false;
                    }
                }
                if (!sane) break;
            }
            if (!sane) {
                dbg("mesh face %d: self-check failed (%s), falling back",
                    fid, mesherKindName(plan.kind));
                parts[fid] = PolyMesh();
                MeshBuilder retry(parts[fid]);
                meshFallback(face, surf, fid, s, retry);
            }
        }
    };

    unsigned threads = std::min<unsigned>(
        std::max(1u, std::thread::hardware_concurrency()), unsigned(faceN));
    if (!settings.parallelMeshing) threads = 1;
    dbg("generate: meshing on %u thread(s), %d cached", threads, cacheHits);
    auto meshFaceCached = [&](int fid) {
        if (!cached[fid]) meshFace(fid);
    };
    if (threads <= 1) {
        for (int fid = 1; fid <= faceN; ++fid) meshFaceCached(fid);
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
                        meshFaceCached(fid);
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

    if (cache) {
        for (int fid = 1; fid <= faceN; ++fid) {
            if (!cached[fid]) {
                cache->faces[fid] = {cacheKey[fid], parts[fid]};
            }
        }
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
