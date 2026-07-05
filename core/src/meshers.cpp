#include "weft/meshers.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <Standard_Failure.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box2d.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GProp_GProps.hxx>
#include <BRep_Tool.hxx>
#include <ElCLib.hxx>
#include <ElSLib.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Vertex.hxx>
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
#include <memory>
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

    const PolyMesh& mesh() const { return mesh_; }

private:
    PolyMesh& mesh_;
};

// ---------------------------------------------------------------------------
// Planning: decide a strategy per face and collect the edges whose
// subdivision counts that strategy consumes, split by parametric direction.

struct FacePlan {
    // Interior closed trim wires on a full revolution band (slots, holes
    // through the wall): the band still meshes as a revolution grid; the
    // cells these wires cover are removed and webbed to the wire's exact
    // border sampling afterwards.
    std::vector<std::vector<int>> insertWires;
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
    // Chained Coons: edge ids per side (wire order) when any side is a
    // chain of several edges. Opposite sides then match by SUM of their
    // per-edge counts (solveDensity's chain pass) instead of union-find.
    std::array<std::vector<int>, 4> coonsSides;
    // PlateWeb: every boundary wire's edge chain (loops[0] = outer wire).
    // Each edge solves independently — a bore drives its own hole loop.
    std::vector<std::vector<int>> loops;
};

// A genuine full revolution band's boundary consists only of its two
// v-rims and (possibly) a seam. Sampled through the pcurves: an edge that
// is neither a rim-hugging v-iso nor a u-iso seam means the face is
// TRIMMED, and drawing the full band would over-mesh across the trim —
// the classifier probe grid can miss small notches entirely.
bool edgesHugRims(const TopoDS_Face& face, const BRepAdaptor_Surface& surf) {
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double uspan = std::max(1e-12, u1 - u0);
    const double vspan = std::max(1e-12, v1 - v0);
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (BRep_Tool::Degenerated(edge)) continue;
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f, l);
        // A boundary edge we can't even place on the surface is exactly
        // the kind the probe grid misses — refuse the full band.
        if (pc.IsNull()) return false;
        double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
        for (int k = 0; k <= 4; ++k) {
            gp_Pnt2d uv = pc->Value(f + (l - f) * k / 4.0);
            umin = std::min(umin, uv.X());
            umax = std::max(umax, uv.X());
            vmin = std::min(vmin, uv.Y());
            vmax = std::max(vmax, uv.Y());
        }
        if (vmax - vmin < 0.02 * vspan) {  // v-iso: must hug a rim
            double v = (vmin + vmax) / 2;
            if (std::min(std::abs(v - v0), std::abs(v - v1)) > 0.05 * vspan) {
                return false;  // a ring mid-band: the face is split there
            }
        } else if (umax - umin < 0.02 * uspan) {
            // u-iso: only a true SEAM (full v traversal) belongs to a
            // full band; a partial u-iso edge is a trim boundary.
            if (vmax - vmin < 0.9 * vspan) return false;
        } else {
            return false;  // slanted/trimmed boundary
        }
    }
    return true;
}

// Like edgesHugRims, but a wire living STRICTLY inside the band (a slot
// or hole through the wall) is collected as an insert instead of
// disqualifying the whole face.
bool edgesHugRimsOrInserts(const TopoDS_Face& face,
                           const BRepAdaptor_Surface& surf,
                           const Model& model,
                           std::vector<std::vector<int>>& wires) {
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double uspan = std::max(1e-12, u1 - u0);
    const double vspan = std::max(1e-12, v1 - v0);
    wires.clear();
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        double wu0 = 1e300, wu1 = -1e300, wv0 = 1e300, wv1 = -1e300;
        std::vector<int> ids;
        bool pcOk = true;
        for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More();
             ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(edge)) continue;
            double f, l;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f, l);
            if (pc.IsNull()) { pcOk = false; break; }
            for (int k = 0; k <= 8; ++k) {
                gp_Pnt2d uv = pc->Value(f + (l - f) * k / 8.0);
                wu0 = std::min(wu0, uv.X());
                wu1 = std::max(wu1, uv.X());
                wv0 = std::min(wv0, uv.Y());
                wv1 = std::max(wv1, uv.Y());
            }
            int eid = model.edges.FindIndex(edge);
            if (eid > 0) ids.push_back(eid);
        }
        if (!pcOk) return false;
        if (ids.empty()) continue;
        const bool interior = wu0 > u0 + 0.03 * uspan &&
                              wu1 < u1 - 0.03 * uspan &&
                              wv0 > v0 + 0.03 * vspan &&
                              wv1 < v1 - 0.03 * vspan;
        if (interior) {
            wires.push_back(std::move(ids));
            continue;
        }
        // Not interior: every edge of this wire must be a rim or seam.
        for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More();
             ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(edge)) continue;
            double f, l;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f, l);
            double eu0 = 1e300, eu1 = -1e300, ev0 = 1e300, ev1 = -1e300;
            for (int k = 0; k <= 4; ++k) {
                gp_Pnt2d uv = pc->Value(f + (l - f) * k / 4.0);
                eu0 = std::min(eu0, uv.X());
                eu1 = std::max(eu1, uv.X());
                ev0 = std::min(ev0, uv.Y());
                ev1 = std::max(ev1, uv.Y());
            }
            if (ev1 - ev0 < 0.02 * vspan) {
                double v = (ev0 + ev1) / 2;
                if (std::min(std::abs(v - v0), std::abs(v - v1)) >
                    0.05 * vspan) {
                    return false;
                }
            } else if (eu1 - eu0 < 0.02 * uspan) {
                if (ev1 - ev0 < 0.9 * vspan) return false;
            } else {
                return false;
            }
        }
    }
    return true;
}

bool isClosedRevolution(const BRepAdaptor_Surface& surf) {
    switch (surf.GetType()) {
        case GeomAbs_Cylinder:
        case GeomAbs_Cone:
        case GeomAbs_Sphere:
        case GeomAbs_Torus:
        case GeomAbs_SurfaceOfRevolution: return surf.IsUClosed();
        default: break;
    }
    // Revolved bsplines (CAD kernels export revolves as NURBS all the
    // time): u-closed is what the ring meshers actually need — exact
    // rim rows, phase-aligned columns — not the analytic type tag.
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

    // The grid meshes the UV bounding box, so the face must actually FILL
    // its box. Node/center classification alone is far too sparse at low
    // counts (a 1x1 grid probes 5 points) and lets a near-rectangle with
    // small notches through — the grid then overlaps the notch faces. For
    // a plane u/v are arc length, so bbox area is exact: compare it to the
    // true face area.
    if (surf.GetType() == GeomAbs_Plane) {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        const double rect = (umax - umin) * (vmax - vmin);
        if (rect <= 0 || props.Mass() < 0.999 * rect) return false;
    }

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
    // Three-sided faces: side 3 collapses to the corner shared by sides
    // 0 and 2 — the grid gains a pole there (fan row), like a revolution
    // apex, and the other three sides keep full quad flow.
    bool collapsedLast = false;
    // The collapsed side came from a DEGENERATE edge (a bspline pole):
    // side 3 walks the pole's pcurve in UV (it spans real parameter
    // space there) while its 3D image stays one point.
    bool poleCurve = false;
    // Corner stub: a fifth edge much shorter than the sides (CAD noise
    // splitting one corner). The chain skips it; the mesher stitches its
    // samples into the corner polygon so the neighbour's seam welds.
    int stubEdgeId = 0;
    bool stubRev = false;
    Handle(Geom2d_Curve) stubPc;
    double stubFirst = 0.0, stubLast = 0.0;
    // Chained sides: a side may be SEVERAL wire edges whose joints are
    // smooth (a band whose long rail is split by a T-junction). Pieces
    // run in wire order; single-edge sides have one piece. The legacy
    // arrays above always mirror piece 0 of each side.
    struct SidePiece {
        int edgeId = 0;
        Handle(Geom2d_Curve) pc;
        double f = 0.0, l = 0.0;
        bool rev = false;
        double len = 0.0;  // 3D curve length (chain parameterization)
    };
    std::array<std::vector<SidePiece>, 4> chain;
    bool chained() const {
        for (const auto& c : chain) {
            if (c.size() > 1) return true;
        }
        return false;
    }

    // Point along side i at t in [0,1], walking the wire direction.
    // Chained sides map t across their pieces by 3D length share.
    gp_Pnt2d side(int i, double t) const {
        if (collapsedLast && i == 3 && !poleCurve) {
            double t0 = rev[0] ? last[0] : first[0];
            return pc[0]->Value(t0);
        }
        if (chain[i].size() > 1) {
            double total = 0;
            for (const auto& pc2 : chain[i]) total += pc2.len;
            double want = t * total;
            for (const auto& pce : chain[i]) {
                if (want <= pce.len || &pce == &chain[i].back()) {
                    double lt = pce.len > 0 ? want / pce.len : 0.0;
                    lt = std::clamp(lt, 0.0, 1.0);
                    double tt = pce.rev ? 1.0 - lt : lt;
                    return pce.pc->Value(pce.f + tt * (pce.l - pce.f));
                }
                want -= pce.len;
            }
        }
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

// Build the patch from the face's outer wire: four sides chained
// head-to-tail with pcurves, interior probes inside the face. Tolerated
// wire noise: ONE degenerate edge (a bspline pole — becomes the
// collapsed side, its pcurve covering the UV gap) or, among five edges,
// ONE corner stub far shorter than the sides (skipped in the chain and
// stitched into the corner polygon at mesh time). `rotate` shifts which
// edge becomes side 0 — it picks the corner the grid anchors to and, on
// triangular patches, which corner the fan terminates in.
bool makeCoonsPatch(const TopoDS_Face& face, const Model& model,
                    CoonsPatch& patch, int rotate = 0,
                    const char** why = nullptr,
                    bool* reflexPlanar = nullptr) {
    auto reject = [&](const char* r) {
        if (why) *why = r;
        return false;
    };
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return reject("no outer wire");
    // A grid paves the whole outer boundary; a face with holes would get
    // its holes quadded over (and the hole edges never sampled).
    {
        int wires = 0;
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            ++wires;
        }
        if (wires != 1) return reject("face has holes");
    }
    struct WireEdge {
        TopoDS_Edge edge;
        bool degenerate;
        double len;
    };
    std::vector<WireEdge> all;
    for (BRepTools_WireExplorer wx(outer, face); wx.More(); wx.Next()) {
        if (all.size() >= 16) return reject("more than 16 edges");
        const TopoDS_Edge edge = wx.Current();
        double f, l;
        Handle(Geom2d_Curve) pcurve =
            BRep_Tool::CurveOnSurface(edge, face, f, l);
        if (pcurve.IsNull()) return reject("pcurve missing");
        const bool degen = BRep_Tool::Degenerated(edge);
        double len = 0.0;
        if (!degen) {
            BRepAdaptor_Curve c(edge);
            len = GCPnts_AbscissaPoint::Length(c);
        }
        all.push_back({edge, degen, len});
    }
    // The "gap": a degenerate pole edge, or a corner stub (five edges,
    // shortest under 2% of the perimeter).
    int gap = -1;
    for (size_t i = 0; i < all.size(); ++i) {
        if (all[i].degenerate) {
            if (gap >= 0) return reject("two pole edges");
            gap = int(i);
        }
    }
    if (gap < 0 && all.size() == 5) {
        double perim = 0.0;
        int shortest = 0;
        for (size_t i = 0; i < all.size(); ++i) {
            perim += all[i].len;
            if (all[i].len < all[shortest].len) shortest = int(i);
        }
        // No short stub: fall through to corner-angle chaining below.
        if (all[shortest].len < 0.02 * perim) gap = shortest;
    }
    const int nReal = int(all.size()) - (gap >= 0 ? 1 : 0);
    if (nReal < 3) return reject("under 3 real edges");

    // Order the real sides starting AFTER the gap, so the gap sits
    // between side (nReal-1)'s end and side 0's start. Plain patches
    // (no gap) honour the requested rotation instead.
    std::vector<int> order;
    const int start = gap >= 0 ? (gap + 1) % int(all.size()) : 0;
    for (int k = 0; k < int(all.size()); ++k) {
        int i = (start + k) % int(all.size());
        if (i != gap) order.push_back(i);
    }

    // More than four sides: group consecutive edges into FOUR sides at
    // the sharpest wire corners (a curved band whose rail is split by a
    // T-junction is still a four-sided patch). Turn angle at each joint
    // comes from the 3D end tangents; the gap (pole/stub) is always a
    // corner.
    // Tangents and SIGNED turns at wire joints. The sign comes from the
    // oriented surface normal: positive = convex (interior < 180 deg),
    // negative = reflex. A reflex corner breaks the whole four-sided
    // abstraction — transfinite interpolation over such a domain must
    // fold — so it rejects the patch instead of shipping folded cells.
    auto tangentAt = [&](int src, bool atEnd) -> gp_Vec {
        BRepAdaptor_Curve c(all[src].edge);
        const bool rev = all[src].edge.Orientation() == TopAbs_REVERSED;
        const double par = (atEnd != rev) ? c.LastParameter()
                                          : c.FirstParameter();
        gp_Pnt pp;
        gp_Vec d;
        c.D1(par, pp, d);
        if (rev) d.Reverse();
        return d;
    };
    BRepAdaptor_Surface signSurf(face);
    auto signedTurnAt = [&](size_t k, const std::vector<int>& ord) {
        int prev = ord[(k + ord.size() - 1) % ord.size()];
        gp_Vec a = tangentAt(prev, true);
        gp_Vec b = tangentAt(ord[k], false);
        if (a.Magnitude() < 1e-12 || b.Magnitude() < 1e-12) return 0.0;
        double f2, l2;
        Handle(Geom2d_Curve) pc =
            BRep_Tool::CurveOnSurface(all[ord[k]].edge, face, f2, l2);
        if (pc.IsNull()) return double(a.Angle(b));
        const bool rev = all[ord[k]].edge.Orientation() == TopAbs_REVERSED;
        gp_Pnt2d uv = pc->Value(rev ? l2 : f2);
        gp_Pnt sp;
        gp_Vec du, dv;
        signSurf.D1(uv.X(), uv.Y(), sp, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
        if (n.Magnitude() < 1e-12) return double(a.Angle(b));
        return std::atan2(a.Crossed(b).Dot(n) / n.Magnitude(), a.Dot(b));
    };

    std::vector<size_t> sideStart;  // indices into `order` that begin sides
    if (nReal > 4) {
        // Joint k sits BEFORE order[k] (between order[k-1] and order[k]).
        // With a gap (pole/stub) joint 0 is FORCED to be a corner; on a
        // plain closed wire it gets its real turn like every other joint,
        // or a smooth wire-start would steal a real corner's slot.
        std::vector<double> turn(order.size(), M_PI);
        std::vector<double> signedT(order.size(), M_PI);
        for (size_t k = gap >= 0 ? 1 : 0; k < order.size(); ++k) {
            signedT[k] = signedTurnAt(k, order);
            turn[k] = std::abs(signedT[k]);
        }
        std::vector<size_t> byTurn(order.size());
        std::iota(byTurn.begin(), byTurn.end(), 0);
        std::sort(byTurn.begin(), byTurn.end(),
                  [&](size_t x, size_t y) { return turn[x] > turn[y]; });
        // Need four clear corners; a fuzzy fourth means this isn't a
        // four-sided patch.
        if (turn[byTurn[3]] < 20.0 * M_PI / 180.0) return reject("no clear fourth corner");
        sideStart = {byTurn[0], byTurn[1], byTurn[2], byTurn[3]};
        std::sort(sideStart.begin(), sideStart.end());
        if (gap >= 0 && sideStart[0] != 0) return reject("gap not at a corner");
        // Reflex screening is DETECTION only, and PLANAR only: the
        // caller may prefer quad-fill for a flat chevron (transfinite
        // interpolation over a reflex domain must fold), but the patch
        // itself stays valid — in modes without a better planar mesher
        // the untangler and the fold overlay handle the outcome.
        if (reflexPlanar && signSurf.GetType() == GeomAbs_Plane) {
            for (size_t k = 0; k < order.size(); ++k) {
                if (gap >= 0 && k == 0) continue;  // forced gap corner
                const bool isCorner = std::find(sideStart.begin(),
                                                sideStart.end(),
                                                k) != sideStart.end();
                if (isCorner && signedT[k] < -45.0 * M_PI / 180.0) {
                    *reflexPlanar = true;
                }
                if (!isCorner && signedT[k] < -60.0 * M_PI / 180.0) {
                    *reflexPlanar = true;
                }
            }
        }
        // Rotate `order` so a corner is first, keeping the gap corner
        // first when there is one.
        if (gap < 0) {
            size_t shift =
                sideStart[size_t(rotate) % sideStart.size()] % order.size();
            if (shift) {
                std::rotate(order.begin(), order.begin() + shift,
                            order.end());
                for (size_t& v : sideStart) {
                    v = (v + order.size() - shift) % order.size();
                }
                std::sort(sideStart.begin(), sideStart.end());
            }
        }
    } else {
        if (gap < 0 && rotate > 0) {
            std::rotate(order.begin(),
                        order.begin() + (rotate % order.size()),
                        order.end());
        }
        for (size_t k = 0; k < std::min<size_t>(4, order.size()); ++k) {
            sideStart.push_back(k);
        }
        // Four plain sides: every joint is a corner, and a reflex one
        // (a dart-shaped face) folds the grid just like the chained
        // case. Detection only, planar only, same reasoning as above.
        if (reflexPlanar && nReal == 4 &&
            signSurf.GetType() == GeomAbs_Plane) {
            for (size_t k = gap >= 0 ? 1 : 0; k < order.size(); ++k) {
                if (signedTurnAt(k, order) < -45.0 * M_PI / 180.0) {
                    *reflexPlanar = true;
                }
            }
        }
    }

    auto pieceOf = [&](int src) {
        CoonsPatch::SidePiece pce;
        const TopoDS_Edge& edge = all[src].edge;
        pce.pc = BRep_Tool::CurveOnSurface(edge, face, pce.f, pce.l);
        pce.rev = edge.Orientation() == TopAbs_REVERSED;
        pce.edgeId = model.edges.FindIndex(edge);
        pce.len = all[src].len;
        return pce;
    };
    // Fill the four sides (legacy arrays mirror each side's first piece).
    const int nSides = nReal == 3 ? 3 : 4;
    for (int sIdx = 0; sIdx < nSides; ++sIdx) {
        size_t from = sideStart[sIdx];
        size_t to = sIdx + 1 < int(sideStart.size())
                        ? sideStart[sIdx + 1]
                        : order.size();
        for (size_t k = from; k < to; ++k) {
            patch.chain[sIdx].push_back(pieceOf(order[k]));
            if (patch.chain[sIdx].front().edgeId < 1) return reject("side has invalid edge");
        }
        const auto& p0 = patch.chain[sIdx].front();
        patch.pc[sIdx] = p0.pc;
        patch.first[sIdx] = p0.f;
        patch.last[sIdx] = p0.l;
        patch.rev[sIdx] = p0.rev;
        patch.edgeIds[sIdx] = p0.edgeId;
    }
    auto fill = [&](int slot, int src) {
        const TopoDS_Edge& edge = all[src].edge;
        double f, l;
        patch.pc[slot] = BRep_Tool::CurveOnSurface(edge, face, f, l);
        patch.first[slot] = f;
        patch.last[slot] = l;
        patch.rev[slot] = edge.Orientation() == TopAbs_REVERSED;
        patch.edgeIds[slot] = model.edges.FindIndex(edge);
    };
    if (nReal == 3) {
        patch.collapsedLast = true;
        if (gap >= 0) {
            // Pole from a degenerate edge: its pcurve spans the UV gap.
            fill(3, gap);
            patch.poleCurve = true;
            patch.edgeIds[3] = patch.edgeIds[1];  // density: v follows side 1
        } else {
            // Plain triangle: side 3 is the sides-0/2 corner point.
            patch.edgeIds[3] = patch.edgeIds[2];
            patch.pc[3] = patch.pc[2];
            patch.first[3] = patch.last[3] = 0.0;
            patch.rev[3] = false;
        }
    } else if (gap >= 0) {
        // Four real sides + corner stub: remember it for stitching.
        const TopoDS_Edge& stub = all[gap].edge;
        patch.stubEdgeId = model.edges.FindIndex(stub);
        patch.stubRev = stub.Orientation() == TopAbs_REVERSED;
        patch.stubPc = BRep_Tool::CurveOnSurface(stub, face,
                                                 patch.stubFirst,
                                                 patch.stubLast);
        if (patch.stubEdgeId < 1) return reject("stub edge unknown");
    }
    const int sides = patch.collapsedLast && !patch.poleCurve ? 3 : 4;
    for (int i = 0; i < sides; ++i) {
        if (patch.edgeIds[i] < 1) return reject("side edge unknown");
    }
    // Head-to-tail continuity in UV (a seam on a periodic surface breaks
    // the chain; such faces are not Coons candidates).
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double span = std::max(umax - umin, vmax - vmin);
    for (int i = 0; i < sides; ++i) {
        gp_Pnt2d a = patch.side(i, 1.0);
        gp_Pnt2d b = patch.side((i + 1) % 4, 0.0);
        if (patch.collapsedLast && !patch.poleCurve && i == 2) {
            b = patch.side(0, 0.0);
        }
        // Tolerance noise on exported pcurves reaches ~1e-3 of the span;
        // an actual seam jump is on the order of the span itself. The
        // corner carrying a stub legitimately jumps by the stub's length.
        double allow = 0.02 * span;
        if (patch.stubEdgeId > 0 && i == sides - 1 &&
            !patch.stubPc.IsNull()) {
            allow += patch.stubPc->Value(patch.stubFirst)
                         .Distance(patch.stubPc->Value(patch.stubLast));
        }
        if (a.Distance(b) > allow) return reject("sides not head-to-tail in UV");
    }
    // Interior probes must land inside the face.
    const double tol = BRep_Tool::Tolerance(face);
    for (int j = 1; j < 4; ++j) {
        for (int i = 1; i < 4; ++i) {
            gp_Pnt2d p = patch.uv(i / 4.0, j / 4.0);
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), p,
                                         tol);
            if (cls.State() == TopAbs_OUT) return reject("interior probe outside face");
        }
    }
    return true;
}

// Emit the Coons grid. Single-edge sides sample at uParams/vParams (the
// 0..1 splits, clustered for fillet strips); CHAINED sides sample each
// piece at its own solved count, so the border matches every neighbour
// vertex-for-vertex and the grid gains a column at each T-junction.
bool meshCoonsGrid(const TopoDS_Face& face, const Model& model, int faceId,
                   const std::vector<double>& uParams,
                   const std::vector<double>& vParams, int rotate,
                   const std::vector<int>& solvedEdge, MeshBuilder& out) {
    CoonsPatch patch;
    if (!makeCoonsPatch(face, model, patch, rotate)) return false;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    // Border vertices evaluate on the shared 3D edge curves, not through
    // this face's pcurve: both faces of an edge then produce bit-identical
    // points and the weld is exact.
    struct BPt {
        gp_Pnt p;
        gp_Pnt2d uv;
    };
    // One side sampled in wire direction. Single-piece sides honour the
    // given 0..1 splits; chains take each piece at its solved count.
    auto sampleSide = [&](int i,
                          const std::vector<double>& params)
        -> std::vector<BPt> {
        std::vector<BPt> row;
        const auto& ch = patch.chain[i];
        if (ch.size() <= 1) {
            BRepAdaptor_Curve c(TopoDS::Edge(model.edges(patch.edgeIds[i])));
            const double f3 = c.FirstParameter(), l3 = c.LastParameter();
            for (double t : params) {
                double tt = patch.rev[i] ? 1.0 - t : t;
                row.push_back(
                    {c.Value(f3 + tt * (l3 - f3)),
                     patch.pc[i]->Value(patch.first[i] +
                                        tt * (patch.last[i] -
                                              patch.first[i]))});
            }
            return row;
        }
        for (size_t k = 0; k < ch.size(); ++k) {
            const auto& pce = ch[k];
            int n = 1;
            if (pce.edgeId > 0 && pce.edgeId < int(solvedEdge.size()) &&
                solvedEdge[pce.edgeId] > 0) {
                n = solvedEdge[pce.edgeId];
            }
            BRepAdaptor_Curve c(TopoDS::Edge(model.edges(pce.edgeId)));
            const double f3 = c.FirstParameter(), l3 = c.LastParameter();
            const int last = k + 1 == ch.size() ? n : n - 1;
            for (int q = 0; q <= last; ++q) {
                double t = double(q) / n;
                double tt = pce.rev ? 1.0 - t : t;
                row.push_back(
                    {c.Value(f3 + tt * (l3 - f3)),
                     pce.pc->Value(pce.f + tt * (pce.l - pce.f))});
            }
        }
        return row;
    };
    auto uniformParams = [](int n) {
        std::vector<double> ps(n + 1);
        for (int i = 0; i <= n; ++i) ps[i] = double(i) / n;
        return ps;
    };

    // Bottom = side0 (wire dir), Top = side2 reversed, Right = side1,
    // Left = side3 reversed — so Bottom[i] pairs Top[i] and Left[j]
    // pairs Right[j], with (0,0) at side0's start.
    //
    // On a CHAINED patch every single side samples at its own SOLVED
    // count (its group never united with the opposite side); when the
    // chain pass couldn't reconcile opposite totals (shared rails,
    // cascades, user pins) the sizes differ and the face falls back
    // visibly instead of breaking the seam. Plain patches keep the
    // given (possibly clustered) splits.
    auto solvedCount = [&](int eid) {
        return eid > 0 && eid < int(solvedEdge.size()) && solvedEdge[eid] > 0
                   ? solvedEdge[eid]
                   : 1;
    };
    auto paramsFor = [&](int i,
                         const std::vector<double>& plain)
        -> std::vector<double> {
        if (patch.chain[i].size() > 1) return {};  // chain: per-piece
        if (patch.chained()) {
            return uniformParams(solvedCount(patch.edgeIds[i]));
        }
        // Plain patches: a UNIFORM request still samples at the edge's
        // OWN solved count — the border contract — with any rail
        // mismatch absorbed by the transition strips. Only deliberately
        // clustered splits (fillet holds) keep the given params; those
        // faces are exempt from the border check.
        bool uniform = true;
        for (size_t k = 0; k < plain.size() && uniform; ++k) {
            uniform = std::abs(plain[k] - double(k) /
                                              double(plain.size() - 1)) <
                      1e-9;
        }
        const int eid = patch.edgeIds[i];
        const int sc = eid > 0 && eid < int(solvedEdge.size())
                           ? solvedEdge[eid]
                           : 0;
        if (uniform && sc >= 1) return uniformParams(sc);
        return plain;
    };
    // (Tried and reverted: sampling a single side at the opposite
    // chain's arc fractions to kill rung skew — every single edge is
    // ALSO someone else's uniformly-sampled seam, and the sweep's opens
    // exploded 50x. Border positions are a shared contract; rung
    // alignment has to come from somewhere else.)
    std::vector<BPt> bottom = sampleSide(0, paramsFor(0, uParams));
    std::vector<BPt> top = sampleSide(2, paramsFor(2, uParams));
    std::reverse(top.begin(), top.end());

    // Decoupled rails: opposite totals may disagree now that the solver
    // no longer grows chains into equality. The deficit rail's NATURAL
    // points stay the emitted border — they are the neighbours' weld
    // contract and are never re-spaced (doctrine). The lattice itself
    // gets an arc-fraction resampling of that rail purely as blending
    // scaffold; at emission a transition strip of quads (plus a 5-gon
    // wherever a count is absorbed — the Plasticity pattern) stitches
    // the natural rail to the first interior grid line, and the
    // scaffold row is not emitted at all.
    // Scaffold rows resample the NATURAL rail by ARC LENGTH, not by
    // chain fraction: side(i,t) walks chains piece-by-piece, so a
    // T-junction with unequal piece densities shears every interior
    // column diagonally (the flaregun jacket rungs). Arc-uniform
    // scaffolds keep columns upright; UV interpolates within a natural
    // segment and re-evaluates on the surface.
    auto resample = [&](const std::vector<BPt>& nat, size_t n) {
        std::vector<double> arc(nat.size(), 0.0);
        for (size_t k = 1; k < nat.size(); ++k) {
            arc[k] = arc[k - 1] + nat[k].p.Distance(nat[k - 1].p);
        }
        const double total = arc.back() > 1e-12 ? arc.back() : 1.0;
        std::vector<BPt> row(n);
        size_t j = 0;
        for (size_t k = 0; k < n; ++k) {
            const double sTarget = total * double(k) / double(n - 1);
            while (j + 2 < nat.size() && arc[j + 1] < sTarget) ++j;
            const double seg = std::max(1e-12, arc[j + 1] - arc[j]);
            const double t = std::clamp((sTarget - arc[j]) / seg, 0.0, 1.0);
            gp_Pnt2d uv(
                nat[j].uv.X() + t * (nat[j + 1].uv.X() - nat[j].uv.X()),
                nat[j].uv.Y() + t * (nat[j + 1].uv.Y() - nat[j].uv.Y()));
            row[k] = {surface->Value(uv.X(), uv.Y()), uv};
        }
        return row;
    };
    std::vector<BPt> natBottom, natTop;  // natural deficit rails
    if (bottom.size() != top.size() && bottom.size() >= 2 &&
        top.size() >= 2) {
        if (bottom.size() < top.size()) {
            natBottom = bottom;
            bottom = resample(natBottom, top.size());
        } else {
            natTop = top;
            top = resample(natTop, bottom.size());
        }
    }
    if (bottom.size() != top.size() || bottom.size() < 2) return false;
    const int nu = int(bottom.size()) - 1;

    std::vector<BPt> right = sampleSide(1, paramsFor(1, vParams));
    std::vector<BPt> left;
    if (patch.collapsedLast) {
        // Pole: the whole left column is one point (fan rows).
        left.assign(right.size(), {bottom.front().p,
                                   patch.side(3, 0.5)});
        for (size_t j = 0; j < left.size(); ++j) {
            left[j].uv = patch.side(3, 1.0 - double(j) / (right.size() - 1));
        }
    } else {
        left = sampleSide(3, paramsFor(3, vParams));
        std::reverse(left.begin(), left.end());
    }
    std::vector<BPt> natLeft, natRight;  // natural deficit rails
    if (right.size() != left.size() && right.size() >= 2 &&
        left.size() >= 2 && !patch.collapsedLast) {
        if (right.size() < left.size()) {
            natRight = right;
            right = resample(natRight, left.size());
        } else {
            natLeft = left;
            left = resample(natLeft, right.size());
        }
    }
    if (patch.collapsedLast && left.size() != right.size()) {
        left.assign(right.size(), left.empty() ? BPt{bottom.front().p,
                                                     patch.side(3, 0.5)}
                                               : left.front());
        for (size_t j = 0; j < left.size(); ++j) {
            left[j].uv = patch.side(3, 1.0 - double(j) / (right.size() - 1));
        }
    }
    if (right.size() != left.size() || right.size() < 2) return false;
    const int nv = int(right.size()) - 1;

    // The (a,b) lattice follows the wire, whose UV handedness varies; the
    // Jacobian sign decides the polygon winding.
    gp_Pnt2d c0 = patch.uv(0.5, 0.5);
    gp_Pnt2d ca = patch.uv(0.55, 0.5);
    gp_Pnt2d cb = patch.uv(0.5, 0.55);
    const double jac = (ca.X() - c0.X()) * (cb.Y() - c0.Y()) -
                       (ca.Y() - c0.Y()) * (cb.X() - c0.X());
    const bool flip = (face.Orientation() == TopAbs_REVERSED) != (jac < 0);

    // Blend weights follow the borders' normalized arc positions so
    // clustered fillet rows stay clustered inside.
    auto arcWeights = [](const std::vector<BPt>& row) {
        std::vector<double> w(row.size(), 0.0);
        for (size_t i = 1; i < row.size(); ++i) {
            w[i] = w[i - 1] + row[i].p.Distance(row[i - 1].p);
        }
        double total = w.back() > 1e-12 ? w.back() : 1.0;
        for (double& x : w) x /= total;
        return w;
    };
    const std::vector<double> awB = arcWeights(bottom);
    const std::vector<double> awT = arcWeights(top);
    const std::vector<double> bwL = arcWeights(left);
    const std::vector<double> bwR = arcWeights(right);

    // Interior verts: discrete Coons blend of the border SAMPLES in 3D,
    // projected onto the surface. Blending in UV folds wherever a band's
    // pcurves bend tighter than the band is wide — EXCEPT on developable
    // charts (cylinders, cones), where the UV blend IS the ruling and
    // the 3D blend+projection is what wobbles (crumpled fillet bands).
    const GeomAbs_SurfaceType chartType =
        BRepAdaptor_Surface(face).GetType();
    const bool ruledChart = chartType == GeomAbs_Cylinder ||
                            chartType == GeomAbs_Cone;
    GeomAPI_ProjectPointOnSurf proj;
    proj.Init(gp_Pnt(0, 0, 0), surface);
    const gp_Pnt c00 = bottom.front().p, c10 = bottom.back().p;
    const gp_Pnt c11 = top.back().p, c01 = top.front().p;
    std::vector<BPt> gpts((nu + 1) * (nv + 1));
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            BPt bp;
            if (j == 0) bp = bottom[i];
            else if (j == nv) bp = top[i];
            else if (i == 0) bp = left[j];
            else if (i == nu) bp = right[j];
            else {
                // Bilinearly blended weights: rows near the top follow the
                // TOP border's spacing, not the bottom's. With a chained
                // side whose spacing drifts a step against the opposite
                // rail, single-border weights skew every interior rung the
                // same way until the last row folds over (bowtie cells).
                const double b0 = 0.5 * (bwL[j] + bwR[j]);
                const double a = (1.0 - b0) * awB[i] + b0 * awT[i];
                const double b = (1.0 - a) * bwL[j] + a * bwR[j];
                gp_XYZ blend =
                    bottom[i].p.XYZ() * (1 - b) + top[i].p.XYZ() * b +
                    left[j].p.XYZ() * (1 - a) + right[j].p.XYZ() * a -
                    (c00.XYZ() * ((1 - a) * (1 - b)) +
                     c10.XYZ() * (a * (1 - b)) + c11.XYZ() * (a * b) +
                     c01.XYZ() * ((1 - a) * b));
                gp_Pnt2d seed = patch.uv(a, b);
                bp.p = surface->Value(seed.X(), seed.Y());
                bp.uv = seed;
                if (!ruledChart) {
                    proj.Perform(gp_Pnt(blend));
                    if (proj.IsDone() && proj.NbPoints() > 0) {
                        bp.p = proj.NearestPoint();
                        double pu, pv;
                        proj.LowerDistanceParameters(pu, pv);
                        bp.uv.SetX(pu);
                        bp.uv.SetY(pv);
                    }
                }
            }
            gpts[j * (nu + 1) + i] = bp;
        }
    }

    // Untangle folded interiors: transfinite blending of a strongly
    // non-convex outline (a chevron plane, a chained strip drifting
    // against its rail) can cross its own rungs. Pinned-border Laplace
    // passes in UV pull interior points back inside the domain; a pass
    // is kept only when it strictly reduces the number of inverted
    // cells, so a wrapped periodic chart can never make things worse.
    if (nu > 1 && nv > 1) {
        auto countFlips = [&](const std::vector<BPt>& g) {
            double total = 0, meanAbs = 0;
            std::vector<double> areas;
            areas.reserve(size_t(nu) * nv);
            for (int j = 0; j < nv; ++j) {
                for (int i = 0; i < nu; ++i) {
                    const gp_Pnt2d& q00 = g[j * (nu + 1) + i].uv;
                    const gp_Pnt2d& q10 = g[j * (nu + 1) + i + 1].uv;
                    const gp_Pnt2d& q11 = g[(j + 1) * (nu + 1) + i + 1].uv;
                    const gp_Pnt2d& q01 = g[(j + 1) * (nu + 1) + i].uv;
                    double a2 =
                        (q10.X() - q00.X()) * (q11.Y() - q00.Y()) -
                        (q11.X() - q00.X()) * (q10.Y() - q00.Y()) +
                        (q11.X() - q00.X()) * (q01.Y() - q00.Y()) -
                        (q01.X() - q00.X()) * (q11.Y() - q00.Y());
                    areas.push_back(a2);
                    total += a2;
                    meanAbs += std::abs(a2);
                }
            }
            meanAbs /= double(std::max<size_t>(1, areas.size()));
            int flips = 0;
            for (double a2 : areas) {
                if (a2 * total < 0 && std::abs(a2) > 1e-3 * meanAbs) {
                    ++flips;
                }
            }
            return flips;
        };
        int bestFlips = countFlips(gpts);
        if (bestFlips > 0) {
            // In-place Gauss-Seidel, alternating sweep direction: roughly
            // twice Jacobi's convergence per pass, no directional bias.
            // Two schemes, each accepted pass-by-pass only on improvement:
            // plain Laplace handles extremely anisotropic charts (thin
            // strips), the Winslow stencil handles non-convex outlines
            // where a harmonic map itself must fold. Whatever survives is
            // the best grid either scheme reached.
            auto sweep = [&](std::vector<BPt>& sm, bool winslow, bool fwd) {
                for (int jj = 1; jj < nv; ++jj) {
                    const int j = fwd ? jj : nv - jj;
                    for (int ii = 1; ii < nu; ++ii) {
                        const int i = fwd ? ii : nu - ii;
                        const gp_Pnt2d& le = sm[j * (nu + 1) + i - 1].uv;
                        const gp_Pnt2d& ri = sm[j * (nu + 1) + i + 1].uv;
                        const gp_Pnt2d& dn = sm[(j - 1) * (nu + 1) + i].uv;
                        const gp_Pnt2d& up = sm[(j + 1) * (nu + 1) + i].uv;
                        BPt& b = sm[j * (nu + 1) + i];
                        if (!winslow) {
                            b.uv = gp_Pnt2d(0.25 * (le.X() + ri.X() +
                                                    dn.X() + up.X()),
                                            0.25 * (le.Y() + ri.Y() +
                                                    dn.Y() + up.Y()));
                            b.p = surface->Value(b.uv.X(), b.uv.Y());
                            continue;
                        }
                        const gp_Pnt2d& pp =
                            sm[(j + 1) * (nu + 1) + i + 1].uv;
                        const gp_Pnt2d& pm =
                            sm[(j - 1) * (nu + 1) + i + 1].uv;
                        const gp_Pnt2d& mp =
                            sm[(j + 1) * (nu + 1) + i - 1].uv;
                        const gp_Pnt2d& mm =
                            sm[(j - 1) * (nu + 1) + i - 1].uv;
                        const double xu = 0.5 * (ri.X() - le.X());
                        const double yu = 0.5 * (ri.Y() - le.Y());
                        const double xv = 0.5 * (up.X() - dn.X());
                        const double yv = 0.5 * (up.Y() - dn.Y());
                        const double al = xv * xv + yv * yv;
                        const double be = xu * xv + yu * yv;
                        const double ga = xu * xu + yu * yu;
                        const double den = 2.0 * (al + ga);
                        if (den < 1e-30) continue;
                        b.uv = gp_Pnt2d(
                            (al * (ri.X() + le.X()) +
                             ga * (up.X() + dn.X()) -
                             0.5 * be *
                                 (pp.X() - mp.X() - pm.X() + mm.X())) /
                                den,
                            (al * (ri.Y() + le.Y()) +
                             ga * (up.Y() + dn.Y()) -
                             0.5 * be *
                                 (pp.Y() - mp.Y() - pm.Y() + mm.Y())) /
                                den);
                        b.p = surface->Value(b.uv.X(), b.uv.Y());
                    }
                }
            };
            for (int scheme = 0; scheme < 2 && bestFlips > 0; ++scheme) {
                std::vector<BPt> sm = gpts;  // start from the best so far
                for (int pass = 0; pass < 40 && bestFlips > 0; ++pass) {
                    sweep(sm, scheme == 1, (pass & 1) == 0);
                    int flips = countFlips(sm);
                    if (flips < bestFlips) {
                        gpts = sm;
                        bestFlips = flips;
                    }
                }
            }
            if (bestFlips > 0) {
                dbg("coons: face %d still has %d inverted cell(s) after "
                    "untangling",
                    faceId, bestFlips);
            }
        }
    }

    // A deficit rail's scaffold row is NOT emitted — its natural points
    // are the border, bridged to the first interior line below. The
    // emitted grid shrinks by one row/column on each such side.
    const int j0 = natBottom.empty() ? 0 : 1;
    const int j1 = natTop.empty() ? nv : nv - 1;
    const int i0 = natLeft.empty() ? 0 : 1;
    const int i1 = natRight.empty() ? nu : nu - 1;

    std::vector<uint32_t> grid((nu + 1) * (nv + 1), 0);
    for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
            const BPt& bp = gpts[j * (nu + 1) + i];
            grid[j * (nu + 1) + i] =
                out.addVertex(bp.p, {faceId, bp.uv.X(), bp.uv.Y()});
        }
    }

    // Corner stub: sample its 3D curve at the solved count and stitch the
    // samples into the (0,0) corner polygon — the chain put the stub
    // between side 3's end and side 0's start, i.e. right there.
    std::vector<uint32_t> stubVerts;  // near end + interiors, wire order
    if (patch.stubEdgeId > 0) {
        int segs = patch.stubEdgeId < int(solvedEdge.size())
                       ? std::max(1, solvedEdge[patch.stubEdgeId])
                       : 1;
        BRepAdaptor_Curve sc(TopoDS::Edge(model.edges(patch.stubEdgeId)));
        const double f = sc.FirstParameter(), l = sc.LastParameter();
        for (int k = 0; k < segs; ++k) {
            double tt = patch.stubRev ? 1.0 - double(k) / segs
                                      : double(k) / segs;
            gp_Pnt pos = sc.Value(f + (l - f) * tt);
            gp_Pnt2d p2 = patch.stubPc->Value(patch.stubFirst +
                                              tt * (patch.stubLast -
                                                    patch.stubFirst));
            stubVerts.push_back(
                out.addVertex(pos, {faceId, p2.X(), p2.Y()}));
        }
    }

    for (int j = j0; j < j1; ++j) {
        for (int i = i0; i < i1; ++i) {
            std::vector<uint32_t> ring = {grid[j * (nu + 1) + i],
                                          grid[j * (nu + 1) + i + 1],
                                          grid[(j + 1) * (nu + 1) + i + 1],
                                          grid[(j + 1) * (nu + 1) + i]};
            if (i == 0 && j == 0 && !stubVerts.empty()) {
                ring.insert(ring.end(), stubVerts.begin(), stubVerts.end());
            }
            out.addPolygon(std::move(ring), faceId, flip);
        }
    }

    // Transition strips for the deficit rails: natural border points
    // bridge to the first interior grid line with a monotone index map —
    // quads where the counts advance together, a 5-gon wherever the
    // dense line contributes an extra point. Lattice-CCW is low line
    // forward then high line backward, matching the grid cells' winding.
    auto railIds = [&](const std::vector<BPt>& row) {
        std::vector<uint32_t> ids(row.size());
        for (size_t k = 0; k < row.size(); ++k) {
            ids[k] = out.addVertex(row[k].p, {faceId, row[k].uv.X(),
                                              row[k].uv.Y()});
        }
        return ids;
    };
    auto arcFractions = [&](const std::vector<uint32_t>& ids) {
        // Normalized cumulative arc of an emitted vertex sequence.
        std::vector<double> arc(ids.size(), 0.0);
        for (size_t k = 1; k < ids.size(); ++k) {
            const auto& A = out.mesh().vertices[ids[k - 1]];
            const auto& B = out.mesh().vertices[ids[k]];
            arc[k] = arc[k - 1] + std::sqrt((B[0] - A[0]) * (B[0] - A[0]) +
                                            (B[1] - A[1]) * (B[1] - A[1]) +
                                            (B[2] - A[2]) * (B[2] - A[2]));
        }
        const double total = arc.back() > 1e-12 ? arc.back() : 1.0;
        for (double& x : arc) x /= total;
        return arc;
    };
    // stubTail (with skipFirst / cornerAfter) reattaches the corner stub
    // when the cell that used to carry it was replaced by a strip.
    auto emitStrip = [&](const std::vector<uint32_t>& low,
                         const std::vector<uint32_t>& high,
                         const std::vector<uint32_t>* stubTail,
                         bool skipFirstStub, uint32_t cornerAfter) {
        const int nLow = int(low.size()) - 1;
        const int nHigh = int(high.size()) - 1;
        if (nLow < 1 || nHigh < 1) return;
        const bool lowSparse = nLow <= nHigh;
        const std::vector<uint32_t>& S = lowSparse ? low : high;
        const std::vector<uint32_t>& D = lowSparse ? high : low;
        // Chained rails are piecewise-nonuniform: map by ARC fraction,
        // not index, or the bridge crosses arc positions into long
        // diagonal slivers (the zig-zag band).
        const std::vector<double> sArc = arcFractions(S);
        const std::vector<double> dArc = arcFractions(D);
        const int m = int(S.size()) - 1;
        const int n = int(D.size()) - 1;
        std::vector<int> mp(m + 1);
        mp[0] = 0;
        mp[m] = n;
        for (int k = 1; k < m; ++k) {
            int j = mp[k - 1];
            while (j + 1 < n && std::abs(dArc[j + 1] - sArc[k]) <=
                                    std::abs(dArc[j] - sArc[k])) {
                ++j;
            }
            mp[k] = j;
        }
        for (int k = 0; k < m; ++k) {
            const int a = mp[k];
            const int b = mp[k + 1];
            std::vector<uint32_t> ring;
            if (lowSparse) {
                ring = {S[k], S[k + 1]};
                for (int t = b; t >= a; --t) ring.push_back(D[t]);
            } else {
                for (int t = a; t <= b; ++t) ring.push_back(D[t]);
                ring.push_back(S[k + 1]);
                ring.push_back(S[k]);
            }
            ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
            if (ring.size() > 1 && ring.front() == ring.back()) {
                ring.pop_back();
            }
            if (k == 0 && stubTail && !stubTail->empty()) {
                ring.insert(ring.end(),
                            stubTail->begin() + (skipFirstStub ? 1 : 0),
                            stubTail->end());
                if (cornerAfter != UINT32_MAX) ring.push_back(cornerAfter);
            }
            if (ring.size() < 3) continue;
            out.addPolygon(std::move(ring), faceId, flip);
        }
    };
    if (!natBottom.empty()) {
        std::vector<uint32_t> high;
        for (int i = i0; i <= i1; ++i) high.push_back(grid[j0 * (nu + 1) + i]);
        emitStrip(railIds(natBottom), high,
                  stubVerts.empty() ? nullptr : &stubVerts, false,
                  UINT32_MAX);
    }
    if (!natTop.empty()) {
        std::vector<uint32_t> low;
        for (int i = i0; i <= i1; ++i) low.push_back(grid[j1 * (nu + 1) + i]);
        emitStrip(low, railIds(natTop), nullptr, false, UINT32_MAX);
    }
    if (!natLeft.empty()) {
        std::vector<uint32_t> low;
        for (int j = j0; j <= j1; ++j) low.push_back(grid[j * (nu + 1) + i0]);
        // The stub's far end is side0's start — a contract point that
        // lives at lattice (0,0), outside the emitted grid here. Close
        // the strip through it. (stubVerts[0] coincides with the natural
        // left rail's first point, so it is skipped.)
        const bool stubHere = !stubVerts.empty() && natBottom.empty();
        uint32_t corner00 = UINT32_MAX;
        if (stubHere) {
            const BPt& c = gpts[0];
            corner00 = out.addVertex(c.p, {faceId, c.uv.X(), c.uv.Y()});
        }
        emitStrip(low, railIds(natLeft),
                  stubHere ? &stubVerts : nullptr, true, corner00);
    }
    if (!natRight.empty()) {
        std::vector<uint32_t> high;
        for (int j = j0; j <= j1; ++j) high.push_back(grid[j * (nu + 1) + i1]);
        emitStrip(railIds(natRight), high, nullptr, false, UINT32_MAX);
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

bool isGeometricallyFlat(const TopoDS_Face& face,
                         const BRepAdaptor_Surface& surf);  // defined below

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
                for (int i = 0; i < n; ++i) {  // endpoint owned by next edge
                    double t = rev ? 1.0 - double(i) / n : double(i) / n;
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
                         const BRepAdaptor_Surface& surf) {
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
    double lo = 1e300, hi = -1e300, diag = 0;
    for (const gp_Pnt& p : pts) {
        double d = (p.XYZ() - c).Dot(n);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
        diag = std::max(diag, p.XYZ().Modulus());
    }
    for (const gp_Pnt& p : pts) {
        for (const gp_Pnt& q : pts) {
            diag = std::max(diag, p.Distance(q));
        }
    }
    return hi - lo < std::max(1e-6, 1e-3 * diag);
}

bool collectPlanarLoops(const TopoDS_Face& face,
                        const BRepAdaptor_Surface& surf, const Model& model,
                        FacePlan& plan, bool requirePlane = true) {
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
                if (c3.IsNull() || c2.IsNull()) return false;
                const bool rev = edge.Orientation() == TopAbs_REVERSED;
                Piece pc;
                for (int i = 0; i <= n; ++i) {
                    double t = rev ? 1.0 - double(i) / n : double(i) / n;
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
            if (ring.uv.size() < 3) return false;
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

// The demotion floor for ANY face with pcurves: every wire sampled at
// the solved counts (the border contract, same formula every mesher
// uses), holes bridged in UV, the region web-triangulated. Interior
// quality is modest, but the borders are exact by construction — a
// face that lands here cannot leak. The OCCT triangulation fallback
// remains only for faces this cannot express (null curves, degenerate
// UV rings).
bool meshContractFallback(const TopoDS_Face& face, const Model& model,
                          int faceId, const std::vector<int>& solvedEdge,
                          int radialDefault, MeshBuilder& out) {
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
        dbg("contract floor %d: ring sampling failed", faceId);
        return false;
    }
    if (rings.empty()) return false;
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    // Curved UV charts are anisotropic (u in radians, v in model units):
    // scale u by the local relative stretch so the triangulator sees
    // true shapes. Positive scale keeps the normalized windings.
    double uScale = 1.0;
    try {
        BRepAdaptor_Surface surf(face);
        const double um =
            (surf.FirstUParameter() + surf.LastUParameter()) / 2;
        const double vm =
            (surf.FirstVParameter() + surf.LastVParameter()) / 2;
        const double su = std::max(
            1e-9,
            surf.Value(um, vm).Distance(surf.Value(um + 1e-3, vm)) / 1e-3);
        const double sv = std::max(
            1e-9,
            surf.Value(um, vm).Distance(surf.Value(um, vm + 1e-3)) / 1e-3);
        uScale = su / sv;
    } catch (const Standard_Failure&) {
    }
    std::vector<WebPoint> outer;
    std::vector<std::vector<WebPoint>> holes;
    for (PlanarRing& r : rings) {
        std::vector<WebPoint> ring;
        for (size_t i = 0; i < r.uv.size(); ++i) {
            ring.push_back(
                {gp_Pnt2d(r.uv[i].X() * uScale, r.uv[i].Y()),
                 out.addVertex(r.p[i], {})});
        }
        if (r.isOuter) outer = std::move(ring);
        else holes.push_back(std::move(ring));
    }
    if (outer.size() < 3) return false;
    if (!triangulateWeb(std::move(outer), std::move(holes), faceId, flip,
                        out)) {
        dbg("contract floor %d: web triangulation failed", faceId);
        return false;
    }
    return true;
}

bool planQuadFill(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, FacePlan& plan) {
    // Any trimmed surface patch works — the grid lives in UV and maps
    // through the surface. Only faces that wrap a FULL period need the
    // seam-aware revolution grids; a small patch trimmed from a closed
    // surface (fillet corners, wedges on cylinders) is a plain chart.
    {
        Handle(Geom_Surface) S = BRep_Tool::Surface(face);
        if (S.IsNull()) return false;
        double umin, umax, vmin, vmax;
        BRepTools::UVBounds(face, umin, umax, vmin, vmax);
        if (S->IsUPeriodic() && umax - umin >= 0.999 * S->UPeriod()) {
            return false;
        }
        if (S->IsVPeriodic() && vmax - vmin >= 0.999 * S->VPeriod()) {
            return false;
        }
    }
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
// Zipper a simple band between an outer ring (CCW in UV) and a hole ring
// (CW): rotational alignment by total rail length, then a greedy walk that
// advances whichever side makes the shorter diagonal. Equal counts give a
// pure quad ring; the rim of a quad-fill face reads as flow, not ear soup.
// The zip is validated before anything is emitted: when the two rings are
// not actually a band (a long thin outline against a small localized
// frontier loop), the greedy walk fans across the void and folds — every
// candidate cell's UV winding must agree, or the caller ear-clips instead.
bool zipperRings(const std::vector<WebPoint>& outer,
                 const std::vector<WebPoint>& hole, int faceId, bool flip,
                 MeshBuilder& out) {
    const int n = int(outer.size());
    const int m = int(hole.size());
    // The hole winds opposite to the outer; reverse it so both progress
    // the same way around the band.
    std::vector<WebPoint> ring(hole.rbegin(), hole.rend());
    auto d2 = [](const WebPoint& a, const WebPoint& b) {
        return a.uv.SquareDistance(b.uv);
    };
    int bestOff = 0;
    double bestSum = 1e300;
    for (int off = 0; off < m; ++off) {
        double sum = 0;
        for (int i = 0; i < n; i += std::max(1, n / 64)) {
            sum += d2(outer[i], ring[(off + i * m / n) % m]);
        }
        if (sum < bestSum) {
            bestSum = sum;
            bestOff = off;
        }
    }
    std::vector<std::array<const WebPoint*, 4>> cells;  // [3] null = tri
    if (n == m) {  // pure quad ring
        for (int i = 0; i < n; ++i) {
            int j = (bestOff + i) % m;
            cells.push_back({&outer[i], &outer[(i + 1) % n],
                             &ring[(j + 1) % m], &ring[j]});
        }
    } else {
        int ia = 0, ib = 0;
        while (ia < n || ib < m) {
            const WebPoint& a = outer[ia % n];
            const WebPoint& a1 = outer[(ia + 1) % n];
            const WebPoint& b = ring[(bestOff + ib) % m];
            const WebPoint& b1 = ring[(bestOff + ib + 1) % m];
            bool stepA;
            if (ia >= n) stepA = false;
            else if (ib >= m) stepA = true;
            else stepA = d2(a1, b) <= d2(a, b1);
            if (stepA) {
                cells.push_back({&a, &a1, &b, nullptr});
                ++ia;
            } else {
                cells.push_back({&a, &b1, &b, nullptr});
                ++ib;
            }
        }
    }
    double total = 0;
    std::vector<double> areas;
    areas.reserve(cells.size());
    double meanAbs = 0;
    for (const auto& c : cells) {
        const int k = c[3] ? 4 : 3;
        double a = 0;
        for (int i = 0; i < k; ++i) {
            const gp_Pnt2d& p = c[i]->uv;
            const gp_Pnt2d& q = c[(i + 1) % k]->uv;
            a += p.X() * q.Y() - q.X() * p.Y();
        }
        areas.push_back(a);
        total += a;
        meanAbs += std::abs(a);
    }
    meanAbs /= double(std::max<size_t>(1, areas.size()));
    for (double a : areas) {
        if (a * total < 0 && std::abs(a) > 1e-3 * meanAbs) return false;
    }
    for (const auto& c : cells) {
        if (c[3]) {
            out.addPolygon({c[0]->vert, c[1]->vert, c[2]->vert, c[3]->vert},
                           faceId, flip);
        } else {
            out.addPolygon({c[0]->vert, c[1]->vert, c[2]->vert}, faceId,
                           flip);
        }
    }
    return true;
}

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
        return triangulateWeb(std::move(faceOuter), std::move(faceHoles),
                              faceId, flip, out);
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
        // A simple band (one boundary ring around one frontier ring, or a
        // frontier pocket around one hole ring) zippers into flowing
        // quads/tris; anything more complex keeps the ear-clipped web.
        bool zipped = false;
        if (r.holes.size() == 1 && r.outer.size() >= 3 &&
            r.holes[0].size() >= 3) {
            zipped = zipperRings(r.outer, r.holes[0], faceId, flip, out);
        }
        if (!zipped &&
            !triangulateWeb(std::move(r.outer), std::move(r.holes), faceId,
                            flip, out)) {
            return false;
        }
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
    bool coonsReflex = false;  // flat outline with a strong reflex bend
    auto coonsOk = [&](CoonsPatch& patch) {
        // Patch construction is cheap; a memoized NEGATIVE skips it (and
        // the probes); a positive still rebuilds the (cheap) patch data.
        // Validity is rotation-independent, so the memo stays keyed by
        // face alone.
        if (cache) {
            auto it = cache->coonsValid.find(fid);
            if (it != cache->coonsValid.end() && !it->second) return false;
        }
        const char* why = nullptr;
        bool reflex = false;
        bool v = makeCoonsPatch(face, model, patch, s.coonsRotate, &why,
                                &reflex);
        if (!v && why) dbg("coons: face %d rejected: %s", fid, why);
        if (v && reflex) dbg("coons: face %d has a reflex flat outline", fid);
        coonsReflex = v && reflex;
        if (cache) {
            cache->coonsValid[fid] = v;
            cache->coonsReflex[fid] = coonsReflex;
        }
        return v;
    };

    auto finishRevolution = [&]() {
        plan.kind = MesherKind::RevolutionGrid;
        // Interior insert wires must never contribute rim candidates —
        // a slot border classified as a rim contaminates the emitted
        // rim row with interior points.
        std::vector<int> rimCandidates;
        {
            std::set<int> insertIds;
            for (const auto& w : plan.insertWires) {
                insertIds.insert(w.begin(), w.end());
            }
            for (int eid : info.edgeIds) {
                if (!insertIds.count(eid)) rimCandidates.push_back(eid);
            }
        }
        collectIsoEdges(face, model, rimCandidates, plan,
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
                    plan.constrains = true;
                    if (patch.chained()) {
                        for (int i = 0; i < 4; ++i) {
                            for (const auto& pce : patch.chain[i]) {
                                plan.coonsSides[i].push_back(pce.edgeId);
                            }
                        }
                    } else {
                        plan.uEdges = {patch.edgeIds[0], patch.edgeIds[2]};
                        plan.vEdges = {patch.edgeIds[1], patch.edgeIds[3]};
                    }
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
        std::vector<std::vector<int>> inserts;
        if (edgesHugRimsOrInserts(face, surf, model, inserts)) {
            plan.insertWires = std::move(inserts);
            finishRevolution();
            return plan;
        }
    }

    int capEdgeId = 0;
    if (surf.GetType() == GeomAbs_Plane &&
        boundingCircle(face, plan.circ, capEdgeId, model)) {
        plan.kind = MesherKind::DiskCap;
        plan.uEdges.push_back(capEdgeId);  // ring count = edge subdivisions
        plan.constrains = true;
        return plan;
    }

    // Game-topology minimal, explicit per-face override: the user asked
    // for THIS face's boundary shape, so it wins even over the junction
    // patterns below.
    if (s.minimal && settings.perFace.count(fid) &&
        planMinimalPlanar(face, surf, model, plan)) {
        return plan;
    }

    // Minimal n-gon owns EVERY flat face it can express when the mode is
    // on (the topology policy: big flats are n-gons, quads go to curves;
    // triangulation is an export option). The junction patterns below
    // only see flat faces when minimal is off or can't build the face.
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
        // Only a plain 2u+2v rectangle ties its grid to its edges. On auto
        // anything else must NOT grid: an unconstrained grid's coarse
        // border (two verts a side at 1x1) can never zip against a denser
        // neighbour — conform merges verts but cannot split edges. A
        // rectangle whose side is split into collinear edges (T-junction)
        // falls through to the fallback, whose shared-edge nodes come from
        // the model-wide triangulation and match the neighbour exactly.
        collectIsoEdges(face, model, info.edgeIds, plan);
        if (plan.uEdges.size() == 2 && plan.vEdges.size() == 2) {
            plan.kind = MesherKind::PlanarGrid;
            if (info.isFillet) {
                plan.isFillet = true;
                // The blend arc runs along u for a cylinder strip and along
                // the minor circle (v) for a toroidal corner patch.
                plan.acrossIsU = surf.GetType() == GeomAbs_Cylinder;
            }
            return plan;
        }
        plan.uEdges.clear();
        plan.vEdges.clear();
    }

    // Four-sided freeform/trimmed faces get a structured Coons grid; the
    // across-the-blend direction of a fillet strip is whichever side pair
    // is shorter in 3D.
    {
        CoonsPatch patch;
        if (coonsOk(patch)) {
            // A flat chevron (reflex outline) folds under any transfinite
            // grid. In quad-dominant mode quad-fill's grid + CDT rim is
            // strictly better and samples the same solved counts. In
            // defaults there is no denser-safe replacement — minimal
            // n-gons starve shared rails against unconstrained fallback
            // neighbours (measured: dup flaps on sliver strips) — so the
            // patch proceeds and the untangler + fold overlay take over.
            if (coonsReflex && s.quadDominant &&
                planQuadFill(face, surf, model, plan)) {
                return plan;
            }
            plan.kind = MesherKind::CoonsGrid;
            plan.constrains = true;
            if (patch.chained()) {
                for (int i = 0; i < 4; ++i) {
                    for (const auto& pce : patch.chain[i]) {
                        plan.coonsSides[i].push_back(pce.edgeId);
                    }
                }
                dbg("coons: face %d chained sides "
                    "[%zu:%d..][%zu:%d..][%zu:%d..][%zu:%d..]",
                    fid, patch.chain[0].size(),
                    patch.chain[0].empty() ? 0 : patch.chain[0][0].edgeId,
                    patch.chain[1].size(),
                    patch.chain[1].empty() ? 0 : patch.chain[1][0].edgeId,
                    patch.chain[2].size(),
                    patch.chain[2].empty() ? 0 : patch.chain[2][0].edgeId,
                    patch.chain[3].size(),
                    patch.chain[3].empty() ? 0 : patch.chain[3][0].edgeId);
            } else {
                plan.uEdges = {patch.edgeIds[0], patch.edgeIds[2]};
                plan.vEdges = {patch.edgeIds[1], patch.edgeIds[3]};
            }
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
    // Quad-dominant decimation moves border verts by up to the chord
    // tolerance; on a face with features SMALLER than that it wraps flaps
    // over the neighbours (folds the weld then has to amputate). Such
    // faces triangulate plainly instead.
    if (s.quadDominant) {
        for (int eid : info.edgeIds) {
            const double len = analysis.edges[eid - 1].length;
            if (len > 1e-12 && len < 3.0 * s.chordTolerance) {
                plan.forceFallbackQuads = 0;
                break;
            }
        }
    }
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
        // Hard sanity ceiling. Chained-rail/ring derivations can cascade
        // counts across a large assembly (observed: a solved count of
        // ~983k on an 8k-face model, which took the Coons mesher down
        // with it). Clamping at the single read point every consumer
        // shares keeps borders consistent on both sides of an edge.
        // The real cure is count decoupling (handoff step 1).
        int n = it == groupCount.end() ? fallback : it->second;
        return std::min(n, 256);
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
    // Global budget knob: counts scale before the group solve (floors
    // reapply after), so one slider re-budgets the whole model.
    const double dScale = std::clamp(settings.densityScale, 0.05, 20.0);
    auto propose = [&](const std::vector<int>& edges, int count,
                       bool overridden) {
        if (edges.empty()) return;
        count = std::max(1, int(std::lround(count * dScale)));
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
            // Floor applies AFTER scaling (propose scales): a ring floor
            // of 6 must survive a 0.5x budget.
            propose({eid}, std::max(int(std::lround(floorA / dScale)),
                                    adaptiveCount(eid, s)),
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
            // Chained Coons sides: every piece proposes on its own; the
            // chain pass below reconciles opposite sides by sum.
            for (int sd = 0; sd < 4; ++sd) {
                for (int e : plan.coonsSides[sd]) {
                    proposeSet({e}, sd % 2 == 0 ? nu : nv, 1,
                               sd % 2 == 0 ? adU : adV, s, overridden);
                }
            }
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
    for (const auto& [root, count] : pinned) {
        sol.groupCount[root] = std::max(1, count);  // a pin of 0 is a leak
    }

    // Chained Coons: opposite sides must sample equal TOTALS. Chains
    // share rails with other chains, so one-shot bumps go stale — grow
    // the SMALLER side's last unpinned edge instead and iterate to a
    // fixpoint (growth is monotone, so it terminates). Anything still
    // unequal falls back at mesh time without breaking seams.
    {
        // Never grow edges that other PATTERN meshers depend on: a
        // revolution band's radial count must stay the sum of its rim
        // arcs, and ring junctions derive their circle from the plate.
        std::set<int> protectedRoots;
        for (const auto& [fid2, plan2] : plans) {
            if (plan2.kind != MesherKind::RevolutionGrid &&
                plan2.kind != MesherKind::DiskCap &&
                plan2.kind != MesherKind::RingJunction) {
                continue;
            }
            for (int e : plan2.uEdges) {
                protectedRoots.insert(sol.groups.find(e));
            }
            if (plan2.circleEdgeId > 0) {
                protectedRoots.insert(sol.groups.find(plan2.circleEdgeId));
            }
        }
        auto sideSum = [&](const std::vector<int>& sd) {
            int t = 0;
            for (int e : sd) t += std::max(1, sol.countFor(e, 1));
            return t;
        };
        auto grow = [&](const std::vector<int>& sd, int by) {
            for (auto it = sd.rbegin(); it != sd.rend(); ++it) {
                int root = sol.groups.find(*it);
                if (pinned.count(root) || protectedRoots.count(root)) {
                    continue;
                }
                sol.groupCount[root] =
                    std::max(1, sol.countFor(*it, 1)) + by;
                return true;
            }
            return false;
        };
        // COUNT DECOUPLING (handoff step 1): the old fixpoint grew the
        // lighter side of every chained coons patch until opposite totals
        // matched, which cascaded counts across shared rails (measured:
        // 37,808-poly defaults on one model, and a ~983k solved count on
        // a dirty assembly). Chained sides now keep their natural counts;
        // meshCoonsGrid arc-length-resamples the deficit rail for its
        // lattice and the post-weld seam absorber splices the neighbours'
        // extra border vertices in as small-edge n-gons — the way the
        // reference CAD export absorbs count mismatches.
        (void)sideSum;
        (void)grow;
    }

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
                        bool* atLastV = nullptr) {
    const double u0 = surf.FirstUParameter();
    if (atLastV) *atLastV = false;
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

void meshRevolutionGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        const Model& model, const std::vector<int>& rimEdges,
                        const std::vector<int>& solvedEdge, int faceId,
                        int nu, int nv, MeshBuilder& out,
                        const std::vector<double>* vRowsOpt = nullptr) {
    nu = std::max(3, nu);
    nv = std::max(1, nv);
    const double v0 = surf.FirstVParameter();
    const double v1 = surf.LastVParameter();
    const double vspan = std::max(1e-12, v1 - v0);
    const double period = surf.LastUParameter() - surf.FirstUParameter();
    const bool vWrap = surf.IsVClosed();
    // Explicit row positions (insert faces put rows exactly at each slot
    // band's v-extents so deleted cells stay strictly interior and the
    // staircase closes). Only meaningful for open-v bands.
    const std::vector<double>* vRows =
        (!vWrap && vRowsOpt && vRowsOpt->size() >= 2) ? vRowsOpt : nullptr;
    if (vRows) nv = int(vRows->size()) - 1;
    if (vWrap) nv = std::max(3, nv);  // a wrapped ring of <3 rows is flat
    // Pole-to-pole band (full sphere / both-ends-closed revolve): both
    // end rows collapse to a point, so nv==1 would emit zero polygons.
    // The added interior ring is face-private (poles are degenerate
    // edges, the seam is internal), so no border sampling changes.
    if (!vWrap && !vRows && nv < 2) {
        auto rowDegenerate = [&](double v) {
            const gp_Pnt p0 = surf.Value(surf.FirstUParameter(), v);
            for (int i = 1; i < 8; ++i) {
                double u = surf.FirstUParameter() + period * i / 8.0;
                if (surf.Value(u, v).Distance(p0) > 1e-9) return false;
            }
            return true;
        };
        if (rowDegenerate(v0) && rowDegenerate(v1)) nv = 2;
    }
    const double dv = (v1 - v0) / nv;
    const int rows = vWrap ? nv : nv + 1;
    auto rowV = [&](int j) {
        return vRows ? (*vRows)[std::min<size_t>(j, vRows->size() - 1)]
                     : v0 + j * dv;
    };
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    // Rim rows sample the rim EDGE CURVES (like every chain mesher), so
    // multi-arc rims keep their joint vertices and neighbours weld
    // bit-identically; interior rows interpolate each column's u between
    // the two rims (wrap-shortest), twisting gently if the rims' origins
    // differ. Falls back to plain uniform rings when there are no usable
    // rims (full tori) or the two rims disagree in count.
    struct RimPt {
        double u;
        gp_Pnt p;
    };
    std::vector<RimPt> rim[2];
    // Face-local edge orientations: sampling must honour them (like
    // every other border sampler) so multi-edge rims with mixed curve
    // senses keep each arc joint exactly once.
    std::map<int, bool> revOf;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        int e = model.edges.FindIndex(ex.Current());
        if (e >= 1) {
            revOf[e] = ex.Current().Orientation() == TopAbs_REVERSED;
        }
    }
    for (int eid : rimEdges) {
        if (eid < 1 || eid > model.edgeCount()) continue;
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        double f2, l2, f3, l3;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f2, l2);
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        if (pc.IsNull() || c3.IsNull()) continue;
        gp_Pnt2d mid = pc->Value((f2 + l2) / 2);
        int side = std::abs(mid.Y() - v0) < std::abs(mid.Y() - v1) ? 0 : 1;
        int n = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
        if (n < 1) n = nu;
        const bool rev = revOf.count(eid) && revOf[eid];
        for (int i = 0; i < n; ++i) {
            double t = double(i) / n;
            if (rev) t = 1.0 - t;
            gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
            gp_Pnt p = c3->Value(f3 + (l3 - f3) * t);
            double u = uv.X();
            u -= period * std::floor((u - surf.FirstUParameter()) / period);
            rim[side].push_back({u, p});
        }
    }
    for (int k = 0; k < 2; ++k) {
        std::sort(rim[k].begin(), rim[k].end(),
                  [](const RimPt& a, const RimPt& b) { return a.u < b.u; });
    }

    // The rim samples OWN the border contract — they are what the
    // neighbouring faces emit on the shared edges, and a border row may
    // never be re-spaced (doctrine). When the rims disagree with the
    // requested radial count, the rims win: if they agree with each
    // other (or there is only one), the whole lattice follows them; if
    // the two rims themselves differ, the interior keeps the requested
    // density and each rim is stitched to its neighbouring uniform ring
    // by a closed transition strip of quads/5-gons below.
    const int nRim0 = int(rim[0].size());
    const int nRim1 = int(rim[1].size());
    const bool rim0ok = !vWrap && nRim0 >= 3;
    const bool rim1ok = !vWrap && nRim1 >= 3;
    if (rim0ok && nRim0 != nu && (!rim1ok || nRim1 == nRim0)) {
        nu = nRim0;
    } else if (!rim0ok && rim1ok && nRim1 != nu) {
        nu = nRim1;
    }
    dbg("revgrid face %d: nu=%d nv=%d rims=%d/%d wrap=%d rimEdges=%zu",
        faceId, nu, nv, nRim0, nRim1, vWrap ? 1 : 0, rimEdges.size());

    std::vector<std::vector<uint32_t>> ring(rows);
    const bool chained = !vWrap && int(rim[0].size()) == nu &&
                         (rim[1].empty() || int(rim[1].size()) == nu);
    if (chained) {
        // Column u at each rim (missing rim mirrors the other).
        const std::vector<RimPt>& A = rim[0];
        const std::vector<RimPt>& B = rim[1].empty() ? rim[0] : rim[1];
        // Rotational alignment of B to A (wrap-shortest total delta).
        int bestOff = 0;
        double bestSum = 1e300;
        for (int off = 0; off < nu; ++off) {
            double sum = 0;
            for (int i = 0; i < nu; ++i) {
                double d = B[(i + off) % nu].u - A[i].u;
                d -= period * std::round(d / period);
                sum += d * d;
            }
            if (sum < bestSum) {
                bestSum = sum;
                bestOff = off;
            }
        }
        for (int j = 0; j < rows; ++j) {
            double v = rowV(j);
            double w = (v - v0) / vspan;
            std::vector<gp_Pnt> pts(nu);
            std::vector<double> us(nu);
            for (int i = 0; i < nu; ++i) {
                double uA = A[i].u;
                double dU = B[(i + bestOff) % nu].u - uA;
                dU -= period * std::round(dU / period);
                us[i] = uA + dU * w;
                if (j == 0) pts[i] = A[i].p;  // exact curve points
                else if (j == nv && !rim[1].empty())
                    pts[i] = B[(i + bestOff) % nu].p;
                else pts[i] = surf.Value(us[i], v);
            }
            bool degenerate = true;
            for (int i = 1; i < nu && degenerate; ++i) {
                degenerate = pts[i].Distance(pts[0]) <= 1e-9;
            }
            if (degenerate) {
                ring[j].assign(nu, out.addVertex(pts[0], {faceId, us[0], v}));
            } else {
                ring[j].resize(nu);
                for (int i = 0; i < nu; ++i) {
                    ring[j][i] = out.addVertex(pts[i], {faceId, us[i], v});
                }
            }
        }
    }
    std::vector<double> ringU[2];  // rim-row u positions (bridge path)
    if (!chained) {
        const double du = period / nu;
        const double u0 = surf.FirstUParameter();
        for (int j = 0; j < rows; ++j) {
            double v = rowV(j);
            // Rim rows with usable samples are the EXACT rim points; the
            // strips below stitch them to the uniform interior.
            const int side = (j == 0 && rim0ok)          ? 0
                             : (j == rows - 1 && rim1ok) ? 1
                                                         : -1;
            if (side >= 0) {
                const auto& R = rim[side];
                ring[j].resize(R.size());
                for (size_t i = 0; i < R.size(); ++i) {
                    ring[j][i] =
                        out.addVertex(R[i].p, {faceId, R[i].u, v});
                    ringU[side].push_back(R[i].u);
                }
                continue;
            }
            std::vector<gp_Pnt> pts(nu);
            bool degenerate = true;
            for (int i = 0; i < nu; ++i) {
                pts[i] = surf.Value(u0 + i * du, v);
                if (i > 0 && pts[i].Distance(pts[0]) > 1e-9) {
                    degenerate = false;
                }
            }
            if (degenerate) {
                ring[j].assign(nu, out.addVertex(pts[0], {faceId, u0, v}));
            } else {
                ring[j].resize(nu);
                for (int i = 0; i < nu; ++i) {
                    ring[j][i] =
                        out.addVertex(pts[i], {faceId, u0 + i * du, v});
                }
            }
        }
    }

    for (int j = 0; j < nv; ++j) {
        const std::vector<uint32_t>& lo = ring[j];
        const std::vector<uint32_t>& hi = ring[(j + 1) % rows];
        if (lo.size() != hi.size()) continue;  // strip-bridged pair
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

    // Closed transition strips between an exact rim row and its
    // neighbouring ring when their counts differ: monotone circular
    // grouping by u — quads where the counts advance together, a 5-gon
    // (or a small fan against a degenerate ring) where the dense ring
    // contributes extra points. Winding matches the lattice cells:
    // lower row forward, upper row backward.
    auto emitClosedStrip = [&](const std::vector<uint32_t>& loI,
                               const std::vector<double>& loU,
                               const std::vector<uint32_t>& hiI,
                               const std::vector<double>& hiU) {
        const int nl = int(loI.size()), nh = int(hiI.size());
        if (nl < 3 || nh < 3) return;
        const bool loSparse = nl <= nh;
        const std::vector<uint32_t>& S = loSparse ? loI : hiI;
        const std::vector<double>& sU = loSparse ? loU : hiU;
        const std::vector<uint32_t>& D = loSparse ? hiI : loI;
        const std::vector<double>& dU = loSparse ? hiU : loU;
        const int ns = int(S.size()), nd = int(D.size());
        auto duAt = [&](int i) {
            int w = ((i % nd) + nd) % nd;
            return dU[w] + period * std::floor(double(i) / nd);
        };
        // m[k] = unwrapped dense index paired with sparse k, monotone,
        // closing after exactly one full turn.
        std::vector<int> m(ns + 1);
        double bd = 1e300;
        for (int i = 0; i < nd; ++i) {
            double d = std::abs(dU[i] - sU[0]);
            d = std::min(d, period - d);
            if (d < bd) { bd = d; m[0] = i; }
        }
        double tPrev = sU[0];
        for (int k = 1; k < ns; ++k) {
            double t = sU[k];
            while (t < tPrev - 1e-12) t += period;
            tPrev = t;
            int best = m[k - 1];
            double bestD = std::abs(duAt(best) - t);
            for (int i = m[k - 1] + 1; i <= m[0] + nd; ++i) {
                double d = std::abs(duAt(i) - t);
                if (d < bestD) { bestD = d; best = i; }
                if (duAt(i) > t + period / nd) break;
            }
            m[k] = best;
        }
        m[ns] = m[0] + nd;
        for (int k = 0; k < ns; ++k) {
            std::vector<uint32_t> ring2;
            if (loSparse) {
                ring2 = {S[k], S[(k + 1) % ns]};
                for (int i = m[k + 1]; i >= m[k]; --i) {
                    ring2.push_back(D[((i % nd) + nd) % nd]);
                }
            } else {
                for (int i = m[k]; i <= m[k + 1]; ++i) {
                    ring2.push_back(D[((i % nd) + nd) % nd]);
                }
                ring2.push_back(S[(k + 1) % ns]);
                ring2.push_back(S[k]);
            }
            ring2.erase(std::unique(ring2.begin(), ring2.end()), ring2.end());
            if (ring2.size() > 1 && ring2.front() == ring2.back()) {
                ring2.pop_back();
            }
            if (ring2.size() < 3) continue;
            out.addPolygon(std::move(ring2), faceId, flip);
        }
    };
    if (!chained) {
        const double du = period / nu;
        const double u0 = surf.FirstUParameter();
        auto uniformU = [&]() {
            std::vector<double> us(nu);
            for (int i = 0; i < nu; ++i) us[i] = u0 + i * du;
            return us;
        };
        if (rim0ok && rim1ok && rows == 2) {
            // No interior ring at all: bridge rim to rim directly.
            emitClosedStrip(ring[0], ringU[0], ring[1], ringU[1]);
        } else {
            if (rim0ok && ring[0].size() != ring[1].size()) {
                emitClosedStrip(ring[0], ringU[0], ring[1], uniformU());
            }
            if (rim1ok && ring[rows - 1].size() != ring[rows - 2].size()) {
                emitClosedStrip(ring[rows - 2], uniformU(), ring[rows - 1],
                                ringU[1]);
            }
        }
    }
}

// A full revolution band with interior slot/hole wires: mesh the plain
// grid, remove the cells the wires cover, and web the staircase to the
// wires' exact border sampling (3D edge curves at solved counts — the
// same contract the slot's wall faces sample, so the weld closes it).
bool meshRevolutionInsert(const TopoDS_Face& face,
                          const BRepAdaptor_Surface& surf, const Model& model,
                          const FacePlan& plan,
                          const std::vector<int>& solvedEdge, int faceId,
                          int nu, int nv, MeshBuilder& out) {
    // Row alignment (rows exactly at each band's v-extents) is what
    // makes the staircase close; a v-closed surface ignores explicit
    // rows, so refuse and let the face take the contract floor.
    if (surf.IsVClosed()) return false;
    // A wire whose solved counts can't even form a triangle would leave
    // its hole open; refuse up front and let the face fall back whole.
    for (const auto& wire : plan.insertWires) {
        int total = 0;
        for (int eid : wire) {
            total += eid > 0 && eid < (int)solvedEdge.size() &&
                             solvedEdge[eid] > 0
                         ? solvedEdge[eid]
                         : 8;
        }
        if (total < 3) return false;
    }
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double du = (u1 - u0) / std::max(3, nu);
    const double vspan = std::max(1e-12, v1 - v0);

    // UV bbox per wire. u grows by most of a cell so sliver cells go
    // too (u wraps, columns always exist on both sides); v stays EXACT —
    // the grid below places rows precisely at these extents, so deleted
    // cells are strictly interior and the staircase closes by
    // construction (a full-height deletion used to clip the rims and
    // leave the whole slot unwebbed, silently).
    struct Box { double u0, u1, v0, v1; };
    std::vector<Box> boxes;
    for (const auto& wire : plan.insertWires) {
        Box b{1e300, -1e300, 1e300, -1e300};
        for (int eid : wire) {
            double f, l;
            Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(
                TopoDS::Edge(model.edges(eid)), face, f, l);
            if (pc.IsNull()) continue;
            for (int k = 0; k <= 16; ++k) {
                gp_Pnt2d uv = pc->Value(f + (l - f) * k / 16.0);
                b.u0 = std::min(b.u0, uv.X());
                b.u1 = std::max(b.u1, uv.X());
                b.v0 = std::min(b.v0, uv.Y());
                b.v1 = std::max(b.v1, uv.Y());
            }
        }
        if (b.u0 > b.u1) return false;  // no usable pcurves on this wire
        b.u0 -= 0.6 * du; b.u1 += 0.6 * du;
        boxes.push_back(b);
    }
    // Row layout: rims plus every band extent. A wire too close to a rim
    // can't be banded — plan-time margins should have excluded it.
    std::vector<double> vRows{v0, v1};
    for (const Box& b : boxes) {
        if (b.v0 <= v0 + 0.01 * vspan || b.v1 >= v1 - 0.01 * vspan) {
            return false;
        }
        vRows.push_back(b.v0);
        vRows.push_back(b.v1);
    }
    std::sort(vRows.begin(), vRows.end());
    vRows.erase(std::unique(vRows.begin(), vRows.end(),
                            [&](double a, double c) {
                                return c - a < 1e-7 * vspan;
                            }),
                vRows.end());
    if (vRows.size() < 3 ||
        std::abs(vRows.back() - v1) > 1e-7 * vspan) {
        return false;
    }

    PolyMesh grid;
    {
        MeshBuilder tmp(grid);
        meshRevolutionGrid(face, surf, model, plan.uEdges, solvedEdge,
                           faceId, nu, int(vRows.size()) - 1, tmp, &vRows);
    }

    const double period = u1 - u0;
    auto covered = [&](const std::vector<uint32_t>& poly) {
        double cu = 0, cv = 0, u0ref = 0; int n = 0;
        for (uint32_t idx : poly) {
            const Anchor& a = grid.anchors[idx];
            if (a.faceId != faceId) return false;
            double u = a.u;
            if (!n) {
                u0ref = u;
            } else {
                // Seam cells mix u0 and u0+period anchors; unwrap
                // against the first corner or the center lands
                // mid-period and the wrong cells are deleted.
                u -= period * std::round((u - u0ref) / period);
            }
            cu += u; cv += a.v; ++n;
        }
        if (!n) return false;
        cu /= n; cv /= n;
        cu -= period * std::floor((cu - u0) / period);
        for (const Box& b : boxes) {
            if (cu >= b.u0 && cu <= b.u1 && cv >= b.v0 && cv <= b.v1) {
                return true;
            }
        }
        return false;
    };

    // Directed boundary edges before/after deletion; the difference is the
    // staircase around the removed region.
    auto directedBoundary = [](const PolyMesh& m, const std::vector<char>& keep) {
        std::map<std::pair<uint32_t, uint32_t>, int> use;
        for (size_t p = 0; p < m.polygons.size(); ++p) {
            if (!keep[p]) continue;
            const auto& poly = m.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                ++use[{std::min(a, b), std::max(a, b)}];
            }
        }
        std::map<uint32_t, uint32_t> next;  // directed open edges a->b
        for (size_t p = 0; p < m.polygons.size(); ++p) {
            if (!keep[p]) continue;
            const auto& poly = m.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                if (use[{std::min(a, b), std::max(a, b)}] == 1) next[a] = b;
            }
        }
        return next;
    };
    std::vector<char> all(grid.polygons.size(), 1);
    std::vector<char> keep(grid.polygons.size(), 1);
    bool any = false;
    for (size_t p = 0; p < grid.polygons.size(); ++p) {
        if (covered(grid.polygons[p])) { keep[p] = 0; any = true; }
    }
    // Wires present but nothing deleted: the intact grid would cover the
    // holes and every wall border would dangle. Refuse visibly.
    if (!any) return false;
    auto before = directedBoundary(grid, all);
    auto after = directedBoundary(grid, keep);

    // New staircase loops = directed open edges present now, absent
    // before. Extract and VALIDATE them BEFORE emitting anything: an
    // unclosed chain means the deletion clipped the outer boundary and
    // the hole could never be webbed — fail the whole face while it is
    // still un-emitted, so the planner's fallback stays contract-clean.
    std::map<uint32_t, uint32_t> stair;
    for (const auto& [a, b] : after) {
        auto it = before.find(a);
        if (it == before.end() || it->second != b) stair[a] = b;
    }
    std::vector<std::vector<uint32_t>> loops;
    while (!stair.empty()) {
        std::vector<uint32_t> loop;
        uint32_t start = stair.begin()->first, cur = start;
        bool closedLoop = false;
        while (loop.size() <= grid.vertices.size()) {
            auto it = stair.find(cur);
            if (it == stair.end()) break;
            loop.push_back(cur);
            cur = it->second;
            stair.erase(it);
            if (cur == start) { closedLoop = true; break; }
        }
        if (!closedLoop || loop.size() < 3) return false;
        loops.push_back(std::move(loop));
    }
    if (loops.empty()) return false;
    // Every wire must belong to a loop or its hole stays open.
    std::vector<std::vector<size_t>> loopWires(loops.size());
    {
        std::vector<char> assigned(boxes.size(), 0);
        for (size_t li = 0; li < loops.size(); ++li) {
            double lu0 = 1e300, lu1 = -1e300, lv0 = 1e300, lv1 = -1e300;
            for (uint32_t idx : loops[li]) {
                lu0 = std::min(lu0, grid.anchors[idx].u);
                lu1 = std::max(lu1, grid.anchors[idx].u);
                lv0 = std::min(lv0, grid.anchors[idx].v);
                lv1 = std::max(lv1, grid.anchors[idx].v);
            }
            for (size_t w = 0; w < boxes.size(); ++w) {
                if (assigned[w]) continue;
                double cu = (boxes[w].u0 + boxes[w].u1) / 2;
                double cv = (boxes[w].v0 + boxes[w].v1) / 2;
                if (cu >= lu0 && cu <= lu1 && cv >= lv0 && cv <= lv1) {
                    loopWires[li].push_back(w);
                    assigned[w] = 1;
                }
            }
        }
        for (char a : assigned) {
            if (!a) return false;
        }
        // And every loop needs at least one wire, or it has no lid.
        for (const auto& lw : loopWires) {
            if (lw.empty()) return false;
        }
    }

    // Topology validated. Build the whole result LOCALLY first — the
    // webs can still fail (ear-clip on a degenerate keyhole ring), and
    // a partially emitted face is a guaranteed leak. `out` receives the
    // part only after every web proved complete.
    PolyMesh webbedMesh;
    MeshBuilder wb(webbedMesh);
    std::vector<uint32_t> remap(grid.vertices.size(), UINT32_MAX);
    auto emitVert = [&](uint32_t i) {
        if (remap[i] == UINT32_MAX) {
            remap[i] = wb.addVertex(gp_Pnt(grid.vertices[i][0],
                                           grid.vertices[i][1],
                                           grid.vertices[i][2]),
                                    grid.anchors[i]);
        }
        return remap[i];
    };
    for (size_t p = 0; p < grid.polygons.size(); ++p) {
        if (!keep[p]) continue;
        std::vector<uint32_t> poly;
        poly.reserve(grid.polygons[p].size());
        for (uint32_t idx : grid.polygons[p]) poly.push_back(emitVert(idx));
        wb.addPolygon(std::move(poly), faceId, false);
    }

    const double rScale =
        std::max(1e-6, surf.Value((u0 + u1) / 2, (v0 + v1) / 2)
                           .Distance(surf.Value((u0 + u1) / 2 + 1e-3,
                                                (v0 + v1) / 2)) /
                           1e-3);

    // One web per staircase loop, splicing in every wire assigned to it
    // (nearby slots can merge into one staircase): keyhole ear-clip.
    for (size_t li = 0; li < loops.size(); ++li) {
        const std::vector<uint32_t>& loop = loops[li];
        const std::vector<size_t>& inLoop = loopWires[li];

        // Working ring: reversed staircase (it bounds the remaining mesh)
        // in synthetic planar coords + output vertex ids.
        std::vector<std::array<double, 3>> ringPts;
        std::vector<uint32_t> ringIds;
        {
            std::vector<uint32_t> outer(loop.rbegin(), loop.rend());
            for (uint32_t idx : outer) {
                ringPts.push_back({grid.anchors[idx].u * rScale,
                                   grid.anchors[idx].v, 0.0});
                ringIds.push_back(emitVert(idx));
            }
        }
        auto ringArea = [&]() {
            double a2 = 0;
            for (size_t i = 0; i < ringPts.size(); ++i) {
                const auto& p1 = ringPts[i];
                const auto& p2 = ringPts[(i + 1) % ringPts.size()];
                a2 += p1[0] * p2[1] - p2[0] * p1[1];
            }
            return a2;
        };
        const double outerSign = ringArea();

        for (size_t w : inLoop) {
            // Wire polyline: each edge sampled on its 3D curve at the
            // solved count (the same contract its wall faces sample), uv
            // through the pcurve; pieces chained by nearest endpoints.
            struct WPt { gp_Pnt p; double u, v; };
            std::vector<std::vector<WPt>> pieces;
            for (int eid : plan.insertWires[w]) {
                int n = eid > 0 && eid < (int)solvedEdge.size() &&
                                solvedEdge[eid] > 0
                            ? solvedEdge[eid]
                            : 8;
                const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
                double f, l;
                Handle(Geom2d_Curve) pc =
                    BRep_Tool::CurveOnSurface(edge, face, f, l);
                if (pc.IsNull()) continue;
                BRepAdaptor_Curve c(edge);
                const double f3 = c.FirstParameter(), l3 = c.LastParameter();
                std::vector<WPt> piece;
                for (int k = 0; k <= n; ++k) {
                    double t = double(k) / n;
                    gp_Pnt2d uv = pc->Value(f + t * (l - f));
                    piece.push_back(
                        {c.Value(f3 + t * (l3 - f3)), uv.X(), uv.Y()});
                }
                pieces.push_back(std::move(piece));
            }
            if (pieces.empty()) continue;
            std::vector<WPt> hole = pieces[0];
            std::vector<char> used(pieces.size(), 0);
            used[0] = 1;
            for (size_t step = 1; step < pieces.size(); ++step) {
                double bd = 1e300; size_t bi = 0; bool rev = false;
                for (size_t k = 0; k < pieces.size(); ++k) {
                    if (used[k]) continue;
                    double dF = hole.back().p.Distance(pieces[k].front().p);
                    double dB = hole.back().p.Distance(pieces[k].back().p);
                    if (dF < bd) { bd = dF; bi = k; rev = false; }
                    if (dB < bd) { bd = dB; bi = k; rev = true; }
                }
                used[bi] = 1;
                std::vector<WPt> pc2 = pieces[bi];
                if (rev) std::reverse(pc2.begin(), pc2.end());
                hole.insert(hole.end(), pc2.begin() + 1, pc2.end());
            }
            if (hole.size() > 1 &&
                hole.front().p.Distance(hole.back().p) < 1e-9) {
                hole.pop_back();
            }
            if (hole.size() < 3) continue;

            double aHole = 0;
            for (size_t i = 0; i < hole.size(); ++i) {
                const WPt& p1 = hole[i];
                const WPt& p2 = hole[(i + 1) % hole.size()];
                aHole += p1.u * rScale * p2.v - p2.u * rScale * p1.v;
            }
            std::vector<WPt> h = hole;
            if (outerSign * aHole > 0) std::reverse(h.begin(), h.end());

            // Splice this hole into the working ring at the nearest pair.
            size_t bo = 0, bh = 0; double bd = 1e300;
            for (size_t i = 0; i < ringPts.size(); ++i) {
                for (size_t j = 0; j < h.size(); ++j) {
                    double dx = ringPts[i][0] - h[j].u * rScale;
                    double dy = ringPts[i][1] - h[j].v;
                    double d = dx * dx + dy * dy;
                    if (d < bd) { bd = d; bo = i; bh = j; }
                }
            }
            std::vector<std::array<double, 3>> np;
            std::vector<uint32_t> ni;
            for (size_t i = 0; i <= bo; ++i) {
                np.push_back(ringPts[i]);
                ni.push_back(ringIds[i]);
            }
            std::vector<uint32_t> holeIds(h.size(), UINT32_MAX);
            auto holeId = [&](size_t j) {
                if (holeIds[j] == UINT32_MAX) {
                    holeIds[j] = wb.addVertex(
                        h[j].p, Anchor{faceId, h[j].u, h[j].v});
                }
                return holeIds[j];
            };
            for (size_t j = 0; j <= h.size(); ++j) {
                size_t k = (bh + j) % h.size();
                np.push_back({h[k].u * rScale, h[k].v, 0.0});
                ni.push_back(holeId(k));
            }
            for (size_t i = bo; i < ringPts.size(); ++i) {
                np.push_back(ringPts[i]);
                ni.push_back(ringIds[i]);
            }
            ringPts = std::move(np);
            ringIds = std::move(ni);
        }

        std::vector<uint32_t> ringIdx(ringPts.size());
        for (size_t i = 0; i < ringIdx.size(); ++i) ringIdx[i] = i;
        size_t emitted = 0;
        for (const auto& t : triangulatePoly(ringPts, ringIdx)) {
            uint32_t a = ringIds[t[0]], b = ringIds[t[1]],
                     c = ringIds[t[2]];
            if (a == b || b == c || a == c) continue;
            wb.addPolygon({a, b, c}, faceId, false);
            ++emitted;
        }
        // A complete ear-clip of a keyhole ring yields exactly V-2
        // triangles (bridge duplicates included). Anything less means
        // the web has an internal hole — fail the face un-emitted.
        if (emitted + 2 < ringPts.size()) return false;
    }

    // Every web complete: splat the local result into the real builder.
    std::vector<uint32_t> outMap(webbedMesh.vertices.size());
    for (uint32_t i = 0; i < webbedMesh.vertices.size(); ++i) {
        outMap[i] = out.addVertex(gp_Pnt(webbedMesh.vertices[i][0],
                                         webbedMesh.vertices[i][1],
                                         webbedMesh.vertices[i][2]),
                                  webbedMesh.anchors[i]);
    }
    for (const auto& poly : webbedMesh.polygons) {
        std::vector<uint32_t> mapped;
        mapped.reserve(poly.size());
        for (uint32_t idx : poly) mapped.push_back(outMap[idx]);
        out.addPolygon(std::move(mapped), faceId, false);
    }
    return true;
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

    // Emit the paired mesh AS IS: quads where two triangles merged,
    // triangles where nothing paired. The old midpoint subdivision
    // ("pure quads") quadrupled density and salted every border with
    // midpoint vertices no neighbour has — un-triangulating is the
    // whole job here, not adding edges.
    for (const auto& ring : paired) {
        std::vector<uint32_t> poly;
        poly.reserve(ring.size());
        for (int v : ring) poly.push_back(verts[v]);
        out.addPolygon(std::move(poly), faceId, flip);
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
    // Conforming across BODIES splices the other solid's vertex ids into
    // this face's polygons — contact faces then fuse into non-manifold
    // sandwiches. Neighbours must share the owning solid.
    std::vector<int> faceSolid(model.faceCount() + 1, 0);
    {
        int solidId = 0;
        auto assign = [&](const TopoDS_Shape& obj) {
            ++solidId;
            for (TopExp_Explorer fx(obj, TopAbs_FACE); fx.More();
                 fx.Next()) {
                int f2 = model.faces.FindIndex(fx.Current());
                if (f2 > 0 && faceSolid[f2] == 0) faceSolid[f2] = solidId;
            }
        };
        for (TopExp_Explorer sx(model.shape, TopAbs_SOLID); sx.More();
             sx.Next()) {
            assign(sx.Current());
        }
        for (TopExp_Explorer sx(model.shape, TopAbs_SHELL, TopAbs_SOLID);
             sx.More(); sx.Next()) {
            assign(sx.Current());
        }
    }
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
            // Contact edges in multi-body files carry 3+ faces; prefer an
            // ANALYTIC neighbour (its borders sample this very curve) over
            // whichever face happens to come last in the map — picking a
            // wrong-solid neighbour finds zero targets and the seam stays
            // open.
            int nfid = 0;
            if (model.edgeToFaces.Contains(ex.Current())) {
                for (const TopoDS_Shape& s :
                     model.edgeToFaces.FindFromKey(ex.Current())) {
                    int f2 = model.faces.FindIndex(s);
                    if (f2 == fid || f2 < 1) continue;
                    if (faceSolid[f2] != faceSolid[fid]) continue;
                    if (nfid < 1 || (!isAnalytic(nfid) && isAnalytic(f2))) {
                        nfid = f2;
                    }
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

            // Exact distance/parameter on the curve for a point. The
            // coarse polyline seeds the answer — Extrema can return
            // nothing at all on tiny curves, and an endpoint-only
            // fallback then bunches every projection at the two ends,
            // scrambling the nearest-by-param snap.
            auto projectPnt = [&](const gp_Pnt& p, double tol,
                                  double* paramOut) -> bool {
                double bestD = 1e300;
                double bestT = f;
                for (int i = 0; i < 33; ++i) {
                    double d = p.Distance(coarse[i]);
                    if (d < bestD) {
                        bestD = d;
                        bestT = f + (l - f) * i / 32.0;
                    }
                }
                if (bestD > tol + slack) return false;
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
            auto project = [&](uint32_t v, double tol,
                               double* paramOut) -> bool {
                return projectPnt(gp_Pnt(mesh.vertices[v][0],
                                         mesh.vertices[v][1],
                                         mesh.vertices[v][2]),
                                  tol, paramOut);
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
            // Fallback/ring borders lie ON their curves, so capture them
            // tightly — a loose radius kidnaps verts that belong to
            // ADJACENT edges when a face is thinner than the slack (a
            // 0.19mm strip's interior verts are within 0.12 of every
            // edge around it) and snapping folds them onto the corners.
            // Only decimated borders (quad-dominant simplification,
            // unconstrained grids) genuinely sit off-curve and keep the
            // loose chord-scaled capture.
            const FacePlan& myPlan = plans.at(fid);
            const bool decimatedBorder =
                myPlan.kind == MesherKind::QuadDominant ||
                (myPlan.kind == MesherKind::Fallback &&
                 (myPlan.forceFallbackQuads >= 0
                      ? myPlan.forceFallbackQuads != 0
                      : settings.forFace(fid).quadDominant)) ||
                (myPlan.kind == MesherKind::PlanarGrid &&
                 !myPlan.constrains);
            const double tolMoverPre =
                decimatedBorder
                    ? std::max(
                          1e-6 * (1.0 + edgeLen),
                          std::max(settings.forFace(fid).chordTolerance,
                                   nfid >= 1
                                       ? settings.forFace(nfid)
                                             .chordTolerance
                                       : 0.0) *
                              1.2)
                    : tolTarget;
            std::map<uint32_t, double> movers;  // vert -> snapped param
            std::map<uint32_t, gp_Pnt> moverOrig;  // pre-snap positions
            for (uint32_t v : borderVerts) {
                double t;
                if (project(v, tolMoverPre, &t)) {
                    movers[v] = t;
                    moverOrig.emplace(v, gp_Pnt(mesh.vertices[v][0],
                                                mesh.vertices[v][1],
                                                mesh.vertices[v][2]));
                }
            }
            if (movers.empty()) continue;

            std::vector<EdgeParamPoint> targets;
            if (analyticNb || freeformSeam) {
                // Triangulation NODES on an edge lie exactly on its curve
                // (only chord midpoints sag), so a freeform authority uses
                // the same tight projection as an analytic one. A loose
                // tolerance here captured the neighbour's verts on OTHER
                // nearly-collinear edges as targets, and the insertion
                // step then dragged this border onto them (folds).
                for (size_t v = range[nfid][0]; v < range[nfid][1]; ++v) {
                    double t;
                    if (project(uint32_t(v), tolTarget, &t)) {
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
            dbg("conform: face %d edge %d: %zu movers, %zu targets", fid,
                eid, movers.size(), targets.size());
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
            // Largest spacing between consecutive targets: a mover whose
            // nearest target is farther than this has NO partner on the
            // chain (the neighbour's matching vert sits off-curve on a
            // sloppy edge, or the chain doesn't reach the mover's end) —
            // snapping it anyway teleports it across the edge and folds
            // its polygons. Leave such movers where they are.
            double maxGap = 0.0;
            for (size_t i = 1; i < targets.size(); ++i) {
                maxGap = std::max(
                    maxGap, targets[i].param - targets[i - 1].param);
            }
            if (closed) {
                maxGap = std::max(
                    maxGap, period - (targets.back().param -
                                      targets.front().param));
            }

            // Equal counts: pair by RANK along the curve — a bijection.
            // Nearest-by-param can send two drifted movers to one target
            // and leave its neighbour unmatched, punching a hole in an
            // otherwise perfectly matched seam.
            if (movers.size() == targets.size() && targets.size() >= 2) {
                std::vector<std::pair<double, uint32_t>> mv;
                mv.reserve(movers.size());
                for (const auto& [v, t] : movers) mv.push_back({t, v});
                std::sort(mv.begin(), mv.end());
                const int nRank = int(mv.size());
                int bestShift = 0;
                if (closed) {
                    double bestCost = 1e300;
                    for (int sft = 0; sft < nRank; ++sft) {
                        double c = 0;
                        for (int i = 0; i < nRank; ++i) {
                            c += paramGap(targets[(i + sft) % nRank].param,
                                          mv[i].first);
                        }
                        if (c < bestCost) {
                            bestCost = c;
                            bestShift = sft;
                        }
                    }
                }
                for (int i = 0; i < nRank; ++i) {
                    const EdgeParamPoint& tgt =
                        targets[(i + bestShift) % nRank];
                    mesh.vertices[mv[i].second] = mesh.vertices[tgt.vert];
                    movers[mv[i].second] = tgt.param;
                }
                continue;
            }

            // Snap every mover to the nearest target (position + param).
            struct SnapPick {
                uint32_t v;
                const EdgeParamPoint* best;
                double dist;
            };
            std::vector<SnapPick> picks;
            for (auto& [v, t] : movers) {
                const EdgeParamPoint* best = &targets[0];
                for (const EdgeParamPoint& cand : targets) {
                    if (paramGap(cand.param, t) < paramGap(best->param, t)) {
                        best = &cand;
                    }
                }
                if (paramGap(best->param, t) > 0.75 * maxGap) {
                    dbg("conform: face %d edge %d vert %u kept (nearest "
                        "target %.4g away, max gap %.4g)",
                        fid, eid, v, paramGap(best->param, t), maxGap);
                    continue;
                }
                const auto& q = mesh.vertices[best->vert];
                picks.push_back(
                    {v, best,
                     moverOrig.at(v).Distance(gp_Pnt(q[0], q[1], q[2]))});
            }
            // (Injective snapping — one mover per target — was tried for
            // decimated borders and reverted: their zip against analytic
            // chains RELIES on many-to-one collapse; forcing uniqueness
            // exploded opens 10x across the sweep.)
            for (const SnapPick& s : picks) {
                mesh.vertices[s.v] = mesh.vertices[s.best->vert];
                movers[s.v] = s.best->param;
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
                    const double span = closed
                        ? std::fmod((forward ? pw - pu : pu - pw) + period,
                                    period)
                        : std::abs(pw - pu);
                    std::vector<const EdgeParamPoint*> between;
                    for (const EdgeParamPoint& cand : targets) {
                        double rel = closed
                            ? std::fmod((forward ? cand.param - pu
                                                 : pu - cand.param) + period,
                                        period)
                            : (forward ? cand.param - pu : pu - cand.param);
                        if (rel > 1e-12 && rel < span - 1e-12) {
                            between.push_back(&cand);
                        }
                    }
                    // A segment swallowing SEVERAL targets must actually
                    // LIE on this edge inside (pu,pw): a border segment of
                    // a DIFFERENT edge can still have both endpoints on
                    // this curve — the two ends of a nearly-closed arc are
                    // joined by its tiny closing edge — and inserting the
                    // chain there wraps the whole arc into that polygon a
                    // second time. Verified with the PRE-SNAP midpoint;
                    // one-or-two-target insertions skip the check (short
                    // spans put the midpoint near the boundary from sheer
                    // projection noise and were being starved).
                    if (between.size() >= 3) {
                        const gp_Pnt& a = moverOrig.at(u);
                        const gp_Pnt& b = moverOrig.at(w);
                        gp_Pnt mid((a.X() + b.X()) / 2, (a.Y() + b.Y()) / 2,
                                   (a.Z() + b.Z()) / 2);
                        double tm;
                        if (!projectPnt(mid,
                                        std::max(tolMoverPre,
                                                 0.3 * a.Distance(b)),
                                        &tm)) {
                            dbg("conform: face %d edge %d seg %u-%u skipped "
                                "(midpoint off curve)",
                                fid, eid, u, w);
                            continue;
                        }
                        double relm = closed
                            ? std::fmod((forward ? tm - pu : pu - tm) +
                                            period, period)
                            : (forward ? tm - pu : pu - tm);
                        if (relm < 0.1 * span || relm > 0.9 * span) {
                            dbg("conform: face %d edge %d seg %u-%u skipped "
                                "(midpoint rel %.3g of span %.4g)",
                                fid, eid, u, w, relm / span, span);
                            continue;
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

// Seam union v1 (the n-gon absorber, plan #37): purely topological
// T-junction healing. An OPEN directed edge (u,v) whose complement is a
// two-step path v->w->u on the neighbouring face means the neighbour
// sampled one extra vertex on the shared border; splicing w into (u,v)'s
// polygon turns a quad into a 5-gon with one short edge — exactly how
// Plasticity absorbs a denser neighbour — and the seam closes without
// moving or collapsing anything. Requiring the exact complement path
// (not curve proximity) makes sliver cross-talk impossible. Iterating
// lets chains of absorbed verts close multi-vert gaps one layer at a
// time. This pass is the contract that will let neighbouring faces
// disagree on border counts (strips vs fillet rings).
void unionSeams(PolyMesh& mesh, const Model& model, double weldTol) {
    (void)model;
    int total = 0;
    for (int pass = 0; pass < 8; ++pass) {
        std::map<std::pair<uint32_t, uint32_t>, size_t> polyOf;
        std::map<std::pair<uint32_t, uint32_t>, int> count;
        std::multimap<uint32_t, uint32_t> outOf;  // v -> w for edge (v,w)
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                auto key = std::make_pair(poly[i],
                                          poly[(i + 1) % poly.size()]);
                polyOf[key] = p;
                ++count[key];
                outOf.emplace(key.first, key.second);
            }
        }
        int spliced = 0;
        for (const auto& [e, c] : count) {
            if (c != 1 || count.count({e.second, e.first})) continue;
            const auto [u, v] = e;
            const auto& U = mesh.vertices[u];
            const auto& V = mesh.vertices[v];
            double ex = V[0] - U[0], ey = V[1] - U[1], ez = V[2] - U[2];
            double ee = ex * ex + ey * ey + ez * ez;
            if (ee < 1e-30) continue;
            // Curvature-tolerant on-segment test: the complement path of
            // an open edge on a CURVED border (cylinder rim, fillet rail)
            // lies on the arc, not the chord — its sagitta reaches 21% of
            // the chord at a 90-degree span, so an 8% chord slack silently
            // dropped every curved seam. Allow 25% perpendicular drift
            // plus an ellipse detour bound (|uw|+|wv| vs |uv|); the walk's
            // monotone parameter and the exact topological closure at u
            // remain the real gatekeepers.
            const double chord = std::sqrt(ee);
            const double slack = std::max(weldTol * 2.0, 0.25 * chord);
            auto onSegment = [&](uint32_t w, double tMax, double& tOut) {
                const auto& W = mesh.vertices[w];
                double px = W[0] - U[0], py = W[1] - U[1], pz = W[2] - U[2];
                double t = (px * ex + py * ey + pz * ez) / ee;
                if (t < -0.01 || t > tMax + 1e-9) return false;
                double dx = px - t * ex, dy = py - t * ey, dz = pz - t * ez;
                if (dx * dx + dy * dy + dz * dz > slack * slack) {
                    return false;
                }
                const double dU = std::sqrt(px * px + py * py + pz * pz);
                const double dV = std::sqrt(
                    (W[0] - V[0]) * (W[0] - V[0]) +
                    (W[1] - V[1]) * (W[1] - V[1]) +
                    (W[2] - V[2]) * (W[2] - V[2]));
                if (dU + dV > 1.35 * chord + 2.0 * weldTol) return false;
                tOut = t;
                return true;
            };
            // Complement path v -> w1 -> ... -> wk -> u on the denser
            // neighbour (multi-vertex gaps, handoff step 1): walk open
            // edges from v, each step landing ON the u-v segment with
            // strictly decreasing t, until an edge into u exists.
            std::vector<uint32_t> path;
            uint32_t cur = v;
            double tCur = 1.0;
            bool closed = false;
            for (int step = 0; step < 24 && !closed; ++step) {
                uint32_t nxt = UINT32_MAX;
                double tNxt = 0;
                bool ambiguous = false;
                for (auto it = outOf.lower_bound(cur);
                     it != outOf.end() && it->first == cur; ++it) {
                    const uint32_t w = it->second;
                    if (w == u && !path.empty()) {
                        if (count.count({w, u})) {}
                        // direct closure candidate handled below
                    }
                    if (w == u) {
                        if (!path.empty()) { nxt = u; tNxt = 0; }
                        continue;
                    }
                    double t;
                    if (!onSegment(w, tCur - 1e-9, t)) continue;
                    if (nxt != UINT32_MAX && nxt != u) {
                        ambiguous = true;  // two candidates: bail, safety
                        break;
                    }
                    if (nxt == UINT32_MAX || nxt == u) { nxt = w; tNxt = t; }
                }
                if (ambiguous || nxt == UINT32_MAX) break;
                if (nxt == u) { closed = true; break; }
                path.push_back(nxt);
                cur = nxt;
                tCur = tNxt;
                if (count.count({cur, u})) { closed = true; break; }
            }
            if (!closed || path.empty()) continue;
            // Splices add (u, p_k), reversed interiors, and (p_1, v):
            // none may already exist or the splice would open a
            // non-manifold edge instead of closing a seam.
            bool clash = count.count({u, path.back()}) ||
                         count.count({path.front(), v});
            for (size_t i = 0; i + 1 < path.size() && !clash; ++i) {
                clash = count.count({path[i + 1], path[i]}) > 0;
            }
            if (clash) continue;
            auto pit = polyOf.find(e);
            if (pit == polyOf.end()) continue;
            auto& poly = mesh.polygons[pit->second];
            for (size_t i = 0; i < poly.size(); ++i) {
                if (poly[i] == u && poly[(i + 1) % poly.size()] == v) {
                    // Ring runs u -> v; the complement ran v -> ... -> u,
                    // so insert the path REVERSED between them.
                    std::vector<uint32_t> rev(path.rbegin(), path.rend());
                    poly.insert(poly.begin() + i + 1, rev.begin(),
                                rev.end());
                    --count[{u, v}];
                    ++count[{u, rev.front()}];
                    for (size_t k = 0; k + 1 < rev.size(); ++k) {
                        ++count[{rev[k], rev[k + 1]}];
                    }
                    ++count[{rev.back(), v}];
                    spliced += int(rev.size());
                    break;
                }
            }
        }
        total += spliced;
        if (!spliced) break;
    }
    if (total) dbg("seam union: absorbed %d vert(s) into n-gons", total);
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
        // Curvature floor, every mode: a curved edge solved below its
        // turn angle collapses to chords — observed as two bracket-bend
        // quarter-pipes flattening into the SAME plane strip and weld-
        // fusing non-manifold. One segment per ~60 degrees of turn is
        // the least that keeps distinct geometry distinct; explicit
        // per-edge pins still win.
        if (settings.perEdge.count(eid)) continue;
        const TopoDS_Edge E = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(E)) continue;
        double f, l;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(E, f, l);
        if (c3.IsNull()) continue;
        GeomAdaptor_Curve gc(c3, f, l);
        if (gc.GetType() == GeomAbs_Line) continue;
        try {
            GCPnts_TangentialDeflection td(gc, M_PI / 3.0, 1e6, 2);
            const int floorN = std::clamp(td.NbPoints() - 1, 1, 32);
            if (solvedEdge[eid] < floorN) solvedEdge[eid] = floorN;
        } catch (const Standard_Failure&) {
        }
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
                // Chained Coons meshes from per-edge counts: fold them
                // into the cache key so density edits regenerate.
                int chainHash = 0;
                for (int sd = 0; sd < 4; ++sd) {
                    for (int e : plan.coonsSides[sd]) {
                        chainHash = chainHash * 31 +
                                    density.countFor(e, 1) * (sd + 1);
                    }
                }
                if (chainHash) counts[fid][2] = chainHash;
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
            "an%.6g fl%d fh%.6g jr%d qd%d mn%d ex%d ms%.6g rd%d sq%d cr%d "
            "ds%.4g",
            int(plan.kind), plan.constrains ? 1 : 0, plan.isFillet ? 1 : 0,
            plan.acrossIsU ? 1 : 0, plan.linkRims ? 1 : 0,
            plan.forceFallbackQuads, counts[fid][0], counts[fid][1],
            counts[fid][2], s.radial, s.axial, s.gridU, s.gridV, int(s.cap),
            s.chordTolerance, s.angleToleranceDeg, s.filletLoops,
            s.filletHold, s.junctionRings, s.quadDominant ? 1 : 0,
            s.minimal ? 1 : 0, s.exclude ? 1 : 0, s.minSize,
            s.relativeDeviation ? 1 : 0, s.squareCollar ? 1 : 0,
            s.coonsRotate, settings.densityScale);
        cacheKey[fid] = key;
        if (plan.kind == MesherKind::AnnulusRing || !plan.loops.empty()) {
            for (int eid : plan.uEdges) {
                cacheKey[fid] += "u" + std::to_string(solvedEdge[eid]);
            }
            for (int eid : plan.vEdges) {
                cacheKey[fid] += "v" + std::to_string(solvedEdge[eid]);
            }
        }
        // Every solved count the part consumes must key the cache, or a
        // density edit on a slot border / chained side / corner stub
        // reuses a stale part against a re-meshed neighbour.
        for (const auto& wire : plan.insertWires) {
            for (int eid : wire) {
                cacheKey[fid] += "w" + std::to_string(solvedEdge[eid]);
            }
        }
        for (int side = 0; side < 4; ++side) {
            for (int eid : plan.coonsSides[side]) {
                cacheKey[fid] += "c" + std::to_string(solvedEdge[eid]);
            }
        }
        if (plan.kind == MesherKind::CoonsGrid) {
            // The corner stub edge is discovered at mesh time (it is in
            // no plan list); key every border edge's count instead.
            for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE);
                 ex.More(); ex.Next()) {
                int eid = model.edges.FindIndex(ex.Current());
                if (eid >= 1 && eid < int(solvedEdge.size())) {
                    cacheKey[fid] += "e" + std::to_string(solvedEdge[eid]);
                }
            }
        }
    }

    // Mesh every face into its own part, in parallel, then merge in face
    // order so the output is deterministic (identical to the serial order).
    std::vector<PolyMesh> parts(faceN + 1);
    // Faces whose planned mesher couldn't build: the part is a fallback
    // triangulation, and the PLAN must follow (conform treats structured
    // meshers as exact-border authorities — a fallback part isn't one).
    std::vector<char> fellBack(faceN + 1, 0);
    int cacheHits = 0;
    if (cache) {
        for (int fid = 1; fid <= faceN; ++fid) {
            auto it = cache->faces.find(fid);
            if (it != cache->faces.end() &&
                it->second.key == cacheKey[fid]) {
                parts[fid] = it->second.part;  // copy: merge mutates
                fellBack[fid] = it->second.fellBack ? 1 : 0;
                cached[fid] = true;
                ++cacheHits;
            }
        }
    }
    // Border-contract oracle: does this part contain every border edge
    // of the face at its solved sampling (each consecutive pair of
    // 3D-curve samples present as a polygon edge)? Returns the first
    // offending edge id, 0 when clean. Seams (edges appearing twice in
    // the face's wires), degenerate and micro edges are exempt.
    auto borderContractViolation = [&](int fid,
                                       const PolyMesh& part) -> int {
        const TopoDS_Face F = TopoDS::Face(model.faces(fid));
        const double q = std::max(1e-9, settings.weldTolerance);
        std::map<std::tuple<long long, long long, long long>,
                 std::vector<uint32_t>>
            cells;
        for (uint32_t vi = 0; vi < part.vertices.size(); ++vi) {
            const auto& P = part.vertices[vi];
            cells[{llround(P[0] / q), llround(P[1] / q),
                   llround(P[2] / q)}]
                .push_back(vi);
        }
        // ALL vertices coinciding with a sample: pre-weld parts hold
        // duplicate corner ids (one per side/strip), and the polygon
        // edge may hang off any of them.
        auto nearVerts = [&](const gp_Pnt& p) {
            std::vector<uint32_t> hits;
            const long long cx = llround(p.X() / q),
                            cy = llround(p.Y() / q),
                            cz = llround(p.Z() / q);
            for (long long dx = -1; dx <= 1; ++dx) {
                for (long long dy = -1; dy <= 1; ++dy) {
                    for (long long dz = -1; dz <= 1; ++dz) {
                        auto it = cells.find({cx + dx, cy + dy, cz + dz});
                        if (it == cells.end()) continue;
                        for (uint32_t vi : it->second) {
                            const auto& P = part.vertices[vi];
                            const double ddx = P[0] - p.X();
                            const double ddy = P[1] - p.Y();
                            const double ddz = P[2] - p.Z();
                            if (ddx * ddx + ddy * ddy + ddz * ddz <
                                q * q) {
                                hits.push_back(vi);
                            }
                        }
                    }
                }
            }
            return hits;
        };
        std::set<uint64_t> partEdges;
        for (const auto& poly : part.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                const uint32_t a = poly[i];
                const uint32_t b = poly[(i + 1) % poly.size()];
                partEdges.insert((uint64_t(std::min(a, b)) << 32) |
                                 std::max(a, b));
            }
        }
        std::map<int, int> occur;  // seams appear twice in the wires
        for (TopExp_Explorer ex(F, TopAbs_EDGE); ex.More(); ex.Next()) {
            int eid = model.edges.FindIndex(ex.Current());
            if (eid >= 1) ++occur[eid];
        }
        for (const auto& [eid, cnt] : occur) {
            if (cnt != 1) continue;  // seam: internal to this face
            const TopoDS_Edge E = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(E)) continue;
            const int n = eid < int(solvedEdge.size()) ? solvedEdge[eid]
                                                       : 0;
            if (n < 1) continue;
            double f, l;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(E, f, l);
            if (c3.IsNull()) continue;
            if (n == 1 &&
                c3->Value(f).Distance(c3->Value(l)) < 4.0 * q) {
                continue;  // micro edge: below weld resolution
            }
            // Corner gaps on sloppy CAD reach the EDGE tolerance (1e-4
            // and worse) and are healed later by corner
            // canonicalization — endpoint samples get that tolerance,
            // interior samples stay at weld exactness.
            // Recorded tolerances LIE on sloppy exports (observed: a
            // 1.4e-4 corner gap on an edge claiming 1e-6). Bound the
            // endpoint radius by the local sample spacing instead —
            // 40% of a step can never capture the wrong border sample.
            const double eTol = std::max(
                {q, BRep_Tool::Tolerance(E),
                 0.4 * c3->Value(f).Distance(c3->Value(l)) /
                     double(std::max(1, n))});
            auto nearVertsEnd = [&](const gp_Pnt& p) {
                std::vector<uint32_t> hits = nearVerts(p);
                if (!hits.empty() || eTol <= q) return hits;
                for (uint32_t vi = 0; vi < part.vertices.size(); ++vi) {
                    const auto& P = part.vertices[vi];
                    const double dx = P[0] - p.X(), dy = P[1] - p.Y(),
                                 dz = P[2] - p.Z();
                    if (dx * dx + dy * dy + dz * dz < eTol * eTol) {
                        hits.push_back(vi);
                    }
                }
                return hits;
            };
            std::vector<uint32_t> prev = nearVertsEnd(c3->Value(f));
            for (int i = 1; i <= n; ++i) {
                std::vector<uint32_t> cur =
                    i == n ? nearVertsEnd(c3->Value(l))
                           : nearVerts(c3->Value(f + (l - f) * i / n));
                bool linked = false;
                for (uint32_t a : prev) {
                    for (uint32_t b : cur) {
                        if (a != b &&
                            partEdges.count(
                                (uint64_t(std::min(a, b)) << 32) |
                                std::max(a, b))) {
                            linked = true;
                            break;
                        }
                    }
                    if (linked) break;
                }
                if (!linked) {
                    const gp_Pnt sp = i == n
                                          ? c3->Value(l)
                                          : c3->Value(f + (l - f) * i / n);
                    double bn = 1e300;
                    for (uint32_t vi = 0; vi < part.vertices.size(); ++vi) {
                        const auto& P = part.vertices[vi];
                        const double dx = P[0] - sp.X(), dy = P[1] - sp.Y(),
                                     dz = P[2] - sp.Z();
                        bn = std::min(bn, dx * dx + dy * dy + dz * dz);
                    }
                    dbg("contract check %d: edge %d sample %d/%d: %s "
                        "(nearest %.3g, eTol %.3g)",
                        fid, eid, i, n,
                        prev.empty() || cur.empty() ? "no vertex" : "no edge",
                        std::sqrt(bn), eTol);
                    return eid;
                }
                prev = std::move(cur);
            }
        }
        return 0;
    };

    // One demotion path for every mesher failure: the contract floor
    // first (exact borders, cannot leak), verified; the raw OCCT
    // triangulation only when even that is unavailable.
    auto demote = [&](int fid, const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf,
                      const FaceMeshSettings& s, const char* why) {
        parts[fid] = PolyMesh();
        fellBack[fid] = 1;
        {
            MeshBuilder retry(parts[fid]);
            const bool built = meshContractFallback(face, model, fid,
                                                    solvedEdge, s.radial,
                                                    retry);
            const int floorBad =
                built ? borderContractViolation(fid, parts[fid]) : -1;
            if (built && floorBad == 0) {
                dbg("mesh face %d: %s -> contract floor", fid, why);
                return;
            }
            dbg("mesh face %d: floor %s (edge %d)", fid,
                built ? "violates contract" : "failed to build", floorBad);
        }
        parts[fid] = PolyMesh();
        MeshBuilder retry(parts[fid]);
        meshFallback(face, surf, fid, s, retry);
        dbg("mesh face %d: %s -> OCCT fallback (no contract floor)", fid,
            why);
    };

    auto meshFace = [&](int fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        if (s.exclude) return;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FacePlan& plan = plans.at(fid);
        dbg("mesh face %d: %s", fid, mesherKindName(plan.kind));
        BRepAdaptor_Surface surf(face);
        MeshBuilder out(parts[fid]);
        const int nu = counts[fid][0], nv = counts[fid][1];

        // Both rims' phases, each mapped to its own v end; a band with
        // one rim (or none) uses the same phase at both ends.
        auto revPhases = [&]() -> std::pair<double, double> {
            double p0 = surf.FirstUParameter(), p1 = p0;
            bool have0 = false, have1 = false;
            for (size_t k = 0; k < plan.uEdges.size() && k < 2; ++k) {
                bool atV1 = false;
                double ph =
                    revolutionUPhase(surf, model, plan.uEdges[k], &atV1);
                if (atV1 && !have1) { p1 = ph; have1 = true; }
                else if (!atV1 && !have0) { p0 = ph; have0 = true; }
            }
            if (have0 && !have1) p1 = p0;
            if (have1 && !have0) p0 = p1;
            return {p0, p1};
        };
        switch (plan.kind) {
            case MesherKind::RevolutionGrid:
                if (!plan.insertWires.empty()) {
                    // Before the taper branch: a taper never cuts the
                    // slots out, so insert bands go first regardless of
                    // rim linkage.
                    if (!meshRevolutionInsert(face, surf, model, plan,
                                              solvedEdge, fid, nu, nv,
                                              out)) {
                        demote(fid, face, surf, s,
                               "revolution insert failed");
                    }
                } else if (!plan.linkRims && counts[fid][2] > 0 &&
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
                    auto [p0, p1] = revPhases();
                    meshRevolutionTaper(face, surf, fid, nA, nB, p0, p1,
                                        out);
                } else {
                    meshRevolutionGrid(face, surf, model, plan.uEdges,
                                       solvedEdge, fid, nu, nv, out);
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
                                   clusteredParams(nv, holdV), s.coonsRotate,
                                   solvedEdge, out)) {
                    demote(fid, face, surf, s, "coons failed");
                }
                break;
            }
            case MesherKind::MinimalNGon:
                if (!plan.loops.empty()) {
                    if (!meshMinimalPlanar(face, model, fid, solvedEdge,
                                           s.radial, out)) {
                        demote(fid, face, surf, s, "minimal planar failed");
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
                if (!meshAnnulusRing(face, model, fid, plan.uEdges,
                                     plan.vEdges, solvedEdge, s.radial,
                                     out)) {
                    demote(fid, face, surf, s, "annulus ring failed");
                }
                break;
            case MesherKind::PlateWeb:
                if (!meshPlateWeb(face, surf, model, fid, solvedEdge,
                                  s.radial, s.junctionRings, s.squareCollar,
                                  out)) {
                    demote(fid, face, surf, s, "plate web failed");
                }
                break;
            case MesherKind::QuadFill:
                if (!meshQuadFill(face, surf, model, fid, solvedEdge,
                                  s.radial, s.minSize, out)) {
                    demote(fid, face, surf, s, "quad fill failed");
                }
                break;
            case MesherKind::QuadDominant:
            case MesherKind::Fallback: {
                FaceMeshSettings fs = s;
                if (plan.forceFallbackQuads >= 0) {
                    fs.quadDominant = plan.forceFallbackQuads != 0;
                }
                // The global budget knob reaches triangulations too:
                // deflection error scales with the SQUARE of linear
                // density, angle linearly (already in the cache key).
                const double dsc =
                    std::clamp(settings.densityScale, 0.05, 20.0);
                if (dsc != 1.0) {
                    fs.chordTolerance /= dsc * dsc;
                    fs.angleToleranceDeg =
                        std::clamp(fs.angleToleranceDeg / dsc, 1.0, 60.0);
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
                demote(fid, face, surf, s, "self-check failed");
            }
        }
        // Border-contract postcondition: any face that cannot prove its
        // borders at the solved sampling demotes to the contract floor
        // VISIBLY — a silent contract break is a guaranteed open seam
        // after the weld. Fallback parts are exempt (they are the
        // floor's floor), as are deliberate clustered fillet holds.
        if (!fellBack[fid] && plan.kind != MesherKind::Fallback &&
            plan.kind != MesherKind::QuadDominant &&
            !(plan.isFillet && s.filletHold > 0.0)) {
            const int bad = borderContractViolation(fid, parts[fid]);
            if (bad) {
                dbg("mesh face %d: border contract failed on edge %d (%s)",
                    fid, bad, mesherKindName(plan.kind));
                demote(fid, face, surf, s, "border contract failed");
            }
        }
    };

    unsigned threads = std::min<unsigned>(
        std::max(1u, std::thread::hardware_concurrency()), unsigned(faceN));
    if (!settings.parallelMeshing) threads = 1;
    if (threads > 1) {
        // OCCT computes pcurves and UV bounds lazily and caches them on
        // the SHARED TShape — workers racing through
        // BRepTools::AddUVBounds / BRep_Tool::CurveOnSurface segfault on
        // large assemblies (observed inside meshCoonsGrid on an 8k-face
        // model). Warm every face's caches single-threaded first; the
        // parallel pass then only reads.
        for (int fid = 1; fid <= faceN; ++fid) {
            if (cached[fid]) continue;
            try {
                Bnd_Box2d warm;
                BRepTools::AddUVBounds(TopoDS::Face(model.faces(fid)), warm);
                (void)BRep_Tool::Surface(TopoDS::Face(model.faces(fid)));
            } catch (...) {
                // A face too broken to bound fails later, visibly.
            }
        }
        for (int eid = 1; eid <= model.edgeCount(); ++eid) {
            try {
                double f = 0, l = 0;
                (void)BRep_Tool::Curve(TopoDS::Edge(model.edges(eid)), f, l);
            } catch (...) {
            }
        }
    }
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
                cache->faces[fid] = {cacheKey[fid], parts[fid],
                                     fellBack[fid] != 0};
            }
        }
    }

    // A part that fell back is a fallback for EVERY downstream stage:
    // conform must treat its borders as freeform movers, not as an
    // exact-border authority, and the report must tell the truth.
    for (int fid = 1; fid <= faceN; ++fid) {
        if (!fellBack[fid]) continue;
        FacePlan& pl = plans.at(fid);
        if (pl.kind == MesherKind::Fallback) continue;
        dbg("mesh face %d: %s couldn't build, plan demoted to fallback",
            fid, mesherKindName(pl.kind));
        pl.kind = MesherKind::Fallback;
        pl.constrains = false;
        pl.coonsSides = {};
        pl.loops.clear();
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

    // Corner canonicalization: curve endpoints of DIFFERENT edges meeting
    // at one B-rep vertex disagree by the vertex tolerance (~1e-4 on real
    // exports), far above the weld tolerance — every face computes its
    // corner from its own edge, so corners never welded. Snap any mesh
    // vertex within a B-rep vertex's tolerance onto its exact point.
    auto finish = [&](PolyMesh& mesh) {
        weft::ShapeMap vmap;
        TopExp::MapShapes(model.shape, TopAbs_VERTEX, vmap);

        // Micro-edge collapse: CAD booleans leave hairline edges (a few
        // microns to ~0.1mm on real parts) whose two vertices are
        // distinct, so sliver faces and unmatchable seams survive every
        // weld. Union the endpoints of any edge shorter than 5e-4 of the
        // model diagonal (a conventional stitch tolerance) — sliver
        // polygons then degenerate away in the weld and the flanking
        // faces zip directly.
        std::vector<int> root(vmap.Extent() + 1);
        std::iota(root.begin(), root.end(), 0);
        auto find = [&](int i) {
            while (root[i] != i) i = root[i] = root[root[i]];
            return i;
        };
        Bnd_Box bb;
        BRepBndLib::Add(model.shape, bb);
        const double microTol = 5e-4 * std::sqrt(bb.SquareExtent());
        int microEdges = 0;
        // Reach: capture radius a fused group needs so that mesh verts
        // SAMPLED ALONG a collapsed micro-edge (a fallback neighbour puts
        // interior nodes on it) snap to the representative too.
        std::vector<double> reach(vmap.Extent() + 1, 0.0);
        for (TopExp_Explorer ex(model.shape, TopAbs_EDGE); ex.More();
             ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(e, v1, v2);
            if (v1.IsNull() || v2.IsNull() || v1.IsSame(v2)) continue;
            if (BRep_Tool::Pnt(v1).Distance(BRep_Tool::Pnt(v2)) >= microTol) {
                continue;
            }
            BRepAdaptor_Curve c(e);
            const double len = GCPnts_AbscissaPoint::Length(c);
            if (len >= microTol) continue;
            const int a = find(vmap.FindIndex(v1));
            const int b = find(vmap.FindIndex(v2));
            if (a != b) {
                root[a] = b;
                reach[b] += reach[a] + len;
            } else {
                reach[b] += len;
            }
            ++microEdges;
        }

        // Snap-test against each vertex's own point/tolerance, but send
        // the mesh vertex to its GROUP representative's point.
        struct Corner {
            gp_Pnt at;      // where mesh verts of this B-rep vertex land
            double tol;     // capture radius around this vertex
            gp_Pnt target;  // canonical point (group representative)
        };
        std::vector<Corner> corners;
        for (int i = 1; i <= vmap.Extent(); ++i) {
            const TopoDS_Vertex v = TopoDS::Vertex(vmap(i));
            const int r = find(i);
            corners.push_back(
                {BRep_Tool::Pnt(v),
                 std::max({1e-7, 2.0 * BRep_Tool::Tolerance(v),
                           1.05 * reach[r]}),
                 BRep_Tool::Pnt(TopoDS::Vertex(vmap(r)))});
        }
        size_t snapped = 0;
        for (auto& mv : mesh.vertices) {
            gp_Pnt p(mv[0], mv[1], mv[2]);
            for (const auto& c : corners) {
                if (p.SquareDistance(c.at) < c.tol * c.tol) {
                    mv = {c.target.X(), c.target.Y(), c.target.Z()};
                    ++snapped;
                    break;
                }
            }
        }
        dbg("generate: %zu corner verts canonicalized, %d micro edges "
            "collapsed",
            snapped, microEdges);

        // Solid-scoped weld: contacting bodies in a multi-body file have
        // coincident skins with opposing windings — a global weld fuses
        // them into non-manifold shared edges (every directed edge used
        // twice). Group vertices by owning solid so only same-body seams
        // merge.
        std::vector<int> weldGroup;
        {
            std::vector<int> faceSolid(faceN + 1, 0);
            int solidId = 0;
            auto assign = [&](const TopoDS_Shape& obj) {
                ++solidId;
                for (TopExp_Explorer fx(obj, TopAbs_FACE); fx.More();
                     fx.Next()) {
                    int fid = model.faces.FindIndex(fx.Current());
                    if (fid > 0 && faceSolid[fid] == 0) {
                        faceSolid[fid] = solidId;
                    }
                }
            };
            for (TopExp_Explorer sx(model.shape, TopAbs_SOLID); sx.More();
                 sx.Next()) {
                assign(sx.Current());
            }
            for (TopExp_Explorer sx(model.shape, TopAbs_SHELL, TopAbs_SOLID);
                 sx.More(); sx.Next()) {
                assign(sx.Current());
            }
            dbg("weld: %d body group(s)", solidId);
            if (solidId > 1) {
                weldGroup.assign(mesh.vertices.size(), 0);
                for (int fid = 1; fid <= faceN; ++fid) {
                    for (size_t v = range[fid][0]; v < range[fid][1]; ++v) {
                        weldGroup[v] = faceSolid[fid];
                    }
                }
            }
        }

        dbg("generate: welding%s", weldGroup.empty() ? "" : " (per solid)");
        weldVertices(mesh, settings.weldTolerance,
                     weldGroup.empty() ? nullptr : &weldGroup);
    };

    finish(mesh);
    if (settings.conformBorders) {
        // Post-weld: borders share ids now, so an open edge with an exact
        // complement path is a REAL T-junction, never a pre-weld ghost.
        unionSeams(mesh, model, settings.weldTolerance);
    }

    // Fold cleanup: a directed edge traversed twice WITHIN one face means
    // conform or decimation wrapped a flap of polygons over its
    // neighbours. The flap is the smaller overlapping polygon — drop it;
    // the tiny open it leaves beats a non-manifold fold.
    {
        std::map<std::pair<uint32_t, uint32_t>, std::vector<size_t>> dir;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                dir[{poly[i], poly[(i + 1) % poly.size()]}].push_back(p);
            }
        }
        auto polyArea = [&](size_t p) {
            const auto& poly = mesh.polygons[p];
            double nx = 0, ny = 0, nz = 0;
            for (size_t i = 0; i < poly.size(); ++i) {
                const auto& a = mesh.vertices[poly[i]];
                const auto& b = mesh.vertices[poly[(i + 1) % poly.size()]];
                nx += a[1] * b[2] - a[2] * b[1];
                ny += a[2] * b[0] - a[0] * b[2];
                nz += a[0] * b[1] - a[1] * b[0];
            }
            return 0.5 * std::sqrt(nx * nx + ny * ny + nz * nz);
        };
        std::set<size_t> drop;
        for (const auto& [e, ps] : dir) {
            if (ps.size() < 2) continue;
            bool sameFace = true;
            for (size_t p : ps) {
                sameFace &= mesh.polygonFaceId[p] ==
                            mesh.polygonFaceId[ps[0]];
            }
            if (!sameFace) continue;  // cross-face dup: not a local flap
            size_t keep = ps[0];
            double best = -1.0;
            for (size_t p : ps) {
                double a = polyArea(p);
                if (a > best) {
                    best = a;
                    keep = p;
                }
            }
            for (size_t p : ps) {
                if (p != keep) drop.insert(p);
            }
        }
        if (!drop.empty()) {
            std::vector<std::vector<uint32_t>> polys;
            std::vector<int> polyFace;
            polys.reserve(mesh.polygons.size() - drop.size());
            polyFace.reserve(polys.capacity());
            for (size_t p = 0; p < mesh.polygons.size(); ++p) {
                if (drop.count(p)) continue;
                polys.push_back(std::move(mesh.polygons[p]));
                polyFace.push_back(mesh.polygonFaceId[p]);
            }
            mesh.polygons = std::move(polys);
            mesh.polygonFaceId = std::move(polyFace);
            dbg("generate: %zu folded polygons dropped", drop.size());
        }
    }
    dbg("generate: done (%zu verts, %zu polys)", mesh.vertexCount(),
        mesh.polygonCount());
    return mesh;
}

std::vector<uint8_t> foldedPolys(const Model& model, const PolyMesh& mesh) {
    std::vector<uint8_t> folded(mesh.polygons.size(), 0);
    // Surface adaptors are built lazily per face; polygons arrive grouped
    // by face so in practice each face is built once.
    int curFace = 0;
    std::unique_ptr<BRepAdaptor_Surface> surf;
    double orient = 1.0;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const int fid = mesh.polygonFaceId[p];
        if (fid <= 0 || fid > model.faceCount()) continue;
        const auto& poly = mesh.polygons[p];
        // Newell normal: robust winding normal for any planar-ish polygon.
        double nx = 0, ny = 0, nz = 0;
        for (size_t i = 0; i < poly.size(); ++i) {
            const auto& a = mesh.vertices[poly[i]];
            const auto& b = mesh.vertices[poly[(i + 1) % poly.size()]];
            nx += (a[1] - b[1]) * (a[2] + b[2]);
            ny += (a[2] - b[2]) * (a[0] + b[0]);
            nz += (a[0] - b[0]) * (a[1] + b[1]);
        }
        const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (nlen < 1e-14) continue;  // degenerate: no winding to judge
        if (fid != curFace) {
            const TopoDS_Face& face = TopoDS::Face(model.faces(fid));
            surf = std::make_unique<BRepAdaptor_Surface>(face);
            orient = face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
            curFace = fid;
        }
        // Every vertex with an anchor on this face votes: surface normal
        // at ITS OWN uv against the polygon winding. Per-vertex sampling
        // (not a uv average) keeps periodic surfaces honest — averaging
        // across a cylinder's seam lands on the far side of the barrel.
        int votes = 0;
        for (uint32_t vi : poly) {
            if (vi >= mesh.anchors.size()) continue;
            const Anchor& an = mesh.anchors[vi];
            if (an.faceId != fid) continue;
            gp_Pnt sp;
            gp_Vec du, dv;
            surf->D1(an.u, an.v, sp, du, dv);
            gp_Vec sn = du.Crossed(dv);
            const double slen = sn.Magnitude();
            if (slen < 1e-14) continue;  // pole: normal undefined there
            const double dot =
                orient * (sn.X() * nx + sn.Y() * ny + sn.Z() * nz) /
                (slen * nlen);
            if (dot > 0.1) ++votes;
            else if (dot < -0.1) --votes;
        }
        if (votes < 0) folded[p] = 1;
    }
    return folded;
}

}  // namespace weft
