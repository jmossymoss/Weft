#pragma once

#include "weft/meshers.hpp"

#include "mesher_sampling.hpp"
#include "mesher_trace.hpp"

#include <functional>
#include <limits>
#include <sstream>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Extrema_ExtPC.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GeomAdaptor_Curve.hxx>
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
#include <Geom2dAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
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
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <thread>
#include <tuple>
#include <vector>


namespace weft::mesher_impl {
using mesher_detail::dbg;
using mesher_detail::edgeIsPinned;
using mesher_detail::edgeSampleFractions;
using mesher_detail::phasedT;
using mesher_detail::PinnedEdges;
using mesher_detail::stableDeflectionCount;

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
    PolyMesh& mesh() { return mesh_; }

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
    // Revolution rims split by band side (v-low / v-high). When a rim is
    // several edges (a T-junction interrupts the circle), the two rims
    // carry a SUM constraint — equal totals — not per-edge equality.
    std::vector<int> rimLow;
    std::vector<int> rimHigh;
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
    // Partial-wrap revolution bands: the two full-height u-iso side
    // edges bounding the open band. They carry the row contract the way
    // a closed band's seam does; everything else on the outline is a rim
    // chain. Empty for closed bands.
    std::vector<int> bandSides;
    // The open band's column driver: a lone flat full-span rim edge
    // whose solved count sets nu (the other rim — however castellated —
    // never dictates the column count; strips and notch webs absorb any
    // mismatch inside the face). 0 when neither chain qualifies.
    int bandDriver = 0;
    // uspan / 2*pi: 'radial' keeps meaning divisions per full turn, so
    // a partial wrap takes its proportional share.
    double bandWrapFrac = 1.0;
    // Full-wrap castellated rim: a u-closed ruled band (cylinder/cone)
    // whose one rim is a plain full circle and the other carries a
    // rim-open notch (walls that drop from the rim to an interior floor).
    // Meshes as a STRAIGHT uniform lattice driven by the plain rim, with
    // the notch boolean-cut and its walls/floor webbed to their exact
    // samples — the full-wrap sibling of the open-band straight lattice.
    // The plain rim (never the castellated total) drives the column count,
    // so the rim-SUM equalization skips these the way it skips open bands.
    bool castellated = false;
    int plainRimEdge = 0;  // the plain full-circle rim: drives nu
    // Forced fallback flavour: -1 = follow settings, 0 = pure tris,
    // 1 = quad-dominant (used when the user forces a mesher).
    int forceFallbackQuads = -1;
    // Hole-cutout lattice floors: a face with interior trim wires needs
    // rows/columns FINE ENOUGH that each hole spans whole cells — the
    // hole demands them even when the border edges are dead straight
    // (straight edges alone propose 1). Solved into the border counts so
    // the lattice lines run border to border with no transition strips.
    int insertMinU = 0;
    int insertMinV = 0;
    // Chained Coons: edge ids per side (wire order) when any side is a
    // chain of several edges. Opposite sides then match by SUM of their
    // per-edge counts (solveDensity's chain pass) instead of union-find.
    std::array<std::vector<int>, 4> coonsSides;
    // PlateWeb: every boundary wire's edge chain (loops[0] = outer wire).
    // Each edge solves independently — a bore drives its own hole loop.
    std::vector<std::vector<int>> loops;
    // Open (C-shaped) annulus band: a single wire of two concentric arc
    // rails (uEdges = outer, vEdges = inner) joined by two radial walls
    // (cWalls). Meshes as radial quad spokes between the rails — the
    // notch-open sibling of the two-closed-loop AnnulusRing. When set the
    // AnnulusRing dispatch routes to meshAnnulusCRing instead of the
    // zipper. The wall edges carry the across-ring (radial row) count.
    bool cRing = false;
    std::vector<int> cWalls;
    // DomeCap: the revolution-like parametrization the mesher walks. The
    // AZIMUTH runs along one parametric direction (periodic / the base loop
    // wraps around the axis), the POLAR angle along the other — one polar end
    // is the base loop (domePolarBase), the opposite end collapses to the pole
    // (domePolarPole). uEdges carry the base rim (drives the meridian count),
    // vEdges the polar boundary meridian (drives the latitude ring count).
    bool domeAzimIsV = true;   // azimuth is the surface's V direction
    double domePolarBase = 0;  // polar param at the base loop
    double domePolarPole = 0;  // polar param at the collapsed pole
    // A bent ribbon carrying a rectangular end notch whose outline STILL
    // chains into a Coons patch (the pocket absorbed into one side). The plan
    // stays Coons — density, borders, and the dense/coarse fallback are all
    // the historic Coons ones — but the dispatch first TRIES the rail sweep's
    // clean notch cut, keeping it only when it welds exactly (a fine-railed
    // solve). When the solve is too coarse for the cut to weld, the untouched
    // Coons plan carries the face exactly as before.
    bool tryRibbonNotch = false;
    // Long thin CHAINED coons strip: chain pieces with unequal densities
    // pair rung stations by piece fraction, not arc length, so every rung
    // shears diagonally (the flaregun jacket / grip-band class). The rail
    // sweep pairs stations by arc length; dispatch tries it first and
    // keeps coons as the byte-identical fallback.
    bool tryRibbonSweep = false;
    // Effective wire rotation for Coons. Analytic drum trims canonicalize
    // this so planning, the GPU proxy, and the CPU mesher share axes.
    int coonsRotate = 0;
    // A single-wire UV-orthogonal trim. This covers rectangular patches
    // whose sides were split by STEP bookkeeping, and cylinders whose side
    // repeatedly steps in/out around slots. The trim
    // is meshed by one global parameter lattice: every surviving cell is a
    // quad, so a notch cannot restart the cylinder spans or create a fan.
    bool orthogonalTrimGrid = false;
    // A freeform comb uses a separate local-cell builder (grip shells).
    // Its hook is intentionally distinct from the analytic drum comb below:
    // the latter keeps the exact clipped primitive lattice and only removes
    // feature stations outside the groove corridor that owns them.
    bool orthogonalLocalComb = false;
    bool orthogonalDrumComb = false;
    // Number of repeated trim pieces that the primitive direction must be
    // able to represent locally. Used only as a geometry-derived density
    // floor; ordinary cylinders/cones leave it zero.
    int orthogonalFeatureCount = 0;
    // A Coons patch that terminates at a genuine analytic pole. Non-adaptive
    // defaults still need the revolution turn represented; these fields map
    // that angular floor onto the patch's semantic grid axis.
    bool coonsPolePatch = false;
    bool coonsPoleAroundIsU = true;
    double coonsPoleTurnFraction = 0.0;
    int orthogonalDriverU = 0;
    int orthogonalDriverV = 0;
    std::vector<int> orthogonalEdges;
    // A separately-proven single-wire trim corridor.  This is deliberately
    // not a relaxation of chained Coons: Coons has already declined the
    // face before this flag can be set.  Two monotone boundary rail chains
    // are matched by arc length; any count/shape mismatch is confined to the
    // two short end closures.  The public kind remains CoonsGrid so the
    // existing opposite-chain density machinery can align the rail samples,
    // while dispatch routes the geometry through the local corridor builder.
    bool trimCorridor = false;
    bool trimCorridorAxisU = true;
    int trimCorridorBands = 1;
    // A broad, single-wire freeform panel whose complete trimmed surface was
    // proven to lie inside a scale-aware chord-deviation slab. Every boundary
    // and dense in-face probe must satisfy the active absolute/relative export
    // tolerance plus CAD sewing noise before the exact border becomes one
    // game n-gon.
    bool deviationFlatPanel = false;
    // Separate from trimCorridor/Coons acceptance: a UV-monotone curved neck
    // transition with two proven longitudinal trim rails. The rails solve to
    // identical station totals and a small number of true-surface cross
    // sections; only bounded concave ends close as local n-gons. No existing
    // corridor predicate is relaxed to obtain this.
    bool sectionStrip = false;
    bool sectionStripAxisU = true;
    int sectionStripBands = 1;
};

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
    // Joint occupied by the stub, in side order.  Ordinary corner stubs sit
    // after side 3 (the lower-left grid corner); a pole-adjacent bookkeeping
    // sliver may instead sit after side 2 (the upper-left pole corner).
    int stubAfterSide = 3;
    bool stubRev = false;
    Handle(Geom2d_Curve) stubPc;
    double stubFirst = 0.0, stubLast = 0.0;
    // Interior trim wires (a hole through a curved top): meshed as a
    // grid CUTOUT — cells deleted, staircase webbed to the wire's exact
    // border sampling. Collected here so the planner can route them.
    std::vector<std::vector<int>> holeWires;
    std::vector<std::array<double, 4>> holeBoxes;  // u0,u1,v0,v1 per wire
    std::array<double, 4> outerBox{0, 1, 0, 1};    // outer wire's uv box
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

// --- Plate-web execution: 2D machinery -------------------------------------
// The plate is planar, so its UV space is an isometric chart — offsets and
// intersection tests run there and map straight back to 3D.

struct WebPoint {
    gp_Pnt2d uv;
    uint32_t vert;  // index in the MeshBuilder
};


// A planar face's wire sampled as one chained ring: each edge at its own
// solved count, positions on the 3D edge curve (weld-exact), UV from the
// pcurve (the plate's isometric chart).
struct PlanarRing {
    std::vector<gp_Pnt2d> uv;
    std::vector<gp_Pnt> p;
    bool isOuter = false;
};

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
    // Groups whose count the user set BY NAME (per-face override or
    // per-edge pin): floors and rim raises must not touch them —
    // 16 radial segments means exactly 16.
    std::set<int> pinnedRoots;

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

// Cross-module private entry points.
bool rimChains(const TopoDS_Face& face, const Model& model,
               const std::set<int>& insertIds, std::vector<int>& low,
               std::vector<int>& high);
bool edgesHugRimsOrInserts(const TopoDS_Face& face,
                           const BRepAdaptor_Surface& surf,
                           const Model& model,
                           std::vector<std::vector<int>>& wires,
                           const std::vector<int>* bandSides = nullptr,
                           bool repeatedNotchedSeam = false);
bool openBandSides(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                   const Model& model, std::vector<int>& sides,
                   bool* repeatedNotched = nullptr,
                   int* repeatedFeatureCount = nullptr);
bool planOrthogonalTrimGrid(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model, FacePlan& plan);
int castellatedRimBand(const TopoDS_Face& face,
                       const BRepAdaptor_Surface& surf, const Model& model,
                       const std::vector<int>& rimLow,
                       const std::vector<int>& rimHigh);
bool isClosedRevolution(const BRepAdaptor_Surface& surf);
bool isGeometricClosedRevolution(const BRepAdaptor_Surface& surf);
double ringAnchorAngle(const gp_Circ& circ);
double closedEdgePhase(const TopoDS_Edge& edge, const Model& model);
bool boundingCircle(const TopoDS_Face& face, gp_Circ& circOut, int& edgeIdOut,
                    const Model& model);
void collectIsoEdges(const TopoDS_Face& face, const Model& model,
                     const std::vector<int>& edgeIds, FacePlan& plan,
                     bool skipNonIso = false);
bool planRingJunction(const TopoDS_Face& face, const Model& model,
                      FacePlan& plan);
bool parametricGridFits(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        int nu, int nv);
bool revolutionCovers(const TopoDS_Face& face);
bool makeCoonsPatch(const TopoDS_Face& face, const Model& model,
                    CoonsPatch& patch, int rotate = 0,
                    const char** why = nullptr,
                    bool* reflexPlanar = nullptr);
void insertCountFloors(const TopoDS_Face& face, const CoonsPatch& patch,
                       int& minU, int& minV);
bool planAnnulus(const TopoDS_Face& face, const Model& model, FacePlan& plan,
                 bool requireRing);
bool meshAnnulusRing(const TopoDS_Face& face, const Model& model, int faceId,
                     const std::vector<int>& outerLoop,
                     const std::vector<int>& innerLoop,
                     const std::vector<int>& solvedEdge, int radialDefault,
                     MeshBuilder& out);
bool planAnnulusCRing(const TopoDS_Face& face, const Model& model,
                      FacePlan& plan);
bool meshAnnulusCRing(const TopoDS_Face& face, const Model& model, int faceId,
                      const std::vector<int>& outerEdges,
                      const std::vector<int>& innerEdges,
                      const std::vector<int>& wallEdges,
                      const std::vector<int>& solvedEdge, int radialDefault,
                      MeshBuilder& out, const PinnedEdges* pins);
bool isGeometricallyFlat(const TopoDS_Face& face,
                         const BRepAdaptor_Surface& surf, double flatFrac = 1e-3);
bool isShallowCapCone(const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf);
bool collectPlanarLoops(const TopoDS_Face& face,
                        const BRepAdaptor_Surface& surf, const Model& model,
                        FacePlan& plan, bool requirePlane = true,
                        bool tolerateDegenerate = false);
double wireElongation(const TopoDS_Wire& wire);
bool planPlateWeb(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, FacePlan& plan,
                  bool requireRoundHoles);
bool planMinimalPlanar(const TopoDS_Face& face,
                       const BRepAdaptor_Surface& surf, const Model& model,
                       FacePlan& plan, bool requirePlane = true);
bool triangulateWeb(std::vector<WebPoint> outer,
                    std::vector<std::vector<WebPoint>> holes, int faceId,
                    bool flip, MeshBuilder& out);
bool meshCoonsGrid(const TopoDS_Face& face, const Model& model, int faceId,
                   const std::vector<double>& uParams,
                   const std::vector<double>& vParams, int rotate,
                   const std::vector<int>& solvedEdge, MeshBuilder& out,
                   const std::vector<std::vector<int>>* inserts = nullptr,
                   int collarRings = 1,
                   const PinnedEdges* pins = nullptr, int cellCap = 0,
                   bool decoupleSeams = false);
bool samplePlanarRings(const TopoDS_Face& face, const Model& model,
                       const std::vector<int>& solvedEdge, int radialDefault,
                       std::vector<PlanarRing>& rings,
                       const PinnedEdges* pins = nullptr);
bool meshPlateWeb(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, int faceId,
                  const std::vector<int>& solvedEdge, int radialDefault,
                  int collarRings, bool squareCollar, MeshBuilder& out);
bool meshMinimalPlanar(const TopoDS_Face& face, const Model& model,
                       int faceId, const std::vector<int>& solvedEdge,
                       int radialDefault, MeshBuilder& out,
                       const PinnedEdges* pins = nullptr);
bool planRailLadder(const TopoDS_Face& face, const Model& model,
                    FacePlan& plan);
bool meshRailLadder(const TopoDS_Face& face, const Model& model, int faceId,
                    const std::vector<int>& solvedEdge, int radialDefault,
                    MeshBuilder& out, std::array<int, 2>* built = nullptr);
bool sampleRibbonRing(const TopoDS_Face& face, const Model& model,
                      const std::vector<int>* solvedEdge, int radialDefault,
                      std::vector<gp_Pnt>& P, std::vector<gp_Pnt2d>& UV,
                      std::vector<int>& corners,
                      const PinnedEdges* pins = nullptr,
                      std::vector<int>* sampleEdges = nullptr);
bool ribbonDetect(const TopoDS_Face& face, const Model& model);
bool ribbonEndNotchDetect(const TopoDS_Face& face, const Model& model);
bool meshRibbonSweep(const TopoDS_Face& face, const Model& model, int faceId,
                     const std::vector<int>& solvedEdge, int radialDefault,
                     MeshBuilder& out, std::array<int, 2>* built = nullptr,
                     const PinnedEdges* pins = nullptr);
bool planTrimCorridor(const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf, const Model& model,
                      const FaceMeshSettings& settings, FacePlan& plan);
bool planSectionStrip(const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf, const Model& model,
                      const FaceMeshSettings& settings, FacePlan& plan);
bool planDeviationFlatPanel(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model,
                            const FaceMeshSettings& settings,
                            FacePlan& plan);
bool meshTrimCorridor(const TopoDS_Face& face, const Model& model,
                      const FacePlan& plan, int faceId,
                      const std::vector<int>& solvedEdge, int radialDefault,
                      MeshBuilder& out, const PinnedEdges* pins,
                      std::array<int, 2>* built = nullptr);
bool meshSurfaceCapFan(const TopoDS_Face& face, const Model& model,
                       int faceId, const std::vector<int>& solvedEdge,
                       int radialDefault, MeshBuilder& out);
int orthogonalSurfaceDivisionFloor(const TopoDS_Face& face,
                                   const BRepAdaptor_Surface& surf,
                                   const FaceMeshSettings& s,
                                   bool alongU);
bool faceWithinDeflection(const TopoDS_Face& face,
                          const FaceMeshSettings& s);
bool meshContractFallback(const TopoDS_Face& face, const Model& model,
                          int faceId, const std::vector<int>& solvedEdge,
                          int radialDefault, MeshBuilder& out,
                          const FaceMeshSettings* refine = nullptr,
                          bool angleSplit = false,
                          const PinnedEdges* pins = nullptr);
bool planQuadFill(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, FacePlan& plan);
bool meshDiskCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const Model& model, int faceId,
                 const std::vector<int>& solvedEdge, int radialDefault,
                 MeshBuilder& out);
bool meshQuadFill(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, int faceId,
                  const std::vector<int>& solvedEdge, int radialDefault,
                  const FaceMeshSettings& fs, MeshBuilder& out,
                  int gridUOverride = 0, int gridVOverride = 0);
FacePlan planFace(int fid, const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings,
                  GenerationCache* cache);
void propagateBandRadialToBlendGroup(const Analysis& analysis,
                                     const std::map<int, FacePlan>& plans,
                                     GenerationSettings& settings);
DensitySolution solveDensity(const Model& model, std::map<int, FacePlan>& plans,
                             const GenerationSettings& settings,
                             GenerationCache* cache);
void pinCastellatedRims(const Model& model,
                        const std::map<int, FacePlan>& plans,
                        const GenerationSettings& settings,
                        const std::vector<int>& solvedEdge,
                        DensitySolution& density, PinnedEdges& pins);
void pinFilletChains(const Model& model,
                     const std::map<int, FacePlan>& plans,
                     std::vector<int>& solvedEdge, PinnedEdges& pins);
void meshRevolutionTaper(const TopoDS_Face& face,
                         const BRepAdaptor_Surface& surf, int faceId, int nA,
                         int nB, double phaseV0, double phaseV1,
                         MeshBuilder& out);
double revolutionUPhase(const BRepAdaptor_Surface& surf,
                        const Model& model, int rimEdgeId,
                        bool* atLastV = nullptr);
bool meshRevolutionOpenBand(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model,
                            const std::vector<int>& rimEdges,
                            const std::vector<int>& solvedEdge, int faceId,
                            int nu, int nv, const std::vector<int>& sides,
                            MeshBuilder& out,
                            const PinnedEdges* pins = nullptr,
                            const std::vector<std::vector<int>>*
                                insertWires = nullptr);
bool meshRevolutionRimNotch(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model,
                            const std::vector<int>& rimLow,
                            const std::vector<int>& rimHigh, int plainRimEdge,
                            const std::vector<int>& solvedEdge, int faceId,
                            int nu, int nv, MeshBuilder& out,
                            const PinnedEdges* pins = nullptr,
                            // Interior row LEVELS (surface v): the insert
                            // composition subdivides every column at these
                            // heights so slot rows exist for the carve.
                            // Only the pinned boolean-cut path supports
                            // them; the strip path bails so the caller's
                            // floor still catches the face.
                            const std::vector<double>* levelsOpt = nullptr);
bool meshDomeCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const Model& model, const FacePlan& plan,
                 const std::vector<int>& solvedEdge, int faceId, int nLat,
                 MeshBuilder& out, const PinnedEdges* pins);
void pinOrthogonalTrimGrids(const Model& model,
                            const std::map<int, FacePlan>& plans,
                            const GenerationSettings& settings,
                            const std::vector<int>& solvedEdge,
                            PinnedEdges& pins);
bool meshOrthogonalLocalComb(const TopoDS_Face& face,
                             const BRepAdaptor_Surface& surf,
                             const Model& model, const FacePlan& plan,
                             const std::vector<int>& solvedEdge, int faceId,
                             int nu, int nv, int cellCap, MeshBuilder& out,
                             const PinnedEdges* pins);
bool meshOrthogonalTrimGrid(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model, const FacePlan& plan,
                            const std::vector<int>& solvedEdge, int faceId,
                            int nu, int nv, MeshBuilder& out,
                            const PinnedEdges* pins);
bool meshRevolutionGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        const Model& model, const std::vector<int>& rimEdges,
                        const std::vector<int>& solvedEdge, int faceId,
                        int nu, int nv, MeshBuilder& out,
                        const std::vector<double>* vRowsOpt = nullptr,
                        const std::vector<int>* rimLowOpt = nullptr,
                        std::array<int, 2>* built = nullptr,
                        bool dedupeDriveRim = false,
                        bool decoupleSeams = false);
bool meshRevolutionInsert(const TopoDS_Face& face,
                          const BRepAdaptor_Surface& surf, const Model& model,
                          const FacePlan& plan,
                          const std::vector<int>& solvedEdge, int faceId,
                          int nu, int nv, MeshBuilder& out,
                          const PinnedEdges* pins = nullptr,
                          bool decoupleSeams = false);
void meshDiskCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const gp_Circ& circ, int faceId, int n, CapStyle cap,
                 MeshBuilder& out, double startAngle = 0.0);
void meshParametricGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        int faceId, const std::vector<double>& uParams,
                        const std::vector<double>& vParams, MeshBuilder& out);
bool meshRingLattice(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                     const Model& model, int faceId,
                     const std::vector<int>& solvedEdge,
                     const PinnedEdges* pins, MeshBuilder& out);
void meshRingJunction(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                      const gp_Circ& circ, int faceId, int nu, int nv,
                      int loops, MeshBuilder& out, double startAngle = 0.0);
void meshMinimalNGon(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                     int faceId, int nu, int nv, MeshBuilder& out);
double quadAngleCost(const std::array<gp_Pnt, 4>& q);
void pairPartTris(PolyMesh& part);
int applyOrthogonalEdgeContracts(
    PolyMesh& mesh, const Model& model,
    const std::map<int, FacePlan>& plans,
    const std::vector<int>& solvedEdge, const PinnedEdges& pins);
void meshFallback(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  int faceId, const FaceMeshSettings& s, MeshBuilder& out);
void conformFallbackBorders(PolyMesh& mesh, const Model& model,
                            const std::map<int, FacePlan>& plans,
                            const GenerationSettings& settings,
                            const std::vector<std::array<size_t, 2>>& range,
                            const std::vector<char>& fellBack);
void fuseSeamTwins(PolyMesh& mesh, const Model& model, double weldTol);
void stitchSeams(PolyMesh& mesh, const Model& model, double weldTol,
                 const std::map<int, FacePlan>& plans,
                 const GenerationSettings& settings,
                 DensitySolution& density);
void unionSeams(PolyMesh& mesh, const Model& model, double weldTol);
int outerWireSolvedTotal(const TopoDS_Face& face, const Model& model,
                         const std::vector<int>& solvedEdge,
                         int radialDefault);

}  // namespace weft::mesher_impl
