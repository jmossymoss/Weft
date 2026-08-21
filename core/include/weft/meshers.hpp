#pragma once

#include <atomic>
#include "weft/analysis.hpp"
#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <array>
#include <cstdio>
#include <map>
#include <memory>
#include <string>

namespace weft {

enum class CapStyle {
    NGon,  // single n-sided polygon
    Fan,   // center vertex + triangle fan
};

// Named, per-face density controls (the plan's §3.3). A face picks up the
// defaults unless an override is present for its FaceId.
struct FaceMeshSettings {
    int radial = 16;        // divisions around a surface of revolution (u)
    int axial = 1;          // divisions along the axis / pole-to-pole (v)
    int gridU = 1;          // planar/parametric grid divisions — start
    int gridV = 1;          // minimal (game topology), densify on demand
    CapStyle cap = CapStyle::NGon;
    // Freeform/trimmed faces (the fallback mesher) are driven by these two,
    // Plasticity-style: max chordal deviation from the true surface, and max
    // angle between adjacent facets. On an imported model most faces are
    // freeform, so these ARE the global density controls.
    double chordTolerance = 0.1;   // max deviation (model units)
    double angleToleranceDeg = 28.0;  // max facet turn angle (degrees)
    // Fillet/blend faces: divisions ACROSS the blend (support loops for
    // baking) and how strongly the loops cluster toward the creases
    // ("hold" loops; 0 = uniform spacing, toward 1 = tight at the edges).
    int filletLoops = 3;
    double filletHold = 0.0;
    // Hole-plate / ring-junction collar depth: concentric quad loops
    // between each hole rim and the plate web. 0 = no collar (default for
    // hole plates — the web meets the bore rim directly); raise to turn
    // collars on. RingJunction still floors at 1 internally.
    int junctionRings = 0;
    // Route flat plates (and reflex coons outlines) through the
    // structured quad-fill grid instead of minimal n-gons / transfinite
    // patches. Routing only — the fallback floor pairs its triangles
    // into quads regardless (see pureTriFloor).
    bool quadDominant = false;
    // The contract-floor fallback pairs its interior triangles into
    // quads by default (greedy on corner-angle quality; borders are
    // exact contract samples either way — pairing merges interior
    // diagonals only). Set to keep the raw triangulation instead; full
    // triangle output for engines is the export-time triangulate flag.
    bool pureTriFloor = false;
    // Game-topology minimalism (plan §1/§4.1): a flat face doesn't need an
    // interior grid. When set, a planar grid-safe face emits one boundary
    // n-gon instead — border vertices stay density-matched, so neighbours
    // still weld watertight, and the engine triangulates however it likes.
    // Default ON: quad flow is spent where geometry curves or deforms
    // (fillets, cylinders, freeform); large flat faces ship as boundary
    // n-gons / hole-bridged webs and can be triangulated at export.
    // --flat-quads (CLI) or minimal=0 (recipe) restores dense flats.
    bool minimal = true;
    // Exclude this face from the output entirely (delete it). Its borders
    // become open boundary loops, which a Bridge op can then reconnect —
    // e.g. drop a bore wall and bridge the two rims shut.
    bool exclude = false;
    // Force a specific mesher instead of the automatic pick (0 = auto,
    // else 1 + MesherKind value). If the forced strategy can't build on
    // the face it falls back to triangulation, so the choice is visible.
    int forceMesher = 0;
    // Revolution bands: keep the two rims at the same count (a quad band)
    // or let them differ — the band then meshes as a triangulated taper
    // between the rims (pin each rim's count per-edge / in the UI).
    bool linkRims = true;
    // Freeform fallback extras: minimum element size (0 = no floor) and
    // deviation measured relative to face size instead of absolute.
    double minSize = 0.0;
    bool relativeDeviation = false;
    // Per-face weld tolerance override in mm (0 = inherit the global
    // GenerationSettings::weldTolerance). Governs how loosely THIS face's
    // boundary edges weld onto their neighbours: a shared edge welds at
    // max(faceA.weldTolerance, faceB.weldTolerance, global), so loosening
    // one side closes that junction. Clamped to the local feature size at
    // use so a value larger than nearby detail can't collapse real
    // geometry (see conformFallbackBorders).
    double weldTolerance = 0.0;
    // Plate-web collars: square borders instead of hole-shaped rings —
    // the classic game pattern (round hole -> square collar -> plate).
    bool squareCollar = false;
    // Coons patch rotation (0-3): shifts which wire edge becomes side 0,
    // picking the corner the grid anchors to — on triangular patches
    // this chooses the corner the fan terminates in.
    int coonsRotate = 0;
    // Plate-web / quad-fill boundary control: total vertex count around
    // the face's OUTER loop, distributed across its edges by arc length
    // and pinned (drives the neighbouring walls' shared edges too).
    // 0 = automatic (radial share / adaptive curvature).
    int boundary = 0;
    // Curvature-adaptive density: analytic meshers derive each border
    // edge's count from the chord/angle tolerances (tangential-deflection
    // sampling), so a big housing gets more segments than a bolt hole
    // instead of both taking one flat radial value. The manual counts act
    // as floors; per-edge pins and manual per-face overrides still win.
    // Off by default in the core (recipes/tests keep exact counts) — the
    // app turns it on for new sessions.
    bool adaptive = false;
    // Lower floor on adaptive counts for closed curved RINGS (cylinder /
    // sphere / torus / fillet circles, annulus bores, …), as a TOTAL around
    // the ring. Plasticity/STEP often splits one circle into open arcs —
    // those arcs still share this floor by span (two semicircles at 12 →
    // 6+6). Ring-junction plates grow their boundary so 2*(nu+nv) meets
    // this floor instead of crushing the bore back to a hexagon. Default 6
    // matches the historic hard floor; raise it (e.g. 12) when CAD/
    // relative-deviation's 60° gate would otherwise leave rings faceted.
    // Straight edges are unaffected. Recipe/CLI: mincurve / --min-curve.
    // Closed curved rings (cylinder/sphere/torus circles): artist floor.
    // 24 is the minimum readable circumferential span for game cylinders
    // at CAD defaults; smaller rings still share this floor so bores do
    // not read as hex prisms next to denser barrels.
    int minCurvedSegments = 24;
    // Per-face pathology guard: a hard ceiling on this face's total cell
    // count (0 = no ceiling). A face's mesh should scale with its surface
    // area; a face carrying vastly more cells than its area-share of the
    // model is a sizing pathology (an offset surface's curvature probe
    // reporting a tiny local radius, a hole-cutout scaffold doubling to
    // separate close bores) rather than real detail. generate() fills this
    // per face from area vs. the model, scaled by the density dial, so the
    // guard only ever bites gross outliers and rides --density like every
    // other count. It is NOT a fixed cap — a legitimately large face gets a
    // proportionally large ceiling.
    int cellCap = 0;
};

struct GenerationSettings {
    FaceMeshSettings defaults;
    std::map<int, FaceMeshSettings> perFace;  // FaceId -> overrides
    // EdgeId -> exact subdivision count. Wins over face proposals for the
    // whole shared-edge group it belongs to (plan §5 per_edge_settings).
    std::map<int, int> perEdge;
    double weldTolerance = 1e-6;
    // One knob for the whole model's budget: every density proposal
    // (flat counts and curvature-adaptive ones) scales by this before
    // the group solve. Explicit per-edge pins and boundary totals are
    // the user's exact numbers and stay untouched.
    double densityScale = 1.0;
    // Runtime/debug knobs (not persisted in recipes): turn off worker
    // threads or the freeform border-conformity pass to bisect problems.
    bool parallelMeshing = true;
    bool conformBorders = true;
    // Interactive preview mode can defer expensive whole-model repair.
    // False still performs the lightweight weld needed by mesh editing, but
    // skips fallback-border conformation, seam stitching, and final cleanup.
    // Export/CLI generation leaves this true for the authoritative mesh.
    bool finalizeMesh = true;
    // Independent tessellation (this fork). When true, CLI mesh/validate
    // call meshIndependent() instead of generate(). Not persisted in recipes.
    bool independentMesh = false;
    // EXPERIMENT (decoupled seams): skip the global count-equalization
    // repairs (chained-coons sum repair, revolution rim SUM constraint)
    // and let the post-weld unionSeams splice reconcile mismatched
    // borders by inserting the denser side's verts into the coarser
    // polygons (quad -> n-gon at the seam). Off by default; the CLI's
    // --stitch flag turns it on for corpus experiments.
    bool decoupleSeams = false;
    // Live progress for UIs: incremented once per meshed face when set
    // (non-owning; the pointee must outlive the generate call).
    std::atomic<int>* progressFaces = nullptr;
    // Updated after planning/density/cache lookup to the number of cache
    // misses that actually require a mesher. UIs can distinguish a local
    // remesh from cached faces participating in the final seam/weld pass.
    std::atomic<int>* progressTotal = nullptr;

    const FaceMeshSettings& forFace(int faceId) const {
        auto it = perFace.find(faceId);
        return it == perFace.end() ? defaults : it->second;
    }
};

// Which strategy generate() picked for each face. GenerationReport::faceBuild
// separately records whether that plan ultimately needed an emergency raw
// OCCT triangulation.
enum class MesherKind {
    RevolutionGrid,  // closed-u cylinder/cone/sphere/torus: quad grid with
                     // wrap-around seams and collapsed apex/pole rows
    DiskCap,         // planar face bounded by one full circle
    PlanarGrid,      // planar/parametric UV grid that passed containment
    CoonsGrid,       // four-sided (trimmed/freeform) face: structured quad
                     // grid blended between its four boundary pcurves —
                     // clean flow on bspline strips instead of triangles
    RingJunction,    // rectangular planar face with one circular hole:
                     // concentric quad rings from the circle to the border
                     // (the cylinder-to-plane junction pattern, plan §3.3/4.2)
    QuadDominant,    // border-exact contract web + guided tri-pairing into
                     // quads (plan §3.5 seed; a cross-field solver slots in
                     // here later)
    MinimalNGon,     // planar face as a single boundary n-gon (flat panels
                     // don't need interior topology for game meshes)
    Fallback,        // border-exact contract floor for faces no structured
                     // family can express. Raw OCCT triangulation is only its
                     // last-resort failure path and is reported as faceBuild=1.
    AnnulusRing,     // face bounded by exactly two closed loops (the flat
                     // ring between two revolution rims): one zippered
                     // band — equal counts give pure quads
    PlateWeb,        // planar face with any number of hole loops (bolt-hole
                     // plates): a quad collar around every hole + an
                     // ear-clipped web tying collars to the outer boundary.
                     // All borders sample the B-rep edge curves at solved
                     // counts, so every neighbour welds watertight.
    QuadFill,        // planar face of any shape: an interior quad grid
                     // sized from the border density, joined to the exact
                     // B-rep boundary by a thin triangulated rim — clean
                     // quad flow on plates instead of fan triangulations.
                     // Auto when quad-dominant is set; always forcible.
    RailLadder,      // two-tip band (crescent/lune/tangent strip — the
                     // outline has exactly two sharp corners): the two
                     // rails pair by arc fraction and ladder into quads
                     // with grouped 5-gons; the tips collapse naturally.
    RibbonSweep,     // long thin BENT strip (grip/trigger-guard rails) coons
                     // rejects: its two long rails are found robustly (not by
                     // corner turns), matched station-by-station, and laddered
                     // into an even quad flow, with the end caps (incl. notches)
                     // webbed locally instead of a global transfinite blend.
    DomeCap,         // spherical / dome cap: a single-outer-wire revolution-
                     // like surface (often a bspline) that bulges from a base
                     // loop to a single pole. Meshes as a UV-sphere hemisphere
                     // — concentric latitude rings + straight meridians that
                     // converge to a pole fan — instead of a spiralling Coons
                     // grid. Works on squashed/ellipsoidal domes: the rings and
                     // meridians follow the true surface, they just aren't
                     // perfect circles.
};

const char* mesherKindName(MesherKind k);

// Why a face did or did not keep the structured topology its plan chose.
//
// `GenerationReport::faceBuild` says only 0/1/2/-1 and the cause is free text,
// so failure classes could not be counted or ranked and fixes got picked by
// whichever face an artist happened to point at. This is the countable view:
// classifyFaceBuild() maps (build, cause) through one central table, and an
// unregistered cause resolves to Unknown rather than being folded silently
// into a healthy bucket.
enum class FaceBuildClass {
    Built,           // the planned mesher built the face
    PlannedFloor,    // routed to the contract floor by design, not by failure
    MesherFailed,    // a structured mesher gave up and the floor caught it
    BorderContract,  // the build violated a shared-border contract
    FoldHeal,        // the floor was taken to clear folded/inverted cells
    SelfCheck,       // the face's own post-build self-check rejected it
    DensityOverride, // demoted because of a per-face density edit
    Raw,             // raw OCCT triangulation (the tri-soup last resort)
    Empty,           // the face emitted no polygons
    Unknown,         // cause string not registered — treated as debt
};

const char* faceBuildClassName(FaceBuildClass c);

// build: GenerationReport::faceBuild value (0 planned, 1 raw, 2 floor,
// -1 empty). cause: the matching GenerationReport::faceBuildCause entry.
FaceBuildClass classifyFaceBuild(int build, const std::string& cause);

// Structure retention for one generate() run. `failedFloor` is the artist-
// visible debt: a face whose planned structure was lost to a failure rather
// than to a deliberate routing decision.
struct StructureSummary {
    int total = 0;
    int structured = 0;
    int plannedFloor = 0;
    int failedFloor = 0;
    int raw = 0;
    int empty = 0;
    std::map<FaceBuildClass, int> byClass;
    // Failed-floor faces grouped by their exact cause string, so the biggest
    // class can be attacked first instead of the newest complaint.
    std::map<std::string, int> failedByCause;
    // Planned-floor faces grouped the same way. A planned floor is a routing
    // gap rather than debt, but the gaps still need ranking: planFace
    // records which ladder stage exhausted, so the census says which class
    // of trim the ladder cannot express yet.
    std::map<std::string, int> plannedByCause;

    double retention() const {
        return total ? double(structured) / double(total) : 1.0;
    }
};

// n+1 monotonically increasing parameters in [0,1] splitting it into n
// intervals. hold=0 is uniform; hold in (0,1) squeezes the intervals toward
// both ends, which is how fillet support loops hug the creases.
std::vector<double> clusteredParams(int divisions, double hold);

struct GenerationReport {
    // Per-face generation cache accounting for this run. A local edit may
    // legitimately miss adjacent faces when shared border counts changed.
    int cacheHits = 0;
    int cacheMisses = 0;
    std::vector<int> reusedFaces;
    std::vector<int> remeshedFaces;
    std::map<int, MesherKind> faceMesher;  // FaceId -> strategy used
    // AD-5 analyze facts echoed into the report so signatures / CLI can
    // show featureClass × chartKind without re-running probes.
    std::map<int, FeatureClass> faceFeatureClass;
    std::map<int, ChartKind> faceChartKind;
    // FaceId -> how the face was actually built: 0 = its planned mesher,
    // 1 = raw OCCT triangulation (the tri-soup last resort), 2 = the
    // contract floor (exact borders, quad-paired web), -1 = the face
    // emitted no polygons at all (a hole in the output). Excluded faces
    // are not listed. faceMesher alone can't tell a contract-floor
    // demotion from a clean build (the floor keeps the planned kind so
    // conform treats its exact borders as authority) — the
    // never-fall-back gate reads this map instead: 1 and -1
    // are failures, 2 is a visible graceful floor.
    std::map<int, int> faceBuild;
    // FaceId -> short cause string for faceBuild values other than 0
    // (raw, empty, or contract-floor). Absent for clean planned builds.
    // CLI/validate print this next to face ids so unsupported cases are
    // explicit rather than silent dbg-only notes.
    std::map<int, std::string> faceBuildCause;
    // EdgeId -> solved subdivision count, for edges that took part in
    // density matching. Adjacent faces sharing an edge agree on this count.
    std::map<int, int> edgeDivisions;
    // EdgeId -> how the solved count was chosen for that edge's density
    // group. Short stable tags for CLI/validate attribution:
    // "sole-proposal", "max-proposal", "face-pin", "edge-pin",
    // "ring-derived", "curvature-floor", "wire-floor", "annulus-floor",
    // "rail-align".
    // Present for the same edges as edgeDivisions when attribution ran.
    std::map<int, std::string> edgeDivisionOwner;
    // Shared-group ownership conflicts: proposing faces disagreed, or a
    // pin/floor raised the count above a face's proposal. Empty when every
    // proposing face agreed and no post-solve floor raised the group.
    // Does not change mesh topology — reporting only.
    struct DensityConflict {
        int edgeId = 0;     // representative edge from the density group
        int solved = 0;     // final subdivision count after floors
        std::string reason; // same tags as edgeDivisionOwner
        // FaceId -> proposed count (0 = non-face source such as a pin).
        std::map<int, int> faceProposals;
    };
    std::vector<DensityConflict> densityConflicts;
    // Revolution faces: the edge ids of their two rims (u-boundary rings),
    // so UIs can pin each rim's count individually when rims are unlinked.
    std::map<int, std::array<int, 2>> faceRims;
    // FaceId -> the solved primary/secondary counts the mesher actually
    // used ({nu, nv} = radial/axial, gridU/gridV, etc.). A UI leaving
    // adaptive seeds its manual fields from these so the count starts at
    // the value the face was already meshed at (no dead zone before a
    // manual count exceeds the adaptive floor).
    std::map<int, std::array<int, 2>> faceCounts;
    // Blend strips (fillet Coons/planar grids): which patch axis the
    // fillet-loops knob (the across-the-blend count) drives — 1 = the
    // u/gridU axis, 2 = the v/gridV axis. On these faces "grid u" is
    // always the ALONG count and "grid v" is inert (the solve remaps
    // semantically), so UIs should present loops/along, not raw u/v.
    // Absent for faces without across semantics.
    std::map<int, int> faceAcross;
    // FaceId -> the mesher's own trace lines for faces that did NOT build
    // cleanly. A bail returns false through one of hundreds of internal
    // guards and the cause string names only the outermost failure, so this
    // is the evidence that used to require adding temporary dbg() calls and
    // rebuilding the core. Populated only for faceBuild != 0 (and only when
    // capture is on, which generate() enables), so healthy runs pay nothing.
    std::map<int, std::vector<std::string>> faceTrace;
};

// Human-readable demotion attribution for CLI/validate: counts plus
// per-face id and cause for contract-floor, raw OCCT, and empty faces.
std::string formatBuildDemotions(const GenerationReport& report);

StructureSummary summarizeStructure(const GenerationReport& report);

// The mesher's own trace for demoted faces (GenerationReport::faceTrace).
// faceId > 0 selects one face; 0 formats every traced face.
std::string formatFaceTrace(const GenerationReport& report, int faceId = 0);

// One-line machine-greppable summary for CLI/gate consumption:
//   structure: faces=N structured=N planned-floor=N failed-floor=N raw=N
//              empty=N retention=0.983
std::string formatStructure(const GenerationReport& report);

// Human-readable density-matching attribution for CLI/validate: matched
// edge counts, ownership tags, and any proposal/pin/floor conflicts.
std::string formatDensityOwnership(const GenerationReport& report);

// Per-face mesh reuse across generate() calls: pass the same cache and
// only faces whose settings, solved counts, or plan changed re-mesh —
// a topology-only face edit re-meshes that face, while a density edit also
// re-meshes the minimum shared-border constraint group needed to avoid cracks.
// The merge/conform/weld stages still run over the assembled result.
struct GenerationCache {
    struct CachedFace {
        std::string key;
        PolyMesh part;
        // The planned mesher couldn't build: 1 = raw OCCT fallback
        // (borders freeform, conform must move them), 2 = contract
        // floor (borders exact at solved counts — conform must leave
        // them alone). 0 = the planned mesher built.
        char fellBack = 0;
        // The primary/secondary counts this face actually built at, for
        // meshers whose built count differs from the pre-mesh `counts`
        // table (annulus body, boundary/rail meshers). {-1,-1} = fall back
        // to the table. Stored so a cache HIT reports the same count a
        // fresh mesh would, not the sparse/zero placeholder.
        std::array<int, 2> builtCounts = {-1, -1};
        // Cause string for fellBack != 0 (and empty-face leftovers). Kept
        // so a cache hit re-emits the same faceBuildCause a fresh mesh
        // would have written into GenerationReport.
        std::string buildCause;
        // Border-contract oracle passed: conform treats this part as an
        // exact-border authority. Must round-trip with the cache or a
        // full hit rebuilds with every face freeform and can drop a
        // weld-degenerate cell the cold path kept (teleporter +1 poly).
        char borderExact = 0;
    };
    std::map<int, CachedFace> faces;
    // Geometry-only memos (settings-independent, per model): results of
    // the point-classifier probes planning runs on every face.
    std::map<int, bool> revolutionCovers;
    std::map<int, bool> geomRevolution;
    std::map<int, bool> coonsValid;
    // Reject reason for the faces coonsValid memoized as false, so a replan
    // that skips patch construction still explains a contract floor.
    std::map<int, std::string> coonsReject;
    // Flat faces whose coons outline has a strong reflex bend (chevron
    // plates): geometry-only, planning may prefer quad-fill for them.
    std::map<int, bool> coonsReflex;
    // Geometry-only surface areas used by the per-face cell pathology budget.
    // BRepGProp::SurfaceProperties over thousands of faces is far too costly
    // to repeat after every interactive settings edit.
    std::map<int, double> faceAreas;
    std::map<int, double> faceOuterPerimeters;
    std::map<int, double> edgeLengths;
    double modelArea = -1.0;
    // Geometry/tolerance memos used by the global density solve.
    std::map<std::array<long long, 5>, int> adaptiveEdgeCounts;
    std::map<int, int> curvatureFloors;
    double modelDiagonal = -1.0;
    // Type-erased internal cache of geometry classification plans. FacePlan
    // stays private to meshers.cpp so OCCT planning details do not leak into
    // the public API.
    std::shared_ptr<void> facePlans;
    std::shared_ptr<void> cornerRepair;
    // Digon chord floor is topology/micro-edge geometry — independent of
    // per-face density edits. Compute once per model, re-apply on warm runs.
    bool digonsDirty = true;
    std::vector<std::pair<int, int>> digonEdgeFloors;  // (edgeId, minCount)
    // IsoBand drum-scale floor depends on model diag + defaults chord tol +
    // band wrap — not on per-face radial. Cache raises; invalidate on clear
    // or when defaults.chordTolerance changes.
    bool drumScalesDirty = true;
    double drumScalesChordTol = -1.0;
    std::vector<std::pair<int, int>> drumScaleEdgeFloors;  // (edgeId, minCount)
    void clear() {
        faces.clear();
        revolutionCovers.clear();
        geomRevolution.clear();
        coonsValid.clear();
        coonsReject.clear();
        coonsReflex.clear();
        faceAreas.clear();
        faceOuterPerimeters.clear();
        edgeLengths.clear();
        modelArea = -1.0;
        adaptiveEdgeCounts.clear();
        curvatureFloors.clear();
        modelDiagonal = -1.0;
        facePlans.reset();
        cornerRepair.reset();
        digonsDirty = true;
        digonEdgeFloors.clear();
        drumScalesDirty = true;
        drumScalesChordTol = -1.0;
        drumScaleEdgeFloors.clear();
    }
};

// Route the generator's stage-by-stage debug trace (plans, density solve,
// each face meshed, conformity per edge, weld) to a stream; null disables.
// Lines are flushed as written so a crash log ends at the crash site.
void setGenerateDebugLog(std::FILE* f);

// Lightweight generation profiler: set WEFT_TIMINGS=1 or WEFT_PROFILE=1 to
// print scoped phase milliseconds (stderr) for generate(). See weft/profile.hpp.

// Generate topology for every face of the model, per-face controllable.
//
// Density matching (plan §3.3, first increment): before meshing, edge
// subdivision counts are solved as shared constraints — edges that a
// parametric mesher requires to be equal (opposite sides of a grid, the
// rings of one revolution face) are grouped, each face proposes its own
// settings onto its edges, and every group resolves to the max proposal
// (or an explicit perEdge override). Meshers then honor the solved counts,
// so neighbouring parametric faces meet vertex-for-vertex.
//
// Every polygon carries its source FaceId; vertices are welded across faces.
PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings,
                  GenerationReport* report = nullptr,
                  GenerationCache* cache = nullptr);

// Per-cell fold detector: flags polygons whose winding opposes the B-rep
// surface normal at their own anchors. Grid cells that overlap a
// neighbour (skewed chained bands, collapsed borders) flip exactly there,
// so the app can paint them as problems. One byte per polygon: 1 = folded.
// Polygons without surface anchors (manual ops, plain fallback) are never
// flagged.
std::vector<uint8_t> foldedPolys(const Model& model, const PolyMesh& mesh);

}  // namespace weft
