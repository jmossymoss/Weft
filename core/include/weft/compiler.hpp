#pragma once

#include "weft/analysis.hpp"
#include "weft/meshers.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <vector>

namespace weft {

// Experimental primitive-aware compiler front-end.  Unlike the legacy
// generator, this representation makes the B-rep/coedge ownership and the
// shared edge discretisation explicit before any face mesher runs.

enum class CurveType {
    Line,
    Circle,
    Ellipse,
    Hyperbola,
    Parabola,
    Bezier,
    BSpline,
    Offset,
    Other,
};

enum class SemanticEdgeType {
    Boundary,
    SharpConvex,
    SharpConcave,
    Smooth,
    SmoothFaceSplit,
    FilletRail,
    PeriodicSeam,
    Degenerate,
};

enum class RegionType {
    Planar,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Revolution,
    Extrusion,
    Fillet,
    Freeform,
};

enum class CountConstraintType {
    StructuredOppositeSides,
    FilletOppositeSides,
};

enum class PatchKind {
    PlanarNGon,
    StructuredSurface,
    FilletRibbon,
    TrimmedRevolution,
    TriangleFallback,
};

enum class PatchFallbackReason {
    None,
    UnsupportedSurface,
    MultipleTrimLoops,
    NonFourSidedTrim,
    OppositeCountMismatch,
};

struct SurfaceDescriptor {
    SurfaceType type = SurfaceType::Other;
    std::array<double, 3> origin{};
    std::array<double, 3> axis{0.0, 0.0, 1.0};
    double radius = 0.0;
    double secondaryRadius = 0.0;
    double semiAngle = 0.0;
    bool uPeriodic = false;
    bool vPeriodic = false;
};

struct BrepVertexNode {
    int id = 0;
    std::array<double, 3> position{};
    double tolerance = 0.0;
    // B-rep vertices occupy the first canonical sample-id range.  Edge
    // endpoints reuse these ids, so separate edges meeting at one CAD vertex
    // cannot drift apart.
    std::uint64_t sampleId = 0;
};

struct BrepFaceNode {
    int id = 0;
    SurfaceDescriptor surface;
    double tolerance = 0.0;
    bool reversed = false;
    bool isFillet = false;
    // Ordered coedge ids per wire; wire 0 is the outer wire when OCCT can
    // identify one.  Every coedge is face-specific even when its 3D edge is
    // shared with another face.
    std::vector<std::vector<int>> wires;
};

struct BrepEdgeNode {
    int id = 0;
    CurveType curve = CurveType::Other;
    SemanticEdgeType semantic = SemanticEdgeType::Boundary;
    int firstVertex = 0;
    int lastVertex = 0;
    double tolerance = 0.0;
    double length = 0.0;
    bool closed = false;
    bool degenerate = false;
    std::vector<int> coedges;
};

struct BrepCoedgeNode {
    int id = 0;
    int edgeId = 0;
    int faceId = 0;
    int wireIndex = 0;
    bool reversed = false;
    // Filled by the canonical edge planning stage.  Both arrays are in the
    // face-wire traversal direction.  sampleIds refer to the one shared 3D
    // edge sample sequence; uv is face-specific.
    std::vector<std::uint64_t> sampleIds;
    std::vector<std::array<double, 2>> uv;
};

struct BrepGraph {
    std::vector<BrepVertexNode> vertices;  // index = id - 1
    std::vector<BrepFaceNode> faces;       // index = id - 1
    std::vector<BrepEdgeNode> edges;       // index = id - 1
    std::vector<BrepCoedgeNode> coedges;   // index = id - 1
};

struct CanonicalEdgeSample {
    std::uint64_t id = 0;
    double curveParameter = 0.0;
    // Normalized distance along the canonical FORWARD edge.  Structured
    // patches pair opposite rails by this value instead of assuming that two
    // unrelated curve parameterizations advance at the same physical rate.
    double normalizedAbscissa = 0.0;
    std::array<double, 3> position{};
    bool valid = false;
};

struct CanonicalEdgePlan {
    int edgeId = 0;
    int idealSegmentCount = 1;
    int segmentCount = 1;
    bool closed = false;
    bool uniformAbscissa = false;
    std::vector<CanonicalEdgeSample> samples;
};

struct CountConstraint {
    CountConstraintType type = CountConstraintType::StructuredOppositeSides;
    int faceId = 0;
    std::vector<int> edgeIds;
};

struct SemanticRegion {
    int id = 0;
    RegionType type = RegionType::Freeform;
    std::vector<int> faceIds;
    std::vector<int> boundaryEdgeIds;
};

struct PatchPlan {
    int faceId = 0;
    PatchKind kind = PatchKind::TriangleFallback;
    PatchFallbackReason fallbackReason =
        PatchFallbackReason::UnsupportedSurface;
    // Four ordered outer-wire coedges for structured patches.  The first and
    // third carry u; the second and fourth carry v.
    std::array<int, 4> sides{};
    // Two monotone trim chains spanning one periodic turn.  Seam coedges are
    // intentionally excluded; they are a chart cut, not a geometric border.
    std::array<std::vector<int>, 2> rims;
    int uSegments = 0;
    int vSegments = 0;
    // 0 means sides 0/2 are the longitudinal rails; 1 means sides 1/3.
    // Only meaningful for FilletRibbon, and derived from physical edge
    // lengths rather than wire order.
    int longitudinalAxis = 0;
};

struct CompilerSettings {
    double chordTolerance = 0.1;
    double angleToleranceDeg = 28.0;
    int radialSegments = 16;
    int axialSegments = 4;
    int filletAcrossSegments = 3;
    int minimumClosedCurveSegments = 6;
    int maximumEdgeSegments = 256;
    std::map<int, int> perEdge;
};

struct CompilerPlan {
    BrepGraph graph;
    std::vector<CanonicalEdgePlan> edgePlans;  // index = EdgeId - 1
    std::vector<CountConstraint> countConstraints;
    std::vector<SemanticRegion> regions;
    std::vector<PatchPlan> patches;  // index = FaceId - 1
    // Filled by generatePrimitiveAware; empty for a planning-only call.
    std::vector<int> nativeFaceIds;
};

// Build the explicit graph and solve one immutable sample sequence for every
// B-rep edge.  No face mesh is generated by this step.
CompilerPlan planPrimitiveAware(const Model& model, const Analysis& analysis,
                                const CompilerSettings& settings = {});

// Runnable transition path for the architecture branch.  The compiler owns
// global edge counts and sample placement; the existing primitive face
// builders are temporarily used behind that contract.  This lets individual
// primitive backends be replaced without changing the planner or CLI again.
PolyMesh generatePrimitiveAware(const Model& model, const Analysis& analysis,
                                const GenerationSettings& settings,
                                CompilerPlan* compilerPlan = nullptr,
                                GenerationReport* generationReport = nullptr,
                                GenerationCache* cache = nullptr);

// Native entry point for applications built around the compiler.  Compiler
// density/topology controls are deliberately separate from the legacy face
// mesher settings; GenerationSettings is retained only for unsupported-face
// containment, export finalization, and welding/manual-edit compatibility.
PolyMesh generatePrimitiveAware(const Model& model, const Analysis& analysis,
                                const CompilerSettings& compilerSettings,
                                const GenerationSettings& fallbackSettings,
                                CompilerPlan* compilerPlan = nullptr,
                                GenerationReport* generationReport = nullptr,
                                GenerationCache* cache = nullptr);

const char* curveTypeName(CurveType type);
const char* semanticEdgeTypeName(SemanticEdgeType type);
const char* regionTypeName(RegionType type);
const char* patchKindName(PatchKind kind);
const char* patchFallbackReasonName(PatchFallbackReason reason);

}  // namespace weft
