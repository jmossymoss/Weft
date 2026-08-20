#pragma once

#include "weft/model.hpp"
#include "weft/topology_cache.hpp"

#include <string>
#include <vector>

namespace weft {

enum class SurfaceType {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Revolution,
    Extrusion,
    BSpline,
    Bezier,
    Offset,
    Other,
};

const char* surfaceTypeName(SurfaceType t);

enum class EdgeConvexity {
    Convex,
    Concave,
    Smooth,    // tangent join, dihedral ~ 0
    Boundary,  // borders fewer than two faces
};

const char* edgeConvexityName(EdgeConvexity c);

// UV / trim chart kind (AD-5). Set by analyze(); planFace must not rediscover.
enum class ChartKind {
    FreeTrim = 0,     // arbitrary trim; no structured chart claim
    Pole,             // degenerate or collapsing polar iso (sphere dimple)
    FullPeriod,       // wraps a full U (or equivalent) period
    IsoBand,          // partial drum/band with iso-ish rims
    GeometricCap,     // sphere/disk cap whose rim is not a UV pole chart
};

const char* chartKindName(ChartKind k);

// Feature class for priority routing (AD-5). planFace maps
// featureClass × chartKind → existing MesherKind.
enum class FeatureClass {
    Freeform = 0,
    Drum,           // cylinder / cone / revolution wall (incl. hole walls)
    SphereCap,      // sphere patch (pole chart or geometric cap)
    FilletStrip,    // constant-radius blend band
    HolePlate,      // planar face with inner wires (holes / webs)
    BossJunction,   // planar ring-junction style boss web
    PlanarPanel,    // simple planar panel
};

const char* featureClassName(FeatureClass c);

// Loop / wire bookkeeping filled by analyze().
struct LoopSignature {
    int wireCount = 0;
    int realEdgeCount = 0;   // non-degenerate edges on the face
    int degEdgeCount = 0;
    // Max/min sample radius ratio of the outer wire (~1 = round).
    double outerRoundness = 1.0;
};

struct FaceInfo {
    int id = 0;
    SurfaceType type = SurfaceType::Other;
    double radius = 0.0;  // cylinder/cone/sphere/torus major radius, else 0
    // Constant-radius blend detection: a cylindrical/toroidal strip whose
    // boundary joins at least two neighbours tangentially. These are the
    // faces that get support loops across their width (plan §3.3).
    bool isFillet = false;
    // A bore wall: closed cylindrical face whose material normal points
    // toward the axis. Bosses/shafts point away and stay false.
    bool isHole = false;
    ChartKind chartKind = ChartKind::FreeTrim;
    FeatureClass featureClass = FeatureClass::Freeform;
    // Product priority: cylinder → sphere → hemisphere → box → torus →
    // curves → cuts. Higher wins when neighbors disagree.
    int priority = 0;
    LoopSignature loop;
    std::vector<int> edgeIds;
    std::vector<int> neighborFaceIds;  // via shared edges (adjacency graph)
};

struct EdgeInfo {
    int id = 0;
    EdgeConvexity convexity = EdgeConvexity::Boundary;
    double dihedralDeg = 0.0;  // angle between face normals at edge midpoint
    double length = 0.0;       // arc length (multi-edge loop distribution)
    std::vector<int> faceIds;
};

struct Analysis {
    std::vector<FaceInfo> faces;  // index = FaceId - 1
    std::vector<EdgeInfo> edges;  // index = EdgeId - 1
    // Object structure: face ids grouped per solid (or per shell/compound
    // part when the file has no solids). One entry per object, in
    // traversal order — what an outliner lists.
    std::vector<std::vector<int>> solidFaces;
    // Flat CSR adjacency built once at analyze() — generate() hot path
    // reads this instead of re-walking TopoDS with TopExp_Explorer.
    // Dirty tags / edgeSegments are updated during interactive generates
    // (mutable: Analysis is otherwise treated as immutable geometry).
    mutable TopologyCache topology;
};

// Classify every face and edge and build the face-adjacency graph.
// Also fills chartKind / featureClass / priority / loop (AD-5).
Analysis analyze(const Model& model);

}  // namespace weft
