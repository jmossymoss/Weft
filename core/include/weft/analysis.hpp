#pragma once

#include "weft/model.hpp"

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
};

// Classify every face and edge and build the face-adjacency graph.
Analysis analyze(const Model& model);

}  // namespace weft
