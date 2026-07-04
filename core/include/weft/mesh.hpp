#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace weft {

// Where a mesh vertex lives on the B-rep: surface-parametric coordinates on
// a face. This is what keeps editing surface-constrained — moves re-project
// through the anchor, and manual ops recorded against (faceId,u,v) survive
// re-tessellation (plan §3.4/§5).
struct Anchor {
    int faceId = 0;  // 0 = unanchored (e.g. fallback triangulation w/o UVs)
    double u = 0.0;
    double v = 0.0;
};

// Polygonal mesh with per-polygon back-references to the B-rep face that
// generated it. Vertices are welded across B-rep face borders so adjacent
// faces that agree on divisions share vertices (no duplicate seams in the
// export).
struct PolyMesh {
    std::vector<std::array<double, 3>> vertices;
    std::vector<Anchor> anchors;                  // parallel to vertices
    std::vector<std::vector<uint32_t>> polygons;  // CCW indices, tri/quad/n-gon
    std::vector<int> polygonFaceId;               // B-rep FaceId per polygon

    size_t vertexCount() const { return vertices.size(); }
    size_t polygonCount() const { return polygons.size(); }

    size_t countQuads() const;
    size_t countTris() const;
    size_t countNgons() const;
};

// Merge vertices closer than `tolerance` and drop degenerate polygons.
// With `group`, only vertices in the same group merge — generation welds
// per SOLID, so contacting bodies in a multi-body file keep their own
// coincident skins instead of fusing into non-manifold shared edges.
void weldVertices(PolyMesh& mesh, double tolerance,
                  const std::vector<int>* group = nullptr);

// Tessellate one polygon for display or export: triples of LOCAL indices
// into `poly`. Convex rings fan; concave and keyhole rings (minimal
// n-gons carry hole loops bridged in with doubled vertices) ear-clip in
// their dominant plane, so a bridged hole tessellates as a hole instead
// of being fanned over.
std::vector<std::array<uint32_t, 3>> triangulatePoly(
    const std::vector<std::array<double, 3>>& verts,
    const std::vector<uint32_t>& poly);

// Game-engine export shaping: triangulate tessellates every quad/n-gon
// (many pipelines want raw tris), yUp converts Z-up CAD space to Y-up
// engine space, and scale converts units (0.01 turns mm into Unreal
// cm... 0.001 into metres for Unity/Blender-metric).
struct Model;  // model.hpp

struct ObjExportOptions {
    bool triangulate = false;
    bool yUp = false;
    double scale = 1.0;
    // Body names for the "o" blocks (parallel to solidFaces; empty entries
    // fall back to object_<n>). Typically Model::solidNames.
    const std::vector<std::string>* objectNames = nullptr;
    // When set, every polygon corner carries the exact surface normal of
    // its own B-rep face ("f v//n"): sharp edges split, fillets shade
    // smooth — no angle-threshold guessing.
    const Model* model = nullptr;
};

// Write Wavefront OBJ. Polygons are grouped per B-rep face
// ("g face_<id>") so CAD face IDs survive into the DCC. When the source
// model's object structure is passed (face ids per solid), each solid
// becomes its own "o object_<n>" block, so importers — including the
// bundled Blender addon — keep separate CAD bodies as separate meshes
// instead of merging everything into one.
void writeObj(const PolyMesh& mesh, const std::string& path,
              const std::vector<std::vector<int>>* solidFaces = nullptr,
              const ObjExportOptions* options = nullptr);

}  // namespace weft
