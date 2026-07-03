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
void weldVertices(PolyMesh& mesh, double tolerance);

// Write Wavefront OBJ. Polygons are grouped per B-rep face
// ("g face_<id>") so CAD face IDs survive into the DCC. When the source
// model's object structure is passed (face ids per solid), each solid
// becomes its own "o object_<n>" block, so importers — including the
// bundled Blender addon — keep separate CAD bodies as separate meshes
// instead of merging everything into one.
void writeObj(const PolyMesh& mesh, const std::string& path,
              const std::vector<std::vector<int>>* solidFaces = nullptr);

}  // namespace weft
