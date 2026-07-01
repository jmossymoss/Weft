#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace weft {

// Polygonal mesh with per-polygon back-references to the B-rep face that
// generated it. Vertices are welded across B-rep face borders so adjacent
// faces that agree on divisions share vertices (no duplicate seams in the
// export).
struct PolyMesh {
    std::vector<std::array<double, 3>> vertices;
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
// ("g face_<id>") so CAD face IDs survive into the DCC.
void writeObj(const PolyMesh& mesh, const std::string& path);

}  // namespace weft
