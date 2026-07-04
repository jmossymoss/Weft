#pragma once

#include <array>
#include <cstdint>
#include <map>
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
    // Part id per polygon (1-based, dense): connected components of the
    // B-rep face-adjacency graph — the assembly's individual bodies.
    // Exporters split output by part so assemblies arrive as parts, not
    // as one fused blob. Empty = single part.
    std::vector<int> polygonPartId;
    // Part id -> STEP product name, when the source carried one; parts
    // absent from the map export as "part_<id>".
    std::map<int, std::string> partNames;

    size_t vertexCount() const { return vertices.size(); }
    size_t polygonCount() const { return polygons.size(); }

    size_t countQuads() const;
    size_t countTris() const;
    size_t countNgons() const;
};

// Merge vertices closer than `tolerance` and drop degenerate polygons.
// When `groups` is given (parallel to vertices), only vertices in the same
// group merge — used to keep separate solids of an assembly from fusing
// into non-manifold contact surfaces where parts touch.
void weldVertices(PolyMesh& mesh, double tolerance,
                  const std::vector<int>* groups = nullptr);

// Write Wavefront OBJ. Polygons are grouped per B-rep face
// ("g face_<id>") so CAD face IDs survive into the DCC.
//
// With a Model, every polygon corner gets an exact CAD normal ("vn",
// f v//n): the true surface normal of the polygon's OWN B-rep face at
// that corner. Because corners of polygons from different faces carry
// each face's normal, tangent joins (fillets) shade smooth and sharp
// edges stay sharp — split normals for free, no angle heuristics.
struct Model;  // model.hpp; kept out of this header to spare OCCT includes
void writeObj(const PolyMesh& mesh, const std::string& path,
              const Model* model = nullptr);

}  // namespace weft
