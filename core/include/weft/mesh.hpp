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

// Exact ownership used by the primitive-aware compiler.  Anchor remains the
// lightweight legacy face-UV editing handle; MeshConstraint records whether a
// vertex is fixed to a B-rep vertex, moves along one exact edge parameter, or
// moves in one exact face's UV domain.
enum class MeshConstraintType {
    None,
    BrepVertex,
    BrepEdge,
    BrepFace,
};

struct MeshConstraint {
    MeshConstraintType type = MeshConstraintType::None;
    int ownerId = 0;
    double t = 0.0;
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
    std::vector<MeshConstraint> constraints;      // optional, parallel to vertices
    std::vector<std::vector<uint32_t>> polygons;  // CCW indices, tri/quad/n-gon
    std::vector<int> polygonFaceId;               // B-rep FaceId per polygon
    // Face-specific UV per polygon corner.  Shared 3D edge samples can have a
    // different UV on each incident face (especially periodic seams), so a
    // single vertex-level Anchor cannot represent compiler output faithfully.
    // Empty when a producer has no corner UV data.
    std::vector<std::vector<Anchor>> polygonCornerAnchors;
    // Deterministic global-index triangulation for every polygon.  Modeling
    // output remains mixed n-gons/quads/tris; rendering, collision and
    // triangle-only export can consume this certified representation without
    // allowing each downstream application to pick different diagonals.
    std::vector<std::vector<std::array<uint32_t, 3>>> certifiedTriangles;

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
//
// `vertTol` (optional, parallel to mesh.vertices) gives each vertex its
// own weld radius: a pair merges when their distance is within the LOOSER
// of the two (max-wins), so raising one face's weld tolerance closes its
// junctions without touching the rest of the model. `tolerance` is then
// the spatial-hash cell size and must be >= every vertTol entry (pass the
// maximum). Absent, every vertex uses `tolerance`.
void weldVertices(PolyMesh& mesh, double tolerance,
                  const std::vector<int>* group = nullptr,
                  const std::vector<double>* vertTol = nullptr);

// Tessellate one polygon for display or export: triples of LOCAL indices
// into `poly`. Convex rings fan; concave and keyhole rings (minimal
// n-gons carry hole loops bridged in with doubled vertices) ear-clip in
// their dominant plane, so a bridged hole tessellates as a hole instead
// of being fanned over.
std::vector<std::array<uint32_t, 3>> triangulatePoly(
    const std::vector<std::array<double, 3>>& verts,
    const std::vector<uint32_t>& poly);

// Rebuild PolyMesh::certifiedTriangles from the current polygons.  Call after
// topology-changing operations such as welding or manual editing.
void refreshCertifiedTriangulations(PolyMesh& mesh);

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
    // smooth — no angle-threshold guessing. Also the source of per-face
    // colors (Model::faceColors) for the .mtl sidecar.
    const Model* model = nullptr;
    // Emit exact CAD normals from `model`. False keeps `model` available for
    // colors/names but writes no "vn"/normal indices.
    bool emitNormals = true;
    // Emit an mtllib + .mtl sidecar and per-group usemtl when `model` carries
    // per-face (or per-solid) colors. No color data => no mtl (bytes unchanged).
    bool emitColors = true;
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
