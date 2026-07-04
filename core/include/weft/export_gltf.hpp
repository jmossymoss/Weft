#pragma once

#include "weft/mesh.hpp"

#include <string>
#include <vector>

namespace weft {

struct Model;  // model.hpp

// Write binary glTF 2.0 (.glb) — the engine-ready delivery next to OBJ.
// Polygons are fan-triangulated; every corner carries the exact CAD
// normal of its own B-rep face (vertices are split per (position, face)
// pair, so sharp edges split and fillets shade smooth exactly like the
// OBJ path); the source B-rep FaceId rides along as the custom
// _WEFT_FACE_ID vertex attribute so CAD faces survive into engines and
// DCCs that read custom attributes.
// solidFaces (face ids per solid, from Analysis) splits the export into
// one node per body; model supplies exact normals and body names.
void writeGlb(const PolyMesh& mesh, const std::string& path,
              const Model* model = nullptr,
              const std::vector<std::vector<int>>* solidFaces = nullptr);

}  // namespace weft
