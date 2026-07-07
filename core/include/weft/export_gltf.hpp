#pragma once

#include "weft/mesh.hpp"

#include <string>
#include <vector>

namespace weft {

struct Model;  // model.hpp

// Engine-space + metadata knobs for the glTF writer. Defaults reproduce the
// original writer byte-for-byte (Z-up, unit scale, exact CAD normals, and —
// when the source carried per-face colors — a materials[] table).
struct GltfExportOptions {
    double scale = 1.0;       // position multiplier (mm -> target unit)
    bool yUp = false;         // rotate Z-up CAD space into Y-up engine space
    bool emitNormals = true;  // exact CAD normals (false => flat polygon normals)
    bool embedColors = true;  // emit materials[] from Model::faceColors
};

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
              const std::vector<std::vector<int>>* solidFaces = nullptr,
              const GltfExportOptions* options = nullptr);

}  // namespace weft
