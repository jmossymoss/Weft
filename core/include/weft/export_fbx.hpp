#pragma once

#include "weft/mesh.hpp"

#include <string>

namespace weft {

// FBX export options mirror the OBJ exporter's engine-space knobs. FBX
// polygons carry arbitrary vertex counts, so quads/n-gons survive
// unless triangulate is set (fan, same as export time elsewhere).
struct FbxExportOptions {
    bool triangulate = false;
    bool yUp = true;      // rotate Z-up CAD space into FBX's Y-up
    double scale = 1.0;   // unit multiplier applied to positions
};

// Write binary FBX 7.4 — one mesh object holding the whole model. The
// writer emits the minimum node tree the mainstream importers (Blender,
// assimp, Unity) accept: Geometry with Vertices/PolygonVertexIndex,
// a Model node, and their connections. Normals are left to the
// importer (they recompute from topology, which matches our smoothing-
// angle display model better than baked per-corner normals would).
void writeFbx(const PolyMesh& mesh, const std::string& path,
              const FbxExportOptions& opts = {});

}  // namespace weft
