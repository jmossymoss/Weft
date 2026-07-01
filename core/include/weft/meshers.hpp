#pragma once

#include "weft/analysis.hpp"
#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <map>

namespace weft {

enum class CapStyle {
    NGon,  // single n-sided polygon
    Fan,   // center vertex + triangle fan
};

// Named, per-face density controls (the plan's §3.3). A face picks up the
// defaults unless an override is present for its FaceId.
struct FaceMeshSettings {
    int radial = 16;        // divisions around a cylinder / circular cap
    int axial = 4;          // divisions along a cylinder axis
    int gridU = 4;          // planar/parametric grid divisions
    int gridV = 4;
    CapStyle cap = CapStyle::NGon;
    double chordTolerance = 0.1;  // fallback triangulation accuracy
};

struct GenerationSettings {
    FaceMeshSettings defaults;
    std::map<int, FaceMeshSettings> perFace;  // FaceId -> overrides
    double weldTolerance = 1e-6;

    const FaceMeshSettings& forFace(int faceId) const {
        auto it = perFace.find(faceId);
        return it == perFace.end() ? defaults : it->second;
    }
};

// Which strategy generate() picked for each face — reported so the CLI can
// show what was parametric and what fell back to triangulation.
enum class MesherKind {
    CylinderGrid,  // full-revolution cylinder: radial x axial quads
    DiskCap,       // planar face bounded by one full circle
    PlanarGrid,    // planar/parametric UV grid that passed containment
    Fallback,      // OCCT incremental triangulation
};

const char* mesherKindName(MesherKind k);

struct GenerationReport {
    std::map<int, MesherKind> faceMesher;  // FaceId -> strategy used
};

// Generate topology for every face of the model, per-face controllable.
// Every polygon carries its source FaceId; vertices are welded across faces.
PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings,
                  GenerationReport* report = nullptr);

}  // namespace weft
