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
    int radial = 16;        // divisions around a surface of revolution (u)
    int axial = 4;          // divisions along the axis / pole-to-pole (v)
    int gridU = 4;          // planar/parametric grid divisions
    int gridV = 4;
    CapStyle cap = CapStyle::NGon;
    double chordTolerance = 0.1;  // fallback triangulation accuracy
};

struct GenerationSettings {
    FaceMeshSettings defaults;
    std::map<int, FaceMeshSettings> perFace;  // FaceId -> overrides
    // EdgeId -> exact subdivision count. Wins over face proposals for the
    // whole shared-edge group it belongs to (plan §5 per_edge_settings).
    std::map<int, int> perEdge;
    double weldTolerance = 1e-6;

    const FaceMeshSettings& forFace(int faceId) const {
        auto it = perFace.find(faceId);
        return it == perFace.end() ? defaults : it->second;
    }
};

// Which strategy generate() picked for each face — reported so the CLI can
// show what was parametric and what fell back to triangulation.
enum class MesherKind {
    RevolutionGrid,  // closed-u cylinder/cone/sphere/torus: quad grid with
                     // wrap-around seams and collapsed apex/pole rows
    DiskCap,         // planar face bounded by one full circle
    PlanarGrid,      // planar/parametric UV grid that passed containment
    Fallback,        // OCCT incremental triangulation
};

const char* mesherKindName(MesherKind k);

struct GenerationReport {
    std::map<int, MesherKind> faceMesher;  // FaceId -> strategy used
    // EdgeId -> solved subdivision count, for edges that took part in
    // density matching. Adjacent faces sharing an edge agree on this count.
    std::map<int, int> edgeDivisions;
};

// Generate topology for every face of the model, per-face controllable.
//
// Density matching (plan §3.3, first increment): before meshing, edge
// subdivision counts are solved as shared constraints — edges that a
// parametric mesher requires to be equal (opposite sides of a grid, the
// rings of one revolution face) are grouped, each face proposes its own
// settings onto its edges, and every group resolves to the max proposal
// (or an explicit perEdge override). Meshers then honor the solved counts,
// so neighbouring parametric faces meet vertex-for-vertex.
//
// Every polygon carries its source FaceId; vertices are welded across faces.
PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings,
                  GenerationReport* report = nullptr);

}  // namespace weft
