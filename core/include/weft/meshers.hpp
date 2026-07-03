#pragma once

#include "weft/analysis.hpp"
#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <array>
#include <cstdio>
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
    int axial = 1;          // divisions along the axis / pole-to-pole (v)
    int gridU = 1;          // planar/parametric grid divisions — start
    int gridV = 1;          // minimal (game topology), densify on demand
    CapStyle cap = CapStyle::NGon;
    // Freeform/trimmed faces (the fallback mesher) are driven by these two,
    // Plasticity-style: max chordal deviation from the true surface, and max
    // angle between adjacent facets. On an imported model most faces are
    // freeform, so these ARE the global density controls.
    double chordTolerance = 0.1;   // max deviation (model units)
    double angleToleranceDeg = 28.0;  // max facet turn angle (degrees)
    // Fillet/blend faces: divisions ACROSS the blend (support loops for
    // baking) and how strongly the loops cluster toward the creases
    // ("hold" loops; 0 = uniform spacing, toward 1 = tight at the edges).
    int filletLoops = 3;
    double filletHold = 0.0;
    // Ring junctions (a hole/boss circle inside a rectangular planar face):
    // number of concentric quad loops between the circle and the boundary.
    int junctionRings = 1;
    // Trimmed/freeform faces that fall back to triangulation: pair the
    // triangles into quads where quality allows (guided by the surface's
    // parametric directions). Off (default) = pure triangles, which keep
    // their borders exactly on the B-rep edges for clean seam conformity.
    bool quadDominant = false;
    // Game-topology minimalism (plan §1/§4.1): a flat face doesn't need an
    // interior grid. When set, a planar grid-safe face emits one boundary
    // n-gon instead — border vertices stay density-matched, so neighbours
    // still weld watertight, and the engine triangulates however it likes.
    bool minimal = false;
    // Exclude this face from the output entirely (delete it). Its borders
    // become open boundary loops, which a Bridge op can then reconnect —
    // e.g. drop a bore wall and bridge the two rims shut.
    bool exclude = false;
    // Force a specific mesher instead of the automatic pick (0 = auto,
    // else 1 + MesherKind value). If the forced strategy can't build on
    // the face it falls back to triangulation, so the choice is visible.
    int forceMesher = 0;
    // Revolution bands: keep the two rims at the same count (a quad band)
    // or let them differ — the band then meshes as a triangulated taper
    // between the rims (pin each rim's count per-edge / in the UI).
    bool linkRims = true;
    // Freeform fallback extras: minimum element size (0 = no floor) and
    // deviation measured relative to face size instead of absolute.
    double minSize = 0.0;
    bool relativeDeviation = false;
};

struct GenerationSettings {
    FaceMeshSettings defaults;
    std::map<int, FaceMeshSettings> perFace;  // FaceId -> overrides
    // EdgeId -> exact subdivision count. Wins over face proposals for the
    // whole shared-edge group it belongs to (plan §5 per_edge_settings).
    std::map<int, int> perEdge;
    double weldTolerance = 1e-6;
    // Runtime/debug knobs (not persisted in recipes): turn off worker
    // threads or the freeform border-conformity pass to bisect problems.
    bool parallelMeshing = true;
    bool conformBorders = true;

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
    CoonsGrid,       // four-sided (trimmed/freeform) face: structured quad
                     // grid blended between its four boundary pcurves —
                     // clean flow on bspline strips instead of triangles
    RingJunction,    // rectangular planar face with one circular hole:
                     // concentric quad rings from the circle to the border
                     // (the cylinder-to-plane junction pattern, plan §3.3/4.2)
    QuadDominant,    // fallback triangulation + guided tri-pairing into
                     // quads (plan §3.5 seed; a cross-field solver slots in
                     // here later)
    MinimalNGon,     // planar face as a single boundary n-gon (flat panels
                     // don't need interior topology for game meshes)
    Fallback,        // OCCT incremental triangulation, pure triangles
    AnnulusRing,     // face bounded by exactly two closed loops (the flat
                     // ring between two revolution rims): one zippered
                     // band — equal counts give pure quads
};

const char* mesherKindName(MesherKind k);

// n+1 monotonically increasing parameters in [0,1] splitting it into n
// intervals. hold=0 is uniform; hold in (0,1) squeezes the intervals toward
// both ends, which is how fillet support loops hug the creases.
std::vector<double> clusteredParams(int divisions, double hold);

struct GenerationReport {
    std::map<int, MesherKind> faceMesher;  // FaceId -> strategy used
    // EdgeId -> solved subdivision count, for edges that took part in
    // density matching. Adjacent faces sharing an edge agree on this count.
    std::map<int, int> edgeDivisions;
    // Revolution faces: the edge ids of their two rims (u-boundary rings),
    // so UIs can pin each rim's count individually when rims are unlinked.
    std::map<int, std::array<int, 2>> faceRims;
};

// Route the generator's stage-by-stage debug trace (plans, density solve,
// each face meshed, conformity per edge, weld) to a stream; null disables.
// Lines are flushed as written so a crash log ends at the crash site.
void setGenerateDebugLog(std::FILE* f);

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
