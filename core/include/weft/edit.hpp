#pragma once

#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <vector>

namespace weft {

// A recorded manual edit (plan §3.4/§5). Ops anchor to the B-rep — a face
// ID plus surface-parametric coordinates — not to mesh vertex indices, so
// they re-apply after re-tessellation or upstream CAD edits.
struct ManualOp {
    enum class Kind {
        LoopInsert,  // insert an edge loop crossing the mesh edge nearest
                     // to (faceId,u,v), at fraction t along that edge
    };
    Kind kind = Kind::LoopInsert;
    int faceId = 0;
    double u = 0.0;
    double v = 0.0;
    double t = 0.5;
};

// Snap a point onto a B-rep face: exact re-projection, not shrinkwrap.
// Updates `p` to the nearest surface point and returns its anchor.
Anchor snapToFace(const Model& model, int faceId, std::array<double, 3>& p);

// Move a vertex toward `target`, constrained to its anchored surface: the
// result is the exact nearest point on the CAD face, with the anchor's
// (u,v) updated to match.
void moveVertex(PolyMesh& mesh, const Model& model, size_t vertIdx,
                const std::array<double, 3>& target);

// Insert an edge loop through the quad strip crossing the mesh edge nearest
// to the op's (faceId,u,v) anchor. The walk continues through quads in both
// directions until it closes on itself or ends at a boundary/non-quad;
// terminal non-quads gain the split vertex so the mesh stays watertight.
// New vertices are evaluated on the live surface (exact), via UV
// interpolation on-face and re-projection across face borders.
// Returns the number of quads the loop crossed (0 = no suitable edge).
int insertLoop(PolyMesh& mesh, const Model& model, const ManualOp& op);

// Re-apply recorded ops after (re)generation, in order.
void applyOps(PolyMesh& mesh, const Model& model,
              const std::vector<ManualOp>& ops);

}  // namespace weft
