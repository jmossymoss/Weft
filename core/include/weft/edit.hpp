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
        Bridge,      // connect the two open boundary loops nearest to the
                     // B-rep edges edgeA/edgeB with a strip: equal vertex
                     // counts give a pure quad ring, unequal counts a
                     // triangulated zipper. Boundaries appear when faces
                     // are excluded from output (FaceMeshSettings.exclude).
        NudgeVertex,  // slide the interior vertex anchored nearest to
                      // (faceId,u,v) to the surface point (u2,v2) — a
                      // vertex tweak that stays exactly on the CAD face
                      // and survives re-tessellation
        FillLoop,     // close the open boundary loop nearest to the B-rep
                      // edge edgeA with a single n-gon (the minimal cap
                      // for a deleted face's border)
        DeletePoly,   // remove the single mesh polygon whose centroid is
                      // nearest to the WORLD point stored in (u,v,t) —
                      // polygon-mode surgery; its border becomes an open
                      // loop for bridging/filling
        DissolveLoop,  // remove the edge LOOP through the mesh edge whose
                       // midpoint is nearest to the WORLD point stored in
                       // (u,v,t), merging the polygons across each loop
                       // edge and dropping the loop's 2-valence verts —
                       // the faces survive (Blender's ctrl+X)
    };
    Kind kind = Kind::LoopInsert;
    int faceId = 0;
    double u = 0.0;
    double v = 0.0;
    double t = 0.5;
    double u2 = 0.0;  // NudgeVertex: target surface parameters
    double v2 = 0.0;
    int edgeA = 0;  // Bridge: stable B-rep edge ids the two loops hug
    int edgeB = 0;
    // Bridge: rotate the rail pairing by N steps PER SIDE — the automatic
    // alignment can land a step off on symmetric loops, which reads as a
    // spiral. [ ] adjusts the active side on the last bridge in the app;
    // shift+wheel flips which side is active (each side KEEPS its value).
    // The two sides counter-rotate (net pairing = twist - twistA), so a
    // spiral that spins both rims is undone by matching them.
    int twist = 0;      // B-side rotation
    int twistA = 0;     // A-side rotation (counter-direction)
    int twistSide = 0;  // app UI state: which side [ ] edits (0 = B, 1 = A)
    // Bridge: rows ACROSS the strip (V spans). Equal-count bridges emit
    // spans x N quads; zipper/same-loop bridges ignore it.
    int spans = 1;
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

// Slide the interior vertex whose anchor is nearest to op's (faceId,u,v)
// to the exact surface point at (u2,v2), updating its anchor. Border
// vertices carry no face anchor (they belong to shared B-rep edges), so
// they can't be nudged — density and conformity own them.
// Returns 1 if a vertex moved, 0 if none was found.
int nudgeVertex(PolyMesh& mesh, const Model& model, const ManualOp& op);

// Walk the edge loop through (a,b): at each 4-valence vertex continue
// with the edge that shares neither adjacent polygon (Blender's rule);
// stops at boundaries, poles, or when it closes. Returns the loop as
// vertex-pair segments including the seed.
std::vector<std::pair<uint32_t, uint32_t>> walkEdgeLoop(const PolyMesh& mesh,
                                                        uint32_t a,
                                                        uint32_t b);

// Dissolve the edge loop nearest the op's world point (u,v,t): merge the
// two polygons across every loop edge and drop loop verts that end up
// with only two remaining edges. Faces survive; only the loop vanishes.
// Returns the number of edges dissolved (0 = nothing found).
int dissolveLoop(PolyMesh& mesh, const Model& model, const ManualOp& op);

// Bridge the two open boundary loops nearest to op.edgeA / op.edgeB (see
// ManualOp::Kind::Bridge). Vertex counts per boundary come from the density
// solver (pin them per-edge to choose quads vs triangles). When both edges
// land on the SAME loop (a deleted band whose rims connect), the loop is
// split into a rail along each picked edge and zippered across, with the
// left-over spans closed as n-gon caps. Returns polygons added (0 = fail).
int bridgeLoops(PolyMesh& mesh, const Model& model, const ManualOp& op);

// Close the open boundary loop nearest to op.edgeA with one n-gon — the
// fill tool for borders left by deleted faces. Returns 1 or 0.
int fillLoop(PolyMesh& mesh, const Model& model, const ManualOp& op);

// Remove the polygon whose centroid is nearest to the world point in the
// op's (u,v,t). Returns 1 or 0 (empty mesh).
int deletePoly(PolyMesh& mesh, const ManualOp& op);

// An open boundary loop of the mesh: ordered vertex ring where each edge is
// used by exactly one polygon. Exposed for interactive tools (hover/pick).
std::vector<std::vector<uint32_t>> boundaryLoops(const PolyMesh& mesh);

// Re-apply recorded ops after (re)generation, in order.
void applyOps(PolyMesh& mesh, const Model& model,
              const std::vector<ManualOp>& ops);

}  // namespace weft
