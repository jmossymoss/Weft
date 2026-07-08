#pragma once

#include "weft/analysis.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"  // GenerationSettings, GenerationReport, MesherKind
#include "weft/model.hpp"

namespace weft {

// Decoupled mesher — the rewrite/decoupled-core architecture.
//
// generate() solves a GLOBAL integer constraint over the face-adjacency graph
// so that opposite grid sides, revolution rims, etc. share a subdivision count;
// the count then ripples across the model and a large machinery of transition
// strips / T-junction absorbers patches the places the solve can't reconcile.
//
// This mesher drops the global solve. Faces couple ONLY through per-edge sample
// counts: every shared B-rep edge is sampled ONCE at a count that is a pure
// input (a per-edge pin, a per-face radial/axial proposal, or a global default),
// and BOTH incident faces read the exact same 3D samples — so they weld with no
// ripple and no global solve. Each face then meshes its INTERIOR at its OWN
// count and absorbs the interior->border gap with a monotone loop bridge (quads
// where counts line up, grouped n-gons where they don't). This is de-risked by
// core/spike/{seam_bridge,decoupled_face}.cpp (watertight, manifold, fold-free
// for wild count ratios).
//
// GenerationSettings is reused so the CLI/app knobs are shared:
//   radial / axial  -> revolution interior (nu around u, nv along v)
//   gridU  / gridV  -> planar / UV interior grid
//   perEdge[eid]    -> pins that edge's sample count directly (pure input)
// Every polygon carries its source FaceId; vertices are welded per solid.
PolyMesh meshDecoupled(const Model& model, const Analysis& analysis,
                       const GenerationSettings& settings,
                       GenerationReport* report = nullptr);

}  // namespace weft
