#pragma once

#include "weft/analysis.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

namespace weft {

// Independent-face tessellation (this fork's product path).
//
// Each face meshes from angle + chord. Planar single-wire faces become
// n-gons. Shared B-rep edges receive one sample count (deflection, then
// max of adjacent feature span requests). There is no density-group
// solver and no model-wide border contract.
//
// generate() remains available for A/B comparison. See
// docs/EXECUTION_PLAN.md and docs/PRODUCTION_PATH.md.
PolyMesh meshIndependent(const Model& model, const Analysis& analysis,
                         const GenerationSettings& settings,
                         GenerationReport* report = nullptr);

}  // namespace weft
