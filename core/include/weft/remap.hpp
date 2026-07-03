#pragma once

#include "weft/analysis.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"

#include <map>

namespace weft {

// Carrying a recipe across a re-import (plan §5): recipes anchor decisions
// to CAD face/edge ids, but a STEP re-export after upstream topology edits
// can renumber them. remapRecipe re-identifies every feature geometrically
// so the decisions follow the actual faces, not their file order.
struct RemapReport {
    // Every old id -> its best new id (missing/0 = no confident match).
    std::map<int, int> faceMap;
    std::map<int, int> edgeMap;
    // Recipe references that matched nothing and were dropped rather than
    // left pointing at the wrong feature.
    int facesDropped = 0;  // per-face overrides lost
    int edgesDropped = 0;  // per-edge pins lost
    int opsDropped = 0;    // manual ops lost
};

// Rewrite every B-rep reference in `recipe` (per-face overrides, per-edge
// pins, op face/edge ids) from `oldModel` to `newModel` by geometric
// signature — surface type + area + centroid (+ radius) for faces, length
// + midpoint for edges. Matches are unique (greedy best-first) and prefer
// the same id on ties, so an unchanged re-export maps to identity; the
// modified faces of a real edit match through their signature instead.
Recipe remapRecipe(const Recipe& recipe, const Model& oldModel,
                   const Analysis& oldAnalysis, const Model& newModel,
                   const Analysis& newAnalysis,
                   RemapReport* report = nullptr);

}  // namespace weft
