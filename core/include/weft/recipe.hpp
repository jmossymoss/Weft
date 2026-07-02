#pragma once

#include "weft/meshers.hpp"

#include <string>

namespace weft {

// The recipe (plan §5): persist decisions, not output. This first slice
// stores generation settings keyed to stable CAD IDs in a line-based text
// format, so a density setup survives re-tessellation and re-import:
//
//   weft-recipe 1
//   default radial=16,axial=4,gridu=4,gridv=4,cap=ngon,chord=0.1
//   face 3 radial=24,axial=2
//   edge 5 20
//
// Manual ops anchored to (faceID, u, v) come with the editing layer.

// Apply one named setting ("radial", "axial", "gridu", "gridv", "cap",
// "chord") to a settings block. Throws on unknown keys/values.
void applySetting(FaceMeshSettings& s, const std::string& key,
                  const std::string& value);

// Apply a comma-separated "key=val,key=val" list.
void applySettingsList(FaceMeshSettings& s, const std::string& list);

void saveRecipe(const GenerationSettings& settings, const std::string& path);
GenerationSettings loadRecipe(const std::string& path);

}  // namespace weft
