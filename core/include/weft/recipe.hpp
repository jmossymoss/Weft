#pragma once

#include "weft/compiler.hpp"
#include "weft/edit.hpp"
#include "weft/meshers.hpp"

#include <string>
#include <vector>

namespace weft {

enum class MeshPipeline {
    PrimitiveCompiler,
    Legacy,
};

// The recipe (plan §5): persist decisions, not output. Generation settings
// keyed to stable CAD IDs plus manual ops anchored to (faceID,u,v), in a
// line-based text format, so the whole setup — density AND edits —
// survives re-tessellation and re-import:
//
//   weft-recipe 1
//   default radial=16,axial=4,gridu=4,gridv=4,cap=ngon,chord=0.1
//   face 3 radial=24,axial=2
//   edge 5 20
//   op loop 3 0.5 0.5 0.35
struct Recipe {
    // The production workflow stays on the more complete legacy mesher set.
    // The primitive compiler remains available as an explicit experiment.
    MeshPipeline pipeline = MeshPipeline::Legacy;
    CompilerSettings compiler;
    GenerationSettings settings;
    std::vector<ManualOp> ops;  // replayed in order after generation
};

// Apply one named setting ("radial", "axial", "gridu", "gridv", "cap",
// "chord") to a settings block. Throws on unknown keys/values.
void applySetting(FaceMeshSettings& s, const std::string& key,
                  const std::string& value);

// Apply a comma-separated "key=val,key=val" list.
void applySettingsList(FaceMeshSettings& s, const std::string& list);

// Canonical, locale-independent persistence form shared by recipe v1 and the
// source-referenced recipe v2 contract.
std::string formatSettingsList(const FaceMeshSettings& settings);

void saveRecipe(const Recipe& recipe, const std::string& path);
Recipe loadRecipe(const std::string& path);

}  // namespace weft
