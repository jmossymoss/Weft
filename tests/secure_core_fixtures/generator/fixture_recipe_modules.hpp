#pragma once

#include "fixture_contract.hpp"

#include <vector>

namespace cad_mesher::fixtures {

// Each family owns its construction code and appends declarative recipes to the
// shared registry.  Keeping these seams in place lets a complete family land as
// one batch without extending the baseline generator monolith.
void append_curve_recipes(std::vector<Recipe>& recipes);
void append_surface_recipes(std::vector<Recipe>& recipes);
void append_topology_recipes(std::vector<Recipe>& recipes);
void append_feature_recipes(std::vector<Recipe>& recipes);
void append_pathology_recipes(std::vector<Recipe>& recipes);
void append_trim_support_recipes(std::vector<Recipe>& recipes);

} // namespace cad_mesher::fixtures
