#pragma once

#include "occt_compat.hpp"

#include <TDocStd_Document.hxx>
#include <TopoDS_Shape.hxx>
#include <functional>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace cad_mesher::fixtures {

enum class EvidenceLane {
  occt_procedural,
  derived_corruption,
  native_fault_injection,
  audited_external,
};

enum class StepSchema { ap203, ap214, ap242_dis };

struct ConstructionParameter final {
  std::string_view name;
  double value{};
  std::string_view unit;
};

struct StepIoProfile final {
  StepSchema schema{StepSchema::ap242_dis};
  bool surface_curve_mode{true};
  bool tessellated_geometry{false};
  bool shape_processing{false};
  bool same_parameter_repair{false};
  bool clean_duplicates{false};
  bool write_nonmanifold{false};
  bool read_nonmanifold{false};
};

// STEP is allowed to canonicalise representational wrappers while preserving the
// same exact geometry.  Each recipe must therefore state which observations are
// required to survive rather than relying on one global all-fields comparison.
struct RoundTripContract final {
  bool expected_brep_valid{true};
  bool topology_counts{true};
  bool geometry_families{true};
  bool concrete_types{true};
  bool occurrence_graph{true};
  bool boundary_loops{true};
  bool model_topology{true};
  bool tolerances{true};
  std::vector<std::string_view> accepted_normalization_codes{};
};

using ShapeFactory = std::function<TopoDS_Shape(std::span<const ConstructionParameter>)>;
using DocumentFactory =
    std::function<occ::handle<TDocStd_Document>(std::span<const ConstructionParameter>)>;

struct Recipe final {
  std::string_view id;
  std::vector<ConstructionParameter> parameters;
  EvidenceLane evidence_lane{EvidenceLane::occt_procedural};
  StepIoProfile step_io{};
  RoundTripContract roundtrip{};
  ShapeFactory shape_factory{};
  DocumentFactory document_factory{};

  Recipe(std::string_view recipe_id, std::vector<ConstructionParameter> recipe_parameters,
         const bool expected_brep_valid, ShapeFactory recipe_shape_factory,
         const bool nonmanifold_mode = false, DocumentFactory recipe_document_factory = {})
      : id(recipe_id), parameters(std::move(recipe_parameters)),
        step_io{.write_nonmanifold = nonmanifold_mode, .read_nonmanifold = nonmanifold_mode},
        roundtrip{.expected_brep_valid = expected_brep_valid},
        shape_factory(std::move(recipe_shape_factory)),
        document_factory(std::move(recipe_document_factory)) {}

  Recipe(std::string_view recipe_id, std::vector<ConstructionParameter> recipe_parameters,
         const EvidenceLane recipe_evidence_lane, StepIoProfile recipe_step_io,
         RoundTripContract recipe_roundtrip, ShapeFactory recipe_shape_factory,
         DocumentFactory recipe_document_factory = {})
      : id(recipe_id), parameters(std::move(recipe_parameters)),
        evidence_lane(recipe_evidence_lane), step_io(std::move(recipe_step_io)),
        roundtrip(std::move(recipe_roundtrip)), shape_factory(std::move(recipe_shape_factory)),
        document_factory(std::move(recipe_document_factory)) {}
};

[[nodiscard]] constexpr std::string_view evidence_lane_name(const EvidenceLane lane) noexcept {
  switch (lane) {
  case EvidenceLane::occt_procedural:
    return "occt_procedural";
  case EvidenceLane::derived_corruption:
    return "derived_corruption";
  case EvidenceLane::native_fault_injection:
    return "native_fault_injection";
  case EvidenceLane::audited_external:
    return "audited_external";
  }
  return "invalid";
}

[[nodiscard]] constexpr std::string_view step_schema_name(const StepSchema schema) noexcept {
  switch (schema) {
  case StepSchema::ap203:
    return "AP203";
  case StepSchema::ap214:
    return "AP214IS";
  case StepSchema::ap242_dis:
    return "AP242DIS";
  }
  return "invalid";
}

} // namespace cad_mesher::fixtures
