#include "fixture_builders.hpp"

#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRep_Builder.hxx>
#include <algorithm>
#include <utility>

namespace cad_mesher::fixtures {

double construction_parameter(const std::span<const ConstructionParameter> parameters,
                              const std::string_view name) {
  const auto found = std::find_if(
      parameters.begin(), parameters.end(),
      [name](const ConstructionParameter& parameter) { return parameter.name == name; });
  if (found == parameters.end()) {
    throw ConstructionFailure("missing construction parameter: " + std::string(name));
  }
  return found->value;
}

TopoDS_Compound make_compound(const std::span<const TopoDS_Shape> children) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const TopoDS_Shape& child : children) {
    if (child.IsNull()) {
      throw ConstructionFailure("cannot add a null child to a fixture compound");
    }
    builder.Add(compound, child);
  }
  return compound;
}

TopoDS_Wire make_wire(const std::span<const TopoDS_Edge> edges) {
  BRepBuilderAPI_MakeWire builder;
  for (const TopoDS_Edge& edge : edges) {
    if (edge.IsNull()) {
      throw ConstructionFailure("cannot add a null edge to a fixture wire");
    }
    builder.Add(edge);
  }
  if (!builder.IsDone()) {
    throw ConstructionFailure("fixture wire builder did not complete");
  }
  return builder.Wire();
}

Recipe procedural_shape_recipe(std::string_view id, std::vector<ConstructionParameter> parameters,
                               ShapeFactory factory, StepIoProfile step_io,
                               RoundTripContract roundtrip) {
  return Recipe{id,
                std::move(parameters),
                EvidenceLane::occt_procedural,
                std::move(step_io),
                std::move(roundtrip),
                std::move(factory)};
}

} // namespace cad_mesher::fixtures
