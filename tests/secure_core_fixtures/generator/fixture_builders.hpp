#pragma once

#include "fixture_contract.hpp"

#include <BRepBuilderAPI_MakeShape.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

namespace cad_mesher::fixtures {

class ConstructionFailure final : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

[[nodiscard]] double construction_parameter(std::span<const ConstructionParameter> parameters,
                                            std::string_view name);

template <typename Builder>
[[nodiscard]] TopoDS_Shape checked_builder_shape(Builder& builder,
                                                 const std::string_view fixture_id) {
  // Several OCCT makers are lazy: their constructor records parameters while
  // Shape() performs the first build. Make that transition explicit so the
  // completion check is meaningful and consistent across maker classes.
  if (!builder.IsDone()) {
    builder.Build();
  }
  if (!builder.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": OCCT builder did not complete");
  }
  TopoDS_Shape shape = builder.Shape();
  if (shape.IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": OCCT builder returned a null shape");
  }
  return shape;
}

[[nodiscard]] TopoDS_Compound make_compound(std::span<const TopoDS_Shape> children);
[[nodiscard]] TopoDS_Wire make_wire(std::span<const TopoDS_Edge> edges);

[[nodiscard]] Recipe procedural_shape_recipe(std::string_view id,
                                             std::vector<ConstructionParameter> parameters,
                                             ShapeFactory factory, StepIoProfile step_io = {},
                                             RoundTripContract roundtrip = {});

} // namespace cad_mesher::fixtures
