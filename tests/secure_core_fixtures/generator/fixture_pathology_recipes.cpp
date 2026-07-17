#include "fixture_builders.hpp"
#include "fixture_recipe_modules.hpp"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <gp_Ax3.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace cad_mesher::fixtures {
namespace {

[[nodiscard]] TopoDS_Shape require_valid_pathology(TopoDS_Shape shape,
                                                   const std::string_view fixture_id) {
  if (shape.IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": constructed a null shape");
  }
  if (!BRepCheck_Analyzer(shape, true).IsValid()) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": valid-pathology baseline is not B-rep valid");
  }
  return shape;
}

[[nodiscard]] RoundTripContract strict_pathology_contract() {
  return RoundTripContract{.accepted_normalization_codes = {}};
}

[[nodiscard]] RoundTripContract tolerance_pathology_contract() {
  return RoundTripContract{
      .tolerances = false,
      .accepted_normalization_codes =
          {
              "step.tolerances_recomputed_from_uncertainty",
          },
  };
}

[[nodiscard]] RoundTripContract standalone_face_pathology_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.standalone_face_wrapped_in_shell",
          },
  };
}

[[nodiscard]] RoundTripContract standalone_face_tolerance_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .tolerances = false,
      .accepted_normalization_codes =
          {
              "step.standalone_face_wrapped_in_shell",
              "step.tolerances_recomputed_from_uncertainty",
          },
  };
}

[[nodiscard]] RoundTripContract free_wire_tolerance_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .tolerances = false,
      .accepted_normalization_codes =
          {
              "step.free_wire_flattened_to_free_edges",
              "step.tolerances_recomputed_from_uncertainty",
          },
  };
}

[[nodiscard]] RoundTripContract scale_pathology_contract() {
  return RoundTripContract{
      .tolerances = false,
      .accepted_normalization_codes =
          {
              "step.model_length_unit_millimetres",
              "step.tolerances_recomputed_from_uncertainty",
          },
  };
}

[[nodiscard]] RoundTripContract orientation_pathology_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .occurrence_graph = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.root_orientation_canonicalized",
              "step.standalone_face_orientation_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract uv_metric_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.cylindrical_parameter_origin_shifted",
              "step.standalone_face_wrapped_in_shell",
          },
  };
}

[[nodiscard]] TopoDS_Face polygon_face(const std::span<const gp_Pnt> points,
                                       const std::string_view fixture_id) {
  BRepBuilderAPI_MakePolygon polygon;
  for (const gp_Pnt& point : points) {
    polygon.Add(point);
  }
  polygon.Close();
  if (!polygon.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": polygon construction failed");
  }
  BRepBuilderAPI_MakeFace face_builder(polygon.Wire(), true);
  return TopoDS::Face(checked_builder_shape(face_builder, fixture_id));
}

[[nodiscard]] TopoDS_Wire square_wire(const double x_offset, const double side,
                                      const std::string_view fixture_id) {
  BRepBuilderAPI_MakePolygon polygon;
  polygon.Add(gp_Pnt(x_offset, 0.0, 0.0));
  polygon.Add(gp_Pnt(x_offset + side, 0.0, 0.0));
  polygon.Add(gp_Pnt(x_offset + side, side, 0.0));
  polygon.Add(gp_Pnt(x_offset, side, 0.0));
  polygon.Close();
  if (!polygon.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": clean closed wire failed");
  }
  return polygon.Wire();
}

void update_shape_tolerances(const TopoDS_Shape& shape, const double tolerance) {
  BRep_Builder builder;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    builder.UpdateFace(TopoDS::Face(explorer.Current()), tolerance);
  }
  for (TopExp_Explorer explorer(shape, TopAbs_EDGE); explorer.More(); explorer.Next()) {
    builder.UpdateEdge(TopoDS::Edge(explorer.Current()), tolerance);
  }
  for (TopExp_Explorer explorer(shape, TopAbs_VERTEX); explorer.More(); explorer.Next()) {
    builder.UpdateVertex(TopoDS::Vertex(explorer.Current()), tolerance);
  }
}

[[nodiscard]] TopoDS_Shape
make_scale_matrix(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.scale.unit_matrix";
  const double source_magnitude = construction_parameter(parameters, "source_magnitude");
  const double millimetre =
      source_magnitude * construction_parameter(parameters, "millimetre_scale_to_millimetres");
  const double inch =
      source_magnitude * construction_parameter(parameters, "inch_scale_to_millimetres");
  const double metre =
      source_magnitude * construction_parameter(parameters, "metre_scale_to_millimetres");
  const double micro = millimetre * construction_parameter(parameters, "micro_scale");
  const double spacing = construction_parameter(parameters, "spacing");
  if (!(micro > 0.0 && millimetre > micro && inch > millimetre && metre > inch)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": scale matrix must be strictly increasing");
  }

  BRepPrimAPI_MakeBox micro_builder(micro, micro, micro);
  BRepPrimAPI_MakeBox millimetre_builder(gp_Pnt(micro + spacing, 0.0, 0.0), millimetre, millimetre,
                                         millimetre);
  BRepPrimAPI_MakeBox inch_builder(gp_Pnt(micro + millimetre + (2.0 * spacing), 0.0, 0.0), inch,
                                   inch, inch);
  BRepPrimAPI_MakeBox metre_builder(gp_Pnt(micro + millimetre + inch + (3.0 * spacing), 0.0, 0.0),
                                    metre, metre, metre);
  const std::array<TopoDS_Shape, 4> children{
      checked_builder_shape(micro_builder, fixture_id),
      checked_builder_shape(millimetre_builder, fixture_id),
      checked_builder_shape(inch_builder, fixture_id),
      checked_builder_shape(metre_builder, fixture_id),
  };
  return require_valid_pathology(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_tolerance_extremes(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.tolerance.extremes";
  const double length = construction_parameter(parameters, "length");
  const double separation = construction_parameter(parameters, "separation");
  const double low_tolerance = construction_parameter(parameters, "low_tolerance");
  const double high_tolerance = construction_parameter(parameters, "high_tolerance");
  if (!(low_tolerance > 0.0 && high_tolerance > low_tolerance && high_tolerance < 0.1 * length)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": tolerance bounds are not controlled by feature scale");
  }

  BRepPrimAPI_MakeBox low_builder(length, length, length);
  BRepPrimAPI_MakeBox high_builder(gp_Pnt(length + separation, 0.0, 0.0), length, length, length);
  TopoDS_Shape low = checked_builder_shape(low_builder, fixture_id);
  TopoDS_Shape high = checked_builder_shape(high_builder, fixture_id);
  update_shape_tolerances(low, low_tolerance);
  update_shape_tolerances(high, high_tolerance);
  const std::array<TopoDS_Shape, 2> children{low, high};
  return require_valid_pathology(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_sliver_needle_microedge(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "face.sliver_needle_microedge";
  const double length = construction_parameter(parameters, "length");
  const double microedge = construction_parameter(parameters, "microedge_length");
  const double needle_width = construction_parameter(parameters, "needle_width");
  const double separation = construction_parameter(parameters, "separation");
  const double sliver_width = construction_parameter(parameters, "sliver_width");
  if (!(microedge > 1.0e-7 && needle_width > microedge && sliver_width > needle_width &&
        length > sliver_width)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": pathology dimensions do not remain above kernel confusion");
  }

  const std::array sliver_points{
      gp_Pnt(0.0, 0.0, 0.0),
      gp_Pnt(length, 0.0, 0.0),
      gp_Pnt(length, sliver_width, 0.0),
  };
  const std::array needle_points{
      gp_Pnt(separation, 0.0, 0.0),
      gp_Pnt(separation + length, 0.0, 0.0),
      gp_Pnt(separation + (0.5 * length), needle_width, 0.0),
  };
  const std::array microedge_points{
      gp_Pnt(2.0 * separation, 0.0, 0.0),
      gp_Pnt((2.0 * separation) + length, 0.0, 0.0),
      gp_Pnt((2.0 * separation) + length, length, 0.0),
      gp_Pnt((2.0 * separation) + microedge, length, 0.0),
      gp_Pnt(2.0 * separation, length - microedge, 0.0),
  };
  const std::array<TopoDS_Shape, 3> children{
      polygon_face(sliver_points, fixture_id),
      polygon_face(needle_points, fixture_id),
      polygon_face(microedge_points, fixture_id),
  };
  return require_valid_pathology(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_extreme_uv_metric_face(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "face.uv.extreme_metric";
  const double radius = construction_parameter(parameters, "radius");
  const double angular_span = construction_parameter(parameters, "angular_span");
  const double axial_span = construction_parameter(parameters, "axial_span");
  if (!(radius > 0.0 && angular_span > 0.0 && axial_span > 0.0 && radius * angular_span > 1.0e-4)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": UV metric still collapses below geometric resolution");
  }
  const gp_Cylinder cylinder(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), radius);
  BRepBuilderAPI_MakeFace face_builder(cylinder, 0.0, angular_span, 0.0, axial_span);
  return require_valid_pathology(checked_builder_shape(face_builder, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_inverted_shell_and_reversed_face(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.orientation.inverted_shell_face";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double length_z = construction_parameter(parameters, "length_z");
  BRepPrimAPI_MakeBox box_builder(length_x, length_y, length_z);
  TopoDS_Solid inverted = TopoDS::Solid(checked_builder_shape(box_builder, fixture_id));
  inverted.Reverse();
  GProp_GProps volume;
  BRepGProp::VolumeProperties(inverted, volume);
  if (!(volume.Mass() < 0.0)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": reversed solid does not retain negative signed volume");
  }

  const double offset = construction_parameter(parameters, "face_offset");
  const std::array face_points{
      gp_Pnt(offset, 0.0, 0.0),
      gp_Pnt(offset + length_x, 0.0, 0.0),
      gp_Pnt(offset + length_x, length_y, 0.0),
      gp_Pnt(offset, length_y, 0.0),
  };
  TopoDS_Face reversed_face = polygon_face(face_points, fixture_id);
  reversed_face.Reverse();
  const std::array<TopoDS_Shape, 2> children{inverted, reversed_face};
  return require_valid_pathology(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_clean_box_baseline(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "baseline.pathology.box";
  BRepPrimAPI_MakeBox box_builder(construction_parameter(parameters, "length_x"),
                                  construction_parameter(parameters, "length_y"),
                                  construction_parameter(parameters, "length_z"));
  return require_valid_pathology(checked_builder_shape(box_builder, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_clean_planar_face_baseline(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "baseline.pathology.planar_face";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const std::array points{
      gp_Pnt(0.0, 0.0, 0.0),
      gp_Pnt(length_x, 0.0, 0.0),
      gp_Pnt(length_x, length_y, 0.0),
      gp_Pnt(0.0, length_y, 0.0),
  };
  return require_valid_pathology(polygon_face(points, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_clean_gap_wire_baseline(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "baseline.pathology.wire_gap_thresholds";
  const double within_gap = construction_parameter(parameters, "within_gap");
  const double beyond_gap = construction_parameter(parameters, "beyond_gap");
  const double tolerance = construction_parameter(parameters, "vertex_tolerance");
  if (!(within_gap > 0.0 && within_gap < tolerance && beyond_gap > tolerance)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": mutation targets do not straddle vertex tolerance");
  }
  TopoDS_Wire within_source =
      square_wire(0.0, construction_parameter(parameters, "side"), fixture_id);
  TopoDS_Wire beyond_source = square_wire(construction_parameter(parameters, "separation"),
                                          construction_parameter(parameters, "side"), fixture_id);
  update_shape_tolerances(within_source, tolerance);
  update_shape_tolerances(beyond_source, tolerance);
  const std::array<TopoDS_Shape, 2> children{within_source, beyond_source};
  return require_valid_pathology(make_compound(children), fixture_id);
}

} // namespace

void append_pathology_recipes(std::vector<Recipe>& recipes) {
  recipes.push_back(procedural_shape_recipe("baseline.pathology.box",
                                            {
                                                {"length_x", 24.0, "millimetres"},
                                                {"length_y", 20.0, "millimetres"},
                                                {"length_z", 16.0, "millimetres"},
                                            },
                                            make_clean_box_baseline, StepIoProfile{},
                                            strict_pathology_contract()));
  recipes.push_back(procedural_shape_recipe("baseline.pathology.planar_face",
                                            {
                                                {"length_x", 30.0, "millimetres"},
                                                {"length_y", 20.0, "millimetres"},
                                                {"near_zero_height_target", 1.0e-8, "millimetres"},
                                                {"zero_height_target", 0.0, "millimetres"},
                                            },
                                            make_clean_planar_face_baseline, StepIoProfile{},
                                            standalone_face_pathology_contract()));
  recipes.push_back(procedural_shape_recipe("baseline.pathology.wire_gap_thresholds",
                                            {
                                                {"beyond_gap", 5.0e-3, "millimetres"},
                                                {"separation", 35.0, "millimetres"},
                                                {"side", 20.0, "millimetres"},
                                                {"vertex_tolerance", 1.0e-3, "millimetres"},
                                                {"within_gap", 5.0e-4, "millimetres"},
                                            },
                                            make_clean_gap_wire_baseline, StepIoProfile{},
                                            free_wire_tolerance_contract()));
  recipes.push_back(procedural_shape_recipe("face.sliver_needle_microedge",
                                            {
                                                {"length", 40.0, "millimetres"},
                                                {"microedge_length", 1.0e-3, "millimetres"},
                                                {"needle_width", 5.0e-3, "millimetres"},
                                                {"separation", 60.0, "millimetres"},
                                                {"sliver_width", 2.0e-2, "millimetres"},
                                            },
                                            make_sliver_needle_microedge, StepIoProfile{},
                                            standalone_face_tolerance_contract()));
  recipes.push_back(procedural_shape_recipe("face.uv.extreme_metric",
                                            {
                                                {"angular_span", 1.0e-4, "radians"},
                                                {"axial_span", 10.0, "millimetres"},
                                                {"radius", 1.0e5, "millimetres"},
                                            },
                                            make_extreme_uv_metric_face, StepIoProfile{},
                                            uv_metric_contract()));
  recipes.push_back(procedural_shape_recipe("solid.orientation.inverted_shell_face",
                                            {
                                                {"face_offset", 40.0, "millimetres"},
                                                {"length_x", 20.0, "millimetres"},
                                                {"length_y", 16.0, "millimetres"},
                                                {"length_z", 12.0, "millimetres"},
                                            },
                                            make_inverted_shell_and_reversed_face, StepIoProfile{},
                                            orientation_pathology_contract()));
  recipes.push_back(
      procedural_shape_recipe("solid.scale.unit_matrix",
                              {
                                  {"inch_scale_to_millimetres", 25.4, "unitless"},
                                  {"metre_scale_to_millimetres", 1000.0, "unitless"},
                                  {"micro_scale", 1.0e-2, "unitless"},
                                  {"millimetre_scale_to_millimetres", 1.0, "unitless"},
                                  {"source_magnitude", 1.0, "unitless"},
                                  {"spacing", 25.0, "millimetres"},
                              },
                              make_scale_matrix, StepIoProfile{}, scale_pathology_contract()));
  recipes.push_back(procedural_shape_recipe("solid.tolerance.extremes",
                                            {
                                                {"high_tolerance", 5.0e-2, "millimetres"},
                                                {"length", 20.0, "millimetres"},
                                                {"low_tolerance", 1.0e-7, "millimetres"},
                                                {"separation", 30.0, "millimetres"},
                                            },
                                            make_tolerance_extremes, StepIoProfile{},
                                            tolerance_pathology_contract()));

  // `corrupt.face.zero_near_zero_area`,
  // `corrupt.self_intersection.curve_trim_surface_shell`,
  // `corrupt.topology.duplicate_vertex_edge`,
  // `corrupt.wire.gap_within_beyond`, and
  // `corrupt.wire.inconsistent_orientation` remain derived_corruption fixtures.
  // Registering their clean sources above does not mislabel a procedural
  // construction as a corruption and gives later deterministic transforms exact
  // threshold inputs.
}

} // namespace cad_mesher::fixtures
