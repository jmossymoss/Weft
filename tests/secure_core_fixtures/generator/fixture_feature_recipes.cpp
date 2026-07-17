#include "fixture_builders.hpp"
#include "fixture_recipe_modules.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_JoinType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_List.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <iterator>
#include <limits>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace cad_mesher::fixtures {
namespace {

constexpr double kSelectionTolerance = 1.0e-7;

[[nodiscard]] bool near(const double left, const double right) {
  return std::abs(left - right) <= kSelectionTolerance;
}

[[nodiscard]] TopoDS_Shape require_valid_feature(TopoDS_Shape shape,
                                                 const std::string_view fixture_id) {
  if (shape.IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": constructed a null shape");
  }
  if (!BRepCheck_Analyzer(shape, true).IsValid()) {
    throw ConstructionFailure(std::string(fixture_id) + ": constructed feature is not B-rep valid");
  }
  return shape;
}

[[nodiscard]] RoundTripContract standalone_face_compound_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .occurrence_graph = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.standalone_faces_wrapped_in_shells",
          },
  };
}

[[nodiscard]] RoundTripContract boolean_feature_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.same_domain_faces_split_or_merged",
              "step.subshape_order_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract tolerance_boolean_feature_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .tolerances = false,
      .accepted_normalization_codes =
          {
              "step.same_domain_faces_split_or_merged",
              "step.subshape_order_canonicalized",
              "step.tolerances_recomputed_from_uncertainty",
          },
  };
}

[[nodiscard]] RoundTripContract invalid_boolean_feature_contract() {
  RoundTripContract contract = boolean_feature_contract();
  contract.expected_brep_valid = false;
  contract.geometry_families = false;
  contract.concrete_types = false;
  contract.accepted_normalization_codes.push_back("step.coaxial_transition_import_invalid");
  contract.accepted_normalization_codes.push_back("step.coaxial_transition_curves_canonicalized");
  return contract;
}

[[nodiscard]] RoundTripContract blend_feature_contract() {
  return RoundTripContract{
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.blend_bspline_parameterization_canonicalized",
              "step.subshape_order_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract invalid_blend_feature_contract() {
  RoundTripContract contract = blend_feature_contract();
  contract.expected_brep_valid = false;
  contract.topology_counts = false;
  contract.model_topology = false;
  contract.accepted_normalization_codes.push_back("step.split_fillet_chain_import_invalid");
  contract.accepted_normalization_codes.push_back("step.single_solid_compound_root_unwrapped");
  return contract;
}

[[nodiscard]] RoundTripContract compound_root_blend_feature_contract() {
  RoundTripContract contract = blend_feature_contract();
  contract.topology_counts = false;
  contract.model_topology = false;
  contract.accepted_normalization_codes.push_back("step.single_solid_compound_root_unwrapped");
  return contract;
}

[[nodiscard]] RoundTripContract
invalid_named_blend_feature_contract(const std::string_view normalization_code,
                                     const bool relax_tolerances = false) {
  RoundTripContract contract = blend_feature_contract();
  contract.expected_brep_valid = false;
  contract.topology_counts = false;
  contract.model_topology = false;
  contract.tolerances = !relax_tolerances;
  contract.accepted_normalization_codes.push_back(normalization_code);
  contract.accepted_normalization_codes.push_back("step.single_solid_compound_root_unwrapped");
  if (relax_tolerances) {
    contract.accepted_normalization_codes.push_back("step.tolerances_recomputed_from_uncertainty");
  }
  return contract;
}

[[nodiscard]] RoundTripContract offset_feature_contract() {
  return RoundTripContract{
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.offset_intersection_curves_canonicalized",
              "step.subshape_order_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract tolerance_offset_feature_contract() {
  RoundTripContract contract = offset_feature_contract();
  contract.tolerances = false;
  contract.accepted_normalization_codes.push_back("step.tolerances_recomputed_from_uncertainty");
  return contract;
}

[[nodiscard]] RoundTripContract revolution_feature_contract() {
  return RoundTripContract{
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.periodic_seam_parameter_shifted",
          },
  };
}

[[nodiscard]] RoundTripContract invalid_revolution_feature_contract() {
  RoundTripContract contract = revolution_feature_contract();
  contract.expected_brep_valid = false;
  contract.accepted_normalization_codes.push_back("step.profiled_revolution_import_invalid");
  return contract;
}

[[nodiscard]] RoundTripContract variable_chamfer_contract() {
  return RoundTripContract{
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.ruled_surface_bspline_canonicalized",
              "step.subshape_order_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract invalid_variable_chamfer_contract() {
  RoundTripContract contract = variable_chamfer_contract();
  contract.expected_brep_valid = false;
  contract.topology_counts = false;
  contract.model_topology = false;
  contract.tolerances = false;
  contract.accepted_normalization_codes.push_back("step.variable_chamfer_import_invalid");
  contract.accepted_normalization_codes.push_back("step.single_solid_compound_root_unwrapped");
  contract.accepted_normalization_codes.push_back("step.tolerances_recomputed_from_uncertainty");
  return contract;
}

[[nodiscard]] RoundTripContract blend_topology_witness_contract() {
  // M1 records exact source topology for unsupported junctions; it does not claim that
  // OCCT or the base mesher generated a setback, T-, Y-, or X-cap. STEP may reorder free
  // subshapes and canonicalise blend representations, but counts, geometry families, and
  // model-level free-edge incidence remain mandatory evidence.
  return RoundTripContract{
      .expected_brep_valid = true,
      .topology_counts = true,
      .geometry_families = true,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = true,
      .tolerances = true,
      .accepted_normalization_codes =
          {
              "step.blend_bspline_parameterization_canonicalized",
              "step.free_subshape_orientation_canonicalized",
              "step.subshape_order_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract
free_graph_blend_topology_witness_contract(const bool expected_brep_valid,
                                           const std::string_view normalization_code) {
  RoundTripContract contract = blend_topology_witness_contract();
  contract.expected_brep_valid = expected_brep_valid;
  contract.topology_counts = false;
  contract.model_topology = false;
  contract.accepted_normalization_codes.push_back("step.free_vertex_identity_split");
  if (!normalization_code.empty()) {
    contract.accepted_normalization_codes.push_back(normalization_code);
  }
  return contract;
}

[[nodiscard]] RoundTripContract termination_matrix_contract() {
  // Each cell is a completed, valid OCCT fillet. The contract preserves support-surface
  // families and topology counts without promising a particular STEP spline wrapper or
  // edge ordering.
  return RoundTripContract{
      .expected_brep_valid = true,
      .topology_counts = true,
      .geometry_families = true,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = true,
      .tolerances = true,
      .accepted_normalization_codes =
          {
              "step.blend_bspline_parameterization_canonicalized",
              "step.subshape_order_canonicalized",
          },
  };
}

[[nodiscard]] RoundTripContract invalid_termination_matrix_contract() {
  RoundTripContract contract = termination_matrix_contract();
  contract.expected_brep_valid = false;
  contract.topology_counts = false;
  contract.geometry_families = false;
  contract.model_topology = false;
  contract.tolerances = false;
  contract.accepted_normalization_codes.push_back("step.fillet_termination_matrix_import_invalid");
  contract.accepted_normalization_codes.push_back("step.fillet_termination_topology_resegmented");
  contract.accepted_normalization_codes.push_back("step.fillet_termination_curves_canonicalized");
  contract.accepted_normalization_codes.push_back("step.tolerances_recomputed_from_uncertainty");
  return contract;
}

[[nodiscard]] TopoDS_Shape fuse_shapes(const TopoDS_Shape& left, const TopoDS_Shape& right,
                                       const std::string_view fixture_id) {
  BRepAlgoAPI_Fuse fuse(left, right);
  fuse.SetRunParallel(false);
  fuse.SetFuzzyValue(0.0);
  fuse.SetNonDestructive(true);
  fuse.Build();
  if (!fuse.IsDone() || fuse.HasErrors() || fuse.Shape().IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": Boolean fuse failed");
  }
  return require_valid_feature(fuse.Shape(), fixture_id);
}

[[nodiscard]] TopoDS_Shape cut_shape(const TopoDS_Shape& argument, const TopoDS_Shape& tool,
                                     const std::string_view fixture_id) {
  BRepAlgoAPI_Cut cut(argument, tool);
  cut.SetRunParallel(false);
  cut.SetFuzzyValue(0.0);
  cut.SetNonDestructive(true);
  cut.Build();
  if (!cut.IsDone() || cut.HasErrors() || cut.Shape().IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": Boolean cut failed");
  }
  return require_valid_feature(cut.Shape(), fixture_id);
}

[[nodiscard]] int subshape_count(const TopoDS_Shape& shape, const TopAbs_ShapeEnum kind) {
  int result = 0;
  for (TopExp_Explorer explorer(shape, kind); explorer.More(); explorer.Next()) {
    ++result;
  }
  return result;
}

template <typename Predicate>
[[nodiscard]] std::vector<TopoDS_Edge> select_edges(const TopoDS_Shape& shape,
                                                    Predicate predicate) {
  std::vector<TopoDS_Edge> result;
  for (TopExp_Explorer explorer(shape, TopAbs_EDGE); explorer.More(); explorer.Next()) {
    const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
    if (std::any_of(result.begin(), result.end(),
                    [&edge](const TopoDS_Edge& known) { return known.IsPartner(edge); })) {
      continue;
    }
    TopoDS_Vertex first;
    TopoDS_Vertex last;
    TopExp::Vertices(edge, first, last, true);
    if (first.IsNull() || last.IsNull()) {
      continue;
    }
    const gp_Pnt first_point = BRep_Tool::Pnt(first);
    const gp_Pnt last_point = BRep_Tool::Pnt(last);
    if (predicate(first_point, last_point)) {
      result.push_back(edge);
    }
  }
  return result;
}

[[nodiscard]] std::vector<TopoDS_Edge>
unique_edges(const NCollection_List<TopoDS_Shape>& edge_occurrences) {
  std::vector<TopoDS_Edge> result;
  for (const TopoDS_Shape& occurrence : edge_occurrences) {
    const TopoDS_Edge edge = TopoDS::Edge(occurrence);
    if (std::none_of(result.begin(), result.end(),
                     [&edge](const TopoDS_Edge& known) { return known.IsPartner(edge); })) {
      result.push_back(edge);
    }
  }
  return result;
}

[[nodiscard]] gp_Pnt edge_midpoint(const TopoDS_Edge& edge) {
  TopoDS_Vertex first;
  TopoDS_Vertex last;
  TopExp::Vertices(edge, first, last, true);
  if (first.IsNull() || last.IsNull()) {
    throw ConstructionFailure("feature edge does not have two finite vertices");
  }
  const gp_Pnt first_point = BRep_Tool::Pnt(first);
  const gp_Pnt last_point = BRep_Tool::Pnt(last);
  return gp_Pnt(0.5 * (first_point.X() + last_point.X()), 0.5 * (first_point.Y() + last_point.Y()),
                0.5 * (first_point.Z() + last_point.Z()));
}

[[nodiscard]] TopoDS_Shape make_coplanar_split_box(const double length_x, const double length_y,
                                                   const double length_z, const double split_x,
                                                   const std::string_view fixture_id) {
  if (!(split_x > 0.0 && split_x < length_x)) {
    throw ConstructionFailure(std::string(fixture_id) + ": split location must lie inside the box");
  }
  BRepPrimAPI_MakeBox left_builder(split_x, length_y, length_z);
  BRepPrimAPI_MakeBox right_builder(gp_Pnt(split_x, 0.0, 0.0), length_x - split_x, length_y,
                                    length_z);
  const TopoDS_Shape left = checked_builder_shape(left_builder, fixture_id);
  const TopoDS_Shape right = checked_builder_shape(right_builder, fixture_id);
  const TopoDS_Shape split = fuse_shapes(left, right, fixture_id);
  if (subshape_count(split, TopAbs_SOLID) != 1 || subshape_count(split, TopAbs_FACE) <= 6) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": Boolean result did not retain coplanar face splits");
  }
  return split;
}

[[nodiscard]] TopoDS_Shape
make_coplanar_split(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.planar.coplanar_split";
  return make_coplanar_split_box(construction_parameter(parameters, "length_x"),
                                 construction_parameter(parameters, "length_y"),
                                 construction_parameter(parameters, "length_z"),
                                 construction_parameter(parameters, "split_x"), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_coaxial_cylinder_cone_transition(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.stepped_coaxial.cylinder_cone";
  const double cylinder_height = construction_parameter(parameters, "cylinder_height");
  const double cylinder_radius = construction_parameter(parameters, "cylinder_radius");
  const double cone_height = construction_parameter(parameters, "cone_height");
  const double top_radius = construction_parameter(parameters, "top_radius");
  const gp_Dir axis_direction(0.0, 0.0, 1.0);

  BRepPrimAPI_MakeCylinder cylinder_builder(cylinder_radius, cylinder_height);
  BRepPrimAPI_MakeCone cone_builder(gp_Ax2(gp_Pnt(0.0, 0.0, cylinder_height), axis_direction),
                                    cylinder_radius, top_radius, cone_height);
  const TopoDS_Shape cylinder = checked_builder_shape(cylinder_builder, fixture_id);
  const TopoDS_Shape cone = checked_builder_shape(cone_builder, fixture_id);
  const TopoDS_Shape result = fuse_shapes(cylinder, cone, fixture_id);

  bool saw_cylinder = false;
  bool saw_cone = false;
  for (TopExp_Explorer explorer(result, TopAbs_FACE); explorer.More(); explorer.Next()) {
    const GeomAbs_SurfaceType kind =
        BRepAdaptor_Surface(TopoDS::Face(explorer.Current()), true).GetType();
    saw_cylinder = saw_cylinder || kind == GeomAbs_Cylinder;
    saw_cone = saw_cone || kind == GeomAbs_Cone;
  }
  if (subshape_count(result, TopAbs_SOLID) != 1 || !saw_cylinder || !saw_cone) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": result lacks the exact coaxial cylinder/cone transition");
  }
  return result;
}

[[nodiscard]] TopoDS_Shape
make_profiled_revolution(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.revolution.profiled";
  const double height_1 = construction_parameter(parameters, "height_1");
  const double height_2 = construction_parameter(parameters, "height_2");
  const double inner_radius = construction_parameter(parameters, "inner_radius");
  const double lower_outer = construction_parameter(parameters, "lower_outer_radius");
  const double upper_outer = construction_parameter(parameters, "upper_outer_radius");
  if (!(inner_radius > 0.0 && lower_outer > inner_radius && upper_outer > inner_radius)) {
    throw ConstructionFailure(std::string(fixture_id) + ": invalid radial profile");
  }

  BRepBuilderAPI_MakePolygon profile;
  profile.Add(gp_Pnt(inner_radius, 0.0, 0.0));
  profile.Add(gp_Pnt(lower_outer, 0.0, 0.0));
  profile.Add(gp_Pnt(lower_outer, 0.0, height_1));
  profile.Add(gp_Pnt(upper_outer, 0.0, height_1 + height_2));
  profile.Add(gp_Pnt(inner_radius, 0.0, height_1 + height_2));
  profile.Close();
  if (!profile.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": profile wire failed");
  }
  BRepBuilderAPI_MakeFace face_builder(profile.Wire(), true);
  const TopoDS_Shape profile_face = checked_builder_shape(face_builder, fixture_id);
  BRepPrimAPI_MakeRevol revol_builder(profile_face,
                                      gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                                      2.0 * std::numbers::pi, true);
  return require_valid_feature(checked_builder_shape(revol_builder, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_slot_pocket_rib(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.features.slot_pocket_rib";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double thickness = construction_parameter(parameters, "thickness");
  const double overcut = construction_parameter(parameters, "overcut");
  const double slot_depth = construction_parameter(parameters, "slot_depth");
  const double slot_radius = construction_parameter(parameters, "slot_radius");
  const double slot_x_1 = construction_parameter(parameters, "slot_x_1");
  const double slot_x_2 = construction_parameter(parameters, "slot_x_2");
  const double slot_y = construction_parameter(parameters, "slot_y");

  BRepPrimAPI_MakeBox base_builder(length_x, length_y, thickness);
  TopoDS_Shape result = checked_builder_shape(base_builder, fixture_id);
  const double cutter_z = thickness - slot_depth;
  BRepPrimAPI_MakeBox slot_bridge_builder(gp_Pnt(slot_x_1, slot_y - slot_radius, cutter_z),
                                          slot_x_2 - slot_x_1, 2.0 * slot_radius,
                                          slot_depth + overcut);
  BRepPrimAPI_MakeCylinder slot_first_builder(
      gp_Ax2(gp_Pnt(slot_x_1, slot_y, cutter_z), gp_Dir(0.0, 0.0, 1.0)), slot_radius,
      slot_depth + overcut);
  BRepPrimAPI_MakeCylinder slot_second_builder(
      gp_Ax2(gp_Pnt(slot_x_2, slot_y, cutter_z), gp_Dir(0.0, 0.0, 1.0)), slot_radius,
      slot_depth + overcut);
  for (const TopoDS_Shape& cutter : std::array<TopoDS_Shape, 3>{
           checked_builder_shape(slot_bridge_builder, fixture_id),
           checked_builder_shape(slot_first_builder, fixture_id),
           checked_builder_shape(slot_second_builder, fixture_id),
       }) {
    result = cut_shape(result, cutter, fixture_id);
  }

  BRepPrimAPI_MakeBox pocket_builder(
      gp_Pnt(construction_parameter(parameters, "pocket_x"),
             construction_parameter(parameters, "pocket_y"),
             thickness - construction_parameter(parameters, "pocket_depth")),
      construction_parameter(parameters, "pocket_length_x"),
      construction_parameter(parameters, "pocket_length_y"),
      construction_parameter(parameters, "pocket_depth") + overcut);
  result = cut_shape(result, checked_builder_shape(pocket_builder, fixture_id), fixture_id);

  BRepPrimAPI_MakeBox rib_builder(gp_Pnt(construction_parameter(parameters, "rib_x"),
                                         construction_parameter(parameters, "rib_y"), thickness),
                                  construction_parameter(parameters, "rib_length_x"),
                                  construction_parameter(parameters, "rib_width"),
                                  construction_parameter(parameters, "rib_height"));
  return fuse_shapes(result, checked_builder_shape(rib_builder, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_counterbore_countersink_boss(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.features.counterbore_countersink_boss";
  const double thickness = construction_parameter(parameters, "thickness");
  const double overcut = construction_parameter(parameters, "overcut");
  const gp_Dir axis(0.0, 0.0, 1.0);
  BRepPrimAPI_MakeBox base_builder(construction_parameter(parameters, "length_x"),
                                   construction_parameter(parameters, "length_y"), thickness);
  TopoDS_Shape result = checked_builder_shape(base_builder, fixture_id);

  const double counterbore_x = construction_parameter(parameters, "counterbore_x");
  const double counterbore_y = construction_parameter(parameters, "counterbore_y");
  const double through_radius = construction_parameter(parameters, "through_radius");
  BRepPrimAPI_MakeCylinder counterbore_through_builder(
      gp_Ax2(gp_Pnt(counterbore_x, counterbore_y, -overcut), axis), through_radius,
      thickness + (2.0 * overcut));
  BRepPrimAPI_MakeCylinder counterbore_recess_builder(
      gp_Ax2(gp_Pnt(counterbore_x, counterbore_y,
                    thickness - construction_parameter(parameters, "counterbore_depth")),
             axis),
      construction_parameter(parameters, "counterbore_radius"),
      construction_parameter(parameters, "counterbore_depth") + overcut);
  result =
      cut_shape(result, checked_builder_shape(counterbore_through_builder, fixture_id), fixture_id);
  result =
      cut_shape(result, checked_builder_shape(counterbore_recess_builder, fixture_id), fixture_id);

  const double countersink_x = construction_parameter(parameters, "countersink_x");
  const double countersink_y = construction_parameter(parameters, "countersink_y");
  const double countersink_depth = construction_parameter(parameters, "countersink_depth");
  BRepPrimAPI_MakeCylinder countersink_through_builder(
      gp_Ax2(gp_Pnt(countersink_x, countersink_y, -overcut), axis), through_radius,
      thickness + (2.0 * overcut));
  BRepPrimAPI_MakeCone countersink_recess_builder(
      gp_Ax2(gp_Pnt(countersink_x, countersink_y, thickness - countersink_depth), axis),
      through_radius, construction_parameter(parameters, "countersink_radius"), countersink_depth);
  result =
      cut_shape(result, checked_builder_shape(countersink_through_builder, fixture_id), fixture_id);
  result =
      cut_shape(result, checked_builder_shape(countersink_recess_builder, fixture_id), fixture_id);

  BRepPrimAPI_MakeCylinder boss_builder(
      gp_Ax2(gp_Pnt(construction_parameter(parameters, "boss_x"),
                    construction_parameter(parameters, "boss_y"), thickness),
             axis),
      construction_parameter(parameters, "boss_radius"),
      construction_parameter(parameters, "boss_height"));
  return fuse_shapes(result, checked_builder_shape(boss_builder, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Face highest_face(const TopoDS_Shape& shape,
                                       const std::string_view fixture_id) {
  double highest_z = -std::numeric_limits<double>::infinity();
  TopoDS_Face result;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    GProp_GProps properties;
    BRepGProp::SurfaceProperties(face, properties);
    if (properties.CentreOfMass().Z() > highest_z) {
      highest_z = properties.CentreOfMass().Z();
      result = face;
    }
  }
  if (result.IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": no face available for shell opening");
  }
  return result;
}

[[nodiscard]] TopoDS_Shape
make_drafted_shell_offset(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.draft_shell_offset";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double length_z = construction_parameter(parameters, "length_z");
  BRepPrimAPI_MakeBox base_builder(length_x, length_y, length_z);
  const TopoDS_Shape base = checked_builder_shape(base_builder, fixture_id);

  BRepOffsetAPI_DraftAngle draft(base);
  int drafted_face_count = 0;
  for (TopExp_Explorer explorer(base, TopAbs_FACE); explorer.More(); explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    BRepAdaptor_Surface surface(face, true);
    if (surface.GetType() != GeomAbs_Plane ||
        std::abs(surface.Plane().Axis().Direction().Z()) > 0.5) {
      continue;
    }
    draft.Add(face, gp_Dir(0.0, 0.0, 1.0), construction_parameter(parameters, "draft_angle"),
              gp_Pln(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), true);
    if (!draft.AddDone()) {
      throw ConstructionFailure(std::string(fixture_id) + ": draft-angle face failed");
    }
    ++drafted_face_count;
  }
  if (drafted_face_count != 4) {
    throw ConstructionFailure(std::string(fixture_id) + ": expected four drafted side faces");
  }
  draft.Build();
  const TopoDS_Shape drafted = checked_builder_shape(draft, fixture_id);

  NCollection_List<TopoDS_Shape> closing_faces;
  closing_faces.Append(highest_face(drafted, fixture_id));
  BRepOffsetAPI_MakeThickSolid shell;
  shell.MakeThickSolidByJoin(drafted, closing_faces,
                             -construction_parameter(parameters, "wall_thickness"), 1.0e-7,
                             BRepOffset_Skin, false, false, GeomAbs_Intersection, true);
  return require_valid_feature(checked_builder_shape(shell, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Face rectangle_face(const double x_min, const double y_min,
                                         const double length_x, const double length_y,
                                         const std::string_view fixture_id) {
  BRepBuilderAPI_MakePolygon polygon;
  polygon.Add(gp_Pnt(x_min, y_min, 0.0));
  polygon.Add(gp_Pnt(x_min + length_x, y_min, 0.0));
  polygon.Add(gp_Pnt(x_min + length_x, y_min + length_y, 0.0));
  polygon.Add(gp_Pnt(x_min, y_min + length_y, 0.0));
  polygon.Close();
  if (!polygon.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": overlap polygon failed");
  }
  BRepBuilderAPI_MakeFace face_builder(polygon.Wire(), true);
  return TopoDS::Face(checked_builder_shape(face_builder, fixture_id));
}

[[nodiscard]] TopoDS_Shape
make_coincident_overlap_faces(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "compound.face.coincident_overlap";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double offset = construction_parameter(parameters, "overlap_offset");
  const TopoDS_Face first = rectangle_face(0.0, 0.0, length_x, length_y, fixture_id);
  const TopoDS_Face second = rectangle_face(0.0, 0.0, length_x, length_y, fixture_id);
  const TopoDS_Face overlap = rectangle_face(offset, offset, length_x, length_y, fixture_id);
  if (first.IsPartner(second)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": coincident faces unexpectedly share a TShape");
  }
  const std::array<TopoDS_Shape, 3> children{first, second, overlap};
  return require_valid_feature(make_compound(children), fixture_id);
}

[[nodiscard]] std::vector<TopoDS_Edge> vertical_box_edges(const TopoDS_Shape& shape) {
  return select_edges(shape, [](const gp_Pnt& first, const gp_Pnt& last) {
    return near(first.X(), last.X()) && near(first.Y(), last.Y()) && !near(first.Z(), last.Z());
  });
}

[[nodiscard]] TopoDS_Shape fillet_edge(const TopoDS_Shape& base, const TopoDS_Edge& edge,
                                       const double radius, const std::string_view fixture_id) {
  if (!(radius > 0.0)) {
    throw ConstructionFailure(std::string(fixture_id) + ": fillet radius must be positive");
  }
  BRepFilletAPI_MakeFillet fillet(base);
  fillet.Add(radius, edge);
  fillet.Build();
  return require_valid_feature(checked_builder_shape(fillet, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
single_vertical_edge_fillet_box(const gp_Pnt& origin, const double length_x, const double length_y,
                                const double length_z, const double radius,
                                const std::string_view fixture_id) {
  BRepPrimAPI_MakeBox box_builder(origin, length_x, length_y, length_z);
  const TopoDS_Shape box = checked_builder_shape(box_builder, fixture_id);
  const std::vector<TopoDS_Edge> vertical_edges = vertical_box_edges(box);
  if (vertical_edges.size() != 4) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": box did not expose four vertical witness edges");
  }
  const auto selected =
      std::min_element(vertical_edges.begin(), vertical_edges.end(),
                       [&origin](const TopoDS_Edge& left, const TopoDS_Edge& right) {
                         return origin.SquareDistance(edge_midpoint(left)) <
                                origin.SquareDistance(edge_midpoint(right));
                       });
  return fillet_edge(box, *selected, radius, fixture_id);
}

[[nodiscard]] std::vector<TopoDS_Edge> corner_edges(const TopoDS_Shape& shape, const gp_Pnt& corner,
                                                    const std::string_view fixture_id) {
  TopoDS_Vertex corner_vertex;
  for (TopExp_Explorer explorer(shape, TopAbs_VERTEX); explorer.More(); explorer.Next()) {
    const TopoDS_Vertex candidate = TopoDS::Vertex(explorer.Current());
    if (BRep_Tool::Pnt(candidate).Distance(corner) <= kSelectionTolerance) {
      corner_vertex = candidate;
      break;
    }
  }
  if (corner_vertex.IsNull()) {
    throw ConstructionFailure(std::string(fixture_id) + ": requested blend corner was not found");
  }

  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
      vertex_to_edges;
  TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertex_to_edges);
  const int corner_index = vertex_to_edges.FindIndex(corner_vertex);
  if (corner_index == 0) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": blend corner must have exactly three incident edges");
  }
  std::vector<TopoDS_Edge> result = unique_edges(vertex_to_edges.FindFromIndex(corner_index));
  if (result.size() != 3) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": blend corner must have exactly three unique incident edges");
  }
  return result;
}

[[nodiscard]] TopoDS_Shape corner_fillet_box(const gp_Pnt& origin, const double length_x,
                                             const double length_y, const double length_z,
                                             const double radius, const std::size_t edge_count,
                                             const std::string_view fixture_id) {
  if (!(radius > 0.0)) {
    throw ConstructionFailure(std::string(fixture_id) + ": fillet radius must be positive");
  }
  BRepPrimAPI_MakeBox box_builder(origin, length_x, length_y, length_z);
  const TopoDS_Shape box = checked_builder_shape(box_builder, fixture_id);
  const std::vector<TopoDS_Edge> incident = corner_edges(box, origin, fixture_id);
  if (edge_count == 0 || edge_count > incident.size()) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": invalid incident-edge count for corner fillet");
  }
  BRepFilletAPI_MakeFillet fillet(box);
  for (std::size_t index = 0; index < edge_count; ++index) {
    fillet.Add(radius, incident[index]);
  }
  fillet.Build();
  return require_valid_feature(checked_builder_shape(fillet, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape shared_vertex_graph(const gp_Pnt& center,
                                               const std::span<const gp_Pnt> endpoints,
                                               const std::string_view fixture_id) {
  if (endpoints.size() < 3) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": a junction witness requires at least three arms");
  }
  BRepBuilderAPI_MakeVertex center_builder(center);
  const TopoDS_Vertex center_vertex =
      TopoDS::Vertex(checked_builder_shape(center_builder, fixture_id));
  std::vector<TopoDS_Shape> arms;
  arms.reserve(endpoints.size());
  for (const gp_Pnt& endpoint : endpoints) {
    if (endpoint.Distance(center) <= kSelectionTolerance) {
      throw ConstructionFailure(std::string(fixture_id) +
                                ": junction witness contains a zero-length arm");
    }
    BRepBuilderAPI_MakeVertex endpoint_builder(endpoint);
    const TopoDS_Vertex endpoint_vertex =
        TopoDS::Vertex(checked_builder_shape(endpoint_builder, fixture_id));
    BRepBuilderAPI_MakeEdge edge_builder(center_vertex, endpoint_vertex);
    arms.push_back(checked_builder_shape(edge_builder, fixture_id));
  }
  return require_valid_feature(make_compound(arms), fixture_id);
}

[[nodiscard]] TopoDS_Edge circular_edge_at_height(const TopoDS_Shape& shape, const double height,
                                                  const std::string_view fixture_id) {
  std::vector<TopoDS_Edge> matches;
  for (TopExp_Explorer explorer(shape, TopAbs_EDGE); explorer.More(); explorer.Next()) {
    const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
    const BRepAdaptor_Curve curve(edge);
    if (curve.GetType() != GeomAbs_Circle || !near(curve.Circle().Location().Z(), height)) {
      continue;
    }
    if (std::none_of(matches.begin(), matches.end(),
                     [&edge](const TopoDS_Edge& known) { return known.IsPartner(edge); })) {
      matches.push_back(edge);
    }
  }
  if (matches.size() != 1) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": expected one circular support edge at the target height");
  }
  return matches.front();
}

[[nodiscard]] TopoDS_Shape
make_constant_variable_fillet(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.fillet.constant_variable";
  BRepPrimAPI_MakeBox base_builder(construction_parameter(parameters, "length_x"),
                                   construction_parameter(parameters, "length_y"),
                                   construction_parameter(parameters, "length_z"));
  const TopoDS_Shape base = checked_builder_shape(base_builder, fixture_id);
  const std::vector<TopoDS_Edge> vertical_edges = vertical_box_edges(base);
  if (vertical_edges.size() != 4) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": box did not expose four vertical fillet edges");
  }
  const TopoDS_Edge constant_edge = vertical_edges.front();
  const gp_Pnt constant_midpoint = edge_midpoint(constant_edge);
  const auto opposite =
      std::max_element(std::next(vertical_edges.begin()), vertical_edges.end(),
                       [&constant_midpoint](const TopoDS_Edge& left, const TopoDS_Edge& right) {
                         return constant_midpoint.SquareDistance(edge_midpoint(left)) <
                                constant_midpoint.SquareDistance(edge_midpoint(right));
                       });

  BRepFilletAPI_MakeFillet fillet(base);
  fillet.Add(construction_parameter(parameters, "constant_radius"), constant_edge);
  fillet.Add(construction_parameter(parameters, "variable_radius_start"),
             construction_parameter(parameters, "variable_radius_end"), *opposite);
  if (fillet.NbContours() != 2 || !fillet.IsConstant(1) || fillet.IsConstant(2)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": constant and variable radius laws were not retained");
  }
  fillet.Build();
  return require_valid_feature(checked_builder_shape(fillet, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_face_edge_corner_blend(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.blend.face_face_face_edge_corner";
  BRepPrimAPI_MakeBox base_builder(construction_parameter(parameters, "length_x"),
                                   construction_parameter(parameters, "length_y"),
                                   construction_parameter(parameters, "length_z"));
  const TopoDS_Shape base = checked_builder_shape(base_builder, fixture_id);

  TopoDS_Vertex corner;
  double coordinate_sum = std::numeric_limits<double>::infinity();
  for (TopExp_Explorer explorer(base, TopAbs_VERTEX); explorer.More(); explorer.Next()) {
    const TopoDS_Vertex vertex = TopoDS::Vertex(explorer.Current());
    const gp_Pnt point = BRep_Tool::Pnt(vertex);
    if (point.X() + point.Y() + point.Z() < coordinate_sum) {
      coordinate_sum = point.X() + point.Y() + point.Z();
      corner = vertex;
    }
  }
  NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>, TopTools_ShapeMapHasher>
      vertex_to_edges;
  TopExp::MapShapesAndAncestors(base, TopAbs_VERTEX, TopAbs_EDGE, vertex_to_edges);
  const int corner_index = vertex_to_edges.FindIndex(corner);
  if (corner_index == 0) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": selected corner does not have three incident edges");
  }

  const std::vector<TopoDS_Edge> incident_edges =
      unique_edges(vertex_to_edges.FindFromIndex(corner_index));
  if (incident_edges.size() != 3) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": selected corner does not have three unique incident edges");
  }

  BRepFilletAPI_MakeFillet fillet(base);
  for (const TopoDS_Edge& edge : incident_edges) {
    fillet.Add(construction_parameter(parameters, "radius"), edge);
  }
  if (fillet.NbContours() != 3) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": trihedral corner did not create three blend contours");
  }
  fillet.Build();
  return require_valid_feature(checked_builder_shape(fillet, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_split_fillet_chain(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.fillet.chain_split_mismatch";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double length_z = construction_parameter(parameters, "length_z");
  const TopoDS_Shape split = make_coplanar_split_box(
      length_x, length_y, length_z, construction_parameter(parameters, "split_x"), fixture_id);
  const std::vector<TopoDS_Edge> chain_edges =
      select_edges(split, [length_z](const gp_Pnt& first, const gp_Pnt& last) {
        return near(first.Y(), 0.0) && near(last.Y(), 0.0) && near(first.Z(), length_z) &&
               near(last.Z(), length_z) && !near(first.X(), last.X());
      });
  if (chain_edges.size() < 2) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": coplanar split did not expose a split fillet chain");
  }

  BRepFilletAPI_MakeFillet fillet(split);
  fillet.Add(construction_parameter(parameters, "fillet_radius"), chain_edges.front());
  if (fillet.NbContours() != 1 || fillet.NbEdges(1) < 2) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": OCCT did not propagate across the raw split chain");
  }
  fillet.Build();
  return require_valid_feature(checked_builder_shape(fillet, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Wire triangular_chamfer_section(const double x, const double width,
                                                     const double height,
                                                     const std::string_view fixture_id) {
  BRepBuilderAPI_MakePolygon polygon;
  polygon.Add(gp_Pnt(x, 0.0, height));
  polygon.Add(gp_Pnt(x, width, height));
  polygon.Add(gp_Pnt(x, 0.0, height - width));
  polygon.Close();
  if (!polygon.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": variable chamfer section failed");
  }
  return polygon.Wire();
}

[[nodiscard]] TopoDS_Shape
make_constant_variable_chamfer(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.chamfer.constant_variable";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double length_z = construction_parameter(parameters, "length_z");
  BRepPrimAPI_MakeBox base_builder(length_x, length_y, length_z);
  const TopoDS_Shape base = checked_builder_shape(base_builder, fixture_id);
  const std::vector<TopoDS_Edge> constant_edges =
      select_edges(base, [length_y, length_z](const gp_Pnt& first, const gp_Pnt& last) {
        return near(first.Y(), length_y) && near(last.Y(), length_y) && near(first.Z(), length_z) &&
               near(last.Z(), length_z) && !near(first.X(), last.X());
      });
  if (constant_edges.size() != 1) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": constant chamfer edge selection is not unique");
  }
  BRepFilletAPI_MakeChamfer constant_chamfer(base);
  constant_chamfer.Add(construction_parameter(parameters, "constant_distance"),
                       constant_edges.front());
  constant_chamfer.Build();
  const TopoDS_Shape constant_result = checked_builder_shape(constant_chamfer, fixture_id);

  const double overcut = construction_parameter(parameters, "wedge_overcut");
  const double start_width = construction_parameter(parameters, "variable_distance_start");
  const double end_width = construction_parameter(parameters, "variable_distance_end");
  const double width_gradient = (end_width - start_width) / length_x;
  const double extended_start_width = start_width - (width_gradient * overcut);
  const double extended_end_width = end_width + (width_gradient * overcut);
  if (!(extended_start_width > 0.0 && extended_end_width > 0.0)) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": extended variable chamfer width is not positive");
  }
  BRepOffsetAPI_ThruSections variable_wedge(true, true, 1.0e-7);
  variable_wedge.AddWire(
      triangular_chamfer_section(-overcut, extended_start_width, length_z, fixture_id));
  variable_wedge.AddWire(
      triangular_chamfer_section(length_x + overcut, extended_end_width, length_z, fixture_id));
  variable_wedge.Build();
  const TopoDS_Shape wedge = checked_builder_shape(variable_wedge, fixture_id);
  return cut_shape(constant_result, wedge, fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_rolling_ball_channel(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.fillet.rolling_ball_channel";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double length_z = construction_parameter(parameters, "length_z");
  const double margin_x = construction_parameter(parameters, "margin_x");
  const double channel_width = construction_parameter(parameters, "channel_width");
  const double channel_depth = construction_parameter(parameters, "channel_depth");
  const double channel_y = 0.5 * (length_y - channel_width);
  const double channel_bottom = length_z - channel_depth;

  BRepPrimAPI_MakeBox base_builder(length_x, length_y, length_z);
  BRepPrimAPI_MakeBox channel_builder(
      gp_Pnt(margin_x, channel_y, channel_bottom), length_x - (2.0 * margin_x), channel_width,
      channel_depth + construction_parameter(parameters, "overcut"));
  const TopoDS_Shape pocketed =
      cut_shape(checked_builder_shape(base_builder, fixture_id),
                checked_builder_shape(channel_builder, fixture_id), fixture_id);
  const std::vector<TopoDS_Edge> bottom_edges =
      select_edges(pocketed, [channel_bottom](const gp_Pnt& first, const gp_Pnt& last) {
        return near(first.Z(), channel_bottom) && near(last.Z(), channel_bottom) &&
               near(first.Y(), last.Y()) && !near(first.X(), last.X());
      });
  if (bottom_edges.size() != 2) {
    throw ConstructionFailure(std::string(fixture_id) +
                              ": pocket did not expose two channel-bottom edges");
  }
  BRepFilletAPI_MakeFillet fillet(pocketed, ChFi3d_Rational);
  for (const TopoDS_Edge& edge : bottom_edges) {
    fillet.Add(construction_parameter(parameters, "fillet_radius"), edge);
  }
  fillet.Build();
  return require_valid_feature(checked_builder_shape(fillet, fixture_id), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_setback_multiway_witness(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.blend.setback_multiway";
  const double length_x = construction_parameter(parameters, "length_x");
  const double length_y = construction_parameter(parameters, "length_y");
  const double length_z = construction_parameter(parameters, "length_z");
  const TopoDS_Shape corner_blend =
      corner_fillet_box(gp_Pnt(0.0, 0.0, 0.0), length_x, length_y, length_z,
                        construction_parameter(parameters, "corner_radius"), 3, fixture_id);

  // The degree-four graph records the exact rail incidence and unequal setback
  // distances supplied to a future named cap. It is deliberately free topology,
  // not a fabricated claim that the base mesher supports a multi-way cap.
  const gp_Pnt center(length_x + construction_parameter(parameters, "graph_clearance") +
                          construction_parameter(parameters, "setback_negative_x"),
                      0.5 * length_y, 0.5 * length_z);
  const std::array endpoints{
      gp_Pnt(center.X() - construction_parameter(parameters, "setback_negative_x"), center.Y(),
             center.Z()),
      gp_Pnt(center.X() + construction_parameter(parameters, "setback_positive_x"), center.Y(),
             center.Z()),
      gp_Pnt(center.X(), center.Y() + construction_parameter(parameters, "setback_positive_y"),
             center.Z()),
      gp_Pnt(center.X(), center.Y(),
             center.Z() + construction_parameter(parameters, "setback_positive_z")),
  };
  const TopoDS_Shape rail_graph = shared_vertex_graph(center, endpoints, fixture_id);
  const std::array<TopoDS_Shape, 2> children{corner_blend, rail_graph};
  return require_valid_feature(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_junction_t_y_x_witness(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.fillet.junction_t_y_x";
  const double length_x = construction_parameter(parameters, "solid_length_x");
  const double length_y = construction_parameter(parameters, "solid_length_y");
  const double length_z = construction_parameter(parameters, "solid_length_z");
  const double spacing = construction_parameter(parameters, "column_spacing");
  const double arm = construction_parameter(parameters, "junction_arm_length");
  const double graph_z = length_z + construction_parameter(parameters, "graph_clearance_z");

  const TopoDS_Shape t_strip = single_vertical_edge_fillet_box(
      gp_Pnt(0.0, 0.0, 0.0), length_x, length_y, length_z,
      construction_parameter(parameters, "t_strip_radius"), fixture_id);
  const TopoDS_Shape y_strip = single_vertical_edge_fillet_box(
      gp_Pnt(spacing, 0.0, 0.0), length_x, length_y, length_z,
      construction_parameter(parameters, "y_strip_radius"), fixture_id);
  const TopoDS_Shape x_strip = single_vertical_edge_fillet_box(
      gp_Pnt(2.0 * spacing, 0.0, 0.0), length_x, length_y, length_z,
      construction_parameter(parameters, "x_strip_radius"), fixture_id);

  const gp_Pnt t_center(0.5 * length_x, 0.5 * length_y, graph_z);
  const std::array t_endpoints{
      gp_Pnt(t_center.X() - arm, t_center.Y(), graph_z),
      gp_Pnt(t_center.X() + arm, t_center.Y(), graph_z),
      gp_Pnt(t_center.X(), t_center.Y() + arm, graph_z),
  };
  const TopoDS_Shape t_graph = shared_vertex_graph(t_center, t_endpoints, fixture_id);

  const gp_Pnt y_center(spacing + (0.5 * length_x), 0.5 * length_y, graph_z);
  const double y_half_height = 0.5 * std::sqrt(3.0) * arm;
  const std::array y_endpoints{
      gp_Pnt(y_center.X() + arm, y_center.Y(), graph_z),
      gp_Pnt(y_center.X() - (0.5 * arm), y_center.Y() + y_half_height, graph_z),
      gp_Pnt(y_center.X() - (0.5 * arm), y_center.Y() - y_half_height, graph_z),
  };
  const TopoDS_Shape y_graph = shared_vertex_graph(y_center, y_endpoints, fixture_id);

  const gp_Pnt x_center((2.0 * spacing) + (0.5 * length_x), 0.5 * length_y, graph_z);
  const std::array x_endpoints{
      gp_Pnt(x_center.X() - arm, x_center.Y(), graph_z),
      gp_Pnt(x_center.X() + arm, x_center.Y(), graph_z),
      gp_Pnt(x_center.X(), x_center.Y() - arm, graph_z),
      gp_Pnt(x_center.X(), x_center.Y() + arm, graph_z),
  };
  const TopoDS_Shape x_graph = shared_vertex_graph(x_center, x_endpoints, fixture_id);

  const std::array<TopoDS_Shape, 6> children{
      t_strip, t_graph, y_strip, y_graph, x_strip, x_graph,
  };
  return require_valid_feature(make_compound(children), fixture_id);
}

[[nodiscard]] TopoDS_Shape
make_fillet_termination_matrix(const std::span<const ConstructionParameter> parameters) {
  constexpr std::string_view fixture_id = "solid.fillet.termination_matrix";
  const double spacing = construction_parameter(parameters, "column_spacing");
  const double height = construction_parameter(parameters, "height");

  const TopoDS_Shape planar_termination = single_vertical_edge_fillet_box(
      gp_Pnt(0.0, 0.0, 0.0), construction_parameter(parameters, "box_length_x"),
      construction_parameter(parameters, "box_length_y"), height,
      construction_parameter(parameters, "plane_support_radius"), fixture_id);

  const gp_Pnt cylinder_origin(spacing, 0.0, 0.0);
  BRepPrimAPI_MakeCylinder cylinder_builder(gp_Ax2(cylinder_origin, gp_Dir(0.0, 0.0, 1.0)),
                                            construction_parameter(parameters, "cylinder_radius"),
                                            height);
  const TopoDS_Shape cylinder = checked_builder_shape(cylinder_builder, fixture_id);
  const TopoDS_Shape cylinder_termination =
      fillet_edge(cylinder, circular_edge_at_height(cylinder, height, fixture_id),
                  construction_parameter(parameters, "cylinder_support_fillet_radius"), fixture_id);

  const gp_Pnt cone_origin(2.0 * spacing, 0.0, 0.0);
  BRepPrimAPI_MakeCone cone_builder(gp_Ax2(cone_origin, gp_Dir(0.0, 0.0, 1.0)),
                                    construction_parameter(parameters, "cone_bottom_radius"),
                                    construction_parameter(parameters, "cone_top_radius"), height);
  const TopoDS_Shape cone = checked_builder_shape(cone_builder, fixture_id);
  const TopoDS_Shape cone_termination =
      fillet_edge(cone, circular_edge_at_height(cone, height, fixture_id),
                  construction_parameter(parameters, "cone_support_fillet_radius"), fixture_id);

  const TopoDS_Shape fillet_to_fillet = corner_fillet_box(
      gp_Pnt(3.0 * spacing, 0.0, 0.0), construction_parameter(parameters, "box_length_x"),
      construction_parameter(parameters, "box_length_y"), height,
      construction_parameter(parameters, "fillet_support_radius"), 2, fixture_id);

  const std::array<TopoDS_Shape, 4> children{
      planar_termination,
      cylinder_termination,
      cone_termination,
      fillet_to_fillet,
  };
  return require_valid_feature(make_compound(children), fixture_id);
}

} // namespace

void append_feature_recipes(std::vector<Recipe>& recipes) {
  recipes.push_back(procedural_shape_recipe("compound.face.coincident_overlap",
                                            {
                                                {"length_x", 18.0, "millimetres"},
                                                {"length_y", 12.0, "millimetres"},
                                                {"overlap_offset", 6.0, "millimetres"},
                                            },
                                            make_coincident_overlap_faces, StepIoProfile{},
                                            standalone_face_compound_contract()));
  recipes.push_back(procedural_shape_recipe(
      "solid.blend.face_face_face_edge_corner",
      {
          {"length_x", 30.0, "millimetres"},
          {"length_y", 24.0, "millimetres"},
          {"length_z", 20.0, "millimetres"},
          {"radius", 2.0, "millimetres"},
      },
      make_face_edge_corner_blend, StepIoProfile{},
      invalid_named_blend_feature_contract("step.trihedral_blend_import_invalid")));
  recipes.push_back(procedural_shape_recipe(
      "solid.blend.setback_multiway",
      {
          {"corner_radius", 2.0, "millimetres"},
          {"graph_clearance", 12.0, "millimetres"},
          {"length_x", 32.0, "millimetres"},
          {"length_y", 26.0, "millimetres"},
          {"length_z", 22.0, "millimetres"},
          {"setback_negative_x", 4.0, "millimetres"},
          {"setback_positive_x", 7.0, "millimetres"},
          {"setback_positive_y", 5.0, "millimetres"},
          {"setback_positive_z", 6.0, "millimetres"},
      },
      make_setback_multiway_witness, StepIoProfile{},
      free_graph_blend_topology_witness_contract(false, "step.setback_witness_import_invalid")));
  recipes.push_back(procedural_shape_recipe("solid.chamfer.constant_variable",
                                            {
                                                {"constant_distance", 2.0, "millimetres"},
                                                {"length_x", 36.0, "millimetres"},
                                                {"length_y", 24.0, "millimetres"},
                                                {"length_z", 18.0, "millimetres"},
                                                {"variable_distance_end", 4.0, "millimetres"},
                                                {"variable_distance_start", 1.0, "millimetres"},
                                                {"wedge_overcut", 1.0, "millimetres"},
                                            },
                                            make_constant_variable_chamfer, StepIoProfile{},
                                            invalid_variable_chamfer_contract()));
  recipes.push_back(procedural_shape_recipe("solid.draft_shell_offset",
                                            {
                                                {"draft_angle", std::numbers::pi / 60.0, "radians"},
                                                {"length_x", 36.0, "millimetres"},
                                                {"length_y", 28.0, "millimetres"},
                                                {"length_z", 24.0, "millimetres"},
                                                {"wall_thickness", 2.0, "millimetres"},
                                            },
                                            make_drafted_shell_offset, StepIoProfile{},
                                            tolerance_offset_feature_contract()));
  recipes.push_back(procedural_shape_recipe("solid.features.counterbore_countersink_boss",
                                            {
                                                {"boss_height", 5.0, "millimetres"},
                                                {"boss_radius", 5.0, "millimetres"},
                                                {"boss_x", 50.0, "millimetres"},
                                                {"boss_y", 18.0, "millimetres"},
                                                {"counterbore_depth", 3.0, "millimetres"},
                                                {"counterbore_radius", 5.0, "millimetres"},
                                                {"counterbore_x", 15.0, "millimetres"},
                                                {"counterbore_y", 18.0, "millimetres"},
                                                {"countersink_depth", 4.0, "millimetres"},
                                                {"countersink_radius", 6.0, "millimetres"},
                                                {"countersink_x", 32.0, "millimetres"},
                                                {"countersink_y", 18.0, "millimetres"},
                                                {"length_x", 64.0, "millimetres"},
                                                {"length_y", 36.0, "millimetres"},
                                                {"overcut", 1.0, "millimetres"},
                                                {"thickness", 10.0, "millimetres"},
                                                {"through_radius", 2.5, "millimetres"},
                                            },
                                            make_counterbore_countersink_boss, StepIoProfile{},
                                            boolean_feature_contract()));
  recipes.push_back(procedural_shape_recipe(
      "solid.features.slot_pocket_rib",
      {
          {"length_x", 70.0, "millimetres"},        {"length_y", 42.0, "millimetres"},
          {"overcut", 1.0, "millimetres"},          {"pocket_depth", 4.0, "millimetres"},
          {"pocket_length_x", 14.0, "millimetres"}, {"pocket_length_y", 12.0, "millimetres"},
          {"pocket_x", 48.0, "millimetres"},        {"pocket_y", 6.0, "millimetres"},
          {"rib_height", 6.0, "millimetres"},       {"rib_length_x", 24.0, "millimetres"},
          {"rib_width", 3.0, "millimetres"},        {"rib_x", 38.0, "millimetres"},
          {"rib_y", 32.0, "millimetres"},           {"slot_depth", 5.0, "millimetres"},
          {"slot_radius", 4.0, "millimetres"},      {"slot_x_1", 14.0, "millimetres"},
          {"slot_x_2", 30.0, "millimetres"},        {"slot_y", 15.0, "millimetres"},
          {"thickness", 10.0, "millimetres"},
      },
      make_slot_pocket_rib, StepIoProfile{}, tolerance_boolean_feature_contract()));
  recipes.push_back(procedural_shape_recipe("solid.fillet.chain_split_mismatch",
                                            {
                                                {"fillet_radius", 2.0, "millimetres"},
                                                {"length_x", 40.0, "millimetres"},
                                                {"length_y", 24.0, "millimetres"},
                                                {"length_z", 18.0, "millimetres"},
                                                {"split_x", 17.0, "millimetres"},
                                            },
                                            make_split_fillet_chain, StepIoProfile{},
                                            invalid_blend_feature_contract()));
  recipes.push_back(procedural_shape_recipe("solid.fillet.constant_variable",
                                            {
                                                {"constant_radius", 2.0, "millimetres"},
                                                {"length_x", 34.0, "millimetres"},
                                                {"length_y", 28.0, "millimetres"},
                                                {"length_z", 24.0, "millimetres"},
                                                {"variable_radius_end", 3.0, "millimetres"},
                                                {"variable_radius_start", 1.0, "millimetres"},
                                            },
                                            make_constant_variable_fillet, StepIoProfile{},
                                            compound_root_blend_feature_contract()));
  recipes.push_back(procedural_shape_recipe(
      "solid.fillet.rolling_ball_channel",
      {
          {"channel_depth", 8.0, "millimetres"},
          {"channel_width", 12.0, "millimetres"},
          {"fillet_radius", 2.0, "millimetres"},
          {"length_x", 48.0, "millimetres"},
          {"length_y", 30.0, "millimetres"},
          {"length_z", 18.0, "millimetres"},
          {"margin_x", 6.0, "millimetres"},
          {"overcut", 1.0, "millimetres"},
      },
      make_rolling_ball_channel, StepIoProfile{},
      invalid_named_blend_feature_contract("step.rolling_ball_channel_import_invalid", true)));
  recipes.push_back(procedural_shape_recipe(
      "solid.fillet.junction_t_y_x",
      {
          {"column_spacing", 42.0, "millimetres"},
          {"graph_clearance_z", 10.0, "millimetres"},
          {"junction_arm_length", 7.0, "millimetres"},
          {"solid_length_x", 28.0, "millimetres"},
          {"solid_length_y", 22.0, "millimetres"},
          {"solid_length_z", 18.0, "millimetres"},
          {"t_strip_radius", 1.5, "millimetres"},
          {"x_strip_radius", 2.5, "millimetres"},
          {"y_strip_radius", 2.0, "millimetres"},
      },
      make_junction_t_y_x_witness, StepIoProfile{},
      free_graph_blend_topology_witness_contract(true, std::string_view{})));
  recipes.push_back(procedural_shape_recipe(
      "solid.fillet.termination_matrix",
      {
          {"box_length_x", 24.0, "millimetres"},
          {"box_length_y", 20.0, "millimetres"},
          {"column_spacing", 38.0, "millimetres"},
          {"cone_bottom_radius", 8.0, "millimetres"},
          {"cone_support_fillet_radius", 0.8, "millimetres"},
          {"cone_top_radius", 5.0, "millimetres"},
          {"cylinder_radius", 7.0, "millimetres"},
          {"cylinder_support_fillet_radius", 1.0, "millimetres"},
          {"fillet_support_radius", 1.5, "millimetres"},
          {"height", 18.0, "millimetres"},
          {"plane_support_radius", 2.0, "millimetres"},
      },
      make_fillet_termination_matrix, StepIoProfile{}, invalid_termination_matrix_contract()));
  recipes.push_back(procedural_shape_recipe("solid.planar.coplanar_split",
                                            {
                                                {"length_x", 32.0, "millimetres"},
                                                {"length_y", 20.0, "millimetres"},
                                                {"length_z", 14.0, "millimetres"},
                                                {"split_x", 13.0, "millimetres"},
                                            },
                                            make_coplanar_split, StepIoProfile{},
                                            boolean_feature_contract()));
  recipes.push_back(procedural_shape_recipe("solid.revolution.profiled",
                                            {
                                                {"height_1", 12.0, "millimetres"},
                                                {"height_2", 10.0, "millimetres"},
                                                {"inner_radius", 5.0, "millimetres"},
                                                {"lower_outer_radius", 12.0, "millimetres"},
                                                {"upper_outer_radius", 18.0, "millimetres"},
                                            },
                                            make_profiled_revolution, StepIoProfile{},
                                            invalid_revolution_feature_contract()));
  recipes.push_back(procedural_shape_recipe("solid.stepped_coaxial.cylinder_cone",
                                            {
                                                {"cone_height", 14.0, "millimetres"},
                                                {"cylinder_height", 16.0, "millimetres"},
                                                {"cylinder_radius", 10.0, "millimetres"},
                                                {"top_radius", 6.0, "millimetres"},
                                            },
                                            make_coaxial_cylinder_cone_transition, StepIoProfile{},
                                            invalid_boolean_feature_contract()));

  // `solid.nurbs.recovered_analytic` remains unregistered because it needs an
  // independent post-import residual oracle rather than an OCCT-only construction.
}

} // namespace cad_mesher::fixtures
