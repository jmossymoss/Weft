#include "fixture_builders.hpp"
#include "fixture_recipe_modules.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_NurbsConvert.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <array>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace cad_mesher::fixtures {
namespace {

using Parameters = std::span<const ConstructionParameter>;

[[nodiscard]] double p(const Parameters parameters, const std::string_view name) {
  return construction_parameter(parameters, name);
}

[[nodiscard]] TopoDS_Edge checked_edge(BRepBuilderAPI_MakeEdge& builder,
                                       const std::string_view fixture_id) {
  return TopoDS::Edge(checked_builder_shape(builder, fixture_id));
}

[[nodiscard]] TopoDS_Face checked_face(BRepBuilderAPI_MakeFace& builder,
                                       const std::string_view fixture_id) {
  return TopoDS::Face(checked_builder_shape(builder, fixture_id));
}

[[nodiscard]] TopoDS_Wire rectangle_wire(const double x_min, const double y_min, const double x_max,
                                         const double y_max, const std::string_view fixture_id) {
  BRepBuilderAPI_MakePolygon polygon;
  polygon.Add(gp_Pnt(x_min, y_min, 0.0));
  polygon.Add(gp_Pnt(x_max, y_min, 0.0));
  polygon.Add(gp_Pnt(x_max, y_max, 0.0));
  polygon.Add(gp_Pnt(x_min, y_max, 0.0));
  polygon.Close();
  if (!polygon.IsDone()) {
    throw ConstructionFailure(std::string(fixture_id) + ": rectangle wire failed");
  }
  return polygon.Wire();
}

[[nodiscard]] TopoDS_Wire circle_wire(const gp_Pnt& centre, const double radius,
                                      const std::string_view fixture_id,
                                      const bool reverse = false) {
  BRepBuilderAPI_MakeEdge edge_builder(gp_Circ(gp_Ax2(centre, gp_Dir(0.0, 0.0, 1.0)), radius));
  const std::array edges{checked_edge(edge_builder, fixture_id)};
  TopoDS_Wire wire = make_wire(edges);
  if (reverse) {
    wire.Reverse();
  }
  return wire;
}

[[nodiscard]] TopoDS_Face perforated_face(const occ::handle<Geom_Surface>& support,
                                          const TopoDS_Wire& outer,
                                          const std::span<const TopoDS_Wire> holes,
                                          const std::string_view fixture_id) {
  BRepBuilderAPI_MakeFace face_builder(support, outer, true);
  for (const TopoDS_Wire& hole : holes) {
    face_builder.Add(hole);
  }
  return checked_face(face_builder, fixture_id);
}

[[nodiscard]] occ::handle<Geom_BSplineSurface> bspline_patch(const double x_offset,
                                                             const double y_offset,
                                                             const double width, const double depth,
                                                             const double height) {
  NCollection_Array2<gp_Pnt> poles(1, 4, 1, 4);
  for (int u = 0; u < 4; ++u) {
    for (int v = 0; v < 4; ++v) {
      const double fu = static_cast<double>(u) / 3.0;
      const double fv = static_cast<double>(v) / 3.0;
      const double z = height * fu * (1.0 - fu) * fv * (1.0 - fv);
      poles.SetValue(u + 1, v + 1, gp_Pnt(x_offset + width * fu, y_offset + depth * fv, z));
    }
  }
  NCollection_Array1<double> u_knots(1, 2);
  NCollection_Array1<double> v_knots(1, 2);
  NCollection_Array1<int> u_multiplicities(1, 2);
  NCollection_Array1<int> v_multiplicities(1, 2);
  u_knots.SetValue(1, 0.0);
  u_knots.SetValue(2, 1.0);
  v_knots.SetValue(1, 0.0);
  v_knots.SetValue(2, 1.0);
  u_multiplicities.SetValue(1, 4);
  u_multiplicities.SetValue(2, 4);
  v_multiplicities.SetValue(1, 4);
  v_multiplicities.SetValue(2, 4);
  return new Geom_BSplineSurface(poles, u_knots, v_knots, u_multiplicities, v_multiplicities, 3, 3,
                                 false, false);
}

[[nodiscard]] TopoDS_Face bspline_face(const double x_offset, const double y_offset,
                                       const double width, const double depth, const double height,
                                       const std::string_view fixture_id) {
  BRepBuilderAPI_MakeFace face_builder(bspline_patch(x_offset, y_offset, width, depth, height),
                                       1.0e-7);
  return checked_face(face_builder, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_cylinder_seam(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.cylinder.seam_dual_pcurve";
  BRepPrimAPI_MakeCylinder cylinder(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                                    p(parameters, "radius"), p(parameters, "height"));
  return checked_builder_shape(cylinder, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_sphere_poles(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.sphere.pole_degenerate_edge";
  BRepPrimAPI_MakeSphere sphere(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                                p(parameters, "radius"));
  return checked_builder_shape(sphere, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_near_zero_edge(const Parameters parameters) {
  constexpr std::string_view fixture_id = "wire.edge.near_zero";
  BRepBuilderAPI_MakeEdge edge_builder(gp_Pnt(0.0, 0.0, 0.0),
                                       gp_Pnt(p(parameters, "length"), 0.0, 0.0));
  const std::array edges{checked_edge(edge_builder, fixture_id)};
  return make_wire(edges);
}

[[nodiscard]] TopoDS_Shape make_planar_cut_graph(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.planar.perforated_cut_graph";
  const occ::handle<Geom_Surface> plane =
      new Geom_Plane(gp_Pln(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)));
  const TopoDS_Wire outer =
      rectangle_wire(0.0, 0.0, p(parameters, "width"), p(parameters, "height"), fixture_id);
  const std::array<TopoDS_Wire, 3> holes{
      circle_wire(gp_Pnt(p(parameters, "hole_1_x"), p(parameters, "hole_y"), 0.0),
                  p(parameters, "hole_radius"), fixture_id, true),
      circle_wire(gp_Pnt(p(parameters, "hole_2_x"), p(parameters, "hole_y"), 0.0),
                  p(parameters, "hole_radius"), fixture_id, true),
      circle_wire(gp_Pnt(p(parameters, "hole_3_x"), p(parameters, "hole_y"), 0.0),
                  p(parameters, "hole_radius"), fixture_id, true),
  };
  return perforated_face(plane, outer, holes, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_single_support_multidomain(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.single_support.multidomain";
  const occ::handle<Geom_Surface> plane =
      new Geom_Plane(gp_Pln(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)));
  const double side = p(parameters, "side");
  const double separation = p(parameters, "separation");
  const TopoDS_Wire first_outer = rectangle_wire(0.0, 0.0, side, side, fixture_id);
  const TopoDS_Wire second_outer =
      rectangle_wire(separation, 0.0, separation + side, side, fixture_id);
  const std::array<TopoDS_Wire, 0> no_holes{};
  const std::array<TopoDS_Shape, 2> domains{
      perforated_face(plane, first_outer, no_holes, fixture_id),
      perforated_face(plane, second_outer, no_holes, fixture_id),
  };
  return make_compound(domains);
}

[[nodiscard]] TopoDS_Shape make_nested_holes(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.trim.nested_holes";
  const occ::handle<Geom_Surface> plane =
      new Geom_Plane(gp_Pln(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)));
  const TopoDS_Wire outer =
      rectangle_wire(0.0, 0.0, p(parameters, "width"), p(parameters, "height"), fixture_id);
  const std::array<TopoDS_Wire, 3> holes{
      circle_wire(gp_Pnt(p(parameters, "major_hole_x"), p(parameters, "major_hole_y"), 0.0),
                  p(parameters, "major_hole_radius"), fixture_id, true),
      circle_wire(gp_Pnt(p(parameters, "minor_hole_1_x"), p(parameters, "minor_hole_y"), 0.0),
                  p(parameters, "minor_hole_radius"), fixture_id, true),
      circle_wire(gp_Pnt(p(parameters, "minor_hole_2_x"), p(parameters, "minor_hole_y"), 0.0),
                  p(parameters, "minor_hole_radius"), fixture_id, true),
  };
  return perforated_face(plane, outer, holes, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_periodic_trim_band(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.trim.periodic_band";
  const occ::handle<Geom_Surface> cylinder = new Geom_CylindricalSurface(
      gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), p(parameters, "radius"));
  BRepBuilderAPI_MakeFace face_builder(cylinder, p(parameters, "u_min"), p(parameters, "u_max"),
                                       p(parameters, "v_min"), p(parameters, "v_max"), 1.0e-7);
  return checked_face(face_builder, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_singular_one_pole(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.trim.singular_one_pole";
  const occ::handle<Geom_Surface> sphere = new Geom_SphericalSurface(
      gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), p(parameters, "radius"));
  BRepBuilderAPI_MakeFace face_builder(sphere, 0.0, p(parameters, "sweep"), p(parameters, "v_min"),
                                       0.5 * std::numbers::pi, 1.0e-7);
  return checked_face(face_builder, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_singular_two_poles(const Parameters parameters) {
  constexpr std::string_view fixture_id = "face.trim.singular_two_poles";
  const occ::handle<Geom_Surface> sphere = new Geom_SphericalSurface(
      gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), p(parameters, "radius"));
  BRepBuilderAPI_MakeFace face_builder(sphere, 0.0, p(parameters, "sweep"), -0.5 * std::numbers::pi,
                                       0.5 * std::numbers::pi, 1.0e-7);
  return checked_face(face_builder, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_structured_bspline_patch(const Parameters parameters) {
  constexpr std::string_view fixture_id = "patch.bspline.four_sided_structured";
  return bspline_face(0.0, 0.0, p(parameters, "width"), p(parameters, "depth"),
                      p(parameters, "height"), fixture_id);
}

[[nodiscard]] TopoDS_Shape nurbs_converted_cylinder(const Parameters parameters,
                                                    const std::string_view fixture_id,
                                                    const double x_offset = 0.0) {
  BRepPrimAPI_MakeCylinder cylinder(gp_Ax2(gp_Pnt(x_offset, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                                    p(parameters, "radius"), p(parameters, "height"));
  const TopoDS_Shape analytic = checked_builder_shape(cylinder, fixture_id);
  BRepBuilderAPI_NurbsConvert converter(analytic, true);
  return checked_builder_shape(converter, fixture_id);
}

[[nodiscard]] TopoDS_Shape make_nurbs_recovered_analytic(const Parameters parameters) {
  return nurbs_converted_cylinder(parameters, "solid.nurbs.recovered_analytic");
}

[[nodiscard]] TopoDS_Shape make_nurbs_confidence_matrix(const Parameters parameters) {
  constexpr std::string_view fixture_id = "semantic.nurbs.confidence_matrix";
  const TopoDS_Shape exact_cylinder = nurbs_converted_cylinder(parameters, fixture_id, 0.0);
  BRepPrimAPI_MakeSphere sphere(
      gp_Ax2(gp_Pnt(p(parameters, "sphere_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "sphere_radius"));
  const TopoDS_Shape analytic_sphere = checked_builder_shape(sphere, fixture_id);
  BRepBuilderAPI_NurbsConvert sphere_converter(analytic_sphere, true);
  const TopoDS_Shape exact_sphere = checked_builder_shape(sphere_converter, fixture_id);
  const TopoDS_Face near_planar =
      bspline_face(p(parameters, "patch_x"), 0.0, p(parameters, "patch_width"),
                   p(parameters, "patch_depth"), p(parameters, "near_height"), fixture_id);
  const TopoDS_Face residual =
      bspline_face(p(parameters, "residual_x"), 0.0, p(parameters, "patch_width"),
                   p(parameters, "patch_depth"), p(parameters, "residual_height"), fixture_id);
  const std::array<TopoDS_Shape, 4> children{exact_cylinder, exact_sphere, near_planar, residual};
  return make_compound(children);
}

[[nodiscard]] TopoDS_Shape make_mixed_support_component(const Parameters parameters) {
  constexpr std::string_view fixture_id = "component.mixed.supported_unsupported";
  BRepPrimAPI_MakeBox box(p(parameters, "box_x"), p(parameters, "box_y"), p(parameters, "box_z"));
  const TopoDS_Face residual =
      bspline_face(p(parameters, "residual_x"), 0.0, p(parameters, "patch_width"),
                   p(parameters, "patch_depth"), p(parameters, "patch_height"), fixture_id);
  const std::array<TopoDS_Shape, 2> children{
      checked_builder_shape(box, fixture_id),
      residual,
  };
  return make_compound(children);
}

[[nodiscard]] RoundTripContract seam_contract() {
  return RoundTripContract{
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.periodic_seam_parameter_shifted",
              "step.seam_coedges_reordered",
          },
  };
}

[[nodiscard]] RoundTripContract sphere_pole_contract() {
  return RoundTripContract{
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.curve.trimmed_wrapper_canonicalized",
              "step.periodic_seam_parameter_shifted",
              "step.seam_coedges_reordered",
          },
  };
}

[[nodiscard]] RoundTripContract standalone_face_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.standalone_face_wrapped_in_shell",
          },
  };
}

[[nodiscard]] RoundTripContract standalone_periodic_face_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.periodic_seam_parameter_shifted",
              "step.seam_coedges_reordered",
              "step.standalone_face_wrapped_in_shell",
          },
  };
}

[[nodiscard]] RoundTripContract standalone_two_pole_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.curve.trimmed_wrapper_canonicalized",
              "step.periodic_seam_parameter_shifted",
              "step.seam_coedges_reordered",
              "step.standalone_face_wrapped_in_shell",
          },
  };
}

[[nodiscard]] RoundTripContract standalone_one_pole_contract() {
  return RoundTripContract{
      .expected_brep_valid = false,
      .topology_counts = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.curve.trimmed_wrapper_canonicalized",
              "step.periodic_seam_parameter_shifted",
              "step.seam_coedges_reordered",
              "step.standalone_face_wrapped_in_shell",
              "step.surface.singular_one_pole_import_invalid",
          },
  };
}

[[nodiscard]] RoundTripContract free_wire_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.free_wire_flattened_to_free_edges",
          },
  };
}

[[nodiscard]] RoundTripContract wrapper_normalization_contract() {
  return RoundTripContract{
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.nurbs_analyticized",
              "step.nurbs_reparameterized",
          },
  };
}

[[nodiscard]] RoundTripContract mixed_wrapper_normalization_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.nurbs_analyticized",
              "step.nurbs_reparameterized",
              "step.standalone_face_wrapped_in_shell",
          },
  };
}

} // namespace

void append_trim_support_recipes(std::vector<Recipe>& recipes) {
  recipes.push_back(
      procedural_shape_recipe("face.cylinder.seam_dual_pcurve",
                              {{"height", 18.0, "millimetres"}, {"radius", 7.0, "millimetres"}},
                              make_cylinder_seam, {}, seam_contract()));
  recipes.push_back(procedural_shape_recipe("face.sphere.pole_degenerate_edge",
                                            {{"radius", 9.0, "millimetres"}}, make_sphere_poles, {},
                                            sphere_pole_contract()));
  recipes.push_back(procedural_shape_recipe("wire.edge.near_zero",
                                            {{"length", 1.0e-5, "millimetres"}},
                                            make_near_zero_edge, {}, free_wire_contract()));
  recipes.push_back(procedural_shape_recipe("face.planar.perforated_cut_graph",
                                            {{"height", 32.0, "millimetres"},
                                             {"hole_1_x", 12.0, "millimetres"},
                                             {"hole_2_x", 27.0, "millimetres"},
                                             {"hole_3_x", 42.0, "millimetres"},
                                             {"hole_radius", 4.0, "millimetres"},
                                             {"hole_y", 16.0, "millimetres"},
                                             {"width", 54.0, "millimetres"}},
                                            make_planar_cut_graph, {}, standalone_face_contract()));
  recipes.push_back(procedural_shape_recipe(
      "face.single_support.multidomain",
      {{"separation", 28.0, "millimetres"}, {"side", 18.0, "millimetres"}},
      make_single_support_multidomain, {},
      RoundTripContract{
          .topology_counts = false,
          .occurrence_graph = false,
          .model_topology = false,
          .accepted_normalization_codes = {"step.shared_support_split",
                                           "step.standalone_face_wrapped_in_shell"},
      }));
  recipes.push_back(procedural_shape_recipe("face.trim.nested_holes",
                                            {{"height", 40.0, "millimetres"},
                                             {"major_hole_radius", 8.0, "millimetres"},
                                             {"major_hole_x", 18.0, "millimetres"},
                                             {"major_hole_y", 20.0, "millimetres"},
                                             {"minor_hole_1_x", 38.0, "millimetres"},
                                             {"minor_hole_2_x", 50.0, "millimetres"},
                                             {"minor_hole_radius", 3.0, "millimetres"},
                                             {"minor_hole_y", 20.0, "millimetres"},
                                             {"width", 62.0, "millimetres"}},
                                            make_nested_holes, {}, standalone_face_contract()));
  recipes.push_back(procedural_shape_recipe("face.trim.periodic_band",
                                            {{"radius", 6.0, "millimetres"},
                                             {"u_max", 7.1, "radians"},
                                             {"u_min", 5.4, "radians"},
                                             {"v_max", 14.0, "millimetres"},
                                             {"v_min", 2.0, "millimetres"}},
                                            make_periodic_trim_band, {},
                                            standalone_periodic_face_contract()));
  recipes.push_back(procedural_shape_recipe("face.trim.singular_one_pole",
                                            {{"radius", 7.0, "millimetres"},
                                             {"sweep", 2.0 * std::numbers::pi, "radians"},
                                             {"v_min", -0.35, "radians"}},
                                            make_singular_one_pole, {},
                                            standalone_one_pole_contract()));
  recipes.push_back(procedural_shape_recipe(
      "face.trim.singular_two_poles",
      {{"radius", 7.0, "millimetres"}, {"sweep", 2.0 * std::numbers::pi, "radians"}},
      make_singular_two_poles, {}, standalone_two_pole_contract()));
  recipes.push_back(procedural_shape_recipe("patch.bspline.four_sided_structured",
                                            {{"depth", 14.0, "millimetres"},
                                             {"height", 4.0, "millimetres"},
                                             {"width", 20.0, "millimetres"}},
                                            make_structured_bspline_patch, {},
                                            standalone_face_contract()));
  recipes.push_back(procedural_shape_recipe("semantic.nurbs.confidence_matrix",
                                            {{"height", 16.0, "millimetres"},
                                             {"near_height", 1.0e-5, "millimetres"},
                                             {"patch_depth", 12.0, "millimetres"},
                                             {"patch_width", 18.0, "millimetres"},
                                             {"patch_x", 62.0, "millimetres"},
                                             {"radius", 6.0, "millimetres"},
                                             {"residual_height", 8.0, "millimetres"},
                                             {"residual_x", 88.0, "millimetres"},
                                             {"sphere_radius", 7.0, "millimetres"},
                                             {"sphere_x", 34.0, "millimetres"}},
                                            make_nurbs_confidence_matrix, {},
                                            mixed_wrapper_normalization_contract()));
  recipes.push_back(
      procedural_shape_recipe("solid.nurbs.recovered_analytic",
                              {{"height", 16.0, "millimetres"}, {"radius", 6.0, "millimetres"}},
                              make_nurbs_recovered_analytic, {}, wrapper_normalization_contract()));
  recipes.push_back(procedural_shape_recipe("component.mixed.supported_unsupported",
                                            {{"box_x", 24.0, "millimetres"},
                                             {"box_y", 18.0, "millimetres"},
                                             {"box_z", 12.0, "millimetres"},
                                             {"patch_depth", 14.0, "millimetres"},
                                             {"patch_height", 5.0, "millimetres"},
                                             {"patch_width", 20.0, "millimetres"},
                                             {"residual_x", 38.0, "millimetres"}},
                                            make_mixed_support_component, {},
                                            standalone_face_contract()));
}

} // namespace cad_mesher::fixtures
