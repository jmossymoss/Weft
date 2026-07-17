#include "fixture_builders.hpp"
#include "fixture_recipe_modules.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GeomInt_IntSS.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Parabola.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <NCollection_Array1.hxx>
#include <TopoDS.hxx>
#include <array>
#include <cmath>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Hypr.hxx>
#include <gp_Parab.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <numbers>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace cad_mesher::fixtures {
namespace {

using Parameters = std::span<const ConstructionParameter>;

double p(const Parameters parameters, const std::string_view name) {
  return construction_parameter(parameters, name);
}

TopoDS_Edge checked_edge(BRepBuilderAPI_MakeEdge& builder, const std::string_view fixture_id) {
  return TopoDS::Edge(checked_builder_shape(builder, fixture_id));
}

TopoDS_Edge edge_from_curve(const occ::handle<Geom_Curve>& curve,
                            const std::string_view fixture_id) {
  BRepBuilderAPI_MakeEdge builder(curve);
  return checked_edge(builder, fixture_id);
}

TopoDS_Edge edge_from_curve(const occ::handle<Geom_Curve>& curve, const double first,
                            const double last, const std::string_view fixture_id) {
  BRepBuilderAPI_MakeEdge builder(curve, first, last);
  return checked_edge(builder, fixture_id);
}

TopoDS_Wire one_edge_wire(const TopoDS_Edge& edge) {
  const std::array edges{edge};
  return make_wire(edges);
}

NCollection_Array1<gp_Pnt> bezier_poles(const int degree, const double y, const double height) {
  NCollection_Array1<gp_Pnt> poles(1, degree + 1);
  for (int index = 0; index <= degree; ++index) {
    const double fraction = static_cast<double>(index) / static_cast<double>(degree);
    const double z = index == 0 || index == degree ? 0.0 : height * (index % 2 == 0 ? 0.55 : 1.0);
    poles.SetValue(index + 1, gp_Pnt(18.0 * fraction, y, z));
  }
  return poles;
}

TopoDS_Shape make_bounded_conics(const Parameters parameters) {
  const gp_Ax2 hyperbola_axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
  BRepBuilderAPI_MakeEdge hyperbola_builder(
      gp_Hypr(hyperbola_axis, p(parameters, "hyperbola_major_radius"),
              p(parameters, "hyperbola_minor_radius")),
      p(parameters, "hyperbola_first_parameter"), p(parameters, "hyperbola_last_parameter"));

  const gp_Ax2 parabola_axis(gp_Pnt(0.0, p(parameters, "parabola_y"), 0.0), gp_Dir(0.0, 0.0, 1.0));
  BRepBuilderAPI_MakeEdge parabola_builder(
      gp_Parab(parabola_axis, p(parameters, "parabola_focal_length")),
      p(parameters, "parabola_first_parameter"), p(parameters, "parabola_last_parameter"));

  const std::array<TopoDS_Shape, 2> children{
      one_edge_wire(checked_edge(hyperbola_builder, "wire.conic.hyperbola_parabola")),
      one_edge_wire(checked_edge(parabola_builder, "wire.conic.hyperbola_parabola")),
  };
  return make_compound(children);
}

TopoDS_Shape make_ellipse_matrix(const Parameters parameters) {
  const gp_Elips full(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                      p(parameters, "full_major_radius"), p(parameters, "full_minor_radius"));
  BRepBuilderAPI_MakeEdge full_builder(full);

  const gp_Elips partial(
      gp_Ax2(gp_Pnt(0.0, p(parameters, "partial_y"), 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "partial_major_radius"), p(parameters, "partial_minor_radius"));
  BRepBuilderAPI_MakeEdge partial_builder(partial, p(parameters, "partial_first_parameter"),
                                          p(parameters, "partial_last_parameter"));

  const std::array<TopoDS_Shape, 2> children{
      one_edge_wire(checked_edge(full_builder, "wire.ellipse.full_partial")),
      one_edge_wire(checked_edge(partial_builder, "wire.ellipse.full_partial")),
  };
  return make_compound(children);
}

TopoDS_Shape make_bezier_matrix(const Parameters parameters) {
  const double spacing = p(parameters, "row_spacing");
  const double height = p(parameters, "control_height");
  std::array<TopoDS_Shape, 4> children;

  {
    const auto poles = bezier_poles(2, 0.0, height);
    const occ::handle<Geom_Curve> curve = new Geom_BezierCurve(poles);
    children[0] = one_edge_wire(edge_from_curve(curve, "wire.bezier.rationality_degree_matrix"));
  }
  {
    const auto poles = bezier_poles(2, spacing, height);
    NCollection_Array1<double> weights(1, 3);
    weights.SetValue(1, 1.0);
    weights.SetValue(2, p(parameters, "quadratic_middle_weight"));
    weights.SetValue(3, 1.0);
    const occ::handle<Geom_Curve> curve = new Geom_BezierCurve(poles, weights);
    children[1] = one_edge_wire(edge_from_curve(curve, "wire.bezier.rationality_degree_matrix"));
  }
  {
    const auto poles = bezier_poles(3, 2.0 * spacing, height);
    const occ::handle<Geom_Curve> curve = new Geom_BezierCurve(poles);
    children[2] = one_edge_wire(edge_from_curve(curve, "wire.bezier.rationality_degree_matrix"));
  }
  {
    const auto poles = bezier_poles(3, 3.0 * spacing, height);
    NCollection_Array1<double> weights(1, 4);
    weights.SetValue(1, 1.0);
    weights.SetValue(2, p(parameters, "cubic_inner_weight_1"));
    weights.SetValue(3, p(parameters, "cubic_inner_weight_2"));
    weights.SetValue(4, 1.0);
    const occ::handle<Geom_Curve> curve = new Geom_BezierCurve(poles, weights);
    children[3] = one_edge_wire(edge_from_curve(curve, "wire.bezier.rationality_degree_matrix"));
  }

  return make_compound(children);
}

occ::handle<Geom_BSplineCurve> make_nonrational_quadratic_bspline(const double y,
                                                                  const double height) {
  NCollection_Array1<gp_Pnt> poles(1, 4);
  poles.SetValue(1, gp_Pnt(0.0, y, 0.0));
  poles.SetValue(2, gp_Pnt(6.0, y, height));
  poles.SetValue(3, gp_Pnt(13.0, y, -0.4 * height));
  poles.SetValue(4, gp_Pnt(20.0, y, 0.0));
  NCollection_Array1<double> knots(1, 3);
  knots.SetValue(1, 0.0);
  knots.SetValue(2, 0.38);
  knots.SetValue(3, 1.0);
  NCollection_Array1<int> multiplicities(1, 3);
  multiplicities.SetValue(1, 3);
  multiplicities.SetValue(2, 1);
  multiplicities.SetValue(3, 3);
  return new Geom_BSplineCurve(poles, knots, multiplicities, 2, false);
}

occ::handle<Geom_BSplineCurve> make_rational_cubic_bspline(const double y, const double height,
                                                           const double light_weight) {
  NCollection_Array1<gp_Pnt> poles(1, 6);
  NCollection_Array1<double> weights(1, 6);
  for (int index = 1; index <= 6; ++index) {
    const double x = 4.0 * static_cast<double>(index - 1);
    const double z = index == 1 || index == 6 ? 0.0 : height * (index % 2 == 0 ? 1.0 : -0.45);
    poles.SetValue(index, gp_Pnt(x, y, z));
    weights.SetValue(index, index == 3 || index == 4 ? light_weight : 1.0);
  }
  NCollection_Array1<double> knots(1, 4);
  knots.SetValue(1, 0.0);
  knots.SetValue(2, 0.24);
  knots.SetValue(3, 0.71);
  knots.SetValue(4, 1.0);
  NCollection_Array1<int> multiplicities(1, 4);
  multiplicities.SetValue(1, 4);
  multiplicities.SetValue(2, 1);
  multiplicities.SetValue(3, 1);
  multiplicities.SetValue(4, 4);
  return new Geom_BSplineCurve(poles, weights, knots, multiplicities, 3, false);
}

occ::handle<Geom_BSplineCurve> make_periodic_quadratic_bspline(const double y,
                                                               const double radius) {
  NCollection_Array1<gp_Pnt> poles(1, 5);
  for (int index = 0; index < 5; ++index) {
    const double angle = 2.0 * std::numbers::pi * static_cast<double>(index) / 5.0;
    poles.SetValue(index + 1, gp_Pnt(radius * std::cos(angle), y + radius * std::sin(angle), 0.0));
  }
  NCollection_Array1<double> knots(1, 6);
  NCollection_Array1<int> multiplicities(1, 6);
  for (int index = 1; index <= 6; ++index) {
    knots.SetValue(index, static_cast<double>(index - 1));
    multiplicities.SetValue(index, 1);
  }
  return new Geom_BSplineCurve(poles, knots, multiplicities, 2, true);
}

TopoDS_Shape make_bspline_matrix(const Parameters parameters) {
  const double spacing = p(parameters, "row_spacing");
  const double height = p(parameters, "control_height");
  const occ::handle<Geom_Curve> nonrational = make_nonrational_quadratic_bspline(0.0, height);
  const occ::handle<Geom_Curve> rational =
      make_rational_cubic_bspline(spacing, height, p(parameters, "rational_inner_weight"));
  const occ::handle<Geom_Curve> periodic =
      make_periodic_quadratic_bspline(2.5 * spacing, p(parameters, "periodic_radius"));

  const std::array<TopoDS_Shape, 3> children{
      one_edge_wire(edge_from_curve(nonrational, "wire.bspline.rationality_knot_matrix")),
      one_edge_wire(edge_from_curve(rational, "wire.bspline.rationality_knot_matrix")),
      one_edge_wire(edge_from_curve(periodic, "wire.bspline.rationality_knot_matrix")),
  };
  return make_compound(children);
}

occ::handle<Geom_BSplineCurve> make_cubic_join_curve(const int internal_multiplicity,
                                                     const double y, const double height) {
  const int pole_count = 4 + internal_multiplicity;
  NCollection_Array1<gp_Pnt> poles(1, pole_count);
  for (int index = 1; index <= pole_count; ++index) {
    const double fraction = static_cast<double>(index - 1) / static_cast<double>(pole_count - 1);
    const double z = height * (0.65 * std::sin(2.0 * std::numbers::pi * fraction) + fraction);
    poles.SetValue(index, gp_Pnt(22.0 * fraction, y, z));
  }
  NCollection_Array1<double> knots(1, 3);
  knots.SetValue(1, 0.0);
  knots.SetValue(2, 0.5);
  knots.SetValue(3, 1.0);
  NCollection_Array1<int> multiplicities(1, 3);
  multiplicities.SetValue(1, 4);
  multiplicities.SetValue(2, internal_multiplicity);
  multiplicities.SetValue(3, 4);
  return new Geom_BSplineCurve(poles, knots, multiplicities, 3, false);
}

TopoDS_Shape make_continuity_matrix(const Parameters parameters) {
  const double spacing = p(parameters, "row_spacing");
  const double height = p(parameters, "control_height");
  const occ::handle<Geom_Curve> c0 = make_cubic_join_curve(3, 0.0, height);
  const occ::handle<Geom_Curve> c1 = make_cubic_join_curve(2, spacing, height);
  const occ::handle<Geom_Curve> c2 = make_cubic_join_curve(1, 2.0 * spacing, height);
  const auto cn_poles = bezier_poles(4, 3.0 * spacing, height);
  const occ::handle<Geom_Curve> cn = new Geom_BezierCurve(cn_poles);

  const std::array<TopoDS_Shape, 4> children{
      one_edge_wire(edge_from_curve(c0, "wire.continuity.c0_c1_c2_cn")),
      one_edge_wire(edge_from_curve(c1, "wire.continuity.c0_c1_c2_cn")),
      one_edge_wire(edge_from_curve(c2, "wire.continuity.c0_c1_c2_cn")),
      one_edge_wire(edge_from_curve(cn, "wire.continuity.c0_c1_c2_cn")),
  };
  return make_compound(children);
}

TopoDS_Shape make_offset_matrix(const Parameters parameters) {
  const gp_Ax2 axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
  const occ::handle<Geom_Curve> ellipse = new Geom_Ellipse(
      axis, p(parameters, "ellipse_major_radius"), p(parameters, "ellipse_minor_radius"));
  const occ::handle<Geom_Curve> trimmed = new Geom_TrimmedCurve(
      ellipse, p(parameters, "trim_first_parameter"), p(parameters, "trim_last_parameter"));
  const occ::handle<Geom_Curve> regular =
      new Geom_OffsetCurve(trimmed, p(parameters, "regular_offset"), gp_Dir(0.0, 0.0, 1.0));
  const occ::handle<Geom_Curve> nested =
      new Geom_OffsetCurve(regular, p(parameters, "nested_offset"), gp_Dir(0.0, 0.0, 1.0));

  const double focal_length = p(parameters, "singular_focal_length");
  const occ::handle<Geom_Curve> parabola = new Geom_Parabola(
      gp_Ax2(gp_Pnt(0.0, p(parameters, "singular_y"), 0.0), gp_Dir(0.0, 0.0, 1.0)), focal_length);
  const occ::handle<Geom_Curve> singular =
      new Geom_OffsetCurve(parabola, 2.0 * focal_length, gp_Dir(0.0, 0.0, 1.0));

  const std::array<TopoDS_Shape, 3> children{
      one_edge_wire(edge_from_curve(regular, "wire.offset.nested_singular")),
      one_edge_wire(edge_from_curve(nested, "wire.offset.nested_singular")),
      one_edge_wire(edge_from_curve(singular, p(parameters, "singular_first_parameter"),
                                    p(parameters, "singular_last_parameter"),
                                    "wire.offset.nested_singular")),
  };
  return make_compound(children);
}

TopoDS_Shape make_composite_intersection_iso(const Parameters parameters) {
  BRepBuilderAPI_MakeEdge segment_1(gp_Pnt(0.0, 0.0, 0.0),
                                    gp_Pnt(p(parameters, "composite_length"), 0.0, 0.0));
  BRepBuilderAPI_MakeEdge segment_2(
      gp_Pnt(p(parameters, "composite_length"), 0.0, 0.0),
      gp_Pnt(p(parameters, "composite_length"), p(parameters, "composite_rise"), 0.0));
  BRepBuilderAPI_MakeEdge segment_3(
      gp_Pnt(p(parameters, "composite_length"), p(parameters, "composite_rise"), 0.0),
      gp_Pnt(2.0 * p(parameters, "composite_length"), p(parameters, "composite_rise"), 0.0));
  const std::array composite_edges{
      checked_edge(segment_1, "wire.composite.intersection_isoparametric"),
      checked_edge(segment_2, "wire.composite.intersection_isoparametric"),
      checked_edge(segment_3, "wire.composite.intersection_isoparametric"),
  };

  const gp_Pnt sphere_center(p(parameters, "intersection_center_x"), 0.0, 0.0);
  const occ::handle<Geom_Surface> sphere = new Geom_SphericalSurface(
      gp_Ax3(sphere_center, gp_Dir(0.0, 0.0, 1.0)), p(parameters, "intersection_sphere_radius"));
  const occ::handle<Geom_Surface> plane =
      new Geom_Plane(gp_Pln(gp_Pnt(sphere_center.X(), 0.0, p(parameters, "intersection_plane_z")),
                            gp_Dir(0.0, 0.0, 1.0)));
  GeomInt_IntSS intersection(sphere, plane, p(parameters, "intersection_tolerance"), false, false,
                             false);
  if (!intersection.IsDone() || intersection.NbLines() != 1) {
    throw ConstructionFailure(
        "wire.composite.intersection_isoparametric: expected one exact sphere-plane line");
  }
  const occ::handle<Geom_Curve> intersection_curve = intersection.Line(1);

  const occ::handle<Geom_CylindricalSurface> cylinder = new Geom_CylindricalSurface(
      gp_Ax3(gp_Pnt(p(parameters, "iso_center_x"), p(parameters, "iso_center_y"), 0.0),
             gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "iso_cylinder_radius"));
  const occ::handle<Geom_Curve> u_iso = cylinder->UIso(p(parameters, "u_iso_parameter"));
  const occ::handle<Geom_Curve> v_iso = cylinder->VIso(p(parameters, "v_iso_parameter"));

  const std::array<TopoDS_Shape, 4> children{
      make_wire(composite_edges),
      one_edge_wire(
          edge_from_curve(intersection_curve, "wire.composite.intersection_isoparametric")),
      one_edge_wire(edge_from_curve(u_iso, p(parameters, "u_iso_first_parameter"),
                                    p(parameters, "u_iso_last_parameter"),
                                    "wire.composite.intersection_isoparametric")),
      one_edge_wire(edge_from_curve(v_iso, "wire.composite.intersection_isoparametric")),
  };
  return make_compound(children);
}

void append_free_wire_normalization(std::vector<std::string_view>& normalization_codes) {
  normalization_codes.push_back("step.free_wire_container_flattened");
}

RoundTripContract free_wire_contract(std::vector<std::string_view> normalization_codes = {}) {
  append_free_wire_normalization(normalization_codes);
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .accepted_normalization_codes = std::move(normalization_codes),
  };
}

RoundTripContract
free_wire_concrete_type_contract(std::vector<std::string_view> normalization_codes) {
  append_free_wire_normalization(normalization_codes);
  return RoundTripContract{
      .topology_counts = false,
      .concrete_types = false,
      .model_topology = false,
      .accepted_normalization_codes = std::move(normalization_codes),
  };
}

RoundTripContract
free_wire_curve_representation_contract(std::vector<std::string_view> normalization_codes,
                                        const bool entities_preserved = true) {
  append_free_wire_normalization(normalization_codes);
  return RoundTripContract{
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = entities_preserved,
      .model_topology = false,
      .tolerances = entities_preserved,
      .accepted_normalization_codes = std::move(normalization_codes),
  };
}

} // namespace

void append_curve_recipes(std::vector<Recipe>& recipes) {
  recipes.push_back(procedural_shape_recipe(
      "wire.bezier.rationality_degree_matrix",
      {{"control_height", 6.0, "millimetres"},
       {"cubic_inner_weight_1", 0.55, "unitless"},
       {"cubic_inner_weight_2", 1.65, "unitless"},
       {"quadratic_middle_weight", 0.42, "unitless"},
       {"row_spacing", 14.0, "millimetres"}},
      make_bezier_matrix, {},
      free_wire_curve_representation_contract({"step.curve.bezier_serialized_as_bspline"})));

  recipes.push_back(procedural_shape_recipe("wire.bspline.rationality_knot_matrix",
                                            {{"control_height", 7.0, "millimetres"},
                                             {"periodic_radius", 5.5, "millimetres"},
                                             {"rational_inner_weight", 0.48, "unitless"},
                                             {"row_spacing", 15.0, "millimetres"}},
                                            make_bspline_matrix, {}, free_wire_contract()));

  recipes.push_back(procedural_shape_recipe(
      "wire.composite.intersection_isoparametric",
      {{"composite_length", 8.0, "millimetres"},
       {"composite_rise", 5.0, "millimetres"},
       {"intersection_center_x", 38.0, "millimetres"},
       {"intersection_plane_z", 2.0, "millimetres"},
       {"intersection_sphere_radius", 7.0, "millimetres"},
       {"intersection_tolerance", 1.0e-9, "millimetres"},
       {"iso_center_x", 58.0, "millimetres"},
       {"iso_center_y", 0.0, "millimetres"},
       {"iso_cylinder_radius", 4.0, "millimetres"},
       {"u_iso_first_parameter", -4.0, "millimetres"},
       {"u_iso_last_parameter", 5.0, "millimetres"},
       {"u_iso_parameter", 0.65, "radians"},
       {"v_iso_parameter", 1.75, "millimetres"}},
      make_composite_intersection_iso, {},
      free_wire_concrete_type_contract(
          {"step.curve.intersection_analyticized", "step.curve.isoparametric_analyticized"})));

  recipes.push_back(procedural_shape_recipe("wire.conic.hyperbola_parabola",
                                            {{"hyperbola_first_parameter", -1.1, "radians"},
                                             {"hyperbola_last_parameter", 0.85, "radians"},
                                             {"hyperbola_major_radius", 5.0, "millimetres"},
                                             {"hyperbola_minor_radius", 3.0, "millimetres"},
                                             {"parabola_first_parameter", -2.2, "unitless"},
                                             {"parabola_focal_length", 3.5, "millimetres"},
                                             {"parabola_last_parameter", 1.7, "unitless"},
                                             {"parabola_y", 20.0, "millimetres"}},
                                            make_bounded_conics, {}, free_wire_contract()));

  recipes.push_back(procedural_shape_recipe(
      "wire.continuity.c0_c1_c2_cn",
      {{"control_height", 5.0, "millimetres"}, {"row_spacing", 12.0, "millimetres"}},
      make_continuity_matrix, {},
      free_wire_curve_representation_contract({"step.curve.bezier_serialized_as_bspline"})));

  recipes.push_back(procedural_shape_recipe("wire.ellipse.full_partial",
                                            {{"full_major_radius", 9.0, "millimetres"},
                                             {"full_minor_radius", 4.5, "millimetres"},
                                             {"partial_first_parameter", 0.35, "radians"},
                                             {"partial_last_parameter", 4.8, "radians"},
                                             {"partial_major_radius", 7.0, "millimetres"},
                                             {"partial_minor_radius", 3.0, "millimetres"},
                                             {"partial_y", 18.0, "millimetres"}},
                                            make_ellipse_matrix, {}, free_wire_contract()));

  recipes.push_back(procedural_shape_recipe(
      "wire.offset.nested_singular",
      {{"ellipse_major_radius", 10.0, "millimetres"},
       {"ellipse_minor_radius", 6.0, "millimetres"},
       {"nested_offset", 1.15, "millimetres"},
       {"regular_offset", 1.75, "millimetres"},
       {"singular_first_parameter", -1.25, "unitless"},
       {"singular_focal_length", 3.0, "millimetres"},
       {"singular_last_parameter", 1.25, "unitless"},
       {"singular_y", 24.0, "millimetres"},
       {"trim_first_parameter", 0.2, "radians"},
       {"trim_last_parameter", 5.55, "radians"}},
      make_offset_matrix, {},
      free_wire_curve_representation_contract(
          {"step.curve.offset_canonicalized", "step.curve.trimmed_basis_canonicalized",
           "step.curve.offset_entities_dropped", "step.tolerances_absent_after_entity_drop"},
          false)));
}

} // namespace cad_mesher::fixtures
