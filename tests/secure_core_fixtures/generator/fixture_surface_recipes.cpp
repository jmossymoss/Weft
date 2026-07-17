#include "fixture_builders.hpp"
#include "fixture_recipe_modules.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Builder.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomFill.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <array>
#include <cmath>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
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

TopoDS_Face checked_face(BRepBuilderAPI_MakeFace& builder, const std::string_view fixture_id) {
  return TopoDS::Face(checked_builder_shape(builder, fixture_id));
}

TopoDS_Edge checked_edge(BRepBuilderAPI_MakeEdge& builder, const std::string_view fixture_id) {
  return TopoDS::Edge(checked_builder_shape(builder, fixture_id));
}

TopoDS_Face natural_face(const occ::handle<Geom_Surface>& surface,
                         const std::string_view fixture_id, const double tolerance = 1.0e-7) {
  BRepBuilderAPI_MakeFace builder(surface, tolerance);
  return checked_face(builder, fixture_id);
}

TopoDS_Face bounded_face(const occ::handle<Geom_Surface>& surface, const double u_min,
                         const double u_max, const double v_min, const double v_max,
                         const std::string_view fixture_id, const double tolerance = 1.0e-7) {
  BRepBuilderAPI_MakeFace builder(surface, u_min, u_max, v_min, v_max, tolerance);
  return checked_face(builder, fixture_id);
}

TopoDS_Wire circle_wire(const gp_Pnt& center, const double radius,
                        const std::string_view fixture_id) {
  BRepBuilderAPI_MakeEdge edge_builder(gp_Circ(gp_Ax2(center, gp_Dir(0.0, 0.0, 1.0)), radius));
  const std::array edges{checked_edge(edge_builder, fixture_id)};
  return make_wire(edges);
}

NCollection_Array2<gp_Pnt> bezier_surface_poles(const int u_degree, const int v_degree,
                                                const double x_offset, const double y_offset,
                                                const double x_extent, const double y_extent,
                                                const double height) {
  NCollection_Array2<gp_Pnt> poles(1, u_degree + 1, 1, v_degree + 1);
  for (int u = 0; u <= u_degree; ++u) {
    for (int v = 0; v <= v_degree; ++v) {
      const double fu = static_cast<double>(u) / static_cast<double>(u_degree);
      const double fv = static_cast<double>(v) / static_cast<double>(v_degree);
      const double z = height * std::sin(std::numbers::pi * fu) * std::sin(std::numbers::pi * fv);
      poles.SetValue(u + 1, v + 1, gp_Pnt(x_offset + x_extent * fu, y_offset + y_extent * fv, z));
    }
  }
  return poles;
}

TopoDS_Shape make_periodic_band(const Parameters parameters) {
  const occ::handle<Geom_Surface> cylinder = new Geom_CylindricalSurface(
      gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), p(parameters, "radius"));
  return bounded_face(cylinder, p(parameters, "u_min"), p(parameters, "u_max"),
                      p(parameters, "v_min"), p(parameters, "v_max"),
                      "face.periodic.band_seam_crossing");
}

TopoDS_Shape make_reversed_surface_matrix(const Parameters parameters) {
  const occ::handle<Geom_Surface> plane =
      new Geom_Plane(gp_Pln(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)));
  TopoDS_Face plane_face =
      bounded_face(plane, 0.0, p(parameters, "plane_u_extent"), 0.0,
                   p(parameters, "plane_v_extent"), "face.surface.reversed_orientation");
  plane_face.Reverse();

  const occ::handle<Geom_Surface> cylinder = new Geom_CylindricalSurface(
      gp_Ax3(gp_Pnt(p(parameters, "cylinder_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "cylinder_radius"));
  TopoDS_Face cylinder_face =
      bounded_face(cylinder, 0.0, p(parameters, "cylinder_sweep"), 0.0,
                   p(parameters, "cylinder_height"), "face.surface.reversed_orientation");
  cylinder_face.Reverse();

  const auto poles =
      bezier_surface_poles(3, 2, p(parameters, "bezier_x"), 0.0, p(parameters, "bezier_u_extent"),
                           p(parameters, "bezier_v_extent"), p(parameters, "bezier_height"));
  const occ::handle<Geom_Surface> bezier = new Geom_BezierSurface(poles);
  TopoDS_Face bezier_face = natural_face(bezier, "face.surface.reversed_orientation");
  bezier_face.Reverse();

  const std::array<TopoDS_Shape, 3> children{plane_face, cylinder_face, bezier_face};
  return make_compound(children);
}

TopoDS_Shape make_apex_cones(const Parameters parameters) {
  BRepPrimAPI_MakeCone full(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                            p(parameters, "full_radius"), 0.0, p(parameters, "full_height"));
  BRepPrimAPI_MakeCone partial(
      gp_Ax2(gp_Pnt(p(parameters, "partial_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "partial_radius"), 0.0, p(parameters, "partial_height"),
      p(parameters, "partial_sweep"));
  const std::array<TopoDS_Shape, 2> children{
      checked_builder_shape(full, "solid.cone.apex_full_partial"),
      checked_builder_shape(partial, "solid.cone.apex_full_partial"),
  };
  return make_compound(children);
}

TopoDS_Face revolution_profile(const double x_offset, const double radius, const double height,
                               const double bulge, const std::string_view fixture_id) {
  NCollection_Array1<gp_Pnt> poles(1, 4);
  poles.SetValue(1, gp_Pnt(x_offset, 0.0, 0.0));
  poles.SetValue(2, gp_Pnt(x_offset + radius, 0.0, bulge * height));
  poles.SetValue(3, gp_Pnt(x_offset + 0.8 * radius, 0.0, (1.0 - bulge) * height));
  poles.SetValue(4, gp_Pnt(x_offset, 0.0, height));
  const occ::handle<Geom_Curve> meridian = new Geom_BezierCurve(poles);
  BRepBuilderAPI_MakeEdge meridian_builder(meridian);
  BRepBuilderAPI_MakeEdge axis_builder(gp_Pnt(x_offset, 0.0, height), gp_Pnt(x_offset, 0.0, 0.0));
  const std::array profile_edges{
      checked_edge(meridian_builder, fixture_id),
      checked_edge(axis_builder, fixture_id),
  };
  BRepBuilderAPI_MakeFace face_builder(make_wire(profile_edges), true);
  return checked_face(face_builder, fixture_id);
}

TopoDS_Shape make_revolution_matrix(const Parameters parameters) {
  const TopoDS_Face full_profile =
      revolution_profile(0.0, p(parameters, "full_radius"), p(parameters, "full_height"),
                         p(parameters, "full_bulge"), "solid.revolution.full_partial_pole");
  BRepPrimAPI_MakeRevol full(full_profile, gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                             true);

  const double partial_x = p(parameters, "partial_x");
  const TopoDS_Face partial_profile = revolution_profile(
      partial_x, p(parameters, "partial_radius"), p(parameters, "partial_height"),
      p(parameters, "partial_bulge"), "solid.revolution.full_partial_pole");
  BRepPrimAPI_MakeRevol partial(partial_profile,
                                gp_Ax1(gp_Pnt(partial_x, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                                p(parameters, "partial_sweep"), true);

  const std::array<TopoDS_Shape, 2> children{
      checked_builder_shape(full, "solid.revolution.full_partial_pole"),
      checked_builder_shape(partial, "solid.revolution.full_partial_pole"),
  };
  return make_compound(children);
}

TopoDS_Shape make_sphere_matrix(const Parameters parameters) {
  BRepPrimAPI_MakeSphere full(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                              p(parameters, "full_radius"));
  BRepPrimAPI_MakeSphere hemisphere(
      gp_Ax2(gp_Pnt(p(parameters, "hemisphere_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "hemisphere_radius"), -0.5 * std::numbers::pi, 0.0);
  BRepPrimAPI_MakeSphere band(
      gp_Ax2(gp_Pnt(p(parameters, "band_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "band_radius"), p(parameters, "band_latitude_min"),
      p(parameters, "band_latitude_max"));

  const double seam_shift = p(parameters, "seam_shift");
  BRepPrimAPI_MakeSphere shifted(gp_Ax2(gp_Pnt(p(parameters, "shifted_x"), 0.0, 0.0),
                                        gp_Dir(0.0, 0.0, 1.0),
                                        gp_Dir(std::cos(seam_shift), std::sin(seam_shift), 0.0)),
                                 p(parameters, "shifted_radius"));

  const std::array<TopoDS_Shape, 4> children{
      checked_builder_shape(full, "solid.sphere.full_hemisphere_band_shifted_seam"),
      checked_builder_shape(hemisphere, "solid.sphere.full_hemisphere_band_shifted_seam"),
      checked_builder_shape(band, "solid.sphere.full_hemisphere_band_shifted_seam"),
      checked_builder_shape(shifted, "solid.sphere.full_hemisphere_band_shifted_seam"),
  };
  return make_compound(children);
}

TopoDS_Shape make_torus_matrix(const Parameters parameters) {
  BRepPrimAPI_MakeTorus ring(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                             p(parameters, "ring_major_radius"),
                             p(parameters, "ring_minor_radius"));
  BRepPrimAPI_MakeTorus horn(
      gp_Ax2(gp_Pnt(p(parameters, "horn_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "horn_radius"), p(parameters, "horn_radius"));
  BRepPrimAPI_MakeTorus spindle(
      gp_Ax2(gp_Pnt(p(parameters, "spindle_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "spindle_major_radius"), p(parameters, "spindle_minor_radius"));
  BRepPrimAPI_MakeTorus partial(
      gp_Ax2(gp_Pnt(p(parameters, "partial_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
      p(parameters, "partial_major_radius"), p(parameters, "partial_minor_radius"),
      p(parameters, "partial_v_min"), p(parameters, "partial_v_max"),
      p(parameters, "partial_sweep"));

  const std::array<TopoDS_Shape, 4> children{
      checked_builder_shape(ring, "solid.torus.ring_horn_spindle_full_partial"),
      checked_builder_shape(horn, "solid.torus.ring_horn_spindle_full_partial"),
      checked_builder_shape(spindle, "solid.torus.ring_horn_spindle_full_partial"),
      checked_builder_shape(partial, "solid.torus.ring_horn_spindle_full_partial"),
  };
  return make_compound(children);
}

TopoDS_Shape make_bezier_surface_matrix(const Parameters parameters) {
  const double spacing = p(parameters, "column_spacing");
  const double u_extent = p(parameters, "u_extent");
  const double v_extent = p(parameters, "v_extent");
  const double height = p(parameters, "control_height");
  std::array<TopoDS_Shape, 4> children;

  {
    const auto poles = bezier_surface_poles(2, 2, 0.0, 0.0, u_extent, v_extent, height);
    const occ::handle<Geom_Surface> surface = new Geom_BezierSurface(poles);
    children[0] = natural_face(surface, "face.bezier.matrix");
  }
  {
    const auto poles = bezier_surface_poles(3, 2, spacing, 0.0, u_extent, v_extent, height);
    const occ::handle<Geom_Surface> surface = new Geom_BezierSurface(poles);
    children[1] = natural_face(surface, "face.bezier.matrix");
  }
  {
    const auto poles = bezier_surface_poles(2, 3, 2.0 * spacing, 0.0, u_extent, v_extent, height);
    NCollection_Array2<double> weights(1, 3, 1, 4);
    for (int u = 1; u <= 3; ++u) {
      for (int v = 1; v <= 4; ++v) {
        const bool interior = u == 2 && (v == 2 || v == 3);
        weights.SetValue(u, v, interior ? p(parameters, "rational_light_weight") : 1.0);
      }
    }
    const occ::handle<Geom_Surface> surface = new Geom_BezierSurface(poles, weights);
    children[2] = natural_face(surface, "face.bezier.matrix");
  }
  {
    const auto poles =
        bezier_surface_poles(3, 3, 3.0 * spacing, 0.0, u_extent, v_extent, 1.25 * height);
    NCollection_Array2<double> weights(1, 4, 1, 4);
    for (int u = 1; u <= 4; ++u) {
      for (int v = 1; v <= 4; ++v) {
        const bool interior = (u == 2 || u == 3) && (v == 2 || v == 3);
        weights.SetValue(u, v, interior ? p(parameters, "rational_heavy_weight") : 1.0);
      }
    }
    const occ::handle<Geom_Surface> surface = new Geom_BezierSurface(poles, weights);
    children[3] = natural_face(surface, "face.bezier.matrix");
  }
  return make_compound(children);
}

occ::handle<Geom_BSplineSurface> nonrational_bspline_surface(const double x_offset,
                                                             const double height) {
  NCollection_Array2<gp_Pnt> poles(1, 4, 1, 4);
  for (int u = 1; u <= 4; ++u) {
    for (int v = 1; v <= 4; ++v) {
      const double fu = static_cast<double>(u - 1) / 3.0;
      const double fv = static_cast<double>(v - 1) / 3.0;
      poles.SetValue(
          u, v,
          gp_Pnt(x_offset + 18.0 * fu, 12.0 * fv,
                 height * std::sin(std::numbers::pi * fu) * std::sin(std::numbers::pi * fv)));
    }
  }
  NCollection_Array1<double> u_knots(1, 3);
  NCollection_Array1<double> v_knots(1, 3);
  NCollection_Array1<int> u_mults(1, 3);
  NCollection_Array1<int> v_mults(1, 3);
  u_knots.SetValue(1, 0.0);
  u_knots.SetValue(2, 0.42);
  u_knots.SetValue(3, 1.0);
  v_knots.SetValue(1, 0.0);
  v_knots.SetValue(2, 0.58);
  v_knots.SetValue(3, 1.0);
  for (int index = 1; index <= 3; ++index) {
    const int multiplicity = index == 2 ? 1 : 3;
    u_mults.SetValue(index, multiplicity);
    v_mults.SetValue(index, multiplicity);
  }
  return new Geom_BSplineSurface(poles, u_knots, v_knots, u_mults, v_mults, 2, 2, false, false);
}

occ::handle<Geom_BSplineSurface>
rational_bspline_surface(const double x_offset, const double height, const double light_weight) {
  NCollection_Array2<gp_Pnt> poles(1, 6, 1, 4);
  NCollection_Array2<double> weights(1, 6, 1, 4);
  for (int u = 1; u <= 6; ++u) {
    for (int v = 1; v <= 4; ++v) {
      const double fu = static_cast<double>(u - 1) / 5.0;
      const double fv = static_cast<double>(v - 1) / 3.0;
      poles.SetValue(
          u, v,
          gp_Pnt(x_offset + 20.0 * fu, 12.0 * fv,
                 height * std::sin(std::numbers::pi * fu) * std::sin(std::numbers::pi * fv)));
      weights.SetValue(u, v, (u == 3 || u == 4) && (v == 2 || v == 3) ? light_weight : 1.0);
    }
  }
  NCollection_Array1<double> u_knots(1, 4);
  NCollection_Array1<int> u_mults(1, 4);
  u_knots.SetValue(1, 0.0);
  u_knots.SetValue(2, 0.24);
  u_knots.SetValue(3, 0.73);
  u_knots.SetValue(4, 1.0);
  u_mults.SetValue(1, 4);
  u_mults.SetValue(2, 1);
  u_mults.SetValue(3, 1);
  u_mults.SetValue(4, 4);
  NCollection_Array1<double> v_knots(1, 3);
  NCollection_Array1<int> v_mults(1, 3);
  v_knots.SetValue(1, 0.0);
  v_knots.SetValue(2, 0.5);
  v_knots.SetValue(3, 1.0);
  v_mults.SetValue(1, 3);
  v_mults.SetValue(2, 1);
  v_mults.SetValue(3, 3);
  return new Geom_BSplineSurface(poles, weights, u_knots, v_knots, u_mults, v_mults, 3, 2, false,
                                 false);
}

occ::handle<Geom_BSplineSurface>
periodic_bspline_surface(const double x_offset, const double radius, const double height) {
  NCollection_Array2<gp_Pnt> poles(1, 5, 1, 4);
  for (int u = 0; u < 5; ++u) {
    const double angle = 2.0 * std::numbers::pi * static_cast<double>(u) / 5.0;
    for (int v = 0; v < 4; ++v) {
      const double fraction = static_cast<double>(v) / 3.0;
      const double local_radius = radius + 0.8 * std::sin(std::numbers::pi * fraction);
      poles.SetValue(u + 1, v + 1,
                     gp_Pnt(x_offset + local_radius * std::cos(angle),
                            local_radius * std::sin(angle), height * fraction));
    }
  }
  NCollection_Array1<double> u_knots(1, 6);
  NCollection_Array1<int> u_mults(1, 6);
  for (int index = 1; index <= 6; ++index) {
    u_knots.SetValue(index, static_cast<double>(index - 1));
    u_mults.SetValue(index, 1);
  }
  NCollection_Array1<double> v_knots(1, 3);
  NCollection_Array1<int> v_mults(1, 3);
  v_knots.SetValue(1, 0.0);
  v_knots.SetValue(2, 0.5);
  v_knots.SetValue(3, 1.0);
  v_mults.SetValue(1, 3);
  v_mults.SetValue(2, 1);
  v_mults.SetValue(3, 3);
  return new Geom_BSplineSurface(poles, u_knots, v_knots, u_mults, v_mults, 2, 2, true, false);
}

TopoDS_Shape make_bspline_surface_matrix(const Parameters parameters) {
  const double spacing = p(parameters, "column_spacing");
  const occ::handle<Geom_Surface> nonrational =
      nonrational_bspline_surface(0.0, p(parameters, "control_height"));
  const occ::handle<Geom_Surface> rational = rational_bspline_surface(
      spacing, p(parameters, "control_height"), p(parameters, "rational_inner_weight"));
  const occ::handle<Geom_Surface> periodic = periodic_bspline_surface(
      2.2 * spacing, p(parameters, "periodic_radius"), p(parameters, "periodic_height"));
  const std::array<TopoDS_Shape, 3> children{
      natural_face(nonrational, "face.bspline_nurbs.matrix"),
      natural_face(rational, "face.bspline_nurbs.matrix"),
      natural_face(periodic, "face.bspline_nurbs.matrix"),
  };
  return make_compound(children);
}

TopoDS_Shape make_offset_surface_matrix(const Parameters parameters) {
  const auto base_poles =
      bezier_surface_poles(3, 3, 0.0, 0.0, p(parameters, "base_u_extent"),
                           p(parameters, "base_v_extent"), p(parameters, "base_height"));
  const occ::handle<Geom_Surface> base = new Geom_BezierSurface(base_poles);
  const occ::handle<Geom_Surface> regular =
      new Geom_OffsetSurface(base, p(parameters, "regular_offset"));
  const occ::handle<Geom_Surface> nested =
      new Geom_OffsetSurface(regular, p(parameters, "nested_offset"));

  const double semi_angle = p(parameters, "singular_cone_semi_angle");
  const double reference_radius = p(parameters, "singular_cone_reference_radius");
  const double offset = p(parameters, "singular_offset");
  const occ::handle<Geom_Surface> cone = new Geom_ConicalSurface(
      gp_Ax3(gp_Pnt(p(parameters, "singular_x"), 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), semi_angle,
      reference_radius);
  const occ::handle<Geom_Surface> singular = new Geom_OffsetSurface(cone, offset);
  const double apex_v = -(reference_radius + offset * std::cos(semi_angle)) / std::sin(semi_angle);

  const std::array<TopoDS_Shape, 3> children{
      natural_face(regular, "face.offset.nested_regular_singular"),
      natural_face(nested, "face.offset.nested_regular_singular"),
      bounded_face(singular, 0.0, 2.0 * std::numbers::pi, apex_v,
                   apex_v + p(parameters, "singular_v_extent"),
                   "face.offset.nested_regular_singular"),
  };
  return make_compound(children);
}

TopoDS_Shape make_rectangular_trimmed_matrix(const Parameters parameters) {
  const occ::handle<Geom_Surface> cylinder = new Geom_CylindricalSurface(
      gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)), p(parameters, "cylinder_radius"));
  const occ::handle<Geom_Surface> cylinder_trim = new Geom_RectangularTrimmedSurface(
      cylinder, p(parameters, "cylinder_u_min"), p(parameters, "cylinder_u_max"),
      p(parameters, "cylinder_v_min"), p(parameters, "cylinder_v_max"));

  const occ::handle<Geom_Surface> bspline =
      nonrational_bspline_surface(p(parameters, "bspline_x"), p(parameters, "bspline_height"));
  const occ::handle<Geom_Surface> bspline_trim = new Geom_RectangularTrimmedSurface(
      bspline, p(parameters, "bspline_u_min"), p(parameters, "bspline_u_max"),
      p(parameters, "bspline_v_min"), p(parameters, "bspline_v_max"));

  const std::array<TopoDS_Shape, 2> children{
      natural_face(cylinder_trim, "face.rectangular_trimmed.wrapper"),
      natural_face(bspline_trim, "face.rectangular_trimmed.wrapper"),
  };
  return make_compound(children);
}

TopoDS_Shape make_ruled_swept_lofted(const Parameters parameters) {
  NCollection_Array1<gp_Pnt> first_poles(1, 4);
  NCollection_Array1<gp_Pnt> second_poles(1, 4);
  for (int index = 0; index < 4; ++index) {
    const double fraction = static_cast<double>(index) / 3.0;
    first_poles.SetValue(
        index + 1, gp_Pnt(16.0 * fraction, 0.0,
                          p(parameters, "ruled_camber") * std::sin(std::numbers::pi * fraction)));
    second_poles.SetValue(index + 1, gp_Pnt(16.0 * fraction, p(parameters, "ruled_separation"),
                                            p(parameters, "ruled_twist") * fraction));
  }
  const occ::handle<Geom_Curve> first_curve = new Geom_BezierCurve(first_poles);
  const occ::handle<Geom_Curve> second_curve = new Geom_BezierCurve(second_poles);
  const occ::handle<Geom_Surface> ruled = GeomFill::Surface(first_curve, second_curve);

  NCollection_Array1<gp_Pnt> sweep_poles(1, 4);
  sweep_poles.SetValue(1, gp_Pnt(p(parameters, "sweep_x"), 0.0, 0.0));
  sweep_poles.SetValue(2, gp_Pnt(p(parameters, "sweep_x") + 5.0, 0.0, 4.0));
  sweep_poles.SetValue(3, gp_Pnt(p(parameters, "sweep_x") + 11.0, 0.0, -2.0));
  sweep_poles.SetValue(4, gp_Pnt(p(parameters, "sweep_x") + 16.0, 0.0, 1.0));
  const occ::handle<Geom_Curve> sweep_curve = new Geom_BezierCurve(sweep_poles);
  const occ::handle<Geom_Surface> swept =
      new Geom_SurfaceOfLinearExtrusion(sweep_curve, gp_Dir(0.0, 1.0, 0.0));

  BRepOffsetAPI_ThruSections loft(false, false, p(parameters, "loft_tolerance"));
  loft.CheckCompatibility(true);
  loft.AddWire(circle_wire(gp_Pnt(p(parameters, "loft_x"), 0.0, 0.0),
                           p(parameters, "loft_radius_1"), "face.ruled_swept_lofted_plate"));
  loft.AddWire(circle_wire(gp_Pnt(p(parameters, "loft_x"), 0.0, p(parameters, "loft_height_2")),
                           p(parameters, "loft_radius_2"), "face.ruled_swept_lofted_plate"));
  loft.AddWire(circle_wire(gp_Pnt(p(parameters, "loft_x"), 0.0, p(parameters, "loft_height_3")),
                           p(parameters, "loft_radius_3"), "face.ruled_swept_lofted_plate"));
  loft.Build();

  const std::array<TopoDS_Shape, 3> children{
      natural_face(ruled, "face.ruled_swept_lofted_plate"),
      bounded_face(swept, 0.0, 1.0, 0.0, p(parameters, "sweep_distance"),
                   "face.ruled_swept_lofted_plate"),
      checked_builder_shape(loft, "face.ruled_swept_lofted_plate"),
  };
  return make_compound(children);
}

TopoDS_Shape make_uv_conditioning_matrix(const Parameters parameters) {
  NCollection_Array2<gp_Pnt> anisotropic_poles(1, 4, 1, 3);
  for (int u = 0; u < 4; ++u) {
    for (int v = 0; v < 3; ++v) {
      const double fu = static_cast<double>(u) / 3.0;
      const double fv = static_cast<double>(v) / 2.0;
      anisotropic_poles.SetValue(
          u + 1, v + 1,
          gp_Pnt(p(parameters, "anisotropic_u_extent") * fu,
                 p(parameters, "anisotropic_v_extent") * fv,
                 p(parameters, "anisotropic_height") * fu * (1.0 - fu) * fv));
    }
  }
  const occ::handle<Geom_Surface> anisotropic = new Geom_BezierSurface(anisotropic_poles);

  NCollection_Array2<gp_Pnt> folded_poles(1, 4, 1, 2);
  const double folded_x = p(parameters, "folded_x");
  const double fold_scale = p(parameters, "fold_scale");
  const double fold_width = p(parameters, "fold_width");
  for (int v = 0; v < 2; ++v) {
    folded_poles.SetValue(1, v + 1, gp_Pnt(folded_x, fold_width * v, 0.0));
    folded_poles.SetValue(2, v + 1, gp_Pnt(folded_x + fold_scale, fold_width * v, 0.0));
    folded_poles.SetValue(3, v + 1, gp_Pnt(folded_x - fold_scale, fold_width * v, 0.0));
    folded_poles.SetValue(4, v + 1, gp_Pnt(folded_x + 2.0 * fold_scale, fold_width * v, 0.0));
  }
  const occ::handle<Geom_Surface> folded = new Geom_BezierSurface(folded_poles);

  const std::array<TopoDS_Shape, 2> children{
      natural_face(anisotropic, "face.uv.anisotropic_folded"),
      natural_face(folded, "face.uv.anisotropic_folded"),
  };
  return make_compound(children);
}

enum class PatchJoin { c0, c1, c2, cn, g0, g1, g2 };

struct ControlPoint2 final {
  double x{};
  double z{};
};

TopoDS_Shape exact_patch_pair(const PatchJoin join, const double y_offset, const double width,
                              const double g_scale, const double curvature,
                              const double sewing_tolerance) {
  std::array<ControlPoint2, 4> left{{{-3.0, 0.7}, {-2.0, curvature}, {-1.0, 0.0}, {0.0, 0.0}}};
  std::array<ControlPoint2, 4> right{{{0.0, 0.0}, {1.0, 0.0}, {2.0, -0.35}, {3.0, 0.6}}};
  GeomAbs_Shape continuity = GeomAbs_C0;

  switch (join) {
  case PatchJoin::c0:
  case PatchJoin::g0:
    right[1] = {1.0, 0.45};
    continuity = GeomAbs_C0;
    break;
  case PatchJoin::c1:
    right[1] = {1.0, 0.0};
    right[2] = {2.0, -curvature};
    continuity = GeomAbs_C1;
    break;
  case PatchJoin::c2:
    right[1] = {1.0, 0.0};
    right[2] = {2.0, curvature};
    continuity = GeomAbs_C2;
    break;
  case PatchJoin::cn:
    for (ControlPoint2& point : left) {
      point.z = 0.0;
    }
    for (ControlPoint2& point : right) {
      point.z = 0.0;
    }
    continuity = GeomAbs_CN;
    break;
  case PatchJoin::g1:
    right[1] = {g_scale, 0.0};
    right[2] = {2.0 * g_scale, -curvature};
    continuity = GeomAbs_G1;
    break;
  case PatchJoin::g2:
    right[1] = {g_scale, 0.0};
    right[2] = {2.0 * g_scale, g_scale * g_scale * curvature};
    continuity = GeomAbs_G2;
    break;
  }

  NCollection_Array2<gp_Pnt> left_poles(1, 4, 1, 2);
  NCollection_Array2<gp_Pnt> right_poles(1, 4, 1, 2);
  for (int u = 0; u < 4; ++u) {
    for (int v = 0; v < 2; ++v) {
      left_poles.SetValue(u + 1, v + 1, gp_Pnt(left[u].x, y_offset + width * v, left[u].z));
      right_poles.SetValue(u + 1, v + 1, gp_Pnt(right[u].x, y_offset + width * v, right[u].z));
    }
  }
  const occ::handle<Geom_Surface> left_surface = new Geom_BezierSurface(left_poles);
  const occ::handle<Geom_Surface> right_surface = new Geom_BezierSurface(right_poles);
  const TopoDS_Face left_face = natural_face(left_surface, "patch.continuity.c0_c1_c2_cn_g0_g1_g2");
  const TopoDS_Face right_face =
      natural_face(right_surface, "patch.continuity.c0_c1_c2_cn_g0_g1_g2");

  BRepBuilderAPI_Sewing sewing(sewing_tolerance, true, true, true, false);
  sewing.Add(left_face);
  sewing.Add(right_face);
  sewing.Perform();
  const TopoDS_Shape sewed = sewing.SewedShape();
  if (sewed.IsNull()) {
    throw ConstructionFailure(
        "patch.continuity.c0_c1_c2_cn_g0_g1_g2: sewing returned a null shape");
  }

  std::vector<TopoDS_Face> faces;
  for (TopExp_Explorer explorer(sewed, TopAbs_FACE); explorer.More(); explorer.Next()) {
    faces.push_back(TopoDS::Face(explorer.Current()));
  }
  if (faces.size() != 2) {
    throw ConstructionFailure(
        "patch.continuity.c0_c1_c2_cn_g0_g1_g2: expected exactly two sewn faces");
  }

  TopoDS_Edge shared_edge;
  int shared_count = 0;
  for (TopExp_Explorer first(faces[0], TopAbs_EDGE); first.More(); first.Next()) {
    const TopoDS_Edge candidate = TopoDS::Edge(first.Current());
    for (TopExp_Explorer second(faces[1], TopAbs_EDGE); second.More(); second.Next()) {
      if (candidate.IsSame(second.Current())) {
        shared_edge = candidate;
        ++shared_count;
      }
    }
  }
  if (shared_count != 1 || shared_edge.IsNull()) {
    throw ConstructionFailure(
        "patch.continuity.c0_c1_c2_cn_g0_g1_g2: expected one shared patch edge");
  }

  // These labels are not synthetic: the Bezier rows above encode coincident boundary
  // points, proportional first derivatives for G1, the squared reparameterization scale
  // for G2, and identical first/second derivatives for C1/C2.  CN is a single affine
  // polynomial split into two parameter intervals.  OCCT stores that exact regularity on
  // the shared edge, which is the representation consumed by BRep_Tool::Continuity.
  BRep_Builder builder;
  builder.Continuity(shared_edge, faces[0], faces[1], continuity);
  return sewed;
}

TopoDS_Shape make_patch_continuity_matrix(const Parameters parameters) {
  const double spacing = p(parameters, "row_spacing");
  const double width = p(parameters, "patch_width");
  const double g_scale = p(parameters, "geometric_scale");
  const double curvature = p(parameters, "curvature_control");
  const double tolerance = p(parameters, "sewing_tolerance");
  const std::array<PatchJoin, 7> joins{PatchJoin::c0, PatchJoin::c1, PatchJoin::c2, PatchJoin::cn,
                                       PatchJoin::g0, PatchJoin::g1, PatchJoin::g2};
  std::array<TopoDS_Shape, 7> children;
  for (std::size_t index = 0; index < joins.size(); ++index) {
    children[index] = exact_patch_pair(joins[index], spacing * static_cast<double>(index), width,
                                       g_scale, curvature, tolerance);
  }
  return make_compound(children);
}

RoundTripContract standalone_face_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.standalone_face_wrapped_in_shell",
          },
  };
}

RoundTripContract standalone_bezier_face_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.curve.bezier_promoted_to_bspline",
              "step.standalone_face_wrapped_in_shell",
              "step.surface.bezier_promoted_to_bspline",
          },
  };
}

RoundTripContract periodic_band_contract() {
  return RoundTripContract{
      .expected_brep_valid = false,
      .topology_counts = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.standalone_face_wrapped_in_shell",
              "step.surface.periodic_domain_rebased",
              "step.surface.periodic_seam_import_invalid",
              "step.surface.periodic_seam_shifted",
          },
  };
}

RoundTripContract reversed_surface_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.curve.bezier_promoted_to_bspline",
              "step.standalone_face_orientation_canonicalized",
              "step.standalone_face_wrapped_in_shell",
              "step.surface.bezier_promoted_to_bspline",
          },
  };
}

RoundTripContract cone_apex_contract() {
  return RoundTripContract{
      .expected_brep_valid = false,
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.surface.cone_apex_import_invalid",
              "step.surface.cone_apex_lateral_face_omitted",
          },
  };
}

RoundTripContract revolution_pole_contract() {
  return RoundTripContract{
      .expected_brep_valid = false,
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.curve.bezier_promoted_to_bspline",
              "step.surface.revolution_pole_import_invalid",
              "step.surface.revolution_pole_topology_resegmented",
          },
  };
}

RoundTripContract sphere_matrix_contract() {
  return RoundTripContract{
      .expected_brep_valid = false,
      .topology_counts = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.curve.trimmed_wrapper_canonicalized",
              "step.surface.sphere_pole_import_invalid",
              "step.surface.sphere_pole_reparameterized",
              "step.surface.sphere_pole_topology_resegmented",
              "step.surface.sphere_seam_shifted",
          },
  };
}

RoundTripContract torus_matrix_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.surface.singular_torus_represented_as_revolution",
              "step.surface.singular_torus_resegmented",
              "step.surface.torus_seam_shifted",
          },
  };
}

RoundTripContract offset_surface_contract() {
  return RoundTripContract{
      .expected_brep_valid = false,
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .tolerances = false,
      .accepted_normalization_codes =
          {
              "step.standalone_face_wrapped_in_shell",
              "step.surface.nested_offset_combined",
              "step.surface.offset_canonicalized",
              "step.surface.singular_conical_offset_canonicalized",
              "step.surface.singular_offset_import_invalid",
              "step.tolerances_recomputed_from_uncertainty",
          },
  };
}

RoundTripContract rectangular_trim_contract() {
  return RoundTripContract{
      .topology_counts = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.standalone_face_wrapped_in_shell",
              "step.surface.rectangular_trim_unwrapped",
              "step.surface.trim_domain_rebased",
          },
  };
}

RoundTripContract ruled_swept_lofted_contract() {
  return RoundTripContract{
      .expected_brep_valid = false,
      .topology_counts = false,
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .model_topology = false,
      .accepted_normalization_codes =
          {
              "step.curve.bezier_promoted_to_bspline",
              "step.standalone_face_wrapped_in_shell",
              "step.surface.loft_bspline_reparameterized",
              "step.surface.ruled_analyticized",
              "step.surface.ruled_swept_lofted_import_invalid",
          },
  };
}

RoundTripContract patch_continuity_contract() {
  return RoundTripContract{
      .geometry_families = false,
      .concrete_types = false,
      .occurrence_graph = false,
      .boundary_loops = false,
      .accepted_normalization_codes =
          {
              "step.curve.bezier_promoted_to_bspline",
              "step.surface.bezier_promoted_to_bspline",
              "step.surface.continuity_metadata_recomputed",
              "step.surface.shared_edge_resegmented",
          },
  };
}

} // namespace

void append_surface_recipes(std::vector<Recipe>& recipes) {
  recipes.push_back(procedural_shape_recipe("face.periodic.band_seam_crossing",
                                            {{"radius", 7.0, "millimetres"},
                                             {"u_max", 7.15, "radians"},
                                             {"u_min", 5.55, "radians"},
                                             {"v_max", 11.0, "millimetres"},
                                             {"v_min", -3.0, "millimetres"}},
                                            make_periodic_band, {}, periodic_band_contract()));

  recipes.push_back(procedural_shape_recipe("face.surface.reversed_orientation",
                                            {{"bezier_height", 3.0, "millimetres"},
                                             {"bezier_u_extent", 12.0, "millimetres"},
                                             {"bezier_v_extent", 8.0, "millimetres"},
                                             {"bezier_x", 42.0, "millimetres"},
                                             {"cylinder_height", 9.0, "millimetres"},
                                             {"cylinder_radius", 5.0, "millimetres"},
                                             {"cylinder_sweep", 4.8, "radians"},
                                             {"cylinder_x", 24.0, "millimetres"},
                                             {"plane_u_extent", 12.0, "millimetres"},
                                             {"plane_v_extent", 8.0, "millimetres"}},
                                            make_reversed_surface_matrix, {},
                                            reversed_surface_contract()));

  recipes.push_back(procedural_shape_recipe("solid.cone.apex_full_partial",
                                            {{"full_height", 16.0, "millimetres"},
                                             {"full_radius", 7.0, "millimetres"},
                                             {"partial_height", 14.0, "millimetres"},
                                             {"partial_radius", 6.0, "millimetres"},
                                             {"partial_sweep", 4.4, "radians"},
                                             {"partial_x", 24.0, "millimetres"}},
                                            make_apex_cones, {}, cone_apex_contract()));

  recipes.push_back(procedural_shape_recipe("solid.revolution.full_partial_pole",
                                            {{"full_bulge", 0.28, "unitless"},
                                             {"full_height", 18.0, "millimetres"},
                                             {"full_radius", 8.0, "millimetres"},
                                             {"partial_bulge", 0.36, "unitless"},
                                             {"partial_height", 16.0, "millimetres"},
                                             {"partial_radius", 7.0, "millimetres"},
                                             {"partial_sweep", 4.7, "radians"},
                                             {"partial_x", 27.0, "millimetres"}},
                                            make_revolution_matrix, {},
                                            revolution_pole_contract()));

  recipes.push_back(procedural_shape_recipe("solid.sphere.full_hemisphere_band_shifted_seam",
                                            {{"band_latitude_max", 0.65, "radians"},
                                             {"band_latitude_min", -0.42, "radians"},
                                             {"band_radius", 5.0, "millimetres"},
                                             {"band_x", 32.0, "millimetres"},
                                             {"full_radius", 6.0, "millimetres"},
                                             {"hemisphere_radius", 5.5, "millimetres"},
                                             {"hemisphere_x", 16.0, "millimetres"},
                                             {"seam_shift", 0.73, "radians"},
                                             {"shifted_radius", 5.0, "millimetres"},
                                             {"shifted_x", 48.0, "millimetres"}},
                                            make_sphere_matrix, {}, sphere_matrix_contract()));

  recipes.push_back(procedural_shape_recipe("solid.torus.ring_horn_spindle_full_partial",
                                            {{"horn_radius", 4.0, "millimetres"},
                                             {"horn_x", 35.0, "millimetres"},
                                             {"partial_major_radius", 9.0, "millimetres"},
                                             {"partial_minor_radius", 3.0, "millimetres"},
                                             {"partial_sweep", 4.6, "radians"},
                                             {"partial_v_max", 2.1, "radians"},
                                             {"partial_v_min", -1.4, "radians"},
                                             {"partial_x", 88.0, "millimetres"},
                                             {"ring_major_radius", 10.0, "millimetres"},
                                             {"ring_minor_radius", 3.0, "millimetres"},
                                             {"spindle_major_radius", 3.0, "millimetres"},
                                             {"spindle_minor_radius", 5.0, "millimetres"},
                                             {"spindle_x", 62.0, "millimetres"}},
                                            make_torus_matrix, {}, torus_matrix_contract()));

  recipes.push_back(procedural_shape_recipe("face.bezier.matrix",
                                            {{"column_spacing", 25.0, "millimetres"},
                                             {"control_height", 4.5, "millimetres"},
                                             {"rational_heavy_weight", 1.8, "unitless"},
                                             {"rational_light_weight", 0.45, "unitless"},
                                             {"u_extent", 18.0, "millimetres"},
                                             {"v_extent", 12.0, "millimetres"}},
                                            make_bezier_surface_matrix, {},
                                            standalone_bezier_face_contract()));

  recipes.push_back(procedural_shape_recipe("face.bspline_nurbs.matrix",
                                            {{"column_spacing", 29.0, "millimetres"},
                                             {"control_height", 4.0, "millimetres"},
                                             {"periodic_height", 12.0, "millimetres"},
                                             {"periodic_radius", 6.0, "millimetres"},
                                             {"rational_inner_weight", 0.52, "unitless"}},
                                            make_bspline_surface_matrix, {},
                                            standalone_face_contract()));

  recipes.push_back(procedural_shape_recipe("face.offset.nested_regular_singular",
                                            {{"base_height", 3.0, "millimetres"},
                                             {"base_u_extent", 16.0, "millimetres"},
                                             {"base_v_extent", 12.0, "millimetres"},
                                             {"nested_offset", 0.8, "millimetres"},
                                             {"regular_offset", 1.4, "millimetres"},
                                             {"singular_cone_reference_radius", 5.0, "millimetres"},
                                             {"singular_cone_semi_angle", 0.48, "radians"},
                                             {"singular_offset", -1.25, "millimetres"},
                                             {"singular_v_extent", 13.0, "millimetres"},
                                             {"singular_x", 34.0, "millimetres"}},
                                            make_offset_surface_matrix, {},
                                            offset_surface_contract()));

  recipes.push_back(procedural_shape_recipe("face.rectangular_trimmed.wrapper",
                                            {{"bspline_height", 3.5, "millimetres"},
                                             {"bspline_u_max", 0.82, "unitless"},
                                             {"bspline_u_min", 0.14, "unitless"},
                                             {"bspline_v_max", 0.9, "unitless"},
                                             {"bspline_v_min", 0.18, "unitless"},
                                             {"bspline_x", 28.0, "millimetres"},
                                             {"cylinder_radius", 5.0, "millimetres"},
                                             {"cylinder_u_max", 5.6, "radians"},
                                             {"cylinder_u_min", 0.45, "radians"},
                                             {"cylinder_v_max", 10.0, "millimetres"},
                                             {"cylinder_v_min", -2.0, "millimetres"}},
                                            make_rectangular_trimmed_matrix, {},
                                            rectangular_trim_contract()));

  recipes.push_back(procedural_shape_recipe("face.ruled_swept_lofted_plate",
                                            {{"loft_height_2", 8.0, "millimetres"},
                                             {"loft_height_3", 18.0, "millimetres"},
                                             {"loft_radius_1", 4.0, "millimetres"},
                                             {"loft_radius_2", 6.0, "millimetres"},
                                             {"loft_radius_3", 3.5, "millimetres"},
                                             {"loft_tolerance", 1.0e-7, "millimetres"},
                                             {"loft_x", 58.0, "millimetres"},
                                             {"ruled_camber", 3.0, "millimetres"},
                                             {"ruled_separation", 10.0, "millimetres"},
                                             {"ruled_twist", 4.0, "millimetres"},
                                             {"sweep_distance", 10.0, "millimetres"},
                                             {"sweep_x", 28.0, "millimetres"}},
                                            make_ruled_swept_lofted, {},
                                            ruled_swept_lofted_contract()));

  recipes.push_back(procedural_shape_recipe("face.uv.anisotropic_folded",
                                            {{"anisotropic_height", 2.0, "millimetres"},
                                             {"anisotropic_u_extent", 48.0, "millimetres"},
                                             {"anisotropic_v_extent", 1.2, "millimetres"},
                                             {"fold_scale", 7.0, "millimetres"},
                                             {"fold_width", 10.0, "millimetres"},
                                             {"folded_x", 65.0, "millimetres"}},
                                            make_uv_conditioning_matrix, {},
                                            standalone_bezier_face_contract()));

  recipes.push_back(procedural_shape_recipe("patch.continuity.c0_c1_c2_cn_g0_g1_g2",
                                            {{"curvature_control", 0.36, "millimetres"},
                                             {"geometric_scale", 1.65, "unitless"},
                                             {"patch_width", 5.0, "millimetres"},
                                             {"row_spacing", 8.0, "millimetres"},
                                             {"sewing_tolerance", 1.0e-8, "millimetres"}},
                                            make_patch_continuity_matrix, {},
                                            patch_continuity_contract()));
}

} // namespace cad_mesher::fixtures
