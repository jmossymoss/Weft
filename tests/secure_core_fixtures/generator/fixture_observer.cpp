#include "fixture_observer.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Hyperbola.hxx>
#include <Geom_Line.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_Parabola.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_SurfaceOfLinearExtrusion.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace cad_mesher::fixtures {
namespace {

template <typename Shape> Shape cast_shape(const TopoDS_Shape& shape);

template <> TopoDS_Edge cast_shape<TopoDS_Edge>(const TopoDS_Shape& shape) {
  return TopoDS::Edge(shape);
}

template <> TopoDS_Face cast_shape<TopoDS_Face>(const TopoDS_Shape& shape) {
  return TopoDS::Face(shape);
}

template <> TopoDS_Vertex cast_shape<TopoDS_Vertex>(const TopoDS_Shape& shape) {
  return TopoDS::Vertex(shape);
}

template <typename Shape>
std::vector<Shape> unique_partner_shapes(const TopoDS_Shape& root, const TopAbs_ShapeEnum type) {
  std::vector<Shape> result;
  for (TopExp_Explorer explorer(root, type); explorer.More(); explorer.Next()) {
    const Shape candidate = cast_shape<Shape>(explorer.Current());
    const bool known = std::any_of(result.begin(), result.end(), [&candidate](const Shape& value) {
      return value.IsPartner(candidate);
    });
    if (!known) {
      result.push_back(candidate);
    }
  }
  return result;
}

template <typename Shape>
int partner_ordinal(const Shape& candidate, const std::vector<Shape>& values) {
  const auto found = std::find_if(values.begin(), values.end(), [&candidate](const Shape& value) {
    return value.IsPartner(candidate);
  });
  if (found == values.end()) {
    throw std::runtime_error("fixture observer could not bind a partner shape");
  }
  return static_cast<int>(std::distance(values.begin(), found)) + 1;
}

std::string curve_family(const GeomAbs_CurveType type) {
  switch (type) {
  case GeomAbs_Line:
    return "line";
  case GeomAbs_Circle:
    return "circle";
  case GeomAbs_Ellipse:
    return "ellipse";
  case GeomAbs_Hyperbola:
    return "hyperbola";
  case GeomAbs_Parabola:
    return "parabola";
  case GeomAbs_BezierCurve:
    return "bezier";
  case GeomAbs_BSplineCurve:
    return "bspline";
  case GeomAbs_OffsetCurve:
    return "offset";
  case GeomAbs_OtherCurve:
    return "kernel_specific";
  }
  return "kernel_specific";
}

std::string surface_family(const GeomAbs_SurfaceType type) {
  switch (type) {
  case GeomAbs_Plane:
    return "plane";
  case GeomAbs_Cylinder:
    return "cylinder";
  case GeomAbs_Cone:
    return "cone";
  case GeomAbs_Sphere:
    return "sphere";
  case GeomAbs_Torus:
    return "torus";
  case GeomAbs_BezierSurface:
    return "bezier";
  case GeomAbs_BSplineSurface:
    return "bspline";
  case GeomAbs_SurfaceOfRevolution:
    return "revolution";
  case GeomAbs_SurfaceOfExtrusion:
    return "extrusion";
  case GeomAbs_OffsetSurface:
    return "offset";
  case GeomAbs_OtherSurface:
    return "kernel_specific";
  }
  return "kernel_specific";
}

std::string orientation_name(const TopAbs_Orientation orientation) {
  switch (orientation) {
  case TopAbs_FORWARD:
    return "FORWARD";
  case TopAbs_REVERSED:
    return "REVERSED";
  case TopAbs_INTERNAL:
    return "INTERNAL";
  case TopAbs_EXTERNAL:
    return "EXTERNAL";
  }
  return "UNKNOWN";
}

std::string continuity_name(const GeomAbs_Shape continuity) {
  switch (continuity) {
  case GeomAbs_C0:
    return "C0";
  case GeomAbs_G1:
    return "G1";
  case GeomAbs_C1:
    return "C1";
  case GeomAbs_G2:
    return "G2";
  case GeomAbs_C2:
    return "C2";
  case GeomAbs_C3:
    return "C3";
  case GeomAbs_CN:
    return "CN";
  }
  return "unknown";
}

void add_numeric(GeometryObservation& observation, std::string code, const double value,
                 std::string unit) {
  if (!std::isfinite(value)) {
    return;
  }
  observation.numeric_properties.push_back(
      NumericPropertyObservation{std::move(code), value, std::move(unit)});
}

void add_integer(GeometryObservation& observation, std::string code, const int value) {
  observation.integer_properties.push_back(IntegerPropertyObservation{std::move(code), value});
}

void add_boolean(GeometryObservation& observation, std::string code, const bool value) {
  observation.boolean_properties.push_back(BooleanPropertyObservation{std::move(code), value});
}

void add_numeric_sequence(GeometryObservation& observation, std::string code,
                          std::vector<double> values, std::string unit) {
  if (std::all_of(values.begin(), values.end(),
                  [](const double value) { return std::isfinite(value); })) {
    observation.numeric_sequences.push_back(
        NumericSequenceObservation{std::move(code), std::move(values), std::move(unit)});
  }
}

void add_integer_sequence(GeometryObservation& observation, std::string code,
                          std::vector<int> values) {
  observation.integer_sequences.push_back(
      IntegerSequenceObservation{std::move(code), std::move(values)});
}

occ::handle<Geom_Curve> unwrap_curve(occ::handle<Geom_Curve> curve,
                                     GeometryObservation& observation) {
  for (int depth = 0; depth < 32 && !curve.IsNull(); ++depth) {
    const occ::handle<Geom_TrimmedCurve> trimmed = occ::down_cast<Geom_TrimmedCurve>(curve);
    if (!trimmed.IsNull()) {
      observation.wrapper_stack.emplace_back(trimmed->DynamicType()->Name());
      const std::string prefix = "wrapper." + std::to_string(depth) + ".trimmed";
      add_numeric(observation, prefix + ".lower", trimmed->FirstParameter(), "curve_parameter");
      add_numeric(observation, prefix + ".upper", trimmed->LastParameter(), "curve_parameter");
      curve = trimmed->BasisCurve();
      continue;
    }
    const occ::handle<Geom_OffsetCurve> offset = occ::down_cast<Geom_OffsetCurve>(curve);
    if (!offset.IsNull()) {
      observation.wrapper_stack.emplace_back(offset->DynamicType()->Name());
      const std::string prefix = "wrapper." + std::to_string(depth) + ".offset";
      add_numeric(observation, prefix + ".distance", offset->Offset(), "millimetre");
      add_numeric(observation, prefix + ".direction_x", offset->Direction().X(), "unitless");
      add_numeric(observation, prefix + ".direction_y", offset->Direction().Y(), "unitless");
      add_numeric(observation, prefix + ".direction_z", offset->Direction().Z(), "unitless");
      curve = offset->BasisCurve();
      continue;
    }
    break;
  }
  return curve;
}

occ::handle<Geom_Surface> unwrap_surface(occ::handle<Geom_Surface> surface,
                                         GeometryObservation& observation) {
  for (int depth = 0; depth < 32 && !surface.IsNull(); ++depth) {
    const occ::handle<Geom_RectangularTrimmedSurface> trimmed =
        occ::down_cast<Geom_RectangularTrimmedSurface>(surface);
    if (!trimmed.IsNull()) {
      observation.wrapper_stack.emplace_back(trimmed->DynamicType()->Name());
      const std::string prefix = "wrapper." + std::to_string(depth) + ".rectangular_trim";
      double u_min = 0.0;
      double u_max = 0.0;
      double v_min = 0.0;
      double v_max = 0.0;
      trimmed->Bounds(u_min, u_max, v_min, v_max);
      add_numeric(observation, prefix + ".u_lower", u_min, "surface_parameter");
      add_numeric(observation, prefix + ".u_upper", u_max, "surface_parameter");
      add_numeric(observation, prefix + ".v_lower", v_min, "surface_parameter");
      add_numeric(observation, prefix + ".v_upper", v_max, "surface_parameter");
      surface = trimmed->BasisSurface();
      continue;
    }
    const occ::handle<Geom_OffsetSurface> offset = occ::down_cast<Geom_OffsetSurface>(surface);
    if (!offset.IsNull()) {
      observation.wrapper_stack.emplace_back(offset->DynamicType()->Name());
      const std::string prefix = "wrapper." + std::to_string(depth) + ".offset";
      add_numeric(observation, prefix + ".distance", offset->Offset(), "millimetre");
      surface = offset->BasisSurface();
      continue;
    }
    break;
  }
  return surface;
}

void observe_curve_parameters(const occ::handle<Geom_Curve>& curve,
                              GeometryObservation& observation) {
  if (curve.IsNull()) {
    return;
  }
  if (const auto circle = occ::down_cast<Geom_Circle>(curve); !circle.IsNull()) {
    add_numeric(observation, "circle.radius", circle->Radius(), "millimetre");
  } else if (const auto ellipse = occ::down_cast<Geom_Ellipse>(curve); !ellipse.IsNull()) {
    add_numeric(observation, "ellipse.major_radius", ellipse->MajorRadius(), "millimetre");
    add_numeric(observation, "ellipse.minor_radius", ellipse->MinorRadius(), "millimetre");
  } else if (const auto hyperbola = occ::down_cast<Geom_Hyperbola>(curve); !hyperbola.IsNull()) {
    add_numeric(observation, "hyperbola.major_radius", hyperbola->MajorRadius(), "millimetre");
    add_numeric(observation, "hyperbola.minor_radius", hyperbola->MinorRadius(), "millimetre");
  } else if (const auto parabola = occ::down_cast<Geom_Parabola>(curve); !parabola.IsNull()) {
    add_numeric(observation, "parabola.focal_length", parabola->Focal(), "millimetre");
  } else if (const auto bezier = occ::down_cast<Geom_BezierCurve>(curve); !bezier.IsNull()) {
    add_integer(observation, "bezier.degree", bezier->Degree());
    add_integer(observation, "bezier.pole_count", bezier->NbPoles());
    add_boolean(observation, "bezier.rational", bezier->IsRational());
    std::vector<double> weights;
    weights.reserve(static_cast<std::size_t>(bezier->NbPoles()));
    for (int index = 1; index <= bezier->NbPoles(); ++index) {
      weights.push_back(bezier->Weight(index));
    }
    add_numeric_sequence(observation, "bezier.weights", std::move(weights), "unitless");
  } else if (const auto bspline = occ::down_cast<Geom_BSplineCurve>(curve); !bspline.IsNull()) {
    add_integer(observation, "bspline.degree", bspline->Degree());
    add_integer(observation, "bspline.pole_count", bspline->NbPoles());
    add_integer(observation, "bspline.knot_count", bspline->NbKnots());
    add_boolean(observation, "bspline.rational", bspline->IsRational());
    add_boolean(observation, "bspline.periodic", bspline->IsPeriodic());
    std::vector<double> knots;
    std::vector<int> multiplicities;
    for (int index = 1; index <= bspline->NbKnots(); ++index) {
      knots.push_back(bspline->Knot(index));
      multiplicities.push_back(bspline->Multiplicity(index));
    }
    std::vector<double> weights;
    for (int index = 1; index <= bspline->NbPoles(); ++index) {
      weights.push_back(bspline->Weight(index));
    }
    add_numeric_sequence(observation, "bspline.knots", std::move(knots), "curve_parameter");
    add_integer_sequence(observation, "bspline.multiplicities", std::move(multiplicities));
    add_numeric_sequence(observation, "bspline.weights", std::move(weights), "unitless");
  }
}

void observe_surface_parameters(const occ::handle<Geom_Surface>& surface,
                                GeometryObservation& observation) {
  if (surface.IsNull()) {
    return;
  }
  if (const auto cylinder = occ::down_cast<Geom_CylindricalSurface>(surface); !cylinder.IsNull()) {
    add_numeric(observation, "cylinder.radius", cylinder->Radius(), "millimetre");
  } else if (const auto cone = occ::down_cast<Geom_ConicalSurface>(surface); !cone.IsNull()) {
    add_numeric(observation, "cone.reference_radius", cone->RefRadius(), "millimetre");
    add_numeric(observation, "cone.semi_angle", cone->SemiAngle(), "radian");
  } else if (const auto sphere = occ::down_cast<Geom_SphericalSurface>(surface); !sphere.IsNull()) {
    add_numeric(observation, "sphere.radius", sphere->Radius(), "millimetre");
  } else if (const auto torus = occ::down_cast<Geom_ToroidalSurface>(surface); !torus.IsNull()) {
    add_numeric(observation, "torus.major_radius", torus->MajorRadius(), "millimetre");
    add_numeric(observation, "torus.minor_radius", torus->MinorRadius(), "millimetre");
  } else if (const auto bezier = occ::down_cast<Geom_BezierSurface>(surface); !bezier.IsNull()) {
    add_integer(observation, "bezier.u_degree", bezier->UDegree());
    add_integer(observation, "bezier.v_degree", bezier->VDegree());
    add_integer(observation, "bezier.u_pole_count", bezier->NbUPoles());
    add_integer(observation, "bezier.v_pole_count", bezier->NbVPoles());
    add_boolean(observation, "bezier.u_rational", bezier->IsURational());
    add_boolean(observation, "bezier.v_rational", bezier->IsVRational());
    std::vector<double> weights;
    for (int u = 1; u <= bezier->NbUPoles(); ++u) {
      for (int v = 1; v <= bezier->NbVPoles(); ++v) {
        weights.push_back(bezier->Weight(u, v));
      }
    }
    add_numeric_sequence(observation, "bezier.weights_u_major", std::move(weights), "unitless");
  } else if (const auto bspline = occ::down_cast<Geom_BSplineSurface>(surface); !bspline.IsNull()) {
    add_integer(observation, "bspline.u_degree", bspline->UDegree());
    add_integer(observation, "bspline.v_degree", bspline->VDegree());
    add_integer(observation, "bspline.u_pole_count", bspline->NbUPoles());
    add_integer(observation, "bspline.v_pole_count", bspline->NbVPoles());
    add_integer(observation, "bspline.u_knot_count", bspline->NbUKnots());
    add_integer(observation, "bspline.v_knot_count", bspline->NbVKnots());
    add_boolean(observation, "bspline.u_rational", bspline->IsURational());
    add_boolean(observation, "bspline.v_rational", bspline->IsVRational());
    add_boolean(observation, "bspline.u_periodic", bspline->IsUPeriodic());
    add_boolean(observation, "bspline.v_periodic", bspline->IsVPeriodic());
    std::vector<double> u_knots;
    std::vector<double> v_knots;
    std::vector<int> u_multiplicities;
    std::vector<int> v_multiplicities;
    for (int index = 1; index <= bspline->NbUKnots(); ++index) {
      u_knots.push_back(bspline->UKnot(index));
      u_multiplicities.push_back(bspline->UMultiplicity(index));
    }
    for (int index = 1; index <= bspline->NbVKnots(); ++index) {
      v_knots.push_back(bspline->VKnot(index));
      v_multiplicities.push_back(bspline->VMultiplicity(index));
    }
    std::vector<double> weights;
    for (int u = 1; u <= bspline->NbUPoles(); ++u) {
      for (int v = 1; v <= bspline->NbVPoles(); ++v) {
        weights.push_back(bspline->Weight(u, v));
      }
    }
    add_numeric_sequence(observation, "bspline.u_knots", std::move(u_knots), "surface_parameter");
    add_numeric_sequence(observation, "bspline.v_knots", std::move(v_knots), "surface_parameter");
    add_integer_sequence(observation, "bspline.u_multiplicities", std::move(u_multiplicities));
    add_integer_sequence(observation, "bspline.v_multiplicities", std::move(v_multiplicities));
    add_numeric_sequence(observation, "bspline.weights_u_major", std::move(weights), "unitless");
  } else if (const auto extrusion = occ::down_cast<Geom_SurfaceOfLinearExtrusion>(surface);
             !extrusion.IsNull()) {
    add_numeric(observation, "extrusion.direction_x", extrusion->Direction().X(), "unitless");
    add_numeric(observation, "extrusion.direction_y", extrusion->Direction().Y(), "unitless");
    add_numeric(observation, "extrusion.direction_z", extrusion->Direction().Z(), "unitless");
    observation.wrapper_stack.emplace_back(extrusion->BasisCurve()->DynamicType()->Name());
  } else if (const auto revolution = occ::down_cast<Geom_SurfaceOfRevolution>(surface);
             !revolution.IsNull()) {
    add_numeric(observation, "revolution.axis_direction_x", revolution->Axis().Direction().X(),
                "unitless");
    add_numeric(observation, "revolution.axis_direction_y", revolution->Axis().Direction().Y(),
                "unitless");
    add_numeric(observation, "revolution.axis_direction_z", revolution->Axis().Direction().Z(),
                "unitless");
    observation.wrapper_stack.emplace_back(revolution->BasisCurve()->DynamicType()->Name());
  }
}

GeometryObservation observe_edge_geometry(const TopoDS_Edge& edge, const int ordinal,
                                          const std::vector<TopoDS_Vertex>& vertices) {
  GeometryObservation observation;
  observation.id = "geometry.edge." + std::to_string(ordinal);
  observation.subject_kind = "edge";
  observation.partner_ordinal = ordinal;
  TopLoc_Location location;
  double first = 0.0;
  double last = 0.0;
  occ::handle<Geom_Curve> stored = BRep_Tool::Curve(edge, location, first, last);
  if (stored.IsNull()) {
    observation.family = "pcurve_only";
    observation.concrete_type = "null";
    return observation;
  }
  observation.family = curve_family(BRepAdaptor_Curve(edge).GetType());
  observation.concrete_type = stored->DynamicType()->Name();
  observation.domains.push_back(ParameterDomainObservation{
      "curve", first, last, stored->IsClosed(), stored->IsPeriodic(),
      stored->IsPeriodic() ? std::optional<double>(stored->Period()) : std::nullopt,
      "curve_parameter"});
  const occ::handle<Geom_Curve> basis = unwrap_curve(stored, observation);
  observe_curve_parameters(basis, observation);
  add_boolean(observation, "edge.degenerated", BRep_Tool::Degenerated(edge));
  add_boolean(observation, "edge.same_parameter", BRep_Tool::SameParameter(edge));
  add_boolean(observation, "edge.same_range", BRep_Tool::SameRange(edge));
  add_numeric(observation, "edge.tolerance", BRep_Tool::Tolerance(edge), "millimetre");
  TopoDS_Vertex first_vertex;
  TopoDS_Vertex last_vertex;
  TopExp::Vertices(edge, first_vertex, last_vertex, true);
  const auto append_vertex = [&observation, &edge, &vertices](const std::string_view role,
                                                              const TopoDS_Vertex& vertex) {
    if (vertex.IsNull()) {
      return;
    }
    VertexParameterObservation endpoint;
    endpoint.role = role;
    endpoint.vertex_partner_ordinal = partner_ordinal(vertex, vertices);
    endpoint.tolerance_mm = BRep_Tool::Tolerance(vertex);
    try {
      endpoint.parameter = BRep_Tool::Parameter(vertex, edge);
    } catch (const Standard_Failure&) {
      endpoint.parameter = std::nullopt;
    }
    observation.vertex_parameters.push_back(std::move(endpoint));
  };
  append_vertex("start", first_vertex);
  append_vertex("end", last_vertex);
  return observation;
}

GeometryObservation observe_face_geometry(const TopoDS_Face& face, const int ordinal) {
  GeometryObservation observation;
  observation.id = "geometry.face." + std::to_string(ordinal);
  observation.subject_kind = "face";
  observation.partner_ordinal = ordinal;
  TopLoc_Location location;
  occ::handle<Geom_Surface> stored = BRep_Tool::Surface(face, location);
  if (stored.IsNull()) {
    observation.family = "kernel_specific";
    observation.concrete_type = "null";
    return observation;
  }
  observation.family = surface_family(BRepAdaptor_Surface(face, true).GetType());
  observation.concrete_type = stored->DynamicType()->Name();
  double u_min = 0.0;
  double u_max = 0.0;
  double v_min = 0.0;
  double v_max = 0.0;
  BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
  observation.domains.push_back(ParameterDomainObservation{
      "u", u_min, u_max, stored->IsUClosed(), stored->IsUPeriodic(),
      stored->IsUPeriodic() ? std::optional<double>(stored->UPeriod()) : std::nullopt,
      "surface_parameter"});
  observation.domains.push_back(ParameterDomainObservation{
      "v", v_min, v_max, stored->IsVClosed(), stored->IsVPeriodic(),
      stored->IsVPeriodic() ? std::optional<double>(stored->VPeriod()) : std::nullopt,
      "surface_parameter"});
  const occ::handle<Geom_Surface> basis = unwrap_surface(stored, observation);
  observe_surface_parameters(basis, observation);
  add_numeric(observation, "face.tolerance", BRep_Tool::Tolerance(face), "millimetre");
  add_boolean(observation, "face.orientation_reversed", face.Orientation() == TopAbs_REVERSED);
  return observation;
}

std::optional<double> curve_surface_residual(const TopoDS_Edge& edge, const TopoDS_Face& face,
                                             const occ::handle<Geom2d_Curve>& pcurve,
                                             const double pcurve_first, const double pcurve_last,
                                             const double edge_first, const double edge_last) {
  if (pcurve.IsNull()) {
    return std::nullopt;
  }
  TopLoc_Location curve_location;
  double stored_first = 0.0;
  double stored_last = 0.0;
  const occ::handle<Geom_Curve> curve =
      BRep_Tool::Curve(edge, curve_location, stored_first, stored_last);
  TopLoc_Location surface_location;
  const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(face, surface_location);
  if (curve.IsNull() || surface.IsNull()) {
    return std::nullopt;
  }
  double maximum = 0.0;
  constexpr std::array<double, 5> fractions{0.0, 0.25, 0.5, 0.75, 1.0};
  for (const double fraction : fractions) {
    const double edge_parameter = edge_first + fraction * (edge_last - edge_first);
    const double pcurve_parameter = pcurve_first + fraction * (pcurve_last - pcurve_first);
    gp_Pnt curve_point = curve->Value(edge_parameter);
    curve_point.Transform(curve_location.Transformation());
    const gp_Pnt2d uv = pcurve->Value(pcurve_parameter);
    gp_Pnt surface_point = surface->Value(uv.X(), uv.Y());
    surface_point.Transform(surface_location.Transformation());
    maximum = std::max(maximum, curve_point.Distance(surface_point));
  }
  return maximum;
}

void sort_geometry_properties(GeometryObservation& observation) {
  std::sort(observation.domains.begin(), observation.domains.end(),
            [](const auto& left, const auto& right) { return left.axis < right.axis; });
  const auto by_code = [](const auto& left, const auto& right) { return left.code < right.code; };
  std::sort(observation.numeric_properties.begin(), observation.numeric_properties.end(), by_code);
  std::sort(observation.integer_properties.begin(), observation.integer_properties.end(), by_code);
  std::sort(observation.boolean_properties.begin(), observation.boolean_properties.end(), by_code);
  std::sort(observation.numeric_sequences.begin(), observation.numeric_sequences.end(), by_code);
  std::sort(observation.integer_sequences.begin(), observation.integer_sequences.end(), by_code);
}

EvaluationObservation evaluate_face_metric(const TopoDS_Face& face, const int ordinal) {
  EvaluationObservation observation;
  observation.id = "evaluation.face." + std::to_string(ordinal);
  observation.subject_kind = "face";
  observation.partner_ordinal = ordinal;
  try {
    double u_min = 0.0;
    double u_max = 0.0;
    double v_min = 0.0;
    double v_max = 0.0;
    BRepTools::UVBounds(face, u_min, u_max, v_min, v_max);
    const double u = 0.5 * (u_min + u_max);
    const double v = 0.5 * (v_min + v_max);
    const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(face);
    if (surface.IsNull() || !std::isfinite(u) || !std::isfinite(v)) {
      observation.outcome = "failure";
      observation.condition_codes.emplace_back("evaluation.domain_nonfinite");
      return observation;
    }
    gp_Pnt point;
    gp_Vec du;
    gp_Vec dv;
    surface->D1(u, v, point, du, dv);
    const double e = du.SquareMagnitude();
    const double f = du.Dot(dv);
    const double g = dv.SquareMagnitude();
    const double discriminant = std::sqrt(std::max(0.0, (e - g) * (e - g) + 4.0 * f * f));
    const double lambda_max = 0.5 * (e + g + discriminant);
    const double lambda_min = 0.5 * (e + g - discriminant);
    const gp_Vec normal = du.Crossed(dv);
    observation.u_derivative_norm = du.Magnitude();
    observation.v_derivative_norm = dv.Magnitude();
    observation.normal_norm = normal.Magnitude();
    if (lambda_min > std::numeric_limits<double>::epsilon() &&
        std::isfinite(lambda_max / lambda_min)) {
      observation.uv_metric_condition = std::sqrt(lambda_max / lambda_min);
    } else {
      observation.condition_codes.emplace_back("surface.metric_singular");
    }
    if (normal.SquareMagnitude() <= std::numeric_limits<double>::epsilon()) {
      observation.condition_codes.emplace_back("surface.normal_undefined");
    }
    observation.outcome = "success";
  } catch (const Standard_Failure&) {
    observation.outcome = "failure";
    observation.condition_codes.emplace_back("evaluation.occt_failure");
  }
  std::sort(observation.condition_codes.begin(), observation.condition_codes.end());
  return observation;
}

std::string json_escape(const std::string_view value) {
  std::ostringstream stream;
  for (const unsigned char character : value) {
    switch (character) {
    case '"':
      stream << "\\\"";
      break;
    case '\\':
      stream << "\\\\";
      break;
    case '\b':
      stream << "\\b";
      break;
    case '\f':
      stream << "\\f";
      break;
    case '\n':
      stream << "\\n";
      break;
    case '\r':
      stream << "\\r";
      break;
    case '\t':
      stream << "\\t";
      break;
    default:
      if (character < 0x20U || character > 0x7eU) {
        stream << "\\u" << std::hex << std::setw(4) << std::setfill('0')
               << static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
      } else {
        stream << static_cast<char>(character);
      }
    }
  }
  return stream.str();
}

std::string json_number(const double value) {
  if (!std::isfinite(value)) {
    throw std::runtime_error("fixture observer cannot serialize a non-finite number");
  }
  if (value == 0.0) {
    return "0";
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return stream.str();
}

void indent(std::ostream& stream, const int amount) {
  stream << std::string(static_cast<std::size_t>(amount), ' ');
}

void write_string_array(std::ostream& stream, const std::vector<std::string>& values) {
  stream << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << '"' << json_escape(values[index]) << '"';
  }
  stream << ']';
}

void write_number_array(std::ostream& stream, const std::vector<double>& values) {
  stream << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << json_number(values[index]);
  }
  stream << ']';
}

void write_integer_array(std::ostream& stream, const std::vector<int>& values) {
  stream << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << values[index];
  }
  stream << ']';
}

} // namespace

FixtureEvidence observe_fixture_evidence(const TopoDS_Shape& shape) {
  if (shape.IsNull()) {
    throw std::runtime_error("fixture observer cannot inspect a null shape");
  }
  FixtureEvidence result;
  const std::vector<TopoDS_Edge> edges = unique_partner_shapes<TopoDS_Edge>(shape, TopAbs_EDGE);
  const std::vector<TopoDS_Face> faces = unique_partner_shapes<TopoDS_Face>(shape, TopAbs_FACE);
  const std::vector<TopoDS_Vertex> vertices =
      unique_partner_shapes<TopoDS_Vertex>(shape, TopAbs_VERTEX);

  for (std::size_t index = 0; index < edges.size(); ++index) {
    GeometryObservation observation =
        observe_edge_geometry(edges[index], static_cast<int>(index) + 1, vertices);
    sort_geometry_properties(observation);
    result.geometry.push_back(std::move(observation));
  }
  for (std::size_t index = 0; index < faces.size(); ++index) {
    GeometryObservation observation =
        observe_face_geometry(faces[index], static_cast<int>(index) + 1);
    sort_geometry_properties(observation);
    result.geometry.push_back(std::move(observation));
    result.evaluations.push_back(evaluate_face_metric(faces[index], static_cast<int>(index) + 1));
  }

  std::map<std::pair<int, int>, int> representation_counts;
  int wire_ordinal = 0;
  for (std::size_t face_index = 0; face_index < faces.size(); ++face_index) {
    const TopoDS_Face& face = faces[face_index];
    for (TopExp_Explorer wire_explorer(face, TopAbs_WIRE); wire_explorer.More();
         wire_explorer.Next()) {
      ++wire_ordinal;
      const TopoDS_Wire wire = TopoDS::Wire(wire_explorer.Current());
      int order_in_wire = 0;
      for (BRepTools_WireExplorer explorer(wire, face); explorer.More(); explorer.Next()) {
        ++order_in_wire;
        const TopoDS_Edge edge = explorer.Current();
        const int edge_ordinal = partner_ordinal(edge, edges);
        const int face_ordinal = static_cast<int>(face_index) + 1;
        const int representation_index = ++representation_counts[{edge_ordinal, face_ordinal}];
        double pcurve_first = 0.0;
        double pcurve_last = 0.0;
        const occ::handle<Geom2d_Curve> pcurve =
            BRep_Tool::CurveOnSurface(edge, face, pcurve_first, pcurve_last);
        TopLoc_Location curve_location;
        double edge_first = 0.0;
        double edge_last = 0.0;
        const occ::handle<Geom_Curve> curve =
            BRep_Tool::Curve(edge, curve_location, edge_first, edge_last);
        PcurveObservation observation;
        observation.id = "pcurve." + std::to_string(result.pcurves.size() + 1);
        observation.edge_partner_ordinal = edge_ordinal;
        observation.face_partner_ordinal = face_ordinal;
        observation.wire_ordinal = wire_ordinal;
        observation.order_in_wire = order_in_wire;
        observation.representation_index = representation_index;
        observation.orientation = orientation_name(edge.Orientation());
        observation.concrete_type = pcurve.IsNull() ? "null" : pcurve->DynamicType()->Name();
        observation.edge_range_lower = edge_first;
        observation.edge_range_upper = edge_last;
        observation.pcurve_range_lower = pcurve_first;
        observation.pcurve_range_upper = pcurve_last;
        observation.same_parameter = BRep_Tool::SameParameter(edge);
        observation.same_range = BRep_Tool::SameRange(edge);
        observation.degenerated = BRep_Tool::Degenerated(edge);
        observation.edge_tolerance_mm = BRep_Tool::Tolerance(edge);
        if (pcurve.IsNull()) {
          observation.condition_codes.emplace_back("pcurve.missing");
        }
        if (curve.IsNull()) {
          observation.condition_codes.emplace_back("curve.3d_missing");
        }
        try {
          observation.maximum_curve_surface_residual_mm = curve_surface_residual(
              edge, face, pcurve, pcurve_first, pcurve_last, edge_first, edge_last);
        } catch (const Standard_Failure&) {
          observation.condition_codes.emplace_back("pcurve.residual_evaluation_failure");
        }
        std::sort(observation.condition_codes.begin(), observation.condition_codes.end());
        result.pcurves.push_back(std::move(observation));
      }
    }
  }

  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    std::vector<int> incident_faces;
    for (std::size_t face_index = 0; face_index < faces.size(); ++face_index) {
      bool found = false;
      for (TopExp_Explorer explorer(faces[face_index], TopAbs_EDGE); explorer.More();
           explorer.Next()) {
        if (edges[edge_index].IsPartner(explorer.Current())) {
          found = true;
          break;
        }
      }
      if (found) {
        incident_faces.push_back(static_cast<int>(face_index) + 1);
      }
    }
    for (std::size_t left = 0; left < incident_faces.size(); ++left) {
      for (std::size_t right = left + 1; right < incident_faces.size(); ++right) {
        const int first_face = incident_faces[left];
        const int second_face = incident_faces[right];
        ContinuityObservation observation;
        observation.id = "continuity." + std::to_string(result.continuity.size() + 1);
        observation.edge_partner_ordinal = static_cast<int>(edge_index) + 1;
        observation.first_face_partner_ordinal = first_face;
        observation.second_face_partner_ordinal = second_face;
        observation.occt_continuity = continuity_name(BRep_Tool::Continuity(
            edges[edge_index], faces[static_cast<std::size_t>(first_face - 1)],
            faces[static_cast<std::size_t>(second_face - 1)]));
        result.continuity.push_back(std::move(observation));
      }
    }
  }

  return result;
}

void write_fixture_evidence_json(std::ostream& stream, const FixtureEvidence& evidence,
                                 const int indentation) {
  stream << "{\n";
  indent(stream, indentation + 2);
  stream << "\"continuity\": [";
  for (std::size_t index = 0; index < evidence.continuity.size(); ++index) {
    const auto& item = evidence.continuity[index];
    if (index != 0) {
      stream << ',';
    }
    stream << "\n";
    indent(stream, indentation + 4);
    stream << "{\"edge_partner_ordinal\":" << item.edge_partner_ordinal
           << ",\"first_face_partner_ordinal\":" << item.first_face_partner_ordinal << ",\"id\":\""
           << json_escape(item.id) << "\",\"occt_continuity\":\""
           << json_escape(item.occt_continuity)
           << "\",\"second_face_partner_ordinal\":" << item.second_face_partner_ordinal << '}';
  }
  if (!evidence.continuity.empty()) {
    stream << "\n";
    indent(stream, indentation + 2);
  }
  stream << "],\n";

  indent(stream, indentation + 2);
  stream << "\"evaluations\": [";
  for (std::size_t index = 0; index < evidence.evaluations.size(); ++index) {
    const auto& item = evidence.evaluations[index];
    if (index != 0) {
      stream << ',';
    }
    stream << "\n";
    indent(stream, indentation + 4);
    stream << "{\"condition_codes\":";
    write_string_array(stream, item.condition_codes);
    stream << ",\"id\":\"" << json_escape(item.id) << "\",\"normal_norm\":";
    if (item.normal_norm.has_value()) {
      stream << json_number(*item.normal_norm);
    } else {
      stream << "null";
    }
    stream << ",\"outcome\":\"" << json_escape(item.outcome)
           << "\",\"partner_ordinal\":" << item.partner_ordinal << ",\"subject_kind\":\""
           << json_escape(item.subject_kind) << "\",\"u_derivative_norm\":";
    if (item.u_derivative_norm.has_value()) {
      stream << json_number(*item.u_derivative_norm);
    } else {
      stream << "null";
    }
    stream << ",\"uv_metric_condition\":";
    if (item.uv_metric_condition.has_value()) {
      stream << json_number(*item.uv_metric_condition);
    } else {
      stream << "null";
    }
    stream << ",\"v_derivative_norm\":";
    if (item.v_derivative_norm.has_value()) {
      stream << json_number(*item.v_derivative_norm);
    } else {
      stream << "null";
    }
    stream << '}';
  }
  if (!evidence.evaluations.empty()) {
    stream << "\n";
    indent(stream, indentation + 2);
  }
  stream << "],\n";

  indent(stream, indentation + 2);
  stream << "\"geometry\": [";
  for (std::size_t index = 0; index < evidence.geometry.size(); ++index) {
    const auto& item = evidence.geometry[index];
    if (index != 0) {
      stream << ',';
    }
    stream << "\n";
    indent(stream, indentation + 4);
    stream << "{\"boolean_properties\":[";
    for (std::size_t property_index = 0; property_index < item.boolean_properties.size();
         ++property_index) {
      if (property_index != 0) {
        stream << ',';
      }
      const auto& property = item.boolean_properties[property_index];
      stream << "{\"code\":\"" << json_escape(property.code)
             << "\",\"value\":" << (property.value ? "true" : "false") << '}';
    }
    stream << "],\"concrete_type\":\"" << json_escape(item.concrete_type) << "\",\"domains\":[";
    for (std::size_t domain_index = 0; domain_index < item.domains.size(); ++domain_index) {
      if (domain_index != 0) {
        stream << ',';
      }
      const auto& domain = item.domains[domain_index];
      stream << "{\"axis\":\"" << json_escape(domain.axis)
             << "\",\"closed\":" << (domain.closed ? "true" : "false")
             << ",\"lower\":" << json_number(domain.lower) << ",\"period\":";
      if (domain.period.has_value()) {
        stream << json_number(*domain.period);
      } else {
        stream << "null";
      }
      stream << ",\"periodic\":" << (domain.periodic ? "true" : "false") << ",\"unit\":\""
             << json_escape(domain.unit) << "\",\"upper\":" << json_number(domain.upper) << '}';
    }
    stream << "],\"family\":\"" << json_escape(item.family) << "\",\"id\":\""
           << json_escape(item.id) << "\",\"integer_properties\":[";
    for (std::size_t property_index = 0; property_index < item.integer_properties.size();
         ++property_index) {
      if (property_index != 0) {
        stream << ',';
      }
      const auto& property = item.integer_properties[property_index];
      stream << "{\"code\":\"" << json_escape(property.code) << "\",\"value\":" << property.value
             << '}';
    }
    stream << "],\"integer_sequences\":[";
    for (std::size_t property_index = 0; property_index < item.integer_sequences.size();
         ++property_index) {
      if (property_index != 0) {
        stream << ',';
      }
      const auto& property = item.integer_sequences[property_index];
      stream << "{\"code\":\"" << json_escape(property.code) << "\",\"values\":";
      write_integer_array(stream, property.values);
      stream << '}';
    }
    stream << "],\"numeric_properties\":[";
    for (std::size_t property_index = 0; property_index < item.numeric_properties.size();
         ++property_index) {
      if (property_index != 0) {
        stream << ',';
      }
      const auto& property = item.numeric_properties[property_index];
      stream << "{\"code\":\"" << json_escape(property.code) << "\",\"unit\":\""
             << json_escape(property.unit) << "\",\"value\":" << json_number(property.value) << '}';
    }
    stream << "],\"numeric_sequences\":[";
    for (std::size_t property_index = 0; property_index < item.numeric_sequences.size();
         ++property_index) {
      if (property_index != 0) {
        stream << ',';
      }
      const auto& property = item.numeric_sequences[property_index];
      stream << "{\"code\":\"" << json_escape(property.code) << "\",\"unit\":\""
             << json_escape(property.unit) << "\",\"values\":";
      write_number_array(stream, property.values);
      stream << '}';
    }
    stream << "],\"partner_ordinal\":" << item.partner_ordinal << ",\"subject_kind\":\""
           << json_escape(item.subject_kind) << "\",\"vertex_parameters\":[";
    for (std::size_t vertex_index = 0; vertex_index < item.vertex_parameters.size();
         ++vertex_index) {
      if (vertex_index != 0) {
        stream << ',';
      }
      const auto& vertex = item.vertex_parameters[vertex_index];
      stream << "{\"parameter\":";
      if (vertex.parameter.has_value()) {
        stream << json_number(*vertex.parameter);
      } else {
        stream << "null";
      }
      stream << ",\"role\":\"" << json_escape(vertex.role)
             << "\",\"tolerance_mm\":" << json_number(vertex.tolerance_mm)
             << ",\"vertex_partner_ordinal\":" << vertex.vertex_partner_ordinal << '}';
    }
    stream << "],\"wrapper_stack\":";
    write_string_array(stream, item.wrapper_stack);
    stream << '}';
  }
  if (!evidence.geometry.empty()) {
    stream << "\n";
    indent(stream, indentation + 2);
  }
  stream << "],\n";

  indent(stream, indentation + 2);
  stream << "\"pcurves\": [";
  for (std::size_t index = 0; index < evidence.pcurves.size(); ++index) {
    const auto& item = evidence.pcurves[index];
    if (index != 0) {
      stream << ',';
    }
    stream << "\n";
    indent(stream, indentation + 4);
    stream << "{\"concrete_type\":\"" << json_escape(item.concrete_type)
           << "\",\"condition_codes\":";
    write_string_array(stream, item.condition_codes);
    stream << ",\"degenerated\":" << (item.degenerated ? "true" : "false")
           << ",\"edge_partner_ordinal\":" << item.edge_partner_ordinal
           << ",\"edge_range_lower\":" << json_number(item.edge_range_lower)
           << ",\"edge_range_upper\":" << json_number(item.edge_range_upper)
           << ",\"edge_tolerance_mm\":" << json_number(item.edge_tolerance_mm)
           << ",\"face_partner_ordinal\":" << item.face_partner_ordinal << ",\"id\":\""
           << json_escape(item.id) << "\",\"maximum_curve_surface_residual_mm\":";
    if (item.maximum_curve_surface_residual_mm.has_value()) {
      stream << json_number(*item.maximum_curve_surface_residual_mm);
    } else {
      stream << "null";
    }
    stream << ",\"order_in_wire\":" << item.order_in_wire << ",\"orientation\":\""
           << json_escape(item.orientation)
           << "\",\"pcurve_range_lower\":" << json_number(item.pcurve_range_lower)
           << ",\"pcurve_range_upper\":" << json_number(item.pcurve_range_upper)
           << ",\"representation_index\":" << item.representation_index
           << ",\"same_parameter\":" << (item.same_parameter ? "true" : "false")
           << ",\"same_range\":" << (item.same_range ? "true" : "false")
           << ",\"wire_ordinal\":" << item.wire_ordinal << '}';
  }
  if (!evidence.pcurves.empty()) {
    stream << "\n";
    indent(stream, indentation + 2);
  }
  stream << "]\n";
  indent(stream, indentation);
  stream << '}';
}

} // namespace cad_mesher::fixtures
