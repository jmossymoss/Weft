#include "fixture_generator.hpp"

#include "fixture_contract.hpp"
#include "fixture_observer.hpp"
#include "fixture_recipe_modules.hpp"
#include "weft/occt_failure.hpp"

#include <APIHeaderSection_MakeHeader.hxx>
#include <BOPAlgo_Builder.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <DESTEP_Parameters.hxx>
#include <ElCLib.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <IFSelect_ReturnStatus.hxx>
#if OCC_VERSION_HEX >= 0x080000
#include <NCollection_HArray1.hxx>
#else
#include <Interface_HArray1OfHAsciiString.hxx>
#endif
#include <NCollection_List.hxx>
#include <NCollection_Sequence.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeProcess.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Handle.hxx>
#include <Standard_Version.hxx>
#include <StepBasic_Product.hxx>
#include <StepData_StepModel.hxx>
#include <StepRepr_NextAssemblyUsageOccurrence.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDF_Label.hxx>
#include <TDF_Tool.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_FormatVersion.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <UnitsMethods_LengthUnit.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

namespace cad_mesher::fixtures {
namespace {

constexpr std::string_view kMetadataSchemaVersion = "1.0.0";
constexpr std::string_view kGeneratorVersion = "1.0.0";
constexpr std::string_view kFixedTimestamp = "2000-01-01T00:00:00";
constexpr double kBoundaryLoopUvQuantum = 1.0e-9;
constexpr double kToleranceQuantumMm = 1.0e-14;
constexpr double kLocationQuantum = 1.0e-12;
constexpr double kCoincidenceQuantumMm = 1.0e-9;

using Matrix4 = std::array<double, 16>;

struct OrientationCounts final {
  int forward{};
  int reversed{};
  int internal{};
  int external{};

  [[nodiscard]] int total() const noexcept { return forward + reversed + internal + external; }

  bool operator==(const OrientationCounts&) const = default;
};

struct TopologyCount final {
  std::string_view family;
  int unique{};
  int occurrences{};
  OrientationCounts orientations;

  bool operator==(const TopologyCount&) const = default;
};

struct ToleranceRange final {
  std::optional<double> minimum_mm;
  std::optional<double> maximum_mm;
};

bool tolerance_observations_equal(const std::optional<double>& left,
                                  const std::optional<double>& right) {
  if (!left.has_value() || !right.has_value()) {
    return left.has_value() == right.has_value();
  }
  const double roundoff =
      8.0 * std::numeric_limits<double>::epsilon() *
      std::max({std::abs(*left), std::abs(*right), kToleranceQuantumMm});
  return std::abs(*left - *right) <= kToleranceQuantumMm + roundoff;
}

struct EdgeOccurrence final {
  std::string id;
  std::string family;
  std::string concrete_type;
  std::string orientation;
  int partner_ordinal{};
  bool degenerated{};
  bool missing_3d_curve{};
  bool periodic_seam{};
  int face_use_count{};
  std::string shell_role;
  Matrix4 geometry_location_matrix_row_major{};
  Matrix4 occurrence_location_matrix_row_major{};

  bool operator==(const EdgeOccurrence&) const = default;
};

struct FaceOccurrence final {
  std::string id;
  std::string family;
  std::string concrete_type;
  std::string orientation;
  int partner_ordinal{};
  int solid_use_count{};
  std::string shell_role;
  Matrix4 geometry_location_matrix_row_major{};
  Matrix4 occurrence_location_matrix_row_major{};

  bool operator==(const FaceOccurrence&) const = default;
};

struct LogicalRegionProxy final {
  std::string id;
  std::string face_occurrence_id;
  std::string family;
  std::string concrete_type;

  bool operator==(const LogicalRegionProxy&) const = default;
};

struct BoundaryLoop final {
  std::string id;
  std::string face_occurrence_id;
  std::string role;
  std::string orientation;
  bool closed{};
  int edge_occurrence_count{};
  double u_min{};
  double u_max{};
  double v_min{};
  double v_max{};

  bool operator==(const BoundaryLoop&) const = default;
};

struct ModelTopology final {
  int closed_shells{};
  int open_shells{};
  int outer_shells{};
  int inner_shells{};
  int standalone_shells{};
  int shared_faces_between_solids{};
  int maximum_faces_per_edge{};
  int nonmanifold_vertices{};
  int free_wires{};
  int free_edges{};
  int isolated_vertices{};
  int coincident_solid_signature_pairs{};

  bool operator==(const ModelTopology&) const = default;
};

struct ShapeSummary final {
  bool valid{};
  std::string root_shape_type;
  std::vector<TopologyCount> topology;
  std::map<std::string, int> curve_families;
  std::map<std::string, int> curve_concrete_types;
  std::map<std::string, int> surface_families;
  std::map<std::string, int> surface_concrete_types;
  std::vector<EdgeOccurrence> edge_occurrences;
  std::vector<FaceOccurrence> face_occurrences;
  std::vector<BoundaryLoop> boundary_loops;
  std::vector<LogicalRegionProxy> logical_region_proxies;
  ModelTopology model_topology;
  int degenerated_edges{};
  int edges_without_3d_curve{};
  int periodic_seam_edges{};
  int free_boundary_edges{};
  int nonmanifold_edges{};
  ToleranceRange vertex_tolerance;
  ToleranceRange edge_tolerance;
  ToleranceRange face_tolerance;
};

struct XdeInstanceOccurrence final {
  std::string label;
  std::string component_label;
  std::optional<std::string> parent_label;
  std::string target_definition;
  std::string target_kind;
  std::string transform_kind;
  Matrix4 local_matrix_row_major{};
  Matrix4 matrix_row_major{};
};

struct XdeSummary final {
  int product_definitions{};
  int free_products{};
  int simple_shape_definitions{};
  int assembly_definitions{};
  int unique_component_labels{};
  int component_instances{};
  std::vector<XdeInstanceOccurrence> instances;
};

struct StepReadback final {
  TopoDS_Shape shape;
  XdeSummary xde;
};

struct ExternalStepInspection final {
  int transfer_roots{};
  std::vector<std::string> schema_identifiers;
  StepReadback readback;
};

struct OutputPaths final {
  std::filesystem::path step;
  std::filesystem::path metadata;
  std::filesystem::path native_brep;
};

class GeneratorError final : public std::runtime_error {
public:
  GeneratorError(std::string code, std::string message)
      : std::runtime_error(std::move(message)), code_(std::move(code)) {}

  [[nodiscard]] const std::string& code() const noexcept { return code_; }

private:
  std::string code_;
};

bool topology_count_structurally_equal(const TopologyCount& left, const TopologyCount& right) {
  if (left.family != right.family || left.unique != right.unique ||
      left.occurrences != right.occurrences) {
    return false;
  }
  // STEP readers may canonicalize the carrier shell polarity while preserving every
  // oriented face, wire, edge, and vertex occurrence that defines the solid boundary.
  return left.family == "shell" || left.orientations == right.orientations;
}

bool topology_structurally_equal(const std::vector<TopologyCount>& left,
                                 const std::vector<TopologyCount>& right) {
  return left.size() == right.size() &&
         std::equal(left.begin(), left.end(), right.begin(), topology_count_structurally_equal);
}

bool edge_occurrence_graph_equal(const EdgeOccurrence& left, const EdgeOccurrence& right) {
  return left.id == right.id && left.orientation == right.orientation &&
         left.partner_ordinal == right.partner_ordinal && left.degenerated == right.degenerated &&
         left.missing_3d_curve == right.missing_3d_curve &&
         left.periodic_seam == right.periodic_seam && left.face_use_count == right.face_use_count &&
         left.shell_role == right.shell_role &&
         left.geometry_location_matrix_row_major == right.geometry_location_matrix_row_major &&
         left.occurrence_location_matrix_row_major == right.occurrence_location_matrix_row_major;
}

bool face_occurrence_graph_equal(const FaceOccurrence& left, const FaceOccurrence& right) {
  return left.id == right.id && left.orientation == right.orientation &&
         left.partner_ordinal == right.partner_ordinal &&
         left.solid_use_count == right.solid_use_count && left.shell_role == right.shell_role &&
         left.geometry_location_matrix_row_major == right.geometry_location_matrix_row_major &&
         left.occurrence_location_matrix_row_major == right.occurrence_location_matrix_row_major;
}

bool logical_region_graph_equal(const LogicalRegionProxy& left, const LogicalRegionProxy& right) {
  return left.id == right.id && left.face_occurrence_id == right.face_occurrence_id;
}

template <typename Value, typename Predicate>
bool vectors_equal_by(const std::vector<Value>& left, const std::vector<Value>& right,
                      Predicate predicate) {
  return left.size() == right.size() &&
         std::equal(left.begin(), left.end(), right.begin(), predicate);
}

bool structurally_equal(const ShapeSummary& left, const ShapeSummary& right,
                        const RoundTripContract& contract) {
  const bool occurrence_graph_equal =
      vectors_equal_by(left.edge_occurrences, right.edge_occurrences,
                       edge_occurrence_graph_equal) &&
      vectors_equal_by(left.face_occurrences, right.face_occurrences,
                       face_occurrence_graph_equal) &&
      vectors_equal_by(left.logical_region_proxies, right.logical_region_proxies,
                       logical_region_graph_equal) &&
      left.degenerated_edges == right.degenerated_edges &&
      left.edges_without_3d_curve == right.edges_without_3d_curve &&
      left.periodic_seam_edges == right.periodic_seam_edges &&
      left.free_boundary_edges == right.free_boundary_edges &&
      left.nonmanifold_edges == right.nonmanifold_edges;
  const bool tolerances_equal =
      tolerance_observations_equal(left.vertex_tolerance.minimum_mm,
                                   right.vertex_tolerance.minimum_mm) &&
      tolerance_observations_equal(left.vertex_tolerance.maximum_mm,
                                   right.vertex_tolerance.maximum_mm) &&
      tolerance_observations_equal(left.edge_tolerance.minimum_mm,
                                   right.edge_tolerance.minimum_mm) &&
      tolerance_observations_equal(left.edge_tolerance.maximum_mm,
                                   right.edge_tolerance.maximum_mm) &&
      tolerance_observations_equal(left.face_tolerance.minimum_mm,
                                   right.face_tolerance.minimum_mm) &&
      tolerance_observations_equal(left.face_tolerance.maximum_mm,
                                   right.face_tolerance.maximum_mm);
  return left.valid == right.valid &&
         (!contract.topology_counts ||
          (left.root_shape_type == right.root_shape_type &&
           topology_structurally_equal(left.topology, right.topology))) &&
         (!contract.geometry_families || (left.curve_families == right.curve_families &&
                                          left.surface_families == right.surface_families)) &&
         (!contract.concrete_types ||
          (left.curve_concrete_types == right.curve_concrete_types &&
           left.surface_concrete_types == right.surface_concrete_types)) &&
         (!contract.occurrence_graph || occurrence_graph_equal) &&
         (!contract.boundary_loops ||
          ((!left.valid && !right.valid) || left.boundary_loops == right.boundary_loops)) &&
         (!contract.model_topology || left.model_topology == right.model_topology) &&
         (!contract.tolerances || tolerances_equal);
}

std::string optional_double_text(const std::optional<double>& value) {
  if (!value.has_value()) {
    return "null";
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<double>::max_digits10) << *value;
  return stream.str();
}

std::string first_structural_difference(const ShapeSummary& actual, const ShapeSummary& expected,
                                        const RoundTripContract& contract) {
  if (actual.valid != expected.valid) {
    return "valid";
  }
  if (contract.topology_counts && actual.root_shape_type != expected.root_shape_type) {
    return "root_shape_type (source=" + expected.root_shape_type +
           ", roundtrip=" + actual.root_shape_type + ")";
  }
  if (contract.topology_counts &&
      !topology_structurally_equal(actual.topology, expected.topology)) {
    for (std::size_t index = 0; index < actual.topology.size() && index < expected.topology.size();
         ++index) {
      if (!topology_count_structurally_equal(actual.topology[index], expected.topology[index])) {
        const TopologyCount& source = expected.topology[index];
        const TopologyCount& roundtrip = actual.topology[index];
        const auto describe = [](const TopologyCount& count) {
          return "unique=" + std::to_string(count.unique) +
                 ", occurrences=" + std::to_string(count.occurrences) +
                 ", orientations=" + std::to_string(count.orientations.forward) + "/" +
                 std::to_string(count.orientations.reversed) + "/" +
                 std::to_string(count.orientations.internal) + "/" +
                 std::to_string(count.orientations.external);
        };
        return "topology." + std::string(source.family) + " (source=" + describe(source) +
               ", roundtrip=" + describe(roundtrip) + ")";
      }
    }
    return "topology.size";
  }
  if (contract.geometry_families && actual.curve_families != expected.curve_families) {
    return "curve_families";
  }
  if (contract.concrete_types && actual.curve_concrete_types != expected.curve_concrete_types) {
    return "curve_concrete_types";
  }
  if (contract.geometry_families && actual.surface_families != expected.surface_families) {
    return "surface_families";
  }
  if (contract.concrete_types && actual.surface_concrete_types != expected.surface_concrete_types) {
    return "surface_concrete_types";
  }
  if (contract.occurrence_graph &&
      !vectors_equal_by(actual.edge_occurrences, expected.edge_occurrences,
                        edge_occurrence_graph_equal)) {
    if (actual.edge_occurrences.size() != expected.edge_occurrences.size()) {
      return "edge_occurrences.size (source=" + std::to_string(expected.edge_occurrences.size()) +
             ", roundtrip=" + std::to_string(actual.edge_occurrences.size()) + ")";
    }
    const auto describe = [](const EdgeOccurrence& occurrence) {
      return occurrence.id + ":" + occurrence.family + ":" + occurrence.concrete_type + ":" +
             occurrence.orientation + ":partner=" + std::to_string(occurrence.partner_ordinal) +
             ":face_uses=" + std::to_string(occurrence.face_use_count) +
             ":shell_role=" + occurrence.shell_role +
             ":flags=" + (occurrence.degenerated ? "1" : "0") +
             (occurrence.missing_3d_curve ? "1" : "0") + (occurrence.periodic_seam ? "1" : "0");
    };
    for (std::size_t index = 0; index < actual.edge_occurrences.size(); ++index) {
      if (!edge_occurrence_graph_equal(actual.edge_occurrences[index],
                                       expected.edge_occurrences[index])) {
        return "edge_occurrences[" + std::to_string(index) +
               "] (source=" + describe(expected.edge_occurrences[index]) +
               ", roundtrip=" + describe(actual.edge_occurrences[index]) + ")";
      }
    }
    return "edge_occurrences";
  }
  if (contract.occurrence_graph &&
      !vectors_equal_by(actual.face_occurrences, expected.face_occurrences,
                        face_occurrence_graph_equal)) {
    if (actual.face_occurrences.size() != expected.face_occurrences.size()) {
      return "face_occurrences.size (source=" + std::to_string(expected.face_occurrences.size()) +
             ", roundtrip=" + std::to_string(actual.face_occurrences.size()) + ")";
    }
    const auto describe = [](const FaceOccurrence& occurrence) {
      return occurrence.id + ":" + occurrence.family + ":" + occurrence.concrete_type + ":" +
             occurrence.orientation + ":partner=" + std::to_string(occurrence.partner_ordinal) +
             ":solid_uses=" + std::to_string(occurrence.solid_use_count) +
             ":shell_role=" + occurrence.shell_role;
    };
    for (std::size_t index = 0; index < actual.face_occurrences.size(); ++index) {
      if (!face_occurrence_graph_equal(actual.face_occurrences[index],
                                       expected.face_occurrences[index])) {
        return "face_occurrences[" + std::to_string(index) +
               "] (source=" + describe(expected.face_occurrences[index]) +
               ", roundtrip=" + describe(actual.face_occurrences[index]) + ")";
      }
    }
    return "face_occurrences";
  }
  if (contract.boundary_loops && (actual.valid || expected.valid) &&
      actual.boundary_loops != expected.boundary_loops) {
    if (actual.boundary_loops.size() != expected.boundary_loops.size()) {
      return "boundary_loops.size (source=" + std::to_string(expected.boundary_loops.size()) +
             ", roundtrip=" + std::to_string(actual.boundary_loops.size()) + ")";
    }
    const auto describe = [](const BoundaryLoop& loop) {
      return loop.id + ":face=" + loop.face_occurrence_id + ":" + loop.role + ":" +
             loop.orientation + ":closed=" + (loop.closed ? "1" : "0") +
             ":edges=" + std::to_string(loop.edge_occurrence_count) +
             ":uv=" + optional_double_text(loop.u_min) + "/" + optional_double_text(loop.u_max) +
             "/" + optional_double_text(loop.v_min) + "/" + optional_double_text(loop.v_max);
    };
    for (std::size_t index = 0; index < actual.boundary_loops.size(); ++index) {
      if (!(actual.boundary_loops[index] == expected.boundary_loops[index])) {
        return "boundary_loops[" + std::to_string(index) +
               "] (source=" + describe(expected.boundary_loops[index]) +
               ", roundtrip=" + describe(actual.boundary_loops[index]) + ")";
      }
    }
    return "boundary_loops";
  }
  if (contract.occurrence_graph &&
      !vectors_equal_by(actual.logical_region_proxies, expected.logical_region_proxies,
                        logical_region_graph_equal)) {
    return "logical_region_proxies";
  }
  if (contract.model_topology && actual.model_topology != expected.model_topology) {
    return "model_topology";
  }
  if (contract.occurrence_graph && actual.degenerated_edges != expected.degenerated_edges) {
    return "edge_conditions.degenerated";
  }
  if (contract.occurrence_graph &&
      actual.edges_without_3d_curve != expected.edges_without_3d_curve) {
    return "edge_conditions.missing_3d_curve";
  }
  if (contract.occurrence_graph && actual.periodic_seam_edges != expected.periodic_seam_edges) {
    return "edge_conditions.periodic_seam";
  }
  if (contract.occurrence_graph && actual.free_boundary_edges != expected.free_boundary_edges) {
    return "edge_conditions.free_boundary";
  }
  if (contract.occurrence_graph && actual.nonmanifold_edges != expected.nonmanifold_edges) {
    return "edge_conditions.nonmanifold";
  }

  const auto tolerance_difference =
      [](const std::string_view family, const ToleranceRange& actual_range,
         const ToleranceRange& expected_range) -> std::optional<std::string> {
    if (!tolerance_observations_equal(actual_range.minimum_mm,
                                      expected_range.minimum_mm)) {
      return "tolerances." + std::string(family) +
             ".minimum_mm (source=" + optional_double_text(expected_range.minimum_mm) +
             ", roundtrip=" + optional_double_text(actual_range.minimum_mm) + ")";
    }
    if (!tolerance_observations_equal(actual_range.maximum_mm,
                                      expected_range.maximum_mm)) {
      return "tolerances." + std::string(family) +
             ".maximum_mm (source=" + optional_double_text(expected_range.maximum_mm) +
             ", roundtrip=" + optional_double_text(actual_range.maximum_mm) + ")";
    }
    return std::nullopt;
  };
  if (contract.tolerances) {
    for (const auto& [family, actual_range, expected_range] :
         {std::tuple<std::string_view, const ToleranceRange&, const ToleranceRange&>{
              "vertex", actual.vertex_tolerance, expected.vertex_tolerance},
          {"edge", actual.edge_tolerance, expected.edge_tolerance},
          {"face", actual.face_tolerance, expected.face_tolerance}}) {
      if (const auto difference = tolerance_difference(family, actual_range, expected_range)) {
        return *difference;
      }
    }
  }
  return "unknown";
}

void observe_tolerance(ToleranceRange& range, const double tolerance_mm) {
  if (!std::isfinite(tolerance_mm) || tolerance_mm < 0.0) {
    throw GeneratorError("inventory", "encountered a negative or non-finite B-rep tolerance");
  }
  const double scaled = tolerance_mm / kToleranceQuantumMm;
  if (!std::isfinite(scaled)) {
    throw GeneratorError("inventory", "B-rep tolerance exceeds canonical observation range");
  }
  const double canonical_tolerance_mm = std::round(scaled) * kToleranceQuantumMm;
  if (!range.minimum_mm.has_value()) {
    range.minimum_mm = canonical_tolerance_mm;
    range.maximum_mm = canonical_tolerance_mm;
    return;
  }
  range.minimum_mm = std::min(*range.minimum_mm, canonical_tolerance_mm);
  range.maximum_mm = std::max(*range.maximum_mm, canonical_tolerance_mm);
}

template <typename Builder>
TopoDS_Shape checked_shape(Builder& builder, const std::string_view fixture_id) {
  const TopoDS_Shape shape = builder.Shape();
  if (!builder.IsDone() || shape.IsNull()) {
    throw GeneratorError("construction", "OCCT construction failed for " + std::string(fixture_id));
  }
  return shape;
}

double parameter(std::span<const ConstructionParameter> parameters, const std::string_view name) {
  const auto found = std::find_if(parameters.begin(), parameters.end(),
                                  [name](const auto& item) { return item.name == name; });
  if (found == parameters.end()) {
    throw GeneratorError("recipe", "missing construction parameter " + std::string(name));
  }
  return found->value;
}

void require_full_sweep(std::span<const ConstructionParameter> parameters,
                        const std::string_view fixture_id) {
  const double sweep_angle = parameter(parameters, "sweep_angle");
  if (std::abs(sweep_angle - (2.0 * std::numbers::pi)) >
      8.0 * std::numeric_limits<double>::epsilon()) {
    throw GeneratorError("recipe", std::string(fixture_id) +
                                       " requires sweep_angle to be exactly 2*pi radians");
  }
}

TopoDS_Shape make_box(const std::span<const ConstructionParameter> parameters) {
  BRepPrimAPI_MakeBox builder(parameter(parameters, "length_x"), parameter(parameters, "length_y"),
                              parameter(parameters, "length_z"));
  return checked_shape(builder, "solid.box");
}

TopoDS_Shape make_full_cylinder(const std::span<const ConstructionParameter> parameters) {
  require_full_sweep(parameters, "solid.cylinder.full");
  BRepPrimAPI_MakeCylinder builder(parameter(parameters, "radius"),
                                   parameter(parameters, "height"));
  return checked_shape(builder, "solid.cylinder.full");
}

TopoDS_Shape make_partial_cylinder(const std::span<const ConstructionParameter> parameters) {
  BRepPrimAPI_MakeCylinder builder(parameter(parameters, "radius"), parameter(parameters, "height"),
                                   parameter(parameters, "sweep_angle"));
  return checked_shape(builder, "solid.cylinder.partial");
}

TopoDS_Shape make_frustum(const std::span<const ConstructionParameter> parameters) {
  require_full_sweep(parameters, "solid.cone.frustum");
  BRepPrimAPI_MakeCone builder(parameter(parameters, "radius_at_z0"),
                               parameter(parameters, "radius_at_zmax"),
                               parameter(parameters, "height"));
  return checked_shape(builder, "solid.cone.frustum");
}

TopoDS_Shape make_sphere_segment(const std::span<const ConstructionParameter> parameters) {
  BRepPrimAPI_MakeSphere builder(
      parameter(parameters, "radius"), parameter(parameters, "latitude_min"),
      parameter(parameters, "latitude_max"), parameter(parameters, "sweep_angle"));
  return checked_shape(builder, "solid.sphere.segment");
}

TopoDS_Shape make_torus_segment(const std::span<const ConstructionParameter> parameters) {
  BRepPrimAPI_MakeTorus builder(parameter(parameters, "major_radius"),
                                parameter(parameters, "minor_radius"),
                                parameter(parameters, "v_min"), parameter(parameters, "v_max"),
                                parameter(parameters, "sweep_angle"));
  return checked_shape(builder, "solid.torus.segment");
}

TopoDS_Shape extrude_face(const TopoDS_Face& face, const double height, const bool canonize,
                          const std::string_view fixture_id) {
  BRepPrimAPI_MakePrism builder(face, gp_Vec(0.0, 0.0, height), true, canonize);
  return checked_shape(builder, fixture_id);
}

TopoDS_Shape cut_plate(const std::span<const ConstructionParameter> parameters,
                       const std::span<const gp_Circ> holes, const std::string_view fixture_id) {
  const double height = parameter(parameters, "height");
  BRepPrimAPI_MakeBox base(parameter(parameters, "length_x"), parameter(parameters, "length_y"),
                           height);
  const TopoDS_Shape base_shape = checked_shape(base, fixture_id);
  const double overcut = parameter(parameters, "overcut");

  NCollection_List<TopoDS_Shape> arguments;
  arguments.Append(base_shape);
  NCollection_List<TopoDS_Shape> tools;
  for (const gp_Circ& hole : holes) {
    BRepPrimAPI_MakeCylinder cutter(
        gp_Ax2(gp_Pnt(hole.Location().X(), hole.Location().Y(), -overcut), gp_Dir(0.0, 0.0, 1.0)),
        hole.Radius(), height + (2.0 * overcut));
    tools.Append(checked_shape(cutter, fixture_id));
  }

  BRepAlgoAPI_Cut cut;
  cut.SetArguments(arguments);
  cut.SetTools(tools);
  cut.SetRunParallel(false);
  cut.SetFuzzyValue(0.0);
  cut.SetNonDestructive(true);
  cut.Build();
  if (!cut.IsDone() || cut.HasErrors() || cut.Shape().IsNull()) {
    throw GeneratorError("construction",
                         "failed to cut through-holes for " + std::string(fixture_id));
  }

  TopExp_Explorer solid_explorer(cut.Shape(), TopAbs_SOLID);
  if (!solid_explorer.More()) {
    throw GeneratorError("construction",
                         "through-hole cut produced no solid for " + std::string(fixture_id));
  }
  const TopoDS_Solid result = TopoDS::Solid(solid_explorer.Current());
  solid_explorer.Next();
  if (solid_explorer.More()) {
    throw GeneratorError("construction", "through-hole cut produced multiple solids for " +
                                             std::string(fixture_id));
  }

  return result;
}

TopoDS_Shape make_single_hole_plate(const std::span<const ConstructionParameter> parameters) {
  const gp_Circ hole(gp_Ax2(gp_Pnt(parameter(parameters, "hole_center_x"),
                                   parameter(parameters, "hole_center_y"), 0.0),
                            gp_Dir(0.0, 0.0, 1.0)),
                     parameter(parameters, "hole_radius"));
  const std::array holes{hole};
  return cut_plate(parameters, holes, "solid.plate.circular_through_hole");
}

TopoDS_Shape make_two_hole_plate(const std::span<const ConstructionParameter> parameters) {
  const gp_Dir normal(0.0, 0.0, 1.0);
  const std::array holes{
      gp_Circ(gp_Ax2(gp_Pnt(parameter(parameters, "hole_1_center_x"),
                            parameter(parameters, "hole_1_center_y"), 0.0),
                     normal),
              parameter(parameters, "hole_1_radius")),
      gp_Circ(gp_Ax2(gp_Pnt(parameter(parameters, "hole_2_center_x"),
                            parameter(parameters, "hole_2_center_y"), 0.0),
                     normal),
              parameter(parameters, "hole_2_radius")),
  };
  return cut_plate(parameters, holes, "solid.plate.two_circular_through_holes");
}

TopoDS_Shape make_concave_prism(const std::span<const ConstructionParameter> parameters) {
  const double length_x = parameter(parameters, "length_x");
  const double length_y = parameter(parameters, "length_y");
  const double notch_x = parameter(parameters, "notch_x");
  const double notch_y = parameter(parameters, "notch_y");
  BRepBuilderAPI_MakePolygon polygon;
  polygon.Add(gp_Pnt(0.0, 0.0, 0.0));
  polygon.Add(gp_Pnt(length_x, 0.0, 0.0));
  polygon.Add(gp_Pnt(length_x, notch_y, 0.0));
  polygon.Add(gp_Pnt(notch_x, notch_y, 0.0));
  polygon.Add(gp_Pnt(notch_x, length_y, 0.0));
  polygon.Add(gp_Pnt(0.0, length_y, 0.0));
  polygon.Close();
  if (!polygon.IsDone()) {
    throw GeneratorError("construction", "failed to build concave prism boundary");
  }
  BRepBuilderAPI_MakeFace face(polygon.Wire(), true);
  if (!face.IsDone()) {
    throw GeneratorError("construction", "failed to build concave prism face");
  }
  return extrude_face(face.Face(), parameter(parameters, "height"), true, "solid.prism.concave");
}

TopoDS_Wire make_full_ellipse_wire(const gp_Elips& ellipse) {
  BRepBuilderAPI_MakeEdge first_half(ellipse, 0.0, std::numbers::pi);
  BRepBuilderAPI_MakeEdge second_half(ellipse, std::numbers::pi, 2.0 * std::numbers::pi);
  if (!first_half.IsDone() || !second_half.IsDone()) {
    throw GeneratorError("construction", "failed to build full ellipse boundary edges");
  }
  BRepBuilderAPI_MakeWire wire(first_half.Edge(), second_half.Edge());
  if (!wire.IsDone()) {
    throw GeneratorError("construction", "failed to build full ellipse wire");
  }
  return wire.Wire();
}

TopoDS_Wire make_partial_ellipse_wire(const gp_Elips& ellipse) {
  BRepBuilderAPI_MakeEdge arc(ellipse, 0.0, std::numbers::pi);
  BRepBuilderAPI_MakeEdge chord(ElCLib::Value(std::numbers::pi, ellipse),
                                ElCLib::Value(0.0, ellipse));
  if (!arc.IsDone() || !chord.IsDone()) {
    throw GeneratorError("construction", "failed to build partial ellipse boundary edges");
  }
  BRepBuilderAPI_MakeWire wire(arc.Edge(), chord.Edge());
  if (!wire.IsDone()) {
    throw GeneratorError("construction", "failed to build partial ellipse wire");
  }
  return wire.Wire();
}

TopoDS_Shape make_ellipse_prism(const std::span<const ConstructionParameter> parameters,
                                const bool full, const std::string_view fixture_id) {
  const gp_Elips ellipse(gp_Ax2(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                         parameter(parameters, "major_radius"),
                         parameter(parameters, "minor_radius"));
  const TopoDS_Wire wire =
      full ? make_full_ellipse_wire(ellipse) : make_partial_ellipse_wire(ellipse);
  BRepBuilderAPI_MakeFace face(wire, true);
  if (!face.IsDone()) {
    throw GeneratorError("construction", "failed to build ellipse prism face");
  }
  return extrude_face(face.Face(), parameter(parameters, "height"), false, fixture_id);
}

TopoDS_Shape make_full_ellipse_prism(const std::span<const ConstructionParameter> parameters) {
  return make_ellipse_prism(parameters, true, "solid.prism.ellipse.full");
}

TopoDS_Shape make_partial_ellipse_prism(const std::span<const ConstructionParameter> parameters) {
  return make_ellipse_prism(parameters, false, "solid.prism.ellipse.partial_cap");
}

TopoDS_Compound make_compound(const std::span<const TopoDS_Shape> children) {
  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const TopoDS_Shape& child : children) {
    if (child.IsNull()) {
      throw GeneratorError("construction", "cannot add a null child to a fixture compound");
    }
    builder.Add(compound, child);
  }
  return compound;
}

TopLoc_Location translation(const double x, const double y, const double z) {
  gp_Trsf transform;
  transform.SetTranslation(gp_Vec(x, y, z));
  return TopLoc_Location(transform);
}

TopoDS_Shape make_coincident_box_pair(const std::span<const ConstructionParameter> parameters) {
  BRepPrimAPI_MakeBox first_builder(parameter(parameters, "length_x"),
                                    parameter(parameters, "length_y"),
                                    parameter(parameters, "length_z"));
  BRepPrimAPI_MakeBox second_builder(parameter(parameters, "length_x"),
                                     parameter(parameters, "length_y"),
                                     parameter(parameters, "length_z"));
  const TopoDS_Shape first = checked_shape(first_builder, "compound.box_pair.coincident");
  const TopoDS_Shape second = checked_shape(second_builder, "compound.box_pair.coincident");
  if (first.IsPartner(second)) {
    throw GeneratorError("construction", "coincident boxes unexpectedly share a TShape");
  }
  const std::array children{first, second};
  return make_compound(children);
}

TopoDS_Shape make_open_box_shell(const std::span<const ConstructionParameter> parameters) {
  BRepPrimAPI_MakeBox box_builder(parameter(parameters, "length_x"),
                                  parameter(parameters, "length_y"),
                                  parameter(parameters, "length_z"));
  const TopoDS_Shape box = checked_shape(box_builder, "shell.box.open");
  BRep_Builder builder;
  TopoDS_Shell shell;
  builder.MakeShell(shell);
  int face_index = 0;
  for (TopExp_Explorer explorer(box, TopAbs_FACE); explorer.More(); explorer.Next()) {
    ++face_index;
    if (face_index != 1) {
      builder.Add(shell, explorer.Current());
    }
  }
  if (face_index != 6) {
    throw GeneratorError("construction", "box source did not contain exactly six faces");
  }
  return shell;
}

TopoDS_Shape make_inner_void_box(const std::span<const ConstructionParameter> parameters) {
  const double wall = parameter(parameters, "wall_thickness");
  const double outer_x = parameter(parameters, "outer_length_x");
  const double outer_y = parameter(parameters, "outer_length_y");
  const double outer_z = parameter(parameters, "outer_length_z");
  if (wall <= 0.0 || 2.0 * wall >= std::min({outer_x, outer_y, outer_z})) {
    throw GeneratorError("recipe", "inner-void wall thickness must leave a positive cavity");
  }

  BRepPrimAPI_MakeBox outer_builder(outer_x, outer_y, outer_z);
  BRepPrimAPI_MakeBox inner_builder(gp_Pnt(wall, wall, wall), outer_x - (2.0 * wall),
                                    outer_y - (2.0 * wall), outer_z - (2.0 * wall));
  const TopoDS_Shape outer = checked_shape(outer_builder, "solid.box.inner_void");
  const TopoDS_Shape inner = checked_shape(inner_builder, "solid.box.inner_void");
  BRepAlgoAPI_Cut cut(outer, inner);
  cut.SetRunParallel(false);
  cut.SetFuzzyValue(0.0);
  cut.SetNonDestructive(true);
  cut.Build();
  if (!cut.IsDone() || cut.HasErrors() || cut.Shape().IsNull()) {
    throw GeneratorError("construction", "failed to cut the inner box cavity");
  }
  std::vector<TopoDS_Shape> solids;
  for (TopExp_Explorer explorer(cut.Shape(), TopAbs_SOLID); explorer.More(); explorer.Next()) {
    solids.push_back(explorer.Current());
  }
  if (solids.size() != 1) {
    throw GeneratorError("construction", "inner-void cut did not produce exactly one solid");
  }
  return solids.front();
}

TopoDS_Shape make_shared_face_box_pair(const std::span<const ConstructionParameter> parameters) {
  const double length_x = parameter(parameters, "length_x");
  const double length_y = parameter(parameters, "length_y");
  const double length_z = parameter(parameters, "length_z");
  BRepPrimAPI_MakeBox left_builder(length_x, length_y, length_z);
  BRepPrimAPI_MakeBox right_builder(gp_Pnt(length_x, 0.0, 0.0), length_x, length_y, length_z);
  const TopoDS_Shape left =
      checked_shape(left_builder, "compound.box_pair.shared_face_nonmanifold");
  const TopoDS_Shape right =
      checked_shape(right_builder, "compound.box_pair.shared_face_nonmanifold");

  BOPAlgo_Builder general_fuse;
  general_fuse.SetRunParallel(false);
  general_fuse.SetFuzzyValue(0.0);
  general_fuse.SetNonDestructive(true);
  general_fuse.AddArgument(left);
  general_fuse.AddArgument(right);
  general_fuse.Perform();
  if (general_fuse.HasErrors() || general_fuse.Shape().IsNull()) {
    throw GeneratorError("construction", "general fuse failed for shared-face boxes");
  }
  std::vector<TopoDS_Shape> solids;
  for (TopExp_Explorer explorer(general_fuse.Shape(), TopAbs_SOLID); explorer.More();
       explorer.Next()) {
    solids.push_back(explorer.Current());
  }
  if (solids.size() != 2) {
    throw GeneratorError("construction", "shared-face fuse did not retain exactly two solids");
  }
  const std::array children{solids[0], solids[1]};
  return make_compound(children);
}

TopoDS_Shape
make_free_edge_isolated_vertex(const std::span<const ConstructionParameter> parameters) {
  BRepPrimAPI_MakeBox box_builder(parameter(parameters, "box_length_x"),
                                  parameter(parameters, "box_length_y"),
                                  parameter(parameters, "box_length_z"));
  const TopoDS_Shape box = checked_shape(box_builder, "compound.free_edge_isolated_vertex");
  BRepBuilderAPI_MakeEdge edge_builder(
      gp_Pnt(parameter(parameters, "edge_start_x"), parameter(parameters, "edge_start_y"),
             parameter(parameters, "edge_start_z")),
      gp_Pnt(parameter(parameters, "edge_end_x"), parameter(parameters, "edge_end_y"),
             parameter(parameters, "edge_end_z")));
  const TopoDS_Shape edge = checked_shape(edge_builder, "compound.free_edge_isolated_vertex");
  BRepBuilderAPI_MakeVertex vertex_builder(gp_Pnt(parameter(parameters, "vertex_x"),
                                                  parameter(parameters, "vertex_y"),
                                                  parameter(parameters, "vertex_z")));
  const TopoDS_Shape vertex = checked_shape(vertex_builder, "compound.free_edge_isolated_vertex");
  const std::array children{box, edge, vertex};
  return make_compound(children);
}

occ::handle<TDocStd_Document>
make_nested_repeated_box_assembly(const std::span<const ConstructionParameter> parameters) {
  const occ::handle<TDocStd_Document> document =
      new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
  XCAFDoc_DocumentTool::Set(document->Main());
  const occ::handle<XCAFDoc_ShapeTool> tool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
  if (tool.IsNull()) {
    throw GeneratorError("construction", "could not create an XDE shape tool");
  }

  BRepPrimAPI_MakeBox box_builder(parameter(parameters, "length_x"),
                                  parameter(parameters, "length_y"),
                                  parameter(parameters, "length_z"));
  const TDF_Label part = tool->AddShape(
      checked_shape(box_builder, "assembly.box.nested_repeated_instances"), false, false);
  const TDF_Label subassembly = tool->NewShape();
  TDataStd_Name::Set(part, TCollection_ExtendedString("box_definition"));
  TDataStd_Name::Set(subassembly, TCollection_ExtendedString("repeated_box_subassembly"));
  const TDF_Label sub_first = tool->AddComponent(subassembly, part, TopLoc_Location());
  const TDF_Label sub_second = tool->AddComponent(subassembly, part, TopLoc_Location());
  const TDF_Label sub_translated = tool->AddComponent(
      subassembly, part, translation(parameter(parameters, "part_translation_x"), 0.0, 0.0));
  TDataStd_Name::Set(sub_first, TCollection_ExtendedString("box_identity_1"));
  TDataStd_Name::Set(sub_second, TCollection_ExtendedString("box_identity_2"));
  TDataStd_Name::Set(sub_translated, TCollection_ExtendedString("box_translated"));

  const TDF_Label root = tool->NewShape();
  TDataStd_Name::Set(root, TCollection_ExtendedString("root_assembly"));
  const TDF_Label root_first = tool->AddComponent(
      root, subassembly, translation(0.0, parameter(parameters, "first_translation_y"), 0.0));
  gp_Trsf rotated;
  rotated.SetRotation(gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
                      parameter(parameters, "second_rotation_z"));
  gp_Trsf moved;
  moved.SetTranslation(gp_Vec(parameter(parameters, "second_translation_x"), 0.0, 0.0));
  const TDF_Label root_second =
      tool->AddComponent(root, subassembly, TopLoc_Location(moved * rotated));
  TDataStd_Name::Set(root_first, TCollection_ExtendedString("subassembly_first"));
  TDataStd_Name::Set(root_second, TCollection_ExtendedString("subassembly_second"));
  tool->UpdateAssemblies();
  return document;
}

const std::vector<Recipe>& recipes() {
  static const std::array<Recipe, 17> baseline{{
      {"assembly.box.nested_repeated_instances",
       {{"first_translation_y", 10.0, "millimetres"},
        {"length_x", 5.0, "millimetres"},
        {"length_y", 6.0, "millimetres"},
        {"length_z", 7.0, "millimetres"},
        {"part_translation_x", 12.0, "millimetres"},
        {"second_rotation_z", 0.5 * std::numbers::pi, "radians"},
        {"second_translation_x", 30.0, "millimetres"}},
       true,
       {},
       false,
       make_nested_repeated_box_assembly},
      {"compound.box_pair.coincident",
       {{"length_x", 5.0, "millimetres"},
        {"length_y", 6.0, "millimetres"},
        {"length_z", 7.0, "millimetres"}},
       true,
       make_coincident_box_pair},
      {"compound.box_pair.shared_face_nonmanifold",
       {{"length_x", 10.0, "millimetres"},
        {"length_y", 10.0, "millimetres"},
        {"length_z", 10.0, "millimetres"}},
       true,
       make_shared_face_box_pair,
       true},
      {"compound.free_edge_isolated_vertex",
       {{"box_length_x", 6.0, "millimetres"},
        {"box_length_y", 7.0, "millimetres"},
        {"box_length_z", 8.0, "millimetres"},
        {"edge_end_x", 20.0, "millimetres"},
        {"edge_end_y", 9.0, "millimetres"},
        {"edge_end_z", 1.0, "millimetres"},
        {"edge_start_x", 15.0, "millimetres"},
        {"edge_start_y", 7.0, "millimetres"},
        {"edge_start_z", 0.0, "millimetres"},
        {"vertex_x", 25.0, "millimetres"},
        {"vertex_y", 4.0, "millimetres"},
        {"vertex_z", 2.0, "millimetres"}},
       true,
       make_free_edge_isolated_vertex},
      {"shell.box.open",
       {{"length_x", 10.0, "millimetres"},
        {"length_y", 11.0, "millimetres"},
        {"length_z", 12.0, "millimetres"}},
       true,
       make_open_box_shell},
      {"solid.box",
       {{"length_x", 10.0, "millimetres"},
        {"length_y", 20.0, "millimetres"},
        {"length_z", 30.0, "millimetres"}},
       true,
       make_box},
      {"solid.box.inner_void",
       {{"outer_length_x", 12.0, "millimetres"},
        {"outer_length_y", 13.0, "millimetres"},
        {"outer_length_z", 14.0, "millimetres"},
        {"wall_thickness", 2.0, "millimetres"}},
       true,
       make_inner_void_box},
      {"solid.cone.frustum",
       {{"height", 25.0, "millimetres"},
        {"radius_at_z0", 5.0, "millimetres"},
        {"radius_at_zmax", 12.0, "millimetres"},
        {"sweep_angle", 2.0 * std::numbers::pi, "radians"}},
       true,
       make_frustum},
      {"solid.cylinder.full",
       {{"height", 25.0, "millimetres"},
        {"radius", 10.0, "millimetres"},
        {"sweep_angle", 2.0 * std::numbers::pi, "radians"}},
       true,
       make_full_cylinder},
      {"solid.cylinder.partial",
       {{"height", 25.0, "millimetres"},
        {"radius", 10.0, "millimetres"},
        {"sweep_angle", 1.5 * std::numbers::pi, "radians"}},
       true,
       make_partial_cylinder},
      {"solid.plate.circular_through_hole",
       {{"height", 8.0, "millimetres"},
        {"hole_center_x", 18.0, "millimetres"},
        {"hole_center_y", 14.0, "millimetres"},
        {"hole_radius", 4.0, "millimetres"},
        {"length_x", 36.0, "millimetres"},
        {"length_y", 28.0, "millimetres"},
        {"overcut", 5.0, "millimetres"}},
       true,
       make_single_hole_plate},
      {"solid.plate.two_circular_through_holes",
       {{"height", 8.0, "millimetres"},
        {"hole_1_center_x", 12.0, "millimetres"},
        {"hole_1_center_y", 14.0, "millimetres"},
        {"hole_1_radius", 3.0, "millimetres"},
        {"hole_2_center_x", 27.0, "millimetres"},
        {"hole_2_center_y", 15.0, "millimetres"},
        {"hole_2_radius", 4.0, "millimetres"},
        {"length_x", 42.0, "millimetres"},
        {"length_y", 30.0, "millimetres"},
        {"overcut", 5.0, "millimetres"}},
       true,
       make_two_hole_plate},
      {"solid.prism.concave",
       {{"height", 9.0, "millimetres"},
        {"length_x", 30.0, "millimetres"},
        {"length_y", 25.0, "millimetres"},
        {"notch_x", 15.0, "millimetres"},
        {"notch_y", 10.0, "millimetres"}},
       true,
       make_concave_prism},
      {"solid.prism.ellipse.full",
       {{"height", 12.0, "millimetres"},
        {"major_radius", 10.0, "millimetres"},
        {"minor_radius", 6.0, "millimetres"}},
       true,
       make_full_ellipse_prism},
      {"solid.prism.ellipse.partial_cap",
       {{"height", 12.0, "millimetres"},
        {"major_radius", 10.0, "millimetres"},
        {"minor_radius", 6.0, "millimetres"}},
       true,
       make_partial_ellipse_prism},
      {"solid.sphere.segment",
       {{"latitude_max", std::numbers::pi / 3.0, "radians"},
        {"latitude_min", -0.25 * std::numbers::pi, "radians"},
        {"radius", 12.0, "millimetres"},
        {"sweep_angle", 1.5 * std::numbers::pi, "radians"}},
       false,
       make_sphere_segment},
      {"solid.torus.segment",
       {{"major_radius", 20.0, "millimetres"},
        {"minor_radius", 5.0, "millimetres"},
        {"sweep_angle", 1.5 * std::numbers::pi, "radians"},
        {"v_max", std::numbers::pi / 3.0, "radians"},
        {"v_min", -std::numbers::pi / 3.0, "radians"}},
       true,
       make_torus_segment},
  }};
  static const std::vector<Recipe> values = [] {
    std::vector<Recipe> result(baseline.begin(), baseline.end());
    append_curve_recipes(result);
    append_surface_recipes(result);
    append_topology_recipes(result);
    append_feature_recipes(result);
    append_pathology_recipes(result);
    append_trim_support_recipes(result);
    std::sort(result.begin(), result.end(),
              [](const Recipe& left, const Recipe& right) { return left.id < right.id; });
    const auto duplicate = std::adjacent_find(
        result.begin(), result.end(),
        [](const Recipe& left, const Recipe& right) { return left.id == right.id; });
    if (duplicate != result.end()) {
      throw GeneratorError("recipe", "duplicate fixture recipe ID: " + std::string(duplicate->id));
    }
    return result;
  }();
  return values;
}

const Recipe* find_recipe(const std::string_view fixture_id) {
  for (const Recipe& recipe : recipes()) {
    if (recipe.id == fixture_id) {
      return &recipe;
    }
  }
  return nullptr;
}

std::string shape_type_name(const TopAbs_ShapeEnum type) {
  switch (type) {
  case TopAbs_COMPOUND:
    return "Compound";
  case TopAbs_COMPSOLID:
    return "CompSolid";
  case TopAbs_SOLID:
    return "Solid";
  case TopAbs_SHELL:
    return "Shell";
  case TopAbs_FACE:
    return "Face";
  case TopAbs_WIRE:
    return "Wire";
  case TopAbs_EDGE:
    return "Edge";
  case TopAbs_VERTEX:
    return "Vertex";
  case TopAbs_SHAPE:
    return "Shape";
  }
  return "Unknown";
}

std::string curve_type_name(const GeomAbs_CurveType type) {
  switch (type) {
  case GeomAbs_Line:
    return "Line";
  case GeomAbs_Circle:
    return "Circle";
  case GeomAbs_Ellipse:
    return "Ellipse";
  case GeomAbs_Hyperbola:
    return "Hyperbola";
  case GeomAbs_Parabola:
    return "Parabola";
  case GeomAbs_BezierCurve:
    return "BezierCurve";
  case GeomAbs_BSplineCurve:
    return "BSplineCurve";
  case GeomAbs_OffsetCurve:
    return "OffsetCurve";
  case GeomAbs_OtherCurve:
    return "OtherCurve";
  }
  return "UnknownCurve";
}

std::string surface_type_name(const GeomAbs_SurfaceType type) {
  switch (type) {
  case GeomAbs_Plane:
    return "Plane";
  case GeomAbs_Cylinder:
    return "Cylinder";
  case GeomAbs_Cone:
    return "Cone";
  case GeomAbs_Sphere:
    return "Sphere";
  case GeomAbs_Torus:
    return "Torus";
  case GeomAbs_BezierSurface:
    return "BezierSurface";
  case GeomAbs_BSplineSurface:
    return "BSplineSurface";
  case GeomAbs_SurfaceOfRevolution:
    return "SurfaceOfRevolution";
  case GeomAbs_SurfaceOfExtrusion:
    return "SurfaceOfExtrusion";
  case GeomAbs_OffsetSurface:
    return "OffsetSurface";
  case GeomAbs_OtherSurface:
    return "OtherSurface";
  }
  return "UnknownSurface";
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

std::string occurrence_id(const std::string_view prefix, const std::size_t ordinal) {
  std::ostringstream stream;
  stream << prefix << std::setw(4) << std::setfill('0') << ordinal;
  return stream.str();
}

OrientationCounts orientation_counts(const TopoDS_Shape& shape, const TopAbs_ShapeEnum type) {
  OrientationCounts counts;
  for (TopExp_Explorer explorer(shape, type); explorer.More(); explorer.Next()) {
    switch (explorer.Current().Orientation()) {
    case TopAbs_FORWARD:
      ++counts.forward;
      break;
    case TopAbs_REVERSED:
      ++counts.reversed;
      break;
    case TopAbs_INTERNAL:
      ++counts.internal;
      break;
    case TopAbs_EXTERNAL:
      ++counts.external;
      break;
    }
  }
  return counts;
}

TopologyCount topology_count(const TopoDS_Shape& shape, const TopAbs_ShapeEnum type,
                             const std::string_view family) {
  std::vector<TopoDS_Shape> unique_shapes;
  for (TopExp_Explorer explorer(shape, type); explorer.More(); explorer.Next()) {
    const TopoDS_Shape& current = explorer.Current();
    const bool known = std::any_of(
        unique_shapes.begin(), unique_shapes.end(),
        [&current](const TopoDS_Shape& candidate) { return candidate.IsPartner(current); });
    if (!known) {
      unique_shapes.push_back(current);
    }
  }
  const OrientationCounts orientations = orientation_counts(shape, type);
  return TopologyCount{family, static_cast<int>(unique_shapes.size()), orientations.total(),
                       orientations};
}

std::vector<TopoDS_Shape> unique_partner_shapes(const TopoDS_Shape& shape,
                                                const TopAbs_ShapeEnum type) {
  std::vector<TopoDS_Shape> result;
  for (TopExp_Explorer explorer(shape, type); explorer.More(); explorer.Next()) {
    const TopoDS_Shape& current = explorer.Current();
    const bool known =
        std::any_of(result.begin(), result.end(), [&current](const TopoDS_Shape& candidate) {
          return candidate.IsPartner(current);
        });
    if (!known) {
      result.push_back(current);
    }
  }
  return result;
}

int partner_ordinal(const TopoDS_Shape& shape, const std::vector<TopoDS_Shape>& partners) {
  const auto found =
      std::find_if(partners.begin(), partners.end(),
                   [&shape](const TopoDS_Shape& candidate) { return candidate.IsPartner(shape); });
  if (found == partners.end()) {
    throw GeneratorError("inventory", "occurrence has no unique partner entry");
  }
  return static_cast<int>(std::distance(partners.begin(), found)) + 1;
}

struct EdgeIncidence final {
  int face_uses{};
  bool periodic_seam{};
};

EdgeIncidence edge_incidence(const TopoDS_Edge& edge, const std::vector<TopoDS_Shape>& faces) {
  EdgeIncidence result;
  for (const TopoDS_Shape& face_shape : faces) {
    const TopoDS_Face& face = TopoDS::Face(face_shape);
    int uses_on_face = 0;
    for (TopExp_Explorer explorer(face, TopAbs_EDGE); explorer.More(); explorer.Next()) {
      if (explorer.Current().IsPartner(edge)) {
        ++uses_on_face;
      }
    }
    result.face_uses += uses_on_face;
    result.periodic_seam = result.periodic_seam || BRep_Tool::IsClosed(edge, face);
  }
  return result;
}

double quantize_value(const double value, const double quantum, const std::string_view label) {
  if (!std::isfinite(value)) {
    throw GeneratorError("inventory", std::string(label) + " contains a non-finite value");
  }
  const double scaled = value / quantum;
  if (!std::isfinite(scaled)) {
    throw GeneratorError("inventory", std::string(label) + " exceeds the quantization range");
  }
  double result = std::round(scaled) * quantum;
  if (result == 0.0) {
    result = 0.0;
  }
  return result;
}

Matrix4 location_matrix(const TopLoc_Location& location) {
  const gp_Trsf transform = location.Transformation();
  Matrix4 matrix{};
  for (int row = 1; row <= 3; ++row) {
    for (int column = 1; column <= 4; ++column) {
      matrix[static_cast<std::size_t>((row - 1) * 4 + (column - 1))] =
          quantize_value(transform.Value(row, column), kLocationQuantum, "location matrix");
    }
  }
  matrix[15] = 1.0;
  return matrix;
}

Matrix4 location_matrix(const gp_Trsf& transform) {
  return location_matrix(TopLoc_Location(transform));
}

bool is_identity_matrix(const Matrix4& matrix) {
  constexpr Matrix4 identity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  return matrix == identity;
}

bool contains_partner(const TopoDS_Shape& ancestor, const TopoDS_Shape& child,
                      const TopAbs_ShapeEnum child_type) {
  for (TopExp_Explorer explorer(ancestor, child_type); explorer.More(); explorer.Next()) {
    if (explorer.Current().IsPartner(child)) {
      return true;
    }
  }
  return false;
}

int ancestor_partner_count(const TopoDS_Shape& child, const std::vector<TopoDS_Shape>& ancestors,
                           const TopAbs_ShapeEnum child_type) {
  return static_cast<int>(std::count_if(ancestors.begin(), ancestors.end(),
                                        [&child, child_type](const TopoDS_Shape& ancestor) {
                                          return contains_partner(ancestor, child, child_type);
                                        }));
}

int free_partner_shape_count(const std::vector<TopoDS_Shape>& children,
                             const std::vector<TopoDS_Shape>& ancestors,
                             const TopAbs_ShapeEnum child_type) {
  return static_cast<int>(std::count_if(
      children.begin(), children.end(), [&ancestors, child_type](const TopoDS_Shape& child) {
        return ancestor_partner_count(child, ancestors, child_type) == 0;
      }));
}

std::string shell_role(const TopoDS_Shape& shell_shape, const std::vector<TopoDS_Shape>& solids) {
  std::optional<std::string> observed_role;
  for (const TopoDS_Shape& solid_shape : solids) {
    if (!contains_partner(solid_shape, shell_shape, TopAbs_SHELL)) {
      continue;
    }
    const TopoDS_Shell outer = BRepClass3d::OuterShell(TopoDS::Solid(solid_shape));
    const std::string current_role =
        !outer.IsNull() && outer.IsPartner(shell_shape) ? "outer" : "inner";
    if (observed_role.has_value() && *observed_role != current_role) {
      throw GeneratorError("inventory", "shell has conflicting outer and inner roles");
    }
    observed_role = current_role;
  }
  return observed_role.value_or("standalone");
}

std::string subshape_shell_role(const TopoDS_Shape& subshape, const TopAbs_ShapeEnum subshape_type,
                                const std::vector<TopoDS_Shape>& shells,
                                const std::vector<TopoDS_Shape>& solids) {
  std::optional<std::string> observed_role;
  for (const TopoDS_Shape& shell : shells) {
    if (!contains_partner(shell, subshape, subshape_type)) {
      continue;
    }
    const std::string current_role = shell_role(shell, solids);
    if (observed_role.has_value() && *observed_role != current_role) {
      throw GeneratorError("inventory", "subshape belongs to shells with conflicting roles");
    }
    observed_role = current_role;
  }
  return observed_role.value_or("standalone");
}

using SolidSignature = std::array<double, 10>;

SolidSignature solid_signature(const TopoDS_Shape& solid) {
  Bnd_Box bounds;
  BRepBndLib::AddOptimal(solid, bounds, false, false);
  if (bounds.IsVoid() || bounds.IsOpen()) {
    throw GeneratorError("inventory", "solid has an unbounded coincidence signature");
  }
  double x_min = 0.0;
  double y_min = 0.0;
  double z_min = 0.0;
  double x_max = 0.0;
  double y_max = 0.0;
  double z_max = 0.0;
  bounds.Get(x_min, y_min, z_min, x_max, y_max, z_max);
  GProp_GProps properties;
  BRepGProp::VolumeProperties(solid, properties);
  const gp_Pnt centre = properties.CentreOfMass();
  SolidSignature result{x_min,      y_min,      z_min,     x_max, y_max, z_max, properties.Mass(),
                        centre.X(), centre.Y(), centre.Z()};
  for (double& value : result) {
    value = quantize_value(value, kCoincidenceQuantumMm, "solid coincidence signature");
  }
  return result;
}

ModelTopology observe_model_topology(const std::vector<TopoDS_Shape>& vertices,
                                     const std::vector<TopoDS_Shape>& edges,
                                     const std::vector<TopoDS_Shape>& faces,
                                     const std::vector<TopoDS_Shape>& wires,
                                     const std::vector<TopoDS_Shape>& shells,
                                     const std::vector<TopoDS_Shape>& solids) {
  ModelTopology result;
  for (const TopoDS_Shape& shell : shells) {
    if (BRep_Tool::IsClosed(shell)) {
      ++result.closed_shells;
    } else {
      ++result.open_shells;
    }
    const std::string role = shell_role(shell, solids);
    if (role == "outer") {
      ++result.outer_shells;
    } else if (role == "inner") {
      ++result.inner_shells;
    } else {
      ++result.standalone_shells;
    }
  }

  for (const TopoDS_Shape& face : faces) {
    if (ancestor_partner_count(face, solids, TopAbs_FACE) > 1) {
      ++result.shared_faces_between_solids;
    }
  }

  std::vector<TopoDS_Shape> nonmanifold_vertex_partners;
  for (const TopoDS_Shape& edge_shape : edges) {
    const EdgeIncidence incidence = edge_incidence(TopoDS::Edge(edge_shape), faces);
    result.maximum_faces_per_edge = std::max(result.maximum_faces_per_edge, incidence.face_uses);
    if (incidence.face_uses <= 2) {
      continue;
    }
    for (TopExp_Explorer explorer(edge_shape, TopAbs_VERTEX); explorer.More(); explorer.Next()) {
      const TopoDS_Shape& vertex = explorer.Current();
      if (std::none_of(nonmanifold_vertex_partners.begin(), nonmanifold_vertex_partners.end(),
                       [&vertex](const TopoDS_Shape& known) { return known.IsPartner(vertex); })) {
        nonmanifold_vertex_partners.push_back(vertex);
      }
    }
  }
  result.nonmanifold_vertices = static_cast<int>(nonmanifold_vertex_partners.size());
  result.free_wires = free_partner_shape_count(wires, faces, TopAbs_WIRE);
  result.free_edges = free_partner_shape_count(edges, faces, TopAbs_EDGE);
  result.isolated_vertices = free_partner_shape_count(vertices, edges, TopAbs_VERTEX);

  std::vector<SolidSignature> signatures;
  signatures.reserve(solids.size());
  for (const TopoDS_Shape& solid : solids) {
    signatures.push_back(solid_signature(solid));
  }
  for (std::size_t left = 0; left < signatures.size(); ++left) {
    for (std::size_t right = left + 1; right < signatures.size(); ++right) {
      if (signatures[left] == signatures[right]) {
        ++result.coincident_solid_signature_pairs;
      }
    }
  }
  return result;
}

void observe_named_orientation(OrientationCounts& counts, const std::string_view orientation) {
  if (orientation == "FORWARD") {
    ++counts.forward;
  } else if (orientation == "REVERSED") {
    ++counts.reversed;
  } else if (orientation == "INTERNAL") {
    ++counts.internal;
  } else if (orientation == "EXTERNAL") {
    ++counts.external;
  } else {
    throw GeneratorError("inventory", "occurrence has an unknown orientation");
  }
}

void canonicalize_edge_partner_ordinals(std::vector<EdgeOccurrence>& occurrences) {
  struct Observation final {
    std::string family;
    std::string concrete_type;
    bool degenerated{};
    bool missing_3d_curve{};
    bool periodic_seam{};
    int face_use_count{};
    std::string shell_role;
    OrientationCounts orientations;
  };

  std::map<int, Observation> observations;
  for (const EdgeOccurrence& occurrence : occurrences) {
    auto [iterator, inserted] = observations.try_emplace(occurrence.partner_ordinal,
                                                         Observation{occurrence.family,
                                                                     occurrence.concrete_type,
                                                                     occurrence.degenerated,
                                                                     occurrence.missing_3d_curve,
                                                                     occurrence.periodic_seam,
                                                                     occurrence.face_use_count,
                                                                     occurrence.shell_role,
                                                                     {}});
    Observation& observation = iterator->second;
    if (!inserted && (observation.family != occurrence.family ||
                      observation.concrete_type != occurrence.concrete_type ||
                      observation.degenerated != occurrence.degenerated ||
                      observation.missing_3d_curve != occurrence.missing_3d_curve ||
                      observation.periodic_seam != occurrence.periodic_seam ||
                      observation.face_use_count != occurrence.face_use_count ||
                      observation.shell_role != occurrence.shell_role)) {
      throw GeneratorError("inventory", "edge partner occurrences disagree on geometry or flags");
    }
    observe_named_orientation(observation.orientations, occurrence.orientation);
  }

  std::vector<int> original_ordinals;
  original_ordinals.reserve(observations.size());
  for (const auto& [ordinal, observation] : observations) {
    static_cast<void>(observation);
    original_ordinals.push_back(ordinal);
  }
  std::sort(
      original_ordinals.begin(), original_ordinals.end(),
      [&observations](const int left_ordinal, const int right_ordinal) {
        const Observation& left = observations.at(left_ordinal);
        const Observation& right = observations.at(right_ordinal);
        return std::tie(left.family, left.concrete_type, left.degenerated, left.missing_3d_curve,
                        left.periodic_seam, left.face_use_count, left.shell_role,
                        left.orientations.forward, left.orientations.reversed,
                        left.orientations.internal, left.orientations.external, left_ordinal) <
               std::tie(right.family, right.concrete_type, right.degenerated,
                        right.missing_3d_curve, right.periodic_seam, right.face_use_count,
                        right.shell_role, right.orientations.forward, right.orientations.reversed,
                        right.orientations.internal, right.orientations.external, right_ordinal);
      });

  std::map<int, int> canonical_ordinals;
  for (std::size_t index = 0; index < original_ordinals.size(); ++index) {
    canonical_ordinals.emplace(original_ordinals[index], static_cast<int>(index) + 1);
  }
  for (EdgeOccurrence& occurrence : occurrences) {
    occurrence.partner_ordinal = canonical_ordinals.at(occurrence.partner_ordinal);
  }
}

bool boundary_loop_less(const BoundaryLoop& left, const BoundaryLoop& right) {
  const int left_role = left.role == "outer" ? 0 : 1;
  const int right_role = right.role == "outer" ? 0 : 1;
  return std::tuple{left_role,  left.orientation, left.closed, left.edge_occurrence_count,
                    left.u_min, left.u_max,       left.v_min,  left.v_max} <
         std::tuple{right_role,  right.orientation, right.closed, right.edge_occurrence_count,
                    right.u_min, right.u_max,       right.v_min,  right.v_max};
}

std::vector<BoundaryLoop> observe_boundary_loops(const TopoDS_Face& face) {
  const TopoDS_Wire outer_wire = BRepTools::OuterWire(face);
  if (outer_wire.IsNull()) {
    throw GeneratorError("inventory", "face occurrence has no outer wire");
  }

  std::vector<BoundaryLoop> loops;
  int outer_count = 0;
  for (TopExp_Explorer explorer(face, TopAbs_WIRE); explorer.More(); explorer.Next()) {
    const TopoDS_Wire wire = TopoDS::Wire(explorer.Current());
    BoundaryLoop loop;
    loop.role = wire.IsPartner(outer_wire) ? "outer" : "inner";
    if (loop.role == "outer") {
      ++outer_count;
    }
    loop.orientation = orientation_name(wire.Orientation());
    loop.closed = BRep_Tool::IsClosed(wire);
    for (TopExp_Explorer edge_explorer(wire, TopAbs_EDGE); edge_explorer.More();
         edge_explorer.Next()) {
      ++loop.edge_occurrence_count;
    }
    if (loop.edge_occurrence_count <= 0) {
      throw GeneratorError("inventory", "boundary loop has no edge occurrences");
    }
    BRepTools::UVBounds(face, wire, loop.u_min, loop.u_max, loop.v_min, loop.v_max);
    for (double* const bound : {&loop.u_min, &loop.u_max, &loop.v_min, &loop.v_max}) {
      const double scaled = *bound / kBoundaryLoopUvQuantum;
      if (!std::isfinite(scaled)) {
        throw GeneratorError("inventory", "boundary-loop UV bound exceeds canonical range");
      }
      *bound = std::round(scaled) * kBoundaryLoopUvQuantum;
      if (*bound == 0.0) {
        *bound = 0.0;
      }
    }
    if (!std::isfinite(loop.u_min) || !std::isfinite(loop.u_max) || !std::isfinite(loop.v_min) ||
        !std::isfinite(loop.v_max) || loop.u_min > loop.u_max || loop.v_min > loop.v_max) {
      throw GeneratorError("inventory", "boundary loop has invalid UV bounds");
    }
    loops.push_back(std::move(loop));
  }
  if (outer_count != 1) {
    throw GeneratorError("inventory", "face occurrence does not have exactly one outer wire");
  }
  std::sort(loops.begin(), loops.end(), boundary_loop_less);
  return loops;
}

std::string boundary_loop_signature(const std::vector<BoundaryLoop>& loops) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::hexfloat;
  for (const BoundaryLoop& loop : loops) {
    stream << loop.role << '|' << loop.orientation << '|' << loop.closed << '|'
           << loop.edge_occurrence_count << '|' << loop.u_min << '|' << loop.u_max << '|'
           << loop.v_min << '|' << loop.v_max << ';';
  }
  return stream.str();
}

std::map<int, int> canonicalize_face_partner_ordinals(
    std::vector<FaceOccurrence>& occurrences,
    const std::map<int, std::vector<BoundaryLoop>>& loops_by_original_ordinal) {
  struct Observation final {
    std::string family;
    std::string concrete_type;
    std::string boundary_signature;
    int solid_use_count{};
    std::string shell_role;
    OrientationCounts orientations;
  };

  std::map<int, Observation> observations;
  for (const FaceOccurrence& occurrence : occurrences) {
    auto [iterator, inserted] = observations.try_emplace(
        occurrence.partner_ordinal,
        Observation{
            occurrence.family,
            occurrence.concrete_type,
            boundary_loop_signature(loops_by_original_ordinal.at(occurrence.partner_ordinal)),
            occurrence.solid_use_count,
            occurrence.shell_role,
            {}});
    Observation& observation = iterator->second;
    if (!inserted &&
        (observation.family != occurrence.family ||
         observation.concrete_type != occurrence.concrete_type ||
         observation.boundary_signature !=
             boundary_loop_signature(loops_by_original_ordinal.at(occurrence.partner_ordinal)) ||
         observation.solid_use_count != occurrence.solid_use_count ||
         observation.shell_role != occurrence.shell_role)) {
      throw GeneratorError("inventory", "face partner occurrences disagree on geometry");
    }
    observe_named_orientation(observation.orientations, occurrence.orientation);
  }

  std::vector<int> original_ordinals;
  original_ordinals.reserve(observations.size());
  for (const auto& [ordinal, observation] : observations) {
    static_cast<void>(observation);
    original_ordinals.push_back(ordinal);
  }
  std::sort(original_ordinals.begin(), original_ordinals.end(),
            [&observations](const int left_ordinal, const int right_ordinal) {
              const Observation& left = observations.at(left_ordinal);
              const Observation& right = observations.at(right_ordinal);
              return std::tie(left.family, left.concrete_type, left.boundary_signature,
                              left.solid_use_count, left.shell_role, left.orientations.forward,
                              left.orientations.reversed, left.orientations.internal,
                              left.orientations.external, left_ordinal) <
                     std::tie(right.family, right.concrete_type, right.boundary_signature,
                              right.solid_use_count, right.shell_role, right.orientations.forward,
                              right.orientations.reversed, right.orientations.internal,
                              right.orientations.external, right_ordinal);
            });

  std::map<int, int> canonical_ordinals;
  for (std::size_t index = 0; index < original_ordinals.size(); ++index) {
    canonical_ordinals.emplace(original_ordinals[index], static_cast<int>(index) + 1);
  }
  for (FaceOccurrence& occurrence : occurrences) {
    occurrence.partner_ordinal = canonical_ordinals.at(occurrence.partner_ordinal);
  }
  return canonical_ordinals;
}

ShapeSummary summarize_shape(const TopoDS_Shape& shape) {
  if (shape.IsNull()) {
    throw GeneratorError("inventory", "cannot summarize a null shape");
  }

  ShapeSummary summary;
  summary.valid = BRepCheck_Analyzer(shape, true).IsValid();
  summary.root_shape_type = shape_type_name(shape.ShapeType());

  constexpr std::array<std::pair<TopAbs_ShapeEnum, std::string_view>, 8> topology_families{{
      {TopAbs_COMPOUND, "compound"},
      {TopAbs_COMPSOLID, "compsolid"},
      {TopAbs_SOLID, "solid"},
      {TopAbs_SHELL, "shell"},
      {TopAbs_FACE, "face"},
      {TopAbs_WIRE, "wire"},
      {TopAbs_EDGE, "edge"},
      {TopAbs_VERTEX, "vertex"},
  }};
  summary.topology.reserve(topology_families.size());
  for (const auto& [type, family] : topology_families) {
    summary.topology.push_back(topology_count(shape, type, family));
  }

  const std::vector<TopoDS_Shape> vertices = unique_partner_shapes(shape, TopAbs_VERTEX);
  for (const TopoDS_Shape& vertex_shape : vertices) {
    const TopoDS_Vertex& vertex = TopoDS::Vertex(vertex_shape);
    observe_tolerance(summary.vertex_tolerance, BRep_Tool::Tolerance(vertex));
  }

  const std::vector<TopoDS_Shape> edges = unique_partner_shapes(shape, TopAbs_EDGE);
  const std::vector<TopoDS_Shape> faces = unique_partner_shapes(shape, TopAbs_FACE);
  const std::vector<TopoDS_Shape> wires = unique_partner_shapes(shape, TopAbs_WIRE);
  const std::vector<TopoDS_Shape> shells = unique_partner_shapes(shape, TopAbs_SHELL);
  const std::vector<TopoDS_Shape> solids = unique_partner_shapes(shape, TopAbs_SOLID);
  for (const TopoDS_Shape& edge_shape : edges) {
    const TopoDS_Edge& edge = TopoDS::Edge(edge_shape);
    observe_tolerance(summary.edge_tolerance, BRep_Tool::Tolerance(edge));
    if (BRep_Tool::Degenerated(edge)) {
      ++summary.degenerated_edges;
    }

    const EdgeIncidence incidence = edge_incidence(edge, faces);
    if (incidence.periodic_seam) {
      ++summary.periodic_seam_edges;
    }
    if (incidence.face_uses < 2) {
      ++summary.free_boundary_edges;
    } else if (incidence.face_uses > 2) {
      ++summary.nonmanifold_edges;
    }

    TopLoc_Location location;
    double first = 0.0;
    double last = 0.0;
    const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(edge, location, first, last);
    if (curve.IsNull()) {
      ++summary.edges_without_3d_curve;
      continue;
    }
    ++summary.curve_families[curve_type_name(BRepAdaptor_Curve(edge).GetType())];
    ++summary.curve_concrete_types[curve->DynamicType()->Name()];
  }

  for (const TopoDS_Shape& face_shape : faces) {
    const TopoDS_Face& face = TopoDS::Face(face_shape);
    observe_tolerance(summary.face_tolerance, BRep_Tool::Tolerance(face));
    ++summary.surface_families[surface_type_name(BRepAdaptor_Surface(face, true).GetType())];
    const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) {
      throw GeneratorError("inventory", "encountered a face without a surface");
    }
    ++summary.surface_concrete_types[surface->DynamicType()->Name()];
  }

  std::map<int, std::vector<BoundaryLoop>> boundary_loops_by_face_partner;
  for (std::size_t index = 0; index < faces.size(); ++index) {
    boundary_loops_by_face_partner.emplace(static_cast<int>(index) + 1,
                                           observe_boundary_loops(TopoDS::Face(faces[index])));
  }

  for (TopExp_Explorer explorer(shape, TopAbs_EDGE); explorer.More(); explorer.Next()) {
    const TopoDS_Edge edge = TopoDS::Edge(explorer.Current());
    TopLoc_Location location;
    double first = 0.0;
    double last = 0.0;
    const occ::handle<Geom_Curve> curve = BRep_Tool::Curve(edge, location, first, last);
    const EdgeIncidence incidence = edge_incidence(edge, faces);
    EdgeOccurrence occurrence;
    occurrence.family =
        curve.IsNull() ? "Missing3dCurve" : curve_type_name(BRepAdaptor_Curve(edge).GetType());
    occurrence.concrete_type = curve.IsNull() ? "null" : curve->DynamicType()->Name();
    occurrence.orientation = orientation_name(edge.Orientation());
    occurrence.partner_ordinal = partner_ordinal(edge, edges);
    occurrence.degenerated = BRep_Tool::Degenerated(edge);
    occurrence.missing_3d_curve = curve.IsNull();
    occurrence.periodic_seam = incidence.periodic_seam;
    occurrence.face_use_count = incidence.face_uses;
    occurrence.shell_role = subshape_shell_role(edge, TopAbs_EDGE, shells, solids);
    occurrence.geometry_location_matrix_row_major = location_matrix(location);
    occurrence.occurrence_location_matrix_row_major = location_matrix(edge.Location());
    summary.edge_occurrences.push_back(std::move(occurrence));
  }
  canonicalize_edge_partner_ordinals(summary.edge_occurrences);
  std::sort(summary.edge_occurrences.begin(), summary.edge_occurrences.end(),
            [](const EdgeOccurrence& left, const EdgeOccurrence& right) {
              return std::tie(left.family, left.concrete_type, left.partner_ordinal,
                              left.orientation, left.face_use_count, left.shell_role,
                              left.geometry_location_matrix_row_major,
                              left.occurrence_location_matrix_row_major) <
                     std::tie(right.family, right.concrete_type, right.partner_ordinal,
                              right.orientation, right.face_use_count, right.shell_role,
                              right.geometry_location_matrix_row_major,
                              right.occurrence_location_matrix_row_major);
            });
  for (std::size_t index = 0; index < summary.edge_occurrences.size(); ++index) {
    summary.edge_occurrences[index].id = occurrence_id("edge_occurrence.e", index + 1);
  }

  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next()) {
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    TopLoc_Location location;
    const occ::handle<Geom_Surface> surface = BRep_Tool::Surface(face, location);
    if (surface.IsNull()) {
      throw GeneratorError("inventory", "encountered a face occurrence without a surface");
    }
    FaceOccurrence occurrence;
    occurrence.family = surface_type_name(BRepAdaptor_Surface(face, true).GetType());
    occurrence.concrete_type = surface->DynamicType()->Name();
    occurrence.orientation = orientation_name(face.Orientation());
    occurrence.partner_ordinal = partner_ordinal(face, faces);
    occurrence.solid_use_count = ancestor_partner_count(face, solids, TopAbs_FACE);
    occurrence.shell_role = subshape_shell_role(face, TopAbs_FACE, shells, solids);
    occurrence.geometry_location_matrix_row_major = location_matrix(location);
    occurrence.occurrence_location_matrix_row_major = location_matrix(face.Location());
    summary.face_occurrences.push_back(std::move(occurrence));
  }
  const std::map<int, int> face_partner_ordinals =
      canonicalize_face_partner_ordinals(summary.face_occurrences, boundary_loops_by_face_partner);
  std::sort(summary.face_occurrences.begin(), summary.face_occurrences.end(),
            [](const FaceOccurrence& left, const FaceOccurrence& right) {
              return std::tie(left.family, left.concrete_type, left.partner_ordinal,
                              left.orientation, left.solid_use_count, left.shell_role,
                              left.geometry_location_matrix_row_major,
                              left.occurrence_location_matrix_row_major) <
                     std::tie(right.family, right.concrete_type, right.partner_ordinal,
                              right.orientation, right.solid_use_count, right.shell_role,
                              right.geometry_location_matrix_row_major,
                              right.occurrence_location_matrix_row_major);
            });
  for (std::size_t index = 0; index < summary.face_occurrences.size(); ++index) {
    FaceOccurrence& face = summary.face_occurrences[index];
    face.id = occurrence_id("face_occurrence.f", index + 1);
    summary.logical_region_proxies.push_back(LogicalRegionProxy{
        occurrence_id("logical_region.r", index + 1), face.id, face.family, face.concrete_type});

    const auto original =
        std::find_if(face_partner_ordinals.begin(), face_partner_ordinals.end(),
                     [&face](const auto& entry) { return entry.second == face.partner_ordinal; });
    if (original == face_partner_ordinals.end()) {
      throw GeneratorError("inventory", "face occurrence has no canonical partner mapping");
    }
    for (BoundaryLoop loop : boundary_loops_by_face_partner.at(original->first)) {
      loop.face_occurrence_id = face.id;
      summary.boundary_loops.push_back(std::move(loop));
    }
  }
  for (std::size_t index = 0; index < summary.boundary_loops.size(); ++index) {
    summary.boundary_loops[index].id = occurrence_id("boundary_loop.l", index + 1);
  }

  summary.model_topology = observe_model_topology(vertices, edges, faces, wires, shells, solids);

  return summary;
}

DESTEP_Parameters step_parameters(const StepIoProfile& profile) {
  DESTEP_Parameters parameters;
  switch (profile.schema) {
  case StepSchema::ap203:
    parameters.WriteSchema = DESTEP_Parameters::WriteMode_StepSchema_AP203;
    break;
  case StepSchema::ap214:
    parameters.WriteSchema = DESTEP_Parameters::WriteMode_StepSchema_AP214IS;
    break;
  case StepSchema::ap242_dis:
    parameters.WriteSchema = DESTEP_Parameters::WriteMode_StepSchema_AP242DIS;
    break;
  }
  parameters.WriteUnit = UnitsMethods_LengthUnit_Millimeter;
  parameters.WriteTessellated = profile.tessellated_geometry
                                    ? DESTEP_Parameters::RWMode_Tessellated_On
                                    : DESTEP_Parameters::RWMode_Tessellated_Off;
  parameters.WriteSurfaceCurMode = profile.surface_curve_mode;
  parameters.WriteProductName = "CAD Mesher deterministic fixture";
#if OCC_VERSION_HEX >= 0x080000
  parameters.CleanDuplicates = profile.clean_duplicates;
#else
  if (profile.clean_duplicates) {
    throw GeneratorError(
        "unsupported-step-option",
        "clean duplicate processing requires OCCT 8.0 or newer");
  }
#endif
  parameters.ReadSameParamMode = profile.same_parameter_repair;
  parameters.ReadSurfaceCurveMode = DESTEP_Parameters::ReadMode_SurfaceCurve_Default;
  parameters.ReadTessellated = profile.tessellated_geometry
                                   ? DESTEP_Parameters::RWMode_Tessellated_On
                                   : DESTEP_Parameters::RWMode_Tessellated_Off;
  parameters.WriteNonmanifold = profile.write_nonmanifold;
  parameters.ReadNonmanifold = profile.read_nonmanifold;
  return parameters;
}

occ::handle<TCollection_HAsciiString> header_string(const std::string_view value) {
  return new TCollection_HAsciiString(std::string(value).c_str());
}

void normalize_header(STEPControl_Writer& writer, const std::string_view fixture_id) {
  const occ::handle<StepData_StepModel> model = writer.Model();
  if (model.IsNull()) {
    throw GeneratorError("step-header", "STEP writer did not produce a model");
  }

  APIHeaderSection_MakeHeader header(model);
  if (!header.IsDone()) {
    throw GeneratorError("step-header", "STEP writer produced an incomplete header");
  }

  const std::string file_name = std::string(fixture_id) + ".step";
  header.SetName(header_string(file_name));
  header.SetTimeStamp(header_string(kFixedTimestamp));

#if OCC_VERSION_HEX >= 0x080000
  using HeaderString = occ::handle<TCollection_HAsciiString>;
  using HeaderStrings = NCollection_HArray1<HeaderString>;
#else
  using HeaderStrings = Interface_HArray1OfHAsciiString;
#endif
  const occ::handle<HeaderStrings> authors = new HeaderStrings(1, 1);
  authors->SetValue(1, header_string("CAD Mesher fixture factory"));
  header.SetAuthor(authors);

  const occ::handle<HeaderStrings> organizations = new HeaderStrings(1, 1);
  organizations->SetValue(1, header_string("CAD Mesher"));
  header.SetOrganization(organizations);

  header.SetPreprocessorVersion(header_string("CAD Mesher fixture generator 1.0.0"));
  header.SetOriginatingSystem(header_string("OCCT deterministic STEP writer"));
  header.SetAuthorisation(header_string("Generated test fixture"));
  header.Apply(model);

  int product_index = 0;
  std::vector<occ::handle<StepRepr_NextAssemblyUsageOccurrence>> assembly_usages;
  for (int entity_index = 1; entity_index <= model->NbEntities(); ++entity_index) {
    const occ::handle<StepBasic_Product> product =
        occ::down_cast<StepBasic_Product>(model->Value(entity_index));
    if (!product.IsNull()) {
      ++product_index;
      const std::string product_name =
          std::string(fixture_id) + ".product." + std::to_string(product_index);
      product->SetId(header_string(product_name));
      product->SetName(header_string(product_name));
    }

    const occ::handle<StepRepr_NextAssemblyUsageOccurrence> assembly_usage =
        occ::down_cast<StepRepr_NextAssemblyUsageOccurrence>(model->Value(entity_index));
    if (!assembly_usage.IsNull()) {
      assembly_usages.push_back(assembly_usage);
    }
  }
  if (product_index == 0) {
    throw GeneratorError("step-header", "STEP writer produced no PRODUCT entity");
  }

  std::set<int> assembly_usage_ids;
  for (const occ::handle<StepRepr_NextAssemblyUsageOccurrence>& assembly_usage : assembly_usages) {
    const occ::handle<TCollection_HAsciiString> id = assembly_usage->Id();
    if (id.IsNull() || !id->IsIntegerValue() || id->IntegerValue() <= 0 ||
        !assembly_usage_ids.insert(id->IntegerValue()).second) {
      throw GeneratorError("step-header",
                           "STEP writer produced an invalid assembly usage identifier");
    }
  }
  if (!assembly_usage_ids.empty()) {
    const int first_id = *assembly_usage_ids.begin();
    const int last_id = *assembly_usage_ids.rbegin();
    if (last_id - first_id + 1 != static_cast<int>(assembly_usage_ids.size())) {
      throw GeneratorError("step-header",
                           "STEP writer produced non-consecutive assembly usage identifiers");
    }
    for (const occ::handle<StepRepr_NextAssemblyUsageOccurrence>& assembly_usage :
         assembly_usages) {
      assembly_usage->SetId(
          header_string(std::to_string(assembly_usage->Id()->IntegerValue() - first_id + 1)));
    }
  }
}

void require_no_shape_processing(const XSAlgo_ShapeProcessor::ProcessingFlags& configured,
                                 const std::string_view stage) {
  if (!configured.second || configured.first.any()) {
    throw GeneratorError("shape-processing-config",
                         std::string(stage) + " did not retain explicit empty processing flags " +
                             "(explicit=" + (configured.second ? "true" : "false") +
                             ", any=" + (configured.first.any() ? "true" : "false") + ")");
  }
}

std::string label_entry(const TDF_Label& label) {
  TCollection_AsciiString entry;
  TDF_Tool::Entry(label, entry);
  return entry.ToCString();
}

using DefinitionMap = std::map<std::string, std::pair<std::string, std::string>>;
using ComponentMap = std::map<std::string, std::string>;

void collect_component_instances(const TDF_Label& definition,
                                 const std::optional<std::string>& parent_label,
                                 const gp_Trsf& parent_world, const DefinitionMap& definitions,
                                 const ComponentMap& component_labels,
                                 std::set<std::string>& active_paths, XdeSummary& summary) {
  const std::string definition_entry = label_entry(definition);
  if (!active_paths.insert(definition_entry).second) {
    throw GeneratorError("xde-cycle", "XDE assembly definition cycle at " + definition_entry);
  }

  NCollection_Sequence<TDF_Label> components;
  if (XCAFDoc_ShapeTool::GetComponents(definition, components, false)) {
    for (const TDF_Label& component : components) {
      if (!XCAFDoc_ShapeTool::IsComponent(component) &&
          !XCAFDoc_ShapeTool::IsReference(component)) {
        throw GeneratorError("xde-component", "XDE component label is not a reference");
      }
      TDF_Label referred;
      if (!XCAFDoc_ShapeTool::GetReferredShape(component, referred)) {
        throw GeneratorError("xde-component", "XDE component has no referred definition");
      }

      const auto target = definitions.find(label_entry(referred));
      if (target == definitions.end()) {
        throw GeneratorError("xde-component", "XDE component refers outside the definition set");
      }
      const auto component_label = component_labels.find(label_entry(component));
      if (component_label == component_labels.end()) {
        throw GeneratorError("xde-component", "XDE component has no stable label binding");
      }
      const gp_Trsf local = XCAFDoc_ShapeTool::GetLocation(component).Transformation();
      if (std::abs(std::abs(local.ScaleFactor()) - 1.0) > kLocationQuantum || local.IsNegative()) {
        throw GeneratorError("xde-transform",
                             "procedural component is not a proper rigid transform");
      }
      const gp_Trsf world = parent_world.Multiplied(local);
      XdeInstanceOccurrence instance;
      instance.label = occurrence_id("instance.i", summary.instances.size() + 1);
      instance.component_label = component_label->second;
      instance.parent_label = parent_label;
      instance.target_definition = target->second.first;
      instance.target_kind = target->second.second;
      instance.local_matrix_row_major = location_matrix(local);
      instance.matrix_row_major = location_matrix(world);
      instance.transform_kind =
          is_identity_matrix(instance.local_matrix_row_major) ? "identity" : "rigid";
      const std::string instance_label = instance.label;
      summary.instances.push_back(std::move(instance));
      if (XCAFDoc_ShapeTool::IsAssembly(referred)) {
        collect_component_instances(referred, instance_label, world, definitions, component_labels,
                                    active_paths, summary);
      }
    }
  }
  active_paths.erase(definition_entry);
}

Matrix4 external_transform_matrix(const gp_Trsf& transform) {
  Matrix4 matrix{};
  for (int row = 1; row <= 3; ++row) {
    for (int column = 1; column <= 4; ++column) {
      matrix[static_cast<std::size_t>((row - 1) * 4 + (column - 1))] = quantize_value(
          transform.Value(row, column), kLocationQuantum, "external XDE transform matrix");
    }
  }
  matrix[15] = 1.0;
  return matrix;
}

std::string external_transform_kind(const gp_Trsf& transform, const Matrix4& matrix) {
  if (is_identity_matrix(matrix)) {
    return "identity";
  }
  if (transform.Form() == gp_Other) {
    return "general_affine";
  }
  if (std::abs(std::abs(transform.ScaleFactor()) - 1.0) > kLocationQuantum) {
    return "scaled";
  }
  return transform.IsNegative() ? "mirrored" : "rigid";
}

void collect_external_component_instances(const TDF_Label& definition,
                                          const std::optional<std::string>& parent_label,
                                          const gp_Trsf& parent_world,
                                          const DefinitionMap& definitions,
                                          const ComponentMap& component_labels,
                                          std::set<std::string>& active_paths,
                                          XdeSummary& summary) {
  const std::string definition_entry = label_entry(definition);
  if (!active_paths.insert(definition_entry).second) {
    throw GeneratorError("xde-cycle", "XDE assembly definition cycle at " + definition_entry);
  }

  NCollection_Sequence<TDF_Label> components;
  if (XCAFDoc_ShapeTool::GetComponents(definition, components, false)) {
    for (const TDF_Label& component : components) {
      if (!XCAFDoc_ShapeTool::IsComponent(component) &&
          !XCAFDoc_ShapeTool::IsReference(component)) {
        throw GeneratorError("xde-component", "XDE component label is not a reference");
      }
      TDF_Label referred;
      if (!XCAFDoc_ShapeTool::GetReferredShape(component, referred)) {
        throw GeneratorError("xde-component", "XDE component has no referred definition");
      }

      const auto target = definitions.find(label_entry(referred));
      if (target == definitions.end()) {
        throw GeneratorError("xde-component", "XDE component refers outside the definition set");
      }
      const auto component_label = component_labels.find(label_entry(component));
      if (component_label == component_labels.end()) {
        throw GeneratorError("xde-component", "XDE component has no stable label binding");
      }

      const gp_Trsf local = XCAFDoc_ShapeTool::GetLocation(component).Transformation();
      const gp_Trsf world = parent_world.Multiplied(local);
      XdeInstanceOccurrence instance;
      instance.label = occurrence_id("instance.i", summary.instances.size() + 1);
      instance.component_label = component_label->second;
      instance.parent_label = parent_label;
      instance.target_definition = target->second.first;
      instance.target_kind = target->second.second;
      instance.local_matrix_row_major = external_transform_matrix(local);
      instance.matrix_row_major = external_transform_matrix(world);
      instance.transform_kind = external_transform_kind(local, instance.local_matrix_row_major);
      const std::string instance_label = instance.label;
      summary.instances.push_back(std::move(instance));
      if (XCAFDoc_ShapeTool::IsAssembly(referred)) {
        collect_external_component_instances(referred, instance_label, world, definitions,
                                             component_labels, active_paths, summary);
      }
    }
  }
  active_paths.erase(definition_entry);
}

StepReadback write_and_read_step(const Recipe& recipe, const TopoDS_Shape& source,
                                 const occ::handle<TDocStd_Document>& source_document,
                                 const std::filesystem::path& output_path) {
  if (recipe.evidence_lane != EvidenceLane::occt_procedural) {
    throw GeneratorError("recipe-lane", "STEP generator only accepts occt_procedural recipes");
  }
  if (recipe.step_io.shape_processing) {
    throw GeneratorError("shape-processing-config",
                         "procedural fixtures prohibit destructive shape processing");
  }
  const ShapeProcess::OperationsFlags no_shape_processing;
  const DESTEP_Parameters parameters = step_parameters(recipe.step_io);
  const std::string output_string = output_path.string();
  if (source_document.IsNull()) {
    STEPControl_Writer writer;
    writer.SetShapeProcessFlags(no_shape_processing);
    require_no_shape_processing(writer.GetShapeProcessFlags(), "STEP writer");
    if (writer.Transfer(source, STEPControl_AsIs, parameters, true) != IFSelect_RetDone) {
      throw GeneratorError("step-transfer", "STEP transfer failed for " + std::string(recipe.id));
    }
    normalize_header(writer, recipe.id);
    if (writer.Write(output_string.c_str()) != IFSelect_RetDone) {
      throw GeneratorError("step-write", "STEP write failed for " + std::string(recipe.id));
    }
  } else {
    STEPCAFControl_Writer writer;
    writer.SetColorMode(false);
    writer.SetLayerMode(false);
    writer.SetPropsMode(false);
    writer.SetNameMode(true);
#if OCC_VERSION_HEX >= 0x080000
    writer.SetCleanDuplicates(recipe.step_io.clean_duplicates);
#endif
    writer.SetShapeProcessFlags(no_shape_processing);
    writer.ChangeWriter().SetShapeProcessFlags(no_shape_processing);
    require_no_shape_processing(writer.GetShapeProcessFlags(), "STEP XDE writer");
    require_no_shape_processing(writer.ChangeWriter().GetShapeProcessFlags(),
                                "STEP XDE base writer");
    if (!writer.Transfer(source_document, parameters, STEPControl_AsIs)) {
      throw GeneratorError("step-transfer",
                           "STEP XDE transfer failed for " + std::string(recipe.id));
    }
    normalize_header(writer.ChangeWriter(), recipe.id);
    if (writer.Write(output_string.c_str()) != IFSelect_RetDone) {
      throw GeneratorError("step-write", "STEP XDE write failed for " + std::string(recipe.id));
    }
  }

  const occ::handle<TDocStd_Document> document =
      new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
  XCAFDoc_DocumentTool::Set(document->Main());

  STEPCAFControl_Reader reader;
  reader.SetColorMode(false);
  reader.SetLayerMode(false);
  reader.SetPropsMode(false);
  reader.SetNameMode(true);
  if (reader.ReadFile(output_string.c_str(), parameters) != IFSelect_RetDone) {
    throw GeneratorError("step-read", "STEP read failed for " + std::string(recipe.id));
  }
  reader.SetShapeProcessFlags(no_shape_processing);
  reader.ChangeReader().SetShapeProcessFlags(no_shape_processing);
  require_no_shape_processing(reader.GetShapeProcessFlags(), "STEP XDE reader");
  require_no_shape_processing(reader.ChangeReader().GetShapeProcessFlags(), "STEP XDE base reader");
  if (reader.NbRootsForTransfer() <= 0 || !reader.Transfer(document)) {
    throw GeneratorError("step-readback",
                         "STEP read-back transferred no roots for " + std::string(recipe.id));
  }

  const occ::handle<XCAFDoc_ShapeTool> shape_tool =
      XCAFDoc_DocumentTool::ShapeTool(document->Main());
  if (shape_tool.IsNull()) {
    throw GeneratorError("xde-transfer", "STEP read-back produced no XDE shape tool");
  }

  NCollection_Sequence<TDF_Label> definitions;
  NCollection_Sequence<TDF_Label> free_products;
  shape_tool->GetShapes(definitions);
  shape_tool->GetFreeShapes(free_products);
  if (free_products.Size() != 1) {
    throw GeneratorError("xde-roots", "procedural fixture must import as exactly one free product");
  }

  XdeSummary xde;
  if (!std::in_range<int>(definitions.Size()) || !std::in_range<int>(free_products.Size())) {
    throw GeneratorError("xde-count", "XDE product count exceeds supported integer range");
  }
  xde.product_definitions = static_cast<int>(definitions.Size());
  xde.free_products = static_cast<int>(free_products.Size());
  for (const TDF_Label& definition : definitions) {
    if (XCAFDoc_ShapeTool::IsAssembly(definition)) {
      ++xde.assembly_definitions;
    } else if (XCAFDoc_ShapeTool::IsSimpleShape(definition)) {
      ++xde.simple_shape_definitions;
    }
  }
  DefinitionMap definition_map;
  ComponentMap component_map;
  int definition_ordinal = 0;
  int component_ordinal = 0;
  for (const TDF_Label& definition : definitions) {
    ++definition_ordinal;
    std::string kind;
    if (XCAFDoc_ShapeTool::IsAssembly(definition)) {
      kind = "assembly";
    } else if (XCAFDoc_ShapeTool::IsSimpleShape(definition)) {
      kind = "simple_shape";
    } else {
      throw GeneratorError("xde-definition", "XDE definition is neither assembly nor simple shape");
    }
    definition_map.emplace(label_entry(definition),
                           std::pair{occurrence_id("definition.d", definition_ordinal), kind});
    NCollection_Sequence<TDF_Label> components;
    if (XCAFDoc_ShapeTool::GetComponents(definition, components, false)) {
      for (const TDF_Label& component : components) {
        if (component_ordinal == std::numeric_limits<int>::max()) {
          throw GeneratorError("xde-count", "XDE component-label count exceeds integer range");
        }
        ++component_ordinal;
        component_map.emplace(label_entry(component),
                              occurrence_id("component.c", component_ordinal));
      }
    }
  }
  xde.unique_component_labels = component_ordinal;
  for (const TDF_Label& free_product : free_products) {
    std::set<std::string> active_paths;
    collect_component_instances(free_product, std::nullopt, gp_Trsf(), definition_map,
                                component_map, active_paths, xde);
  }
  if (!std::in_range<int>(xde.instances.size())) {
    throw GeneratorError("xde-count", "expanded XDE instance count exceeds integer range");
  }
  xde.component_instances = static_cast<int>(xde.instances.size());

  const TopoDS_Shape roundtrip = XCAFDoc_ShapeTool::GetShape(free_products.First());
  if (roundtrip.IsNull()) {
    throw GeneratorError("step-readback",
                         "STEP read-back produced a null shape for " + std::string(recipe.id));
  }
  return StepReadback{roundtrip, xde};
}

std::vector<std::string> read_step_schema_identifiers(const STEPCAFControl_Reader& reader) {
  const occ::handle<StepData_StepModel> model = reader.Reader().StepModel();
  if (model.IsNull()) {
    throw GeneratorError("step-header", "STEP reader produced no model for header inspection");
  }

  APIHeaderSection_MakeHeader header(model);
  if (!header.HasFs() || header.NbSchemaIdentifiers() <= 0) {
    throw GeneratorError("step-header", "STEP input has no FILE_SCHEMA declaration");
  }

  std::vector<std::string> identifiers;
  identifiers.reserve(static_cast<std::size_t>(header.NbSchemaIdentifiers()));
  for (int index = 1; index <= header.NbSchemaIdentifiers(); ++index) {
    const occ::handle<TCollection_HAsciiString> identifier = header.SchemaIdentifiersValue(index);
    if (identifier.IsNull() || identifier->Length() == 0) {
      throw GeneratorError("step-header", "STEP input has an empty FILE_SCHEMA identifier");
    }
    identifiers.emplace_back(identifier->ToCString());
  }
  return identifiers;
}

StepReadback extract_external_xde_readback(const occ::handle<TDocStd_Document>& document) {
  const occ::handle<XCAFDoc_ShapeTool> shape_tool =
      XCAFDoc_DocumentTool::ShapeTool(document->Main());
  if (shape_tool.IsNull()) {
    throw GeneratorError("xde-transfer", "STEP input produced no XDE shape tool");
  }

  NCollection_Sequence<TDF_Label> definitions;
  NCollection_Sequence<TDF_Label> free_products;
  shape_tool->GetShapes(definitions);
  shape_tool->GetFreeShapes(free_products);
  if (free_products.Size() <= 0) {
    throw GeneratorError("xde-roots", "STEP input produced no free XDE product");
  }

  XdeSummary xde;
  if (!std::in_range<int>(definitions.Size()) || !std::in_range<int>(free_products.Size())) {
    throw GeneratorError("xde-count", "XDE product count exceeds supported integer range");
  }
  xde.product_definitions = static_cast<int>(definitions.Size());
  xde.free_products = static_cast<int>(free_products.Size());
  for (const TDF_Label& definition : definitions) {
    if (XCAFDoc_ShapeTool::IsAssembly(definition)) {
      ++xde.assembly_definitions;
    } else if (XCAFDoc_ShapeTool::IsSimpleShape(definition)) {
      ++xde.simple_shape_definitions;
    }
  }

  DefinitionMap definition_map;
  ComponentMap component_map;
  int definition_ordinal = 0;
  int component_ordinal = 0;
  for (const TDF_Label& definition : definitions) {
    ++definition_ordinal;
    std::string kind;
    if (XCAFDoc_ShapeTool::IsAssembly(definition)) {
      kind = "assembly";
    } else if (XCAFDoc_ShapeTool::IsSimpleShape(definition)) {
      kind = "simple_shape";
    } else {
      throw GeneratorError("xde-definition", "XDE definition is neither assembly nor simple shape");
    }
    definition_map.emplace(label_entry(definition),
                           std::pair{occurrence_id("definition.d", definition_ordinal), kind});

    NCollection_Sequence<TDF_Label> components;
    if (XCAFDoc_ShapeTool::GetComponents(definition, components, false)) {
      for (const TDF_Label& component : components) {
        if (component_ordinal == std::numeric_limits<int>::max()) {
          throw GeneratorError("xde-count", "XDE component-label count exceeds integer range");
        }
        ++component_ordinal;
        component_map.emplace(label_entry(component),
                              occurrence_id("component.c", component_ordinal));
      }
    }
  }
  xde.unique_component_labels = component_ordinal;
  for (const TDF_Label& free_product : free_products) {
    std::set<std::string> active_paths;
    collect_external_component_instances(free_product, std::nullopt, gp_Trsf(), definition_map,
                                         component_map, active_paths, xde);
  }
  if (!std::in_range<int>(xde.instances.size())) {
    throw GeneratorError("xde-count", "expanded XDE instance count exceeds integer range");
  }
  xde.component_instances = static_cast<int>(xde.instances.size());

  if (free_products.Size() == 1) {
    const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(free_products.First());
    if (shape.IsNull()) {
      throw GeneratorError("step-readback", "STEP input produced a null free-product shape");
    }
    return StepReadback{shape, std::move(xde)};
  }

  BRep_Builder builder;
  TopoDS_Compound compound;
  builder.MakeCompound(compound);
  for (const TDF_Label& free_product : free_products) {
    const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(free_product);
    if (shape.IsNull()) {
      throw GeneratorError("step-readback", "STEP input produced a null free-product shape");
    }
    builder.Add(compound, shape);
  }
  return StepReadback{compound, std::move(xde)};
}

ExternalStepInspection read_external_step(const std::filesystem::path& input_path) {
  const occ::handle<TDocStd_Document> document =
      new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
  XCAFDoc_DocumentTool::Set(document->Main());

  const StepIoProfile inspection_profile;
  const DESTEP_Parameters parameters = step_parameters(inspection_profile);
  const ShapeProcess::OperationsFlags no_shape_processing;
  STEPCAFControl_Reader reader;
  reader.SetColorMode(false);
  reader.SetLayerMode(false);
  reader.SetPropsMode(false);
  reader.SetNameMode(true);
  const std::string input_string = input_path.string();
  if (reader.ReadFile(input_string.c_str(), parameters) != IFSelect_RetDone) {
    throw GeneratorError("step-read", "STEP read failed for " + input_path.filename().string());
  }

  std::vector<std::string> schema_identifiers = read_step_schema_identifiers(reader);
  reader.SetShapeProcessFlags(no_shape_processing);
  reader.ChangeReader().SetShapeProcessFlags(no_shape_processing);
  require_no_shape_processing(reader.GetShapeProcessFlags(), "STEP XDE reader");
  require_no_shape_processing(reader.ChangeReader().GetShapeProcessFlags(), "STEP XDE base reader");
  const int transfer_roots = reader.NbRootsForTransfer();
  if (transfer_roots <= 0 || !reader.Transfer(document)) {
    throw GeneratorError("step-readback",
                         "STEP input transferred no roots for " + input_path.filename().string());
  }

  return ExternalStepInspection{transfer_roots, std::move(schema_identifiers),
                                extract_external_xde_readback(document)};
}

std::string json_escape(const std::string_view value) {
  std::ostringstream stream;
  for (const unsigned char character : value) {
    switch (character) {
    case '\"':
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
      if (character < 0x20U) {
        stream << "\\u" << std::hex << std::setw(4) << std::setfill('0')
               << static_cast<unsigned int>(character) << std::dec;
      } else {
        stream << static_cast<char>(character);
      }
    }
  }
  return stream.str();
}

void write_integer_map(std::ostream& stream, const std::map<std::string, int>& values,
                       const int indentation) {
  stream << "{";
  if (!values.empty()) {
    stream << '\n';
    std::size_t index = 0;
    for (const auto& [name, count] : values) {
      stream << std::string(static_cast<std::size_t>(indentation + 2), ' ') << "\""
             << json_escape(name) << "\": " << count;
      if (++index != values.size()) {
        stream << ',';
      }
      stream << '\n';
    }
    stream << std::string(static_cast<std::size_t>(indentation), ' ');
  }
  stream << '}';
}

std::string json_number(const double value) {
  if (!std::isfinite(value)) {
    throw GeneratorError("metadata-write", "cannot serialize a non-finite JSON number");
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10)
         << value;
  return stream.str();
}

std::string json_quantized_uv_number(const double value) {
  if (!std::isfinite(value)) {
    throw GeneratorError("metadata-write", "cannot serialize a non-finite UV bound");
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::fixed << std::setprecision(9) << value;
  return stream.str();
}

void write_matrix(std::ostream& stream, const Matrix4& matrix) {
  stream << '[';
  for (std::size_t index = 0; index < matrix.size(); ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << json_number(matrix[index]);
  }
  stream << ']';
}

void write_tolerance_range(std::ostream& stream, const ToleranceRange& range) {
  stream << "{\"min\": ";
  if (range.minimum_mm.has_value()) {
    stream << json_number(*range.minimum_mm);
  } else {
    stream << "null";
  }
  stream << ", \"max\": ";
  if (range.maximum_mm.has_value()) {
    stream << json_number(*range.maximum_mm);
  } else {
    stream << "null";
  }
  stream << '}';
}

void write_edge_occurrences(std::ostream& stream, const std::vector<EdgeOccurrence>& occurrences,
                            const int indentation) {
  stream << "[";
  if (!occurrences.empty()) {
    stream << '\n';
    for (std::size_t index = 0; index < occurrences.size(); ++index) {
      const EdgeOccurrence& occurrence = occurrences[index];
      stream << std::string(static_cast<std::size_t>(indentation + 2), ' ') << "{\"id\": \""
             << json_escape(occurrence.id) << "\", \"family\": \"" << json_escape(occurrence.family)
             << "\", \"concrete_type\": \"" << json_escape(occurrence.concrete_type)
             << "\", \"orientation\": \"" << json_escape(occurrence.orientation)
             << "\", \"partner_ordinal\": " << occurrence.partner_ordinal
             << ", \"degenerated\": " << (occurrence.degenerated ? "true" : "false")
             << ", \"missing_3d_curve\": " << (occurrence.missing_3d_curve ? "true" : "false")
             << ", \"periodic_seam\": " << (occurrence.periodic_seam ? "true" : "false")
             << ", \"face_use_count\": " << occurrence.face_use_count << ", \"shell_role\": \""
             << json_escape(occurrence.shell_role) << "\""
             << ", \"geometry_location_matrix_row_major\": ";
      write_matrix(stream, occurrence.geometry_location_matrix_row_major);
      stream << ", \"occurrence_location_matrix_row_major\": ";
      write_matrix(stream, occurrence.occurrence_location_matrix_row_major);
      stream << '}';
      if (index + 1 != occurrences.size()) {
        stream << ',';
      }
      stream << '\n';
    }
    stream << std::string(static_cast<std::size_t>(indentation), ' ');
  }
  stream << ']';
}

void write_face_occurrences(std::ostream& stream, const std::vector<FaceOccurrence>& occurrences,
                            const int indentation) {
  stream << "[";
  if (!occurrences.empty()) {
    stream << '\n';
    for (std::size_t index = 0; index < occurrences.size(); ++index) {
      const FaceOccurrence& occurrence = occurrences[index];
      stream << std::string(static_cast<std::size_t>(indentation + 2), ' ') << "{\"id\": \""
             << json_escape(occurrence.id) << "\", \"family\": \"" << json_escape(occurrence.family)
             << "\", \"concrete_type\": \"" << json_escape(occurrence.concrete_type)
             << "\", \"orientation\": \"" << json_escape(occurrence.orientation)
             << "\", \"partner_ordinal\": " << occurrence.partner_ordinal
             << ", \"solid_use_count\": " << occurrence.solid_use_count << ", \"shell_role\": \""
             << json_escape(occurrence.shell_role)
             << "\", \"geometry_location_matrix_row_major\": ";
      write_matrix(stream, occurrence.geometry_location_matrix_row_major);
      stream << ", \"occurrence_location_matrix_row_major\": ";
      write_matrix(stream, occurrence.occurrence_location_matrix_row_major);
      stream << '}';
      if (index + 1 != occurrences.size()) {
        stream << ',';
      }
      stream << '\n';
    }
    stream << std::string(static_cast<std::size_t>(indentation), ' ');
  }
  stream << ']';
}

void write_boundary_loops(std::ostream& stream, const std::vector<BoundaryLoop>& loops,
                          const int indentation) {
  stream << "[";
  if (!loops.empty()) {
    stream << '\n';
    for (std::size_t index = 0; index < loops.size(); ++index) {
      const BoundaryLoop& loop = loops[index];
      stream << std::string(static_cast<std::size_t>(indentation + 2), ' ') << "{\"id\": \""
             << json_escape(loop.id) << "\", \"face_occurrence_id\": \""
             << json_escape(loop.face_occurrence_id) << "\", \"role\": \"" << json_escape(loop.role)
             << "\", \"orientation\": \"" << json_escape(loop.orientation)
             << "\", \"closed\": " << (loop.closed ? "true" : "false")
             << ", \"edge_occurrence_count\": " << loop.edge_occurrence_count
             << ", \"uv_bounds\": {\"u_min\": " << json_quantized_uv_number(loop.u_min)
             << ", \"u_max\": " << json_quantized_uv_number(loop.u_max)
             << ", \"v_min\": " << json_quantized_uv_number(loop.v_min)
             << ", \"v_max\": " << json_quantized_uv_number(loop.v_max) << "}}";
      if (index + 1 != loops.size()) {
        stream << ',';
      }
      stream << '\n';
    }
    stream << std::string(static_cast<std::size_t>(indentation), ' ');
  }
  stream << ']';
}

void write_logical_region_proxies(std::ostream& stream,
                                  const std::vector<LogicalRegionProxy>& proxies,
                                  const int indentation) {
  stream << "[";
  if (!proxies.empty()) {
    stream << '\n';
    for (std::size_t index = 0; index < proxies.size(); ++index) {
      const LogicalRegionProxy& proxy = proxies[index];
      stream << std::string(static_cast<std::size_t>(indentation + 2), ' ') << "{\"id\": \""
             << json_escape(proxy.id) << "\", \"face_occurrence_id\": \""
             << json_escape(proxy.face_occurrence_id) << "\", \"family\": \""
             << json_escape(proxy.family) << "\", \"concrete_type\": \""
             << json_escape(proxy.concrete_type) << "\"}";
      if (index + 1 != proxies.size()) {
        stream << ',';
      }
      stream << '\n';
    }
    stream << std::string(static_cast<std::size_t>(indentation), ' ');
  }
  stream << ']';
}

void write_shape_summary(std::ostream& stream, const ShapeSummary& summary, const int indentation) {
  const std::string base(static_cast<std::size_t>(indentation), ' ');
  const std::string nested(static_cast<std::size_t>(indentation + 2), ' ');
  stream << "{\n";
  stream << nested << "\"valid\": " << (summary.valid ? "true" : "false") << ",\n";
  stream << nested << "\"root_shape_type\": \"" << json_escape(summary.root_shape_type) << "\",\n";
  stream << nested << "\"topology\": {\n";
  for (std::size_t index = 0; index < summary.topology.size(); ++index) {
    const TopologyCount& count = summary.topology[index];
    stream << std::string(static_cast<std::size_t>(indentation + 4), ' ') << "\"" << count.family
           << "\": {\"unique\": " << count.unique << ", \"occurrences\": " << count.occurrences
           << ", \"orientations\": {\"FORWARD\": " << count.orientations.forward
           << ", \"REVERSED\": " << count.orientations.reversed
           << ", \"INTERNAL\": " << count.orientations.internal
           << ", \"EXTERNAL\": " << count.orientations.external << "}}";
    if (index + 1 != summary.topology.size()) {
      stream << ',';
    }
    stream << '\n';
  }
  stream << nested << "},\n";
  stream << nested << "\"edge_occurrences\": ";
  write_edge_occurrences(stream, summary.edge_occurrences, indentation + 2);
  stream << ",\n";
  stream << nested << "\"face_occurrences\": ";
  write_face_occurrences(stream, summary.face_occurrences, indentation + 2);
  stream << ",\n";
  stream << nested << "\"boundary_loops\": ";
  write_boundary_loops(stream, summary.boundary_loops, indentation + 2);
  stream << ",\n";
  stream << nested << "\"logical_region_proxies\": ";
  write_logical_region_proxies(stream, summary.logical_region_proxies, indentation + 2);
  stream << ",\n";
  stream << nested
         << "\"model_topology\": {\"closed_shells\": " << summary.model_topology.closed_shells
         << ", \"open_shells\": " << summary.model_topology.open_shells
         << ", \"outer_shells\": " << summary.model_topology.outer_shells
         << ", \"inner_shells\": " << summary.model_topology.inner_shells
         << ", \"standalone_shells\": " << summary.model_topology.standalone_shells
         << ", \"shared_faces_between_solids\": "
         << summary.model_topology.shared_faces_between_solids
         << ", \"maximum_faces_per_edge\": " << summary.model_topology.maximum_faces_per_edge
         << ", \"nonmanifold_vertices\": " << summary.model_topology.nonmanifold_vertices
         << ", \"free_wires\": " << summary.model_topology.free_wires
         << ", \"free_edges\": " << summary.model_topology.free_edges
         << ", \"isolated_vertices\": " << summary.model_topology.isolated_vertices
         << ", \"coincident_solid_signature_pairs\": "
         << summary.model_topology.coincident_solid_signature_pairs << "},\n";
  stream << nested << "\"curve_families\": ";
  write_integer_map(stream, summary.curve_families, indentation + 2);
  stream << ",\n";
  stream << nested << "\"curve_concrete_types\": ";
  write_integer_map(stream, summary.curve_concrete_types, indentation + 2);
  stream << ",\n";
  stream << nested << "\"surface_families\": ";
  write_integer_map(stream, summary.surface_families, indentation + 2);
  stream << ",\n";
  stream << nested << "\"surface_concrete_types\": ";
  write_integer_map(stream, summary.surface_concrete_types, indentation + 2);
  stream << ",\n";
  stream << nested << "\"edge_conditions\": {\"degenerated\": " << summary.degenerated_edges
         << ", \"missing_3d_curve\": " << summary.edges_without_3d_curve
         << ", \"periodic_seam\": " << summary.periodic_seam_edges
         << ", \"free_boundary\": " << summary.free_boundary_edges
         << ", \"nonmanifold\": " << summary.nonmanifold_edges << "},\n";
  stream << nested << "\"tolerance_ranges_mm\": {\n";
  stream << std::string(static_cast<std::size_t>(indentation + 4), ' ') << "\"vertex\": ";
  write_tolerance_range(stream, summary.vertex_tolerance);
  stream << ",\n";
  stream << std::string(static_cast<std::size_t>(indentation + 4), ' ') << "\"edge\": ";
  write_tolerance_range(stream, summary.edge_tolerance);
  stream << ",\n";
  stream << std::string(static_cast<std::size_t>(indentation + 4), ' ') << "\"face\": ";
  write_tolerance_range(stream, summary.face_tolerance);
  stream << '\n' << nested << "}\n";
  stream << base << '}';
}

void write_construction_parameters(std::ostream& stream,
                                   const std::vector<ConstructionParameter>& parameters) {
  stream << "[\n";
  for (std::size_t index = 0; index < parameters.size(); ++index) {
    const ConstructionParameter& parameter_value = parameters[index];
    stream << "      {\"name\": \"" << json_escape(parameter_value.name) << "\", \"unit\": \""
           << json_escape(parameter_value.unit)
           << "\", \"value\": " << json_number(parameter_value.value) << "}";
    if (index + 1 != parameters.size()) {
      stream << ',';
    }
    stream << '\n';
  }
  stream << "    ]";
}

void write_xde_instances(std::ostream& stream,
                         const std::vector<XdeInstanceOccurrence>& instances) {
  stream << '[';
  if (!instances.empty()) {
    stream << '\n';
    for (std::size_t index = 0; index < instances.size(); ++index) {
      const XdeInstanceOccurrence& instance = instances[index];
      stream << "      {\"label\": \"" << json_escape(instance.label)
             << "\", \"component_label\": \"" << json_escape(instance.component_label)
             << "\", \"parent_label\": ";
      if (instance.parent_label.has_value()) {
        stream << '"' << json_escape(*instance.parent_label) << '"';
      } else {
        stream << "null";
      }
      stream << ", \"target_definition\": \"" << json_escape(instance.target_definition)
             << "\", \"target_kind\": \"" << json_escape(instance.target_kind)
             << "\", \"transform_kind\": \"" << json_escape(instance.transform_kind)
             << "\", \"local_matrix_row_major\": ";
      write_matrix(stream, instance.local_matrix_row_major);
      stream << ", \"matrix_row_major\": ";
      write_matrix(stream, instance.matrix_row_major);
      stream << '}';
      if (index + 1 != instances.size()) {
        stream << ',';
      }
      stream << '\n';
    }
    stream << "    ";
  }
  stream << ']';
}

void write_metadata(const Recipe& recipe, const ShapeSummary& source,
                    const FixtureEvidence& source_evidence, const ShapeSummary& roundtrip,
                    const FixtureEvidence& roundtrip_evidence, const XdeSummary& xde,
                    const std::filesystem::path& output_path) {
  std::ofstream stream(output_path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw GeneratorError("metadata-write",
                         "could not open metadata output: " + output_path.string());
  }
  stream.imbue(std::locale::classic());

  stream << "{\n";
  stream << "  \"schema_version\": \"" << kMetadataSchemaVersion << "\",\n";
  stream << "  \"fixture_id\": \"" << json_escape(recipe.id) << "\",\n";
  stream << "  \"generator\": {\n";
  stream << "    \"version\": \"" << kGeneratorVersion << "\",\n";
  stream << "    \"evidence_lane\": \"" << evidence_lane_name(recipe.evidence_lane) << "\",\n";
  stream << "    \"occt_version\": \"" << OCC_VERSION_STRING_EXT << "\",\n";
  stream << "    \"step_schema\": \"" << step_schema_name(recipe.step_io.schema) << "\",\n";
  stream << "    \"writer\": \""
         << (recipe.document_factory ? "STEPCAFControl_Writer" : "STEPControl_Writer") << "\",\n";
  stream << "    \"length_unit\": \"millimetre\",\n";
  stream << "    \"tessellated\": " << (recipe.step_io.tessellated_geometry ? "true" : "false")
         << ",\n";
  stream << "    \"surface_curve_mode\": " << (recipe.step_io.surface_curve_mode ? "true" : "false")
         << ",\n";
  stream << "    \"shape_processing\": "
         << (recipe.step_io.shape_processing ? "[\"recipe_declared\"]" : "[]") << ",\n";
  stream << "    \"read_same_parameter_repair\": "
         << (recipe.step_io.same_parameter_repair ? "true" : "false") << ",\n";
  stream << "    \"read_tessellated\": " << (recipe.step_io.tessellated_geometry ? "true" : "false")
         << ",\n";
  stream << "    \"read_surface_curve_mode\": \"default\",\n";
  stream << "    \"clean_duplicates\": " << (recipe.step_io.clean_duplicates ? "true" : "false")
         << ",\n";
  stream << "    \"write_nonmanifold\": " << (recipe.step_io.write_nonmanifold ? "true" : "false")
         << ",\n";
  stream << "    \"read_nonmanifold\": " << (recipe.step_io.read_nonmanifold ? "true" : "false")
         << ",\n";
  stream << "    \"roundtrip_contract\": {\n";
  stream << "      \"boundary_loops\": " << (recipe.roundtrip.boundary_loops ? "true" : "false")
         << ",\n";
  stream << "      \"concrete_types\": " << (recipe.roundtrip.concrete_types ? "true" : "false")
         << ",\n";
  stream << "      \"expected_brep_valid\": "
         << (recipe.roundtrip.expected_brep_valid ? "true" : "false") << ",\n";
  stream << "      \"geometry_families\": "
         << (recipe.roundtrip.geometry_families ? "true" : "false") << ",\n";
  stream << "      \"model_topology\": " << (recipe.roundtrip.model_topology ? "true" : "false")
         << ",\n";
  stream << "      \"occurrence_graph\": " << (recipe.roundtrip.occurrence_graph ? "true" : "false")
         << ",\n";
  stream << "      \"tolerances\": " << (recipe.roundtrip.tolerances ? "true" : "false") << ",\n";
  stream << "      \"topology_counts\": " << (recipe.roundtrip.topology_counts ? "true" : "false")
         << ",\n";
  stream << "      \"accepted_normalization_codes\": [";
  for (std::size_t index = 0; index < recipe.roundtrip.accepted_normalization_codes.size();
       ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << "\"" << json_escape(recipe.roundtrip.accepted_normalization_codes[index]) << "\"";
  }
  stream << "]\n";
  stream << "    },\n";
  stream << "    \"boundary_loop_uv_quantum\": " << json_number(kBoundaryLoopUvQuantum) << ",\n";
  stream << "    \"tolerance_quantum_mm\": " << json_number(kToleranceQuantumMm) << ",\n";
  stream << "    \"location_quantum\": " << json_number(kLocationQuantum) << ",\n";
  stream << "    \"coincidence_quantum_mm\": " << json_number(kCoincidenceQuantumMm) << ",\n";
  stream << "    \"construction_parameters\": ";
  write_construction_parameters(stream, recipe.parameters);
  stream << "\n";
  stream << "  },\n";
  stream << "  \"xde\": {\n";
  stream << "    \"reader\": \"STEPCAFControl_Reader\",\n";
  stream << "    \"product_definitions\": " << xde.product_definitions << ",\n";
  stream << "    \"free_products\": " << xde.free_products << ",\n";
  stream << "    \"simple_shape_definitions\": " << xde.simple_shape_definitions << ",\n";
  stream << "    \"assembly_definitions\": " << xde.assembly_definitions << ",\n";
  stream << "    \"unique_component_labels\": " << xde.unique_component_labels << ",\n";
  stream << "    \"component_instances\": " << xde.component_instances << ",\n";
  stream << "    \"instances\": ";
  write_xde_instances(stream, xde.instances);
  stream << "\n";
  stream << "  },\n";
  stream << "  \"observations\": {\n";
  stream << "    \"roundtrip\": ";
  write_fixture_evidence_json(stream, roundtrip_evidence, 4);
  stream << ",\n";
  stream << "    \"source\": ";
  write_fixture_evidence_json(stream, source_evidence, 4);
  stream << "\n";
  stream << "  },\n";
  stream << "  \"source\": ";
  write_shape_summary(stream, source, 2);
  stream << ",\n";
  stream << "  \"roundtrip\": ";
  write_shape_summary(stream, roundtrip, 2);
  stream << "\n}\n";

  if (!stream) {
    throw GeneratorError("metadata-write",
                         "failed while writing metadata: " + output_path.string());
  }
}

void write_external_import_summary(const std::filesystem::path& input_path,
                                   const ExternalStepInspection& inspection,
                                   const ShapeSummary& shape, const FixtureEvidence& observations,
                                   const std::filesystem::path& output_path) {
  std::ofstream stream(output_path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw GeneratorError("metadata-write",
                         "could not open inspection summary output: " + output_path.string());
  }
  stream.imbue(std::locale::classic());

  stream << "{\n";
  stream << "  \"schema_version\": \"" << kMetadataSchemaVersion << "\",\n";
  stream << "  \"evidence\": {\n";
  stream << "    \"artifact_role\": \"external_or_derived_step_input\",\n";
  stream << "    \"evidence_class\": \"generator_observation\",\n";
  stream << "    \"independent_oracle\": false,\n";
  stream << "    \"description\": \"Read-only generator evidence; not an independent oracle.\"\n";
  stream << "  },\n";
  stream << "  \"input\": {\n";
  stream << "    \"file_name\": \"" << json_escape(input_path.filename().string()) << "\",\n";
  stream << "    \"step_declaration\": {\n";
  stream << "      \"entity\": \"FILE_SCHEMA\",\n";
  stream << "      \"schema_identifiers\": [";
  for (std::size_t index = 0; index < inspection.schema_identifiers.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << "\"" << json_escape(inspection.schema_identifiers[index]) << "\"";
  }
  stream << "]\n";
  stream << "    }\n";
  stream << "  },\n";
  stream << "  \"generator\": {\n";
  stream << "    \"version\": \"" << kGeneratorVersion << "\",\n";
  stream << "    \"occt_version\": \"" << OCC_VERSION_STRING_EXT << "\",\n";
  stream << "    \"boundary_loop_uv_quantum\": " << json_number(kBoundaryLoopUvQuantum) << ",\n";
  stream << "    \"tolerance_quantum_mm\": " << json_number(kToleranceQuantumMm) << ",\n";
  stream << "    \"location_quantum\": " << json_number(kLocationQuantum) << ",\n";
  stream << "    \"coincidence_quantum_mm\": " << json_number(kCoincidenceQuantumMm) << "\n";
  stream << "  },\n";
  stream << "  \"reader\": {\n";
  stream << "    \"implementation\": \"STEPCAFControl_Reader\",\n";
  stream << "    \"input_access\": \"read_only\",\n";
  stream << "    \"transfer_boundary\": \"TDocStd_Document/XCAFDoc_ShapeTool\",\n";
  stream << "    \"transfer_root_count\": " << inspection.transfer_roots << ",\n";
  stream << "    \"modes\": {\n";
  stream << "      \"color\": false,\n";
  stream << "      \"layer\": false,\n";
  stream << "      \"properties\": false,\n";
  stream << "      \"name\": true\n";
  stream << "    },\n";
  stream << "    \"read_parameters\": {\n";
  stream << "      \"same_parameter_repair\": false,\n";
  stream << "      \"tessellated\": false,\n";
  stream << "      \"surface_curve_mode\": \"default\",\n";
  stream << "      \"nonmanifold\": false\n";
  stream << "    },\n";
  stream << "    \"shape_processing_flags\": {\n";
  stream << "      \"xde_reader\": [],\n";
  stream << "      \"base_reader\": []\n";
  stream << "    }\n";
  stream << "  },\n";
  stream << "  \"xde\": {\n";
  stream << "    \"reader\": \"STEPCAFControl_Reader\",\n";
  stream << "    \"product_definitions\": " << inspection.readback.xde.product_definitions << ",\n";
  stream << "    \"free_products\": " << inspection.readback.xde.free_products << ",\n";
  stream << "    \"simple_shape_definitions\": " << inspection.readback.xde.simple_shape_definitions
         << ",\n";
  stream << "    \"assembly_definitions\": " << inspection.readback.xde.assembly_definitions
         << ",\n";
  stream << "    \"unique_component_labels\": " << inspection.readback.xde.unique_component_labels
         << ",\n";
  stream << "    \"component_instances\": " << inspection.readback.xde.component_instances << ",\n";
  stream << "    \"instances\": ";
  write_xde_instances(stream, inspection.readback.xde.instances);
  stream << "\n";
  stream << "  },\n";
  stream << "  \"observations\": ";
  write_fixture_evidence_json(stream, observations, 2);
  stream << ",\n";
  stream << "  \"shape\": ";
  write_shape_summary(stream, shape, 2);
  stream << "\n}\n";

  if (!stream) {
    throw GeneratorError("metadata-write",
                         "failed while writing inspection summary: " + output_path.string());
  }
}

bool has_step_extension(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  for (char& character : extension) {
    if (character >= 'A' && character <= 'Z') {
      character = static_cast<char>(character - 'A' + 'a');
    }
  }
  return extension == ".step" || extension == ".stp";
}

bool has_json_extension(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  for (char& character : extension) {
    if (character >= 'A' && character <= 'Z') {
      character = static_cast<char>(character - 'A' + 'a');
    }
  }
  return extension == ".json";
}

OutputPaths output_paths(const std::string_view fixture_id, const std::filesystem::path& output) {
  if (has_step_extension(output)) {
    std::filesystem::path metadata = output;
    metadata.replace_extension(".json");
    std::filesystem::path native_brep = output;
    native_brep.replace_extension(".brep");
    return OutputPaths{output, metadata, native_brep};
  }

  const std::string stem(fixture_id);
  return OutputPaths{output / (stem + ".step"), output / (stem + ".json"),
                     output / (stem + ".brep")};
}

void prepare_output_directory(const OutputPaths& paths) {
  const std::filesystem::path parent = paths.step.parent_path();
  if (parent.empty()) {
    return;
  }
  std::error_code error;
  std::filesystem::create_directories(parent, error);
  if (error) {
    throw GeneratorError("filesystem", "could not create output directory " + parent.string() +
                                           ": " + error.message());
  }
}

void prepare_summary_directory(const std::filesystem::path& output_path) {
  const std::filesystem::path parent = output_path.parent_path();
  if (parent.empty()) {
    return;
  }
  std::error_code error;
  std::filesystem::create_directories(parent, error);
  if (error) {
    throw GeneratorError("filesystem", "could not create output directory " + parent.string() +
                                           ": " + error.message());
  }
}

void validate_inspection_paths(const std::filesystem::path& input,
                               const std::filesystem::path& output) {
  if (input.empty()) {
    throw GeneratorError("arguments", "--inspect-step requires a non-empty input path");
  }
  if (!has_step_extension(input)) {
    throw GeneratorError("arguments", "--inspect-step input must have a .step or .stp extension");
  }
  if (output.empty() || !has_json_extension(output)) {
    throw GeneratorError("arguments", "--inspect-step output must be a .json file");
  }

  std::error_code error;
  const bool input_exists = std::filesystem::exists(input, error);
  if (error) {
    throw GeneratorError("filesystem",
                         "could not inspect STEP input " + input.string() + ": " + error.message());
  }
  if (!input_exists) {
    throw GeneratorError("step-read", "STEP input does not exist: " + input.string());
  }
  const bool input_is_file = std::filesystem::is_regular_file(input, error);
  if (error) {
    throw GeneratorError("filesystem",
                         "could not inspect STEP input " + input.string() + ": " + error.message());
  }
  if (!input_is_file) {
    throw GeneratorError("step-read", "STEP input is not a regular file: " + input.string());
  }

  const bool output_exists = std::filesystem::exists(output, error);
  if (error) {
    throw GeneratorError("filesystem", "could not inspect summary output " + output.string() +
                                           ": " + error.message());
  }
  if (output_exists) {
    const bool output_is_directory = std::filesystem::is_directory(output, error);
    if (error) {
      throw GeneratorError("filesystem", "could not inspect summary output " + output.string() +
                                             ": " + error.message());
    }
    if (output_is_directory) {
      throw GeneratorError("arguments", "--inspect-step output must be a .json file");
    }
    const bool same_file = std::filesystem::equivalent(input, output, error);
    if (error) {
      throw GeneratorError("filesystem", "could not compare inspection paths: " + error.message());
    }
    if (same_file) {
      throw GeneratorError("arguments", "inspection output must not overwrite the STEP input");
    }
  }
}

void inspect_external_step_file(const std::filesystem::path& input,
                                const std::filesystem::path& output) {
  validate_inspection_paths(input, output);
  const ExternalStepInspection inspection = read_external_step(input);
  const ShapeSummary shape = summarize_shape(inspection.readback.shape);
  const FixtureEvidence observations = observe_fixture_evidence(inspection.readback.shape);
  prepare_summary_directory(output);
  write_external_import_summary(input, inspection, shape, observations, output);
}

void write_source_native_brep(const TopoDS_Shape& source,
                              const std::filesystem::path& output_path) {
  std::ofstream stream(output_path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw GeneratorError("brep-write",
                         "could not open native BRep output: " + output_path.string());
  }
  stream.imbue(std::locale::classic());
  BRepTools::Write(source, stream, false, false, TopTools_FormatVersion_VERSION_3);
  if (!stream) {
    throw GeneratorError("brep-write", "failed while writing native BRep: " + output_path.string());
  }
}

void generate(const Recipe& recipe, const std::filesystem::path& output) {
  const OutputPaths paths = output_paths(recipe.id, output);
  prepare_output_directory(paths);

  const bool has_shape_factory = static_cast<bool>(recipe.shape_factory);
  const bool has_document_factory = static_cast<bool>(recipe.document_factory);
  if (has_shape_factory == has_document_factory) {
    throw GeneratorError("recipe", "recipe must select exactly one source factory");
  }

  occ::handle<TDocStd_Document> source_document;
  TopoDS_Shape source;
  if (has_document_factory) {
    source_document = recipe.document_factory(recipe.parameters);
    if (source_document.IsNull()) {
      throw GeneratorError("construction", "XDE source factory returned a null document");
    }
    const occ::handle<XCAFDoc_ShapeTool> source_tool =
        XCAFDoc_DocumentTool::ShapeTool(source_document->Main());
    if (source_tool.IsNull()) {
      throw GeneratorError("construction", "XDE source document has no shape tool");
    }
    NCollection_Sequence<TDF_Label> source_roots;
    source_tool->GetFreeShapes(source_roots);
    if (source_roots.Size() != 1) {
      throw GeneratorError("construction", "XDE source must have exactly one free product");
    }
    source = XCAFDoc_ShapeTool::GetShape(source_roots.First());
  } else {
    source = recipe.shape_factory(recipe.parameters);
  }
  if (source.IsNull()) {
    throw GeneratorError("construction", "source factory produced a null shape");
  }
  const ShapeSummary source_summary = summarize_shape(source);
  const FixtureEvidence source_evidence = observe_fixture_evidence(source);
  if (!source_summary.valid) {
    throw GeneratorError("source-invalid", "source B-rep is invalid for " + std::string(recipe.id));
  }

  write_source_native_brep(source, paths.native_brep);
  const StepReadback readback = write_and_read_step(recipe, source, source_document, paths.step);
  const ShapeSummary roundtrip_summary = summarize_shape(readback.shape);
  if (roundtrip_summary.valid != recipe.roundtrip.expected_brep_valid) {
    throw GeneratorError("roundtrip-validity",
                         "round-trip B-rep validity changed for " + std::string(recipe.id));
  }
  const FixtureEvidence roundtrip_evidence = observe_fixture_evidence(readback.shape);
  ShapeSummary expected_structure = source_summary;
  expected_structure.valid = recipe.roundtrip.expected_brep_valid;
  if (!structurally_equal(roundtrip_summary, expected_structure, recipe.roundtrip)) {
    throw GeneratorError(
        "roundtrip-mismatch",
        "round-trip structure changed at " +
            first_structural_difference(roundtrip_summary, expected_structure, recipe.roundtrip) +
            " for " + std::string(recipe.id));
  }

  write_metadata(recipe, source_summary, source_evidence, roundtrip_summary, roundtrip_evidence,
                 readback.xde, paths.metadata);
}

} // namespace

const std::vector<std::string_view>& fixture_ids() {
  static const std::vector<std::string_view> ids = [] {
    std::vector<std::string_view> result;
    result.reserve(recipes().size());
    for (const Recipe& recipe : recipes()) {
      result.push_back(recipe.id);
    }
    return result;
  }();
  return ids;
}

bool generate_fixture(const std::string_view fixture_id, const std::filesystem::path& output,
                      GenerationFailure& failure) {
  const Recipe* const recipe = find_recipe(fixture_id);
  if (recipe == nullptr) {
    failure =
        GenerationFailure{"unknown-fixture", "unknown fixture ID: " + std::string(fixture_id)};
    return false;
  }

  try {
    generate(*recipe, output);
    return true;
  } catch (const GeneratorError& exception) {
    failure = GenerationFailure{exception.code(), exception.what()};
  } catch (const Standard_Failure& exception) {
    failure = GenerationFailure{"occt", weft::occtFailureMessage(exception)};
  } catch (const std::filesystem::filesystem_error& exception) {
    failure = GenerationFailure{"filesystem", exception.what()};
  } catch (const std::exception& exception) {
    failure = GenerationFailure{"generation", exception.what()};
  }
  return false;
}

bool generate_all(const std::filesystem::path& output_directory, GenerationFailure& failure) {
  if (has_step_extension(output_directory)) {
    failure = GenerationFailure{"arguments", "--all requires an output directory"};
    return false;
  }

  for (const Recipe& recipe : recipes()) {
    if (!generate_fixture(recipe.id, output_directory, failure)) {
      failure.message = std::string(recipe.id) + ": " + failure.message;
      return false;
    }
  }
  return true;
}

bool inspect_step(const std::filesystem::path& input, const std::filesystem::path& output,
                  GenerationFailure& failure) {
  try {
    inspect_external_step_file(input, output);
    return true;
  } catch (const GeneratorError& exception) {
    failure = GenerationFailure{exception.code(), exception.what()};
  } catch (const Standard_Failure& exception) {
    failure = GenerationFailure{"occt", weft::occtFailureMessage(exception)};
  } catch (const std::filesystem::filesystem_error& exception) {
    failure = GenerationFailure{"filesystem", exception.what()};
  } catch (const std::exception& exception) {
    failure = GenerationFailure{"inspection", exception.what()};
  }
  return false;
}

} // namespace cad_mesher::fixtures
