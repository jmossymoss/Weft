#pragma once

#include "occt_compat.hpp"

#include <TopoDS_Shape.hxx>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace cad_mesher::fixtures {

struct ParameterDomainObservation final {
  std::string axis;
  double lower{};
  double upper{};
  bool closed{};
  bool periodic{};
  std::optional<double> period;
  std::string unit;
};

struct NumericPropertyObservation final {
  std::string code;
  double value{};
  std::string unit;
};

struct IntegerPropertyObservation final {
  std::string code;
  int value{};
};

struct BooleanPropertyObservation final {
  std::string code;
  bool value{};
};

struct NumericSequenceObservation final {
  std::string code;
  std::vector<double> values;
  std::string unit;
};

struct IntegerSequenceObservation final {
  std::string code;
  std::vector<int> values;
};

struct VertexParameterObservation final {
  std::string role;
  int vertex_partner_ordinal{};
  std::optional<double> parameter;
  double tolerance_mm{};
};

struct GeometryObservation final {
  std::string id;
  std::string subject_kind;
  int partner_ordinal{};
  std::string family;
  std::string concrete_type;
  std::vector<std::string> wrapper_stack;
  std::vector<ParameterDomainObservation> domains;
  std::vector<NumericPropertyObservation> numeric_properties;
  std::vector<IntegerPropertyObservation> integer_properties;
  std::vector<BooleanPropertyObservation> boolean_properties;
  std::vector<NumericSequenceObservation> numeric_sequences;
  std::vector<IntegerSequenceObservation> integer_sequences;
  std::vector<VertexParameterObservation> vertex_parameters;
};

struct PcurveObservation final {
  std::string id;
  int edge_partner_ordinal{};
  int face_partner_ordinal{};
  int wire_ordinal{};
  int order_in_wire{};
  int representation_index{};
  std::string orientation;
  std::string concrete_type;
  double edge_range_lower{};
  double edge_range_upper{};
  double pcurve_range_lower{};
  double pcurve_range_upper{};
  bool same_parameter{};
  bool same_range{};
  bool degenerated{};
  double edge_tolerance_mm{};
  std::optional<double> maximum_curve_surface_residual_mm;
  std::vector<std::string> condition_codes;
};

struct ContinuityObservation final {
  std::string id;
  int edge_partner_ordinal{};
  int first_face_partner_ordinal{};
  int second_face_partner_ordinal{};
  std::string occt_continuity;
};

struct EvaluationObservation final {
  std::string id;
  std::string subject_kind;
  int partner_ordinal{};
  std::string outcome;
  std::optional<double> u_derivative_norm;
  std::optional<double> v_derivative_norm;
  std::optional<double> normal_norm;
  std::optional<double> uv_metric_condition;
  std::vector<std::string> condition_codes;
};

struct FixtureEvidence final {
  std::vector<GeometryObservation> geometry;
  std::vector<PcurveObservation> pcurves;
  std::vector<ContinuityObservation> continuity;
  std::vector<EvaluationObservation> evaluations;
};

[[nodiscard]] FixtureEvidence observe_fixture_evidence(const TopoDS_Shape& shape);

void write_fixture_evidence_json(std::ostream& stream, const FixtureEvidence& evidence,
                                 int indentation);

} // namespace cad_mesher::fixtures
