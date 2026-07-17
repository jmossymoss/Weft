#pragma once

#include "weft/model.hpp"

#include <array>
#include <compare>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace weft {

enum class RepairProfile {
    Conservative,
    Compatibility,
};

const char* repairProfileName(RepairProfile profile) noexcept;

class SecureImportError : public std::runtime_error {
public:
    SecureImportError(std::string code, std::string message)
        : std::runtime_error(std::move(message)), code_(std::move(code)) {}

    const std::string& code() const noexcept { return code_; }

private:
    std::string code_;
};

enum class StableIdKind {
    Invalid = 0,
    Model = 1,
    Assembly = 2,
    Instance = 3,
    Solid = 4,
    Shell = 5,
    Face = 6,
    Wire = 7,
    Coedge = 8,
    Edge = 9,
    Vertex = 10,
    Region = 11,
    Boundary = 12,
    Diagnostic = 13,
    // Appended to preserve the serialized/digest values of every existing
    // StableIdKind enumerator.
    Compound = 14,
    CompSolid = 15,
};

struct StableId {
    StableIdKind kind = StableIdKind::Invalid;
    std::uint64_t ordinal = 0;

    bool valid() const noexcept {
        return kind != StableIdKind::Invalid && ordinal != 0;
    }
    auto operator<=>(const StableId&) const = default;
};

enum class TopologyOrientation {
    Forward,
    Reversed,
    Internal,
    External,
};

struct SourceMetadata {
    std::string sourceName;
    std::string sourceSha256;
    std::uint64_t sourceByteLength = 0;
    std::optional<double> lengthUnitMm;
    // True when lengthUnitMm was supplied by an explicit caller resolution
    // rather than declared by the source format.
    bool lengthUnitExplicitlyResolved = false;
    std::optional<std::string> stepSchema;
    std::string importerVersion;
    std::vector<std::string> effectiveTranslatorConfiguration;
};

enum class DiagnosticSeverity {
    Info,
    Warning,
    Error,
    Fatal,
};

struct ImportDiagnostic {
    StableId id;
    std::string code;
    DiagnosticSeverity severity = DiagnosticSeverity::Info;
    std::vector<StableId> subjects;
    std::string message;
};

struct ImportDiagnostics {
    std::vector<ImportDiagnostic> events;

    bool hasErrors() const noexcept;
};

struct TopologyOccurrence {
    StableId id;
    StableId underlyingId;
    std::optional<StableId> parentId;
    std::vector<StableId> childIds;
    TopologyOrientation orientation = TopologyOrientation::Forward;
    double tolerance = 0.0;
    bool hasExactRepresentation = false;
    std::vector<std::string> conditionCodes;
    std::optional<StableId> instanceId;
    std::array<double, 16> localTransform{};
    std::array<double, 16> worldTransform{};
};

struct PcurveRef {
    StableId coedgeId;
    StableId faceId;
    std::uint32_t representationIndex = 0;
    auto operator<=>(const PcurveRef&) const = default;
};

struct CoedgeRecord {
    StableId id;
    StableId edgeId;
    StableId wireId;
    StableId faceId;
    std::uint32_t ordinalInWire = 0;
    TopologyOrientation orientation = TopologyOrientation::Forward;
    std::vector<PcurveRef> pcurveRepresentations;
    std::vector<std::string> conditionCodes;
};

struct AssemblyRecord {
    StableId id;
    std::string sourceLabel;
};

struct InstanceRecord {
    StableId id;
    StableId parentId;
    std::optional<StableId> targetAssembly;
    std::vector<StableId> topologyRoots;
    std::string sourceDefinition;
    std::string sourceComponent;
    std::array<double, 16> localTransform{};
    std::array<double, 16> worldTransform{};
};

struct TopologyCoedgeRecord {
    StableId id;
    StableId edgeId;
    StableId wireId;
    std::optional<StableId> faceId;
    std::optional<StableId> instanceId;
    std::uint32_t ordinalInWire = 0;
    TopologyOrientation orientation = TopologyOrientation::Forward;
    std::vector<PcurveRef> pcurveRepresentations;
    std::vector<std::string> conditionCodes;
};

struct TopologyAccount {
    std::vector<AssemblyRecord> assemblies;
    std::vector<InstanceRecord> instances;
    std::vector<StableId> assemblyRoots;
    std::vector<StableId> topologyRoots;
    std::vector<StableId> uniqueEntityIds;
    std::vector<TopologyOccurrence> occurrences;
    std::vector<TopologyCoedgeRecord> coedges;
    std::map<StableId, TopoDS_Shape> exactShapes;
    std::vector<std::string> constructionFailureCodes;
};

struct TopologyAccountCheck {
    std::string code;
    std::size_t expected = 0;
    std::size_t checked = 0;
    std::size_t skipped = 0;
    std::size_t failed = 0;

    bool complete() const noexcept {
        return checked == expected && skipped == 0 && failed == 0 &&
               (expected == 0 || checked != 0);
    }
};

struct TopologyAccountValidation {
    std::vector<TopologyAccountCheck> checks;
    std::vector<std::string> failureCodes;

    bool complete() const noexcept;
    const TopologyAccountCheck* find(std::string_view code) const noexcept;
};

TopologyAccountValidation validateTopologyAccount(
    const TopologyAccount& account);

struct EdgeTopologyRecord {
    StableId id;
    std::optional<StableId> lowerVertex;
    std::optional<StableId> upperVertex;
    bool degenerate = false;
};

struct BRepSnapshot {
    Model model;
    ShapeMap wires;
    ShapeMap vertices;
    std::vector<TopologyOccurrence> occurrences;
    std::vector<CoedgeRecord> coedges;
    std::vector<EdgeTopologyRecord> edgeTopology;
    // Authoritative occurrence-preserving topology. The map-based fields
    // above remain a temporary application/meshing compatibility view.
    TopologyAccount topology;
};

struct SourceBRep {
    SourceMetadata metadata;
    BRepSnapshot snapshot;
};

struct WorkingBRep {
    RepairProfile profile = RepairProfile::Conservative;
    BRepSnapshot snapshot;
};

enum class CorrespondenceRelation {
    Identity,
    Modified,
    Split,
    Merged,
    Removed,
    Introduced,
};

struct CorrespondenceRecord {
    StableId sourceId;
    std::vector<StableId> workingIds;
    CorrespondenceRelation relation = CorrespondenceRelation::Identity;
};

struct SourceWorkingMap {
    std::vector<CorrespondenceRecord> records;
    std::vector<CorrespondenceRecord> topologyOccurrenceRecords;
    bool topologyComplete = false;
    bool complete = false;

    const CorrespondenceRecord* find(StableId source) const noexcept;
};

struct RepairOperation {
    std::string code;
    std::vector<StableId> sourceSubjects;
    std::vector<StableId> workingSubjects;
    std::string detail;
};

struct ToleranceChange {
    StableId sourceId;
    StableId workingId;
    double before = 0.0;
    double after = 0.0;
    // Non-zero only for certified envelope repairs; inferred audits leave these
    // at zero so vacuous proof cannot pass validation.
    std::size_t expectedPcurveUses = 0;
    std::size_t checkedPcurveUses = 0;
    double maximumDiscrepancy = 0.0;
};

struct RepresentationChange {
    StableId sourceEdge;
    StableId workingEdge;
    std::size_t sourceStoredPcurveUses = 0;
    std::size_t workingStoredPcurveUses = 0;
    // True only when a certified copy-on-write geometry replacement produced
    // this delta. Unexplained representation deltas fail validation.
    bool certifiedCopyOnWrite = false;
};

// Evidence for a flag-only repair; curve, p-curve, and tolerance data remain
// unchanged and every existing p-curve use must be checked.
struct ParameterizationFlagChange {
    StableId sourceEdge;
    StableId workingEdge;
    bool sourceSameParameter = false;
    bool sourceSameRange = false;
    bool workingSameParameter = false;
    bool workingSameRange = false;
    std::size_t expectedPcurveUses = 0;
    std::size_t checkedPcurveUses = 0;
    double maximumDiscrepancy = 0.0;
    double toleranceEnvelope = 0.0;
};

// Evidence for an occurrence-orientation-only repair. Geometry handles,
// tolerances, stored representations, and topology cardinality remain
// unchanged; only TopoDS occurrence orientations may differ.
struct OrientationChange {
    StableId sourceId;
    StableId workingId;
    TopologyOrientation sourceOrientation = TopologyOrientation::Forward;
    TopologyOrientation workingOrientation = TopologyOrientation::Forward;
    std::size_t manifoldEdgesChecked = 0;
    std::size_t polarityChecks = 0;
};

struct TopologyCardinalityChange {
    StableIdKind kind = StableIdKind::Invalid;
    std::size_t before = 0;
    std::size_t after = 0;
};

struct RepairValidationEvidence {
    std::string code;
    std::size_t expected = 0;
    std::size_t checked = 0;
    std::size_t skipped = 0;
    std::size_t failed = 0;

    bool complete() const noexcept {
        return checked == expected && skipped == 0 && failed == 0 &&
               (expected == 0 || checked != 0);
    }
};

struct RepairCertificate {
    RepairProfile profile = RepairProfile::Conservative;
    std::string sourceShapeSha256;
    std::string workingShapeSha256;
    bool sourceValid = false;
    bool workingValid = false;
    bool sourceTopologyComplete = false;
    bool workingTopologyComplete = false;
    bool correspondenceComplete = false;
    bool identity = false;
    bool meshable = false;
    std::size_t sourceFaces = 0;
    std::size_t workingFaces = 0;
    std::size_t sourceEdges = 0;
    std::size_t workingEdges = 0;
    std::vector<RepairOperation> operations;
    std::vector<ToleranceChange> toleranceChanges;
    std::vector<RepresentationChange> representationChanges;
    std::vector<ParameterizationFlagChange> parameterizationFlagChanges;
    std::vector<OrientationChange> orientationChanges;
    std::vector<TopologyCardinalityChange> topologyCardinalityChanges;
    std::vector<RepairValidationEvidence> validationEvidence;
};

struct ParameterDomain {
    std::optional<double> lower;
    std::optional<double> upper;
    bool closed = false;
    bool periodic = false;
    std::optional<double> period;
};

struct CurveEvaluation {
    StableId edgeId;
    double parameter = 0.0;
    std::array<double, 3> position{};
    std::array<double, 3> firstDerivative{};
};

struct VertexEvaluation {
    StableId vertexId;
    std::array<double, 3> position{};
};

struct PcurveEvaluation {
    PcurveRef reference;
    double parameter = 0.0;
    std::array<double, 2> uv{};
    std::array<double, 2> firstDerivative{};
};

struct SurfaceEvaluation {
    StableId faceId;
    std::array<double, 2> uv{};
    std::array<double, 3> position{};
    std::array<double, 3> derivativeU{};
    std::array<double, 3> derivativeV{};
    std::optional<std::array<double, 3>> unitNormal;
};

struct PlanarProjectionEvaluation {
    StableId faceId;
    std::array<double, 3> inputPosition{};
    std::array<double, 2> uv{};
    std::array<double, 3> surfacePosition{};
    double discrepancy = 0.0;
};

struct CurveOnSurfaceEvaluation {
    PcurveRef reference;
    double edgeParameter = 0.0;
    double pcurveParameter = 0.0;
    std::array<double, 2> uv{};
    std::array<double, 3> curvePosition{};
    std::array<double, 3> surfacePosition{};
    double discrepancy = 0.0;
};

struct EvaluationFailure {
    std::string code;
    std::string message;
    std::optional<StableId> subject;
};

template <typename T>
struct EvaluationResult {
    std::optional<T> value;
    std::optional<EvaluationFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

class GeometryEvaluator {
public:
    virtual ~GeometryEvaluator() = default;

    virtual EvaluationResult<ParameterDomain> curveDomain(StableId edge) const = 0;
    virtual EvaluationResult<VertexEvaluation> evaluateVertex(
        StableId vertex) const = 0;
    virtual EvaluationResult<CurveEvaluation> evaluateCurve(
        StableId edge, double parameter) const = 0;
    virtual EvaluationResult<PcurveEvaluation> evaluatePcurve(
        PcurveRef representation, double parameter) const = 0;
    virtual EvaluationResult<SurfaceEvaluation> evaluateSurface(
        StableId face, std::array<double, 2> uv) const = 0;
    virtual EvaluationResult<PlanarProjectionEvaluation> projectPointToPlane(
        StableId face, std::array<double, 3> position) const = 0;
    virtual EvaluationResult<CurveOnSurfaceEvaluation> evaluateCurveOnSurface(
        PcurveRef representation, double edgeParameter) const = 0;
};

struct ImportedModel {
    std::shared_ptr<const SourceBRep> source;
    std::shared_ptr<const WorkingBRep> working;
    SourceWorkingMap correspondence;
    RepairCertificate repair;
    ImportDiagnostics diagnostics;
    std::shared_ptr<const GeometryEvaluator> sourceEvaluator;
    std::shared_ptr<const GeometryEvaluator> workingEvaluator;

    const Model& workingModel() const;
    bool meshable() const noexcept { return repair.meshable; }
};

// Secure STEP import. The default conservative profile retains the exact
// transferred representation as immutable source evidence and builds a
// topology-isolated, representation-identical working copy. Compatibility
// runs the existing Weft healing pipeline but exposes its changes through the
// certificate.
ImportedModel importStepSecure(
    const std::string& path,
    RepairProfile profile = RepairProfile::Conservative);

// Secure IGES import. Parsing and provenance consume one immutable byte
// snapshot; working topology uses the same audited repair derivation as STEP.
ImportedModel importIgesSecure(
    const std::string& path,
    RepairProfile profile = RepairProfile::Conservative);

// Secure native OCCT ASCII B-rep import. Parsing, source-byte provenance, and
// geometry transfer all consume one immutable byte snapshot; the working
// topology is derived through the same audited repair stages as STEP.
//
// Native B-rep declares no physical unit. Callers may supply an explicit
// millimetre scale for export/provenance; coordinates are never rescaled.
ImportedModel importBRepSecure(
    const std::string& path,
    RepairProfile profile = RepairProfile::Conservative,
    std::optional<double> lengthUnitMm = std::nullopt);

}  // namespace weft
