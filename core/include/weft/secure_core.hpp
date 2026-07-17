#pragma once

#include "weft/model.hpp"

#include <array>
#include <compare>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
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
    Invalid,
    Model,
    Assembly,
    Instance,
    Solid,
    Shell,
    Face,
    Wire,
    Coedge,
    Edge,
    Vertex,
    Region,
    Diagnostic,
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

struct BRepSnapshot {
    Model model;
    std::vector<TopologyOccurrence> occurrences;
    std::vector<CoedgeRecord> coedges;
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
};

struct RepresentationChange {
    StableId sourceEdge;
    StableId workingEdge;
    std::size_t sourceStoredPcurveUses = 0;
    std::size_t workingStoredPcurveUses = 0;
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
    virtual EvaluationResult<CurveEvaluation> evaluateCurve(
        StableId edge, double parameter) const = 0;
    virtual EvaluationResult<PcurveEvaluation> evaluatePcurve(
        PcurveRef representation, double parameter) const = 0;
    virtual EvaluationResult<SurfaceEvaluation> evaluateSurface(
        StableId face, std::array<double, 2> uv) const = 0;
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
// transferred shape as both source and working geometry until a bounded,
// separately proven repair stage is enabled. Compatibility runs the existing
// Weft healing pipeline but exposes its changes through the certificate.
ImportedModel importStepSecure(
    const std::string& path,
    RepairProfile profile = RepairProfile::Conservative);

}  // namespace weft
