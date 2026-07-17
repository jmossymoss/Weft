#pragma once

#include "weft/secure_core.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace weft {

enum class GeometryTaxonomy {
    Curve,
    Surface,
};

enum class TrimDomainClass {
    SimpleDisk,
    Annulus,
    MultiplyPerforated,
    ConvexSimpleRegion,
    FullPeriodicWithCapBoundaries,
    TouchesOneSingularity,
    TouchesTwoSingularities,
    InvalidOrUnresolved,
};

enum class RecognitionConfidence {
    ProvenAnalytic,
    ExactlyTyped,
    NotRecognised,
};

enum class GeometrySupportState {
    SupportedAnalyticTemplate,
    DeferredResidualSurface,
    InvalidImportedGeometry,
    UnrecognisedExactGeometry,
};

struct ExactGeometryClassification {
    StableId subjectId;
    GeometryTaxonomy taxonomy = GeometryTaxonomy::Curve;
    std::string familyCode;
    std::string concreteType;
    std::vector<std::string> wrapperStack;
    std::vector<ParameterDomain> parameterDomains;
    std::vector<std::string> conditionCodes;
    std::optional<TrimDomainClass> trimDomain;
    RecognitionConfidence confidence = RecognitionConfidence::NotRecognised;
    GeometrySupportState support =
        GeometrySupportState::UnrecognisedExactGeometry;
    std::string strategyOrReasonCode;
    std::vector<StableId> sourceSubjects;
};

struct LogicalRegion {
    StableId id;
    std::string code;
    std::vector<StableId> workingFaces;
    std::vector<StableId> sourceFaces;
    std::vector<StableId> boundaryEdges;
};

struct ReconnaissanceDiagnostic {
    std::string code;
    StableId subject;
    std::string message;
};

struct ReconnaissanceReport {
    std::vector<ExactGeometryClassification> records;
    std::vector<LogicalRegion> regions;
    std::vector<ReconnaissanceDiagnostic> diagnostics;
    std::size_t expectedSubjects = 0;
    std::size_t checkedSubjects = 0;
    std::size_t unsupportedSubjects = 0;
    bool complete = false;

    const ExactGeometryClassification* find(StableId subject) const noexcept;
};

const char* geometrySupportStateName(GeometrySupportState state) noexcept;

// Total, read-only classification of the working B-rep. Every face and edge
// receives exactly one record; unknown exact types are named and never folded
// into a supported family.
ReconnaissanceReport reconnoitre(const ImportedModel& imported);

}  // namespace weft
