#include "weft/canonical_boundary.hpp"

#include <BRep_Tool.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <span>
#include <utility>

namespace weft {
namespace {

constexpr double kCriticalParamEpsilon = 1e-12;
constexpr double kTwoPi = 6.283185307179586476925286766559;

// Collapse sub-8-ULP platform libm/OCCT drift so exact dyadic CDT inputs and
// mesh fingerprints match across Windows and Linux. Applied only after
// discrepancy certificates have already accepted the raw evaluations.
void stabilizeCoordinate(double& value) {
    if (!std::isfinite(value)) return;
    auto bits = std::bit_cast<std::uint64_t>(value);
    bits &= ~std::uint64_t{0x7};
    value = std::bit_cast<double>(bits);
}

void stabilizeCoordinates(std::span<double> values) {
    for (double& value : values) stabilizeCoordinate(value);
}

CanonicalBoundaryBuildResult buildFailure(
    CanonicalBoundaryReport report, std::string code, std::string message,
    std::vector<StableId> subjects = {}) {
    ++report.failed;
    CanonicalBoundaryBuildResult result;
    result.validation = report;
    result.failure = CanonicalBoundaryFailure{
        std::move(code), std::move(message), std::move(subjects)};
    return result;
}

double occurrenceTolerance(const BRepSnapshot& snapshot, StableId subject) {
    const auto found = std::find_if(
        snapshot.occurrences.begin(), snapshot.occurrences.end(),
        [subject](const TopologyOccurrence& occurrence) {
            return occurrence.id == subject;
        });
    return found == snapshot.occurrences.end() ? 0.0 : found->tolerance;
}

std::optional<StableId> sourceForWorking(const ImportedModel& imported,
                                         StableId working) {
    std::optional<StableId> source;
    for (const CorrespondenceRecord& record : imported.correspondence.records) {
        if (std::find(record.workingIds.begin(), record.workingIds.end(), working) ==
            record.workingIds.end()) {
            continue;
        }
        if (source.has_value()) return std::nullopt;
        source = record.sourceId;
    }
    return source;
}

struct MappingState {
    const CoedgeRecord* coedge = nullptr;
    const ExactGeometryClassification* faceClassification = nullptr;
    std::optional<PcurveRef> representation;
    BoundaryUvMappingKind kind = BoundaryUvMappingKind::StoredPcurve;
    std::array<std::optional<double>, 2> periods;
    std::array<double, 2> previousLifted{};
    bool hasPrevious = false;
};

std::array<std::optional<double>, 2> periodsFor(
    const ExactGeometryClassification& classification) {
    std::array<std::optional<double>, 2> periods;
    for (std::size_t axis = 0;
         axis < periods.size() && axis < classification.parameterDomains.size();
         ++axis) {
        const ParameterDomain& domain = classification.parameterDomains[axis];
        if (domain.periodic && domain.period && std::isfinite(*domain.period) &&
            *domain.period > 0.0) {
            periods[axis] = domain.period;
        }
    }
    return periods;
}

bool validConfiguration(const CanonicalBoundaryConfiguration& configuration) {
    return std::isfinite(configuration.minimumDiscrepancyTolerance) &&
        configuration.minimumDiscrepancyTolerance >= 0.0 &&
        std::isfinite(configuration.maximumDiscrepancyTolerance) &&
        configuration.maximumDiscrepancyTolerance > 0.0 &&
        configuration.maximumDiscrepancyTolerance >=
            configuration.minimumDiscrepancyTolerance &&
        std::isfinite(configuration.sourceToleranceScale) &&
        configuration.sourceToleranceScale >= 1.0 &&
        std::isfinite(configuration.periodicLiftAmbiguityTolerance) &&
        configuration.periodicLiftAmbiguityTolerance >= 0.0 &&
        configuration.periodicLiftAmbiguityTolerance < 0.5;
}

bool almostEqualParameter(double a, double b, double span) {
    const double scale = std::max(1.0, std::abs(span));
    return std::abs(a - b) <= kCriticalParamEpsilon * scale;
}

double normalizeClosedParameter(double parameter, double lower, double upper,
                                bool closed, double span) {
    if (closed && almostEqualParameter(parameter, upper, span)) {
        return lower;
    }
    return parameter;
}

bool hasCondition(const ExactGeometryClassification& record,
                  const char* code) {
    return std::find(record.conditionCodes.begin(), record.conditionCodes.end(),
                     code) != record.conditionCodes.end();
}

double discrepancyCap(const CanonicalBoundaryConfiguration& configuration,
                      const ExactGeometryClassification* face) {
    // Plasticity freeform STEP extracts can carry larger B-rep tolerances
    // than the default 1e-3 analytic cap; keep a bounded subclass raise.
    if (face && hasCondition(*face, "freeform.uv_grid_candidate")) {
        return std::max(configuration.maximumDiscrepancyTolerance, 1e-1);
    }
    return configuration.maximumDiscrepancyTolerance;
}

bool supportedSegmentationFamily(const ExactGeometryClassification& record,
                                 GeometryTaxonomy taxonomy) {
    if (record.taxonomy != taxonomy) return false;
    if (taxonomy == GeometryTaxonomy::Curve) {
        if (record.familyCode == "line" || record.familyCode == "circle") {
            // Degenerate apex circles stay InvalidImportedGeometry but still
            // need the singular canonical station path.
            return record.support ==
                       GeometrySupportState::SupportedAnalyticTemplate ||
                (record.support ==
                     GeometrySupportState::InvalidImportedGeometry &&
                 hasCondition(record, "degenerate"));
        }
        // MAP-B: bounded bspline/bezier edge sampling for mapped patches.
        if (record.familyCode == "bspline" || record.familyCode == "bezier") {
            return record.support ==
                       GeometrySupportState::SupportedAnalyticTemplate ||
                record.support == GeometrySupportState::DeferredResidualSurface;
        }
        return false;
    }
    if (record.familyCode == "plane" || record.familyCode == "cylinder") {
        return record.support ==
                   GeometrySupportState::SupportedAnalyticTemplate ||
            record.support == GeometrySupportState::DeferredResidualSurface;
    }
    // Analytic singular/periodic families may remain DeferredResidualSurface
    // until their FAMILY-C consumer promotes support, but FAMILY-B still needs
    // singular/periodic critical events.
    if (record.familyCode == "cone" || record.familyCode == "sphere" ||
        record.familyCode == "torus") {
        return record.support ==
                   GeometrySupportState::SupportedAnalyticTemplate ||
            record.support == GeometrySupportState::DeferredResidualSurface;
    }
    // MAP-B / FREE-B: mapped/freeform UV-grid candidates may build boundaries
    // while deferred.
    if ((record.familyCode == "bspline" || record.familyCode == "bezier" ||
         record.familyCode == "extrusion" ||
         record.familyCode == "revolution") &&
        (hasCondition(record, "mapped.four_sided_candidate") ||
         hasCondition(record, "freeform.uv_grid_candidate"))) {
        return record.support ==
                   GeometrySupportState::SupportedAnalyticTemplate ||
            record.support == GeometrySupportState::DeferredResidualSurface;
    }
    return false;
}

bool isSingularAnalyticFace(const ExactGeometryClassification& face) {
    return face.familyCode == "cone" || face.familyCode == "sphere";
}

bool vertexTouchesDegenerateEdge(const BRepSnapshot& snapshot,
                                 StableId vertex) {
    if (!vertex.valid() || vertex.kind != StableIdKind::Vertex) return false;
    for (const EdgeTopologyRecord& edge : snapshot.edgeTopology) {
        if (!edge.degenerate) continue;
        if (edge.lowerVertex == vertex || edge.upperVertex == vertex) {
            return true;
        }
    }
    return false;
}

int criticalEventKindOrder(CriticalParameterEventKind kind) {
    switch (kind) {
        case CriticalParameterEventKind::DomainEndpoint:
            return 0;
        case CriticalParameterEventKind::ContactCritical:
            return 1;
        case CriticalParameterEventKind::PeriodicSeam:
            return 2;
        case CriticalParameterEventKind::MonotonicExtremum:
            return 3;
        case CriticalParameterEventKind::Singular:
            return 4;
    }
    return 100;
}

bool criticalEventLess(const CriticalParameterEvent& a,
                       const CriticalParameterEvent& b) {
    if (a.curveParameter != b.curveParameter) {
        return a.curveParameter < b.curveParameter;
    }
    const int kindA = criticalEventKindOrder(a.kind);
    const int kindB = criticalEventKindOrder(b.kind);
    if (kindA != kindB) return kindA < kindB;
    if (a.face != b.face) return a.face < b.face;
    if (a.coedge != b.coedge) return a.coedge < b.coedge;
    if (a.axis != b.axis) return a.axis < b.axis;
    return a.detectionCode < b.detectionCode;
}

bool criticalEventSameKey(const CriticalParameterEvent& a,
                          const CriticalParameterEvent& b) {
    return a.kind == b.kind && a.edge == b.edge && a.coedge == b.coedge &&
        a.face == b.face && a.axis == b.axis &&
        a.detectionCode == b.detectionCode &&
        almostEqualParameter(a.curveParameter, b.curveParameter, 1.0);
}

void appendCriticalEvent(std::vector<CriticalParameterEvent>& events,
                         CriticalParameterEvent event, double lower,
                         double upper, bool closed, double span) {
    event.curveParameter = normalizeClosedParameter(
        event.curveParameter, lower, upper, closed, span);
    if (event.curveParameter < lower - kCriticalParamEpsilon * std::max(1.0, span) ||
        event.curveParameter > upper + kCriticalParamEpsilon * std::max(1.0, span)) {
        return;
    }
    if (closed && almostEqualParameter(event.curveParameter, upper, span)) {
        event.curveParameter = lower;
    }
    for (const CriticalParameterEvent& existing : events) {
        if (criticalEventSameKey(existing, event)) return;
    }
    events.push_back(std::move(event));
}

Handle(Geom_Curve) basisCurve3d(const Handle(Geom_Curve)& curve) {
    if (curve.IsNull()) return curve;
    if (Handle(Geom_TrimmedCurve) trimmed = Handle(Geom_TrimmedCurve)::DownCast(curve)) {
        return basisCurve3d(trimmed->BasisCurve());
    }
    return curve;
}

Handle(Geom2d_Curve) basisCurve2d(const Handle(Geom2d_Curve)& curve) {
    if (curve.IsNull()) return curve;
    if (Handle(Geom2d_TrimmedCurve) trimmed =
            Handle(Geom2d_TrimmedCurve)::DownCast(curve)) {
        return basisCurve2d(trimmed->BasisCurve());
    }
    return curve;
}

void appendPeriodicTargets(double startCoord, double endCoord, double period,
                           double lower, double upper,
                           const std::function<double(double)>& parameterAtCoord,
                           CriticalParameterEvent seed,
                           std::vector<CriticalParameterEvent>& events,
                           bool closed, double span) {
    if (!(period > 0.0) || !std::isfinite(period)) return;
    const double cMin = std::min(startCoord, endCoord);
    const double cMax = std::max(startCoord, endCoord);
    const double scale = std::max({1.0, std::abs(cMin), std::abs(cMax), period});
    const long long first =
        static_cast<long long>(std::llround(std::floor(cMin / period))) - 1;
    const long long last =
        static_cast<long long>(std::llround(std::ceil(cMax / period))) + 1;
    for (long long index = first; index <= last; ++index) {
        const double target = static_cast<double>(index) * period;
        if (target < cMin - kCriticalParamEpsilon * scale ||
            target > cMax + kCriticalParamEpsilon * scale) {
            continue;
        }
        CriticalParameterEvent event = seed;
        event.kind = CriticalParameterEventKind::PeriodicSeam;
        event.detectionCode = "event.periodic_seam";
        event.curveParameter = parameterAtCoord(target);
        appendCriticalEvent(events, std::move(event), lower, upper, closed,
                            span);
    }
}

void appendCircleIntrinsicMonotonicEvents(
    double lower, double upper, bool closed, double span,
    CriticalParameterEvent seed,
    std::vector<CriticalParameterEvent>& events) {
    // Quarter-turn lattice relative to the trimmed edge domain. When
    // intervalCount is a multiple of four on a full-period circle these
    // coincide with the uniform samples and do not change ring cardinality.
    constexpr double step = kTwoPi * 0.25;
    const long long count =
        static_cast<long long>(std::llround(std::floor(span / step))) + 2;
    for (long long k = 0; k <= count; ++k) {
        CriticalParameterEvent event = seed;
        event.kind = CriticalParameterEventKind::MonotonicExtremum;
        event.detectionCode = "event.monotonic_extremum";
        event.curveParameter = lower + static_cast<double>(k) * step;
        appendCriticalEvent(events, std::move(event), lower, upper, closed,
                            span);
    }
}

struct CriticalSegmentationResult {
    std::vector<CriticalParameterEvent> events;
    std::optional<CanonicalBoundaryFailure> failure;
};

CriticalSegmentationResult collectSupportedCriticalEvents(
    const ImportedModel& imported,
    const ExactGeometryClassification& curveClassification,
    const EdgeTopologyRecord& topology,
    const std::vector<MappingState>& mappings,
    const std::optional<StableId>& sourceEdge, double lower, double upper,
    bool closed, double span) {
    CriticalSegmentationResult result;
    const StableId edgeId = topology.id;

    CriticalParameterEvent endpointSeed;
    endpointSeed.edge = edgeId;
    endpointSeed.sourceEdge = sourceEdge;
    endpointSeed.kind = CriticalParameterEventKind::DomainEndpoint;
    endpointSeed.detectionCode = "event.domain_endpoint";
    endpointSeed.curveParameter = lower;
    appendCriticalEvent(result.events, endpointSeed, lower, upper, closed,
                        span);
    if (!closed) {
        endpointSeed.curveParameter = upper;
        appendCriticalEvent(result.events, endpointSeed, lower, upper, closed,
                            span);
    }

    if (topology.lowerVertex) {
        CriticalParameterEvent contact = endpointSeed;
        contact.kind = CriticalParameterEventKind::ContactCritical;
        contact.detectionCode = "event.contact_vertex";
        contact.curveParameter = lower;
        appendCriticalEvent(result.events, contact, lower, upper, closed, span);
    }
    if (!closed && topology.upperVertex) {
        CriticalParameterEvent contact = endpointSeed;
        contact.kind = CriticalParameterEventKind::ContactCritical;
        contact.detectionCode = "event.contact_vertex";
        contact.curveParameter = upper;
        appendCriticalEvent(result.events, contact, lower, upper, closed, span);
    }

    if (curveClassification.familyCode == "circle") {
        appendCircleIntrinsicMonotonicEvents(lower, upper, closed, span,
                                             endpointSeed, result.events);
        if (!curveClassification.parameterDomains.empty() &&
            curveClassification.parameterDomains.front().periodic && closed) {
            CriticalParameterEvent seam = endpointSeed;
            seam.kind = CriticalParameterEventKind::PeriodicSeam;
            seam.detectionCode = "event.periodic_seam";
            seam.curveParameter = lower;
            appendCriticalEvent(result.events, seam, lower, upper, closed,
                                span);
        }
    }

    const auto edgeShapeIt =
        imported.working->snapshot.topology.exactShapes.find(edgeId);
    if (edgeShapeIt == imported.working->snapshot.topology.exactShapes.end()) {
        result.failure = CanonicalBoundaryFailure{
            "boundary.critical_segmentation_unsupported",
            "working edge has no exact shape for critical segmentation",
            {edgeId}};
        return result;
    }
    const TopoDS_Edge edge = TopoDS::Edge(edgeShapeIt->second);

    for (const MappingState& mapping : mappings) {
        const bool singularTrim =
            mapping.faceClassification->trimDomain ==
                TrimDomainClass::TouchesOneSingularity ||
            mapping.faceClassification->trimDomain ==
                TrimDomainClass::TouchesTwoSingularities;
        if (singularTrim &&
            !isSingularAnalyticFace(*mapping.faceClassification)) {
            result.failure = CanonicalBoundaryFailure{
                "boundary.critical_segmentation_unsupported",
                "singular trim domains require a dedicated critical-event solver",
                {edgeId, mapping.coedge->id, mapping.coedge->faceId}};
            return result;
        }

        CriticalParameterEvent seed;
        seed.edge = edgeId;
        seed.coedge = mapping.coedge->id;
        seed.face = mapping.coedge->faceId;
        seed.sourceEdge = sourceEdge;
        seed.sourceFace =
            sourceForWorking(imported, mapping.coedge->faceId);

        if (singularTrim &&
            isSingularAnalyticFace(*mapping.faceClassification)) {
            const char* singularCode =
                mapping.faceClassification->familyCode == "sphere"
                ? "event.sphere_pole"
                : "event.cone_apex";
            // Pole/apex contact on generators or meridians: mark endpoints that
            // share a degenerate edge.
            if (curveClassification.familyCode == "line" ||
                curveClassification.familyCode == "circle") {
                if (topology.lowerVertex &&
                    vertexTouchesDegenerateEdge(imported.working->snapshot,
                                                *topology.lowerVertex)) {
                    CriticalParameterEvent singular = seed;
                    singular.kind = CriticalParameterEventKind::Singular;
                    singular.detectionCode = singularCode;
                    singular.curveParameter = lower;
                    appendCriticalEvent(result.events, singular, lower, upper,
                                        closed, span);
                }
                if (!closed && topology.upperVertex &&
                    vertexTouchesDegenerateEdge(imported.working->snapshot,
                                                *topology.upperVertex)) {
                    CriticalParameterEvent singular = seed;
                    singular.kind = CriticalParameterEventKind::Singular;
                    singular.detectionCode = singularCode;
                    singular.curveParameter = upper;
                    appendCriticalEvent(result.events, singular, lower, upper,
                                        closed, span);
                }
            }
        }

        if (mapping.kind == BoundaryUvMappingKind::StoredPcurve &&
            mapping.representation) {
            const auto faceShapeIt =
                imported.working->snapshot.topology.exactShapes.find(
                    mapping.coedge->faceId);
            if (faceShapeIt ==
                imported.working->snapshot.topology.exactShapes.end()) {
                result.failure = CanonicalBoundaryFailure{
                    "boundary.critical_segmentation_unsupported",
                    "owning face has no exact shape for critical segmentation",
                    {edgeId, mapping.coedge->faceId}};
                return result;
            }
            TopoDS_Edge oriented = edge;
            if (mapping.representation->representationIndex == 1) {
                oriented.Reverse();
            }
            const TopoDS_Face face = TopoDS::Face(faceShapeIt->second);
            double first = 0.0;
            double last = 0.0;
            bool stored = false;
            Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                oriented, face, first, last, &stored);
            if (pcurve.IsNull() || !stored) {
                // Domain/contact/curve events remain; UV-period refinement is
                // skipped when the stored representation cannot be fetched.
                continue;
            }
            Handle(Geom2d_Curve) basis = basisCurve2d(pcurve);
            if (Handle(Geom2d_Line) line = Handle(Geom2d_Line)::DownCast(basis)) {
                const gp_Pnt2d start = line->Value(lower);
                const gp_Pnt2d end = line->Value(upper);
                for (std::size_t axis = 0; axis < mapping.periods.size();
                     ++axis) {
                    if (!mapping.periods[axis]) continue;
                    seed.axis = axis;
                    if (closed &&
                        almostEqualParameter(span, *mapping.periods[axis],
                                             span)) {
                        CriticalParameterEvent seam = seed;
                        seam.kind = CriticalParameterEventKind::PeriodicSeam;
                        seam.detectionCode = "event.periodic_seam";
                        seam.curveParameter = lower;
                        appendCriticalEvent(result.events, std::move(seam),
                                            lower, upper, closed, span);
                        continue;
                    }
                    const double startCoord =
                        axis == 0 ? start.X() : start.Y();
                    const double endCoord = axis == 0 ? end.X() : end.Y();
                    const gp_Dir2d direction = line->Direction();
                    const double slope =
                        axis == 0 ? direction.X() : direction.Y();
                    appendPeriodicTargets(
                        startCoord, endCoord, *mapping.periods[axis], lower,
                        upper,
                        [&](double target) {
                            if (std::abs(slope) <= kCriticalParamEpsilon) {
                                return lower;
                            }
                            const gp_Pnt2d location = line->Location();
                            const double origin =
                                axis == 0 ? location.X() : location.Y();
                            return (target - origin) / slope;
                        },
                        seed, result.events, closed, span);
                }
            } else if (Handle(Geom2d_Circle) circle =
                           Handle(Geom2d_Circle)::DownCast(basis)) {
                for (std::size_t axis = 0; axis < mapping.periods.size();
                     ++axis) {
                    if (!mapping.periods[axis]) continue;
                    seed.axis = axis;
                    if (closed && almostEqualParameter(span, kTwoPi, span)) {
                        CriticalParameterEvent seam = seed;
                        seam.kind = CriticalParameterEventKind::PeriodicSeam;
                        seam.detectionCode = "event.periodic_seam";
                        seam.curveParameter = lower;
                        appendCriticalEvent(result.events, std::move(seam),
                                            lower, upper, closed, span);
                        continue;
                    }
                    const gp_Pnt2d start = circle->Value(lower);
                    const gp_Pnt2d end = circle->Value(upper);
                    const double startCoord =
                        axis == 0 ? start.X() : start.Y();
                    const double endCoord = axis == 0 ? end.X() : end.Y();
                    for (double coord : {startCoord, endCoord}) {
                        const double periods = coord / *mapping.periods[axis];
                        if (!std::isfinite(periods)) continue;
                        const double nearest = std::round(periods);
                        if (std::abs(periods - nearest) >
                            kCriticalParamEpsilon) {
                            continue;
                        }
                        CriticalParameterEvent seam = seed;
                        seam.kind = CriticalParameterEventKind::PeriodicSeam;
                        seam.detectionCode = "event.periodic_seam";
                        seam.curveParameter =
                            almostEqualParameter(coord, startCoord, span)
                                ? lower
                                : upper;
                        appendCriticalEvent(result.events, std::move(seam),
                                            lower, upper, closed, span);
                    }
                }
            } else {
                // MAP-B / FREE-B: non-line/circle p-curves on mapped/freeform
                // UV-grid faces keep domain/contact events only.
                const bool uvGridFace =
                    mapping.faceClassification &&
                    (hasCondition(*mapping.faceClassification,
                                  "mapped.four_sided_candidate") ||
                     hasCondition(*mapping.faceClassification,
                                  "freeform.uv_grid_candidate"));
                if (!uvGridFace) {
                    result.failure = CanonicalBoundaryFailure{
                        "boundary.critical_segmentation_unsupported",
                        "p-curve family is outside the supported critical-event set",
                        {edgeId, mapping.coedge->id, mapping.coedge->faceId}};
                    return result;
                }
            }
            continue;
        }

        // Derived planar projection: lines need no further events; circles
        // already received intrinsic quarter-turn events above. Mapped
        // bspline/bezier edges likewise rely on domain/contact events only.
        if (curveClassification.familyCode != "line" &&
            curveClassification.familyCode != "circle" &&
            curveClassification.familyCode != "bspline" &&
            curveClassification.familyCode != "bezier") {
            result.failure = CanonicalBoundaryFailure{
                "boundary.critical_segmentation_unsupported",
                "3D curve family is outside the supported critical-event set",
                {edgeId, mapping.coedge->faceId}};
            return result;
        }
    }

    std::sort(result.events.begin(), result.events.end(), criticalEventLess);
    return result;
}

std::vector<double> mergeCriticalParameters(
    double lower, double upper, bool closed, double span,
    std::uint32_t intervalCount,
    const std::vector<CriticalParameterEvent>& events) {
    std::vector<double> parameters;
    parameters.reserve(static_cast<std::size_t>(intervalCount) + 1U +
                       events.size());
    const std::size_t uniformCount = closed
        ? static_cast<std::size_t>(intervalCount)
        : static_cast<std::size_t>(intervalCount) + 1U;
    for (std::size_t sampleIndex = 0; sampleIndex < uniformCount;
         ++sampleIndex) {
        const double parameter =
            !closed && sampleIndex + 1U == uniformCount
            ? upper
            : lower + span * static_cast<double>(sampleIndex) /
                          static_cast<double>(intervalCount);
        parameters.push_back(
            normalizeClosedParameter(parameter, lower, upper, closed, span));
    }
    for (const CriticalParameterEvent& event : events) {
        parameters.push_back(normalizeClosedParameter(
            event.curveParameter, lower, upper, closed, span));
    }
    std::sort(parameters.begin(), parameters.end());
    parameters.erase(
        std::unique(parameters.begin(), parameters.end(),
                    [span](double a, double b) {
                        return almostEqualParameter(a, b, span);
                    }),
        parameters.end());
    if (closed) {
        parameters.erase(
            std::remove_if(parameters.begin(), parameters.end(),
                           [&](double parameter) {
                               return almostEqualParameter(parameter, upper,
                                                           span);
                           }),
            parameters.end());
    }
    return parameters;
}

bool ambiguousIntegerPeriod(double requestedPeriods,
                            double ambiguityTolerance) {
    if (!std::isfinite(requestedPeriods)) return true;
    const double nearest = std::round(requestedPeriods);
    return std::abs(requestedPeriods - nearest) >=
        0.5 - ambiguityTolerance;
}

const CoedgeUvUse* findUse(const CanonicalBoundarySample& sample,
                           StableId coedge,
                           const std::optional<PcurveRef>& representation,
                           BoundaryUvMappingKind kind) {
    for (const CoedgeUvUse& use : sample.faceUses) {
        if (use.coedge != coedge || use.mappingKind != kind) continue;
        if (use.representation != representation) continue;
        return &use;
    }
    return nullptr;
}

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vector3 vectorOf(const std::array<double, 3>& point) {
    return {point[0], point[1], point[2]};
}

Vector3 subtract(Vector3 a, Vector3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 cross(Vector3 a, Vector3 b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

double dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(Vector3 value) {
    return std::sqrt(dot(value, value));
}

Vector3 centroid(const std::vector<std::array<double, 3>>& ring) {
    Vector3 sum;
    for (const auto& point : ring) {
        sum.x += point[0];
        sum.y += point[1];
        sum.z += point[2];
    }
    const double inverse = 1.0 / static_cast<double>(ring.size());
    return {sum.x * inverse, sum.y * inverse, sum.z * inverse};
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

AzimuthRegistrationResult registrationFailure(std::string code,
                                              std::string message) {
    AzimuthRegistrationResult result;
    result.failure = CanonicalBoundaryFailure{
        std::move(code), std::move(message), {}};
    return result;
}

}  // namespace

const CanonicalBoundary* CanonicalBoundarySet::find(
    StableId edge) const noexcept {
    const auto found = std::lower_bound(
        boundaries.begin(), boundaries.end(), edge,
        [](const CanonicalBoundary& boundary, StableId id) {
            return boundary.edge < id;
        });
    return found == boundaries.end() || found->edge != edge ? nullptr : &*found;
}

CanonicalBoundaryBuildResult buildCanonicalBoundaries(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const IntervalSolution& intervals,
    const CanonicalBoundaryConfiguration& configuration) {
    CanonicalBoundaryReport report;
    if (!validConfiguration(configuration)) {
        return buildFailure(report, "boundary.invalid_configuration",
                            "canonical-boundary tolerances are invalid");
    }
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !imported.correspondence.complete) {
        return buildFailure(
            report, "boundary.working_model_not_meshable",
            "canonical boundaries require a meshable certified working model");
    }
    if (!reconnaissance.complete) {
        return buildFailure(
            report, "boundary.reconnaissance_incomplete",
            "canonical boundaries require complete geometry reconnaissance");
    }

    const BRepSnapshot& snapshot = imported.working->snapshot;
    report.expectedEdges =
        static_cast<std::size_t>(snapshot.model.edges.Extent());
    if (report.expectedEdges == 0 ||
        snapshot.edgeTopology.size() != report.expectedEdges) {
        return buildFailure(
            report, "boundary.edge_topology_incomplete",
            "every working edge must have exactly one endpoint record");
    }

    CanonicalBoundarySet set;
    set.boundaries.reserve(report.expectedEdges);
    const std::uint64_t topologicalVertexCount = static_cast<std::uint64_t>(
        std::count_if(snapshot.occurrences.begin(), snapshot.occurrences.end(),
                      [](const TopologyOccurrence& occurrence) {
                          return occurrence.id.kind == StableIdKind::Vertex;
                      }));
    std::uint64_t nextInteriorVertex = topologicalVertexCount;

    for (const EdgeTopologyRecord& topology : snapshot.edgeTopology) {
        const StableId edgeId = topology.id;
        const StableId boundaryId{StableIdKind::Boundary, edgeId.ordinal};
        if (!edgeId.valid() || edgeId.kind != StableIdKind::Edge) {
            return buildFailure(report, "boundary.edge_topology_invalid",
                                "an endpoint record has no valid working edge");
        }
        const auto count = intervals.find(boundaryId);
        if (!count || *count == 0) {
            return buildFailure(
                report, "boundary.interval_count_missing",
                "every working edge requires a positive solved interval count",
                {edgeId, boundaryId});
        }
        if (topology.degenerate) {
            // Singular apex station: one topological vertex sample, no curve
            // sampling. Interval count must be exactly one.
            if (*count != 1) {
                return buildFailure(
                    report, "boundary.degenerate_interval_count_invalid",
                    "a degenerate singular edge requires interval count 1",
                    {edgeId, boundaryId});
            }
            if (!topology.lowerVertex && !topology.upperVertex) {
                return buildFailure(
                    report, "boundary.degenerate_vertex_missing",
                    "a degenerate singular edge must resolve a topological vertex",
                    {edgeId});
            }
            const StableId apexVertex =
                topology.lowerVertex ? *topology.lowerVertex
                                     : *topology.upperVertex;
            const auto apex =
                imported.workingEvaluator->evaluateVertex(apexVertex);
            const std::optional<StableId> sourceVertex =
                sourceForWorking(imported, apexVertex);
            const std::optional<StableId> sourceEdge =
                sourceForWorking(imported, edgeId);
            if (!apex || !sourceVertex ||
                sourceVertex->kind != StableIdKind::Vertex || !sourceEdge ||
                sourceEdge->kind != StableIdKind::Edge) {
                return buildFailure(
                    report, "boundary.degenerate_vertex_evaluation_failed",
                    "degenerate apex vertex did not evaluate with source provenance",
                    {edgeId, apexVertex});
            }

            std::vector<MappingState> mappings;
            for (const CoedgeRecord& coedge : snapshot.coedges) {
                if (coedge.edgeId != edgeId) continue;
                const ExactGeometryClassification* face =
                    reconnaissance.find(coedge.faceId);
                if (!face || face->taxonomy != GeometryTaxonomy::Surface) {
                    return buildFailure(
                        report, "boundary.face_classification_missing",
                        "an owning coedge has no surface classification",
                        {edgeId, coedge.id, coedge.faceId});
                }
                if (!supportedSegmentationFamily(*face,
                                                 GeometryTaxonomy::Surface)) {
                    return buildFailure(
                        report, "boundary.critical_segmentation_unsupported",
                        "degenerate singular stations require a cone or sphere face",
                        {edgeId, coedge.id, coedge.faceId});
                }
                if (!isSingularAnalyticFace(*face)) {
                    return buildFailure(
                        report, "boundary.degenerate_face_unsupported",
                        "degenerate singular edges are only certified on cone or sphere faces",
                        {edgeId, coedge.id, coedge.faceId});
                }
                const auto periods = periodsFor(*face);
                if (coedge.pcurveRepresentations.empty()) {
                    return buildFailure(
                        report, "boundary.pcurve_missing",
                        "degenerate cone apex coedges require a stored p-curve",
                        {edgeId, coedge.id, coedge.faceId});
                }
                for (const PcurveRef& representation :
                     coedge.pcurveRepresentations) {
                    mappings.push_back(
                        {&coedge, face, representation,
                         BoundaryUvMappingKind::StoredPcurve, periods});
                }
            }
            if (mappings.empty()) {
                return buildFailure(report, "boundary.edge_has_no_coedge",
                                    "working edge has no owning coedge UV use",
                                    {edgeId});
            }

            const ExactGeometryClassification* curveClassification =
                reconnaissance.find(edgeId);
            const bool degenerateCurveOk =
                curveClassification &&
                (supportedSegmentationFamily(*curveClassification,
                                             GeometryTaxonomy::Curve) ||
                 (std::find(curveClassification->conditionCodes.begin(),
                            curveClassification->conditionCodes.end(),
                            "degenerate") !=
                      curveClassification->conditionCodes.end() &&
                  (curveClassification->familyCode == "circle" ||
                   curveClassification->familyCode == "line" ||
                   curveClassification->familyCode == "kernel_specific")));
            if (!degenerateCurveOk) {
                return buildFailure(
                    report, "boundary.critical_segmentation_unsupported",
                    "degenerate singular edges require a classified curve family",
                    {edgeId});
            }

            report.expectedSamples += 1;
            report.expectedUvUses += mappings.size();
            report.expectedCriticalEvents += 1;
            report.expectedVertexCurveChecks += 1;

            CanonicalBoundary boundary;
            boundary.edge = edgeId;
            boundary.boundaryId = boundaryId;
            boundary.closed = true;
            boundary.intervalCount = 1;

            CriticalParameterEvent apexEvent;
            apexEvent.kind = CriticalParameterEventKind::Singular;
            apexEvent.edge = edgeId;
            apexEvent.sourceEdge = sourceEdge;
            apexEvent.detectionCode =
                mappings.front().faceClassification->familyCode == "sphere"
                ? "event.sphere_pole"
                : "event.cone_apex";
            apexEvent.curveParameter = 0.0;
            apexEvent.sampleOrdinal = 0;
            if (!mappings.empty()) {
                apexEvent.coedge = mappings.front().coedge->id;
                apexEvent.face = mappings.front().coedge->faceId;
                apexEvent.sourceFace = sourceForWorking(
                    imported, mappings.front().coedge->faceId);
            }
            boundary.criticalEvents.push_back(apexEvent);

            CanonicalBoundarySample sample;
            sample.id = {boundaryId, 0};
            sample.workingEdge = edgeId;
            sample.sourceEdge = sourceEdge;
            sample.curveParameter = 0.0;
            sample.position = apex.value->position;
            sample.canonicalVertexIndex = apexVertex.ordinal - 1U;
            stabilizeCoordinates(sample.position);
            ++report.checkedVertexCurveChecks;

            for (MappingState& mapping : mappings) {
                CoedgeUvUse use;
                use.face = mapping.coedge->faceId;
                use.sourceFace =
                    sourceForWorking(imported, mapping.coedge->faceId);
                if (!use.sourceFace ||
                    use.sourceFace->kind != StableIdKind::Face) {
                    return buildFailure(
                        report, "boundary.source_provenance_missing",
                        "working face does not resolve to one source face",
                        {edgeId, mapping.coedge->faceId});
                }
                use.coedge = mapping.coedge->id;
                use.representation = mapping.representation;
                use.mappingKind = mapping.kind;
                use.traversalOrientation = mapping.coedge->orientation;
                // Degenerate p-curves may refuse midpoint evaluation; fall
                // back to the singular UV corner of the cone face domain.
                bool haveUv = false;
                if (mapping.representation) {
                    const auto domain =
                        imported.workingEvaluator->curveDomain(edgeId);
                    if (domain && domain.value->lower) {
                        const auto composed =
                            imported.workingEvaluator->evaluateCurveOnSurface(
                                *mapping.representation, *domain.value->lower);
                        if (composed) {
                            use.uv = composed.value->uv;
                            use.measuredCurveOnSurfaceDiscrepancy =
                                composed.value->discrepancy;
                            haveUv = true;
                        }
                    }
                }
                if (!haveUv) {
                    if (mapping.faceClassification->parameterDomains.size() <
                        2) {
                        return buildFailure(
                            report, "boundary.degenerate_uv_unavailable",
                            "cone apex UV could not be recovered from p-curve or face domain",
                            {edgeId, mapping.coedge->faceId});
                    }
                    const ParameterDomain& uDomain =
                        mapping.faceClassification->parameterDomains[0];
                    const ParameterDomain& vDomain =
                        mapping.faceClassification->parameterDomains[1];
                    if (!uDomain.lower || !vDomain.lower || !vDomain.upper) {
                        return buildFailure(
                            report, "boundary.degenerate_uv_unavailable",
                            "cone apex UV domain bounds are incomplete",
                            {edgeId, mapping.coedge->faceId});
                    }
                    use.uv = {*uDomain.lower, *vDomain.upper};
                    use.measuredCurveOnSurfaceDiscrepancy = 0.0;
                }
                // Prefer the V domain end whose surface image matches the apex.
                if (mapping.faceClassification->parameterDomains.size() >= 2 &&
                    mapping.faceClassification->parameterDomains[1].lower &&
                    mapping.faceClassification->parameterDomains[1].upper) {
                    const double v0 =
                        *mapping.faceClassification->parameterDomains[1].lower;
                    const double v1 =
                        *mapping.faceClassification->parameterDomains[1].upper;
                    double best = std::numeric_limits<double>::infinity();
                    double chosenV = use.uv[1];
                    for (double candidateV : {v0, v1}) {
                        const auto at =
                            imported.workingEvaluator->evaluateSurface(
                                mapping.coedge->faceId,
                                {use.uv[0], candidateV});
                        if (!at) continue;
                        const double d = (at.value->position[0] -
                                          sample.position[0]) *
                                (at.value->position[0] - sample.position[0]) +
                            (at.value->position[1] - sample.position[1]) *
                                (at.value->position[1] - sample.position[1]) +
                            (at.value->position[2] - sample.position[2]) *
                                (at.value->position[2] - sample.position[2]);
                        if (d < best) {
                            best = d;
                            chosenV = candidateV;
                        }
                    }
                    use.uv[1] = chosenV;
                }
                const double sourceEdgeTolerance =
                    occurrenceTolerance(imported.source->snapshot, *sourceEdge);
                const double sourceVertexTolerance = occurrenceTolerance(
                    imported.source->snapshot, *sourceVertex);
                const double sourceEnvelope =
                    configuration.sourceToleranceScale *
                    (sourceEdgeTolerance + sourceVertexTolerance +
                     occurrenceTolerance(imported.source->snapshot,
                                         *use.sourceFace));
                const double allowed = std::max(
                    configuration.minimumDiscrepancyTolerance, sourceEnvelope);
                const double maxAllowed =
                    discrepancyCap(configuration, mapping.faceClassification);
                if (!std::isfinite(allowed) || allowed > maxAllowed) {
                    return buildFailure(
                        report, "boundary.source_tolerance_unbounded",
                        "source tolerance envelope exceeds the canonical-boundary cap",
                        {edgeId, mapping.coedge->faceId});
                }
                use.allowedCurveOnSurfaceDiscrepancy = allowed;
                use.liftedUv = use.uv;
                if (mapping.periods[0]) {
                    // Keep the principal period lift at zero for a point.
                    use.liftedUv[0] = use.uv[0];
                }
                stabilizeCoordinates(use.uv);
                stabilizeCoordinates(use.liftedUv);
                sample.faceUses.push_back(std::move(use));
                ++report.checkedUvUses;
            }

            boundary.samples.push_back(std::move(sample));
            ++report.checkedSamples;
            ++report.checkedCriticalEvents;
            ++report.checkedEdges;
            set.boundaries.push_back(std::move(boundary));
            continue;
        }
        const auto domain = imported.workingEvaluator->curveDomain(edgeId);
        if (!domain || !domain.value->lower || !domain.value->upper) {
            return buildFailure(
                report, "boundary.curve_domain_invalid",
                "working edge has no finite increasing exact curve domain",
                {edgeId});
        }
        const double lower = *domain.value->lower;
        const double upper = *domain.value->upper;
        const double span = upper - lower;
        if (!std::isfinite(span) || !(span > 0.0)) {
            return buildFailure(report, "boundary.curve_domain_invalid",
                                "working edge parameter span is not positive",
                                {edgeId});
        }
        // Curve periodicity describes the supporting curve, not the trimmed
        // B-rep edge. A partial circular arc still reports a periodic circle;
        // only topological endpoint identity makes this edge sequence closed.
        const bool closed = topology.lowerVertex && topology.upperVertex &&
            *topology.lowerVertex == *topology.upperVertex;
        if (!closed && (!topology.lowerVertex || !topology.upperVertex)) {
            return buildFailure(
                report, "boundary.open_edge_endpoint_missing",
                "an open working edge must resolve both topological endpoints",
                {edgeId});
        }
        report.expectedVertexCurveChecks += closed ? 1U : 2U;

        const std::optional<StableId> sourceEdge =
            sourceForWorking(imported, edgeId);
        if (!sourceEdge || sourceEdge->kind != StableIdKind::Edge) {
            return buildFailure(
                report, "boundary.source_provenance_missing",
                "working edge does not resolve to one source edge", {edgeId});
        }

        std::vector<MappingState> mappings;
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            if (coedge.edgeId != edgeId) continue;
            const ExactGeometryClassification* face =
                reconnaissance.find(coedge.faceId);
            if (!face || face->taxonomy != GeometryTaxonomy::Surface) {
                return buildFailure(
                    report, "boundary.face_classification_missing",
                    "an owning coedge has no surface classification",
                    {edgeId, coedge.id, coedge.faceId});
            }
            const auto periods = periodsFor(*face);
            if (coedge.pcurveRepresentations.empty()) {
                const bool analyticDerivable =
                    (face->familyCode == "plane" ||
                     face->familyCode == "cylinder" ||
                     face->familyCode == "cone" ||
                     face->familyCode == "sphere" ||
                     face->familyCode == "torus") &&
                    (face->support ==
                         GeometrySupportState::SupportedAnalyticTemplate ||
                     face->support ==
                         GeometrySupportState::DeferredResidualSurface);
                if (!analyticDerivable) {
                    return buildFailure(
                        report, "boundary.pcurve_missing",
                        "only proven analytic surfaces may derive UV without a stored p-curve",
                        {edgeId, coedge.id, coedge.faceId});
                }
                mappings.push_back(
                    {&coedge, face, std::nullopt,
                     BoundaryUvMappingKind::DerivedPlanarProjection, periods});
                continue;
            }
            for (const PcurveRef& representation :
                 coedge.pcurveRepresentations) {
                mappings.push_back(
                    {&coedge, face, representation,
                     BoundaryUvMappingKind::StoredPcurve, periods});
            }
        }
        if (mappings.empty()) {
            return buildFailure(report, "boundary.edge_has_no_coedge",
                                "working edge has no owning coedge UV use",
                                {edgeId});
        }

        const ExactGeometryClassification* curveClassification =
            reconnaissance.find(edgeId);
        if (!curveClassification ||
            !supportedSegmentationFamily(*curveClassification,
                                         GeometryTaxonomy::Curve)) {
            return buildFailure(
                report, "boundary.critical_segmentation_unsupported",
                "critical segmentation supports only exact line and circle edges",
                {edgeId});
        }
        for (const MappingState& mapping : mappings) {
            if (!supportedSegmentationFamily(*mapping.faceClassification,
                                             GeometryTaxonomy::Surface)) {
                return buildFailure(
                    report, "boundary.critical_segmentation_unsupported",
                    "critical segmentation supports only exact plane, cylinder, and cone faces",
                    {edgeId, mapping.coedge->id, mapping.coedge->faceId});
            }
        }

        CriticalSegmentationResult critical =
            collectSupportedCriticalEvents(
                imported, *curveClassification, topology, mappings, sourceEdge,
                lower, upper, closed, span);
        if (critical.failure) {
            return buildFailure(report, critical.failure->code,
                                critical.failure->message,
                                critical.failure->subjects);
        }
        if (critical.events.empty()) {
            return buildFailure(
                report, "boundary.critical_segmentation_unsupported",
                "supported critical segmentation produced no events",
                {edgeId});
        }

        const std::vector<double> parameters = mergeCriticalParameters(
            lower, upper, closed, span, *count, critical.events);
        if (parameters.empty()) {
            return buildFailure(
                report, "boundary.critical_segmentation_unsupported",
                "critical segmentation produced an empty sample parameter set",
                {edgeId});
        }

        const std::size_t sampleCount = parameters.size();
        report.expectedSamples += sampleCount;
        report.expectedUvUses += sampleCount * mappings.size();
        report.expectedCriticalEvents += critical.events.size();

        CanonicalBoundary boundary;
        boundary.edge = edgeId;
        boundary.boundaryId = boundaryId;
        boundary.closed = closed;
        boundary.intervalCount = *count;
        boundary.samples.reserve(sampleCount);
        boundary.criticalEvents = std::move(critical.events);

        const double sourceEdgeTolerance =
            occurrenceTolerance(imported.source->snapshot, *sourceEdge);
        for (std::size_t sampleIndex = 0; sampleIndex < sampleCount;
             ++sampleIndex) {
            const double parameter = parameters[sampleIndex];
            const auto curve =
                imported.workingEvaluator->evaluateCurve(edgeId, parameter);
            if (!curve) {
                return buildFailure(
                    report, "boundary.curve_evaluation_failed",
                    "canonical sample did not evaluate on the exact 3D curve",
                    {edgeId});
            }

            CanonicalBoundarySample sample;
            sample.id = {boundaryId,
                         static_cast<std::uint32_t>(sampleIndex)};
            sample.workingEdge = edgeId;
            sample.sourceEdge = sourceEdge;
            sample.curveParameter = parameter;
            sample.position = curve.value->position;
            std::optional<StableId> endpointVertex;
            if (almostEqualParameter(parameter, lower, span) &&
                topology.lowerVertex) {
                endpointVertex = topology.lowerVertex;
                sample.canonicalVertexIndex = topology.lowerVertex->ordinal - 1U;
            } else if (!closed && almostEqualParameter(parameter, upper, span) &&
                       topology.upperVertex) {
                endpointVertex = topology.upperVertex;
                sample.canonicalVertexIndex = topology.upperVertex->ordinal - 1U;
            } else {
                sample.canonicalVertexIndex = nextInteriorVertex++;
            }
            double sourceVertexTolerance = 0.0;
            if (endpointVertex) {
                const auto vertex =
                    imported.workingEvaluator->evaluateVertex(*endpointVertex);
                const std::optional<StableId> sourceVertex =
                    sourceForWorking(imported, *endpointVertex);
                if (!vertex || !sourceVertex ||
                    sourceVertex->kind != StableIdKind::Vertex) {
                    return buildFailure(
                        report, "boundary.vertex_evaluation_failed",
                        "a canonical endpoint does not resolve through its exact working and source vertices",
                        {edgeId, *endpointVertex});
                }
                sourceVertexTolerance = occurrenceTolerance(
                    imported.source->snapshot, *sourceVertex);
                const auto workingEdgeShape =
                    imported.working->snapshot.topology.exactShapes.find(
                        edgeId);
                const double workingEdgeTolerance =
                    workingEdgeShape ==
                            imported.working->snapshot.topology.exactShapes
                                .end()
                        ? sourceEdgeTolerance
                        : BRep_Tool::Tolerance(
                              TopoDS::Edge(workingEdgeShape->second));
                // Plasticity freeform extracts often need the working edge
                // tolerance in the envelope (source alone is too tight).
                double endpointAllowed = std::max(
                    configuration.minimumDiscrepancyTolerance,
                    configuration.sourceToleranceScale *
                        (sourceEdgeTolerance + sourceVertexTolerance +
                         workingEdgeTolerance));
                const ExactGeometryClassification* freeformOwner = nullptr;
                for (const MappingState& mapping : mappings) {
                    if (mapping.faceClassification &&
                        hasCondition(*mapping.faceClassification,
                                     "freeform.uv_grid_candidate")) {
                        freeformOwner = mapping.faceClassification;
                        break;
                    }
                }
                const double maxAllowed =
                    discrepancyCap(configuration, freeformOwner);
                endpointAllowed = std::min(endpointAllowed, maxAllowed);
                if (freeformOwner) {
                    endpointAllowed = std::max(endpointAllowed,
                                               workingEdgeTolerance * 10.0);
                    endpointAllowed = std::min(endpointAllowed, maxAllowed);
                }
                const double endpointDiscrepancy = norm(subtract(
                    vectorOf(curve.value->position),
                    vectorOf(vertex.value->position)));
                if (!std::isfinite(endpointAllowed) ||
                    endpointAllowed > maxAllowed ||
                    !std::isfinite(endpointDiscrepancy) ||
                    endpointDiscrepancy > endpointAllowed) {
                    return buildFailure(
                        report, "boundary.vertex_curve_discrepancy",
                        "an exact B-rep vertex lies outside its source edge tolerance envelope",
                        {edgeId, *endpointVertex, *sourceVertex});
                }
                // One authoritative topological vertex position is shared by
                // every incident edge sample. Curve evaluations remain the
                // evidence used to prove that this normalization is bounded.
                sample.position = vertex.value->position;
                ++report.checkedVertexCurveChecks;
            }
            sample.faceUses.reserve(mappings.size());

            for (MappingState& mapping : mappings) {
                CoedgeUvUse use;
                use.face = mapping.coedge->faceId;
                use.sourceFace =
                    sourceForWorking(imported, mapping.coedge->faceId);
                if (!use.sourceFace ||
                    use.sourceFace->kind != StableIdKind::Face) {
                    return buildFailure(
                        report, "boundary.source_provenance_missing",
                        "working face does not resolve to one source face",
                        {edgeId, mapping.coedge->faceId});
                }
                use.coedge = mapping.coedge->id;
                use.representation = mapping.representation;
                use.mappingKind = mapping.kind;
                use.traversalOrientation = mapping.coedge->orientation;
                if (mapping.kind == BoundaryUvMappingKind::StoredPcurve) {
                    const auto composed =
                        imported.workingEvaluator->evaluateCurveOnSurface(
                            *mapping.representation, parameter);
                    if (!composed) {
                        return buildFailure(
                            report, "boundary.curve_on_surface_evaluation_failed",
                            "stored p-curve did not compose with the exact edge and surface",
                            {edgeId, mapping.coedge->id,
                             mapping.coedge->faceId});
                    }
                    use.uv = composed.value->uv;
                    use.measuredCurveOnSurfaceDiscrepancy =
                        composed.value->discrepancy;
                } else {
                    const auto projected =
                        imported.workingEvaluator->projectPointToSurface(
                            mapping.coedge->faceId, sample.position);
                    if (!projected) {
                        return buildFailure(
                            report, "boundary.surface_projection_failed",
                            "derived analytic boundary UV did not evaluate",
                            {edgeId, mapping.coedge->id,
                             mapping.coedge->faceId});
                    }
                    use.uv = projected.value->uv;
                    use.measuredCurveOnSurfaceDiscrepancy =
                        projected.value->discrepancy;
                }

                const double sourceEnvelope =
                    configuration.sourceToleranceScale *
                    (sourceEdgeTolerance + sourceVertexTolerance +
                     occurrenceTolerance(imported.source->snapshot,
                                         *use.sourceFace));
                const double allowed = std::max(
                    configuration.minimumDiscrepancyTolerance,
                    sourceEnvelope);
                const double maxAllowed =
                    discrepancyCap(configuration, mapping.faceClassification);
                if (!std::isfinite(allowed) || allowed > maxAllowed) {
                    return buildFailure(
                        report, "boundary.source_tolerance_unbounded",
                        "source tolerance envelope exceeds the canonical-boundary cap",
                        {edgeId, mapping.coedge->faceId});
                }
                if (!std::isfinite(
                        use.measuredCurveOnSurfaceDiscrepancy) ||
                    use.measuredCurveOnSurfaceDiscrepancy > allowed) {
                    return buildFailure(
                        report, "boundary.curve_on_surface_discrepancy",
                        "curve-on-surface discrepancy exceeds the bounded source envelope",
                        {edgeId, mapping.coedge->id,
                         mapping.coedge->faceId});
                }
                use.allowedCurveOnSurfaceDiscrepancy = allowed;

                for (std::size_t axis = 0; axis < use.uv.size(); ++axis) {
                    std::int64_t lift = 0;
                    if (mapping.periods[axis] && mapping.hasPrevious) {
                        const double period = *mapping.periods[axis];
                        const double requested =
                            (mapping.previousLifted[axis] - use.uv[axis]) /
                            period;
                        if (!std::isfinite(requested) ||
                            std::abs(requested) >
                                static_cast<double>(
                                    std::numeric_limits<std::int64_t>::max() /
                                    2)) {
                            return buildFailure(
                                report, "boundary.periodic_lift_overflow",
                                "periodic UV lift cannot be represented",
                                {edgeId, mapping.coedge->faceId});
                        }
                        if (ambiguousIntegerPeriod(
                                requested,
                                configuration.periodicLiftAmbiguityTolerance)) {
                            return buildFailure(
                                report, "boundary.periodic_lift_ambiguous",
                                "periodic UV lift is not uniquely determined",
                                {edgeId, mapping.coedge->id,
                                 mapping.coedge->faceId});
                        }
                        lift = static_cast<std::int64_t>(std::llround(requested));
                    }
                    use.periodicLift[axis] = lift;
                    use.liftedUv[axis] = use.uv[axis] +
                        static_cast<double>(lift) *
                            mapping.periods[axis].value_or(0.0);
                    if (mapping.periods[axis] && mapping.hasPrevious) {
                        const double step = std::abs(
                            use.liftedUv[axis] - mapping.previousLifted[axis]);
                        if (!(step < 0.5 * *mapping.periods[axis])) {
                            return buildFailure(
                                report, "boundary.periodic_lift_discontinuous",
                                "periodic UV lift jumps by half a period or more",
                                {edgeId, mapping.coedge->id,
                                 mapping.coedge->faceId});
                        }
                    }
                    mapping.previousLifted[axis] = use.liftedUv[axis];
                }
                mapping.hasPrevious = true;
                sample.faceUses.push_back(std::move(use));
                ++report.checkedUvUses;
            }
            stabilizeCoordinates(sample.position);
            for (CoedgeUvUse& use : sample.faceUses) {
                stabilizeCoordinates(use.uv);
                stabilizeCoordinates(use.liftedUv);
            }
            boundary.samples.push_back(std::move(sample));
            ++report.checkedSamples;
        }

        for (CriticalParameterEvent& event : boundary.criticalEvents) {
            const auto found = std::find_if(
                boundary.samples.begin(), boundary.samples.end(),
                [&](const CanonicalBoundarySample& sample) {
                    return almostEqualParameter(
                        sample.curveParameter, event.curveParameter, span);
                });
            if (found == boundary.samples.end()) {
                return buildFailure(
                    report, "boundary.critical_event_missing",
                    "a required critical parameter event has no sample",
                    {edgeId});
            }
            event.sampleOrdinal = found->id.ordinal;
            ++report.checkedCriticalEvents;
        }
        for (std::size_t left = 0; left < boundary.criticalEvents.size();
             ++left) {
            for (std::size_t right = left + 1;
                 right < boundary.criticalEvents.size(); ++right) {
                if (criticalEventSameKey(boundary.criticalEvents[left],
                                         boundary.criticalEvents[right])) {
                    return buildFailure(
                        report, "boundary.critical_event_duplicate",
                        "critical parameter events must not be double-counted",
                        {edgeId});
                }
            }
        }

        if (closed) {
            for (const MappingState& mapping : mappings) {
                for (std::size_t axis = 0; axis < mapping.periods.size();
                     ++axis) {
                    if (!mapping.periods[axis]) continue;
                    ++report.expectedPeriodicClosures;
                    const CoedgeUvUse* firstUse = findUse(
                        boundary.samples.front(), mapping.coedge->id,
                        mapping.representation, mapping.kind);
                    const CoedgeUvUse* lastUse = findUse(
                        boundary.samples.back(), mapping.coedge->id,
                        mapping.representation, mapping.kind);
                    if (!firstUse || !lastUse) {
                        return buildFailure(
                            report, "boundary.periodic_closure_use_missing",
                            "closed periodic coedge is missing endpoint UV uses",
                            {edgeId, mapping.coedge->id,
                             mapping.coedge->faceId});
                    }
                    PeriodicUvClosureWitness seed;
                    seed.edge = edgeId;
                    seed.coedge = mapping.coedge->id;
                    seed.face = mapping.coedge->faceId;
                    seed.sourceEdge = sourceEdge;
                    seed.sourceFace = firstUse->sourceFace;
                    seed.traversalOrientation = mapping.coedge->orientation;
                    seed.axis = axis;
                    seed.period = *mapping.periods[axis];
                    seed.firstUv = firstUse->uv[axis];
                    seed.lastUv = lastUse->uv[axis];
                    seed.firstLiftedUv = firstUse->liftedUv[axis];
                    seed.lastLiftedUv = lastUse->liftedUv[axis];
                    seed.selectedFirstLift = firstUse->periodicLift[axis];
                    seed.selectedLastLift = lastUse->periodicLift[axis];
                    const PeriodicUvClosureResult closure =
                        provePeriodicUvClosure(
                            std::move(seed),
                            configuration.periodicLiftAmbiguityTolerance);
                    if (!closure) {
                        return buildFailure(
                            report,
                            closure.failure
                                ? closure.failure->code
                                : "boundary.periodic_closure_failed",
                            closure.failure
                                ? closure.failure->message
                                : "periodic UV closure could not be proved",
                            closure.failure ? closure.failure->subjects
                                            : std::vector<StableId>{edgeId});
                    }
                    boundary.periodicClosures.push_back(*closure.value);
                    ++report.checkedPeriodicClosures;
                }
            }
        }
        set.boundaries.push_back(std::move(boundary));
        ++report.checkedEdges;
    }

    set.validation = report;
    set.canonicalVertexCount = nextInteriorVertex;
    if (!report.complete()) {
        return buildFailure(
            report, "boundary.validation_incomplete",
            "canonical-boundary validation coverage is incomplete");
    }
    CanonicalBoundaryBuildResult result;
    result.validation = report;
    result.value = std::move(set);
    return result;
}

AzimuthRegistrationResult azimuthRegistration(
    const std::vector<std::array<double, 3>>& ringA,
    const std::vector<std::array<double, 3>>& ringB) {
    const std::size_t count = ringA.size();
    if (count < 3 || ringB.size() != count) {
        return registrationFailure(
            "boundary.azimuth_incompatible",
            "rings must have the same non-trivial sample count");
    }
    const Vector3 centreA = centroid(ringA);
    const Vector3 centreB = centroid(ringB);
    Vector3 axis = subtract(centreB, centreA);
    const double axisLength = norm(axis);
    if (!std::isfinite(axisLength) || !(axisLength > 0.0)) {
        return registrationFailure(
            "boundary.azimuth_incompatible",
            "ring centres coincide, so no cylinder axis exists");
    }
    axis = {axis.x / axisLength, axis.y / axisLength, axis.z / axisLength};
    const Vector3 radial = subtract(vectorOf(ringA.front()), centreA);
    const double axial = dot(radial, axis);
    Vector3 reference =
        subtract(radial, {axis.x * axial, axis.y * axial, axis.z * axial});
    const double referenceLength = norm(reference);
    if (!std::isfinite(referenceLength) || !(referenceLength > 0.0)) {
        return registrationFailure(
            "boundary.azimuth_incompatible",
            "a ring sample lies on the axis, so no azimuth frame exists");
    }
    reference = {reference.x / referenceLength, reference.y / referenceLength,
                 reference.z / referenceLength};
    const Vector3 side = cross(axis, reference);
    const auto azimuth = [&](const std::array<double, 3>& point,
                             Vector3 centre) {
        const Vector3 radialPoint = subtract(vectorOf(point), centre);
        return std::atan2(dot(radialPoint, side),
                          dot(radialPoint, reference));
    };

    std::vector<double> anglesA(count);
    std::vector<double> anglesB(count);
    for (std::size_t index = 0; index < count; ++index) {
        anglesA[index] = azimuth(ringA[index], centreA);
        anglesB[index] = azimuth(ringB[index], centreB);
    }
    const double signA =
        wrapAngle(anglesA[1] - anglesA[0]) >= 0.0 ? 1.0 : -1.0;
    const double signB =
        wrapAngle(anglesB[1] - anglesB[0]) >= 0.0 ? 1.0 : -1.0;
    // Open cylinder bands (MP9) often traverse rims with opposite winding
    // relative to the axis. Align by reversing ring B's azimuth sequence.
    const bool reflected = signA != signB;
    if (reflected) {
        std::reverse(anglesB.begin(), anglesB.end());
    }

    double minimumStep = kTwoPi;
    for (std::size_t index = 0; index < count; ++index) {
        const double step = std::abs(wrapAngle(
            anglesA[(index + 1U) % count] - anglesA[index]));
        if (std::isfinite(step) && step > 0.0) {
            minimumStep = std::min(minimumStep, step);
        }
    }
    if (!(minimumStep > 0.0) || !std::isfinite(minimumStep)) {
        return registrationFailure(
            "boundary.azimuth_incompatible",
            "ring azimuth steps are not finite and positive");
    }
    const double tolerance = minimumStep / 4.0;

    double bestError = std::numeric_limits<double>::infinity();
    std::size_t bestOffset = 0;
    for (std::size_t offset = 0; offset < count; ++offset) {
        double maxError = 0.0;
        for (std::size_t index = 0; index < count; ++index) {
            const std::size_t indexB = (index + offset) % count;
            maxError = std::max(
                maxError,
                std::abs(wrapAngle(anglesA[index] - anglesB[indexB])));
        }
        if (maxError < bestError) {
            bestError = maxError;
            bestOffset = offset;
        }
    }
    if (!(bestError <= tolerance)) {
        return registrationFailure(
            "boundary.azimuth_twist",
            "rings are twisted or otherwise not registrable by a cyclic rotation");
    }

    std::vector<std::uint32_t> permutation(count);
    for (std::size_t index = 0; index < count; ++index) {
        std::size_t indexB = (index + bestOffset) % count;
        if (reflected) {
            indexB = count - 1U - indexB;
        }
        permutation[index] = static_cast<std::uint32_t>(indexB);
    }
    AzimuthRegistrationResult result;
    result.permutation = std::move(permutation);
    return result;
}

PeriodicUvClosureResult provePeriodicUvClosure(
    PeriodicUvClosureWitness seed, double ambiguityTolerance) {
    PeriodicUvClosureResult result;
    const auto fail = [&](std::string code, std::string message) {
        result.failure = CanonicalBoundaryFailure{
            std::move(code), std::move(message),
            {seed.edge, seed.coedge, seed.face}};
        return result;
    };
    if (!(ambiguityTolerance >= 0.0) || !(ambiguityTolerance < 0.5) ||
        !std::isfinite(ambiguityTolerance)) {
        return fail("boundary.periodic_closure_invalid_tolerance",
                    "periodic closure ambiguity tolerance is invalid");
    }
    if (!std::isfinite(seed.period) || !(seed.period > 0.0) ||
        !std::isfinite(seed.firstUv) || !std::isfinite(seed.lastUv) ||
        !std::isfinite(seed.firstLiftedUv) ||
        !std::isfinite(seed.lastLiftedUv)) {
        return fail("boundary.periodic_closure_invalid_domain",
                    "periodic closure inputs are not finite positive-period UV values");
    }

    const double firstLiftAmount =
        (seed.firstLiftedUv - seed.firstUv) / seed.period;
    if (!std::isfinite(firstLiftAmount) ||
        ambiguousIntegerPeriod(firstLiftAmount, ambiguityTolerance) ||
        std::abs(firstLiftAmount - std::round(firstLiftAmount)) >
            ambiguityTolerance) {
        return fail("boundary.periodic_lift_inconsistent",
                    "first sample lifted UV is not an integer period from raw UV");
    }
    const auto firstLift =
        static_cast<std::int64_t>(std::llround(firstLiftAmount));
    if (seed.selectedFirstLift != firstLift) {
        return fail("boundary.periodic_lift_inconsistent",
                    "recorded first periodic lift does not match lifted UV");
    }

    const double lastLiftAmount =
        (seed.lastLiftedUv - seed.lastUv) / seed.period;
    if (!std::isfinite(lastLiftAmount) ||
        std::abs(lastLiftAmount - std::round(lastLiftAmount)) >
            ambiguityTolerance) {
        return fail("boundary.periodic_lift_inconsistent",
                    "last sample lifted UV is not an integer period from raw UV");
    }
    const auto lastLift =
        static_cast<std::int64_t>(std::llround(lastLiftAmount));
    if (seed.selectedLastLift != lastLift) {
        return fail("boundary.periodic_lift_inconsistent",
                    "recorded last periodic lift does not match lifted UV");
    }

    const double requestedClosingPeriods =
        (seed.lastLiftedUv - seed.firstUv) / seed.period;
    if (!std::isfinite(requestedClosingPeriods) ||
        std::abs(requestedClosingPeriods) >
            static_cast<double>(std::numeric_limits<std::int64_t>::max() / 2)) {
        return fail("boundary.periodic_closure_overflow",
                    "periodic UV closure lift cannot be represented");
    }
    if (ambiguousIntegerPeriod(requestedClosingPeriods, ambiguityTolerance)) {
        return fail("boundary.periodic_closure_ambiguous",
                    "periodic UV closure lift is not uniquely determined");
    }
    const auto closingPeriods =
        static_cast<std::int64_t>(std::llround(requestedClosingPeriods));
    const double closingLifted =
        seed.firstUv + static_cast<double>(closingPeriods) * seed.period;
    const double closingDelta = closingLifted - seed.lastLiftedUv;
    if (!std::isfinite(closingDelta) ||
        !(std::abs(closingDelta) < 0.5 * seed.period)) {
        return fail("boundary.periodic_closure_discontinuous",
                    "closed periodic UV does not return through a continuous covering-space step");
    }

    // Reject a tampered wrong integer wrap that stays continuous only because
    // an extra full period was added to the last sample: the closing image of
    // the first UV must equal the first lifted UV plus the loop winding.
    const std::int64_t periodsCrossed = closingPeriods - firstLift;
    const double reconstitutedFirst =
        seed.firstLiftedUv + static_cast<double>(periodsCrossed) * seed.period;
    if (std::abs(reconstitutedFirst - closingLifted) >
        ambiguityTolerance * seed.period) {
        return fail("boundary.periodic_closure_inconsistent",
                    "periodic closure winding does not match endpoint lifts");
    }

    seed.closingLiftedUv = closingLifted;
    seed.periodsCrossed = periodsCrossed;
    result.value = std::move(seed);
    return result;
}

}  // namespace weft
