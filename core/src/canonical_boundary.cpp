#include "weft/canonical_boundary.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace weft {
namespace {

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

AzimuthRegistrationResult registrationFailure(std::string message) {
    AzimuthRegistrationResult result;
    result.failure = CanonicalBoundaryFailure{
        "boundary.azimuth_registration_failed", std::move(message), {}};
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
        if (topology.degenerate) {
            return buildFailure(
                report, "boundary.degenerate_edge_unsupported",
                "degenerate singular edges require a dedicated canonical form",
                {edgeId});
        }
        const auto count = intervals.find(boundaryId);
        if (!count || *count == 0) {
            return buildFailure(
                report, "boundary.interval_count_missing",
                "every working edge requires a positive solved interval count",
                {edgeId, boundaryId});
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
                if (face->familyCode != "plane" ||
                    face->support !=
                        GeometrySupportState::SupportedAnalyticTemplate) {
                    return buildFailure(
                        report, "boundary.pcurve_missing",
                        "only a proven plane may derive UV without a stored p-curve",
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

        const std::size_t sampleCount = closed
            ? static_cast<std::size_t>(*count)
            : static_cast<std::size_t>(*count) + 1U;
        report.expectedSamples += sampleCount;
        report.expectedUvUses += sampleCount * mappings.size();

        CanonicalBoundary boundary;
        boundary.edge = edgeId;
        boundary.boundaryId = boundaryId;
        boundary.closed = closed;
        boundary.intervalCount = *count;
        boundary.samples.reserve(sampleCount);

        const double sourceEdgeTolerance =
            occurrenceTolerance(imported.source->snapshot, *sourceEdge);
        for (std::size_t sampleIndex = 0; sampleIndex < sampleCount;
             ++sampleIndex) {
            const double parameter =
                !closed && sampleIndex + 1U == sampleCount
                ? upper
                : lower + span * static_cast<double>(sampleIndex) /
                              static_cast<double>(*count);
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
            if (sampleIndex == 0 && topology.lowerVertex) {
                endpointVertex = topology.lowerVertex;
                sample.canonicalVertexIndex = topology.lowerVertex->ordinal - 1U;
            } else if (!closed && sampleIndex + 1U == sampleCount &&
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
                const double endpointAllowed = std::max(
                    configuration.minimumDiscrepancyTolerance,
                    configuration.sourceToleranceScale *
                        (sourceEdgeTolerance + sourceVertexTolerance));
                const double endpointDiscrepancy = norm(subtract(
                    vectorOf(curve.value->position),
                    vectorOf(vertex.value->position)));
                if (!std::isfinite(endpointAllowed) ||
                    endpointAllowed >
                        configuration.maximumDiscrepancyTolerance ||
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
                        imported.workingEvaluator->projectPointToPlane(
                            mapping.coedge->faceId, sample.position);
                    if (!projected) {
                        return buildFailure(
                            report, "boundary.planar_projection_failed",
                            "derived planar boundary UV did not evaluate",
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
                if (!std::isfinite(allowed) ||
                    allowed > configuration.maximumDiscrepancyTolerance) {
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
            boundary.samples.push_back(std::move(sample));
            ++report.checkedSamples;
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
            "rings must have the same non-trivial sample count");
    }
    const Vector3 centreA = centroid(ringA);
    const Vector3 centreB = centroid(ringB);
    Vector3 axis = subtract(centreB, centreA);
    const double axisLength = norm(axis);
    if (!std::isfinite(axisLength) || !(axisLength > 0.0)) {
        return registrationFailure(
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

    constexpr double twoPi = 6.283185307179586476925286766559;
    const double step = twoPi / static_cast<double>(count);
    const double signA = wrapAngle(azimuth(ringA[1], centreA)) >= 0.0
        ? 1.0
        : -1.0;
    const double phaseB = azimuth(ringB.front(), centreB);
    const double signB =
        wrapAngle(azimuth(ringB[1], centreB) - phaseB) >= 0.0
        ? 1.0
        : -1.0;
    if (signA != signB) {
        return registrationFailure(
            "ring winding differs; reflection is not a supported registration");
    }

    const long long offset =
        static_cast<long long>(std::llround(phaseB / (signA * step)));
    const long long modulo = static_cast<long long>(count);
    std::vector<std::uint32_t> permutation(count);
    for (std::size_t index = 0; index < count; ++index) {
        long long registered =
            (static_cast<long long>(index) - offset) % modulo;
        if (registered < 0) registered += modulo;
        permutation[index] = static_cast<std::uint32_t>(registered);
    }

    const double tolerance = step / 4.0;
    for (std::size_t index = 0; index < count; ++index) {
        const double expected = signA * static_cast<double>(index) * step;
        const double actualA = azimuth(ringA[index], centreA);
        const double actual =
            azimuth(ringB[permutation[index]], centreB);
        if (std::abs(wrapAngle(actualA - expected)) > tolerance ||
            std::abs(wrapAngle(actual - expected)) > tolerance) {
            return registrationFailure(
                "rings are not registrable by a cyclic rotation");
        }
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
