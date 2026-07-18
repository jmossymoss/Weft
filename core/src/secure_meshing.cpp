#include "weft/secure_meshing.hpp"

#include "weft/planar_trim_assembly.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace weft {
namespace {

void setFailure(SecureMeshingResult& result, std::string code,
                std::string message, std::vector<StableId> subjects = {}) {
    if (!result.failure) {
        result.failure = SecureMeshingFailure{
            std::move(code), std::move(message), std::move(subjects)};
    }
}

void appendCoverage(ValidationCertificate& certificate, std::string code,
                    std::size_t expected, std::size_t checked,
                    std::size_t skipped, std::size_t failed) {
    certificate.checks.push_back(
        {std::move(code), expected, checked, skipped, failed});
}

std::string faceCode(const std::string& code, StableId face) {
    return code + ".face_" + std::to_string(face.ordinal);
}

double vectorLength(const std::array<double, 3>& vector) {
    return std::hypot(vector[0], vector[1], vector[2]);
}

const EdgeTopologyRecord* edgeTopology(const BRepSnapshot& snapshot,
                                       StableId edge) {
    const auto found = std::find_if(
        snapshot.edgeTopology.begin(), snapshot.edgeTopology.end(),
        [edge](const EdgeTopologyRecord& record) {
            return record.id == edge;
        });
    return found == snapshot.edgeTopology.end() ? nullptr : &*found;
}

struct IntervalProblemResult {
    std::optional<IntervalProblem> value;
    std::optional<SecureMeshingFailure> failure;
};

IntervalProblemResult buildIntervalProblem(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const SecureMeshingConfiguration& configuration) {
    IntervalProblemResult result;
    if (configuration.cylinderAxialIntervals == 0) {
        result.failure = SecureMeshingFailure{
            "secure_pipeline.axial_count_invalid",
            "cylinder axial intervals must be positive", {}};
        return result;
    }
    IntervalProblem problem;
    const BRepSnapshot& snapshot = imported.working->snapshot;
    for (const auto& [edge, count] :
         configuration.exactEdgeIntervalCounts) {
        if (!edge.valid() || edge.kind != StableIdKind::Edge || count == 0 ||
            edge.ordinal >
                static_cast<std::uint64_t>(snapshot.model.edgeCount()) ||
            !edgeTopology(snapshot, edge)) {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.edge_constraint_invalid",
                "an exact edge count must name one existing working edge and be positive",
                {edge}};
            return result;
        }
    }
    std::set<StableId> cylinderAxialEdges;
    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy != GeometryTaxonomy::Surface ||
            face.familyCode != "cylinder") {
            continue;
        }
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            const ExactGeometryClassification* edge =
                reconnaissance.find(coedge.edgeId);
            if (coedge.faceId == face.subjectId && edge &&
                edge->familyCode == "line") {
                cylinderAxialEdges.insert(coedge.edgeId);
            }
        }
    }
    for (const EdgeTopologyRecord& topology : snapshot.edgeTopology) {
        const ExactGeometryClassification* classification =
            reconnaissance.find(topology.id);
        if (!classification ||
            classification->taxonomy != GeometryTaxonomy::Curve) {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.edge_classification_missing",
                "a working edge has no exact curve classification",
                {topology.id}};
            return result;
        }
        std::uint32_t count = 0;
        if (classification->familyCode == "line") {
            count = cylinderAxialEdges.contains(topology.id)
                ? configuration.cylinderAxialIntervals
                : lineSegmentCount();
        } else if (classification->familyCode == "circle") {
            const EvaluationResult<ParameterDomain> domain =
                imported.workingEvaluator->curveDomain(topology.id);
            if (!domain || !domain.value->lower ||
                !domain.value->upper) {
                result.failure = SecureMeshingFailure{
                    "secure_pipeline.circle_domain_invalid",
                    "a supported circular edge has no finite exact domain",
                    {topology.id}};
                return result;
            }
            const double midpoint =
                (*domain.value->lower + *domain.value->upper) * 0.5;
            const EvaluationResult<CurveEvaluation> evaluated =
                imported.workingEvaluator->evaluateCurve(topology.id,
                                                          midpoint);
            const double radius = evaluated
                ? vectorLength(evaluated.value->firstDerivative)
                : 0.0;
            const bool fullCircle = topology.lowerVertex &&
                topology.upperVertex &&
                *topology.lowerVertex == *topology.upperVertex;
            const SegmentCountResult demanded = circularArcSegmentCount(
                radius, *domain.value->upper - *domain.value->lower,
                fullCircle, configuration.sampling);
            if (!demanded) {
                result.failure = SecureMeshingFailure{
                    demanded.failure ? demanded.failure->code
                                     : "secure_pipeline.circle_count_failed",
                    demanded.failure
                        ? demanded.failure->message
                        : "a circular segment count could not be proven",
                    {topology.id}};
                return result;
            }
            count = *demanded.count;
        } else {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.unsupported_curve_family",
                "the secure automatic pipeline currently supports only line and circle edges",
                {topology.id}};
            return result;
        }
        const auto exact =
            configuration.exactEdgeIntervalCounts.find(topology.id);
        problem.variables.push_back(
            {{StableIdKind::Boundary, topology.id.ordinal},
             static_cast<double>(count), count,
             configuration.requireEvenEdgeIntervals.contains(topology.id),
             exact == configuration.exactEdgeIntervalCounts.end()
                 ? std::optional<std::uint32_t>{}
                 : std::optional<std::uint32_t>{exact->second}});
    }

    for (const EdgeIntervalChainSum& sum : configuration.edgeChainSums) {
        auto resolveSide = [&](const std::vector<StableId>& edges,
                               std::vector<StableId>& boundaries)
            -> std::optional<SecureMeshingFailure> {
            if (edges.empty()) {
                return SecureMeshingFailure{
                    "secure_pipeline.chain_sum_invalid",
                    "a chain-sum side must name at least one working edge",
                    {}};
            }
            for (const StableId& edge : edges) {
                if (!edge.valid() || edge.kind != StableIdKind::Edge ||
                    edge.ordinal >
                        static_cast<std::uint64_t>(
                            snapshot.model.edgeCount()) ||
                    !edgeTopology(snapshot, edge)) {
                    return SecureMeshingFailure{
                        "secure_pipeline.chain_sum_invalid",
                        "a chain-sum side must name existing working edges",
                        {edge}};
                }
                boundaries.push_back(
                    {StableIdKind::Boundary, edge.ordinal});
            }
            return std::nullopt;
        };
        IntervalSum intervalSum;
        if (const auto failure = resolveSide(sum.lhs, intervalSum.lhs)) {
            result.failure = failure;
            return result;
        }
        if (const auto failure = resolveSide(sum.rhs, intervalSum.rhs)) {
            result.failure = failure;
            return result;
        }
        problem.sums.push_back(std::move(intervalSum));
    }

    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy != GeometryTaxonomy::Surface ||
            face.familyCode != "cylinder") {
            continue;
        }
        const bool fullPeriodic = face.trimDomain ==
            TrimDomainClass::FullPeriodicWithCapBoundaries;
        const bool partialBand = face.trimDomain ==
            TrimDomainClass::PeriodicBandCrossingSeam;
        if (!fullPeriodic && !partialBand) continue;
        std::set<StableId> rimBoundaries;
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            if (coedge.faceId != face.subjectId) continue;
            const ExactGeometryClassification* edge =
                reconnaissance.find(coedge.edgeId);
            const EdgeTopologyRecord* topology =
                edgeTopology(snapshot, coedge.edgeId);
            if (!edge || !topology || edge->familyCode != "circle" ||
                !topology->lowerVertex || !topology->upperVertex) {
                continue;
            }
            const bool closedRim =
                *topology->lowerVertex == *topology->upperVertex;
            if ((fullPeriodic && closedRim) ||
                (partialBand && !closedRim)) {
                rimBoundaries.insert(
                    {StableIdKind::Boundary, coedge.edgeId.ordinal});
            }
        }
        if (rimBoundaries.size() != 2) {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.cylinder_rims_unresolved",
                fullPeriodic
                    ? "a full cylinder does not resolve exactly two circular rim boundaries"
                    : "a partial cylinder does not resolve exactly two open circular rim arcs",
                {face.subjectId}};
            return result;
        }
        problem.equalities.push_back(
            {{rimBoundaries.begin(), rimBoundaries.end()}});
    }
    result.value = std::move(problem);
    return result;
}

std::set<StableId> chainSumParticipantEdges(
    const std::vector<EdgeIntervalChainSum>& sums) {
    std::set<StableId> participants;
    for (const EdgeIntervalChainSum& sum : sums) {
        participants.insert(sum.lhs.begin(), sum.lhs.end());
        participants.insert(sum.rhs.begin(), sum.rhs.end());
    }
    return participants;
}

}  // namespace

std::optional<SecureMeshingFailure> certifySolvedIntervalConsumption(
    const IntervalSolution& solved,
    const CanonicalBoundarySet& boundaries,
    const MeshingResult* mesh) {
    if (solved.counts.empty()) {
        return SecureMeshingFailure{
            "secure_pipeline.interval_consumption_mismatch",
            "solved interval consumption requires a non-empty solution",
            {}};
    }
    for (const SolvedInterval& interval : solved.counts) {
        const CanonicalBoundary* boundary = nullptr;
        for (const CanonicalBoundary& candidate : boundaries.boundaries) {
            if (candidate.boundaryId == interval.boundaryId) {
                boundary = &candidate;
                break;
            }
        }
        if (!boundary) {
            return SecureMeshingFailure{
                "secure_pipeline.interval_consumption_mismatch",
                "a solved interval has no canonical boundary consumer",
                {interval.boundaryId}};
        }
        if (boundary->intervalCount != interval.count) {
            return SecureMeshingFailure{
                "secure_pipeline.interval_consumption_mismatch",
                "canonical boundary interval count does not match the solved count",
                {interval.boundaryId, boundary->edge}};
        }
        if (mesh) {
            const auto reported = mesh->generation.edgeDivisions.find(
                static_cast<int>(boundary->edge.ordinal));
            if (reported == mesh->generation.edgeDivisions.end() ||
                reported->second != static_cast<int>(interval.count)) {
                return SecureMeshingFailure{
                    "secure_pipeline.interval_consumption_mismatch",
                    "generation report count does not match the solved interval",
                    {interval.boundaryId, boundary->edge}};
            }
            std::set<std::uint32_t> consumedSamples;
            for (const CertifiedVertex& vertex : mesh->certified.vertices) {
                for (const CertifiedVertexUse& use : vertex.provenance) {
                    if (use.boundary.workingEdge == boundary->edge) {
                        consumedSamples.insert(use.boundary.sample.ordinal);
                    }
                }
            }
            if (consumedSamples.size() != boundary->samples.size()) {
                return SecureMeshingFailure{
                    "secure_pipeline.interval_consumption_mismatch",
                    "certified mesh sample provenance does not consume every boundary sample",
                    {interval.boundaryId, boundary->edge}};
            }
            for (const CanonicalBoundarySample& sample : boundary->samples) {
                if (!consumedSamples.contains(sample.id.ordinal)) {
                    return SecureMeshingFailure{
                        "secure_pipeline.interval_consumption_mismatch",
                        "a canonical sample is missing from certified mesh provenance",
                        {interval.boundaryId, boundary->edge}};
                }
            }
        }
    }
    return std::nullopt;
}

SecureMeshingResult generateSecureMesh(
    const ImportedModel& imported,
    const SecureMeshingConfiguration& configuration) {
    SecureMeshingResult result;
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator) {
        setFailure(result, "secure_pipeline.import_not_meshable",
                   "the audited working B-rep is not meshable");
        return result;
    }
    for (const RepairValidationEvidence& evidence :
         imported.repair.validationEvidence) {
        appendCoverage(result.validation, evidence.code, evidence.expected,
                       evidence.checked, evidence.skipped, evidence.failed);
    }

    const ReconnaissanceReport reconnaissance = reconnoitre(imported);
    appendCoverage(result.validation, "secure_pipeline.reconnaissance",
                   reconnaissance.expectedSubjects,
                   reconnaissance.checkedSubjects, 0,
                   reconnaissance.complete ? 0 : 1);
    if (!reconnaissance.complete) {
        setFailure(result, "secure_pipeline.reconnaissance_incomplete",
                   "exact topology and geometry reconnaissance is incomplete");
        return result;
    }

    const IntervalProblemResult intervalProblem = buildIntervalProblem(
        imported, reconnaissance, configuration);
    if (!intervalProblem.value) {
        result.failure = intervalProblem.failure;
        return result;
    }
    const IntervalSolveResult intervals = solveIntervals(
        *intervalProblem.value, configuration.sampling);
    if (!intervals) {
        setFailure(result,
                   intervals.failure ? intervals.failure->code
                                     : "secure_pipeline.interval_solve_failed",
                   intervals.failure
                       ? intervals.failure->message
                       : "the exact interval problem did not solve",
                   intervals.failure ? intervals.failure->subjects
                                     : std::vector<StableId>{});
        return result;
    }
    appendCoverage(result.validation, "secure_pipeline.interval_counts",
                   intervalProblem.value->variables.size(),
                   intervals.solution->counts.size(), 0, 0);
    if (!configuration.exactEdgeIntervalCounts.empty()) {
        appendCoverage(
            result.validation,
            "secure_pipeline.exact_edge_interval_constraints",
            configuration.exactEdgeIntervalCounts.size(),
            configuration.exactEdgeIntervalCounts.size(), 0, 0);
    }
    const std::set<StableId> chainParticipants =
        chainSumParticipantEdges(configuration.edgeChainSums);
    if (!configuration.edgeChainSums.empty()) {
        appendCoverage(result.validation,
                       "secure_pipeline.chain_sum_requested",
                       configuration.edgeChainSums.size(),
                       intervalProblem.value->sums.size(), 0, 0);
        std::size_t solvedParticipants = 0;
        for (const StableId& edge : chainParticipants) {
            const StableId boundary{StableIdKind::Boundary, edge.ordinal};
            if (intervals.solution->find(boundary)) {
                ++solvedParticipants;
            }
        }
        appendCoverage(result.validation, "secure_pipeline.chain_sum_solved",
                       chainParticipants.size(), solvedParticipants, 0,
                       solvedParticipants == chainParticipants.size() ? 0
                                                                      : 1);
        if (solvedParticipants != chainParticipants.size()) {
            setFailure(
                result, "secure_pipeline.interval_consumption_mismatch",
                "a chain-sum participant is missing from the solved intervals");
            return result;
        }
    }

    const CanonicalBoundaryBuildResult boundaries =
        buildCanonicalBoundaries(imported, reconnaissance,
                                 *intervals.solution);
    appendCoverage(result.validation, "secure_pipeline.boundary_edges",
                   boundaries.validation.expectedEdges,
                   boundaries.validation.checkedEdges, 0,
                   boundaries.validation.failed);
    appendCoverage(result.validation, "secure_pipeline.boundary_samples",
                   boundaries.validation.expectedSamples,
                   boundaries.validation.checkedSamples, 0,
                   boundaries.validation.failed);
    appendCoverage(result.validation, "secure_pipeline.boundary_uv_uses",
                   boundaries.validation.expectedUvUses,
                   boundaries.validation.checkedUvUses, 0,
                   boundaries.validation.failed);
    appendCoverage(
        result.validation, "secure_pipeline.vertex_curve_identity",
        boundaries.validation.expectedVertexCurveChecks,
        boundaries.validation.checkedVertexCurveChecks, 0,
        boundaries.validation.failed);
    appendCoverage(
        result.validation, "secure_pipeline.periodic_uv_closure",
        boundaries.validation.expectedPeriodicClosures,
        boundaries.validation.checkedPeriodicClosures, 0,
        boundaries.validation.failed);
    appendCoverage(
        result.validation, "secure_pipeline.critical_parameter_events",
        boundaries.validation.expectedCriticalEvents,
        boundaries.validation.checkedCriticalEvents, 0,
        boundaries.validation.failed);
    if (!boundaries) {
        setFailure(result,
                   boundaries.failure
                       ? boundaries.failure->code
                       : "secure_pipeline.boundary_build_failed",
                   boundaries.failure
                       ? boundaries.failure->message
                       : "canonical boundary construction failed",
                   boundaries.failure ? boundaries.failure->subjects
                                      : std::vector<StableId>{});
        return result;
    }
    if (const auto consumption = certifySolvedIntervalConsumption(
            *intervals.solution, *boundaries.value)) {
        result.failure = consumption;
        return result;
    }
    if (!configuration.edgeChainSums.empty()) {
        std::size_t consumedParticipants = 0;
        for (const StableId& edge : chainParticipants) {
            const CanonicalBoundary* boundary = boundaries.value->find(edge);
            const auto count = intervals.solution->find(
                {StableIdKind::Boundary, edge.ordinal});
            if (boundary && count && boundary->intervalCount == *count) {
                ++consumedParticipants;
            }
        }
        appendCoverage(result.validation,
                       "secure_pipeline.chain_sum_consumed",
                       chainParticipants.size(), consumedParticipants, 0,
                       consumedParticipants == chainParticipants.size() ? 0
                                                                        : 1);
        if (consumedParticipants != chainParticipants.size()) {
            setFailure(
                result, "secure_pipeline.interval_consumption_mismatch",
                "a chain-sum participant was not consumed by canonical boundaries");
            return result;
        }
    }

    const auto cdt = makeExactLawsonReferencePlanarCdtBackend();
    std::vector<PlanarCdtMesh> faceMeshes;
    std::vector<StableId> expectedFaces;
    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy != GeometryTaxonomy::Surface) continue;
        expectedFaces.push_back(face.subjectId);
        if (face.support !=
            GeometrySupportState::SupportedAnalyticTemplate) {
            setFailure(result, "secure_pipeline.unsupported_surface_family",
                       "an inspectable surface has no certified automatic floor",
                       {face.subjectId});
            return result;
        }
        if (face.familyCode == "plane") {
            const PlanarTrimAssemblyResult trim =
                assemblePlanarTrimDomain(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId);
            for (const PlanarTrimAssemblyEvidence& evidence : trim.evidence) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!trim) {
                setFailure(result,
                           trim.failure ? trim.failure->code
                                        : "secure_pipeline.planar_trim_failed",
                           trim.failure
                               ? trim.failure->message
                               : "planar trim assembly failed",
                           trim.failure ? trim.failure->subjects
                                        : std::vector<StableId>{});
                return result;
            }
            const PlanarCdtResult triangulated =
                cdt->triangulate(*trim.value);
            for (const TrimValidationEvidence& evidence :
                 triangulated.trimValidation.evidence) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            for (const PlanarCdtValidationEvidence& evidence :
                 triangulated.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!triangulated) {
                setFailure(
                    result,
                    triangulated.failure
                        ? triangulated.failure->code
                        : "secure_pipeline.planar_cdt_failed",
                    triangulated.failure
                        ? triangulated.failure->message
                        : "exact planar CDT failed",
                    triangulated.failure
                        ? triangulated.failure->subjects
                        : std::vector<StableId>{});
                return result;
            }
            faceMeshes.push_back(*triangulated.value);
            continue;
        }
        if (face.familyCode == "cylinder") {
            CylinderWallConfiguration cylinder;
            cylinder.maximumChordDeviation =
                configuration.sampling.chordTolerance;
            cylinder.maximumNormalDeviationRadians =
                configuration.sampling.normalAngleToleranceRadians;
            cylinder.axialIntervals = configuration.cylinderAxialIntervals;
            const CylinderWallResult wall = buildFullCylinderWall(
                imported, reconnaissance, *boundaries.value,
                face.subjectId, cylinder);
            for (const CylinderWallValidationEvidence& evidence :
                 wall.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!wall) {
                setFailure(result,
                           wall.failure ? wall.failure->code
                                        : "secure_pipeline.cylinder_failed",
                           wall.failure
                               ? wall.failure->message
                               : "certified cylinder construction failed",
                           wall.failure ? wall.failure->subjects
                                        : std::vector<StableId>{});
                return result;
            }
            faceMeshes.push_back(*wall.value);
            continue;
        }
        setFailure(result, "secure_pipeline.unsupported_surface_family",
                   "the secure automatic pipeline currently supports only plane and full-cylinder faces",
                   {face.subjectId});
        return result;
    }
    if (expectedFaces.empty()) {
        setFailure(result, "secure_pipeline.face_set_empty",
                   "the working B-rep contains no faces");
        return result;
    }

    const CertifiedMeshAssemblyResult assembled =
        assembleCertifiedBoundaryMesh(
            imported, *boundaries.value, faceMeshes, expectedFaces,
            configuration.assembly);
    result.validation.checks.insert(
        result.validation.checks.end(), assembled.validation.checks.begin(),
        assembled.validation.checks.end());
    if (!assembled) {
        setFailure(result,
                   assembled.failure
                       ? assembled.failure->code
                       : "secure_pipeline.body_assembly_failed",
                   assembled.failure
                       ? assembled.failure->message
                       : "certified body assembly failed",
                   assembled.failure ? assembled.failure->subjects
                                     : std::vector<StableId>{});
        return result;
    }
    if (!result.validation.complete()) {
        setFailure(result, "secure_pipeline.validation_incomplete",
                   "one or more required secure validators are incomplete");
        return result;
    }

    GenerationReport generation;
    for (const SolvedInterval& interval : intervals.solution->counts) {
        if (interval.boundaryId.kind == StableIdKind::Boundary) {
            generation.edgeDivisions.emplace(
                static_cast<int>(interval.boundaryId.ordinal),
                static_cast<int>(interval.count));
        }
    }
    MeshingResult meshed = makeCertifiedFloorMeshingResult(
        *assembled.value, result.validation, std::move(generation),
        "structured modeling topology is not yet proven for every face");
    if (auto independent =
            tryBuildIndependentModelingMesh(meshed.certified)) {
        meshed.modeling = std::move(*independent);
    }
    if (const auto consumption = certifySolvedIntervalConsumption(
            *intervals.solution, *boundaries.value, &meshed)) {
        result.failure = consumption;
        return result;
    }
    const ModelingProvenanceResult modeling =
        validateModelingProvenance(meshed);
    appendCoverage(result.validation, modeling.coverage.code,
                   modeling.coverage.expected, modeling.coverage.checked,
                   modeling.coverage.skipped, modeling.coverage.failed);
    appendCoverage(meshed.validation, modeling.coverage.code,
                   modeling.coverage.expected, modeling.coverage.checked,
                   modeling.coverage.skipped, modeling.coverage.failed);
    if (!modeling) {
        setFailure(result,
                   modeling.failure ? modeling.failure->code
                                    : "modeling.provenance_incomplete",
                   modeling.failure
                       ? modeling.failure->message
                       : "modelling provenance validation failed");
        return result;
    }
    result.value = std::move(meshed);
    return result;
}

PolyMesh makeCertifiedPolyMeshAdapter(const MeshingResult& result) {
    PolyMesh adapter;
    const ModelingProvenanceResult modeling =
        validateModelingProvenance(result);
    adapter.selectedOutput = modeling ? modeling.selectedOutput : "certified";
    adapter.vertices.reserve(result.certified.vertices.size());
    adapter.anchors.reserve(result.certified.vertices.size());
    adapter.constraints.reserve(result.certified.vertices.size());
    for (const CertifiedVertex& vertex : result.certified.vertices) {
        adapter.vertices.push_back(vertex.position);
        Anchor anchor;
        MeshConstraint constraint;
        if (!vertex.provenance.empty()) {
            const CertifiedVertexUse& use = vertex.provenance.front();
            anchor.faceId = static_cast<int>(use.workingFace.ordinal);
            anchor.u = use.boundary.uv[0];
            anchor.v = use.boundary.uv[1];
            constraint.type = MeshConstraintType::BrepFace;
            constraint.ownerId = anchor.faceId;
            constraint.u = anchor.u;
            constraint.v = anchor.v;
        }
        adapter.anchors.push_back(anchor);
        adapter.constraints.push_back(constraint);
    }

    adapter.polygons.reserve(result.certified.triangles.size());
    adapter.polygonFaceId.reserve(result.certified.triangles.size());
    adapter.polygonCornerAnchors.reserve(result.certified.triangles.size());
    adapter.certifiedTriangles.reserve(result.certified.triangles.size());
    for (const CertifiedTriangle& triangle : result.certified.triangles) {
        adapter.polygons.push_back(
            {triangle.vertices.begin(), triangle.vertices.end()});
        adapter.polygonFaceId.push_back(
            static_cast<int>(triangle.workingFace.ordinal));
        std::vector<Anchor> corners;
        corners.reserve(3);
        for (const PredicatePoint2& uv : triangle.cornerUv) {
            corners.push_back(
                {static_cast<int>(triangle.workingFace.ordinal), uv[0],
                 uv[1]});
        }
        adapter.polygonCornerAnchors.push_back(std::move(corners));
        adapter.certifiedTriangles.push_back({triangle.vertices});
    }
    return adapter;
}

namespace {

class LocaleIndependentFnv1a64 {
public:
    void add(std::uint64_t value) {
        for (unsigned byte = 0; byte < 8; ++byte) {
            state_ ^= static_cast<std::uint8_t>(value >> (byte * 8U));
            state_ *= 1099511628211ULL;
        }
    }

    void addBytes(const void* data, std::size_t size) {
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        for (std::size_t index = 0; index < size; ++index) {
            state_ ^= bytes[index];
            state_ *= 1099511628211ULL;
        }
    }

    void addString(const std::string& value) {
        add(value.size());
        addBytes(value.data(), value.size());
    }

    void addDouble(double value) {
        add(std::bit_cast<std::uint64_t>(value));
    }

    void addId(StableId id) {
        add(static_cast<std::uint64_t>(id.kind));
        add(id.ordinal);
    }

    std::string finish() const {
        char buffer[17]{};
        std::snprintf(buffer, sizeof(buffer), "%016llx",
                      static_cast<unsigned long long>(state_));
        return buffer;
    }

private:
    std::uint64_t state_ = 14695981039346656037ULL;
};

}  // namespace

M3DeterminismDigest digestM3Determinism(const SecureMeshingResult& result) {
    M3DeterminismDigest digest;
    LocaleIndependentFnv1a64 counts;
    LocaleIndependentFnv1a64 boundary;
    LocaleIndependentFnv1a64 lifts;
    LocaleIndependentFnv1a64 report;

    if (!result.value) {
        counts.addString("empty");
        boundary.addString("empty");
        lifts.addString("empty");
        report.addString("empty");
        digest.counts = counts.finish();
        digest.boundary = boundary.finish();
        digest.lifts = lifts.finish();
        digest.report = report.finish();
        return digest;
    }

    const MeshingResult& mesh = *result.value;
    counts.add(mesh.certified.vertices.size());
    counts.add(mesh.certified.triangles.size());
    counts.add(mesh.generation.edgeDivisions.size());
    for (const auto& [edge, division] : mesh.generation.edgeDivisions) {
        counts.add(static_cast<std::uint64_t>(edge));
        counts.add(static_cast<std::uint64_t>(division));
    }
    counts.add(mesh.generation.faceCounts.size());
    for (const auto& [face, pair] : mesh.generation.faceCounts) {
        counts.add(static_cast<std::uint64_t>(face));
        counts.add(static_cast<std::uint64_t>(pair[0]));
        counts.add(static_cast<std::uint64_t>(pair[1]));
    }

    boundary.add(mesh.certified.vertices.size());
    for (const CertifiedVertex& vertex : mesh.certified.vertices) {
        boundary.add(vertex.canonicalVertexIndex);
        boundary.add(vertex.provenance.size());
        std::vector<const CertifiedVertexUse*> uses;
        uses.reserve(vertex.provenance.size());
        for (const CertifiedVertexUse& use : vertex.provenance) {
            uses.push_back(&use);
        }
        std::sort(uses.begin(), uses.end(),
                  [](const CertifiedVertexUse* left,
                     const CertifiedVertexUse* right) {
                      if (left->workingFace != right->workingFace) {
                          return left->workingFace < right->workingFace;
                      }
                      if (left->boundary.workingEdge !=
                          right->boundary.workingEdge) {
                          return left->boundary.workingEdge <
                              right->boundary.workingEdge;
                      }
                      if (left->boundary.sample.boundary !=
                          right->boundary.sample.boundary) {
                          return left->boundary.sample.boundary <
                              right->boundary.sample.boundary;
                      }
                      if (left->boundary.sample.ordinal !=
                          right->boundary.sample.ordinal) {
                          return left->boundary.sample.ordinal <
                              right->boundary.sample.ordinal;
                      }
                      return left->boundary.coedge < right->boundary.coedge;
                  });
        for (const CertifiedVertexUse* use : uses) {
            boundary.addId(use->workingFace);
            boundary.addId(use->sourceFace.value_or(StableId{}));
            boundary.addId(use->boundary.workingEdge);
            boundary.addId(use->boundary.sourceEdge.value_or(StableId{}));
            boundary.addId(use->boundary.coedge);
            boundary.addId(use->boundary.sample.boundary);
            boundary.add(use->boundary.sample.ordinal);
        }
    }
    boundary.add(mesh.certified.triangles.size());
    for (const CertifiedTriangle& triangle : mesh.certified.triangles) {
        boundary.addId(triangle.workingFace);
        boundary.addId(triangle.sourceFace.value_or(StableId{}));
        for (std::uint32_t vertex : triangle.vertices) {
            boundary.add(vertex);
        }
    }

    lifts.add(mesh.certified.vertices.size());
    for (const CertifiedVertex& vertex : mesh.certified.vertices) {
        lifts.add(vertex.canonicalVertexIndex);
        for (double coordinate : vertex.position) {
            lifts.addDouble(coordinate);
        }
        for (const CertifiedVertexUse& use : vertex.provenance) {
            lifts.addDouble(use.boundary.uv[0]);
            lifts.addDouble(use.boundary.uv[1]);
        }
    }
    lifts.add(mesh.certified.triangles.size());
    for (const CertifiedTriangle& triangle : mesh.certified.triangles) {
        for (const PredicatePoint2& uv : triangle.cornerUv) {
            lifts.addDouble(uv[0]);
            lifts.addDouble(uv[1]);
        }
    }

    std::vector<ValidationCoverage> checks = result.validation.checks;
    std::sort(checks.begin(), checks.end(),
              [](const ValidationCoverage& left,
                 const ValidationCoverage& right) {
                  if (left.code != right.code) return left.code < right.code;
                  if (left.expected != right.expected) {
                      return left.expected < right.expected;
                  }
                  if (left.checked != right.checked) {
                      return left.checked < right.checked;
                  }
                  if (left.skipped != right.skipped) {
                      return left.skipped < right.skipped;
                  }
                  return left.failed < right.failed;
              });
    report.add(checks.size());
    for (const ValidationCoverage& coverage : checks) {
        report.addString(coverage.code);
        report.add(coverage.expected);
        report.add(coverage.checked);
        report.add(coverage.skipped);
        report.add(coverage.failed);
    }
    report.addString(mesh.certified.topologyFingerprint);

    digest.counts = counts.finish();
    digest.boundary = boundary.finish();
    digest.lifts = lifts.finish();
    digest.report = report.finish();
    return digest;
}

}  // namespace weft
