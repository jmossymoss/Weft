#include "weft/secure_meshing.hpp"

#include "weft/cone_template.hpp"
#include "weft/planar_trim_assembly.hpp"
#include "weft/sphere_template.hpp"
#include "weft/mapped_template.hpp"
#include "weft/torus_template.hpp"

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

void secureProgress(const SecureMeshingConfiguration& configuration,
                    const char* message) {
    if (!configuration.progressToStderr || message == nullptr) return;
    std::fprintf(stderr, "WEFT_PROGRESS %s\n", message);
    std::fflush(stderr);
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
    // Fail fast on unsupported curve families (e.g. 211 ellipses on MP9)
    // before the expensive per-edge bspline UV-span walk.
    for (const ExactGeometryClassification& record : reconnaissance.records) {
        if (record.taxonomy != GeometryTaxonomy::Curve) continue;
        const bool supportedCurve =
            record.familyCode == "line" || record.familyCode == "circle" ||
            record.familyCode == "ellipse" ||
            record.familyCode == "bspline" || record.familyCode == "bezier" ||
            std::find(record.conditionCodes.begin(),
                      record.conditionCodes.end(),
                      "degenerate") != record.conditionCodes.end();
        if (!supportedCurve) {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.unsupported_curve_family",
                "curve family '" + record.familyCode +
                    "' has no certified automatic interval consumer",
                {record.subjectId}};
            return result;
        }
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
        if (topology.degenerate ||
            std::find(classification->conditionCodes.begin(),
                      classification->conditionCodes.end(),
                      "degenerate") != classification->conditionCodes.end()) {
            // Apex/pole singular edges: one canonical station, no sagitta
            // count. Unsupported surface families still refuse later.
            count = 1;
        } else if (classification->familyCode == "line") {
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
            // Partial cylinder/cone bands need ≥2 rim intervals (3 samples)
            // so the wall template can form at least two azimuth columns.
            if (!fullCircle) {
                count = std::max<std::uint32_t>(count, 2);
            }
        } else if (classification->familyCode == "ellipse") {
            const EvaluationResult<ParameterDomain> domain =
                imported.workingEvaluator->curveDomain(topology.id);
            if (!domain || !domain.value->lower || !domain.value->upper) {
                result.failure = SecureMeshingFailure{
                    "secure_pipeline.ellipse_domain_invalid",
                    "a supported elliptical edge has no finite exact domain",
                    {topology.id}};
                return result;
            }
            // Bound radius by the maximum first-derivative length sampled on
            // the arc (conservative vs. the sharper minor-axis region).
            double boundRadius = 0.0;
            for (double fraction : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                const double parameter = *domain.value->lower +
                    (*domain.value->upper - *domain.value->lower) * fraction;
                const EvaluationResult<CurveEvaluation> evaluated =
                    imported.workingEvaluator->evaluateCurve(topology.id,
                                                              parameter);
                if (evaluated) {
                    boundRadius = std::max(
                        boundRadius,
                        vectorLength(evaluated.value->firstDerivative));
                }
            }
            const bool fullEllipse = topology.lowerVertex &&
                topology.upperVertex &&
                *topology.lowerVertex == *topology.upperVertex;
            const SegmentCountResult demanded = ellipticalArcSegmentCount(
                boundRadius, boundRadius,
                *domain.value->upper - *domain.value->lower, fullEllipse,
                configuration.sampling);
            if (!demanded) {
                result.failure = SecureMeshingFailure{
                    demanded.failure ? demanded.failure->code
                                     : "secure_pipeline.ellipse_count_failed",
                    demanded.failure
                        ? demanded.failure->message
                        : "an elliptical segment count could not be proven",
                    {topology.id}};
                return result;
            }
            count = *demanded.count;
            if (!fullEllipse) {
                count = std::max<std::uint32_t>(count, 2);
            }
        } else if (classification->familyCode == "bspline" ||
                   classification->familyCode == "bezier") {
            // MAP/FREE UV-grid: align edge interval counts to the face UV
            // cell size so split rails (half-edges) land on grid stations.
            // Uniform minClosedCurveSegments on every edge makes short rails
            // sample at half-cell offsets and breaks seam matching.
            count = std::max<std::uint32_t>(
                4, configuration.sampling.minimumClosedCurveSegments);
            const std::uint32_t gridIntervals = std::max<std::uint32_t>(
                8, configuration.sampling.minimumClosedCurveSegments);
            for (const CoedgeRecord& coedge : snapshot.coedges) {
                if (coedge.edgeId != topology.id) continue;
                const ExactGeometryClassification* face =
                    reconnaissance.find(coedge.faceId);
                if (!face || face->taxonomy != GeometryTaxonomy::Surface) {
                    continue;
                }
                const bool uvGridFace =
                    std::find(face->conditionCodes.begin(),
                              face->conditionCodes.end(),
                              "mapped.four_sided_candidate") !=
                        face->conditionCodes.end() ||
                    std::find(face->conditionCodes.begin(),
                              face->conditionCodes.end(),
                              "freeform.uv_grid_candidate") !=
                        face->conditionCodes.end();
                if (!uvGridFace || face->parameterDomains.size() < 2 ||
                    !face->parameterDomains[0].lower ||
                    !face->parameterDomains[0].upper ||
                    !face->parameterDomains[1].lower ||
                    !face->parameterDomains[1].upper) {
                    continue;
                }
                if (coedge.pcurveRepresentations.empty()) continue;
                const EvaluationResult<ParameterDomain> domain =
                    imported.workingEvaluator->curveDomain(topology.id);
                if (!domain || !domain.value->lower || !domain.value->upper) {
                    continue;
                }
                const PcurveRef& pref = coedge.pcurveRepresentations.front();
                const auto uvLower =
                    imported.workingEvaluator->evaluateCurveOnSurface(
                        pref, *domain.value->lower);
                const auto uvUpper =
                    imported.workingEvaluator->evaluateCurveOnSurface(
                        pref, *domain.value->upper);
                if (!uvLower || !uvUpper) continue;
                const double faceDu = *face->parameterDomains[0].upper -
                    *face->parameterDomains[0].lower;
                const double faceDv = *face->parameterDomains[1].upper -
                    *face->parameterDomains[1].lower;
                if (!(faceDu > 0.0) || !(faceDv > 0.0)) continue;
                const double cellU = faceDu / static_cast<double>(gridIntervals);
                const double cellV = faceDv / static_cast<double>(gridIntervals);
                const double edgeDu =
                    std::abs(uvUpper.value->uv[0] - uvLower.value->uv[0]);
                const double edgeDv =
                    std::abs(uvUpper.value->uv[1] - uvLower.value->uv[1]);
                const double cells = std::round(edgeDu / cellU) +
                    std::round(edgeDv / cellV);
                if (cells >= 1.0) {
                    count = static_cast<std::uint32_t>(cells);
                }
                break;
            }
        } else {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.unsupported_curve_family",
                "the secure automatic pipeline currently supports only line, circle, ellipse, and bounded bspline/bezier edges",
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
            (face.familyCode != "cylinder" && face.familyCode != "cone")) {
            continue;
        }
        const bool fullPeriodic = face.trimDomain ==
            TrimDomainClass::FullPeriodicWithCapBoundaries;
        const bool partialBand = face.trimDomain ==
            TrimDomainClass::PeriodicBandCrossingSeam;
        // Apex cones are not two-rim bands.
        if (face.familyCode == "cone" &&
            face.trimDomain == TrimDomainClass::TouchesOneSingularity) {
            continue;
        }
        if (!fullPeriodic && !partialBand) continue;
        std::set<StableId> rimBoundaries;
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            if (coedge.faceId != face.subjectId) continue;
            const ExactGeometryClassification* edge =
                reconnaissance.find(coedge.edgeId);
            const EdgeTopologyRecord* topology =
                edgeTopology(snapshot, coedge.edgeId);
            // Circle rims and planar-section ellipses (MP9 cylinder cuts).
            if (!edge || !topology ||
                (edge->familyCode != "circle" &&
                 edge->familyCode != "ellipse") ||
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

    secureProgress(configuration, "reconnaissance.begin");
    const ReconnaissanceReport reconnaissance = reconnoitre(imported);
    {
        const std::string msg = "reconnaissance.done subjects=" +
            std::to_string(reconnaissance.checkedSubjects);
        secureProgress(configuration, msg.c_str());
    }
    appendCoverage(result.validation, "secure_pipeline.reconnaissance",
                   reconnaissance.expectedSubjects,
                   reconnaissance.checkedSubjects, 0,
                   reconnaissance.complete ? 0 : 1);
    if (!reconnaissance.complete) {
        setFailure(result, "secure_pipeline.reconnaissance_incomplete",
                   "exact topology and geometry reconnaissance is incomplete");
        return result;
    }

    // ADR-0014: one unsupported face → no MeshingResult. Fail before
    // interval/boundary work so residuals stay named (WP-175).
    if (!configuration.collectAllUnsupported) {
        for (const ExactGeometryClassification& record :
             reconnaissance.records) {
            if (record.taxonomy != GeometryTaxonomy::Surface) continue;
            if (record.support ==
                GeometrySupportState::SupportedAnalyticTemplate) {
                continue;
            }
            std::string detail = "surface family '" + record.familyCode +
                "' has no certified automatic floor";
            const char* preferred = nullptr;
            for (const std::string& code : record.conditionCodes) {
                if (code.find("_deferred") == std::string::npos) continue;
                if (code.rfind("freeform.high_edge", 0) == 0 ||
                    code.rfind("cone.non_apex", 0) == 0 ||
                    code.rfind("sphere.partial", 0) == 0 ||
                    code.rfind("cylinder.complex", 0) == 0 ||
                    code.rfind("extrusion.", 0) == 0 ||
                    code.rfind("offset.", 0) == 0) {
                    preferred = code.c_str();
                    break;
                }
                if (preferred == nullptr) preferred = code.c_str();
            }
            if (preferred != nullptr) {
                detail += " (";
                detail += preferred;
                detail += ")";
            }
            setFailure(result, "secure_pipeline.unsupported_surface_family",
                       detail, {record.subjectId});
            return result;
        }
    }

    // Inventory dry-run: aggregate unsupported curve/surface families from
    // reconnaissance without attempting a full mesh certificate.
    if (configuration.collectAllUnsupported) {
        secureProgress(configuration, "inventory.aggregate_unsupported.begin");
        for (const ExactGeometryClassification& record :
             reconnaissance.records) {
            if (record.taxonomy == GeometryTaxonomy::Curve) {
                const bool supportedCurve =
                    record.familyCode == "line" ||
                    record.familyCode == "circle" ||
                    record.familyCode == "ellipse" ||
                    record.familyCode == "bspline" ||
                    record.familyCode == "bezier" ||
                    std::find(record.conditionCodes.begin(),
                              record.conditionCodes.end(),
                              "degenerate") != record.conditionCodes.end();
                if (!supportedCurve) {
                    result.unsupportedRecords.push_back(
                        {std::string("secure_pipeline.unsupported_curve_family"),
                         "curve family has no certified automatic consumer",
                         {record.subjectId}, record.familyCode});
                }
            } else if (record.taxonomy == GeometryTaxonomy::Surface) {
                if (record.support !=
                    GeometrySupportState::SupportedAnalyticTemplate) {
                    result.unsupportedRecords.push_back(
                        {std::string(
                             "secure_pipeline.unsupported_surface_family"),
                         "surface has no certified automatic floor",
                         {record.subjectId}, record.familyCode});
                }
            }
        }
        setFailure(result, "secure_pipeline.inventory_unsupported",
                   "collectAllUnsupported aggregated unsupported subjects; "
                   "no MeshingResult is produced");
        {
            const std::string msg =
                "inventory.aggregate_unsupported.done count=" +
                std::to_string(result.unsupportedRecords.size());
            secureProgress(configuration, msg.c_str());
        }
        return result;
    }

    secureProgress(configuration, "intervals.begin");
    const IntervalProblemResult intervalProblem = buildIntervalProblem(
        imported, reconnaissance, configuration);
    if (!intervalProblem.value) {
        result.failure = intervalProblem.failure;
        const std::string msg = "intervals.failed " +
            (result.failure ? result.failure->code : std::string("-"));
        secureProgress(configuration, msg.c_str());
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
    std::size_t faceOrdinal = 0;
    std::size_t faceTotal = 0;
    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy == GeometryTaxonomy::Surface) ++faceTotal;
    }
    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy != GeometryTaxonomy::Surface) continue;
        ++faceOrdinal;
        if (configuration.progressToStderr) {
            const std::string msg = "face " + std::to_string(faceOrdinal) +
                "/" + std::to_string(faceTotal) + " id=" +
                std::to_string(face.subjectId.ordinal) +
                " family=" + face.familyCode;
            secureProgress(configuration, msg.c_str());
        }
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
        if (face.familyCode == "cone") {
            const bool apex =
                face.trimDomain == TrimDomainClass::TouchesOneSingularity;
            const bool frustumBand =
                face.trimDomain ==
                    TrimDomainClass::FullPeriodicWithCapBoundaries ||
                face.trimDomain == TrimDomainClass::PeriodicBandCrossingSeam;
            if (apex) {
                ConeWallConfiguration cone;
                cone.maximumChordDeviation =
                    configuration.sampling.chordTolerance;
                cone.maximumNormalDeviationRadians =
                    configuration.sampling.normalAngleToleranceRadians;
                const ConeWallResult wall = buildApexConeWall(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId, cone);
                for (const ConeWallValidationEvidence& evidence :
                     wall.validation) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                if (!wall) {
                    setFailure(result,
                               wall.failure ? wall.failure->code
                                            : "secure_pipeline.cone_failed",
                               wall.failure
                                   ? wall.failure->message
                                   : "certified apex-cone construction failed",
                               wall.failure ? wall.failure->subjects
                                            : std::vector<StableId>{});
                    return result;
                }
                faceMeshes.push_back(*wall.value);
                continue;
            }
            if (frustumBand) {
                // Truncated cone = revolved band; reuse cylinder wall consumer.
                // At least one axial interval yields a pure quad ring between
                // the two rims; bump to 2 when the caller left the default
                // so modelling has a non-trivial strip lattice.
                CylinderWallConfiguration band;
                band.maximumChordDeviation =
                    configuration.sampling.chordTolerance;
                band.maximumNormalDeviationRadians =
                    configuration.sampling.normalAngleToleranceRadians;
                band.axialIntervals = std::max<std::uint32_t>(
                    1U, configuration.cylinderAxialIntervals);
                const CylinderWallResult wall = buildFullCylinderWall(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId, band);
                for (const CylinderWallValidationEvidence& evidence :
                     wall.validation) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                if (!wall) {
                    setFailure(result,
                               wall.failure
                                   ? wall.failure->code
                                   : "secure_pipeline.cone_frustum_failed",
                               wall.failure
                                   ? wall.failure->message
                                   : "certified truncated-cone band failed",
                               wall.failure ? wall.failure->subjects
                                            : std::vector<StableId>{});
                    return result;
                }
                faceMeshes.push_back(*wall.value);
                continue;
            }
            setFailure(result, "secure_pipeline.unsupported_surface_family",
                       "cone trim is not an apex cone or truncated band",
                       {face.subjectId});
            return result;
        }
        if (face.familyCode == "sphere") {
            SphereWallConfiguration sphere;
            sphere.maximumChordDeviation =
                configuration.sampling.chordTolerance;
            sphere.maximumNormalDeviationRadians =
                configuration.sampling.normalAngleToleranceRadians;
            double sphereRadius = 1.0;
            if (face.parameterDomains.size() >= 2 &&
                face.parameterDomains[0].lower &&
                face.parameterDomains[1].lower &&
                face.parameterDomains[1].upper) {
                const double midV = (*face.parameterDomains[1].lower +
                                     *face.parameterDomains[1].upper) *
                    0.5;
                const EvaluationResult<SurfaceEvaluation> equator =
                    imported.workingEvaluator->evaluateSurface(
                        face.subjectId, {*face.parameterDomains[0].lower, midV});
                if (equator) {
                    sphereRadius = vectorLength(equator.value->position);
                    if (!(sphereRadius > 0.0)) sphereRadius = 1.0;
                }
            }
            const SegmentCountResult azimuth = circularArcSegmentCount(
                sphereRadius, 6.28318530717958647692, true,
                configuration.sampling);
            if (azimuth && *azimuth.count >= 3) {
                sphere.azimuthIntervals = *azimuth.count;
            } else {
                sphere.azimuthIntervals = std::max<std::uint32_t>(
                    16, configuration.sampling.minimumClosedCurveSegments);
            }
            // Guard parallel-circle sagitta near the equator.
            sphere.azimuthIntervals =
                std::max<std::uint32_t>(sphere.azimuthIntervals, 16);
            const SphereWallResult wall = buildFullSphereWall(
                imported, reconnaissance, *boundaries.value, face.subjectId,
                sphere);
            for (const SphereWallValidationEvidence& evidence :
                 wall.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!wall) {
                setFailure(result,
                           wall.failure ? wall.failure->code
                                        : "secure_pipeline.sphere_failed",
                           wall.failure
                               ? wall.failure->message
                               : "certified sphere construction failed",
                           wall.failure ? wall.failure->subjects
                                        : std::vector<StableId>{});
                return result;
            }
            faceMeshes.push_back(*wall.value);
            continue;
        }
        if (face.familyCode == "torus") {
            TorusWallConfiguration torus;
            torus.maximumChordDeviation = configuration.sampling.chordTolerance;
            torus.maximumNormalDeviationRadians =
                configuration.sampling.normalAngleToleranceRadians;
            torus.majorIntervals = std::max<std::uint32_t>(
                16, configuration.sampling.minimumClosedCurveSegments);
            torus.minorIntervals = std::max<std::uint32_t>(
                12, configuration.sampling.minimumClosedCurveSegments / 2);
            const TorusWallResult wall = buildFullTorusWall(
                imported, reconnaissance, *boundaries.value, face.subjectId,
                torus);
            for (const TorusWallValidationEvidence& evidence : wall.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!wall) {
                setFailure(result,
                           wall.failure ? wall.failure->code
                                        : "secure_pipeline.torus_failed",
                           wall.failure ? wall.failure->message
                                        : "certified torus construction failed",
                           wall.failure ? wall.failure->subjects
                                        : std::vector<StableId>{});
                return result;
            }
            faceMeshes.push_back(*wall.value);
            continue;
        }
        const bool mappedFourSided =
            std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                      "mapped.four_sided_candidate") !=
            face.conditionCodes.end();
        const bool freeformUvGrid =
            std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                      "freeform.uv_grid_candidate") !=
            face.conditionCodes.end();
        if (mappedFourSided || freeformUvGrid) {
            MappedPatchConfiguration mapped;
            mapped.maximumChordDeviation =
                configuration.sampling.chordTolerance;
            mapped.maximumNormalDeviationRadians =
                configuration.sampling.normalAngleToleranceRadians;
            mapped.uIntervals = std::max<std::uint32_t>(
                8, configuration.sampling.minimumClosedCurveSegments);
            mapped.vIntervals = mapped.uIntervals;
            // WP-174: non-periodic extrusion/offset patches need denser UV
            // so facet normals stay within the LOD budget.
            if (face.familyCode == "extrusion" ||
                face.familyCode == "offset") {
                mapped.uIntervals = std::max<std::uint32_t>(
                    mapped.uIntervals, 32);
                mapped.vIntervals = std::max<std::uint32_t>(
                    mapped.vIntervals, 32);
            }
            const MappedPatchResult patch = buildMappedFourSidedPatch(
                imported, reconnaissance, *boundaries.value, face.subjectId,
                mapped);
            for (const MappedPatchValidationEvidence& evidence :
                 patch.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!patch) {
                setFailure(result,
                           patch.failure ? patch.failure->code
                                         : "secure_pipeline.mapped_failed",
                           patch.failure
                               ? patch.failure->message
                               : "certified mapped patch construction failed",
                           patch.failure ? patch.failure->subjects
                                         : std::vector<StableId>{});
                return result;
            }
            faceMeshes.push_back(*patch.value);
            continue;
        }
        setFailure(result, "secure_pipeline.unsupported_surface_family",
                   "the secure automatic pipeline currently supports only plane, cylinder, apex-cone, sphere, torus, and four-sided mapped faces",
                   {face.subjectId});
        return result;
    }
    if (expectedFaces.empty()) {
        setFailure(result, "secure_pipeline.face_set_empty",
                   "the working B-rep contains no faces");
        return result;
    }

    CertifiedMeshAssemblyConfiguration assemblyConfig = configuration.assembly;
    // Single-face mapped patches are open shells; do not demand closed
    // manifold incidence for that narrow MAP-C product class.
    if (imported.working && imported.working->snapshot.model.faceCount() == 1) {
        assemblyConfig.requireClosedManifold = false;
    }
    const CertifiedMeshAssemblyResult assembled =
        assembleCertifiedBoundaryMesh(
            imported, *boundaries.value, faceMeshes, expectedFaces,
            assemblyConfig);
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
    generation.namedLodEffects["cylinderAxialIntervals"] =
        std::to_string(configuration.cylinderAxialIntervals) +
        " axial intervals on cylinder walls";
    generation.namedLodEffects["chordTolerance"] =
        std::to_string(configuration.sampling.chordTolerance) +
        " model-unit chord budget";
    generation.namedLodEffects["normalAngleToleranceRadians"] =
        std::to_string(configuration.sampling.normalAngleToleranceRadians) +
        " facet normal turn budget";
    generation.namedLodEffects["minimumClosedCurveSegments"] =
        std::to_string(configuration.sampling.minimumClosedCurveSegments) +
        " minimum closed-curve segments";
    generation.namedLodEffects["exactEdgeOverrides"] =
        std::to_string(configuration.exactEdgeIntervalCounts.size()) +
        " exact per-edge interval constraints";
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
    meshed.generation.namedLodEffects["selectedOutput"] =
        modeling.selectedOutput;
    result.value = std::move(meshed);
    return result;
}

CertifiedAdmissionResult admitCertifiedMeshingResult(
    const MeshingResult& result,
    std::optional<std::uint64_t> expectedGenerationEpoch,
    std::optional<std::uint64_t> actualGenerationEpoch) {
    CertifiedAdmissionResult out;
    if (!result.validation.complete()) {
        out.failure = CertifiedAdmissionFailure{
            "admission.certificate_incomplete",
            "MeshingResult validation certificate is incomplete"};
        return out;
    }
    if (result.certified.triangles.empty() ||
        result.certified.topologyFingerprint.empty()) {
        out.failure = CertifiedAdmissionFailure{
            "admission.certified_mesh_empty",
            "MeshingResult has no certified mesh to admit"};
        return out;
    }
    if (expectedGenerationEpoch && actualGenerationEpoch &&
        *expectedGenerationEpoch != *actualGenerationEpoch) {
        out.failure = CertifiedAdmissionFailure{
            "admission.stale_generation",
            "MeshingResult generation epoch does not match the request"};
        return out;
    }
    const ModelingProvenanceResult modeling =
        validateModelingProvenance(result);
    if (!modeling) {
        out.failure = CertifiedAdmissionFailure{
            modeling.failure ? modeling.failure->code
                             : "admission.modeling_provenance_invalid",
            modeling.failure
                ? modeling.failure->message
                : "modelling provenance is invalid for admission"};
        return out;
    }
    out.selectedOutput = modeling.selectedOutput;
    return out;
}

std::string fingerprintSecureCacheKey(const SecureCacheKey& key) {
    return key.sourceSha256 + "|" + key.recipeFingerprint + "|" +
        key.settingsFingerprint + "|" + key.implementationVersion + "|" +
        key.certificateFingerprint;
}

bool secureCacheKeysMatch(const SecureCacheKey& left,
                          const SecureCacheKey& right) {
    return fingerprintSecureCacheKey(left) == fingerprintSecureCacheKey(right);
}

SecureCacheLookupResult lookupSecureCache(
    const SecureCacheKey& request, const SecureCacheKey& cached,
    const MeshingResult* cachedResult) {
    SecureCacheLookupResult out;
    if (request.implementationVersion != kSecureImplementationVersion ||
        cached.implementationVersion != kSecureImplementationVersion) {
        out.failure = CertifiedAdmissionFailure{
            "cache.implementation_mismatch",
            "secure cache implementation version does not match"};
        return out;
    }
    if (!secureCacheKeysMatch(request, cached)) {
        out.failure = CertifiedAdmissionFailure{
            "cache.key_mismatch",
            "secure cache key does not match the request"};
        return out;
    }
    if (!cachedResult) {
        out.failure = CertifiedAdmissionFailure{
            "cache.entry_corrupt",
            "secure cache entry has no MeshingResult payload"};
        return out;
    }
    if (cached.certificateFingerprint !=
        cachedResult->certified.topologyFingerprint) {
        out.failure = CertifiedAdmissionFailure{
            "cache.certificate_mismatch",
            "secure cache certificate fingerprint does not match the payload"};
        return out;
    }
    const CertifiedAdmissionResult admitted =
        admitCertifiedMeshingResult(*cachedResult);
    if (!admitted) {
        out.failure = admitted.failure;
        return out;
    }
    out.hit = true;
    return out;
}

PolyMesh makeCertifiedPolyMeshAdapter(const MeshingResult& result) {
    PolyMesh adapter;
    const CertifiedAdmissionResult admitted =
        admitCertifiedMeshingResult(result);
    if (!admitted) {
        // Fail closed: return an empty adapter rather than an uncertified mesh.
        adapter.selectedOutput = admitted.failure->code;
        return adapter;
    }
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
