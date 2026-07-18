#include "weft/secure_meshing.hpp"

#include "weft/planar_trim_assembly.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
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
             static_cast<double>(count), count, false,
             exact == configuration.exactEdgeIntervalCounts.end()
                 ? std::optional<std::uint32_t>{}
                 : std::optional<std::uint32_t>{exact->second}});
    }

    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy != GeometryTaxonomy::Surface ||
            face.familyCode != "cylinder" ||
            face.trimDomain !=
                TrimDomainClass::FullPeriodicWithCapBoundaries) {
            continue;
        }
        std::set<StableId> rimBoundaries;
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            if (coedge.faceId != face.subjectId) continue;
            const ExactGeometryClassification* edge =
                reconnaissance.find(coedge.edgeId);
            const EdgeTopologyRecord* topology =
                edgeTopology(snapshot, coedge.edgeId);
            if (edge && topology && edge->familyCode == "circle" &&
                topology->lowerVertex && topology->upperVertex &&
                *topology->lowerVertex == *topology->upperVertex) {
                rimBoundaries.insert(
                    {StableIdKind::Boundary, coedge.edgeId.ordinal});
            }
        }
        if (rimBoundaries.size() != 2) {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.cylinder_rims_unresolved",
                "a full cylinder does not resolve exactly two circular rim boundaries",
                {face.subjectId}};
            return result;
        }
        problem.equalities.push_back(
            {{rimBoundaries.begin(), rimBoundaries.end()}});
    }
    result.value = std::move(problem);
    return result;
}

}  // namespace

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
    result.value = makeCertifiedFloorMeshingResult(
        *assembled.value, result.validation, std::move(generation),
        "structured modeling topology is not yet proven for every face");
    return result;
}

PolyMesh makeCertifiedPolyMeshAdapter(const MeshingResult& result) {
    PolyMesh adapter;
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

}  // namespace weft
