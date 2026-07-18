#include "weft/cone_template.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace weft {
namespace {

enum EvidenceIndex : std::size_t {
    Prerequisites = 0,
    BoundaryCoverage,
    TriangleOrientation,
    ChordBound,
    NormalBound,
};

struct FaceSampleUse {
    const CanonicalBoundarySample* sample = nullptr;
    const CoedgeUvUse* use = nullptr;
};

void setFailure(ConeWallResult& result, std::size_t evidence, std::string code,
                std::string message, std::vector<StableId> subjects) {
    ++result.validation[evidence].failed;
    if (!result.failure) {
        result.failure = ConeWallFailure{std::move(code), std::move(message),
                                         std::move(subjects)};
    }
}

std::optional<StableId> uniqueSourceForWorking(const ImportedModel& imported,
                                               StableId working) {
    std::optional<StableId> source;
    for (const CorrespondenceRecord& record : imported.correspondence.records) {
        if (std::find(record.workingIds.begin(), record.workingIds.end(),
                      working) == record.workingIds.end()) {
            continue;
        }
        if (source) return std::nullopt;
        source = record.sourceId;
    }
    return source;
}

PlanarTrimBoundaryUse boundaryUse(const FaceSampleUse& item) {
    return {item.sample->id,
            item.sample->workingEdge,
            item.sample->sourceEdge,
            item.use->coedge,
            item.use->liftedUv,
            item.use->measuredCurveOnSurfaceDiscrepancy,
            item.use->allowedCurveOnSurfaceDiscrepancy,
            item.use->representation};
}

double squaredDistance(const std::array<double, 3>& first,
                       const std::array<double, 3>& second) {
    const double x = first[0] - second[0];
    const double y = first[1] - second[1];
    const double z = first[2] - second[2];
    return x * x + y * y + z * z;
}

std::array<double, 3> triangleNormal(const std::array<double, 3>& a,
                                     const std::array<double, 3>& b,
                                     const std::array<double, 3>& c) {
    const std::array<double, 3> ab{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const std::array<double, 3> ac{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    return {ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]};
}

double dot(const std::array<double, 3>& first,
           const std::array<double, 3>& second) {
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
}

std::optional<std::array<double, 3>> unit(const std::array<double, 3>& vector) {
    const double lengthSquared = dot(vector, vector);
    if (!std::isfinite(lengthSquared) || !(lengthSquared > 0.0)) {
        return std::nullopt;
    }
    const double inverse = 1.0 / std::sqrt(lengthSquared);
    return std::array<double, 3>{vector[0] * inverse, vector[1] * inverse,
                                 vector[2] * inverse};
}

double periodicNear(double value, double reference, double period) {
    return value + std::round((reference - value) / period) * period;
}

bool orientTriangle(PlanarCdtTriangle& triangle,
                    const GeometricPredicates& predicates,
                    ConeWallResult& result, StableId face) {
    const auto sign = predicates.orient2d((*triangle.cornerUv)[0],
                                          (*triangle.cornerUv)[1],
                                          (*triangle.cornerUv)[2]);
    ++result.validation[TriangleOrientation].checked;
    if (!sign || *sign.value == ExactSign::Zero) {
        setFailure(result, TriangleOrientation,
                   sign ? "cone.triangle_uv_degenerate"
                        : "cone.predicate_failure",
                   sign ? "a cone triangle has zero exact UV area"
                        : (sign.failure ? sign.failure->message
                                        : "triangle orientation failed"),
                   {face});
        return false;
    }
    if (*sign.value == ExactSign::Negative) {
        std::swap(triangle.vertices[1], triangle.vertices[2]);
        std::swap((*triangle.cornerUv)[1], (*triangle.cornerUv)[2]);
    }
    return true;
}

std::optional<std::array<double, 3>> samplePosition(
    const ImportedModel& imported, StableId workingFace,
    const PlanarTrimVertex& vertex) {
    if (!vertex.boundaryUses.empty()) {
        // Prefer an exact boundary sample position via surface evaluation of
        // the attached UV (boundary samples already certified elsewhere).
        const EvaluationResult<SurfaceEvaluation> evaluated =
            imported.workingEvaluator->evaluateSurface(workingFace, vertex.uv);
        if (evaluated) return evaluated.value->position;
    }
    const EvaluationResult<SurfaceEvaluation> evaluated =
        imported.workingEvaluator->evaluateSurface(workingFace, vertex.uv);
    if (!evaluated) return std::nullopt;
    return evaluated.value->position;
}

}  // namespace

ConeWallResult buildApexConeWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries, StableId workingFace,
    const ConeWallConfiguration& configuration,
    std::shared_ptr<const GeometricPredicates> predicates) {
    ConeWallResult result;
    result.validation = {
        {"cone.prerequisites", 1},
        {"cone.boundary_coverage", 0},
        {"cone.triangle_orientation", 0},
        {"cone.chord_bound", 0},
        {"cone.normal_bound", 0},
    };
    ++result.validation[Prerequisites].checked;
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !reconnaissance.complete ||
        !boundaries.validation.complete() || !predicates ||
        !predicates->exactForFiniteDoubleInputs()) {
        setFailure(result, Prerequisites, "cone.prerequisite_incomplete",
                   "cone assembly requires certified import, reconnaissance, boundaries, and exact predicates",
                   {workingFace});
        return result;
    }
    if (!std::isfinite(configuration.maximumChordDeviation) ||
        !(configuration.maximumChordDeviation > 0.0) ||
        !std::isfinite(configuration.maximumNormalDeviationRadians) ||
        !(configuration.maximumNormalDeviationRadians > 0.0) ||
        !(configuration.maximumNormalDeviationRadians <
          1.57079632679489661923)) {
        setFailure(result, Prerequisites, "cone.configuration_invalid",
                   "cone error limits must be finite, positive, and below ninety degrees",
                   {workingFace});
        return result;
    }

    const ExactGeometryClassification* classification =
        reconnaissance.find(workingFace);
    if (!classification ||
        classification->taxonomy != GeometryTaxonomy::Surface ||
        classification->familyCode != "cone" ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate ||
        classification->trimDomain !=
            TrimDomainClass::TouchesOneSingularity) {
        setFailure(result, Prerequisites, "cone.face_unsupported",
                   "only a proven supported apex-cone face may use this template",
                   {workingFace});
        return result;
    }
    if (classification->parameterDomains.size() < 2 ||
        !classification->parameterDomains[0].periodic ||
        !classification->parameterDomains[0].period ||
        !std::isfinite(*classification->parameterDomains[0].period) ||
        !(*classification->parameterDomains[0].period > 0.0)) {
        setFailure(result, Prerequisites, "cone.period_missing",
                   "the conical U period is not proven", {workingFace});
        return result;
    }
    const double period = *classification->parameterDomains[0].period;
    const std::optional<StableId> sourceFace =
        uniqueSourceForWorking(imported, workingFace);
    if (!sourceFace || sourceFace->kind != StableIdKind::Face) {
        setFailure(result, Prerequisites, "cone.source_face_missing",
                   "the cone does not resolve to exactly one source face",
                   {workingFace});
        return result;
    }

    std::vector<FaceSampleUse> allFaceUses;
    std::vector<FaceSampleUse> rimSamples;
    std::optional<FaceSampleUse> apexSample;
    for (const CanonicalBoundary& boundary : boundaries.boundaries) {
        const ExactGeometryClassification* edgeClass =
            reconnaissance.find(boundary.edge);
        const EdgeTopologyRecord* topology = nullptr;
        for (const EdgeTopologyRecord& record :
             imported.working->snapshot.edgeTopology) {
            if (record.id == boundary.edge) {
                topology = &record;
                break;
            }
        }
        const bool degenerate = topology && topology->degenerate;
        const bool rimCandidate = edgeClass &&
            edgeClass->taxonomy == GeometryTaxonomy::Curve &&
            edgeClass->familyCode == "circle" && boundary.closed &&
            !degenerate;

        for (const CanonicalBoundarySample& sample : boundary.samples) {
            for (const CoedgeUvUse& use : sample.faceUses) {
                if (use.face != workingFace) continue;
                FaceSampleUse item{&sample, &use};
                allFaceUses.push_back(item);
                if (degenerate) {
                    if (apexSample) {
                        setFailure(result, BoundaryCoverage,
                                   "cone.apex_ambiguous",
                                   "cone face has more than one singular apex sample use",
                                   {workingFace, boundary.edge});
                        return result;
                    }
                    apexSample = item;
                } else if (rimCandidate) {
                    rimSamples.push_back(item);
                }
            }
        }
    }

    result.validation[BoundaryCoverage].expected = allFaceUses.size();
    if (!apexSample) {
        setFailure(result, BoundaryCoverage, "cone.apex_missing",
                   "apex-cone wall requires one singular apex station",
                   {workingFace});
        return result;
    }
    if (rimSamples.size() < 3) {
        setFailure(result, BoundaryCoverage, "cone.rim_count_insufficient",
                   "apex-cone wall requires a closed circular rim with at least three samples",
                   {workingFace});
        return result;
    }

    // Order the rim by lifted U around the period.
    std::sort(rimSamples.begin(), rimSamples.end(),
              [](const FaceSampleUse& left, const FaceSampleUse& right) {
                  return left.use->liftedUv[0] < right.use->liftedUv[0];
              });

    double rimMeanV = 0.0;
    for (const FaceSampleUse& rim : rimSamples) {
        rimMeanV += rim.use->liftedUv[1];
    }
    rimMeanV /= static_cast<double>(rimSamples.size());
    std::optional<std::array<double, 3>> apexPosition;
    for (const EdgeTopologyRecord& topology :
         imported.working->snapshot.edgeTopology) {
        if (!topology.degenerate) continue;
        const StableId vertexId =
            topology.lowerVertex ? *topology.lowerVertex
                                 : *topology.upperVertex;
        const auto evaluated =
            imported.workingEvaluator->evaluateVertex(vertexId);
        if (evaluated) {
            apexPosition = evaluated.value->position;
            break;
        }
    }
    if (!apexPosition) {
        setFailure(result, BoundaryCoverage, "cone.apex_position_missing",
                   "cone apex topological vertex could not be evaluated",
                   {workingFace});
        return result;
    }
    // Degenerate apex p-curves often collapse to an arbitrary U at the rim V.
    // Choose the domain V endpoint whose surface image is closest to the apex.
    PredicatePoint2 apexUv = apexSample->use->liftedUv;
    if (classification->parameterDomains.size() >= 2 &&
        classification->parameterDomains[1].lower &&
        classification->parameterDomains[1].upper) {
        const double v0 = *classification->parameterDomains[1].lower;
        const double v1 = *classification->parameterDomains[1].upper;
        const PredicatePoint2 uv0{apexUv[0], v0};
        const PredicatePoint2 uv1{apexUv[0], v1};
        const auto at0 =
            imported.workingEvaluator->evaluateSurface(workingFace, uv0);
        const auto at1 =
            imported.workingEvaluator->evaluateSurface(workingFace, uv1);
        double best = std::numeric_limits<double>::infinity();
        if (at0) {
            const double d =
                squaredDistance(at0.value->position, *apexPosition);
            if (d < best) {
                best = d;
                apexUv[1] = v0;
            }
        }
        if (at1) {
            const double d =
                squaredDistance(at1.value->position, *apexPosition);
            if (d < best) {
                best = d;
                apexUv[1] = v1;
            }
        }
        constexpr double kApexUvTolerance = 1e-3;
        if (!std::isfinite(best) ||
            best > kApexUvTolerance * kApexUvTolerance) {
            setFailure(result, BoundaryCoverage, "cone.apex_uv_unresolved",
                       "cone apex UV does not re-evaluate to the apex vertex",
                       {workingFace});
            return result;
        }
    }
    if (!(std::abs(apexUv[1] - rimMeanV) > 1e-12)) {
        setFailure(result, BoundaryCoverage, "cone.apex_v_unresolved",
                   "cone apex V coincides with the rim; singular UV is unusable",
                   {workingFace});
        return result;
    }

    PlanarCdtMesh mesh;
    mesh.workingFace = workingFace;
    mesh.sourceFace = sourceFace;
    mesh.vertices.reserve(rimSamples.size() + 1);

    PlanarTrimVertex apexVertex;
    apexVertex.canonicalVertexIndex =
        apexSample->sample->canonicalVertexIndex;
    apexVertex.uv = apexUv;
    for (const FaceSampleUse& candidate : allFaceUses) {
        if (candidate.sample->canonicalVertexIndex !=
            apexVertex.canonicalVertexIndex) {
            continue;
        }
        apexVertex.boundaryUses.push_back(boundaryUse(candidate));
    }
    mesh.vertices.push_back(std::move(apexVertex));
    const std::uint32_t apexIndex = 0;

    std::vector<std::uint32_t> rimLoop;
    rimLoop.reserve(rimSamples.size());
    std::set<const CoedgeUvUse*> consumed;
    for (const FaceSampleUse& rim : rimSamples) {
        PlanarTrimVertex vertex;
        vertex.canonicalVertexIndex = rim.sample->canonicalVertexIndex;
        vertex.uv = rim.use->liftedUv;
        for (const FaceSampleUse& candidate : allFaceUses) {
            if (candidate.sample->canonicalVertexIndex !=
                vertex.canonicalVertexIndex) {
                continue;
            }
            vertex.boundaryUses.push_back(boundaryUse(candidate));
            consumed.insert(candidate.use);
        }
        rimLoop.push_back(static_cast<std::uint32_t>(mesh.vertices.size()));
        mesh.vertices.push_back(std::move(vertex));
    }
    for (const FaceSampleUse& candidate : allFaceUses) {
        if (candidate.sample->canonicalVertexIndex ==
            mesh.vertices[apexIndex].canonicalVertexIndex) {
            consumed.insert(candidate.use);
        }
    }

    // Seam/generator samples share rim or apex canonical vertices and must
    // already be attached above. Any leftover use is a coverage failure.
    result.validation[BoundaryCoverage].checked = consumed.size();
    if (consumed.size() != allFaceUses.size()) {
        setFailure(result, BoundaryCoverage, "cone.boundary_use_unconsumed",
                   "cone face sample uses remain after apex fan assembly",
                   {workingFace});
        return result;
    }

    mesh.boundaryLoops.push_back(rimLoop);
    const std::size_t count = rimSamples.size();
    mesh.constrainedEdges.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        mesh.constrainedEdges.push_back(
            {rimLoop[index], rimLoop[(index + 1) % count]});
    }

    result.validation[TriangleOrientation].expected = count;
    result.validation[ChordBound].expected = count;
    result.validation[NormalBound].expected = count * 3;
    const double chordSquared = configuration.maximumChordDeviation *
        configuration.maximumChordDeviation;
    const double minimumNormalDot =
        std::cos(configuration.maximumNormalDeviationRadians);
    mesh.triangles.reserve(count);

    const auto apexPos =
        samplePosition(imported, workingFace, mesh.vertices[apexIndex]);
    if (!apexPos) {
        setFailure(result, Prerequisites, "cone.apex_position_failed",
                   "cone apex position could not be evaluated",
                   {workingFace});
        return result;
    }

    for (std::size_t index = 0; index < count; ++index) {
        const std::uint32_t a = rimLoop[index];
        const std::uint32_t b = rimLoop[(index + 1) % count];
        PredicatePoint2 uvA = mesh.vertices[a].uv;
        PredicatePoint2 uvB = mesh.vertices[b].uv;
        uvB[0] = periodicNear(uvB[0], uvA[0], period);
        // Apex UV is singular in U; pin each fan triangle's apex corner to the
        // sector's first rim U so the UV triangle has positive area.
        PredicatePoint2 uvApex = mesh.vertices[apexIndex].uv;
        uvApex[0] = uvA[0];

        const auto posA = samplePosition(imported, workingFace, mesh.vertices[a]);
        const auto posB = samplePosition(imported, workingFace, mesh.vertices[b]);
        if (!posA || !posB) {
            setFailure(result, ChordBound, "cone.rim_position_failed",
                       "a cone rim sample position could not be evaluated",
                       {workingFace});
            return result;
        }

        // Put a rim corner first so assembly's normal probe is not at the
        // singular apex UV (where Geom_Cone normals are unstable). Face
        // orientation swapping is left to assembleCertifiedBoundaryMesh.
        PlanarCdtTriangle triangle{
            workingFace, sourceFace, {a, b, apexIndex},
            std::array<PredicatePoint2, 3>{uvA, uvB, uvApex}};
        if (!orientTriangle(triangle, *predicates, result, workingFace)) {
            return result;
        }
        (void)posA;
        (void)posB;

        // Generators are linear on the cone; the only curved error in a fan
        // triangle is the base-rim circular chord sagitta.
        const PredicatePoint2 baseMidUv{(uvA[0] + uvB[0]) * 0.5,
                                        (uvA[1] + uvB[1]) * 0.5};
        const EvaluationResult<SurfaceEvaluation> baseMid =
            imported.workingEvaluator->evaluateSurface(workingFace,
                                                       baseMidUv);
        const std::array<double, 3> chordMidpoint{
            ((*posA)[0] + (*posB)[0]) * 0.5,
            ((*posA)[1] + (*posB)[1]) * 0.5,
            ((*posA)[2] + (*posB)[2]) * 0.5};
        ++result.validation[ChordBound].checked;
        if (!baseMid ||
            squaredDistance(baseMid.value->position, chordMidpoint) >
                chordSquared) {
            setFailure(result, ChordBound, "cone.chord_bound_exceeded",
                       "a cone rim chord exceeds the requested analytic midpoint chord bound",
                       {workingFace});
            return result;
        }

        const std::optional<std::array<double, 3>> facetNormal =
            unit(triangleNormal(*apexPos, *posA, *posB));
        if (!facetNormal) {
            setFailure(result, NormalBound, "cone.triangle_degenerate",
                       "a cone fan triangle has zero 3D area", {workingFace});
            return result;
        }
        for (const PredicatePoint2& uv :
             {uvApex, uvA, uvB}) {
            const EvaluationResult<SurfaceEvaluation> surface =
                imported.workingEvaluator->evaluateSurface(workingFace, uv);
            ++result.validation[NormalBound].checked;
            if (!surface || !surface.value->unitNormal ||
                std::abs(dot(*facetNormal, *surface.value->unitNormal)) <
                    minimumNormalDot) {
                setFailure(result, NormalBound, "cone.normal_bound_exceeded",
                           "a cone facet exceeds the requested normal-deviation bound",
                           {workingFace});
                return result;
            }
        }
        mesh.triangles.push_back(std::move(triangle));
    }

    if (!std::all_of(result.validation.begin(), result.validation.end(),
                     [](const ConeWallValidationEvidence& evidence) {
                         return evidence.complete();
                     })) {
        setFailure(result, Prerequisites, "cone.validation_incomplete",
                   "cone wall validation coverage is incomplete",
                   {workingFace});
        return result;
    }
    result.value = std::move(mesh);
    return result;
}

}  // namespace weft
