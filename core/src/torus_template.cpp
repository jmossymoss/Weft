#include "weft/torus_template.hpp"

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

void setFailure(TorusWallResult& result, std::size_t evidence, std::string code,
                std::string message, std::vector<StableId> subjects) {
    ++result.validation[evidence].failed;
    if (!result.failure) {
        result.failure = TorusWallFailure{std::move(code), std::move(message),
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
                    TorusWallResult& result, StableId face) {
    const auto sign = predicates.orient2d((*triangle.cornerUv)[0],
                                          (*triangle.cornerUv)[1],
                                          (*triangle.cornerUv)[2]);
    ++result.validation[TriangleOrientation].checked;
    if (!sign || *sign.value == ExactSign::Zero) {
        setFailure(result, TriangleOrientation,
                   sign ? "torus.triangle_uv_degenerate"
                        : "torus.predicate_failure",
                   sign ? "a torus triangle has zero exact UV area"
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

}  // namespace

TorusWallResult buildFullTorusWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries, StableId workingFace,
    const TorusWallConfiguration& configuration,
    std::shared_ptr<const GeometricPredicates> predicates) {
    TorusWallResult result;
    result.validation = {
        {"torus.prerequisites", 1},
        {"torus.boundary_coverage", 0},
        {"torus.triangle_orientation", 0},
        {"torus.chord_bound", 0},
        {"torus.normal_bound", 0},
    };
    ++result.validation[Prerequisites].checked;
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !reconnaissance.complete ||
        !boundaries.validation.complete() || !predicates ||
        !predicates->exactForFiniteDoubleInputs()) {
        setFailure(result, Prerequisites, "torus.prerequisite_incomplete",
                   "torus assembly requires certified import, reconnaissance, boundaries, and exact predicates",
                   {workingFace});
        return result;
    }
    if (!std::isfinite(configuration.maximumChordDeviation) ||
        !(configuration.maximumChordDeviation > 0.0) ||
        configuration.majorIntervals < 3 || configuration.minorIntervals < 3) {
        setFailure(result, Prerequisites, "torus.configuration_invalid",
                   "torus intervals must be >= 3 with positive chord bound",
                   {workingFace});
        return result;
    }

    const ExactGeometryClassification* classification =
        reconnaissance.find(workingFace);
    if (!classification ||
        classification->taxonomy != GeometryTaxonomy::Surface ||
        classification->familyCode != "torus" ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate) {
        setFailure(result, Prerequisites, "torus.face_unsupported",
                   "only a proven supported torus face may use this template",
                   {workingFace});
        return result;
    }
    if (classification->parameterDomains.size() < 2 ||
        !classification->parameterDomains[0].periodic ||
        !classification->parameterDomains[1].periodic ||
        !classification->parameterDomains[0].period ||
        !classification->parameterDomains[1].period) {
        setFailure(result, Prerequisites, "torus.period_missing",
                   "torus requires proven U and V periods", {workingFace});
        return result;
    }
    const double periodU = *classification->parameterDomains[0].period;
    const double periodV = *classification->parameterDomains[1].period;
    const std::optional<StableId> sourceFace =
        uniqueSourceForWorking(imported, workingFace);
    if (!sourceFace || sourceFace->kind != StableIdKind::Face) {
        setFailure(result, Prerequisites, "torus.source_face_missing",
                   "the torus does not resolve to exactly one source face",
                   {workingFace});
        return result;
    }

    std::vector<FaceSampleUse> allFaceUses;
    std::set<const CoedgeUvUse*> seenUses;
    for (const CanonicalBoundary& boundary : boundaries.boundaries) {
        for (const CanonicalBoundarySample& sample : boundary.samples) {
            for (const CoedgeUvUse& use : sample.faceUses) {
                if (use.face != workingFace) continue;
                if (!seenUses.insert(&use).second) continue;
                allFaceUses.push_back({&sample, &use});
            }
        }
    }
    result.validation[BoundaryCoverage].expected = allFaceUses.size();
    if (allFaceUses.empty()) {
        setFailure(result, BoundaryCoverage, "torus.boundary_empty",
                   "torus face has no boundary UV uses", {workingFace});
        return result;
    }

    // Prefer seam sample cardinalities when they are denser than the request.
    std::uint32_t seamUCount = 0;
    std::uint32_t seamVCount = 0;
    for (const CanonicalBoundary& boundary : boundaries.boundaries) {
        std::size_t faceSamples = 0;
        double uRange = 0.0;
        double vRange = 0.0;
        PredicatePoint2 firstUv{};
        bool haveFirst = false;
        for (const CanonicalBoundarySample& sample : boundary.samples) {
            for (const CoedgeUvUse& use : sample.faceUses) {
                if (use.face != workingFace) continue;
                ++faceSamples;
                if (!haveFirst) {
                    firstUv = use.liftedUv;
                    haveFirst = true;
                } else {
                    uRange = std::max(
                        uRange,
                        std::abs(periodicNear(use.liftedUv[0], firstUv[0],
                                              periodU) -
                                 firstUv[0]));
                    vRange = std::max(
                        vRange,
                        std::abs(periodicNear(use.liftedUv[1], firstUv[1],
                                              periodV) -
                                 firstUv[1]));
                }
            }
        }
        if (faceSamples == 0) continue;
        if (uRange >= vRange) {
            seamUCount = std::max<std::uint32_t>(
                seamUCount, static_cast<std::uint32_t>(faceSamples));
        } else {
            seamVCount = std::max<std::uint32_t>(
                seamVCount, static_cast<std::uint32_t>(faceSamples));
        }
    }
    constexpr std::uint32_t kMaxIntervals = 48;
    const std::uint32_t nu = std::min(
        kMaxIntervals,
        std::max(configuration.majorIntervals,
                 std::max<std::uint32_t>(3, seamUCount)));
    const std::uint32_t nv = std::min(
        kMaxIntervals,
        std::max(configuration.minorIntervals,
                 std::max<std::uint32_t>(3, seamVCount)));
    PlanarCdtMesh mesh;
    mesh.workingFace = workingFace;
    mesh.sourceFace = sourceFace;
    mesh.vertices.resize(static_cast<std::size_t>(nu) * nv);
    std::set<const CoedgeUvUse*> consumed;

    auto indexOf = [&](std::uint32_t i, std::uint32_t j) -> std::uint32_t {
        return i * nv + j;
    };

    for (std::uint32_t i = 0; i < nu; ++i) {
        for (std::uint32_t j = 0; j < nv; ++j) {
            PredicatePoint2 uv{periodU * (static_cast<double>(i) / nu),
                               periodV * (static_cast<double>(j) / nv)};
            PlanarTrimVertex vertex;
            vertex.uv = uv;
            const EvaluationResult<SurfaceEvaluation> evaluated =
                imported.workingEvaluator->evaluateSurface(workingFace, uv);
            if (!evaluated) {
                setFailure(result, BoundaryCoverage,
                           "torus.interior_station_evaluation_failed",
                           "a torus grid station could not be evaluated",
                           {workingFace});
                return result;
            }
            vertex.cylinderInterior = CylinderInteriorStation{
                workingFace, sourceFace, uv, evaluated.value->position, i, j};
            mesh.vertices[indexOf(i, j)] = std::move(vertex);
        }
    }

    // Attach every boundary use to the nearest grid station while keeping the
    // dual-periodic interior station as the position authority.
    for (const FaceSampleUse& candidate : allFaceUses) {
        if (consumed.contains(candidate.use)) continue;
        std::uint32_t bestIndex = 0;
        double best = std::numeric_limits<double>::infinity();
        for (std::uint32_t i = 0; i < nu; ++i) {
            for (std::uint32_t j = 0; j < nv; ++j) {
                const PredicatePoint2& uv =
                    mesh.vertices[indexOf(i, j)].uv;
                const double du = periodicNear(candidate.use->liftedUv[0],
                                               uv[0], periodU) -
                    uv[0];
                const double dv = periodicNear(candidate.use->liftedUv[1],
                                               uv[1], periodV) -
                    uv[1];
                const double score = du * du + dv * dv;
                if (score < best) {
                    best = score;
                    bestIndex = indexOf(i, j);
                }
            }
        }
        const double cell =
            (periodU / nu) * (periodU / nu) + (periodV / nv) * (periodV / nv);
        if (!(best <= cell)) {
            setFailure(result, BoundaryCoverage, "torus.seam_sample_unmatched",
                       "a torus seam sample does not land on the dual-periodic grid",
                       {workingFace, candidate.sample->workingEdge});
            return result;
        }
        PlanarTrimVertex& vertex = mesh.vertices[bestIndex];
        if (vertex.canonicalVertexIndex == InvalidCanonicalVertexIndex) {
            vertex.canonicalVertexIndex =
                candidate.sample->canonicalVertexIndex;
            vertex.uv = candidate.use->liftedUv;
            // Reconcile the dual-periodic station to the exact seam sample so
            // assembly accepts boundary+interior provenance together.
            vertex.cylinderInterior = CylinderInteriorStation{
                workingFace, sourceFace, vertex.uv, candidate.sample->position,
                vertex.cylinderInterior ? vertex.cylinderInterior->axialRing
                                        : 0,
                vertex.cylinderInterior
                    ? vertex.cylinderInterior->azimuthColumn
                    : 0};
        } else if (vertex.canonicalVertexIndex !=
                   candidate.sample->canonicalVertexIndex) {
            // Distinct seam sample IDs at one corner: keep uses only when the
            // 3D sample positions match within a tiny absolute tolerance.
            const auto& left = vertex.cylinderInterior->position;
            const auto& right = candidate.sample->position;
            const double err2 = squaredDistance(left, right);
            if (!(err2 <= 1e-24)) {
                setFailure(result, BoundaryCoverage,
                           "torus.seam_identity_conflict",
                           "two torus seam identities disagree at one grid corner",
                           {workingFace, candidate.sample->workingEdge});
                return result;
            }
        }
        vertex.boundaryUses.push_back(boundaryUse(candidate));
        consumed.insert(candidate.use);
    }

    result.validation[BoundaryCoverage].checked = consumed.size();
    if (consumed.size() != allFaceUses.size()) {
        setFailure(result, BoundaryCoverage, "torus.boundary_use_unconsumed",
                   "torus seam samples remain after dual-periodic grid assembly",
                   {workingFace});
        return result;
    }

    const std::size_t triCount = static_cast<std::size_t>(nu) * nv * 2;
    result.validation[TriangleOrientation].expected = triCount;
    result.validation[ChordBound].expected = triCount;
    result.validation[NormalBound].expected = triCount * 3;
    const double chordSquared = configuration.maximumChordDeviation *
        configuration.maximumChordDeviation;
    const double minimumNormalDot =
        std::cos(configuration.maximumNormalDeviationRadians);
    mesh.triangles.reserve(triCount);

    auto positionOf = [&](std::uint32_t index)
        -> std::optional<std::array<double, 3>> {
        const PlanarTrimVertex& vertex = mesh.vertices[index];
        if (vertex.cylinderInterior) return vertex.cylinderInterior->position;
        const EvaluationResult<SurfaceEvaluation> evaluated =
            imported.workingEvaluator->evaluateSurface(workingFace, vertex.uv);
        if (!evaluated) return std::nullopt;
        return evaluated.value->position;
    };

    for (std::uint32_t i = 0; i < nu; ++i) {
        for (std::uint32_t j = 0; j < nv; ++j) {
            const std::uint32_t i1 = (i + 1) % nu;
            const std::uint32_t j1 = (j + 1) % nv;
            const std::uint32_t a = indexOf(i, j);
            const std::uint32_t b = indexOf(i1, j);
            const std::uint32_t c = indexOf(i1, j1);
            const std::uint32_t d = indexOf(i, j1);
            PredicatePoint2 uvA = mesh.vertices[a].uv;
            PredicatePoint2 uvB = mesh.vertices[b].uv;
            PredicatePoint2 uvC = mesh.vertices[c].uv;
            PredicatePoint2 uvD = mesh.vertices[d].uv;
            uvB[0] = periodicNear(uvB[0], uvA[0], periodU);
            uvC[0] = periodicNear(uvC[0], uvA[0], periodU);
            uvD[0] = periodicNear(uvD[0], uvA[0], periodU);
            uvB[1] = periodicNear(uvB[1], uvA[1], periodV);
            uvC[1] = periodicNear(uvC[1], uvA[1], periodV);
            uvD[1] = periodicNear(uvD[1], uvA[1], periodV);

            PlanarCdtTriangle tris[2] = {
                {workingFace, sourceFace, {a, b, c},
                 std::array<PredicatePoint2, 3>{uvA, uvB, uvC}},
                {workingFace, sourceFace, {a, c, d},
                 std::array<PredicatePoint2, 3>{uvA, uvC, uvD}},
            };
            for (PlanarCdtTriangle& tri : tris) {
                if (!orientTriangle(tri, *predicates, result, workingFace)) {
                    return result;
                }
                const auto p0 = positionOf(tri.vertices[0]);
                const auto p1 = positionOf(tri.vertices[1]);
                const auto p2 = positionOf(tri.vertices[2]);
                if (!p0 || !p1 || !p2) {
                    setFailure(result, ChordBound, "torus.position_failed",
                               "a torus triangle corner could not be evaluated",
                               {workingFace});
                    return result;
                }
                const double u0 = (*tri.cornerUv)[0][0];
                const double u1 = periodicNear((*tri.cornerUv)[1][0], u0, periodU);
                const double v0 = (*tri.cornerUv)[0][1];
                const double v1 = periodicNear((*tri.cornerUv)[1][1], v0, periodV);
                const PredicatePoint2 edgeMid{(u0 + u1) * 0.5, (v0 + v1) * 0.5};
                const auto mid = imported.workingEvaluator->evaluateSurface(
                    workingFace, edgeMid);
                const std::array<double, 3> chordMid{
                    ((*p0)[0] + (*p1)[0]) * 0.5, ((*p0)[1] + (*p1)[1]) * 0.5,
                    ((*p0)[2] + (*p1)[2]) * 0.5};
                ++result.validation[ChordBound].checked;
                if (!mid ||
                    squaredDistance(mid.value->position, chordMid) >
                        chordSquared) {
                    setFailure(result, ChordBound, "torus.chord_bound_exceeded",
                               "a torus edge exceeds the requested analytic midpoint chord bound",
                               {workingFace});
                    return result;
                }
                const auto facet = unit(triangleNormal(*p0, *p1, *p2));
                if (!facet) {
                    setFailure(result, NormalBound, "torus.triangle_degenerate",
                               "a torus triangle has zero 3D area",
                               {workingFace});
                    return result;
                }
                for (const PredicatePoint2& uv : *tri.cornerUv) {
                    const auto surface =
                        imported.workingEvaluator->evaluateSurface(workingFace,
                                                                   uv);
                    ++result.validation[NormalBound].checked;
                    if (!surface || !surface.value->unitNormal ||
                        std::abs(dot(*facet, *surface.value->unitNormal)) <
                            minimumNormalDot) {
                        setFailure(result, NormalBound,
                                   "torus.normal_bound_exceeded",
                                   "a torus facet exceeds the requested normal-deviation bound",
                                   {workingFace});
                        return result;
                    }
                }
                mesh.triangles.push_back(std::move(tri));
            }
        }
    }

    if (!std::all_of(result.validation.begin(), result.validation.end(),
                     [](const TorusWallValidationEvidence& evidence) {
                         return evidence.complete();
                     })) {
        setFailure(result, Prerequisites, "torus.validation_incomplete",
                   "torus wall validation coverage is incomplete",
                   {workingFace});
        return result;
    }
    result.value = std::move(mesh);
    return result;
}

}  // namespace weft
