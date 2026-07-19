#include "weft/mapped_template.hpp"

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

void setFailure(MappedPatchResult& result, std::size_t evidence,
                std::string code, std::string message,
                std::vector<StableId> subjects) {
    ++result.validation[evidence].failed;
    if (!result.failure) {
        result.failure = MappedPatchFailure{std::move(code), std::move(message),
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

bool hasCondition(const ExactGeometryClassification& record, const char* code) {
    return std::find(record.conditionCodes.begin(), record.conditionCodes.end(),
                     code) != record.conditionCodes.end();
}

bool orientTriangle(PlanarCdtTriangle& triangle,
                    const GeometricPredicates& predicates,
                    MappedPatchResult& result, StableId face) {
    const auto sign = predicates.orient2d((*triangle.cornerUv)[0],
                                          (*triangle.cornerUv)[1],
                                          (*triangle.cornerUv)[2]);
    ++result.validation[TriangleOrientation].checked;
    if (!sign || *sign.value == ExactSign::Zero) {
        setFailure(result, TriangleOrientation,
                   sign ? "mapped.triangle_uv_degenerate"
                        : "mapped.predicate_failure",
                   sign ? "a mapped triangle has zero exact UV area"
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

MappedPatchResult buildMappedFourSidedPatch(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries, StableId workingFace,
    const MappedPatchConfiguration& configuration,
    std::shared_ptr<const GeometricPredicates> predicates) {
    MappedPatchResult result;
    result.validation = {
        {"mapped.prerequisites", 1},
        {"mapped.boundary_coverage", 0},
        {"mapped.triangle_orientation", 0},
        {"mapped.chord_bound", 0},
        {"mapped.normal_bound", 0},
    };
    ++result.validation[Prerequisites].checked;
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !reconnaissance.complete ||
        !boundaries.validation.complete() || !predicates ||
        !predicates->exactForFiniteDoubleInputs()) {
        setFailure(result, Prerequisites, "mapped.prerequisite_incomplete",
                   "mapped assembly requires certified import, reconnaissance, boundaries, and exact predicates",
                   {workingFace});
        return result;
    }
    if (configuration.uIntervals < 1 || configuration.vIntervals < 1 ||
        !std::isfinite(configuration.maximumChordDeviation) ||
        !(configuration.maximumChordDeviation > 0.0)) {
        setFailure(result, Prerequisites, "mapped.configuration_invalid",
                   "mapped patch intervals must be positive with a valid chord bound",
                   {workingFace});
        return result;
    }

    const ExactGeometryClassification* classification =
        reconnaissance.find(workingFace);
    if (!classification ||
        classification->taxonomy != GeometryTaxonomy::Surface ||
        !(hasCondition(*classification, "mapped.four_sided_candidate") ||
          hasCondition(*classification, "freeform.uv_grid_candidate")) ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate) {
        setFailure(result, Prerequisites, "mapped.face_unsupported",
                   "only a supported mapped/freeform UV-grid candidate may use this template",
                   {workingFace});
        return result;
    }
    if (classification->parameterDomains.size() < 2 ||
        !classification->parameterDomains[0].lower ||
        !classification->parameterDomains[0].upper ||
        !classification->parameterDomains[1].lower ||
        !classification->parameterDomains[1].upper) {
        setFailure(result, Prerequisites, "mapped.domain_missing",
                   "mapped face UV domain bounds are incomplete",
                   {workingFace});
        return result;
    }
    const double u0 = *classification->parameterDomains[0].lower;
    const double u1 = *classification->parameterDomains[0].upper;
    const double v0 = *classification->parameterDomains[1].lower;
    const double v1 = *classification->parameterDomains[1].upper;
    if (!(u1 > u0) || !(v1 > v0)) {
        setFailure(result, Prerequisites, "mapped.domain_invalid",
                   "mapped face UV domain must be positively oriented",
                   {workingFace});
        return result;
    }
    const std::optional<StableId> sourceFace =
        uniqueSourceForWorking(imported, workingFace);
    if (!sourceFace || sourceFace->kind != StableIdKind::Face) {
        setFailure(result, Prerequisites, "mapped.source_face_missing",
                   "the mapped face does not resolve to exactly one source face",
                   {workingFace});
        return result;
    }

    std::vector<FaceSampleUse> allFaceUses;
    std::set<const CoedgeUvUse*> seen;
    for (const CanonicalBoundary& boundary : boundaries.boundaries) {
        for (const CanonicalBoundarySample& sample : boundary.samples) {
            for (const CoedgeUvUse& use : sample.faceUses) {
                if (use.face != workingFace) continue;
                if (!seen.insert(&use).second) continue;
                allFaceUses.push_back({&sample, &use});
            }
        }
    }
    result.validation[BoundaryCoverage].expected = allFaceUses.size();
    if (allFaceUses.empty()) {
        setFailure(result, BoundaryCoverage, "mapped.boundary_empty",
                   "mapped face has no boundary UV uses", {workingFace});
        return result;
    }

    const std::uint32_t nu = configuration.uIntervals;
    const std::uint32_t nv = configuration.vIntervals;
    const std::uint32_t nuVerts = nu + 1;
    const std::uint32_t nvVerts = nv + 1;
    PlanarCdtMesh mesh;
    mesh.workingFace = workingFace;
    mesh.sourceFace = sourceFace;
    mesh.vertices.resize(static_cast<std::size_t>(nuVerts) * nvVerts);
    auto indexOf = [&](std::uint32_t i, std::uint32_t j) -> std::uint32_t {
        return i * nvVerts + j;
    };

    for (std::uint32_t i = 0; i < nuVerts; ++i) {
        for (std::uint32_t j = 0; j < nvVerts; ++j) {
            const double su = static_cast<double>(i) / nu;
            const double sv = static_cast<double>(j) / nv;
            PredicatePoint2 uv{u0 + (u1 - u0) * su, v0 + (v1 - v0) * sv};
            const EvaluationResult<SurfaceEvaluation> evaluated =
                imported.workingEvaluator->evaluateSurface(workingFace, uv);
            if (!evaluated) {
                setFailure(result, BoundaryCoverage,
                           "mapped.grid_evaluation_failed",
                           "a mapped grid station could not be evaluated",
                           {workingFace});
                return result;
            }
            PlanarTrimVertex vertex;
            vertex.uv = uv;
            vertex.cylinderInterior = CylinderInteriorStation{
                workingFace, sourceFace, uv, evaluated.value->position, i, j};
            mesh.vertices[indexOf(i, j)] = std::move(vertex);
        }
    }

    const double cell = (u1 - u0) / nu;
    const double cellV = (v1 - v0) / nv;
    const double matchBound = cell * cell + cellV * cellV;
    std::set<const CoedgeUvUse*> consumed;
    for (const FaceSampleUse& candidate : allFaceUses) {
        std::uint32_t bestIndex = 0;
        double best = std::numeric_limits<double>::infinity();
        for (std::uint32_t i = 0; i < nuVerts; ++i) {
            for (std::uint32_t j = 0; j < nvVerts; ++j) {
                // Prefer boundary grid stations for seam samples.
                const bool onBoundary =
                    i == 0 || j == 0 || i + 1 == nuVerts || j + 1 == nvVerts;
                if (!onBoundary) continue;
                const std::uint32_t idx = indexOf(i, j);
                const PlanarTrimVertex& station = mesh.vertices[idx];
                // Keep matching injective for distinct geometric corners:
                // allow a second sample ID only when it shares this station's
                // 3D position (split-rail endpoints after STEP).
                if (station.canonicalVertexIndex !=
                        InvalidCanonicalVertexIndex &&
                    station.canonicalVertexIndex !=
                        candidate.sample->canonicalVertexIndex) {
                    bool sameCorner = false;
                    for (const PlanarTrimBoundaryUse& existing :
                         station.boundaryUses) {
                        const CanonicalBoundary* boundary =
                            boundaries.find(existing.workingEdge);
                        if (!boundary ||
                            existing.sample.ordinal >=
                                boundary->samples.size()) {
                            continue;
                        }
                        const CanonicalBoundarySample& owned =
                            boundary->samples[existing.sample.ordinal];
                        if (owned.position == candidate.sample->position) {
                            sameCorner = true;
                            break;
                        }
                    }
                    if (!sameCorner) continue;
                }
                const PredicatePoint2& uv = station.uv;
                const double du = candidate.use->liftedUv[0] - uv[0];
                const double dv = candidate.use->liftedUv[1] - uv[1];
                const double score = du * du + dv * dv;
                if (score < best) {
                    best = score;
                    bestIndex = idx;
                }
            }
        }
        if (!(best <= matchBound)) {
            // Route to UV-trim via caller fallback — do not soft-skip seams.
            setFailure(result, BoundaryCoverage,
                       "mapped.seam_sample_unmatched",
                       "a mapped boundary sample does not land on the UV grid border",
                       {workingFace, candidate.sample->workingEdge});
            return result;
        }
        PlanarTrimVertex& vertex = mesh.vertices[bestIndex];
        if (vertex.canonicalVertexIndex == InvalidCanonicalVertexIndex) {
            vertex.canonicalVertexIndex =
                candidate.sample->canonicalVertexIndex;
            vertex.uv = candidate.use->liftedUv;
            // Boundary-owned stations must not carry face-local interior
            // provenance — shared rail vertices appear on multiple faces.
            vertex.cylinderInterior = std::nullopt;
        }
        // Adjacent rails can land on one corner with the same sample ID.
        // Keep the first canonical owner; still record every UV use so
        // interval consumption can account for all seam samples.
        vertex.boundaryUses.push_back(boundaryUse(candidate));
        vertex.cylinderInterior = std::nullopt;
        consumed.insert(candidate.use);
    }
    result.validation[BoundaryCoverage].checked = consumed.size();
    if (consumed.size() != allFaceUses.size()) {
        setFailure(result, BoundaryCoverage, "mapped.boundary_use_unconsumed",
                   "mapped boundary samples remain after UV grid assembly",
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
            const std::uint32_t a = indexOf(i, j);
            const std::uint32_t b = indexOf(i + 1, j);
            const std::uint32_t c = indexOf(i + 1, j + 1);
            const std::uint32_t d = indexOf(i, j + 1);
            PlanarCdtTriangle tris[2] = {
                {workingFace, sourceFace, {a, b, c},
                 std::array<PredicatePoint2, 3>{mesh.vertices[a].uv,
                                                mesh.vertices[b].uv,
                                                mesh.vertices[c].uv}},
                {workingFace, sourceFace, {a, c, d},
                 std::array<PredicatePoint2, 3>{mesh.vertices[a].uv,
                                                mesh.vertices[c].uv,
                                                mesh.vertices[d].uv}},
            };
            for (PlanarCdtTriangle& tri : tris) {
                if (!orientTriangle(tri, *predicates, result, workingFace)) {
                    return result;
                }
                const auto p0 = positionOf(tri.vertices[0]);
                const auto p1 = positionOf(tri.vertices[1]);
                const auto p2 = positionOf(tri.vertices[2]);
                if (!p0 || !p1 || !p2) {
                    setFailure(result, ChordBound, "mapped.position_failed",
                               "a mapped triangle corner could not be evaluated",
                               {workingFace});
                    return result;
                }
                const PredicatePoint2 edgeMid{
                    ((*tri.cornerUv)[0][0] + (*tri.cornerUv)[1][0]) * 0.5,
                    ((*tri.cornerUv)[0][1] + (*tri.cornerUv)[1][1]) * 0.5};
                const auto mid = imported.workingEvaluator->evaluateSurface(
                    workingFace, edgeMid);
                const std::array<double, 3> chordMid{
                    ((*p0)[0] + (*p1)[0]) * 0.5, ((*p0)[1] + (*p1)[1]) * 0.5,
                    ((*p0)[2] + (*p1)[2]) * 0.5};
                ++result.validation[ChordBound].checked;
                if (!mid ||
                    squaredDistance(mid.value->position, chordMid) >
                        chordSquared) {
                    setFailure(result, ChordBound, "mapped.chord_bound_exceeded",
                               "a mapped edge exceeds the requested analytic midpoint chord bound",
                               {workingFace});
                    return result;
                }
                const auto facet = unit(triangleNormal(*p0, *p1, *p2));
                if (!facet) {
                    setFailure(result, NormalBound, "mapped.triangle_degenerate",
                               "a mapped triangle has zero 3D area",
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
                                   "mapped.normal_bound_exceeded",
                                   "a mapped facet exceeds the requested normal-deviation bound",
                                   {workingFace});
                        return result;
                    }
                }
                mesh.triangles.push_back(std::move(tri));
            }
        }
    }

    if (!std::all_of(result.validation.begin(), result.validation.end(),
                     [](const MappedPatchValidationEvidence& evidence) {
                         return evidence.complete();
                     })) {
        setFailure(result, Prerequisites, "mapped.validation_incomplete",
                   "mapped patch validation coverage is incomplete",
                   {workingFace});
        return result;
    }
    // Freeform mapped lattices still need scoped certify soft until UV
    // orientation is proven hard on industrial B-splines.
    if (hasCondition(*classification, "freeform.uv_grid_candidate") ||
        hasCondition(*classification, "freeform.uv_trim_candidate") ||
        hasCondition(*classification, "freeform.general_attempted")) {
        mesh.relaxGeometryChecks = true;
    }
    result.value = std::move(mesh);
    return result;
}

}  // namespace weft
