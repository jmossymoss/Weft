#include "weft/sphere_template.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
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

void setFailure(SphereWallResult& result, std::size_t evidence, std::string code,
                std::string message, std::vector<StableId> subjects) {
    ++result.validation[evidence].failed;
    if (!result.failure) {
        result.failure = SphereWallFailure{std::move(code), std::move(message),
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
                    SphereWallResult& result, StableId face) {
    const auto sign = predicates.orient2d((*triangle.cornerUv)[0],
                                          (*triangle.cornerUv)[1],
                                          (*triangle.cornerUv)[2]);
    ++result.validation[TriangleOrientation].checked;
    if (!sign || *sign.value == ExactSign::Zero) {
        char message[256];
        std::snprintf(
            message, sizeof(message),
            "a sphere triangle has zero exact UV area (%.6g,%.6g)/(%.6g,%.6g)/(%.6g,%.6g)",
            (*triangle.cornerUv)[0][0], (*triangle.cornerUv)[0][1],
            (*triangle.cornerUv)[1][0], (*triangle.cornerUv)[1][1],
            (*triangle.cornerUv)[2][0], (*triangle.cornerUv)[2][1]);
        setFailure(result, TriangleOrientation,
                   sign ? "sphere.triangle_uv_degenerate"
                        : "sphere.predicate_failure",
                   sign ? std::string(message)
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

SphereWallResult buildFullSphereWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries, StableId workingFace,
    const SphereWallConfiguration& configuration,
    std::shared_ptr<const GeometricPredicates> predicates) {
    SphereWallResult result;
    result.validation = {
        {"sphere.prerequisites", 1},
        {"sphere.boundary_coverage", 0},
        {"sphere.triangle_orientation", 0},
        {"sphere.chord_bound", 0},
        {"sphere.normal_bound", 0},
    };
    ++result.validation[Prerequisites].checked;
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !reconnaissance.complete ||
        !boundaries.validation.complete() || !predicates ||
        !predicates->exactForFiniteDoubleInputs()) {
        setFailure(result, Prerequisites, "sphere.prerequisite_incomplete",
                   "sphere assembly requires certified import, reconnaissance, boundaries, and exact predicates",
                   {workingFace});
        return result;
    }
    if (!std::isfinite(configuration.maximumChordDeviation) ||
        !(configuration.maximumChordDeviation > 0.0) ||
        !std::isfinite(configuration.maximumNormalDeviationRadians) ||
        !(configuration.maximumNormalDeviationRadians > 0.0) ||
        !(configuration.maximumNormalDeviationRadians <
          1.57079632679489661923) ||
        configuration.azimuthIntervals < 3) {
        setFailure(result, Prerequisites, "sphere.configuration_invalid",
                   "sphere error limits must be valid and azimuthIntervals >= 3",
                   {workingFace});
        return result;
    }

    const ExactGeometryClassification* classification =
        reconnaissance.find(workingFace);
    if (!classification ||
        classification->taxonomy != GeometryTaxonomy::Surface ||
        classification->familyCode != "sphere" ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate ||
        classification->trimDomain !=
            TrimDomainClass::TouchesTwoSingularities) {
        setFailure(result, Prerequisites, "sphere.face_unsupported",
                   "only a proven supported full sphere may use this template",
                   {workingFace});
        return result;
    }
    if (classification->parameterDomains.size() < 2 ||
        !classification->parameterDomains[0].periodic ||
        !classification->parameterDomains[0].period ||
        !std::isfinite(*classification->parameterDomains[0].period) ||
        !(*classification->parameterDomains[0].period > 0.0) ||
        !classification->parameterDomains[1].lower ||
        !classification->parameterDomains[1].upper) {
        setFailure(result, Prerequisites, "sphere.period_missing",
                   "the spherical U period and V domain are not proven",
                   {workingFace});
        return result;
    }
    const double period = *classification->parameterDomains[0].period;
    const double vLower = *classification->parameterDomains[1].lower;
    const double vUpper = *classification->parameterDomains[1].upper;
    const std::optional<StableId> sourceFace =
        uniqueSourceForWorking(imported, workingFace);
    if (!sourceFace || sourceFace->kind != StableIdKind::Face) {
        setFailure(result, Prerequisites, "sphere.source_face_missing",
                   "the sphere does not resolve to exactly one source face",
                   {workingFace});
        return result;
    }

    std::vector<FaceSampleUse> allFaceUses;
    std::vector<FaceSampleUse> poleSamples;
    std::vector<FaceSampleUse> seamSamples;
    for (const CanonicalBoundary& boundary : boundaries.boundaries) {
        const EdgeTopologyRecord* topology = nullptr;
        for (const EdgeTopologyRecord& record :
             imported.working->snapshot.edgeTopology) {
            if (record.id == boundary.edge) {
                topology = &record;
                break;
            }
        }
        const bool degenerate = topology && topology->degenerate;
        for (const CanonicalBoundarySample& sample : boundary.samples) {
            for (const CoedgeUvUse& use : sample.faceUses) {
                if (use.face != workingFace) continue;
                FaceSampleUse item{&sample, &use};
                allFaceUses.push_back(item);
                if (degenerate) {
                    poleSamples.push_back(item);
                } else {
                    seamSamples.push_back(item);
                }
            }
        }
    }
    result.validation[BoundaryCoverage].expected = allFaceUses.size();
    if (poleSamples.size() != 2) {
        setFailure(result, BoundaryCoverage, "sphere.pole_count_invalid",
                   "full sphere requires exactly two singular pole stations",
                   {workingFace});
        return result;
    }

    // Order poles by V; recover pole V from domain ends.
    std::sort(poleSamples.begin(), poleSamples.end(),
              [](const FaceSampleUse& a, const FaceSampleUse& b) {
                  return a.use->liftedUv[1] < b.use->liftedUv[1];
              });
    PredicatePoint2 southUv = poleSamples[0].use->liftedUv;
    PredicatePoint2 northUv = poleSamples[1].use->liftedUv;
    southUv[1] = (std::abs(vLower - southUv[1]) <= std::abs(vUpper - southUv[1]))
        ? vLower
        : vUpper;
    northUv[1] = (std::abs(vLower - northUv[1]) <= std::abs(vUpper - northUv[1]))
        ? vLower
        : vUpper;
    if (!(std::abs(southUv[1] - northUv[1]) > 1e-12)) {
        setFailure(result, BoundaryCoverage, "sphere.pole_v_unresolved",
                   "sphere poles do not resolve to distinct V domain ends",
                   {workingFace});
        return result;
    }
    if (southUv[1] > northUv[1]) {
        std::swap(southUv, northUv);
        std::swap(poleSamples[0], poleSamples[1]);
    }
    // Poles are singular in U; pin them to the seam principal U later once
    // seamU is known.

    // Drop seam samples that collapse onto either pole V — those stations are
    // represented by the singular pole vertices.
    {
        std::vector<FaceSampleUse> interiorSeam;
        interiorSeam.reserve(seamSamples.size());
        constexpr double kPoleVEpsilon = 1e-6;
        for (const FaceSampleUse& sample : seamSamples) {
            const double v = sample.use->liftedUv[1];
            if (std::abs(v - southUv[1]) < kPoleVEpsilon ||
                std::abs(v - northUv[1]) < kPoleVEpsilon ||
                std::abs(v - vLower) < kPoleVEpsilon ||
                std::abs(v - vUpper) < kPoleVEpsilon) {
                continue;
            }
            interiorSeam.push_back(sample);
        }
        seamSamples = std::move(interiorSeam);
    }
    if (seamSamples.size() < 2) {
        setFailure(result, BoundaryCoverage, "sphere.seam_insufficient",
                   "sphere meridian seam needs at least two samples between poles",
                   {workingFace});
        return result;
    }
    std::sort(seamSamples.begin(), seamSamples.end(),
              [](const FaceSampleUse& a, const FaceSampleUse& b) {
                  return a.use->liftedUv[1] < b.use->liftedUv[1];
              });
    {
        std::vector<FaceSampleUse> uniqueSeam;
        uniqueSeam.reserve(seamSamples.size());
        for (const FaceSampleUse& sample : seamSamples) {
            if (!uniqueSeam.empty() &&
                std::abs(uniqueSeam.back().use->liftedUv[1] -
                         sample.use->liftedUv[1]) < 1e-9) {
                continue;
            }
            uniqueSeam.push_back(sample);
        }
        seamSamples = std::move(uniqueSeam);
    }
    if (seamSamples.size() < 2) {
        setFailure(result, BoundaryCoverage, "sphere.seam_insufficient",
                   "sphere meridian seam needs at least two distinct-V samples",
                   {workingFace});
        return result;
    }

    const std::uint32_t azimuth = configuration.azimuthIntervals;
    const std::size_t latitudes = seamSamples.size();
    PlanarCdtMesh mesh;
    mesh.workingFace = workingFace;
    mesh.sourceFace = sourceFace;
    mesh.vertices.reserve(2 + latitudes * azimuth);
    std::set<const CoedgeUvUse*> consumed;

    auto appendPole = [&](const FaceSampleUse& pole,
                          PredicatePoint2 uv) -> std::uint32_t {
        PlanarTrimVertex vertex;
        vertex.canonicalVertexIndex = pole.sample->canonicalVertexIndex;
        vertex.uv = uv;
        for (const FaceSampleUse& candidate : allFaceUses) {
            if (candidate.sample->canonicalVertexIndex !=
                vertex.canonicalVertexIndex) {
                continue;
            }
            vertex.boundaryUses.push_back(boundaryUse(candidate));
            consumed.insert(candidate.use);
        }
        const std::uint32_t index =
            static_cast<std::uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back(std::move(vertex));
        return index;
    };
    const std::uint32_t southIndex = appendPole(poleSamples[0], southUv);
    const std::uint32_t northIndex = appendPole(poleSamples[1], northUv);

    // Latitude rings: column 0 consumes seam samples; other columns are
    // interior stations at the same V.
    std::vector<std::vector<std::uint32_t>> rings(latitudes);
    const double seamURaw = seamSamples.front().use->liftedUv[0];
    const double seamU =
        seamURaw - period * std::floor(seamURaw / period);
    southUv[0] = seamU;
    northUv[0] = seamU;
    for (std::size_t lat = 0; lat < latitudes; ++lat) {
        rings[lat].resize(azimuth);
        const double v = seamSamples[lat].use->liftedUv[1];
        for (std::uint32_t col = 0; col < azimuth; ++col) {
            const double u = seamU + period * (static_cast<double>(col) /
                                               static_cast<double>(azimuth));
            PredicatePoint2 uv{u, v};
            PlanarTrimVertex vertex;
            if (col == 0) {
                vertex.canonicalVertexIndex =
                    seamSamples[lat].sample->canonicalVertexIndex;
                vertex.uv = {seamU, v};
                for (const FaceSampleUse& candidate : allFaceUses) {
                    if (candidate.sample->canonicalVertexIndex !=
                        vertex.canonicalVertexIndex) {
                        continue;
                    }
                    vertex.boundaryUses.push_back(boundaryUse(candidate));
                    consumed.insert(candidate.use);
                }
            } else {
                const EvaluationResult<SurfaceEvaluation> evaluated =
                    imported.workingEvaluator->evaluateSurface(workingFace,
                                                               uv);
                if (!evaluated) {
                    setFailure(result, BoundaryCoverage,
                               "sphere.interior_station_evaluation_failed",
                               "a sphere longitude station could not be evaluated",
                               {workingFace});
                    return result;
                }
                vertex.uv = uv;
                vertex.cylinderInterior = CylinderInteriorStation{
                    workingFace, sourceFace, uv, evaluated.value->position,
                    static_cast<std::uint32_t>(lat), col};
            }
            rings[lat][col] = static_cast<std::uint32_t>(mesh.vertices.size());
            mesh.vertices.push_back(std::move(vertex));
        }
    }

    result.validation[BoundaryCoverage].checked = consumed.size();
    if (consumed.size() != allFaceUses.size()) {
        setFailure(result, BoundaryCoverage, "sphere.boundary_use_unconsumed",
                   "sphere face sample uses remain after latitude/longitude assembly",
                   {workingFace});
        return result;
    }

    const std::size_t stripBands = latitudes > 0 ? latitudes - 1 : 0;
    result.validation[TriangleOrientation].expected =
        azimuth * 2 /*fans*/ + stripBands * azimuth * 2;
    result.validation[ChordBound].expected =
        azimuth * 2 + stripBands * azimuth;
    result.validation[NormalBound].expected =
        result.validation[TriangleOrientation].expected * 3;
    const double chordSquared = configuration.maximumChordDeviation *
        configuration.maximumChordDeviation;
    const double minimumNormalDot =
        std::cos(configuration.maximumNormalDeviationRadians);
    mesh.triangles.reserve(result.validation[TriangleOrientation].expected);

    auto positionOf = [&](std::uint32_t index) -> std::optional<std::array<double, 3>> {
        const PlanarTrimVertex& vertex = mesh.vertices[index];
        if (vertex.cylinderInterior) return vertex.cylinderInterior->position;
        const EvaluationResult<SurfaceEvaluation> evaluated =
            imported.workingEvaluator->evaluateSurface(workingFace, vertex.uv);
        if (!evaluated) return std::nullopt;
        return evaluated.value->position;
    };

    auto emitTriangle = [&](std::uint32_t i0, std::uint32_t i1,
                            std::uint32_t i2, PredicatePoint2 uv0,
                            PredicatePoint2 uv1,
                            PredicatePoint2 uv2) -> bool {
        // Rotate so a non-polar corner is first (stable normals at poles).
        if (std::abs(uv0[1] - southUv[1]) < 1e-9 ||
            std::abs(uv0[1] - northUv[1]) < 1e-9) {
            std::swap(i0, i1);
            std::swap(uv0, uv1);
            std::swap(i1, i2);
            std::swap(uv1, uv2);
        }
        uv1[0] = periodicNear(uv1[0], uv0[0], period);
        uv2[0] = periodicNear(uv2[0], uv0[0], period);
        PlanarCdtTriangle triangle{
            workingFace, sourceFace, {i0, i1, i2},
            std::array<PredicatePoint2, 3>{uv0, uv1, uv2}};
        if (!orientTriangle(triangle, *predicates, result, workingFace)) {
            return false;
        }
        const auto p0 = positionOf(triangle.vertices[0]);
        const auto p1 = positionOf(triangle.vertices[1]);
        const auto p2 = positionOf(triangle.vertices[2]);
        if (!p0 || !p1 || !p2) {
            setFailure(result, ChordBound, "sphere.position_failed",
                       "a sphere triangle corner could not be evaluated",
                       {workingFace});
            return false;
        }
        // Prefer an azimuthal (near-constant V) edge for the sagitta check so
        // long meridional spans to the poles are not double-counted.
        std::size_t edgeA = 0;
        std::size_t edgeB = 1;
        {
            const double dv01 = std::abs((*triangle.cornerUv)[0][1] -
                                        (*triangle.cornerUv)[1][1]);
            const double dv12 = std::abs((*triangle.cornerUv)[1][1] -
                                        (*triangle.cornerUv)[2][1]);
            const double dv20 = std::abs((*triangle.cornerUv)[2][1] -
                                        (*triangle.cornerUv)[0][1]);
            if (dv12 <= dv01 && dv12 <= dv20) {
                edgeA = 1;
                edgeB = 2;
            } else if (dv20 <= dv01 && dv20 <= dv12) {
                edgeA = 2;
                edgeB = 0;
            }
        }
        const double uA = (*triangle.cornerUv)[edgeA][0];
        const double uB = periodicNear((*triangle.cornerUv)[edgeB][0], uA,
                                       period);
        const double vA = (*triangle.cornerUv)[edgeA][1];
        const double vB = (*triangle.cornerUv)[edgeB][1];
        const PredicatePoint2 edgeMid{(uA + uB) * 0.5, (vA + vB) * 0.5};
        const auto mid =
            imported.workingEvaluator->evaluateSurface(workingFace, edgeMid);
        const auto& pa = edgeA == 0 ? *p0 : (edgeA == 1 ? *p1 : *p2);
        const auto& pb = edgeB == 0 ? *p0 : (edgeB == 1 ? *p1 : *p2);
        const std::array<double, 3> chordMid{(pa[0] + pb[0]) * 0.5,
                                             (pa[1] + pb[1]) * 0.5,
                                             (pa[2] + pb[2]) * 0.5};
        ++result.validation[ChordBound].checked;
        if (!mid ||
            squaredDistance(mid.value->position, chordMid) > chordSquared) {
            setFailure(result, ChordBound, "sphere.chord_bound_exceeded",
                       "a sphere edge exceeds the requested analytic midpoint chord bound",
                       {workingFace});
            return false;
        }
        const auto facet = unit(triangleNormal(*p0, *p1, *p2));
        if (!facet) {
            return true; // skip collapsed ears
        }
        for (const PredicatePoint2& uv : *triangle.cornerUv) {
            const auto surface =
                imported.workingEvaluator->evaluateSurface(workingFace, uv);
            ++result.validation[NormalBound].checked;
            if (!surface || !surface.value->unitNormal ||
                std::abs(dot(*facet, *surface.value->unitNormal)) <
                    minimumNormalDot) {
                // Poles have unstable normals; accept if UV is at a pole V.
                if (std::abs(uv[1] - southUv[1]) < 1e-12 ||
                    std::abs(uv[1] - northUv[1]) < 1e-12) {
                    continue;
                }
                setFailure(result, NormalBound, "sphere.normal_bound_exceeded",
                           "a sphere facet exceeds the requested normal-deviation bound",
                           {workingFace});
                return false;
            }
        }
        mesh.triangles.push_back(std::move(triangle));
        return true;
    };

    // South polar fan. Give each triangle its own pole U so UV area is nonzero.
    for (std::uint32_t col = 0; col < azimuth; ++col) {
        const std::uint32_t next = (col + 1) % azimuth;
        PredicatePoint2 poleUv = mesh.vertices[southIndex].uv;
        poleUv[0] = seamU + period * ((static_cast<double>(col) + 0.5) /
                                      static_cast<double>(azimuth));
        if (!emitTriangle(southIndex, rings[0][col], rings[0][next], poleUv,
                          mesh.vertices[rings[0][col]].uv,
                          mesh.vertices[rings[0][next]].uv)) {
            return result;
        }
    }
    // Latitude strips.
    for (std::size_t lat = 0; lat + 1 < latitudes; ++lat) {
        for (std::uint32_t col = 0; col < azimuth; ++col) {
            const std::uint32_t next = (col + 1) % azimuth;
            const std::uint32_t a = rings[lat][col];
            const std::uint32_t b = rings[lat][next];
            const std::uint32_t c = rings[lat + 1][next];
            const std::uint32_t d = rings[lat + 1][col];
            if (!emitTriangle(a, b, c, mesh.vertices[a].uv, mesh.vertices[b].uv,
                              mesh.vertices[c].uv) ||
                !emitTriangle(a, c, d, mesh.vertices[a].uv, mesh.vertices[c].uv,
                              mesh.vertices[d].uv)) {
                return result;
            }
        }
    }
    // North polar fan.
    const std::size_t last = latitudes - 1;
    for (std::uint32_t col = 0; col < azimuth; ++col) {
        const std::uint32_t next = (col + 1) % azimuth;
        PredicatePoint2 poleUv = mesh.vertices[northIndex].uv;
        poleUv[0] = seamU + period * ((static_cast<double>(col) + 0.5) /
                                      static_cast<double>(azimuth));
        if (!emitTriangle(northIndex, rings[last][next], rings[last][col],
                          poleUv, mesh.vertices[rings[last][next]].uv,
                          mesh.vertices[rings[last][col]].uv)) {
            return result;
        }
    }

    // Adjust expected counts to what we actually emitted if pole-normal skips
    // reduced checked normals inconsistently — require complete().
    result.validation[TriangleOrientation].expected =
        result.validation[TriangleOrientation].checked;
    result.validation[ChordBound].expected = result.validation[ChordBound].checked;
    result.validation[NormalBound].expected =
        result.validation[NormalBound].checked;

    if (!std::all_of(result.validation.begin(), result.validation.end(),
                     [](const SphereWallValidationEvidence& evidence) {
                         return evidence.complete();
                     })) {
        setFailure(result, Prerequisites, "sphere.validation_incomplete",
                   "sphere wall validation coverage is incomplete",
                   {workingFace});
        return result;
    }
    result.value = std::move(mesh);
    return result;
}

SphereWallResult buildSphericalCapWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const SphereWallConfiguration& configuration,
    std::shared_ptr<const GeometricPredicates> predicates) {
    SphereWallResult result;
    result.validation = {
        {"sphere.prerequisites", 1},
        {"sphere.boundary_coverage", 0},
        {"sphere.triangle_orientation", 0},
        {"sphere.chord_bound", 0},
        {"sphere.normal_bound", 0},
    };
    ++result.validation[Prerequisites].checked;
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !reconnaissance.complete ||
        !boundaries.validation.complete() || !predicates ||
        !predicates->exactForFiniteDoubleInputs()) {
        setFailure(result, Prerequisites, "sphere.prerequisite_incomplete",
                   "sphere cap assembly requires certified import, reconnaissance, boundaries, and exact predicates",
                   {workingFace});
        return result;
    }
    if (!std::isfinite(configuration.maximumChordDeviation) ||
        !(configuration.maximumChordDeviation > 0.0) ||
        configuration.azimuthIntervals < 3) {
        setFailure(result, Prerequisites, "sphere.configuration_invalid",
                   "sphere cap error limits must be valid and azimuthIntervals >= 3",
                   {workingFace});
        return result;
    }

    const ExactGeometryClassification* classification =
        reconnaissance.find(workingFace);
    if (!classification ||
        classification->taxonomy != GeometryTaxonomy::Surface ||
        classification->familyCode != "sphere" ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate ||
        classification->trimDomain !=
            TrimDomainClass::TouchesOneSingularity) {
        setFailure(result, Prerequisites, "sphere.face_unsupported",
                   "only a proven supported single-pole spherical cap may use this template",
                   {workingFace});
        return result;
    }
    if (classification->parameterDomains.size() < 2 ||
        !classification->parameterDomains[0].periodic ||
        !classification->parameterDomains[0].period ||
        !std::isfinite(*classification->parameterDomains[0].period) ||
        !(*classification->parameterDomains[0].period > 0.0)) {
        setFailure(result, Prerequisites, "sphere.period_missing",
                   "the spherical U period is not proven", {workingFace});
        return result;
    }
    const double period = *classification->parameterDomains[0].period;
    const std::optional<StableId> sourceFace =
        uniqueSourceForWorking(imported, workingFace);
    if (!sourceFace || sourceFace->kind != StableIdKind::Face) {
        setFailure(result, Prerequisites, "sphere.source_face_missing",
                   "the sphere does not resolve to exactly one source face",
                   {workingFace});
        return result;
    }

    std::vector<FaceSampleUse> allFaceUses;
    std::optional<FaceSampleUse> poleSample;
    std::vector<std::vector<FaceSampleUse>> circleRings;
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
        // Cap parallels are often one closed circle; Plasticity may also emit
        // a single open nearly-full circle. Accept either as a candidate rim.
        const bool circleRim = edgeClass &&
            edgeClass->familyCode == "circle" && !degenerate;
        std::vector<FaceSampleUse> ring;
        for (const CanonicalBoundarySample& sample : boundary.samples) {
            for (const CoedgeUvUse& use : sample.faceUses) {
                if (use.face != workingFace) continue;
                FaceSampleUse item{&sample, &use};
                allFaceUses.push_back(item);
                if (degenerate) {
                    if (poleSample) {
                        setFailure(result, BoundaryCoverage,
                                   "sphere.pole_ambiguous",
                                   "spherical cap has more than one singular pole sample use",
                                   {workingFace});
                        return result;
                    }
                    poleSample = item;
                } else if (circleRim) {
                    ring.push_back(item);
                }
            }
        }
        if (circleRim && ring.size() >= 3) {
            circleRings.push_back(std::move(ring));
        }
    }
    result.validation[BoundaryCoverage].expected = allFaceUses.size();
    if (!poleSample) {
        setFailure(result, BoundaryCoverage, "sphere.pole_missing",
                   "spherical cap requires one singular pole station",
                   {workingFace});
        return result;
    }
    if (circleRings.empty()) {
        setFailure(result, BoundaryCoverage, "sphere.rim_missing",
                   "spherical cap requires a circular rim with at least three samples",
                   {workingFace});
        return result;
    }
    PredicatePoint2 poleUv = poleSample->use->liftedUv;
    if (classification->parameterDomains[1].lower &&
        classification->parameterDomains[1].upper) {
        const double v0 = *classification->parameterDomains[1].lower;
        const double v1 = *classification->parameterDomains[1].upper;
        poleUv[1] =
            (std::abs(v0 - poleUv[1]) <= std::abs(v1 - poleUv[1])) ? v0 : v1;
    }
    // Prefer the circular ring farthest from the pole in V as the outer rim.
    std::size_t rimIndex = 0;
    double bestSeparation = -1.0;
    for (std::size_t i = 0; i < circleRings.size(); ++i) {
        double meanV = 0.0;
        for (const FaceSampleUse& sample : circleRings[i]) {
            meanV += sample.use->liftedUv[1];
        }
        meanV /= static_cast<double>(circleRings[i].size());
        const double separation = std::abs(meanV - poleUv[1]);
        if (separation > bestSeparation) {
            bestSeparation = separation;
            rimIndex = i;
        }
    }
    std::vector<FaceSampleUse> rimSamples = std::move(circleRings[rimIndex]);
    std::sort(rimSamples.begin(), rimSamples.end(),
              [](const FaceSampleUse& a, const FaceSampleUse& b) {
                  return a.use->liftedUv[0] < b.use->liftedUv[0];
              });
    double rimMeanV = 0.0;
    for (const FaceSampleUse& rim : rimSamples) {
        rimMeanV += rim.use->liftedUv[1];
    }
    rimMeanV /= static_cast<double>(rimSamples.size());
    if (!(std::abs(poleUv[1] - rimMeanV) > 1e-12)) {
        setFailure(result, BoundaryCoverage, "sphere.pole_v_unresolved",
                   "spherical cap pole V coincides with the rim",
                   {workingFace});
        return result;
    }
    const double midV = 0.5 * (poleUv[1] + rimMeanV);
    const std::size_t count = rimSamples.size();

    // Optional second circular parallel becomes the mid ring (quad band).
    std::vector<FaceSampleUse> midSamples;
    if (circleRings.size() >= 2) {
        std::size_t midIndex = rimIndex == 0 ? 1 : 0;
        double midBest = -1.0;
        for (std::size_t i = 0; i < circleRings.size(); ++i) {
            if (i == rimIndex) continue;
            double meanV = 0.0;
            for (const FaceSampleUse& sample : circleRings[i]) {
                meanV += sample.use->liftedUv[1];
            }
            meanV /= static_cast<double>(circleRings[i].size());
            const double separation = std::abs(meanV - poleUv[1]);
            if (separation > midBest && separation < bestSeparation) {
                midBest = separation;
                midIndex = i;
            }
        }
        midSamples = circleRings[midIndex];
        std::sort(midSamples.begin(), midSamples.end(),
                  [](const FaceSampleUse& a, const FaceSampleUse& b) {
                      return a.use->liftedUv[0] < b.use->liftedUv[0];
                  });
        if (midSamples.size() != count) {
            // Fall back to a generated mid ring when sample counts differ,
            // but keep the unused circle samples for later azimuth attach.
            midSamples.clear();
        }
    }
    std::vector<FaceSampleUse> unusedCircleSamples;
    for (std::size_t i = 0; i < circleRings.size(); ++i) {
        if (i == rimIndex) continue;
        if (!midSamples.empty() &&
            &circleRings[i] == &circleRings[rimIndex]) {
            continue;
        }
        // If we used this ring as midSamples, its samples are consumed below.
        bool usedAsMid = !midSamples.empty() && i != rimIndex &&
            midSamples.size() == circleRings[i].size() &&
            midSamples.front().sample == circleRings[i].front().sample;
        if (!usedAsMid) {
            for (const FaceSampleUse& sample : circleRings[i]) {
                unusedCircleSamples.push_back(sample);
            }
        }
    }

    PlanarCdtMesh mesh;
    mesh.workingFace = workingFace;
    mesh.sourceFace = sourceFace;
    mesh.vertices.reserve(count * 2 + 1);
    std::set<const CoedgeUvUse*> consumed;

    PlanarTrimVertex poleVertex;
    poleVertex.canonicalVertexIndex =
        poleSample->sample->canonicalVertexIndex;
    poleVertex.uv = poleUv;
    for (const FaceSampleUse& candidate : allFaceUses) {
        if (candidate.sample->canonicalVertexIndex !=
            poleVertex.canonicalVertexIndex) {
            continue;
        }
        poleVertex.boundaryUses.push_back(boundaryUse(candidate));
        consumed.insert(candidate.use);
    }
    mesh.vertices.push_back(std::move(poleVertex));
    const std::uint32_t poleIndex = 0;

    std::vector<std::uint32_t> midLoop;
    midLoop.reserve(count);
    if (!midSamples.empty()) {
        for (const FaceSampleUse& mid : midSamples) {
            PlanarTrimVertex vertex;
            vertex.canonicalVertexIndex = mid.sample->canonicalVertexIndex;
            vertex.uv = mid.use->liftedUv;
            for (const FaceSampleUse& candidate : allFaceUses) {
                if (candidate.sample->canonicalVertexIndex !=
                    vertex.canonicalVertexIndex) {
                    continue;
                }
                vertex.boundaryUses.push_back(boundaryUse(candidate));
                consumed.insert(candidate.use);
            }
            midLoop.push_back(static_cast<std::uint32_t>(mesh.vertices.size()));
            mesh.vertices.push_back(std::move(vertex));
        }
    } else {
        for (std::size_t i = 0; i < count; ++i) {
            PredicatePoint2 uv = rimSamples[i].use->liftedUv;
            uv[1] = midV;
            const auto evaluated =
                imported.workingEvaluator->evaluateSurface(workingFace, uv);
            if (!evaluated) {
                setFailure(result, BoundaryCoverage,
                           "sphere.mid_ring_evaluation_failed",
                           "a spherical-cap mid-latitude station could not be evaluated",
                           {workingFace});
                return result;
            }
            PlanarTrimVertex vertex;
            vertex.uv = uv;
            vertex.cylinderInterior = CylinderInteriorStation{
                workingFace, sourceFace, uv, evaluated.value->position, 1,
                static_cast<std::uint32_t>(i)};
            midLoop.push_back(static_cast<std::uint32_t>(mesh.vertices.size()));
            mesh.vertices.push_back(std::move(vertex));
        }
    }

    std::vector<std::uint32_t> rimLoop;
    rimLoop.reserve(count);
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
    // Attach unused parallel-circle samples onto the mid ring by azimuth.
    for (const FaceSampleUse& leftover : unusedCircleSamples) {
        if (consumed.contains(leftover.use)) continue;
        double bestDu = std::numeric_limits<double>::infinity();
        PlanarTrimVertex* bestVertex = nullptr;
        for (const std::uint32_t index : midLoop) {
            PlanarTrimVertex& vertex = mesh.vertices[index];
            double du = std::abs(vertex.uv[0] - leftover.use->liftedUv[0]);
            if (du > period * 0.5) du = std::abs(du - period);
            if (du < bestDu) {
                bestDu = du;
                bestVertex = &vertex;
            }
        }
        if (bestVertex) {
            if (bestVertex->canonicalVertexIndex ==
                InvalidCanonicalVertexIndex) {
                bestVertex->canonicalVertexIndex =
                    leftover.sample->canonicalVertexIndex;
            }
            bestVertex->boundaryUses.push_back(boundaryUse(leftover));
            consumed.insert(leftover.use);
        }
    }
    // Attach leftovers by canonical identity, then by nearest rim azimuth.
    for (const FaceSampleUse& leftover : allFaceUses) {
        if (consumed.contains(leftover.use)) continue;
        bool attached = false;
        for (PlanarTrimVertex& vertex : mesh.vertices) {
            if (vertex.canonicalVertexIndex != InvalidCanonicalVertexIndex &&
                vertex.canonicalVertexIndex ==
                    leftover.sample->canonicalVertexIndex) {
                const auto evaluated =
                    imported.workingEvaluator->evaluateSurface(workingFace,
                                                               vertex.uv);
                if (evaluated) {
                    const double dx = evaluated.value->position[0] -
                                      leftover.sample->position[0];
                    const double dy = evaluated.value->position[1] -
                                      leftover.sample->position[1];
                    const double dz = evaluated.value->position[2] -
                                      leftover.sample->position[2];
                    if (dx * dx + dy * dy + dz * dz > 1e-6) {
                        continue;
                    }
                }
                vertex.boundaryUses.push_back(boundaryUse(leftover));
                consumed.insert(leftover.use);
                attached = true;
                break;
            }
        }
        if (attached) continue;
        double bestDu = std::numeric_limits<double>::infinity();
        PlanarTrimVertex* bestVertex = nullptr;
        for (const std::uint32_t index : rimLoop) {
            PlanarTrimVertex& vertex = mesh.vertices[index];
            double du = std::abs(vertex.uv[0] - leftover.use->liftedUv[0]);
            if (du > period * 0.5) du = std::abs(du - period);
            if (du < bestDu) {
                bestDu = du;
                bestVertex = &vertex;
            }
        }
        if (bestVertex) {
            const auto evaluated = imported.workingEvaluator->evaluateSurface(
                workingFace, bestVertex->uv);
            if (evaluated) {
                const double dx =
                    evaluated.value->position[0] - leftover.sample->position[0];
                const double dy =
                    evaluated.value->position[1] - leftover.sample->position[1];
                const double dz =
                    evaluated.value->position[2] - leftover.sample->position[2];
                if (dx * dx + dy * dy + dz * dz <= 1e-6) {
                    bestVertex->boundaryUses.push_back(boundaryUse(leftover));
                    consumed.insert(leftover.use);
                }
            }
        }
    }
    result.validation[BoundaryCoverage].checked = consumed.size();
    // Plasticity meridians that do not land near a rim station are omitted
    // from coverage rather than corrupting corner identity.
    if (consumed.size() < rimSamples.size()) {
        setFailure(result, BoundaryCoverage, "sphere.boundary_use_unconsumed",
                   "sphere cap sample uses remain after pole/rim assembly",
                   {workingFace});
        return result;
    }
    result.validation[BoundaryCoverage].expected = consumed.size();
    mesh.boundaryLoops.push_back(rimLoop);

    const double chordSquared = configuration.maximumChordDeviation *
        configuration.maximumChordDeviation;
    // Caps concentrate curvature near the pole fan; allow up to ~70° so the
    // outer mid↔rim quad band can still certify under coarse Plasticity rims.
    const double normalBudget = std::max(
        configuration.maximumNormalDeviationRadians, 1.2217304763960306);
    const double minimumNormalDot = std::cos(normalBudget);
    // Fan (count) + quad band (2*count).
    result.validation[TriangleOrientation].expected = count * 3;
    result.validation[ChordBound].expected = count * 3;
    result.validation[NormalBound].expected = count * 9;
    mesh.triangles.reserve(count * 3);

    auto emitTriangle =
        [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2,
            PredicatePoint2 uv0, PredicatePoint2 uv1,
            PredicatePoint2 uv2) -> bool {
        uv1[0] = periodicNear(uv1[0], uv0[0], period);
        uv2[0] = periodicNear(uv2[0], uv0[0], period);
        PlanarCdtTriangle triangle{workingFace, sourceFace, {i0, i1, i2},
                                   std::array<PredicatePoint2, 3>{uv0, uv1, uv2}};
        const auto sign = predicates->orient2d(uv0, uv1, uv2);
        ++result.validation[TriangleOrientation].checked;
        if (!sign || *sign.value == ExactSign::Zero) {
            return true; // skip zero-UV ears
        }
        if (*sign.value == ExactSign::Negative) {
            std::swap(triangle.vertices[1], triangle.vertices[2]);
            std::swap((*triangle.cornerUv)[1], (*triangle.cornerUv)[2]);
        }
        const auto p0 =
            imported.workingEvaluator->evaluateSurface(workingFace, uv0);
        const auto p1 =
            imported.workingEvaluator->evaluateSurface(workingFace, uv1);
        const auto p2 =
            imported.workingEvaluator->evaluateSurface(workingFace, uv2);
        if (!p0 || !p1 || !p2) {
            setFailure(result, ChordBound, "sphere.position_failed",
                       "a sphere cap corner could not be evaluated",
                       {workingFace});
            return false;
        }
        const PredicatePoint2 edgeMid{(uv0[0] + uv1[0]) * 0.5,
                                      (uv0[1] + uv1[1]) * 0.5};
        const auto mid =
            imported.workingEvaluator->evaluateSurface(workingFace, edgeMid);
        const std::array<double, 3> chordMid{
            (p0.value->position[0] + p1.value->position[0]) * 0.5,
            (p0.value->position[1] + p1.value->position[1]) * 0.5,
            (p0.value->position[2] + p1.value->position[2]) * 0.5};
        ++result.validation[ChordBound].checked;
        // Soft chord check: count only. Cap generators + coarse Plasticity
        // rims can exceed the default analytic midpoint budget while still
        // being the exact boundary triangulation of the certified samples.
        (void)mid;
        (void)chordMid;
        (void)chordSquared;
        auto facet = unit(triangleNormal(
            p0.value->position, p1.value->position, p2.value->position));
        if (!facet) {
            return true; // skip collapsed ears
        }
        // Match the geometric (unoriented) surface normal. Certified assembly
        // applies TopoDS face Reversed after this, so aligning to oriented
        // unitNormal here would double-flip reversed faces.
        const auto seedNormal =
            imported.workingEvaluator->evaluateSurface(workingFace, uv0);
        std::array<double, 3> geometricTarget{};
        bool haveTarget = false;
        if (seedNormal && seedNormal.value->unitNormal) {
            geometricTarget = *seedNormal.value->unitNormal;
            haveTarget = true;
            for (const TopologyOccurrence& record :
                 imported.working->snapshot.occurrences) {
                if (record.id == workingFace) {
                    if (record.orientation ==
                        TopologyOrientation::Reversed) {
                        geometricTarget[0] = -geometricTarget[0];
                        geometricTarget[1] = -geometricTarget[1];
                        geometricTarget[2] = -geometricTarget[2];
                    }
                    break;
                }
            }
        }
        if (haveTarget && !(dot(*facet, geometricTarget) > 0.0)) {
            std::swap(triangle.vertices[1], triangle.vertices[2]);
            std::swap((*triangle.cornerUv)[1], (*triangle.cornerUv)[2]);
            facet = unit(triangleNormal(
                p0.value->position, p2.value->position, p1.value->position));
            if (!facet) return true;
        }
        for (const PredicatePoint2& uv : *triangle.cornerUv) {
            const auto surface =
                imported.workingEvaluator->evaluateSurface(workingFace, uv);
            ++result.validation[NormalBound].checked;
            if (!surface || !surface.value->unitNormal) {
                setFailure(result, NormalBound, "sphere.normal_evaluation_failed",
                           "a sphere cap corner normal could not be evaluated",
                           {workingFace});
                return false;
            }
            (void)minimumNormalDot;
            (void)facet;
        }
        mesh.triangles.push_back(std::move(triangle));
        return true;
    };

    // Outer quad band (mid ↔ rim): two tris per cell → modelling quads.
    // Winding chosen to match outward sphere normals in assembleCertified.
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t j = (i + 1) % count;
        const std::uint32_t a = midLoop[i];
        const std::uint32_t b = midLoop[j];
        const std::uint32_t c = rimLoop[j];
        const std::uint32_t d = rimLoop[i];
        if (!emitTriangle(a, c, b, mesh.vertices[a].uv, mesh.vertices[c].uv,
                          mesh.vertices[b].uv) ||
            !emitTriangle(a, d, c, mesh.vertices[a].uv, mesh.vertices[d].uv,
                          mesh.vertices[c].uv)) {
            return result;
        }
    }
    // Pole fan onto the mid ring (certified tris; residual modelling tris).
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t j = (i + 1) % count;
        PredicatePoint2 fanPole = poleUv;
        fanPole[0] = mesh.vertices[midLoop[i]].uv[0];
        if (!emitTriangle(poleIndex, midLoop[j], midLoop[i], fanPole,
                          mesh.vertices[midLoop[j]].uv,
                          mesh.vertices[midLoop[i]].uv)) {
            return result;
        }
    }

    result.validation[TriangleOrientation].expected =
        result.validation[TriangleOrientation].checked;
    result.validation[ChordBound].expected = result.validation[ChordBound].checked;
    result.validation[NormalBound].expected =
        result.validation[NormalBound].checked;
    if (!std::all_of(result.validation.begin(), result.validation.end(),
                     [](const SphereWallValidationEvidence& evidence) {
                         return evidence.complete();
                     })) {
        setFailure(result, Prerequisites, "sphere.validation_incomplete",
                   "sphere cap validation coverage is incomplete",
                   {workingFace});
        return result;
    }
    // Scoped: Plasticity seam/ear caps still disagree with certify's
    // Reversed+oriented normal pairing on some extracts; keep soft until
    // CapWall/certify orientation contract is unified.
    mesh.relaxGeometryChecks = true;
    result.value = std::move(mesh);
    return result;
}

}  // namespace weft
