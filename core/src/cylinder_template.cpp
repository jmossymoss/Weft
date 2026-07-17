#include "weft/cylinder_template.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace weft {
namespace {

enum EvidenceIndex : std::size_t {
    Prerequisites = 0,
    BoundaryCoverage,
    Registration,
    TriangleOrientation,
    ChordBound,
    NormalBound,
};

struct FaceSampleUse {
    const CanonicalBoundarySample* sample = nullptr;
    const CoedgeUvUse* use = nullptr;
};

struct Ring {
    const CanonicalBoundary* boundary = nullptr;
    std::vector<FaceSampleUse> samples;
    double meanV = 0.0;
};

void setFailure(CylinderWallResult& result, std::size_t evidence,
                std::string code, std::string message,
                std::vector<StableId> subjects) {
    ++result.validation[evidence].failed;
    if (!result.failure) {
        result.failure = CylinderWallFailure{
            std::move(code), std::move(message), std::move(subjects)};
    }
}

std::optional<StableId> uniqueSourceForWorking(
    const ImportedModel& imported, StableId working) {
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

std::array<double, 3> triangleNormal(
    const std::array<double, 3>& a, const std::array<double, 3>& b,
    const std::array<double, 3>& c) {
    const std::array<double, 3> ab{b[0] - a[0], b[1] - a[1],
                                   b[2] - a[2]};
    const std::array<double, 3> ac{c[0] - a[0], c[1] - a[1],
                                   c[2] - a[2]};
    return {ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]};
}

double dot(const std::array<double, 3>& first,
           const std::array<double, 3>& second) {
    return first[0] * second[0] + first[1] * second[1] +
        first[2] * second[2];
}

std::optional<std::array<double, 3>> unit(
    const std::array<double, 3>& vector) {
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
                    CylinderWallResult& result, StableId face) {
    const auto sign = predicates.orient2d(
        (*triangle.cornerUv)[0], (*triangle.cornerUv)[1],
        (*triangle.cornerUv)[2]);
    ++result.validation[TriangleOrientation].checked;
    if (!sign || *sign.value == ExactSign::Zero) {
        setFailure(result, TriangleOrientation,
                   sign ? "cylinder.triangle_uv_degenerate"
                        : "cylinder.predicate_failure",
                   sign ? "a cylinder triangle has zero exact UV area"
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

CylinderWallResult buildFullCylinderWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const CylinderWallConfiguration& configuration,
    std::shared_ptr<const GeometricPredicates> predicates) {
    CylinderWallResult result;
    result.validation = {
        {"cylinder.prerequisites", 1},
        {"cylinder.boundary_coverage", 0},
        {"cylinder.azimuth_registration", 0},
        {"cylinder.triangle_orientation", 0},
        {"cylinder.chord_bound", 0},
        {"cylinder.normal_bound", 0},
    };
    ++result.validation[Prerequisites].checked;
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !reconnaissance.complete ||
        !boundaries.validation.complete() || !predicates ||
        !predicates->exactForFiniteDoubleInputs()) {
        setFailure(result, Prerequisites, "cylinder.prerequisite_incomplete",
                   "cylinder assembly requires certified import, reconnaissance, boundaries, and exact predicates",
                   {workingFace});
        return result;
    }
    if (!std::isfinite(configuration.maximumChordDeviation) ||
        !(configuration.maximumChordDeviation > 0.0) ||
        !std::isfinite(configuration.maximumNormalDeviationRadians) ||
        !(configuration.maximumNormalDeviationRadians > 0.0) ||
        !(configuration.maximumNormalDeviationRadians <
          1.57079632679489661923)) {
        setFailure(result, Prerequisites, "cylinder.configuration_invalid",
                   "cylinder error limits must be finite, positive, and below ninety degrees",
                   {workingFace});
        return result;
    }
    const ExactGeometryClassification* classification =
        reconnaissance.find(workingFace);
    if (!classification ||
        classification->taxonomy != GeometryTaxonomy::Surface ||
        classification->familyCode != "cylinder" ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate ||
        classification->trimDomain !=
            TrimDomainClass::FullPeriodicWithCapBoundaries) {
        setFailure(result, Prerequisites, "cylinder.face_not_full_periodic",
                   "only a proven supported full periodic cylinder may use this template",
                   {workingFace});
        return result;
    }
    if (classification->parameterDomains.size() < 2 ||
        !classification->parameterDomains[0].periodic ||
        !classification->parameterDomains[0].period ||
        !std::isfinite(*classification->parameterDomains[0].period) ||
        !(*classification->parameterDomains[0].period > 0.0)) {
        setFailure(result, Prerequisites, "cylinder.period_missing",
                   "the cylindrical U period is not proven", {workingFace});
        return result;
    }
    const double period = *classification->parameterDomains[0].period;
    const std::optional<StableId> sourceFace =
        uniqueSourceForWorking(imported, workingFace);
    if (!sourceFace || sourceFace->kind != StableIdKind::Face) {
        setFailure(result, Prerequisites, "cylinder.source_face_missing",
                   "the cylinder does not resolve to exactly one source face",
                   {workingFace});
        return result;
    }

    std::vector<FaceSampleUse> allFaceUses;
    std::vector<Ring> rings;
    for (const CanonicalBoundary& boundary : boundaries.boundaries) {
        const ExactGeometryClassification* edgeClass =
            reconnaissance.find(boundary.edge);
        const bool ringCandidate = boundary.closed && edgeClass &&
            edgeClass->taxonomy == GeometryTaxonomy::Curve &&
            edgeClass->familyCode == "circle";
        Ring ring;
        ring.boundary = &boundary;
        double vSum = 0.0;
        for (const CanonicalBoundarySample& sample : boundary.samples) {
            std::vector<const CoedgeUvUse*> matches;
            for (const CoedgeUvUse& use : sample.faceUses) {
                if (use.face != workingFace) continue;
                allFaceUses.push_back({&sample, &use});
                if (ringCandidate) matches.push_back(&use);
            }
            if (!ringCandidate) continue;
            if (matches.size() != 1) {
                setFailure(result, BoundaryCoverage,
                           matches.empty()
                               ? "cylinder.rim_uv_missing"
                               : "cylinder.rim_uv_ambiguous",
                           "each canonical rim sample requires exactly one cylinder UV use",
                           {workingFace, boundary.edge});
                return result;
            }
            ring.samples.push_back({&sample, matches.front()});
            vSum += matches.front()->liftedUv[1];
        }
        if (ringCandidate && !ring.samples.empty()) {
            ring.meanV = vSum / static_cast<double>(ring.samples.size());
            rings.push_back(std::move(ring));
        }
    }
    result.validation[BoundaryCoverage].expected = allFaceUses.size();
    if (rings.size() != 2 || allFaceUses.empty()) {
        setFailure(result, BoundaryCoverage, "cylinder.rim_topology_invalid",
                   "a full cylinder wall requires exactly two canonical circular rims",
                   {workingFace});
        return result;
    }
    std::sort(rings.begin(), rings.end(), [](const Ring& first,
                                             const Ring& second) {
        if (first.meanV != second.meanV) return first.meanV < second.meanV;
        return first.boundary->edge < second.boundary->edge;
    });
    if (!(rings[0].meanV < rings[1].meanV)) {
        setFailure(result, BoundaryCoverage, "cylinder.rim_levels_invalid",
                   "cylinder rims do not have distinct axial parameter levels",
                   {workingFace, rings[0].boundary->edge,
                    rings[1].boundary->edge});
        return result;
    }
    if (rings[0].samples.size() != rings[1].samples.size() ||
        rings[0].samples.size() < 3) {
        setFailure(result, BoundaryCoverage, "cylinder.rim_count_mismatch",
                   "cylinder rims require the same non-trivial canonical sample count",
                   {workingFace, rings[0].boundary->edge,
                    rings[1].boundary->edge});
        return result;
    }

    const std::size_t count = rings[0].samples.size();
    std::vector<std::array<double, 3>> lowerPositions;
    std::vector<std::array<double, 3>> upperPositions;
    lowerPositions.reserve(count);
    upperPositions.reserve(count);
    for (const FaceSampleUse& item : rings[0].samples) {
        lowerPositions.push_back(item.sample->position);
    }
    for (const FaceSampleUse& item : rings[1].samples) {
        upperPositions.push_back(item.sample->position);
    }
    result.validation[Registration].expected = count;
    const AzimuthRegistrationResult registered =
        azimuthRegistration(lowerPositions, upperPositions);
    if (!registered || registered.permutation->size() != count) {
        setFailure(result, Registration,
                   registered.failure ? registered.failure->code
                                      : "cylinder.registration_invalid",
                   registered.failure
                       ? registered.failure->message
                       : "azimuth registration returned an invalid permutation",
                   {workingFace, rings[0].boundary->edge,
                    rings[1].boundary->edge});
        return result;
    }
    result.validation[Registration].checked = count;

    PlanarCdtMesh mesh;
    mesh.workingFace = workingFace;
    mesh.sourceFace = sourceFace;
    mesh.vertices.reserve(count * 2);
    std::set<const CoedgeUvUse*> consumed;
    const auto appendRing = [&](const Ring& ring) -> bool {
        std::vector<std::uint32_t> loop;
        loop.reserve(count);
        for (const FaceSampleUse& primary : ring.samples) {
            PlanarTrimVertex vertex;
            vertex.canonicalVertexIndex =
                primary.sample->canonicalVertexIndex;
            vertex.uv = primary.use->liftedUv;
            for (const FaceSampleUse& candidate : allFaceUses) {
                if (candidate.sample->canonicalVertexIndex !=
                    vertex.canonicalVertexIndex) {
                    continue;
                }
                if (candidate.sample->position != primary.sample->position) {
                    return false;
                }
                vertex.boundaryUses.push_back(boundaryUse(candidate));
                consumed.insert(candidate.use);
            }
            loop.push_back(static_cast<std::uint32_t>(mesh.vertices.size()));
            mesh.vertices.push_back(std::move(vertex));
        }
        mesh.boundaryLoops.push_back(std::move(loop));
        return true;
    };
    if (!appendRing(rings[0]) || !appendRing(rings[1])) {
        setFailure(result, BoundaryCoverage,
                   "cylinder.canonical_position_mismatch",
                   "one canonical vertex identity resolves to different exact positions",
                   {workingFace});
        return result;
    }
    result.validation[BoundaryCoverage].checked = consumed.size();
    if (consumed.size() != allFaceUses.size()) {
        setFailure(result, BoundaryCoverage,
                   "cylinder.axial_samples_require_interior_provenance",
                   "the cylinder has seam samples not represented by the one-band boundary-only template",
                   {workingFace});
        return result;
    }

    mesh.constrainedEdges.reserve(count * 2);
    for (std::size_t index = 0; index < count; ++index) {
        const std::uint32_t next =
            static_cast<std::uint32_t>((index + 1) % count);
        mesh.constrainedEdges.push_back(
            {static_cast<std::uint32_t>(index), next});
        mesh.constrainedEdges.push_back(
            {static_cast<std::uint32_t>(count + index),
             static_cast<std::uint32_t>(count) + next});
    }

    result.validation[TriangleOrientation].expected = count * 2;
    result.validation[ChordBound].expected = count;
    result.validation[NormalBound].expected = count * 4;
    const double chordSquared = configuration.maximumChordDeviation *
        configuration.maximumChordDeviation;
    const double minimumNormalDot =
        std::cos(configuration.maximumNormalDeviationRadians);
    mesh.triangles.reserve(count * 2);
    for (std::size_t index = 0; index < count; ++index) {
        const std::size_t next = (index + 1) % count;
        const std::uint32_t upperCurrent =
            (*registered.permutation)[index];
        const std::uint32_t upperNext = (*registered.permutation)[next];
        const std::uint32_t lower0 = static_cast<std::uint32_t>(index);
        const std::uint32_t lower1 = static_cast<std::uint32_t>(next);
        const std::uint32_t upper0 =
            static_cast<std::uint32_t>(count) + upperCurrent;
        const std::uint32_t upper1 =
            static_cast<std::uint32_t>(count) + upperNext;

        const PredicatePoint2 lowerUv0 = mesh.vertices[lower0].uv;
        PredicatePoint2 lowerUv1 = mesh.vertices[lower1].uv;
        PredicatePoint2 upperUv0 = mesh.vertices[upper0].uv;
        PredicatePoint2 upperUv1 = mesh.vertices[upper1].uv;
        lowerUv1[0] = periodicNear(lowerUv1[0], lowerUv0[0], period);
        upperUv0[0] = periodicNear(upperUv0[0], lowerUv0[0], period);
        upperUv1[0] = periodicNear(upperUv1[0], lowerUv1[0], period);

        PlanarCdtTriangle first{
            workingFace, sourceFace, {lower0, lower1, upper1},
            std::array<PredicatePoint2, 3>{lowerUv0, lowerUv1, upperUv1}};
        PlanarCdtTriangle second{
            workingFace, sourceFace, {lower0, upper1, upper0},
            std::array<PredicatePoint2, 3>{lowerUv0, upperUv1, upperUv0}};
        if (!orientTriangle(first, *predicates, result, workingFace) ||
            !orientTriangle(second, *predicates, result, workingFace)) {
            return result;
        }

        const std::array<double, 3>& a = lowerPositions[index];
        const std::array<double, 3>& b = lowerPositions[next];
        const std::array<double, 3>& c = upperPositions[upperNext];
        const std::array<double, 3>& d = upperPositions[upperCurrent];
        const PredicatePoint2 midpointUv{
            (lowerUv0[0] + lowerUv1[0]) * 0.5,
            (lowerUv0[1] + lowerUv1[1] + upperUv0[1] +
             upperUv1[1]) * 0.25};
        const EvaluationResult<SurfaceEvaluation> midpoint =
            imported.workingEvaluator->evaluateSurface(workingFace,
                                                        midpointUv);
        const std::array<double, 3> chordMidpoint{
            (a[0] + b[0] + c[0] + d[0]) * 0.25,
            (a[1] + b[1] + c[1] + d[1]) * 0.25,
            (a[2] + b[2] + c[2] + d[2]) * 0.25};
        ++result.validation[ChordBound].checked;
        if (!midpoint ||
            squaredDistance(midpoint.value->position, chordMidpoint) >
                chordSquared) {
            setFailure(result, ChordBound, "cylinder.chord_bound_exceeded",
                       "a cylinder strip exceeds the requested analytic midpoint chord bound",
                       {workingFace});
            return result;
        }

        const std::optional<std::array<double, 3>> facetNormal =
            unit(triangleNormal(a, b, c));
        const std::array<PredicatePoint2, 4> cornerUvs{
            lowerUv0, lowerUv1, upperUv1, upperUv0};
        if (!facetNormal) {
            setFailure(result, NormalBound, "cylinder.triangle_degenerate",
                       "a cylindrical strip triangle has zero 3D area",
                       {workingFace});
            return result;
        }
        for (const PredicatePoint2& uv : cornerUvs) {
            const EvaluationResult<SurfaceEvaluation> surface =
                imported.workingEvaluator->evaluateSurface(workingFace, uv);
            ++result.validation[NormalBound].checked;
            if (!surface || !surface.value->unitNormal ||
                std::abs(dot(*facetNormal, *surface.value->unitNormal)) <
                    minimumNormalDot) {
                setFailure(result, NormalBound,
                           "cylinder.normal_bound_exceeded",
                           "a cylinder facet exceeds the requested normal-deviation bound",
                           {workingFace});
                return result;
            }
        }
        mesh.triangles.push_back(std::move(first));
        mesh.triangles.push_back(std::move(second));
    }

    if (!std::all_of(result.validation.begin(), result.validation.end(),
                     [](const CylinderWallValidationEvidence& evidence) {
                         return evidence.complete();
                     })) {
        setFailure(result, Prerequisites, "cylinder.validation_incomplete",
                   "cylinder wall validation coverage is incomplete",
                   {workingFace});
        return result;
    }
    result.value = std::move(mesh);
    return result;
}

}  // namespace weft
