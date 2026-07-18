#include "weft/cylinder_template.hpp"

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
    const bool fullPeriodic = classification &&
        classification->trimDomain ==
            TrimDomainClass::FullPeriodicWithCapBoundaries;
    const bool partialBand = classification &&
        classification->trimDomain ==
            TrimDomainClass::PeriodicBandCrossingSeam;
    if (!classification ||
        classification->taxonomy != GeometryTaxonomy::Surface ||
        classification->familyCode != "cylinder" ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate ||
        (!fullPeriodic && !partialBand)) {
        setFailure(result, Prerequisites, "cylinder.face_unsupported",
                   "only a proven supported full or partial cylinder band may use this template",
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
        const bool ringCandidate = edgeClass &&
            edgeClass->taxonomy == GeometryTaxonomy::Curve &&
            edgeClass->familyCode == "circle" &&
            ((fullPeriodic && boundary.closed) ||
             (partialBand && !boundary.closed));
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
                   fullPeriodic
                       ? "a full cylinder wall requires exactly two canonical circular rims"
                       : "a partial cylinder wall requires exactly two open circular rim arcs",
                   {workingFace});
        return result;
    }
    if (partialBand) {
        std::size_t railCount = 0;
        for (const CanonicalBoundary& boundary : boundaries.boundaries) {
            const ExactGeometryClassification* edgeClass =
                reconnaissance.find(boundary.edge);
            bool onFace = false;
            for (const CanonicalBoundarySample& sample : boundary.samples) {
                for (const CoedgeUvUse& use : sample.faceUses) {
                    if (use.face == workingFace) {
                        onFace = true;
                        break;
                    }
                }
                if (onFace) break;
            }
            if (onFace && edgeClass &&
                edgeClass->familyCode == "line") {
                ++railCount;
            }
        }
        if (railCount != 2) {
            setFailure(result, BoundaryCoverage, "cylinder.rail_topology_invalid",
                       "a partial cylinder wall requires exactly two linear side rails",
                       {workingFace});
            return result;
        }
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

    if (configuration.axialIntervals == 0) {
        setFailure(result, Prerequisites, "cylinder.axial_count_invalid",
                   "cylinder axial intervals must be positive", {workingFace});
        return result;
    }

    PlanarCdtMesh mesh;
    mesh.workingFace = workingFace;
    mesh.sourceFace = sourceFace;
    const std::uint32_t axialIntervals = configuration.axialIntervals;
    const std::size_t ringCount =
        static_cast<std::size_t>(axialIntervals) + 1U;
    mesh.vertices.reserve(count * ringCount);
    std::set<const CoedgeUvUse*> consumed;
    const auto appendBoundaryRing = [&](const Ring& ring) -> bool {
        std::vector<std::uint32_t> loop;
        loop.reserve(count);
        for (std::size_t column = 0; column < count; ++column) {
            const FaceSampleUse& primary = ring.samples[column];
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
    if (!appendBoundaryRing(rings[0])) {
        setFailure(result, BoundaryCoverage,
                   "cylinder.canonical_position_mismatch",
                   "one canonical vertex identity resolves to different exact positions",
                   {workingFace});
        return result;
    }

    std::vector<std::uint32_t> upperOrder(count);
    for (std::size_t index = 0; index < count; ++index) {
        upperOrder[index] = (*registered.permutation)[index];
    }

    std::vector<FaceSampleUse> leftoverUses;
    for (const FaceSampleUse& candidate : allFaceUses) {
        if (!consumed.contains(candidate.use)) leftoverUses.push_back(candidate);
    }
    for (std::uint32_t ringIndex = 1; ringIndex < axialIntervals;
         ++ringIndex) {
        const double t = static_cast<double>(ringIndex) /
            static_cast<double>(axialIntervals);
        for (std::size_t column = 0; column < count; ++column) {
            const FaceSampleUse& lower = rings[0].samples[column];
            const FaceSampleUse& upper =
                rings[1].samples[upperOrder[column]];
            PredicatePoint2 uv = lower.use->liftedUv;
            const double upperU = periodicNear(upper.use->liftedUv[0], uv[0],
                                               period);
            uv[0] = uv[0] + (upperU - uv[0]) * t;
            uv[1] = lower.use->liftedUv[1] +
                (upper.use->liftedUv[1] - lower.use->liftedUv[1]) * t;

            // Prefer an unmatched seam/axial sample at this column when its UV
            // lands on this ring. Exact geometric samples own the station.
            const FaceSampleUse* matched = nullptr;
            double bestScore = std::numeric_limits<double>::infinity();
            for (const FaceSampleUse& candidate : leftoverUses) {
                if (consumed.contains(candidate.use)) continue;
                const double candidateU = periodicNear(
                    candidate.use->liftedUv[0], uv[0], period);
                const double du = candidateU - uv[0];
                const double dv = candidate.use->liftedUv[1] - uv[1];
                const double score = du * du + dv * dv;
                if (score < bestScore) {
                    bestScore = score;
                    matched = &candidate;
                }
            }
            // Accept a leftover sample only when it is closer to this target
            // than to neighboring columns/rings. With equal axial spacing the
            // nearest-ring/column sample is an exact boundary witness.
            const double ringSpacing =
                std::abs(rings[1].meanV - rings[0].meanV) /
                static_cast<double>(axialIntervals);
            const double columnSpacing = period / static_cast<double>(count);
            const bool acceptMatched = matched != nullptr &&
                bestScore <= (0.25 * ringSpacing * ringSpacing +
                              0.25 * columnSpacing * columnSpacing);

            PlanarTrimVertex vertex;
            if (acceptMatched) {
                vertex.canonicalVertexIndex =
                    matched->sample->canonicalVertexIndex;
                vertex.uv = matched->use->liftedUv;
                vertex.uv[0] = periodicNear(vertex.uv[0], uv[0], period);
                for (const FaceSampleUse& candidate : allFaceUses) {
                    if (candidate.sample->canonicalVertexIndex !=
                        vertex.canonicalVertexIndex) {
                        continue;
                    }
                    if (candidate.sample->position !=
                        matched->sample->position) {
                        setFailure(result, BoundaryCoverage,
                                   "cylinder.canonical_position_mismatch",
                                   "one canonical vertex identity resolves to different exact positions",
                                   {workingFace});
                        return result;
                    }
                    vertex.boundaryUses.push_back(boundaryUse(candidate));
                    consumed.insert(candidate.use);
                }
                vertex.cylinderInterior = CylinderInteriorStation{
                    workingFace, sourceFace, vertex.uv,
                    matched->sample->position, ringIndex,
                    static_cast<std::uint32_t>(column)};
            } else {
                const EvaluationResult<SurfaceEvaluation> evaluated =
                    imported.workingEvaluator->evaluateSurface(workingFace,
                                                               uv);
                if (!evaluated) {
                    setFailure(result, BoundaryCoverage,
                               "cylinder.interior_station_evaluation_failed",
                               "a cylinder interior station could not be evaluated",
                               {workingFace});
                    return result;
                }
                vertex.uv = uv;
                vertex.cylinderInterior = CylinderInteriorStation{
                    workingFace, sourceFace, uv, evaluated.value->position,
                    ringIndex, static_cast<std::uint32_t>(column)};
            }
            mesh.vertices.push_back(std::move(vertex));
        }
    }

    if (!appendBoundaryRing(rings[1])) {
        setFailure(result, BoundaryCoverage,
                   "cylinder.canonical_position_mismatch",
                   "one canonical vertex identity resolves to different exact positions",
                   {workingFace});
        return result;
    }

    // Reorder the final rim vertices so column i matches the registered
    // azimuth pairing used by the interior rings and lower rim.
    if (axialIntervals >= 1) {
        const std::size_t upperBegin = count * axialIntervals;
        std::vector<PlanarTrimVertex> reordered;
        reordered.reserve(count);
        for (std::size_t column = 0; column < count; ++column) {
            reordered.push_back(
                std::move(mesh.vertices[upperBegin + upperOrder[column]]));
        }
        for (std::size_t column = 0; column < count; ++column) {
            mesh.vertices[upperBegin + column] = std::move(reordered[column]);
        }
        if (!mesh.boundaryLoops.empty()) {
            std::vector<std::uint32_t>& upperLoop = mesh.boundaryLoops.back();
            for (std::size_t column = 0; column < count; ++column) {
                upperLoop[column] =
                    static_cast<std::uint32_t>(upperBegin + column);
            }
        }
    }

    // Partial one-band walls: side-rail samples often share canonical vertex
    // identities with rim corners. Attach leftovers only onto an existing rim
    // vertex with the same canonical index (exact shared endpoint).
    if (partialBand && axialIntervals == 1 &&
        consumed.size() != allFaceUses.size()) {
        for (const FaceSampleUse& leftover : allFaceUses) {
            if (consumed.contains(leftover.use)) continue;
            bool attached = false;
            for (PlanarTrimVertex& vertex : mesh.vertices) {
                if (vertex.canonicalVertexIndex ==
                    leftover.sample->canonicalVertexIndex) {
                    vertex.boundaryUses.push_back(boundaryUse(leftover));
                    consumed.insert(leftover.use);
                    attached = true;
                    break;
                }
            }
            if (!attached) {
                // Leave unconsumed; the failure below names the gap.
            }
        }
    }

    result.validation[BoundaryCoverage].checked = consumed.size();
    if (consumed.size() != allFaceUses.size()) {
        setFailure(result, BoundaryCoverage,
                   "cylinder.axial_samples_require_interior_provenance",
                   axialIntervals == 1
                       ? "the cylinder has seam samples not represented by the one-band boundary-only template"
                       : "cylinder axial samples remain after certified interior rings were generated",
                   {workingFace});
        return result;
    }

    const std::size_t columnSpans =
        fullPeriodic ? count : (count > 0 ? count - 1 : 0);
    if (!fullPeriodic && columnSpans < 2) {
        setFailure(result, BoundaryCoverage, "cylinder.rim_count_mismatch",
                   "a partial cylinder wall needs at least three rim samples",
                   {workingFace});
        return result;
    }
    mesh.constrainedEdges.reserve(columnSpans * ringCount);
    for (std::size_t ringIndex = 0; ringIndex < ringCount; ++ringIndex) {
        for (std::size_t index = 0; index < columnSpans; ++index) {
            const std::uint32_t base =
                static_cast<std::uint32_t>(ringIndex * count);
            const std::uint32_t next = fullPeriodic
                ? static_cast<std::uint32_t>((index + 1) % count)
                : static_cast<std::uint32_t>(index + 1);
            mesh.constrainedEdges.push_back(
                {base + static_cast<std::uint32_t>(index), base + next});
        }
    }

    const std::size_t bandCount = ringCount - 1;
    result.validation[TriangleOrientation].expected =
        columnSpans * 2 * bandCount;
    result.validation[ChordBound].expected = columnSpans * bandCount;
    result.validation[NormalBound].expected = columnSpans * 4 * bandCount;
    const double chordSquared = configuration.maximumChordDeviation *
        configuration.maximumChordDeviation;
    const double minimumNormalDot =
        std::cos(configuration.maximumNormalDeviationRadians);
    mesh.triangles.reserve(columnSpans * 2 * bandCount);
    for (std::size_t band = 0; band < bandCount; ++band) {
        for (std::size_t index = 0; index < columnSpans; ++index) {
            const std::size_t next =
                fullPeriodic ? ((index + 1) % count) : (index + 1);
            const std::uint32_t lower0 =
                static_cast<std::uint32_t>(band * count + index);
            const std::uint32_t lower1 =
                static_cast<std::uint32_t>(band * count + next);
            const std::uint32_t upper0 =
                static_cast<std::uint32_t>((band + 1) * count + index);
            const std::uint32_t upper1 =
                static_cast<std::uint32_t>((band + 1) * count + next);

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

            const auto positionOf = [&](std::uint32_t vertexIndex) {
                if (mesh.vertices[vertexIndex].cylinderInterior) {
                    return mesh.vertices[vertexIndex]
                        .cylinderInterior->position;
                }
                // Boundary rim vertices resolve through their first attached
                // sample use; fall back to surface evaluation for safety.
                const EvaluationResult<SurfaceEvaluation> evaluated =
                    imported.workingEvaluator->evaluateSurface(
                        workingFace, mesh.vertices[vertexIndex].uv);
                return evaluated ? evaluated.value->position
                                 : std::array<double, 3>{};
            };
            const std::array<double, 3> a = positionOf(lower0);
            const std::array<double, 3> b = positionOf(lower1);
            const std::array<double, 3> c = positionOf(upper1);
            const std::array<double, 3> d = positionOf(upper0);
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
                    imported.workingEvaluator->evaluateSurface(workingFace,
                                                               uv);
                ++result.validation[NormalBound].checked;
                if (!surface || !surface.value->unitNormal ||
                    std::abs(dot(*facetNormal,
                                 *surface.value->unitNormal)) <
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
