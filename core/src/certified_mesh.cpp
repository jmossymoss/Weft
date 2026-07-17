#include "weft/certified_mesh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

namespace weft {
namespace {

enum CheckIndex : std::size_t {
    FaceCoverage = 0,
    BoundaryProvenance,
    CanonicalIdentity,
    TriangleProvenance,
    TriangleGeometry,
    EdgeIncidence,
    EdgeWinding,
    TriangleIntersection,
    EulerCharacteristic,
    Fingerprint,
};

std::array<double, 3> triangleNormal(const std::array<double, 3>& a,
                                     const std::array<double, 3>& b,
                                     const std::array<double, 3>& c);

// Exact triangle/triangle disjointness over finite doubles. Any contact —
// crossing, touching, or coplanar overlap — reports non-disjoint; a
// predicate failure reports nothing so callers can fail closed.
std::size_t dominantNormalAxis(const std::array<PredicatePoint3, 3>& triangle) {
    const std::array<double, 3> normal =
        triangleNormal(triangle[0], triangle[1], triangle[2]);
    std::size_t dropAxis = 0;
    for (std::size_t axis = 1; axis < 3; ++axis) {
        if (std::abs(normal[axis]) > std::abs(normal[dropAxis])) {
            dropAxis = axis;
        }
    }
    return dropAxis;
}

PredicatePoint2 flattenPoint(PredicatePoint3 point, std::size_t dropAxis) {
    PredicatePoint2 flat{};
    std::size_t out = 0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (axis != dropAxis) flat[out++] = point[axis];
    }
    return flat;
}

// Inclusive 2D point-in-triangle: boundary contact counts as inside.
std::optional<bool> pointMeetsTriangle2d(
    const GeometricPredicates& predicates,
    const std::array<PredicatePoint2, 3>& triangle, PredicatePoint2 point) {
    const auto winding =
        predicates.orient2d(triangle[0], triangle[1], triangle[2]);
    if (!winding) return std::nullopt;
    if (*winding.value == ExactSign::Zero) return std::nullopt;
    for (std::size_t edge = 0; edge < 3; ++edge) {
        const auto side = predicates.orient2d(
            triangle[edge], triangle[(edge + 1) % 3], point);
        if (!side) return std::nullopt;
        if (*side.value != ExactSign::Zero &&
            *side.value != *winding.value) {
            return false;
        }
    }
    return true;
}

std::optional<bool> segmentMeetsTriangle(
    const GeometricPredicates& predicates, PredicatePoint3 p,
    PredicatePoint3 q, const std::array<PredicatePoint3, 3>& triangle) {
    const auto pSide =
        predicates.orient3d(triangle[0], triangle[1], triangle[2], p);
    const auto qSide =
        predicates.orient3d(triangle[0], triangle[1], triangle[2], q);
    if (!pSide || !qSide) return std::nullopt;
    const bool pOnPlane = *pSide.value == ExactSign::Zero;
    const bool qOnPlane = *qSide.value == ExactSign::Zero;
    if (pOnPlane || qOnPlane) {
        // The segment meets the supporting plane at the on-plane endpoints
        // (or along itself when coplanar); contact exists only where those
        // points actually lie on the triangle.
        const std::size_t dropAxis = dominantNormalAxis(triangle);
        const std::array<PredicatePoint2, 3> flat{
            flattenPoint(triangle[0], dropAxis),
            flattenPoint(triangle[1], dropAxis),
            flattenPoint(triangle[2], dropAxis)};
        if (pOnPlane && qOnPlane) {
            const PredicatePoint2 p2 = flattenPoint(p, dropAxis);
            const PredicatePoint2 q2 = flattenPoint(q, dropAxis);
            for (std::size_t edge = 0; edge < 3; ++edge) {
                const auto crossing = predicates.segmentIntersection(
                    p2, q2, flat[edge], flat[(edge + 1) % 3]);
                if (!crossing) return std::nullopt;
                if (*crossing.value != SegmentIntersectionKind::None) {
                    return true;
                }
            }
            const auto pInside = pointMeetsTriangle2d(predicates, flat, p2);
            const auto qInside = pointMeetsTriangle2d(predicates, flat, q2);
            if (!pInside || !qInside) return std::nullopt;
            return *pInside || *qInside;
        }
        const PredicatePoint3 onPlane = pOnPlane ? p : q;
        return pointMeetsTriangle2d(predicates, flat,
                                    flattenPoint(onPlane, dropAxis));
    }
    if (*pSide.value == *qSide.value) return false;
    // The segment pierces the supporting plane at one interior point; the
    // signs around the triangle edges decide where that point lies. Mixed
    // strict signs put it outside; otherwise it is inside or exactly on the
    // boundary, both of which are contact.
    bool sawPositive = false;
    bool sawNegative = false;
    for (std::size_t edge = 0; edge < 3; ++edge) {
        const auto side = predicates.orient3d(
            p, q, triangle[edge], triangle[(edge + 1) % 3]);
        if (!side) return std::nullopt;
        if (*side.value == ExactSign::Positive) sawPositive = true;
        if (*side.value == ExactSign::Negative) sawNegative = true;
    }
    return !(sawPositive && sawNegative);
}

std::optional<bool> coplanarTrianglesDisjoint(
    const GeometricPredicates& predicates,
    const std::array<PredicatePoint3, 3>& one,
    const std::array<PredicatePoint3, 3>& two) {
    const std::array<double, 3> normal = triangleNormal(one[0], one[1], one[2]);
    std::size_t dropAxis = 0;
    for (std::size_t axis = 1; axis < 3; ++axis) {
        if (std::abs(normal[axis]) > std::abs(normal[dropAxis])) {
            dropAxis = axis;
        }
    }
    const auto flatten = [dropAxis](PredicatePoint3 point) {
        PredicatePoint2 flat{};
        std::size_t out = 0;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            if (axis != dropAxis) flat[out++] = point[axis];
        }
        return flat;
    };
    std::array<PredicatePoint2, 3> first;
    std::array<PredicatePoint2, 3> second;
    for (std::size_t corner = 0; corner < 3; ++corner) {
        first[corner] = flatten(one[corner]);
        second[corner] = flatten(two[corner]);
    }
    for (std::size_t a = 0; a < 3; ++a) {
        for (std::size_t b = 0; b < 3; ++b) {
            const auto crossing = predicates.segmentIntersection(
                first[a], first[(a + 1) % 3], second[b],
                second[(b + 1) % 3]);
            if (!crossing) return std::nullopt;
            if (*crossing.value != SegmentIntersectionKind::None) {
                return false;
            }
        }
    }
    const auto containsStrictly =
        [&](const std::array<PredicatePoint2, 3>& outer,
            PredicatePoint2 candidate) -> std::optional<bool> {
        const auto winding =
            predicates.orient2d(outer[0], outer[1], outer[2]);
        if (!winding) return std::nullopt;
        if (*winding.value == ExactSign::Zero) return false;
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const auto side = predicates.orient2d(
                outer[edge], outer[(edge + 1) % 3], candidate);
            if (!side) return std::nullopt;
            if (*side.value != *winding.value) return false;
        }
        return true;
    };
    for (std::size_t corner = 0; corner < 3; ++corner) {
        const auto inside = containsStrictly(first, second[corner]);
        const auto reverse = containsStrictly(second, first[corner]);
        if (!inside || !reverse) return std::nullopt;
        if (*inside || *reverse) return false;
    }
    return true;
}

std::optional<bool> trianglesDisjoint(
    const GeometricPredicates& predicates,
    const std::array<PredicatePoint3, 3>& one,
    const std::array<PredicatePoint3, 3>& two) {
    std::array<ExactSign, 3> twoAgainstOne{};
    std::array<ExactSign, 3> oneAgainstTwo{};
    for (std::size_t corner = 0; corner < 3; ++corner) {
        const auto sideTwo =
            predicates.orient3d(one[0], one[1], one[2], two[corner]);
        const auto sideOne =
            predicates.orient3d(two[0], two[1], two[2], one[corner]);
        if (!sideTwo || !sideOne) return std::nullopt;
        twoAgainstOne[corner] = *sideTwo.value;
        oneAgainstTwo[corner] = *sideOne.value;
    }
    const auto strictlyOneSide = [](const std::array<ExactSign, 3>& signs) {
        return (signs[0] == ExactSign::Positive &&
                signs[1] == ExactSign::Positive &&
                signs[2] == ExactSign::Positive) ||
            (signs[0] == ExactSign::Negative &&
             signs[1] == ExactSign::Negative &&
             signs[2] == ExactSign::Negative);
    };
    if (strictlyOneSide(twoAgainstOne) || strictlyOneSide(oneAgainstTwo)) {
        return true;
    }
    const auto allZero = [](const std::array<ExactSign, 3>& signs) {
        return signs[0] == ExactSign::Zero && signs[1] == ExactSign::Zero &&
            signs[2] == ExactSign::Zero;
    };
    if (allZero(twoAgainstOne) && allZero(oneAgainstTwo)) {
        return coplanarTrianglesDisjoint(predicates, one, two);
    }
    for (std::size_t edge = 0; edge < 3; ++edge) {
        const auto oneEdge = segmentMeetsTriangle(
            predicates, one[edge], one[(edge + 1) % 3], two);
        const auto twoEdge = segmentMeetsTriangle(
            predicates, two[edge], two[(edge + 1) % 3], one);
        if (!oneEdge || !twoEdge) return std::nullopt;
        if (*oneEdge || *twoEdge) return false;
    }
    return true;
}

struct Edge {
    std::uint32_t lower = 0;
    std::uint32_t upper = 0;

    auto operator<=>(const Edge&) const = default;
};

struct DirectedEdgeUse {
    bool lowerToUpper = false;
};

Edge edge(std::uint32_t first, std::uint32_t second) {
    return first < second ? Edge{first, second} : Edge{second, first};
}

void setFailure(CertifiedMeshAssemblyResult& result, std::string code,
                std::string message, std::vector<StableId> subjects) {
    if (!result.failure) {
        result.failure = CertifiedMeshAssemblyFailure{
            std::move(code), std::move(message), std::move(subjects)};
    }
}

std::optional<TopologyOrientation> faceOrientation(
    const BRepSnapshot& snapshot, StableId face) {
    const auto found = std::find_if(
        snapshot.occurrences.begin(), snapshot.occurrences.end(),
        [face](const TopologyOccurrence& occurrence) {
            return occurrence.id == face;
        });
    return found == snapshot.occurrences.end()
        ? std::nullopt
        : std::optional<TopologyOrientation>(found->orientation);
}

bool exactPositionEqual(const std::array<double, 3>& first,
                        const std::array<double, 3>& second) {
    return first == second;
}

const CanonicalBoundarySample* resolveBoundarySample(
    const CanonicalBoundarySet& boundaries,
    const PlanarTrimBoundaryUse& use) {
    const CanonicalBoundary* boundary = boundaries.find(use.workingEdge);
    if (!boundary || boundary->boundaryId != use.sample.boundary ||
        use.sample.ordinal >= boundary->samples.size()) {
        return nullptr;
    }
    const CanonicalBoundarySample& sample =
        boundary->samples[use.sample.ordinal];
    return sample.id == use.sample && sample.workingEdge == use.workingEdge &&
        sample.sourceEdge == use.sourceEdge
        ? &sample
        : nullptr;
}

bool resolvesFaceUse(const CanonicalBoundarySample& sample,
                     const CertifiedVertexUse& use) {
    return std::any_of(
        sample.faceUses.begin(), sample.faceUses.end(),
        [&](const CoedgeUvUse& faceUse) {
            return faceUse.face == use.workingFace &&
                faceUse.sourceFace == use.sourceFace &&
                faceUse.coedge == use.boundary.coedge &&
                faceUse.representation == use.boundary.representation &&
                faceUse.liftedUv == use.boundary.uv &&
                faceUse.measuredCurveOnSurfaceDiscrepancy ==
                    use.boundary.measuredCurveOnSurfaceDiscrepancy &&
                faceUse.allowedCurveOnSurfaceDiscrepancy ==
                    use.boundary.allowedCurveOnSurfaceDiscrepancy;
        });
}

bool sameProvenance(const CertifiedVertexUse& first,
                    const CertifiedVertexUse& second) {
    return first.workingFace == second.workingFace &&
        first.sourceFace == second.sourceFace &&
        first.boundary.sample == second.boundary.sample &&
        first.boundary.workingEdge == second.boundary.workingEdge &&
        first.boundary.sourceEdge == second.boundary.sourceEdge &&
        first.boundary.coedge == second.boundary.coedge &&
        first.boundary.representation == second.boundary.representation &&
        first.boundary.uv == second.boundary.uv &&
        first.boundary.measuredCurveOnSurfaceDiscrepancy ==
            second.boundary.measuredCurveOnSurfaceDiscrepancy &&
        first.boundary.allowedCurveOnSurfaceDiscrepancy ==
            second.boundary.allowedCurveOnSurfaceDiscrepancy;
}

StableId optionalId(const std::optional<StableId>& value) {
    return value.value_or(StableId{});
}

bool provenanceLess(const CertifiedVertexUse& first,
                    const CertifiedVertexUse& second) {
    return std::tuple(first.workingFace, optionalId(first.sourceFace),
                      first.boundary.workingEdge,
                      optionalId(first.boundary.sourceEdge),
                      first.boundary.coedge,
                      first.boundary.representation,
                      first.boundary.sample.boundary,
                      first.boundary.sample.ordinal, first.boundary.uv,
                      first.boundary.measuredCurveOnSurfaceDiscrepancy,
                      first.boundary.allowedCurveOnSurfaceDiscrepancy) <
        std::tuple(second.workingFace, optionalId(second.sourceFace),
                   second.boundary.workingEdge,
                   optionalId(second.boundary.sourceEdge),
                   second.boundary.coedge,
                   second.boundary.representation,
                   second.boundary.sample.boundary,
                   second.boundary.sample.ordinal, second.boundary.uv,
                   second.boundary.measuredCurveOnSurfaceDiscrepancy,
                   second.boundary.allowedCurveOnSurfaceDiscrepancy);
}

double squaredDistance(std::array<double, 3> first,
                       std::array<double, 3> second) {
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

class Fnv1a64 {
public:
    void add(std::uint64_t value) {
        for (unsigned byte = 0; byte < 8; ++byte) {
            state_ ^= static_cast<std::uint8_t>(value >> (byte * 8U));
            state_ *= 1099511628211ULL;
        }
    }

    std::string finish() const {
        std::ostringstream stream;
        stream << std::hex << std::setfill('0') << std::setw(16) << state_;
        return stream.str();
    }

private:
    std::uint64_t state_ = 14695981039346656037ULL;
};

std::string fingerprintOf(const CertifiedMesh& mesh) {
    Fnv1a64 digest;
    digest.add(mesh.vertices.size());
    for (const CertifiedVertex& vertex : mesh.vertices) {
        digest.add(vertex.canonicalVertexIndex);
        for (double coordinate : vertex.position) {
            digest.add(std::bit_cast<std::uint64_t>(coordinate));
        }
    }
    digest.add(mesh.triangles.size());
    for (const CertifiedTriangle& triangle : mesh.triangles) {
        digest.add(static_cast<std::uint64_t>(triangle.workingFace.kind));
        digest.add(triangle.workingFace.ordinal);
        const StableId source = optionalId(triangle.sourceFace);
        digest.add(static_cast<std::uint64_t>(source.kind));
        digest.add(source.ordinal);
        for (std::uint32_t vertex : triangle.vertices) digest.add(vertex);
    }
    return digest.finish();
}

}  // namespace

bool ValidationCertificate::complete() const noexcept {
    return !checks.empty() &&
        std::all_of(checks.begin(), checks.end(),
                    [](const ValidationCoverage& coverage) {
                        return coverage.complete();
                    });
}

ModelingMesh makeCertifiedFloorModelingMesh(
    const CertifiedMesh& certified,
    std::optional<std::string> safeFloorReason) {
    ModelingMesh modeling;
    modeling.vertices = certified.vertices;
    modeling.aliasesCertified = true;
    modeling.safeFloorReason = std::move(safeFloorReason);
    modeling.polygons.reserve(certified.triangles.size());
    for (const CertifiedTriangle& triangle : certified.triangles) {
        modeling.polygons.push_back(
            {triangle.workingFace, triangle.sourceFace,
             {triangle.vertices.begin(), triangle.vertices.end()}});
    }
    return modeling;
}

MeshingResult makeCertifiedFloorMeshingResult(
    CertifiedMesh certified,
    ValidationCertificate validation,
    GenerationReport generation,
    std::optional<std::string> safeFloorReason) {
    MeshingResult result;
    result.modeling = makeCertifiedFloorModelingMesh(
        certified, std::move(safeFloorReason));
    result.certified = std::move(certified);
    result.generation = std::move(generation);
    result.validation = std::move(validation);
    return result;
}

CertifiedMeshAssemblyResult assembleCertifiedBoundaryMesh(
    const ImportedModel& imported,
    const CanonicalBoundarySet& boundaries,
    std::span<const PlanarCdtMesh> faceMeshes,
    std::span<const StableId> expectedWorkingFaces,
    const CertifiedMeshAssemblyConfiguration& configuration) {
    CertifiedMeshAssemblyResult result;
    result.validation.checks = {
        {"certified.face_coverage", expectedWorkingFaces.size()},
        {"certified.boundary_provenance", 0},
        {"certified.canonical_vertex_identity", 0},
        {"certified.triangle_provenance", 0},
        {"certified.triangle_geometry", 0},
        {"certified.edge_incidence", 0},
        {"certified.edge_winding", 0},
        {"certified.triangle_intersection", 0},
        {"certified.euler_characteristic", 1},
        {"certified.fingerprint", 1},
    };
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !boundaries.validation.complete()) {
        ++result.validation.checks[FaceCoverage].failed;
        setFailure(result, "certified.prerequisite_incomplete",
                   "certified assembly requires a meshable working model and complete boundaries",
                   {});
        return result;
    }
    if (!std::isfinite(configuration.maximumVertexSurfaceDiscrepancy) ||
        configuration.maximumVertexSurfaceDiscrepancy < 0.0) {
        ++result.validation.checks[FaceCoverage].failed;
        setFailure(result, "certified.invalid_configuration",
                   "the vertex-on-surface discrepancy limit is invalid", {});
        return result;
    }

    std::set<StableId> expectedFaces;
    for (StableId face : expectedWorkingFaces) {
        if (face.kind != StableIdKind::Face || !face.valid() ||
            !expectedFaces.insert(face).second) {
            ++result.validation.checks[FaceCoverage].failed;
            setFailure(result, "certified.expected_face_set_invalid",
                       "expected working faces must be unique valid face IDs",
                       {face});
            return result;
        }
    }
    if (expectedFaces.empty()) {
        ++result.validation.checks[FaceCoverage].failed;
        setFailure(result, "certified.expected_face_set_empty",
                   "a certified body cannot be assembled without expected faces",
                   {});
        return result;
    }
    std::map<StableId, const PlanarCdtMesh*> meshesByFace;
    for (const PlanarCdtMesh& mesh : faceMeshes) {
        if (!meshesByFace.emplace(mesh.workingFace, &mesh).second) {
            ++result.validation.checks[FaceCoverage].failed;
            setFailure(result, "certified.duplicate_face_mesh",
                       "a working face has more than one certified face mesh",
                       {mesh.workingFace});
            return result;
        }
    }
    ValidationCoverage& faceCoverage =
        result.validation.checks[FaceCoverage];
    for (StableId face : expectedFaces) {
        ++faceCoverage.checked;
        if (!meshesByFace.contains(face)) {
            ++faceCoverage.failed;
            setFailure(result, "certified.face_mesh_missing",
                       "an expected working face has no certified mesh", {face});
        }
    }
    for (const auto& [face, mesh] : meshesByFace) {
        (void)mesh;
        if (!expectedFaces.contains(face)) {
            ++faceCoverage.failed;
            setFailure(result, "certified.unexpected_face_mesh",
                       "a certified face mesh is outside the expected face set",
                       {face});
        }
    }
    if (faceCoverage.failed != 0) return result;

    std::map<std::uint64_t, CertifiedVertex> verticesByCanonicalIndex;
    ValidationCoverage& boundaryProvenance =
        result.validation.checks[BoundaryProvenance];
    ValidationCoverage& identity =
        result.validation.checks[CanonicalIdentity];
    for (const auto& [face, meshPointer] : meshesByFace) {
        const PlanarCdtMesh& faceMesh = *meshPointer;
        if (faceMesh.workingFace != face ||
            faceMesh.sourceFace == std::nullopt) {
            ++identity.failed;
            setFailure(result, "certified.face_provenance_invalid",
                       "a planar face mesh lacks source/working face provenance",
                       {face});
            return result;
        }
        identity.expected += faceMesh.vertices.size();
        for (const PlanarTrimVertex& localVertex : faceMesh.vertices) {
            ++identity.checked;
            boundaryProvenance.expected += localVertex.boundaryUses.size();
            if (localVertex.boundaryUses.empty() ||
                localVertex.canonicalVertexIndex ==
                    InvalidCanonicalVertexIndex ||
                localVertex.canonicalVertexIndex >=
                    boundaries.canonicalVertexCount) {
                ++identity.failed;
                setFailure(result, "certified.canonical_vertex_invalid",
                           "a face vertex has no valid canonical identity",
                           {face});
                return result;
            }

            std::optional<std::array<double, 3>> resolvedPosition;
            std::vector<CertifiedVertexUse> resolvedUses;
            for (const PlanarTrimBoundaryUse& boundaryUse :
                 localVertex.boundaryUses) {
                ++boundaryProvenance.checked;
                const CanonicalBoundarySample* sample =
                    resolveBoundarySample(boundaries, boundaryUse);
                CertifiedVertexUse certifiedUse{
                    face, faceMesh.sourceFace, boundaryUse};
                if (!sample ||
                    sample->canonicalVertexIndex !=
                        localVertex.canonicalVertexIndex ||
                    !resolvesFaceUse(*sample, certifiedUse)) {
                    ++boundaryProvenance.failed;
                    setFailure(result, "certified.boundary_provenance_invalid",
                               "a face vertex does not resolve through its canonical sample and coedge UV use",
                               {face, boundaryUse.workingEdge,
                                boundaryUse.coedge});
                    return result;
                }
                if (resolvedPosition &&
                    !exactPositionEqual(*resolvedPosition, sample->position)) {
                    ++identity.failed;
                    setFailure(result, "certified.vertex_position_mismatch",
                               "incident canonical samples disagree exactly in 3D",
                               {face, boundaryUse.workingEdge});
                    return result;
                }
                resolvedPosition = sample->position;
                resolvedUses.push_back(std::move(certifiedUse));
            }

            auto [found, inserted] = verticesByCanonicalIndex.emplace(
                localVertex.canonicalVertexIndex,
                CertifiedVertex{localVertex.canonicalVertexIndex,
                                *resolvedPosition, {}});
            CertifiedVertex& global = found->second;
            if (!inserted &&
                !exactPositionEqual(global.position, *resolvedPosition)) {
                ++identity.failed;
                setFailure(result, "certified.vertex_position_mismatch",
                           "one canonical vertex resolves to different 3D positions",
                           {face});
                return result;
            }
            for (CertifiedVertexUse& use : resolvedUses) {
                const bool duplicate = std::any_of(
                    global.provenance.begin(), global.provenance.end(),
                    [&](const CertifiedVertexUse& existing) {
                        return sameProvenance(existing, use);
                    });
                if (!duplicate) global.provenance.push_back(std::move(use));
            }
        }
    }

    CertifiedMesh mesh;
    if (verticesByCanonicalIndex.size() >
        static_cast<std::size_t>(
            std::numeric_limits<std::uint32_t>::max())) {
        ++identity.failed;
        setFailure(result, "certified.vertex_index_overflow",
                   "the certified mesh exceeds the global index width", {});
        return result;
    }
    mesh.vertices.reserve(verticesByCanonicalIndex.size());
    std::map<std::uint64_t, std::uint32_t> canonicalToGlobal;
    for (auto& [canonicalIndex, vertex] : verticesByCanonicalIndex) {
        std::sort(vertex.provenance.begin(), vertex.provenance.end(),
                  provenanceLess);
        const std::uint32_t globalIndex =
            static_cast<std::uint32_t>(mesh.vertices.size());
        canonicalToGlobal.emplace(canonicalIndex, globalIndex);
        mesh.vertices.push_back(std::move(vertex));
    }

    ValidationCoverage& triangleProvenance =
        result.validation.checks[TriangleProvenance];
    ValidationCoverage& triangleGeometry =
        result.validation.checks[TriangleGeometry];
    for (const auto& [face, meshPointer] : meshesByFace) {
        const PlanarCdtMesh& faceMesh = *meshPointer;
        const std::optional<TopologyOrientation> orientation =
            faceOrientation(imported.working->snapshot, face);
        if (!orientation ||
            (*orientation != TopologyOrientation::Forward &&
             *orientation != TopologyOrientation::Reversed)) {
            ++triangleProvenance.failed;
            setFailure(result, "certified.face_orientation_invalid",
                       "the working face has no orientable topology occurrence",
                       {face});
            return result;
        }
        triangleProvenance.expected += faceMesh.triangles.size();
        triangleGeometry.expected += faceMesh.triangles.size();
        for (const PlanarCdtTriangle& localTriangle : faceMesh.triangles) {
            ++triangleProvenance.checked;
            ++triangleGeometry.checked;
            if (localTriangle.workingFace != face ||
                localTriangle.sourceFace != faceMesh.sourceFace) {
                ++triangleProvenance.failed;
                setFailure(result, "certified.triangle_provenance_invalid",
                           "a face triangle has inconsistent face provenance",
                           {face});
                return result;
            }
            CertifiedTriangle triangle;
            triangle.workingFace = face;
            triangle.sourceFace = faceMesh.sourceFace;
            std::array<std::uint32_t, 3> orientedLocalIndices =
                localTriangle.vertices;
            for (std::size_t corner = 0; corner < 3; ++corner) {
                const std::uint32_t localIndex =
                    localTriangle.vertices[corner];
                if (localIndex >= faceMesh.vertices.size()) {
                    ++triangleProvenance.failed;
                    setFailure(result, "certified.triangle_vertex_invalid",
                               "a face triangle references a missing local vertex",
                               {face});
                    return result;
                }
                const PlanarTrimVertex& localVertex =
                    faceMesh.vertices[localIndex];
                triangle.vertices[corner] = canonicalToGlobal.at(
                    localVertex.canonicalVertexIndex);
                triangle.cornerUv[corner] = localTriangle.cornerUv
                    ? (*localTriangle.cornerUv)[corner]
                    : localVertex.uv;
            }
            if (*orientation == TopologyOrientation::Reversed) {
                std::swap(triangle.vertices[1], triangle.vertices[2]);
                std::swap(triangle.cornerUv[1], triangle.cornerUv[2]);
                std::swap(orientedLocalIndices[1], orientedLocalIndices[2]);
            }

            const std::array<double, 3>& a =
                mesh.vertices[triangle.vertices[0]].position;
            const std::array<double, 3>& b =
                mesh.vertices[triangle.vertices[1]].position;
            const std::array<double, 3>& c =
                mesh.vertices[triangle.vertices[2]].position;
            const std::array<double, 3> normal = triangleNormal(a, b, c);
            const double normalSquared = dot(normal, normal);
            if (!std::isfinite(normalSquared) || !(normalSquared > 0.0)) {
                ++triangleGeometry.failed;
                setFailure(result, "certified.triangle_degenerate",
                           "a certified triangle has zero or non-finite 3D area",
                           {face});
                return result;
            }
            const EvaluationResult<SurfaceEvaluation> surface =
                imported.workingEvaluator->evaluateSurface(
                    face, triangle.cornerUv[0]);
            if (!surface || !surface.value->unitNormal) {
                ++triangleGeometry.failed;
                setFailure(result, "certified.surface_normal_unavailable",
                           "the exact surface has no evaluable normal",
                           {face});
                return result;
            }
            // GeometryEvaluator already applies TopoDS face orientation to
            // unitNormal. The triangle was swapped above for the same face
            // orientation, so direct positive alignment is required.
            if (!(dot(normal, *surface.value->unitNormal) > 0.0)) {
                ++triangleGeometry.failed;
                setFailure(result, "certified.triangle_orientation_invalid",
                           "a triangle winding disagrees with the oriented B-rep face normal",
                           {face});
                return result;
            }
            for (std::size_t corner = 0; corner < 3; ++corner) {
                const EvaluationResult<SurfaceEvaluation> evaluated =
                    imported.workingEvaluator->evaluateSurface(
                        face, triangle.cornerUv[corner]);
                const PlanarTrimVertex& localVertex =
                    faceMesh.vertices[orientedLocalIndices[corner]];
                double allowed =
                    configuration.maximumVertexSurfaceDiscrepancy;
                for (const PlanarTrimBoundaryUse& use :
                     localVertex.boundaryUses) {
                    allowed = std::min(
                        allowed,
                        use.allowedCurveOnSurfaceDiscrepancy);
                }
                const double maximumSquared = allowed * allowed;
                if (!evaluated ||
                    squaredDistance(
                        mesh.vertices[triangle.vertices[corner]].position,
                        evaluated.value->position) > maximumSquared) {
                    ++triangleGeometry.failed;
                    setFailure(result, "certified.vertex_off_surface",
                               "a triangle vertex exceeds its exact surface discrepancy limit",
                               {face});
                    return result;
                }
            }
            mesh.triangles.push_back(std::move(triangle));
        }
    }
    std::sort(mesh.triangles.begin(), mesh.triangles.end(),
              [](const CertifiedTriangle& first,
                 const CertifiedTriangle& second) {
                  return std::tie(first.workingFace, first.vertices,
                                  first.cornerUv) <
                      std::tie(second.workingFace, second.vertices,
                               second.cornerUv);
              });

    std::map<Edge, std::vector<DirectedEdgeUse>> edgeUses;
    for (const CertifiedTriangle& triangle : mesh.triangles) {
        for (std::size_t local = 0; local < 3; ++local) {
            const std::uint32_t first = triangle.vertices[local];
            const std::uint32_t second = triangle.vertices[(local + 1) % 3];
            const Edge key = edge(first, second);
            edgeUses[key].push_back({first == key.lower});
        }
    }
    ValidationCoverage& incidence =
        result.validation.checks[EdgeIncidence];
    ValidationCoverage& winding = result.validation.checks[EdgeWinding];
    incidence.expected = edgeUses.size();
    for (const auto& [key, uses] : edgeUses) {
        (void)key;
        ++incidence.checked;
        const bool incidenceValid = configuration.requireClosedManifold
            ? uses.size() == 2
            : (uses.size() == 1 || uses.size() == 2);
        if (!incidenceValid) {
            ++incidence.failed;
            setFailure(result, "certified.edge_incidence_invalid",
                       "a global triangle edge has invalid manifold incidence",
                       {});
        }
        if (uses.size() == 2) {
            ++winding.expected;
            ++winding.checked;
            if (uses[0].lowerToUpper == uses[1].lowerToUpper) {
                ++winding.failed;
                setFailure(result, "certified.edge_winding_conflict",
                           "two triangles traverse a shared edge in the same direction",
                           {});
            }
        }
    }
    if (incidence.failed != 0 || winding.failed != 0) return result;

    // Exact cross-face disjointness: triangles from different working faces
    // that share no global vertex must neither cross nor touch anywhere. Any
    // contact without shared canonical identity is a T-junction or a pierce
    // and fails closed; an unprovable pair also fails closed.
    {
        const std::shared_ptr<const GeometricPredicates> predicates =
            makeExactDyadicPredicates();
        ValidationCoverage& disjointness =
            result.validation.checks[TriangleIntersection];
        struct TriangleBounds {
            std::array<double, 3> lower{};
            std::array<double, 3> upper{};
        };
        std::vector<TriangleBounds> bounds;
        bounds.reserve(mesh.triangles.size());
        for (const CertifiedTriangle& triangle : mesh.triangles) {
            TriangleBounds box;
            for (std::size_t axis = 0; axis < 3; ++axis) {
                box.lower[axis] = std::numeric_limits<double>::infinity();
                box.upper[axis] = -std::numeric_limits<double>::infinity();
            }
            for (std::uint32_t vertex : triangle.vertices) {
                const std::array<double, 3>& position =
                    mesh.vertices[vertex].position;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    box.lower[axis] =
                        std::min(box.lower[axis], position[axis]);
                    box.upper[axis] =
                        std::max(box.upper[axis], position[axis]);
                }
            }
            bounds.push_back(box);
        }
        const auto trianglePoints = [&](const CertifiedTriangle& triangle) {
            std::array<PredicatePoint3, 3> points;
            for (std::size_t corner = 0; corner < 3; ++corner) {
                points[corner] =
                    mesh.vertices[triangle.vertices[corner]].position;
            }
            return points;
        };
        for (std::size_t first = 0;
             first < mesh.triangles.size() && disjointness.failed == 0;
             ++first) {
            for (std::size_t second = first + 1;
                 second < mesh.triangles.size(); ++second) {
                const CertifiedTriangle& one = mesh.triangles[first];
                const CertifiedTriangle& two = mesh.triangles[second];
                if (one.workingFace == two.workingFace) continue;
                bool sharesVertex = false;
                for (std::uint32_t a : one.vertices) {
                    for (std::uint32_t b : two.vertices) {
                        if (a == b) sharesVertex = true;
                    }
                }
                if (sharesVertex) continue;
                bool boxesDisjoint = false;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    if (bounds[first].upper[axis] <
                            bounds[second].lower[axis] ||
                        bounds[second].upper[axis] <
                            bounds[first].lower[axis]) {
                        boxesDisjoint = true;
                    }
                }
                if (boxesDisjoint) continue;
                ++disjointness.expected;
                const std::optional<bool> disjoint = trianglesDisjoint(
                    *predicates, trianglePoints(one), trianglePoints(two));
                if (!disjoint) {
                    ++disjointness.failed;
                    setFailure(result,
                               "certified.triangle_intersection_unproven",
                               "a cross-face triangle pair could not be "
                               "proven disjoint",
                               {one.workingFace, two.workingFace});
                    return result;
                }
                ++disjointness.checked;
                if (!*disjoint) {
                    ++disjointness.failed;
                    setFailure(result,
                               "certified.triangle_intersection_found",
                               "two non-adjacent certified triangles from "
                               "different faces touch or cross",
                               {one.workingFace, two.workingFace});
                    return result;
                }
            }
        }
    }

    // Refinement preserves the Euler characteristic exactly, so the
    // certified complex must report the same V - E + F as the source
    // face-complex it refines; any drift means lost or invented topology.
    {
        ValidationCoverage& euler =
            result.validation.checks[EulerCharacteristic];
        std::set<StableId> sourceEdges;
        std::set<StableId> sourceVertices;
        std::map<StableId, std::set<StableId>> wiresByFace;
        for (const CoedgeRecord& coedge :
             imported.working->snapshot.coedges) {
            if (expectedFaces.contains(coedge.faceId)) {
                sourceEdges.insert(coedge.edgeId);
                wiresByFace[coedge.faceId].insert(coedge.wireId);
            }
        }
        for (const EdgeTopologyRecord& record :
             imported.working->snapshot.edgeTopology) {
            if (!sourceEdges.contains(record.id)) continue;
            if (record.lowerVertex) sourceVertices.insert(*record.lowerVertex);
            if (record.upperVertex) sourceVertices.insert(*record.upperVertex);
        }
        // A face with w boundary wires is homotopic to a disk with w-1
        // holes and contributes 2-w, not 1, to the complex characteristic.
        long long faceContribution = 0;
        for (StableId face : expectedFaces) {
            const auto wires = wiresByFace.find(face);
            const long long wireCount =
                wires == wiresByFace.end()
                    ? 1
                    : static_cast<long long>(wires->second.size());
            faceContribution += 2 - wireCount;
        }
        const long long sourceCharacteristic =
            static_cast<long long>(sourceVertices.size()) -
            static_cast<long long>(sourceEdges.size()) + faceContribution;
        const long long meshCharacteristic =
            static_cast<long long>(mesh.vertices.size()) -
            static_cast<long long>(edgeUses.size()) +
            static_cast<long long>(mesh.triangles.size());
        ++euler.checked;
        if (sourceCharacteristic != meshCharacteristic) {
            ++euler.failed;
            setFailure(result, "certified.euler_characteristic_mismatch",
                       "the certified complex changed the source Euler "
                       "characteristic",
                       {});
            return result;
        }
    }

    mesh.topologyFingerprint = fingerprintOf(mesh);
    ValidationCoverage& fingerprint = result.validation.checks[Fingerprint];
    ++fingerprint.checked;
    if (mesh.topologyFingerprint.size() != 16) {
        ++fingerprint.failed;
        setFailure(result, "certified.fingerprint_failed",
                   "deterministic topology fingerprint generation failed", {});
        return result;
    }
    if (!result.validation.complete()) {
        setFailure(result, "certified.validation_incomplete",
                   "certified mesh validation coverage is incomplete", {});
        return result;
    }
    result.value = std::move(mesh);
    return result;
}

CertifiedMeshAssemblyResult assembleCertifiedPlanarMesh(
    const ImportedModel& imported,
    const CanonicalBoundarySet& boundaries,
    std::span<const PlanarCdtMesh> faceMeshes,
    std::span<const StableId> expectedWorkingFaces,
    const CertifiedMeshAssemblyConfiguration& configuration) {
    return assembleCertifiedBoundaryMesh(
        imported, boundaries, faceMeshes, expectedWorkingFaces,
        configuration);
}

}  // namespace weft
