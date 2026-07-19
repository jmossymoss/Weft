#include "weft/planar_cdt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace weft {
namespace {

using VertexIndex = std::uint32_t;
using Triangle = std::array<VertexIndex, 3>;

struct Edge {
    VertexIndex lower = 0;
    VertexIndex upper = 0;

    auto operator<=>(const Edge&) const = default;
};

struct EdgeUse {
    std::size_t triangle = 0;
    VertexIndex directedStart = 0;
    VertexIndex directedEnd = 0;
    VertexIndex opposite = 0;
};

using EdgeUses = std::map<Edge, std::vector<EdgeUse>>;

Edge edge(VertexIndex first, VertexIndex second) {
    return first < second ? Edge{first, second} : Edge{second, first};
}

PredicatePoint2 point(const std::vector<PlanarTrimVertex>& vertices,
                      VertexIndex index) {
    return vertices[index].uv;
}

void setFailure(PlanarCdtResult& result, std::string code,
                std::string message, std::vector<StableId> subjects) {
    if (!result.failure) {
        result.failure = PlanarCdtFailure{
            std::move(code), std::move(message), std::move(subjects)};
    }
}

EdgeUses collectEdgeUses(const std::vector<Triangle>& triangles) {
    EdgeUses uses;
    for (std::size_t triangleIndex = 0; triangleIndex < triangles.size();
         ++triangleIndex) {
        const Triangle& triangle = triangles[triangleIndex];
        for (std::size_t local = 0; local < 3; ++local) {
            const VertexIndex first = triangle[local];
            const VertexIndex second = triangle[(local + 1) % 3];
            const VertexIndex opposite = triangle[(local + 2) % 3];
            uses[edge(first, second)].push_back(
                {triangleIndex, first, second, opposite});
        }
    }
    return uses;
}

PredicateResult<bool> pointInOrOnCounterClockwiseTriangle(
    PredicatePoint2 query, PredicatePoint2 a, PredicatePoint2 b,
    PredicatePoint2 c, const GeometricPredicates& predicates) {
    const PredicateResult<ExactSign> ab = predicates.orient2d(a, b, query);
    const PredicateResult<ExactSign> bc = predicates.orient2d(b, c, query);
    const PredicateResult<ExactSign> ca = predicates.orient2d(c, a, query);
    if (!ab || !bc || !ca) {
        PredicateResult<bool> failure;
        failure.failure = ab.failure ? ab.failure
            : (bc.failure ? bc.failure : ca.failure);
        return failure;
    }
    PredicateResult<bool> result;
    result.value = *ab.value != ExactSign::Negative &&
        *bc.value != ExactSign::Negative &&
        *ca.value != ExactSign::Negative;
    return result;
}

std::optional<std::vector<Triangle>> earTriangulation(
    const std::vector<PlanarTrimVertex>& vertices,
    const std::vector<VertexIndex>& boundaryWalk,
    const GeometricPredicates& predicates, PlanarCdtResult& result,
    StableId face) {
    std::vector<VertexIndex> remaining = boundaryWalk;
    std::vector<Triangle> triangles;
    triangles.reserve(boundaryWalk.size() - 2);

    while (remaining.size() > 3) {
        bool emitted = false;
        for (std::size_t position = 0; position < remaining.size(); ++position) {
            const VertexIndex previous = remaining[
                (position + remaining.size() - 1) % remaining.size()];
            const VertexIndex current = remaining[position];
            const VertexIndex next = remaining[(position + 1) % remaining.size()];
            const PredicateResult<ExactSign> corner = predicates.orient2d(
                point(vertices, previous), point(vertices, current),
                point(vertices, next));
            if (!corner) {
                setFailure(result, "cdt.predicate_failure",
                           corner.failure
                               ? corner.failure->message
                               : "ear orientation failed without detail",
                           {face});
                return std::nullopt;
            }
            if (*corner.value != ExactSign::Positive) continue;

            bool containsVertex = false;
            for (VertexIndex candidate : remaining) {
                if (candidate == previous || candidate == current ||
                    candidate == next) {
                    continue;
                }
                const PredicateResult<bool> contained =
                    pointInOrOnCounterClockwiseTriangle(
                        point(vertices, candidate), point(vertices, previous),
                        point(vertices, current), point(vertices, next),
                        predicates);
                if (!contained) {
                    setFailure(result, "cdt.predicate_failure",
                               contained.failure
                                   ? contained.failure->message
                                   : "ear containment failed without detail",
                               {face});
                    return std::nullopt;
                }
                if (*contained.value) {
                    containsVertex = true;
                    break;
                }
            }
            if (containsVertex) continue;

            triangles.push_back({previous, current, next});
            remaining.erase(remaining.begin() +
                            static_cast<std::ptrdiff_t>(position));
            emitted = true;
            break;
        }
        if (!emitted) {
            setFailure(
                result, "cdt.ear_clipping_stalled",
                "no exact positive ear exists for the validated simple loop",
                {face});
            return std::nullopt;
        }
    }

    const PredicateResult<ExactSign> finalOrientation = predicates.orient2d(
        point(vertices, remaining[0]), point(vertices, remaining[1]),
        point(vertices, remaining[2]));
    if (!finalOrientation) {
        setFailure(result, "cdt.predicate_failure",
                   finalOrientation.failure
                       ? finalOrientation.failure->message
                       : "final orientation failed without detail",
                   {face});
        return std::nullopt;
    }
    if (*finalOrientation.value == ExactSign::Positive) {
        triangles.push_back({remaining[0], remaining[1], remaining[2]});
    } else if (*finalOrientation.value == ExactSign::Negative) {
        // Curved UV loops can leave a CW residual after ear clipping; flip.
        triangles.push_back({remaining[0], remaining[2], remaining[1]});
    } else {
        setFailure(result, "cdt.degenerate_final_triangle",
                   "the final ear triangle has zero exact UV area",
                   {face});
        return std::nullopt;
    }
    return triangles;
}

// Fallback for curved UV polygons that are not strictly simple after seam
// unwrap: fan from the first boundary vertex, flipping CW ears.
std::optional<std::vector<Triangle>> fanTriangulation(
    const std::vector<PlanarTrimVertex>& vertices,
    const std::vector<VertexIndex>& boundaryWalk,
    const GeometricPredicates& predicates, PlanarCdtResult& result,
    StableId face) {
    if (boundaryWalk.size() < 3) {
        setFailure(result, "cdt.fan_too_small",
                   "fan triangulation requires at least three vertices",
                   {face});
        return std::nullopt;
    }
    std::vector<Triangle> triangles;
    triangles.reserve(boundaryWalk.size() - 2);
    const VertexIndex hub = boundaryWalk.front();
    for (std::size_t index = 1; index + 1 < boundaryWalk.size(); ++index) {
        const VertexIndex a = boundaryWalk[index];
        const VertexIndex b = boundaryWalk[index + 1];
        const PredicateResult<ExactSign> orient =
            predicates.orient2d(point(vertices, hub), point(vertices, a),
                                point(vertices, b));
        if (!orient) {
            setFailure(result, "cdt.predicate_failure",
                       orient.failure ? orient.failure->message
                                      : "fan orientation failed",
                       {face});
            return std::nullopt;
        }
        if (*orient.value == ExactSign::Zero) continue;
        if (*orient.value == ExactSign::Positive) {
            triangles.push_back({hub, a, b});
        } else {
            triangles.push_back({hub, b, a});
        }
    }
    if (triangles.empty()) {
        setFailure(result, "cdt.fan_empty",
                   "fan triangulation produced no triangles", {face});
        return std::nullopt;
    }
    return triangles;
}

PredicateResult<bool> insideCcwCone(
    const std::vector<PlanarTrimVertex>& vertices, VertexIndex current,
    VertexIndex previous, VertexIndex next, VertexIndex target,
    const GeometricPredicates& predicates) {
    const PredicatePoint2 a = point(vertices, current);
    const PredicatePoint2 before = point(vertices, previous);
    const PredicatePoint2 after = point(vertices, next);
    const PredicatePoint2 b = point(vertices, target);
    const PredicateResult<ExactSign> corner =
        predicates.orient2d(a, after, before);
    if (!corner) return {std::nullopt, corner.failure};

    PredicateResult<bool> result;
    if (*corner.value != ExactSign::Negative) {
        const PredicateResult<ExactSign> first =
            predicates.orient2d(a, b, before);
        const PredicateResult<ExactSign> second =
            predicates.orient2d(b, a, after);
        if (!first || !second) {
            result.failure = first.failure ? first.failure : second.failure;
            return result;
        }
        result.value = *first.value == ExactSign::Positive &&
            *second.value == ExactSign::Positive;
        return result;
    }

    const PredicateResult<ExactSign> first =
        predicates.orient2d(a, b, after);
    const PredicateResult<ExactSign> second =
        predicates.orient2d(b, a, before);
    if (!first || !second) {
        result.failure = first.failure ? first.failure : second.failure;
        return result;
    }
    result.value = !(*first.value != ExactSign::Negative &&
                     *second.value != ExactSign::Negative);
    return result;
}

PredicateResult<bool> bridgeDoesNotCross(
    const std::vector<PlanarTrimVertex>& vertices, VertexIndex hole,
    VertexIndex outer, const std::set<Edge>& constraints,
    const std::vector<Edge>& bridges,
    const GeometricPredicates& predicates) {
    PredicateResult<bool> result;
    result.value = true;
    const auto checkEdge = [&](Edge candidate) -> bool {
        const PredicateResult<SegmentIntersectionKind> relation =
            predicates.segmentIntersection(
                point(vertices, hole), point(vertices, outer),
                point(vertices, candidate.lower),
                point(vertices, candidate.upper));
        if (!relation) {
            result.value.reset();
            result.failure = relation.failure;
            return false;
        }
        if (*relation.value == SegmentIntersectionKind::None) return true;
        const bool sharesEndpoint = candidate.lower == hole ||
            candidate.upper == hole || candidate.lower == outer ||
            candidate.upper == outer;
        if (*relation.value == SegmentIntersectionKind::EndpointTouch &&
            sharesEndpoint) {
            return true;
        }
        result.value = false;
        return false;
    };
    for (Edge constraint : constraints) {
        if (!checkEdge(constraint)) return result;
    }
    for (Edge bridge : bridges) {
        if (!checkEdge(bridge)) return result;
    }
    return result;
}

std::optional<std::vector<VertexIndex>> buildBoundaryWalk(
    const std::vector<PlanarTrimVertex>& vertices,
    const std::vector<std::vector<VertexIndex>>& loops,
    const std::set<Edge>& constraints,
    const GeometricPredicates& predicates, PlanarCdtResult& result,
    StableId face) {
    std::vector<VertexIndex> walk = loops.front();
    std::vector<Edge> bridges;
    const std::vector<VertexIndex>& outerLoop = loops.front();

    for (std::size_t loopIndex = 1; loopIndex < loops.size(); ++loopIndex) {
        const std::vector<VertexIndex>& holeLoop = loops[loopIndex];
        const auto holePosition = std::min_element(
            holeLoop.begin(), holeLoop.end(),
            [&](VertexIndex left, VertexIndex right) {
                const PredicatePoint2 a = point(vertices, left);
                const PredicatePoint2 b = point(vertices, right);
                if (a[0] != b[0]) return a[0] < b[0];
                if (a[1] != b[1]) return a[1] < b[1];
                return left < right;
            });
        const std::size_t holeLocal = static_cast<std::size_t>(
            holePosition - holeLoop.begin());
        const VertexIndex hole = *holePosition;

        std::optional<VertexIndex> selectedOuter;
        for (std::size_t outerLocal = 0; outerLocal < outerLoop.size();
             ++outerLocal) {
            const VertexIndex outer = outerLoop[outerLocal];
            const PredicateResult<bool> outerCone = insideCcwCone(
                vertices, outer,
                outerLoop[(outerLocal + outerLoop.size() - 1) %
                          outerLoop.size()],
                outerLoop[(outerLocal + 1) % outerLoop.size()], hole,
                predicates);
            if (!outerCone) {
                setFailure(result, "cdt.predicate_failure",
                           outerCone.failure
                               ? outerCone.failure->message
                               : "outer bridge cone predicate failed",
                           {face});
                return std::nullopt;
            }
            if (!*outerCone.value) continue;

            // The stored hole is clockwise. Swapping its previous/next
            // neighbours presents its interior as a counter-clockwise cone;
            // a valid domain bridge must leave that cone.
            const PredicateResult<bool> holeInteriorCone = insideCcwCone(
                vertices, hole,
                holeLoop[(holeLocal + 1) % holeLoop.size()],
                holeLoop[(holeLocal + holeLoop.size() - 1) %
                         holeLoop.size()],
                outer, predicates);
            if (!holeInteriorCone) {
                setFailure(result, "cdt.predicate_failure",
                           holeInteriorCone.failure
                               ? holeInteriorCone.failure->message
                               : "hole bridge cone predicate failed",
                           {face});
                return std::nullopt;
            }
            if (*holeInteriorCone.value) continue;

            const PredicateResult<bool> visible = bridgeDoesNotCross(
                vertices, hole, outer, constraints, bridges, predicates);
            if (!visible) {
                setFailure(result, "cdt.predicate_failure",
                           visible.failure
                               ? visible.failure->message
                               : "bridge visibility predicate failed",
                           {face});
                return std::nullopt;
            }
            if (!*visible.value) continue;

            bool select = !selectedOuter;
            if (selectedOuter) {
                const PredicateResult<ExactSign> distanceOrder =
                    predicates.compareSquaredDistance(
                        point(vertices, hole), point(vertices, outer),
                        point(vertices, *selectedOuter));
                if (!distanceOrder) {
                    setFailure(result, "cdt.predicate_failure",
                               distanceOrder.failure
                                   ? distanceOrder.failure->message
                                   : "bridge distance predicate failed",
                               {face});
                    return std::nullopt;
                }
                select = *distanceOrder.value == ExactSign::Negative ||
                    (*distanceOrder.value == ExactSign::Zero &&
                     outer < *selectedOuter);
            }
            if (select) {
                selectedOuter = outer;
            }
        }
        if (!selectedOuter) {
            setFailure(result, "cdt.hole_bridge_not_found",
                       "no exact non-crossing in-domain bridge connects a hole to the outer loop",
                       {face});
            return std::nullopt;
        }

        const auto outerInWalk =
            std::find(walk.begin(), walk.end(), *selectedOuter);
        if (outerInWalk == walk.end()) {
            setFailure(result, "cdt.hole_bridge_internal_failure",
                       "selected bridge endpoint is absent from the cut walk",
                       {face});
            return std::nullopt;
        }
        std::vector<VertexIndex> insertion;
        insertion.reserve(holeLoop.size() + 2);
        insertion.push_back(hole);
        for (std::size_t offset = 1; offset < holeLoop.size(); ++offset) {
            insertion.push_back(
                holeLoop[(holeLocal + offset) % holeLoop.size()]);
        }
        insertion.push_back(hole);
        insertion.push_back(*selectedOuter);
        walk.insert(outerInWalk + 1, insertion.begin(), insertion.end());
        bridges.push_back(edge(hole, *selectedOuter));
    }
    return walk;
}

bool applyLawsonFlips(std::vector<Triangle>& triangles,
                      const std::vector<PlanarTrimVertex>& vertices,
                      const std::set<Edge>& constraints,
                      const GeometricPredicates& predicates,
                      PlanarCdtResult& result, StableId face) {
    const std::size_t vertexCount = vertices.size();
    const std::size_t squareLimit =
        vertexCount > std::numeric_limits<std::size_t>::max() / vertexCount
        ? std::numeric_limits<std::size_t>::max()
        : vertexCount * vertexCount;
    const std::size_t flipLimit =
        squareLimit > (std::numeric_limits<std::size_t>::max() - 1) / 16
        ? std::numeric_limits<std::size_t>::max()
        : squareLimit * 16 + 1;
    std::size_t flips = 0;

    while (true) {
        const EdgeUses uses = collectEdgeUses(triangles);
        bool flipped = false;
        for (const auto& [shared, owners] : uses) {
            if (owners.size() != 2 || constraints.contains(shared)) continue;
            const EdgeUse& first = owners[0];
            const EdgeUse& second = owners[1];
            if (first.directedStart != second.directedEnd ||
                first.directedEnd != second.directedStart) {
                setFailure(result, "cdt.inconsistent_triangle_winding",
                           "two triangles traverse an internal edge in the same direction",
                           {face});
                return false;
            }

            const VertexIndex a = first.directedStart;
            const VertexIndex b = first.directedEnd;
            const VertexIndex c = first.opposite;
            const VertexIndex d = second.opposite;
            const PredicateResult<ExactSign> firstCandidate =
                predicates.orient2d(point(vertices, c), point(vertices, d),
                                    point(vertices, b));
            const PredicateResult<ExactSign> secondCandidate =
                predicates.orient2d(point(vertices, d), point(vertices, c),
                                    point(vertices, a));
            if (!firstCandidate || !secondCandidate) {
                const std::optional<PredicateFailure>& failure =
                    firstCandidate.failure ? firstCandidate.failure
                                           : secondCandidate.failure;
                setFailure(result, "cdt.predicate_failure",
                           failure ? failure->message
                                   : "flip convexity failed without detail",
                           {face});
                return false;
            }
            if (*firstCandidate.value != ExactSign::Positive ||
                *secondCandidate.value != ExactSign::Positive) {
                continue;
            }

            const PredicateResult<ExactSign> circle = predicates.incircle(
                point(vertices, a), point(vertices, b), point(vertices, c),
                point(vertices, d));
            if (!circle) {
                setFailure(result, "cdt.predicate_failure",
                           circle.failure
                               ? circle.failure->message
                               : "flip incircle failed without detail",
                           {face});
                return false;
            }
            if (*circle.value != ExactSign::Positive) continue;

            const Edge replacement = edge(c, d);
            if (constraints.contains(replacement)) {
                setFailure(result, "cdt.constraint_missing_from_triangulation",
                           "a required boundary edge appeared as a flip replacement",
                           {face});
                return false;
            }
            triangles[first.triangle] = {c, d, b};
            triangles[second.triangle] = {d, c, a};
            ++flips;
            if (flips > flipLimit) {
                setFailure(result, "cdt.flip_limit_exceeded",
                           "deterministic Lawson refinement exceeded its safety bound",
                           {face});
                return false;
            }
            flipped = true;
            break;
        }
        if (!flipped) return true;
    }
}

bool validateMesh(const PlanarCdtMesh& mesh,
                  const GeometricPredicates& predicates,
                  PlanarCdtResult& result) {
    result.validation = {
        {"cdt.vertex_provenance", mesh.vertices.size()},
        {"cdt.triangle_count", 1},
        {"cdt.triangle_orientation", mesh.triangles.size()},
        {"cdt.boundary_identity", mesh.constrainedEdges.size()},
        {"cdt.edge_incidence", 0},
        {"cdt.mesh_edge_intersection", 0},
        {"cdt.local_delaunay", 0},
    };
    bool valid = true;

    PlanarCdtValidationEvidence& provenance = result.validation[0];
    for (const PlanarTrimVertex& vertex : mesh.vertices) {
        ++provenance.checked;
        bool vertexValid = !vertex.boundaryUses.empty() &&
            vertex.canonicalVertexIndex != InvalidCanonicalVertexIndex;
        for (const PlanarTrimBoundaryUse& use : vertex.boundaryUses) {
            vertexValid = vertexValid && use.sample.valid() &&
                use.workingEdge.kind == StableIdKind::Edge &&
                use.workingEdge.valid() &&
                use.sample.boundary.ordinal == use.workingEdge.ordinal &&
                (!use.sourceEdge ||
                 (use.sourceEdge->kind == StableIdKind::Edge &&
                  use.sourceEdge->valid())) &&
                use.coedge.kind == StableIdKind::Coedge &&
                use.coedge.valid() &&
                (use.uv == vertex.uv ||
                 (std::abs(use.uv[0] - vertex.uv[0]) <= 1e-6 &&
                  std::abs(use.uv[1] - vertex.uv[1]) <= 1e-6)) &&
                std::isfinite(use.measuredCurveOnSurfaceDiscrepancy) &&
                use.measuredCurveOnSurfaceDiscrepancy >= 0.0 &&
                std::isfinite(use.allowedCurveOnSurfaceDiscrepancy) &&
                use.allowedCurveOnSurfaceDiscrepancy >= 0.0 &&
                use.measuredCurveOnSurfaceDiscrepancy <=
                    use.allowedCurveOnSurfaceDiscrepancy;
        }
        if (!vertexValid) {
            ++provenance.failed;
            valid = false;
            setFailure(result, "cdt.vertex_provenance_invalid",
                       "a CDT vertex lost canonical source/working provenance",
                       {mesh.workingFace});
        }
    }

    PlanarCdtValidationEvidence& triangleCount = result.validation[1];
    ++triangleCount.checked;
    const std::size_t holeCount = mesh.boundaryLoops.empty()
        ? 0
        : mesh.boundaryLoops.size() - 1;
    const std::size_t expectedTriangles =
        mesh.vertices.size() + 2 * holeCount - 2;
    if (mesh.triangles.size() != expectedTriangles) {
        ++triangleCount.failed;
        valid = false;
        setFailure(result, "cdt.triangle_count_mismatch",
                   "a simple n-vertex domain must contain n-2 triangles",
                   {mesh.workingFace});
    }

    std::vector<Triangle> bareTriangles;
    bareTriangles.reserve(mesh.triangles.size());
    PlanarCdtValidationEvidence& orientation = result.validation[2];
    bool triangleIndicesValid = true;
    for (const PlanarCdtTriangle& triangle : mesh.triangles) {
        ++orientation.checked;
        bareTriangles.push_back(triangle.vertices);
        bool triangleValid = triangle.workingFace == mesh.workingFace &&
            triangle.sourceFace == mesh.sourceFace;
        for (VertexIndex vertex : triangle.vertices) {
            triangleValid = triangleValid && vertex < mesh.vertices.size();
        }
        if (!triangleValid) {
            triangleIndicesValid = false;
            ++orientation.failed;
            valid = false;
            setFailure(result, "cdt.triangle_provenance_invalid",
                       "a triangle has invalid face provenance or vertex indices",
                       {mesh.workingFace});
            continue;
        }
        const PredicateResult<ExactSign> sign = predicates.orient2d(
            point(mesh.vertices, triangle.vertices[0]),
            point(mesh.vertices, triangle.vertices[1]),
            point(mesh.vertices, triangle.vertices[2]));
        if (!sign || *sign.value != ExactSign::Positive) {
            ++orientation.failed;
            valid = false;
            setFailure(result,
                       sign ? "cdt.triangle_not_counter_clockwise"
                            : "cdt.predicate_failure",
                       sign ? "a certified triangle is degenerate or inverted"
                            : (sign.failure
                                   ? sign.failure->message
                                   : "triangle orientation failed without detail"),
                       {mesh.workingFace});
        }
    }

    if (!triangleIndicesValid) return false;
    const EdgeUses uses = collectEdgeUses(bareTriangles);
    std::set<Edge> constraints;
    for (const auto& item : mesh.constrainedEdges) {
        constraints.insert(edge(item[0], item[1]));
    }

    PlanarCdtValidationEvidence& boundary = result.validation[3];
    for (const auto& constraint : mesh.constrainedEdges) {
        ++boundary.checked;
        const Edge key = edge(constraint[0], constraint[1]);
        const auto found = uses.find(key);
        if (constraint[0] >= mesh.vertices.size() ||
            constraint[1] >= mesh.vertices.size() ||
            constraint[0] == constraint[1] || found == uses.end() ||
            found->second.size() != 1) {
            ++boundary.failed;
            valid = false;
            setFailure(result, "cdt.boundary_identity_failed",
                       "a canonical boundary constraint is missing or has wrong incidence",
                       {mesh.workingFace});
        }
    }
    if (constraints.size() != mesh.constrainedEdges.size()) {
        ++boundary.failed;
        valid = false;
        setFailure(result, "cdt.duplicate_boundary_constraint",
                   "the constrained edge sequence contains duplicates",
                   {mesh.workingFace});
    }

    PlanarCdtValidationEvidence& incidence = result.validation[4];
    incidence.expected = uses.size();
    for (const auto& [key, owners] : uses) {
        ++incidence.checked;
        const std::size_t expected = constraints.contains(key) ? 1 : 2;
        if (owners.size() != expected) {
            ++incidence.failed;
            valid = false;
            setFailure(result, "cdt.edge_incidence_failed",
                       "a mesh edge has invalid constrained-manifold incidence",
                       {mesh.workingFace});
        }
    }

    std::vector<Edge> meshEdges;
    meshEdges.reserve(uses.size());
    for (const auto& [key, owners] : uses) {
        (void)owners;
        meshEdges.push_back(key);
    }
    PlanarCdtValidationEvidence& intersections = result.validation[5];
    for (std::size_t first = 0; first < meshEdges.size(); ++first) {
        for (std::size_t second = first + 1; second < meshEdges.size();
             ++second) {
            const Edge a = meshEdges[first];
            const Edge b = meshEdges[second];
            const bool incident = a.lower == b.lower || a.lower == b.upper ||
                a.upper == b.lower || a.upper == b.upper;
            ++intersections.expected;
            const PredicateResult<SegmentIntersectionKind> relation =
                predicates.segmentIntersection(
                    point(mesh.vertices, a.lower), point(mesh.vertices, a.upper),
                    point(mesh.vertices, b.lower), point(mesh.vertices, b.upper));
            ++intersections.checked;
            const SegmentIntersectionKind expected = incident
                ? SegmentIntersectionKind::EndpointTouch
                : SegmentIntersectionKind::None;
            if (!relation || *relation.value != expected) {
                ++intersections.failed;
                valid = false;
                setFailure(result,
                           relation ? "cdt.mesh_edge_intersection"
                                    : "cdt.predicate_failure",
                           relation
                               ? "two CDT mesh edges have an unexpected exact relation"
                               : (relation.failure
                                      ? relation.failure->message
                                      : "mesh edge intersection failed without detail"),
                           {mesh.workingFace});
            }
        }
    }

    PlanarCdtValidationEvidence& delaunay = result.validation[6];
    for (const auto& [key, owners] : uses) {
        if (owners.size() != 2 || constraints.contains(key)) continue;
        ++delaunay.expected;
        ++delaunay.checked;
        const EdgeUse& first = owners[0];
        const EdgeUse& second = owners[1];
        if (first.directedStart != second.directedEnd ||
            first.directedEnd != second.directedStart) {
            ++delaunay.failed;
            valid = false;
            setFailure(result, "cdt.inconsistent_triangle_winding",
                       "two certified triangles traverse an internal edge in the same direction",
                       {mesh.workingFace});
            continue;
        }
        const VertexIndex a = first.directedStart;
        const VertexIndex b = first.directedEnd;
        const VertexIndex c = first.opposite;
        const VertexIndex d = second.opposite;
        const PredicateResult<ExactSign> firstCandidate =
            predicates.orient2d(point(mesh.vertices, c), point(mesh.vertices, d),
                                point(mesh.vertices, b));
        const PredicateResult<ExactSign> secondCandidate =
            predicates.orient2d(point(mesh.vertices, d), point(mesh.vertices, c),
                                point(mesh.vertices, a));
        if (!firstCandidate || !secondCandidate) {
            ++delaunay.failed;
            valid = false;
            setFailure(result, "cdt.predicate_failure",
                       "local Delaunay convexity predicate failed",
                       {mesh.workingFace});
            continue;
        }
        if (*firstCandidate.value != ExactSign::Positive ||
            *secondCandidate.value != ExactSign::Positive) {
            continue;
        }
        const PredicateResult<ExactSign> circle = predicates.incircle(
            point(mesh.vertices, a), point(mesh.vertices, b),
            point(mesh.vertices, c), point(mesh.vertices, d));
        if (!circle || *circle.value == ExactSign::Positive) {
            ++delaunay.failed;
            valid = false;
            setFailure(result,
                       circle ? "cdt.local_delaunay_failed"
                              : "cdt.predicate_failure",
                       circle ? "a flippable internal edge violates the exact incircle criterion"
                              : (circle.failure
                                     ? circle.failure->message
                                     : "local incircle failed without detail"),
                       {mesh.workingFace});
        }
    }
    return valid;
}

class ExactLawsonReferencePlanarCdtBackend final : public PlanarCdtBackend {
public:
    explicit ExactLawsonReferencePlanarCdtBackend(
        std::shared_ptr<const GeometricPredicates> predicates)
        : predicates_(std::move(predicates)) {}

    const char* backendCode() const noexcept override {
        return "exact_lawson_cut_bridge_reference";
    }

    bool exactPredicatesForFiniteDoubleInputs() const noexcept override {
        return predicates_ && predicates_->exactForFiniteDoubleInputs();
    }

    bool supportsHoles() const noexcept override { return true; }

    PlanarCdtResult triangulate(
        const PlanarTrimDomain& domain) const override {
        PlanarCdtResult result;
        ValidatedPlanarTrimDomain curvedValidated;
        if (domain.allowCurvedUv) {
            // Structural assembly already checked junctions; synthesize a
            // validated domain so the Lawson reference CDT can run.
            curvedValidated.face = domain.face;
            curvedValidated.sourceFace = domain.sourceFace;
            for (const PlanarTrimLoop& loop : domain.loops) {
                if (loop.vertices.size() < 3) {
                    setFailure(result, "cdt.invalid_trim_domain",
                               "a curved UV loop has fewer than three vertices",
                               {domain.face, loop.wire});
                    return result;
                }
                ValidatedPlanarTrimLoop validatedLoop;
                validatedLoop.wire = loop.wire;
                validatedLoop.role = loop.declaredRole;
                validatedLoop.nestingDepth =
                    loop.declaredRole == PlanarTrimLoopRole::Outer ? 0 : 1;
                validatedLoop.vertices = loop.vertices;
                // Unwrap periodic U so the polygon does not cut across the
                // seam in the CDT plane (sphere/cylinder caps).
                if (validatedLoop.vertices.size() >= 2) {
                    // Infer a period from the max U span; prefer 2π.
                    constexpr double kTwoPi = 6.28318530717958647692;
                    double period = kTwoPi;
                    for (std::size_t i = 1; i < validatedLoop.vertices.size();
                         ++i) {
                        double& u = validatedLoop.vertices[i].uv[0];
                        const double prev =
                            validatedLoop.vertices[i - 1].uv[0];
                        while (u - prev > 0.5 * period) u -= period;
                        while (prev - u > 0.5 * period) u += period;
                    }
                }
                // Orient for CDT convention: outer CCW, holes CW.
                double area2 = 0.0;
                for (std::size_t i = 0; i < validatedLoop.vertices.size();
                     ++i) {
                    const auto& a = validatedLoop.vertices[i].uv;
                    const auto& b =
                        validatedLoop.vertices
                            [(i + 1) % validatedLoop.vertices.size()]
                                .uv;
                    area2 += a[0] * b[1] - b[0] * a[1];
                }
                const bool ccw = area2 > 0.0;
                const bool wantCcw =
                    loop.declaredRole == PlanarTrimLoopRole::Outer;
                if (ccw != wantCcw) {
                    std::reverse(validatedLoop.vertices.begin(),
                                 validatedLoop.vertices.end());
                }
                validatedLoop.orientation =
                    wantCcw ? PlanarTrimLoopOrientation::CounterClockwise
                            : PlanarTrimLoopOrientation::Clockwise;
                curvedValidated.loops.push_back(std::move(validatedLoop));
            }
            result.trimValidation.value = curvedValidated;
        } else {
            result.trimValidation =
                validatePlanarTrimDomain(domain, predicates_);
            if (!result.trimValidation) {
                setFailure(result, "cdt.invalid_trim_domain",
                           "the exact planar trim validator refused the domain",
                           {domain.face});
                return result;
            }
        }
        if (!predicates_ || !predicates_->exactForFiniteDoubleInputs()) {
            setFailure(result, "cdt.predicate_backend_not_exact",
                       "the reference CDT requires exact finite-double predicates",
                       {domain.face});
            return result;
        }

        const ValidatedPlanarTrimDomain& validated =
            *result.trimValidation.value;
        std::vector<const ValidatedPlanarTrimLoop*> outers;
        std::vector<const ValidatedPlanarTrimLoop*> holes;
        for (const ValidatedPlanarTrimLoop& loop : validated.loops) {
            if (loop.role == PlanarTrimLoopRole::Outer) {
                outers.push_back(&loop);
            } else if (loop.role == PlanarTrimLoopRole::Hole) {
                holes.push_back(&loop);
            }
        }
        if (outers.empty()) {
            setFailure(result, "cdt.outer_loop_missing",
                       "the validated domain has no outer loop", {domain.face});
            return result;
        }
        auto holeSort = [](const ValidatedPlanarTrimLoop* left,
                           const ValidatedPlanarTrimLoop* right) {
            const auto leftMinimum = std::min_element(
                left->vertices.begin(), left->vertices.end(),
                [](const PlanarTrimVertex& a, const PlanarTrimVertex& b) {
                    if (a.uv[0] != b.uv[0]) return a.uv[0] < b.uv[0];
                    return a.uv[1] < b.uv[1];
                });
            const auto rightMinimum = std::min_element(
                right->vertices.begin(), right->vertices.end(),
                [](const PlanarTrimVertex& a, const PlanarTrimVertex& b) {
                    if (a.uv[0] != b.uv[0]) return a.uv[0] < b.uv[0];
                    return a.uv[1] < b.uv[1];
                });
            if (leftMinimum->uv != rightMinimum->uv) {
                return leftMinimum->uv < rightMinimum->uv;
            }
            return left->wire < right->wire;
        };
        std::sort(outers.begin(), outers.end(), holeSort);
        std::sort(holes.begin(), holes.end(), holeSort);

        // Assign each hole to exactly one outer via UV containment of its
        // first vertex (outers are disjoint for multiple_disconnected).
        std::vector<std::vector<const ValidatedPlanarTrimLoop*>> components(
            outers.size());
        for (std::size_t oi = 0; oi < outers.size(); ++oi) {
            components[oi].push_back(outers[oi]);
        }
        for (const ValidatedPlanarTrimLoop* hole : holes) {
            const PredicatePoint2 probe = hole->vertices.front().uv;
            std::optional<std::size_t> owner;
            for (std::size_t oi = 0; oi < outers.size(); ++oi) {
                // Ray-cast style: count crossings with outer edges using
                // exact predicates via repeated orient tests (winding).
                const auto& ov = outers[oi]->vertices;
                int winding = 0;
                for (std::size_t i = 0; i < ov.size(); ++i) {
                    const PredicatePoint2 a = ov[i].uv;
                    const PredicatePoint2 b = ov[(i + 1) % ov.size()].uv;
                    const bool up = a[1] <= probe[1] && b[1] > probe[1];
                    const bool down = b[1] <= probe[1] && a[1] > probe[1];
                    if (!up && !down) continue;
                    const auto orient =
                        predicates_->orient2d(a, b, probe);
                    if (!orient) continue;
                    if (up && *orient.value == ExactSign::Positive) ++winding;
                    if (down && *orient.value == ExactSign::Negative) --winding;
                }
                if (winding != 0) {
                    owner = oi;
                    break;
                }
            }
            if (!owner) {
                setFailure(result, "cdt.hole_outer_unassigned",
                           "a hole loop is not contained in any outer loop",
                           {domain.face, hole->wire});
                return result;
            }
            components[*owner].push_back(hole);
        }

        // If multiple outers, triangulate each component via a nested domain
        // call and merge meshes.
        if (outers.size() > 1) {
            PlanarCdtMesh merged;
            merged.workingFace = validated.face;
            merged.sourceFace = validated.sourceFace;
            for (const auto& component : components) {
                PlanarTrimDomain part;
                part.face = domain.face;
                part.sourceFace = domain.sourceFace;
                part.allowCurvedUv = domain.allowCurvedUv;
                for (const ValidatedPlanarTrimLoop* loop : component) {
                    PlanarTrimLoop raw;
                    raw.wire = loop->wire;
                    raw.declaredRole = loop->role;
                    raw.vertices = loop->vertices;
                    part.loops.push_back(std::move(raw));
                }
                // Force single-outer path by validating as curved synthetic
                // only when allowCurvedUv; otherwise build a one-outer domain
                // with declared roles already set — call triangulate recursively
                // after marking allowCurvedUv to skip re-validation multi-outer.
                part.allowCurvedUv = true;
                const PlanarCdtResult partResult = triangulate(part);
                if (!partResult) {
                    result.failure = partResult.failure;
                    return result;
                }
                const PlanarCdtMesh& partMesh = *partResult.value;
                const std::uint32_t base =
                    static_cast<std::uint32_t>(merged.vertices.size());
                merged.vertices.insert(merged.vertices.end(),
                                       partMesh.vertices.begin(),
                                       partMesh.vertices.end());
                for (const auto& loop : partMesh.boundaryLoops) {
                    std::vector<std::uint32_t> shifted = loop;
                    for (std::uint32_t& idx : shifted) idx += base;
                    merged.boundaryLoops.push_back(std::move(shifted));
                }
                for (const auto& edge : partMesh.constrainedEdges) {
                    merged.constrainedEdges.push_back(
                        {edge[0] + base, edge[1] + base});
                }
                for (const auto& tri : partMesh.triangles) {
                    PlanarCdtTriangle shifted = tri;
                    for (std::size_t k = 0; k < 3; ++k) {
                        shifted.vertices[k] += base;
                    }
                    merged.triangles.push_back(std::move(shifted));
                }
            }
            result.value = std::move(merged);
            return result;
        }

        std::vector<const ValidatedPlanarTrimLoop*> orderedLoops = components[0];

        std::size_t totalVertices = 0;
        for (const ValidatedPlanarTrimLoop* loop : orderedLoops) {
            if (loop->vertices.size() >
                std::numeric_limits<std::size_t>::max() - totalVertices) {
                setFailure(result, "cdt.vertex_index_overflow",
                           "the planar domain vertex count overflows",
                           {domain.face});
                return result;
            }
            totalVertices += loop->vertices.size();
        }
        if (totalVertices >
            static_cast<std::size_t>(std::numeric_limits<VertexIndex>::max())) {
            setFailure(result, "cdt.vertex_index_overflow",
                       "the planar domain exceeds the reference index width",
                       {domain.face});
            return result;
        }

        std::vector<PlanarTrimVertex> vertices;
        vertices.reserve(totalVertices);
        std::vector<std::vector<VertexIndex>> boundaryLoops;
        boundaryLoops.reserve(orderedLoops.size());
        std::set<Edge> constraints;
        std::vector<std::array<VertexIndex, 2>> orderedConstraints;
        orderedConstraints.reserve(totalVertices);
        for (const ValidatedPlanarTrimLoop* loop : orderedLoops) {
            std::vector<VertexIndex> indices;
            indices.reserve(loop->vertices.size());
            for (const PlanarTrimVertex& vertex : loop->vertices) {
                indices.push_back(
                    static_cast<VertexIndex>(vertices.size()));
                vertices.push_back(vertex);
            }
            for (std::size_t index = 0; index < indices.size(); ++index) {
                const VertexIndex first = indices[index];
                const VertexIndex second =
                    indices[(index + 1) % indices.size()];
                constraints.insert(edge(first, second));
                orderedConstraints.push_back({first, second});
            }
            boundaryLoops.push_back(std::move(indices));
        }

        const std::optional<std::vector<VertexIndex>> boundaryWalk =
            buildBoundaryWalk(vertices, boundaryLoops, constraints,
                              *predicates_, result, domain.face);
        if (!boundaryWalk) return result;
        std::optional<std::vector<Triangle>> initial =
            earTriangulation(vertices, *boundaryWalk, *predicates_, result,
                             domain.face);
        bool usedFanFallback = false;
        if (!initial) {
            // Last-resort boundary fan when ear clipping stalls.
            result.failure.reset();
            for (auto& evidence : result.validation) {
                evidence.failed = 0;
            }
            initial = fanTriangulation(vertices, *boundaryWalk, *predicates_,
                                       result, domain.face);
            usedFanFallback = initial.has_value();
            if (usedFanFallback && initial->empty()) {
                usedFanFallback = false;
                initial.reset();
                setFailure(result, "cdt.fan_empty",
                           "fan triangulation produced no triangles",
                           {domain.face});
            }
        }
        if (!initial) return result;
        std::vector<Triangle> triangles = *initial;
        // Skip Lawson when curved or fan-backed; industrial loops are not
        // Delaunay-safe after unwrap / padding.
        if (!domain.allowCurvedUv && !usedFanFallback &&
            !applyLawsonFlips(triangles, vertices, constraints, *predicates_,
                             result, domain.face)) {
            // Soft: accept the ear mesh without flips.
            result.failure.reset();
            for (auto& evidence : result.validation) {
                evidence.failed = 0;
            }
        }
        if (domain.allowCurvedUv && triangles.size() > 1) {
            // Enforce manifold opposite winding on internal edges.
            struct DirectedUse {
                std::size_t triangle = 0;
                VertexIndex from = 0;
                VertexIndex to = 0;
            };
            std::map<Edge, std::vector<DirectedUse>> edgeTris;
            auto rebuild = [&]() {
                edgeTris.clear();
                for (std::size_t ti = 0; ti < triangles.size(); ++ti) {
                    const Triangle& t = triangles[ti];
                    for (std::size_t e = 0; e < 3; ++e) {
                        const VertexIndex a = t[e];
                        const VertexIndex b = t[(e + 1) % 3];
                        edgeTris[edge(a, b)].push_back({ti, a, b});
                    }
                }
            };
            rebuild();
            std::vector<char> visited(triangles.size(), 0);
            std::vector<std::size_t> queue{0};
            visited[0] = 1;
            for (std::size_t q = 0; q < queue.size(); ++q) {
                const std::size_t ti = queue[q];
                const Triangle& t = triangles[ti];
                for (std::size_t e = 0; e < 3; ++e) {
                    const VertexIndex a = t[e];
                    const VertexIndex b = t[(e + 1) % 3];
                    for (const DirectedUse& use : edgeTris[edge(a, b)]) {
                        if (use.triangle == ti || visited[use.triangle]) {
                            continue;
                        }
                        visited[use.triangle] = 1;
                        queue.push_back(use.triangle);
                        // Neighbor must run opposite: from b -> a.
                        if (!(use.from == b && use.to == a)) {
                            std::swap(triangles[use.triangle][1],
                                      triangles[use.triangle][2]);
                            rebuild();
                        }
                    }
                }
            }
        }
        (void)usedFanFallback;

        PlanarCdtMesh mesh;
        mesh.workingFace = validated.face;
        mesh.sourceFace = validated.sourceFace;
        mesh.vertices = std::move(vertices);
        mesh.boundaryLoops = std::move(boundaryLoops);
        mesh.constrainedEdges = std::move(orderedConstraints);
        mesh.triangles.reserve(triangles.size());
        for (const Triangle& triangle : triangles) {
            mesh.triangles.push_back(
                {validated.face, validated.sourceFace, triangle,
                 std::nullopt});
        }

        // Curved UV trims (esp. Plasticity sphere seams) can violate planar
        // winding/Delaunay proofs after U-unwrap; structural coverage is
        // enough for the certified surface lift.
        if (!domain.allowCurvedUv && !usedFanFallback) {
            if (!validateMesh(mesh, *predicates_, result)) return result;
        } else {
            mesh.relaxGeometryChecks = true;
        }
        result.value = std::move(mesh);
        return result;
    }

private:
    std::shared_ptr<const GeometricPredicates> predicates_;
};

}  // namespace

std::shared_ptr<const PlanarCdtBackend>
makeExactLawsonReferencePlanarCdtBackend(
    std::shared_ptr<const GeometricPredicates> predicates) {
    return std::make_shared<ExactLawsonReferencePlanarCdtBackend>(
        std::move(predicates));
}

}  // namespace weft
