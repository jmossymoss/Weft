#include "weft/planar_cdt.hpp"

#include <algorithm>
#include <array>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
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
    const GeometricPredicates& predicates, PlanarCdtResult& result,
    StableId face) {
    std::vector<VertexIndex> remaining(vertices.size());
    std::iota(remaining.begin(), remaining.end(), VertexIndex{0});
    std::vector<Triangle> triangles;
    triangles.reserve(vertices.size() - 2);

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
    if (!finalOrientation ||
        *finalOrientation.value != ExactSign::Positive) {
        setFailure(result,
                   finalOrientation ? "cdt.degenerate_final_triangle"
                                    : "cdt.predicate_failure",
                   finalOrientation
                       ? "the final ear triangle is not counter-clockwise"
                       : (finalOrientation.failure
                              ? finalOrientation.failure->message
                              : "final orientation failed without detail"),
                   {face});
        return std::nullopt;
    }
    triangles.push_back({remaining[0], remaining[1], remaining[2]});
    return triangles;
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
                use.uv == vertex.uv &&
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
    const std::size_t expectedTriangles = mesh.vertices.size() - 2;
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
        return "exact_lawson_single_loop_reference";
    }

    bool exactPredicatesForFiniteDoubleInputs() const noexcept override {
        return predicates_ && predicates_->exactForFiniteDoubleInputs();
    }

    bool supportsHoles() const noexcept override { return false; }

    PlanarCdtResult triangulate(
        const PlanarTrimDomain& domain) const override {
        PlanarCdtResult result;
        result.trimValidation = validatePlanarTrimDomain(domain, predicates_);
        if (!result.trimValidation) {
            setFailure(result, "cdt.invalid_trim_domain",
                       "the exact planar trim validator refused the domain",
                       {domain.face});
            return result;
        }
        if (!predicates_ || !predicates_->exactForFiniteDoubleInputs()) {
            setFailure(result, "cdt.predicate_backend_not_exact",
                       "the reference CDT requires exact finite-double predicates",
                       {domain.face});
            return result;
        }

        const ValidatedPlanarTrimDomain& validated =
            *result.trimValidation.value;
        if (validated.loops.size() != 1 ||
            validated.loops.front().role != PlanarTrimLoopRole::Outer) {
            setFailure(
                result, "cdt.holes_not_supported_by_reference",
                "the reference backend currently accepts exactly one outer loop",
                {domain.face});
            return result;
        }
        const std::vector<PlanarTrimVertex>& vertices =
            validated.loops.front().vertices;
        if (vertices.size() >
            static_cast<std::size_t>(std::numeric_limits<VertexIndex>::max())) {
            setFailure(result, "cdt.vertex_index_overflow",
                       "the planar domain exceeds the reference index width",
                       {domain.face});
            return result;
        }

        const std::optional<std::vector<Triangle>> initial =
            earTriangulation(vertices, *predicates_, result, domain.face);
        if (!initial) return result;
        std::vector<Triangle> triangles = *initial;

        std::set<Edge> constraints;
        std::vector<std::array<VertexIndex, 2>> orderedConstraints;
        orderedConstraints.reserve(vertices.size());
        for (std::size_t index = 0; index < vertices.size(); ++index) {
            const VertexIndex first = static_cast<VertexIndex>(index);
            const VertexIndex second = static_cast<VertexIndex>(
                (index + 1) % vertices.size());
            constraints.insert(edge(first, second));
            orderedConstraints.push_back({first, second});
        }
        if (!applyLawsonFlips(triangles, vertices, constraints, *predicates_,
                             result, domain.face)) {
            return result;
        }

        PlanarCdtMesh mesh;
        mesh.workingFace = validated.face;
        mesh.sourceFace = validated.sourceFace;
        mesh.vertices = vertices;
        mesh.constrainedEdges = std::move(orderedConstraints);
        mesh.triangles.reserve(triangles.size());
        for (const Triangle& triangle : triangles) {
            mesh.triangles.push_back(
                {validated.face, validated.sourceFace, triangle});
        }

        if (!validateMesh(mesh, *predicates_, result)) return result;
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
