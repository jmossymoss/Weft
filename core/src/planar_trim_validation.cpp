#include "weft/planar_trim_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace weft {
namespace {

enum EvidenceIndex : std::size_t {
    DomainStructure = 0,
    LoopStructure,
    LoopEdgeRelations,
    InterLoopRelations,
    Containment,
    RoleAndNesting,
    Orientation,
    EvidenceCount,
};

enum class PointContainment {
    Outside,
    Inside,
    Boundary,
};

struct PointContainmentResult {
    std::optional<PointContainment> value;
    std::optional<PredicateFailure> failure;
};

std::size_t edgePairCount(std::size_t edgeCount) {
    return edgeCount < 2 ? 0 : edgeCount * (edgeCount - 1) / 2;
}

bool finite(PredicatePoint2 point) {
    return std::isfinite(point[0]) && std::isfinite(point[1]);
}

bool equal(PredicatePoint2 left, PredicatePoint2 right) {
    return left[0] == right[0] && left[1] == right[1];
}

bool onBoundingBox(PredicatePoint2 point, PredicatePoint2 first,
                   PredicatePoint2 second) {
    return point[0] >= std::min(first[0], second[0]) &&
        point[0] <= std::max(first[0], second[0]) &&
        point[1] >= std::min(first[1], second[1]) &&
        point[1] <= std::max(first[1], second[1]);
}

const char* intersectionName(SegmentIntersectionKind kind) {
    switch (kind) {
        case SegmentIntersectionKind::None:
            return "none";
        case SegmentIntersectionKind::Proper:
            return "proper intersection";
        case SegmentIntersectionKind::EndpointTouch:
            return "endpoint touch";
        case SegmentIntersectionKind::CollinearOverlap:
            return "collinear overlap";
    }
    return "unknown intersection";
}

void addDiagnostic(PlanarTrimValidationResult& result, std::string code,
                   std::string message, std::vector<StableId> subjects) {
    result.diagnostics.push_back(
        {std::move(code), std::move(message), std::move(subjects)});
}

PointContainmentResult pointInLoop(
    PredicatePoint2 point, const PlanarTrimLoop& loop,
    const GeometricPredicates& predicates) {
    int winding = 0;
    for (std::size_t index = 0; index < loop.vertices.size(); ++index) {
        const PredicatePoint2 a = loop.vertices[index].uv;
        const PredicatePoint2 b =
            loop.vertices[(index + 1) % loop.vertices.size()].uv;
        const PredicateResult<ExactSign> side = predicates.orient2d(a, b, point);
        if (!side) {
            return {std::nullopt, side.failure};
        }
        if (*side.value == ExactSign::Zero && onBoundingBox(point, a, b)) {
            return {PointContainment::Boundary, std::nullopt};
        }
        if (a[1] <= point[1]) {
            if (b[1] > point[1] && *side.value == ExactSign::Positive) {
                ++winding;
            }
        } else if (b[1] <= point[1] &&
                   *side.value == ExactSign::Negative) {
            --winding;
        }
    }
    return {winding == 0 ? PointContainment::Outside
                         : PointContainment::Inside,
            std::nullopt};
}

PredicateResult<PlanarTrimLoopOrientation> loopOrientation(
    const PlanarTrimLoop& loop, const GeometricPredicates& predicates) {
    const auto extreme = std::min_element(
        loop.vertices.begin(), loop.vertices.end(),
        [](const PlanarTrimVertex& left, const PlanarTrimVertex& right) {
            if (left.uv[0] != right.uv[0]) return left.uv[0] < right.uv[0];
            return left.uv[1] < right.uv[1];
        });
    const std::size_t index =
        static_cast<std::size_t>(extreme - loop.vertices.begin());
    const PredicatePoint2 previous =
        loop.vertices[(index + loop.vertices.size() - 1) %
                      loop.vertices.size()]
            .uv;
    const PredicatePoint2 current = loop.vertices[index].uv;
    const PredicatePoint2 next =
        loop.vertices[(index + 1) % loop.vertices.size()].uv;
    const PredicateResult<ExactSign> turn =
        predicates.orient2d(previous, current, next);
    if (!turn) {
        return {std::nullopt, turn.failure};
    }
    if (*turn.value == ExactSign::Zero) {
        PredicateResult<PlanarTrimLoopOrientation> failure;
        failure.failure = PredicateFailure{
            "trim.loop.degenerate_orientation",
            "the exact turn at the lexicographically minimal vertex is zero"};
        return failure;
    }
    PredicateResult<PlanarTrimLoopOrientation> result;
    result.value = *turn.value == ExactSign::Positive
        ? PlanarTrimLoopOrientation::CounterClockwise
        : PlanarTrimLoopOrientation::Clockwise;
    return result;
}

bool adjacentEdges(std::size_t first, std::size_t second,
                   std::size_t edgeCount) {
    return (first + 1) % edgeCount == second ||
        (second + 1) % edgeCount == first;
}

}  // namespace

PlanarTrimValidationResult validatePlanarTrimDomain(
    const PlanarTrimDomain& domain,
    std::shared_ptr<const GeometricPredicates> predicates) {
    PlanarTrimValidationResult result;
    result.evidence = {
        {"trim.domain_structure", 1},
        {"trim.loop_structure", domain.loops.size()},
        {"trim.loop_edge_relations", 0},
        {"trim.inter_loop_relations", 0},
        {"trim.containment", domain.loops.size() *
                                 (domain.loops.empty()
                                      ? 0
                                      : domain.loops.size() - 1)},
        {"trim.role_and_nesting", domain.loops.size()},
        {"trim.orientation", domain.loops.size()},
    };

    for (const PlanarTrimLoop& loop : domain.loops) {
        result.evidence[LoopEdgeRelations].expected +=
            edgePairCount(loop.vertices.size());
    }
    for (std::size_t first = 0; first < domain.loops.size(); ++first) {
        for (std::size_t second = first + 1; second < domain.loops.size();
             ++second) {
            result.evidence[InterLoopRelations].expected +=
                domain.loops[first].vertices.size() *
                domain.loops[second].vertices.size();
        }
    }

    TrimValidationEvidence& domainEvidence = result.evidence[DomainStructure];
    ++domainEvidence.checked;
    bool domainValid = true;
    if (domain.face.kind != StableIdKind::Face || !domain.face.valid()) {
        domainValid = false;
        ++domainEvidence.failed;
        addDiagnostic(result, "trim.domain.invalid_face",
                      "the planar trim domain requires a valid working face ID",
                      {domain.face});
    }
    if (domain.sourceFace &&
        (domain.sourceFace->kind != StableIdKind::Face ||
         !domain.sourceFace->valid())) {
        domainValid = false;
        ++domainEvidence.failed;
        addDiagnostic(result, "trim.domain.invalid_source_face",
                      "the optional source face ID is not valid",
                      {domain.face, *domain.sourceFace});
    }
    if (domain.loops.empty()) {
        domainValid = false;
        ++domainEvidence.failed;
        addDiagnostic(result, "trim.domain.empty",
                      "a planar face cannot be validated without trim loops",
                      {domain.face});
    }
    if (!predicates) {
        domainValid = false;
        ++domainEvidence.failed;
        addDiagnostic(result, "trim.predicate_backend_missing",
                      "exact trim validation requires a predicate backend",
                      {domain.face});
    } else if (!predicates->exactForFiniteDoubleInputs()) {
        domainValid = false;
        ++domainEvidence.failed;
        addDiagnostic(
            result, "trim.predicate_backend_not_exact",
            "the predicate backend does not certify exact finite-double signs",
            {domain.face});
    }

    const std::size_t loopCount = domain.loops.size();
    std::vector<bool> structurallyValid(loopCount, true);
    for (std::size_t loopIndex = 0; loopIndex < loopCount; ++loopIndex) {
        const PlanarTrimLoop& loop = domain.loops[loopIndex];
        bool valid = true;
        if (loop.wire.kind != StableIdKind::Wire || !loop.wire.valid()) {
            valid = false;
            addDiagnostic(result, "trim.loop.invalid_wire",
                          "a trim loop requires a valid wire ID",
                          {domain.face, loop.wire});
        }
        if (!loop.closed) {
            valid = false;
            addDiagnostic(result, "trim.loop.open",
                          "the topology assembler did not certify a closed wire",
                          {domain.face, loop.wire});
        }
        if (loop.vertices.size() < 3) {
            valid = false;
            addDiagnostic(result, "trim.loop.too_few_vertices",
                          "a trim loop requires at least three vertices",
                          {domain.face, loop.wire});
        }

        for (std::size_t vertex = 0; vertex < loop.vertices.size(); ++vertex) {
            const PlanarTrimVertex& item = loop.vertices[vertex];
            bool provenanceValid = !item.boundaryUses.empty() &&
                item.canonicalVertexIndex != InvalidCanonicalVertexIndex;
            for (std::size_t useIndex = 0;
                 useIndex < item.boundaryUses.size(); ++useIndex) {
                const PlanarTrimBoundaryUse& use =
                    item.boundaryUses[useIndex];
                provenanceValid = provenanceValid && use.sample.valid() &&
                    use.workingEdge.kind == StableIdKind::Edge &&
                    use.workingEdge.valid() &&
                    use.sample.boundary.ordinal == use.workingEdge.ordinal &&
                    (!use.sourceEdge ||
                     (use.sourceEdge->kind == StableIdKind::Edge &&
                      use.sourceEdge->valid())) &&
                    finite(use.uv) && equal(use.uv, item.uv);
                for (std::size_t previousUse = 0;
                     previousUse < useIndex; ++previousUse) {
                    provenanceValid = provenanceValid &&
                        !(use.sample ==
                          item.boundaryUses[previousUse].sample);
                }
                for (std::size_t previousVertex = 0;
                     previousVertex < vertex; ++previousVertex) {
                    for (const PlanarTrimBoundaryUse& previousUse :
                         loop.vertices[previousVertex].boundaryUses) {
                        provenanceValid = provenanceValid &&
                            !(use.sample == previousUse.sample);
                    }
                }
            }
            if (!provenanceValid) {
                valid = false;
                addDiagnostic(result, "trim.loop.invalid_provenance",
                              "every trim vertex requires consistent canonical boundary-use provenance",
                              {domain.face, loop.wire});
            }
            if (!finite(item.uv)) {
                valid = false;
                addDiagnostic(result, "trim.loop.non_finite_uv",
                              "trim UV coordinates must be finite",
                              {domain.face, loop.wire});
            }
            for (std::size_t previous = 0; previous < vertex; ++previous) {
                const PlanarTrimVertex& other = loop.vertices[previous];
                if (item.canonicalVertexIndex == other.canonicalVertexIndex) {
                    valid = false;
                    addDiagnostic(
                        result, "trim.loop.repeated_canonical_vertex",
                        "a canonical vertex occurs more than once in one loop",
                        {domain.face, loop.wire});
                }
                if (equal(item.uv, other.uv)) {
                    valid = false;
                    addDiagnostic(result, "trim.loop.repeated_uv_vertex",
                                  "two canonical vertices have identical UV coordinates",
                                  {domain.face, loop.wire});
                }
            }
        }

        structurallyValid[loopIndex] = valid;
        TrimValidationEvidence& evidence = result.evidence[LoopStructure];
        ++evidence.checked;
        if (!valid) {
            ++evidence.failed;
            domainValid = false;
        }
    }

    std::vector<bool> simple(loopCount, false);
    TrimValidationEvidence& edgeEvidence =
        result.evidence[LoopEdgeRelations];
    if (!predicates || !predicates->exactForFiniteDoubleInputs()) {
        edgeEvidence.skipped = edgeEvidence.expected;
    } else {
        for (std::size_t loopIndex = 0; loopIndex < loopCount; ++loopIndex) {
            const PlanarTrimLoop& loop = domain.loops[loopIndex];
            const std::size_t pairCount = edgePairCount(loop.vertices.size());
            if (!structurallyValid[loopIndex]) {
                edgeEvidence.skipped += pairCount;
                continue;
            }
            bool loopSimple = true;
            const std::size_t edgeCount = loop.vertices.size();
            for (std::size_t first = 0; first < edgeCount; ++first) {
                const PredicatePoint2 a = loop.vertices[first].uv;
                const PredicatePoint2 b =
                    loop.vertices[(first + 1) % edgeCount].uv;
                for (std::size_t second = first + 1; second < edgeCount;
                     ++second) {
                    const PredicatePoint2 c = loop.vertices[second].uv;
                    const PredicatePoint2 d =
                        loop.vertices[(second + 1) % edgeCount].uv;
                    const PredicateResult<SegmentIntersectionKind> relation =
                        predicates->segmentIntersection(a, b, c, d);
                    ++edgeEvidence.checked;
                    if (!relation) {
                        loopSimple = false;
                        ++edgeEvidence.failed;
                        addDiagnostic(
                            result, "trim.predicate_failure",
                            relation.failure
                                ? relation.failure->message
                                : "an exact segment predicate failed without detail",
                            {domain.face, loop.wire});
                        continue;
                    }
                    const bool adjacent =
                        adjacentEdges(first, second, edgeCount);
                    const SegmentIntersectionKind expected = adjacent
                        ? SegmentIntersectionKind::EndpointTouch
                        : SegmentIntersectionKind::None;
                    if (*relation.value != expected) {
                        loopSimple = false;
                        ++edgeEvidence.failed;
                        addDiagnostic(
                            result,
                            adjacent ? "trim.loop.adjacent_edge_overlap"
                                     : "trim.loop.self_intersection",
                            std::string("unexpected ") +
                                intersectionName(*relation.value) +
                                " between loop edges",
                            {domain.face, loop.wire});
                    }
                }
            }
            simple[loopIndex] = loopSimple;
            if (!loopSimple) domainValid = false;
        }
    }

    std::vector<std::vector<bool>> pairBoundariesDisjoint(
        loopCount, std::vector<bool>(loopCount, false));
    TrimValidationEvidence& interEvidence =
        result.evidence[InterLoopRelations];
    if (!predicates || !predicates->exactForFiniteDoubleInputs()) {
        interEvidence.skipped = interEvidence.expected;
    } else {
        for (std::size_t first = 0; first < loopCount; ++first) {
            for (std::size_t second = first + 1; second < loopCount;
                 ++second) {
                const PlanarTrimLoop& aLoop = domain.loops[first];
                const PlanarTrimLoop& bLoop = domain.loops[second];
                const std::size_t pairCount =
                    aLoop.vertices.size() * bLoop.vertices.size();
                if (!simple[first] || !simple[second]) {
                    interEvidence.skipped += pairCount;
                    continue;
                }
                bool disjoint = true;
                for (std::size_t aIndex = 0;
                     aIndex < aLoop.vertices.size(); ++aIndex) {
                    const PredicatePoint2 a = aLoop.vertices[aIndex].uv;
                    const PredicatePoint2 b =
                        aLoop.vertices[(aIndex + 1) % aLoop.vertices.size()].uv;
                    for (std::size_t bIndex = 0;
                         bIndex < bLoop.vertices.size(); ++bIndex) {
                        const PredicatePoint2 c = bLoop.vertices[bIndex].uv;
                        const PredicatePoint2 d = bLoop
                            .vertices[(bIndex + 1) % bLoop.vertices.size()]
                            .uv;
                        const PredicateResult<SegmentIntersectionKind> relation =
                            predicates->segmentIntersection(a, b, c, d);
                        ++interEvidence.checked;
                        if (!relation ||
                            *relation.value != SegmentIntersectionKind::None) {
                            disjoint = false;
                            ++interEvidence.failed;
                            addDiagnostic(
                                result,
                                relation ? "trim.loops.boundary_intersection"
                                         : "trim.predicate_failure",
                                relation
                                    ? std::string("loop boundaries have a ") +
                                          intersectionName(*relation.value)
                                    : (relation.failure
                                           ? relation.failure->message
                                           : "an exact segment predicate failed without detail"),
                                {domain.face, aLoop.wire, bLoop.wire});
                        }
                    }
                }
                pairBoundariesDisjoint[first][second] = disjoint;
                pairBoundariesDisjoint[second][first] = disjoint;
                if (!disjoint) domainValid = false;
            }
        }
    }

    std::vector<std::vector<PointContainment>> containment(
        loopCount, std::vector<PointContainment>(loopCount,
                                                 PointContainment::Outside));
    std::vector<bool> containmentComplete(loopCount, true);
    TrimValidationEvidence& containmentEvidence = result.evidence[Containment];
    if (!predicates || !predicates->exactForFiniteDoubleInputs()) {
        containmentEvidence.skipped = containmentEvidence.expected;
        std::fill(containmentComplete.begin(), containmentComplete.end(), false);
    } else {
        for (std::size_t inner = 0; inner < loopCount; ++inner) {
            for (std::size_t container = 0; container < loopCount;
                 ++container) {
                if (inner == container) continue;
                if (!simple[inner] || !simple[container] ||
                    !pairBoundariesDisjoint[inner][container]) {
                    ++containmentEvidence.skipped;
                    containmentComplete[inner] = false;
                    continue;
                }
                const PointContainmentResult relation = pointInLoop(
                    domain.loops[inner].vertices.front().uv,
                    domain.loops[container], *predicates);
                ++containmentEvidence.checked;
                if (!relation.value) {
                    ++containmentEvidence.failed;
                    containmentComplete[inner] = false;
                    domainValid = false;
                    addDiagnostic(
                        result, "trim.predicate_failure",
                        relation.failure
                            ? relation.failure->message
                            : "an exact containment predicate failed without detail",
                        {domain.face, domain.loops[inner].wire,
                         domain.loops[container].wire});
                    continue;
                }
                containment[inner][container] = *relation.value;
                if (*relation.value == PointContainment::Boundary) {
                    ++containmentEvidence.failed;
                    containmentComplete[inner] = false;
                    domainValid = false;
                    addDiagnostic(
                        result, "trim.loops.boundary_touch",
                        "a loop representative lies on another loop boundary",
                        {domain.face, domain.loops[inner].wire,
                         domain.loops[container].wire});
                }
            }
        }
    }

    std::vector<std::size_t> nestingDepth(loopCount, 0);
    std::vector<PlanarTrimLoopRole> inferredRole(
        loopCount, PlanarTrimLoopRole::Outer);
    std::vector<bool> roleValid(loopCount, false);
    std::size_t inferredOuterCount = 0;
    TrimValidationEvidence& roleEvidence = result.evidence[RoleAndNesting];
    for (std::size_t loopIndex = 0; loopIndex < loopCount; ++loopIndex) {
        if (!simple[loopIndex] || !containmentComplete[loopIndex]) {
            ++roleEvidence.skipped;
            continue;
        }
        ++roleEvidence.checked;
        const std::size_t depth = static_cast<std::size_t>(std::count(
            containment[loopIndex].begin(), containment[loopIndex].end(),
            PointContainment::Inside));
        nestingDepth[loopIndex] = depth;
        if (depth > 1) {
            ++roleEvidence.failed;
            domainValid = false;
            addDiagnostic(result, "trim.loop.nested_hole_unsupported",
                          "a planar face loop is nested more than one level",
                          {domain.face, domain.loops[loopIndex].wire});
            continue;
        }
        inferredRole[loopIndex] = depth == 0 ? PlanarTrimLoopRole::Outer
                                             : PlanarTrimLoopRole::Hole;
        if (inferredRole[loopIndex] == PlanarTrimLoopRole::Outer) {
            ++inferredOuterCount;
        }
        if (inferredRole[loopIndex] !=
            domain.loops[loopIndex].declaredRole) {
            ++roleEvidence.failed;
            domainValid = false;
            addDiagnostic(result, "trim.loop.role_mismatch",
                          "declared outer/hole role disagrees with exact containment",
                          {domain.face, domain.loops[loopIndex].wire});
            continue;
        }
        roleValid[loopIndex] = true;
    }
    if (loopCount != 0 && inferredOuterCount != 1 &&
        roleEvidence.skipped == 0) {
        ++roleEvidence.failed;
        domainValid = false;
        addDiagnostic(result, "trim.domain.outer_count",
                      "a planar face requires exactly one inferred outer loop",
                      {domain.face});
    }

    std::vector<PlanarTrimLoopOrientation> orientations(
        loopCount, PlanarTrimLoopOrientation::CounterClockwise);
    TrimValidationEvidence& orientationEvidence = result.evidence[Orientation];
    if (!predicates || !predicates->exactForFiniteDoubleInputs()) {
        orientationEvidence.skipped = orientationEvidence.expected;
    } else {
        for (std::size_t loopIndex = 0; loopIndex < loopCount; ++loopIndex) {
            if (!simple[loopIndex] || !roleValid[loopIndex]) {
                ++orientationEvidence.skipped;
                continue;
            }
            ++orientationEvidence.checked;
            const PredicateResult<PlanarTrimLoopOrientation> orientation =
                loopOrientation(domain.loops[loopIndex], *predicates);
            if (!orientation) {
                ++orientationEvidence.failed;
                domainValid = false;
                addDiagnostic(
                    result,
                    orientation.failure
                        ? orientation.failure->code
                        : "trim.predicate_failure",
                    orientation.failure
                        ? orientation.failure->message
                        : "exact loop orientation failed without detail",
                    {domain.face, domain.loops[loopIndex].wire});
                continue;
            }
            orientations[loopIndex] = *orientation.value;
            const PlanarTrimLoopOrientation expected =
                inferredRole[loopIndex] == PlanarTrimLoopRole::Outer
                ? PlanarTrimLoopOrientation::CounterClockwise
                : PlanarTrimLoopOrientation::Clockwise;
            if (*orientation.value != expected) {
                ++orientationEvidence.failed;
                domainValid = false;
                addDiagnostic(
                    result, "trim.loop.orientation_mismatch",
                    inferredRole[loopIndex] == PlanarTrimLoopRole::Outer
                        ? "the outer loop must be counter-clockwise in lifted UV"
                        : "a hole loop must be clockwise in lifted UV",
                    {domain.face, domain.loops[loopIndex].wire});
            }
        }
    }

    if (!domainValid || !result.diagnostics.empty()) return result;

    ValidatedPlanarTrimDomain validated;
    validated.face = domain.face;
    validated.sourceFace = domain.sourceFace;
    validated.loops.reserve(loopCount);
    for (std::size_t loopIndex = 0; loopIndex < loopCount; ++loopIndex) {
        validated.loops.push_back(
            {domain.loops[loopIndex].wire, inferredRole[loopIndex],
             orientations[loopIndex], nestingDepth[loopIndex],
             domain.loops[loopIndex].vertices});
    }
    result.value = std::move(validated);
    return result;
}

}  // namespace weft
