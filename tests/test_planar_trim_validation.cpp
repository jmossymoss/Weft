#include "weft/planar_trim_validation.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition)                                                  \
    do {                                                                  \
        if (!(condition)) {                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__,          \
                        #condition);                                      \
            ++failures;                                                   \
        }                                                                 \
    } while (false)

weft::StableId id(weft::StableIdKind kind, std::uint64_t ordinal) {
    return {kind, ordinal};
}

weft::PlanarTrimLoop loop(
    std::uint64_t ordinal, weft::PlanarTrimLoopRole role,
    std::initializer_list<weft::PredicatePoint2> points, bool closed = true) {
    weft::PlanarTrimLoop result;
    result.wire = id(weft::StableIdKind::Wire, ordinal);
    result.declaredRole = role;
    result.closed = closed;
    std::uint32_t sampleOrdinal = 0;
    for (weft::PredicatePoint2 point : points) {
        ++sampleOrdinal;
        const std::uint64_t workingEdgeOrdinal =
            ordinal * 100 + sampleOrdinal;
        weft::PlanarTrimBoundaryUse use;
        use.sample = {{weft::StableIdKind::Boundary, workingEdgeOrdinal},
                      sampleOrdinal};
        use.workingEdge = {weft::StableIdKind::Edge, workingEdgeOrdinal};
        use.sourceEdge = weft::StableId{
            weft::StableIdKind::Edge, ordinal * 1000 + sampleOrdinal};
        use.coedge = {weft::StableIdKind::Coedge, workingEdgeOrdinal};
        use.uv = point;
        use.measuredCurveOnSurfaceDiscrepancy = 0.0;
        use.allowedCurveOnSurfaceDiscrepancy = 1e-7;
        use.representation = std::nullopt;

        weft::PlanarTrimVertex vertex;
        vertex.boundaryUses.push_back(std::move(use));
        vertex.canonicalVertexIndex = workingEdgeOrdinal;
        vertex.uv = point;
        result.vertices.push_back(std::move(vertex));
    }
    return result;
}

weft::PlanarTrimDomain domain(std::vector<weft::PlanarTrimLoop> loops) {
    return {{weft::StableIdKind::Face, 1},
            weft::StableId{weft::StableIdKind::Face, 101},
            std::move(loops)};
}

bool hasDiagnostic(const weft::PlanarTrimValidationResult& result,
                   const std::string& code) {
    return std::any_of(
        result.diagnostics.begin(), result.diagnostics.end(),
        [&](const weft::TrimValidationDiagnostic& diagnostic) {
            return diagnostic.code == code;
        });
}

const weft::TrimValidationEvidence* evidence(
    const weft::PlanarTrimValidationResult& result, const std::string& code) {
    const auto found = std::find_if(
        result.evidence.begin(), result.evidence.end(),
        [&](const weft::TrimValidationEvidence& item) {
            return item.code == code;
        });
    return found == result.evidence.end() ? nullptr : &*found;
}

void checkCompleteEvidence(const weft::PlanarTrimValidationResult& result) {
    CHECK(result.evidence.size() == 7);
    for (const weft::TrimValidationEvidence& item : result.evidence) {
        CHECK(item.complete());
        CHECK(item.failed == 0);
        CHECK(item.skipped == 0);
        if (item.expected != 0) CHECK(item.checked != 0);
    }
}

void testValidOuterAndHole() {
    auto validDomain = domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}}),
        loop(2, weft::PlanarTrimLoopRole::Hole,
             {{1.0, 1.0}, {1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}}),
    });
    // CanonicalBoundarySet indices are zero-based; zero is real provenance,
    // not an absent-value sentinel.
    validDomain.loops.front().vertices.front().canonicalVertexIndex = 0;
    const auto result = weft::validatePlanarTrimDomain(validDomain);
    CHECK(result);
    CHECK(result.diagnostics.empty());
    CHECK(result.value && result.value->loops.size() == 2);
    if (result.value && result.value->loops.size() == 2) {
        CHECK(result.value->loops[0].role ==
              weft::PlanarTrimLoopRole::Outer);
        CHECK(result.value->loops[0].orientation ==
              weft::PlanarTrimLoopOrientation::CounterClockwise);
        CHECK(result.value->loops[0].nestingDepth == 0);
        CHECK(result.value->loops[1].role ==
              weft::PlanarTrimLoopRole::Hole);
        CHECK(result.value->loops[1].orientation ==
              weft::PlanarTrimLoopOrientation::Clockwise);
        CHECK(result.value->loops[1].nestingDepth == 1);
    }
    checkCompleteEvidence(result);
    const auto* edgeRelations = evidence(result, "trim.loop_edge_relations");
    CHECK(edgeRelations && edgeRelations->expected == 12);
    const auto* inter = evidence(result, "trim.inter_loop_relations");
    CHECK(inter && inter->expected == 16);
    const auto* containment = evidence(result, "trim.containment");
    CHECK(containment && containment->expected == 2);
}

void testTriangleIsNonVacuous() {
    const auto result = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}}),
    }));
    CHECK(result);
    checkCompleteEvidence(result);
    const auto* edgeRelations = evidence(result, "trim.loop_edge_relations");
    CHECK(edgeRelations && edgeRelations->expected == 3);
    CHECK(edgeRelations && edgeRelations->checked == 3);
    const auto* orientation = evidence(result, "trim.orientation");
    CHECK(orientation && orientation->expected == 1);
    CHECK(orientation && orientation->checked == 1);
}

void testStructuralRefusals() {
    const auto empty = weft::validatePlanarTrimDomain(domain({}));
    CHECK(!empty);
    CHECK(hasDiagnostic(empty, "trim.domain.empty"));

    const auto open = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}, false),
    }));
    CHECK(!open);
    CHECK(hasDiagnostic(open, "trim.loop.open"));
    const auto* openEdges = evidence(open, "trim.loop_edge_relations");
    CHECK(openEdges && openEdges->expected == 3);
    CHECK(openEdges && openEdges->skipped == 3);

    auto missingProvenanceLoop = loop(
        1, weft::PlanarTrimLoopRole::Outer,
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}});
    missingProvenanceLoop.vertices[0].boundaryUses.clear();
    missingProvenanceLoop.vertices[0].canonicalVertexIndex =
        weft::InvalidCanonicalVertexIndex;
    const auto missingProvenance = weft::validatePlanarTrimDomain(
        domain({std::move(missingProvenanceLoop)}));
    CHECK(!missingProvenance);
    CHECK(hasDiagnostic(missingProvenance,
                        "trim.loop.invalid_provenance"));

    auto repeatedSampleLoop = loop(
        1, weft::PlanarTrimLoopRole::Outer,
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}});
    repeatedSampleLoop.vertices[1].boundaryUses.front().sample =
        repeatedSampleLoop.vertices[0].boundaryUses.front().sample;
    const auto repeatedSample = weft::validatePlanarTrimDomain(
        domain({std::move(repeatedSampleLoop)}));
    CHECK(!repeatedSample);
    CHECK(hasDiagnostic(repeatedSample, "trim.loop.invalid_provenance"));

    auto repeatedLoop = loop(
        1, weft::PlanarTrimLoopRole::Outer,
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {0.0, 0.0}});
    repeatedLoop.vertices.back().canonicalVertexIndex =
        repeatedLoop.vertices.front().canonicalVertexIndex;
    const auto repeated =
        weft::validatePlanarTrimDomain(domain({std::move(repeatedLoop)}));
    CHECK(!repeated);
    CHECK(hasDiagnostic(repeated, "trim.loop.repeated_canonical_vertex"));
    CHECK(hasDiagnostic(repeated, "trim.loop.repeated_uv_vertex"));

    auto nonFiniteLoop = loop(
        1, weft::PlanarTrimLoopRole::Outer,
        {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}});
    nonFiniteLoop.vertices[1].uv[0] =
        std::numeric_limits<double>::infinity();
    const auto nonFinite =
        weft::validatePlanarTrimDomain(domain({std::move(nonFiniteLoop)}));
    CHECK(!nonFinite);
    CHECK(hasDiagnostic(nonFinite, "trim.loop.non_finite_uv"));
}

void testSelfIntersectionRefusals() {
    const auto bowTie = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {2.0, 0.0}}),
    }));
    CHECK(!bowTie);
    CHECK(hasDiagnostic(bowTie, "trim.loop.self_intersection"));
    const auto* bowTieEdges = evidence(bowTie, "trim.loop_edge_relations");
    CHECK(bowTieEdges && bowTieEdges->checked == bowTieEdges->expected);
    CHECK(bowTieEdges && bowTieEdges->failed != 0);

    const auto overlap = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {2.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}),
    }));
    CHECK(!overlap);
    CHECK(hasDiagnostic(overlap, "trim.loop.adjacent_edge_overlap"));
}

void testInterLoopAndNestingRefusals() {
    auto repeatedCanonicalDomain = domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}}),
        loop(2, weft::PlanarTrimLoopRole::Hole,
             {{1.0, 1.0}, {1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}}),
    });
    repeatedCanonicalDomain.loops[1].vertices[0].canonicalVertexIndex =
        repeatedCanonicalDomain.loops[0].vertices[0].canonicalVertexIndex;
    const auto repeatedCanonical =
        weft::validatePlanarTrimDomain(repeatedCanonicalDomain);
    CHECK(!repeatedCanonical);
    CHECK(hasDiagnostic(repeatedCanonical,
                        "trim.domain.repeated_canonical_vertex"));

    const auto crossing = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}}),
        loop(2, weft::PlanarTrimLoopRole::Hole,
             {{3.0, 1.0}, {3.0, 3.0}, {5.0, 3.0}, {5.0, 1.0}}),
    }));
    CHECK(!crossing);
    CHECK(hasDiagnostic(crossing, "trim.loops.boundary_intersection"));

    const auto outsideHole = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}}),
        loop(2, weft::PlanarTrimLoopRole::Hole,
             {{5.0, 1.0}, {5.0, 3.0}, {7.0, 3.0}, {7.0, 1.0}}),
    }));
    CHECK(!outsideHole);
    CHECK(hasDiagnostic(outsideHole, "trim.loop.role_mismatch"));
    CHECK(hasDiagnostic(outsideHole, "trim.domain.outer_count"));

    const auto nestedHole = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}}),
        loop(2, weft::PlanarTrimLoopRole::Hole,
             {{1.0, 1.0}, {1.0, 9.0}, {9.0, 9.0}, {9.0, 1.0}}),
        loop(3, weft::PlanarTrimLoopRole::Hole,
             {{2.0, 2.0}, {2.0, 3.0}, {3.0, 3.0}, {3.0, 2.0}}),
    }));
    CHECK(!nestedHole);
    CHECK(hasDiagnostic(nestedHole, "trim.loop.nested_hole_unsupported"));
}

void testOrientationAndBackendRefusals() {
    const auto clockwiseOuter = weft::validatePlanarTrimDomain(domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}, {2.0, 0.0}}),
    }));
    CHECK(!clockwiseOuter);
    CHECK(hasDiagnostic(clockwiseOuter, "trim.loop.orientation_mismatch"));

    const auto missingBackend = weft::validatePlanarTrimDomain(
        domain({loop(1, weft::PlanarTrimLoopRole::Outer,
                     {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}})}),
        nullptr);
    CHECK(!missingBackend);
    CHECK(hasDiagnostic(missingBackend, "trim.predicate_backend_missing"));
    const auto* relations =
        evidence(missingBackend, "trim.loop_edge_relations");
    CHECK(relations && relations->skipped == relations->expected);
}

void testExactSubnormalDomainAndDeterminism() {
    volatile double tinyInput = std::numeric_limits<double>::denorm_min();
    const double tiny = tinyInput;
    const auto tinyDomain = domain({
        loop(1, weft::PlanarTrimLoopRole::Outer,
             {{0.0, 0.0}, {tiny, 0.0}, {tiny, tiny}, {0.0, tiny}}),
    });
    const auto first = weft::validatePlanarTrimDomain(tinyDomain);
    const auto second = weft::validatePlanarTrimDomain(tinyDomain);
    CHECK(first);
    CHECK(second);
    CHECK(first.evidence.size() == second.evidence.size());
    CHECK(first.diagnostics.size() == second.diagnostics.size());
    for (std::size_t index = 0; index < first.evidence.size() &&
                                index < second.evidence.size();
         ++index) {
        CHECK(first.evidence[index].code == second.evidence[index].code);
        CHECK(first.evidence[index].expected ==
              second.evidence[index].expected);
        CHECK(first.evidence[index].checked ==
              second.evidence[index].checked);
        CHECK(first.evidence[index].skipped ==
              second.evidence[index].skipped);
        CHECK(first.evidence[index].failed == second.evidence[index].failed);
    }
}

}  // namespace

int main() {
    testValidOuterAndHole();
    testTriangleIsNonVacuous();
    testStructuralRefusals();
    testSelfIntersectionRefusals();
    testInterLoopAndNestingRefusals();
    testOrientationAndBackendRefusals();
    testExactSubnormalDomainAndDeterminism();
    if (failures == 0) {
        std::printf("planar trim validation checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d planar trim validation failure(s)\n", failures);
    return EXIT_FAILURE;
}
