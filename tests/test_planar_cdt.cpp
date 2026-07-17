#include "weft/planar_cdt.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <set>
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

weft::PlanarTrimLoop makeLoop(
    std::uint64_t loopOrdinal, weft::PlanarTrimLoopRole role,
    const std::vector<weft::PredicatePoint2>& points) {
    weft::PlanarTrimLoop loop;
    loop.wire = {weft::StableIdKind::Wire, loopOrdinal};
    loop.declaredRole = role;
    loop.closed = true;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const std::uint64_t edgeOrdinal =
            loopOrdinal * 1000 + static_cast<std::uint64_t>(index) + 1;
        loop.vertices.push_back(
            {{{weft::StableIdKind::Boundary, edgeOrdinal},
              static_cast<std::uint32_t>(index)},
             {weft::StableIdKind::Edge, edgeOrdinal},
             weft::StableId{weft::StableIdKind::Edge, edgeOrdinal + 100000},
             static_cast<std::uint64_t>(index), points[index]});
    }
    return loop;
}

weft::PlanarTrimDomain makeDomain(
    const std::vector<weft::PredicatePoint2>& outer) {
    weft::PlanarTrimDomain domain;
    domain.face = {weft::StableIdKind::Face, 1};
    domain.sourceFace = weft::StableId{weft::StableIdKind::Face, 101};
    domain.loops.push_back(
        makeLoop(1, weft::PlanarTrimLoopRole::Outer, outer));
    return domain;
}

const weft::PlanarCdtValidationEvidence* evidence(
    const weft::PlanarCdtResult& result, const std::string& code) {
    const auto found = std::find_if(
        result.validation.begin(), result.validation.end(),
        [&](const weft::PlanarCdtValidationEvidence& item) {
            return item.code == code;
        });
    return found == result.validation.end() ? nullptr : &*found;
}

void checkCertified(const weft::PlanarCdtResult& result,
                    std::size_t expectedVertices) {
    CHECK(result);
    CHECK(!result.failure);
    CHECK(result.trimValidation);
    CHECK(result.value && result.value->vertices.size() == expectedVertices);
    CHECK(result.value &&
          result.value->triangles.size() == expectedVertices - 2);
    CHECK(result.value &&
          result.value->constrainedEdges.size() == expectedVertices);
    CHECK(result.validation.size() == 7);
    for (const weft::PlanarCdtValidationEvidence& item : result.validation) {
        CHECK(item.complete());
        CHECK(item.failed == 0);
        CHECK(item.skipped == 0);
        if (item.expected != 0) CHECK(item.checked != 0);
    }
    if (!result.value) return;
    for (std::size_t index = 0; index < result.value->vertices.size(); ++index) {
        const weft::PlanarTrimVertex& vertex = result.value->vertices[index];
        CHECK(vertex.canonicalVertexIndex == index);
        CHECK(vertex.sample.boundary.ordinal == vertex.workingEdge.ordinal);
    }
    for (const weft::PlanarCdtTriangle& triangle :
         result.value->triangles) {
        CHECK(triangle.workingFace == result.value->workingFace);
        CHECK(triangle.sourceFace == result.value->sourceFace);
    }
}

std::set<std::pair<std::uint32_t, std::uint32_t>> meshEdges(
    const weft::PlanarCdtMesh& mesh) {
    std::set<std::pair<std::uint32_t, std::uint32_t>> edges;
    for (const weft::PlanarCdtTriangle& triangle : mesh.triangles) {
        for (std::size_t local = 0; local < 3; ++local) {
            const std::uint32_t first = triangle.vertices[local];
            const std::uint32_t second = triangle.vertices[(local + 1) % 3];
            edges.emplace(std::min(first, second), std::max(first, second));
        }
    }
    return edges;
}

void testBackendContract(const weft::PlanarCdtBackend& backend) {
    CHECK(std::string(backend.backendCode()) ==
          "exact_lawson_single_loop_reference");
    CHECK(backend.exactPredicatesForFiniteDoubleInputs());
    CHECK(!backend.supportsHoles());
}

void testTriangleAndCocircularSquare(const weft::PlanarCdtBackend& backend) {
    const auto triangle = backend.triangulate(
        makeDomain({{0.0, 0.0}, {2.0, 0.0}, {0.0, 1.0}}));
    checkCertified(triangle, 3);
    const auto* triangleEdges =
        evidence(triangle, "cdt.mesh_edge_intersection");
    CHECK(triangleEdges && triangleEdges->expected == 3);

    const auto squareDomain =
        makeDomain({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}});
    const auto first = backend.triangulate(squareDomain);
    const auto second = backend.triangulate(squareDomain);
    checkCertified(first, 4);
    checkCertified(second, 4);
    CHECK(first.value && second.value &&
          first.value->triangles.size() == second.value->triangles.size());
    if (first.value && second.value) {
        for (std::size_t index = 0; index < first.value->triangles.size();
             ++index) {
            CHECK(first.value->triangles[index].vertices ==
                  second.value->triangles[index].vertices);
        }
        const auto edges = meshEdges(*first.value);
        CHECK(edges.contains({1, 3}));
        CHECK(!edges.contains({0, 2}));
    }
}

void testExactLawsonFlip(const weft::PlanarCdtBackend& backend) {
    const auto result = backend.triangulate(
        makeDomain({{0.0, 0.0}, {3.0, 0.0}, {2.0, 1.0}, {0.0, 2.0}}));
    checkCertified(result, 4);
    if (!result.value) return;
    const auto edges = meshEdges(*result.value);
    CHECK(edges.contains({0, 2}));
    CHECK(!edges.contains({1, 3}));
    const auto* delaunay = evidence(result, "cdt.local_delaunay");
    CHECK(delaunay && delaunay->expected == 1);
    CHECK(delaunay && delaunay->checked == 1);
}

void testConcaveAndCollinearDomains(const weft::PlanarCdtBackend& backend) {
    const auto concave = backend.triangulate(makeDomain(
        {{0.0, 0.0}, {3.0, 0.0}, {3.0, 1.0}, {1.5, 0.5}, {0.0, 1.0}}));
    checkCertified(concave, 5);

    for (std::uint32_t divisions = 1; divisions <= 12; ++divisions) {
        std::vector<weft::PredicatePoint2> points;
        const double extent = static_cast<double>(divisions);
        for (std::uint32_t index = 0; index < divisions; ++index) {
            points.push_back({static_cast<double>(index), 0.0});
        }
        for (std::uint32_t index = 0; index < divisions; ++index) {
            points.push_back({extent, static_cast<double>(index)});
        }
        for (std::uint32_t index = 0; index < divisions; ++index) {
            points.push_back(
                {extent - static_cast<double>(index), extent});
        }
        for (std::uint32_t index = 0; index < divisions; ++index) {
            points.push_back(
                {0.0, extent - static_cast<double>(index)});
        }
        const auto result = backend.triangulate(makeDomain(points));
        checkCertified(result, points.size());
    }
}

void testStarPropertyBattery(const weft::PlanarCdtBackend& backend) {
    constexpr double twoPi = 6.283185307179586476925286766559;
    for (std::uint32_t caseIndex = 0; caseIndex < 64; ++caseIndex) {
        const std::uint32_t count = 5 + caseIndex % 16;
        std::vector<weft::PredicatePoint2> points;
        points.reserve(count);
        for (std::uint32_t index = 0; index < count; ++index) {
            const double angle = twoPi * static_cast<double>(index) /
                static_cast<double>(count);
            const double radius = 5.0 +
                static_cast<double>((index * 17 + caseIndex * 13) % 6);
            points.push_back({radius * std::cos(angle),
                              radius * std::sin(angle)});
        }
        const auto source = makeDomain(points);
        const auto first = backend.triangulate(source);
        const auto second = backend.triangulate(source);
        checkCertified(first, points.size());
        checkCertified(second, points.size());
        if (!first.value || !second.value) continue;
        CHECK(first.value->triangles.size() == second.value->triangles.size());
        for (std::size_t index = 0;
             index < first.value->triangles.size() &&
             index < second.value->triangles.size();
             ++index) {
            CHECK(first.value->triangles[index].vertices ==
                  second.value->triangles[index].vertices);
        }
    }
}

void testSubnormalDomain(const weft::PlanarCdtBackend& backend) {
    volatile double tinyInput = std::numeric_limits<double>::denorm_min();
    const double tiny = tinyInput;
    const auto result = backend.triangulate(
        makeDomain({{0.0, 0.0}, {tiny, 0.0}, {tiny, tiny}, {0.0, tiny}}));
    checkCertified(result, 4);
}

void testNamedRefusals(const weft::PlanarCdtBackend& backend) {
    const auto invalid = backend.triangulate(
        makeDomain({{0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {2.0, 0.0}}));
    CHECK(!invalid);
    CHECK(invalid.failure &&
          invalid.failure->code == "cdt.invalid_trim_domain");
    CHECK(!invalid.trimValidation);

    weft::PlanarTrimDomain perforated = makeDomain(
        {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}});
    perforated.loops.push_back(makeLoop(
        2, weft::PlanarTrimLoopRole::Hole,
        {{1.0, 1.0}, {1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}}));
    const auto hole = backend.triangulate(perforated);
    CHECK(!hole);
    CHECK(hole.trimValidation);
    CHECK(hole.failure &&
          hole.failure->code == "cdt.holes_not_supported_by_reference");

    const auto unavailable =
        weft::makeExactLawsonReferencePlanarCdtBackend(nullptr);
    CHECK(unavailable != nullptr);
    if (unavailable) {
        CHECK(!unavailable->exactPredicatesForFiniteDoubleInputs());
        const auto refused = unavailable->triangulate(
            makeDomain({{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}));
        CHECK(!refused);
        CHECK(refused.failure &&
              refused.failure->code == "cdt.invalid_trim_domain");
    }
}

}  // namespace

int main() {
    const auto backend = weft::makeExactLawsonReferencePlanarCdtBackend();
    CHECK(backend != nullptr);
    if (backend) {
        testBackendContract(*backend);
        testTriangleAndCocircularSquare(*backend);
        testExactLawsonFlip(*backend);
        testConcaveAndCollinearDomains(*backend);
        testStarPropertyBattery(*backend);
        testSubnormalDomain(*backend);
        testNamedRefusals(*backend);
    }
    if (failures == 0) {
        std::printf("exact planar CDT reference checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d planar CDT failure(s)\n", failures);
    return EXIT_FAILURE;
}
