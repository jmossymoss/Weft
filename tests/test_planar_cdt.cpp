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
        weft::PlanarTrimBoundaryUse use;
        use.sample = {{weft::StableIdKind::Boundary, edgeOrdinal},
                      static_cast<std::uint32_t>(index)};
        use.workingEdge = {weft::StableIdKind::Edge, edgeOrdinal};
        use.sourceEdge = weft::StableId{
            weft::StableIdKind::Edge, edgeOrdinal + 100000};
        use.coedge = {weft::StableIdKind::Coedge, edgeOrdinal};
        use.uv = points[index];
        use.measuredCurveOnSurfaceDiscrepancy = 0.0;
        use.allowedCurveOnSurfaceDiscrepancy = 1e-7;
        use.representation = std::nullopt;

        weft::PlanarTrimVertex vertex;
        vertex.boundaryUses.push_back(std::move(use));
        vertex.canonicalVertexIndex =
            (loopOrdinal - 1) * 100 + static_cast<std::uint64_t>(index);
        vertex.uv = points[index];
        loop.vertices.push_back(std::move(vertex));
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
                    std::size_t expectedVertices,
                    std::size_t expectedHoles = 0) {
    CHECK(result);
    CHECK(!result.failure);
    CHECK(result.trimValidation);
    CHECK(result.value && result.value->vertices.size() == expectedVertices);
    CHECK(result.value &&
          result.value->triangles.size() ==
              expectedVertices + 2 * expectedHoles - 2);
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
    std::set<std::uint64_t> canonicalIndices;
    for (std::size_t index = 0; index < result.value->vertices.size(); ++index) {
        const weft::PlanarTrimVertex& vertex = result.value->vertices[index];
        CHECK(canonicalIndices.insert(vertex.canonicalVertexIndex).second);
        CHECK(!vertex.boundaryUses.empty());
        if (!vertex.boundaryUses.empty()) {
            CHECK(vertex.boundaryUses.front().sample.boundary.ordinal ==
                  vertex.boundaryUses.front().workingEdge.ordinal);
        }
    }
    for (const weft::PlanarCdtTriangle& triangle :
         result.value->triangles) {
        CHECK(triangle.workingFace == result.value->workingFace);
        CHECK(triangle.sourceFace == result.value->sourceFace);
        CHECK(!triangle.cornerUv);
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
          "exact_lawson_cut_bridge_reference");
    CHECK(backend.exactPredicatesForFiniteDoubleInputs());
    CHECK(backend.supportsHoles());
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

void testConcavePerforatedDomain(const weft::PlanarCdtBackend& backend) {
    weft::PlanarTrimDomain concave = makeDomain(
        {{0.0, 0.0}, {8.0, 0.0}, {8.0, 8.0}, {5.0, 8.0},
         {5.0, 3.0}, {3.0, 3.0}, {3.0, 8.0}, {0.0, 8.0}});
    concave.loops.push_back(makeLoop(
        2, weft::PlanarTrimLoopRole::Hole,
        {{1.0, 1.0}, {1.0, 2.0}, {2.0, 2.0}, {2.0, 1.0}}));
    const auto result = backend.triangulate(concave);
    checkCertified(result, 12, 1);
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
    checkCertified(hole, 8, 1);
    CHECK(hole.value && hole.value->boundaryLoops.size() == 2);

    weft::PlanarTrimDomain twoHoles = makeDomain(
        {{0.0, 0.0}, {10.0, 0.0}, {10.0, 8.0}, {0.0, 8.0}});
    twoHoles.loops.push_back(makeLoop(
        2, weft::PlanarTrimLoopRole::Hole,
        {{1.0, 1.0}, {1.0, 3.0}, {3.0, 3.0}, {3.0, 1.0}}));
    twoHoles.loops.push_back(makeLoop(
        3, weft::PlanarTrimLoopRole::Hole,
        {{6.0, 2.0}, {6.0, 5.0}, {8.0, 5.0}, {8.0, 2.0}}));
    const auto twicePerforated = backend.triangulate(twoHoles);
    checkCertified(twicePerforated, 12, 2);
    CHECK(twicePerforated.value &&
          twicePerforated.value->boundaryLoops.size() == 3);

    std::swap(twoHoles.loops[1], twoHoles.loops[2]);
    const auto reordered = backend.triangulate(twoHoles);
    checkCertified(reordered, 12, 2);
    CHECK(twicePerforated.value && reordered.value &&
          twicePerforated.value->triangles.size() ==
              reordered.value->triangles.size());
    if (twicePerforated.value && reordered.value) {
        for (std::size_t index = 0;
             index < twicePerforated.value->triangles.size(); ++index) {
            CHECK(twicePerforated.value->triangles[index].vertices ==
                  reordered.value->triangles[index].vertices);
        }
    }

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

void testMultiOuterSeparateDomains(const weft::PlanarCdtBackend& backend) {
    // Two disjoint outers on one face: certify as separate CDT components.
    weft::PlanarTrimDomain multi;
    multi.face = {weft::StableIdKind::Face, 1};
    multi.sourceFace = weft::StableId{weft::StableIdKind::Face, 101};
    multi.allowCurvedUv = false;
    multi.loops.push_back(makeLoop(
        1, weft::PlanarTrimLoopRole::Outer,
        {{0.0, 0.0}, {3.0, 0.0}, {3.0, 3.0}, {0.0, 3.0}}));
    multi.loops.push_back(makeLoop(
        2, weft::PlanarTrimLoopRole::Outer,
        {{5.0, 0.0}, {8.0, 0.0}, {8.0, 3.0}, {5.0, 3.0}}));
    const auto result = backend.triangulate(multi);
    CHECK(result);
    CHECK(result.value);
    if (!result.value) return;
    CHECK(!result.value->relaxGeometryChecks);
    CHECK(result.value->boundaryLoops.size() == 2);
    CHECK(result.value->triangles.size() == 4);
    CHECK(result.value->vertices.size() == 8);
}

void testHoleBridgeOccludedMinLex(const weft::PlanarCdtBackend& backend) {
    // C-shaped outer: the hole's min-lex corner faces into the pocket and
    // has no visible outer bridge; a later hole station must win.
    weft::PlanarTrimDomain domain;
    domain.face = {weft::StableIdKind::Face, 1};
    domain.sourceFace = weft::StableId{weft::StableIdKind::Face, 101};
    domain.allowCurvedUv = false;
    domain.loops.push_back(makeLoop(
        1, weft::PlanarTrimLoopRole::Outer,
        {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {6.0, 10.0},
         {6.0, 4.0}, {4.0, 4.0}, {4.0, 10.0}, {0.0, 10.0}}));
    // Hole in the right bay; min-lex vertex is near the bay's left wall.
    domain.loops.push_back(makeLoop(
        2, weft::PlanarTrimLoopRole::Hole,
        {{7.0, 1.0}, {7.0, 3.0}, {9.0, 3.0}, {9.0, 1.0}}));
    const auto result = backend.triangulate(domain);
    checkCertified(result, 12, 1);
    CHECK(result.value && !result.value->relaxGeometryChecks);
}

void testPlaneNeverRelaxes(const weft::PlanarCdtBackend& backend) {
    const auto square = backend.triangulate(
        makeDomain({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}}));
    CHECK(square);
    CHECK(square.value && !square.value->relaxGeometryChecks);

    // Non-cocircular convex outer: ear mesh is locally Delaunay without
    // Lawson, so curved UV ears should keep validateMesh-on (no relax).
    weft::PlanarTrimDomain curved = makeDomain(
        {{0.0, 0.0}, {3.0, 0.0}, {2.0, 1.5}, {0.0, 2.0}});
    curved.allowCurvedUv = true;
    const auto curvedOk = backend.triangulate(curved);
    CHECK(curvedOk);
    CHECK(curvedOk.value);
    if (!curvedOk.value) return;
    CHECK(!curvedOk.value->relaxGeometryChecks);
}

void testCurvedUvAnnulusWindingNoAv(const weft::PlanarCdtBackend& backend) {
    // Regression for MP9 face-136 AV: curved UV hole bridging + fan can
    // flip neighbor windings; rebuild() must not invalidate live edgeTris
    // iteration (was ACCESS_VIOLATION).
    weft::PlanarTrimDomain annulus = makeDomain(
        {{0.0, 0.0}, {6.0, 0.0}, {6.0, 6.0}, {0.0, 6.0}});
    annulus.allowCurvedUv = true;
    annulus.loops.push_back(makeLoop(
        2, weft::PlanarTrimLoopRole::Hole,
        {{2.0, 2.0}, {2.0, 4.0}, {4.0, 4.0}, {4.0, 2.0}}));
    const auto result = backend.triangulate(annulus);
    CHECK(result);
    CHECK(result.value);
    if (!result.value) return;
    CHECK(result.value->triangles.size() >= 8);
    CHECK(result.value->boundaryLoops.size() == 2);
}

void testCurvedUvAuthoritativePeriodUnwrap(
    const weft::PlanarCdtBackend& backend) {
    // freeform_137 subclass: bspline U period is 1, not 2π. A band that
    // crosses the principal cut must unwrap with curvedUvUPeriod=1 or the
    // CDT polygon stays seam-crossed.
    weft::PlanarTrimDomain band = makeDomain(
        {{0.6, 0.0},
         {0.85, 0.0},
         {0.1, 0.0},  // principal jump across U=0
         {0.35, 0.0},
         {0.35, 0.4},
         {0.1, 0.4},
         {0.85, 0.4},
         {0.6, 0.4}});
    band.allowCurvedUv = true;
    band.curvedUvUPeriod = 1.0;
    const auto withPeriod = backend.triangulate(band);
    CHECK(withPeriod);
    CHECK(withPeriod.value);
    if (withPeriod.value) {
        CHECK(withPeriod.value->triangles.size() >= 6);
    }
}

void testCurvedUvTinyVPeriodDualImage(
    const weft::PlanarCdtBackend& backend) {
    // freeform_186 subclass: tiny VPeriod with a digon spur between two
    // UV images of the same canonical seam vertex. Dual-image placement +
    // spur reverse must open a positive-area chart (not shred / zero-area).
    constexpr double kPeriod = 0.016;
    weft::PlanarTrimDomain band;
    band.face = {weft::StableIdKind::Face, 1};
    band.sourceFace = weft::StableId{weft::StableIdKind::Face, 101};
    band.allowCurvedUv = true;
    band.curvedUvVPeriod = kPeriod;
    weft::PlanarTrimLoop loop;
    loop.wire = {weft::StableIdKind::Wire, 1};
    loop.declaredRole = weft::PlanarTrimLoopRole::Outer;
    loop.closed = true;
    auto add = [&](std::uint64_t cv, double u, double v) {
        const std::uint64_t edgeOrdinal = cv + 1;
        weft::PlanarTrimBoundaryUse use;
        use.sample = {{weft::StableIdKind::Boundary, edgeOrdinal},
                      static_cast<std::uint32_t>(loop.vertices.size())};
        use.workingEdge = {weft::StableIdKind::Edge, edgeOrdinal};
        use.sourceEdge = weft::StableId{weft::StableIdKind::Edge,
                                       edgeOrdinal + 100000};
        use.coedge = {weft::StableIdKind::Coedge, edgeOrdinal};
        use.uv = {u, v};
        use.allowedCurveOnSurfaceDiscrepancy = 1e-7;
        weft::PlanarTrimVertex vertex;
        vertex.boundaryUses.push_back(std::move(use));
        vertex.canonicalVertexIndex = cv;
        vertex.uv = {u, v};
        loop.vertices.push_back(std::move(vertex));
    };
    // Seam at v=0 (cv 0→1), digon spur on left returning to cv 1, seam
    // reverse (cv 1→0), right edge.
    add(0, 0.4, 0.0);
    add(5, 0.2, 0.0);
    add(1, 0.0, 0.0);
    add(14, 0.0, kPeriod);           // jump to other sheet
    add(13, 0.0, 0.75 * kPeriod);    // digon back
    add(12, 0.0, 0.25 * kPeriod);
    add(1, 0.0, 0.0);                // same sheet again (collapsed dual)
    add(5, 0.2, 0.0);
    add(0, 0.4, 0.0);
    add(20, 0.4, 0.5 * kPeriod);
    add(21, 0.4, kPeriod);
    band.loops.push_back(std::move(loop));
    const auto result = backend.triangulate(band);
    CHECK(result);
    CHECK(result.value);
    if (!result.value) return;
    CHECK(result.value->triangles.size() >= 4);
    // Opened chart must span ~one V period across dual seam images.
    double vMin = 1e300;
    double vMax = -1e300;
    for (const weft::PlanarTrimVertex& vertex : result.value->vertices) {
        vMin = std::min(vMin, vertex.uv[1]);
        vMax = std::max(vMax, vertex.uv[1]);
    }
    CHECK(vMax - vMin > 0.5 * kPeriod);
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
        testConcavePerforatedDomain(*backend);
        testNamedRefusals(*backend);
        testMultiOuterSeparateDomains(*backend);
        testHoleBridgeOccludedMinLex(*backend);
        testPlaneNeverRelaxes(*backend);
        testCurvedUvAnnulusWindingNoAv(*backend);
        testCurvedUvAuthoritativePeriodUnwrap(*backend);
        testCurvedUvTinyVPeriodDualImage(*backend);
    }
    if (failures == 0) {
        std::printf("exact planar CDT reference checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d planar CDT failure(s)\n", failures);
    return EXIT_FAILURE;
}
