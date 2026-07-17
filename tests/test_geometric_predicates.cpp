#include "weft/geometric_predicates.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>

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

weft::ExactSign signOf(std::int64_t value) {
    if (value < 0) return weft::ExactSign::Negative;
    if (value > 0) return weft::ExactSign::Positive;
    return weft::ExactSign::Zero;
}

class DeterministicLcg {
public:
    explicit DeterministicLcg(std::uint64_t state) : state_(state) {}

    std::int64_t coordinate(std::int64_t radius) {
        state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
        const std::uint64_t span = static_cast<std::uint64_t>(2 * radius + 1);
        return static_cast<std::int64_t>((state_ >> 32U) % span) - radius;
    }

private:
    std::uint64_t state_;
};

void testOrientation(const weft::GeometricPredicates& predicates) {
    CHECK(std::string(predicates.backendCode()) == "exact_ieee754_dyadic");
    CHECK(predicates.exactForFiniteDoubleInputs());
    const auto positive =
        predicates.orient2d({0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0});
    CHECK(positive && *positive.value == weft::ExactSign::Positive);
    const auto negative =
        predicates.orient2d({0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0});
    CHECK(negative && *negative.value == weft::ExactSign::Negative);
    const auto zero =
        predicates.orient2d({0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0});
    CHECK(zero && *zero.value == weft::ExactSign::Zero);

    // Both products underflow in ordinary double arithmetic, but the exact
    // determinant of the supplied doubles is positive.
    volatile double tinyInput = std::numeric_limits<double>::denorm_min();
    const double tiny = tinyInput;
    const double naiveTiny = tiny * tiny;
    CHECK(naiveTiny == 0.0);
    const auto subnormal =
        predicates.orient2d({0.0, 0.0}, {tiny, 0.0}, {0.0, tiny});
    CHECK(subnormal && *subnormal.value == weft::ExactSign::Positive);

    // Deterministic cancellation witness found by comparing the ordinary
    // determinant to exact rational arithmetic over the same input doubles.
    const weft::PredicatePoint2 a{-2.538723182042409e-68,
                                   7.483486746689121e-68};
    const weft::PredicatePoint2 b{-4.778200006175269e-68,
                                   2.544294621697599e-68};
    const weft::PredicatePoint2 c{-7.544819338865486e-68,
                                  -3.5575164234619973e-68};
    const double naive = (b[0] - a[0]) * (c[1] - a[1]) -
        (b[1] - a[1]) * (c[0] - a[0]);
    CHECK(naive == 0.0);
    const auto robust = predicates.orient2d(a, b, c);
    CHECK(robust && *robust.value == weft::ExactSign::Negative);

    const auto invalid = predicates.orient2d(
        {std::numeric_limits<double>::quiet_NaN(), 0.0}, {0.0, 0.0},
        {1.0, 0.0});
    CHECK(!invalid);
    CHECK(invalid.failure &&
          invalid.failure->code == "predicate.non_finite_input");
}

void testIncircle(const weft::GeometricPredicates& predicates) {
    const auto inside = predicates.incircle(
        {0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}, {0.5, 0.5});
    CHECK(inside && *inside.value == weft::ExactSign::Positive);
    const auto on = predicates.incircle(
        {0.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}, {2.0, 2.0});
    CHECK(on && *on.value == weft::ExactSign::Zero);

    const weft::PredicatePoint2 a{-9.85288280426692e-38,
                                   4.0453018198470705e-37};
    const weft::PredicatePoint2 b{-1.5246232959862515e-37,
                                   2.0462653016788466e-37};
    const weft::PredicatePoint2 c{5.281537970195876e-37,
                                  -5.897939601716289e-37};
    const weft::PredicatePoint2 d{1.2323748548112106e-36,
                                  -9.368678453334858e-38};
    const double adx = a[0] - d[0];
    const double ady = a[1] - d[1];
    const double bdx = b[0] - d[0];
    const double bdy = b[1] - d[1];
    const double cdx = c[0] - d[0];
    const double cdy = c[1] - d[1];
    const double naive = (adx * adx + ady * ady) *
            (bdx * cdy - bdy * cdx) +
        (bdx * bdx + bdy * bdy) * (cdx * ady - cdy * adx) +
        (cdx * cdx + cdy * cdy) * (adx * bdy - ady * bdx);
    CHECK(naive < 0.0);
    const auto robust = predicates.incircle(a, b, c, d);
    CHECK(robust && *robust.value == weft::ExactSign::Positive);
}

void testSquaredDistanceComparison(
    const weft::GeometricPredicates& predicates) {
    const auto farther = predicates.compareSquaredDistance(
        {0.0, 0.0}, {3.0, 4.0}, {1.0, 1.0});
    CHECK(farther && *farther.value == weft::ExactSign::Positive);
    const auto nearer = predicates.compareSquaredDistance(
        {0.0, 0.0}, {1.0, 1.0}, {3.0, 4.0});
    CHECK(nearer && *nearer.value == weft::ExactSign::Negative);
    const auto tied = predicates.compareSquaredDistance(
        {1.0, 1.0}, {2.0, 2.0}, {0.0, 0.0});
    CHECK(tied && *tied.value == weft::ExactSign::Zero);

    volatile double tinyInput = std::numeric_limits<double>::denorm_min();
    const double tiny = tinyInput;
    CHECK(tiny * tiny == 0.0);
    const auto subnormal = predicates.compareSquaredDistance(
        {0.0, 0.0}, {tiny, 0.0}, {0.0, 0.0});
    CHECK(subnormal && *subnormal.value == weft::ExactSign::Positive);

    const auto invalid = predicates.compareSquaredDistance(
        {0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0},
        {0.0, 0.0});
    CHECK(!invalid);
    CHECK(invalid.failure &&
          invalid.failure->code == "predicate.non_finite_input");
}

void testIntegerPropertyBattery(const weft::GeometricPredicates& predicates) {
    DeterministicLcg random(0xC0FFEE1234567890ULL);
    for (int iteration = 0; iteration < 1000; ++iteration) {
        const std::int64_t ax = random.coordinate(1000);
        const std::int64_t ay = random.coordinate(1000);
        const std::int64_t bx = random.coordinate(1000);
        const std::int64_t by = random.coordinate(1000);
        const std::int64_t cx = random.coordinate(1000);
        const std::int64_t cy = random.coordinate(1000);
        const std::int64_t determinant =
            (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        const auto exact = predicates.orient2d(
            {static_cast<double>(ax), static_cast<double>(ay)},
            {static_cast<double>(bx), static_cast<double>(by)},
            {static_cast<double>(cx), static_cast<double>(cy)});
        CHECK(exact && *exact.value == signOf(determinant));
    }

    for (int iteration = 0; iteration < 500; ++iteration) {
        const std::int64_t ax = random.coordinate(20);
        const std::int64_t ay = random.coordinate(20);
        const std::int64_t bx = random.coordinate(20);
        const std::int64_t by = random.coordinate(20);
        const std::int64_t cx = random.coordinate(20);
        const std::int64_t cy = random.coordinate(20);
        const std::int64_t dx = random.coordinate(20);
        const std::int64_t dy = random.coordinate(20);
        const std::int64_t adx = ax - dx;
        const std::int64_t ady = ay - dy;
        const std::int64_t bdx = bx - dx;
        const std::int64_t bdy = by - dy;
        const std::int64_t cdx = cx - dx;
        const std::int64_t cdy = cy - dy;
        const std::int64_t determinant =
            (adx * adx + ady * ady) * (bdx * cdy - bdy * cdx) +
            (bdx * bdx + bdy * bdy) * (cdx * ady - cdy * adx) +
            (cdx * cdx + cdy * cdy) * (adx * bdy - ady * bdx);
        const auto exact = predicates.incircle(
            {static_cast<double>(ax), static_cast<double>(ay)},
            {static_cast<double>(bx), static_cast<double>(by)},
            {static_cast<double>(cx), static_cast<double>(cy)},
            {static_cast<double>(dx), static_cast<double>(dy)});
        CHECK(exact && *exact.value == signOf(determinant));
    }
}

void testSegmentIntersection(const weft::GeometricPredicates& predicates) {
    const auto proper = predicates.segmentIntersection(
        {0.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}, {2.0, 0.0});
    CHECK(proper &&
          *proper.value == weft::SegmentIntersectionKind::Proper);
    const auto endpoint = predicates.segmentIntersection(
        {0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {2.0, 1.0});
    CHECK(endpoint &&
          *endpoint.value == weft::SegmentIntersectionKind::EndpointTouch);
    const auto overlap = predicates.segmentIntersection(
        {0.0, 0.0}, {3.0, 0.0}, {1.0, 0.0}, {2.0, 0.0});
    CHECK(overlap &&
          *overlap.value == weft::SegmentIntersectionKind::CollinearOverlap);
    const auto none = predicates.segmentIntersection(
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0});
    CHECK(none && *none.value == weft::SegmentIntersectionKind::None);
    const auto pointOnSegment = predicates.segmentIntersection(
        {1.0, 0.0}, {1.0, 0.0}, {0.0, 0.0}, {2.0, 0.0});
    CHECK(pointOnSegment &&
          *pointOnSegment.value ==
              weft::SegmentIntersectionKind::EndpointTouch);
}

}  // namespace

int main() {
    const auto predicates = weft::makeExactDyadicPredicates();
    CHECK(predicates != nullptr);
    if (predicates) {
        testOrientation(*predicates);
        testIncircle(*predicates);
        testSquaredDistanceComparison(*predicates);
        testIntegerPropertyBattery(*predicates);
        testSegmentIntersection(*predicates);
    }
    if (failures == 0) {
        std::printf("exact geometric predicate checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d geometric predicate failure(s)\n", failures);
    return EXIT_FAILURE;
}
