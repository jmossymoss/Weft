#include "weft/interval_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <numeric>
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

weft::StableId boundary(std::uint64_t ordinal) {
    return {weft::StableIdKind::Boundary, ordinal};
}

weft::IntervalVariable variable(std::uint64_t ordinal, double desired,
                                std::uint32_t minimum = 1,
                                bool even = false) {
    return {boundary(ordinal), desired, minimum, even};
}

weft::IntervalEquality equality(
    std::initializer_list<std::uint64_t> ordinals) {
    weft::IntervalEquality result;
    for (std::uint64_t ordinal : ordinals) {
        result.boundaryIds.push_back(boundary(ordinal));
    }
    return result;
}

std::vector<std::uint32_t> exhaustiveReference(
    const weft::IntervalProblem& problem, std::uint32_t cap) {
    const std::size_t size = problem.variables.size();
    std::vector<std::size_t> labels(size);
    std::iota(labels.begin(), labels.end(), std::size_t{0});
    const auto indexOf = [&](weft::StableId id) {
        for (std::size_t index = 0; index < size; ++index) {
            if (problem.variables[index].boundaryId == id) return index;
        }
        return size;
    };
    for (const weft::IntervalEquality& equal : problem.equalities) {
        const std::size_t anchor = indexOf(equal.boundaryIds.front());
        for (weft::StableId id : equal.boundaryIds) {
            const std::size_t member = indexOf(id);
            const std::size_t from = labels[member];
            const std::size_t to = labels[anchor];
            for (std::size_t& label : labels) {
                if (label == from) label = to;
            }
        }
    }

    std::vector<std::uint32_t> result(size, 0);
    std::vector<bool> complete(size, false);
    for (std::size_t seed = 0; seed < size; ++seed) {
        if (complete[seed]) continue;
        std::vector<std::size_t> members;
        for (std::size_t index = 0; index < size; ++index) {
            if (labels[index] == labels[seed]) {
                members.push_back(index);
                complete[index] = true;
            }
        }
        std::uint32_t minimum = 1;
        bool even = false;
        for (std::size_t member : members) {
            minimum = std::max(minimum, problem.variables[member].minimum);
            even = even || problem.variables[member].requireEven;
        }
        double bestObjective = 0.0;
        std::uint32_t best = 0;
        bool haveBest = false;
        for (std::uint32_t candidate = 1; candidate <= cap; ++candidate) {
            if (candidate < minimum || (even && candidate % 2 != 0)) continue;
            double objective = 0.0;
            for (std::size_t member : members) {
                objective += std::abs(static_cast<double>(candidate) -
                                      problem.variables[member].desired);
            }
            if (!haveBest || objective < bestObjective) {
                bestObjective = objective;
                best = candidate;
                haveBest = true;
            }
        }
        for (std::size_t member : members) result[member] = best;
    }
    return result;
}

class DeterministicLcg {
public:
    explicit DeterministicLcg(std::uint64_t state) : state_(state) {}
    std::uint32_t below(std::uint32_t bound) {
        state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<std::uint32_t>((state_ >> 33U) % bound);
    }

private:
    std::uint64_t state_;
};

void testAnalyticCounts() {
    constexpr double halfPi = 1.5707963267948966;
    const auto quarter =
        weft::circularArcSegmentCount(1.0, halfPi, false);
    CHECK(quarter);
    if (quarter) CHECK(*quarter.count == 90);
    const auto full = weft::circularArcSegmentCount(
        1.0, 6.283185307179586, true);
    CHECK(full);
    if (full) CHECK(*full.count == 360);
    CHECK(weft::lineSegmentCount() == 1);

    const auto invalid =
        weft::circularArcSegmentCount(0.0, halfPi, false);
    CHECK(!invalid);
    CHECK(invalid.failure &&
          invalid.failure->code == "interval.invalid_problem");
}

void testExactClassesAndFailures() {
    weft::IntervalProblem problem;
    problem.variables = {variable(1, 4.0), variable(2, 10.0),
                         variable(3, 11.0)};
    problem.equalities.push_back(equality({1, 2, 3}));
    const auto solved = weft::solveIntervals(problem);
    CHECK(solved);
    if (solved) {
        CHECK(solved.solution->find(boundary(1)) == 10);
        CHECK(solved.solution->find(boundary(2)) == 10);
        CHECK(solved.solution->find(boundary(3)) == 10);
    }

    weft::IntervalProblem tie;
    tie.variables = {variable(1, 5.0, 1, true)};
    const auto tied = weft::solveIntervals(tie);
    CHECK(tied && tied.solution->find(boundary(1)) == 4);

    weft::IntervalProblem fixed;
    fixed.variables = {variable(1, 20.0), variable(2, 4.0)};
    fixed.variables.front().exact = 7;
    fixed.equalities.push_back(equality({1, 2}));
    const auto fixedResult = weft::solveIntervals(fixed);
    CHECK(fixedResult);
    CHECK(fixedResult && fixedResult.solution->find(boundary(1)) == 7);
    CHECK(fixedResult && fixedResult.solution->find(boundary(2)) == 7);

    weft::IntervalProblem exactConflict = fixed;
    exactConflict.variables.back().exact = 8;
    const auto conflictResult = weft::solveIntervals(exactConflict);
    CHECK(!conflictResult);
    CHECK(conflictResult.failure &&
          conflictResult.failure->code == "interval.exact_conflict");

    weft::IntervalProblem belowMinimum;
    belowMinimum.variables = {variable(1, 10.0, 9)};
    belowMinimum.variables.front().exact = 8;
    const auto belowResult = weft::solveIntervals(belowMinimum);
    CHECK(!belowResult);
    CHECK(belowResult.failure &&
          belowResult.failure->code == "interval.exact_below_minimum");

    weft::IntervalProblem parity;
    parity.variables = {variable(1, 8.0, 2, true)};
    parity.variables.front().exact = 7;
    const auto parityResult = weft::solveIntervals(parity);
    CHECK(!parityResult);
    CHECK(parityResult.failure &&
          parityResult.failure->code == "interval.exact_parity_conflict");

    weft::IntervalProblem capped;
    capped.variables = {variable(1, 20.0, 17, true)};
    weft::SamplingConfiguration configuration;
    configuration.minimumClosedCurveSegments = 3;
    configuration.maximumSegmentCount = 17;
    const auto capFailure = weft::solveIntervals(capped, configuration);
    CHECK(!capFailure);
    CHECK(capFailure.failure &&
          capFailure.failure->code == "interval.count_exceeds_maximum");
    CHECK(capFailure.failure &&
          capFailure.failure->subjects == std::vector<weft::StableId>{boundary(1)});

    weft::IntervalProblem future;
    future.variables = {variable(1, 4.0), variable(2, 4.0)};
    future.sums.push_back({{boundary(1)}, {boundary(2)}});
    const auto unsupported = weft::solveIntervals(future);
    CHECK(!unsupported);
    CHECK(unsupported.failure &&
          unsupported.failure->code == "interval.unsupported_constraint");
}

void testPropertyBattery() {
    DeterministicLcg random(0x5EED5EED5EED5EEDULL);
    for (std::uint32_t iteration = 0; iteration < 200; ++iteration) {
        weft::IntervalProblem problem;
        const std::uint32_t count = 1 + random.below(6);
        std::uint64_t ordinal = 0;
        for (std::uint32_t index = 0; index < count; ++index) {
            ordinal += 1 + random.below(3);
            problem.variables.push_back(variable(
                ordinal, static_cast<double>(random.below(481)) / 16.0,
                1 + random.below(20), random.below(4) == 0));
        }
        for (std::uint32_t index = 0; index < random.below(3); ++index) {
            problem.equalities.push_back(
                {{problem.variables[random.below(count)].boundaryId,
                  problem.variables[random.below(count)].boundaryId}});
        }

        const std::vector<std::uint32_t> reference =
            exhaustiveReference(problem, 4096);
        const auto first = weft::solveIntervals(problem);
        const auto second = weft::solveIntervals(problem);
        CHECK(first && second);
        if (!first || !second) continue;
        CHECK(first.solution->counts.size() == reference.size());
        CHECK(first.solution->counts == second.solution->counts);
        for (std::size_t index = 0; index < reference.size(); ++index) {
            CHECK(first.solution->counts[index].count == reference[index]);
        }
    }
}

}  // namespace

int main() {
    testAnalyticCounts();
    testExactClassesAndFailures();
    testPropertyBattery();
    if (failures == 0) {
        std::printf("interval solver checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d interval solver failure(s)\n", failures);
    return EXIT_FAILURE;
}
