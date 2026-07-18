#include "weft/interval_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <limits>
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
    return {boundary(ordinal), desired, minimum, even, std::nullopt};
}

weft::IntervalEquality equality(
    std::initializer_list<std::uint64_t> ordinals) {
    weft::IntervalEquality result;
    for (std::uint64_t ordinal : ordinals) {
        result.boundaryIds.push_back(boundary(ordinal));
    }
    return result;
}

weft::IntervalSum sumEquality(
    std::initializer_list<std::uint64_t> lhs,
    std::initializer_list<std::uint64_t> rhs) {
    weft::IntervalSum result;
    for (std::uint64_t ordinal : lhs) result.lhs.push_back(boundary(ordinal));
    for (std::uint64_t ordinal : rhs) result.rhs.push_back(boundary(ordinal));
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
            bool exactMatch = true;
            for (std::size_t member : members) {
                if (problem.variables[member].exact &&
                    candidate != *problem.variables[member].exact) {
                    exactMatch = false;
                }
            }
            if (candidate < minimum || (even && candidate % 2 != 0) ||
                !exactMatch) {
                continue;
            }
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

struct ExhaustiveSumReference {
    bool feasible = false;
    double objective = 0.0;
    std::uint64_t commonTotal = 0;
    std::vector<std::uint32_t> counts;
};

bool equivalentObjective(double left, double right) {
    if (!std::isfinite(left) || !std::isfinite(right)) return left == right;
    const double scale =
        std::max({1.0, std::abs(left), std::abs(right)});
    return std::abs(left - right) <= 1e-12 * scale;
}

bool betterObjective(double candidate, double incumbent) {
    return candidate < incumbent &&
        !equivalentObjective(candidate, incumbent);
}

ExhaustiveSumReference exhaustiveSumReference(
    const weft::IntervalProblem& problem, std::uint32_t cap) {
    ExhaustiveSumReference result;
    std::vector<std::uint32_t> assignment(problem.variables.size(), 0U);
    const auto indexOf = [&](weft::StableId id) {
        for (std::size_t index = 0; index < problem.variables.size(); ++index) {
            if (problem.variables[index].boundaryId == id) return index;
        }
        return problem.variables.size();
    };
    const auto visit = [&](const auto& self, std::size_t index) -> void {
        if (index != problem.variables.size()) {
            const weft::IntervalVariable& variable = problem.variables[index];
            std::uint32_t start = variable.minimum;
            const std::uint32_t step = variable.requireEven ? 2U : 1U;
            if (variable.requireEven && start % 2U != 0U) ++start;
            if (variable.exact) start = *variable.exact;
            for (std::uint32_t candidate = start; candidate <= cap;) {
                if ((!variable.exact || candidate == *variable.exact) &&
                    (!variable.requireEven || candidate % 2U == 0U) &&
                    candidate >= variable.minimum) {
                    assignment[index] = candidate;
                    self(self, index + 1U);
                }
                if (variable.exact || candidate > cap - step) break;
                candidate += step;
            }
            return;
        }

        for (const weft::IntervalEquality& equal : problem.equalities) {
            const std::uint32_t expected =
                assignment[indexOf(equal.boundaryIds.front())];
            for (weft::StableId id : equal.boundaryIds) {
                if (assignment[indexOf(id)] != expected) return;
            }
        }
        std::uint64_t firstTotal = 0;
        for (std::size_t sumIndex = 0; sumIndex < problem.sums.size();
             ++sumIndex) {
            const weft::IntervalSum& sum = problem.sums[sumIndex];
            std::uint64_t lhs = 0;
            std::uint64_t rhs = 0;
            for (weft::StableId id : sum.lhs) lhs += assignment[indexOf(id)];
            for (weft::StableId id : sum.rhs) rhs += assignment[indexOf(id)];
            if (lhs != rhs) return;
            if (sumIndex == 0) firstTotal = lhs;
        }
        double objective = 0.0;
        for (std::size_t item = 0; item < assignment.size(); ++item) {
            objective += std::abs(
                static_cast<double>(assignment[item]) -
                problem.variables[item].desired);
        }
        const bool tied = result.feasible &&
            equivalentObjective(objective, result.objective);
        const bool tieBreaksEarlier =
            tied &&
            (problem.sums.size() == 1U
                 ? firstTotal < result.commonTotal
                 : assignment < result.counts);
        if (!result.feasible || betterObjective(objective, result.objective) ||
            tieBreaksEarlier) {
            result.feasible = true;
            result.objective = objective;
            result.commonTotal = firstTotal;
            result.counts = assignment;
        }
    };
    visit(visit, 0U);
    return result;
}

double solutionObjective(const weft::IntervalProblem& problem,
                         const weft::IntervalSolution& solution) {
    double objective = 0.0;
    for (const weft::IntervalVariable& variable : problem.variables) {
        objective += std::abs(
            static_cast<double>(*solution.find(variable.boundaryId)) -
            variable.desired);
    }
    return objective;
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

}

void testExactSumConstraints() {
    weft::SamplingConfiguration configuration;
    configuration.minimumClosedCurveSegments = 3;
    configuration.maximumSegmentCount = 12;

    weft::IntervalProblem chain;
    chain.variables = {variable(1, 2.0), variable(2, 5.0),
                       variable(3, 10.0)};
    chain.sums.push_back(sumEquality({1, 2}, {3}));
    const auto chainResult = weft::solveIntervals(chain, configuration);
    CHECK(chainResult);
    CHECK(chainResult && chainResult.solution->find(boundary(1)) == 2);
    CHECK(chainResult && chainResult.solution->find(boundary(2)) == 5);
    CHECK(chainResult && chainResult.solution->find(boundary(3)) == 7);

    weft::IntervalProblem mixed;
    mixed.variables = {variable(1, 4.0, 2, true), variable(2, 3.0),
                       variable(3, 9.0)};
    mixed.variables.back().exact = 9;
    mixed.sums.push_back(sumEquality({1, 2}, {3}));
    const auto mixedResult = weft::solveIntervals(mixed, configuration);
    CHECK(mixedResult);
    CHECK(mixedResult && mixedResult.solution->find(boundary(1)) == 4);
    CHECK(mixedResult && mixedResult.solution->find(boundary(2)) == 5);
    CHECK(mixedResult && mixedResult.solution->find(boundary(3)) == 9);

    weft::IntervalProblem cancelled;
    cancelled.variables = {variable(1, 2.0), variable(2, 20.0),
                           variable(3, 8.0), variable(4, 4.0)};
    cancelled.equalities.push_back(equality({2, 4}));
    cancelled.sums.push_back(sumEquality({1, 2}, {3, 4}));
    const auto cancelledResult =
        weft::solveIntervals(cancelled, configuration);
    CHECK(cancelledResult);
    CHECK(cancelledResult &&
          cancelledResult.solution->find(boundary(1)) == 2);
    CHECK(cancelledResult &&
          cancelledResult.solution->find(boundary(3)) == 2);
    CHECK(cancelledResult &&
          cancelledResult.solution->find(boundary(2)) == 4);
    CHECK(cancelledResult &&
          cancelledResult.solution->find(boundary(4)) == 4);

    weft::IntervalProblem independent;
    independent.variables = {variable(1, 1.0), variable(2, 3.0),
                             variable(3, 6.0), variable(4, 4.0)};
    independent.sums.push_back(sumEquality({1}, {2}));
    independent.sums.push_back(sumEquality({3}, {4}));
    const auto independentResult =
        weft::solveIntervals(independent, configuration);
    CHECK(independentResult);
    CHECK(independentResult &&
          independentResult.solution->find(boundary(1)) == 1);
    CHECK(independentResult &&
          independentResult.solution->find(boundary(2)) == 1);
    CHECK(independentResult &&
          independentResult.solution->find(boundary(3)) == 4);
    CHECK(independentResult &&
          independentResult.solution->find(boundary(4)) == 4);

    weft::IntervalProblem tautology;
    tautology.variables = {variable(1, 3.0), variable(2, 7.0)};
    tautology.sums.push_back(sumEquality({1, 2}, {1, 2}));
    const auto tautologyResult =
        weft::solveIntervals(tautology, configuration);
    CHECK(tautologyResult);
    CHECK(tautologyResult &&
          tautologyResult.solution->find(boundary(1)) == 3);
    CHECK(tautologyResult &&
          tautologyResult.solution->find(boundary(2)) == 7);

    weft::IntervalProblem infeasible;
    infeasible.variables = {variable(1, 1.0), variable(2, 2.0)};
    infeasible.variables[0].exact = 1;
    infeasible.variables[1].exact = 2;
    infeasible.sums.push_back(sumEquality({1}, {2}));
    const auto infeasibleResult =
        weft::solveIntervals(infeasible, configuration);
    CHECK(!infeasibleResult);
    CHECK(infeasibleResult.failure &&
          infeasibleResult.failure->code == "interval.sum_infeasible");

    weft::IntervalProblem aliased;
    aliased.variables = {variable(1, 2.0), variable(2, 2.0),
                         variable(3, 4.0)};
    aliased.equalities.push_back(equality({1, 2}));
    aliased.sums.push_back(sumEquality({1, 2}, {3}));
    const auto aliasResult =
        weft::solveIntervals(aliased, configuration);
    CHECK(!aliasResult);
    CHECK(aliasResult.failure &&
          aliasResult.failure->code ==
              "interval.sum_aliasing_unsupported");

    weft::IntervalProblem coupled;
    coupled.variables = {variable(1, 2.0), variable(2, 2.0),
                         variable(3, 2.0)};
    coupled.sums.push_back(sumEquality({1}, {2}));
    coupled.sums.push_back(sumEquality({1}, {3}));
    const auto coupledResult =
        weft::solveIntervals(coupled, configuration);
    CHECK(coupledResult);
    CHECK(coupledResult &&
          coupledResult.solution->find(boundary(1)) == 2);
    CHECK(coupledResult &&
          coupledResult.solution->find(boundary(2)) == 2);
    CHECK(coupledResult &&
          coupledResult.solution->find(boundary(3)) == 2);

    weft::IntervalProblem invalid;
    invalid.variables = {variable(1, 2.0), variable(2, 2.0)};
    invalid.sums.push_back(sumEquality({}, {2}));
    const auto invalidResult =
        weft::solveIntervals(invalid, configuration);
    CHECK(!invalidResult);
    CHECK(invalidResult.failure &&
          invalidResult.failure->code == "interval.invalid_problem");

    weft::IntervalProblem complex;
    complex.variables = {variable(1, 2.0), variable(2, 2.0, 2, true),
                         variable(3, 3.0)};
    complex.variables[2].exact = 3;
    complex.sums.push_back(sumEquality({1, 2}, {3}));
    weft::SamplingConfiguration large;
    large.minimumClosedCurveSegments = 3;
    large.maximumSegmentCount = 4096;
    const auto complexityResult = weft::solveIntervals(complex, large);
    CHECK(!complexityResult);
    CHECK(complexityResult.failure &&
          complexityResult.failure->code ==
              "interval.sum_complexity_exceeded");
}

void testBoundedCoupledSumConstraints() {
    weft::SamplingConfiguration configuration;
    configuration.minimumClosedCurveSegments = 3;
    configuration.maximumSegmentCount = 8;

    weft::IntervalProblem coupled;
    coupled.variables = {variable(1, 2.0), variable(2, 3.0),
                         variable(3, 7.0), variable(4, 4.0)};
    coupled.sums.push_back(sumEquality({1, 2}, {3}));
    coupled.sums.push_back(sumEquality({3}, {4}));
    const auto solved = weft::solveIntervals(coupled, configuration);
    CHECK(solved);
    CHECK(solved && solved.solution->find(boundary(1)) == 2);
    CHECK(solved && solved.solution->find(boundary(2)) == 3);
    CHECK(solved && solved.solution->find(boundary(3)) == 5);
    CHECK(solved && solved.solution->find(boundary(4)) == 5);

    weft::IntervalProblem infeasible = coupled;
    infeasible.variables[0].exact = 1;
    infeasible.variables[1].exact = 1;
    infeasible.variables[2].exact = 3;
    infeasible.variables[3].exact = 3;
    const auto infeasibleResult =
        weft::solveIntervals(infeasible, configuration);
    CHECK(!infeasibleResult);
    CHECK(infeasibleResult.failure &&
          infeasibleResult.failure->code == "interval.sum_infeasible");

    weft::IntervalProblem equalityAliased;
    equalityAliased.variables = {
        variable(1, 2.0), variable(2, 8.0), variable(3, 3.0),
        variable(4, 5.0), variable(5, 5.0)};
    equalityAliased.variables[0].exact = 2;
    equalityAliased.variables[2].exact = 3;
    equalityAliased.equalities.push_back(equality({1, 2}));
    equalityAliased.sums.push_back(sumEquality({1, 3}, {4}));
    equalityAliased.sums.push_back(sumEquality({2, 3}, {5}));
    const auto equalityAliasResult =
        weft::solveIntervals(equalityAliased, configuration);
    CHECK(equalityAliasResult);
    CHECK(equalityAliasResult &&
          equalityAliasResult.solution->find(boundary(1)) == 2);
    CHECK(equalityAliasResult &&
          equalityAliasResult.solution->find(boundary(2)) == 2);
    CHECK(equalityAliasResult &&
          equalityAliasResult.solution->find(boundary(4)) == 5);
    CHECK(equalityAliasResult &&
          equalityAliasResult.solution->find(boundary(5)) == 5);

    weft::IntervalProblem twoComponents;
    for (std::uint64_t ordinal = 1; ordinal <= 6; ++ordinal) {
        twoComponents.variables.push_back(variable(ordinal, 2.0));
    }
    twoComponents.sums.push_back(sumEquality({1}, {2}));
    twoComponents.sums.push_back(sumEquality({1}, {3}));
    twoComponents.sums.push_back(sumEquality({4}, {5}));
    twoComponents.sums.push_back(sumEquality({4}, {6}));
    const auto unsupported =
        weft::solveIntervals(twoComponents, configuration);
    CHECK(!unsupported);
    CHECK(unsupported.failure &&
          unsupported.failure->code ==
              "interval.coupled_sum_unsupported");

    weft::IntervalProblem tooManyClasses;
    for (std::uint64_t ordinal = 1; ordinal <= 10; ++ordinal) {
        tooManyClasses.variables.push_back(variable(ordinal, 2.0));
        if (ordinal > 1) {
            tooManyClasses.sums.push_back(sumEquality({1}, {ordinal}));
        }
    }
    const auto classBudget =
        weft::solveIntervals(tooManyClasses, configuration);
    CHECK(!classBudget);
    CHECK(classBudget.failure &&
          classBudget.failure->code ==
              "interval.sum_complexity_exceeded");

    weft::IntervalProblem assignmentOverflow;
    assignmentOverflow.variables = {
        variable(1, 2.0), variable(2, 2.0), variable(3, 2.0)};
    assignmentOverflow.sums.push_back(sumEquality({1}, {2}));
    assignmentOverflow.sums.push_back(sumEquality({1}, {3}));
    weft::SamplingConfiguration maximumDomain = configuration;
    maximumDomain.maximumSegmentCount =
        std::numeric_limits<std::uint32_t>::max();
    const auto overflow =
        weft::solveIntervals(assignmentOverflow, maximumDomain);
    CHECK(!overflow);
    CHECK(overflow.failure &&
          overflow.failure->code == "interval.sum_complexity_exceeded");
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

void testSumPropertyBattery() {
    DeterministicLcg random(0x51A5A11C0FFEE123ULL);
    weft::SamplingConfiguration configuration;
    configuration.minimumClosedCurveSegments = 3;
    configuration.maximumSegmentCount = 8;
    for (std::uint32_t iteration = 0; iteration < 100; ++iteration) {
        weft::IntervalProblem problem;
        const std::uint32_t count = 3 + random.below(3);
        const std::uint32_t split = 1 + random.below(count - 1U);
        for (std::uint32_t index = 0; index < count; ++index) {
            problem.variables.push_back(variable(
                index + 1U,
                static_cast<double>(random.below(101)) / 10.0,
                1 + random.below(3), random.below(4) == 0));
            weft::IntervalVariable& generated = problem.variables.back();
            if (random.below(8) == 0) {
                std::uint32_t start = generated.minimum;
                const std::uint32_t step = generated.requireEven ? 2U : 1U;
                if (generated.requireEven && start % 2U != 0U) ++start;
                const std::uint32_t choices = (8U - start) / step + 1U;
                generated.exact = start + step * random.below(choices);
            }
        }
        weft::IntervalSum sum;
        for (std::uint32_t index = 0; index < count; ++index) {
            (index < split ? sum.lhs : sum.rhs)
                .push_back(problem.variables[index].boundaryId);
        }
        problem.sums.push_back(std::move(sum));

        const ExhaustiveSumReference reference =
            exhaustiveSumReference(problem, 8U);
        const auto first = weft::solveIntervals(problem, configuration);
        const auto repeated = weft::solveIntervals(problem, configuration);
        CHECK(static_cast<bool>(first) == reference.feasible);
        CHECK(static_cast<bool>(first) == static_cast<bool>(repeated));
        if (!first || !repeated) {
            CHECK(first.failure &&
                  first.failure->code == "interval.sum_infeasible");
            continue;
        }
        CHECK(first.solution->counts == repeated.solution->counts);
        CHECK(std::abs(solutionObjective(problem, *first.solution) -
                       reference.objective) < 1e-12);
        std::uint64_t lhs = 0;
        std::uint64_t rhs = 0;
        for (weft::StableId id : problem.sums.front().lhs) {
            lhs += *first.solution->find(id);
        }
        for (weft::StableId id : problem.sums.front().rhs) {
            rhs += *first.solution->find(id);
        }
        CHECK(lhs == rhs);
        CHECK(lhs == reference.commonTotal);
    }
}

void testCoupledSumPropertyBattery() {
    DeterministicLcg random(0xC0A71ED5A11C0DEULL);
    weft::SamplingConfiguration configuration;
    configuration.minimumClosedCurveSegments = 3;
    configuration.maximumSegmentCount = 5;
    for (std::uint32_t iteration = 0; iteration < 100; ++iteration) {
        weft::IntervalProblem problem;
        for (std::uint32_t index = 0; index < 4; ++index) {
            problem.variables.push_back(variable(
                index + 1U,
                static_cast<double>(random.below(61)) / 10.0,
                1 + random.below(2), random.below(4) == 0));
            weft::IntervalVariable& generated = problem.variables.back();
            if (random.below(8) == 0) {
                std::uint32_t start = generated.minimum;
                const std::uint32_t step =
                    generated.requireEven ? 2U : 1U;
                if (generated.requireEven && start % 2U != 0U) ++start;
                const std::uint32_t choices = (5U - start) / step + 1U;
                generated.exact = start + step * random.below(choices);
            }
        }
        problem.sums.push_back(sumEquality({1, 2}, {3}));
        problem.sums.push_back(sumEquality({3}, {4}));

        const ExhaustiveSumReference reference =
            exhaustiveSumReference(problem, 5U);
        const auto first = weft::solveIntervals(problem, configuration);
        const auto repeated =
            weft::solveIntervals(problem, configuration);
        CHECK(static_cast<bool>(first) == reference.feasible);
        CHECK(static_cast<bool>(first) == static_cast<bool>(repeated));
        if (!first || !repeated) {
            CHECK(first.failure &&
                  first.failure->code == "interval.sum_infeasible");
            continue;
        }
        CHECK(first.solution->counts == repeated.solution->counts);
        CHECK(std::abs(solutionObjective(problem, *first.solution) -
                       reference.objective) < 1e-12);
        CHECK(first.solution->counts.size() == reference.counts.size());
        for (std::size_t index = 0;
             index < first.solution->counts.size() &&
             index < reference.counts.size();
             ++index) {
            CHECK(first.solution->counts[index].count ==
                  reference.counts[index]);
        }
    }
}

}  // namespace

int main() {
    testAnalyticCounts();
    testExactClassesAndFailures();
    testExactSumConstraints();
    testBoundedCoupledSumConstraints();
    testPropertyBattery();
    testSumPropertyBattery();
    testCoupledSumPropertyBattery();
    if (failures == 0) {
        std::printf("interval solver checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d interval solver failure(s)\n", failures);
    return EXIT_FAILURE;
}
