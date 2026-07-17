#include "weft/interval_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <utility>

namespace weft {
namespace {

IntervalSolveResult solveFailure(std::string code, std::string message,
                                 std::vector<StableId> subjects = {}) {
    IntervalSolveResult result;
    result.failure = IntervalFailure{std::move(code), std::move(message),
                                     std::move(subjects)};
    return result;
}

SegmentCountResult countFailure(std::string code, std::string message) {
    SegmentCountResult result;
    result.failure =
        IntervalFailure{std::move(code), std::move(message), {}};
    return result;
}

bool validConfiguration(const SamplingConfiguration& configuration) {
    return std::isfinite(configuration.chordTolerance) &&
        configuration.chordTolerance > 0.0 &&
        std::isfinite(configuration.normalAngleToleranceRadians) &&
        configuration.normalAngleToleranceRadians > 0.0 &&
        configuration.minimumClosedCurveSegments >= 3 &&
        configuration.maximumSegmentCount >=
            configuration.minimumClosedCurveSegments;
}

struct IntervalClassDomain {
    std::vector<std::size_t> members;
    std::uint32_t start = 1;
    std::uint32_t maximum = 1;
    std::uint32_t step = 1;
};

double classObjective(const IntervalProblem& problem,
                      const IntervalClassDomain& domain,
                      std::uint32_t candidate) {
    double objective = 0.0;
    for (std::size_t member : domain.members) {
        objective += std::abs(static_cast<double>(candidate) -
                              problem.variables[member].desired);
    }
    return objective;
}

struct MarginalIncrement {
    double delta = 0.0;
    std::size_t classIndex = 0;
    std::uint32_t nextCount = 0;
};

struct MarginalIncrementOrder {
    bool operator()(const MarginalIncrement& left,
                    const MarginalIncrement& right) const noexcept {
        if (left.delta != right.delta) return left.delta > right.delta;
        // On an objective tie, increment the later stable class first so the
        // earlier class remains at its smaller count.
        return left.classIndex < right.classIndex;
    }
};

struct AllocationCurve {
    std::vector<double> costs;
    std::vector<std::size_t> actions;
};

AllocationCurve buildAllocationCurve(
    const IntervalProblem& problem,
    const std::vector<IntervalClassDomain>& domains,
    const std::vector<std::size_t>& classIndices,
    std::uint32_t step) {
    AllocationCurve curve;
    double baseObjective = 0.0;
    std::size_t actionCount = 0;
    std::priority_queue<MarginalIncrement,
                        std::vector<MarginalIncrement>,
                        MarginalIncrementOrder>
        queue;
    for (std::size_t classIndex : classIndices) {
        const IntervalClassDomain& domain = domains[classIndex];
        baseObjective += classObjective(problem, domain, domain.start);
        actionCount +=
            static_cast<std::size_t>((domain.maximum - domain.start) / step);
        if (domain.start < domain.maximum) {
            const std::uint32_t next = domain.start + step;
            queue.push({classObjective(problem, domain, next) -
                            classObjective(problem, domain, domain.start),
                        classIndex, next});
        }
    }
    curve.costs.reserve(actionCount + 1);
    curve.actions.reserve(actionCount);
    curve.costs.push_back(baseObjective);
    while (!queue.empty()) {
        const MarginalIncrement increment = queue.top();
        queue.pop();
        curve.actions.push_back(increment.classIndex);
        curve.costs.push_back(curve.costs.back() + increment.delta);
        const IntervalClassDomain& domain = domains[increment.classIndex];
        if (increment.nextCount < domain.maximum) {
            const std::uint32_t next = increment.nextCount + step;
            queue.push(
                {classObjective(problem, domain, next) -
                     classObjective(problem, domain, increment.nextCount),
                 increment.classIndex, next});
        }
    }
    return curve;
}

struct SideAllocation {
    std::uint64_t minimumTotal = 0;
    std::uint64_t maximumTotal = 0;
    AllocationCurve unit;
    AllocationCurve even;
    std::vector<double> costs;
    std::vector<std::uint32_t> unitActions;
    std::vector<std::uint32_t> evenActions;

    bool reachable(std::uint64_t total) const noexcept {
        if (total < minimumTotal || total > maximumTotal) return false;
        return std::isfinite(
            costs[static_cast<std::size_t>(total - minimumTotal)]);
    }
};

struct SideAllocationResult {
    std::optional<SideAllocation> value;
    std::optional<IntervalFailure> failure;
};

constexpr std::uint64_t kMaximumSumStates = 2'000'000;
constexpr std::uint64_t kMaximumMixedTransitions = 4'000'000;
constexpr double kObjectiveTieTolerance = 1e-12;

bool equivalentObjective(double left, double right) noexcept {
    if (!std::isfinite(left) || !std::isfinite(right)) return left == right;
    const double scale =
        std::max({1.0, std::abs(left), std::abs(right)});
    return std::abs(left - right) <= kObjectiveTieTolerance * scale;
}

bool betterObjective(double candidate, double incumbent) noexcept {
    return candidate < incumbent &&
        !equivalentObjective(candidate, incumbent);
}

SideAllocationResult buildSideAllocation(
    const IntervalProblem& problem,
    const std::vector<IntervalClassDomain>& domains,
    const std::vector<std::size_t>& classIndices,
    const std::vector<StableId>& subjects) {
    SideAllocationResult result;
    SideAllocation side;
    std::vector<std::size_t> unitClasses;
    std::vector<std::size_t> evenClasses;
    std::uint64_t unitCount = 0;
    std::uint64_t evenCount = 0;
    for (std::size_t classIndex : classIndices) {
        const IntervalClassDomain& domain = domains[classIndex];
        if (side.minimumTotal >
            std::numeric_limits<std::uint64_t>::max() - domain.start) {
            result.failure = IntervalFailure{
                "interval.sum_complexity_exceeded",
                "a sum total exceeds the bounded integer representation",
                subjects};
            return result;
        }
        side.minimumTotal += domain.start;
        if (domain.step == 1) {
            unitClasses.push_back(classIndex);
            unitCount += domain.maximum - domain.start;
        } else {
            evenClasses.push_back(classIndex);
            evenCount += (domain.maximum - domain.start) / 2U;
        }
        if (unitCount > kMaximumSumStates ||
            evenCount > kMaximumSumStates) {
            result.failure = IntervalFailure{
                "interval.sum_complexity_exceeded",
                "a sum side exceeds the bounded exact state budget",
                subjects};
            return result;
        }
    }
    if (unitCount > kMaximumSumStates ||
        evenCount > kMaximumSumStates ||
        unitCount > kMaximumSumStates -
                std::min(evenCount * 2U, kMaximumSumStates)) {
        result.failure = IntervalFailure{
            "interval.sum_complexity_exceeded",
            "a sum side exceeds the bounded exact state budget", subjects};
        return result;
    }
    side.maximumTotal = side.minimumTotal + unitCount + evenCount * 2U;
    const std::uint64_t stateCount =
        side.maximumTotal - side.minimumTotal + 1U;
    if (stateCount > kMaximumSumStates ||
        (unitCount + 1U) >
            kMaximumMixedTransitions / (evenCount + 1U)) {
        result.failure = IntervalFailure{
            "interval.sum_complexity_exceeded",
            "mixed parity sum allocation exceeds the bounded exact transition budget",
            subjects};
        return result;
    }

    side.unit =
        buildAllocationCurve(problem, domains, unitClasses, 1U);
    side.even =
        buildAllocationCurve(problem, domains, evenClasses, 2U);
    const double infinity = std::numeric_limits<double>::infinity();
    side.costs.assign(static_cast<std::size_t>(stateCount), infinity);
    side.unitActions.assign(static_cast<std::size_t>(stateCount), 0U);
    side.evenActions.assign(static_cast<std::size_t>(stateCount), 0U);
    for (std::size_t unit = 0; unit < side.unit.costs.size(); ++unit) {
        for (std::size_t even = 0; even < side.even.costs.size(); ++even) {
            const std::size_t offset = unit + even * 2U;
            const double objective =
                side.unit.costs[unit] + side.even.costs[even];
            if (betterObjective(objective, side.costs[offset]) ||
                (equivalentObjective(objective, side.costs[offset]) &&
                 even < side.evenActions[offset])) {
                side.costs[offset] = objective;
                side.unitActions[offset] =
                    static_cast<std::uint32_t>(unit);
                side.evenActions[offset] =
                    static_cast<std::uint32_t>(even);
            }
        }
    }
    result.value = std::move(side);
    return result;
}

void applySideAllocation(
    const SideAllocation& side, std::uint64_t total,
    const std::vector<IntervalClassDomain>& domains,
    const std::vector<std::size_t>& classIndices,
    std::vector<std::uint32_t>& classCounts) {
    for (std::size_t classIndex : classIndices) {
        classCounts[classIndex] = domains[classIndex].start;
    }
    const std::size_t offset =
        static_cast<std::size_t>(total - side.minimumTotal);
    for (std::size_t action = 0;
         action < side.unitActions[offset]; ++action) {
        ++classCounts[side.unit.actions[action]];
    }
    for (std::size_t action = 0;
         action < side.evenActions[offset]; ++action) {
        classCounts[side.even.actions[action]] += 2U;
    }
}

struct SumPlan {
    std::vector<std::size_t> lhsClasses;
    std::vector<std::size_t> rhsClasses;
    std::vector<StableId> subjects;
};

}  // namespace

std::optional<std::uint32_t> IntervalSolution::find(
    StableId boundary) const noexcept {
    const auto found = std::lower_bound(
        counts.begin(), counts.end(), boundary,
        [](const SolvedInterval& interval, StableId id) {
            return interval.boundaryId < id;
        });
    if (found == counts.end() || found->boundaryId != boundary) {
        return std::nullopt;
    }
    return found->count;
}

IntervalSolveResult solveIntervals(const IntervalProblem& problem,
                                   const SamplingConfiguration& configuration) {
    if (!validConfiguration(configuration)) {
        return solveFailure("interval.invalid_configuration",
                            "sampling configuration is not finite and positive");
    }
    StableId previous;
    std::map<StableId, std::size_t> variableIndex;
    for (std::size_t index = 0; index < problem.variables.size(); ++index) {
        const IntervalVariable& variable = problem.variables[index];
        if (!variable.boundaryId.valid() ||
            variable.boundaryId.kind != StableIdKind::Boundary ||
            (previous.valid() && !(previous < variable.boundaryId)) ||
            !std::isfinite(variable.desired) || variable.desired < 0.0 ||
            variable.minimum < 1 ||
            (variable.exact && *variable.exact < 1)) {
            return solveFailure(
                "interval.invalid_problem",
                "variables must be unique ordered boundaries with valid domains");
        }
        previous = variable.boundaryId;
        variableIndex.emplace(variable.boundaryId, index);
    }

    std::vector<std::size_t> parent(problem.variables.size());
    std::iota(parent.begin(), parent.end(), std::size_t{0});
    const auto findRoot = [&parent](std::size_t index) {
        while (parent[index] != index) {
            parent[index] = parent[parent[index]];
            index = parent[index];
        }
        return index;
    };
    for (const IntervalEquality& equality : problem.equalities) {
        if (equality.boundaryIds.empty()) {
            return solveFailure("interval.invalid_problem",
                                "an equality constrains no boundary");
        }
        const auto first = variableIndex.find(equality.boundaryIds.front());
        if (first == variableIndex.end()) {
            return solveFailure("interval.invalid_problem",
                                "an equality references an unknown boundary");
        }
        for (StableId boundary : equality.boundaryIds) {
            const auto found = variableIndex.find(boundary);
            if (found == variableIndex.end()) {
                return solveFailure(
                    "interval.invalid_problem",
                    "an equality references an unknown boundary");
            }
            std::size_t rootA = findRoot(first->second);
            std::size_t rootB = findRoot(found->second);
            if (rootA == rootB) continue;
            if (rootB < rootA) std::swap(rootA, rootB);
            parent[rootB] = rootA;
        }
    }

    std::map<std::size_t, std::vector<std::size_t>> membersByRoot;
    for (std::size_t index = 0; index < problem.variables.size(); ++index) {
        membersByRoot[findRoot(index)].push_back(index);
    }

    const std::uint32_t cap = configuration.maximumSegmentCount;
    std::vector<IntervalClassDomain> domains;
    domains.reserve(membersByRoot.size());
    std::vector<std::size_t> classByVariable(
        problem.variables.size(), std::size_t{0});
    for (const auto& entry : membersByRoot) {
        const std::vector<std::size_t>& members = entry.second;
        std::uint32_t minimum = 1;
        bool requireEven = false;
        std::optional<std::uint32_t> exact;
        for (std::size_t member : members) {
            minimum = std::max(minimum, problem.variables[member].minimum);
            requireEven = requireEven || problem.variables[member].requireEven;
            if (problem.variables[member].exact) {
                if (exact && *exact != *problem.variables[member].exact) {
                    std::vector<StableId> subjects;
                    for (std::size_t item : members) {
                        subjects.push_back(
                            problem.variables[item].boundaryId);
                    }
                    return solveFailure(
                        "interval.exact_conflict",
                        "an equality class contains incompatible exact counts",
                        std::move(subjects));
                }
                exact = problem.variables[member].exact;
            }
        }
        if (exact && *exact < minimum) {
            std::vector<StableId> subjects;
            for (std::size_t member : members) {
                subjects.push_back(problem.variables[member].boundaryId);
            }
            return solveFailure(
                "interval.exact_below_minimum",
                "an exact count is below its certified minimum",
                std::move(subjects));
        }
        if (exact && requireEven && *exact % 2 != 0) {
            std::vector<StableId> subjects;
            for (std::size_t member : members) {
                subjects.push_back(problem.variables[member].boundaryId);
            }
            return solveFailure(
                "interval.exact_parity_conflict",
                "an exact count violates an even-parity constraint",
                std::move(subjects));
        }
        std::uint32_t start = minimum;
        if (requireEven && start % 2 != 0) ++start;
        if (exact) start = *exact;
        if (start > cap) {
            std::vector<StableId> subjects;
            for (std::size_t member : members) {
                subjects.push_back(problem.variables[member].boundaryId);
            }
            return solveFailure(
                "interval.count_exceeds_maximum",
                "an equality class minimum exceeds the configured segment cap",
                std::move(subjects));
        }

        const std::uint32_t step = requireEven ? 2U : 1U;
        std::uint32_t maximum = exact ? *exact : cap;
        if (requireEven && maximum % 2U != 0) --maximum;
        IntervalClassDomain domain;
        domain.members = members;
        domain.start = start;
        domain.maximum = maximum;
        domain.step = step;
        const std::size_t classIndex = domains.size();
        for (std::size_t member : members) {
            classByVariable[member] = classIndex;
        }
        domains.push_back(std::move(domain));
    }

    std::vector<SumPlan> sumPlans;
    std::vector<std::optional<std::size_t>> sumOwner(domains.size());
    for (const IntervalSum& sum : problem.sums) {
        if (sum.lhs.empty() || sum.rhs.empty()) {
            return solveFailure("interval.invalid_problem",
                                "a sum requires two non-empty sides");
        }
        SumPlan plan;
        const auto hasRepeatedBoundary = [](std::vector<StableId> side) {
            std::sort(side.begin(), side.end());
            return std::adjacent_find(side.begin(), side.end()) != side.end();
        };
        if (hasRepeatedBoundary(sum.lhs) || hasRepeatedBoundary(sum.rhs)) {
            return solveFailure(
                "interval.invalid_problem",
                "a sum side cannot repeat a boundary reference");
        }
        plan.subjects.insert(plan.subjects.end(), sum.lhs.begin(),
                             sum.lhs.end());
        plan.subjects.insert(plan.subjects.end(), sum.rhs.begin(),
                             sum.rhs.end());
        std::sort(plan.subjects.begin(), plan.subjects.end());
        plan.subjects.erase(
            std::unique(plan.subjects.begin(), plan.subjects.end()),
            plan.subjects.end());
        std::map<std::size_t, int> coefficients;
        const auto collect = [&](const std::vector<StableId>& boundaries,
                                 int sign) -> bool {
            for (StableId boundary : boundaries) {
                const auto found = variableIndex.find(boundary);
                if (found == variableIndex.end()) return false;
                coefficients[classByVariable[found->second]] += sign;
            }
            return true;
        };
        if (!collect(sum.lhs, 1) || !collect(sum.rhs, -1)) {
            return solveFailure(
                "interval.invalid_problem",
                "a sum references an unknown boundary", plan.subjects);
        }
        for (const auto& [classIndex, coefficient] : coefficients) {
            if (coefficient > 1 || coefficient < -1) {
                return solveFailure(
                    "interval.sum_aliasing_unsupported",
                    "an equality class occurs more than once on one net sum side",
                    plan.subjects);
            }
            if (coefficient == 1) plan.lhsClasses.push_back(classIndex);
            if (coefficient == -1) plan.rhsClasses.push_back(classIndex);
        }
        if (plan.lhsClasses.empty() && plan.rhsClasses.empty()) continue;
        if (plan.lhsClasses.empty() || plan.rhsClasses.empty()) {
            return solveFailure(
                "interval.sum_infeasible",
                "positive interval counts cannot satisfy an empty net sum side",
                plan.subjects);
        }
        const std::size_t planIndex = sumPlans.size();
        for (std::size_t classIndex : plan.lhsClasses) {
            if (sumOwner[classIndex]) {
                return solveFailure(
                    "interval.coupled_sum_unsupported",
                    "a boundary class participates in multiple non-trivial sums",
                    plan.subjects);
            }
            sumOwner[classIndex] = planIndex;
        }
        for (std::size_t classIndex : plan.rhsClasses) {
            if (sumOwner[classIndex]) {
                return solveFailure(
                    "interval.coupled_sum_unsupported",
                    "a boundary class participates in multiple non-trivial sums",
                    plan.subjects);
            }
            sumOwner[classIndex] = planIndex;
        }
        sumPlans.push_back(std::move(plan));
    }

    std::vector<std::uint32_t> classCounts(domains.size(), 0U);
    for (const SumPlan& plan : sumPlans) {
        const SideAllocationResult lhs = buildSideAllocation(
            problem, domains, plan.lhsClasses, plan.subjects);
        if (!lhs.value) {
            IntervalSolveResult failed;
            failed.failure = lhs.failure;
            return failed;
        }
        const SideAllocationResult rhs = buildSideAllocation(
            problem, domains, plan.rhsClasses, plan.subjects);
        if (!rhs.value) {
            IntervalSolveResult failed;
            failed.failure = rhs.failure;
            return failed;
        }
        const std::uint64_t firstTotal =
            std::max(lhs.value->minimumTotal, rhs.value->minimumTotal);
        const std::uint64_t lastTotal =
            std::min(lhs.value->maximumTotal, rhs.value->maximumTotal);
        bool found = false;
        double bestObjective = 0.0;
        std::uint64_t bestTotal = 0;
        if (firstTotal <= lastTotal) {
            for (std::uint64_t total = firstTotal;; ++total) {
                if (!lhs.value->reachable(total) ||
                    !rhs.value->reachable(total)) {
                    if (total == lastTotal) break;
                    continue;
                } else {
                    const double objective =
                        lhs.value->costs[static_cast<std::size_t>(
                            total - lhs.value->minimumTotal)] +
                        rhs.value->costs[static_cast<std::size_t>(
                            total - rhs.value->minimumTotal)];
                    if (!found ||
                        betterObjective(objective, bestObjective)) {
                        found = true;
                        bestObjective = objective;
                        bestTotal = total;
                    }
                }
                if (total == lastTotal) break;
            }
        }
        if (!found) {
            return solveFailure(
                "interval.sum_infeasible",
                "no bounded integer assignment satisfies a sum constraint",
                plan.subjects);
        }
        applySideAllocation(*lhs.value, bestTotal, domains,
                            plan.lhsClasses, classCounts);
        applySideAllocation(*rhs.value, bestTotal, domains,
                            plan.rhsClasses, classCounts);
    }

    for (std::size_t classIndex = 0; classIndex < domains.size();
         ++classIndex) {
        if (sumOwner[classIndex]) continue;
        const IntervalClassDomain& domain = domains[classIndex];
        double bestObjective = 0.0;
        std::uint32_t bestCount = 0;
        bool haveBest = false;
        for (std::uint32_t candidate = domain.start;;) {
            const double objective =
                classObjective(problem, domain, candidate);
            if (!haveBest || objective < bestObjective) {
                bestObjective = objective;
                bestCount = candidate;
                haveBest = true;
            } else if (objective > bestObjective) {
                break;
            }
            if (candidate == domain.maximum ||
                candidate > domain.maximum -
                    std::min(domain.step, domain.maximum)) {
                break;
            }
            candidate += domain.step;
        }
        classCounts[classIndex] = bestCount;
    }

    std::vector<SolvedInterval> counts(problem.variables.size());
    for (std::size_t index = 0; index < problem.variables.size(); ++index) {
        counts[index] = {problem.variables[index].boundaryId,
                         classCounts[classByVariable[index]]};
    }

    for (std::size_t index = 0; index < problem.variables.size(); ++index) {
        const IntervalVariable& variable = problem.variables[index];
        const std::uint32_t count = counts[index].count;
        if (count < variable.minimum || count > cap ||
            (variable.requireEven && count % 2 != 0) ||
            (variable.exact && count != *variable.exact)) {
            return solveFailure("interval.internal_self_check_failed",
                                "a solved count violates its variable domain");
        }
    }
    for (const IntervalEquality& equality : problem.equalities) {
        const std::uint32_t expected =
            counts[variableIndex.at(equality.boundaryIds.front())].count;
        for (StableId boundary : equality.boundaryIds) {
            if (counts[variableIndex.at(boundary)].count != expected) {
                return solveFailure(
                    "interval.internal_self_check_failed",
                    "a solved equality class contains differing counts");
            }
        }
    }
    for (const IntervalSum& sum : problem.sums) {
        std::uint64_t lhs = 0;
        std::uint64_t rhs = 0;
        for (StableId boundary : sum.lhs) {
            lhs += counts[variableIndex.at(boundary)].count;
        }
        for (StableId boundary : sum.rhs) {
            rhs += counts[variableIndex.at(boundary)].count;
        }
        if (lhs != rhs) {
            return solveFailure(
                "interval.internal_self_check_failed",
                "a solved assignment violates a sum constraint");
        }
    }

    IntervalSolveResult result;
    result.solution = IntervalSolution{std::move(counts)};
    return result;
}

SegmentCountResult circularArcSegmentCount(
    double radius, double spanRadians, bool fullCircle,
    const SamplingConfiguration& configuration) {
    if (!validConfiguration(configuration)) {
        return countFailure("interval.invalid_configuration",
                            "sampling configuration is not finite and positive");
    }
    if (!std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(spanRadians) || spanRadians <= 0.0) {
        return countFailure(
            "interval.invalid_problem",
            "a circular span requires a finite positive radius and span");
    }
    double step = configuration.normalAngleToleranceRadians;
    if (configuration.chordTolerance < radius) {
        const double cosine = std::clamp(
            1.0 - configuration.chordTolerance / radius, -1.0, 1.0);
        step = std::min(step, 2.0 * std::acos(cosine));
    }
    if (!std::isfinite(step) || step <= 0.0) {
        return countFailure("interval.invalid_configuration",
                            "sampling bounds produce no positive angular step");
    }
    const double demanded = std::ceil(spanRadians / step);
    double required = std::max(1.0, demanded);
    if (fullCircle) {
        required = std::max(
            required,
            static_cast<double>(configuration.minimumClosedCurveSegments));
    }
    if (required > static_cast<double>(configuration.maximumSegmentCount)) {
        return countFailure(
            "interval.count_exceeds_maximum",
            "the circular span requires more segments than the configured cap");
    }
    SegmentCountResult result;
    result.count = static_cast<std::uint32_t>(required);
    return result;
}

}  // namespace weft
