#include "weft/interval_solver.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>
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
    if (!problem.sums.empty()) {
        std::vector<StableId> subjects;
        for (const IntervalSum& sum : problem.sums) {
            subjects.insert(subjects.end(), sum.lhs.begin(), sum.lhs.end());
            subjects.insert(subjects.end(), sum.rhs.begin(), sum.rhs.end());
        }
        std::sort(subjects.begin(), subjects.end());
        subjects.erase(std::unique(subjects.begin(), subjects.end()),
                       subjects.end());
        return solveFailure(
            "interval.unsupported_constraint",
            "sum constraints require the later bounded exact solver",
            std::move(subjects));
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
    std::vector<SolvedInterval> counts(problem.variables.size());
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
        double bestObjective = 0.0;
        std::uint32_t bestCount = 0;
        bool haveBest = false;
        for (std::uint32_t candidate = start;;) {
            double objective = 0.0;
            for (std::size_t member : members) {
                objective += std::abs(
                    static_cast<double>(candidate) -
                    problem.variables[member].desired);
            }
            if (!haveBest || objective < bestObjective) {
                bestObjective = objective;
                bestCount = candidate;
                haveBest = true;
            } else if (objective > bestObjective) {
                break;
            }
            if (exact) break;
            if (candidate > cap - std::min(step, cap)) break;
            candidate += step;
            if (candidate > cap) break;
        }
        for (std::size_t member : members) {
            counts[member] =
                {problem.variables[member].boundaryId, bestCount};
        }
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
