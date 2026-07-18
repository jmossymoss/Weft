#pragma once

#include "weft/secure_core.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct SamplingConfiguration {
    double chordTolerance = 5e-5;
    double normalAngleToleranceRadians = 0.017453292519943295;
    std::uint32_t minimumClosedCurveSegments = 16;
    std::uint32_t maximumSegmentCount = 4096;
};

struct IntervalVariable {
    StableId boundaryId;
    double desired = 1.0;
    std::uint32_t minimum = 1;
    bool requireEven = false;
    std::optional<std::uint32_t> exact;
};

struct IntervalEquality {
    std::vector<StableId> boundaryIds;
};

struct IntervalSum {
    std::vector<StableId> lhs;
    std::vector<StableId> rhs;
};

struct IntervalProblem {
    std::vector<IntervalVariable> variables;
    std::vector<IntervalEquality> equalities;
    std::vector<IntervalSum> sums;
};

struct SolvedInterval {
    StableId boundaryId;
    std::uint32_t count = 0;

    bool operator==(const SolvedInterval& other) const noexcept {
        return boundaryId == other.boundaryId && count == other.count;
    }
};

struct IntervalSolution {
    std::vector<SolvedInterval> counts;

    std::optional<std::uint32_t> find(StableId boundary) const noexcept;
};

struct IntervalFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct IntervalSolveResult {
    std::optional<IntervalSolution> solution;
    std::optional<IntervalFailure> failure;

    explicit operator bool() const noexcept { return solution.has_value(); }
};

struct SegmentCountResult {
    std::optional<std::uint32_t> count;
    std::optional<IntervalFailure> failure;

    explicit operator bool() const noexcept { return count.has_value(); }
};

// Exact over equality/minimum/even-parity/fixed-count constraints, independent
// chain sums, and one bounded connected coupled-sum component. Every feasible
// count is capped. Coupled search minimizes global L1 cost and breaks ties by
// the stable class order; additional components, coefficient multiplicity, and
// over-budget systems refuse by stable code rather than being approximated.
IntervalSolveResult solveIntervals(
    const IntervalProblem& problem,
    const SamplingConfiguration& configuration = {});

// Analytic circle bound: chord sagitta r(1-cos(step/2)) and normal turn both
// bound the angular step; ceiling always errs toward more segments.
SegmentCountResult circularArcSegmentCount(
    double radius, double spanRadians, bool fullCircle,
    const SamplingConfiguration& configuration = {});

// Conservative elliptical-arc demand: uses the major radius as a circle
// bound (errs toward denser sampling on the sharper minor-axis region).
SegmentCountResult ellipticalArcSegmentCount(
    double majorRadius, double minorRadius, double spanRadians,
    bool fullEllipse,
    const SamplingConfiguration& configuration = {});

constexpr std::uint32_t lineSegmentCount() noexcept { return 1; }

}  // namespace weft
