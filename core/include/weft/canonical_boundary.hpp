#pragma once

#include "weft/interval_solver.hpp"
#include "weft/secure_reconnaissance.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct SampleId {
    StableId boundary;
    std::uint32_t ordinal = 0;

    bool valid() const noexcept {
        return boundary.kind == StableIdKind::Boundary && boundary.valid();
    }
    bool operator==(const SampleId& other) const noexcept {
        return boundary == other.boundary && ordinal == other.ordinal;
    }
};

enum class BoundaryUvMappingKind {
    StoredPcurve,
    DerivedPlanarProjection,
};

struct CoedgeUvUse {
    StableId face;
    std::optional<StableId> sourceFace;
    StableId coedge;
    std::optional<PcurveRef> representation;
    BoundaryUvMappingKind mappingKind = BoundaryUvMappingKind::StoredPcurve;
    std::array<double, 2> uv{};
    std::array<std::int64_t, 2> periodicLift{};
    std::array<double, 2> liftedUv{};
    TopologyOrientation traversalOrientation = TopologyOrientation::Forward;
    double measuredCurveOnSurfaceDiscrepancy = 0.0;
    double allowedCurveOnSurfaceDiscrepancy = 0.0;
};

struct CanonicalBoundarySample {
    SampleId id;
    StableId workingEdge;
    std::optional<StableId> sourceEdge;
    double curveParameter = 0.0;
    std::array<double, 3> position{};
    std::uint64_t canonicalVertexIndex = 0;
    std::vector<CoedgeUvUse> faceUses;
};

struct CanonicalBoundary {
    StableId edge;
    StableId boundaryId;
    bool closed = false;
    std::uint32_t intervalCount = 0;
    std::vector<CanonicalBoundarySample> samples;
};

struct CanonicalBoundaryConfiguration {
    double minimumDiscrepancyTolerance = 1e-9;
    double maximumDiscrepancyTolerance = 1e-3;
    double sourceToleranceScale = 2.0;
};

struct CanonicalBoundaryReport {
    std::size_t expectedEdges = 0;
    std::size_t checkedEdges = 0;
    std::size_t expectedSamples = 0;
    std::size_t checkedSamples = 0;
    std::size_t expectedUvUses = 0;
    std::size_t checkedUvUses = 0;
    std::size_t failed = 0;

    bool complete() const noexcept {
        return expectedEdges != 0 && checkedEdges == expectedEdges &&
            checkedSamples == expectedSamples &&
            checkedUvUses == expectedUvUses && failed == 0;
    }
};

struct CanonicalBoundaryFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct CanonicalBoundarySet {
    std::vector<CanonicalBoundary> boundaries;
    CanonicalBoundaryReport validation;
    std::uint64_t canonicalVertexCount = 0;

    const CanonicalBoundary* find(StableId edge) const noexcept;
};

struct CanonicalBoundaryBuildResult {
    std::optional<CanonicalBoundarySet> value;
    std::optional<CanonicalBoundaryFailure> failure;
    CanonicalBoundaryReport validation;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Builds all working-edge sequences atomically. A single edge/use failure
// returns no partial boundary set.
CanonicalBoundaryBuildResult buildCanonicalBoundaries(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const IntervalSolution& intervals,
    const CanonicalBoundaryConfiguration& configuration = {});

struct AzimuthRegistrationResult {
    std::optional<std::vector<std::uint32_t>> permutation;
    std::optional<CanonicalBoundaryFailure> failure;

    explicit operator bool() const noexcept { return permutation.has_value(); }
};

// Returns a pure cyclic permutation aligning two equally sampled coaxial
// rings. Reflections and non-uniform/twisted pairs fail closed.
AzimuthRegistrationResult azimuthRegistration(
    const std::vector<std::array<double, 3>>& ringA,
    const std::vector<std::array<double, 3>>& ringB);

}  // namespace weft
