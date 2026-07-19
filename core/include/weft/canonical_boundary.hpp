#pragma once

#include "weft/interval_solver.hpp"
#include "weft/secure_reconnaissance.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
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

struct PeriodicUvClosureWitness {
    StableId edge;
    StableId coedge;
    StableId face;
    std::optional<StableId> sourceEdge;
    std::optional<StableId> sourceFace;
    TopologyOrientation traversalOrientation = TopologyOrientation::Forward;
    std::size_t axis = 0;
    double period = 0.0;
    double firstUv = 0.0;
    double lastUv = 0.0;
    double firstLiftedUv = 0.0;
    double lastLiftedUv = 0.0;
    double closingLiftedUv = 0.0;
    std::int64_t selectedFirstLift = 0;
    std::int64_t selectedLastLift = 0;
    std::int64_t periodsCrossed = 0;
};

enum class CriticalParameterEventKind {
    DomainEndpoint,
    ContactCritical,
    PeriodicSeam,
    MonotonicExtremum,
    Singular,
};

struct CriticalParameterEvent {
    CriticalParameterEventKind kind = CriticalParameterEventKind::DomainEndpoint;
    StableId edge;
    std::optional<StableId> coedge;
    std::optional<StableId> face;
    std::optional<StableId> sourceEdge;
    std::optional<StableId> sourceFace;
    std::optional<std::size_t> axis;
    double curveParameter = 0.0;
    std::uint32_t sampleOrdinal = 0;
    std::string detectionCode;
};

struct CanonicalBoundary {
    StableId edge;
    StableId boundaryId;
    bool closed = false;
    std::uint32_t intervalCount = 0;
    std::vector<CanonicalBoundarySample> samples;
    std::vector<PeriodicUvClosureWitness> periodicClosures;
    std::vector<CriticalParameterEvent> criticalEvents;
};

struct CanonicalBoundaryConfiguration {
    double minimumDiscrepancyTolerance = 1e-9;
    double maximumDiscrepancyTolerance = 1e-3;
    double sourceToleranceScale = 2.0;
    double periodicLiftAmbiguityTolerance = 1e-6;
    // Industrial preview: skip expensive soft discrepancy proofs and report
    // edge progress (done/total working edges).
    bool previewFast = false;
    std::function<void(int /*done*/, int /*total*/)> progress;
};

struct CanonicalBoundaryReport {
    std::size_t expectedEdges = 0;
    std::size_t checkedEdges = 0;
    std::size_t expectedSamples = 0;
    std::size_t checkedSamples = 0;
    std::size_t expectedUvUses = 0;
    std::size_t checkedUvUses = 0;
    std::size_t expectedVertexCurveChecks = 0;
    std::size_t checkedVertexCurveChecks = 0;
    std::size_t expectedPeriodicClosures = 0;
    std::size_t checkedPeriodicClosures = 0;
    std::size_t expectedCriticalEvents = 0;
    std::size_t checkedCriticalEvents = 0;
    std::size_t failed = 0;

    bool complete() const noexcept {
        return expectedEdges != 0 && checkedEdges == expectedEdges &&
            checkedSamples == expectedSamples &&
            checkedUvUses == expectedUvUses &&
            checkedVertexCurveChecks == expectedVertexCurveChecks &&
            checkedPeriodicClosures == expectedPeriodicClosures &&
            expectedCriticalEvents != 0 &&
            checkedCriticalEvents == expectedCriticalEvents &&
            failed == 0;
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

// Returns a cyclic permutation aligning two equally sampled coaxial rings.
// Opposite rim winding (common on open cylinder bands) is handled by a
// reflection-aware permutation. Non-uniform/twisted pairs fail closed.
AzimuthRegistrationResult azimuthRegistration(
    const std::vector<std::array<double, 3>>& ringA,
    const std::vector<std::array<double, 3>>& ringB);

struct PeriodicUvClosureResult {
    std::optional<PeriodicUvClosureWitness> value;
    std::optional<CanonicalBoundaryFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Proves that a closed coedge UV sequence returns to the first sample through
// an unambiguous integer-period covering-space step. Used by canonical
// boundary construction and by adversarial tamper checks.
PeriodicUvClosureResult provePeriodicUvClosure(
    PeriodicUvClosureWitness seed,
    double ambiguityTolerance = 1e-6);

}  // namespace weft
