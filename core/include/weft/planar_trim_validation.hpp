#pragma once

#include "weft/canonical_boundary.hpp"
#include "weft/geometric_predicates.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace weft {

inline constexpr std::uint64_t InvalidCanonicalVertexIndex =
    std::numeric_limits<std::uint64_t>::max();

enum class PlanarTrimLoopRole {
    Outer,
    Hole,
};

enum class PlanarTrimLoopOrientation {
    CounterClockwise,
    Clockwise,
};

struct PlanarTrimBoundaryUse {
    SampleId sample;
    StableId workingEdge;
    std::optional<StableId> sourceEdge;
    StableId coedge;
    PredicatePoint2 uv{};
    double measuredCurveOnSurfaceDiscrepancy = 0.0;
    double allowedCurveOnSurfaceDiscrepancy = 0.0;
    std::optional<PcurveRef> representation;
};

// Provenance for generated cylinder interior stations that are not owned by a
// canonical boundary sample. Seam samples at the same (u,v) may still attach
// as ordinary boundaryUses on the same vertex.
struct CylinderInteriorStation {
    StableId workingFace;
    std::optional<StableId> sourceFace;
    PredicatePoint2 uv{};
    std::array<double, 3> position{};
    std::uint32_t axialRing = 0;
    std::uint32_t azimuthColumn = 0;
};

struct PlanarTrimVertex {
    std::vector<PlanarTrimBoundaryUse> boundaryUses;
    std::optional<CylinderInteriorStation> cylinderInterior;
    std::uint64_t canonicalVertexIndex = InvalidCanonicalVertexIndex;
    PredicatePoint2 uv{};
};

// Vertices do not repeat the first vertex at the end. The final-to-first edge
// is implicit, and `closed` proves that the topology assembler observed a
// closed wire rather than merely receiving enough points to form a polygon.
struct PlanarTrimLoop {
    StableId wire;
    PlanarTrimLoopRole declaredRole = PlanarTrimLoopRole::Outer;
    bool closed = false;
    bool reversedForCanonicalCdt = false;
    std::vector<PlanarTrimVertex> vertices;
};

struct PlanarTrimDomain {
    StableId face;
    std::optional<StableId> sourceFace;
    std::vector<PlanarTrimLoop> loops;
    // When true, CDT skips planar nesting/orientation proofs and triangulates
    // the assembled UV loops directly (cylinder/freeform/sphere UV trims).
    bool allowCurvedUv = false;
};

struct TrimValidationEvidence {
    std::string code;
    std::size_t expected = 0;
    std::size_t checked = 0;
    std::size_t skipped = 0;
    std::size_t failed = 0;

    bool complete() const noexcept {
        return checked == expected && skipped == 0 && failed == 0 &&
            (expected == 0 || checked != 0);
    }
};

struct TrimValidationDiagnostic {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct ValidatedPlanarTrimLoop {
    StableId wire;
    PlanarTrimLoopRole role = PlanarTrimLoopRole::Outer;
    PlanarTrimLoopOrientation orientation =
        PlanarTrimLoopOrientation::CounterClockwise;
    std::size_t nestingDepth = 0;
    std::vector<PlanarTrimVertex> vertices;
};

struct ValidatedPlanarTrimDomain {
    StableId face;
    std::optional<StableId> sourceFace;
    std::vector<ValidatedPlanarTrimLoop> loops;
};

struct PlanarTrimValidationResult {
    std::optional<ValidatedPlanarTrimDomain> value;
    std::vector<TrimValidationEvidence> evidence;
    std::vector<TrimValidationDiagnostic> diagnostics;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Validates an already lifted planar UV domain without changing sample order
// or coordinates. The canonical CDT convention is one counter-clockwise
// outer loop and zero or more clockwise, directly contained holes. Boundary
// touching, crossing, nested holes, multiple outers, and vacuous input fail
// closed with stable diagnostic codes.
PlanarTrimValidationResult validatePlanarTrimDomain(
    const PlanarTrimDomain& domain,
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
