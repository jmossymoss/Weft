#pragma once

#include "weft/planar_trim_validation.hpp"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct PlanarTrimAssemblyEvidence {
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

struct PlanarTrimAssemblyFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct PlanarTrimAssemblyResult {
    std::optional<PlanarTrimDomain> value;
    PlanarTrimValidationResult validation;
    std::vector<PlanarTrimAssemblyEvidence> evidence;
    std::optional<PlanarTrimAssemblyFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Assembles one planar working face from ordered coedges and canonical edge
// sequences. Shared topological corners merge by canonical vertex index while
// retaining every incident boundary-use record. Any missing/ambiguous UV use,
// non-closing wire, or UV disagreement fails atomically.
PlanarTrimAssemblyResult assemblePlanarTrimDomain(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates(),
    // HardSurfaceFloor last resort: emit allowCurvedUv domain when plane
    // nesting/self-intersect validation fails (soft residual, not G1 default).
    bool softPlaneFallback = false);

}  // namespace weft
