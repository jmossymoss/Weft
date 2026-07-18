#pragma once

#include "weft/planar_cdt.hpp"
#include "weft/secure_reconnaissance.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct MappedPatchConfiguration {
    double maximumChordDeviation = 5e-5;
    double maximumNormalDeviationRadians = 0.017453292519943295;
    std::uint32_t uIntervals = 8;
    std::uint32_t vIntervals = 8;
};

struct MappedPatchValidationEvidence {
    std::string code;
    std::size_t expected = 0;
    std::size_t checked = 0;
    std::size_t skipped = 0;
    std::size_t failed = 0;

    bool complete() const noexcept {
        return checked == expected && skipped == 0 && failed == 0 &&
            expected != 0 && checked != 0;
    }
};

struct MappedPatchFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct MappedPatchResult {
    std::optional<PlanarCdtMesh> value;
    std::vector<MappedPatchValidationEvidence> validation;
    std::optional<MappedPatchFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Boundary-exact UV grid for a four-sided mapped.four_sided_candidate face.
MappedPatchResult buildMappedFourSidedPatch(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const MappedPatchConfiguration& configuration = {},
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
