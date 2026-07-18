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

struct TorusWallConfiguration {
    double maximumChordDeviation = 5e-5;
    double maximumNormalDeviationRadians = 0.017453292519943295;
    std::uint32_t majorIntervals = 16;
    std::uint32_t minorIntervals = 12;
};

struct TorusWallValidationEvidence {
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

struct TorusWallFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct TorusWallResult {
    std::optional<PlanarCdtMesh> value;
    std::vector<TorusWallValidationEvidence> validation;
    std::optional<TorusWallFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Boundary-exact full torus: dual-periodic UV grid consuming both seam
// sample sequences.
TorusWallResult buildFullTorusWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const TorusWallConfiguration& configuration = {},
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
