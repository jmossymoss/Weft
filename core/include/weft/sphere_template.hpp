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

struct SphereWallConfiguration {
    double maximumChordDeviation = 5e-5;
    double maximumNormalDeviationRadians = 0.017453292519943295;
    std::uint32_t azimuthIntervals = 16;
};

struct SphereWallValidationEvidence {
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

struct SphereWallFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct SphereWallResult {
    std::optional<PlanarCdtMesh> value;
    std::vector<SphereWallValidationEvidence> validation;
    std::optional<SphereWallFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Boundary-exact full sphere: two polar singular stations plus a meridian
// seam. Longitude columns are generated as certified interior stations that
// consume matching seam samples on column 0.
SphereWallResult buildFullSphereWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const SphereWallConfiguration& configuration = {},
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
