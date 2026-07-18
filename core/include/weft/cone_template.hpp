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

struct ConeWallConfiguration {
    double maximumChordDeviation = 5e-5;
    double maximumNormalDeviationRadians = 0.017453292519943295;
};

struct ConeWallValidationEvidence {
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

struct ConeWallFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct ConeWallResult {
    std::optional<PlanarCdtMesh> value;
    std::vector<ConeWallValidationEvidence> validation;
    std::optional<ConeWallFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Boundary-exact apex-cone wall: one closed circular base rim plus one
// singular apex station. Triangles fan from the apex to consecutive base
// samples. Every face/coedge sample use (rim, seam, apex) must be consumed.
ConeWallResult buildApexConeWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const ConeWallConfiguration& configuration = {},
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
