#pragma once

#include "weft/planar_cdt.hpp"
#include "weft/secure_reconnaissance.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct CylinderWallConfiguration {
    double maximumChordDeviation = 5e-5;
    double maximumNormalDeviationRadians = 0.017453292519943295;
    // Number of equal axial intervals between the two rims. 1 preserves the
    // original one-band boundary-only template. Values > 1 generate certified
    // interior rings with CylinderInteriorStation provenance.
    std::uint32_t axialIntervals = 1;
};

struct CylinderWallValidationEvidence {
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

struct CylinderWallFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct CylinderWallResult {
    std::optional<PlanarCdtMesh> value;
    std::vector<CylinderWallValidationEvidence> validation;
    std::optional<CylinderWallFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Builds the boundary-exact full/partial revolved band wall for a supported
// cylinder or truncated (non-apex) cone. Both rims must already have equal
// canonical counts. With axialIntervals == 1 this is the original one-band
// template. With axialIntervals > 1, equal interior rings are generated with
// CylinderInteriorStation provenance; every face/coedge sample use must still
// be consumed (rim or matching interior seam sample).
CylinderWallResult buildFullCylinderWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const CylinderWallConfiguration& configuration = {},
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
