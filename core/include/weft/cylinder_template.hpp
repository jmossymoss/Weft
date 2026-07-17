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

// Builds the boundary-exact one-band template for a proven full periodic
// cylinder. Both rims must already have equal canonical counts. Every
// face/coedge sample use is consumed; additional axial seam samples are a
// named refusal until certified interior-vertex provenance is introduced.
CylinderWallResult buildFullCylinderWall(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    const CylinderWallConfiguration& configuration = {},
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
