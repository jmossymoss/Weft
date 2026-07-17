#pragma once

#include "weft/planar_trim_validation.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct PlanarCdtTriangle {
    StableId workingFace;
    std::optional<StableId> sourceFace;
    std::array<std::uint32_t, 3> vertices{};
};

struct PlanarCdtMesh {
    StableId workingFace;
    std::optional<StableId> sourceFace;
    std::vector<PlanarTrimVertex> vertices;
    std::vector<std::array<std::uint32_t, 2>> constrainedEdges;
    std::vector<PlanarCdtTriangle> triangles;
};

struct PlanarCdtValidationEvidence {
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

struct PlanarCdtFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct PlanarCdtResult {
    std::optional<PlanarCdtMesh> value;
    PlanarTrimValidationResult trimValidation;
    std::vector<PlanarCdtValidationEvidence> validation;
    std::optional<PlanarCdtFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

class PlanarCdtBackend {
public:
    virtual ~PlanarCdtBackend() = default;

    virtual const char* backendCode() const noexcept = 0;
    virtual bool exactPredicatesForFiniteDoubleInputs() const noexcept = 0;
    virtual bool supportsHoles() const noexcept = 0;

    virtual PlanarCdtResult triangulate(
        const PlanarTrimDomain& domain) const = 0;
};

// Distribution-safe reference backend for a validated single outer loop. It
// creates a boundary-preserving ear triangulation and applies deterministic
// Lawson flips using the exact predicate interface. Hole support remains a
// named refusal until constraint recovery/cut-graph proof is implemented.
std::shared_ptr<const PlanarCdtBackend>
makeExactLawsonReferencePlanarCdtBackend(
    std::shared_ptr<const GeometricPredicates> predicates =
        makeExactDyadicPredicates());

}  // namespace weft
