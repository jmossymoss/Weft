#pragma once

#include "weft/meshers.hpp"
#include "weft/planar_cdt.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace weft {

struct ValidationCoverage {
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

struct ValidationCertificate {
    std::vector<ValidationCoverage> checks;

    bool complete() const noexcept;
};

struct CertifiedVertexUse {
    StableId workingFace;
    std::optional<StableId> sourceFace;
    PlanarTrimBoundaryUse boundary;
};

struct CertifiedVertex {
    std::uint64_t canonicalVertexIndex = InvalidCanonicalVertexIndex;
    std::array<double, 3> position{};
    std::vector<CertifiedVertexUse> provenance;
};

struct CertifiedTriangle {
    StableId workingFace;
    std::optional<StableId> sourceFace;
    std::array<std::uint32_t, 3> vertices{};
    std::array<PredicatePoint2, 3> cornerUv{};
};

struct CertifiedMesh {
    std::vector<CertifiedVertex> vertices;
    std::vector<CertifiedTriangle> triangles;
    std::string topologyFingerprint;
};

struct ModelingPolygon {
    StableId workingFace;
    std::optional<StableId> sourceFace;
    std::vector<std::uint32_t> vertices;
};

struct ModelingMesh {
    std::vector<CertifiedVertex> vertices;
    std::vector<ModelingPolygon> polygons;
    bool aliasesCertified = false;
    std::optional<std::string> safeFloorReason;
};

struct MeshingResult {
    CertifiedMesh certified;
    ModelingMesh modeling;
    GenerationReport generation;
    ValidationCertificate validation;
};

ModelingMesh makeCertifiedFloorModelingMesh(
    const CertifiedMesh& certified,
    std::optional<std::string> safeFloorReason = std::nullopt);

MeshingResult makeCertifiedFloorMeshingResult(
    CertifiedMesh certified,
    ValidationCertificate validation,
    GenerationReport generation = {},
    std::optional<std::string> safeFloorReason = std::nullopt);

struct CertifiedMeshAssemblyConfiguration {
    bool requireClosedManifold = true;
    double maximumVertexSurfaceDiscrepancy = 1e-3;
};

struct CertifiedMeshAssemblyFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct CertifiedMeshAssemblyResult {
    std::optional<CertifiedMesh> value;
    ValidationCertificate validation;
    std::optional<CertifiedMeshAssemblyFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Assembles boundary-exact face triangle products by canonical vertex
// identity. No spatial search or weld is performed. Curved templates provide
// per-triangle periodic corner lifts; planar CDT uses vertex UVs directly.
// `expectedWorkingFaces` prevents a partial face set claiming a complete body.
CertifiedMeshAssemblyResult assembleCertifiedBoundaryMesh(
    const ImportedModel& imported,
    const CanonicalBoundarySet& boundaries,
    std::span<const PlanarCdtMesh> faceMeshes,
    std::span<const StableId> expectedWorkingFaces,
    const CertifiedMeshAssemblyConfiguration& configuration = {});

// Compatibility name for the first planar-only callers.
CertifiedMeshAssemblyResult assembleCertifiedPlanarMesh(
    const ImportedModel& imported,
    const CanonicalBoundarySet& boundaries,
    std::span<const PlanarCdtMesh> faceMeshes,
    std::span<const StableId> expectedWorkingFaces,
    const CertifiedMeshAssemblyConfiguration& configuration = {});

}  // namespace weft
