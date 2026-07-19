#pragma once

#include "weft/canonical_boundary.hpp"
#include "weft/certified_mesh.hpp"
#include "weft/cylinder_template.hpp"
#include "weft/interval_solver.hpp"

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace weft {

// Exact equal-sum constraint over working-edge interval variables. Edge IDs
// become boundary IDs inside the interval problem; one bounded connected
// coupled component is supported and all wider classes refuse by name.
struct EdgeIntervalChainSum {
    std::vector<StableId> lhs;
    std::vector<StableId> rhs;
};

struct SecureMeshingConfiguration {
    SamplingConfiguration sampling;
    CertifiedMeshAssemblyConfiguration assembly;
    std::uint32_t cylinderAxialIntervals = 1;
    // Azimuth/rim budget for cylinder, cone, sphere, torus, circular bores.
    std::uint32_t revolutionRadialSegments = 32;
    // Soft preview triangle target; 0 disables adaptive coarsening.
    std::uint32_t previewTriangleBudget = 70000;
    // Working-edge IDs resolved through recipe-v2 correspondence. Counts are
    // exact solver constraints, never face-local sampling requests.
    std::map<StableId, std::uint32_t> exactEdgeIntervalCounts;
    // Exact bounded chain-sum equalities consumed by planar/cylinder templates
    // through the shared canonical boundary counts.
    std::vector<EdgeIntervalChainSum> edgeChainSums;
    // Optional even-parity requirements for adversarial/parity fixtures.
    std::set<StableId> requireEvenEdgeIntervals;
    // Line-buffered stderr progress for large models (MP9 inventory / gates).
    bool progressToStderr = false;
    // Inventory-only: continue past unsupported faces/curves to aggregate
    // refusal codes. Still returns failure and never a MeshingResult.
    bool collectAllUnsupported = false;
};

struct SecureMeshingUnsupportedRecord {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
    std::string familyCode;
};

struct SecureMeshingFailure {
    std::string code;
    std::string message;
    std::vector<StableId> subjects;
};

struct SecureMeshingResult {
    std::optional<MeshingResult> value;
    ValidationCertificate validation;
    std::optional<SecureMeshingFailure> failure;
    // Populated when configuration.collectAllUnsupported is true.
    std::vector<SecureMeshingUnsupportedRecord> unsupportedRecords;

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Compact cross-platform digests for M3 count/boundary/lift/report evidence.
// Serialization is locale-independent and orders every map/set by key.
struct M3DeterminismDigest {
    std::string counts;
    std::string boundary;
    std::string lifts;
    std::string report;
};

// Atomic secure pipeline for the currently proven automatic families: planar
// faces (including direct holes) and full periodic cylinders. Any unsupported
// face, failed count/boundary/template stage, or incomplete validator returns
// no body; there is no legacy or OCCT-triangle fallback.
SecureMeshingResult generateSecureMesh(
    const ImportedModel& imported,
    const SecureMeshingConfiguration& configuration = {});

// Digests count assignments, boundary topology provenance, UV/position lifts,
// and validation coverage from a successful secure mesh result.
M3DeterminismDigest digestM3Determinism(const SecureMeshingResult& result);

// Proves every solved interval is present in the canonical boundary set with
// an identical intervalCount. When mesh is provided, also proves generation
// report counts and certified sample provenance consume those intervals.
std::optional<SecureMeshingFailure> certifySolvedIntervalConsumption(
    const IntervalSolution& solved,
    const CanonicalBoundarySet& boundaries,
    const MeshingResult* mesh = nullptr);

// Workflow/export adapter. Certified triangles and their face-corner UVs are
// copied verbatim; no weld, topology edit, or downstream triangulation occurs.
PolyMesh makeCertifiedPolyMeshAdapter(const MeshingResult& result);

struct CertifiedAdmissionFailure {
    std::string code;
    std::string message;
};

struct CertifiedAdmissionResult {
    std::optional<std::string> selectedOutput;
    std::optional<CertifiedAdmissionFailure> failure;

    explicit operator bool() const noexcept { return !failure.has_value(); }
};

// Central admission gate for display/export/live-link consumers. Requires a
// complete MeshingResult certificate and truthful modelling provenance.
CertifiedAdmissionResult admitCertifiedMeshingResult(
    const MeshingResult& result,
    std::optional<std::uint64_t> expectedGenerationEpoch = std::nullopt,
    std::optional<std::uint64_t> actualGenerationEpoch = std::nullopt);

struct SecureCacheKey {
    std::string sourceSha256;
    std::string recipeFingerprint;
    std::string settingsFingerprint;
    std::string implementationVersion;
    std::string certificateFingerprint;
};

inline constexpr const char* kSecureImplementationVersion = "weft-secure-1";

std::string fingerprintSecureCacheKey(const SecureCacheKey& key);
bool secureCacheKeysMatch(const SecureCacheKey& left, const SecureCacheKey& right);

struct SecureCacheLookupResult {
    bool hit = false;
    std::optional<CertifiedAdmissionFailure> failure;
};

// Returns a hit only when keys match byte-for-byte and the cached result still
// admits. Mismatched or corrupt entries refuse by name.
SecureCacheLookupResult lookupSecureCache(
    const SecureCacheKey& request,
    const SecureCacheKey& cached,
    const MeshingResult* cachedResult);

}  // namespace weft
