#pragma once

#include "weft/certified_mesh.hpp"
#include "weft/cylinder_template.hpp"
#include "weft/interval_solver.hpp"

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct SecureMeshingConfiguration {
    SamplingConfiguration sampling;
    CertifiedMeshAssemblyConfiguration assembly;
    std::uint32_t cylinderAxialIntervals = 1;
    // Working-edge IDs resolved through recipe-v2 correspondence. Counts are
    // exact solver constraints, never face-local sampling requests.
    std::map<StableId, std::uint32_t> exactEdgeIntervalCounts;
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

    explicit operator bool() const noexcept { return value.has_value(); }
};

// Atomic secure pipeline for the currently proven automatic families: planar
// faces (including direct holes) and full periodic cylinders. Any unsupported
// face, failed count/boundary/template stage, or incomplete validator returns
// no body; there is no legacy or OCCT-triangle fallback.
SecureMeshingResult generateSecureMesh(
    const ImportedModel& imported,
    const SecureMeshingConfiguration& configuration = {});

// Workflow/export adapter. Certified triangles and their face-corner UVs are
// copied verbatim; no weld, topology edit, or downstream triangulation occurs.
PolyMesh makeCertifiedPolyMeshAdapter(const MeshingResult& result);

}  // namespace weft
