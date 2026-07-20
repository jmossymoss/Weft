#pragma once

// Cross-platform topology signature (EXECUTION_PLAN §3.2 / WP2).
// Machine-comparable artifact for face routing, build status, polygon
// arity, and connectivity/validity. Byte-identical OBJ floats are not
// required; anchored UV agreement is summarized with a quantized hash.

#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"

#include <string>
#include <vector>

namespace weft {

inline constexpr const char* kTopologySignatureSchema =
    "weft.topology_signature.v1";

// Default UV quantization for the optional anchor summary hash.
inline constexpr double kTopologySignatureAnchorTol = 1e-6;

struct TopologySignatureInfo {
    // Basename or path label only (informational; ignored by compare).
    std::string inputLabel;
};

// Emit a stable line-oriented signature. Policy lines (compared) have no
// prefix; informational lines use "info.". Comments start with '#'.
std::string formatTopologySignature(const PolyMesh& mesh, const Model& model,
                                    const GenerationReport& report,
                                    const ValidationReport& validation,
                                    const TopologySignatureInfo& info = {});

// Compare two signatures under the §3.2 policy. Returns true when equal.
// On mismatch, writes a short human diff into `diffOut` when non-null.
bool topologySignaturesEqual(const std::string& a, const std::string& b,
                             std::string* diffOut = nullptr);

// Extract sorted policy key=value lines (no comments, no info.*).
std::vector<std::string> topologySignaturePolicyLines(const std::string& text);

}  // namespace weft
