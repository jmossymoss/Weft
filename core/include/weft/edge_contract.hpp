#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace weft {

// One immutable sample owned by a B-rep edge.  Face meshers may project the
// sample into their own UV chart, but the id, curve parameter and 3D position
// are shared by every incident coedge.
struct CanonicalEdgeSample {
    std::uint64_t id = 0;
    double curveParameter = 0.0;
    // Exact normalized parameter used to build the sequence.  Keeping this
    // value avoids a parameter -> fraction round trip moving legacy vertices
    // by a last-bit amount when the adapter feeds an existing face mesher.
    double curveFraction = 0.0;
    // Normalized physical station along the canonical forward sequence.
    double normalizedAbscissa = 0.0;
    std::array<double, 3> position{};
    bool valid = false;
};

struct CanonicalEdgePlan {
    int edgeId = 0;
    int idealSegmentCount = 1;
    int segmentCount = 1;
    bool closed = false;
    bool uniformAbscissa = false;
    // Lines and circles are proportional in curve parameter and physical arc;
    // these are the first guarded family consumed directly by legacy face
    // builders.  Freeform curves stay table-observed until their UV projection
    // path is migrated as one transaction.
    bool parameterLinear = false;
    // Rotation of a closed sequence in normalized curve parameter space.
    // It is part of the identity: callers requesting a different phase stay
    // on the legacy path until that sequence is explicitly reconciled.
    double phase = 0.0;
    std::vector<CanonicalEdgeSample> samples;
};

// Legacy meshers receive one table after the final density and pin passes.
// A zero edgeId means that the edge is intentionally still on the legacy
// path (one-owner, degenerate or otherwise outside the guarded rollout).
struct CanonicalEdgeTable {
    std::vector<CanonicalEdgePlan> edgePlans;  // index = EdgeId - 1

    const CanonicalEdgePlan* find(int edgeId) const {
        if (edgeId < 1 || edgeId > static_cast<int>(edgePlans.size())) {
            return nullptr;
        }
        const CanonicalEdgePlan& plan = edgePlans[edgeId - 1];
        return plan.edgeId == edgeId ? &plan : nullptr;
    }
};

}  // namespace weft
