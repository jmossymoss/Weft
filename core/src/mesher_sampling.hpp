#pragma once

#include "weft/edge_contract.hpp"

#include <memory>
#include <vector>

class Adaptor3d_Curve;

namespace weft {
struct Model;
}

namespace weft::mesher_detail {

// Explicit edge fractions override the solved uniform count.  The canonical
// table is attached only after every density repair and pin pass has finished;
// this makes the same immutable sequence available to all legacy face
// builders without threading a second context through every mesher signature.
struct PinnedEdges : std::vector<std::vector<double>> {
    using std::vector<std::vector<double>>::vector;
    std::shared_ptr<const CanonicalEdgeTable> canonical;
};

bool edgeIsPinned(int edgeId, const PinnedEdges* pins);

double phasedT(int index, int divisions, double phase, bool reversed);

// Fractions in face traversal order.  When the final canonical table owns this
// edge/count, this is an adapter over that sequence; otherwise it preserves the
// existing deterministic legacy sampling rule.
std::vector<double> edgeSampleFractions(int edgeId, int divisions,
                                        double phase, bool reversed,
                                        bool includeLast,
                                        const PinnedEdges* pins,
                                        const Model* model);

// Build the guarded shared-edge rollout from the legacy solver's FINAL counts
// and pins.  R1 currently enrolls ordinary two-owner, non-degenerate
// line/circle edges; freeform UV/sample-id migration is a later slice.
std::shared_ptr<const CanonicalEdgeTable> buildCanonicalEdgeTable(
    const Model& model, const std::vector<int>& finalSegmentCounts,
    const PinnedEdges& pins, const std::vector<double>& edgePhases);

// Deterministic subdivision count for the uniform-parameter border contract.
int stableDeflectionCount(const Adaptor3d_Curve& curve, double angleTolerance,
                          double chordTolerance);

}  // namespace weft::mesher_detail
