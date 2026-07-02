#pragma once

#include "weft/model.hpp"

#include <array>
#include <vector>

namespace weft {

// Polyline sampling of the B-rep edges for viewport display (feature
// colouring by convexity is the caller's job — pair with Analysis).
struct EdgePolyline {
    int edgeId = 0;
    std::vector<std::array<double, 3>> points;
};

std::vector<EdgePolyline> sampleEdges(const Model& model, int segmentsPerEdge);

}  // namespace weft
