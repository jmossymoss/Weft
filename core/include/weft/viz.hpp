#pragma once

#include "weft/model.hpp"

#include <array>
#include <map>
#include <vector>

namespace weft {

// Polyline sampling of the B-rep edges for viewport display (feature
// colouring by convexity is the caller's job — pair with Analysis).
struct EdgePolyline {
    int edgeId = 0;
    std::vector<std::array<double, 3>> points;
};

std::vector<EdgePolyline> sampleEdges(const Model& model, int segmentsPerEdge);

// Same, but edges with a solved subdivision count (GenerationReport's
// edgeDivisions) sample at exactly that count, so the overlay's chords
// coincide with the generated mesh instead of ghosting past it.
std::vector<EdgePolyline> sampleEdges(const Model& model, int segmentsPerEdge,
                                      const std::map<int, int>& perEdge);

}  // namespace weft
