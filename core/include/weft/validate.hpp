#pragma once

#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <cstddef>
#include <string>

namespace weft {

// Bake-ready validation (plan §4.2): the checks a game mesh must pass
// before it's worth baking or shipping. Everything is computed from the
// generated PolyMesh; deviation additionally needs the live B-rep.
struct ValidationReport {
    size_t polygons = 0;
    size_t openEdges = 0;         // mesh edges used by exactly one polygon
    size_t nonManifoldEdges = 0;  // mesh edges used by three or more
    size_t windingConflicts = 0;  // interior edges traversed the same way
                                  // by both polygons (inconsistent normals)
    size_t degeneratePolygons = 0;  // repeated vertices or near-zero area
    size_t sliverPolygons = 0;      // a corner angle below sliverAngleDeg

    // Max/mean distance between polygon centers and the true B-rep surface,
    // sampled on polygons whose corners are anchored to their source face.
    double maxDeviation = 0.0;
    double meanDeviation = 0.0;
    size_t deviationSamples = 0;

    double sliverAngleDeg = 5.0;

    bool watertight() const { return openEdges == 0 && nonManifoldEdges == 0; }
    bool clean() const {
        return watertight() && windingConflicts == 0 && degeneratePolygons == 0;
    }
};

// Validate a generated mesh. `model` is optional; when present, chord
// deviation against the live surfaces is measured through the anchors.
ValidationReport validateMesh(const PolyMesh& mesh, const Model* model = nullptr);

// Human-readable multi-line summary (the CLI's `validate` output).
std::string formatReport(const ValidationReport& r);

}  // namespace weft
