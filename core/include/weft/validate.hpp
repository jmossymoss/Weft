#pragma once

#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

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

    // B-rep faces ranked by how many open mesh edges their polygons own —
    // the "where does it leak" list. (faceId, openEdgeCount), descending.
    std::vector<std::pair<int, size_t>> leakyFaces;

    // The open mesh edges themselves (vertex index pairs), for viewport
    // highlighting. Capped at 200k entries on pathological meshes.
    std::vector<std::pair<uint32_t, uint32_t>> openEdgeList;

    // B-rep edges bordering fewer than two faces: the INPUT is an open
    // shell there, so a matching share of open mesh edges is expected and
    // not a meshing defect.
    size_t inputBoundaryEdges = 0;
    // Of openEdges: how many run along those input boundaries (expected)
    // vs. anywhere else (real cracks between faces).
    size_t openEdgesOnInputBoundary = 0;
    // B-rep edges bordering three or more faces: the INPUT is non-manifold
    // there, and the welded mesh necessarily is too.
    size_t inputNonManifoldEdges = 0;
    // Non-degenerate B-rep edges in the indexed model (for open-shell
    // fraction). Authored sheet bodies / broken sources often have
    // ≥⅓ of edges on the open boundary.
    size_t inputEdges = 0;
    // True when the source is an open surface model / broken solid by
    // the same ≥⅓ open-shell gate used in import capping — not a
    // closed-solid watertightness target.
    bool brokenSource = false;

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
