// Static B-rep adjacency cache (data-oriented).
//
// Built once in analyze() from the indexed Model. Interactive generate()
// reads face↔edge and edge↔face adjacency as O(1) CSR slices — no
// TopExp_Explorer on the hot path. Division counts and 3D samples still
// come from the density solve / meshers (AD-1 border contract); this cache
// does not invent a second mesher.
//
#pragma once

#include <cstdint>
#include <algorithm>
#include <vector>

namespace weft {

struct Model;
struct Analysis;

struct TopologyCache {
    int faceCount = 0;
    int edgeCount = 0;

    // CSR: face -> boundary edge ids (order matches analyze()'s edgeIds).
    std::vector<uint32_t> faceEdgeOffset;  // size faceCount+1
    std::vector<uint32_t> faceEdgeIds;

    // CSR: edge -> adjacent face ids.
    std::vector<uint32_t> edgeFaceOffset;  // size edgeCount+1
    std::vector<uint32_t> edgeFaceIds;

    // CSR: face -> neighbor face ids (via shared edges).
    std::vector<uint32_t> faceNbrOffset;  // size faceCount+1
    std::vector<uint32_t> faceNbrIds;

    // Arc lengths from analyze (mm); index = edgeId-1.
    std::vector<double> edgeLength;

    // Surface areas (mm^2); index = faceId-1. Filled at analyze so
    // generate() never calls BRepGProp::SurfaceProperties on the hot path.
    std::vector<double> faceArea;
    double modelArea = 0.0;

    // Interactive dirty tags: remesh candidates. Cleared by generate after
    // the remesh set is known (cache miss list). Not a substitute for
    // GenerationCache keys — only a fast filter / overlay hint.
    std::vector<uint8_t> faceDirty;

    // Mirrors last solvedEdge counts (optional; filled by generate).
    std::vector<int> edgeSegments;

    bool empty() const { return faceCount == 0; }

    void clear();

    // Build flat arrays from Analysis adjacency (no OCCT traversal).
    void buildFromAnalysis(const Analysis& analysis);

    // One-shot OCCT surface-area fill (call from analyze with Model).
    void fillFaceAreas(const Model& model);

    // Mark a face and every face sharing an edge (border closure).
    void markDirtyClosure(uint32_t faceId);

    void clearDirty() {
        std::fill(faceDirty.begin(), faceDirty.end(), uint8_t{0});
    }

    // Span helpers (faceId / edgeId are 1-based).
    const uint32_t* faceEdgesBegin(uint32_t faceId) const {
        return faceEdgeIds.data() + faceEdgeOffset[faceId - 1];
    }
    const uint32_t* faceEdgesEnd(uint32_t faceId) const {
        return faceEdgeIds.data() + faceEdgeOffset[faceId];
    }
    uint32_t faceEdgeCount(uint32_t faceId) const {
        return faceEdgeOffset[faceId] - faceEdgeOffset[faceId - 1];
    }

    const uint32_t* edgeFacesBegin(uint32_t edgeId) const {
        return edgeFaceIds.data() + edgeFaceOffset[edgeId - 1];
    }
    const uint32_t* edgeFacesEnd(uint32_t edgeId) const {
        return edgeFaceIds.data() + edgeFaceOffset[edgeId];
    }

    const uint32_t* faceNbrsBegin(uint32_t faceId) const {
        return faceNbrIds.data() + faceNbrOffset[faceId - 1];
    }
    const uint32_t* faceNbrsEnd(uint32_t faceId) const {
        return faceNbrIds.data() + faceNbrOffset[faceId];
    }
};

}  // namespace weft
