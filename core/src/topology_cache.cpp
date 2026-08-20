#include "weft/topology_cache.hpp"

#include "weft/analysis.hpp"
#include "weft/model.hpp"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>

#include <algorithm>

namespace weft {

void TopologyCache::clear() {
    faceCount = edgeCount = 0;
    faceEdgeOffset.clear();
    faceEdgeIds.clear();
    edgeFaceOffset.clear();
    edgeFaceIds.clear();
    faceNbrOffset.clear();
    faceNbrIds.clear();
    edgeLength.clear();
    faceArea.clear();
    modelArea = 0.0;
    faceDirty.clear();
    edgeSegments.clear();
}

void TopologyCache::buildFromAnalysis(const Analysis& analysis) {
    clear();
    faceCount = int(analysis.faces.size());
    edgeCount = int(analysis.edges.size());
    if (faceCount < 1 || edgeCount < 1) return;

    faceEdgeOffset.assign(size_t(faceCount) + 1, 0);
    edgeFaceOffset.assign(size_t(edgeCount) + 1, 0);
    faceNbrOffset.assign(size_t(faceCount) + 1, 0);
    edgeLength.assign(size_t(edgeCount), 0.0);
    faceArea.assign(size_t(faceCount), 0.0);
    faceDirty.assign(size_t(faceCount), 0);
    edgeSegments.assign(size_t(edgeCount) + 1, 0);

    size_t nFaceEdges = 0, nEdgeFaces = 0, nNbrs = 0;
    for (const FaceInfo& f : analysis.faces) {
        nFaceEdges += f.edgeIds.size();
        nNbrs += f.neighborFaceIds.size();
    }
    for (const EdgeInfo& e : analysis.edges) {
        nEdgeFaces += e.faceIds.size();
    }
    faceEdgeIds.reserve(nFaceEdges);
    edgeFaceIds.reserve(nEdgeFaces);
    faceNbrIds.reserve(nNbrs);

    for (int fi = 0; fi < faceCount; ++fi) {
        faceEdgeOffset[size_t(fi)] = uint32_t(faceEdgeIds.size());
        for (int eid : analysis.faces[size_t(fi)].edgeIds) {
            if (eid >= 1) faceEdgeIds.push_back(uint32_t(eid));
        }
        faceNbrOffset[size_t(fi)] = uint32_t(faceNbrIds.size());
        for (int nid : analysis.faces[size_t(fi)].neighborFaceIds) {
            if (nid >= 1) faceNbrIds.push_back(uint32_t(nid));
        }
    }
    faceEdgeOffset[size_t(faceCount)] = uint32_t(faceEdgeIds.size());
    faceNbrOffset[size_t(faceCount)] = uint32_t(faceNbrIds.size());

    for (int ei = 0; ei < edgeCount; ++ei) {
        edgeFaceOffset[size_t(ei)] = uint32_t(edgeFaceIds.size());
        edgeLength[size_t(ei)] = analysis.edges[size_t(ei)].length;
        for (int fid : analysis.edges[size_t(ei)].faceIds) {
            if (fid >= 1) edgeFaceIds.push_back(uint32_t(fid));
        }
    }
    edgeFaceOffset[size_t(edgeCount)] = uint32_t(edgeFaceIds.size());
}

void TopologyCache::fillFaceAreas(const Model& model) {
    if (faceCount < 1 || faceCount != model.faceCount()) return;
    faceArea.assign(size_t(faceCount), 0.0);
    modelArea = 0.0;
    for (int fi = 0; fi < faceCount; ++fi) {
        try {
            GProp_GProps gp;
            const TopoDS_Shape s = model.faces(fi + 1);
            BRepGProp::SurfaceProperties(s, gp);
            faceArea[size_t(fi)] = std::max(0.0, gp.Mass());
        } catch (const Standard_Failure&) {
            faceArea[size_t(fi)] = 0.0;
        }
        modelArea += faceArea[size_t(fi)];
    }
}

void TopologyCache::markDirtyClosure(uint32_t faceId) {
    if (faceId < 1 || int(faceId) > faceCount) return;
    faceDirty[faceId - 1] = 1;
    for (const uint32_t* p = faceNbrsBegin(faceId); p != faceNbrsEnd(faceId);
         ++p) {
        const uint32_t n = *p;
        if (n >= 1 && int(n) <= faceCount) faceDirty[n - 1] = 1;
    }
}

}  // namespace weft
