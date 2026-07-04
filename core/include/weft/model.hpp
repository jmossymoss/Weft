#pragma once

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>

#include <string>
#include <vector>

namespace weft {

// A loaded B-rep with stable integer IDs for faces and edges.
//
// IDs are 1-based indices into the shape's face/edge maps, assigned in
// deterministic traversal order. For STEP files re-exported from the same
// source these stay stable; synthesizing IDs from geometric hashes (for
// reconciling upstream CAD edits) is planned but not part of the spike.
struct Model {
    TopoDS_Shape shape;
    TopTools_IndexedMapOfShape faces;   // FaceId -> TopoDS_Face
    TopTools_IndexedMapOfShape edges;   // EdgeId -> TopoDS_Edge
    TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;

    // Bodies and their STEP product names (parallel; empty string when the
    // source carried none). Names come from the representation items of
    // the transfer session, so "bolt"/"nut" survive into the exports.
    TopTools_IndexedMapOfShape solids;  // SolidId -> TopoDS_Solid
    std::vector<std::string> solidNames;

    int faceCount() const { return faces.Extent(); }
    int edgeCount() const { return edges.Extent(); }
};

// Load a STEP file, heal/sew it, and index its topology.
// Throws std::runtime_error on failure.
Model loadStep(const std::string& path);

// Write any shape to STEP (used by the fixture generator and tests).
void writeStep(const TopoDS_Shape& shape, const std::string& path);

}  // namespace weft
