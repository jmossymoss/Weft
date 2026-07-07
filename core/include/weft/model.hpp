#pragma once

#include <Standard_Version.hxx>
#include <TopoDS_Shape.hxx>

// OCCT 8.0 deprecated the TopTools_* typedef headers in favour of the
// NCollection templates they alias. Spell the containers per version so
// neither toolchain warns; the underlying types are identical, so every
// OCCT API (TopExp etc.) binds unchanged.
#if OCC_VERSION_HEX >= 0x080000
#include <NCollection_IndexedDataMap.hxx>
#include <NCollection_IndexedMap.hxx>
#include <NCollection_List.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#else
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#endif

#include <array>
#include <string>
#include <vector>

namespace weft {

#if OCC_VERSION_HEX >= 0x080000
using ShapeMap = NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher>;
using ShapeList = NCollection_List<TopoDS_Shape>;
using EdgeFaceMap =
    NCollection_IndexedDataMap<TopoDS_Shape, ShapeList,
                               TopTools_ShapeMapHasher>;
#else
using ShapeMap = TopTools_IndexedMapOfShape;
using ShapeList = TopTools_ListOfShape;
using EdgeFaceMap = TopTools_IndexedDataMapOfShapeListOfShape;
#endif

// A compact node in the imported assembly tree (for the glTF node graph /
// OBJ object grouping). Populated only when the source carried real
// hierarchy; a flat model leaves Model::assembly empty.
struct AssemblyNode {
    std::string name;
    int solidId = -1;                    // 1-based index into Model.solids; -1 = grouping node
    std::array<double, 16> transform{};  // absolute, column-major, mm (identity if leaf)
    std::vector<int> children;           // indices into Model.assembly
};

// A loaded B-rep with stable integer IDs for faces and edges.
//
// IDs are 1-based indices into the shape's face/edge maps, assigned in
// deterministic traversal order. For STEP files re-exported from the same
// source these stay stable; synthesizing IDs from geometric hashes (for
// reconciling upstream CAD edits) is planned but not part of the spike.
struct Model {
    TopoDS_Shape shape;
    ShapeMap faces;   // FaceId -> TopoDS_Face
    ShapeMap edges;   // EdgeId -> TopoDS_Edge
    EdgeFaceMap edgeToFaces;

    // Bodies and their STEP product names (parallel; empty string when the
    // source carried none). Names come from the representation items of
    // the transfer session, so Plasticity object names survive to export.
    ShapeMap solids;
    std::vector<std::string> solidNames;

    // --- NEW import metadata (all optional; empty when the source carried
    // none). Read by nobody in the retopo/mesh pipeline; purely additive so
    // exporters can preserve source appearance/structure. ---
    std::vector<std::array<float, 3>> faceColors;   // parallel to faces (FaceId-1), linear RGB
    std::vector<char> faceHasColor;                 // parallel; 0 = unset (don't trust black)
    std::vector<std::array<float, 3>> solidColors;  // parallel to solids
    std::vector<char> solidHasColor;
    std::vector<std::string> solidLayers;     // parallel to solids ("" = none)
    std::vector<std::string> solidMaterials;  // parallel to solids ("" = none)
    std::vector<AssemblyNode> assembly;       // roots first; empty = flat model
    double lengthUnitMm = 1.0;                // file's declared length unit, in mm

    int faceCount() const { return faces.Extent(); }
    int edgeCount() const { return edges.Extent(); }
};

// Load a STEP file, heal/sew it, and index its topology.
// Throws std::runtime_error on failure.
Model loadStep(const std::string& path);

// Write any shape to STEP (used by the fixture generator and tests).
void writeStep(const TopoDS_Shape& shape, const std::string& path);

}  // namespace weft
