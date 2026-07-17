#pragma once

// Internal to core/src/io — a stateless facade over an XDE document plus the
// metadata capture / re-association helpers shared by the B-rep readers.

#include "weft/model.hpp"
#include "weft/secure_core.hpp"

#include <BRepTools_History.hxx>
#include <Quantity_Color.hxx>
#include <Standard_Handle.hxx>
#include <NCollection_Sequence.hxx>
#include <TDF_Label.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Shape.hxx>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

class TDocStd_Document;
class XCAFDoc_ShapeTool;
class XCAFDoc_ColorTool;
class XCAFDoc_LayerTool;
class XCAFDoc_MaterialTool;
class XCAFDoc_Material;

namespace weft::io {

using LabelSequence = NCollection_Sequence<TDF_Label>;

// Port of Mayo's XCaf: fetches XDE tools on demand from doc->Main() and
// exposes typed helpers. Stateless beyond the main label.
class XCaf {
public:
    explicit XCaf(const Handle(TDocStd_Document)& doc);

    const Handle(TDocStd_Document)& document() const { return m_doc; }

    Handle(XCAFDoc_ShapeTool) shapeTool() const;
    Handle(XCAFDoc_ColorTool) colorTool() const;
    Handle(XCAFDoc_LayerTool) layerTool() const;
    Handle(XCAFDoc_MaterialTool) materialTool() const;

    LabelSequence topLevelFreeShapes() const;  // shapeTool()->GetFreeShapes()

    static bool isShapeAssembly(const TDF_Label&);   // IsAssembly
    static bool isShapeReference(const TDF_Label&);  // IsReference (instance + placement)
    static bool isShapeSimple(const TDF_Label&);     // IsSimpleShape
    static LabelSequence shapeComponents(const TDF_Label&);  // GetComponents
    static LabelSequence shapeSubs(const TDF_Label&);        // GetSubShapes (per-face labels)
    static TDF_Label shapeReferred(const TDF_Label&);           // GetReferredShape
    static TopoDS_Shape shape(const TDF_Label&);                // GetShape
    static TopLoc_Location shapeReferenceLocation(const TDF_Label&);

    bool hasShapeColor(const TDF_Label&) const;    // ColorTool::IsSet(Gen|Surf|Curv)
    Quantity_Color shapeColor(const TDF_Label&) const;  // loop Surf,Curv,Gen -> GetColor
    static std::string labelName(const TDF_Label&);     // TDataStd_Name -> UTF-8
    std::string layerName(const TDF_Label&) const;      // first layer of the label ("" = none)
    Handle(XCAFDoc_Material) shapeMaterial(const TDF_Label&) const;

private:
    Handle(TDocStd_Document) m_doc;
    TDF_Label m_main;
};

// Provenance captured on the ORIGINAL (pre-heal) shapes, keyed by the
// underlying TShape pointer (stable and location-independent — the same key
// the legacy shapeNames() walk uses, so instanced parts collapse together).
struct ImportMeta {
    std::unordered_map<const void*, std::array<float, 3>> faceColor;  // linear RGB 0..1
    std::unordered_map<const void*, bool> faceHasColor;
    std::unordered_map<const void*, std::string> faceLayer;
    std::unordered_map<const void*, std::string> solidName;
    std::unordered_map<const void*, std::array<float, 3>> solidColor;
    std::unordered_map<const void*, bool> solidHasColor;
    std::unordered_map<const void*, std::string> solidLayer;
    std::unordered_map<const void*, std::string> solidMaterial;
    std::vector<AssemblyNode> assembly;
    std::vector<const void*> leafSolidKey;  // parallel to assembly; nullptr for group nodes
    double lengthUnitMm = 1.0;
};

// Walk the assembly, resolving colors/names/layers/materials onto the
// original faces/solids and building the assembly tree.
void captureMeta(const XCaf& xc, const LabelSequence& roots, ImportMeta& out);

// Recover the file's declared length unit as millimetres-per-model-unit
// (1.0 for a mm file, 25.4 for inch, 1000 for metre). OCCT keeps geometry in
// mm regardless; this is metadata only.
double readLengthUnit(const Handle(TDocStd_Document)& doc);

// Re-associate captured face colors/layers onto model faces through `hist`
// (original -> working). The legacy healed-model path may opt into its
// geometric midpoint fallback; secure source/working models must not because
// proximity is not a provenance certificate.
void reassociateMeta(Model& m, const TopoDS_Shape& origShape, const ImportMeta& meta,
                     const BRepTools_History& hist,
                     bool allowGeometricFallback = true);

// Fill Model::solidNames / solidColors / solidLayers / solidMaterials from the
// captured metadata, pairing original solids with healed solids by traversal
// order (suffixing instanced duplicate names), exactly like the legacy path.
void fillSolidMetaFromCaf(Model& m, const TopoDS_Shape& origShape, const ImportMeta& meta);

// End-to-end: capture metadata off `oneShape` (pre-heal) using the XDE doc,
// run the verbatim heal pipeline (accumulating history), index, then
// re-associate. solidNames are filled from XCAF here; STEP overrides them with
// its legacy STEP-entity names afterward to keep byte-identical output.
Model cafToModel(const TopoDS_Shape& oneShape, const Handle(TDocStd_Document)& doc);

// Secure-core import path. `oneShape` is the processing-disabled XDE transfer
// result and is always retained as the immutable source model. Conservative
// import derives an identity working copy; compatibility import runs the
// historical Weft heal pipeline as an explicit, certified repair stage.
ImportedModel cafToImportedModel(const TopoDS_Shape& oneShape,
                                 const Handle(TDocStd_Document)& doc,
                                 SourceMetadata metadata,
                                 RepairProfile profile);

}  // namespace weft::io

namespace weft {
// Defined in model.cpp. Exposed (previously file-static) so the io readers can
// drive the exact same indexing and heal pipeline loadStep used.
Model indexShape(const TopoDS_Shape& shape);
TopoDS_Shape healWithHistory(const TopoDS_Shape& input, Handle(BRepTools_History)& outHist);
}  // namespace weft
