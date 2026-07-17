#include "xcaf.hpp"

#include "../secure_core_internal.hpp"

#include <Standard_Version.hxx>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_Tool.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <UnitsMethods.hxx>
#include <XCAFDoc.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_GraphNode.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_Material.hxx>
#include <XCAFDoc_MaterialTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Trsf.hxx>

#include <cmath>
#include <map>

namespace weft::io {

// ------------------------------------------------------------------ XCaf ----

XCaf::XCaf(const Handle(TDocStd_Document)& doc) : m_doc(doc), m_main(doc->Main()) {}

Handle(XCAFDoc_ShapeTool) XCaf::shapeTool() const {
    return XCAFDoc_DocumentTool::ShapeTool(m_main);
}
Handle(XCAFDoc_ColorTool) XCaf::colorTool() const {
    return XCAFDoc_DocumentTool::ColorTool(m_main);
}
Handle(XCAFDoc_LayerTool) XCaf::layerTool() const {
    return XCAFDoc_DocumentTool::LayerTool(m_main);
}
Handle(XCAFDoc_MaterialTool) XCaf::materialTool() const {
    return XCAFDoc_DocumentTool::MaterialTool(m_main);
}

LabelSequence XCaf::topLevelFreeShapes() const {
    LabelSequence seq;
    shapeTool()->GetFreeShapes(seq);
    return seq;
}

bool XCaf::isShapeAssembly(const TDF_Label& l) { return XCAFDoc_ShapeTool::IsAssembly(l); }
bool XCaf::isShapeReference(const TDF_Label& l) { return XCAFDoc_ShapeTool::IsReference(l); }
bool XCaf::isShapeSimple(const TDF_Label& l) { return XCAFDoc_ShapeTool::IsSimpleShape(l); }

LabelSequence XCaf::shapeComponents(const TDF_Label& l) {
    LabelSequence seq;
    XCAFDoc_ShapeTool::GetComponents(l, seq);
    return seq;
}
LabelSequence XCaf::shapeSubs(const TDF_Label& l) {
    LabelSequence seq;
    XCAFDoc_ShapeTool::GetSubShapes(l, seq);
    return seq;
}
TDF_Label XCaf::shapeReferred(const TDF_Label& l) {
    TDF_Label ref;
    XCAFDoc_ShapeTool::GetReferredShape(l, ref);
    return ref;
}
TopoDS_Shape XCaf::shape(const TDF_Label& l) { return XCAFDoc_ShapeTool::GetShape(l); }
TopLoc_Location XCaf::shapeReferenceLocation(const TDF_Label& l) {
    return XCAFDoc_ShapeTool::GetLocation(l);
}

bool XCaf::hasShapeColor(const TDF_Label& l) const {
    Handle(XCAFDoc_ColorTool) ct = colorTool();
    return ct->IsSet(l, XCAFDoc_ColorSurf) || ct->IsSet(l, XCAFDoc_ColorCurv) ||
           ct->IsSet(l, XCAFDoc_ColorGen);
}

Quantity_Color XCaf::shapeColor(const TDF_Label& l) const {
    Handle(XCAFDoc_ColorTool) ct = colorTool();
    Quantity_Color col;
    for (XCAFDoc_ColorType t : {XCAFDoc_ColorSurf, XCAFDoc_ColorCurv, XCAFDoc_ColorGen}) {
        if (ct->IsSet(l, t) && ct->GetColor(l, t, col)) return col;
    }
    return col;
}

std::string XCaf::labelName(const TDF_Label& label) {
    Handle(TDataStd_Name) nameAttr;
    if (!label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) return {};
    const TCollection_ExtendedString& es = nameAttr->Get();
    const int len = es.LengthOfCString();
    if (len <= 0) return {};
    std::string out(static_cast<size_t>(len) + 1, '\0');
    Standard_PCharacter p = out.data();
    es.ToUTF8CString(p);
    out.resize(std::char_traits<char>::length(out.c_str()));
    return out;
}

std::string XCaf::layerName(const TDF_Label& l) const {
    Handle(XCAFDoc_LayerTool) lt = layerTool();
    const auto layers = lt->GetLayers(l);
    if (layers.IsNull() || layers->IsEmpty()) return {};
    const TCollection_ExtendedString& es = layers->Value(1);
    const int len = es.LengthOfCString();
    if (len <= 0) return {};
    std::string out(static_cast<size_t>(len) + 1, '\0');
    Standard_PCharacter p = out.data();
    es.ToUTF8CString(p);
    out.resize(std::char_traits<char>::length(out.c_str()));
    return out;
}

Handle(XCAFDoc_Material) XCaf::shapeMaterial(const TDF_Label& label) const {
    Handle(XCAFDoc_GraphNode) node;
    if (label.FindAttribute(XCAFDoc::MaterialRefGUID(), node) && node->NbFathers() > 0) {
        TDF_Label matLabel = node->GetFather(1)->Label();
        Handle(XCAFDoc_Material) mat;
        if (matLabel.FindAttribute(XCAFDoc_Material::GetID(), mat)) return mat;
    }
    return {};
}

// -------------------------------------------------------------- helpers -----

static std::array<float, 3> toRGB(const Quantity_Color& c) {
    return {static_cast<float>(c.Red()), static_cast<float>(c.Green()),
            static_cast<float>(c.Blue())};
}

static std::array<double, 16> toMatrix(const TopLoc_Location& loc) {
    // Column-major 4x4 from OCCT's 3x4 transformation.
    gp_Trsf t = loc.Transformation();
    std::array<double, 16> m{};
    m[0] = t.Value(1, 1);
    m[1] = t.Value(2, 1);
    m[2] = t.Value(3, 1);
    m[3] = 0.0;
    m[4] = t.Value(1, 2);
    m[5] = t.Value(2, 2);
    m[6] = t.Value(3, 2);
    m[7] = 0.0;
    m[8] = t.Value(1, 3);
    m[9] = t.Value(2, 3);
    m[10] = t.Value(3, 3);
    m[11] = 0.0;
    m[12] = t.Value(1, 4);
    m[13] = t.Value(2, 4);
    m[14] = t.Value(3, 4);
    m[15] = 1.0;
    return m;
}

static std::string labelEntry(const TDF_Label& label) {
    TCollection_AsciiString entry;
    TDF_Tool::Entry(label, entry);
    return entry.ToCString();
}

static std::array<double, 3> faceCentroid(const TopoDS_Face& f) {
    GProp_GProps props;
    BRepGProp::SurfaceProperties(f, props);
    gp_Pnt p = props.CentreOfMass();
    return {p.X(), p.Y(), p.Z()};
}

static int faceSurfType(const TopoDS_Face& f) {
    BRepAdaptor_Surface s(f, false);
    return static_cast<int>(s.GetType());
}

// Carried down the assembly tree from parent to child.
struct Inherited {
    bool hasColor = false;
    std::array<float, 3> color{};
    std::string layer;
    std::string material;
};

static void recordSolidMeta(const TopoDS_Shape& sh, const Inherited& inh, ImportMeta& out,
                            const std::string& name) {
    auto stamp = [&](const TopoDS_Shape& body) {
        const void* key = body.TShape().get();
        if (!name.empty()) out.solidName.emplace(key, name);
        if (inh.hasColor) {
            out.solidColor.emplace(key, inh.color);
            out.solidHasColor.emplace(key, true);
        }
        if (!inh.layer.empty()) out.solidLayer.emplace(key, inh.layer);
        if (!inh.material.empty()) out.solidMaterial.emplace(key, inh.material);
    };
    bool any = false;
    for (TopExp_Explorer sx(sh, TopAbs_SOLID); sx.More(); sx.Next()) {
        stamp(sx.Current());
        any = true;
    }
    for (TopExp_Explorer sx(sh, TopAbs_SHELL, TopAbs_SOLID); sx.More(); sx.Next()) {
        stamp(sx.Current());
        any = true;
    }
    if (!any) stamp(sh);
}

static std::vector<TopoDS_Shape> bodyUses(const TopoDS_Shape& shape) {
    std::vector<TopoDS_Shape> bodies;
    for (TopExp_Explorer solid(shape, TopAbs_SOLID); solid.More(); solid.Next()) {
        bodies.push_back(solid.Current());
    }
    for (TopExp_Explorer shell(shape, TopAbs_SHELL, TopAbs_SOLID); shell.More();
         shell.Next()) {
        bodies.push_back(shell.Current());
    }
    return bodies;
}

// Recursive assembly walk; returns the index of the node created in
// out.assembly (or -1). Metadata is stamped on ORIGINAL faces/solids.
static int walkLabel(const XCaf& xc, const TDF_Label& label, const TopLoc_Location& loc,
                     int parent, Inherited inh, ImportMeta& out, int& leaves,
                     bool& sawAssembly) {
    TDF_Label refLabel = label;
    TopLoc_Location here = loc;
    TopLoc_Location local;
    const bool isReference = XCaf::isShapeReference(label);
    if (isReference) {
        local = XCaf::shapeReferenceLocation(label);
        here = loc * local;
        if (xc.hasShapeColor(label)) {
            inh.hasColor = true;
            inh.color = toRGB(xc.shapeColor(label));
        }
        if (std::string ln = xc.layerName(label); !ln.empty()) inh.layer = ln;
        refLabel = XCaf::shapeReferred(label);
    }
    std::string name = XCaf::labelName(label);
    if (name.empty() && !(refLabel == label)) name = XCaf::labelName(refLabel);

    if (XCaf::isShapeAssembly(refLabel)) {
        sawAssembly = true;
        AssemblyNode node;
        node.name = name;
        node.sourceDefinition = labelEntry(refLabel);
        node.sourceComponent = isReference ? labelEntry(label) : std::string{};
        node.parent = parent;
        node.isAssembly = true;
        node.solidId = -1;
        node.localTransform = toMatrix(local);
        node.transform = toMatrix(here);
        const int idx = static_cast<int>(out.assembly.size());
        out.assembly.push_back(node);
        out.leafBodyShapes.emplace_back();
        std::vector<int> childIdx;
        for (const TDF_Label& c : XCaf::shapeComponents(refLabel)) {
            int ci = walkLabel(xc, c, here, idx, inh, out, leaves, sawAssembly);
            if (ci >= 0) childIdx.push_back(ci);
        }
        out.assembly[idx].children = std::move(childIdx);
        return idx;
    }

    // A simple part.
    if (xc.hasShapeColor(refLabel)) {
        inh.hasColor = true;
        inh.color = toRGB(xc.shapeColor(refLabel));
    }
    if (std::string ln = xc.layerName(refLabel); !ln.empty()) inh.layer = ln;
    if (Handle(XCAFDoc_Material) mat = xc.shapeMaterial(refLabel);
        !mat.IsNull() && !mat->GetName().IsNull()) {
        inh.material = mat->GetName()->ToCString();
    }

    const TopoDS_Shape sh = XCaf::shape(refLabel);
    recordSolidMeta(sh, inh, out, name);

    // Per-face colors: own face-label color overrides inherited.
    for (const TDF_Label& fl : XCaf::shapeSubs(refLabel)) {
        TopoDS_Shape fsh = XCaf::shape(fl);
        if (fsh.IsNull() || fsh.ShapeType() != TopAbs_FACE) continue;
        const void* key = fsh.TShape().get();
        if (xc.hasShapeColor(fl)) {
            out.faceColor[key] = toRGB(xc.shapeColor(fl));
            out.faceHasColor[key] = true;
        }
        if (std::string ln = xc.layerName(fl); !ln.empty()) out.faceLayer[key] = ln;
    }
    // Faces without their own color inherit the part/instance color.
    if (inh.hasColor) {
        for (TopExp_Explorer fx(sh, TopAbs_FACE); fx.More(); fx.Next()) {
            const void* key = fx.Current().TShape().get();
            auto it = out.faceHasColor.find(key);
            if (it == out.faceHasColor.end() || !it->second) {
                out.faceColor[key] = inh.color;
                out.faceHasColor[key] = true;
            }
        }
    }

    ++leaves;
    AssemblyNode node;
    node.name = name;
    node.sourceDefinition = labelEntry(refLabel);
    node.sourceComponent = isReference ? labelEntry(label) : std::string{};
    node.parent = parent;
    node.isAssembly = false;
    node.solidId = -1;
    node.localTransform = toMatrix(local);
    node.transform = toMatrix(here);
    const int idx = static_cast<int>(out.assembly.size());
    out.assembly.push_back(node);
    TopoDS_Shape worldUse = XCaf::shape(label);
    if (!loc.IsIdentity()) worldUse.Move(loc);
    out.leafBodyShapes.push_back(bodyUses(worldUse));
    return idx;
}

// ---------------------------------------------------------- entry points ----

void captureMeta(const XCaf& xc, const LabelSequence& roots, ImportMeta& out) {
    int leaves = 0;
    bool sawAssembly = false;
    for (int i = 1; i <= roots.Length(); ++i)
        walkLabel(xc, roots.Value(i), TopLoc_Location(), -1, Inherited{}, out,
                  leaves, sawAssembly);
}

double readLengthUnit(const Handle(TDocStd_Document)& doc) {
    double f = 1.0;  // mm per model-unit
#if OCC_VERSION_HEX >= 0x070500
    if (!XCAFDoc_DocumentTool::GetLengthUnit(doc, f, UnitsMethods_LengthUnit_Millimeter))
        f = UnitsMethods::GetCasCadeLengthUnit();
#else
    f = UnitsMethods::GetCasCadeLengthUnit();
#endif
    if (!(f > 0.0)) f = 1.0;
    return f;
}

static std::vector<TopoDS_Shape> mappedFinals(const BRepTools_History& hist,
                                              const TopoDS_Shape& s) {
    std::vector<TopoDS_Shape> out;
    if (hist.IsRemoved(s)) return out;
    const ShapeList& mod = hist.Modified(s);
    if (!mod.IsEmpty()) {
        // Range-based iteration over the NCollection list — portable across
        // OCCT versions/platforms (the dedicated
        // TopTools_ListIteratorOfListOfShape.hxx header is absent in some
        // Windows OCCT packages).
        for (const TopoDS_Shape& sm : mod) out.push_back(sm);
    } else {
        out.push_back(s);  // identity: survived heal unchanged
    }
    return out;
}

static void fillUnresolvedByGeometry(Model& m, const TopoDS_Shape& origShape,
                                     const ImportMeta& meta) {
    struct Ref {
        std::array<double, 3> c;
        int type;
        std::array<float, 3> col;
    };
    std::vector<Ref> refs;
    for (TopExp_Explorer fx(origShape, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Face f = TopoDS::Face(fx.Current());
        const void* key = f.TShape().get();
        auto hc = meta.faceHasColor.find(key);
        if (hc == meta.faceHasColor.end() || !hc->second) continue;
        refs.push_back({faceCentroid(f), faceSurfType(f), meta.faceColor.at(key)});
    }
    if (refs.empty()) return;

    Bnd_Box box;
    BRepBndLib::Add(origShape, box);
    double tol2 = 1e-6;
    if (!box.IsVoid()) {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        double diag = std::sqrt((xmax - xmin) * (xmax - xmin) + (ymax - ymin) * (ymax - ymin) +
                                (zmax - zmin) * (zmax - zmin));
        double tol = 0.03 * diag;  // conservative: catches identity/merge misses
        tol2 = tol * tol;
    }
    for (int fid = 1; fid <= m.faces.Extent(); ++fid) {
        if (m.faceHasColor[fid - 1]) continue;
        const TopoDS_Face f = TopoDS::Face(m.faces(fid));
        std::array<double, 3> c = faceCentroid(f);
        int t = faceSurfType(f);
        double best = 1e300;
        int bi = -1;
        for (size_t i = 0; i < refs.size(); ++i) {
            if (refs[i].type != t) continue;
            double dx = c[0] - refs[i].c[0], dy = c[1] - refs[i].c[1], dz = c[2] - refs[i].c[2];
            double d = dx * dx + dy * dy + dz * dz;
            if (d < best) {
                best = d;
                bi = static_cast<int>(i);
            }
        }
        if (bi >= 0 && best <= tol2) {
            m.faceColors[fid - 1] = refs[bi].col;
            m.faceHasColor[fid - 1] = 1;
        }
    }
}

void reassociateMeta(Model& m, const TopoDS_Shape& origShape, const ImportMeta& meta,
                     const BRepTools_History& hist,
                     bool allowGeometricFallback) {
    const int nf = m.faces.Extent();
    m.faceColors.assign(nf, std::array<float, 3>{});
    m.faceHasColor.assign(nf, 0);

    // Forward-map every colored original face to its healed images.
    std::unordered_map<const void*, std::array<float, 3>> finalColor;
    for (TopExp_Explorer fx(origShape, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Shape& of = fx.Current();
        const void* key = of.TShape().get();
        auto hc = meta.faceHasColor.find(key);
        if (hc == meta.faceHasColor.end() || !hc->second) continue;
        const std::array<float, 3>& col = meta.faceColor.at(key);
        for (const TopoDS_Shape& ff : mappedFinals(hist, of))
            finalColor[ff.TShape().get()] = col;
    }
    for (int fid = 1; fid <= nf; ++fid) {
        auto it = finalColor.find(m.faces(fid).TShape().get());
        if (it != finalColor.end()) {
            m.faceColors[fid - 1] = it->second;
            m.faceHasColor[fid - 1] = 1;
        }
    }
    // The legacy healing adapter retains its historical safety net. Secure
    // import forbids it: geometry proximity cannot prove source provenance.
    if (allowGeometricFallback) {
        fillUnresolvedByGeometry(m, origShape, meta);
    }
}

void fillSolidMetaFromCaf(Model& m, const TopoDS_Shape& origShape, const ImportMeta& meta) {
    const int ns = m.solids.Extent();
    m.solidColors.assign(ns, std::array<float, 3>{});
    m.solidHasColor.assign(ns, 0);
    m.solidLayers.assign(ns, std::string());
    m.solidMaterials.assign(ns, std::string());

    // Original bodies in the same order indexShape enumerates the healed ones
    // (solids first, then shells outside solids).
    std::vector<TopoDS_Shape> origBodies;
    for (TopExp_Explorer sx(origShape, TopAbs_SOLID); sx.More(); sx.Next())
        origBodies.push_back(sx.Current());
    for (TopExp_Explorer sx(origShape, TopAbs_SHELL, TopAbs_SOLID); sx.More(); sx.Next())
        origBodies.push_back(sx.Current());
    if (static_cast<int>(origBodies.size()) != ns) return;  // count changed: leave defaults

    std::map<std::string, int> used;
    for (int i = 0; i < ns; ++i) {
        const void* key = origBodies[i].TShape().get();
        if (auto it = meta.solidName.find(key); it != meta.solidName.end() && !it->second.empty()) {
            std::string name = it->second;
            int n = ++used[name];
            if (n > 1) name += "_" + std::to_string(n);
            m.solidNames[i] = std::move(name);
        }
        if (auto it = meta.solidColor.find(key); it != meta.solidColor.end()) {
            m.solidColors[i] = it->second;
            m.solidHasColor[i] = 1;
        }
        if (auto it = meta.solidLayer.find(key); it != meta.solidLayer.end())
            m.solidLayers[i] = it->second;
        if (auto it = meta.solidMaterial.find(key); it != meta.solidMaterial.end())
            m.solidMaterials[i] = it->second;
    }
}

static void resolveAssemblySolidIds(const TopoDS_Shape& origShape, const Model& m,
                                    ImportMeta& meta) {
    std::vector<TopoDS_Shape> origBodies;
    for (TopExp_Explorer sx(origShape, TopAbs_SOLID); sx.More(); sx.Next())
        origBodies.push_back(sx.Current());
    for (TopExp_Explorer sx(origShape, TopAbs_SHELL, TopAbs_SOLID); sx.More(); sx.Next())
        origBodies.push_back(sx.Current());
    if (static_cast<int>(origBodies.size()) != m.solids.Extent()) return;
    std::vector<char> consumed(origBodies.size(), 0);
    for (size_t i = 0; i < meta.assembly.size(); ++i) {
        if (i >= meta.leafBodyShapes.size()) break;
        AssemblyNode& node = meta.assembly[i];
        std::vector<std::size_t> matches;
        for (const TopoDS_Shape& body : meta.leafBodyShapes[i]) {
            std::size_t match = origBodies.size();
            for (std::size_t candidate = 0; candidate < origBodies.size();
                 ++candidate) {
                if (!consumed[candidate] &&
                    std::find(matches.begin(), matches.end(), candidate) ==
                        matches.end() &&
                    origBodies[candidate].IsSame(body)) {
                    match = candidate;
                    break;
                }
            }
            if (match == origBodies.size()) {
                matches.clear();
                break;
            }
            matches.push_back(match);
        }
        node.solidIds.clear();
        for (std::size_t match : matches) {
            consumed[match] = 1;
            node.solidIds.push_back(static_cast<int>(match) + 1);
        }
        node.solidId = node.solidIds.empty() ? -1 : node.solidIds.front();
    }
}

Model cafToModel(const TopoDS_Shape& oneShape, const Handle(TDocStd_Document)& doc) {
    XCaf xc(doc);
    LabelSequence roots = xc.topLevelFreeShapes();

    ImportMeta meta;
    captureMeta(xc, roots, meta);
    meta.lengthUnitMm = readLengthUnit(doc);

    Handle(BRepTools_History) hist;
    TopoDS_Shape healed = weft::healWithHistory(oneShape, hist);

    Model m = weft::indexShape(healed);
    reassociateMeta(m, oneShape, meta, *hist);
    fillSolidMetaFromCaf(m, oneShape, meta);
    m.lengthUnitMm = meta.lengthUnitMm;

    // Hierarchy is metadata only; a flat single body keeps assembly empty.
    if (m.solids.Extent() <= 1) {
        meta.assembly.clear();
    } else {
        resolveAssemblySolidIds(oneShape, m, meta);
    }
    m.assembly = std::move(meta.assembly);
    return m;
}

namespace {

void applyCapturedMeta(Model& model, const TopoDS_Shape& sourceShape,
                       ImportMeta meta, const BRepTools_History& history) {
    reassociateMeta(model, sourceShape, meta, history, false);
    fillSolidMetaFromCaf(model, sourceShape, meta);
    model.lengthUnitMm = meta.lengthUnitMm;

    // Hierarchy is metadata only; a flat single body keeps assembly empty.
    if (model.solids.Extent() <= 1) {
        meta.assembly.clear();
    } else {
        resolveAssemblySolidIds(sourceShape, model, meta);
    }
    model.assembly = std::move(meta.assembly);
}

Handle(BRepTools_History) composeCopyRepairHistory(
    const TopoDS_Shape& sourceShape, const BRepBuilderAPI_Copy& copier,
    const Handle(BRepTools_History)& repairHistory) {
    Handle(BRepTools_History) composed = new BRepTools_History();
    for (TopAbs_ShapeEnum kind : {TopAbs_FACE, TopAbs_EDGE}) {
        for (TopExp_Explorer explorer(sourceShape, kind); explorer.More();
             explorer.Next()) {
            const TopoDS_Shape& source = explorer.Current();
            const TopoDS_Shape copied = copier.ModifiedShape(source);
            if (copied.IsNull() ||
                (!repairHistory.IsNull() && repairHistory->IsRemoved(copied))) {
                composed->Remove(source);
                continue;
            }
            bool mapped = false;
            if (!repairHistory.IsNull()) {
                const ShapeList& repaired = repairHistory->Modified(copied);
                for (const TopoDS_Shape& finalShape : repaired) {
                    composed->AddModified(source, finalShape);
                    mapped = true;
                }
            }
            if (!mapped) composed->AddModified(source, copied);
        }
    }
    return composed;
}

}  // namespace

ImportedModel cafToImportedModel(const TopoDS_Shape& oneShape,
                                 const Handle(TDocStd_Document)& doc,
                                 SourceMetadata metadata,
                                 RepairProfile profile) {
    XCaf xc(doc);
    ImportMeta captured;
    captureMeta(xc, xc.topLevelFreeShapes(), captured);
    captured.lengthUnitMm = readLengthUnit(doc);

    // Build source evidence directly from the processing-disabled transfer.
    // An empty history deliberately means every source entity is an identity.
    Handle(BRepTools_History) sourceHistory = new BRepTools_History();
    Model source = weft::indexShape(oneShape);
    applyCapturedMeta(source, oneShape, captured, *sourceHistory);

    TopoDS_Shape workingShape = oneShape;
    Handle(BRepTools_History) workingHistory = new BRepTools_History();
    std::vector<RepairOperation> operations;
    if (profile == RepairProfile::Compatibility) {
        // Compatibility APIs receive a geometry-deep working copy. A repair
        // can therefore never alter a TShape or geometry handle retained by
        // SourceBRep. Compose copy and repair mappings back to source IDs.
        BRepBuilderAPI_Copy copier(oneShape, true, false);
        Handle(BRepTools_History) repairHistory;
        workingShape = weft::healWithHistory(copier.Shape(), repairHistory);
        workingHistory = composeCopyRepairHistory(
            oneShape, copier, repairHistory);
        operations.push_back({
            "repair.compatibility_pipeline", {}, {},
            "historical Weft healing pipeline applied to a geometry-deep working copy after immutable source capture"});
    }

    Model working = weft::indexShape(workingShape);
    applyCapturedMeta(working, oneShape, captured, *workingHistory);
    return secure_detail::buildImportedModel(
        std::move(source), std::move(working), std::move(metadata), profile,
        workingHistory, std::move(operations));
}

}  // namespace weft::io
