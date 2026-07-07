#include "xcaf.hpp"

#include <Standard_Version.hxx>

#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TColStd_HSequenceOfExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
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

TDF_LabelSequence XCaf::topLevelFreeShapes() const {
    TDF_LabelSequence seq;
    shapeTool()->GetFreeShapes(seq);
    return seq;
}

bool XCaf::isShapeAssembly(const TDF_Label& l) { return XCAFDoc_ShapeTool::IsAssembly(l); }
bool XCaf::isShapeReference(const TDF_Label& l) { return XCAFDoc_ShapeTool::IsReference(l); }
bool XCaf::isShapeSimple(const TDF_Label& l) { return XCAFDoc_ShapeTool::IsSimpleShape(l); }

TDF_LabelSequence XCaf::shapeComponents(const TDF_Label& l) {
    TDF_LabelSequence seq;
    XCAFDoc_ShapeTool::GetComponents(l, seq);
    return seq;
}
TDF_LabelSequence XCaf::shapeSubs(const TDF_Label& l) {
    TDF_LabelSequence seq;
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
    const Standard_Integer len = es.LengthOfCString();
    if (len <= 0) return {};
    std::string out(static_cast<size_t>(len) + 1, '\0');
    Standard_PCharacter p = out.data();
    es.ToUTF8CString(p);
    out.resize(std::char_traits<char>::length(out.c_str()));
    return out;
}

std::string XCaf::layerName(const TDF_Label& l) const {
    Handle(XCAFDoc_LayerTool) lt = layerTool();
    Handle(TColStd_HSequenceOfExtendedString) layers;
    if (!lt->GetLayers(l, layers) || layers.IsNull() || layers->IsEmpty()) return {};
    const TCollection_ExtendedString& es = layers->Value(1);
    const Standard_Integer len = es.LengthOfCString();
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

static const void* firstBodyKey(const TopoDS_Shape& sh) {
    for (TopExp_Explorer sx(sh, TopAbs_SOLID); sx.More(); sx.Next())
        return sx.Current().TShape().get();
    for (TopExp_Explorer sx(sh, TopAbs_SHELL, TopAbs_SOLID); sx.More(); sx.Next())
        return sx.Current().TShape().get();
    return nullptr;
}

// Recursive assembly walk; returns the index of the node created in
// out.assembly (or -1). Metadata is stamped on ORIGINAL faces/solids.
static int walkLabel(const XCaf& xc, const TDF_Label& label, const TopLoc_Location& loc,
                     Inherited inh, ImportMeta& out, int& leaves, bool& sawAssembly) {
    TDF_Label refLabel = label;
    TopLoc_Location here = loc;
    if (XCaf::isShapeReference(label)) {
        here = loc * XCaf::shapeReferenceLocation(label);
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
        node.solidId = -1;
        node.transform = toMatrix(here);
        const int idx = static_cast<int>(out.assembly.size());
        out.assembly.push_back(node);
        out.leafSolidKey.push_back(nullptr);
        std::vector<int> childIdx;
        for (const TDF_Label& c : XCaf::shapeComponents(refLabel)) {
            int ci = walkLabel(xc, c, here, inh, out, leaves, sawAssembly);
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
    node.solidId = -1;
    node.transform = toMatrix(here);
    const int idx = static_cast<int>(out.assembly.size());
    out.assembly.push_back(node);
    out.leafSolidKey.push_back(firstBodyKey(sh));
    return idx;
}

// ---------------------------------------------------------- entry points ----

void captureMeta(const XCaf& xc, const TDF_LabelSequence& roots, ImportMeta& out) {
    int leaves = 0;
    bool sawAssembly = false;
    for (Standard_Integer i = 1; i <= roots.Length(); ++i)
        walkLabel(xc, roots.Value(i), TopLoc_Location(), Inherited{}, out, leaves, sawAssembly);
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
    const TopTools_ListOfShape& mod = hist.Modified(s);
    if (!mod.IsEmpty()) {
        for (TopTools_ListIteratorOfListOfShape it(mod); it.More(); it.Next())
            out.push_back(it.Value());
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
                     const BRepTools_History& hist) {
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
    // Safety net for faces history could not chain (sewing can drop records).
    fillUnresolvedByGeometry(m, origShape, meta);
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
    std::unordered_map<const void*, int> solidIdByKey;
    for (size_t i = 0; i < origBodies.size(); ++i)
        solidIdByKey[origBodies[i].TShape().get()] = static_cast<int>(i) + 1;
    for (size_t i = 0; i < meta.assembly.size(); ++i) {
        const void* key = (i < meta.leafSolidKey.size()) ? meta.leafSolidKey[i] : nullptr;
        if (!key) continue;
        if (auto it = solidIdByKey.find(key); it != solidIdByKey.end())
            meta.assembly[i].solidId = it->second;
    }
}

Model cafToModel(const TopoDS_Shape& oneShape, const Handle(TDocStd_Document)& doc) {
    XCaf xc(doc);
    TDF_LabelSequence roots = xc.topLevelFreeShapes();

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

}  // namespace weft::io
