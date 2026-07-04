#include "weft/model.hpp"

#include <BRepBuilderAPI_Sewing.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeFix_Shape.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TransferBRep.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>

#include <map>
#include <stdexcept>
#include <vector>

namespace weft {

static Model indexShape(const TopoDS_Shape& shape) {
    Model m;
    m.shape = shape;
    TopExp::MapShapes(shape, TopAbs_FACE, m.faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, m.edges);
    TopExp::MapShapes(shape, TopAbs_SOLID, m.solids);
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m.edgeToFaces);
    m.solidNames.assign(m.solids.Extent(), std::string());
    return m;
}

// Names carried by the STEP representation items (MANIFOLD_SOLID_BREP
// and friends — Plasticity writes its object names there), keyed by the
// underlying TShape so assembly placements don't break the lookup.
// Forward walk over the transfer map; the reverse lookup
// (EntityFromShapeResult) misses located instances.
static std::map<const void*, std::string> shapeNames(
    const STEPControl_Reader& reader) {
    std::map<const void*, std::string> names;
    const Handle(XSControl_TransferReader)& tr =
        reader.WS()->TransferReader();
    if (tr.IsNull()) return names;
    Handle(Transfer_TransientProcess) tp = tr->TransientProcess();
    if (tp.IsNull()) return names;
    for (int i = 1; i <= tp->NbMapped(); ++i) {
        Handle(StepRepr_RepresentationItem) item =
            Handle(StepRepr_RepresentationItem)::DownCast(tp->Mapped(i));
        if (item.IsNull() || item->Name().IsNull() ||
            item->Name()->Length() == 0) {
            continue;
        }
        TopoDS_Shape sh = TransferBRep::ShapeResult(tp, tp->Mapped(i));
        if (sh.IsNull()) continue;
        names.try_emplace(sh.TShape().get(), item->Name()->ToCString());
    }
    return names;
}

// Name for one body: the solid itself, or its shell (some writers name
// the shell-based representation instead).
static std::string solidName(const std::map<const void*, std::string>& names,
                             const TopoDS_Shape& solid) {
    auto it = names.find(solid.TShape().get());
    if (it != names.end()) return it->second;
    for (TopExp_Explorer sx(solid, TopAbs_SHELL); sx.More(); sx.Next()) {
        it = names.find(sx.Current().TShape().get());
        if (it != names.end()) return it->second;
    }
    return {};
}

Model loadStep(const std::string& path) {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
    if (status != IFSelect_RetDone) {
        throw std::runtime_error("failed to read STEP file: " + path);
    }
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        throw std::runtime_error("STEP file contained no transferable shapes: " + path);
    }

    // Capture per-solid names BEFORE healing (the transfer map knows the
    // reader's original shapes, not the healed copies)...
    const std::map<const void*, std::string> names = shapeNames(reader);
    std::vector<std::string> rawNames;
    for (TopExp_Explorer sx(shape, TopAbs_SOLID); sx.More(); sx.Next()) {
        rawNames.push_back(solidName(names, sx.Current()));
    }

    ShapeFix_Shape fixer(shape);
    fixer.Perform();
    shape = fixer.Shape();

    Model m = indexShape(shape);
    // ...then pair them with the healed solids by traversal order, which
    // healing preserves; bail to anonymous parts if the count changed.
    if ((int)rawNames.size() == m.solids.Extent()) {
        size_t i = 0;
        for (TopExp_Explorer sx(shape, TopAbs_SOLID); sx.More();
             sx.Next(), ++i) {
            int sid = m.solids.FindIndex(sx.Current());
            if (sid >= 1) m.solidNames[sid - 1] = rawNames[i];
        }
    }
    return m;
}

void writeStep(const TopoDS_Shape& shape, const std::string& path) {
    STEPControl_Writer writer;
    writer.Transfer(shape, STEPControl_AsIs);
    if (writer.Write(path.c_str()) != IFSelect_RetDone) {
        throw std::runtime_error("failed to write STEP file: " + path);
    }
}

}  // namespace weft
