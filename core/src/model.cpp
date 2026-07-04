#include "weft/model.hpp"

#include <BRepBuilderAPI_Sewing.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <ShapeFix_Shape.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TransferBRep.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <cstdio>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <vector>

namespace weft {

static Model indexShape(const TopoDS_Shape& shape) {
    Model m;
    m.shape = shape;
    TopExp::MapShapes(shape, TopAbs_FACE, m.faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, m.edges);
    // Bodies in the same order analysis enumerates them (solids, then
    // shells outside solids) — sewing can demote an assembly's solids to
    // shells, and names must stay attached to the right body.
    for (TopExp_Explorer sx(shape, TopAbs_SOLID); sx.More(); sx.Next()) {
        m.solids.Add(sx.Current());
    }
    for (TopExp_Explorer sx(shape, TopAbs_SHELL, TopAbs_SOLID); sx.More();
         sx.Next()) {
        m.solids.Add(sx.Current());
    }
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m.edgeToFaces);
    m.solidNames.assign(m.solids.Extent(), std::string());
    return m;
}

// Names carried by the STEP representation items (MANIFOLD_SOLID_BREP and
// friends — Plasticity writes its object names there), keyed by the
// underlying TShape so assembly placements don't break the lookup. Forward
// walk over the transfer map; the reverse lookup (EntityFromShapeResult)
// misses located instances.
static std::map<const void*, std::string> shapeNames(
    const STEPControl_Reader& reader) {
    std::map<const void*, std::string> names;
    const Handle(XSControl_TransferReader)& tr = reader.WS()->TransferReader();
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

    // Names must be read off the reader's ORIGINAL shapes — sewing and
    // healing rebuild the TShapes the transfer map is keyed by.
    const std::map<const void*, std::string> names = shapeNames(reader);
    std::vector<std::string> rawNames;
    for (TopExp_Explorer sx(shape, TopAbs_SOLID); sx.More(); sx.Next()) {
        rawNames.push_back(solidName(names, sx.Current()));
    }
    for (TopExp_Explorer sx(shape, TopAbs_SHELL, TopAbs_SOLID); sx.More();
         sx.Next()) {
        rawNames.push_back(solidName(names, sx.Current()));
    }

    // Sew faces that arrive with their own duplicate copies of shared
    // edges (common in some exporters): unshared edges can't take part in
    // density matching or welding, leaving open seams through the model.
    BRepBuilderAPI_Sewing sewing(1e-4);
    sewing.Add(shape);
    sewing.Perform();
    if (!sewing.SewedShape().IsNull()) shape = sewing.SewedShape();

    ShapeFix_Shape fixer(shape);
    fixer.Perform();
    shape = fixer.Shape();

    Model m = indexShape(shape);
    // ...then pair them with the healed solids by traversal order, which
    // sew/heal preserve; bail to anonymous parts if the count changed.
    if ((int)rawNames.size() == m.solids.Extent()) {
        // Instanced parts share one product name; suffix duplicates so
        // importers keep them as separate objects.
        std::map<std::string, int> used;
        for (int sid = 1; sid <= m.solids.Extent(); ++sid) {
            std::string name = rawNames[sid - 1];
            if (name.empty()) continue;
            int n = ++used[name];
            if (n > 1) name += "_" + std::to_string(n);
            m.solidNames[sid - 1] = std::move(name);
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
