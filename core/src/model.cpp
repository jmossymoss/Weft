#include "weft/model.hpp"

#include "io/xcaf.hpp"  // declares weft::indexShape / weft::healWithHistory
#include "weft/io/reader.hpp"
#include "weft/io/system.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepTools_History.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>

#include <memory>
#include <stdexcept>

namespace weft {

Model indexShape(const TopoDS_Shape& shape) {
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
    for (TopExp_Explorer sx(shape, TopAbs_SHELL, TopAbs_SOLID); sx.More(); sx.Next()) {
        m.solids.Add(sx.Current());
    }
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m.edgeToFaces);
    m.solidNames.assign(m.solids.Extent(), std::string());
    return m;
}

// Build a per-stage history for a modifier that exposes IsModified/Modified
// (faces) and IsModifiedSubShape/ModifiedSubShape (edges) — i.e. sewing.
// Records only same-type modifications (BRepTools_History supports face/edge,
// not compounds a split might yield); anything skipped is caught by the
// geometric fallback during re-association.
static Handle(BRepTools_History) historyOfSewing(const TopoDS_Shape& before,
                                                 BRepBuilderAPI_Sewing& sew) {
    Handle(BRepTools_History) h = new BRepTools_History();
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(before, TopAbs_FACE, faces);
    for (int i = 1; i <= faces.Extent(); ++i) {
        const TopoDS_Shape& s = faces(i);
        if (sew.IsModified(s)) {
            const TopoDS_Shape& r = sew.Modified(s);
            if (!r.IsNull() && !r.IsSame(s) && r.ShapeType() == s.ShapeType())
                h->AddModified(s, r);
        }
    }
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(before, TopAbs_EDGE, edges);
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Shape& s = edges(i);
        if (sew.IsModifiedSubShape(s)) {
            TopoDS_Shape r = sew.ModifiedSubShape(s);
            if (!r.IsNull() && !r.IsSame(s) && r.ShapeType() == s.ShapeType())
                h->AddModified(s, r);
        }
    }
    return h;
}

// Build a per-stage history from a ShapeFix ReShape context by applying it to
// each original face/edge.
static Handle(BRepTools_History) historyOfReShape(const TopoDS_Shape& before,
                                                  const Handle(ShapeBuild_ReShape)& ctx) {
    Handle(BRepTools_History) h = new BRepTools_History();
    if (ctx.IsNull()) return h;
    for (TopAbs_ShapeEnum type : {TopAbs_FACE, TopAbs_EDGE}) {
        TopTools_IndexedMapOfShape map;
        TopExp::MapShapes(before, type, map);
        for (int i = 1; i <= map.Extent(); ++i) {
            const TopoDS_Shape& s = map(i);
            TopoDS_Shape r = ctx->Apply(s);
            if (r.IsNull())
                h->Remove(s);
            else if (!r.IsSame(s) && r.ShapeType() == s.ShapeType())
                h->AddModified(s, r);
        }
    }
    return h;
}

TopoDS_Shape healWithHistory(const TopoDS_Shape& input, Handle(BRepTools_History)& outHist) {
    outHist = new BRepTools_History();
    TopoDS_Shape shape = input;

    // Sew faces that arrive with their own duplicate copies of shared
    // edges (common in some exporters): unshared edges can't take part in
    // density matching or welding, leaving open seams through the model.
    TopoDS_Shape preSew = shape;
    BRepBuilderAPI_Sewing sewing(1e-4);
    sewing.Add(shape);
    sewing.Perform();
    if (!sewing.SewedShape().IsNull()) shape = sewing.SewedShape();
    outHist->Merge(historyOfSewing(preSew, sewing));

    TopoDS_Shape preFix = shape;
    ShapeFix_Shape fixer(shape);
    fixer.Perform();
    shape = fixer.Shape();
    outHist->Merge(historyOfReShape(preFix, fixer.Context()));

    // STEP kernels split closed revolves into half-faces, so a bore
    // arrives as two half-cylinders with seam lines and split rim arcs.
    // Unify faces that lie on one revolved surface back into a single
    // periodic face (and tangent same-curve edge chains into single
    // edges — the rim halves become one closed circle) so holes and
    // bosses solve as single revolution rings. Planar and freeform
    // faces are kept as authored: only revolved geometry is healed.
    {
        ShapeUpgrade_UnifySameDomain unify(shape, /*UnifyEdges*/ true,
                                           /*UnifyFaces*/ true,
                                           /*ConcatBSplines*/ false);
        for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) {
            BRepAdaptor_Surface s(TopoDS::Face(fx.Current()), false);
            switch (s.GetType()) {
                case GeomAbs_Cylinder:
                case GeomAbs_Cone:
                case GeomAbs_Sphere:
                case GeomAbs_Torus:
                case GeomAbs_SurfaceOfRevolution:
                    break;
                default:
                    unify.KeepShape(fx.Current());
            }
        }
        unify.Build();
        if (!unify.Shape().IsNull()) shape = unify.Shape();
        if (!unify.History().IsNull()) outHist->Merge(unify.History());
    }
    // Second, unscoped edge pass: tangent same-curve chains merge into
    // single edges everywhere (the kept planar faces blocked arc merges
    // along their wires in the scoped pass above, leaving one rim of a
    // band as a full circle and the other as two halves — a structural
    // count mismatch). Feature circles come out as ONE closed edge.
    {
        ShapeUpgrade_UnifySameDomain unify(shape, /*UnifyEdges*/ true,
                                           /*UnifyFaces*/ false,
                                           /*ConcatBSplines*/ true);
        unify.Build();
        if (!unify.Shape().IsNull()) shape = unify.Shape();
        if (!unify.History().IsNull()) outHist->Merge(unify.History());
    }

    return shape;
}

Model loadStep(const std::string& path) {
    // Route through the content-detected registry so .stp/.step/mis-extended
    // files and (now) IGES/BREP inputs all import; preserve the old
    // "assume STEP on an unrecognized file" behavior.
    io::System sys;
    io::bootstrapIo(sys);
    io::Format f = sys.probeFormat(path);
    if (f == io::Format::Unknown) f = io::Format::Step;
    std::unique_ptr<io::Reader> reader = sys.createReader(f);
    if (!reader) reader = sys.createReader(io::Format::Step);
    if (!reader || !reader->readFile(path)) {
        throw std::runtime_error("failed to read STEP file: " + path);
    }
    return reader->transfer();
}

void writeStep(const TopoDS_Shape& shape, const std::string& path) {
    STEPControl_Writer writer;
    writer.Transfer(shape, STEPControl_AsIs);
    if (writer.Write(path.c_str()) != IFSelect_RetDone) {
        throw std::runtime_error("failed to write STEP file: " + path);
    }
}

}  // namespace weft
