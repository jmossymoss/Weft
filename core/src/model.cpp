#include "weft/model.hpp"

#include "io/xcaf.hpp"  // declares weft::indexShape / weft::healWithHistory
#include "weft/io/reader.hpp"
#include "weft/io/system.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepFill_Filling.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_Shape.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <ShapeProcess.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeBuild_ReShape.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>

#include <memory>
#include <stdexcept>
#include <vector>

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

// Count a shape's open-shell edges: non-degenerate edges bordering fewer
// than two faces. The watertightness gate for the capping pass below.
static int countOpenShellEdges(const TopoDS_Shape& shape) {
    TopTools_IndexedDataMapOfShapeListOfShape e2f;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, e2f);
    int open = 0;
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
        if (BRep_Tool::Degenerated(e)) continue;
        if (e2f(i).Extent() < 2) ++open;
    }
    return open;
}

// Reconstruct faces the translator dropped. A STEP solid whose
// OFFSET_SURFACE (or other exotic) faces fail to build arrives as a
// nearly-closed shell with holes — every hole a CLOSED loop of boundary
// edges (foam: 881 faces declared, 877 transferred, 3 loops). No sewing
// tolerance can close a hole whose face does not exist, so every mesh of
// the model leaks exactly there. Cap each loop with a real B-rep face
// (planar when the loop is planar, a filling patch otherwise) and sew it
// in, so the border contract makes the caps watertight like any face.
//
// Authored sheet bodies must NOT be capped: a shell that is mostly
// boundary (tork: 51-89% of its edges open) is an open surface model by
// design. Broken solids are nearly closed (foam: 1.7-28.6%), so the pass
// only treats shells less than one third open.
static TopoDS_Shape capDroppedFaces(const TopoDS_Shape& shape,
                                    Handle(BRepTools_History)& outHist) {
    std::vector<TopoDS_Shape> caps;
    for (TopExp_Explorer sx(shape, TopAbs_SHELL); sx.More(); sx.Next()) {
        TopTools_IndexedDataMapOfShapeListOfShape e2f;
        TopExp::MapShapesAndAncestors(sx.Current(), TopAbs_EDGE, TopAbs_FACE,
                                      e2f);
        std::vector<TopoDS_Edge> boundary;
        int total = 0;
        for (int i = 1; i <= e2f.Extent(); ++i) {
            const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
            if (BRep_Tool::Degenerated(e)) continue;
            ++total;
            if (e2f(i).Extent() < 2) boundary.push_back(e);
        }
        if (boundary.empty() || int(boundary.size()) * 3 >= total) continue;

        // Chain the boundary edges into loops by shared vertices.
        std::vector<char> used(boundary.size(), 0);
        for (size_t s = 0; s < boundary.size(); ++s) {
            if (used[s]) continue;
            std::vector<TopoDS_Edge> loop{boundary[s]};
            used[s] = 1;
            TopoDS_Vertex v0, cur;
            TopExp::Vertices(boundary[s], v0, cur);
            if (v0.IsNull() || cur.IsNull()) continue;
            // A single closed-curve edge (unify merges a hole's rim into
            // one periodic edge) is already a complete loop.
            bool closed = v0.IsSame(cur);
            while (!closed && loop.size() < 64) {
                bool advanced = false;
                for (size_t j = 0; j < boundary.size(); ++j) {
                    if (used[j]) continue;
                    TopoDS_Vertex a, b;
                    TopExp::Vertices(boundary[j], a, b);
                    if (a.IsNull() || b.IsNull()) continue;
                    if (a.IsSame(cur) || b.IsSame(cur)) {
                        cur = a.IsSame(cur) ? b : a;
                        loop.push_back(boundary[j]);
                        used[j] = 1;
                        advanced = true;
                        break;
                    }
                }
                if (!advanced) break;
                closed = cur.IsSame(v0);
            }
            if (!closed) continue;

            try {
                BRepBuilderAPI_MakeWire mw;
                for (const TopoDS_Edge& e : loop) mw.Add(e);
                if (!mw.IsDone()) continue;
                const TopoDS_Wire wire = mw.Wire();
                TopoDS_Face cap;
                {
                    // A planar loop takes an exact planar cap.
                    BRepBuilderAPI_MakeFace mf(wire, Standard_True);
                    if (mf.IsDone()) cap = mf.Face();
                }
                if (cap.IsNull()) {
                    BRepFill_Filling fill;
                    for (const TopoDS_Edge& e : loop) {
                        fill.Add(e, GeomAbs_C0);
                    }
                    fill.Build();
                    if (fill.IsDone()) cap = fill.Face();
                }
                if (!cap.IsNull()) caps.push_back(cap);
            } catch (const Standard_Failure&) {
                // A loop the filler can't express stays open — honest
                // output beats a corrupt patch.
            }
        }
    }
    if (caps.empty()) return shape;

    const int before = countOpenShellEdges(shape);
    try {
        TopoDS_Shape preSew = shape;
        BRepBuilderAPI_Sewing sew(1e-4);
        sew.Add(shape);
        for (const TopoDS_Shape& c : caps) sew.Add(c);
        sew.Perform();
        TopoDS_Shape sewn = sew.SewedShape();
        if (!sewn.IsNull() && countOpenShellEdges(sewn) < before) {
            outHist->Merge(historyOfSewing(preSew, sew));
            return sewn;
        }
    } catch (const Standard_Failure&) {
    }
    return shape;
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

    // Cap holes left by faces the translator dropped (nearly-closed
    // shells only; authored sheet bodies keep their boundary).
    shape = capDroppedFaces(shape, outHist);

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
    // STEPCAFControl_Controller::Init() changes which process-global ToSTEP
    // operations a later writer sees.  Pin the standard OCCT STEP operations
    // explicitly so fixtures serialize identically no matter what this process
    // imported earlier (an apex cone otherwise loses its lateral face).
    ShapeProcess::OperationsFlags processFlags;
    processFlags.set(ShapeProcess::SplitCommonVertex);
    processFlags.set(ShapeProcess::DirectFaces);
    writer.SetShapeProcessFlags(processFlags);
    if (writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone ||
        writer.Write(path.c_str()) != IFSelect_RetDone) {
        throw std::runtime_error("failed to write STEP file: " + path);
    }
}

}  // namespace weft
