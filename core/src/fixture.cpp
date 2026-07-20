#include "weft/fixture.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepLib.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <ShapeFix_Shape.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <NCollection_Array1.hxx>
#include <NCollection_Array2.hxx>
#include <NCollection_List.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <BRep_Tool.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Hyperbola.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_Parabola.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Vec2d.hxx>

#include <cmath>
#include <stdexcept>

namespace weft {

TopoDS_Shape makeFixture(const std::string& name) {
    if (name == "cylinder") {
        return BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape();
    }
    if (name == "partial_cylinder") {
        // 270-degree open cylindrical band with two linear side rails.
        return BRepPrimAPI_MakeCylinder(10.0, 30.0, 1.5 * M_PI).Shape();
    }
    if (name == "box") {
        return BRepPrimAPI_MakeBox(20.0, 30.0, 15.0).Shape();
    }
    if (name == "cone") {
        return BRepPrimAPI_MakeCone(10.0, 0.0, 20.0).Shape();
    }
    if (name == "truncated_cone") {
        // Frustum / non-apex cone band (r1=10, r2=4, h=20) for the
        // revolved-band consumer shared with cylinders.
        return BRepPrimAPI_MakeCone(10.0, 4.0, 20.0).Shape();
    }
    if (name == "sphere") {
        return BRepPrimAPI_MakeSphere(10.0).Shape();
    }
    if (name == "sphere_cap") {
        // Upper hemisphere: cut the lower half-space from a full sphere so
        // the curved face is a single-pole cap (+ planar disk).
        const TopoDS_Shape ball = BRepPrimAPI_MakeSphere(10.0).Shape();
        const TopoDS_Shape cutter =
            BRepPrimAPI_MakeBox(gp_Pnt(-20.0, -20.0, -20.0), 40.0, 40.0, 20.0)
                .Shape();
        return BRepAlgoAPI_Cut(ball, cutter).Shape();
    }
    if (name == "torus") {
        return BRepPrimAPI_MakeTorus(10.0, 3.0).Shape();
    }
    if (name == "fillet") {
        // Analytic fillet strip: plate + convex quarter-cylinder bead along
        // one long edge (planes + PeriodicBandCrossingSeam cylinder).
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(20.0, 30.0, 4.0).Shape();
        const gp_Ax2 ax(gp_Pnt(20.0, 0.0, 4.0), gp_Dir(0.0, 1.0, 0.0),
                        gp_Dir(1.0, 0.0, 0.0));
        TopoDS_Shape bead =
            BRepPrimAPI_MakeCylinder(ax, 4.0, 30.0, 0.5 * M_PI).Shape();
        return BRepAlgoAPI_Fuse(plate, bead).Shape();
    }
    if (name == "demo") {
        // The demo IS the torture scene: one solid carrying the
        // campaign's issue classes (stacked barrel + blend, slot through
        // the wall, rim-open notch, crossing bores, hairline and large
        // fillets, angled struts, micro-chamfer) — the model a build is
        // judged on. The old cylinder+box compound lives on as the
        // separate primitives fixtures.
        return makeFixture("torture");
    }
    if (name == "hole") {
        // Plate with a through-bore: two ring-junction faces + a bore wall.
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
        gp_Ax2 axis(gp_Pnt(20.0, 20.0, -1.0), gp_Dir(0, 0, 1));
        TopoDS_Shape drill = BRepPrimAPI_MakeCylinder(axis, 8.0, 12.0).Shape();
        return BRepAlgoAPI_Cut(plate, drill).Shape();
    }
    if (name == "ellipse_hole") {
        // Planar plate with an exact elliptical inner wire (major=10,
        // minor=5), extruded to a solid. Avoid boolean cuts — they often
        // approximate ellipses as bsplines and defeat WP-171 proofs.
        const gp_Pln plane(gp_Ax3(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0, 0, 1)));
        const TopoDS_Edge o1 =
            BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(40, 0, 0)).Edge();
        const TopoDS_Edge o2 =
            BRepBuilderAPI_MakeEdge(gp_Pnt(40, 0, 0), gp_Pnt(40, 40, 0)).Edge();
        const TopoDS_Edge o3 =
            BRepBuilderAPI_MakeEdge(gp_Pnt(40, 40, 0), gp_Pnt(0, 40, 0)).Edge();
        const TopoDS_Edge o4 =
            BRepBuilderAPI_MakeEdge(gp_Pnt(0, 40, 0), gp_Pnt(0, 0, 0)).Edge();
        BRepBuilderAPI_MakeWire outerMaker;
        outerMaker.Add(o1);
        outerMaker.Add(o2);
        outerMaker.Add(o3);
        outerMaker.Add(o4);
        const TopoDS_Wire outer = outerMaker.Wire();
        Handle(Geom_Ellipse) ellipse = new Geom_Ellipse(
            gp_Ax2(gp_Pnt(20.0, 20.0, 0.0), gp_Dir(0, 0, 1)), 10.0, 5.0);
        const TopoDS_Wire hole =
            BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(ellipse).Edge())
                .Wire();
        BRepBuilderAPI_MakeFace faceMaker(plane, outer);
        faceMaker.Add(hole);
        // Return the planar face alone so STEP round-trip keeps the exact
        // Geom_Ellipse edge and the body has no extrusion wall to defer.
        return faceMaker.Face();
    }
    if (name == "plate_slot") {
        // Thin plate with a rectangular through-slot (all-plane cutout).
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(50.0, 30.0, 6.0).Shape();
        TopoDS_Shape slot =
            BRepPrimAPI_MakeBox(gp_Pnt(15.0, 10.0, -1.0), 20.0, 10.0, 8.0)
                .Shape();
        return BRepAlgoAPI_Cut(plate, slot).Shape();
    }
    if (name == "slotted") {
        // Barrel case from the flaregun: a tube with a capsule slot milled
        // through the wall. The outer cylinder stays closed in u but
        // carries an interior trim — the mesher must keep cylinder
        // topology and insert around the slot, not fall back to strips.
        gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
        TopoDS_Shape tube = BRepAlgoAPI_Cut(
            BRepPrimAPI_MakeCylinder(axis, 12.0, 60.0).Shape(),
            BRepPrimAPI_MakeCylinder(axis, 9.0, 60.0).Shape());
        gp_Ax2 slotAx(gp_Pnt(0, -20.0, 20.0), gp_Dir(0, 1, 0));
        TopoDS_Shape slot =
            BRepPrimAPI_MakeCylinder(slotAx, 3.0, 40.0).Shape();
        gp_Trsf up; up.SetTranslation(gp_Vec(0, 0, 20.0));
        TopoDS_Shape slot2 = slot.Moved(up);
        TopoDS_Shape cut = BRepAlgoAPI_Cut(tube, slot).Shape();
        return BRepAlgoAPI_Cut(cut, slot2).Shape();
    }
    if (name == "barrel") {
        // The flaregun face-81/87 class: a PARTIAL-wrap wall (the tube
        // loses a quarter to a lengthwise cut) with a capsule slot
        // milled through what remains. Not u-closed, so revolution
        // grids don't apply — the wall is a coons chart with an
        // interior trim wire and must mesh as a grid CUTOUT, not a
        // fallback fan.
        gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
        TopoDS_Shape tube = BRepAlgoAPI_Cut(
            BRepPrimAPI_MakeCylinder(axis, 12.0, 60.0).Shape(),
            BRepPrimAPI_MakeCylinder(axis, 9.0, 60.0).Shape());
        TopoDS_Shape sector =
            BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, -1.0),
                                gp_Pnt(30.0, 30.0, 61.0))
                .Shape();
        TopoDS_Shape cut = BRepAlgoAPI_Cut(tube, sector).Shape();
        // Capsule slot through the -X wall: box + rounded ends.
        TopoDS_Shape sBox =
            BRepPrimAPI_MakeBox(gp_Pnt(-14.0, -3.0, 20.0),
                                gp_Pnt(-8.0, 3.0, 40.0))
                .Shape();
        gp_Ax2 e1(gp_Pnt(-14.0, 0.0, 20.0), gp_Dir(1, 0, 0));
        gp_Ax2 e2(gp_Pnt(-14.0, 0.0, 40.0), gp_Dir(1, 0, 0));
        TopoDS_Shape cap1 = BRepPrimAPI_MakeCylinder(e1, 3.0, 6.0).Shape();
        TopoDS_Shape cap2 = BRepPrimAPI_MakeCylinder(e2, 3.0, 6.0).Shape();
        TopoDS_Shape slot =
            BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(sBox, cap1).Shape(), cap2)
                .Shape();
        return BRepAlgoAPI_Cut(cut, slot).Shape();
    }
    if (name == "drilled") {
        // A round bolt hole through a partial-wrap wall — the most
        // common real cutout (vents, bores, mounting holes). The wall
        // must keep straight full-height columns with the hole carved
        // as a local collar, not fan across the primitive.
        gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
        TopoDS_Shape tube = BRepAlgoAPI_Cut(
            BRepPrimAPI_MakeCylinder(axis, 12.0, 60.0).Shape(),
            BRepPrimAPI_MakeCylinder(axis, 9.0, 60.0).Shape());
        TopoDS_Shape sector =
            BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, -1.0),
                                gp_Pnt(30.0, 30.0, 61.0))
                .Shape();
        TopoDS_Shape cut = BRepAlgoAPI_Cut(tube, sector).Shape();
        gp_Ax2 hAx(gp_Pnt(-14.0, 0.0, 30.0), gp_Dir(1, 0, 0));
        TopoDS_Shape hole = BRepPrimAPI_MakeCylinder(hAx, 4.0, 6.0).Shape();
        return BRepAlgoAPI_Cut(cut, hole).Shape();
    }
    if (name == "barrel2") {
        // The realistic barrel wall: partial wrap, a capsule slot through
        // the wall AND a step interrupting the top rim — the outer wire
        // becomes a 7+ edge chain, which is how real gun parts arrive
        // (adjacent features slice the rims). The wall must still take
        // the coons cutout via chained sides, not fall back to tri fans.
        gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
        TopoDS_Shape tube = BRepAlgoAPI_Cut(
            BRepPrimAPI_MakeCylinder(axis, 12.0, 60.0).Shape(),
            BRepPrimAPI_MakeCylinder(axis, 9.0, 60.0).Shape());
        TopoDS_Shape sector =
            BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, -1.0),
                                gp_Pnt(30.0, 30.0, 61.0))
                .Shape();
        TopoDS_Shape cut = BRepAlgoAPI_Cut(tube, sector).Shape();
        // Step in the top rim on the -Y side.
        TopoDS_Shape step =
            BRepPrimAPI_MakeBox(gp_Pnt(-6.0, -30.0, 50.0),
                                gp_Pnt(6.0, 0.0, 61.0))
                .Shape();
        cut = BRepAlgoAPI_Cut(cut, step).Shape();
        // Capsule slot through the -X wall, below the step.
        TopoDS_Shape sBox =
            BRepPrimAPI_MakeBox(gp_Pnt(-14.0, -3.0, 15.0),
                                gp_Pnt(-8.0, 3.0, 35.0))
                .Shape();
        gp_Ax2 e1(gp_Pnt(-14.0, 0.0, 15.0), gp_Dir(1, 0, 0));
        gp_Ax2 e2(gp_Pnt(-14.0, 0.0, 35.0), gp_Dir(1, 0, 0));
        TopoDS_Shape cap1 = BRepPrimAPI_MakeCylinder(e1, 3.0, 6.0).Shape();
        TopoDS_Shape cap2 = BRepPrimAPI_MakeCylinder(e2, 3.0, 6.0).Shape();
        TopoDS_Shape slot =
            BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(sBox, cap1).Shape(), cap2)
                .Shape();
        return BRepAlgoAPI_Cut(cut, slot).Shape();
    }
    if (name == "notched") {
        // The flaregun face-81 class: a tube whose wall carries a channel
        // cut clean THROUGH the top rim (the notch opens to the border).
        // The outer cylinder stays u-closed below the notch; its top rim
        // is arcs + two wall drops + a notch floor. Revolution grid must
        // stay the basis with the notch handled as a rim-open cutout -
        // not a fallback-tri fan.
        gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
        TopoDS_Shape tube = BRepAlgoAPI_Cut(
            BRepPrimAPI_MakeCylinder(axis, 12.0, 40.0).Shape(),
            BRepPrimAPI_MakeCylinder(axis, 9.0, 40.0).Shape());
        // Channel: a box from mid-height out through the rim and wall.
        TopoDS_Shape channel =
            BRepPrimAPI_MakeBox(gp_Pnt(-4.0, 0.0, 22.0),
                                gp_Pnt(4.0, 20.0, 44.0))
                .Shape();
        return BRepAlgoAPI_Cut(tube, channel).Shape();
    }
    if (name == "bossfillet") {
        // The HDD class: a round boss whose top rim is blended — the
        // fillet ring is a full 360-degree torus band and must mesh as
        // a revolution ring, not a coons patch with a seam twist.
        TopoDS_Shape base = BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
        gp_Ax2 axis(gp_Pnt(20.0, 20.0, 10.0), gp_Dir(0, 0, 1));
        TopoDS_Shape boss = BRepPrimAPI_MakeCylinder(axis, 8.0, 15.0).Shape();
        TopoDS_Shape fused = BRepAlgoAPI_Fuse(base, boss).Shape();
        BRepFilletAPI_MakeFillet fillet(fused);
        for (TopExp_Explorer ex(fused, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            double f, l;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
            if (c.IsNull()) continue;
            gp_Pnt m = c->Value((f + l) / 2);
            if (std::abs(m.Z() - 25.0) < 1e-6) {  // boss top rim
                fillet.Add(2.0, e);
            }
        }
        return fillet.Shape();
    }
    if (name == "boss") {
        TopoDS_Shape base = BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
        gp_Ax2 axis(gp_Pnt(20.0, 20.0, 10.0), gp_Dir(0, 0, 1));
        TopoDS_Shape boss = BRepPrimAPI_MakeCylinder(axis, 8.0, 15.0).Shape();
        return BRepAlgoAPI_Fuse(base, boss).Shape();
    }
    if (name == "mapped_patch" || name == "freeform_patch") {
        // Open bspline sheets. mapped_patch is four-sided; freeform_patch is
        // five-sided (one UV-border edge split at mid-U) so FREE is beyond
        // four-sided while samples stay on the UV-grid border.
        const double amplitude = (name == "freeform_patch") ? 1.25 : 0.5;
        NCollection_Array2<gp_Pnt> net(1, 4, 1, 4);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                const double u = double(i) / 3.0;
                const double v = double(j) / 3.0;
                net.SetValue(i + 1, j + 1,
                             gp_Pnt(30.0 * u, 20.0 * v,
                                    amplitude * std::sin(M_PI * u) *
                                        std::sin(M_PI * v)));
            }
        }
        Handle(Geom_BSplineSurface) surf =
            GeomAPI_PointsToBSplineSurface(net).Surface();
        if (name == "mapped_patch") {
            return BRepBuilderAPI_MakeFace(surf, 1e-6).Face();
        }
        double uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
        surf->Bounds(uMin, uMax, vMin, vMax);
        const double uMid = 0.5 * (uMin + uMax);
        const gp_Pnt2d pts[5] = {
            gp_Pnt2d(uMin, vMin), gp_Pnt2d(uMid, vMin), gp_Pnt2d(uMax, vMin),
            gp_Pnt2d(uMax, vMax), gp_Pnt2d(uMin, vMax),
        };
        BRepBuilderAPI_MakeWire wire;
        for (int k = 0; k < 5; ++k) {
            const gp_Pnt2d& a = pts[k];
            const gp_Pnt2d& b = pts[(k + 1) % 5];
            const gp_Vec2d vec(a, b);
            const double len = vec.Magnitude();
            if (len <= 1e-12) {
                throw std::runtime_error("freeform_patch degenerate UV edge");
            }
            Handle(Geom2d_Line) line2d = new Geom2d_Line(a, gp_Dir2d(vec));
            Handle(Geom2d_TrimmedCurve) trimmed =
                new Geom2d_TrimmedCurve(line2d, 0.0, len);
            BRepBuilderAPI_MakeEdge makeEdge(trimmed, surf);
            if (!makeEdge.IsDone()) {
                throw std::runtime_error("freeform_patch UV edge failed");
            }
            TopoDS_Edge edge = makeEdge.Edge();
            BRepLib::BuildCurves3d(edge);
            wire.Add(edge);
        }
        if (!wire.IsDone()) {
            throw std::runtime_error("freeform_patch UV wire failed");
        }
        BRepBuilderAPI_MakeFace face(surf, wire.Wire());
        if (!face.IsDone()) {
            throw std::runtime_error("freeform_patch face from UV wire failed");
        }
        ShapeFix_Shape fix(face.Face());
        fix.SetPrecision(1e-7);
        fix.Perform();
        return fix.Shape();
    }
    if (name == "extrusion_quad") {
        // Wave D: four-sided linear-extrusion face via MakePrism of a
        // bspline edge (raw Geom_SurfaceOfLinearExtrusion faces often lose
        // complete edge topology after STEP round-trip).
        NCollection_Array1<gp_Pnt> pts(1, 4);
        pts.SetValue(1, gp_Pnt(0.0, 0.0, 0.0));
        pts.SetValue(2, gp_Pnt(10.0, 3.0, 0.0));
        pts.SetValue(3, gp_Pnt(20.0, -2.0, 0.0));
        pts.SetValue(4, gp_Pnt(30.0, 1.0, 0.0));
        Handle(Geom_BSplineCurve) basis =
            GeomAPI_PointsToBSpline(pts).Curve();
        TopoDS_Edge profile = BRepBuilderAPI_MakeEdge(basis).Edge();
        return BRepPrimAPI_MakePrism(profile, gp_Vec(0.0, 0.0, 12.0)).Shape();
    }
    if (name == "revolution_ngon") {
        // Wave D: five-sided UV trim on SurfaceOfRevolution (bspline
        // meridian — line meridians become cylinder/cone).
        NCollection_Array1<gp_Pnt> pts(1, 4);
        pts.SetValue(1, gp_Pnt(10.0, 0.0, 0.0));
        pts.SetValue(2, gp_Pnt(12.0, 0.0, 6.0));
        pts.SetValue(3, gp_Pnt(11.0, 0.0, 12.0));
        pts.SetValue(4, gp_Pnt(9.0, 0.0, 18.0));
        Handle(Geom_BSplineCurve) meridian =
            GeomAPI_PointsToBSpline(pts).Curve();
        Handle(Geom_SurfaceOfRevolution) surf = new Geom_SurfaceOfRevolution(
            meridian, gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)));
        double uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
        surf->Bounds(uMin, uMax, vMin, vMax);
        const double u0 = uMin + 0.15 * (uMax - uMin);
        const double u1 = uMin + 0.85 * (uMax - uMin);
        const double v0 = vMin + 0.10 * (vMax - vMin);
        const double v1 = vMin + 0.90 * (vMax - vMin);
        const double uMid = 0.5 * (u0 + u1);
        const gp_Pnt2d ptsUv[5] = {
            gp_Pnt2d(u0, v0), gp_Pnt2d(uMid, v0), gp_Pnt2d(u1, v0),
            gp_Pnt2d(u1, v1), gp_Pnt2d(u0, v1),
        };
        BRepBuilderAPI_MakeWire wire;
        for (int k = 0; k < 5; ++k) {
            const gp_Pnt2d& a = ptsUv[k];
            const gp_Pnt2d& b = ptsUv[(k + 1) % 5];
            const gp_Vec2d vec(a, b);
            const double len = vec.Magnitude();
            if (len <= 1e-12) {
                throw std::runtime_error("revolution_ngon degenerate UV edge");
            }
            Handle(Geom2d_Line) line2d = new Geom2d_Line(a, gp_Dir2d(vec));
            Handle(Geom2d_TrimmedCurve) trimmed =
                new Geom2d_TrimmedCurve(line2d, 0.0, len);
            BRepBuilderAPI_MakeEdge makeEdge(trimmed, surf);
            if (!makeEdge.IsDone()) {
                throw std::runtime_error("revolution_ngon UV edge failed");
            }
            TopoDS_Edge edge = makeEdge.Edge();
            BRepLib::BuildCurves3d(edge);
            wire.Add(edge);
        }
        if (!wire.IsDone()) {
            throw std::runtime_error("revolution_ngon UV wire failed");
        }
        BRepBuilderAPI_MakeFace face(surf, wire.Wire());
        if (!face.IsDone()) {
            throw std::runtime_error("revolution_ngon face from UV wire failed");
        }
        ShapeFix_Shape fix(face.Face());
        fix.SetPrecision(1e-7);
        fix.Perform();
        return fix.Shape();
    }
    if (name == "ribbon" || name == "ribbonnotch") {
        // The flaregun grip/trigger-guard class: a long, thin, BENT strip
        // whose surface is freeform (not a plane or a cylinder wrap) and
        // whose flattened outline is NON-CONVEX. A transfinite/coons blend
        // folds across such an outline; the strip must instead be swept
        // rail-to-rail. Built as a ruled bspline surface between two bent
        // rails, then thickened into a solid so the strip is a real
        // outer-wire face. "ribbonnotch" cuts a notch clean through one
        // end (the deeply-notched end-cap class) to exercise the local
        // notch web on the sweep.
        const int N = 25;          // stations along the strip
        const double Lx = 80.0;    // length extent
        const double Amp = 8.0;    // in-plane S-bend (freeform, not a wrap)
        const double Hump = 7.0;   // out-of-plane lift (freeform surface)
        const double W = 18.0;     // strip width
        const double thick = 4.0;  // slab thickness
        // A 2 x N net of poles: column 1 = rail A, column 2 = rail B. The
        // width offset is a pure +/-Y shift so the two end caps stay clean
        // axial lines (easy to notch); the S in Y and the hump in Z make
        // the surface a genuine freeform bspline the rails must be swept
        // across, not a plane or a cylinder.
        NCollection_Array2<gp_Pnt> net(1, N, 1, 2);
        for (int i = 0; i < N; ++i) {
            const double t = double(i) / (N - 1);
            const double cx = Lx * t;
            // Tapered S: the sin(pi t) window pins cy and its slope to ~0
            // at both ends, so the end caps stay axial (cleanly notchable)
            // while the middle sweeps + then - (the reflex).
            const double cy =
                Amp * std::sin(2.0 * M_PI * t) * std::sin(M_PI * t);
            const double cz = Hump * std::sin(M_PI * t);
            net.SetValue(i + 1, 1, gp_Pnt(cx, cy + 0.5 * W, cz));
            net.SetValue(i + 1, 2, gp_Pnt(cx, cy - 0.5 * W, cz));
        }
        Handle(Geom_BSplineSurface) surf =
            GeomAPI_PointsToBSplineSurface(net).Surface();
        TopoDS_Face strip = BRepBuilderAPI_MakeFace(surf, 1e-6).Face();
        TopoDS_Shape slab =
            BRepPrimAPI_MakePrism(strip, gp_Vec(0, 0, thick)).Shape();
        if (name == "ribbonnotch") {
            // A rectangular slot bitten into the middle of the far (x=Lx)
            // end cap: it opens that cap into a rail-drop-notch-drop-rail
            // chain, so the outline is no longer four-sided and coons rejects
            // it — the notched end-cap class the sweep must web locally
            // rather than fan.
            TopoDS_Shape notch =
                BRepPrimAPI_MakeBox(gp_Pnt(Lx - 10.0, -4.0, -5.0),
                                    gp_Pnt(Lx + 5.0, 4.0, Hump + thick + 5.0))
                    .Shape();
            slab = BRepAlgoAPI_Cut(slab, notch).Shape();
        }
        return slab;
    }
    if (name == "hairline") {
        // The foam-674/675 / weldment-chamfer class ISOLATED: a boss whose
        // base rim carries a HAIRLINE fillet — a 0.15-wide torus band
        // whose two rails run parallel a strip-width apart while the
        // along-pitch is 20-50x larger. Every proximity heuristic that
        // scales with the along-pitch swallows the twin rail; only HOME
        // attribution (nearest own curve) separates them.
        TopoDS_Shape base = BRepPrimAPI_MakeBox(40.0, 40.0, 8.0).Shape();
        gp_Ax2 axis(gp_Pnt(20.0, 20.0, 8.0), gp_Dir(0, 0, 1));
        TopoDS_Shape boss = BRepPrimAPI_MakeCylinder(axis, 10.0, 12.0).Shape();
        TopoDS_Shape fused = BRepAlgoAPI_Fuse(base, boss).Shape();
        BRepFilletAPI_MakeFillet fillet(fused);
        for (TopExp_Explorer ex(fused, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            double f, l;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
            if (c.IsNull()) continue;
            gp_Pnt m = c->Value((f + l) / 2);
            // The boss/base junction circle at z=8, r=10.
            if (std::abs(m.Z() - 8.0) < 1e-6 &&
                std::abs(std::hypot(m.X() - 20.0, m.Y() - 20.0) - 10.0) <
                    1e-6) {
                fillet.Add(0.15, e);
            }
        }
        return fillet.Shape();
    }
    if (name == "canrev") {
        // The foam can-body class ISOLATED: a vase revolved from a bspline
        // profile, then hollowed with MakeThickSolid — the walls become
        // OFFSET_SURFACE geometry that is a perfect surface of revolution
        // but is not TYPED as one, so it takes coons patchwork unless the
        // planner detects revolution geometry from the shape itself
        // (MVP demand #2).
        NCollection_Array1<gp_Pnt> pts(1, 6);
        pts.SetValue(1, gp_Pnt(14.0, 0.0, 0.0));
        pts.SetValue(2, gp_Pnt(15.5, 0.0, 8.0));
        pts.SetValue(3, gp_Pnt(16.0, 0.0, 20.0));
        pts.SetValue(4, gp_Pnt(15.0, 0.0, 32.0));
        pts.SetValue(5, gp_Pnt(12.0, 0.0, 42.0));
        pts.SetValue(6, gp_Pnt(9.0, 0.0, 48.0));
        Handle(Geom_BSplineCurve) prof =
            GeomAPI_PointsToBSpline(pts).Curve();
        BRepBuilderAPI_MakeWire wire;
        wire.Add(BRepBuilderAPI_MakeEdge(prof).Edge());
        wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(9.0, 0.0, 48.0),
                                         gp_Pnt(0.0, 0.0, 48.0))
                     .Edge());
        wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0.0, 0.0, 48.0),
                                         gp_Pnt(0.0, 0.0, 0.0))
                     .Edge());
        wire.Add(BRepBuilderAPI_MakeEdge(gp_Pnt(0.0, 0.0, 0.0),
                                         gp_Pnt(14.0, 0.0, 0.0))
                     .Edge());
        TopoDS_Face profFace = BRepBuilderAPI_MakeFace(wire.Wire()).Face();
        TopoDS_Shape vase =
            BRepPrimAPI_MakeRevol(profFace,
                                  gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)))
                .Shape();
        // Hollow it: remove the top cap so the shell opens like a can —
        // the remaining walls are offset surfaces of the revolve.
        NCollection_List<TopoDS_Shape> toRemove;
        for (TopExp_Explorer ex(vase, TopAbs_FACE); ex.More(); ex.Next()) {
            const TopoDS_Face f = TopoDS::Face(ex.Current());
            // The flat annular top at z=48.
            bool top = true;
            for (TopExp_Explorer vx(f, TopAbs_VERTEX); vx.More();
                 vx.Next()) {
                gp_Pnt p = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
                if (std::abs(p.Z() - 48.0) > 1e-6) {
                    top = false;
                    break;
                }
            }
            if (top) {
                toRemove.Append(f);
                break;
            }
        }
        BRepOffsetAPI_MakeThickSolid hollow;
        hollow.MakeThickSolidByJoin(vase, toRemove, -1.5, 1e-6);
        return hollow.Shape();
    }
    if (name == "slitdrill") {
        // The nasty_cheese face-6 class ISOLATED: two equal bores crossing
        // inside a block (their walls meet in ellipse seams), plus a
        // rectangular pocket grazing one bore so a LENGTHWISE line edge
        // lands in the bore wall's rim chain — the slit that must kick a
        // full-wrap pure lattice back to the strip/floor machinery.
        TopoDS_Shape block = BRepPrimAPI_MakeBox(40.0, 40.0, 30.0).Shape();
        gp_Ax2 axZ(gp_Pnt(20.0, 20.0, -1.0), gp_Dir(0, 0, 1));
        TopoDS_Shape boreZ =
            BRepPrimAPI_MakeCylinder(axZ, 6.0, 32.0).Shape();
        gp_Ax2 axX(gp_Pnt(-1.0, 20.0, 15.0), gp_Dir(1, 0, 0));
        TopoDS_Shape boreX =
            BRepPrimAPI_MakeCylinder(axX, 6.0, 42.0).Shape();
        TopoDS_Shape cut = BRepAlgoAPI_Cut(block, boreZ).Shape();
        cut = BRepAlgoAPI_Cut(cut, boreX).Shape();
        // Pocket whose wall passes exactly through the Z bore's surface
        // (x = 26 = 20 + r): the intersection leaves a line edge running
        // the length of the bore wall.
        TopoDS_Shape pocket =
            BRepPrimAPI_MakeBox(gp_Pnt(26.0, 10.0, -1.0),
                                gp_Pnt(41.0, 30.0, 31.0))
                .Shape();
        return BRepAlgoAPI_Cut(cut, pocket).Shape();
    }
    if (name == "microedge") {
        // The nasty edge-699 class ISOLATED: a 0.12-long chamfer edge — a
        // micro-edge between full-size faces. One side samples it as a
        // handful of segments, the other side's border jumps straight
        // across it; the seam machinery must not lose the corner.
        TopoDS_Shape box = BRepPrimAPI_MakeBox(30.0, 30.0, 12.0).Shape();
        BRepFilletAPI_MakeChamfer cham(box);
        for (TopExp_Explorer ex(box, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            double f, l;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
            if (c.IsNull()) continue;
            gp_Pnt m = c->Value((f + l) / 2);
            // One vertical corner edge only.
            if (std::abs(m.X()) < 1e-6 && std::abs(m.Y()) < 1e-6) {
                cham.Add(0.12, e);
            }
        }
        return cham.Shape();
    }
    if (name == "filletslot") {
        // The artist's coons-loop-spam report ISOLATED (MVP demand #1b):
        // a slot through a plate whose four interior vertical edges are
        // blended r=2 — the slot ends become quarter-round constant-
        // radius fillet strips that weld into the flat slot walls. The
        // FilletBand mesher must run rungs at the WALL's count, not the
        // adaptive/pitch-floor spam a coons grid produces.
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(50.0, 30.0, 10.0).Shape();
        TopoDS_Shape slot =
            BRepPrimAPI_MakeBox(gp_Pnt(10.0, 11.0, -1.0),
                                gp_Pnt(40.0, 19.0, 11.0))
                .Shape();
        TopoDS_Shape cut = BRepAlgoAPI_Cut(plate, slot).Shape();
        BRepFilletAPI_MakeFillet fillet(cut);
        for (TopExp_Explorer ex(cut, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            double f, l;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
            if (c.IsNull()) continue;
            gp_Pnt a = c->Value(f), b = c->Value(l);
            // The slot's four interior vertical edges.
            const bool vertical = std::abs(a.X() - b.X()) < 1e-9 &&
                                  std::abs(a.Y() - b.Y()) < 1e-9;
            const bool slotCorner =
                (std::abs(a.X() - 10.0) < 1e-6 ||
                 std::abs(a.X() - 40.0) < 1e-6) &&
                (std::abs(a.Y() - 11.0) < 1e-6 ||
                 std::abs(a.Y() - 19.0) < 1e-6);
            if (vertical && slotCorner) fillet.Add(2.0, e);
        }
        return fillet.Shape();
    }
    if (name == "torture") {
        // The demo scene: one solid carrying most of the campaign's issue
        // classes at once, so a build can be judged on a single model.
        //   - stacked two-diameter barrel joined by a blend fillet
        //     (flaregun barrel class: column flow across bands),
        //   - a capsule slot through the barrel wall (insert class) and a
        //     channel cut through its top rim (rim-open notch class),
        //   - two crossing bores in the plate (nasty ellipse-seam class),
        //   - a hairline 0.15 fillet ring at a small boss (foam twin-rail
        //     class),
        //   - a slot with r=2 blended end edges (the artist's fillet-band
        //     class),
        //   - a 0.12 micro-chamfer on one plate corner (nasty micro-edge
        //     class).
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(120.0, 80.0, 12.0).Shape();
        // Stacked barrel at (30, 40): wide band below, narrow band above.
        gp_Ax2 axB(gp_Pnt(30.0, 40.0, 12.0), gp_Dir(0, 0, 1));
        TopoDS_Shape band1 =
            BRepPrimAPI_MakeCylinder(axB, 14.0, 22.0).Shape();
        gp_Ax2 axT(gp_Pnt(30.0, 40.0, 34.0), gp_Dir(0, 0, 1));
        TopoDS_Shape band2 =
            BRepPrimAPI_MakeCylinder(axT, 11.0, 24.0).Shape();
        TopoDS_Shape solid = BRepAlgoAPI_Fuse(plate, band1).Shape();
        solid = BRepAlgoAPI_Fuse(solid, band2).Shape();
        // Hollow the top band into a muzzle: bore from above.
        gp_Ax2 axBore(gp_Pnt(30.0, 40.0, 20.0), gp_Dir(0, 0, 1));
        TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(axBore, 8.0, 40.0).Shape();
        solid = BRepAlgoAPI_Cut(solid, bore).Shape();
        // Capsule slot through the top band's wall.
        {
            TopoDS_Shape sBox =
                BRepPrimAPI_MakeBox(gp_Pnt(14.0, 37.0, 38.0),
                                    gp_Pnt(24.0, 43.0, 48.0))
                    .Shape();
            gp_Ax2 e1(gp_Pnt(14.0, 40.0, 38.0), gp_Dir(1, 0, 0));
            gp_Ax2 e2(gp_Pnt(14.0, 40.0, 48.0), gp_Dir(1, 0, 0));
            TopoDS_Shape c1 =
                BRepPrimAPI_MakeCylinder(e1, 3.0, 10.0).Shape();
            TopoDS_Shape c2 =
                BRepPrimAPI_MakeCylinder(e2, 3.0, 10.0).Shape();
            TopoDS_Shape slot =
                BRepAlgoAPI_Fuse(BRepAlgoAPI_Fuse(sBox, c1).Shape(), c2)
                    .Shape();
            solid = BRepAlgoAPI_Cut(solid, slot).Shape();
        }
        // Channel through the muzzle's top rim (rim-open notch).
        {
            TopoDS_Shape channel =
                BRepPrimAPI_MakeBox(gp_Pnt(27.0, 40.0, 52.0),
                                    gp_Pnt(33.0, 60.0, 60.0))
                    .Shape();
            solid = BRepAlgoAPI_Cut(solid, channel).Shape();
        }
        // Crossing bores in the plate at (80, 30).
        {
            gp_Ax2 azV(gp_Pnt(80.0, 30.0, -1.0), gp_Dir(0, 0, 1));
            TopoDS_Shape bV =
                BRepPrimAPI_MakeCylinder(azV, 5.0, 14.0).Shape();
            gp_Ax2 azH(gp_Pnt(80.0, -1.0, 6.0), gp_Dir(0, 1, 0));
            TopoDS_Shape bH =
                BRepPrimAPI_MakeCylinder(azH, 5.0, 82.0).Shape();
            solid = BRepAlgoAPI_Cut(solid, bV).Shape();
            solid = BRepAlgoAPI_Cut(solid, bH).Shape();
        }
        // Small boss at (105, 62) with a hairline base fillet.
        {
            gp_Ax2 axS(gp_Pnt(105.0, 62.0, 12.0), gp_Dir(0, 0, 1));
            TopoDS_Shape boss =
                BRepPrimAPI_MakeCylinder(axS, 6.0, 10.0).Shape();
            solid = BRepAlgoAPI_Fuse(solid, boss).Shape();
            BRepFilletAPI_MakeFillet fillet(solid);
            for (TopExp_Explorer ex(solid, TopAbs_EDGE); ex.More();
                 ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                double f, l;
                Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
                if (c.IsNull()) continue;
                gp_Pnt m = c->Value((f + l) / 2);
                if (std::abs(m.Z() - 12.0) < 1e-6 &&
                    std::abs(std::hypot(m.X() - 105.0, m.Y() - 62.0) -
                             6.0) < 1e-6) {
                    fillet.Add(0.15, e);
                }
            }
            solid = fillet.Shape();
        }
        // Slot with blended end edges at (60..90, 55..65).
        {
            TopoDS_Shape slot =
                BRepPrimAPI_MakeBox(gp_Pnt(58.0, 55.0, -1.0),
                                    gp_Pnt(88.0, 65.0, 13.0))
                    .Shape();
            solid = BRepAlgoAPI_Cut(solid, slot).Shape();
            BRepFilletAPI_MakeFillet fillet(solid);
            for (TopExp_Explorer ex(solid, TopAbs_EDGE); ex.More();
                 ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                double f, l;
                Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
                if (c.IsNull()) continue;
                gp_Pnt a = c->Value(f), b = c->Value(l);
                const bool vertical = std::abs(a.X() - b.X()) < 1e-9 &&
                                      std::abs(a.Y() - b.Y()) < 1e-9;
                const bool corner =
                    (std::abs(a.X() - 58.0) < 1e-6 ||
                     std::abs(a.X() - 88.0) < 1e-6) &&
                    (std::abs(a.Y() - 55.0) < 1e-6 ||
                     std::abs(a.Y() - 65.0) < 1e-6);
                if (vertical && corner) fillet.Add(2.0, e);
            }
            solid = fillet.Shape();
        }
        // Two ANGLED cylinders fused in and blended with LARGE fillets —
        // the tilted-boss class real models carry (a leaning strut welded
        // into a deck): the junction curve is a warped ellipse and the
        // blend is a wide freeform-ish band, not a neat torus ring.
        {
            // Strut 1: leaning 30 deg toward +x at (95, 20).
            gp_Dir tilt1(std::sin(M_PI / 6.0), 0.0, std::cos(M_PI / 6.0));
            gp_Ax2 axA(gp_Pnt(95.0, 20.0, 8.0), tilt1);
            TopoDS_Shape strutA =
                BRepPrimAPI_MakeCylinder(axA, 6.0, 26.0).Shape();
            solid = BRepAlgoAPI_Fuse(solid, strutA).Shape();
            // Strut 2: leaning 40 deg toward -y at (58, 26), thicker.
            gp_Dir tilt2(0.0, -std::sin(2.0 * M_PI / 9.0),
                         std::cos(2.0 * M_PI / 9.0));
            gp_Ax2 axC(gp_Pnt(58.0, 26.0, 8.0), tilt2);
            TopoDS_Shape strutC =
                BRepPrimAPI_MakeCylinder(axC, 5.0, 24.0).Shape();
            solid = BRepAlgoAPI_Fuse(solid, strutC).Shape();
            // Large blends where the struts meet the plate top: junction
            // edges live near z=12 within reach of each strut's axis.
            BRepFilletAPI_MakeFillet fillet(solid);
            int added = 0;
            for (TopExp_Explorer ex(solid, TopAbs_EDGE); ex.More();
                 ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                double f, l;
                Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
                if (c.IsNull()) continue;
                gp_Pnt m = c->Value((f + l) / 2);
                if (std::abs(m.Z() - 12.0) > 1.0) continue;
                if (std::hypot(m.X() - 95.0, m.Y() - 20.0) < 11.0) {
                    fillet.Add(4.0, e);
                    ++added;
                } else if (std::hypot(m.X() - 58.0, m.Y() - 26.0) < 10.0) {
                    fillet.Add(3.5, e);
                    ++added;
                }
            }
            if (added) solid = fillet.Shape();
        }
        // Small fillets on two plate top edges (r=0.8 — the ordinary
        // break-the-edge rounds most real models carry; distinct from
        // both the hairline 0.15 and the large 3.5-4 blends).
        {
            BRepFilletAPI_MakeFillet fillet(solid);
            int added = 0;
            for (TopExp_Explorer ex(solid, TopAbs_EDGE); ex.More();
                 ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                double f, l;
                Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
                if (c.IsNull()) continue;
                gp_Pnt a = c->Value(f), b = c->Value(l);
                gp_Pnt m = c->Value((f + l) / 2);
                const bool straight =
                    std::abs((b.XYZ() - a.XYZ()).Modulus() -
                             a.Distance(b)) < 1e-9;
                if (!straight || std::abs(m.Z() - 12.0) > 1e-6) continue;
                // Front (y=0) and right (x=120) top rims of the plate.
                if (std::abs(m.Y()) < 1e-6 ||
                    std::abs(m.X() - 120.0) < 1e-6) {
                    fillet.Add(0.8, e);
                    ++added;
                }
            }
            if (added) solid = fillet.Shape();
        }
        // Micro-chamfer on the plate corner at the origin.
        {
            BRepFilletAPI_MakeChamfer cham(solid);
            for (TopExp_Explorer ex(solid, TopAbs_EDGE); ex.More();
                 ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                double f, l;
                Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
                if (c.IsNull()) continue;
                gp_Pnt m = c->Value((f + l) / 2);
                if (std::abs(m.X()) < 1e-6 && std::abs(m.Y()) < 1e-6) {
                    cham.Add(0.12, e);
                }
            }
            solid = cham.Shape();
        }
        return solid;
    }
    if (name == "curve_hyperbola") {
        // Planar face closed by a hyperbola branch + chord. Product mesh
        // must named-refuse the hyperbola edge (Wave F).
        const gp_Ax2 ax(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
        const Handle(Geom_Hyperbola) hypr = new Geom_Hyperbola(ax, 5.0, 3.0);
        const TopoDS_Edge branch =
            BRepBuilderAPI_MakeEdge(hypr, -1.0, 1.0).Edge();
        const TopoDS_Edge chord =
            BRepBuilderAPI_MakeEdge(hypr->Value(-1.0), hypr->Value(1.0)).Edge();
        const TopoDS_Wire wire =
            BRepBuilderAPI_MakeWire(branch, chord).Wire();
        return BRepBuilderAPI_MakeFace(wire, true).Face();
    }
    if (name == "curve_parabola") {
        const gp_Ax2 ax(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
        const Handle(Geom_Parabola) parab = new Geom_Parabola(ax, 3.5);
        const TopoDS_Edge branch =
            BRepBuilderAPI_MakeEdge(parab, -2.0, 2.0).Edge();
        const TopoDS_Edge chord =
            BRepBuilderAPI_MakeEdge(parab->Value(-2.0), parab->Value(2.0))
                .Edge();
        const TopoDS_Wire wire =
            BRepBuilderAPI_MakeWire(branch, chord).Wire();
        return BRepBuilderAPI_MakeFace(wire, true).Face();
    }
    if (name == "curve_offset") {
        // Offset of a circle retains GeomAbs_OffsetCurve in native B-rep;
        // STEP often flattens it — Wave F commits the .brep form.
        const gp_Ax2 ax(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0));
        const Handle(Geom_Curve) circle = new Geom_Circle(ax, 10.0);
        const Handle(Geom_Curve) offset =
            new Geom_OffsetCurve(circle, 2.0, gp_Dir(0.0, 0.0, 1.0));
        const TopoDS_Edge edge =
            BRepBuilderAPI_MakeEdge(offset, 0.0, 2.0 * M_PI).Edge();
        const TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge).Wire();
        return BRepBuilderAPI_MakeFace(wire, true).Face();
    }
    throw std::runtime_error(
        "unknown fixture: " + name +
        " (expected cylinder|partial_cylinder|box|cone|truncated_cone|sphere|"
        "sphere_cap|torus|fillet|hole|ellipse_hole|plate_slot|demo|boss|"
        "hairline|canrev|slitdrill|microedge|filletslot|torture|ribbon|ribbonnotch|"
        "mapped_patch|freeform_patch|curve_hyperbola|curve_parabola|curve_offset)");
}

}  // namespace weft
