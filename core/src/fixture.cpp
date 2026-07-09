#include "weft/fixture.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <BRep_Tool.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>

#include <cmath>
#include <stdexcept>

namespace weft {

TopoDS_Shape makeFixture(const std::string& name) {
    if (name == "cylinder") {
        return BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape();
    }
    if (name == "box") {
        return BRepPrimAPI_MakeBox(20.0, 30.0, 15.0).Shape();
    }
    if (name == "cone") {
        return BRepPrimAPI_MakeCone(10.0, 0.0, 20.0).Shape();
    }
    if (name == "sphere") {
        return BRepPrimAPI_MakeSphere(10.0).Shape();
    }
    if (name == "torus") {
        return BRepPrimAPI_MakeTorus(10.0, 3.0).Shape();
    }
    if (name == "fillet") {
        // Box with one long edge blended: produces a quarter-cylinder strip
        // with two tangent-smooth joins (the canonical support-loop case).
        TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 30.0, 15.0).Shape();
        BRepFilletAPI_MakeFillet fillet(box);
        for (TopExp_Explorer ex(box, TopAbs_EDGE); ex.More(); ex.Next()) {
            fillet.Add(4.0, TopoDS::Edge(ex.Current()));
            break;  // just the first edge
        }
        return fillet.Shape();
    }
    if (name == "demo") {
        TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(10.0, 30.0).Shape();
        gp_Trsf move;
        move.SetTranslation(gp_Vec(30.0, 0.0, 0.0));
        TopoDS_Shape box =
            BRepPrimAPI_MakeBox(20.0, 30.0, 15.0).Shape().Moved(move);

        TopoDS_Compound comp;
        BRep_Builder builder;
        builder.MakeCompound(comp);
        builder.Add(comp, cyl);
        builder.Add(comp, box);
        return comp;
    }
    if (name == "hole") {
        // Plate with a through-bore: two ring-junction faces + a bore wall.
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
        gp_Ax2 axis(gp_Pnt(20.0, 20.0, -1.0), gp_Dir(0, 0, 1));
        TopoDS_Shape drill = BRepPrimAPI_MakeCylinder(axis, 8.0, 12.0).Shape();
        return BRepAlgoAPI_Cut(plate, drill).Shape();
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
        TColgp_Array2OfPnt net(1, N, 1, 2);
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
    throw std::runtime_error(
        "unknown fixture: " + name +
        " (expected cylinder|box|cone|sphere|torus|fillet|hole|demo|boss)");
}

}  // namespace weft
