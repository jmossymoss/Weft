#include "weft/fixture.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
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
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Precision.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Elips.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

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
        // planner detects revolution geometry from the shape itself.
        TColgp_Array1OfPnt pts(1, 6);
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
        TopTools_ListOfShape toRemove;
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
        // Isolates the reported coons-loop-spam failure:
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
    if (name == "extrusion") {
        // Explicit linear extrusion (prism) of a polygonal profile.
        BRepBuilderAPI_MakePolygon poly;
        poly.Add(gp_Pnt(0, 0, 0));
        poly.Add(gp_Pnt(20, 0, 0));
        poly.Add(gp_Pnt(15, 12, 0));
        poly.Add(gp_Pnt(5, 12, 0));
        poly.Close();
        const TopoDS_Face profile =
            BRepBuilderAPI_MakeFace(poly.Wire(), true).Face();
        return BRepPrimAPI_MakePrism(profile, gp_Vec(0, 0, 18)).Shape();
    }
    if (name == "bspline_slab") {
        TColgp_Array2OfPnt poles(1, 4, 1, 4);
        for (int u = 1; u <= 4; ++u) {
            for (int v = 1; v <= 4; ++v) {
                const double z = 2.0 * std::sin(0.7 * u) * std::cos(0.5 * v);
                poles.SetValue(u, v, gp_Pnt(8.0 * (u - 1), 8.0 * (v - 1), z));
            }
        }
        Handle(Geom_BSplineSurface) surf =
            GeomAPI_PointsToBSplineSurface(poles).Surface();
        const TopoDS_Face face =
            BRepBuilderAPI_MakeFace(surf, Precision::Confusion()).Face();
        return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, 3)).Shape();
    }
    if (name == "bezier_slab" || name == "bezier_face") {
        TColgp_Array2OfPnt poles(1, 3, 1, 3);
        poles.SetValue(1, 1, gp_Pnt(0, 0, 0));
        poles.SetValue(1, 2, gp_Pnt(0, 10, 2));
        poles.SetValue(1, 3, gp_Pnt(0, 20, 0));
        poles.SetValue(2, 1, gp_Pnt(10, 0, 1));
        poles.SetValue(2, 2, gp_Pnt(10, 10, 4));
        poles.SetValue(2, 3, gp_Pnt(10, 20, 1));
        poles.SetValue(3, 1, gp_Pnt(20, 0, 0));
        poles.SetValue(3, 2, gp_Pnt(20, 10, 2));
        poles.SetValue(3, 3, gp_Pnt(20, 20, 0));
        Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(poles);
        const TopoDS_Face face =
            BRepBuilderAPI_MakeFace(surf, Precision::Confusion()).Face();
        // Face-only keeps Geom_BezierSurface; prisms often promote to BSpline.
        if (name == "bezier_face") return face;
        return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, 2.5)).Shape();
    }
    if (name == "offset_slab") {
        // Prism of a Geom_OffsetSurface so analysis can classify Offset.
        TColgp_Array2OfPnt poles(1, 3, 1, 3);
        for (int u = 1; u <= 3; ++u) {
            for (int v = 1; v <= 3; ++v) {
                poles.SetValue(u, v,
                               gp_Pnt(10.0 * (u - 1), 10.0 * (v - 1),
                                      (u == 2 && v == 2) ? 3.0 : 0.0));
            }
        }
        Handle(Geom_BSplineSurface) base =
            GeomAPI_PointsToBSplineSurface(poles).Surface();
        Handle(Geom_OffsetSurface) offset = new Geom_OffsetSurface(base, 1.2);
        const TopoDS_Face face =
            BRepBuilderAPI_MakeFace(offset, Precision::Confusion()).Face();
        return BRepPrimAPI_MakePrism(face, gp_Vec(0, 0, 2)).Shape();
    }
    if (name == "ellipse_plate") {
        // Planar plate with an elliptical through-hole (ellipse trim curve).
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(40.0, 30.0, 6.0).Shape();
        const gp_Ax2 ax(gp_Pnt(20.0, 15.0, -1.0), gp_Dir(0, 0, 1));
        Handle(Geom_Ellipse) ell =
            new Geom_Ellipse(gp_Elips(ax, 10.0, 5.0));
        const double twoPi = 2.0 * std::acos(-1.0);
        Handle(Geom_TrimmedCurve) trimmed =
            new Geom_TrimmedCurve(ell, 0.0, twoPi);
        const TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(trimmed).Edge();
        const TopoDS_Wire wire = BRepBuilderAPI_MakeWire(edge).Wire();
        const TopoDS_Face profile =
            BRepBuilderAPI_MakeFace(wire, true).Face();
        const TopoDS_Shape cutter =
            BRepPrimAPI_MakePrism(profile, gp_Vec(0, 0, 8)).Shape();
        return BRepAlgoAPI_Cut(plate, cutter).Shape();
    }
    if (name == "plate_holes") {
        // Interaction: planar plate with two circular holes.
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(60.0, 30.0, 5.0).Shape();
        for (double x : {18.0, 42.0}) {
            const TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(x, 15.0, -1.0), gp_Dir(0, 0, 1)), 5.0, 7.0)
                                         .Shape();
            plate = BRepAlgoAPI_Cut(plate, bore).Shape();
        }
        return plate;
    }
    if (name == "compound2") {
        // Two disconnected closed solids in one compound (assembly-like).
        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);
        builder.Add(compound, BRepPrimAPI_MakeCylinder(8.0, 20.0).Shape());
        gp_Trsf move;
        move.SetTranslation(gp_Vec(25.0, 0, 0));
        builder.Add(compound,
                    BRepBuilderAPI_Transform(
                        BRepPrimAPI_MakeBox(12.0, 10.0, 8.0).Shape(), move)
                        .Shape());
        return compound;
    }
    if (name == "open_shell") {
        // Intentionally open: a single planar face (not a closed solid).
        return BRepBuilderAPI_MakeFace(
                   BRepBuilderAPI_MakePolygon(
                       gp_Pnt(0, 0, 0), gp_Pnt(20, 0, 0), gp_Pnt(20, 15, 0),
                       gp_Pnt(0, 15, 0), true)
                       .Wire(),
                   true)
            .Face();
    }
    if (name == "dirty_gap") {
        // Two face shells sewn with a deliberate gap — invalid/open source.
        const TopoDS_Face a =
            BRepBuilderAPI_MakeFace(
                BRepBuilderAPI_MakePolygon(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0),
                                           gp_Pnt(10, 10, 0), gp_Pnt(0, 10, 0),
                                           true)
                    .Wire(),
                true)
                .Face();
        const TopoDS_Face b =
            BRepBuilderAPI_MakeFace(
                BRepBuilderAPI_MakePolygon(gp_Pnt(10.2, 0, 0),
                                           gp_Pnt(20, 0, 0), gp_Pnt(20, 10, 0),
                                           gp_Pnt(10.2, 10, 0), true)
                    .Wire(),
                true)
                .Face();
        BRepBuilderAPI_Sewing sew(0.05);
        sew.Add(a);
        sew.Add(b);
        sew.Perform();
        return sew.SewedShape();
    }
    throw std::runtime_error(
        "unknown fixture: " + name +
        " (expected cylinder|box|cone|sphere|torus|fillet|hole|demo|boss|"
        "hairline|canrev|slitdrill|microedge|filletslot|torture|extrusion|"
        "bspline_slab|bezier_slab|bezier_face|offset_slab|ellipse_plate|"
        "plate_holes|compound2|open_shell|dirty_gap)");
}

}  // namespace weft
