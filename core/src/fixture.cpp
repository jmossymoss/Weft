#include "weft/fixture.hpp"

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>

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
    if (name == "boss") {
        TopoDS_Shape base = BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
        gp_Ax2 axis(gp_Pnt(20.0, 20.0, 10.0), gp_Dir(0, 0, 1));
        TopoDS_Shape boss = BRepPrimAPI_MakeCylinder(axis, 8.0, 15.0).Shape();
        return BRepAlgoAPI_Fuse(base, boss).Shape();
    }
    throw std::runtime_error(
        "unknown fixture: " + name +
        " (expected cylinder|box|cone|sphere|torus|fillet|hole|demo|boss)");
}

}  // namespace weft
