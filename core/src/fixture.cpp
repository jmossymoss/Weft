#include "weft/fixture.hpp"

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
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
    if (name == "boss") {
        TopoDS_Shape base = BRepPrimAPI_MakeBox(40.0, 40.0, 10.0).Shape();
        gp_Ax2 axis(gp_Pnt(20.0, 20.0, 10.0), gp_Dir(0, 0, 1));
        TopoDS_Shape boss = BRepPrimAPI_MakeCylinder(axis, 8.0, 15.0).Shape();
        return BRepAlgoAPI_Fuse(base, boss).Shape();
    }
    throw std::runtime_error("unknown fixture: " + name +
                             " (expected cylinder|box|demo|boss)");
}

}  // namespace weft
