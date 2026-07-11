// probe92: surface adaptor closure/type census (why does an offset-of-
// revolution wall fail isClosedRevolution?).
#include "weft/analysis.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_SurfaceOfRevolution.hxx>
#include <TopoDS.hxx>
#include <cstdio>
static const char* surfName(GeomAbs_SurfaceType t) {
    switch (t) {
        case GeomAbs_Plane: return "plane";
        case GeomAbs_Cylinder: return "cylinder";
        case GeomAbs_Cone: return "cone";
        case GeomAbs_Sphere: return "sphere";
        case GeomAbs_Torus: return "torus";
        case GeomAbs_BSplineSurface: return "bspline";
        case GeomAbs_SurfaceOfRevolution: return "revolution";
        case GeomAbs_SurfaceOfExtrusion: return "extrusion";
        case GeomAbs_OffsetSurface: return "offset";
        default: return "other";
    }
}
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    for (int fid = 1; fid <= m.faceCount(); ++fid) {
        const TopoDS_Face f = TopoDS::Face(m.faces(fid));
        BRepAdaptor_Surface sa(f);
        Handle(Geom_Surface) gs = BRep_Tool::Surface(f);
        const char* basis = "-";
        if (!gs.IsNull()) {
            Handle(Geom_OffsetSurface) os =
                Handle(Geom_OffsetSurface)::DownCast(gs);
            if (!os.IsNull()) {
                Handle(Geom_SurfaceOfRevolution) rev =
                    Handle(Geom_SurfaceOfRevolution)::DownCast(
                        os->BasisSurface());
                basis = rev.IsNull() ? "offset-of-other" : "offset-of-rev";
            }
        }
        std::printf(
            "face %d: %s uClosed=%d vClosed=%d u=[%.3f,%.3f] v=[%.3f,%.3f]"
            " basis=%s\n",
            fid, surfName(sa.GetType()), sa.IsUClosed() ? 1 : 0,
            sa.IsVClosed() ? 1 : 0, sa.FirstUParameter(),
            sa.LastUParameter(), sa.FirstVParameter(), sa.LastVParameter(),
            basis);
    }
    return 0;
}
