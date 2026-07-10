// probe87: context for the stitch residuals — face surface types, edge
// curve types, OCCT edge tolerances, and curve-to-curve proximity for the
// corner-contamination pairs (337 vs 613 etc.).
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>

#include <cstdio>
#include <cstdlib>

static const char* surfName(GeomAbs_SurfaceType t) {
    switch (t) {
        case GeomAbs_Plane: return "plane";
        case GeomAbs_Cylinder: return "cylinder";
        case GeomAbs_Cone: return "cone";
        case GeomAbs_Sphere: return "sphere";
        case GeomAbs_Torus: return "torus";
        case GeomAbs_BezierSurface: return "bezier";
        case GeomAbs_BSplineSurface: return "bspline";
        case GeomAbs_SurfaceOfRevolution: return "revolution";
        case GeomAbs_SurfaceOfExtrusion: return "extrusion";
        case GeomAbs_OffsetSurface: return "offset";
        default: return "other";
    }
}
static const char* curveName(GeomAbs_CurveType t) {
    switch (t) {
        case GeomAbs_Line: return "line";
        case GeomAbs_Circle: return "circle";
        case GeomAbs_Ellipse: return "ellipse";
        case GeomAbs_BSplineCurve: return "bspline";
        case GeomAbs_BezierCurve: return "bezier";
        default: return "other";
    }
}

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    for (int i = 2; i < argc; ++i) {
        const char* s = argv[i];
        if (s[0] == 'f') {
            int fid = std::atoi(s + 1);
            const TopoDS_Face f = TopoDS::Face(m.faces(fid));
            BRepAdaptor_Surface sa(f);
            std::printf("face %d: %s tol=%.4g\n", fid, surfName(sa.GetType()),
                        BRep_Tool::Tolerance(f));
        } else if (s[0] == 'e') {
            int eid = std::atoi(s + 1);
            const TopoDS_Edge e = TopoDS::Edge(m.edges(eid));
            double cf, cl;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, cf, cl);
            GeomAdaptor_Curve ca(c3);
            std::printf("edge %d: %s tol=%.4g range=[%.4g,%.4g]", eid,
                        curveName(ca.GetType()), BRep_Tool::Tolerance(e), cf,
                        cl);
            gp_Pnt p0 = c3->Value(cf), p1 = c3->Value(cl);
            std::printf(" p0=(%.3f,%.3f,%.3f) p1=(%.3f,%.3f,%.3f)", p0.X(),
                        p0.Y(), p0.Z(), p1.X(), p1.Y(), p1.Z());
            if (m.edgeToFaces.Contains(e)) {
                std::printf(" owners:");
                for (const auto& o : m.edgeToFaces.FindFromKey(e)) {
                    std::printf(" %d", m.faces.FindIndex(o));
                }
            }
            std::printf("\n");
        }
    }
    return 0;
}
