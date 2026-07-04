// One face's winding conventions: orientation, raw wire UV area (as
// WireExplorer walks it), and the chart handedness du x dv vs 3D wire.
#include "weft/analysis.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Vec.hxx>
#include <cstdio>
#include <cstdlib>
#include <vector>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    for (int i = 2; i < argc; ++i) {
        int fid = std::atoi(argv[i]);
        TopoDS_Face face = TopoDS::Face(m.faces(fid));
        TopoDS_Wire wire = BRepTools::OuterWire(face);
        std::vector<gp_Pnt2d> uv;
        std::vector<gp_Pnt> p3;
        for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
            const TopoDS_Edge e = we.Current();
            double f2, l2, f3, l3;
            Handle(Geom2d_Curve) c2 = BRep_Tool::CurveOnSurface(e, face, f2, l2);
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f3, l3);
            const bool rev = e.Orientation() == TopAbs_REVERSED;
            for (int k = 0; k < 8; ++k) {
                double t = rev ? 1.0 - k / 8.0 : k / 8.0;
                uv.push_back(c2->Value(f2 + (l2 - f2) * t));
                p3.push_back(c3->Value(f3 + (l3 - f3) * t));
            }
        }
        double a2 = 0;
        for (size_t k = 0; k < uv.size(); ++k) {
            const gp_Pnt2d& a = uv[k];
            const gp_Pnt2d& b = uv[(k + 1) % uv.size()];
            a2 += a.X() * b.Y() - b.X() * a.Y();
        }
        // 3D Newell normal of the wire polygon
        double nx = 0, ny = 0, nz = 0;
        for (size_t k = 0; k < p3.size(); ++k) {
            const gp_Pnt& a = p3[k];
            const gp_Pnt& b = p3[(k + 1) % p3.size()];
            nx += (a.Y() - b.Y()) * (a.Z() + b.Z());
            ny += (a.Z() - b.Z()) * (a.X() + b.X());
            nz += (a.X() - b.X()) * (a.Y() + b.Y());
        }
        BRepAdaptor_Surface s(face);
        gp_Pnt sp;
        gp_Vec du, dv;
        s.D1((s.FirstUParameter() + s.LastUParameter()) / 2,
             (s.FirstVParameter() + s.LastVParameter()) / 2, sp, du, dv);
        gp_Vec n = du.Crossed(dv);
        double dot = n.X() * nx + n.Y() * ny + n.Z() * nz;
        std::printf(
            "face %d %s: wire uvArea=%+.4g  newell.(duxdv)=%+.4g\n", fid,
            face.Orientation() == TopAbs_REVERSED ? "REVERSED" : "FORWARD",
            a2 / 2, dot);
    }
    return 0;
}
