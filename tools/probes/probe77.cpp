// Which B-rep edges pass through the given 3D point(s)? Samples every
// edge densely and reports edges whose polyline comes within tol.
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <cstdio>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    gp_Pnt P(atof(argv[2]), atof(argv[3]), atof(argv[4]));
    const double tol = argc > 5 ? atof(argv[5]) : 1e-3;
    for (int e = 1; e <= m.edges.Extent(); ++e) {
        const TopoDS_Edge& edge = TopoDS::Edge(m.edges(e));
        if (BRep_Tool::Degenerated(edge)) continue;
        BRepAdaptor_Curve c(edge);
        const double f = c.FirstParameter(), l = c.LastParameter();
        double best = 1e300;
        for (int i = 0; i <= 256; ++i) {
            best = std::min(best,
                            P.Distance(c.Value(f + (l - f) * i / 256.0)));
        }
        if (best > tol) continue;
        std::printf("edge #%d len %.4f dist %.6f faces:", e,
                    GCPnts_AbscissaPoint::Length(c), best);
        if (m.edgeToFaces.Contains(edge)) {
            for (const TopoDS_Shape& s : m.edgeToFaces.FindFromKey(edge)) {
                std::printf(" %d", m.faces.FindIndex(s));
            }
        }
        std::printf("\n");
    }
    return 0;
}
