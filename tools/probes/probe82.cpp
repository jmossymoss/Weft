// Which uniform sample count would place vertices at the observed
// positions on edge 798? And what does the density solve say?
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <TopoDS.hxx>
#include <cstdio>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    const int eid = 798;
    auto it = rep.edgeDivisions.find(eid);
    std::printf("edgeDivisions[798] = %d\n",
                it == rep.edgeDivisions.end() ? -1 : it->second);
    gp_Pnt A(4, 6.1404, -74.4635), B(4, 5.9720, -73.8346),
        C(4, 6.0039, -74.0701);
    BRepAdaptor_Curve c(TopoDS::Edge(m.edges(eid)));
    const double f = c.FirstParameter(), l = c.LastParameter();
    for (int n : {8, 12, 16, 20, 24, 28, 32, 40, 48, 56, 64}) {
        double dA = 1e300, dB = 1e300, dC = 1e300;
        for (int i = 0; i <= n; ++i) {
            gp_Pnt p = c.Value(f + (l - f) * i / n);
            dA = std::min(dA, p.Distance(A));
            dB = std::min(dB, p.Distance(B));
            dC = std::min(dC, p.Distance(C));
        }
        std::printf("n=%2d: dA %.4f dB %.4f dC %.4f\n", n, dA, dB, dC);
    }
    return 0;
}
