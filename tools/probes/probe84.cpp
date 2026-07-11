// Dump each face's border-vertex chain along one B-rep edge (positions by
// curve parameter) to see exactly how the two sides sampled it.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <Extrema_ExtPC.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    const int eid = atoi(argv[2]);
    const int fidA = atoi(argv[3]), fidB = atoi(argv[4]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    auto it = rep.edgeDivisions.find(eid);
    std::printf("edgeDivisions[%d] = %d\n", eid,
                it == rep.edgeDivisions.end() ? -1 : it->second);
    BRepAdaptor_Curve c(TopoDS::Edge(m.edges(eid)));
    const double f = c.FirstParameter(), l = c.LastParameter();
    for (int fid : {fidA, fidB}) {
        std::vector<std::pair<double, uint32_t>> chain;
        std::vector<char> seen(mesh.vertices.size(), 0);
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            for (uint32_t v : mesh.polygons[p]) {
                if (seen[v]) continue;
                seen[v] = 1;
                gp_Pnt P(mesh.vertices[v][0], mesh.vertices[v][1],
                         mesh.vertices[v][2]);
                double bestD = 1e300, bestT = 0;
                for (int i = 0; i <= 128; ++i) {
                    double t = f + (l - f) * i / 128.0;
                    double d = P.Distance(c.Value(t));
                    if (d < bestD) { bestD = d; bestT = t; }
                }
                if (bestD < 0.08) chain.push_back({bestT, v});
            }
        }
        std::sort(chain.begin(), chain.end());
        std::printf("face %d: %zu verts on edge %d, params:", fid,
                    chain.size(), eid);
        for (auto& [t, v] : chain) std::printf(" %.3f", (t - f) / (l - f));
        std::printf("\n");
    }
    return 0;
}
