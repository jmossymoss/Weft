// For two faces sharing an edge: list each face's mesh vertices lying on
// that edge's curve, sorted by curve parameter.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <Extrema_ExtPC.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <set>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.defaults.quadDominant = std::atoi(argv[2]) != 0;
    int eid = std::atoi(argv[3]);
    weft::PolyMesh mesh = weft::generate(m, a, gs);
    BRepAdaptor_Curve c(TopoDS::Edge(m.edges(eid)));
    for (int i = 4; i < argc; ++i) {
        int fid = std::atoi(argv[i]);
        std::set<uint32_t> vs;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            for (uint32_t vi : mesh.polygons[p]) vs.insert(vi);
        }
        std::vector<std::pair<double, uint32_t>> on;
        for (uint32_t vi : vs) {
            const auto& v = mesh.vertices[vi];
            gp_Pnt P(v[0], v[1], v[2]);
            try {
                Extrema_ExtPC ext(P, c);
                if (!ext.IsDone()) continue;
                double bd = 1e300, bt = 0;
                for (int k = 1; k <= ext.NbExt(); ++k) {
                    double d = std::sqrt(ext.SquareDistance(k));
                    if (d < bd) { bd = d; bt = ext.Point(k).Parameter(); }
                }
                if (bd < 1e-4) on.push_back({bt, vi});
            } catch (...) {}
        }
        std::sort(on.begin(), on.end());
        std::printf("face %d: %zu verts on edge %d:\n", fid, on.size(), eid);
        for (auto& [t, vi] : on) {
            const auto& v = mesh.vertices[vi];
            std::printf("  t=%-10.5g v%-6u (%.5g %.5g %.5g)\n", t, vi,
                        v[0], v[1], v[2]);
        }
    }
    return 0;
}
