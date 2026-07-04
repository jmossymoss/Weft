// Plasticity-comparison stats: defaults vs minimal profile, with per-
// mesher triangle attribution to find where the tris come from.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <map>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    for (int minimal = 0; minimal <= 1; ++minimal) {
        weft::GenerationSettings gs;
        gs.defaults.adaptive = true;
        gs.defaults.minimal = minimal != 0;
        weft::GenerationReport rep;
        weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
        std::map<std::pair<uint32_t, uint32_t>, int> dir;
        for (auto& poly : mesh.polygons)
            for (size_t i = 0; i < poly.size(); ++i)
                ++dir[{poly[i], poly[(i + 1) % poly.size()]}];
        int multi = 0, open = 0;
        for (auto& [e, c] : dir) {
            if (c != 1) ++multi;
            else if (!dir.count({e.second, e.first})) ++open;
        }
        std::printf("minimal=%d: polys %zu (q %zu t %zu n %zu) multi %d "
                    "open %d\n",
                    minimal, mesh.polygonCount(), mesh.countQuads(),
                    mesh.countTris(), mesh.countNgons(), multi, open);
        // triangle attribution by mesher kind
        std::map<std::string, int> triBy, polyBy;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            int fid = mesh.polygonFaceId[p];
            const char* k =
                fid > 0 && rep.faceMesher.count(fid)
                    ? weft::mesherKindName(rep.faceMesher.at(fid))
                    : "op";
            polyBy[k]++;
            if (mesh.polygons[p].size() == 3) triBy[k]++;
        }
        for (auto& [k, n] : triBy) {
            std::printf("  tris from %-14s %6d (of %d polys)\n", k.c_str(),
                        n, polyBy[k]);
        }
    }
    return 0;
}
