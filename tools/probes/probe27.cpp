#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <map>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    for (int quads = 0; quads <= 1; ++quads) {
        weft::GenerationSettings gs;
        gs.defaults.adaptive = true;
        gs.defaults.quadDominant = quads != 0;
        try {
            weft::PolyMesh mesh = weft::generate(m, a, gs);
            std::map<std::pair<uint32_t, uint32_t>, int> dir;
            for (auto& poly : mesh.polygons)
                for (size_t i = 0; i < poly.size(); ++i)
                    ++dir[{poly[i], poly[(i + 1) % poly.size()]}];
            int multi = 0, open = 0;
            for (auto& [e, c] : dir) {
                if (c != 1) ++multi;
                else if (!dir.count({e.second, e.first})) ++open;
            }
            std::printf("%s faces=%d quads=%d: polys %zu (q %zu t %zu) "
                        "multi %d open %d\n",
                        argv[1], m.faceCount(), quads, mesh.polygonCount(),
                        mesh.countQuads(), mesh.countTris(), multi, open);
        } catch (const std::exception& e) {
            std::printf("%s quads=%d THREW: %s\n", argv[1], quads, e.what());
        }
    }
    return 0;
}
