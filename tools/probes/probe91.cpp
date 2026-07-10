// probe91: solved edge divisions for given edges, stitch vs default.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    for (bool stitch : {false, true}) {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.decoupleSeams = stitch;
        weft::GenerationReport rep;
        weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
        std::printf("stitch=%d:", stitch ? 1 : 0);
        for (int i = 2; i < argc; ++i) {
            int eid = std::atoi(argv[i]);
            auto it = rep.edgeDivisions.find(eid);
            std::printf(" e%d=%d", eid,
                        it == rep.edgeDivisions.end() ? -1 : it->second);
        }
        std::printf("\n");
    }
    return 0;
}
