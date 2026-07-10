// probe89: census of face build states under stitch vs default.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <map>
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
        std::map<int, int> tally;
        for (const auto& [fid, b] : rep.faceBuild) ++tally[b];
        std::printf("stitch=%d:", stitch ? 1 : 0);
        for (const auto& [b, n] : tally) std::printf(" build[%d]=%d", b, n);
        std::printf("  empties:");
        int shown = 0;
        for (const auto& [fid, b] : rep.faceBuild) {
            if (b == -1 && shown < 20) {
                std::printf(" %d(%s)", fid,
                            weft::mesherKindName(rep.faceMesher[fid]));
                ++shown;
            }
        }
        std::printf("\n");
    }
    return 0;
}
