// Trace face 481 (a reconstructed cap) at default settings.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.parallelMeshing = false;
    std::FILE* log = std::fopen(argv[2], "w");
    weft::setGenerateDebugLog(log);
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::fclose(log);
    for (int fid : {481, 825}) {
        const auto& fi = a.faces[fid - 1];
        std::printf("face %d edges:", fid);
        for (int e : fi.edgeIds) {
            std::printf(" #%d(div %d)", e,
                        rep.edgeDivisions.count(e) ? rep.edgeDivisions[e]
                                                   : -1);
        }
        std::printf("\n");
    }
    return 0;
}
