// Trace face 294's meshing with the debug log filtered to relevant lines.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.parallelMeshing = false;  // keep the log ordered
    std::FILE* log = std::fopen(argv[2], "w");
    weft::setGenerateDebugLog(log);
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::fclose(log);
    return 0;
}
