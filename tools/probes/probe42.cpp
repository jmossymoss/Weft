#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::setGenerateDebugLog(stderr);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.parallelMeshing = false;
    weft::PolyMesh mesh = weft::generate(m, a, gs, nullptr);
    return 0;
}
