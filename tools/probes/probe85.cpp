#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"
#include <cstdio>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    for (bool conform : {true, false}) {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.conformBorders = conform;
        weft::GenerationReport rep;
        weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
        weft::ValidationReport vr = weft::validateMesh(mesh, &m);
        std::printf("conform=%d: open %zu nm %zu\n", conform, vr.openEdges,
                    vr.nonManifoldEdges);
    }
    return 0;
}
