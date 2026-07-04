#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    for (int i = 2; i < argc; ++i) {
        int fid = std::atoi(argv[i]);
        std::printf("face %d mesher %s edges %zu\n", fid,
                    weft::mesherKindName(rep.faceMesher.at(fid)),
                    a.faces[fid - 1].edgeIds.size());
    }
    return 0;
}
