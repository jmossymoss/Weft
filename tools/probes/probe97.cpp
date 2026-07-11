// probe97: does a coons patch ROTATION fix face 18's folded skirt on the
// demo fixture? Try rotate 0..3 per run, count that face's folds.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    const int fid = std::atoi(argv[2]);
    for (int rot = 0; rot < 4; ++rot) {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.perFace[fid] = gs.defaults;
        gs.perFace[fid].coonsRotate = rot;
        weft::GenerationReport rep;
        weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
        auto it = rep.faceBuild.find(fid);
        std::printf("rotate=%d: build=%d\n", rot,
                    it == rep.faceBuild.end() ? -99 : it->second);
    }
    return 0;
}
