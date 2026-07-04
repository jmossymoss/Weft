// For flagged faces: folded vs total polys, to spot whole-face flags
// (winding-convention mismatch) vs genuine partial folds.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
#include <map>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.defaults.quadDominant = argc > 2 && std::atoi(argv[2]) != 0;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    auto folded = weft::foldedPolys(m, mesh);
    std::map<int, std::pair<int, int>> perFace;  // fid -> {folded, total}
    for (size_t p = 0; p < folded.size(); ++p) {
        auto& e = perFace[mesh.polygonFaceId[p]];
        e.first += folded[p] ? 1 : 0;
        e.second += 1;
    }
    for (auto& [fid, e] : perFace) {
        if (!e.first) continue;
        std::printf("face %-4d (%s): %d/%d folded\n", fid,
                    fid > 0 ? weft::mesherKindName(rep.faceMesher.at(fid))
                            : "op",
                    e.first, e.second);
    }
    return 0;
}
