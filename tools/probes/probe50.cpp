// Run foldedPolys() on a model and report folded counts per face.
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
        weft::GenerationReport rep;
        weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
        auto folded = weft::foldedPolys(m, mesh);
        std::map<int, int> perFace;
        int total = 0;
        for (size_t p = 0; p < folded.size(); ++p) {
            if (!folded[p]) continue;
            ++perFace[mesh.polygonFaceId[p]];
            ++total;
        }
        std::printf("%s quads=%d: %d folded polys in %zu\n", argv[1], quads,
                    total, mesh.polygonCount());
        for (auto& [fid, n] : perFace) {
            std::printf("  face %-4d (%s): %d folded\n", fid,
                        weft::mesherKindName(rep.faceMesher.at(fid)), n);
        }
    }
    return 0;
}
