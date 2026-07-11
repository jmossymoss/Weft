// probe101: mesh-wide folded-poly census by face (weft::foldedPolys —
// the app's ruler), for chasing folds that slip the per-face self-heal
// (parts under 8 polys skip that census).
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"
#include <cstdio>
#include <map>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    const std::vector<uint8_t> folded = weft::foldedPolys(m, mesh);
    std::map<int, int> byFace;
    for (size_t p = 0; p < folded.size(); ++p) {
        if (folded[p]) ++byFace[mesh.polygonFaceId[p]];
    }
    int total = 0;
    for (const auto& [fid, n] : byFace) {
        const auto kit = rep.faceMesher.find(fid);
        std::printf("face %d: %d folded (%s, build=%d)\n", fid, n,
                    kit == rep.faceMesher.end()
                        ? "?"
                        : weft::mesherKindName(kit->second),
                    rep.faceBuild.count(fid) ? rep.faceBuild[fid] : -99);
        total += n;
    }
    std::printf("total %d\n", total);
    return 0;
}
