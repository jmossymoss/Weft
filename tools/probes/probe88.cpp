// probe88: per-face polygon counts + report kinds for suspicious faces
// under stitch (which faces emitted nothing / what mesher won).
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
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.decoupleSeams = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::map<int, int> polyCount;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        ++polyCount[mesh.polygonFaceId[p]];
    }
    for (int i = 2; i < argc; ++i) {
        int fid = std::atoi(argv[i]);
        std::printf("face %d: polys=%d", fid, polyCount[fid]);
        auto it = rep.faceBuild.find(fid);
        if (it != rep.faceBuild.end()) {
            std::printf(" build=%d", it->second);
        }
        auto mit = rep.faceMesher.find(fid);
        if (mit != rep.faceMesher.end()) {
            std::printf(" kind=%s", weft::mesherKindName(mit->second));
        }
        auto cit = rep.faceCounts.find(fid);
        if (cit != rep.faceCounts.end()) {
            std::printf(" counts=%d/%d", cit->second[0], cit->second[1]);
        }
        std::printf("\n");
    }
    return 0;
}
