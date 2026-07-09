// Per-face forensics around foam's slit: poly counts, mesher kind, and
// whether each face's part vertices actually lie on its own B-rep border.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <map>
#include <set>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::map<int, int> polyCount;
    std::map<int, int> polySizeMax;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        int f = mesh.polygonFaceId[p];
        ++polyCount[f];
        polySizeMax[f] =
            std::max(polySizeMax[f], (int)mesh.polygons[p].size());
    }
    for (int fid : {275, 280, 281, 292, 294, 296, 298, 300, 302, 304}) {
        auto it = rep.faceMesher.find(fid);
        std::printf("face #%d: %s, %d polys (max size %d)\n", fid,
                    it == rep.faceMesher.end() ? "?" : weft::mesherKindName(it->second),
                    polyCount.count(fid) ? polyCount[fid] : 0,
                    polySizeMax.count(fid) ? polySizeMax[fid] : 0);
    }
    return 0;
}
