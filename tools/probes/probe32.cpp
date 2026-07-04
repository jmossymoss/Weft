// For chosen faces: solved edge divisions (if constrained), part polys,
// and the face's actual border vertices along each B-rep edge.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
#include <set>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    for (int i = 2; i < argc; ++i) {
        int eid = std::atoi(argv[i]);
        auto it = rep.edgeDivisions.find(eid);
        if (it != rep.edgeDivisions.end())
            std::printf("edge %d divisions %d\n", eid, it->second);
        else
            std::printf("edge %d divisions (unconstrained)\n", eid);
    }
    // count polys per face for a few small faces
    std::set<int> small = {3, 7};
    for (int fid : small) {
        int n = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p)
            if (mesh.polygonFaceId[p] == fid) ++n;
        std::printf("face %d polys %d\n", fid, n);
    }
    return 0;
}
