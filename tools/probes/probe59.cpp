// Duplicate directed edges: which faces own the offending polygons.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.defaults.quadDominant = std::atoi(argv[2]) != 0;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::map<std::pair<uint32_t, uint32_t>, std::vector<size_t>> dir;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            dir[{poly[i], poly[(i + 1) % poly.size()]}].push_back(p);
        }
    }
    for (auto& [e, ps] : dir) {
        if (ps.size() < 2) continue;
        auto& A = mesh.vertices[e.first];
        std::printf("dup edge v%u->v%u at (%.4g %.4g %.4g) faces:", e.first,
                    e.second, A[0], A[1], A[2]);
        for (size_t p : ps) {
            int fid = mesh.polygonFaceId[p];
            std::printf(" %d(%s,%zuv)", fid,
                        fid > 0 ? weft::mesherKindName(rep.faceMesher.at(fid))
                                : "op",
                        mesh.polygons[p].size());
        }
        std::printf("\n");
    }
    return 0;
}
