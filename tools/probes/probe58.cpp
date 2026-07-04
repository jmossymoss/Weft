// Which polygons use given vertex ids; prints face + poly verts around it.
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
    gs.defaults.quadDominant = std::atoi(argv[2]) != 0;
    weft::PolyMesh mesh = weft::generate(m, a, gs);
    std::set<uint32_t> want;
    for (int i = 3; i < argc; ++i) want.insert(uint32_t(std::atoi(argv[i])));
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        bool hit = false;
        for (uint32_t vi : poly) hit |= want.count(vi) > 0;
        if (!hit) continue;
        std::printf("poly %zu face %d (%zu verts):", p,
                    mesh.polygonFaceId[p], poly.size());
        if (poly.size() > 12) {
            std::printf(" [big] ");
            for (uint32_t vi : poly) {
                if (want.count(vi)) std::printf(" *%u*", vi);
            }
            // print neighbours of wanted ids in ring order
            for (size_t i = 0; i < poly.size(); ++i) {
                if (!want.count(poly[i])) continue;
                std::printf(" [..%u %u %u..]",
                            poly[(i + poly.size() - 1) % poly.size()],
                            poly[i], poly[(i + 1) % poly.size()]);
            }
        } else {
            for (uint32_t vi : poly) std::printf(" %u", vi);
        }
        std::printf("\n");
    }
    return 0;
}
