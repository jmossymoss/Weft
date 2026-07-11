// probe102: dump every polygon of one face — vertex ids, anchors (uv),
// 3D positions, and the foldedPolys flag. For the tiny openband drums
// that fold one cell below the self-heal's size gate.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    const int want = std::atoi(argv[2]);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    const std::vector<uint8_t> folded = weft::foldedPolys(m, mesh);
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] != want) continue;
        std::printf("poly %zu%s:", p, folded[p] ? " FOLDED" : "");
        for (uint32_t v : mesh.polygons[p]) {
            const auto& P = mesh.vertices[v];
            const weft::Anchor& an = mesh.anchors[v];
            std::printf(" v%u[f%d uv=%.4f,%.4f p=%.3f,%.3f,%.3f]", v,
                        an.faceId, an.u, an.v, P[0], P[1], P[2]);
        }
        std::printf("\n");
    }
    return 0;
}
