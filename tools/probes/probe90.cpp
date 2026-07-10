// probe90: dump a face's polygons whose verts lie near a given point
// (diagnose dropped/decimated hole rings).
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.decoupleSeams = std::getenv("NOSTITCHSET") ? false : true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    const int fid = std::atoi(argv[2]);
    const double cx = std::atof(argv[3]), cy = std::atof(argv[4]),
                 cz = std::atof(argv[5]), R = std::atof(argv[6]);
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] != fid) continue;
        const auto& poly = mesh.polygons[p];
        int nNear = 0;
        for (uint32_t v : poly) {
            const auto& P = mesh.vertices[v];
            const double dx = P[0] - cx, dy = P[1] - cy, dz = P[2] - cz;
            if (dx * dx + dy * dy + dz * dz < R * R) ++nNear;
        }
        std::printf("poly %zu: %zu verts, %d within %g of center\n", p,
                    poly.size(), nNear, R);
        if (nNear) {
            for (uint32_t v : poly) {
                const auto& P = mesh.vertices[v];
                const double dx = P[0] - cx, dy = P[1] - cy,
                             dz = P[2] - cz;
                const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (d < R) {
                    std::printf("  v%u (%.3f,%.3f,%.3f) d=%.3f\n", v, P[0],
                                P[1], P[2], d);
                }
            }
        }
    }
    return 0;
}
