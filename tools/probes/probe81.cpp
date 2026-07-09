// Locate open mesh edges at cad profile: positions, owning faces, and the
// nearest B-rep edge with its solved count context.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <map>
#include <vector>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::map<std::pair<uint32_t, uint32_t>, std::vector<size_t>> uses;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t u = poly[i], v = poly[(i + 1) % poly.size()];
            if (u > v) std::swap(u, v);
            uses[{u, v}].push_back(p);
        }
    }
    for (const auto& [e, polys] : uses) {
        if (polys.size() == 2) continue;
        const auto& A = mesh.vertices[e.first];
        const auto& B = mesh.vertices[e.second];
        std::printf("%s edge (%.4f,%.4f,%.4f)-(%.4f,%.4f,%.4f) len %.4f faces:",
                    polys.size() == 1 ? "open" : "nonmanifold", A[0], A[1],
                    A[2], B[0], B[1], B[2],
                    std::hypot(A[0]-B[0], std::hypot(A[1]-B[1], A[2]-B[2])));
        for (size_t p : polys) std::printf(" #%d", mesh.polygonFaceId[p]);
        std::printf("\n");
    }
    return 0;
}
