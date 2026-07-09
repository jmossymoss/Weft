// Locate non-manifold mesh edges: which polygons/faces share them, where.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <map>
#include <set>
#include <vector>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = false;
    gs.defaults.relativeDeviation = false;
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
        if (polys.size() <= 2) continue;
        const auto& p0 = mesh.vertices[e.first];
        const auto& p1 = mesh.vertices[e.second];
        std::printf("non-manifold edge v%u-v%u  (%.4f,%.4f,%.4f)-(%.4f,%.4f,%.4f) used by %zu polys, faces:",
                    e.first, e.second, p0[0], p0[1], p0[2], p1[0], p1[1],
                    p1[2], polys.size());
        std::set<int> fids;
        for (size_t p : polys) fids.insert(mesh.polygonFaceId[p]);
        for (int f : fids) {
            auto it = rep.faceMesher.find(f);
            std::printf(" #%d(%s)", f,
                        it == rep.faceMesher.end()
                            ? "?"
                            : weft::mesherKindName(it->second));
        }
        std::printf("\n");
        for (size_t p : polys) {
            std::printf("    poly %zu face #%d size %zu verts:", p,
                        mesh.polygonFaceId[p], mesh.polygons[p].size());
            for (uint32_t v : mesh.polygons[p]) std::printf(" %u", v);
            std::printf("\n");
        }
    }
    return 0;
}
