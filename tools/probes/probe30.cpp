// Dump open-edge segments of chosen faces: 3D coords + the neighbour face
// meshers, to see WHERE seams fail to zip.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <map>
#include <set>
#include <vector>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::set<int> want;
    for (int i = 2; i < argc; ++i) want.insert(std::atoi(argv[i]));
    std::map<std::pair<uint32_t, uint32_t>, std::pair<int, int>> dir;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            auto key = std::make_pair(poly[i], poly[(i + 1) % poly.size()]);
            auto [it, ins] = dir.try_emplace(
                key, std::make_pair(0, mesh.polygonFaceId[p]));
            it->second.first++;
        }
    }
    for (auto& [e, info] : dir) {
        if (info.first != 1 || dir.count({e.second, e.first})) continue;
        if (!want.empty() && !want.count(info.second)) continue;
        auto& A = mesh.vertices[e.first];
        auto& B = mesh.vertices[e.second];
        double len = std::sqrt((A[0]-B[0])*(A[0]-B[0]) +
                               (A[1]-B[1])*(A[1]-B[1]) +
                               (A[2]-B[2])*(A[2]-B[2]));
        std::printf("face %-4d open len %.4g  (%.4g %.4g %.4g)-(%.4g %.4g %.4g)\n",
                    info.second, len, A[0], A[1], A[2], B[0], B[1], B[2]);
        // who owns nearby verts? find other faces with a vertex within len
        std::map<int, int> near;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] == info.second) continue;
            for (uint32_t vi : mesh.polygons[p]) {
                auto& V = mesh.vertices[vi];
                double dA = std::sqrt((V[0]-A[0])*(V[0]-A[0]) +
                                      (V[1]-A[1])*(V[1]-A[1]) +
                                      (V[2]-A[2])*(V[2]-A[2]));
                if (dA < len * 1.05) { near[mesh.polygonFaceId[p]]++; break; }
            }
        }
        std::printf("   near faces:");
        for (auto& [fid, n] : near) {
            std::printf(" %d(%s)", fid,
                        fid > 0 ? weft::mesherKindName(rep.faceMesher.at(fid))
                                : "op");
        }
        std::printf("\n");
    }
    return 0;
}
