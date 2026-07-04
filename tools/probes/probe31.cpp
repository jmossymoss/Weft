// Dump B-rep edge topology for chosen faces: edge ids, lengths, and which
// faces share each edge (looking for sliver edges / T-junctions).
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
#include <set>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    std::set<int> want;
    for (int i = 2; i < argc; ++i) want.insert(std::atoi(argv[i]));
    for (int fid : want) {
        const auto& fi = a.faces[fid - 1];
        std::printf("face %d type=%s edges:", fid,
                    weft::surfaceTypeName(fi.type));
        std::printf("\n");
        for (int eid : fi.edgeIds) {
            const auto& ei = a.edges[eid - 1];
            std::printf("  edge %-4d len %-10.5g faces:", eid, ei.length);
            for (int of : ei.faceIds) std::printf(" %d", of);
            std::printf("\n");
        }
    }
    return 0;
}
