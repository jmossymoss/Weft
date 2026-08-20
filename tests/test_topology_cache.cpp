#include "weft/analysis.hpp"
#include "weft/model.hpp"
#include "weft/topology_cache.hpp"

#include <cstdio>

static int gFails = 0;
#define CHECK(c)                                                             \
    do {                                                                     \
        if (!(c)) {                                                          \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++gFails;                                                        \
        }                                                                    \
    } while (0)

int main() {
    weft::Model model = weft::loadStep("tests/fixtures/demo.step");
    weft::Analysis a = weft::analyze(model);
    CHECK(!a.topology.empty());
    CHECK(a.topology.faceCount == model.faceCount());
    CHECK(a.topology.edgeCount == model.edgeCount());
    CHECK(a.topology.faceEdgeOffset.size() ==
          size_t(a.topology.faceCount) + 1);
    // Every face's CSR edge count matches FaceInfo::edgeIds.
    for (int fid = 1; fid <= a.topology.faceCount; ++fid) {
        CHECK(a.topology.faceEdgeCount(uint32_t(fid)) ==
              uint32_t(a.faces[size_t(fid) - 1].edgeIds.size()));
    }
    a.topology.markDirtyClosure(1);
    CHECK(a.topology.faceDirty[0] == 1);
    a.topology.clearDirty();
    CHECK(a.topology.faceDirty[0] == 0);
    if (gFails) {
        std::fprintf(stderr, "%d FAILURE(S)\n", gFails);
        return 1;
    }
    std::printf("topology_cache: ok (faces=%d edges=%d)\n",
                a.topology.faceCount, a.topology.edgeCount);
    return 0;
}
