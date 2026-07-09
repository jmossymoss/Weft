// Bisect foam's face-294 collapse: mesh with conform on/off and report
// face 294/275 poly counts, open and non-manifold edges each way.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"
#include <cstdio>

static void run(const weft::Model& m, const weft::Analysis& a, bool conform) {
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.conformBorders = conform;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    weft::ValidationReport vr = weft::validateMesh(mesh, &m);
    int f294 = 0, f275 = 0;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] == 294) ++f294;
        if (mesh.polygonFaceId[p] == 275) ++f275;
    }
    std::printf("conform=%d: open %zu nm %zu, face294 polys %d, face275 polys %d\n",
                conform, vr.openEdges, vr.nonManifoldEdges, f294, f275);
}

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    run(m, a, true);
    run(m, a, false);
    return 0;
}
