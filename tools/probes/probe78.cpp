// Which faces demoted (faceBuild 1/-1/2), at default and cad settings?
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <TopExp_Explorer.hxx>
#include <cstdio>

static void run(const weft::Model& m, const weft::Analysis& a, bool cad) {
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    if (cad) {
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
    }
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::printf("%s:\n", cad ? "cad" : "default");
    for (const auto& [fid, how] : rep.faceBuild) {
        if (how == 0) continue;
        const auto& fi = a.faces[fid - 1];
        std::printf("  face #%d %s (%s) edges %zu -> %s\n", fid,
                    weft::mesherKindName(rep.faceMesher.at(fid)),
                    weft::surfaceTypeName(fi.type), fi.edgeIds.size(),
                    how == 2 ? "contract floor"
                    : how == 1 ? "RAW TRIANGULATION"
                               : "EMPTY");
    }
}

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    std::printf("faces: %d\n", m.faceCount());
    run(m, a, false);
    run(m, a, true);
    return 0;
}
