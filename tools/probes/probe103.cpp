// probe103: report faceAcross (which patch axis the fillet-loops knob
// drives) for every blend strip — the semantic-knob mapping monitor.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    for (const auto& [fid, ax] : rep.faceAcross) {
        const auto cit = rep.faceCounts.find(fid);
        std::printf("face %d: across=%s counts=%d,%d\n", fid,
                    ax == 1 ? "u" : "v",
                    cit == rep.faceCounts.end() ? -1 : cit->second[0],
                    cit == rep.faceCounts.end() ? -1 : cit->second[1]);
    }
    return 0;
}
