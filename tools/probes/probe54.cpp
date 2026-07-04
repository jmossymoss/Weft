// SVG dump of one face's polygons (projected to XY), folded ones red.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.defaults.quadDominant = std::atoi(argv[2]) != 0;
    int fid = std::atoi(argv[3]);
    weft::PolyMesh mesh = weft::generate(m, a, gs);
    auto folded = weft::foldedPolys(m, mesh);
    double x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] != fid) continue;
        for (uint32_t vi : mesh.polygons[p]) {
            x0 = std::min(x0, mesh.vertices[vi][0]);
            x1 = std::max(x1, mesh.vertices[vi][0]);
            y0 = std::min(y0, mesh.vertices[vi][1]);
            y1 = std::max(y1, mesh.vertices[vi][1]);
        }
    }
    double s = 900.0 / std::max(x1 - x0, y1 - y0);
    FILE* f = fopen(argv[4], "w");
    fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='950' "
               "height='950' viewBox='-25 -25 950 950'>\n");
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            if ((folded[p] != 0) != (pass == 1)) continue;
            fprintf(f, "<polygon points='");
            for (uint32_t vi : mesh.polygons[p]) {
                fprintf(f, "%.2f,%.2f ",
                        (mesh.vertices[vi][0] - x0) * s,
                        900.0 - (mesh.vertices[vi][1] - y0) * s);
            }
            fprintf(f, "' fill='%s' stroke='black' stroke-width='0.6' "
                       "fill-opacity='0.5'/>\n",
                    pass ? "red" : "#9c9");
        }
    }
    fprintf(f, "</svg>\n");
    fclose(f);
    printf("wrote %s\n", argv[4]);
    return 0;
}
