// SVG dump of one face's polygons in UV space (vertices projected onto
// the face surface), folded ones red. Also prints face edge topology.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <map>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.defaults.quadDominant = std::atoi(argv[2]) != 0;
    int fid = std::atoi(argv[3]);
    weft::PolyMesh mesh = weft::generate(m, a, gs);
    auto folded = weft::foldedPolys(m, mesh);

    const auto& fi = a.faces[fid - 1];
    std::printf("face %d type=%s edges:", fid,
                weft::surfaceTypeName(fi.type));
    for (int eid : fi.edgeIds) std::printf(" %d", eid);
    std::printf("\n");

    TopoDS_Face face = TopoDS::Face(m.faces(fid));
    Handle(Geom_Surface) hs = BRep_Tool::Surface(face);
    GeomAPI_ProjectPointOnSurf proj;
    proj.Init(gp_Pnt(0, 0, 0), hs);
    std::map<uint32_t, std::pair<double, double>> uvCache;
    auto uvOf = [&](uint32_t vi) {
        auto it = uvCache.find(vi);
        if (it != uvCache.end()) return it->second;
        const auto& v = mesh.vertices[vi];
        double pu = 0, pv = 0;
        if (vi < mesh.anchors.size() && mesh.anchors[vi].faceId == fid) {
            pu = mesh.anchors[vi].u;
            pv = mesh.anchors[vi].v;
        } else {
            proj.Perform(gp_Pnt(v[0], v[1], v[2]));
            if (proj.IsDone() && proj.NbPoints() > 0)
                proj.LowerDistanceParameters(pu, pv);
        }
        uvCache[vi] = {pu, pv};
        return uvCache[vi];
    };
    double x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] != fid) continue;
        for (uint32_t vi : mesh.polygons[p]) {
            auto [u, v] = uvOf(vi);
            x0 = std::min(x0, u); x1 = std::max(x1, u);
            y0 = std::min(y0, v); y1 = std::max(y1, v);
        }
    }
    double s = 900.0 / std::max(x1 - x0, std::max(y1 - y0, 1e-12));
    FILE* f = fopen(argv[4], "w");
    fprintf(f, "<svg xmlns='http://www.w3.org/2000/svg' width='950' "
               "height='950' viewBox='-25 -25 950 950'>\n");
    for (int pass = 0; pass < 2; ++pass) {
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            if ((folded[p] != 0) != (pass == 1)) continue;
            fprintf(f, "<polygon points='");
            for (uint32_t vi : mesh.polygons[p]) {
                auto [u, v] = uvOf(vi);
                fprintf(f, "%.2f,%.2f ", (u - x0) * s,
                        900.0 - (v - y0) * s);
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
