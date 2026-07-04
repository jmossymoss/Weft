// Which faces fall back under the minimal profile, and why coons said no.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <BRepTools.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <cstdio>
#include <map>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.defaults.minimal = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    std::map<int, int> polyCount;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        ++polyCount[mesh.polygonFaceId[p]];
    }
    for (auto& [fid, kind] : rep.faceMesher) {
        if (kind != weft::MesherKind::Fallback) continue;
        const auto& fi = a.faces[fid - 1];
        int wires = 0;
        for (TopExp_Explorer wx(TopoDS::Face(m.faces(fid)), TopAbs_WIRE);
             wx.More(); wx.Next()) {
            ++wires;
        }
        std::printf("face %-4d %-10s edges %-3zu wires %d polys %d\n", fid,
                    weft::surfaceTypeName(fi.type), fi.edgeIds.size(),
                    wires, polyCount.count(fid) ? polyCount[fid] : 0);
    }
    return 0;
}
