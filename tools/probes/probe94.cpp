// probe94: coons-planned faces ranked by area — the artist-visible
// patchwork. Reports fid, surface type, area, u/v spans.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cstdio>
#include <vector>
static const char* surfName(GeomAbs_SurfaceType t) {
    switch (t) {
        case GeomAbs_Plane: return "plane";
        case GeomAbs_Cylinder: return "cylinder";
        case GeomAbs_Cone: return "cone";
        case GeomAbs_Sphere: return "sphere";
        case GeomAbs_Torus: return "torus";
        case GeomAbs_BSplineSurface: return "bspline";
        case GeomAbs_SurfaceOfRevolution: return "revolution";
        case GeomAbs_SurfaceOfExtrusion: return "extrusion";
        case GeomAbs_OffsetSurface: return "offset";
        default: return "other";
    }
}
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    struct Row { int fid; double area; };
    std::vector<Row> rows;
    for (const auto& [fid, kind] : rep.faceMesher) {
        if (kind != weft::MesherKind::CoonsGrid) continue;
        GProp_GProps props;
        BRepGProp::SurfaceProperties(TopoDS::Face(m.faces(fid)), props);
        rows.push_back({fid, props.Mass()});
    }
    std::sort(rows.begin(), rows.end(),
              [](const Row& x, const Row& y) { return x.area > y.area; });
    std::printf("%zu coons faces; top by area:\n", rows.size());
    for (size_t i = 0; i < rows.size() && i < 15; ++i) {
        const TopoDS_Face f = TopoDS::Face(m.faces(rows[i].fid));
        BRepAdaptor_Surface sa(f);
        std::printf("  face %d: %s area %.1f u=[%.2f,%.2f] v=[%.2f,%.2f]\n",
                    rows[i].fid, surfName(sa.GetType()), rows[i].area,
                    sa.FirstUParameter(), sa.LastUParameter(),
                    sa.FirstVParameter(), sa.LastVParameter());
    }
    return 0;
}
