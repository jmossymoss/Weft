// Per-poly winding vs oriented surface normal for chosen faces (surface
// evaluated directly, no anchors needed — works for border-only polys).
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <set>
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.adaptive = true;
    gs.defaults.quadDominant = std::atoi(argv[2]) != 0;
    weft::PolyMesh mesh = weft::generate(m, a, gs);
    std::set<int> want;
    for (int i = 3; i < argc; ++i) want.insert(std::atoi(argv[i]));
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        int fid = mesh.polygonFaceId[p];
        if (!want.count(fid)) continue;
        const auto& poly = mesh.polygons[p];
        double nx = 0, ny = 0, nz = 0;
        for (size_t i = 0; i < poly.size(); ++i) {
            const auto& A = mesh.vertices[poly[i]];
            const auto& B = mesh.vertices[poly[(i + 1) % poly.size()]];
            nx += (A[1] - B[1]) * (A[2] + B[2]);
            ny += (A[2] - B[2]) * (A[0] + B[0]);
            nz += (A[0] - B[0]) * (A[1] + B[1]);
        }
        TopoDS_Face face = TopoDS::Face(m.faces(fid));
        BRepAdaptor_Surface s(face);
        gp_Pnt sp;
        gp_Vec du, dv;
        s.D1((s.FirstUParameter() + s.LastUParameter()) / 2,
             (s.FirstVParameter() + s.LastVParameter()) / 2, sp, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
        double dot = n.X() * nx + n.Y() * ny + n.Z() * nz;
        std::printf("face %d poly %zu (%zuv): dot=%+.4g %s\n", fid, p,
                    poly.size(), dot, dot < 0 ? "FLIPPED" : "ok");
    }
    return 0;
}
