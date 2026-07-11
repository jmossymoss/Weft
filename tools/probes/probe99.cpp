// probe99: density census for the radius-scaling work — per face: surface
// type, mesher kind, solved counts; per edge: curve type, length, radius
// (from mid-parameter curvature), solved divisions, owner face kinds.
// Sorted views make the "big ring coarse / small ring dense" inversion
// jump out.
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>

#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);

    std::printf("== edges ==\n");
    for (int eid = 1; eid <= m.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(m.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        double f, l;
        if (BRep_Tool::Curve(edge, f, l).IsNull()) continue;
        BRepAdaptor_Curve c(edge);
        const double len = GCPnts_AbscissaPoint::Length(c);
        // Radius from curvature at three params (min/mid over samples).
        double kmin = 1e300, kmax = 0.0;
        for (int i = 0; i < 5; ++i) {
            const double t =
                c.FirstParameter() +
                (c.LastParameter() - c.FirstParameter()) * i / 4.0;
            gp_Pnt P;
            gp_Vec D1, D2;
            try {
                c.D2(t, P, D1, D2);
            } catch (...) {
                continue;
            }
            const double d1 = D1.Magnitude();
            if (d1 < 1e-9) continue;
            const double k = D1.Crossed(D2).Magnitude() / (d1 * d1 * d1);
            kmin = std::min(kmin, k);
            kmax = std::max(kmax, k);
        }
        const bool closed =
            c.Value(c.FirstParameter()).Distance(c.Value(c.LastParameter())) <
            1e-9;
        auto it = rep.edgeDivisions.find(eid);
        const int div = it == rep.edgeDivisions.end() ? -1 : it->second;
        const char* ct = "other";
        switch (c.GetType()) {
            case GeomAbs_Line: ct = "line"; break;
            case GeomAbs_Circle: ct = "circle"; break;
            case GeomAbs_Ellipse: ct = "ellipse"; break;
            case GeomAbs_BSplineCurve: ct = "bspline"; break;
            case GeomAbs_BezierCurve: ct = "bezier"; break;
            default: break;
        }
        std::printf(
            "e%-4d %-8s len=%8.3f r=%8.3f..%-8.3f closed=%d div=%-3d "
            "pitch=%7.3f owners:",
            eid, ct, len, kmax > 1e-9 ? 1.0 / kmax : 0.0,
            kmin < 1e299 && kmin > 1e-9 ? 1.0 / kmin : 0.0, closed ? 1 : 0,
            div, div > 0 ? len / div : 0.0);
        if (m.edgeToFaces.Contains(edge)) {
            for (const TopoDS_Shape& fs : m.edgeToFaces.FindFromKey(edge)) {
                const int fid = m.faces.FindIndex(fs);
                const auto kit = rep.faceMesher.find(fid);
                std::printf(" f%d(%s)", fid,
                            kit == rep.faceMesher.end()
                                ? "?"
                                : weft::mesherKindName(kit->second));
            }
        }
        std::printf("\n");
    }
    std::printf("== faces ==\n");
    for (int fid = 1; fid <= m.faceCount(); ++fid) {
        BRepAdaptor_Surface surf(TopoDS::Face(m.faces(fid)));
        const char* st = "other";
        switch (surf.GetType()) {
            case GeomAbs_Plane: st = "plane"; break;
            case GeomAbs_Cylinder: st = "cyl"; break;
            case GeomAbs_Cone: st = "cone"; break;
            case GeomAbs_Sphere: st = "sph"; break;
            case GeomAbs_Torus: st = "torus"; break;
            case GeomAbs_BSplineSurface: st = "bspl"; break;
            default: break;
        }
        const auto kit = rep.faceMesher.find(fid);
        const auto cit = rep.faceCounts.find(fid);
        std::printf("f%-4d %-6s %-16s counts=%d,%d", fid, st,
                    kit == rep.faceMesher.end()
                        ? "?"
                        : weft::mesherKindName(kit->second),
                    cit == rep.faceCounts.end() ? -1 : cit->second[0],
                    cit == rep.faceCounts.end() ? -1 : cit->second[1]);
        if (surf.GetType() == GeomAbs_Cylinder) {
            std::printf(" R=%.3f", surf.Cylinder().Radius());
        }
        if (surf.GetType() == GeomAbs_Torus) {
            std::printf(" R=%.3f r=%.3f", surf.Torus().MajorRadius(),
                        surf.Torus().MinorRadius());
        }
        std::printf("\n");
    }
    return 0;
}
