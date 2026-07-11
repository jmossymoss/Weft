// probe98: dump the boundary topology of a face for the ring-lattice
// design — wires, per-wire edge chains (edge id, length, solved divisions,
// pcurve uv extents), and the surface's uv box / closure flags. Target:
// the demo skirt faces (v-closed blend rings).
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    for (int i = 2; i < argc; ++i) {
        const int fid = std::atoi(argv[i]);
        const TopoDS_Face face = TopoDS::Face(m.faces(fid));
        BRepAdaptor_Surface surf(face);
        std::printf(
            "face %d type=%d u=[%.4g,%.4g] v=[%.4g,%.4g] uClosed=%d "
            "vClosed=%d uPer=%d vPer=%d\n",
            fid, (int)surf.GetType(), surf.FirstUParameter(),
            surf.LastUParameter(), surf.FirstVParameter(),
            surf.LastVParameter(), surf.IsUClosed() ? 1 : 0,
            surf.IsVClosed() ? 1 : 0, surf.IsUPeriodic() ? 1 : 0,
            surf.IsVPeriodic() ? 1 : 0);
        int w = 0;
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            std::printf("  wire %d:\n", w++);
            for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face);
                 we.More(); we.Next()) {
                const TopoDS_Edge e = we.Current();
                int eid = 0;
                if (m.edges.Contains(e)) eid = m.edges.FindIndex(e);
                double len = 0;
                double cf = 0, cl = 0;
                bool degen = BRep_Tool::Degenerated(e);
                if (!degen) {
                    Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, cf, cl);
                    if (!c3.IsNull()) {
                        GeomAdaptor_Curve gac(c3, cf, cl);
                        len = GCPnts_AbscissaPoint::Length(gac);
                    }
                }
                double pf = 0, pl = 0;
                Handle(Geom2d_Curve) pc =
                    BRep_Tool::CurveOnSurface(e, face, pf, pl);
                double u0 = 0, v0 = 0, u1 = 0, v1 = 0;
                if (!pc.IsNull()) {
                    gp_Pnt2d A = pc->Value(pf), B = pc->Value(pl);
                    u0 = A.X();
                    v0 = A.Y();
                    u1 = B.X();
                    v1 = B.Y();
                }
                auto it = rep.edgeDivisions.find(eid);
                std::printf(
                    "    e%d%s len=%.4g div=%d rev=%d uv=(%.4g,%.4g)->"
                    "(%.4g,%.4g)\n",
                    eid, degen ? " DEGEN" : "", len,
                    it == rep.edgeDivisions.end() ? -1 : it->second,
                    e.Orientation() == TopAbs_REVERSED ? 1 : 0, u0, v0, u1,
                    v1);
            }
        }
        auto bit = rep.faceBuild.find(fid);
        std::printf("  build=%d\n",
                    bit == rep.faceBuild.end() ? -99 : bit->second);
    }
    return 0;
}
