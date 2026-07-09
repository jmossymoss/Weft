// Characterize a model's open-shell B-rep edges: for every edge bordering
// fewer than two faces, report its length and the closest approach to every
// other open-shell edge — are the "holes" really near-coincident seam pairs
// a sewing pass could close, or genuine unpaired boundary?
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    struct Open {
        int eid;
        std::vector<gp_Pnt> pts;
        double len;
    };
    std::vector<Open> open;
    for (int e = 1; e <= m.edges.Extent(); ++e) {
        const TopoDS_Edge& edge = TopoDS::Edge(m.edges(e));
        int nf = 0;
        if (m.edgeToFaces.Contains(edge)) {
            nf = m.edgeToFaces.FindFromKey(edge).Extent();
        }
        if (nf >= 2) continue;
        if (BRep_Tool::Degenerated(edge)) continue;
        Open o;
        o.eid = e;
        BRepAdaptor_Curve c(edge);
        o.len = GCPnts_AbscissaPoint::Length(c);
        const double f = c.FirstParameter(), l = c.LastParameter();
        for (int i = 0; i <= 32; ++i) {
            o.pts.push_back(c.Value(f + (l - f) * i / 32.0));
        }
        open.push_back(std::move(o));
        std::printf("open edge #%d  faces=%d  len=%.4f\n", e, nf, o.len);
    }
    std::printf("total open-shell edges: %zu\n", open.size());
    // Closest approach between each pair (sampled, both directions).
    for (size_t a = 0; a < open.size(); ++a) {
        double best = 1e300;
        int bestEid = -1;
        for (size_t b = 0; b < open.size(); ++b) {
            if (a == b) continue;
            double worst = 0;  // Hausdorff-ish: max over a's samples of min dist to b
            for (const gp_Pnt& p : open[a].pts) {
                double mn = 1e300;
                for (const gp_Pnt& q : open[b].pts) {
                    mn = std::min(mn, p.Distance(q));
                }
                worst = std::max(worst, mn);
            }
            if (worst < best) {
                best = worst;
                bestEid = open[b].eid;
            }
        }
        std::printf("edge #%d len=%.4f: best partner #%d hausdorff=%.6f\n",
                    open[a].eid, open[a].len, bestEid, best);
    }
    return 0;
}
