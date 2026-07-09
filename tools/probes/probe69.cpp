// Find "twin" B-rep edge pairs: distinct edges that are geometrically
// near-coincident (sampled Hausdorff below a tolerance). These are the
// zero-width slit / imprint artifacts that weld into non-manifold mesh
// edges. Report the pair, the gap, their lengths, and adjacent faces.
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    const double tol = argc > 2 ? atof(argv[2]) : 0.01;
    struct E {
        int eid;
        std::vector<gp_Pnt> pts;
        double len;
        gp_Pnt mid;
    };
    std::vector<E> es;
    for (int e = 1; e <= m.edges.Extent(); ++e) {
        const TopoDS_Edge& edge = TopoDS::Edge(m.edges(e));
        if (BRep_Tool::Degenerated(edge)) continue;
        E rec;
        rec.eid = e;
        BRepAdaptor_Curve c(edge);
        rec.len = GCPnts_AbscissaPoint::Length(c);
        if (rec.len < 1e-9) continue;
        const double f = c.FirstParameter(), l = c.LastParameter();
        for (int i = 0; i <= 16; ++i)
            rec.pts.push_back(c.Value(f + (l - f) * i / 16.0));
        rec.mid = rec.pts[8];
        es.push_back(std::move(rec));
    }
    int found = 0;
    for (size_t a = 0; a < es.size(); ++a) {
        for (size_t b = a + 1; b < es.size(); ++b) {
            if (std::abs(es[a].len - es[b].len) > tol * 4) continue;
            if (es[a].mid.Distance(es[b].mid) > tol * 4) continue;
            double worst = 0;
            // forward or reverse pairing, take the better
            double worstR = 0;
            for (int i = 0; i <= 16; ++i) {
                worst = std::max(worst, es[a].pts[i].Distance(es[b].pts[i]));
                worstR = std::max(worstR,
                                  es[a].pts[i].Distance(es[b].pts[16 - i]));
            }
            const double gap = std::min(worst, worstR);
            if (gap > tol) continue;
            ++found;
            std::printf("twin edges #%d/#%d len %.4f gap %.6f faces", es[a].eid,
                        es[b].eid, es[a].len, gap);
            for (int eid : {es[a].eid, es[b].eid}) {
                std::printf(" [");
                const TopoDS_Edge& edge = TopoDS::Edge(m.edges(eid));
                if (m.edgeToFaces.Contains(edge)) {
                    for (const TopoDS_Shape& f :
                         m.edgeToFaces.FindFromKey(edge)) {
                        std::printf(" %d", m.faces.FindIndex(f));
                    }
                }
                std::printf(" ]");
            }
            std::printf("\n");
        }
    }
    std::printf("total twin pairs at tol %.4f: %d\n", tol, found);
    return 0;
}
