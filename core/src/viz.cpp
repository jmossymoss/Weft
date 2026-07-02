#include "weft/viz.hpp"

#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>

namespace weft {

std::vector<EdgePolyline> sampleEdges(const Model& model, int segmentsPerEdge,
                                      const std::map<int, int>& perEdge) {
    std::vector<EdgePolyline> out;
    out.reserve(model.edgeCount());
    const int fallback = segmentsPerEdge < 1 ? 1 : segmentsPerEdge;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        double f = 0, l = 0;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
        if (curve.IsNull()) continue;
        auto it = perEdge.find(eid);
        const int n = it != perEdge.end() && it->second >= 1 ? it->second
                                                             : fallback;
        EdgePolyline line;
        line.edgeId = eid;
        line.points.reserve(n + 1);
        for (int i = 0; i <= n; ++i) {
            gp_Pnt p = curve->Value(f + (l - f) * double(i) / n);
            line.points.push_back({p.X(), p.Y(), p.Z()});
        }
        out.push_back(std::move(line));
    }
    return out;
}

std::vector<EdgePolyline> sampleEdges(const Model& model, int segmentsPerEdge) {
    return sampleEdges(model, segmentsPerEdge, {});
}

}  // namespace weft
