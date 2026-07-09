// Compare the RAW STEP shape's open-shell edges against the healed model's:
// does import healing open the shell, or does the source arrive open?
// Also chain the healed open edges into boundary loops by shared endpoints.
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <cstdio>
#include <map>
#include <vector>

static void reportOpen(const TopoDS_Shape& shape, const char* label) {
    TopTools_IndexedDataMapOfShapeListOfShape e2f;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, e2f);
    int open = 0;
    std::vector<TopoDS_Edge> openEdges;
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
        if (BRep_Tool::Degenerated(e)) continue;
        if (e2f(i).Extent() < 2) {
            ++open;
            openEdges.push_back(e);
        }
    }
    std::printf("%s: %d open-shell edges\n", label, open);
    // Chain into loops by endpoint proximity.
    struct End { gp_Pnt a, b; bool used = false; double len; };
    std::vector<End> ends;
    for (const TopoDS_Edge& e : openEdges) {
        BRepAdaptor_Curve c(e);
        End en;
        en.a = c.Value(c.FirstParameter());
        en.b = c.Value(c.LastParameter());
        en.len = GCPnts_AbscissaPoint::Length(c);
        ends.push_back(en);
    }
    const double tol = 1e-3;
    for (size_t s = 0; s < ends.size(); ++s) {
        if (ends[s].used) continue;
        ends[s].used = true;
        gp_Pnt start = ends[s].a, cur = ends[s].b;
        int n = 1;
        double len = ends[s].len;
        bool closed = false;
        for (int guard = 0; guard < 64 && !closed; ++guard) {
            if (cur.Distance(start) < tol && n > 1) { closed = true; break; }
            bool advanced = false;
            for (size_t j = 0; j < ends.size(); ++j) {
                if (ends[j].used) continue;
                if (cur.Distance(ends[j].a) < tol) {
                    cur = ends[j].b; ends[j].used = true; ++n; len += ends[j].len;
                    advanced = true; break;
                }
                if (cur.Distance(ends[j].b) < tol) {
                    cur = ends[j].a; ends[j].used = true; ++n; len += ends[j].len;
                    advanced = true; break;
                }
            }
            if (!advanced) break;
            if (cur.Distance(start) < tol) closed = true;
        }
        std::printf("  boundary loop: %d edges, total len %.3f, %s\n", n, len,
                    closed ? "CLOSED" : "OPEN CHAIN (dangling!)");
    }
}

int main(int argc, char** argv) {
    STEPControl_Reader rd;
    rd.ReadFile(argv[1]);
    rd.TransferRoots();
    TopoDS_Shape raw = rd.OneShape();
    reportOpen(raw, "RAW step shape");
    weft::Model m = weft::loadStep(argv[1]);
    reportOpen(m.shape, "healed model");
    return 0;
}
