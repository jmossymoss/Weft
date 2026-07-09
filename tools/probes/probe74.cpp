// Are a model's open-shell edges inside TopAbs_SOLIDs (broken solid ->
// cap) or free shells (authored sheet -> leave alone)?
#include "weft/model.hpp"
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <cstdio>
#include <set>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    std::set<int> solidFaces;
    for (TopExp_Explorer sx(m.shape, TopAbs_SOLID); sx.More(); sx.Next()) {
        for (TopExp_Explorer fx(sx.Current(), TopAbs_FACE); fx.More();
             fx.Next()) {
            solidFaces.insert(m.faces.FindIndex(fx.Current()));
        }
    }
    int inSolid = 0, inSheet = 0;
    for (int e = 1; e <= m.edges.Extent(); ++e) {
        const TopoDS_Edge& edge = TopoDS::Edge(m.edges(e));
        if (BRep_Tool::Degenerated(edge)) continue;
        int nf = 0;
        bool solid = false;
        if (m.edgeToFaces.Contains(edge)) {
            for (const TopoDS_Shape& f : m.edgeToFaces.FindFromKey(edge)) {
                ++nf;
                if (solidFaces.count(m.faces.FindIndex(f))) solid = true;
            }
        }
        if (nf >= 2) continue;
        (solid ? inSolid : inSheet)++;
    }
    std::printf("%s: open edges in solids %d, in free shells %d\n", argv[1],
                inSolid, inSheet);
    return 0;
}
