// Per-shell boundary fraction: edges used once within the shell vs total,
// to calibrate the "nearly-closed shell" capping gate.
#include "weft/model.hpp"
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <cstdio>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    int shellId = 0;
    for (TopExp_Explorer sx(m.shape, TopAbs_SHELL); sx.More(); sx.Next()) {
        ++shellId;
        TopTools_IndexedDataMapOfShapeListOfShape e2f;
        TopExp::MapShapesAndAncestors(sx.Current(), TopAbs_EDGE, TopAbs_FACE,
                                      e2f);
        int open = 0, total = 0;
        for (int i = 1; i <= e2f.Extent(); ++i) {
            const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
            if (BRep_Tool::Degenerated(e)) continue;
            ++total;
            if (e2f(i).Extent() < 2) ++open;
        }
        int faces = 0;
        for (TopExp_Explorer fx(sx.Current(), TopAbs_FACE); fx.More();
             fx.Next()) {
            ++faces;
        }
        if (open) {
            std::printf("shell %d: %d faces, %d/%d open (%.1f%%)\n", shellId,
                        faces, open, total, 100.0 * open / total);
        }
    }
    return 0;
}
