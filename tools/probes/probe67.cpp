// Does STEP translation drop faces from foam's closed solids? Count faces
// in the file's shells vs the transferred shape, and print transfer checks.
// Then find which faces border the open loops and whether a bigger sewing
// tolerance closes the shell.
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_PrintCount.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <cstdio>

static int openCount(const TopoDS_Shape& shape) {
    TopTools_IndexedDataMapOfShapeListOfShape e2f;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, e2f);
    int open = 0;
    for (int i = 1; i <= e2f.Extent(); ++i) {
        const TopoDS_Edge& e = TopoDS::Edge(e2f.FindKey(i));
        if (BRep_Tool::Degenerated(e)) continue;
        if (e2f(i).Extent() < 2) ++open;
    }
    return open;
}

int main(int argc, char** argv) {
    STEPControl_Reader rd;
    rd.ReadFile(argv[1]);
    rd.TransferRoots();
    TopoDS_Shape raw = rd.OneShape();
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(raw, TopAbs_FACE, faces);
    std::printf("transferred faces: %d\n", faces.Extent());
    std::printf("check-transfer messages:\n");
    rd.PrintCheckTransfer(Standard_True, IFSelect_CountByItem);
    std::printf("raw opens: %d\n", openCount(raw));
    for (double tol : {1e-4, 1e-3, 1e-2, 0.05, 0.1}) {
        BRepBuilderAPI_Sewing sew(tol);
        sew.Add(raw);
        sew.Perform();
        TopoDS_Shape s = sew.SewedShape();
        std::printf("sewn at %.4f: opens %d\n", tol,
                    s.IsNull() ? -1 : openCount(s));
    }
    return 0;
}
