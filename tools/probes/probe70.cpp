// Dump the B-rep neighbourhood of a 3D segment: every edge with an endpoint
// near one of the two probe points, its faces, length, and endpoints; and
// every face wire that contains any of those edges (edge sequence).
#include "weft/model.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <cstdio>
#include <set>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    gp_Pnt A(atof(argv[2]), atof(argv[3]), atof(argv[4]));
    gp_Pnt B(atof(argv[5]), atof(argv[6]), atof(argv[7]));
    const double tol = 0.05;
    std::set<int> hit;
    for (int e = 1; e <= m.edges.Extent(); ++e) {
        const TopoDS_Edge& edge = TopoDS::Edge(m.edges(e));
        if (BRep_Tool::Degenerated(edge)) continue;
        BRepAdaptor_Curve c(edge);
        gp_Pnt p0 = c.Value(c.FirstParameter());
        gp_Pnt p1 = c.Value(c.LastParameter());
        const bool nearA = p0.Distance(A) < tol || p1.Distance(A) < tol;
        const bool nearB = p0.Distance(B) < tol || p1.Distance(B) < tol;
        if (!nearA && !nearB) continue;
        hit.insert(e);
        std::printf(
            "edge #%d len %.4f  (%.4f,%.4f,%.4f)-(%.4f,%.4f,%.4f) faces:", e,
            GCPnts_AbscissaPoint::Length(c), p0.X(), p0.Y(), p0.Z(), p1.X(),
            p1.Y(), p1.Z());
        if (m.edgeToFaces.Contains(edge)) {
            for (const TopoDS_Shape& f : m.edgeToFaces.FindFromKey(edge)) {
                std::printf(" %d", m.faces.FindIndex(f));
            }
        }
        std::printf("\n");
    }
    // Wires of face 275 (and any face bordering a hit edge) for context.
    std::set<int> facesToDump;
    for (int e : hit) {
        const TopoDS_Edge& edge = TopoDS::Edge(m.edges(e));
        if (m.edgeToFaces.Contains(edge)) {
            for (const TopoDS_Shape& f : m.edgeToFaces.FindFromKey(edge)) {
                facesToDump.insert(m.faces.FindIndex(f));
            }
        }
    }
    for (int fid : facesToDump) {
        const TopoDS_Face& face = TopoDS::Face(m.faces(fid));
        std::printf("face #%d wires:\n", fid);
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            std::printf("  wire:");
            for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face);
                 we.More(); we.Next()) {
                std::printf(" %d%s", m.edges.FindIndex(we.Current()),
                            we.Current().Orientation() == TopAbs_REVERSED
                                ? "r"
                                : "");
            }
            std::printf("\n");
        }
    }
    return 0;
}
