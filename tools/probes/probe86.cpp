// probe86: classify the residual open/non-manifold edges under --stitch
// (cad profile). For each defect edge: position, owning face(s), nearest
// B-rep edge, that B-rep edge's owner count, and distance from the mesh
// edge midpoint to the B-rep curve — enough to sort each residual into
// the three suspect classes (3+-owner edges, off-curve borders, endpoint-
// only seams).
#include "weft/analysis.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"

#include <BRep_Tool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>

#include <cstdio>
#include <map>
#include <set>
#include <vector>

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    weft::Analysis a = weft::analyze(m);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.decoupleSeams = true;
    weft::GenerationReport rep;
    weft::PolyMesh mesh = weft::generate(m, a, gs, &rep);
    weft::ValidationReport vr = weft::validateMesh(mesh, &m);
    std::printf("open %zu nm %zu\n", vr.openEdges, vr.nonManifoldEdges);

    // Undirected edge -> owning polygons.
    std::map<std::pair<uint32_t, uint32_t>, std::vector<size_t>> use;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t x = poly[i], y = poly[(i + 1) % poly.size()];
            if (x > y) std::swap(x, y);
            use[{x, y}].push_back(p);
        }
    }
    // Pre-sample every B-rep edge's curve for nearest-curve attribution.
    struct CurveSamples {
        std::vector<gp_Pnt> pts;
        int owners = 0;
    };
    std::vector<CurveSamples> curves(m.edgeCount() + 1);
    for (int eid = 1; eid <= m.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(m.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        double cf, cl;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, cf, cl);
        if (c3.IsNull()) continue;
        for (int k = 0; k <= 64; ++k) {
            curves[eid].pts.push_back(c3->Value(cf + (cl - cf) * k / 64.0));
        }
        if (m.edgeToFaces.Contains(edge)) {
            curves[eid].owners = m.edgeToFaces.FindFromKey(edge).Extent();
        }
    }
    auto report = [&](const char* kind,
                      const std::pair<uint32_t, uint32_t>& e,
                      const std::vector<size_t>& polys) {
        const auto& A = mesh.vertices[e.first];
        const auto& B = mesh.vertices[e.second];
        gp_Pnt mid((A[0] + B[0]) / 2, (A[1] + B[1]) / 2, (A[2] + B[2]) / 2);
        double dx = A[0] - B[0], dy = A[1] - B[1], dz = A[2] - B[2];
        double len = std::sqrt(dx * dx + dy * dy + dz * dz);
        // Nearest B-rep edge by curve-sample distance to the midpoint.
        int bestEid = 0;
        double bestD = 1e300;
        for (int eid = 1; eid <= m.edgeCount(); ++eid) {
            for (const gp_Pnt& q : curves[eid].pts) {
                double d = q.Distance(mid);
                if (d < bestD) {
                    bestD = d;
                    bestEid = eid;
                }
            }
        }
        std::set<int> fids;
        for (size_t p : polys) fids.insert(mesh.polygonFaceId[p]);
        std::printf("%s v%u-v%u len=%.4g mid=(%.3f,%.3f,%.3f) faces=[", kind,
                    e.first, e.second, len, mid.X(), mid.Y(), mid.Z());
        for (int f : fids) std::printf("%d ", f);
        std::printf("] nearest brep edge %d dist=%.4g owners=%d\n", bestEid,
                    bestD, bestEid ? curves[bestEid].owners : -1);
        // Endpoint distances to that curve too.
        if (bestEid) {
            double dA = 1e300, dB = 1e300;
            for (const gp_Pnt& q : curves[bestEid].pts) {
                gp_Pnt pa(A[0], A[1], A[2]), pb(B[0], B[1], B[2]);
                dA = std::min(dA, q.Distance(pa));
                dB = std::min(dB, q.Distance(pb));
            }
            std::printf("    endpointsDist=%.4g/%.4g curveLen~%.4g\n", dA, dB,
                        curves[bestEid].pts.front().Distance(
                            curves[bestEid].pts.back()));
        }
    };
    for (const auto& [e, polys] : use) {
        if (polys.size() == 1) report("OPEN", e, polys);
    }
    for (const auto& [e, polys] : use) {
        if (polys.size() >= 3) report("NM  ", e, polys);
    }
    return 0;
}
