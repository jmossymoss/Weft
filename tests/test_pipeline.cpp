// End-to-end test for the phase-0 loop: fixture STEP → import → analyze →
// generate with exact division controls → OBJ. No test framework; each CHECK
// prints and exits non-zero on failure so CTest reports it.

#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                 \
        }                                                               \
    } while (0)

#define CHECK_EQ(a, b)                                                        \
    do {                                                                      \
        auto va = (a);                                                        \
        auto vb = (b);                                                        \
        if (!(va == vb)) {                                                    \
            std::printf("FAIL %s:%d: %s == %s (%lld vs %lld)\n", __FILE__,    \
                        __LINE__, #a, #b, (long long)va, (long long)vb);      \
            ++failures;                                                       \
        }                                                                     \
    } while (0)

namespace {

std::string tmpPath(const std::string& name) {
    const char* dir = std::getenv("TMPDIR");
    return std::string(dir ? dir : "/tmp") + "/" + name;
}

// Every mesh edge of a closed solid must be used by exactly two polygons,
// once in each direction (consistent winding, watertight).
bool isWatertight(const weft::PolyMesh& mesh) {
    std::map<std::pair<uint32_t, uint32_t>, int> directed;
    for (const auto& poly : mesh.polygons) {
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t a = poly[i];
            uint32_t b = poly[(i + 1) % poly.size()];
            if (a == b) return false;
            ++directed[{a, b}];
        }
    }
    for (const auto& [edge, count] : directed) {
        if (count != 1) return false;
        auto rev = directed.find({edge.second, edge.first});
        if (rev == directed.end() || rev->second != 1) return false;
    }
    return true;
}

void testCylinder() {
    std::printf("-- cylinder --\n");
    std::string stepPath = tmpPath("weft_test_cylinder.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    CHECK_EQ(model.faceCount(), 3);  // side + two caps

    weft::Analysis a = weft::analyze(model);
    int cylinders = 0, planes = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) {
            ++cylinders;
            CHECK(std::abs(f.radius - 10.0) < 1e-9);
            CHECK_EQ(f.neighborFaceIds.size(), 2);  // both caps
        }
        if (f.type == weft::SurfaceType::Plane) ++planes;
    }
    CHECK_EQ(cylinders, 1);
    CHECK_EQ(planes, 2);

    // Cap/side edges of a solid cylinder are convex ~90 degrees.
    for (const auto& e : a.edges) {
        if (e.faceIds.size() == 2 && e.dihedralDeg > 1.0) {
            CHECK(e.convexity == weft::EdgeConvexity::Convex);
            CHECK(std::abs(e.dihedralDeg - 90.0) < 1.0);
        }
    }

    // Exact division control: 12 radial, 3 axial, n-gon caps.
    weft::GenerationSettings gs;
    gs.defaults.radial = 12;
    gs.defaults.axial = 3;
    gs.defaults.cap = weft::CapStyle::NGon;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    int revolutionGrids = 0, diskCaps = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::RevolutionGrid) ++revolutionGrids;
        if (kind == weft::MesherKind::DiskCap) ++diskCaps;
    }
    CHECK_EQ(revolutionGrids, 1);
    CHECK_EQ(diskCaps, 2);

    CHECK_EQ(mesh.countQuads(), 12 * 3);
    CHECK_EQ(mesh.countNgons(), 2);   // the two 12-gon caps
    CHECK_EQ(mesh.countTris(), 0);
    // 12 columns x 4 rings; cap rings weld onto the side's boundary rings.
    CHECK_EQ(mesh.vertexCount(), 12 * 4);
    CHECK(isWatertight(mesh));

    // Fan caps: welded apex per cap, 12 tris each.
    gs.defaults.cap = weft::CapStyle::Fan;
    weft::PolyMesh fanMesh = weft::generate(model, a, gs);
    CHECK_EQ(fanMesh.countTris(), 2 * 12);
    CHECK_EQ(fanMesh.vertexCount(), 12 * 4 + 2);
    CHECK(isWatertight(fanMesh));

    // Per-face override: crank only the side face's radial count. Density
    // matching must propagate 24 to the shared circle edges, so the caps
    // become 24-gons and the solid stays watertight.
    int sideFaceId = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) sideFaceId = f.id;
    }
    weft::GenerationSettings gsOverride;
    gsOverride.defaults.radial = 12;
    gsOverride.defaults.axial = 3;
    weft::FaceMeshSettings side = gsOverride.defaults;
    side.radial = 24;
    side.axial = 2;
    gsOverride.perFace[sideFaceId] = side;
    weft::PolyMesh overrideMesh = weft::generate(model, a, gsOverride);
    CHECK_EQ(overrideMesh.countQuads(), 24 * 2);
    for (const auto& poly : overrideMesh.polygons) {
        if (poly.size() > 4) CHECK_EQ(poly.size(), 24);  // caps followed
    }
    CHECK(isWatertight(overrideMesh));

    // Per-edge pin: force one circle edge to 20; the whole matched group
    // (side ring + both caps) must follow.
    weft::GenerationSettings gsEdge;
    gsEdge.defaults.radial = 12;
    gsEdge.defaults.axial = 2;
    int circleEdgeId = 0;
    for (const auto& e : a.edges) {
        if (e.faceIds.size() == 2) circleEdgeId = e.id;
    }
    gsEdge.perEdge[circleEdgeId] = 20;
    weft::GenerationReport edgeReport;
    weft::PolyMesh pinnedMesh = weft::generate(model, a, gsEdge, &edgeReport);
    CHECK_EQ(pinnedMesh.countQuads(), 20 * 2);
    CHECK_EQ(edgeReport.edgeDivisions[circleEdgeId], 20);
    CHECK(isWatertight(pinnedMesh));

    std::string objPath = tmpPath("weft_test_cylinder.obj");
    weft::writeObj(mesh, objPath);
    std::ifstream obj(objPath);
    CHECK(obj.good());
    int vLines = 0, fLines = 0, gLines = 0;
    for (std::string line; std::getline(obj, line);) {
        if (line.rfind("v ", 0) == 0) ++vLines;
        if (line.rfind("f ", 0) == 0) ++fLines;
        if (line.rfind("g ", 0) == 0) ++gLines;
    }
    CHECK_EQ(vLines, (int)mesh.vertexCount());
    CHECK_EQ(fLines, (int)mesh.polygonCount());
    CHECK_EQ(gLines, 3);  // one group per B-rep face
}

void testBox() {
    std::printf("-- box --\n");
    std::string stepPath = tmpPath("weft_test_box.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    CHECK_EQ(model.faceCount(), 6);
    CHECK_EQ(model.edgeCount(), 12);

    weft::Analysis a = weft::analyze(model);
    for (const auto& f : a.faces) {
        CHECK(f.type == weft::SurfaceType::Plane);
        CHECK_EQ(f.neighborFaceIds.size(), 4);
    }
    for (const auto& e : a.edges) {
        CHECK(e.convexity == weft::EdgeConvexity::Convex);
        CHECK(std::abs(e.dihedralDeg - 90.0) < 1e-6);
    }

    weft::GenerationSettings gs;
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    for (const auto& [fid, kind] : report.faceMesher) {
        CHECK(kind == weft::MesherKind::PlanarGrid);
    }
    CHECK_EQ(mesh.countQuads(), 6 * 3 * 3);
    CHECK_EQ(mesh.countTris(), 0);
    // 3x3 grid per face welds into the classic (n+1)^3 - (n-1)^3 shell count.
    CHECK_EQ(mesh.vertexCount(), 4 * 4 * 4 - 2 * 2 * 2);
    CHECK(isWatertight(mesh));
}

void testCone() {
    std::printf("-- cone --\n");
    std::string stepPath = tmpPath("weft_test_cone.step");
    weft::writeStep(weft::makeFixture("cone"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.radial = 12;
    gs.defaults.axial = 3;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    // Side: 12 x 3 grid whose apex row collapses -> 2 quad bands + 12 tris.
    // Base: one 12-gon cap.
    CHECK_EQ(mesh.countQuads(), 12 * 2);
    CHECK_EQ(mesh.countTris(), 12);
    CHECK_EQ(mesh.countNgons(), 1);
    CHECK_EQ(mesh.vertexCount(), 12 * 3 + 1);  // 3 rings + apex
    CHECK(isWatertight(mesh));
}

void testSphere() {
    std::printf("-- sphere --\n");
    std::string stepPath = tmpPath("weft_test_sphere.step");
    weft::writeStep(weft::makeFixture("sphere"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.radial = 16;
    gs.defaults.axial = 6;
    weft::PolyMesh mesh = weft::generate(model, a, gs);

    // 6 latitude bands: 4 quad bands + 2 pole fans of 16 tris.
    CHECK_EQ(mesh.countQuads(), 16 * 4);
    CHECK_EQ(mesh.countTris(), 2 * 16);
    CHECK_EQ(mesh.vertexCount(), 16 * 5 + 2);  // 5 rings + 2 poles
    CHECK(isWatertight(mesh));
}

void testTorus() {
    std::printf("-- torus --\n");
    std::string stepPath = tmpPath("weft_test_torus.step");
    weft::writeStep(weft::makeFixture("torus"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.radial = 24;
    gs.defaults.axial = 8;
    weft::PolyMesh mesh = weft::generate(model, a, gs);

    // Doubly periodic: pure quads, no poles, wrap in both directions.
    CHECK_EQ(mesh.countQuads(), 24 * 8);
    CHECK_EQ(mesh.countTris(), 0);
    CHECK_EQ(mesh.vertexCount(), 24 * 8);
    CHECK(isWatertight(mesh));
}

void testBoxDensityMatching() {
    std::printf("-- box density matching --\n");
    std::string stepPath = tmpPath("weft_test_box_density.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    // One face asks for a denser grid; the shared-edge groups must drag the
    // neighbouring faces along so the box stays watertight.
    weft::GenerationSettings gs;
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    weft::FaceMeshSettings dense = gs.defaults;
    dense.gridU = 5;
    gs.perFace[1] = dense;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    CHECK(isWatertight(mesh));
    CHECK(mesh.countQuads() > 6 * 3 * 3);  // denser than the uniform box
    CHECK_EQ(mesh.countTris(), 0);

    // The solved counts must be internally consistent: every planar-grid
    // face's polygon count equals the product of its two edge-group counts.
    std::map<int, int> polysPerFace;
    // polygons are contiguous per face; count per FaceId
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        ++polysPerFace[mesh.polygonFaceId[p]];
    }
    for (const auto& f : a.faces) {
        int nu = 0, nv = 0;
        for (int eid : f.edgeIds) {
            auto it = report.edgeDivisions.find(eid);
            if (it == report.edgeDivisions.end()) continue;
            if (nu == 0) nu = it->second;
            else if (it->second != nu && nv == 0) nv = it->second;
        }
        if (nv == 0) nv = nu;  // all four edges solved to the same count
        CHECK_EQ(polysPerFace[f.id], nu * nv);
    }
}

void testFillet() {
    std::printf("-- fillet detection + support loops --\n");
    std::string stepPath = tmpPath("weft_test_fillet.step");
    weft::writeStep(weft::makeFixture("fillet"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    CHECK_EQ(model.faceCount(), 7);  // 6 box faces (2 shrunk, 2 notched) + strip

    // Exactly one fillet face: the quarter-cylinder strip, r=4, joined to
    // its two planar neighbours by tangent-smooth edges.
    int filletFaceId = 0, filletCount = 0, smoothEdges = 0;
    for (const auto& f : a.faces) {
        if (f.isFillet) {
            ++filletCount;
            filletFaceId = f.id;
            CHECK(f.type == weft::SurfaceType::Cylinder);
            CHECK(std::abs(f.radius - 4.0) < 1e-9);
        }
    }
    for (const auto& e : a.edges) {
        if (e.convexity == weft::EdgeConvexity::Smooth) ++smoothEdges;
    }
    CHECK_EQ(filletCount, 1);
    CHECK_EQ(smoothEdges, 2);

    // 5 support loops across the blend, density-matched 4 along its length.
    weft::GenerationSettings gs;
    gs.defaults.gridU = 4;
    gs.defaults.gridV = 4;
    gs.defaults.filletLoops = 5;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    CHECK(report.faceMesher[filletFaceId] == weft::MesherKind::PlanarGrid);
    std::map<int, int> polysPerFace;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        ++polysPerFace[mesh.polygonFaceId[p]];
    }
    CHECK_EQ(polysPerFace[filletFaceId], 5 * 4);

    // Hold clustering: same counts, but the loops crowd toward the creases —
    // the first across-interval must shrink vs the uniform mesh.
    weft::GenerationSettings gsHold = gs;
    gsHold.defaults.filletHold = 0.8;
    weft::PolyMesh held = weft::generate(model, a, gsHold);
    CHECK_EQ(held.vertexCount(), mesh.vertexCount());
    CHECK_EQ(held.polygonCount(), mesh.polygonCount());

    auto t0 = weft::clusteredParams(5, 0.0);
    auto t1 = weft::clusteredParams(5, 0.8);
    CHECK(t1[1] - t1[0] < 0.5 * (t0[1] - t0[0]));           // tight at crease
    CHECK(t1[3] - t1[2] > (t0[3] - t0[2]));                 // loose mid-span
    for (size_t i = 1; i < t1.size(); ++i) CHECK(t1[i] > t1[i - 1]);
    CHECK(t1.front() == 0.0);
    CHECK(t1.back() == 1.0);
}

void testRecipeRoundTrip() {
    std::printf("-- recipe round trip --\n");
    weft::GenerationSettings gs;
    gs.defaults.radial = 20;
    gs.defaults.cap = weft::CapStyle::Fan;
    weft::FaceMeshSettings dense = gs.defaults;
    dense.radial = 40;
    dense.gridV = 7;
    gs.perFace[3] = dense;
    gs.perEdge[5] = 13;

    std::string path = tmpPath("weft_test.recipe");
    weft::saveRecipe(gs, path);
    weft::GenerationSettings loaded = weft::loadRecipe(path);

    CHECK_EQ(loaded.defaults.radial, 20);
    CHECK(loaded.defaults.cap == weft::CapStyle::Fan);
    CHECK_EQ(loaded.perFace.size(), 1);
    CHECK_EQ(loaded.perFace[3].radial, 40);
    CHECK_EQ(loaded.perFace[3].gridV, 7);
    CHECK_EQ(loaded.perEdge[5], 13);

    // Same recipe, same B-rep => identical topology (regenerability is the
    // point of persisting decisions instead of meshes).
    std::string stepPath = tmpPath("weft_test_recipe_cyl.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    weft::PolyMesh m1 = weft::generate(model, a, gs);
    weft::PolyMesh m2 = weft::generate(model, a, loaded);
    CHECK_EQ(m1.vertexCount(), m2.vertexCount());
    CHECK_EQ(m1.polygonCount(), m2.polygonCount());
}

void testBoss() {
    std::printf("-- boss (trimmed faces -> fallback) --\n");
    std::string stepPath = tmpPath("weft_test_boss.step");
    weft::writeStep(weft::makeFixture("boss"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    // The boss root edge is concave (material on both sides of the joint).
    int concave = 0;
    for (const auto& e : a.edges) {
        if (e.convexity == weft::EdgeConvexity::Concave) ++concave;
    }
    CHECK(concave >= 1);

    weft::GenerationSettings gs;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    // The box top carries the boss's circular trim: parametric grid must
    // refuse it and fall back rather than emit a broken grid.
    int fallbacks = 0, grids = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::Fallback) ++fallbacks;
        if (kind == weft::MesherKind::PlanarGrid) ++grids;
    }
    CHECK(fallbacks >= 1);
    CHECK(grids >= 4);
    CHECK(mesh.polygonCount() > 0);
}

}  // namespace

int main() {
    testCylinder();
    testBox();
    testCone();
    testSphere();
    testTorus();
    testBoxDensityMatching();
    testFillet();
    testRecipeRoundTrip();
    testBoss();
    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
