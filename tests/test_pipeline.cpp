// End-to-end test for the phase-0 loop: fixture STEP → import → analyze →
// generate with exact division controls → OBJ. No test framework; each CHECK
// prints and exits non-zero on failure so CTest reports it.

#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

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

    int cylinderGrids = 0, diskCaps = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::CylinderGrid) ++cylinderGrids;
        if (kind == weft::MesherKind::DiskCap) ++diskCaps;
    }
    CHECK_EQ(cylinderGrids, 1);
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

    // Per-face override: crank only the side face's radial count. Caps keep
    // the default 12, so the borders no longer match — polygon counts still
    // must reflect the override exactly.
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
    testBoss();
    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
