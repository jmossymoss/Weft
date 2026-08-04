// Smoke test for the accuracy-tessellator branch base.
#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

static int gFails = 0;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
            ++gFails;                                                          \
        }                                                                      \
    } while (0)

static std::string tmpStep(const char* name) {
    const char* dir = std::getenv("TMPDIR");
    if (!dir || !dir[0]) dir = "/tmp";
    return std::string(dir) + "/" + name;
}

static weft::Model loadFixture(const char* shape, const char* file) {
    const std::string path = tmpStep(file);
    weft::writeStep(weft::makeFixture(shape), path);
    return weft::loadStep(path);
}

static void testCylinderMedium() {
    std::printf("-- cylinder Medium sag --\n");
    weft::Model model = loadFixture("cylinder", "weft_acc_cyl.step");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    weft::applyQualityPreset(gs.defaults, weft::QualityPreset::Medium);
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(mesh.vertexCount() > 8);
    CHECK(mesh.polygonCount() > 0);
    CHECK(mesh.countNgons() + mesh.countTris() + mesh.countQuads() ==
          mesh.polygonCount());
    std::printf("  %zu verts, %zu polys (tris=%zu ngons=%zu)\n",
                mesh.vertexCount(), mesh.polygonCount(), mesh.countTris(),
                mesh.countNgons());
}

static void testBoxPlanarNgons() {
    std::printf("-- box planar n-gons --\n");
    weft::Model model = loadFixture("box", "weft_acc_box.step");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    weft::applyQualityPreset(gs.defaults, weft::QualityPreset::Medium);
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(mesh.polygonCount() >= 6);
    CHECK(mesh.countNgons() + mesh.countQuads() > 0);
    for (const auto& [fid, kind] : report.faceMesher) {
        CHECK(kind == weft::MesherKind::MinimalNGon);
        (void)fid;
    }
    std::printf("  %zu verts, %zu polys (ngons=%zu)\n", mesh.vertexCount(),
                mesh.polygonCount(), mesh.countNgons());
}

static void testPresetHonesDensity() {
    std::printf("-- Low vs Medium vs High polycount --\n");
    weft::Model model = loadFixture("cylinder", "weft_acc_cyl2.step");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings lowGs, medGs, highGs;
    weft::applyQualityPreset(lowGs.defaults, weft::QualityPreset::Low);
    weft::applyQualityPreset(medGs.defaults, weft::QualityPreset::Medium);
    weft::applyQualityPreset(highGs.defaults, weft::QualityPreset::High);
    weft::GenerationReport lowRep, medRep, highRep;
    weft::PolyMesh low = weft::generate(model, analysis, lowGs, &lowRep);
    weft::PolyMesh med = weft::generate(model, analysis, medGs, &medRep);
    weft::PolyMesh high = weft::generate(model, analysis, highGs, &highRep);
    CHECK(med.polygonCount() > low.polygonCount());
    CHECK(high.polygonCount() > med.polygonCount());
    // Circumferential rim divisions must rise with tighter sag.
    auto rimDiv = [](const weft::GenerationReport& r) {
        int best = 0;
        for (const auto& [eid, n] : r.edgeDivisions) best = std::max(best, n);
        return best;
    };
    CHECK(rimDiv(medRep) > rimDiv(lowRep));
    CHECK(rimDiv(highRep) > rimDiv(medRep));
    std::printf("  Low=%zu/%d Medium=%zu/%d High=%zu/%d (polys/rimDiv)\n",
                low.polygonCount(), rimDiv(lowRep), med.polygonCount(),
                rimDiv(medRep), high.polygonCount(), rimDiv(highRep));
}

static void testCylinderCapsAreNgons() {
    std::printf("-- cylinder caps are n-gons, wall is analytic --\n");
    weft::Model model = loadFixture("cylinder", "weft_acc_cyl3.step");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    weft::applyQualityPreset(gs.defaults, weft::QualityPreset::Medium);
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    int walls = 0, caps = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::RevolutionGrid) ++walls;
        if (kind == weft::MesherKind::MinimalNGon) ++caps;
    }
    CHECK(walls == 1);
    CHECK(caps == 2);
    CHECK(mesh.countNgons() >= 2);
    std::printf("  walls=%d caps=%d ngons=%zu\n", walls, caps,
                mesh.countNgons());
}

static void testFilletFixture() {
    std::printf("-- fillet fixture meshes --\n");
    weft::Model model = loadFixture("fillet", "weft_acc_fillet.step");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    weft::applyQualityPreset(gs.defaults, weft::QualityPreset::High);
    // Fillets need maxAngle like Pixyz docs recommend.
    gs.defaults.angleToleranceDeg = 20.0;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(mesh.polygonCount() > 0);
    CHECK(mesh.vertexCount() > 8);
    std::printf("  faces=%d verts=%zu polys=%zu\n", model.faceCount(),
                mesh.vertexCount(), mesh.polygonCount());
}

int main() {
    try {
        testBoxPlanarNgons();
        testCylinderMedium();
        testCylinderCapsAreNgons();
        testPresetHonesDensity();
        testFilletFixture();
    } catch (const std::exception& e) {
        std::printf("exception: %s\n", e.what());
        return 1;
    }
    if (gFails) {
        std::printf("%d check(s) failed\n", gFails);
        return 1;
    }
    std::printf("ok\n");
    return 0;
}
