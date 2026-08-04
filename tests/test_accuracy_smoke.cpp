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
    std::printf("-- Low vs High polycount --\n");
    weft::Model model = loadFixture("cylinder", "weft_acc_cyl2.step");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings lowGs, highGs;
    weft::applyQualityPreset(lowGs.defaults, weft::QualityPreset::Low);
    weft::applyQualityPreset(highGs.defaults, weft::QualityPreset::High);
    weft::PolyMesh low = weft::generate(model, analysis, lowGs);
    weft::PolyMesh high = weft::generate(model, analysis, highGs);
    CHECK(high.polygonCount() > low.polygonCount());
    std::printf("  Low=%zu High=%zu polys\n", low.polygonCount(),
                high.polygonCount());
}

int main() {
    try {
        testBoxPlanarNgons();
        testCylinderMedium();
        testPresetHonesDensity();
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
