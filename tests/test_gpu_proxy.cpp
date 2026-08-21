// GPU preview: lattice counts match generate() faceCounts for grid meshers,
// and isolines use the trimmed face UV box rather than the parent surface.
#include "../app/gpu_proxy.hpp"

#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

static int gFails = 0;
#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__,     \
                         #cond);                                             \
            ++gFails;                                                        \
        }                                                                    \
    } while (0)
#define CHECK_EQ(a, b)                                                       \
    do {                                                                     \
        auto _va = (a);                                                      \
        auto _vb = (b);                                                      \
        if (!(_va == _vb)) {                                                 \
            std::fprintf(stderr, "FAIL %s:%d: %s == %s (%s vs %s)\n",        \
                         __FILE__, __LINE__, #a, #b,                         \
                         std::to_string(_va).c_str(),                        \
                         std::to_string(_vb).c_str());                       \
            ++gFails;                                                        \
        }                                                                    \
    } while (0)

static std::string tmpPath(const std::string& name) {
    return (std::filesystem::temp_directory_path() / name).string();
}

static void testLatticeCountsMatchGenerate() {
    std::printf("-- gpu proxy counts match generate faceCounts --\n");
    const std::string path = tmpPath("weft_gpu_proxy_cyl.step");
    weft::writeStep(weft::makeFixture("cylinder"), path);
    weft::Model model = weft::loadStep(path);
    weft::Analysis analysis = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.finalizeMesh = false;
    gs.defaults.adaptive = false;
    gs.defaults.radial = 24;
    gs.defaults.axial = 4;
    weft::GenerationReport report;
    (void)weft::generate(model, analysis, gs, &report);

    int drum = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::RevolutionGrid) {
            drum = fid;
            break;
        }
    }
    CHECK(drum > 0);
    const auto solved = report.faceCounts[drum];
    const weft::FaceMeshSettings& s = gs.forFace(drum);
    const auto grid = weft_app::gpuProxyGrid(
        weft::MesherKind::RevolutionGrid, s, s, solved, 0, false);
    CHECK(grid.drawLattice);
    CHECK_EQ(int(std::lround(grid.u)), solved[0]);
    CHECK_EQ(int(std::lround(grid.v)), solved[1]);
    CHECK_EQ(int(std::lround(grid.u)), 24);
    CHECK_EQ(int(std::lround(grid.v)), 4);
}

static void testNonLatticeHidesOverlay() {
    weft::FaceMeshSettings s;
    s.radial = 16;
    s.boundary = 32;
    const auto plate = weft_app::gpuProxyGrid(weft::MesherKind::PlateWeb, s, s,
                                              {8, 1}, 0, false);
    CHECK(!plate.drawLattice);
    const auto disk = weft_app::gpuProxyGrid(weft::MesherKind::DiskCap, s, s,
                                             {16, 1}, 0, false);
    CHECK(!disk.drawLattice);
    const auto ngon = weft_app::gpuProxyGrid(weft::MesherKind::MinimalNGon, s,
                                             s, {1, 1}, 0, false);
    CHECK(!ngon.drawLattice);
}

static void testFilletAcrossAndCoonsRotate() {
    weft::FaceMeshSettings s;
    s.radial = 20;
    s.filletLoops = 3;
    s.gridU = 18;
    s.gridV = 5;
    s.coonsRotate = 1;
    const auto strip = weft_app::gpuProxyGrid(
        weft::MesherKind::RevolutionGrid, s, s, {20, 3}, 2, true);
    CHECK(strip.drawLattice);
    CHECK_EQ(int(std::lround(strip.u)), 20);
    CHECK_EQ(int(std::lround(strip.v)), 3);

    const auto coons = weft_app::gpuProxyGrid(weft::MesherKind::CoonsGrid, s,
                                              s, {18, 5}, 0, false);
    CHECK(coons.drawLattice);
    // Odd coonsRotate swaps the overlay axes to the patch the mesher walks.
    CHECK_EQ(int(std::lround(coons.u)), 5);
    CHECK_EQ(int(std::lround(coons.v)), 18);
}

static void testProxyActiveWhileQueued() {
    CHECK(weft_app::gpuProxyActive(weft_app::FaceFidelity::Queued, false,
                                   false));
    CHECK(weft_app::gpuProxyActive(weft_app::FaceFidelity::Baking, false,
                                   false));
    CHECK(weft_app::gpuProxyActive(weft_app::FaceFidelity::HighFidelity, true,
                                   false));
    CHECK(!weft_app::gpuProxyActive(weft_app::FaceFidelity::HighFidelity,
                                    false, false));
    CHECK(weft_app::gpuProxyActive(weft_app::FaceFidelity::HighFidelity, false,
                                   true));
}

static void testTrimmedFaceUvBox() {
    std::printf("-- gpu proxy UV uses face trim, not surface domain --\n");
    const std::string cylPath = tmpPath("weft_gpu_proxy_uv_cyl.step");
    weft::writeStep(weft::makeFixture("cylinder"), cylPath);
    weft::Model cyl = weft::loadStep(cylPath);
    weft::Analysis cylA = weft::analyze(cyl);
    bool fullPeriod = false;
    for (const auto& f : cylA.faces) {
        if (f.type != weft::SurfaceType::Cylinder) continue;
        const auto box = weft_app::faceProxyUvBox(cyl, f.id);
        CHECK(box.valid);
        const double faceSpan = box.u1 - box.u0;
        const double surfSpan = box.surfU1 - box.surfU0;
        CHECK(faceSpan > 1.5);
        // Closed cylinder wall: trim box is the full period.
        if (faceSpan > 0.9 * surfSpan) fullPeriod = true;
        const auto uv = weft_app::normalizeProxyUv(box, box.u0, box.v0);
        CHECK(std::abs(uv[0]) < 1e-9);
        CHECK(std::abs(uv[1]) < 1e-9);
        const auto uv1 = weft_app::normalizeProxyUv(box, box.u1, box.v1);
        CHECK(std::abs(uv1[0] - 1.0) < 1e-9);
        CHECK(std::abs(uv1[1] - 1.0) < 1e-9);
    }
    CHECK(fullPeriod);

    const std::string boxPath = tmpPath("weft_gpu_proxy_uv_box.step");
    weft::writeStep(weft::makeFixture("box"), boxPath);
    weft::Model box = weft::loadStep(boxPath);
    weft::Analysis boxA = weft::analyze(box);
    bool boxOk = false;
    for (const auto& f : boxA.faces) {
        if (f.type != weft::SurfaceType::Plane) continue;
        const auto b = weft_app::faceProxyUvBox(box, f.id);
        CHECK(b.valid);
        const double faceU = b.u1 - b.u0;
        const double faceV = b.v1 - b.v0;
        CHECK(faceU > 1e-6 && std::isfinite(faceU));
        CHECK(faceV > 1e-6 && std::isfinite(faceV));
        const auto uv0 = weft_app::normalizeProxyUv(b, b.u0, b.v0);
        const auto uv1 = weft_app::normalizeProxyUv(b, b.u1, b.v1);
        CHECK(std::abs(uv0[0]) < 1e-9 && std::abs(uv0[1]) < 1e-9);
        CHECK(std::abs(uv1[0] - 1.0) < 1e-9 && std::abs(uv1[1] - 1.0) < 1e-9);
        boxOk = true;
    }
    CHECK(boxOk);

    const std::string slabPath = tmpPath("weft_gpu_proxy_uv_slab.step");
    weft::writeStep(weft::makeFixture("bspline_slab"), slabPath);
    weft::Model slab = weft::loadStep(slabPath);
    weft::Analysis slabA = weft::analyze(slab);
    bool slabOk = false;
    for (const auto& f : slabA.faces) {
        const auto b = weft_app::faceProxyUvBox(slab, f.id);
        if (!b.valid) continue;
        const auto uv0 = weft_app::normalizeProxyUv(b, b.u0, b.v0);
        const auto uv1 = weft_app::normalizeProxyUv(b, b.u1, b.v1);
        CHECK(std::abs(uv0[0]) < 1e-9 && std::abs(uv0[1]) < 1e-9);
        CHECK(std::abs(uv1[0] - 1.0) < 1e-9 && std::abs(uv1[1] - 1.0) < 1e-9);
        slabOk = true;
    }
    CHECK(slabOk);
}

int main() {
    testLatticeCountsMatchGenerate();
    testNonLatticeHidesOverlay();
    testFilletAcrossAndCoonsRotate();
    testProxyActiveWhileQueued();
    testTrimmedFaceUvBox();
    if (gFails) {
        std::fprintf(stderr, "%d FAILURE(S)\n", gFails);
        return 1;
    }
    std::printf("gpu_proxy: ok\n");
    return 0;
}
