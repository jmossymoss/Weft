// Independent tessellation: planar n-gons, no density solver, local spans.
#include "weft/analysis.hpp"
#include "weft/fixture.hpp"
#include "weft/independent_mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/validate.hpp"

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

static weft::Model loadFixture(const std::string& name) {
    const std::string path = tmpPath("weft_ind_" + name + ".step");
    weft::writeStep(weft::makeFixture(name), path);
    return weft::loadStep(path);
}

static void testBoxNgons() {
    std::printf("-- independent box: one polygon per planar face --\n");
    weft::Model model = loadFixture("box");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = false;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs, &report);
    CHECK_EQ(int(mesh.polygonCount()), model.faceCount());
    CHECK_EQ(mesh.countTris(), size_t(0));
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
    int ngonFaces = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::MinimalNGon) ++ngonFaces;
    }
    CHECK_EQ(ngonFaces, model.faceCount());
}

static void testCylinderCaps() {
    std::printf("-- independent cylinder: planar caps, tessellated wall --\n");
    weft::Model model = loadFixture("cylinder");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = false;
    gs.defaults.radial = 16;
    gs.defaults.axial = 4;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs, &report);
    CHECK(mesh.polygonCount() > 0);
    CHECK(mesh.vertexCount() > 0);
    int planar = 0, drums = 0;
    size_t capPolys = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass == weft::FeatureClass::PlanarPanel ||
            f.type == weft::SurfaceType::Plane) {
            ++planar;
            size_t n = 0;
            for (size_t p = 0; p < mesh.polygonFaceId.size(); ++p) {
                if (mesh.polygonFaceId[p] == f.id) ++n;
            }
            CHECK_EQ(n, size_t(1));
            capPolys += n;
        }
        if (f.featureClass == weft::FeatureClass::Drum) ++drums;
    }
    CHECK(planar >= 2);
    CHECK(drums >= 1);
    CHECK(capPolys >= 2);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
    CHECK_EQ(vr.windingConflicts, size_t(0));
}

static void testHoleKeepsRim() {
    std::printf("-- independent hole: plate is not empty --\n");
    weft::Model model = loadFixture("hole");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs, &report);
    CHECK(mesh.polygonCount() > 0);
    bool sawPlate = false;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::HolePlate &&
            f.loop.wireCount < 2) {
            continue;
        }
        size_t n = 0;
        for (int id : mesh.polygonFaceId) {
            if (id == f.id) ++n;
        }
        CHECK(n > 0);
        sawPlate = true;
    }
    CHECK(sawPlate);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
}

static void testFilletWatertight() {
    std::printf("-- independent fillet: blend + flats stay closed --\n");
    weft::Model model = loadFixture("fillet");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs);
    CHECK(mesh.polygonCount() > 0);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
}

static void testSphereClosed() {
    std::printf("-- independent sphere: not empty, closed --\n");
    weft::Model model = loadFixture("sphere");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    gs.defaults.radial = 16;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs);
    CHECK(mesh.polygonCount() > 0);
    CHECK(mesh.vertexCount() > 0);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
}

static void testDrumSpanIsLocal() {
    std::printf("-- independent drum radial override changes that face --\n");
    weft::Model model = loadFixture("cylinder");
    weft::Analysis analysis = weft::analyze(model);
    int drum = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass == weft::FeatureClass::Drum) {
            drum = f.id;
            break;
        }
    }
    CHECK(drum > 0);
    weft::GenerationSettings a;
    a.defaults.minimal = true;
    a.defaults.radial = 12;
    weft::PolyMesh ma = weft::meshIndependent(model, analysis, a);
    weft::GenerationSettings b = a;
    weft::FaceMeshSettings fs = b.defaults;
    fs.radial = 32;
    b.perFace[drum] = fs;
    weft::PolyMesh mb = weft::meshIndependent(model, analysis, b);
    CHECK(mb.polygonCount() != ma.polygonCount() ||
          mb.vertexCount() != ma.vertexCount());
}

static void testTorusUvLattice() {
    std::printf("-- independent torus: UV lattice, not an ear-clip fan --\n");
    weft::Model model = loadFixture("torus");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs, &report);
    CHECK(mesh.countQuads() >= 64);
    CHECK(mesh.countQuads() > mesh.countTris());
    CHECK(report.faceMesher[1] == weft::MesherKind::RevolutionGrid);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
    CHECK(vr.sliverPolygons < mesh.polygonCount() / 4);
}

static void testEllipseWallIsTube() {
    std::printf("-- independent ellipse_plate: hole wall is a tube --\n");
    weft::Model model = loadFixture("ellipse_plate");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs, &report);
    int holePlates = 0;
    int extrusionQuads = 0;
    for (const auto& f : analysis.faces) {
        size_t n = 0, tris = 0, quads = 0;
        for (size_t p = 0; p < mesh.polygonFaceId.size(); ++p) {
            if (mesh.polygonFaceId[p] != f.id) continue;
            ++n;
            const size_t a = mesh.polygons[p].size();
            if (a == 3) ++tris;
            else if (a == 4) ++quads;
        }
        if (f.featureClass == weft::FeatureClass::HolePlate) {
            CHECK_EQ(n, size_t(1));
            ++holePlates;
        }
        if (f.type == weft::SurfaceType::Extrusion) {
            extrusionQuads += int(quads);
            CHECK(quads > tris);
            CHECK(report.faceMesher[f.id] == weft::MesherKind::RevolutionGrid ||
                  report.faceMesher[f.id] == weft::MesherKind::CoonsGrid);
        }
    }
    CHECK(holePlates >= 2);
    CHECK(extrusionQuads > 0);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
}

static void testBezierSlabWinding() {
    std::printf("-- independent bezier_slab: 3D coons keeps winding --\n");
    weft::Model model = loadFixture("bezier_slab");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
    CHECK_EQ(vr.windingConflicts, size_t(0));
}

static void testTrimmedDrumKeepsSamples() {
    std::printf("-- independent slotted: trimmed drums keep exact samples --\n");
    weft::Model model = loadFixture("slotted");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs, &report);
    int drums = 0, occtDrums = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Cylinder) continue;
        ++drums;
        auto it = report.faceMesher.find(f.id);
        if (it != report.faceMesher.end() &&
            it->second == weft::MesherKind::Fallback) {
            ++occtDrums;
        }
    }
    CHECK(drums >= 1);
    CHECK_EQ(occtDrums, 0);
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, size_t(0));
    CHECK_EQ(vr.nonManifoldEdges, size_t(0));
}

static void testNotchedWallNotFan() {
    std::printf("-- independent notched: OCCT UV CDT, not an ear-clip fan --\n");
    weft::Model model = loadFixture("notched");
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.independentMesh = true;
    gs.defaults.minimal = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::meshIndependent(model, analysis, gs, &report);
    bool sawDrum = false;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Cylinder) continue;
        sawDrum = true;
        std::vector<int> valence(mesh.vertexCount(), 0);
        size_t tris = 0, quads = 0;
        for (size_t p = 0; p < mesh.polygonFaceId.size(); ++p) {
            if (mesh.polygonFaceId[p] != f.id) continue;
            const auto& poly = mesh.polygons[p];
            if (poly.size() == 3) ++tris;
            else if (poly.size() == 4) ++quads;
            for (uint32_t v : poly) {
                if (v < valence.size()) ++valence[v];
            }
        }
        int maxVal = 0;
        for (int v : valence) maxVal = std::max(maxVal, v);
        // A fan from one rim vertex uses ~N-2 triangles. OCCT's polygon
        // CDT keeps valence bounded.
        CHECK(maxVal < 12);
        CHECK(quads + tris > 0);
        auto it = report.faceMesher.find(f.id);
        CHECK(it != report.faceMesher.end());
        CHECK(it->second != weft::MesherKind::Fallback || quads > 0);
    }
    CHECK(sawDrum);
}

int main() {
    testBoxNgons();
    testCylinderCaps();
    testHoleKeepsRim();
    testFilletWatertight();
    testSphereClosed();
    testDrumSpanIsLocal();
    testTorusUvLattice();
    testEllipseWallIsTube();
    testBezierSlabWinding();
    testTrimmedDrumKeepsSamples();
    testNotchedWallNotFan();
    if (gFails) {
        std::fprintf(stderr, "%d FAILURE(S)\n", gFails);
        return 1;
    }
    std::printf("independent_mesh: ok\n");
    return 0;
}
