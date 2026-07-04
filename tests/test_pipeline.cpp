// End-to-end test for the phase-0 loop: fixture STEP → import → analyze →
// generate with exact division controls → OBJ. No test framework; each CHECK
// prints and exits non-zero on failure so CTest reports it.

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"
#include "weft/validate.hpp"

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
    // TMPDIR is the Unix convention, TMP/TEMP the Windows one; fall back
    // to the working directory rather than /tmp, which Windows lacks.
    for (const char* var : {"TMPDIR", "TMP", "TEMP"}) {
        if (const char* dir = std::getenv(var); dir && *dir) {
            return std::string(dir) + "/" + name;
        }
    }
    return name;
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

void testMinimalNGon() {
    std::printf("-- minimal n-gon (flat panels stay flat) --\n");
    std::string stepPath = tmpPath("weft_test_minimal_box.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    weft::FaceMeshSettings flat = gs.defaults;
    flat.minimal = true;
    gs.perFace[1] = flat;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    // Face 1 is one 12-vertex n-gon (3+3+3+3 border divisions); everything
    // else keeps its grid, and the solid still welds watertight because the
    // ring carries the density-matched border vertices.
    CHECK(report.faceMesher[1] == weft::MesherKind::MinimalNGon);
    CHECK_EQ(mesh.countNgons(), 1);
    CHECK_EQ(mesh.countQuads(), 5 * 9);
    for (const auto& poly : mesh.polygons) {
        if (poly.size() > 4) CHECK_EQ(poly.size(), 12);
    }
    CHECK(isWatertight(mesh));

    // All-minimal box: 6 n-gons, still watertight — the game-topology
    // "flat panel needs no interior" case taken to its extreme.
    weft::GenerationSettings gsAll;
    gsAll.defaults.gridU = 3;
    gsAll.defaults.gridV = 3;
    gsAll.defaults.minimal = true;
    weft::PolyMesh minimalMesh = weft::generate(model, a, gsAll);
    CHECK_EQ(minimalMesh.countNgons(), 6);
    CHECK_EQ(minimalMesh.countQuads(), 0);
    CHECK(isWatertight(minimalMesh));
}

void testSurfaceConstrainedEditing() {
    std::printf("-- surface-constrained editing --\n");
    std::string stepPath = tmpPath("weft_test_edit_cyl.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    int sideFaceId = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) sideFaceId = f.id;
    }

    weft::GenerationSettings gs;
    gs.defaults.radial = 12;
    gs.defaults.axial = 2;
    weft::PolyMesh mesh = weft::generate(model, a, gs);
    CHECK_EQ(mesh.anchors.size(), mesh.vertices.size());
    CHECK_EQ(mesh.countQuads(), 24);
    size_t vertsBefore = mesh.vertexCount();

    auto radiusOf = [](const std::array<double, 3>& p) {
        return std::sqrt(p[0] * p[0] + p[1] * p[1]);  // cylinder axis = Z
    };

    // Horizontal loop around the cylinder: crosses the 12 axial edges of
    // the bottom band and closes on itself. (u,v) targets the seam column's
    // vertical edge midpoint; v is height on the cylinder (h=30, rows at
    // 0/15/30).
    weft::ManualOp op{weft::ManualOp::Kind::LoopInsert, sideFaceId, 0.0, 7.5,
                      0.5};
    int crossed = weft::insertLoop(mesh, model, op);
    CHECK_EQ(crossed, 12);
    CHECK_EQ(mesh.countQuads(), 24 + 12);
    CHECK_EQ(mesh.vertexCount(), vertsBefore + 12);
    CHECK(isWatertight(mesh));

    // The killer property: every inserted vertex lies EXACTLY on the CAD
    // surface (true snapping, not shrinkwrap).
    for (size_t v = vertsBefore; v < mesh.vertexCount(); ++v) {
        CHECK(std::abs(radiusOf(mesh.vertices[v]) - 10.0) < 1e-9);
        CHECK_EQ(mesh.anchors[v].faceId, sideFaceId);
    }

    // Vertical loop: runs pole-to-pole equivalent — from cap to cap —
    // terminating at both n-gon caps, which must absorb the new vertices
    // to stay watertight (13-gons now).
    weft::ManualOp vop{weft::ManualOp::Kind::LoopInsert, sideFaceId,
                       M_PI / 12.0, 0.0, 0.5};
    size_t quadsBefore = mesh.countQuads();
    int vcrossed = weft::insertLoop(mesh, model, vop);
    CHECK(vcrossed >= 3);  // three bands after the horizontal loop
    CHECK(isWatertight(mesh));
    size_t ngonMax = 0;
    for (const auto& poly : mesh.polygons) {
        ngonMax = std::max(ngonMax, poly.size());
    }
    CHECK_EQ(ngonMax, 13);  // caps absorbed one vertex each
    CHECK_EQ(mesh.countQuads(), quadsBefore + vcrossed);

    // moveVertex snaps back to the surface: shove a side vertex outward,
    // it must land exactly on r=10 again.
    size_t vid = vertsBefore;  // one of the loop vertices
    std::array<double, 3> p = mesh.vertices[vid];
    std::array<double, 3> target{p[0] * 1.7, p[1] * 1.7, p[2] + 3.0};
    weft::moveVertex(mesh, model, vid, target);
    CHECK(std::abs(radiusOf(mesh.vertices[vid]) - 10.0) < 1e-9);
    CHECK(std::abs(mesh.vertices[vid][2] - (p[2] + 3.0)) < 1e-9);
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

    // The notched end faces can't take a grid; guided pairing + one
    // midpoint subdivision must make them quad-dominant — not tri soup.
    // Their borders against parametric grids stay unsplit for conformity,
    // so a few n-gon cells at the seams are expected and correct.
    int quadDominantFaces = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::QuadDominant) continue;
        ++quadDominantFaces;
        size_t quads = 0, total = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            ++total;
            if (mesh.polygons[p].size() == 4) ++quads;
        }
        CHECK(total > 0);
        // These small faces have every border pinned by parametric
        // neighbours and no interior refinement, so pairing can't reach
        // full quad-dominance — but it must be far from tri soup.
        CHECK(3 * quads >= total);
    }
    CHECK_EQ(quadDominantFaces, 2);

    // What the old pure-quad assertion couldn't promise: the whole solid,
    // grids and freeform faces together, is watertight.
    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, 0u);
    CHECK_EQ(vr.nonManifoldEdges, 0u);
    CHECK_EQ(vr.windingConflicts, 0u);

    // Hold clustering: same counts, but the loops crowd toward the creases —
    // the first across-interval must shrink vs the uniform mesh.
    weft::GenerationSettings gsHold = gs;
    gsHold.defaults.filletHold = 0.8;
    weft::PolyMesh held = weft::generate(model, a, gsHold);
    std::map<int, int> heldPerFace;
    for (size_t p = 0; p < held.polygons.size(); ++p) {
        ++heldPerFace[held.polygonFaceId[p]];
    }
    // Same loop count on the blend itself; the conformal neighbours may
    // re-triangulate slightly as the loop positions move.
    CHECK_EQ(heldPerFace[filletFaceId], polysPerFace[filletFaceId]);

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
    weft::Recipe recipe;
    weft::GenerationSettings& gs = recipe.settings;
    gs.defaults.radial = 20;
    gs.defaults.cap = weft::CapStyle::Fan;
    weft::FaceMeshSettings dense = gs.defaults;
    dense.radial = 40;
    dense.gridV = 7;
    gs.perFace[3] = dense;
    gs.perEdge[5] = 13;
    recipe.ops.push_back({weft::ManualOp::Kind::LoopInsert, 1, 0.25, 7.5, 0.5});

    std::string path = tmpPath("weft_test.recipe");
    weft::saveRecipe(recipe, path);
    weft::Recipe loaded = weft::loadRecipe(path);

    CHECK_EQ(loaded.settings.defaults.radial, 20);
    CHECK(loaded.settings.defaults.cap == weft::CapStyle::Fan);
    CHECK_EQ(loaded.settings.perFace.size(), 1);
    CHECK_EQ(loaded.settings.perFace[3].radial, 40);
    CHECK_EQ(loaded.settings.perFace[3].gridV, 7);
    CHECK_EQ(loaded.settings.perEdge[5], 13);
    CHECK_EQ(loaded.ops.size(), 1);
    CHECK_EQ(loaded.ops[0].faceId, 1);
    CHECK(std::abs(loaded.ops[0].t - 0.5) < 1e-12);

    // Same recipe, same B-rep => identical topology, manual ops included
    // (regenerability is the point of persisting decisions, not meshes).
    std::string stepPath = tmpPath("weft_test_recipe_cyl.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    weft::PolyMesh m1 = weft::generate(model, a, gs);
    weft::applyOps(m1, model, recipe.ops);
    weft::PolyMesh m2 = weft::generate(model, a, loaded.settings);
    weft::applyOps(m2, model, loaded.ops);
    CHECK_EQ(m1.vertexCount(), m2.vertexCount());
    CHECK_EQ(m1.polygonCount(), m2.polygonCount());
}

void testBoss() {
    std::printf("-- boss (ring junction) --\n");
    std::string stepPath = tmpPath("weft_test_boss.step");
    weft::writeStep(weft::makeFixture("boss"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    // The boss root edge is concave (material on both sides of the joint),
    // and the boss wall is a shaft, not a hole.
    int concave = 0;
    for (const auto& e : a.edges) {
        if (e.convexity == weft::EdgeConvexity::Concave) ++concave;
    }
    CHECK(concave >= 1);
    for (const auto& f : a.faces) CHECK(!f.isHole);

    weft::GenerationSettings gs;
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    gs.defaults.junctionRings = 2;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    // The box top (rectangle with the boss's circular trim) is the
    // cylinder-to-plane junction: no face needs fallback triangulation.
    int fallbacks = 0, junctions = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::Fallback) ++fallbacks;
        if (kind == weft::MesherKind::RingJunction) ++junctions;
    }
    CHECK_EQ(fallbacks, 0);
    CHECK_EQ(junctions, 1);

    // The plate drives the boss: ring count = 2*(3+3) = 12, so the boss
    // wall gets 12 radial divisions and its cap is a 12-gon; the whole
    // fused solid is watertight quads + one n-gon.
    CHECK_EQ(mesh.countTris(), 0);
    CHECK_EQ(mesh.countNgons(), 1);
    for (const auto& poly : mesh.polygons) {
        if (poly.size() > 4) CHECK_EQ(poly.size(), 12);
    }
    CHECK(isWatertight(mesh));
}

void testHolePlate() {
    std::printf("-- hole (through-bore plate) --\n");
    std::string stepPath = tmpPath("weft_test_hole.step");
    weft::writeStep(weft::makeFixture("hole"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    int holes = 0;
    for (const auto& f : a.faces) {
        if (f.isHole) {
            ++holes;
            CHECK(f.type == weft::SurfaceType::Cylinder);
            CHECK(std::abs(f.radius - 8.0) < 1e-9);
        }
    }
    CHECK_EQ(holes, 1);

    weft::GenerationSettings gs;
    gs.defaults.gridU = 4;
    gs.defaults.gridV = 4;
    gs.defaults.axial = 2;
    gs.defaults.junctionRings = 3;
    // Deliberately absurd radial: the junctions must override it to 16.
    gs.defaults.radial = 99;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    int junctions = 0, fallbacks = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::RingJunction) ++junctions;
        if (kind == weft::MesherKind::Fallback) ++fallbacks;
    }
    CHECK_EQ(junctions, 2);  // top and bottom of the plate
    CHECK_EQ(fallbacks, 0);
    CHECK_EQ(mesh.countTris(), 0);
    CHECK_EQ(mesh.countNgons(), 0);
    CHECK(isWatertight(mesh));

    // 2 junctions (3 rings x 16) + bore wall (16 x 2) + 4 sides (4x4 each).
    CHECK_EQ(mesh.countQuads(), 2 * 3 * 16 + 16 * 2 + 4 * 16);
}

// The groove wall is an open-u cylinder strip meshed as a parametric grid;
// the box ends are trimmed planes with arc borders that triangulate. The
// shared arcs are where conformal boundary surgery must keep the solid
// watertight while the parametric side keeps exact division control.
void testNotch() {
    std::printf("-- notch (parametric/triangulated borders) --\n");
    std::string stepPath = tmpPath("weft_test_notch.step");
    weft::writeStep(weft::makeFixture("notch"), stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    int grids = 0, triangulated = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::PlanarGrid) ++grids;
        if (kind == weft::MesherKind::QuadDominant ||
            kind == weft::MesherKind::Fallback) {
            ++triangulated;
        }
    }
    CHECK(grids >= 1);         // the groove wall (plus untouched box sides)
    CHECK(triangulated >= 2);  // the two notched end faces

    weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, 0u);
    CHECK_EQ(vr.nonManifoldEdges, 0u);
    CHECK_EQ(vr.windingConflicts, 0u);
    CHECK_EQ(vr.degeneratePolygons, 0u);

    // Densify the groove: its neighbours must follow and stay watertight.
    weft::GenerationSettings dense = gs;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::PlanarGrid) continue;
        const auto& info = a.faces[fid - 1];
        if (info.type == weft::SurfaceType::Cylinder) {
            weft::FaceMeshSettings s = dense.defaults;
            s.gridU = 12;
            s.gridV = 6;
            dense.perFace[fid] = s;
        }
    }
    CHECK(!dense.perFace.empty());
    weft::PolyMesh fine = weft::generate(model, a, dense);
    weft::ValidationReport vr2 = weft::validateMesh(fine, &model);
    CHECK_EQ(vr2.openEdges, 0u);
    CHECK_EQ(vr2.nonManifoldEdges, 0u);
    CHECK(fine.polygonCount() > mesh.polygonCount());
}

// Every fixture, meshed with defaults, must come out bake-ready: closed,
// consistently wound, with no degenerate polygons.
void testAllFixturesValidate() {
    std::printf("-- all fixtures validate --\n");
    for (const char* name :
         {"cylinder", "box", "cone", "sphere", "torus", "fillet", "hole",
          "notch", "demo", "boss"}) {
        std::string stepPath = tmpPath(std::string("weft_test_v_") + name +
                                       ".step");
        weft::writeStep(weft::makeFixture(name), stepPath);
        weft::Model model = weft::loadStep(stepPath);
        weft::Analysis a = weft::analyze(model);
        weft::PolyMesh mesh = weft::generate(model, a, weft::GenerationSettings{});
        weft::ValidationReport vr = weft::validateMesh(mesh, &model);
        if (!vr.clean()) {
            std::printf("  %s: open=%zu nonmanifold=%zu winding=%zu degen=%zu\n",
                        name, vr.openEdges, vr.nonManifoldEdges,
                        vr.windingConflicts, vr.degeneratePolygons);
        }
        CHECK(vr.clean());
        // Chord tolerance (0.1) bounds triangulated faces only; parametric
        // grids honour exact division counts instead, so a 16x4 sphere or
        // torus legitimately sags ~1 unit. Bound is loose but catches
        // unit-scale blunders and centroid/UV mismatches.
        CHECK(vr.maxDeviation <= 1.5);
    }
}

}  // namespace

// Announce each test and turn stray exceptions into a named failure
// instead of a silent fail-fast crash (0xc0000409 on Windows).
#define RUN(fn)                                               \
    do {                                                      \
        std::printf("%-32s", #fn);                            \
        std::fflush(stdout);                                  \
        try {                                                 \
            fn();                                             \
            std::printf("ok\n");                              \
        } catch (const std::exception& e) {                   \
            std::printf("EXCEPTION: %s\n", e.what());         \
            ++failures;                                       \
        } catch (...) {                                       \
            std::printf("EXCEPTION (unknown type)\n");        \
            ++failures;                                       \
        }                                                     \
    } while (0)

int main() {
    RUN(testCylinder);
    RUN(testBox);
    RUN(testCone);
    RUN(testSphere);
    RUN(testTorus);
    RUN(testBoxDensityMatching);
    RUN(testMinimalNGon);
    RUN(testSurfaceConstrainedEditing);
    RUN(testFillet);
    RUN(testRecipeRoundTrip);
    RUN(testBoss);
    RUN(testHolePlate);
    RUN(testNotch);
    RUN(testAllFixturesValidate);
    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
