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

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp_Ax2.hxx>

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
    gs.defaults.quadDominant = true;  // exercise guided pairing explicitly
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    CHECK(report.faceMesher[filletFaceId] == weft::MesherKind::PlanarGrid);
    std::map<int, int> polysPerFace;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        ++polysPerFace[mesh.polygonFaceId[p]];
    }
    CHECK_EQ(polysPerFace[filletFaceId], 5 * 4);

    // The notched end faces can't take a grid; guided pairing + one
    // midpoint subdivision turns their interiors into quads, while their
    // border polygons conform to the neighbouring grids' divisions (which
    // can add or drop sides). Quad-dominant, and the solid is watertight.
    int quadDominantFaces = 0;
    size_t qdPolys = 0, qdQuads = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::QuadDominant) continue;
        ++quadDominantFaces;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            ++qdPolys;
            if (mesh.polygons[p].size() == 4) ++qdQuads;
        }
    }
    CHECK_EQ(quadDominantFaces, 2);
    CHECK(qdQuads > 0);         // pairing still yields interior quads
    CHECK(qdPolys > qdQuads);   // borders conformed (non-quads at seams)
    CHECK(isWatertight(mesh));  // ...which is the point: no leaks

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

}  // namespace

// Delete a face, bridge the resulting boundary loops (equal counts -> pure
// quad ring, unequal -> triangle zipper), stay watertight throughout.
void testBridge() {
    std::printf("-- bridge --\n");
    std::string stepPath = tmpPath("weft_test_bridge.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    int sideFace = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) sideFace = f.id;
    }
    CHECK(sideFace > 0);
    // The two rim circles are the side face's edges shared with the caps.
    std::vector<int> rims;
    for (const auto& e : a.edges) {
        for (int fid : e.faceIds) {
            if (fid == sideFace && e.faceIds.size() == 2) rims.push_back(e.id);
        }
    }
    CHECK_EQ(rims.size(), 2);

    weft::GenerationSettings gs;
    gs.defaults.radial = 12;
    gs.perFace[sideFace] = gs.defaults;
    gs.perFace[sideFace].exclude = true;  // delete the wall

    weft::PolyMesh open = weft::generate(model, a, gs);
    CHECK(!isWatertight(open));  // two open rims
    CHECK_EQ(weft::boundaryLoops(open).size(), 2);

    // Equal rim counts (both caps at radial=12): bridging yields a pure
    // quad ring and the solid closes back up as a plain tube.
    weft::ManualOp bridge;
    bridge.kind = weft::ManualOp::Kind::Bridge;
    bridge.edgeA = rims[0];
    bridge.edgeB = rims[1];
    bridge.twist = 3;
    weft::PolyMesh closed = open;
    size_t before = closed.polygonCount();
    CHECK_EQ(weft::bridgeLoops(closed, model, bridge), 12);
    CHECK_EQ(closed.polygonCount(), before + 12);
    CHECK(isWatertight(closed));
    CHECK_EQ(closed.countQuads(), 12);

    // Unequal rim counts: pin one rim to 18. The strip triangulates
    // (12 + 18 edges -> 30 triangles) and still closes watertight.
    gs.perEdge[rims[1]] = 18;
    weft::PolyMesh open2 = weft::generate(model, a, gs);
    CHECK(!isWatertight(open2));
    weft::PolyMesh closed2 = open2;
    CHECK_EQ(weft::bridgeLoops(closed2, model, bridge), 30);
    CHECK(isWatertight(closed2));

    // Twisting the pairing keeps the strip watertight (any rotation of
    // the rails is still a closed strip) and rotates the rail seams.
    for (int twist : {1, -2, 7}) {
        weft::ManualOp twisted = bridge;
        twisted.twist = twist;
        weft::PolyMesh tw = open;
        CHECK_EQ(weft::bridgeLoops(tw, model, twisted), 12);
        CHECK(isWatertight(tw));
    }

    // Pinned resample on a boundary with no analytic driver: the wall is
    // deleted, so its rims border nothing — pinning a rim re-cuts that
    // boundary loop to exactly the pinned count, on the curve.
    {
        weft::GenerationSettings gsPin = gs;
        gsPin.perEdge[rims[0]] = 9;
        weft::PolyMesh pinned = weft::generate(model, a, gsPin);
        bool found = false;
        for (const auto& loop : weft::boundaryLoops(pinned)) {
            if (loop.size() == 9) found = true;
        }
        CHECK(found);
    }

    // The op replays through applyOps and recipes round-trip it.
    weft::Recipe recipe;
    recipe.settings = gs;
    recipe.ops.push_back(bridge);
    std::string rPath = tmpPath("weft_test_bridge.recipe");
    weft::saveRecipe(recipe, rPath);
    weft::Recipe loaded = weft::loadRecipe(rPath);
    CHECK_EQ(loaded.ops.size(), 1);
    CHECK(loaded.ops[0].kind == weft::ManualOp::Kind::Bridge);
    CHECK_EQ(loaded.ops[0].twist, bridge.twist);
    CHECK(loaded.settings.forFace(sideFace).exclude);
    weft::PolyMesh replayed = weft::generate(model, a, loaded.settings);
    weft::applyOps(replayed, model, loaded.ops);
    CHECK(isWatertight(replayed));
}

// A washer (cylinder minus coaxial bore): the two annulus faces have no
// parametric mesher, so they triangulate freeform — their borders must
// still conform vertex-for-vertex to the analytic rims (plan §7.1's first
// bite) or the solid leaks at every shared edge.
void testFreeformBorderConformity() {
    std::printf("-- freeform border conformity --\n");
    TopoDS_Shape outer =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                                 20.0, 8.0)
            .Shape();
    TopoDS_Shape bore =
        BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -1), gp_Dir(0, 0, 1)),
                                 12.0, 10.0)
            .Shape();
    TopoDS_Shape washer = BRepAlgoAPI_Cut(outer, bore).Shape();
    std::string stepPath = tmpPath("weft_test_washer.step");
    weft::writeStep(washer, stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.radial = 24;
    gs.defaults.quadDominant = true;  // subdivision midpoints conform too
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    int annulus = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::AnnulusRing) ++annulus;
    }
    CHECK(annulus >= 2);           // the two flat rings
    CHECK(mesh.countQuads() >= 2 * 24);  // ...as pure quad rings
    CHECK(isWatertight(mesh));

    // Pure-triangle fallback conforms too.
    gs.defaults.quadDominant = false;
    weft::PolyMesh triMesh = weft::generate(model, a, gs);
    CHECK(isWatertight(triMesh));
}

// Unlinked rims: a revolution band whose two rims carry different counts
// meshes as a triangulated taper and the solid stays watertight.
void testUnlinkedRims() {
    std::printf("-- unlinked rims --\n");
    std::string stepPath = tmpPath("weft_test_rims.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    int side = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) side = f.id;
    }
    weft::GenerationSettings gs;
    gs.defaults.radial = 12;
    gs.perFace[side] = gs.defaults;
    gs.perFace[side].linkRims = false;

    weft::GenerationReport report;
    weft::PolyMesh linked = weft::generate(model, a, gs, &report);
    CHECK(isWatertight(linked));  // equal rims: still the quad band
    CHECK(report.faceRims.count(side) == 1);

    // Pin one rim higher: taper (12+18 triangles), still watertight, and
    // the caps follow their own rims (12-gon and 18-gon).
    gs.perEdge[report.faceRims[side][1]] = 18;
    weft::PolyMesh tapered = weft::generate(model, a, gs);
    CHECK(isWatertight(tapered));
    CHECK_EQ(tapered.countTris(), 12 + 18);
    CHECK_EQ(tapered.countNgons(), 2);
}

// A bolt-hole plate (box minus two bores): its top/bottom faces carry three
// wires each, beyond what ring-junction or annulus handle, so the plate-web
// mesher takes them — a quad collar around every hole plus an ear-clipped
// web, with all borders on the B-rep curves so the solid stays watertight.
void testPlateWeb() {
    std::printf("-- plate web --\n");
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(60.0, 30.0, 5.0).Shape();
    for (double x : {18.0, 42.0}) {
        TopoDS_Shape bore =
            BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(x, 15.0, -1.0), gp_Dir(0, 0, 1)), 5.0, 7.0)
                .Shape();
        plate = BRepAlgoAPI_Cut(plate, bore).Shape();
    }
    std::string stepPath = tmpPath("weft_test_plate.step");
    weft::writeStep(plate, stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.radial = 12;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    int plateWebs = 0, revolutions = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::PlateWeb) ++plateWebs;
        if (kind == weft::MesherKind::RevolutionGrid) ++revolutions;
    }
    CHECK_EQ(plateWebs, 2);    // top + bottom of the plate
    CHECK(revolutions >= 2);   // the two bore walls
    // Each plate face collars both holes: 2 faces x 2 holes x 12 quads,
    // plus the bore walls (12 each) and whatever the box sides add.
    CHECK(mesh.countQuads() >= 2 * 2 * 12 + 2 * 12);
    CHECK(isWatertight(mesh));

    // The bore drives its hole: pin one bore's radial higher and the plate
    // collars must follow it, staying watertight.
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) {
            gs.perFace[f.id] = gs.defaults;
            gs.perFace[f.id].radial = 20;
            break;
        }
    }
    weft::PolyMesh pinned = weft::generate(model, a, gs);
    CHECK(isWatertight(pinned));
    CHECK(pinned.countQuads() > mesh.countQuads());
}

// Vertex nudges anchor to the B-rep (face id + surface params), so they
// stay exactly on the CAD surface and re-apply identically after any
// regeneration — the app's G-grab records exactly this op.
void testNudgeVertex() {
    std::printf("-- nudge vertex --\n");
    std::string stepPath = tmpPath("weft_test_nudge.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    int side = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) side = f.id;
    }
    weft::GenerationSettings gs;
    gs.defaults.radial = 12;
    gs.defaults.axial = 2;
    weft::PolyMesh mesh = weft::generate(model, a, gs);

    size_t src = mesh.vertexCount();
    for (size_t v = 0; v < mesh.vertexCount(); ++v) {
        if (mesh.anchors[v].faceId == side) { src = v; break; }
    }
    CHECK(src < mesh.vertexCount());
    std::array<double, 3> before = mesh.vertices[src];

    weft::ManualOp op;
    op.kind = weft::ManualOp::Kind::NudgeVertex;
    op.faceId = side;
    op.u = mesh.anchors[src].u;
    op.v = mesh.anchors[src].v;
    op.u2 = op.u + 0.25;  // rotate around the axis: stays on the wall
    op.v2 = op.v;
    CHECK_EQ(weft::nudgeVertex(mesh, model, op), 1);

    auto radiusOf = [](const std::array<double, 3>& p) {
        return std::sqrt(p[0] * p[0] + p[1] * p[1]);
    };
    CHECK(std::abs(radiusOf(mesh.vertices[src]) - 10.0) < 1e-9);  // on-face
    double moved = std::hypot(mesh.vertices[src][0] - before[0],
                              mesh.vertices[src][1] - before[1]);
    CHECK(moved > 1.0);
    CHECK(isWatertight(mesh));  // connectivity untouched

    // Replay determinism: applyOps on a fresh generate lands the same
    // vertex at the same place.
    weft::PolyMesh fresh = weft::generate(model, a, gs);
    weft::applyOps(fresh, model, {op});
    CHECK(std::abs(fresh.vertices[src][0] - mesh.vertices[src][0]) < 1e-12);
    CHECK(std::abs(fresh.vertices[src][1] - mesh.vertices[src][1]) < 1e-12);

    // Recipe round-trip keeps the op.
    weft::Recipe recipe;
    recipe.settings = gs;
    recipe.ops.push_back(op);
    std::string recipePath = tmpPath("weft_test_nudge.recipe");
    weft::saveRecipe(recipe, recipePath);
    weft::Recipe loaded = weft::loadRecipe(recipePath);
    CHECK_EQ(loaded.ops.size(), 1);
    CHECK(loaded.ops[0].kind == weft::ManualOp::Kind::NudgeVertex);
    CHECK(std::abs(loaded.ops[0].u2 - op.u2) < 1e-15);
    CHECK(std::abs(loaded.ops[0].v2 - op.v2) < 1e-15);
}

// The generation cache must be invisible: cached regenerates match fresh
// ones exactly, including after a single face's settings change.
void testGenerationCache() {
    std::printf("-- generation cache --\n");
    std::string stepPath = tmpPath("weft_test_cache.step");
    weft::writeStep(weft::makeFixture("boss"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.radial = 14;
    weft::GenerationCache cache;
    weft::PolyMesh first = weft::generate(model, a, gs, nullptr, &cache);
    weft::PolyMesh again = weft::generate(model, a, gs, nullptr, &cache);
    CHECK_EQ(again.vertexCount(), first.vertexCount());
    CHECK_EQ(again.polygonCount(), first.polygonCount());
    CHECK(isWatertight(again));

    // Change one face; the cached result must equal a cache-less one.
    gs.perFace[1] = gs.defaults;
    gs.perFace[1].gridU = 3;
    gs.perFace[1].gridV = 3;
    weft::PolyMesh cachedRun = weft::generate(model, a, gs, nullptr, &cache);
    weft::PolyMesh freshRun = weft::generate(model, a, gs);
    CHECK_EQ(cachedRun.vertexCount(), freshRun.vertexCount());
    CHECK_EQ(cachedRun.polygonCount(), freshRun.polygonCount());
    CHECK(isWatertight(cachedRun));
}

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
    RUN(testBridge);
    RUN(testFreeformBorderConformity);
    RUN(testUnlinkedRims);
    RUN(testPlateWeb);
    RUN(testNudgeVertex);
    RUN(testGenerationCache);
    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
