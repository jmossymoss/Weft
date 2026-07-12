// End-to-end test for the phase-0 loop: fixture STEP → import → analyze →
// generate with exact division controls → OBJ. No test framework; each CHECK
// prints and exits non-zero on failure so CTest reports it.

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/io/system.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"
#include "weft/remap.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRep_Builder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Ax2.hxx>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <thread>

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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gsOverride.defaults.minimal = false;  // legacy dense-flat counts
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
    gsEdge.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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

    // All-minimal box: with nothing else driving the edges, every border
    // solves to 1 and the box collapses to its 6 corner quads — the
    // game-topology "flat panel needs no interior" case at its extreme.
    weft::GenerationSettings gsAll;
    gsAll.defaults.minimal = false;  // legacy dense-flat counts
    gsAll.defaults.gridU = 3;
    gsAll.defaults.gridV = 3;
    gsAll.defaults.minimal = true;
    weft::PolyMesh minimalMesh = weft::generate(model, a, gsAll);
    CHECK_EQ(minimalMesh.countNgons(), 0);
    CHECK_EQ(minimalMesh.countQuads(), 6);
    CHECK_EQ(minimalMesh.vertexCount(), 8);
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
    gs.defaults.gridU = 4;
    gs.defaults.gridV = 4;
    gs.defaults.filletLoops = 5;
    gs.defaults.quadDominant = true;  // exercise guided pairing explicitly
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    // Curved fillet strips now take the Coons patch (border rows on
    // the 3D edge curves) instead of a surface-sampled grid.
    CHECK(report.faceMesher[filletFaceId] == weft::MesherKind::CoonsGrid);
    std::map<int, int> polysPerFace;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        ++polysPerFace[mesh.polygonFaceId[p]];
    }
    CHECK_EQ(polysPerFace[filletFaceId], 5 * 4);

    // The notched end faces can't take a grid; with quad-dominant set
    // they now take the structured quad-fill (interior quad grid + a thin
    // conforming rim web) instead of triangulate-and-pair. Interior quads,
    // rim polygons conforming to the neighbours, and the solid watertight.
    int quadFillFaces = 0;
    size_t qdPolys = 0, qdQuads = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::QuadFill) continue;
        ++quadFillFaces;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            ++qdPolys;
            if (mesh.polygons[p].size() == 4) ++qdQuads;
        }
    }
    CHECK_EQ(quadFillFaces, 2);
    CHECK(qdQuads > 0);          // the interior grid is quads
    CHECK(qdPolys >= qdQuads);   // plus the conforming rim
    CHECK(isWatertight(mesh));   // ...which is the point: no leaks

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
    gs.weldTolerance = 0.02;  // surfaced global weld
    weft::FaceMeshSettings dense = gs.defaults;
    dense.radial = 40;
    dense.gridV = 7;
    dense.weldTolerance = 0.005;  // per-face override
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
    CHECK(std::abs(loaded.settings.weldTolerance - 0.02) < 1e-9);
    CHECK(std::abs(loaded.settings.perFace[3].weldTolerance - 0.005) < 1e-9);
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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

    // V spans: the same bridge with 3 rows across emits 3x the quads and
    // stays watertight.
    {
        weft::ManualOp spanned = bridge;
        spanned.spans = 3;
        weft::PolyMesh multi = open;
        CHECK(weft::bridgeLoops(multi, model, spanned) > 0);
        CHECK(isWatertight(multi));
        CHECK_EQ(multi.countQuads(), 12 * 3);
    }

    // Unequal rim counts: pin one rim to 18. The strip triangulates
    // (12 + 18 edges -> 30 triangles) and still closes watertight.
    gs.perEdge[rims[1]] = 18;
    weft::PolyMesh open2 = weft::generate(model, a, gs);
    CHECK(!isWatertight(open2));
    weft::PolyMesh closed2 = open2;
    const size_t beforeZip = open2.polygonCount();
    CHECK_EQ(weft::bridgeLoops(closed2, model, bridge), 30);
    CHECK(isWatertight(closed2));

    // No fan collapse: the arc-fraction zipper advances whichever rail
    // is proportionally behind, so no vertex absorbs more than its share
    // of the strip. (Distance-greedy could consume one rail whole and
    // fan the remainder around a single vertex.)
    {
        std::map<uint32_t, int> uses;
        for (size_t pi = beforeZip; pi < closed2.polygonCount(); ++pi) {
            for (uint32_t v : closed2.polygons[pi]) ++uses[v];
        }
        int maxUse = 0;
        for (const auto& [v, c] : uses) maxUse = std::max(maxUse, c);
        CHECK(maxUse <= 5);  // 12-vs-18: proportional share is 3-4
    }

    // Twisting the pairing keeps the strip watertight (any rotation of
    // the rails is still a closed strip) and rotates the rail seams.
    for (int twist : {1, -2, 7}) {
        weft::ManualOp twisted = bridge;
        twisted.twist = twist;
        weft::PolyMesh tw = open;
        CHECK_EQ(weft::bridgeLoops(tw, model, twisted), 12);
        CHECK(isWatertight(tw));
    }

    // Per-side twist: A counter-rotates against B and each side keeps
    // its own value — matching them cancels back to the automatic
    // alignment exactly.
    {
        weft::ManualOp both = bridge;
        both.twist = 2;
        both.twistA = 2;  // net zero
        weft::PolyMesh twBoth = open;
        CHECK_EQ(weft::bridgeLoops(twBoth, model, both), 12);
        CHECK(isWatertight(twBoth));
        weft::ManualOp none = bridge;
        none.twist = 0;
        weft::PolyMesh twNone = open;
        CHECK_EQ(weft::bridgeLoops(twNone, model, none), 12);
        CHECK(twBoth.polygons == twNone.polygons);
        // And a lone A twist really rotates (differs from no twist).
        weft::ManualOp onlyA = none;
        onlyA.twistA = 1;
        weft::PolyMesh twA = open;
        CHECK_EQ(weft::bridgeLoops(twA, model, onlyA), 12);
        CHECK(isWatertight(twA));
        CHECK(twA.polygons != twNone.polygons);
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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

// Auto-mesher gates: a plate with a slot has "two wires" but is NOT an
// annulus and its hole is NOT collar material — on auto it must stay with
// the fallback, while forcing plate-web or minimal-ngon still builds.
void testAutoGates() {
    std::printf("-- auto-mesher gates (slotted plate) --\n");
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(80.0, 30.0, 5.0).Shape();
    TopoDS_Shape slot =
        BRepPrimAPI_MakeBox(gp_Ax2(gp_Pnt(25.0, 12.0, -1.0), gp_Dir(0, 0, 1)),
                            30.0, 6.0, 7.0)
            .Shape();
    plate = BRepAlgoAPI_Cut(plate, slot).Shape();
    std::string stepPath = tmpPath("weft_test_slot.step");
    weft::writeStep(plate, stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    // The two slotted plate faces: planar, two wires each.
    std::vector<int> plateFaces;
    for (const auto& f : a.faces) {
        if (f.type != weft::SurfaceType::Plane) continue;
        int wires = 0;
        for (TopExp_Explorer wx(model.faces(f.id), TopAbs_WIRE); wx.More();
             wx.Next()) {
            ++wires;
        }
        if (wires == 2) plateFaces.push_back(f.id);
    }
    CHECK_EQ(plateFaces.size(), 2);

    weft::GenerationSettings gs;
    gs.defaults.minimal = false;  // legacy dense-flat counts
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);
    for (int fid : plateFaces) {
        weft::MesherKind k = report.faceMesher.at(fid);
        CHECK(k != weft::MesherKind::AnnulusRing);  // not a ring
        CHECK(k != weft::MesherKind::PlateWeb);     // hole isn't round
    }
    CHECK(isWatertight(mesh));

    // Forcing still builds: plate-web and minimal-ngon on the same faces.
    weft::GenerationSettings gsForce;
    gsForce.defaults.minimal = false;  // legacy dense-flat counts
    gsForce.perFace[plateFaces[0]] = gsForce.defaults;
    gsForce.perFace[plateFaces[0]].forceMesher =
        1 + int(weft::MesherKind::PlateWeb);
    weft::GenerationReport repForce;
    weft::PolyMesh forced = weft::generate(model, a, gsForce, &repForce);
    CHECK(repForce.faceMesher.at(plateFaces[0]) ==
          weft::MesherKind::PlateWeb);
    CHECK(isWatertight(forced));

    // Minimal everywhere: both plate faces become flat hole-bridged webs
    // with zero interior vertices, and the solid still welds.
    weft::GenerationSettings gsMin;
    gsMin.defaults.minimal = false;  // legacy dense-flat counts
    gsMin.defaults.minimal = true;
    weft::GenerationReport repMin;
    weft::PolyMesh minimal = weft::generate(model, a, gsMin, &repMin);
    for (int fid : plateFaces) {
        CHECK(repMin.faceMesher.at(fid) == weft::MesherKind::MinimalNGon);
    }
    CHECK(isWatertight(minimal));
}

// Adaptive density: with `adaptive` set, an edge's count comes from its
// curvature under the chord/angle tolerances — a big cylinder solves with
// more radial segments than a small one, straight edges stay at their
// floors, and explicit per-edge pins still win over everything.
void testAdaptiveDensity() {
    std::printf("-- adaptive density --\n");
    auto radialOf = [&](double radius, int* outCount) {
        TopoDS_Shape cyl =
            BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), radius, 8.0)
                .Shape();
        std::string path = tmpPath("weft_test_adaptive.step");
        weft::writeStep(cyl, path);
        weft::Model model = weft::loadStep(path);
        weft::Analysis a = weft::analyze(model);
        weft::GenerationSettings gs;
        gs.defaults.minimal = false;  // legacy dense-flat counts
        gs.defaults.adaptive = true;
        weft::GenerationReport report;
        weft::PolyMesh mesh = weft::generate(model, a, gs, &report);
        CHECK(isWatertight(mesh));
        int side = 0;
        for (const auto& f : a.faces) {
            if (f.type == weft::SurfaceType::Cylinder) side = f.id;
        }
        auto rims = report.faceRims.find(side);
        CHECK(rims != report.faceRims.end());
        *outCount = report.edgeDivisions.at(rims->second[0]);
        return model;
    };
    // NB: not "small"/"large" — Windows' rpcndr.h #defines those.
    int smallCount = 0, largeCount = 0;
    radialOf(4.0, &smallCount);
    weft::Model largeModel = radialOf(60.0, &largeCount);
    CHECK(smallCount >= 6);          // closed-ring floor holds
    CHECK(largeCount > smallCount);  // curvature drives the count up with size
    CHECK(largeCount <= 256);        // ...within the cap

    // A per-edge pin still beats the adaptive proposal.
    weft::Analysis a = weft::analyze(largeModel);
    weft::GenerationSettings gs;
    gs.defaults.minimal = false;  // legacy dense-flat counts
    gs.defaults.adaptive = true;
    weft::GenerationReport rep;
    weft::generate(largeModel, a, gs, &rep);
    int rim = 0;
    for (const auto& [fid, rims] : rep.faceRims) rim = rims[0];
    CHECK(rim > 0);
    gs.perEdge[rim] = 14;
    weft::GenerationReport pinnedRep;
    weft::PolyMesh pinned = weft::generate(largeModel, a, gs, &pinnedRep);
    CHECK_EQ(pinnedRep.edgeDivisions.at(rim), 14);
    CHECK(isWatertight(pinned));

    // Plate boundary control: the two-bore plate's outer loop takes a
    // pinned TOTAL, distributed by edge length, and the walls follow.
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(60.0, 30.0, 5.0).Shape();
    for (double x : {18.0, 42.0}) {
        TopoDS_Shape bore =
            BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(x, 15.0, -1.0), gp_Dir(0, 0, 1)), 5.0, 7.0)
                .Shape();
        plate = BRepAlgoAPI_Cut(plate, bore).Shape();
    }
    std::string platePath = tmpPath("weft_test_boundary.step");
    weft::writeStep(plate, platePath);
    weft::Model plateModel = weft::loadStep(platePath);
    weft::Analysis plateA = weft::analyze(plateModel);
    weft::GenerationSettings pgs;
    pgs.defaults.minimal = false;  // exercise the plate-web pattern
    pgs.defaults.radial = 12;
    weft::GenerationReport prep;
    weft::generate(plateModel, plateA, pgs, &prep);
    int plateFace = 0;
    for (const auto& [fid, kind] : prep.faceMesher) {
        if (kind == weft::MesherKind::PlateWeb) plateFace = fid;
    }
    CHECK(plateFace > 0);
    pgs.perFace[plateFace] = pgs.defaults;
    pgs.perFace[plateFace].boundary = 24;
    weft::GenerationReport boundedRep;
    weft::PolyMesh bounded =
        weft::generate(plateModel, plateA, pgs, &boundedRep);
    int outerSum = 0;
    TopoDS_Wire outerWire =
        BRepTools::OuterWire(TopoDS::Face(plateModel.faces(plateFace)));
    for (TopExp_Explorer ex(outerWire, TopAbs_EDGE); ex.More(); ex.Next()) {
        int eid = plateModel.edges.FindIndex(ex.Current());
        outerSum += boundedRep.edgeDivisions.at(eid);
    }
    CHECK_EQ(outerSum, 24);
    CHECK(isWatertight(bounded));
}

// Polygon surgery + collar rings: DeletePoly removes exactly one polygon
// (replayable via its world centroid), and plate-web's junction rings
// multiply the concentric quad collars around each hole.
void testDeletePolyAndCollarRings() {
    std::printf("-- delete poly + collar rings --\n");
    std::string stepPath = tmpPath("weft_test_delpoly.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = false;  // legacy dense-flat counts
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    weft::PolyMesh mesh = weft::generate(model, a, gs);
    size_t before = mesh.polygonCount();

    weft::ManualOp op;
    op.kind = weft::ManualOp::Kind::DeletePoly;
    for (uint32_t v : mesh.polygons[0]) {  // centroid of polygon 0
        op.u += mesh.vertices[v][0] / mesh.polygons[0].size();
        op.v += mesh.vertices[v][1] / mesh.polygons[0].size();
        op.t += mesh.vertices[v][2] / mesh.polygons[0].size();
    }
    CHECK_EQ(weft::deletePoly(mesh, op), 1);
    CHECK_EQ(mesh.polygonCount(), before - 1);
    CHECK(!isWatertight(mesh));  // one open ring where the poly was

    // ...which the fill op can then cap again.
    weft::ManualOp fill;
    fill.kind = weft::ManualOp::Kind::FillLoop;
    fill.edgeA = a.faces[0].edgeIds[0];
    CHECK_EQ(weft::fillLoop(mesh, model, fill), 1);
    CHECK(isWatertight(mesh));

    // Recipe round trip for the delete op.
    weft::Recipe recipe;
    recipe.ops.push_back(op);
    std::string recipePath = tmpPath("weft_test_delpoly.recipe");
    weft::saveRecipe(recipe, recipePath);
    weft::Recipe loaded = weft::loadRecipe(recipePath);
    CHECK_EQ(loaded.ops.size(), 1);
    CHECK(loaded.ops[0].kind == weft::ManualOp::Kind::DeletePoly);
    CHECK(std::abs(loaded.ops[0].u - op.u) < 1e-12);

    // Collar rings: the two-bore plate with 3 junction rings grows two
    // extra quad rings per hole per plate face over the single-ring run.
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(60.0, 30.0, 5.0).Shape();
    for (double x : {18.0, 42.0}) {
        TopoDS_Shape bore =
            BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(x, 15.0, -1.0), gp_Dir(0, 0, 1)), 5.0, 7.0)
                .Shape();
        plate = BRepAlgoAPI_Cut(plate, bore).Shape();
    }
    std::string platePath = tmpPath("weft_test_rings.step");
    weft::writeStep(plate, platePath);
    weft::Model plateModel = weft::loadStep(platePath);
    weft::Analysis plateA = weft::analyze(plateModel);
    weft::GenerationSettings one;
    one.defaults.minimal = false;  // exercise the collar-ring pattern
    one.defaults.radial = 12;
    weft::PolyMesh oneRing = weft::generate(plateModel, plateA, one);
    weft::GenerationSettings three = one;
    three.defaults.junctionRings = 3;
    weft::PolyMesh threeRings = weft::generate(plateModel, plateA, three);
    CHECK(isWatertight(oneRing));
    CHECK(isWatertight(threeRings));
    // 2 faces x 2 holes x 2 extra rings x 12 quads
    CHECK_EQ(threeRings.countQuads(), oneRing.countQuads() + 2 * 2 * 2 * 12);
}

// Quad-fill: a slotted plate with quad-dominant set gets an interior quad
// grid joined to the exact boundary by a rim web — mostly quads, fully
// watertight, instead of the fan triangulations of triangulate-and-pair.
void testQuadFill() {
    std::printf("-- quad fill (slotted plate) --\n");
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(80.0, 30.0, 5.0).Shape();
    TopoDS_Shape slot =
        BRepPrimAPI_MakeBox(gp_Ax2(gp_Pnt(25.0, 12.0, -1.0), gp_Dir(0, 0, 1)),
                            30.0, 6.0, 7.0)
            .Shape();
    plate = BRepAlgoAPI_Cut(plate, slot).Shape();
    std::string stepPath = tmpPath("weft_test_quadfill.step");
    weft::writeStep(plate, stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.minimal = false;  // legacy dense-flat counts
    gs.defaults.quadDominant = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, a, gs, &report);

    int quadFill = 0;
    size_t fillQuads = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::QuadFill) continue;
        ++quadFill;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] == fid &&
                mesh.polygons[p].size() == 4) {
                ++fillQuads;
            }
        }
    }
    CHECK(quadFill >= 2);      // the two slotted plate faces
    CHECK(fillQuads >= 2 * 20);  // real interior grids, not fans
    CHECK(isWatertight(mesh));
}

// Same-loop bridge + fill: deleting a face leaves ONE boundary loop; the
// bridge must split it between the two picked edges (a band whose rims
// merged), and the fill tool must cap it with a single n-gon.
void testSameLoopBridgeAndFill() {
    std::printf("-- same-loop bridge + fill --\n");
    std::string stepPath = tmpPath("weft_test_fill.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.minimal = false;  // legacy dense-flat counts
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    gs.perFace[1] = gs.defaults;
    gs.perFace[1].exclude = true;  // one open 12-vertex boundary loop

    // Two opposite edges of the deleted face (max midpoint distance).
    const auto& edgeIds = a.faces[0].edgeIds;
    auto midOf = [&](int eid) {
        BRepAdaptor_Curve c(TopoDS::Edge(model.edges(eid)));
        return c.Value((c.FirstParameter() + c.LastParameter()) / 2);
    };
    int eA = 0, eB = 0;
    double far2 = -1;
    for (size_t i = 0; i < edgeIds.size(); ++i) {
        for (size_t j = i + 1; j < edgeIds.size(); ++j) {
            double d = midOf(edgeIds[i]).Distance(midOf(edgeIds[j]));
            if (d > far2) {
                far2 = d;
                eA = edgeIds[i];
                eB = edgeIds[j];
            }
        }
    }
    CHECK(eA > 0 && eB > 0 && eA != eB);

    weft::PolyMesh open = weft::generate(model, a, gs);
    CHECK(!isWatertight(open));  // the hole is real

    weft::ManualOp bridge;
    bridge.kind = weft::ManualOp::Kind::Bridge;
    bridge.edgeA = eA;
    bridge.edgeB = eB;
    weft::PolyMesh bridged = open;
    CHECK(weft::bridgeLoops(bridged, model, bridge) > 0);
    CHECK(isWatertight(bridged));

    // Fill: one n-gon closes the same hole.
    weft::ManualOp fill;
    fill.kind = weft::ManualOp::Kind::FillLoop;
    fill.edgeA = eA;
    weft::PolyMesh filled = open;
    size_t before = filled.polygonCount();
    CHECK_EQ(weft::fillLoop(filled, model, fill), 1);
    CHECK_EQ(filled.polygonCount(), before + 1);
    CHECK(isWatertight(filled));

    // The fill op persists through a recipe round trip.
    weft::Recipe recipe;
    recipe.settings = gs;
    recipe.ops.push_back(fill);
    std::string recipePath = tmpPath("weft_test_fill.recipe");
    weft::saveRecipe(recipe, recipePath);
    weft::Recipe loaded = weft::loadRecipe(recipePath);
    CHECK_EQ(loaded.ops.size(), 1);
    CHECK(loaded.ops[0].kind == weft::ManualOp::Kind::FillLoop);
    CHECK_EQ(loaded.ops[0].edgeA, eA);
}

// Recipe remapping: a re-export after upstream CAD edits renumbers faces
// and edges; remapRecipe must follow the features geometrically so the
// user's overrides land on the same bore, not the same file index.
void testRecipeRemap() {
    std::printf("-- recipe remap --\n");
    auto makePlate = [&](bool extraBore) {
        TopoDS_Shape plate = BRepPrimAPI_MakeBox(60.0, 30.0, 5.0).Shape();
        if (extraBore) {  // cut FIRST so downstream face ids all shift
            TopoDS_Shape mid =
                BRepPrimAPI_MakeCylinder(
                    gp_Ax2(gp_Pnt(30.0, 15.0, -1.0), gp_Dir(0, 0, 1)), 3.0,
                    7.0)
                    .Shape();
            plate = BRepAlgoAPI_Cut(plate, mid).Shape();
        }
        for (double x : {18.0, 42.0}) {
            TopoDS_Shape bore =
                BRepPrimAPI_MakeCylinder(
                    gp_Ax2(gp_Pnt(x, 15.0, -1.0), gp_Dir(0, 0, 1)), 5.0,
                    7.0)
                    .Shape();
            plate = BRepAlgoAPI_Cut(plate, bore).Shape();
        }
        return plate;
    };
    std::string pathA = tmpPath("weft_test_remap_a.step");
    std::string pathB = tmpPath("weft_test_remap_b.step");
    weft::writeStep(makePlate(false), pathA);
    weft::writeStep(makePlate(true), pathB);
    weft::Model modelA = weft::loadStep(pathA);
    weft::Model modelB = weft::loadStep(pathB);
    weft::Analysis aA = weft::analyze(modelA);
    weft::Analysis aB = weft::analyze(modelB);

    // The bore wall at x=18 in A, found geometrically (radius 5, and its
    // rims' vertices sit around x=18).
    auto boreAt = [&](const weft::Model& m, const weft::Analysis& a,
                      double x) {
        for (const auto& f : a.faces) {
            if (f.type != weft::SurfaceType::Cylinder) continue;
            if (std::abs(f.radius - 5.0) > 1e-6) continue;
            BRepAdaptor_Surface s(TopoDS::Face(m.faces(f.id)));
            gp_Pnt p = s.Value(
                (s.FirstUParameter() + s.LastUParameter()) / 2,
                (s.FirstVParameter() + s.LastVParameter()) / 2);
            double cx = p.X();  // wall point; centre is within radius
            if (std::abs(cx - x) < 5.5) return f.id;
        }
        return 0;
    };
    int boreA = boreAt(modelA, aA, 18.0);
    CHECK(boreA > 0);

    weft::Recipe recipe;
    recipe.settings.perFace[boreA] = recipe.settings.defaults;
    recipe.settings.perFace[boreA].radial = 20;
    int rimA = aA.faces[boreA - 1].edgeIds[0];
    recipe.settings.perEdge[rimA] = 20;
    weft::ManualOp op;
    op.kind = weft::ManualOp::Kind::NudgeVertex;
    op.faceId = boreA;
    recipe.ops.push_back(op);

    // Identity: remapping onto the same model keeps every id.
    weft::RemapReport idRep;
    weft::Recipe same =
        weft::remapRecipe(recipe, modelA, aA, modelA, aA, &idRep);
    CHECK_EQ(idRep.faceMap.at(boreA), boreA);
    CHECK_EQ(same.settings.perFace.count(boreA), 1);
    CHECK_EQ(idRep.facesDropped + idRep.edgesDropped + idRep.opsDropped, 0);

    // Real edit: the extra bore shifts ids in B; the override must follow
    // the geometry to B's x=18 bore.
    weft::RemapReport rep;
    weft::Recipe moved =
        weft::remapRecipe(recipe, modelA, aA, modelB, aB, &rep);
    int boreB = boreAt(modelB, aB, 18.0);
    CHECK(boreB > 0);
    CHECK_EQ(rep.faceMap.at(boreA), boreB);
    CHECK_EQ(moved.settings.perFace.count(boreB), 1);
    CHECK_EQ(moved.settings.perFace.at(boreB).radial, 20);
    CHECK_EQ(moved.settings.perEdge.size(), 1);
    CHECK_EQ(moved.ops.size(), 1);
    CHECK_EQ(moved.ops[0].faceId, boreB);
    CHECK_EQ(rep.facesDropped + rep.edgesDropped + rep.opsDropped, 0);

    // The remapped recipe generates cleanly on the new model, with the
    // followed override driving its bore.
    weft::GenerationReport gen;
    weft::PolyMesh mesh =
        weft::generate(modelB, aB, moved.settings, &gen);
    CHECK(isWatertight(mesh));
    CHECK(gen.faceMesher.at(boreB) == weft::MesherKind::RevolutionGrid);
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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
    gs.defaults.minimal = false;  // legacy dense-flat counts
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

// Top-level generation is a library API and may be called by independent
// workers. Seam mode must belong to each invocation; one caller must not
// change another caller's planning or emission decisions.
void testConcurrentGenerationSettings() {
    std::printf("-- concurrent generation settings --\n");
    std::string stepPath = tmpPath("weft_test_concurrent.step");
    weft::writeStep(weft::makeFixture("demo"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings coupled;
    coupled.parallelMeshing = false;
    weft::GenerationSettings decoupled = coupled;
    decoupled.decoupleSeams = true;

    const weft::PolyMesh coupledBase = weft::generate(model, a, coupled);
    const weft::PolyMesh decoupledBase = weft::generate(model, a, decoupled);
    CHECK(coupledBase.polygonCount() != decoupledBase.polygonCount());

    for (int pass = 0; pass < 4; ++pass) {
        std::atomic<int> ready{0};
        std::atomic<bool> go{false};
        weft::PolyMesh coupledRun, decoupledRun;
        auto run = [&](const weft::GenerationSettings& settings,
                       weft::PolyMesh& result) {
            ready.fetch_add(1, std::memory_order_release);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            result = weft::generate(model, a, settings);
        };
        std::thread aThread(run, std::cref(coupled),
                            std::ref(coupledRun));
        std::thread bThread(run, std::cref(decoupled),
                            std::ref(decoupledRun));
        while (ready.load(std::memory_order_acquire) != 2) {
            std::this_thread::yield();
        }
        go.store(true, std::memory_order_release);
        aThread.join();
        bThread.join();

        CHECK_EQ(coupledRun.vertexCount(), coupledBase.vertexCount());
        CHECK_EQ(coupledRun.polygonCount(), coupledBase.polygonCount());
        CHECK_EQ(decoupledRun.vertexCount(), decoupledBase.vertexCount());
        CHECK_EQ(decoupledRun.polygonCount(), decoupledBase.polygonCount());
    }
}

// The high-level conversion path owns the full CAD -> mesh -> writer
// workflow. Multi-body CAD must stay multi-object in DCC-oriented output.
void testCadConversionPreservesObjects() {
    std::printf("-- CAD conversion object grouping --\n");
    BRep_Builder builder;
    TopoDS_Compound assembly;
    builder.MakeCompound(assembly);
    builder.Add(assembly, BRepPrimAPI_MakeBox(1.0, 2.0, 3.0).Shape());
    builder.Add(assembly,
                BRepPrimAPI_MakeBox(gp_Pnt(5.0, 0.0, 0.0),
                                    2.0, 2.0, 2.0).Shape());

    const std::string stepPath = tmpPath("weft_test_convert_objects.step");
    const std::string objPath = tmpPath("weft_test_convert_objects.obj");
    weft::writeStep(assembly, stepPath);

    weft::io::System io;
    weft::io::bootstrapIo(io);
    weft::io::convertFile(io, stepPath, objPath);

    std::ifstream obj(objPath);
    CHECK(obj.good());
    int objects = 0;
    int faceGroups = 0;
    for (std::string line; std::getline(obj, line);) {
        if (line.rfind("o ", 0) == 0) ++objects;
        if (line.rfind("g face_", 0) == 0) ++faceGroups;
    }
    CHECK_EQ(objects, 2);
    CHECK(faceGroups > 0);
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

// Weld: merge picked vertices into one (center/last/first), polygons
// remap and degenerates drop; the op replays from world points and
// round-trips through recipes.
void testWeldTolerance() {
    std::printf("-- weld tolerance (per-vertex, max-wins) --\n");
    // Two quads on two faces whose shared border is NEAR-coincident: the
    // right edge of face 1 and the left edge of face 2 sit `g` apart.
    const double g = 0.01;
    auto makeMesh = []() {
        weft::PolyMesh m;
        m.vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                      {1 + 0.01, 0, 0}, {2, 0, 0}, {2, 1, 0}, {1 + 0.01, 1, 0}};
        m.anchors.resize(8);
        m.polygons = {{0, 1, 2, 3}, {4, 5, 6, 7}};
        m.polygonFaceId = {1, 2};
        return m;
    };
    auto weldWith = [&](double cell, const std::vector<double>* vt) {
        weft::PolyMesh m = makeMesh();
        weft::weldVertices(m, cell, nullptr, vt);
        return m.vertexCount();
    };

    // Control: tight tolerance everywhere leaves the seam split (8 verts).
    std::vector<double> tight(8, 1e-6);
    CHECK_EQ(weldWith(1e-6, &tight), (size_t)8);
    CHECK_EQ(weldWith(1e-6, nullptr), (size_t)8);  // scalar path agrees

    // Loosen ONLY face 1 (verts 0..3) past the gap: the junction closes
    // even though face 2 stayed tight — max-wins (looser side pulls it in).
    std::vector<double> loosA = {0.05, 0.05, 0.05, 0.05, 1e-6, 1e-6, 1e-6, 1e-6};
    CHECK_EQ(weldWith(0.05, &loosA), (size_t)6);
    // Symmetric: loosening ONLY face 2 closes the SAME junction.
    std::vector<double> loosB = {1e-6, 1e-6, 1e-6, 1e-6, 0.05, 0.05, 0.05, 0.05};
    CHECK_EQ(weldWith(0.05, &loosB), (size_t)6);
    // A tolerance BELOW the gap on both sides never merges, regardless of
    // the (larger) hash cell size.
    std::vector<double> sub = {g * 0.5, g * 0.5, g * 0.5, g * 0.5,
                               g * 0.5, g * 0.5, g * 0.5, g * 0.5};
    CHECK_EQ(weldWith(0.05, &sub), (size_t)8);
    (void)g;

    // generate() level: a per-face weld override is scoped and safe — an
    // absurd value never collapses a clean solid into non-manifold soup
    // (clamped to the local mesh resolution).
    std::string stepPath = tmpPath("weft_test_weldtol.step");
    weft::writeStep(weft::makeFixture("boss"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    weft::GenerationSettings gs;
    weft::PolyMesh base = weft::generate(model, a, gs);
    CHECK(isWatertight(base));
    weft::GenerationSettings gpf = gs;
    weft::FaceMeshSettings over;  // absurd per-face weld on face 1
    over.weldTolerance = 1000.0;
    gpf.perFace[1] = over;
    weft::PolyMesh pf = weft::generate(model, a, gpf);
    // Still a valid closed solid (the clamp forbade a runaway collapse),
    // and no MORE vertices than the baseline (weld only ever merges).
    CHECK(isWatertight(pf));
    CHECK(pf.vertexCount() <= base.vertexCount());
}

void testWeldVerts() {
    std::printf("-- weld verts --\n");
    std::string stepPath = tmpPath("weft_test_weld.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = false;  // dense flats: interior verts exist
    gs.defaults.gridU = 4;
    gs.defaults.gridV = 4;
    weft::PolyMesh mesh = weft::generate(model, a, gs);
    CHECK(isWatertight(mesh));

    // Weld two ADJACENT verts (a mesh edge's ends) at their center: the
    // two polygons sharing that edge lose a corner, everything else
    // remaps, and the solid stays closed.
    uint32_t va = 0, vb = 0;
    bool found = false;
    for (size_t pp = 0; pp < mesh.polygons.size() && !found; ++pp) {
        const auto& poly = mesh.polygons[pp];
        if (poly.size() == 4) {
            va = poly[0];
            vb = poly[1];
            found = true;
        }
    }
    CHECK(found);
    weft::ManualOp weld;
    weld.kind = weft::ManualOp::Kind::WeldVerts;
    weld.weldMode = 0;  // center
    weld.weldPoints.push_back(mesh.vertices[va]);
    weld.weldPoints.push_back(mesh.vertices[vb]);
    const size_t polysBefore = mesh.polygonCount();
    weft::PolyMesh welded = mesh;
    CHECK_EQ(weft::weldVerts(welded, weld), 2);
    CHECK(welded.polygonCount() <= polysBefore);
    for (const auto& poly : welded.polygons) {
        std::set<uint32_t> distinct(poly.begin(), poly.end());
        CHECK(distinct.size() >= 3);
        CHECK_EQ(distinct.size(), poly.size());  // no repeated corners
    }
    CHECK(isWatertight(welded));

    // weld-to-last lands exactly on the last pick's position.
    weft::ManualOp toLast = weld;
    toLast.weldMode = 1;
    weft::PolyMesh w2 = mesh;
    std::array<double, 3> lastPos = mesh.vertices[vb];
    CHECK_EQ(weft::weldVerts(w2, toLast), 2);
    CHECK(w2.vertices[va] == lastPos);

    // Recipe round trip carries mode and points.
    weft::Recipe recipe;
    recipe.ops.push_back(toLast);
    std::string rPath = tmpPath("weft_test_weld.recipe");
    weft::saveRecipe(recipe, rPath);
    weft::Recipe loaded = weft::loadRecipe(rPath);
    CHECK_EQ(loaded.ops.size(), 1);
    CHECK(loaded.ops[0].kind == weft::ManualOp::Kind::WeldVerts);
    CHECK_EQ(loaded.ops[0].weldMode, 1);
    CHECK_EQ(loaded.ops[0].weldPoints.size(), 2);
    CHECK(std::abs(loaded.ops[0].weldPoints[1][0] - lastPos[0]) < 1e-9);
}

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
    RUN(testRecipeRemap);
    RUN(testAutoGates);
    RUN(testAdaptiveDensity);
    RUN(testQuadFill);
    RUN(testDeletePolyAndCollarRings);
    RUN(testSameLoopBridgeAndFill);
    RUN(testWeldTolerance);
    RUN(testWeldVerts);
    RUN(testGenerationCache);
    RUN(testConcurrentGenerationSettings);
    RUN(testCadConversionPreservesObjects);
    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
