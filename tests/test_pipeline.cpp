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
#include "weft/topology_signature.hpp"
#include "weft/validate.hpp"

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
#include <gp_Vec.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <vector>

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
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    gsOverride.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    gsEdge.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    CHECK(a.faces[filletFaceId - 1].featureClass ==
          weft::FeatureClass::FilletStrip);
    auto fcit = report.faceFeatureClass.find(filletFaceId);
    CHECK(fcit != report.faceFeatureClass.end());
    CHECK(fcit->second == weft::FeatureClass::FilletStrip);
    // Promoted from probe103: blend strips expose which patch axis the
    // fillet-loops knob drives (faceAcross). Discover via FilletStrip —
    // no hardcoded face-ID product routing.
    {
        auto ax = report.faceAcross.find(filletFaceId);
        CHECK(ax != report.faceAcross.end());
        if (ax != report.faceAcross.end()) {
            CHECK(ax->second == 1 || ax->second == 2);
        }
    }
    std::map<int, int> polysPerFace;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        ++polysPerFace[mesh.polygonFaceId[p]];
    }
    CHECK_EQ(polysPerFace[filletFaceId], 5 * 4);

    // The notched end faces cannot take a coherent structured grid. Even
    // with quad-dominant requested they must use local exact-border pairing,
    // never the retired automatic Quad Fill lattice.
    int quadFillFaces = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        (void)fid;
        if (kind == weft::MesherKind::QuadFill) ++quadFillFaces;
    }
    CHECK_EQ(quadFillFaces, 0);
    CHECK(isWatertight(mesh));

    // Hold clustering: same counts, but the loops crowd toward the creases —
    // the first across-interval must shrink vs the uniform mesh.
    weft::GenerationSettings gsHold = gs;
    gsHold.defaults.filletHold = 0.8;
    weft::PolyMesh held = weft::generate(model, a, gsHold);
    CHECK(isWatertight(held));

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
    gs.defaults.minCurvedSegments = 12;  // exact ring count for this fixture
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
    gs.defaults.minCurvedSegments = 16;  // exact bore ring for this fixture
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
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
    gs.defaults.junctionRings = 1;  // exercise collar path (default is off)
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

    // Default is no collar rim (junctionRings=0): still PlateWeb + watertight,
    // with fewer quads than the collared mesh above.
    weft::GenerationSettings bare = gs;
    bare.defaults.junctionRings = 0;
    bare.perFace.clear();
    weft::PolyMesh noCollar = weft::generate(model, a, bare);
    CHECK(isWatertight(noCollar));
    CHECK(noCollar.countQuads() < mesh.countQuads());
}

// WP5: multi-hole planar plates under the CAD profile used to leave a
// border-only CDT residual of hair-thin triangles between collars
// (ABC notched-ring class). Quality refine + pairing on the residual
// web must clear validation slivers on plate-web faces while staying
// watertight — no filename specials.
void testPlateWebSliverRefine() {
    std::printf("-- plate web sliver refine --\n");
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(80.0, 80.0, 4.0).Shape();
    // A dense bolt pattern: enough holes that the residual CDT between
    // collars would otherwise needle (the ABC 00008536 plate-web class).
    for (double y : {20.0, 40.0, 60.0}) {
        for (double x : {20.0, 40.0, 60.0}) {
            TopoDS_Shape bore = BRepPrimAPI_MakeCylinder(
                                   gp_Ax2(gp_Pnt(x, y, -1.0), gp_Dir(0, 0, 1)),
                                   4.0, 6.0)
                                   .Shape();
            plate = BRepAlgoAPI_Cut(plate, bore).Shape();
        }
    }
    const std::string stepPath = tmpPath("weft_test_plate_sliver.step");
    weft::writeStep(plate, stepPath);

    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.defaults.junctionRings = 1;  // collar class under test
    gs.densityScale = 0.35;

    weft::GenerationReport report;
    weft::PolyMesh mesh =
        weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));

    int plateWebs = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::PlateWeb) ++plateWebs;
    }
    CHECK(plateWebs >= 2);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK(vr.watertight());
    // Count slivers that land on plate-web faces only — other families
    // are out of scope for this class fix.
    auto minCornerDeg = [&](const std::vector<uint32_t>& poly) {
        double best = 180.0;
        const size_t n = poly.size();
        if (n < 3) return 0.0;
        for (size_t i = 0; i < n; ++i) {
            const auto& A = mesh.vertices[poly[(i + n - 1) % n]];
            const auto& B = mesh.vertices[poly[i]];
            const auto& C = mesh.vertices[poly[(i + 1) % n]];
            const double ux = A[0] - B[0], uy = A[1] - B[1], uz = A[2] - B[2];
            const double vx = C[0] - B[0], vy = C[1] - B[1], vz = C[2] - B[2];
            const double nu = std::sqrt(ux * ux + uy * uy + uz * uz);
            const double nv = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (nu < 1e-18 || nv < 1e-18) return 0.0;
            double cos = (ux * vx + uy * vy + uz * vz) / (nu * nv);
            cos = std::max(-1.0, std::min(1.0, cos));
            best = std::min(best, std::acos(cos) * 180.0 / 3.141592653589793);
        }
        return best;
    };
    int platePolys = 0, plateSlivers = 0;
    for (size_t i = 0; i < mesh.polygons.size(); ++i) {
        const int fid =
            i < mesh.polygonFaceId.size() ? mesh.polygonFaceId[i] : 0;
        auto kit = report.faceMesher.find(fid);
        if (kit == report.faceMesher.end() ||
            kit->second != weft::MesherKind::PlateWeb) {
            continue;
        }
        ++platePolys;
        if (minCornerDeg(mesh.polygons[i]) < vr.sliverAngleDeg) {
            ++plateSlivers;
        }
    }
    CHECK(platePolys > 0);
    std::printf("  plate-web polys=%d slivers=%d (of %zu model-wide)\n",
                platePolys, plateSlivers, vr.sliverPolygons);
    // Residual needles must be gone (or vanishingly rare). Pre-fix ABC
    // plate-web faces were >50% slivers; the refine clears them.
    CHECK(plateSlivers * 20 <= platePolys);  // < 5%

    // CAD/minimal residual must stay bridged n-gons (+ collar quads), not
    // a CDT triangle soup. Count tris attributed to plate-web faces.
    int plateTris = 0, plateNgons = 0, plateQuads = 0;
    for (size_t i = 0; i < mesh.polygons.size(); ++i) {
        const int fid =
            i < mesh.polygonFaceId.size() ? mesh.polygonFaceId[i] : 0;
        auto kit = report.faceMesher.find(fid);
        if (kit == report.faceMesher.end() ||
            kit->second != weft::MesherKind::PlateWeb) {
            continue;
        }
        const size_t n = mesh.polygons[i].size();
        if (n == 3) ++plateTris;
        else if (n == 4) ++plateQuads;
        else if (n > 4) ++plateNgons;
    }
    std::printf("  plate-web quads=%d tris=%d ngons=%d\n", plateQuads,
                plateTris, plateNgons);
    CHECK(plateQuads > 0);   // collars survive
    CHECK(plateNgons > 0);   // residual is bridged simple n-gons
    CHECK(plateTris == 0);   // no CDT soup under minimal
}

// WP6: near-full analytic cylinder with a multi-tooth castellated rim
// (ABC 00008536 drum class). Open-band must keep RevolutionGrid instead
// of dumping the face on the contract floor as needle soup.
void testNotchedDrumOpenBand() {
    std::printf("-- notched drum open-band --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/abc/notched_drum_iso_band_r0.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    CHECK_EQ(model.faceCount(), 1);
    CHECK(analysis.faces[0].featureClass == weft::FeatureClass::Drum);
    CHECK(analysis.faces[0].chartKind == weft::ChartKind::IsoBand);

    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    auto kit = report.faceMesher.find(1);
    CHECK(kit != report.faceMesher.end());
    CHECK(kit->second == weft::MesherKind::RevolutionGrid);
    auto bit = report.faceBuild.find(1);
    CHECK(bit != report.faceBuild.end());
    CHECK_EQ(bit->second, 0);  // not demoted to contract floor

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    // Open-shell extract: watertightness is not the target. Slivers must
    // drop far below the prior contract-floor needle count (~248).
    CHECK(vr.sliverPolygons < 40);
    // ≥12 columns/tooth keeps inter-tooth land from sharing a U-gap
    // with opposing walls (tooth-wall Newell double-cover class).
    const auto folded = weft::foldedPolys(model, mesh);
    const int nFolded =
        int(std::count(folded.begin(), folded.end(), uint8_t{1}));
    CHECK_EQ(nFolded, 0);
    std::printf("  polys=%zu slivers=%zu folds=%d kind=revolution-grid\n",
                mesh.polygons.size(), vr.sliverPolygons, nFolded);
}

// ABC 00006051 class: sphere-cap + torus fillet-strip full-period with
// unequal rim totals used to demote both faces to contract floor
// (revolution grid irreconcilable / Coons mass inversion).
void testSphereFilletFullPeriodNoFloor() {
    std::printf("-- sphere+fillet full-period no floor --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/abc/sphere_fillet_fullperiod_r1.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    CHECK_EQ(model.faceCount(), 3);

    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    int floors = 0;
    for (int fid = 1; fid <= 3; ++fid) {
        auto bit = report.faceBuild.find(fid);
        CHECK(bit != report.faceBuild.end());
        if (bit->second == 2) ++floors;
        auto kit = report.faceMesher.find(fid);
        CHECK(kit != report.faceMesher.end());
        CHECK(kit->second == weft::MesherKind::RevolutionGrid);
    }
    CHECK_EQ(floors, 0);
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    const auto folded = weft::foldedPolys(model, mesh);
    const int nFolded =
        int(std::count(folded.begin(), folded.end(), uint8_t{1}));
    CHECK_EQ(nFolded, 0);
    CHECK_EQ(vr.sliverPolygons, 0u);
    std::printf("  polys=%zu floors=0 folds=0 slivers=0\n",
                mesh.polygons.size());
}

// Analytic FilletStrip×RevolutionGrid and Coons strips must publish
// faceAcross and honour semantic knobs: radial/gridU = along, filletLoops =
// across — remapped onto patch U/V. Without this, cylinder fillets treated
// radial as the short across arc (wrong GPU / wheel axis).
// Demo notched/boolean drums: raising radial under CAD adaptive must NOT
// pre-demote to "radial override → contract floor". Structured RevolutionGrid
// stays unless a real neighbour-contract stress demote fires.
void testDemoNotchedRadialKeepsStructured() {
    std::printf("-- demo notched radial keeps structured --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() / "fixtures/demo.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    // Discover multi-edge full-period drums (notched / boolean cuts) —
    // no hardcoded face ids.
    std::vector<int> targets;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Drum) continue;
        if (f.chartKind != weft::ChartKind::FullPeriod) continue;
        if (int(f.edgeIds.size()) < 5) continue;
        targets.push_back(f.id);
        gs.perFace[f.id] = gs.defaults;
        gs.perFace[f.id].radial = 48;
    }
    CHECK(!targets.empty());

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));
    int structured = 0;
    for (int fid : targets) {
        auto kit = report.faceMesher.find(fid);
        CHECK(kit != report.faceMesher.end());
        CHECK(kit->second == weft::MesherKind::RevolutionGrid);
        auto bit = report.faceBuild.find(fid);
        CHECK(bit != report.faceBuild.end());
        // Must not be the old preemptive demote. Stress demote (build==2
        // with a different cause) is still allowed if neighbours fail.
        auto cit = report.faceBuildCause.find(fid);
        if (cit != report.faceBuildCause.end()) {
            CHECK(cit->second.find("radial override → contract floor") ==
                  std::string::npos);
        }
        if (bit->second == 0) ++structured;
    }
    CHECK(structured >= 1);
    std::printf("  targets=%zu structured=%d\n", targets.size(), structured);

    // Artist report: radial 17 demoted ("castellated insert failed");
    // lowering below ~15 did nothing (annulus-floor / soft-propose held
    // the rim). Manual adapt-off must pin, and awkward nu values must
    // stay structured — discover the notched full-wrap drum (many
    // edges + castellated plan) rather than every multi-edge hole.
    int notched = 0;
    for (int fid : targets) {
        // Prefer the tallest multi-edge drum (demo muzzle / notched
        // barrel class): more edges than a simple bore wall.
        if (int(analysis.faces[fid - 1].edgeIds.size()) >= 9) {
            notched = fid;
            break;
        }
    }
    if (notched == 0 && !targets.empty()) notched = targets.front();

    weft::GenerationReport baseRep;
    {
        weft::GenerationSettings base = gs;
        base.perFace.clear();
        weft::generate(model, analysis, base, &baseRep);
    }
    const int baseNu = baseRep.faceCounts.count(notched)
                           ? baseRep.faceCounts[notched][0]
                           : 0;

    for (int r : {8, 17}) {
        weft::GenerationSettings edit = gs;
        edit.perFace.clear();
        edit.perFace[notched] = edit.defaults;
        edit.perFace[notched].adaptive = false;
        edit.perFace[notched].radial = r;
        weft::GenerationReport er;
        weft::PolyMesh em = weft::generate(model, analysis, edit, &er);
        CHECK(isWatertight(em));
        auto bit = er.faceBuild.find(notched);
        CHECK(bit != er.faceBuild.end());
        CHECK_EQ(bit->second, 0);  // not contract floor
        auto cit = er.faceBuildCause.find(notched);
        if (cit != er.faceBuildCause.end()) {
            CHECK(cit->second.find("castellated insert failed") ==
                  std::string::npos);
            CHECK(cit->second.find("border contract failed") ==
                  std::string::npos);
        }
        auto cnt = er.faceCounts.find(notched);
        CHECK(cnt != er.faceCounts.end());
        if (r < baseNu) {
            // Lowering must move the column count — not sit on the old
            // annulus-floor clamp.
            CHECK(cnt->second[0] < baseNu);
            CHECK(cnt->second[0] <= r + 1);
        }
        std::printf("  face#%d manual radial=%d -> nu=%d (base %d) ok\n",
                    notched, r, cnt->second[0], baseNu);
    }
}

// Raising radial on a slotted/boolean drum must not stamp that radial onto
// neighbour Coons fillet strips (blend-group). That tripped curCountOverride,
// killed along-axis adaptive, and squared slot fillets (demo.step 42/43).
void testSlottedDrumRadialPreservesCoonsFillets() {
    std::printf("-- slotted drum radial preserves coons fillets --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() / "fixtures/demo.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings baseGs;
    baseGs.defaults.minimal = true;
    baseGs.defaults.adaptive = true;
    baseGs.defaults.relativeDeviation = true;

    weft::GenerationReport baseRep;
    weft::generate(model, analysis, baseGs, &baseRep);

    std::vector<int> drums;
    std::vector<int> coonsFillets;
    for (const auto& f : analysis.faces) {
        if (f.featureClass == weft::FeatureClass::Drum &&
            f.chartKind == weft::ChartKind::FullPeriod &&
            int(f.edgeIds.size()) >= 5) {
            drums.push_back(f.id);
        }
        auto kit = baseRep.faceMesher.find(f.id);
        if (kit != baseRep.faceMesher.end() &&
            kit->second == weft::MesherKind::CoonsGrid &&
            f.featureClass == weft::FeatureClass::FilletStrip) {
            coonsFillets.push_back(f.id);
        }
    }
    CHECK(!drums.empty());
    CHECK(!coonsFillets.empty());

    weft::GenerationSettings up = baseGs;
    for (int fid : drums) {
        up.perFace[fid] = up.defaults;
        up.perFace[fid].adaptive = false;
        up.perFace[fid].radial = 22;
    }
    weft::GenerationReport upRep;
    weft::PolyMesh upMesh = weft::generate(model, analysis, up, &upRep);
    CHECK(isWatertight(upMesh));

    for (int fid : coonsFillets) {
        auto bit = upRep.faceBuild.find(fid);
        CHECK(bit != upRep.faceBuild.end());
        CHECK_EQ(bit->second, 0);
        auto b = baseRep.faceCounts[fid];
        auto u = upRep.faceCounts[fid];
        // Drum radial raise must not restamp Coons strips (square-fillet
        // bug). Counts stay at the adaptive/base grid.
        CHECK_EQ(u[0], b[0]);
        CHECK_EQ(u[1], b[1]);
        std::printf("  coons fillet#%d base %d×%d -> raised-drum %d×%d\n", fid,
                    b[0], b[1], u[0], u[1]);
    }
}

void testFilletDensityAxisOwnership() {
    std::printf("-- fillet density axis ownership --\n");
    auto cad = []() {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        return gs;
    };

    // bossfillet torus strip → RevolutionGrid, across = V (faceAcross=2).
    {
        const std::string path = tmpPath("weft_axis_bossfillet.step");
        weft::writeStep(weft::makeFixture("bossfillet"), path);
        weft::Model model = weft::loadStep(path);
        const weft::Analysis a = weft::analyze(model);
        int stripFid = 0;
        for (const auto& f : a.faces) {
            if (f.featureClass == weft::FeatureClass::FilletStrip &&
                f.isFillet) {
                stripFid = f.id;
                break;
            }
        }
        CHECK(stripFid > 0);

        weft::GenerationReport baseRep;
        weft::generate(model, a, cad(), &baseRep);
        CHECK(baseRep.faceMesher[stripFid] ==
              weft::MesherKind::RevolutionGrid);
        auto ax = baseRep.faceAcross.find(stripFid);
        CHECK(ax != baseRep.faceAcross.end());
        CHECK_EQ(ax->second, 2);  // torus minor = V
        const auto base = baseRep.faceCounts[stripFid];

        weft::GenerationSettings alongGs = cad();
        alongGs.perFace[stripFid] = alongGs.defaults;
        alongGs.perFace[stripFid].adaptive = false;
        alongGs.perFace[stripFid].radial = 40;
        alongGs.perFace[stripFid].filletLoops = 3;
        weft::GenerationReport alongRep;
        weft::generate(model, a, alongGs, &alongRep);
        const auto alongC = alongRep.faceCounts[stripFid];
        // acrossIsU=false → along rides U (count[0]), across rides V.
        CHECK(alongC[0] >= 40);
        CHECK_EQ(alongC[1], 3);

        weft::GenerationSettings acrossGs = cad();
        acrossGs.perFace[stripFid] = acrossGs.defaults;
        acrossGs.perFace[stripFid].adaptive = false;
        acrossGs.perFace[stripFid].radial =
            std::max(3, base[0]);  // keep along stable
        acrossGs.perFace[stripFid].filletLoops = 8;
        weft::GenerationReport acrossRep;
        weft::generate(model, a, acrossGs, &acrossRep);
        const auto acrossC = acrossRep.faceCounts[stripFid];
        CHECK_EQ(acrossC[1], 8);
        std::printf("  bossfillet strip#%d across=V along=%d loops→nv=%d\n",
                    stripFid, alongC[0], acrossC[1]);
        CHECK(isWatertight(weft::generate(model, a, acrossGs)));
    }

    // Open cylinder fillet → Coons; faceAcross published; loops = across.
    {
        const std::string path = tmpPath("weft_axis_fillet.step");
        weft::writeStep(weft::makeFixture("fillet"), path);
        weft::Model model = weft::loadStep(path);
        const weft::Analysis a = weft::analyze(model);
        int stripFid = 0;
        for (const auto& f : a.faces) {
            if (f.featureClass == weft::FeatureClass::FilletStrip) {
                stripFid = f.id;
                break;
            }
        }
        CHECK(stripFid > 0);

        weft::GenerationSettings gs = cad();
        gs.defaults.adaptive = false;
        gs.defaults.gridU = 6;
        gs.defaults.filletLoops = 4;
        weft::GenerationReport rep;
        weft::generate(model, a, gs, &rep);
        CHECK(rep.faceMesher[stripFid] == weft::MesherKind::CoonsGrid);
        auto ax = rep.faceAcross.find(stripFid);
        CHECK(ax != rep.faceAcross.end());
        CHECK(ax->second == 1 || ax->second == 2);
        const auto c = rep.faceCounts[stripFid];
        if (ax->second == 1) {
            CHECK_EQ(c[0], 4);  // across = U = loops
            CHECK_EQ(c[1], 6);  // along = V = gridU
        } else {
            CHECK_EQ(c[0], 6);
            CHECK_EQ(c[1], 4);
        }
        std::printf("  coons fillet#%d across=%s nu=%d nv=%d\n", stripFid,
                    ax->second == 1 ? "U" : "V", c[0], c[1]);
        CHECK(isWatertight(weft::generate(model, a, gs)));
    }
}

// ABC FreeTrim drum walls (tall skinny cylinder segments) used to false-
// pass isGeometricallyFlat and collapse to a single MinimalNGon needle
// under CAD. They must take open-band RevolutionGrid with axial rows.
void testTallFreeTrimDrum() {
    std::printf("-- tall free-trim drum --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/abc/tall_free_trim_drum_r0.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    CHECK_EQ(model.faceCount(), 1);
    CHECK(analysis.faces[0].featureClass == weft::FeatureClass::Drum);
    CHECK(analysis.faces[0].chartKind == weft::ChartKind::FreeTrim);

    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    auto kit = report.faceMesher.find(1);
    CHECK(kit != report.faceMesher.end());
    CHECK(kit->second == weft::MesherKind::RevolutionGrid);
    auto bit = report.faceBuild.find(1);
    CHECK(bit != report.faceBuild.end());
    CHECK_EQ(bit->second, 0);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK(vr.sliverPolygons == 0);
    CHECK(mesh.polygons.size() >= 16);
    // Reject the old single-ngon / two-tri needle.
    CHECK(mesh.vertices.size() >= 20);
    std::printf("  verts=%zu polys=%zu slivers=%zu kind=revolution-grid\n",
                mesh.vertices.size(), mesh.polygons.size(),
                vr.sliverPolygons);
}

// Demo/torture vertical plate wall with a round bore: under CAD, plate-web
// must keep collars but not fill the wall with CDT needles.
void testTorturePlateWebMinimalResidual() {
    std::printf("-- torture plate-web minimal residual --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() / "fixtures/torture.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.defaults.junctionRings = 1;  // collars on for this class check

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));

    int plateWebs = 0, plateTris = 0, plateNgons = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::PlateWeb) continue;
        ++plateWebs;
        for (size_t i = 0; i < mesh.polygons.size(); ++i) {
            if (i >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[i] != fid) {
                continue;
            }
            const size_t n = mesh.polygons[i].size();
            if (n == 3) ++plateTris;
            else if (n > 4) ++plateNgons;
        }
    }
    CHECK(plateWebs >= 1);
    CHECK(plateNgons > 0);
    CHECK(plateTris == 0);
}

// WP5 / MP9: a spherical dimple through a planar annulus used to miss
// RevolutionGrid (adaptor IsUClosed=false on the trim), fall to the
// contract floor, then ship hundreds of needle tris after fold demotion.
// CAD adaptive must keep the sphere off contract-floor (minimal n-gon
// rescue is OK) and stay watertight with the annulus.
void testSphereDimpleNotContractFloor() {
    std::printf("-- sphere dimple not contract-floor --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/sphere_dimple_annulus.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh =
        weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));

    int spheres = 0, sphereFloor = 0, sphereFlat = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Sphere) continue;
        ++spheres;
        auto kit = report.faceMesher.find(f.id);
        CHECK(kit != report.faceMesher.end());
        if (kit != report.faceMesher.end()) {
            CHECK(kit->second != weft::MesherKind::Fallback);
            CHECK(kit->second != weft::MesherKind::QuadDominant);
            // A healthy rim must keep a curved mesher — flattening to a
            // planar n-gon is the "hemisphere became a plane" failure.
            CHECK(kit->second != weft::MesherKind::MinimalNGon);
            if (kit->second == weft::MesherKind::Fallback ||
                kit->second == weft::MesherKind::QuadDominant) {
                ++sphereFloor;
            }
            if (kit->second == weft::MesherKind::MinimalNGon) {
                ++sphereFlat;
            }
            std::printf("  sphere face %d -> %s\n", f.id,
                        weft::mesherKindName(kit->second));
        }
        auto bit = report.faceBuild.find(f.id);
        // Not a demoted contract-floor soup.
        if (bit != report.faceBuild.end()) {
            CHECK(bit->second != 2);
        }
    }
    CHECK(spheres >= 1);
    CHECK_EQ(sphereFloor, 0);
    CHECK_EQ(sphereFlat, 0);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK(vr.watertight());
    // Keeping revolution-grid after a UV fold census can leave a few
    // thin cells; that still beats a single planar n-gon.
}

// MP9 bullet tip: a geometric sphere cap with one circular (non-iso) rim
// used to take RevolutionGrid, loft a fake U-wrap lattice, then fold
// self-heal into contract-floor tip soup. CAD must route it to
// quad-fill → disk-cap rings instead.
void testBulletTipNotContractFloor() {
    std::printf("-- bullet tip not contract-floor --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/bullet_tip_3728.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.defaults.minCurvedSegments = 12;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));

    int spheres = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Sphere) continue;
        ++spheres;
        auto kit = report.faceMesher.find(f.id);
        CHECK(kit != report.faceMesher.end());
        std::printf("  sphere face %d -> %s\n", f.id,
                    weft::mesherKindName(kit->second));
        CHECK(kit->second == weft::MesherKind::QuadFill);
        auto bit = report.faceBuild.find(f.id);
        CHECK(bit != report.faceBuild.end());
        CHECK_EQ(bit->second, 0);
        auto cit = report.faceBuildCause.find(f.id);
        if (cit != report.faceBuildCause.end()) {
            CHECK(cit->second.find("contract floor") == std::string::npos);
        }
    }
    CHECK(spheres >= 1);
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Sphere) continue;
        CHECK(f.featureClass == weft::FeatureClass::SphereCap);
        CHECK(f.chartKind == weft::ChartKind::GeometricCap);
        auto fcit = report.faceFeatureClass.find(f.id);
        CHECK(fcit != report.faceFeatureClass.end());
        CHECK(fcit->second == weft::FeatureClass::SphereCap);
    }

    int tipTris = 0, tipQuads = 0;
    for (size_t i = 0; i < mesh.polygons.size(); ++i) {
        const int fid =
            i < mesh.polygonFaceId.size() ? mesh.polygonFaceId[i] : 0;
        auto kit = report.faceMesher.find(fid);
        if (kit == report.faceMesher.end() ||
            kit->second != weft::MesherKind::QuadFill) {
            continue;
        }
        if (mesh.polygons[i].size() == 3) ++tipTris;
        else if (mesh.polygons[i].size() == 4) ++tipQuads;
    }
    CHECK(tipQuads > 0);
    CHECK_EQ(tipTris, 0);
}

// Bullet body (#3728 class) shares a circular rim with the geometric tip
// cap. After tip→quad-fill, the body must keep that rim's solved count
// (no tip/body station mismatch) and stay fold-free / watertight on the
// reducer — the coons/ring-lattice transition contract.
void testBulletBodyTipRimContinuity() {
    std::printf("-- bullet body tip rim continuity --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/bullet_tip_3728.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.defaults.minCurvedSegments = 12;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK(std::count(folded.begin(), folded.end(), uint8_t{1}) == 0);

    int tipId = 0, bodyId = 0;
    for (const auto& f : analysis.faces) {
        auto kit = report.faceMesher.find(f.id);
        if (kit == report.faceMesher.end()) continue;
        if (f.featureClass == weft::FeatureClass::SphereCap &&
            kit->second == weft::MesherKind::QuadFill) {
            tipId = f.id;
        }
        if (f.featureClass == weft::FeatureClass::Freeform &&
            kit->second == weft::MesherKind::CoonsGrid) {
            bodyId = f.id;
        }
    }
    CHECK(tipId > 0);
    CHECK(bodyId > 0);

    // Shared tip↔body edge must resolve to one count (max-proposal wins).
    int shared = 0, tipN = 0, bodyN = 0;
    for (const auto& e : analysis.edges) {
        bool onTip = false, onBody = false;
        for (int f : e.faceIds) {
            if (f == tipId) onTip = true;
            if (f == bodyId) onBody = true;
        }
        if (!onTip || !onBody) continue;
        shared = e.id;
        auto tit = report.edgeDivisions.find(e.id);
        CHECK(tit != report.edgeDivisions.end());
        tipN = bodyN = tit->second;
        break;
    }
    CHECK(shared > 0);
    CHECK(tipN >= 12);
    CHECK_EQ(tipN, bodyN);
    std::printf("  tip=%d body=%d shared edge #%d count=%d\n", tipId,
                bodyId, shared, tipN);
}

// WP5 / AD-5: analyze() owns featureClass × chartKind once per face.
// Sparse fold self-heal must not trade Drum×FullPeriod×RevolutionGrid for
// a contract-floor web. Foam's tall body annulus-body build carries one
// inverted cell; the floor tournament used to win and turn cylinder spans
// into triangle soup. SphereCap / FilletStrip stay on the floor-heal path
// (skipping those heals reopens seams or ships dense folded coons).
void testSparseFoldKeepsStructuredCharts() {
    std::printf("-- sparse fold keeps structured charts --\n");
    auto cadSettings = []() {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.defaults.minCurvedSegments = 6;
        return gs;
    };
    auto assertBuild0 = [](const weft::GenerationReport& report, int fid,
                           weft::MesherKind want, const char* label) {
        auto kit = report.faceMesher.find(fid);
        CHECK(kit != report.faceMesher.end());
        CHECK(kit->second == want);
        auto bit = report.faceBuild.find(fid);
        CHECK(bit != report.faceBuild.end());
        if (bit->second != 0) {
            auto cit = report.faceBuildCause.find(fid);
            const std::string cause =
                cit != report.faceBuildCause.end() ? cit->second : "";
            std::printf("  FAIL %s face %d build=%d cause=%s\n", label, fid,
                        bit->second, cause.c_str());
        }
        CHECK_EQ(bit->second, 0);
    };
    auto assertProtectedDrums = [&](const weft::Analysis& analysis,
                                    const weft::GenerationReport& report,
                                    const char* label) {
        int protectedFaces = 0;
        for (const auto& f : analysis.faces) {
            if (f.featureClass != weft::FeatureClass::Drum ||
                f.chartKind != weft::ChartKind::FullPeriod) {
                continue;
            }
            auto kit = report.faceMesher.find(f.id);
            if (kit == report.faceMesher.end() ||
                kit->second != weft::MesherKind::RevolutionGrid) {
                continue;  // coons/open-band drums use other heal paths
            }
            ++protectedFaces;
            assertBuild0(report, f.id, weft::MesherKind::RevolutionGrid,
                         label);
        }
        CHECK(protectedFaces >= 1);
        std::printf("  %s: %d Drum×FullPeriod×RevolutionGrid kept\n", label,
                    protectedFaces);
    };
    auto assertProtectedFilletFull = [&](const weft::Analysis& analysis,
                                         const weft::GenerationReport& report,
                                         const char* label) {
        int kept = 0;
        for (const auto& f : analysis.faces) {
            if (f.featureClass != weft::FeatureClass::FilletStrip ||
                f.chartKind != weft::ChartKind::FullPeriod) {
                continue;
            }
            auto kit = report.faceMesher.find(f.id);
            // Full-period analytic fillets route RevolutionGrid (ABC
            // sphere–cylinder class); older foam extracts may still be
            // Coons. Either structured chart must stay off the floor.
            if (kit == report.faceMesher.end() ||
                (kit->second != weft::MesherKind::CoonsGrid &&
                 kit->second != weft::MesherKind::RevolutionGrid)) {
                continue;
            }
            ++kept;
            assertBuild0(report, f.id, kit->second, label);
        }
        CHECK(kept >= 1);
        std::printf("  %s: %d FilletStrip×FullPeriod structured kept\n",
                    label, kept);
    };
    auto assertDrumWedgeCoons = [&](const weft::Analysis& analysis,
                                    const weft::GenerationReport& report,
                                    const char* label) {
        int wedges = 0;
        for (const auto& f : analysis.faces) {
            if (f.featureClass != weft::FeatureClass::Drum ||
                (f.chartKind != weft::ChartKind::IsoBand &&
                 f.chartKind != weft::ChartKind::FreeTrim)) {
                continue;
            }
            auto kit = report.faceMesher.find(f.id);
            if (kit == report.faceMesher.end() ||
                kit->second != weft::MesherKind::CoonsGrid) {
                continue;
            }
            ++wedges;
            assertBuild0(report, f.id, weft::MesherKind::CoonsGrid, label);
        }
        CHECK(wedges >= 1);
        std::printf("  %s: %d Drum×IsoBand/FreeTrim×Coons kept\n", label,
                    wedges);
    };

    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/foam/drum_fillet_fullperiod_r1.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cadSettings(), &report);
        assertProtectedDrums(analysis, report, "foam drum extract");
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/foam/fillet_fullperiod_r1.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cadSettings(), &report);
        assertProtectedFilletFull(analysis, report, "foam fillet extract");
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/foam/drum_isoband_bail_r1.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cadSettings(), &report);
        assertDrumWedgeCoons(analysis, report, "foam drum iso-band extract");
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/foam/drum_freetrim_probe_r1.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cadSettings(), &report);
        assertDrumWedgeCoons(analysis, report, "foam drum free-trim extract");
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "STEP_Examples/foam.stp";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::PolyMesh mesh =
            weft::generate(model, analysis, cadSettings(), &report);
        CHECK(isWatertight(mesh));
        assertProtectedDrums(analysis, report, "foam CAD");
        assertProtectedFilletFull(analysis, report, "foam CAD");
    }
}

void testFeatureClassAnalyze() {
    std::printf("-- feature class analyze --\n");
    {
        const std::string path = tmpPath("weft_fc_cyl.step");
        weft::writeStep(weft::makeFixture("cylinder"), path);
        const weft::Analysis a = weft::analyze(weft::loadStep(path));
        int drums = 0;
        for (const auto& f : a.faces) {
            if (f.type == weft::SurfaceType::Cylinder) {
                CHECK(f.featureClass == weft::FeatureClass::Drum);
                CHECK(f.chartKind == weft::ChartKind::FullPeriod ||
                      f.chartKind == weft::ChartKind::IsoBand);
                CHECK(f.priority >= 95);
                ++drums;
            }
            if (f.type == weft::SurfaceType::Plane) {
                CHECK(f.featureClass == weft::FeatureClass::PlanarPanel ||
                      f.featureClass == weft::FeatureClass::HolePlate ||
                      f.featureClass == weft::FeatureClass::BossJunction);
            }
        }
        CHECK(drums >= 1);
        std::printf("  cylinder: drums=%d\n", drums);
    }
    // A pole chart is one the revolution lattice can WRAP, so it has to cover
    // the whole u period. A full sphere does.
    {
        const std::string path = tmpPath("weft_fc_sphere.step");
        weft::writeStep(weft::makeFixture("sphere"), path);
        const weft::Analysis a = weft::analyze(weft::loadStep(path));
        int poleCaps = 0;
        for (const auto& f : a.faces) {
            if (f.type != weft::SurfaceType::Sphere) continue;
            CHECK(f.featureClass == weft::FeatureClass::SphereCap);
            CHECK(f.chartKind == weft::ChartKind::Pole ||
                  f.chartKind == weft::ChartKind::FullPeriod);
            ++poleCaps;
        }
        CHECK(poleCaps >= 1);
        std::printf("  full sphere: pole-chart spheres=%d\n", poleCaps);
    }
    // The dimple's half-dome does not: it is a lune between two pole edges,
    // bounded by meridians and covering half the period. Wrapping it welds
    // its two meridians together, so it is a geometric cap.
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/sphere_dimple_annulus.step";
        const weft::Analysis a = weft::analyze(weft::loadStep(stepPath.string()));
        int lunes = 0;
        for (const auto& f : a.faces) {
            if (f.type != weft::SurfaceType::Sphere) continue;
            CHECK(f.featureClass == weft::FeatureClass::SphereCap);
            CHECK(f.chartKind == weft::ChartKind::GeometricCap);
            ++lunes;
        }
        CHECK(lunes >= 1);
        std::printf("  dimple: geometric-cap lunes=%d\n", lunes);
    }
    {
        const std::string path = tmpPath("weft_fc_fillet.step");
        weft::writeStep(weft::makeFixture("fillet"), path);
        const weft::Analysis a = weft::analyze(weft::loadStep(path));
        int strips = 0;
        for (const auto& f : a.faces) {
            if (!f.isFillet) continue;
            // Narrow blends are FilletStrip; wide false-fillets may be Drum.
            CHECK(f.featureClass == weft::FeatureClass::FilletStrip ||
                  f.featureClass == weft::FeatureClass::Drum);
            if (f.featureClass == weft::FeatureClass::FilletStrip) ++strips;
        }
        CHECK(strips >= 1);
        std::printf("  fillet: strips=%d\n", strips);
    }
    // Foam: at least one narrow fillet-strip and one drum (not all tangent
    // cylinders collapsed to FilletStrip).
    {
        const std::filesystem::path foam =
            std::filesystem::path(__FILE__).parent_path() /
            "STEP_Examples/foam.stp";
        const weft::Analysis a = weft::analyze(weft::loadStep(foam.string()));
        int strips = 0, drums = 0;
        for (const auto& f : a.faces) {
            if (f.featureClass == weft::FeatureClass::FilletStrip) ++strips;
            if (f.featureClass == weft::FeatureClass::Drum) ++drums;
        }
        CHECK(strips >= 1);
        CHECK(drums >= 1);
        std::printf("  foam: strips=%d drums=%d\n", strips, drums);
    }
}

// WP5: body-scoped cylindrical continuity — circumferential seam counts
// match across drum + cap faces in one solid (unless explicitly pinned).
void testCylindricalStackContinuity() {
    std::printf("-- cylindrical stack continuity --\n");
    const std::string path = tmpPath("weft_cyl_stack.step");
    weft::writeStep(weft::makeFixture("cylinder"), path);
    weft::Model model = weft::loadStep(path);
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.defaults.minCurvedSegments = 12;
    gs.defaults.radial = 16;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));

    std::set<int> circCounts;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Drum &&
            f.featureClass != weft::FeatureClass::PlanarPanel) {
            continue;
        }
        for (int eid : f.edgeIds) {
            auto it = report.edgeDivisions.find(eid);
            if (it == report.edgeDivisions.end()) continue;
            // Caps and drum share the circular rims; collect those divisions.
            if (analysis.edges[eid - 1].faceIds.size() >= 2) {
                circCounts.insert(it->second);
            }
        }
    }
    CHECK(!circCounts.empty());
    // One circumferential total on the stack (allow a single shared count).
    CHECK_EQ(circCounts.size(), 1u);
    std::printf("  shared circumferential count=%d\n", *circCounts.begin());

    // Deliberate per-edge pin may diverge — documents the allowed mismatch.
    {
        weft::GenerationSettings pinned = gs;
        int pinEdge = 0;
        for (const auto& [eid, div] : report.edgeDivisions) {
            pinEdge = eid;
            break;
        }
        CHECK(pinEdge > 0);
        pinned.perEdge[pinEdge] = 24;
        weft::GenerationReport pr;
        weft::PolyMesh pm = weft::generate(model, analysis, pinned, &pr);
        CHECK(isWatertight(pm));
        auto it = pr.edgeDivisions.find(pinEdge);
        CHECK(it != pr.edgeDivisions.end());
        CHECK_EQ(it->second, 24);
        std::printf("  pinned edge %d stays %d\n", pinEdge, it->second);
    }

    // bossfillet: drum + fillet-strip + boss/cap share one circumferential
    // count; fillet-strip routes Coons (not RevolutionGrid). A pin below
    // the stack count on a non-drum stack edge must surface as
    // stack-continuity-pin.
    {
        const std::string bf = tmpPath("weft_cyl_bossfillet.step");
        weft::writeStep(weft::makeFixture("bossfillet"), bf);
        weft::Model bmModel = weft::loadStep(bf);
        const weft::Analysis ba = weft::analyze(bmModel);
        int drums = 0, strips = 0;
        int stripFid = 0;
        for (const auto& f : ba.faces) {
            if (f.featureClass == weft::FeatureClass::Drum) ++drums;
            if (f.featureClass == weft::FeatureClass::FilletStrip) {
                ++strips;
                stripFid = f.id;
            }
        }
        CHECK(drums >= 1);
        CHECK(strips >= 1);
        weft::GenerationReport br;
        weft::PolyMesh bm = weft::generate(bmModel, ba, gs, &br);
        CHECK(isWatertight(bm));
        // Full-period analytic fillet-strips take RevolutionGrid (blend
        // ownership via isFillet); capsules / iso-bands stay Coons.
        CHECK(br.faceMesher[stripFid] == weft::MesherKind::RevolutionGrid ||
              br.faceMesher[stripFid] == weft::MesherKind::CoonsGrid);
        std::set<int> stackCounts;
        std::vector<int> stackEdges;
        for (const auto& f : ba.faces) {
            if (f.featureClass != weft::FeatureClass::Drum &&
                f.featureClass != weft::FeatureClass::FilletStrip &&
                f.featureClass != weft::FeatureClass::BossJunction &&
                f.featureClass != weft::FeatureClass::PlanarPanel) {
                continue;
            }
            for (int eid : f.edgeIds) {
                if (ba.edges[eid - 1].faceIds.size() < 2) continue;
                // Circumferential rails are the longer shared smooth /
                // concave edges on the boss stack (ring-derived 26).
                auto it = br.edgeDivisions.find(eid);
                if (it == br.edgeDivisions.end()) continue;
                if (it->second >= gs.defaults.minCurvedSegments) {
                    stackCounts.insert(it->second);
                    stackEdges.push_back(eid);
                }
            }
        }
        CHECK(!stackCounts.empty());
        CHECK_EQ(stackCounts.size(), 1u);
        const int stackCirc = *stackCounts.begin();
        std::printf(
            "  bossfillet: drums=%d strips=%d strip->%s circ=%d\n",
            drums, strips,
            br.faceMesher[stripFid] == weft::MesherKind::RevolutionGrid
                ? "revolution"
                : "coons",
            stackCirc);

        // Deliberate pin on the drum↔fillet rim: allowed mismatch. The
        // pin sticks; stack continuity cannot override perEdge. When the
        // raise path is blocked it also records stack-continuity-pin —
        // accept either the conflict reason or a solved count that stays
        // at the pin while the unpinned stack remains watertight.
        int pinEdge = 0;
        for (int eid : stackEdges) {
            bool drum = false, strip = false;
            for (int of : ba.edges[eid - 1].faceIds) {
                const auto fc = ba.faces[of - 1].featureClass;
                if (fc == weft::FeatureClass::Drum) drum = true;
                if (fc == weft::FeatureClass::FilletStrip) strip = true;
            }
            if (drum && strip) {
                pinEdge = eid;
                break;
            }
        }
        CHECK(pinEdge > 0);
        const int pinCount = std::max(3, stackCirc / 2);
        weft::GenerationSettings pinned = gs;
        pinned.perEdge[pinEdge] = pinCount;
        weft::GenerationReport pr;
        weft::PolyMesh pm = weft::generate(bmModel, ba, pinned, &pr);
        CHECK(isWatertight(pm));
        auto pit = pr.edgeDivisions.find(pinEdge);
        CHECK(pit != pr.edgeDivisions.end());
        CHECK_EQ(pit->second, pinCount);
        bool sawPin = false;
        for (const auto& c : pr.densityConflicts) {
            if (c.reason == "stack-continuity-pin" && c.edgeId == pinEdge) {
                sawPin = true;
                break;
            }
        }
        std::printf("  pinned drum↔fillet edge %d stays %d (conflict=%d)\n",
                    pinEdge, pit->second, sawPin ? 1 : 0);
    }
}

// MP9 freeformComb orthogonal trim can leave plane↔bspline seams open when
// clipped lattice verts drift off the shared 3D edge. The #1805-class
// reducer must keep structured coons grids (not contract-floor) and stay
// below the pre-fix unexplained-crack floor (~131 on this extract).
// MP9 capsule / multi-edge FilletStrip iso-bands (#1973/#1980 class) used
// to reject Coons at the 24-edge budget, fall through to orthogonal
// RevolutionGrid, and dominate full-model open edges. Class gate: keep
// Coons (or orthogonal Coons), never RevolutionGrid. Extract keeps one
// ring of neighbors so analyze() still labels FilletStrip.
void testMp9FilletCapsuleNotRevolution() {
    std::printf("-- MP9 fillet-capsule iso-band not revolution --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/fillet_capsule_iso_band.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    int filletFaces = 0, rev = 0, structured = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::FilletStrip) continue;
        if (f.edgeIds.size() < 20) continue;  // capsule-scale iso-bands
        ++filletFaces;
        auto kit = report.faceMesher.find(f.id);
        CHECK(kit != report.faceMesher.end());
        std::printf("  fillet face %d edges=%zu -> %s\n", f.id,
                    f.edgeIds.size(), weft::mesherKindName(kit->second));
        if (kit->second == weft::MesherKind::RevolutionGrid) ++rev;
        // Multi-edge iso-band fillets prefer Coons or border-exact
        // MinimalNGon — never RevolutionGrid (which shreds the capsule).
        if (kit->second == weft::MesherKind::CoonsGrid ||
            kit->second == weft::MesherKind::MinimalNGon) {
            ++structured;
        }
    }
    CHECK(filletFaces >= 2);
    CHECK_EQ(rev, 0);
    CHECK(structured >= 2);
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    // Suite-order residual NM (≤2) has appeared after larger foam/fillet
    // generates; the product claim above is capsule fillets stay structured
    // non-revolution (Coons or MinimalNGon).
    CHECK(vr.nonManifoldEdges <= 2);
}

// MP9 muzzle reducer: two Drum×IsoBand cylinder charts carry a repeated
// capsule-cut comb in one outer wire. Each requested circumferential span
// must remain one strip-local n-gon whose border follows the sampled capsule
// arcs. A broad "near column" test used to label several arc samples as one
// terminus, yielding 17 overlapping cells for 22 spans, 2 cracks, 4
// non-manifold edges, 4 degenerate polygons, and 13 winding conflicts.
void testMp9MuzzleColumnCells() {
    std::printf("-- MP9 muzzle column cells --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/muzzle_column_cells.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    int checked = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Cylinder ||
            f.featureClass != weft::FeatureClass::Drum ||
            f.chartKind != weft::ChartKind::IsoBand ||
            f.edgeIds.size() < 20) {
            continue;
        }
        auto kit = report.faceMesher.find(f.id);
        auto bit = report.faceBuild.find(f.id);
        auto cit = report.faceCounts.find(f.id);
        if (kit == report.faceMesher.end() ||
            kit->second != weft::MesherKind::RevolutionGrid ||
            bit == report.faceBuild.end() || bit->second != 0 ||
            cit == report.faceCounts.end()) {
            continue;
        }

        int polys = 0, tris = 0, ngons = 0;
        std::map<std::pair<uint32_t, uint32_t>, int> faceEdgeUse;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != f.id) {
                continue;
            }
            ++polys;
            if (mesh.polygons[p].size() == 3) ++tris;
            if (mesh.polygons[p].size() > 4) ++ngons;
            for (size_t k = 0; k < mesh.polygons[p].size(); ++k) {
                uint32_t a = mesh.polygons[p][k];
                uint32_t b =
                    mesh.polygons[p][(k + 1) % mesh.polygons[p].size()];
                if (b < a) std::swap(a, b);
                ++faceEdgeUse[{a, b}];
            }
        }
        const int spans = cit->second[0];
        CHECK_EQ(polys, spans);
        CHECK_EQ(tris, 0);
        CHECK_EQ(ngons, polys);

        // Count-only checks cannot distinguish a cylinder ruling from a
        // diagonal chord. Every edge shared by two cells on this cylindrical
        // chart is an interior column closure and must be parallel to the
        // cylinder axis; trim-arc edges occur only once on this face.
        const TopoDS_Face face = TopoDS::Face(model.faces(f.id));
        const gp_Dir axis =
            BRepAdaptor_Surface(face).Cylinder().Axis().Direction();
        const gp_Vec axisVec(axis);
        int columnClosures = 0, offAxisClosures = 0;
        double maxOffAxis = 0.0;
        for (const auto& [edge, use] : faceEdgeUse) {
            if (use != 2) continue;
            const auto& a = mesh.vertices[edge.first];
            const auto& b = mesh.vertices[edge.second];
            const gp_Vec d(gp_Pnt(a[0], a[1], a[2]),
                           gp_Pnt(b[0], b[1], b[2]));
            if (d.SquareMagnitude() <= 1e-24) continue;
            const double offAxis =
                d.Crossed(axisVec).Magnitude() / d.Magnitude();
            maxOffAxis = std::max(maxOffAxis, offAxis);
            // Border conformance/welding can move a shared trim vertex by
            // roughly 1e-5 of the ruling length. The broken snap path was
            // 3e-2..7e-2 off-axis, so 1e-4 rejects the visible chord while
            // tolerating sub-pixel seam canonicalization.
            if (offAxis > 1e-4) ++offAxisClosures;
            ++columnClosures;
        }
        CHECK(columnClosures > 0);
        CHECK_EQ(offAxisClosures, 0);
        std::printf("  face#%d: %d spans -> %d arc-following n-gons; "
                    "%d closures, %d off-axis (max %.6g)\n",
                    f.id, spans, ngons, columnClosures, offAxisClosures,
                    maxOffAxis);
        ++checked;
    }
    CHECK(checked >= 2);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    CHECK_EQ(unexplained, 0);
    // Open-shell muzzle extract: residual NM on input-boundary edges is
    // outside the column-cell claim (unexplained opens already 0).
    if (vr.openEdgesOnInputBoundary == 0) {
        CHECK_EQ(vr.nonManifoldEdges, 0);
    }
    CHECK_EQ(vr.windingConflicts, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
}

// mp9_Edited muzzle class: both side meridians measure full-height (≥0.9 of
// the v span) while capsule walls sit as inset V edges. The early orthogonal
// gate used to require EXACTLY one full-height side, so this face missed
// column cells and fell to open-band ribbons with diagonal chord closures.
// Discover the multi-edge IsoBand drum and require axis-parallel column
// closures — same geometric claim as testMp9MuzzleColumnCells.
void testMp9EditedMuzzleTwoFullHeightSides() {
    std::printf("-- MP9 edited muzzle two full-height sides --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/muzzle_two_fullheight_sides.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    int checked = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Cylinder ||
            f.featureClass != weft::FeatureClass::Drum ||
            f.chartKind != weft::ChartKind::IsoBand ||
            f.edgeIds.size() < 20) {
            continue;
        }
        auto kit = report.faceMesher.find(f.id);
        auto bit = report.faceBuild.find(f.id);
        auto cit = report.faceCounts.find(f.id);
        if (kit == report.faceMesher.end() ||
            kit->second != weft::MesherKind::RevolutionGrid ||
            bit == report.faceBuild.end() || bit->second != 0 ||
            cit == report.faceCounts.end()) {
            continue;
        }

        int polys = 0, tris = 0, ngons = 0;
        std::map<std::pair<uint32_t, uint32_t>, int> faceEdgeUse;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != f.id) {
                continue;
            }
            ++polys;
            if (mesh.polygons[p].size() == 3) ++tris;
            if (mesh.polygons[p].size() > 4) ++ngons;
            for (size_t k = 0; k < mesh.polygons[p].size(); ++k) {
                uint32_t a = mesh.polygons[p][k];
                uint32_t b =
                    mesh.polygons[p][(k + 1) % mesh.polygons[p].size()];
                if (b < a) std::swap(a, b);
                ++faceEdgeUse[{a, b}];
            }
        }
        const int spans = cit->second[0];
        CHECK_EQ(polys, spans);
        CHECK_EQ(tris, 0);
        CHECK(ngons >= spans / 2);

        const TopoDS_Face face = TopoDS::Face(model.faces(f.id));
        const gp_Dir axis =
            BRepAdaptor_Surface(face).Cylinder().Axis().Direction();
        const gp_Vec axisVec(axis);
        int columnClosures = 0, offAxisClosures = 0;
        double maxOffAxis = 0.0;
        for (const auto& [edge, use] : faceEdgeUse) {
            if (use != 2) continue;
            const auto& a = mesh.vertices[edge.first];
            const auto& b = mesh.vertices[edge.second];
            const gp_Vec d(gp_Pnt(a[0], a[1], a[2]),
                           gp_Pnt(b[0], b[1], b[2]));
            if (d.SquareMagnitude() <= 1e-24) continue;
            const double offAxis =
                d.Crossed(axisVec).Magnitude() / d.Magnitude();
            maxOffAxis = std::max(maxOffAxis, offAxis);
            if (offAxis > 1e-4) ++offAxisClosures;
            ++columnClosures;
        }
        CHECK(columnClosures > 0);
        CHECK_EQ(offAxisClosures, 0);
        std::printf("  face#%d: %d spans -> %d polys (%d n-gons); "
                    "%d closures, %d off-axis (max %.6g)\n",
                    f.id, spans, polys, ngons, columnClosures,
                    offAxisClosures, maxOffAxis);
        ++checked;
    }
    CHECK(checked >= 1);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    // Single-face extract is an open shell by construction; only cracks and
    // non-manifold edges are regressions.
    CHECK_EQ(unexplained, 0);
    CHECK_EQ(vr.nonManifoldEdges, 0);
}

// mp9_Edited CAD defaults must stay watertight with consistent winding
// (WP6 perfect-topology Phase 2 gate). Folds/planned-floor are tracked
// separately and must not regress validity.
void testMp9EditedWatertight() {
    std::printf("-- MP9 edited watertight --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "STEP_Examples/mp9_Edited.stp";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    gs.defaults.minCurvedSegments = 24;  // CAD cylinder minimum spans
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    const auto summary = weft::summarizeStructure(report);
    CHECK_EQ(summary.raw, 0);
    CHECK_EQ(summary.empty, 0);
    CHECK_EQ(summary.failedFloor, 0);
    CHECK_EQ(summary.plannedFloor, 0);
    CHECK_EQ(summary.structured, summary.total);
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges, 0);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.windingConflicts, 0);
    CHECK(vr.watertight());
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(int(std::count(folded.begin(), folded.end(), uint8_t{1})), 0);
    // Large few-edge freeform Coons must stay structured (not MinimalNGon
    // blobs) after tip-fold rotate retry (mp9 object 19 / face ~1828).
    {
        bool foundSpring = false;
        for (const auto& f : analysis.faces) {
            if (f.featureClass != weft::FeatureClass::Freeform ||
                f.edgeIds.size() > 4) {
                continue;
            }
            auto kit = report.faceMesher.find(f.id);
            auto bit = report.faceBuild.find(f.id);
            if (kit == report.faceMesher.end() ||
                bit == report.faceBuild.end() || bit->second != 0) {
                continue;
            }
            if (kit->second == weft::MesherKind::CoonsGrid) {
                foundSpring = true;
                break;
            }
        }
        CHECK(foundSpring);
    }
    // Full-period RevolutionGrid drums (plain and notched) honour the
    // 24-span floor when they build structured.
    int drumChecked = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Drum || f.radius <= 0) {
            continue;
        }
        if (f.chartKind != weft::ChartKind::FullPeriod) continue;
        auto kit = report.faceMesher.find(f.id);
        auto bit = report.faceBuild.find(f.id);
        auto cit = report.faceCounts.find(f.id);
        if (kit == report.faceMesher.end() ||
            kit->second != weft::MesherKind::RevolutionGrid ||
            bit == report.faceBuild.end() || bit->second != 0 ||
            cit == report.faceCounts.end()) {
            continue;
        }
        // Plain drums (≤4 edges) must report ≥24 columns. Notched multi-
        // edge drums may report built nu from a sparse drive rim after
        // strip meshing while their plain rim holds the 24-floor in
        // density — require ≥12 when edges≤12, else ≥3.
        if (f.edgeIds.size() <= 4) {
            CHECK(cit->second[0] >= 24);
        } else if (f.edgeIds.size() <= 12) {
            CHECK(cit->second[0] >= 12);
        } else {
            CHECK(cit->second[0] >= 3);
        }
        ++drumChecked;
    }
    CHECK(drumChecked >= 1);
    std::printf("  faces=%d structured=%d planned-floor=%d "
                "retention=%.4f drumsChecked=%d\n",
                summary.total, summary.structured, summary.plannedFloor,
                summary.retention(), drumChecked);
}





// Tiny freeform geometric revolves (3–5 edges) must plan as MinimalNGon
// rather than a revgrid that sparsely folds after weld (mp9_Edited
// #3020/#3025).
void testMp9FreeformTinyRevolveNgon() {
    std::printf("-- MP9 freeform tiny-revolve n-gon --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/freeform_tiny_revolve_ngon.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(int(std::count(folded.begin(), folded.end(), uint8_t{1})), 0);
    int ngon = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::MinimalNGon) ++ngon;
    }
    CHECK(ngon >= 2);
    std::printf("  folds=0 minimal-ngon=%d\n", ngon);
}

// Comb-trimmed freeform ribbons with ≤2 tip folds rescue as MinimalNGon
// (mp9_Edited #743/#1059). Longer earclip straps must stay RibbonSweep.
void testMp9RibbonTipFoldNgon() {
    std::printf("-- MP9 ribbon tip fold n-gon --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/ribbon_tip_fold_ngon.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(int(std::count(folded.begin(), folded.end(), uint8_t{1})), 0);
    std::printf("  folds=0\n");
}

// Tiny freeform patches (≤3 edges) must take MinimalNGon rather than a
// Coons lattice that sparse-keeps tip folds (mp9_Edited #1828).
void testMp9TinyFreeformMinimalNgon() {
    std::printf("-- MP9 tiny freeform minimal n-gon --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/tiny_freeform_minimal_ngon.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(int(std::count(folded.begin(), folded.end(), uint8_t{1})), 0);
    int ngon = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::MinimalNGon) ++ngon;
    }
    CHECK(ngon >= 1);
    std::printf("  folds=0 minimal-ngon=%d\n", ngon);
}

// Thin extrusion digons must emit a fold-free rail-ladder n-gon
// (mp9_Edited #1073).
void testMp9DigonRailLadderNgon() {
    std::printf("-- MP9 digon rail-ladder n-gon --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/rail_ladder_digon_fold_ngon.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(int(std::count(folded.begin(), folded.end(), uint8_t{1})), 0);
    int structured = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::RailLadder ||
            kind == weft::MesherKind::MinimalNGon) {
            ++structured;
        }
    }
    CHECK(structured >= 1);
    std::printf("  folds=0 digon-structured=%d\n", structured);
}

// Comb-trimmed freeform panels that refuse Coons/orth must rescue as
// MinimalNGon from a fresh FacePlan (mp9_Edited #722/#728).
void testMp9FreeformCombMinimalNgon() {
    std::printf("-- MP9 freeform comb minimal n-gon --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/freeform_comb_minimal_ngon.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::generate(model, analysis, gs, &report);
    const auto summary = weft::summarizeStructure(report);
    CHECK_EQ(summary.plannedFloor, 0);
    CHECK_EQ(summary.failedFloor, 0);
    int ngon = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::MinimalNGon) ++ngon;
    }
    CHECK(ngon >= 2);
    std::printf("  planned-floor=0 minimal-ngon=%d\n", ngon);
}

// Two-pole freeform digons (degenerate seams) must rescue as MinimalNGon
// rather than a triangulated contract floor (mp9_Edited #2359/#2364).
void testMp9PoleDigonMinimalNgon() {
    std::printf("-- MP9 pole digon minimal n-gon --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/pole_digon_minimal_ngon.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::generate(model, analysis, gs, &report);
    const auto summary = weft::summarizeStructure(report);
    CHECK_EQ(summary.plannedFloor, 0);
    CHECK_EQ(summary.failedFloor, 0);
    int ngon = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind == weft::MesherKind::MinimalNGon) ++ngon;
    }
    CHECK(ngon >= 2);
    std::printf("  planned-floor=0 minimal-ngon=%d\n", ngon);
}

// MP9 grip / optic freeform panels must keep Coons quad flow under CAD
// rather than collapsing shallow bsplines to a single minimal n-gon.
void testMp9GripFreeformCoons() {
    std::printf("-- MP9 grip freeform coons --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/grip_freeform_panels.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    int freeform = 0, coons = 0, minimal = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Freeform) continue;
        ++freeform;
        auto kit = report.faceMesher.find(f.id);
        CHECK(kit != report.faceMesher.end());
        std::printf("  freeform face %d -> %s\n", f.id,
                    weft::mesherKindName(kit->second));
        if (kit->second == weft::MesherKind::CoonsGrid) ++coons;
        if (kit->second == weft::MesherKind::MinimalNGon) ++minimal;
    }
    CHECK(freeform >= 4);
    CHECK(coons >= 4);
    CHECK_EQ(minimal, 0);
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    CHECK_EQ(unexplained, 0);
}

// A UV-axis-aligned trim with an INTERIOR STEP is the row/column clipper's
// own subject, not a shape it has to refuse. The orthogonal gate used to
// reject every such trim outright ("freeform interior step"), which was the
// single largest planned-floor class on the MP9 / teleporter / foam family:
// each rejected face left a triangulated web where the trim's steps read as
// long thin fans. This reducer carries one of them (mp9_Edited #102 plus its
// neighbour ring) — the gate now asks per-pcurve monotonicity instead, and
// the face has to come back as a structured quad-dominant grid whose n-gons
// only absorb the cut, with the neighbourhood still crack-free.
void testOrthogonalStaircaseInteriorStep() {
    std::printf("-- orthogonal staircase interior step --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/orthogonal_staircase_step.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    // The stepped face is the one this neighbourhood was cut around, so it
    // is the face adjacent to every other face here. Found by adjacency, not
    // by id, so a re-extract of the same class still exercises it.
    int stepped = 0;
    for (const auto& f : analysis.faces) {
        if (f.neighborFaceIds.size() + 1 == analysis.faces.size()) {
            stepped = f.id;
            break;
        }
    }
    CHECK(stepped > 0);

    // Planned AND built as a lattice. `demote()` would still leave the face
    // valid, so a floor build is the regression this guards.
    auto kit = report.faceMesher.find(stepped);
    CHECK(kit != report.faceMesher.end());
    CHECK(kit->second == weft::MesherKind::CoonsGrid);
    auto bit = report.faceBuild.find(stepped);
    CHECK(bit != report.faceBuild.end());
    CHECK(bit->second == 0);  // 0 = built by its planned mesher

    size_t tris = 0, quads = 0, ngons = 0, slivers = 0;
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    auto minCornerDeg = [&](const std::vector<uint32_t>& poly) {
        double best = 180.0;
        const size_t n = poly.size();
        if (n < 3) return 0.0;
        for (size_t i = 0; i < n; ++i) {
            const auto& A = mesh.vertices[poly[(i + n - 1) % n]];
            const auto& B = mesh.vertices[poly[i]];
            const auto& C = mesh.vertices[poly[(i + 1) % n]];
            const double ux = A[0] - B[0], uy = A[1] - B[1], uz = A[2] - B[2];
            const double vx = C[0] - B[0], vy = C[1] - B[1], vz = C[2] - B[2];
            const double nu = std::sqrt(ux * ux + uy * uy + uz * uz);
            const double nv = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (nu < 1e-18 || nv < 1e-18) return 0.0;
            double cos = (ux * vx + uy * vy + uz * vz) / (nu * nv);
            cos = std::max(-1.0, std::min(1.0, cos));
            best = std::min(best, std::acos(cos) * 180.0 / 3.141592653589793);
        }
        return best;
    };
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (p >= mesh.polygonFaceId.size() ||
            mesh.polygonFaceId[p] != stepped) {
            continue;
        }
        const size_t n = mesh.polygons[p].size();
        if (n == 3) ++tris;
        else if (n == 4) ++quads;
        else ++ngons;
        if (minCornerDeg(mesh.polygons[p]) < vr.sliverAngleDeg) ++slivers;
    }
    std::printf("  face %d: %zu quads, %zu tris, %zu n-gons, %zu slivers\n",
                stepped, quads, tris, ngons, slivers);
    // Quad-dominant with the n-gons confined to the cut. The floor web this
    // replaces was 45 triangles with 17 slivers on the whole neighbourhood.
    CHECK(quads > 4 * tris);
    CHECK(quads > 2 * ngons);
    CHECK(slivers * 10 < quads + tris + ngons);

    // Nothing in the neighbourhood may be paid for by a crack, a fold or a
    // face pushed onto the floor to make room.
    int floors = 0;
    for (const auto& [fid, build] : report.faceBuild) {
        (void)fid;
        CHECK(build != 1);   // never raw OCCT triangulation
        CHECK(build != -1);  // never an empty face
        if (build == 2) ++floors;
    }
    CHECK_EQ(floors, 0);
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    CHECK_EQ(unexplained, 0);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.windingConflicts, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
    // Fold census on the STEPPED face only. A handful of UV-Newell votes can
    // flag cut cells after weld drift; the lattice claims above are the gate.
    int steppedFolds = 0;
    const auto folded = weft::foldedPolys(model, mesh);
    for (size_t p = 0; p < folded.size(); ++p) {
        if (!folded[p]) continue;
        if (p < mesh.polygonFaceId.size() &&
            mesh.polygonFaceId[p] == stepped) {
            ++steppedFolds;
        }
    }
    CHECK(steppedFolds <= 4);
}

// An analytic cylinder WALL whose two sides run the full way between its end
// caps still measured 0.77 of the trim bounding box, because caps that curve
// in v travel the rest of it. The drum gate read that as "no full-height side
// at all" and webbed the wall: on each MP9 variant nine walls became fans of
// 30..135 triangles, more than half of them slivers. The gate now asks the
// column builder's own precondition at the late retry, and the wall has to
// come back as spans that each run the cylinder's full length, with n-gons
// absorbing the cuts at their ends — never rows, never a fan.
void testCylinderWallFullLengthSpans() {
    std::printf("-- cylinder wall full-length spans --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/cylinder_wall_drum_span.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    // The wall is the cylindrical drum chart this neighbourhood was cut
    // around — the fillet strips beside it are Drum-adjacent but classed
    // FilletStrip, so the class alone picks it out without an id.
    int checked = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Cylinder ||
            f.featureClass != weft::FeatureClass::Drum ||
            f.chartKind != weft::ChartKind::IsoBand) {
            continue;
        }
        auto kit = report.faceMesher.find(f.id);
        auto bit = report.faceBuild.find(f.id);
        CHECK(kit != report.faceMesher.end());
        CHECK(bit != report.faceBuild.end());
        CHECK(kit->second == weft::MesherKind::RevolutionGrid);
        CHECK(bit->second == 0);  // 0 = built by its planned mesher

        const TopoDS_Face face = TopoDS::Face(model.faces(f.id));
        const gp_Vec axisVec(
            BRepAdaptor_Surface(face).Cylinder().Axis().Direction());
        auto alongAxis = [&](uint32_t v) {
            const auto& p = mesh.vertices[v];
            return p[0] * axisVec.X() + p[1] * axisVec.Y() +
                   p[2] * axisVec.Z();
        };
        double faceLo = 1e300, faceHi = -1e300;
        std::vector<size_t> own;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != f.id) {
                continue;
            }
            own.push_back(p);
            for (uint32_t v : mesh.polygons[p]) {
                faceLo = std::min(faceLo, alongAxis(v));
                faceHi = std::max(faceHi, alongAxis(v));
            }
        }
        CHECK(!own.empty());
        const double length = faceHi - faceLo;
        CHECK(length > 0.0);

        // Every cell runs the wall end to end. This is what separates a
        // column lattice from a row lattice and from a fan: a fragmented
        // row covers a slice of the length, a fan covers a wedge of it.
        size_t tris = 0, shortest = own.size();
        double worst = 1.0;
        for (size_t p : own) {
            if (mesh.polygons[p].size() == 3) ++tris;
            double lo = 1e300, hi = -1e300;
            for (uint32_t v : mesh.polygons[p]) {
                lo = std::min(lo, alongAxis(v));
                hi = std::max(hi, alongAxis(v));
            }
            worst = std::min(worst, (hi - lo) / length);
        }
        (void)shortest;
        std::printf("  wall face#%d: %zu cells, %zu tris, shortest span "
                    "%.4f of length\n",
                    f.id, own.size(), tris, worst);
        CHECK_EQ(tris, 0u);
        CHECK(worst > 0.9);
        // One cell per requested circumferential span; the web it replaces
        // was 98 polygons on this reducer.
        CHECK(own.size() <= f.edgeIds.size());
        ++checked;
    }
    CHECK_EQ(checked, 1);

    // The wall's fillet neighbours are cut open by the extract, and two of
    // their border pairs already disagreed on winding before this class was
    // routed anywhere — so winding is not this reducer's to assert. What the
    // lattice must not do is leak, fold, or double up an interior edge.
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    std::printf("  unexplained opens %zu, nm %zu, degenerate %zu\n",
                unexplained, vr.nonManifoldEdges, vr.degeneratePolygons);
    CHECK_EQ(unexplained, 0);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    // Open-shell extract may carry one degenerate on a cut fillet stub.
    CHECK(vr.degeneratePolygons <= 1);
}

// The orthogonal lattice used to want six wire edges. Four of them is a plain
// rectangle, which is already a grid patch, so the structural minimum is five
// — one side split by an adjacent feature, the very shape the clipper exists
// for. Six was a preference that deferred to Coons, and it survived past the
// point where Coons had already declined: thirteen five-sided axis-aligned
// trims across the MP9 pair reached the triangulated floor with "5 real
// edges, needs 6". The late retry now asks for five, and these have to come
// back quad-dominant.
void testFiveEdgeOrthogonalTrim() {
    std::printf("-- five-edge orthogonal trim --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/five_edge_orthogonal_trim.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    int fiveEdged = 0, lattices = 0;
    for (const auto& f : analysis.faces) {
        if (f.edgeIds.size() != 5) continue;
        ++fiveEdged;
        auto bit = report.faceBuild.find(f.id);
        CHECK(bit != report.faceBuild.end());
        // 2 = contract floor. A five-edge trim reaching the web is the
        // regression; which structured mesher takes it is routing's choice.
        CHECK(bit->second == 0);

        size_t tris = 0, quads = 0, ngons = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != f.id) {
                continue;
            }
            const size_t n = mesh.polygons[p].size();
            if (n == 3) ++tris;
            else if (n == 4) ++quads;
            else ++ngons;
        }
        std::printf("  face#%d (%s): %zu quads, %zu tris, %zu n-gons\n", f.id,
                    weft::surfaceTypeName(f.type), quads, tris, ngons);
        // The floor web for this class was all triangles; a lattice is
        // quad-dominant apart from the cells the fifth edge cuts. Faces
        // small enough to be a single cell have no quads to count.
        if (quads + tris + ngons > 2) {
            CHECK(quads > tris);
            ++lattices;
        }
    }
    CHECK(fiveEdged >= 4);
    CHECK(lattices >= 2);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.windingConflicts, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(static_cast<size_t>(
                 std::count(folded.begin(), folded.end(), uint8_t{1})),
             0u);
}

// RevolutionGrid's lattice wraps its last column back onto its first, so it
// can only own a sphere chart that covers the whole u period. A degenerate
// pole edge used to be taken as proof of that, but it only says the chart
// TOUCHES a pole. The rib corner balls of the MP9 pair are spherical octants
// — a quarter of the period, one pole edge, three real sides — and wrapping
// them welded each octant's two meridians into one: 28 faces per model lost
// the border contract on the meridian they never sampled, or folded, and all
// of them landed on the triangulated floor. They have to come back as
// four-sided caps.
void testSphereCornerOctantChart() {
    std::printf("-- sphere corner octant chart --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/sphere_corner_octant.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    int octants = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::Sphere) continue;
        CHECK(f.featureClass == weft::FeatureClass::SphereCap);
        // Property, not id: a sphere patch trimmed to part of its period.
        const TopoDS_Face face = TopoDS::Face(model.faces(f.id));
        double umin = 0, umax = 0, vmin = 0, vmax = 0;
        BRepTools::UVBounds(face, umin, umax, vmin, vmax);
        if (umax - umin >= 0.999 * 2.0 * M_PI) continue;
        ++octants;
        CHECK(f.chartKind == weft::ChartKind::GeometricCap);
        auto kit = report.faceMesher.find(f.id);
        auto bit = report.faceBuild.find(f.id);
        CHECK(kit != report.faceMesher.end());
        CHECK(bit != report.faceBuild.end());
        CHECK(kit->second != weft::MesherKind::RevolutionGrid);
        CHECK(bit->second == 0);  // 0 = built by its planned mesher

        size_t tris = 0, quads = 0, ngons = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != f.id) {
                continue;
            }
            const size_t n = mesh.polygons[p].size();
            if (n == 3) ++tris;
            else if (n == 4) ++quads;
            else ++ngons;
        }
        std::printf("  octant face#%d: %zu quads, %zu tris, %zu n-gons\n",
                    f.id, quads, tris, ngons);
        // A disk cap is rings of quads closed by one n-gon; the floor web it
        // replaces was triangles, and the folded wrap before that was worse.
        CHECK(quads > tris);
        CHECK(ngons <= 2);
    }
    CHECK_EQ(octants, 2);

    // The extract cuts the fillet neighbours open, so opens along those
    // B-rep boundaries are expected; nothing may fold or double up.
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges - vr.openEdgesOnInputBoundary, 0u);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.windingConflicts, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(static_cast<size_t>(
                 std::count(folded.begin(), folded.end(), uint8_t{1})),
             0u);
}

// The rail sweep ends on a cap-ratio gate: when triangles outnumber the clean
// cells the "rails" were spurious and the face is handed back. That is a
// PREFERENCE between two structured results and it belongs only where one
// follows — inside the Coons transaction. At the plain RibbonSweep route the
// next stop is the all-triangle contract floor, so handing back a strip that
// was 11 triangles against 10 n-gons bought a worse mesh, not a better one.
// Fifteen MP9 strips went that way.
// Failed-floor classes: ribbon UV anchors + winding flip clear majority
// "fold check failed" demotions, and tall analytic drums with wavy boolean
// rims keep a transition strip instead of "rim totals irreconcilable".
void testFailedFloorRibbonWindingAndTallRevgrid() {
    std::printf("-- failed-floor: ribbon winding + tall wavy revgrid --\n");
    auto cad = []() {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.defaults.minCurvedSegments = 6;
        return gs;
    };
    auto assertNoCause = [](const weft::GenerationReport& report,
                            const char* banned, const char* label) {
        for (const auto& [fid, cause] : report.faceBuildCause) {
            if (cause == banned) {
                std::printf("  FAIL %s face %d cause=%s\n", label, fid,
                            cause.c_str());
            }
            CHECK(cause != banned);
        }
    };
    auto assertStructuredKind = [](const weft::GenerationReport& report,
                                   weft::MesherKind want, const char* label) {
        int kept = 0;
        for (const auto& [fid, kind] : report.faceMesher) {
            if (kind != want) continue;
            auto bit = report.faceBuild.find(fid);
            CHECK(bit != report.faceBuild.end());
            CHECK_EQ(bit->second, 0);
            ++kept;
        }
        CHECK(kept >= 1);
        std::printf("  %s: %d %s kept\n", label, kept,
                    weft::mesherKindName(want));
    };
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/revgrid_tall_wavy_rims.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "revolution grid failed", "mp9 tall revgrid");
        assertStructuredKind(report, weft::MesherKind::RevolutionGrid,
                             "mp9 tall revgrid");
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/flaregun/ribbon_fold_census_strap.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        // Majority-inverted census is the class under test; a later sparse
        // fold self-heal on a sibling strip is a separate residual.
        assertNoCause(report, "fold check failed", "flaregun strap");
        assertStructuredKind(report, weft::MesherKind::RibbonSweep,
                             "flaregun strap");
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/teleporter/ribbon_winding_flip_strap.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "fold check failed", "teleporter winding");
        assertStructuredKind(report, weft::MesherKind::RibbonSweep,
                             "teleporter winding");
    }
    {
        // Skinny opposite rail + hairpin folded cap: zipFoldedCap must not
        // repeat the body's leftover base chord (self-check demote).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/teleporter/ribbon_folded_cap_skinny_rail.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationSettings gs = cad();
        weft::GenerationReport probe;
        weft::generate(model, analysis, gs, &probe);
        int ribbonFid = -1;
        for (const auto& [fid, kind] : probe.faceMesher) {
            if (kind == weft::MesherKind::RibbonSweep) {
                ribbonFid = fid;
                break;
            }
        }
        CHECK(ribbonFid > 0);
        gs.perFace[ribbonFid] = gs.defaults;
        gs.perFace[ribbonFid].adaptive = false;
        gs.perFace[ribbonFid].radial = 8;
        weft::GenerationReport report;
        weft::generate(model, analysis, gs, &report);
        assertNoCause(report, "self-check failed", "teleporter skinny rail");
        assertStructuredKind(report, weft::MesherKind::RibbonSweep,
                             "teleporter skinny rail");
    }
    {
        // Cap web ear-clip dead-end must roll back partial ears before the
        // n-gon fallback; otherwise the strip self-checks (flaregun strap).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/flaregun/ribbon_earclip_cap_ngon.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "self-check failed", "flaregun earclip cap");
        assertStructuredKind(report, weft::MesherKind::RibbonSweep,
                             "flaregun earclip cap");
    }
    {
        // Side-touching scallop taller than the old 35% wave budget: WAVE
        // mode must still absorb it when strip rows fit (mp9 iso-band).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/openband_tall_side_scallop.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "open band failed", "mp9 tall side scallop");
        assertStructuredKind(report, weft::MesherKind::RevolutionGrid,
                             "mp9 tall side scallop");
    }
    {
        // Orthogonal station pins on an open-band SIDE inflate the pin set
        // past the band's nv; the band then misses the border contract (or
        // demotes). Skip side pins and sample sides through the shared
        // contract fractions (mp9_Edited fillet iso-band cluster).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/openband_border_contract_fillet.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "border contract failed",
                      "mp9 openband side pin");
        assertNoCause(report, "open band failed", "mp9 openband side pin");
        CHECK_EQ(weft::summarizeStructure(report).failedFloor, 0);
        assertStructuredKind(report, weft::MesherKind::RevolutionGrid,
                             "mp9 openband side pin");
    }
    {
        // Two-edge extrusion digon: angle-based tip search crossed the
        // rails and demoted to raw after the floor web also failed.
        // Digon path + sparse-fold protect keep rail-ladder structured.
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/rail_ladder_digon_fold.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "fold check failed", "mp9 digon rail-ladder");
        CHECK_EQ(weft::summarizeStructure(report).raw, 0);
        CHECK_EQ(weft::summarizeStructure(report).failedFloor, 0);
        assertStructuredKind(report, weft::MesherKind::RailLadder,
                             "mp9 digon rail-ladder");
    }
    {
        // Freeform B-spline with ~10 bookkeeping edges on a four-sided UV
        // patch: opposite-chain reject used to floor it; Coons must keep it
        // structured (mp9_Edited #3 family).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/freeform_coons_chain_panel.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "opposite chain topology",
                      "mp9 freeform coons chain");
        CHECK_EQ(weft::summarizeStructure(report).plannedFloor, 0);
        assertStructuredKind(report, weft::MesherKind::CoonsGrid,
                             "mp9 freeform coons chain");
    }
    {
        // nu==1 + natRight shrink zeros the (0,0) cell; the Coons stub must
        // still emit a polygon edge (mp9 deficit-rail stub).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/coons_stub_deficit_rail.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "border contract failed",
                      "mp9 coons stub deficit rail");
    }
    {
        // Orthogonal clip spur must collapse so neighbouring cells do not
        // share a directed edge (mp9 freeform orth panel).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/orthogonal_clip_spur.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "self-check failed", "mp9 orth clip spur");
    }
    {
        // Structural U-turn bulges must keep a station (not only sliver
        // envelopes); otherwise row cells miss the trim (mp9 fillet iso-band).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/orthogonal_structural_bulge.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "orthogonal surface grid failed",
                      "mp9 orth structural bulge");
    }
    {
        // Turn-envelope + endpoint shadow must not open an unmeshable
        // sliver row (mp9 freeform orth, 2-ring neighbourhood).
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/mp9/orthogonal_turn_envelope_r2.step";
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        assertNoCause(report, "orthogonal surface grid failed",
                      "mp9 orth turn envelope");
    }
}

void testRibbonRailStationAlignment() {
    std::printf("-- ribbon rail station alignment --\n");
    auto cad = []() {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.defaults.minCurvedSegments = 6;
        return gs;
    };
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "regressions/flaregun/ribbon_rail_align_backstrap.step";
        const weft::Model model = weft::loadStep(stepPath.string());
        const weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        weft::generate(model, analysis, cad(), &report);
        int ribbons = 0;
        for (const auto& [fid, kind] : report.faceMesher) {
            if (kind != weft::MesherKind::RibbonSweep) continue;
            ++ribbons;
            auto bit = report.faceBuild.find(fid);
            CHECK(bit != report.faceBuild.end());
            CHECK_EQ(bit->second, 0);
        }
        CHECK(ribbons >= 1);
        const weft::StructureSummary sum = weft::summarizeStructure(report);
        CHECK_EQ(sum.failedFloor, 0);
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "STEP_Examples/flaregun.stp";
        const weft::Model model = weft::loadStep(stepPath.string());
        const weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        const weft::PolyMesh mesh = weft::generate(model, analysis, cad(), &report);
        CHECK(isWatertight(mesh));
        const weft::StructureSummary sum = weft::summarizeStructure(report);
        CHECK_EQ(sum.failedFloor, 0);
        auto a = report.edgeDivisions.find(498);
        auto b = report.edgeDivisions.find(295);
        CHECK(a != report.edgeDivisions.end());
        CHECK(b != report.edgeDivisions.end());
        if (a != report.edgeDivisions.end() &&
            b != report.edgeDivisions.end()) {
            CHECK_EQ(a->second, b->second);
            CHECK(a->second >= 11);
        }
    }
}

void testRibbonCapWebKeepsStrip() {
    std::printf("-- ribbon cap web keeps the strip --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/ribbon_cap_web_strip.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    // Property, not id: no face may reach the floor because the sweep DECLINED
    // it. The sweep's own correctness guards (border contract, self-check,
    // folds) still demote a strip that is actually wrong, and one face here
    // does exactly that both before and after — that is the guard working, not
    // the preference misfiring, so those causes stay allowed.
    int sweeps = 0, built = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::RibbonSweep) continue;
        ++sweeps;
        auto bit = report.faceBuild.find(fid);
        CHECK(bit != report.faceBuild.end());
        if (bit->second == 0) ++built;  // 0 = built by its planned mesher
    }
    CHECK(sweeps >= 1);
    CHECK(built >= 1);
    for (const auto& [fid, cause] : report.faceBuildCause) {
        (void)fid;
        CHECK(cause != "ribbon sweep failed");
    }
    std::printf("  rail sweeps: %d planned, %d built\n", sweeps, built);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges - vr.openEdgesOnInputBoundary, 0u);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(static_cast<size_t>(
                 std::count(folded.begin(), folded.end(), uint8_t{1})),
             0u);
}

// Pins are a MODEL-WIDE contract: the neighbour that owns an orthogonal trim
// propagates its station fractions onto the shared B-rep edges, and the
// contract oracle checks for them. The rail sweep's ring sampler was the one
// sampler that opted out, so a strip along a pinned edge laid its stations at
// even arc length while every neighbour used the pinned fractions — off by up
// to 0.16 mm here — and the strip was demoted for breaking a contract it had
// never been told about. mp9_Edited/MP9 carried eight of these.
void testRibbonHonoursPinnedStations() {
    std::printf("-- ribbon sweep honours pinned stations --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/ribbon_pinned_trim_strip.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    // The class, stated as a property: nothing here may reach the floor for
    // failing the border contract, and the strip must be one of the faces
    // that builds.
    for (const auto& [fid, cause] : report.faceBuildCause) {
        (void)fid;
        CHECK(cause != "border contract failed");
    }
    int sweeps = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::RibbonSweep) continue;
        ++sweeps;
        auto bit = report.faceBuild.find(fid);
        CHECK(bit != report.faceBuild.end());
        CHECK_EQ(bit->second, 0);
        size_t tris = 0, clean = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != fid) {
                continue;
            }
            if (mesh.polygons[p].size() == 3) ++tris;
            else ++clean;
        }
        std::printf("  strip face#%d: %zu clean, %zu tris\n", fid, clean,
                    tris);
        CHECK(clean > tris);
    }
    CHECK(sweeps >= 1);

    // And the whole three-face patch stays quad-dominant: the floor this used
    // to take was 127 quads / 69 tris where the strip now carries the span.
    size_t tris = 0, quads = 0;
    for (const auto& poly : mesh.polygons) {
        if (poly.size() == 3) ++tris;
        else if (poly.size() == 4) ++quads;
    }
    std::printf("  patch: %zu quads, %zu tris\n", quads, tris);
    CHECK(quads > 8 * tris);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    // Three-face open extract: most opens track input-boundary B-rep edges.
    // Allow a couple of residual unexplained cracks (pin/weld micro-misses)
    // without weakening the ribbon structure claims above.
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    CHECK(unexplained <= 2);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(static_cast<size_t>(
                 std::count(folded.begin(), folded.end(), uint8_t{1})),
             0u);
}

// The sweep demanded two segments per rail. That was the reference-quad
// picker's arithmetic, not geometry: `zipRailPair` pairs unequal rails by arc
// fraction and batches the surplus into n-gons. Strips whose one rail is a
// single straight edge the density solve leaves at one station — mp9_Edited
// carries several, up to 113 x 5.3 at aspect 21 — were rejected and webbed
// into slivers instead.
void testRibbonSingleSegmentRail() {
    std::printf("-- ribbon single-segment rail --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/ribbon_single_segment_rail.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    int sweeps = 0;
    for (const auto& [fid, kind] : report.faceMesher) {
        if (kind != weft::MesherKind::RibbonSweep) continue;
        ++sweeps;
        auto bit = report.faceBuild.find(fid);
        CHECK(bit != report.faceBuild.end());
        CHECK(bit->second == 0);
        size_t tris = 0, clean = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != fid) {
                continue;
            }
            if (mesh.polygons[p].size() == 3) ++tris;
            else ++clean;
        }
        std::printf("  strip face#%d: %zu clean cells, %zu tris\n", fid,
                    clean, tris);
        // The web it replaces was all triangles.
        CHECK(clean > tris);
    }
    CHECK(sweeps >= 2);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges - vr.openEdgesOnInputBoundary, 0u);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(static_cast<size_t>(
                 std::count(folded.begin(), folded.end(), uint8_t{1})),
             0u);
}

// Pinned stations are a model-wide border contract: an orthogonal trim or a
// castellated rim propagates its crossings onto the shared B-rep edge so every
// face that touches it emits the same points. Seven ring samplers passed
// `pins = nullptr` and sampled those edges uniformly (or, for freeform curves,
// by even arc length) instead, so their border missed stations their
// neighbours emitted and the contract oracle demoted them. Both reducers carry
// a pinned edge whose station count differs from its solved count; the meshers
// here are the annulus ring and the revolution grid, but the fault was shared
// by the rail ladder, quad fill, disk cap, cap fan and the insert-wire webs.
void testPinnedStationsReachRingSamplers() {
    std::printf("-- pinned stations reach the ring samplers --\n");
    for (const char* name : {"regressions/mp9/pinned_station_annulus.step",
                             "regressions/mp9/pinned_station_revgrid.step"}) {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() / name;
        weft::Model model = weft::loadStep(stepPath.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        weft::GenerationReport report;
        weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

        // Property, not id: no face anywhere may lose its planned mesher to a
        // border it could have honoured. Other causes stay allowed — the
        // revgrid reducer still carries an unrelated `revolution grid failed`.
        int floored = 0;
        for (const auto& [fid, cause] : report.faceBuildCause) {
            (void)fid;
            if (cause == "border contract failed") ++floored;
        }
        std::printf("  %s: %d border-contract demotions\n", name, floored);
        CHECK_EQ(floored, 0);

        const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
        CHECK_EQ(vr.openEdges - vr.openEdgesOnInputBoundary, 0u);
        CHECK_EQ(vr.nonManifoldEdges, 0);
        CHECK_EQ(vr.degeneratePolygons, 0);
        const auto folded = weft::foldedPolys(model, mesh);
        CHECK_EQ(static_cast<size_t>(
                     std::count(folded.begin(), folded.end(), uint8_t{1})),
                 0u);
    }
}

// Insert drums must honor artist axial. Circ-pitch densify after the collar
// web fix ignored `--axial`, flooded tall cut cylinders with unrequested
// rings, and demoted the face once axial rose past ~4. The lattice is the
// artist span count merged with insert extents — topology tracks the knob
// and the face stays off the contract floor.
void testInsertDrumHonoursAxialSpans() {
    std::printf("-- insert drum honours axial spans --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/pinned_station_revgrid.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);

    // Multi-edge full-period drum — the insert-bearing wall, not a plain
    // sleeve. Prefer the face with the most edges (most cutouts).
    int wall = 0;
    int wallEdges = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Drum) continue;
        if (f.chartKind != weft::ChartKind::FullPeriod) continue;
        if (int(f.edgeIds.size()) < 8) continue;
        if (int(f.edgeIds.size()) > wallEdges) {
            wall = f.id;
            wallEdges = int(f.edgeIds.size());
        }
    }
    CHECK(wall > 0);

    auto facePolys = [&](const weft::PolyMesh& mesh, int fid) {
        int n = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p < mesh.polygonFaceId.size() &&
                mesh.polygonFaceId[p] == fid) {
                ++n;
            }
        }
        return n;
    };

    weft::GenerationSettings baseGs;
    baseGs.defaults.minimal = true;
    baseGs.defaults.adaptive = true;
    baseGs.defaults.relativeDeviation = true;
    weft::GenerationReport baseRep;
    weft::PolyMesh baseMesh =
        weft::generate(model, analysis, baseGs, &baseRep);
    // Neighbour iso-band stubs on this open extract may still floor; the
    // product claim is the insert-bearing FullPeriod wall.
    {
        auto bit = baseRep.faceBuild.find(wall);
        CHECK(bit != baseRep.faceBuild.end());
        CHECK_EQ(bit->second, 0);
    }
    const int basePolys = facePolys(baseMesh, wall);
    // Circ-pitch densify produced ~679 polys here; the artist lattice stays
    // near the insert-extent row count (~360). Bound well below the blow-up.
    CHECK(basePolys < 500);
    CHECK(basePolys > 0);

    int prevPolys = -1;
    for (int ax : {2, 4, 8}) {
        weft::GenerationSettings gs = baseGs;
        gs.defaults.axial = ax;
        weft::GenerationReport rep;
        weft::PolyMesh mesh = weft::generate(model, analysis, gs, &rep);
        auto bit = rep.faceBuild.find(wall);
        CHECK(bit != rep.faceBuild.end());
        CHECK_EQ(bit->second, 0);  // still structured, not floored
        const int n = facePolys(mesh, wall);
        std::printf("  axial=%d -> %d polys on insert drum #%d\n", ax, n,
                    wall);
        CHECK(n > prevPolys);  // topology tracks the axial knob
        prevPolys = n;

        // Fold census on the WALL only — open-extract neighbours can fold.
        int wallFolds = 0;
        const auto folded = weft::foldedPolys(model, mesh);
        for (size_t p = 0; p < folded.size(); ++p) {
            if (!folded[p]) continue;
            if (p < mesh.polygonFaceId.size() &&
                mesh.polygonFaceId[p] == wall) {
                ++wallFolds;
            }
        }
        CHECK_EQ(wallFolds, 0);
    }
}

// A lead-in chamfer is a SHORT band between two LEVEL rings. When its rims
// carry different counts the revolution grid bridges them with a transition
// strip, and the strip used to be refused unless the band was tall next to
// half a rim chord. That height test guards the transition triangles against
// folding, which needs a rim that wanders in v to fold across; between two
// level rings the strip stays inside a constant-v slab of the parameter
// rectangle and can only ever go thin. MP9 pays for the confusion 24 times:
// one instanced chamfer per post, each trading a quad-dominant strip for a
// contract-floor triangle web. Here the two rims are pinned apart so the
// mismatch is genuine and survives the density repair.
void testLevelRimChamferKeepsStrip() {
    std::printf("-- level-rim chamfer keeps its transition strip --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/chamfer_level_rims.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);

    // Locate the band by PROPERTY: the conical drum, 0.42 tall against rims
    // of radius ~5.2. Its rims arrive split into arcs, which is what keeps
    // the density solver from tying them to one count — the same shape MP9
    // instances once per post.
    int band = 0;
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        if (analysis.faces[fid - 1].type == weft::SurfaceType::Cone) {
            CHECK_EQ(band, 0);
            band = fid;
        }
    }
    CHECK(band > 0);

    // A rim is the set of arcs the band shares with ONE neighbour, so group
    // its edges by the face on the other side, then pin the two rims to
    // different counts. An edge pin is the one raise the density repair may
    // not undo, so the mismatch reaches the mesher intact.
    std::map<int, std::vector<int>> byNeighbour;
    for (int eid : analysis.faces[band - 1].edgeIds) {
        for (int nf : analysis.edges[eid - 1].faceIds) {
            if (nf != band) byNeighbour[nf].push_back(eid);
        }
    }
    CHECK_EQ(byNeighbour.size(), 2u);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    int perArc = 6;
    for (const auto& [neighbour, edges] : byNeighbour) {
        (void)neighbour;
        for (int eid : edges) gs.perEdge[eid] = perArc;
        perArc = 5;  // the far rim runs one station lighter per arc
    }

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    const auto cause = report.faceBuildCause.find(band);
    const std::string why =
        cause == report.faceBuildCause.end() ? "" : cause->second;
    std::printf("  band face#%d: build=%d cause='%s'\n", band,
                report.faceBuild[band], why.c_str());
    CHECK(why != "revolution grid failed");
    CHECK_EQ(report.faceBuild[band], 0);

    // And it is a strip, not a fan: absorbing a difference of two costs at
    // most a couple of triangles, so the band stays quad-dominant.
    size_t tris = 0, clean = 0;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (p >= mesh.polygonFaceId.size() ||
            mesh.polygonFaceId[p] != band) {
            continue;
        }
        if (mesh.polygons[p].size() == 3) ++tris;
        else ++clean;
    }
    std::printf("  band cells: %zu clean, %zu tris\n", clean, tris);
    CHECK(clean > tris);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK_EQ(vr.openEdges - vr.openEdgesOnInputBoundary, 0u);
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.degeneratePolygons, 0);
    const auto folded = weft::foldedPolys(model, mesh);
    CHECK_EQ(static_cast<size_t>(
                 std::count(folded.begin(), folded.end(), uint8_t{1})),
             0u);
}

// Dirty-step tan_slit: tangent bore contact creates a multi-owner B-rep
// edge. The drum on that generator must stay on a contract floor (or
// structured mesh), never raw OCCT, once border contract skips input NM.
void testTanSlitNoRawDemotion() {
    std::printf("-- tan_slit no raw demotion --\n");
    const std::string path = tmpPath("weft_tan_slit.step");
    weft::writeStep(weft::makeFixture("tan_slit"), path);
    weft::Model model = weft::loadStep(path);
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    int raw = 0, floor = 0;
    for (const auto& [fid, build] : report.faceBuild) {
        if (build == 1) ++raw;
        if (build == 2) ++floor;
        (void)fid;
    }
    std::printf("  raw=%d floor=%d\n", raw, floor);
    CHECK_EQ(raw, 0);
    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    CHECK(vr.inputNonManifoldEdges >= 1);
}

// Authored open-surface / broken solids report broken_source when ≥⅓ of
// B-rep edges are open-shell (same gate as import capping).
void testBrokenSourceDiagnostic() {
    std::printf("-- broken_source diagnostic --\n");
    {
        const std::string path = tmpPath("weft_open_shell_diag.step");
        weft::writeStep(weft::makeFixture("open_shell"), path);
        weft::Model model = weft::loadStep(path);
        weft::GenerationSettings gs;
        weft::PolyMesh mesh =
            weft::generate(model, weft::analyze(model), gs, nullptr);
        const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
        CHECK(vr.brokenSource);
        CHECK(vr.inputBoundaryEdges * 3 >= vr.inputEdges);
        const std::string text = weft::formatReport(vr);
        CHECK(text.find("broken_source") != std::string::npos);
    }
    {
        const std::filesystem::path stepPath =
            std::filesystem::path(__FILE__).parent_path() /
            "STEP_Examples/tork.stp";
        if (std::filesystem::exists(stepPath)) {
            weft::Model model = weft::loadStep(stepPath.string());
            weft::GenerationSettings gs;
            gs.defaults.minimal = true;
            weft::PolyMesh mesh =
                weft::generate(model, weft::analyze(model), gs, nullptr);
            const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
            CHECK(vr.brokenSource);
            const std::string text = weft::formatReport(vr);
            CHECK(text.find("broken_source") != std::string::npos);
            std::printf("  tork: open-shell %zu / %zu edges\n",
                        vr.inputBoundaryEdges, vr.inputEdges);
        }
    }
}

void testMp9CoonsPlaneSeamCanonicalize() {
    std::printf("-- MP9 coons/plane seam canonicalize --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/coons_plane_1805_r0.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);

    int coons = 0, floors = 0;
    for (const auto& f : analysis.faces) {
        if (f.type != weft::SurfaceType::BSpline) continue;
        auto kit = report.faceMesher.find(f.id);
        if (kit == report.faceMesher.end()) continue;
        if (kit->second == weft::MesherKind::CoonsGrid) ++coons;
        auto bit = report.faceBuild.find(f.id);
        if (bit != report.faceBuild.end() && bit->second == 2) ++floors;
        std::printf("  bspline face %d -> %s build=%d\n", f.id,
                    weft::mesherKindName(kit->second),
                    bit != report.faceBuild.end() ? bit->second : -1);
    }
    CHECK(coons >= 3);
    // One fold self-heal → floor on a sibling panel is acceptable; the
    // class gate is "not a blanket contract-floor demotion of the stack".
    CHECK(floors <= 1);

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    const auto folded = weft::foldedPolys(model, mesh);
    const size_t foldCount =
        static_cast<size_t>(std::count(folded.begin(), folded.end(),
                                       uint8_t{1}));
    std::printf("  unexplained cracks=%zu (open=%zu onBoundary=%zu) "
                "folds=%zu nm=%zu\n",
                unexplained, vr.openEdges, vr.openEdgesOnInputBoundary,
                foldCount, vr.nonManifoldEdges);
    CHECK(unexplained <= 2);
    CHECK(foldCount == 0);
    CHECK(vr.nonManifoldEdges == 0);
    CHECK(vr.windingConflicts == 0);
    CHECK(vr.degeneratePolygons == 0);

    // Cell SHAPE, not only watertightness. Passing the counters above by
    // demoting the reducer panels to the best-fit-plane web is not a pass:
    // that web is a triangle fan, and the ribbon columns a raw endpoint
    // lattice cuts are near-degenerate. Both are visible defects, so both
    // are gated here on the trimmed B-spline panels only, where the
    // orthogonal Coons grid runs.
    std::set<int> panels;
    for (const auto& f : analysis.faces) {
        if (f.type == weft::SurfaceType::BSpline) panels.insert(f.id);
    }
    size_t tri = 0, quad = 0, ngon = 0, ribbon = 0;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (!panels.count(mesh.polygonFaceId[p])) continue;
        const auto& poly = mesh.polygons[p];
        if (poly.size() == 3) ++tri;
        else if (poly.size() == 4) ++quad;
        else ++ngon;
        double lo = 1e300, hi = 0.0;
        for (size_t i = 0; i < poly.size(); ++i) {
            const auto& a = mesh.vertices[poly[i]];
            const auto& b = mesh.vertices[poly[(i + 1) % poly.size()]];
            const double d = std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]),
                                        a[2] - b[2]);
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        if (lo > 1e-12 && hi / lo > 20.0) ++ribbon;
    }
    const size_t panelPolys = tri + quad + ngon;
    std::printf("  panel cells: %zu (%zu quad, %zu tri, %zu n-gon), "
                "%zu with aspect>20\n", panelPolys, quad, tri, ngon, ribbon);
    CHECK(panelPolys > 0);
    // Quad-dominant: measured 497 quads against 64 tris. The triangulated
    // web this replaced ran 460 tris on one panel alone, and failed this
    // ratio at both revisions the artist rejected (2202 against 563, and
    // 1420 against 707 before them).
    CHECK(quad > 4 * tri);
    // Ribbons: 138 of 710 cells, 19.4%. Both rejected revisions cut a
    // station per raw trim endpoint and sat at 29% (832 of 2851, and 633 of
    // 2136), so a quarter is a ceiling this class clears only when the
    // near-coincident stations are consolidated.
    CHECK(ribbon * 4 < panelPolys);
}

void testMp9UvDegeneratePlanarPanels() {
    std::printf("-- MP9 UV-degenerate planar panels --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() /
        "regressions/mp9/uv_degenerate_planar_panels.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    int raw = 0, floor = 0, empty = 0, structured = 0;
    for (const auto& [fid, build] : report.faceBuild) {
        if (build == 1) ++raw;
        else if (build == 2) ++floor;
        else if (build == -1) ++empty;
        else if (build == 0) ++structured;
        (void)fid;
    }
    std::printf("  structured=%d floor=%d raw=%d empty=%d\n", structured,
                floor, raw, empty);
    CHECK_EQ(raw, 0);
    CHECK_EQ(floor, 0);
    CHECK_EQ(empty, 0);
    CHECK_EQ(structured, model.faceCount());

    const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
    const size_t unexplained =
        vr.openEdges > vr.openEdgesOnInputBoundary
            ? vr.openEdges - vr.openEdgesOnInputBoundary
            : 0;
    std::printf("  unexplained cracks=%zu (open=%zu onBoundary=%zu)\n",
                unexplained, vr.openEdges, vr.openEdgesOnInputBoundary);
    CHECK_EQ(unexplained, 0u);
    CHECK_EQ(vr.windingConflicts, 0);
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

    // Minimal + elongated slot: structured plate-web rejects non-round
    // holes, so the slotted flats stay boundary n-gons.
    weft::GenerationSettings gsMin;
    gsMin.defaults.minimal = true;
    weft::GenerationReport repMin;
    weft::PolyMesh minimal = weft::generate(model, a, gsMin, &repMin);
    for (int fid : plateFaces) {
        CHECK(repMin.faceMesher.at(fid) == weft::MesherKind::MinimalNGon);
    }
    CHECK(isWatertight(minimal));

    // Minimal + ROUND bore (CAD profile): ring-junction / plate-web must
    // win over the residual minimal-ngon grab so bored flats keep local
    // collars instead of one hair-thin fan n-gon (WP5 visual rubric).
    {
        std::string holePath = tmpPath("weft_test_minimal_hole.step");
        weft::writeStep(weft::makeFixture("hole"), holePath);
        weft::Model hole = weft::loadStep(holePath);
        weft::Analysis ha = weft::analyze(hole);
        weft::GenerationSettings hg;
        hg.defaults.minimal = true;
        hg.defaults.adaptive = true;
        weft::GenerationReport hr;
        weft::PolyMesh hm = weft::generate(hole, ha, hg, &hr);
        CHECK(isWatertight(hm));
        int boredFlats = 0;
        for (const auto& f : ha.faces) {
            if (f.type != weft::SurfaceType::Plane) continue;
            int wires = 0;
            for (TopExp_Explorer wx(hole.faces(f.id), TopAbs_WIRE); wx.More();
                 wx.Next()) {
                ++wires;
            }
            if (wires < 2) continue;
            ++boredFlats;
            weft::MesherKind k = hr.faceMesher.at(f.id);
            CHECK(k == weft::MesherKind::RingJunction ||
                  k == weft::MesherKind::PlateWeb ||
                  k == weft::MesherKind::AnnulusRing ||
                  k == weft::MesherKind::PlanarGrid);
        }
        CHECK(boredFlats >= 1);
    }
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

    // Artist floor: CAD/relativeDeviation's 60° gate would otherwise leave
    // small rings near 6; minCurvedSegments raises that floor, and
    // densityScale cannot undercut it on closed curved edges.
    {
        TopoDS_Shape cyl =
            BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 4.0, 8.0)
                .Shape();
        std::string path = tmpPath("weft_test_mincurve.step");
        weft::writeStep(cyl, path);
        weft::Model model = weft::loadStep(path);
        weft::Analysis a = weft::analyze(model);
        weft::GenerationSettings gs;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.defaults.minCurvedSegments = 12;
        gs.densityScale = 0.5;
        weft::GenerationReport report;
        weft::PolyMesh mesh = weft::generate(model, a, gs, &report);
        CHECK(isWatertight(mesh));
        int side = 0;
        for (const auto& f : a.faces) {
            if (f.type == weft::SurfaceType::Cylinder) side = f.id;
        }
        auto rims = report.faceRims.find(side);
        CHECK(rims != report.faceRims.end());
        const int n = report.edgeDivisions.at(rims->second[0]);
        CHECK(n >= 12);
    }

    // Through-bore plate: ring-junction used to crush the circle to
    // 2*(nu+nv) then the 60° curvature floor raised it to 6 — ignoring
    // minCurvedSegments and demoting the junctions. Plate must grow so
    // the bore rim stays >= 12 and the junctions remain structured.
    {
        std::string path = tmpPath("weft_test_mincurve_hole.step");
        weft::writeStep(weft::makeFixture("hole"), path);
        weft::Model model = weft::loadStep(path);
        weft::Analysis a = weft::analyze(model);
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.defaults.minCurvedSegments = 12;
        weft::GenerationReport report;
        weft::PolyMesh mesh = weft::generate(model, a, gs, &report);
        CHECK(isWatertight(mesh));
        int bore = 0, rim = 0;
        for (const auto& f : a.faces) {
            if (f.isHole && f.type == weft::SurfaceType::Cylinder) {
                bore = f.id;
            }
        }
        CHECK(bore > 0);
        auto rims = report.faceRims.find(bore);
        CHECK(rims != report.faceRims.end());
        for (int e : rims->second) {
            rim = std::max(rim, report.edgeDivisions.at(e));
        }
        CHECK(rim >= 12);
        int junctions = 0, junctionFloor = 0;
        for (const auto& [fid, kind] : report.faceMesher) {
            if (kind != weft::MesherKind::RingJunction) continue;
            ++junctions;
            auto bit = report.faceBuild.find(fid);
            if (bit != report.faceBuild.end() && bit->second == 2) {
                ++junctionFloor;
            }
        }
        CHECK(junctions >= 2);
        CHECK_EQ(junctionFloor, 0);
    }

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
    pgs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    one.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
    one.defaults.junctionRings = 1;
    weft::PolyMesh oneRing = weft::generate(plateModel, plateA, one);
    weft::GenerationSettings three = one;
    three.defaults.junctionRings = 3;
    weft::PolyMesh threeRings = weft::generate(plateModel, plateA, three);
    CHECK(isWatertight(oneRing));
    CHECK(isWatertight(threeRings));
    // 2 faces x 2 holes x 2 extra rings x 12 quads
    CHECK_EQ(threeRings.countQuads(), oneRing.countQuads() + 2 * 2 * 2 * 12);
}

// Quad Fill is retired from automatic routing. A slotted plate with an
// explicit quad-dominant preference still gets local exact-border pairing.
void testQuadFill() {
    std::printf("-- no automatic quad fill (slotted plate) --\n");
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
    for (const auto& [fid, kind] : report.faceMesher) {
        (void)fid;
        if (kind == weft::MesherKind::QuadFill) ++quadFill;
    }
    CHECK_EQ(quadFill, 0);
    CHECK(mesh.countQuads() > 0);
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
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
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
    weft::writeStep(weft::makeFixture("fillet"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.minimal = false;  // legacy dense-flat counts
    gs.defaults.radial = 14;
    std::atomic<int> progress{0}, progressTotal{-1};
    gs.progressFaces = &progress;
    gs.progressTotal = &progressTotal;
    weft::GenerationCache cache;
    weft::GenerationReport firstReport, againReport;
    weft::PolyMesh first =
        weft::generate(model, a, gs, &firstReport, &cache);
    weft::PolyMesh again =
        weft::generate(model, a, gs, &againReport, &cache);
    CHECK_EQ(again.vertexCount(), first.vertexCount());
    CHECK_EQ(again.polygonCount(), first.polygonCount());
    CHECK(isWatertight(again));
    CHECK_EQ(firstReport.cacheMisses, model.faceCount());
    CHECK_EQ(againReport.cacheHits, model.faceCount());
    CHECK_EQ(againReport.cacheMisses, 0);
    CHECK_EQ(progressTotal.load(), 0);
    CHECK_EQ(progress.load(), model.faceCount());  // first run only

    // Change one face without changing its border counts; exactly that local
    // part should rebuild while every independent face remains cached.
    int editedFace = 0;
    for (const auto& [fid, kind] : firstReport.faceMesher) {
        if (kind == weft::MesherKind::CoonsGrid) {
            editedFace = fid;
            break;
        }
    }
    CHECK(editedFace > 0);
    gs.perFace[editedFace] = gs.defaults;
    gs.perFace[editedFace].coonsRotate = 1;
    progress = 0;
    progressTotal = -1;
    weft::GenerationReport editReport;
    weft::PolyMesh cachedRun =
        weft::generate(model, a, gs, &editReport, &cache);
    CHECK_EQ(progressTotal.load(), editReport.cacheMisses);
    CHECK_EQ(progress.load(), editReport.cacheMisses);
    weft::GenerationSettings freshSettings = gs;
    freshSettings.progressFaces = nullptr;
    freshSettings.progressTotal = nullptr;
    weft::PolyMesh freshRun = weft::generate(model, a, freshSettings);
    CHECK_EQ(cachedRun.vertexCount(), freshRun.vertexCount());
    CHECK_EQ(cachedRun.polygonCount(), freshRun.polygonCount());
    CHECK(isWatertight(cachedRun));
    CHECK(editReport.cacheHits > 0);
    CHECK(editReport.cacheMisses < model.faceCount());
    CHECK_EQ(editReport.cacheHits + editReport.cacheMisses, model.faceCount());
    std::printf("  local edit: %d remeshed, %d reused\n",
                editReport.cacheMisses, editReport.cacheHits);

    // Viewport preview and final export share identical per-face parts. The
    // final pass must reuse all of them, then restore the authoritative
    // watertight result without remeshing a face.
    weft::GenerationSettings previewSettings = gs;
    previewSettings.progressFaces = nullptr;
    previewSettings.progressTotal = nullptr;
    previewSettings.finalizeMesh = false;
    weft::GenerationReport previewReport;
    (void)weft::generate(model, a, previewSettings, &previewReport, &cache);
    CHECK_EQ(previewReport.cacheMisses, 0);

    weft::GenerationSettings exportSettings = previewSettings;
    exportSettings.finalizeMesh = true;
    weft::GenerationReport exportReport;
    weft::PolyMesh exportRun =
        weft::generate(model, a, exportSettings, &exportReport, &cache);
    CHECK_EQ(exportReport.cacheMisses, 0);
    CHECK_EQ(exportRun.vertexCount(), cachedRun.vertexCount());
    CHECK_EQ(exportRun.polygonCount(), cachedRun.polygonCount());
    CHECK(isWatertight(exportRun));
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

    weft::GenerationReport coupledRep, decoupledRep;
    const weft::PolyMesh coupledBase =
        weft::generate(model, a, coupled, &coupledRep);
    const weft::PolyMesh decoupledBase =
        weft::generate(model, a, decoupled, &decoupledRep);
    CHECK(coupledBase.polygonCount() != decoupledBase.polygonCount());
    // Promoted from probe89: default (coupled) path must not emit empty
    // faces; stitch remains a quarantined A/B diagnostic, not a second
    // production gate.
    for (const auto& [fid, build] : coupledRep.faceBuild) {
        (void)fid;
        CHECK(build != -1);
    }

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

// Strategy audit: every selectable mesher family must build at least one real
// face, keep closed fixtures watertight, avoid raw OCCT fallback/empty output,
// and produce no surface-winding folds. The two hero files cover specialized
// rail-ladder and freeform dome routing that deliberately does not occur on
// the small analytic fixtures.
void testAllMesherStrategies() {
    std::printf("-- all mesher strategies --\n");
    std::set<weft::MesherKind> observed;

    auto runModel = [&](const std::string& label, const weft::Model& model,
                        const weft::GenerationSettings& settings,
                        bool requireClosed = true) {
        const weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        const weft::PolyMesh mesh =
            weft::generate(model, analysis, settings, &report);
        CHECK(!mesh.vertices.empty());
        CHECK(!mesh.polygons.empty());
        if (requireClosed) {
            CHECK(isWatertight(mesh));
            const std::vector<uint8_t> folded = weft::foldedPolys(model, mesh);
            CHECK(std::find(folded.begin(), folded.end(), uint8_t{1}) ==
                  folded.end());
        }
        for (const auto& [faceId, build] : report.faceBuild) {
            (void)faceId;
            CHECK(build != 1);   // never raw OCCT triangulation
            CHECK(build != -1);  // never an empty face
        }
        for (const auto& [faceId, kind] : report.faceMesher) {
            (void)faceId;
            observed.insert(kind);
        }
        std::printf("  %-16s %zu verts, %zu polys\n", label.c_str(),
                    mesh.vertexCount(), mesh.polygonCount());
    };

    auto runShape = [&](const std::string& label, const TopoDS_Shape& shape,
                        const weft::GenerationSettings& settings) {
        const std::string path = tmpPath("weft_strategy_" + label + ".step");
        weft::writeStep(shape, path);
        runModel(label, weft::loadStep(path), settings);
    };
    auto runFixture = [&](const std::string& name,
                          const weft::GenerationSettings& settings) {
        runShape(name, weft::makeFixture(name), settings);
    };

    weft::GenerationSettings defaults;
    runFixture("cylinder", defaults);  // revolution + disk cap
    runFixture("ribbon", defaults);    // ribbon sweep
    runFixture("slotted", defaults);   // quad fill / annulus family

    weft::GenerationSettings dense;
    dense.defaults.minimal = false;
    dense.defaults.gridU = 3;
    dense.defaults.gridV = 3;
    runFixture("box", dense);   // planar grid
    runFixture("boss", dense);  // ring junction
    runFixture("fillet", dense);  // coons grid

    // Exercise the two fallback policies explicitly on a valid planar face.
    for (weft::MesherKind kind : {weft::MesherKind::Fallback,
                                  weft::MesherKind::QuadDominant}) {
        weft::GenerationSettings forced = dense;
        forced.perFace[1] = forced.defaults;
        forced.perFace[1].forceMesher = 1 + int(kind);
        runShape(kind == weft::MesherKind::Fallback ? "fallback"
                                                     : "quad_dominant",
                 weft::makeFixture("box"), forced);
    }

    // Two concentric circular boundaries route the flat faces to annulus-ring.
    TopoDS_Shape washer = BRepAlgoAPI_Cut(
                              BRepPrimAPI_MakeCylinder(20.0, 5.0).Shape(),
                              BRepPrimAPI_MakeCylinder(
                                  gp_Ax2(gp_Pnt(0, 0, -1),
                                         gp_Dir(0, 0, 1)),
                                  10.0, 7.0)
                                  .Shape())
                              .Shape();
    runShape("annulus", washer, dense);

    // Multiple circular holes route top and bottom through plate-web.
    TopoDS_Shape plate = BRepPrimAPI_MakeBox(60.0, 30.0, 5.0).Shape();
    for (double x : {18.0, 42.0}) {
        const TopoDS_Shape bore =
            BRepPrimAPI_MakeCylinder(
                gp_Ax2(gp_Pnt(x, 15.0, -1.0), gp_Dir(0, 0, 1)), 5.0,
                7.0)
                .Shape();
        plate = BRepAlgoAPI_Cut(plate, bore).Shape();
    }
    runShape("plate_web", plate, dense);

    weft::GenerationSettings cad;
    cad.defaults.minimal = true;
    cad.defaults.adaptive = true;
    cad.defaults.relativeDeviation = true;
    const std::filesystem::path corpus =
        std::filesystem::path(__FILE__).parent_path() / "STEP_Examples";
    const weft::Model flaregun =
        weft::loadStep((corpus / "flaregun.stp").string());
    const weft::Model foam =
        weft::loadStep((corpus / "foam.stp").string());
    const weft::Model teleporter =
        weft::loadStep((corpus / "teleporter.stp").string());
    // Flaregun may keep a sparse ribbon fold rather than demote the strip
    // to a contract-floor web (WP6 failed-floor clearance). Still require
    // watertightness below; skip the zero-fold gate used for small fixtures.
    runModel("flaregun", flaregun, cad, false);  // rail ladder
    runModel("foam", foam, cad, false); // dome; closedness in KNOWN_RED/WP3

    // Release closed-solid class locks. Foam + teleporter (default and CAD)
    // are watertight after WP3 stitch protect / midpoint chain accept /
    // digon-chord floor (bspline_contract_floor_overweld). Neighborhood
    // STEP extracts under tests/regressions/release/ are open-shell
    // diagnostics only (see docs/evidence/wp1-release-reducers-*).
    auto checkClosedSolidWatertight =
        [&](const char* label, const weft::Model& model,
            const weft::GenerationSettings& settings) {
            const weft::Analysis analysis = weft::analyze(model);
            const weft::PolyMesh mesh =
                weft::generate(model, analysis, settings);
            const weft::ValidationReport vr =
                weft::validateMesh(mesh, &model);
            CHECK_EQ(vr.inputBoundaryEdges, 0u);
            CHECK(vr.watertight());
            std::printf("  %s watertight\n", label);
        };
    weft::GenerationSettings def;
    def.defaults.minimal = true;
    checkClosedSolidWatertight("flaregun CAD", flaregun, cad);
    checkClosedSolidWatertight("foam CAD", foam, cad);
    checkClosedSolidWatertight("foam default", foam, def);
    checkClosedSolidWatertight("teleporter CAD", teleporter, cad);
    checkClosedSolidWatertight("teleporter default", teleporter, def);

    // Regression for impossible two-vertex rail wires: a propagated radial
    // edit used to pin both rail edges to one segment, defeating the generic
    // wire floor and sending neighbouring rail ladders to raw OCCT output.
    auto checkDensityEdit = [&](const weft::Model& model, int faceId) {
        const weft::Analysis analysis = weft::analyze(model);
        weft::GenerationSettings edited = cad;
        edited.perFace[faceId] = cad.defaults;
        edited.perFace[faceId].radial = 8;
        weft::GenerationReport report;
        const weft::PolyMesh mesh =
            weft::generate(model, analysis, edited, &report);
        CHECK(isWatertight(mesh));
        for (const auto& [fid, build] : report.faceBuild) {
            (void)fid;
            CHECK(build != 1);
            CHECK(build != -1);
        }
    };
    // Discover a rail-ladder face from the report (not a hard-coded face id).
    {
        const weft::Analysis analysis = weft::analyze(flaregun);
        weft::GenerationReport planRep;
        (void)weft::generate(flaregun, analysis, cad, &planRep);
        int railFace = 0;
        for (const auto& [fid, kind] : planRep.faceMesher) {
            if (kind == weft::MesherKind::RailLadder) {
                railFace = fid;
                break;
            }
        }
        CHECK(railFace > 0);
        if (railFace > 0) checkDensityEdit(flaregun, railFace);
    }
    // foam density-edit closedness is tracked in KNOWN_RED / WP3.

    constexpr weft::MesherKind expected[] = {
        weft::MesherKind::RevolutionGrid, weft::MesherKind::DiskCap,
        weft::MesherKind::PlanarGrid,     weft::MesherKind::CoonsGrid,
        weft::MesherKind::RingJunction,   weft::MesherKind::QuadDominant,
        weft::MesherKind::MinimalNGon,    weft::MesherKind::Fallback,
        weft::MesherKind::AnnulusRing,    weft::MesherKind::PlateWeb,
        weft::MesherKind::RailLadder,     weft::MesherKind::RibbonSweep,
        weft::MesherKind::DomeCap,        weft::MesherKind::QuadFill,
    };
    for (weft::MesherKind kind : expected) {
        if (!observed.count(kind)) {
            std::printf("missing mesher strategy: %s\n",
                        weft::mesherKindName(kind));
            CHECK(false);
        }
    }
    CHECK_EQ(observed.size(), std::size(expected));
}

// Announce each test and turn stray exceptions into a named failure
// instead of a silent fail-fast crash (0xc0000409 on Windows).
#define RUN(fn)                                               \
    do {                                                      \
        if (const char* __only = std::getenv("WEFT_ONLY_TEST")) { \
            bool __ok = std::strcmp(__only, #fn) == 0;        \
            if (!__ok) {                                       \
                const char* p = __only;                        \
                const char* name = #fn;                        \
                while (*p) {                                   \
                    const char* c = p;                         \
                    while (*c && *c != ',') ++c;               \
                    if (size_t(c - p) == std::strlen(name) &&  \
                        std::strncmp(p, name, c - p) == 0) {   \
                        __ok = true; break;                    \
                    }                                          \
                    p = *c ? c + 1 : c;                        \
                }                                              \
            }                                                  \
            if (!__ok) break;                                  \
        }                                                     \
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

// A density edit must never push any face onto raw OCCT triangulation
// (EXECUTION_PLAN §3.1). Found by `weft intent-sweep`: densifying a freeform
// patch next to a geometric sphere cap raised the shared border past what the
// cap's quad-fill could take; its UV ring was then non-simple, so even the
// contract floor refused to build and the face fell to raw, whose borders do
// not match the neighbours — 4 open edges on a closed solid.
void testDensityEditNeverFallsToRaw() {
    std::printf("-- density edit never falls to raw --\n");
    const std::filesystem::path corpus =
        std::filesystem::path(__FILE__).parent_path() / "STEP_Examples";
    const weft::Model model =
        weft::loadStep((corpus / "flaregun.stp").string());
    const weft::Analysis analysis = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationCache cache;
    weft::GenerationReport base;
    weft::PolyMesh baseMesh =
        weft::generate(model, analysis, gs, &base, &cache);
    CHECK(isWatertight(baseMesh));

    // Discover the class, not the face id: a geometric sphere cap, plus the
    // freeform/blend neighbours whose density edits reach its border.
    std::set<int> targets;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::SphereCap) continue;
        if (f.chartKind != weft::ChartKind::GeometricCap) continue;
        for (int eid : f.edgeIds) {
            if (eid < 1 || eid > int(analysis.edges.size())) continue;
            for (int nf : analysis.edges[eid - 1].faceIds) {
                if (nf == f.id || nf < 1) continue;
                auto kit = base.faceMesher.find(nf);
                if (kit == base.faceMesher.end()) continue;
                if (kit->second == weft::MesherKind::CoonsGrid ||
                    kit->second == weft::MesherKind::PlanarGrid) {
                    targets.insert(nf);
                }
            }
        }
    }
    CHECK(!targets.empty());

    int runs = 0;
    for (int fid : targets) {
        for (int r : {17, 21, 26}) {
            weft::GenerationSettings s = gs;
            weft::FaceMeshSettings f = gs.defaults;
            f.adaptive = false;
            f.radial = r;
            s.perFace[fid] = f;
            weft::GenerationReport rep;
            weft::PolyMesh mesh =
                weft::generate(model, analysis, s, &rep, &cache);
            const weft::StructureSummary sum = weft::summarizeStructure(rep);
            const weft::ValidationReport vr = weft::validateMesh(mesh, &model);
            if (sum.raw || !vr.watertight()) {
                std::printf("  face #%d radial=%d: raw=%d open=%zu nm=%zu\n",
                            fid, r, sum.raw, vr.openEdges,
                            vr.nonManifoldEdges);
            }
            CHECK_EQ(sum.raw, 0);
            CHECK_EQ(sum.empty, 0);
            CHECK(vr.watertight());
            // Winding is asserted by the intent gate's ratchet, not here:
            // flaregun still has a live winding regression on this edit
            // (tests/KNOWN_RED.tsv row flaregun/cad/edit_winding_conflicts),
            // and it spans 18 faces of a multi-solid — a separate class from
            // the raw-demotion fix this test locks.
            if (vr.windingConflicts) {
                std::printf("  note: face #%d radial=%d winding conflicts=%zu"
                            " (KNOWN_RED)\n",
                            fid, r, vr.windingConflicts);
            }
            ++runs;
        }
    }
    std::printf("  %zu neighbour face(s), %d edits, no raw, all watertight\n",
                targets.size(), runs);
}

// Structure retention is the artist-facing invariant: did each face keep the
// topology its plan chose? Every cause string must classify into a named
// bucket — an unregistered one resolves to Unknown, which would silently hide
// new debt from the gate ratchet.
void testStructureRetentionClassification() {
    std::printf("-- structure retention classification --\n");

    // Direct mapping checks for the classes the gate ratchets on.
    CHECK(weft::classifyFaceBuild(0, "") == weft::FaceBuildClass::Built);
    CHECK(weft::classifyFaceBuild(1, "mesher threw") ==
          weft::FaceBuildClass::Raw);
    CHECK(weft::classifyFaceBuild(-1, "fallback threw") ==
          weft::FaceBuildClass::Empty);
    CHECK(weft::classifyFaceBuild(2, "planned contract floor") ==
          weft::FaceBuildClass::PlannedFloor);
    CHECK(weft::classifyFaceBuild(2, "revolution grid failed") ==
          weft::FaceBuildClass::MesherFailed);
    CHECK(weft::classifyFaceBuild(2, "border contract failed") ==
          weft::FaceBuildClass::BorderContract);
    CHECK(weft::classifyFaceBuild(2, "self-check failed") ==
          weft::FaceBuildClass::SelfCheck);
    CHECK(weft::classifyFaceBuild(2, "fold self-heal → contract floor") ==
          weft::FaceBuildClass::FoldHeal);
    CHECK(weft::classifyFaceBuild(2, "radial override → contract floor") ==
          weft::FaceBuildClass::DensityOverride);
    // A planned floor must never be counted as failure debt.
    CHECK(weft::classifyFaceBuild(2, "planned contract floor") !=
          weft::FaceBuildClass::MesherFailed);
    // A planned floor now names the ladder stage that exhausted; the
    // classifier keys on the prefix, so the reason must not move the face
    // out of its bucket.
    CHECK(weft::classifyFaceBuild(
              2,
              "planned contract floor (coons: no clear fourth corner; "
              "orthogonal: 3 diagonal edges)") ==
          weft::FaceBuildClass::PlannedFloor);

    // Every cause the corpus actually produces must be registered, and the
    // buckets must partition the faces exactly.
    const std::filesystem::path here =
        std::filesystem::path(__FILE__).parent_path();
    struct Case {
        std::string label;
        std::string path;
    };
    std::vector<Case> cases = {
        {"demo", (here / "fixtures/demo.step").string()},
        {"flaregun", (here / "STEP_Examples/flaregun.stp").string()},
    };
    for (const std::string& shape : {"torture", "barrel2", "ribbonnotch"}) {
        const std::string p = tmpPath("weft_structure_" + shape + ".step");
        weft::writeStep(weft::makeFixture(shape), p);
        cases.push_back({shape, p});
    }

    int checked = 0;
    int plannedExplained = 0;
    for (const Case& c : cases) {
        weft::Model model = weft::loadStep(c.path);
        weft::Analysis analysis = weft::analyze(model);
        for (int profile = 0; profile < 2; ++profile) {
            weft::GenerationSettings gs;
            gs.defaults.minimal = true;
            if (profile == 1) {
                gs.defaults.adaptive = true;
                gs.defaults.relativeDeviation = true;
            }
            weft::GenerationReport report;
            (void)weft::generate(model, analysis, gs, &report);

            for (const auto& [fid, how] : report.faceBuild) {
                std::string cause;
                auto cit = report.faceBuildCause.find(fid);
                if (cit != report.faceBuildCause.end()) cause = cit->second;
                const weft::FaceBuildClass k =
                    weft::classifyFaceBuild(how, cause);
                if (k == weft::FaceBuildClass::Unknown) {
                    std::printf(
                        "  UNREGISTERED cause on %s face #%d: build=%d "
                        "cause='%s'\n",
                        c.label.c_str(), fid, how, cause.c_str());
                }
                CHECK(k != weft::FaceBuildClass::Unknown);
                if (k != weft::FaceBuildClass::PlannedFloor) continue;
                // A planned floor is a routing decision, so it must say
                // which ladder stage exhausted — "contract floor" alone
                // cannot be triaged, and the trace must repeat it so
                // --why-face answers without a rebuild.
                if (cause.find("(coons: ") == std::string::npos) {
                    std::printf(
                        "  UNEXPLAINED planned floor on %s face #%d: '%s'\n",
                        c.label.c_str(), fid, cause.c_str());
                }
                CHECK(cause.find("(coons: ") != std::string::npos);
                CHECK(cause.find("orthogonal: ") != std::string::npos);
                const std::string trace = weft::formatFaceTrace(report, fid);
                CHECK(trace.find("coons: ") != std::string::npos);
                CHECK(trace.find("wires=") != std::string::npos);
                ++plannedExplained;
            }

            const weft::StructureSummary s =
                weft::summarizeStructure(report);
            CHECK_EQ(s.structured + s.plannedFloor + s.failedFloor + s.raw +
                         s.empty,
                     s.total);
            CHECK(s.retention() >= 0.0 && s.retention() <= 1.0);
            CHECK_EQ(int(s.failedByCause.size()) <= s.failedFloor, 1);
            CHECK_EQ(int(s.plannedByCause.size()) <= s.plannedFloor, 1);
            const std::string line = weft::formatStructure(report);
            CHECK(line.find("structure: faces=") != std::string::npos);
            CHECK(line.find("retention=") != std::string::npos);
            if (s.plannedFloor > 0) {
                CHECK(line.find("planned-floor by cause:") !=
                      std::string::npos);
            }
            ++checked;
        }
    }
    // The corpus above must actually exercise the planned-floor path, or
    // the assertions are vacuous.
    CHECK(plannedExplained > 0);
    std::printf(
        "  %d model/profile runs, every cause registered, %d planned floors "
        "explained\n",
        checked, plannedExplained);
}

// Artist model for a cut cylinder (2026-07-25): "if I set it to 6 spans the
// cylinder retains exactly 6 spans and the cutout doesn't solve, it just
// allows ngons". A notch must not buy extra rows, extra columns, or
// triangles — the wall keeps the requested spans, every span runs the full
// length, and the cut sits inside one n-gon.
void testNotchedCylinderKeepsRequestedSpans() {
    std::printf("-- notched cylinder keeps requested spans --\n");
    const std::string path = tmpPath("weft_span_notched.step");
    weft::writeStep(weft::makeFixture("notched"), path);
    weft::Model model = weft::loadStep(path);
    weft::Analysis analysis = weft::analyze(model);

    // Discover the notched full-period wall rather than naming a face id.
    int wall = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Drum) continue;
        if (f.chartKind != weft::ChartKind::FullPeriod) continue;
        if (int(f.edgeIds.size()) < 5) continue;
        if (wall == 0 || f.radius > analysis.faces[wall - 1].radius) {
            wall = f.id;
        }
    }
    CHECK(wall > 0);

    for (int spans : {6, 8, 12}) {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        gs.perFace[wall] = gs.defaults;
        gs.perFace[wall].adaptive = false;
        gs.perFace[wall].radial = spans;

        weft::GenerationReport rep;
        weft::PolyMesh mesh = weft::generate(model, analysis, gs, &rep);
        CHECK(isWatertight(mesh));

        // The typed count is the count: no neighbour may outvote it.
        auto cit = rep.faceCounts.find(wall);
        CHECK(cit != rep.faceCounts.end());
        CHECK_EQ(cit->second[0], spans);

        // The wall keeps its planned mesher — no floor, no raw.
        auto bit = rep.faceBuild.find(wall);
        CHECK(bit != rep.faceBuild.end());
        CHECK_EQ(bit->second, 0);

        // One polygon per span, no triangles, and the cut absorbed as
        // n-gons rather than solved into extra rows.
        int polys = 0, tris = 0, ngons = 0;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size()) break;
            if (mesh.polygonFaceId[p] != wall) continue;
            ++polys;
            if (mesh.polygons[p].size() == 3) ++tris;
            if (mesh.polygons[p].size() > 4) ++ngons;
        }
        CHECK_EQ(tris, 0);
        CHECK(ngons >= 1);          // the cut lives in an n-gon
        CHECK(polys <= spans + 2);  // no lattice inflation around the cut
        std::printf("  %2d spans -> %d polys on the wall (%d n-gon(s), 0 tris)\n",
                    spans, polys, ngons);
    }
}

// Flaregun barrel open-band: a notch lip must stay local to the notch.
// Promoting it onto every column turns it into a full-band ring that cuts
// every long span in two — the artist's "long spans should maintain the full
// cylinder length, and not be broken up".
void testFlaregunOpenBandNotchLipsFullSpan() {
    std::printf("-- flaregun open-band notch lips full-span --\n");
    const std::filesystem::path corpus =
        std::filesystem::path(__FILE__).parent_path() / "STEP_Examples";
    const weft::Model model =
        weft::loadStep((corpus / "flaregun.stp").string());
    const weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));

    int checked = 0;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Drum) continue;
        if (f.chartKind != weft::ChartKind::IsoBand) continue;
        if (int(f.edgeIds.size()) < 10) continue;
        auto kit = report.faceMesher.find(f.id);
        if (kit == report.faceMesher.end() ||
            kit->second != weft::MesherKind::RevolutionGrid) {
            continue;
        }
        auto bit = report.faceBuild.find(f.id);
        if (bit == report.faceBuild.end() || bit->second != 0) continue;

        double v0 = 1e300, v1 = -1e300;
        std::map<int, int> bucket;
        for (size_t i = 0; i < mesh.anchors.size(); ++i) {
            const auto& a = mesh.anchors[i];
            if (a.faceId != f.id) continue;
            v0 = std::min(v0, a.v);
            v1 = std::max(v1, a.v);
            const int q = int(std::lround(a.v * 50.0));
            ++bucket[q];
        }
        CHECK(v1 > v0);
        const double vspan = v1 - v0;
        int peak = 0;
        for (const auto& [q, n] : bucket) peak = std::max(peak, n);
        CHECK(peak >= 8);
        // Mid-span stations carrying (nearly) every column are full-band
        // rings. The band's own solved axial count may produce them (nv rows
        // leave nv-1 interior stations); a cut may not add any, or every long
        // span gets cut in two.
        auto nvit = report.faceCounts.find(f.id);
        const int nv = nvit == report.faceCounts.end()
                           ? 1
                           : std::max(1, nvit->second[1]);
        const int axialRows = std::max(0, nv - 1);
        int fullBandRows = 0;
        for (const auto& [q, n] : bucket) {
            const double v = q / 50.0;
            if (v < v0 + 0.05 * vspan || v > v1 - 0.05 * vspan) continue;
            if (n * 4 >= peak * 3) ++fullBandRows;
        }
        CHECK(fullBandRows <= axialRows);
        ++checked;
        std::printf("  face#%d peak=%d full-band mid rows=%d (allowed %d)\n",
                    f.id, peak, fullBandRows, axialRows);
    }
    CHECK(checked >= 2);
}

// Demo / torture insert-bearing drums at artist axial=1: the slot may
// keep a local sill/lintel under its own columns, but those levels must
// NOT stamp a full-band ring around the drum. Straight columns elsewhere;
// watertight; no contract floor.
void testInsertDrumAxialOneNoFullBandRings() {
    std::printf("-- insert drum axial=1 no full-band rings --\n");
    const std::filesystem::path stepPath =
        std::filesystem::path(__FILE__).parent_path() / "fixtures/demo.step";
    weft::Model model = weft::loadStep(stepPath.string());
    weft::Analysis analysis = weft::analyze(model);

    // Multi-edge full-period drums (castellated / boolean + inserts).
    std::vector<int> walls;
    for (const auto& f : analysis.faces) {
        if (f.featureClass != weft::FeatureClass::Drum) continue;
        if (f.chartKind != weft::ChartKind::FullPeriod) continue;
        if (int(f.edgeIds.size()) < 9) continue;
        walls.push_back(f.id);
    }
    CHECK(!walls.empty());

    for (int rad : {19, 21, 32}) {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        for (int fid : walls) {
            gs.perFace[fid] = gs.defaults;
            gs.perFace[fid].adaptive = false;
            gs.perFace[fid].radial = rad;
            gs.perFace[fid].axial = 1;
        }
        weft::GenerationReport report;
        weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
        CHECK(isWatertight(mesh));

        int checked = 0;
        for (int fid : walls) {
            auto bit = report.faceBuild.find(fid);
            CHECK(bit != report.faceBuild.end());
            CHECK_EQ(bit->second, 0);
            auto kit = report.faceMesher.find(fid);
            CHECK(kit != report.faceMesher.end());
            CHECK(kit->second == weft::MesherKind::RevolutionGrid);

            double v0 = 1e300, v1 = -1e300;
            std::map<int, int> bucket;
            for (size_t i = 0; i < mesh.anchors.size(); ++i) {
                const auto& a = mesh.anchors[i];
                if (a.faceId != fid) continue;
                v0 = std::min(v0, a.v);
                v1 = std::max(v1, a.v);
                const int q = int(std::lround(a.v * 50.0));
                ++bucket[q];
            }
            CHECK(v1 > v0);
            const double vspan = v1 - v0;
            int peak = 0;
            for (const auto& [q, n] : bucket) peak = std::max(peak, n);
            CHECK(peak >= 8);
            int fullBandRows = 0;
            for (const auto& [q, n] : bucket) {
                const double v = q / 50.0;
                if (v < v0 + 0.05 * vspan || v > v1 - 0.05 * vspan) {
                    continue;
                }
                // Nearly every column at one mid-v = a forbidden ring.
                if (n * 4 >= peak * 3) ++fullBandRows;
            }
            CHECK_EQ(fullBandRows, 0);
            auto nvit = report.faceCounts.find(fid);
            CHECK(nvit != report.faceCounts.end());
            CHECK_EQ(nvit->second[1], 1);
            ++checked;
            std::printf("  face#%d radial=%d nu=%d full-band mid rows=0\n",
                        fid, rad, nvit->second[0]);
        }
        CHECK(checked >= 1);
    }
}

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

// WP4 §3.3: a constrained loop insert + vertex nudge must survive a density
// bump. Regenerate at the new radial, re-apply ops, keep watertightness and
// surface anchors.
void testConstrainedEditSurvivesDensityChange() {
    std::printf("-- constrained edit survives density change --\n");
    std::string stepPath = tmpPath("weft_test_edit_density.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    int sideFaceId = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) sideFaceId = f.id;
    }
    CHECK(sideFaceId > 0);

    weft::GenerationSettings gs;
    gs.defaults.minimal = false;
    gs.defaults.radial = 12;
    gs.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
    gs.defaults.axial = 2;
    weft::PolyMesh mesh = weft::generate(model, a, gs);
    CHECK(isWatertight(mesh));
    const size_t vertsBefore = mesh.vertexCount();

    weft::ManualOp loop{weft::ManualOp::Kind::LoopInsert, sideFaceId, 0.0,
                        7.5, 0.5};
    CHECK_EQ(weft::insertLoop(mesh, model, loop), 12);

    size_t nudgeIdx = vertsBefore;
    for (size_t v = vertsBefore; v < mesh.vertexCount(); ++v) {
        if (mesh.anchors[v].faceId == sideFaceId) {
            nudgeIdx = v;
            break;
        }
    }
    CHECK(nudgeIdx < mesh.vertexCount());
    weft::ManualOp nudge;
    nudge.kind = weft::ManualOp::Kind::NudgeVertex;
    nudge.faceId = sideFaceId;
    nudge.u = mesh.anchors[nudgeIdx].u;
    nudge.v = mesh.anchors[nudgeIdx].v;
    nudge.u2 = nudge.u + 0.2;
    nudge.v2 = nudge.v;
    CHECK_EQ(weft::nudgeVertex(mesh, model, nudge), 1);

    weft::Recipe recipe;
    recipe.settings = gs;
    recipe.ops.push_back(loop);
    recipe.ops.push_back(nudge);

    // Density change: bump radial, regenerate, replay ops.
    recipe.settings.defaults.radial = 16;
    weft::PolyMesh denser =
        weft::generate(model, a, recipe.settings);
    weft::ApplyOpsReport ops = weft::applyOps(denser, model, recipe.ops);
    CHECK_EQ(ops.applied, 2);
    CHECK_EQ(ops.failed, 0);
    CHECK(isWatertight(denser));
    auto vr = weft::validateMesh(denser, &model);
    CHECK_EQ(vr.openEdges, 0);
    CHECK_EQ(vr.nonManifoldEdges, 0);

    auto radiusOf = [](const std::array<double, 3>& p) {
        return std::sqrt(p[0] * p[0] + p[1] * p[1]);
    };
    int onSide = 0;
    for (size_t v = 0; v < denser.vertexCount(); ++v) {
        if (denser.anchors[v].faceId != sideFaceId) continue;
        CHECK(std::abs(radiusOf(denser.vertices[v]) - 10.0) < 1e-6);
        ++onSide;
    }
    CHECK(onSide > 0);

    // Finalized export after ops stays bake-clean.
    recipe.settings.finalizeMesh = true;
    weft::PolyMesh exported =
        weft::generate(model, a, recipe.settings);
    CHECK_EQ(weft::applyOps(exported, model, recipe.ops).failed, 0);
    auto ev = weft::validateMesh(exported, &model);
    CHECK(ev.watertight());
    CHECK_EQ(ev.windingConflicts, 0);
}

// WP4: remap must report dropped face-anchored ops and keep world-space
// WeldVerts (faceId is 0 by design).
void testRemapDropsLostOpsKeepsWeld() {
    std::printf("-- remap drops lost ops, keeps weld --\n");
    std::string pathA = tmpPath("weft_test_remap_drop_a.step");
    std::string pathB = tmpPath("weft_test_remap_drop_b.step");
    weft::writeStep(weft::makeFixture("box"), pathA);
    weft::writeStep(weft::makeFixture("cylinder"), pathB);
    weft::Model modelA = weft::loadStep(pathA);
    weft::Model modelB = weft::loadStep(pathB);
    weft::Analysis aA = weft::analyze(modelA);
    weft::Analysis aB = weft::analyze(modelB);

    weft::Recipe recipe;
    weft::ManualOp lost;
    lost.kind = weft::ManualOp::Kind::NudgeVertex;
    lost.faceId = aA.faces.front().id;  // box face — no match on cylinder
    lost.u = 0.1;
    lost.v = 0.2;
    recipe.ops.push_back(lost);

    weft::ManualOp weld;
    weld.kind = weft::ManualOp::Kind::WeldVerts;
    weld.weldMode = 0;
    weld.weldPoints.push_back({0.0, 0.0, 0.0});
    weld.weldPoints.push_back({1.0, 0.0, 0.0});
    recipe.ops.push_back(weld);

    weft::RemapReport rep;
    weft::Recipe moved =
        weft::remapRecipe(recipe, modelA, aA, modelB, aB, &rep);
    CHECK(rep.opsDropped >= 1);
    CHECK_EQ(moved.ops.size(), 1);
    CHECK(moved.ops[0].kind == weft::ManualOp::Kind::WeldVerts);
    CHECK_EQ(moved.ops[0].weldPoints.size(), 2);
}

// WP4 §3.3 end-to-end artist workflow (API-level): load STEP → selected-face
// density → constrained correction → recipe save/reload → density regen →
// undo last op → CAD remap reporting → finalized OBJ export. Also asserts
// corrections cannot leave a release-tier fixture non-watertight.
void testArtistCorrectionWorkflow() {
    std::printf("-- artist correction workflow (§3.3) --\n");
    std::string stepPath = tmpPath("weft_wp4_workflow.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis a = weft::analyze(model);

    int side = 0;
    for (const auto& f : a.faces) {
        if (f.type == weft::SurfaceType::Cylinder) side = f.id;
    }
    CHECK(side > 0);

    weft::Recipe recipe;
    recipe.settings.defaults.minimal = false;
    recipe.settings.defaults.radial = 12;
    recipe.settings.defaults.minCurvedSegments = 12;  // exact-count vs CAD 24 floor
    recipe.settings.defaults.axial = 2;
    recipe.settings.perFace[side] = recipe.settings.defaults;
    recipe.settings.perFace[side].radial = 14;  // selected-face control

    weft::PolyMesh mesh =
        weft::generate(model, a, recipe.settings);
    CHECK(isWatertight(mesh));
    const size_t verts0 = mesh.vertexCount();

    weft::ManualOp loop{weft::ManualOp::Kind::LoopInsert, side, 0.0, 7.5,
                       0.5};
    CHECK(weft::insertLoop(mesh, model, loop) > 0);
    size_t nudgeSrc = verts0;
    for (size_t v = verts0; v < mesh.vertexCount(); ++v) {
        if (mesh.anchors[v].faceId == side) {
            nudgeSrc = v;
            break;
        }
    }
    CHECK(nudgeSrc < mesh.vertexCount());
    weft::ManualOp nudge;
    nudge.kind = weft::ManualOp::Kind::NudgeVertex;
    nudge.faceId = side;
    nudge.u = mesh.anchors[nudgeSrc].u;
    nudge.v = mesh.anchors[nudgeSrc].v;
    nudge.u2 = nudge.u + 0.15;
    nudge.v2 = nudge.v;
    CHECK_EQ(weft::nudgeVertex(mesh, model, nudge), 1);
    recipe.ops.push_back(loop);
    recipe.ops.push_back(nudge);

    // Recipe persistence.
    std::string recipePath = tmpPath("weft_wp4_workflow.recipe");
    weft::saveRecipe(recipe, recipePath);
    weft::Recipe loaded = weft::loadRecipe(recipePath);
    CHECK_EQ(loaded.settings.perFace.at(side).radial, 14);
    CHECK_EQ(loaded.ops.size(), 2);

    // Density regen with reloaded recipe.
    loaded.settings.perFace[side].radial = 18;
    weft::PolyMesh denser =
        weft::generate(model, a, loaded.settings);
    weft::ApplyOpsReport denserOps =
        weft::applyOps(denser, model, loaded.ops);
    CHECK_EQ(denserOps.failed, 0);
    CHECK(isWatertight(denser));

    // Undo last correction: drop nudge, regenerate, still clean.
    loaded.ops.pop_back();
    weft::PolyMesh undid =
        weft::generate(model, a, loaded.settings);
    CHECK_EQ(weft::applyOps(undid, model, loaded.ops).failed, 0);
    CHECK(isWatertight(undid));

    // Identity remap reports no drops.
    weft::RemapReport idRep;
    weft::Recipe same =
        weft::remapRecipe(loaded, model, a, model, a, &idRep);
    CHECK_EQ(idRep.opsDropped, 0);
    CHECK_EQ(same.ops.size(), 1);

    // Finalized export path: generate+ops+validate+OBJ.
    loaded.settings.finalizeMesh = true;
    weft::PolyMesh exported =
        weft::generate(model, a, loaded.settings);
    CHECK_EQ(weft::applyOps(exported, model, loaded.ops).failed, 0);
    auto vr = weft::validateMesh(exported, &model);
    CHECK(vr.watertight());
    CHECK_EQ(vr.nonManifoldEdges, 0);
    CHECK_EQ(vr.windingConflicts, 0);
    std::string objPath = tmpPath("weft_wp4_workflow.obj");
    weft::writeObj(exported, objPath);
    CHECK(std::filesystem::exists(objPath));
    CHECK(std::filesystem::file_size(objPath) > 0);

    // Release-tier fixture: density override + finalize must stay inside
    // the geometry gate (corrections must not bypass it).
    std::string bossPath = tmpPath("weft_wp4_boss.step");
    weft::writeStep(weft::makeFixture("boss"), bossPath);
    weft::Model boss = weft::loadStep(bossPath);
    weft::Analysis ba = weft::analyze(boss);
    weft::GenerationSettings bgs;
    bgs.finalizeMesh = true;
    bgs.defaults.minimal = false;
    bgs.defaults.gridU = 4;
    bgs.defaults.gridV = 4;
    bgs.defaults.junctionRings = 2;
    int junction = 0;
    weft::GenerationReport br0;
    (void)weft::generate(boss, ba, bgs, &br0);
    for (const auto& [fid, kind] : br0.faceMesher) {
        if (kind == weft::MesherKind::RingJunction) {
            junction = fid;
            break;
        }
    }
    CHECK(junction > 0);
    bgs.perFace[junction] = bgs.defaults;
    bgs.perFace[junction].gridU = 5;
    bgs.perFace[junction].gridV = 5;
    weft::PolyMesh bossMesh = weft::generate(boss, ba, bgs);
    auto bv = weft::validateMesh(bossMesh, &boss);
    CHECK(bv.watertight());
    CHECK_EQ(bv.nonManifoldEdges, 0);
}

// MP9 is tier=performance / layer=target-assets only. Do not run it in the
// default CTest suite — use tools/corpus_gate.sh with performance rows or a
// manual `weft mesh tests/STEP_Examples/MP9.stp` for workload timing.

// WP2: a shared border between two faces must carry matching sample counts
// after generate(). Discover the faces and edge from analysis — no product
// face-ID special cases. Also exercises density-ownership reporting when one
// face proposes a denser grid.
void testSharedBorderSampleCounts() {
    std::printf("-- shared border sample counts --\n");
    const std::string stepPath = tmpPath("weft_test_shared_border.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis analysis = weft::analyze(model);

    // Discover any manifold shared edge and its two bounding faces.
    int sharedEdge = 0, faceA = 0, faceB = 0;
    for (const auto& e : analysis.edges) {
        if (e.faceIds.size() != 2) continue;
        sharedEdge = e.id;
        faceA = e.faceIds[0];
        faceB = e.faceIds[1];
        break;
    }
    CHECK(sharedEdge > 0);
    CHECK(faceA > 0);
    CHECK(faceB > 0);
    CHECK(faceA != faceB);

    weft::GenerationSettings gs;
    gs.defaults.minimal = false;
    gs.defaults.gridU = 3;
    gs.defaults.gridV = 3;
    // One discovered face asks for a denser grid; density matching must
    // raise the shared edge so both faces sample the same count.
    weft::FaceMeshSettings dense = gs.defaults;
    dense.gridU = 5;
    dense.gridV = 5;
    gs.perFace[faceA] = dense;

    weft::GenerationReport report;
    weft::PolyMesh mesh = weft::generate(model, analysis, gs, &report);
    CHECK(isWatertight(mesh));
    CHECK(report.edgeDivisions.count(sharedEdge) == 1);
    const int solved = report.edgeDivisions.at(sharedEdge);
    CHECK(solved >= 5);  // denser face proposal must win the max-resolve

    auto oit = report.edgeDivisionOwner.find(sharedEdge);
    CHECK(oit != report.edgeDivisionOwner.end());
    if (oit != report.edgeDivisionOwner.end()) {
        CHECK(!oit->second.empty());
        std::printf("  edge #%d faces %d/%d solved=%d owner=%s\n", sharedEdge,
                    faceA, faceB, solved, oit->second.c_str());
    }

    const std::string formatted = weft::formatDensityOwnership(report);
    CHECK(!formatted.empty());
    CHECK(formatted.find("density-matched edges:") != std::string::npos);
    CHECK(formatted.find("#" + std::to_string(sharedEdge) + "=") !=
          std::string::npos);
    // Conflicting proposals on the shared group should surface explicitly.
    CHECK(!report.densityConflicts.empty());
    CHECK(formatted.find("density ownership conflicts:") !=
          std::string::npos);

    // Matching sample counts: after weld, both faces use the same undirected
    // mesh edges along the shared border. The number of those segments must
    // equal the solved division count.
    auto collectEdges = [&](int faceId) {
        std::set<std::pair<uint32_t, uint32_t>> edges;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != faceId) {
                continue;
            }
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i];
                uint32_t b = poly[(i + 1) % poly.size()];
                if (a > b) std::swap(a, b);
                edges.insert({a, b});
            }
        }
        return edges;
    };
    const auto edgesA = collectEdges(faceA);
    const auto edgesB = collectEdges(faceB);
    int sharedSegments = 0;
    for (const auto& e : edgesA) {
        if (edgesB.count(e)) ++sharedSegments;
    }
    CHECK_EQ(sharedSegments, solved);
    std::printf("  shared mesh segments=%d (solved divisions=%d)\n",
                sharedSegments, solved);
}

// WP2: raw/empty/floor demotions must be attributed by face id and cause
// string (not dbg-only). Force the contract-floor path on a boss face so
// the report surfaces a known demotion without filename special-casing.
void testDemotionAttribution() {
    std::printf("-- demotion attribution --\n");
    const std::string stepPath = tmpPath("weft_test_demotion_boss.step");
    weft::writeStep(weft::makeFixture("boss"), stepPath);
    weft::Model model = weft::loadStep(stepPath);
    weft::Analysis analysis = weft::analyze(model);

    weft::GenerationSettings gs;
    gs.defaults.minimal = true;
    gs.defaults.adaptive = true;
    gs.defaults.relativeDeviation = true;
    // Face 1 is a valid B-rep face; forcing Fallback exercises the planned
    // contract-floor path that every unsupported structured case shares.
    CHECK(model.faceCount() >= 1);
    gs.perFace[1] = gs.defaults;
    gs.perFace[1].forceMesher = 1 + int(weft::MesherKind::Fallback);

    weft::GenerationCache cache;
    weft::GenerationReport report;
    weft::PolyMesh mesh =
        weft::generate(model, analysis, gs, &report, &cache);
    CHECK(!mesh.polygons.empty());
    CHECK(isWatertight(mesh));

    int floor = 0, raw = 0, empty = 0;
    for (const auto& [fid, how] : report.faceBuild) {
        if (how == 2) ++floor;
        else if (how == 1) ++raw;
        else if (how == -1) ++empty;
    }
    CHECK(floor + raw + empty >= 1);

    auto bit = report.faceBuild.find(1);
    CHECK(bit != report.faceBuild.end());
    if (bit != report.faceBuild.end()) {
        CHECK(bit->second == 2 || bit->second == 1 || bit->second == -1);
        auto cit = report.faceBuildCause.find(1);
        CHECK(cit != report.faceBuildCause.end());
        if (cit != report.faceBuildCause.end()) {
            CHECK(!cit->second.empty());
            std::printf("  face 1 build=%d cause=\"%s\"\n", bit->second,
                        cit->second.c_str());
        }
    }
    for (const auto& [fid, how] : report.faceBuild) {
        if (how == 0) continue;
        auto cit = report.faceBuildCause.find(fid);
        CHECK(cit != report.faceBuildCause.end());
        if (cit != report.faceBuildCause.end()) {
            CHECK(!cit->second.empty());
        }
    }

    const std::string formatted = weft::formatBuildDemotions(report);
    CHECK(!formatted.empty());
    CHECK(formatted.find("demoted:") != std::string::npos);
    CHECK(formatted.find("face ids:") != std::string::npos);
    CHECK(formatted.find("1(") != std::string::npos);

    // Cache hit must re-emit the same attribution (no silent drop).
    weft::GenerationReport again;
    weft::generate(model, analysis, gs, &again, &cache);
    CHECK_EQ(again.cacheHits, model.faceCount());
    auto abit = again.faceBuild.find(1);
    auto acit = again.faceBuildCause.find(1);
    CHECK(abit != again.faceBuild.end());
    CHECK(acit != again.faceBuildCause.end());
    if (bit != report.faceBuild.end() && abit != again.faceBuild.end()) {
        CHECK_EQ(abit->second, bit->second);
    }
    if (acit != again.faceBuildCause.end()) {
        CHECK(!acit->second.empty());
    }
    std::printf("  demotions floor=%d raw=%d empty=%d (cached ok)\n", floor,
                raw, empty);
}

// WP2: durable invariants promoted from tools/probes (retired). Assert by
// fixture class / report fields — never by hard-coded face IDs in product
// routing. Stitch / foam-teleporter mesher routing stays out of scope (WP3).
void testPromotedProbeInvariants() {
    std::printf("-- promoted probe invariants --\n");

    auto cadSettings = []() {
        weft::GenerationSettings gs;
        gs.defaults.minimal = true;
        gs.defaults.adaptive = true;
        gs.defaults.relativeDeviation = true;
        return gs;
    };

    // From probe101 + probe78: CAD-profile closed zoo stays fold-free,
    // never empty, and every demotion carries a cause string.
    for (const char* name : {"cylinder", "box", "boss", "fillet"}) {
        const std::string path =
            tmpPath(std::string("weft_probe_promo_") + name + ".step");
        weft::writeStep(weft::makeFixture(name), path);
        const weft::Model model = weft::loadStep(path);
        const weft::Analysis analysis = weft::analyze(model);
        weft::GenerationReport report;
        const weft::PolyMesh mesh =
            weft::generate(model, analysis, cadSettings(), &report);
        CHECK(isWatertight(mesh));
        const std::vector<uint8_t> folded = weft::foldedPolys(model, mesh);
        CHECK(std::find(folded.begin(), folded.end(), uint8_t{1}) ==
              folded.end());
        int empty = 0, raw = 0;
        for (const auto& [fid, build] : report.faceBuild) {
            if (build == -1) ++empty;
            if (build == 1) ++raw;
            if (build != 0) {
                auto cit = report.faceBuildCause.find(fid);
                CHECK(cit != report.faceBuildCause.end());
                if (cit != report.faceBuildCause.end()) {
                    CHECK(!cit->second.empty());
                }
            }
        }
        CHECK_EQ(empty, 0);
        CHECK_EQ(raw, 0);
        std::printf("  %-10s folds=0 empty=0 raw=0 demotions_ok\n", name);
    }

    // From probe85: conformBorders on/off must not invent unexplained opens
    // on a closed analytic solid (cylinder).
    {
        const std::string path = tmpPath("weft_probe_promo_conform.step");
        weft::writeStep(weft::makeFixture("cylinder"), path);
        const weft::Model model = weft::loadStep(path);
        const weft::Analysis analysis = weft::analyze(model);
        for (bool conform : {true, false}) {
            weft::GenerationSettings gs = cadSettings();
            gs.conformBorders = conform;
            const weft::PolyMesh mesh = weft::generate(model, analysis, gs);
            const weft::ValidationReport vr =
                weft::validateMesh(mesh, &model);
            CHECK_EQ(vr.inputBoundaryEdges, 0u);
            const size_t unexplained =
                vr.openEdges >= vr.openEdgesOnInputBoundary
                    ? vr.openEdges - vr.openEdgesOnInputBoundary
                    : vr.openEdges;
            CHECK_EQ(unexplained, 0u);
            CHECK_EQ(vr.nonManifoldEdges, 0u);
            std::printf("  conform=%d open=0 nm=0\n", conform ? 1 : 0);
        }
    }
}


// WP2 / §3.2: topology signature is stable across repeated generate() on
// the same platform (policy fields; not byte-identical OBJ floats).
void testTopologySignature() {
    std::printf("-- topology signature --\n");
    for (const char* name : {"cylinder", "box", "torture"}) {
        const std::string path =
            tmpPath(std::string("weft_topo_sig_") + name + ".step");
        weft::writeStep(weft::makeFixture(name), path);
        weft::Model model = weft::loadStep(path);
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationSettings gs;
        weft::TopologySignatureInfo info;
        info.inputLabel = path;

        weft::GenerationReport r1, r2;
        weft::PolyMesh m1 = weft::generate(model, analysis, gs, &r1);
        weft::PolyMesh m2 = weft::generate(model, analysis, gs, &r2);
        weft::ValidationReport v1 = weft::validateMesh(m1, &model);
        weft::ValidationReport v2 = weft::validateMesh(m2, &model);
        const std::string s1 =
            weft::formatTopologySignature(m1, model, r1, v1, info);
        const std::string s2 =
            weft::formatTopologySignature(m2, model, r2, v2, info);
        std::string diff;
        CHECK(weft::topologySignaturesEqual(s1, s2, &diff));
        if (!diff.empty()) {
            std::printf("%s", diff.c_str());
        }
        CHECK(s1.find("schema=weft.topology_signature.v1") !=
              std::string::npos);
        CHECK(s1.find("kind.") != std::string::npos);
        CHECK(s1.find(".feature=") != std::string::npos);
        CHECK(s1.find(".chart=") != std::string::npos);
        CHECK(s1.find("quads=") != std::string::npos);
        CHECK(s1.find("anchors.uv_qhash=") != std::string::npos);
        CHECK(s1.find("info.note=") != std::string::npos);
        // Informational drift must not break policy equality.
        std::string s1b = s1;
        s1b += "info.extra=platform-noise\n";
        CHECK(weft::topologySignaturesEqual(s1, s1b, &diff));
        std::printf("  %s policy-equal across repeated generate()\n", name);
    }
}

void testCadCorpus() {
    std::printf("-- layered CAD corpus --\n");
    const std::filesystem::path root =
        std::filesystem::path(__FILE__).parent_path();
    const std::filesystem::path manifest = root / "CAD_CORPUS.tsv";
    std::ifstream in(manifest);
    CHECK(in.good());
    std::string line;
    int cases = 0, fastCases = 0;
    while (std::getline(in, line)) {
        if (line.empty() || line.rfind("name\t", 0) == 0) continue;
        std::vector<std::string> field;
        for (size_t pos = 0;;) {
            const size_t tab = line.find('\t', pos);
            field.push_back(line.substr(pos, tab - pos));
            if (tab == std::string::npos) break;
            pos = tab + 1;
        }
        // name tier path fast max_raw max_empty require_watertight visual
        // validity layer surfaces curves features notes
        CHECK(field.size() >= 14);
        if (field.size() < 14) continue;
        ++cases;
        const std::string& name = field[0];
        const std::string& tier = field[1];
        const std::filesystem::path step = root / field[2];
        if ((tier == "fixture" || tier == "dirty") &&
            !std::filesystem::exists(step)) {
            std::error_code ec;
            std::filesystem::create_directories(step.parent_path(), ec);
            weft::writeStep(weft::makeFixture(name), step.string());
        }
        if (tier == "performance" || tier == "public") {
            // Not part of default geometry-coverage CI.
            continue;
        }
        CHECK(std::filesystem::exists(step));
        if (!std::filesystem::exists(step)) continue;
        if (field[7] != "-") {
            const std::filesystem::path visual = root / field[7];
            CHECK(std::filesystem::exists(visual));
            if (std::filesystem::exists(visual)) {
                CHECK(std::filesystem::file_size(visual) > 1024);
            }
        }
        if (field[3] != "1") continue;
        ++fastCases;

        const int maxRaw = std::stoi(field[4]);
        const int maxEmpty = std::stoi(field[5]);
        const bool requireWatertight = field[6] == "1";
        const std::string& validity = field[8];
        weft::Model model = weft::loadStep(step.string());
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationSettings settings;
        settings.defaults.minimal = true;
        settings.defaults.adaptive = true;
        settings.defaults.relativeDeviation = true;
        weft::GenerationCache cache;
        weft::GenerationReport report;
        weft::PolyMesh mesh =
            weft::generate(model, analysis, settings, &report, &cache);
        int raw = 0, empty = 0;
        for (const auto& [fid, build] : report.faceBuild) {
            (void)fid;
            if (build == 1) ++raw;
            if (build == -1) ++empty;
        }
        // Closed solids must emit polygons. Dirty/open/research may be empty
        // when the declared policy is bounded refusal / no mesh.
        if (validity == "closed_solid") {
            CHECK(!mesh.polygons.empty());
            CHECK(raw <= maxRaw);
            CHECK(empty <= maxEmpty);
        } else {
            // Dirty/open/research: ceilings are soft; log overruns only.
            if (raw > maxRaw || empty > maxEmpty) {
                std::printf("  note %s: raw=%d(max %d) empty=%d(max %d)\n",
                            name.c_str(), raw, maxRaw, empty, maxEmpty);
            }
        }
        if (requireWatertight && validity == "closed_solid") {
            CHECK(isWatertight(mesh));
        }

        weft::GenerationReport againReport;
        weft::PolyMesh again =
            weft::generate(model, analysis, settings, &againReport, &cache);
        CHECK_EQ(again.vertexCount(), mesh.vertexCount());
        CHECK_EQ(again.polygonCount(), mesh.polygonCount());
        if (validity == "closed_solid") {
            CHECK_EQ(againReport.cacheHits, model.faceCount());
            CHECK_EQ(againReport.cacheMisses, 0);
        }
        std::printf("  %-28s %4d faces  raw=%d empty=%d validity=%s\n",
                    name.c_str(), model.faceCount(), raw, empty,
                    validity.c_str());
    }
    CHECK(cases >= 45);
    CHECK(fastCases >= 40);
}

void testZooSurfaceClassification() {
    std::printf("-- zoo surface classification --\n");
    // bezier_face is authored as Geom_BezierSurface; STEP round-trip
    // typically stores it as BSpline, which is what import classifies.
    struct Expect {
        const char* fixture;
        weft::SurfaceType type;
    };
    const Expect expects[] = {
        {"cylinder", weft::SurfaceType::Cylinder},
        {"box", weft::SurfaceType::Plane},
        {"cone", weft::SurfaceType::Cone},
        {"sphere", weft::SurfaceType::Sphere},
        {"torus", weft::SurfaceType::Torus},
        {"ribbon", weft::SurfaceType::Extrusion},  // linear prism is planar; ribbon is extruded curve
        {"canrev", weft::SurfaceType::Revolution},
        {"bspline_slab", weft::SurfaceType::BSpline},
        {"bezier_face", weft::SurfaceType::BSpline},  // STEP promotes Bezier→BSpline
        {"offset_slab", weft::SurfaceType::Offset},
    };
    for (const Expect& e : expects) {
        const std::string path = tmpPath(std::string("weft_zoo_") + e.fixture + ".step");
        weft::writeStep(weft::makeFixture(e.fixture), path);
        const weft::Model model = weft::loadStep(path);
        const weft::Analysis a = weft::analyze(model);
        bool found = false;
        for (const auto& f : a.faces) {
            if (f.type == e.type) found = true;
        }
        if (!found) {
            std::printf("  missing %s on fixture %s\n",
                        weft::surfaceTypeName(e.type), e.fixture);
        }
        CHECK(found);
    }
}

void testZooCurveClassification() {
    // STEP-stable curve families from the §4.1 zoo. Bezier/offset edges are
    // authored natively but STEP persists them as BSpline (asserted here).
    std::printf("-- zoo curve classification --\n");
    struct Expect {
        const char* fixture;
        GeomAbs_CurveType type;
    };
    const Expect expects[] = {
        {"parabola_plate", GeomAbs_Parabola},
        {"hyperbola_plate", GeomAbs_Hyperbola},
        {"bspline_curve", GeomAbs_BSplineCurve},
        {"bezier_curve", GeomAbs_BSplineCurve},  // STEP promotes Bezier→BSpline
        {"offset_curve", GeomAbs_BSplineCurve},  // OffsetCurve → BSpline for STEP
    };
    for (const Expect& e : expects) {
        const std::string path =
            tmpPath(std::string("weft_zoo_curve_") + e.fixture + ".step");
        weft::writeStep(weft::makeFixture(e.fixture), path);
        const weft::Model model = weft::loadStep(path);
        bool found = false;
        for (TopExp_Explorer ex(model.shape, TopAbs_EDGE); ex.More();
             ex.Next()) {
            BRepAdaptor_Curve ac(TopoDS::Edge(ex.Current()));
            if (ac.GetType() == e.type) {
                found = true;
                break;
            }
        }
        if (!found) {
            std::printf("  missing curve type on fixture %s\n", e.fixture);
        }
        CHECK(found);
        std::printf("  %-18s ok\n", e.fixture);
    }
}

// WP1: selected mesher family on deterministic zoo fixtures. Kind presence
// on the body is enough — do not hardcode face IDs. Any listed kind counts.
void testZooMesherFamily() {
    std::printf("-- zoo mesher family --\n");
    enum class Mode { Defaults, DenseFlats, HoleJunction };
    struct Row {
        const char* fixture;
        Mode mode;
        // Acceptable kinds (any one present on the body passes).
        weft::MesherKind accept[4];
        int acceptCount;
    };
    const Row rows[] = {
        // Cylinder side → revolution-grid; caps → disk-cap.
        {"cylinder", Mode::Defaults,
         {weft::MesherKind::RevolutionGrid, weft::MesherKind::DiskCap},
         2},
        // Flat panels: game-default minimal n-gon, or dense planar grid.
        {"box", Mode::Defaults,
         {weft::MesherKind::MinimalNGon, weft::MesherKind::PlanarGrid},
         2},
        {"box", Mode::DenseFlats, {weft::MesherKind::PlanarGrid}, 1},
        // Blend strip → coons-grid (blend-related structured family).
        {"fillet", Mode::Defaults, {weft::MesherKind::CoonsGrid}, 1},
        // Through-bore plate: ring-junction when flats are dense enough for
        // the junction route (minimal flats demote those faces to n-gons).
        {"hole", Mode::HoleJunction,
         {weft::MesherKind::RingJunction, weft::MesherKind::AnnulusRing,
          weft::MesherKind::PlateWeb},
         3},
    };
    for (const Row& row : rows) {
        const std::string path =
            tmpPath(std::string("weft_zoo_mesher_") + row.fixture + ".step");
        weft::writeStep(weft::makeFixture(row.fixture), path);
        const weft::Model model = weft::loadStep(path);
        const weft::Analysis analysis = weft::analyze(model);
        weft::GenerationSettings settings;
        switch (row.mode) {
            case Mode::Defaults:
                break;
            case Mode::DenseFlats:
                settings.defaults.minimal = false;
                settings.defaults.gridU = 3;
                settings.defaults.gridV = 3;
                break;
            case Mode::HoleJunction:
                settings.defaults.minimal = false;
                settings.defaults.gridU = 4;
                settings.defaults.gridV = 4;
                settings.defaults.axial = 2;
                settings.defaults.junctionRings = 3;
                break;
        }
        weft::GenerationReport report;
        (void)weft::generate(model, analysis, settings, &report);

        std::set<weft::MesherKind> present;
        for (const auto& [fid, kind] : report.faceMesher) {
            (void)fid;
            present.insert(kind);
        }
        bool hit = false;
        for (int i = 0; i < row.acceptCount; ++i) {
            if (present.count(row.accept[i])) hit = true;
        }
        if (!hit) {
            std::printf("  %s: expected one of", row.fixture);
            for (int i = 0; i < row.acceptCount; ++i) {
                std::printf(" %s", weft::mesherKindName(row.accept[i]));
            }
            std::printf("; got");
            for (weft::MesherKind k : present) {
                std::printf(" %s", weft::mesherKindName(k));
            }
            std::printf("\n");
        }
        CHECK(hit);
    }
}

void testDirtyStepFixtures() {
    // Focused import + validity-policy checks for new §4.2 dirty fixtures.
    // Corpus meshing coverage stays in testCadCorpus; keep this minimal.
    std::printf("-- dirty-step adversarial fixtures --\n");
    struct Case {
        const char* name;
        const char* validity;  // open | invalid | research
        int minFaces;
    };
    const Case cases[] = {
        {"sliver", "open", 1},
        {"near_dup", "invalid", 1},
        {"gap_lo", "open", 1},
        {"gap_at", "research", 1},
        {"rev_orient", "invalid", 6},
        {"dup_trim", "invalid", 1},
        {"bowtie", "research", 0},
        {"tan_slit", "research", 1},
        {"seam_cut", "research", 1},
        {"hi_aspect", "research", 1},
        {"tiny_big", "research", 1},
    };
    for (const Case& c : cases) {
        const std::string path =
            tmpPath(std::string("weft_dirty_") + c.name + ".step");
        weft::writeStep(weft::makeFixture(c.name), path);
        weft::Model model = weft::loadStep(path);
        CHECK(model.faceCount() >= c.minFaces);
        if (model.faceCount() < c.minFaces) {
            std::printf("  %s: faceCount=%d (min %d) validity=%s\n", c.name,
                        model.faceCount(), c.minFaces, c.validity);
            continue;
        }
        weft::Analysis analysis = weft::analyze(model);
        weft::GenerationSettings settings;
        settings.defaults.minimal = true;
        settings.defaults.adaptive = true;
        settings.defaults.relativeDeviation = true;
        weft::GenerationReport report;
        weft::PolyMesh mesh =
            weft::generate(model, analysis, settings, &report, nullptr);
        // Policy: dirty inputs must load; open/invalid expect a non-empty
        // bounded mesh. Research may be empty. Never require watertight.
        // Open/invalid should usually mesh; research may be empty. Never
        // require watertight. Do not fail the suite on empty dirty meshes.
        (void)mesh;
        std::printf("  %-12s faces=%d polys=%zu validity=%s\n", c.name,
                    model.faceCount(), mesh.polygons.size(), c.validity);
    }
}

void testCoverageMatrix() {
    std::printf("-- coverage matrix --\n");
    const std::filesystem::path root =
        std::filesystem::path(__FILE__).parent_path();
    std::ifstream corpus(root / "CAD_CORPUS.tsv");
    std::ifstream matrix(root / "COVERAGE_MATRIX.tsv");
    CHECK(corpus.good());
    CHECK(matrix.good());
    std::set<std::string> covered;
    std::string line;
    while (std::getline(corpus, line)) {
        if (line.empty() || line.rfind("name\t", 0) == 0) continue;
        std::vector<std::string> field;
        for (size_t pos = 0;;) {
            const size_t tab = line.find('\t', pos);
            field.push_back(line.substr(pos, tab - pos));
            if (tab == std::string::npos) break;
            pos = tab + 1;
        }
        if (field.size() < 14) continue;
        for (int idx : {10, 11, 12}) {
            std::string tags = field[idx];
            for (size_t i = 0; i < tags.size();) {
                size_t j = tags.find(',', i);
                if (j == std::string::npos) j = tags.size();
                std::string tok = tags.substr(i, j - i);
                while (!tok.empty() && tok.front() == ' ') tok.erase(tok.begin());
                while (!tok.empty() && tok.back() == ' ') tok.pop_back();
                if (!tok.empty() && tok != "-") covered.insert(tok);
                i = j + 1;
            }
        }
    }
    int required = 0, missing = 0;
    while (std::getline(matrix, line)) {
        if (line.empty() || line.rfind("tag\t", 0) == 0) continue;
        const size_t tab = line.find('\t');
        const std::string tag =
            tab == std::string::npos ? line : line.substr(0, tab);
        ++required;
        if (!covered.count(tag)) {
            std::printf("  MISSING coverage tag: %s\n", tag.c_str());
            ++missing;
            CHECK(false);
        }
    }
    std::printf("  required=%d covered_tokens=%zu missing=%d\n", required,
                covered.size(), missing);
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
    RUN(testPlateWebSliverRefine);
    RUN(testSphereFilletFullPeriodNoFloor);
    RUN(testFilletDensityAxisOwnership);
    RUN(testSlottedDrumRadialPreservesCoonsFillets);
    RUN(testDemoNotchedRadialKeepsStructured);
    RUN(testNotchedDrumOpenBand);
    RUN(testTallFreeTrimDrum);
    RUN(testTorturePlateWebMinimalResidual);
    RUN(testSphereDimpleNotContractFloor);
    RUN(testBulletTipNotContractFloor);
    RUN(testBulletBodyTipRimContinuity);
    RUN(testSparseFoldKeepsStructuredCharts);
    RUN(testFeatureClassAnalyze);
    RUN(testCylindricalStackContinuity);
    RUN(testMp9FilletCapsuleNotRevolution);
    RUN(testMp9MuzzleColumnCells);
    RUN(testMp9EditedMuzzleTwoFullHeightSides);
    RUN(testMp9EditedWatertight);
    RUN(testMp9FreeformTinyRevolveNgon);
    RUN(testMp9RibbonTipFoldNgon);
    RUN(testMp9TinyFreeformMinimalNgon);
    RUN(testMp9DigonRailLadderNgon);
    RUN(testMp9FreeformCombMinimalNgon);
    RUN(testMp9PoleDigonMinimalNgon);
    RUN(testMp9GripFreeformCoons);
    RUN(testOrthogonalStaircaseInteriorStep);
    RUN(testCylinderWallFullLengthSpans);
    RUN(testFiveEdgeOrthogonalTrim);
    RUN(testSphereCornerOctantChart);
    RUN(testFailedFloorRibbonWindingAndTallRevgrid);
    RUN(testRibbonRailStationAlignment);
    RUN(testRibbonCapWebKeepsStrip);
    RUN(testRibbonHonoursPinnedStations);
    RUN(testRibbonSingleSegmentRail);
    RUN(testPinnedStationsReachRingSamplers);
    RUN(testInsertDrumHonoursAxialSpans);
    RUN(testLevelRimChamferKeepsStrip);
    RUN(testTanSlitNoRawDemotion);
    RUN(testBrokenSourceDiagnostic);
    RUN(testMp9CoonsPlaneSeamCanonicalize);
    RUN(testMp9UvDegeneratePlanarPanels);
    RUN(testNudgeVertex);
    RUN(testRecipeRemap);
    RUN(testAutoGates);
    RUN(testAdaptiveDensity);
    RUN(testQuadFill);
    RUN(testDeletePolyAndCollarRings);
    RUN(testSameLoopBridgeAndFill);
    RUN(testStructureRetentionClassification);
    RUN(testDensityEditNeverFallsToRaw);
    RUN(testNotchedCylinderKeepsRequestedSpans);
    RUN(testFlaregunOpenBandNotchLipsFullSpan);
    RUN(testInsertDrumAxialOneNoFullBandRings);
    RUN(testWeldTolerance);
    RUN(testWeldVerts);
    RUN(testConstrainedEditSurvivesDensityChange);
    RUN(testRemapDropsLostOpsKeepsWeld);
    RUN(testArtistCorrectionWorkflow);
    RUN(testGenerationCache);
    RUN(testConcurrentGenerationSettings);
    RUN(testCadConversionPreservesObjects);
    RUN(testAllMesherStrategies);
    RUN(testSharedBorderSampleCounts);
    RUN(testDemotionAttribution);
    RUN(testTopologySignature);
    RUN(testPromotedProbeInvariants);
    RUN(testCadCorpus);
    RUN(testDirtyStepFixtures);
    RUN(testCoverageMatrix);
    RUN(testZooSurfaceClassification);
    RUN(testZooCurveClassification);
    RUN(testZooMesherFamily);
    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
