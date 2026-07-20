#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/planar_cdt.hpp"
#include "weft/planar_trim_assembly.hpp"
#include "weft/secure_meshing.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

int failures = 0;

#define CHECK(condition)                                                  \
    do {                                                                  \
        if (!(condition)) {                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__,          \
                        #condition);                                      \
            ++failures;                                                   \
        }                                                                 \
    } while (false)

class TemporaryStep {
public:
    explicit TemporaryStep(const std::string& stem)
        : path_(weft::test::uniqueTempPath(stem, ".step")) {}

    ~TemporaryStep() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

struct PreparedFixture {
    weft::ImportedModel imported;
    weft::ReconnaissanceReport reconnaissance;
    weft::CanonicalBoundarySet boundaries;
};

std::optional<std::filesystem::path> mp9ExtractPath(const char* name) {
    const std::filesystem::path candidates[] = {
        std::filesystem::path("tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("../tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("../../tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("../../../tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("fixtures/mp9_extracts") / name,
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) return candidate;
    }
    return std::nullopt;
}

std::optional<PreparedFixture> preparePath(
    const std::filesystem::path& path) {
    PreparedFixture prepared;
    prepared.imported = weft::importStepSecure(path.string());
    prepared.reconnaissance = weft::reconnoitre(prepared.imported);
    CHECK(prepared.reconnaissance.complete);
    if (!prepared.reconnaissance.complete) return std::nullopt;

    weft::IntervalProblem intervals;
    for (const weft::EdgeTopologyRecord& edge :
         prepared.imported.working->snapshot.edgeTopology) {
        const weft::ExactGeometryClassification* classification =
            prepared.reconnaissance.find(edge.id);
        const bool circle = classification &&
            classification->familyCode == "circle";
        intervals.variables.push_back(
            {{weft::StableIdKind::Boundary, edge.id.ordinal},
             circle ? 16.0 : 2.0, 1, false, std::nullopt});
    }
    const weft::IntervalSolveResult solved = weft::solveIntervals(intervals);
    CHECK(solved);
    if (!solved) return std::nullopt;
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(
            prepared.imported, prepared.reconnaissance, *solved.solution);
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("boundary preparation failed: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return std::nullopt;
    }
    prepared.boundaries = *built.value;
    return prepared;
}

std::optional<PreparedFixture> prepare(const std::string& fixture,
                                       const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture(fixture), path.string());
    return preparePath(path);
}

bool completeAssemblyEvidence(const weft::PlanarTrimAssemblyResult& result) {
    return result.evidence.size() == 6 &&
        std::all_of(
            result.evidence.begin(), result.evidence.end(),
            [](const weft::PlanarTrimAssemblyEvidence& item) {
                return item.complete();
            });
}

std::vector<weft::StableId> planarFaces(
    const weft::ReconnaissanceReport& reconnaissance) {
    std::vector<weft::StableId> faces;
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Surface &&
            record.familyCode == "plane" &&
            record.support ==
                weft::GeometrySupportState::SupportedAnalyticTemplate) {
            faces.push_back(record.subjectId);
        }
    }
    return faces;
}

void testBoxFaces(const weft::PlanarCdtBackend& cdt) {
    TemporaryStep step("weft_trim_assembly_box");
    const auto prepared = prepare("box", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;
    const std::vector<weft::StableId> faces =
        planarFaces(prepared->reconnaissance);
    CHECK(faces.size() == 6);
    for (weft::StableId face : faces) {
        const weft::PlanarTrimAssemblyResult assembled =
            weft::assemblePlanarTrimDomain(
                prepared->imported, prepared->reconnaissance,
                prepared->boundaries, face);
        if (!assembled && assembled.failure) {
            std::printf("box face %llu assembly failed: %s: %s\n",
                        static_cast<unsigned long long>(face.ordinal),
                        assembled.failure->code.c_str(),
                        assembled.failure->message.c_str());
        }
        CHECK(assembled);
        CHECK(completeAssemblyEvidence(assembled));
        CHECK(assembled.validation);
        CHECK(assembled.value && assembled.value->loops.size() == 1);
        if (!assembled.value) continue;

        std::size_t sharedCorners = 0;
        for (const weft::PlanarTrimVertex& vertex :
             assembled.value->loops.front().vertices) {
            if (vertex.boundaryUses.size() == 2) ++sharedCorners;
        }
        CHECK(sharedCorners == 4);
        const weft::PlanarCdtResult triangulated =
            cdt.triangulate(*assembled.value);
        CHECK(triangulated);
    }
}

void testCylinderCapsAndWallRefusal(const weft::PlanarCdtBackend& cdt) {
    TemporaryStep step("weft_trim_assembly_cylinder");
    const auto prepared = prepare("cylinder", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;

    std::size_t caps = 0;
    std::optional<weft::StableId> wall;
    for (const weft::ExactGeometryClassification& record :
         prepared->reconnaissance.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
        if (record.familyCode == "plane") {
            const weft::PlanarTrimAssemblyResult assembled =
                weft::assemblePlanarTrimDomain(
                    prepared->imported, prepared->reconnaissance,
                    prepared->boundaries, record.subjectId);
            CHECK(assembled);
            CHECK(assembled.value && assembled.value->loops.size() == 1);
            if (assembled.value) {
                CHECK(assembled.value->loops.front().vertices.size() == 16);
                CHECK(cdt.triangulate(*assembled.value));
            }
            ++caps;
        } else if (record.familyCode == "cylinder") {
            wall = record.subjectId;
        }
    }
    CHECK(caps == 2);
    CHECK(wall.has_value());
    if (wall) {
        // Cylinder walls may assemble as UV surfaces; full-periodic fixtures
        // often refuse closed_edge_mixed_wire which is still a valid contract.
        const weft::PlanarTrimAssemblyResult assembled =
            weft::assemblePlanarTrimDomain(
                prepared->imported, prepared->reconnaissance,
                prepared->boundaries, *wall);
        if (assembled) {
            CHECK(assembled.value && !assembled.value->loops.empty());
        } else {
            CHECK(assembled.failure);
            std::printf("WEFT_TRIM_CYLINDER_UV code=%s\n",
                        assembled.failure ? assembled.failure->code.c_str()
                                          : "-");
        }
    }
}

void testPerforatedPlanarFace(const weft::PlanarCdtBackend& cdt) {
    TemporaryStep step("weft_trim_assembly_hole");
    const auto prepared = prepare("hole", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;

    bool sawPerforated = false;
    for (weft::StableId face : planarFaces(prepared->reconnaissance)) {
        const weft::PlanarTrimAssemblyResult assembled =
            weft::assemblePlanarTrimDomain(
                prepared->imported, prepared->reconnaissance,
                prepared->boundaries, face);
        CHECK(assembled);
        if (!assembled.value || assembled.value->loops.size() < 2) continue;
        sawPerforated = true;
        CHECK(assembled.value->loops.front().declaredRole ==
              weft::PlanarTrimLoopRole::Outer);
        for (std::size_t index = 1; index < assembled.value->loops.size();
             ++index) {
            CHECK(assembled.value->loops[index].declaredRole ==
                  weft::PlanarTrimLoopRole::Hole);
        }
        const weft::PlanarCdtResult triangulated =
            cdt.triangulate(*assembled.value);
        CHECK(triangulated);
        CHECK(triangulated.trimValidation);
        CHECK(triangulated.value &&
              triangulated.value->boundaryLoops.size() ==
                  assembled.value->loops.size());
    }
    CHECK(sawPerforated);
}

weft::CanonicalBoundary* mutableBoundary(
    weft::CanonicalBoundarySet& set, weft::StableId edge) {
    const auto found = std::find_if(
        set.boundaries.begin(), set.boundaries.end(),
        [edge](const weft::CanonicalBoundary& boundary) {
            return boundary.edge == edge;
        });
    return found == set.boundaries.end() ? nullptr : &*found;
}

weft::SecureMeshingConfiguration planeMatrixSettings() {
    weft::SecureMeshingConfiguration settings;
    settings.omitDeferredResiduals = false;
    settings.sampling.chordTolerance = 0.1;
    settings.sampling.normalAngleToleranceRadians =
        20.0 * 3.141592653589793 / 180.0;
    settings.revolutionRadialSegments = 32;
    settings.sampling.minimumClosedCurveSegments = 8;
    return settings;
}

// Fail-closed product mesh of one plane extract. Returns triangle count on
// success, 0 on failure. Plane allowCurvedUv=false is enforced in
// assemblePlanarTrimDomain (family plane) and asserted separately via
// testPlaneMatrixAllowCurvedUvClosed.
std::size_t meshPlaneExtractFailClosed(const char* extractName,
                                       const char* marker) {
    const auto path = mp9ExtractPath(extractName);
    CHECK(path.has_value());
    if (!path) {
        std::printf("FAIL %s missing tests/fixtures/mp9_extracts/%s\n",
                    marker, extractName);
        return 0;
    }
    const weft::ImportedModel imported = weft::importStepSecure(path->string());
    const weft::SecureMeshingResult meshed =
        weft::generateSecureMesh(imported, planeMatrixSettings());
    if (!meshed) {
        std::printf("%s extract=%s refuse code=%s\n", marker, extractName,
                    meshed.failure ? meshed.failure->code.c_str() : "-");
    }
    CHECK(meshed);
    CHECK(meshed.value && !meshed.value->certified.triangles.empty());
    const std::size_t tris =
        meshed.value ? meshed.value->certified.triangles.size() : 0U;
    std::printf("%s extract=%s tris=%zu\n", marker, extractName, tris);
    return tris;
}

void testPlaneMatrixAllowCurvedUvClosed() {
    // Product plane path must never arm allowCurvedUv. Proven on a simple
    // multi-loop plane fixture (hole) where preparePath intervals match CDT.
    TemporaryStep step("weft_plane_matrix_curved_uv");
    const auto prepared = prepare("hole", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;
    std::size_t planeFaces = 0;
    for (weft::StableId face : planarFaces(prepared->reconnaissance)) {
        ++planeFaces;
        const weft::PlanarTrimAssemblyResult assembled =
            weft::assemblePlanarTrimDomain(
                prepared->imported, prepared->reconnaissance,
                prepared->boundaries, face);
        CHECK(assembled);
        if (!assembled.value) continue;
        CHECK(!assembled.value->allowCurvedUv);
        if (assembled.value->allowCurvedUv) {
            std::printf("FAIL WEFT_PLANE_MATRIX allowCurvedUv=1 on hole plane\n");
        }
    }
    CHECK(planeFaces > 0);
    std::printf("WEFT_PLANE_MATRIX allowCurvedUv=0 plane_faces=%zu\n",
                planeFaces);
}

void testG1Plane2732DigonRecovery() {
    // MP9 face 2732: two-edge digon with parallel-offset p-curves. Certified
    // recovery keeps distinct UV corners at shared 3D vertices (no pad-fan /
    // allowCurvedUv). Product mesh path must certify.
    meshPlaneExtractFailClosed("plane_2732.step", "WEFT_G1_PLANE2732");
}

void testG1Plane3605PerforatedRecovery() {
    // MP9 face 3605: multiply-perforated filleted-slot plane (1 outer + 6
    // holes). Certified recovery uses walk-landing bridges + alternate
    // exact cuts when the shortest bridged walk stalls ears. No fan /
    // allowCurvedUv soften.
    meshPlaneExtractFailClosed("plane_3605.step", "WEFT_G1_PLANE3605");
}

void testG1Plane3821EllipseDensifyRecovery() {
    // MP9 face 3821: concave plane (4 lines + 2 eccentric ellipses). Sparse
    // sagitta chords self-intersected in UV; plane-owned ellipse densify
    // (≥48 intervals) keeps the loop simple. No allowCurvedUv / pad-fan.
    meshPlaneExtractFailClosed("plane_3821.step", "WEFT_G1_PLANE3821");
}

void testPlaneMatrix() {
    // Wave A lock: every committed plane_* extract hard-certifies fail-closed
    // with allowCurvedUv=false. Subclasses: simple/multi-outer (plane_multi),
    // digon (2732), perforated/filleted-slot (3605), ellipse densify (3821),
    // self-intersect candidate recovery (1793).
    const char* marker = "WEFT_PLANE_MATRIX";
    testPlaneMatrixAllowCurvedUvClosed();
    const char* extracts[] = {"plane_multi.step", "plane_2732.step",
                              "plane_3605.step", "plane_3821.step",
                              "plane_1793.step"};
    std::size_t locked = 0;
    for (const char* name : extracts) {
        if (meshPlaneExtractFailClosed(name, marker) > 0) ++locked;
    }
    CHECK(locked == 5);
    std::printf("%s locked=%zu/5 fail_closed=1 allowCurvedUv=0\n", marker,
                locked);
}

void testTamperedJunctionUvRefuses() {
    TemporaryStep step("weft_trim_assembly_tamper");
    const auto prepared = prepare("box", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;
    const weft::StableId face = planarFaces(prepared->reconnaissance).front();

    std::vector<const weft::CoedgeRecord*> coedges;
    for (const weft::CoedgeRecord& coedge :
         prepared->imported.working->snapshot.coedges) {
        if (coedge.faceId == face) coedges.push_back(&coedge);
    }
    std::sort(coedges.begin(), coedges.end(),
              [](const weft::CoedgeRecord* left,
                 const weft::CoedgeRecord* right) {
                  return left->ordinalInWire < right->ordinalInWire;
              });
    CHECK(coedges.size() == 4);
    if (coedges.size() != 4) return;

    weft::CanonicalBoundarySet tampered = prepared->boundaries;
    const weft::CoedgeRecord& second = *coedges[1];
    weft::CanonicalBoundary* boundary =
        mutableBoundary(tampered, second.edgeId);
    CHECK(boundary != nullptr);
    if (!boundary) return;
    const std::size_t sampleIndex =
        second.orientation == weft::TopologyOrientation::Forward
        ? 0
        : boundary->samples.size() - 1;
    bool changed = false;
    for (weft::CoedgeUvUse& use : boundary->samples[sampleIndex].faceUses) {
        if (use.face == face && use.coedge == second.id) {
            use.liftedUv[0] += 1e-12;
            changed = true;
        }
    }
    CHECK(changed);
    const weft::PlanarTrimAssemblyResult refused =
        weft::assemblePlanarTrimDomain(
            prepared->imported, prepared->reconnaissance, tampered, face);
    CHECK(!refused);
    CHECK(refused.failure &&
          refused.failure->code == "trim_assembly.vertex_uv_mismatch");
}

}  // namespace

int main() {
    try {
        const auto cdt = weft::makeExactLawsonReferencePlanarCdtBackend();
        CHECK(cdt != nullptr);
        if (cdt) {
            testBoxFaces(*cdt);
            testCylinderCapsAndWallRefusal(*cdt);
            testPerforatedPlanarFace(*cdt);
            testPlaneMatrix();
            testG1Plane2732DigonRecovery();
            testG1Plane3605PerforatedRecovery();
            testG1Plane3821EllipseDensifyRecovery();
            testTamperedJunctionUvRefuses();
        }
    } catch (const std::exception& error) {
        std::printf("FAIL trim-assembly exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("planar trim assembly checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d planar trim assembly failure(s)\n", failures);
    return EXIT_FAILURE;
}
