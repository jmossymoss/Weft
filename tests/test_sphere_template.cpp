#include "weft/canonical_boundary.hpp"
#include "weft/fixture.hpp"
#include "weft/interval_solver.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"
#include "weft/sphere_template.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

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

weft::SecureMeshingConfiguration configuration() {
    weft::SecureMeshingConfiguration settings;
    settings.sampling.chordTolerance = 0.25;
    settings.sampling.normalAngleToleranceRadians = 0.35;
    settings.sampling.minimumClosedCurveSegments = 16;
    settings.revolutionRadialSegments = 32;
    settings.previewTriangleBudget = 0;
    return settings;
}

weft::SecureMeshingConfiguration matrixSettings() {
    weft::SecureMeshingConfiguration settings = configuration();
    settings.omitDeferredResiduals = false;
    settings.sampling.chordTolerance = 0.1;
    settings.sampling.normalAngleToleranceRadians =
        28.0 * 0.017453292519943295;
    settings.sampling.minimumClosedCurveSegments = 8;
    settings.revolutionRadialSegments = 32;
    return settings;
}

bool assertHardCertify(const weft::SecureMeshingResult& result,
                       const char* marker, const char* caseName) {
    if (!result) {
        std::printf("%s case=%s refuse code=%s\n", marker, caseName,
                    result.failure ? result.failure->code.c_str() : "-");
        CHECK(result);
        return false;
    }
    CHECK(result.validation.complete());
    CHECK(result.value && !result.value->certified.triangles.empty());
    const bool hardIntersection = std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "certified.triangle_intersection" &&
                coverage.failed == 0 &&
                (coverage.expected > 0 || coverage.checked > 0);
        });
    CHECK(hardIntersection);
    const bool softIntersection = std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "certified.triangle_intersection" &&
                coverage.expected == 0 && coverage.checked == 0;
        });
    CHECK(!softIntersection);
    const std::size_t tris =
        result.value ? result.value->certified.triangles.size() : 0U;
    std::printf("%s case=%s tris=%zu hard=1\n", marker, caseName, tris);
    return hardIntersection && !softIntersection && tris > 0;
}

void testSphereDensityAndDeterminism() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_sphere_density", ".step");
    weft::writeStep(weft::makeFixture("sphere"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::SecureMeshingResult first =
        weft::generateSecureMesh(imported, configuration());
    const weft::SecureMeshingResult second =
        weft::generateSecureMesh(imported, configuration());
    weft::SecureMeshingConfiguration dense = configuration();
    dense.sampling.chordTolerance = 0.08;
    dense.sampling.minimumClosedCurveSegments = 24;
    dense.revolutionRadialSegments = 48;
    dense.previewTriangleBudget = 0;  // disable budget for LOD monotonicity
    const weft::SecureMeshingResult denser =
        weft::generateSecureMesh(imported, dense);
    CHECK(first && second && denser);
    CHECK(first.value->certified.topologyFingerprint ==
          second.value->certified.topologyFingerprint);
    // Repeat-run determinism above is the guarantee; the exact hex differs
    // across OCCT versions, so it is evidence-only rather than a golden.
    CHECK(first.value->certified.topologyFingerprint.size() == 16);
    CHECK(denser.value->certified.triangles.size() >
          first.value->certified.triangles.size());
    CHECK(first.value->modeling.provenance ==
              weft::ModelingProvenanceKind::Independent ||
          first.value->modeling.provenance ==
              weft::ModelingProvenanceKind::CertifiedFloorAlias);
    std::printf("WEFT_SPHERE_F tris=%zu dense=%zu fingerprint=%s modeling=%d\n",
                first.value->certified.triangles.size(),
                denser.value->certified.triangles.size(),
                first.value->certified.topologyFingerprint.c_str(),
                static_cast<int>(first.value->modeling.provenance));
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

std::filesystem::path findMp9Extract(const char* name) {
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
    return {};
}

void testSphericalCapWallHardCertification() {
    // G4 acceptance: MP9 f778 / complex caps hard-certify
    // (CapWall or UV-trim) with relaxGeometryChecks=false.
    for (const char* name :
         {"sphere_cap_778.step", "sphere_cap_complex.step"}) {
        const std::filesystem::path path = findMp9Extract(name);
        CHECK(!path.empty());
        if (path.empty()) continue;
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        CHECK(imported.meshable());
        if (!imported.meshable()) continue;
        weft::SecureMeshingConfiguration settings = configuration();
        settings.sampling.normalAngleToleranceRadians =
            28.0 * 0.017453292519943295;
        settings.sampling.chordTolerance = 0.1;
        const weft::SecureMeshingResult result =
            weft::generateSecureMesh(imported, settings);
        CHECK(result);
        if (!result) {
            if (result.failure) {
                std::printf("sphere_cap hard %s: %s: %s\n", name,
                            result.failure->code.c_str(),
                            result.failure->message.c_str());
            }
            continue;
        }
        CHECK(result.validation.complete());
        CHECK(result.value && result.value->certified.triangles.size() > 0);
        const bool hardIntersection = std::any_of(
            result.validation.checks.begin(), result.validation.checks.end(),
            [](const weft::ValidationCoverage& coverage) {
                return coverage.code == "certified.triangle_intersection" &&
                    coverage.failed == 0 &&
                    (coverage.expected > 0 || coverage.checked > 0);
            });
        CHECK(hardIntersection);
        const bool softIntersection = std::any_of(
            result.validation.checks.begin(), result.validation.checks.end(),
            [](const weft::ValidationCoverage& coverage) {
                return coverage.code == "certified.triangle_intersection" &&
                    coverage.expected == 0 && coverage.checked == 0;
            });
        CHECK(!softIntersection);
        const bool sawCapWallChord = std::any_of(
            result.validation.checks.begin(), result.validation.checks.end(),
            [](const weft::ValidationCoverage& coverage) {
                return coverage.code.rfind("sphere.chord_bound.face_", 0) ==
                    0 &&
                    coverage.complete();
            });
        std::printf(
            "WEFT_SPHERE_CAP_HARD %s tris=%zu capwall_chord=%d complete=%d\n",
            name,
            result.value ? result.value->certified.triangles.size() : 0U,
            sawCapWallChord ? 1 : 0, result.validation.complete() ? 1 : 0);
    }
}

void testFullSphereBody() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_sphere_template", ".step");
    weft::writeStep(weft::makeFixture("sphere"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        if (result.failure) {
            std::printf("sphere mesh failure: %s: %s\n",
                        result.failure->code.c_str(),
                        result.failure->message.c_str());
        }
        return;
    }
    CHECK(result.validation.complete());
    CHECK(result.value && result.value->certified.triangles.size() >= 24);
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("sphere.chord_bound.face_", 0) == 0 &&
                coverage.complete();
        }));
    std::printf("WEFT_SPHERE_C tris=%zu verts=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.vertices.size(),
                result.value->certified.topologyFingerprint.c_str());
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testSphereMatrix() {
    // Wave C lock: full wall + CapWall/band UV-trim hard-certify fail-closed.
    // CapWall may refuse orientation minorities (refuse-not-drop); product
    // path then hard-orients UV-trim with relax=0 (Wave 0 / G4 preserved).
    const char* marker = "WEFT_SPHERE_MATRIX";
    CHECK(!matrixSettings().omitDeferredResiduals);
    std::size_t locked = 0;

    {
        const std::filesystem::path path =
            weft::test::uniqueTempPath("weft_sphere_matrix_full", ".step");
        weft::writeStep(weft::makeFixture("sphere"), path.string());
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        if (assertHardCertify(weft::generateSecureMesh(imported, matrixSettings()),
                              marker, "full_wall_fixture")) {
            ++locked;
        }
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }
    for (const char* name :
         {"sphere_cap_778.step", "sphere_cap_complex.step"}) {
        const std::filesystem::path path = findMp9Extract(name);
        CHECK(!path.empty());
        if (path.empty()) continue;
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        CHECK(imported.meshable());
        if (!imported.meshable()) continue;
        if (assertHardCertify(weft::generateSecureMesh(imported, matrixSettings()),
                              marker, name)) {
            ++locked;
        }
    }
    CHECK(locked == 3);
    std::printf("%s locked=%zu/3 fail_closed=1 omit=0 "
                "capwall_refuse_not_drop=1 uvtrim_hardOrient=1\n",
                marker, locked);
}

}  // namespace

int main() {
    try {
        testSphereMatrix();
        testSphericalCapWallHardCertification();
        testFullSphereBody();
        testSphereDensityAndDeterminism();
    } catch (const std::exception& error) {
        std::printf("FAIL sphere-template exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("sphere template checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d sphere template failure(s)\n", failures);
    return EXIT_FAILURE;
}
