#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"
#include "weft/torus_template.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>

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
    settings.sampling.chordTolerance = 0.5;
    settings.sampling.normalAngleToleranceRadians = 0.9;
    settings.sampling.minimumClosedCurveSegments = 20;
    settings.revolutionRadialSegments = 32;
    settings.previewTriangleBudget = 0;
    return settings;
}

weft::SecureMeshingConfiguration matrixSettings() {
    weft::SecureMeshingConfiguration settings;
    settings.omitDeferredResiduals = false;
    settings.sampling.chordTolerance = 0.1;
    settings.sampling.normalAngleToleranceRadians =
        28.0 * 0.017453292519943295;
    settings.sampling.minimumClosedCurveSegments = 8;
    settings.revolutionRadialSegments = 32;
    settings.previewTriangleBudget = 45000;
    return settings;
}

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

void testTorusPreviewLodChordValid() {
    // Default CLI preview LOD: chord 0.1, angle 28°, radial 32, budget 45k.
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_torus_preview_lod", ".step");
    weft::writeStep(weft::makeFixture("torus"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    weft::SecureMeshingConfiguration settings;
    settings.sampling.chordTolerance = 0.1;
    settings.sampling.normalAngleToleranceRadians =
        28.0 * 0.017453292519943295;
    settings.sampling.minimumClosedCurveSegments = 8;
    settings.revolutionRadialSegments = 32;
    settings.previewTriangleBudget = 45000;
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, settings);
    CHECK(result);
    CHECK(result.validation.complete());
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("torus.chord_bound.face_", 0) == 0 &&
                coverage.complete() && coverage.failed == 0;
        }));
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("torus.normal_bound.face_", 0) == 0 &&
                coverage.complete() && coverage.failed == 0;
        }));
    std::printf("WEFT_TORUS_PREVIEW_LOD tris=%zu\n",
                result.value ? result.value->certified.triangles.size() : 0U);
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testFullTorusBody() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_torus_template", ".step");
    weft::writeStep(weft::makeFixture("torus"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        if (result.failure) {
            std::printf("torus mesh failure: %s: %s\n",
                        result.failure->code.c_str(),
                        result.failure->message.c_str());
        }
        return;
    }
    CHECK(result.value && result.value->certified.triangles.size() >= 48);
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("torus.chord_bound.face_", 0) == 0 &&
                coverage.complete();
        }));
    const weft::SecureMeshingResult again =
        weft::generateSecureMesh(imported, configuration());
    CHECK(again.value &&
          again.value->certified.topologyFingerprint ==
              result.value->certified.topologyFingerprint);
    std::printf("WEFT_TORUS_C tris=%zu verts=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.vertices.size(),
                result.value->certified.topologyFingerprint.c_str());
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testTorusMatrix() {
    // Wave C lock: structured wall fixture + face_116_torus densify/hardOrient
    // in matrix ctest (not fixture-only preview LOD).
    const char* marker = "WEFT_TORUS_MATRIX";
    CHECK(!matrixSettings().omitDeferredResiduals);
    std::size_t locked = 0;

    {
        const std::filesystem::path path =
            weft::test::uniqueTempPath("weft_torus_matrix_full", ".step");
        weft::writeStep(weft::makeFixture("torus"), path.string());
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        if (assertHardCertify(weft::generateSecureMesh(imported, matrixSettings()),
                              marker, "full_wall_fixture")) {
            ++locked;
        }
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }
    {
        const auto path = mp9ExtractPath("face_116_torus.step");
        CHECK(path.has_value());
        if (path) {
            const weft::ImportedModel imported =
                weft::importStepSecure(path->string());
            CHECK(imported.meshable());
            if (imported.meshable() &&
                assertHardCertify(
                    weft::generateSecureMesh(imported, matrixSettings()),
                    marker, "face_116_torus.step")) {
                ++locked;
            }
        }
    }
    {
        const auto path = mp9ExtractPath("face_115_torus.step");
        CHECK(path.has_value());
        if (path) {
            const weft::ImportedModel imported =
                weft::importStepSecure(path->string());
            CHECK(imported.meshable());
            if (imported.meshable() &&
                assertHardCertify(
                    weft::generateSecureMesh(imported, matrixSettings()),
                    marker, "face_115_torus.step")) {
                ++locked;
            }
        }
    }
    {
        const auto path = mp9ExtractPath("face_143_torus.step");
        CHECK(path.has_value());
        if (path) {
            const weft::ImportedModel imported =
                weft::importStepSecure(path->string());
            CHECK(imported.meshable());
            if (imported.meshable() &&
                assertHardCertify(
                    weft::generateSecureMesh(imported, matrixSettings()),
                    marker, "face_143_torus.step")) {
                ++locked;
            }
        }
    }
    CHECK(locked == 4);
    std::printf("%s locked=%zu/4 fail_closed=1 omit=0 "
                "face_116_in_matrix=1 face_115_orient_densify=1 "
                "face_143_uv_degen_densify=1 "
                "not_fixture_only_lod=1\n",
                marker, locked);
}

}  // namespace

int main() {
    try {
        testTorusMatrix();
        testTorusPreviewLodChordValid();
        testFullTorusBody();
    } catch (const std::exception& error) {
        std::printf("FAIL torus-template exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("torus template checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d torus template failure(s)\n", failures);
    return EXIT_FAILURE;
}
