#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"
#include "weft/torus_template.hpp"

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
    settings.sampling.chordTolerance = 0.5;
    settings.sampling.normalAngleToleranceRadians = 0.9;
    settings.sampling.minimumClosedCurveSegments = 20;
    settings.revolutionRadialSegments = 32;
    settings.previewTriangleBudget = 0;
    return settings;
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

}  // namespace

int main() {
    try {
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
