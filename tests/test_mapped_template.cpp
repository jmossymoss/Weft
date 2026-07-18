#include "weft/fixture.hpp"
#include "weft/mapped_template.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"

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
    settings.sampling.chordTolerance = 2.0;
    settings.sampling.normalAngleToleranceRadians = 1.2;
    settings.sampling.minimumClosedCurveSegments = 16;
    return settings;
}

void testFreeformUvGridBody() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_freeform_template", ".step");
    weft::writeStep(weft::makeFixture("freeform_patch"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    bool sawFree = false;
    bool sawGeneral = false;
    for (const auto& record : recon.records) {
        for (const auto& code : record.conditionCodes) {
            if (code == "freeform.uv_grid_candidate") sawFree = true;
            if (code == "freeform.general_deferred") sawGeneral = true;
        }
    }
    CHECK(sawFree);
    std::printf("WEFT_FREE_A uv_grid_candidate=%d general_deferred=%d\n",
                (int)sawFree, (int)sawGeneral);
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        if (result.failure) {
            std::printf("freeform mesh failure: %s: %s\n",
                        result.failure->code.c_str(),
                        result.failure->message.c_str());
        }
        return;
    }
    CHECK(result.value && result.value->certified.triangles.size() >= 8);
    std::printf("WEFT_FREE_C tris=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.topologyFingerprint.c_str());
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testRibbonMappedBody() {

    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_mapped_template", ".step");
    weft::writeStep(weft::makeFixture("mapped_patch"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        if (result.failure) {
            std::printf("mapped mesh failure: %s: %s\n",
                        result.failure->code.c_str(),
                        result.failure->message.c_str());
        }
        return;
    }
    CHECK(result.value && result.value->certified.triangles.size() >= 16);
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("mapped.chord_bound.face_", 0) == 0 &&
                coverage.complete();
        }));
    std::printf("WEFT_MAP_C tris=%zu verts=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.vertices.size(),
                result.value->certified.topologyFingerprint.c_str());
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

}  // namespace

int main() {
    try {
        testFreeformUvGridBody();
        testRibbonMappedBody();
    } catch (const std::exception& error) {
        std::printf("FAIL mapped-template exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("mapped template checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d mapped template failure(s)\n", failures);
    return EXIT_FAILURE;
}
