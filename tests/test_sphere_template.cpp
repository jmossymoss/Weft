#include "weft/fixture.hpp"
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

}  // namespace

int main() {
    try {
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
