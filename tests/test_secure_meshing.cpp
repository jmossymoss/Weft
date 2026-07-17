#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

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
    explicit TemporaryStep(const std::string& fixture)
        : path_(weft::test::uniqueTempPath(
              "weft_secure_pipeline_" + fixture, ".step")) {
        weft::writeStep(weft::makeFixture(fixture), path_.string());
    }

    ~TemporaryStep() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

weft::SecureMeshingConfiguration configuration() {
    weft::SecureMeshingConfiguration result;
    result.sampling.chordTolerance = 0.05;
    result.sampling.normalAngleToleranceRadians = 0.1;
    result.sampling.minimumClosedCurveSegments = 16;
    result.sampling.maximumSegmentCount = 4096;
    return result;
}

bool hasCoverage(const weft::ValidationCertificate& certificate,
                 const std::string& code) {
    return std::any_of(
        certificate.checks.begin(), certificate.checks.end(),
        [&](const weft::ValidationCoverage& coverage) {
            return coverage.code == code;
        });
}

weft::SecureMeshingResult generateFixture(
    const std::string& fixture,
    const weft::SecureMeshingConfiguration& settings = configuration()) {
    TemporaryStep step(fixture);
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    return weft::generateSecureMesh(imported, settings);
}

void checkSuccessfulResult(const weft::SecureMeshingResult& result) {
    CHECK(result);
    CHECK(!result.failure);
    CHECK(result.validation.complete());
    CHECK(result.value && result.value->validation.complete());
    CHECK(result.value && !result.value->certified.vertices.empty());
    CHECK(result.value && !result.value->certified.triangles.empty());
    CHECK(result.value &&
          result.value->certified.topologyFingerprint.size() == 16);
    CHECK(result.value && result.value->modeling.aliasesCertified);
    CHECK(result.value && result.value->modeling.safeFloorReason.has_value());
    CHECK(hasCoverage(result.validation,
                      "repair.source_working_correspondence"));
    CHECK(hasCoverage(result.validation,
                      "secure_pipeline.vertex_curve_identity"));
    CHECK(hasCoverage(result.validation, "certified.edge_incidence"));
    for (const weft::ValidationCoverage& coverage :
         result.validation.checks) {
        CHECK(coverage.complete());
        CHECK(coverage.failed == 0);
        CHECK(coverage.skipped == 0);
    }
    if (result.value) {
        const weft::PolyMesh adapter =
            weft::makeCertifiedPolyMeshAdapter(*result.value);
        CHECK(adapter.vertices.size() ==
              result.value->certified.vertices.size());
        CHECK(adapter.polygons.size() ==
              result.value->certified.triangles.size());
        CHECK(adapter.polygonCornerAnchors.size() ==
              adapter.polygons.size());
        CHECK(adapter.certifiedTriangles.size() == adapter.polygons.size());
        CHECK(adapter.countTris() == adapter.polygonCount());
        for (std::size_t index = 0; index < adapter.polygons.size(); ++index) {
            CHECK(adapter.polygons[index].size() == 3);
            CHECK(adapter.polygonCornerAnchors[index].size() == 3);
            CHECK(adapter.certifiedTriangles[index].size() == 1);
            CHECK(adapter.certifiedTriangles[index].front() ==
                  result.value->certified.triangles[index].vertices);
        }
    }
}

void testPlanarBox() {
    const weft::SecureMeshingResult result = generateFixture("box");
    checkSuccessfulResult(result);
    CHECK(result.value && result.value->certified.vertices.size() == 8);
    CHECK(result.value && result.value->certified.triangles.size() == 12);
}

void testFullCylinderDeterminism() {
    TemporaryStep step("cylinder");
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    const weft::SecureMeshingResult first =
        weft::generateSecureMesh(imported, configuration());
    const weft::SecureMeshingResult second =
        weft::generateSecureMesh(imported, configuration());
    checkSuccessfulResult(first);
    checkSuccessfulResult(second);
    CHECK(first.value && first.value->certified.vertices.size() >= 32);
    CHECK(first.value &&
          first.value->certified.triangles.size() ==
              first.value->certified.vertices.size() * 2 - 4);
    CHECK(first.value && second.value &&
          first.value->certified.topologyFingerprint ==
              second.value->certified.topologyFingerprint);
    CHECK(std::any_of(
        first.validation.checks.begin(), first.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("cylinder.chord_bound.face_", 0) == 0 &&
                coverage.complete();
        }));
}

void testConnectedThroughHole() {
    const weft::SecureMeshingResult result = generateFixture("hole");
    if (result.failure) {
        std::printf("through-hole refusal: %s: %s\n",
                    result.failure->code.c_str(),
                    result.failure->message.c_str());
    }
    checkSuccessfulResult(result);
    CHECK(result.value && result.value->certified.vertices.size() > 100);
    CHECK(result.value && result.value->certified.triangles.size() > 200);
}

void testUnsupportedAndConfigurationRefusals() {
    const weft::SecureMeshingResult sphere = generateFixture("sphere");
    CHECK(!sphere);
    CHECK(sphere.failure);
    CHECK(sphere.failure && !sphere.failure->code.empty());

    weft::SecureMeshingConfiguration invalid = configuration();
    invalid.sampling.chordTolerance = -1.0;
    const weft::SecureMeshingResult refused =
        generateFixture("cylinder", invalid);
    CHECK(!refused);
    CHECK(refused.failure &&
          refused.failure->code == "interval.invalid_configuration");

    weft::SecureMeshingConfiguration axial = configuration();
    axial.cylinderAxialIntervals = 2;
    const weft::SecureMeshingResult axialRefusal =
        generateFixture("cylinder", axial);
    CHECK(!axialRefusal);
    CHECK(axialRefusal.failure &&
          axialRefusal.failure->code ==
              "cylinder.axial_samples_require_interior_provenance");
}

}  // namespace

int main() {
    try {
        testPlanarBox();
        testFullCylinderDeterminism();
        testConnectedThroughHole();
        testUnsupportedAndConfigurationRefusals();
    } catch (const std::exception& error) {
        std::printf("FAIL secure-meshing exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("secure end-to-end meshing checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d secure meshing failure(s)\n", failures);
    return EXIT_FAILURE;
}
