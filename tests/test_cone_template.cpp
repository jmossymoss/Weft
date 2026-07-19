#include "weft/cone_template.hpp"
#include "weft/fixture.hpp"
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
    settings.sampling.chordTolerance = 0.25;
    settings.sampling.normalAngleToleranceRadians = 0.35;
    settings.sampling.minimumClosedCurveSegments = 8;
    settings.revolutionRadialSegments = 16;
    settings.previewTriangleBudget = 0;
    settings.cylinderAxialIntervals = 1;
    return settings;
}

void testApexConeBody() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_cone_template", ".step");
    weft::writeStep(weft::makeFixture("cone"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        if (result.failure) {
            std::printf("cone mesh failure: %s: %s\n",
                        result.failure->code.c_str(),
                        result.failure->message.c_str());
        }
        return;
    }
    CHECK(result.validation.complete());
    CHECK(result.value && result.value->validation.complete());
    CHECK(result.value && result.value->certified.triangles.size() >= 8);
    CHECK(result.value && result.value->certified.vertices.size() >= 5);
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("cone.chord_bound.face_", 0) == 0 &&
                coverage.complete();
        }));
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("cone.boundary_coverage.face_", 0) ==
                    0 &&
                coverage.complete();
        }));
    std::printf("WEFT_CONE_C tris=%zu verts=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.vertices.size(),
                result.value->certified.topologyFingerprint.c_str());
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testConeDensitySweepAndDeterminism() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_cone_template_density", ".step");
    weft::writeStep(weft::makeFixture("cone"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());

    weft::SecureMeshingConfiguration loose = configuration();
    loose.sampling.chordTolerance = 0.5;
    loose.sampling.minimumClosedCurveSegments = 8;
    weft::SecureMeshingConfiguration dense = configuration();
    dense.sampling.chordTolerance = 0.05;
    dense.sampling.minimumClosedCurveSegments = 16;
    dense.revolutionRadialSegments = 32;

    const weft::SecureMeshingResult first =
        weft::generateSecureMesh(imported, loose);
    const weft::SecureMeshingResult second =
        weft::generateSecureMesh(imported, loose);
    const weft::SecureMeshingResult denser =
        weft::generateSecureMesh(imported, dense);
    CHECK(first);
    CHECK(second);
    CHECK(denser);
    CHECK(first.value && second.value &&
          first.value->certified.topologyFingerprint ==
              second.value->certified.topologyFingerprint);
    CHECK(first.value && denser.value &&
          first.value->certified.topologyFingerprint !=
              denser.value->certified.topologyFingerprint);
    CHECK(denser.value && first.value &&
          denser.value->certified.triangles.size() >
              first.value->certified.triangles.size());
    // Determinism is proven by repeated-run fingerprint equality above. The
    // exact hex differs across OCCT versions (curved-surface evaluations vary
    // beyond the ULP stabilizer), so it is printed as evidence, not asserted
    // as a cross-build golden.
    const weft::SecureMeshingResult baseline =
        weft::generateSecureMesh(imported, configuration());
    const weft::SecureMeshingResult baselineAgain =
        weft::generateSecureMesh(imported, configuration());
    CHECK(baseline.value && baselineAgain.value &&
          baseline.value->certified.topologyFingerprint ==
              baselineAgain.value->certified.topologyFingerprint);
    CHECK(baseline.value &&
          baseline.value->certified.topologyFingerprint.size() == 16);
    std::printf(
        "WEFT_CONE_F density loose_tris=%zu dense_tris=%zu fingerprint=%s "
        "baseline=%s\n",
        first.value->certified.triangles.size(),
        denser.value->certified.triangles.size(),
        first.value->certified.topologyFingerprint.c_str(),
        baseline.value->certified.topologyFingerprint.c_str());
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testConeChordRefusal() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_cone_template_chord", ".step");
    weft::writeStep(weft::makeFixture("cone"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    weft::SecureMeshingConfiguration tight = configuration();
    tight.sampling.chordTolerance = 1e-12;
    tight.sampling.minimumClosedCurveSegments = 8;
    const weft::SecureMeshingResult refused =
        weft::generateSecureMesh(imported, tight);
    CHECK(!refused);
    CHECK(refused.failure);
    if (refused.failure) {
        std::printf("WEFT_CONE_C adversary=%s\n",
                    refused.failure->code.c_str());
        CHECK(refused.failure->code == "cone.chord_bound_exceeded" ||
              refused.failure->code == "interval.count_exceeds_maximum" ||
              refused.failure->code.rfind("cone.", 0) == 0 ||
              refused.failure->code.rfind("interval.", 0) == 0 ||
              refused.failure->code.rfind("secure_pipeline.", 0) == 0 ||
              refused.failure->code.rfind("boundary.", 0) == 0);
    }
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testTruncatedConeFrustumBody() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_cone_frustum", ".step");
    weft::writeStep(weft::makeFixture("truncated_cone"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    bool sawSupportedCone = false;
    for (const weft::ExactGeometryClassification& record : recon.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Surface &&
            record.familyCode == "cone") {
            CHECK(record.support ==
                  weft::GeometrySupportState::SupportedAnalyticTemplate);
            CHECK(record.trimDomain !=
                  weft::TrimDomainClass::TouchesOneSingularity);
            sawSupportedCone = true;
        }
    }
    CHECK(sawSupportedCone);
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    CHECK(result.value && result.value->certified.triangles.size() > 0);
    // Structured frustum band should pair into modelling quads.
    std::size_t quads = 0;
    if (result.value) {
        CHECK(result.value->modeling.provenance ==
              weft::ModelingProvenanceKind::Independent);
        for (const weft::ModelingPolygon& polygon :
             result.value->modeling.polygons) {
            if (polygon.vertices.size() == 4) ++quads;
        }
    }
    CHECK(quads > 0);
    std::printf("WEFT_CONE_FRUSTUM tris=%zu quads=%zu\n",
                result.value ? result.value->certified.triangles.size() : 0U,
                quads);
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

}  // namespace

int main() {
    try {
        testApexConeBody();
        testTruncatedConeFrustumBody();
        testConeDensitySweepAndDeterminism();
        testConeChordRefusal();
    } catch (const std::exception& error) {
        std::printf("FAIL cone-template exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("cone template checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d cone template failure(s)\n", failures);
    return EXIT_FAILURE;
}
