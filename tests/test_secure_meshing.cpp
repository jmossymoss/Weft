#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <set>
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

std::map<std::uint32_t, std::set<weft::StableId>> sampleFacesForEdge(
    const weft::SecureMeshingResult& result, weft::StableId edge) {
    std::map<std::uint32_t, std::set<weft::StableId>> samples;
    if (!result.value) return samples;
    for (const weft::CertifiedVertex& vertex :
         result.value->certified.vertices) {
        for (const weft::CertifiedVertexUse& use : vertex.provenance) {
            if (use.boundary.workingEdge == edge) {
                samples[use.boundary.sample.ordinal].insert(use.workingFace);
            }
        }
    }
    return samples;
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
    CHECK(std::any_of(
        first.validation.checks.begin(), first.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "secure_pipeline.periodic_uv_closure" &&
                coverage.expected != 0 && coverage.complete();
        }));
}

void testConnectedThroughHole() {
    const weft::SecureMeshingResult result = generateFixture("hole");
    checkSuccessfulResult(result);
    CHECK(result.value && result.value->certified.vertices.size() > 100);
    CHECK(result.value && result.value->certified.triangles.size() > 200);
}

void testExactEdgeIntervals() {
    TemporaryStep boxStep("box");
    const weft::ImportedModel box =
        weft::importStepSecure(boxStep.path().string());
    const weft::StableId boxEdge{weft::StableIdKind::Edge, 1};
    weft::SecureMeshingConfiguration pinnedBox = configuration();
    pinnedBox.exactEdgeIntervalCounts[boxEdge] = 4;
    const weft::SecureMeshingResult boxFirst =
        weft::generateSecureMesh(box, pinnedBox);
    const weft::SecureMeshingResult boxRepeated =
        weft::generateSecureMesh(box, pinnedBox);
    checkSuccessfulResult(boxFirst);
    checkSuccessfulResult(boxRepeated);
    CHECK(hasCoverage(
        boxFirst.validation,
        "secure_pipeline.exact_edge_interval_constraints"));
    CHECK(boxFirst.value &&
          boxFirst.value->generation.edgeDivisions.at(1) == 4);
    const auto boxSamples = sampleFacesForEdge(boxFirst, boxEdge);
    CHECK(boxSamples.size() == 5);
    for (const auto& [ordinal, faces] : boxSamples) {
        (void)ordinal;
        CHECK(faces.size() >= 2);
    }
    CHECK(boxFirst.value && boxRepeated.value &&
          boxFirst.value->certified.topologyFingerprint ==
              boxRepeated.value->certified.topologyFingerprint);

    TemporaryStep cylinderStep("cylinder");
    const weft::ImportedModel cylinder =
        weft::importStepSecure(cylinderStep.path().string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(cylinder);
    std::vector<weft::StableId> circleEdges;
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Curve &&
            record.familyCode == "circle") {
            circleEdges.push_back(record.subjectId);
        }
    }
    CHECK(circleEdges.size() == 2);
    if (circleEdges.size() == 2) {
        weft::SecureMeshingConfiguration pinnedCylinder = configuration();
        pinnedCylinder.exactEdgeIntervalCounts[circleEdges.front()] = 64;
        const weft::SecureMeshingResult cylinderResult =
            weft::generateSecureMesh(cylinder, pinnedCylinder);
        checkSuccessfulResult(cylinderResult);
        CHECK(sampleFacesForEdge(cylinderResult, circleEdges[0]).size() == 64);
        CHECK(sampleFacesForEdge(cylinderResult, circleEdges[1]).size() == 64);
        CHECK(cylinderResult.value &&
              cylinderResult.value->generation.edgeDivisions.at(
                  static_cast<int>(circleEdges[0].ordinal)) == 64);
        CHECK(cylinderResult.value &&
              cylinderResult.value->generation.edgeDivisions.at(
                  static_cast<int>(circleEdges[1].ordinal)) == 64);

        weft::SecureMeshingConfiguration conflictingCylinder = configuration();
        conflictingCylinder.exactEdgeIntervalCounts[circleEdges[0]] = 64;
        conflictingCylinder.exactEdgeIntervalCounts[circleEdges[1]] = 48;
        const weft::SecureMeshingResult conflicting =
            weft::generateSecureMesh(cylinder, conflictingCylinder);
        CHECK(!conflicting);
        CHECK(conflicting.failure &&
              conflicting.failure->code == "interval.exact_conflict");

        pinnedCylinder.exactEdgeIntervalCounts[circleEdges.front()] = 8;
        const weft::SecureMeshingResult belowMinimum =
            weft::generateSecureMesh(cylinder, pinnedCylinder);
        CHECK(!belowMinimum);
        CHECK(belowMinimum.failure &&
              belowMinimum.failure->code == "interval.exact_below_minimum");
    }

    weft::SecureMeshingConfiguration invalid = configuration();
    invalid.exactEdgeIntervalCounts[
        {weft::StableIdKind::Edge, 999999}] = 4;
    const weft::SecureMeshingResult missingEdge =
        weft::generateSecureMesh(box, invalid);
    CHECK(!missingEdge);
    CHECK(missingEdge.failure &&
          missingEdge.failure->code ==
              "secure_pipeline.edge_constraint_invalid");

    invalid = configuration();
    invalid.exactEdgeIntervalCounts[boxEdge] = 0;
    const weft::SecureMeshingResult zeroCount =
        weft::generateSecureMesh(box, invalid);
    CHECK(!zeroCount);
    CHECK(zeroCount.failure &&
          zeroCount.failure->code ==
              "secure_pipeline.edge_constraint_invalid");

    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Curve &&
            record.familyCode == "line") {
            weft::SecureMeshingConfiguration axialEdge = configuration();
            axialEdge.exactEdgeIntervalCounts[record.subjectId] = 2;
            const weft::SecureMeshingResult axialEdgeRefusal =
                weft::generateSecureMesh(cylinder, axialEdge);
            CHECK(!axialEdgeRefusal);
            CHECK(axialEdgeRefusal.failure &&
                  axialEdgeRefusal.failure->code ==
                      "cylinder.axial_samples_require_interior_provenance");
            break;
        }
    }
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
        testExactEdgeIntervals();
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
