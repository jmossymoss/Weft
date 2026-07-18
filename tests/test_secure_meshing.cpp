#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
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
    CHECK(result.value &&
          (result.value->modeling.provenance ==
               weft::ModelingProvenanceKind::CertifiedFloorAlias ||
           result.value->modeling.provenance ==
               weft::ModelingProvenanceKind::Independent));
    if (result.value &&
        result.value->modeling.provenance ==
            weft::ModelingProvenanceKind::CertifiedFloorAlias) {
        CHECK(result.value->modeling.aliasesCertified);
        CHECK(result.value->modeling.safeFloorReason.has_value());
    }
    if (result.value &&
        result.value->modeling.provenance ==
            weft::ModelingProvenanceKind::Independent) {
        CHECK(!result.value->modeling.aliasesCertified);
        CHECK(result.value->modeling.independentValidation.complete());
        CHECK(!result.value->modeling.polygons.empty());
    }
    CHECK(hasCoverage(result.validation,
                      "repair.source_working_correspondence"));
    CHECK(hasCoverage(result.validation,
                      "secure_pipeline.vertex_curve_identity"));
    CHECK(hasCoverage(result.validation,
                      "secure_pipeline.critical_parameter_events"));
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
    CHECK(result.value &&
          result.value->modeling.provenance ==
              weft::ModelingProvenanceKind::Independent);
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

struct M3GoldenDigest {
    const char* fixture;
    const char* counts;
    const char* boundary;
    const char* lifts;
    const char* report;
};

// Proven bit-identical on Linux GCC / OCCT 7.6 (this lane). Report digests
// include certified.triangle_intersection coverage from WP-020; count /
// boundary / lift digests are unchanged from the WP-015 cross-platform set.
constexpr M3GoldenDigest kM3GoldenDigests[] = {
    {"box", "4d4b56a97e4194a1", "bfc6fa72b8fb0801", "2d5083505e7bff41",
     "dde25bfd7f60eb38"},
    {"cylinder", "df476af694433848", "8928e6e02ad2fa92", "612aaa31fa6d5784",
     "66f0513eae801c52"},
    {"hole", "559a67e76d02c618", "303cc08b7ef4e792", "c362170936b054d8",
     "d39b86eee593c637"},
};

void checkDeterminismDigest(const M3GoldenDigest& golden,
                            const weft::M3DeterminismDigest& first,
                            const weft::M3DeterminismDigest& second) {
    CHECK(first.counts.size() == 16);
    CHECK(first.boundary.size() == 16);
    CHECK(first.lifts.size() == 16);
    CHECK(first.report.size() == 16);
    CHECK(first.counts == second.counts);
    CHECK(first.boundary == second.boundary);
    CHECK(first.lifts == second.lifts);
    CHECK(first.report == second.report);
    CHECK(first.counts == golden.counts);
    CHECK(first.boundary == golden.boundary);
    CHECK(first.lifts == golden.lifts);
    CHECK(first.report == golden.report);
    std::printf(
        "WEFT_M3_DIGEST fixture=%s counts=%s boundary=%s lifts=%s report=%s\n",
        golden.fixture, first.counts.c_str(), first.boundary.c_str(),
        first.lifts.c_str(), first.report.c_str());
}

void testCrossPlatformM3Determinism() {
    for (const M3GoldenDigest& golden : kM3GoldenDigests) {
        TemporaryStep step(golden.fixture);
        const weft::ImportedModel imported =
            weft::importStepSecure(step.path().string());
        const weft::SecureMeshingConfiguration settings = configuration();
        const weft::SecureMeshingResult first =
            weft::generateSecureMesh(imported, settings);
        const weft::SecureMeshingResult second =
            weft::generateSecureMesh(imported, settings);
        checkSuccessfulResult(first);
        checkSuccessfulResult(second);
        const weft::M3DeterminismDigest firstDigest =
            weft::digestM3Determinism(first);
        const weft::M3DeterminismDigest secondDigest =
            weft::digestM3Determinism(second);
        checkDeterminismDigest(golden, firstDigest, secondDigest);
        CHECK(first.value && second.value &&
              first.value->certified.topologyFingerprint ==
                  second.value->certified.topologyFingerprint);
    }
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
    CHECK(sphere.failure &&
          (sphere.failure->code ==
               "boundary.critical_segmentation_unsupported" ||
           sphere.failure->code ==
               "secure_pipeline.unsupported_surface_family" ||
           sphere.failure->code ==
               "secure_pipeline.unsupported_curve_family" ||
           sphere.failure->code ==
               "secure_pipeline.degenerate_curve_unsupported" ||
           sphere.failure->code.rfind("boundary.", 0) == 0 ||
           sphere.failure->code.rfind("secure_pipeline.", 0) == 0));

    weft::SecureMeshingConfiguration invalid = configuration();
    invalid.sampling.chordTolerance = -1.0;
    const weft::SecureMeshingResult refused =
        generateFixture("cylinder", invalid);
    CHECK(!refused);
    CHECK(refused.failure &&
          refused.failure->code == "interval.invalid_configuration");

    weft::SecureMeshingConfiguration axial = configuration();
    axial.cylinderAxialIntervals = 2;
    const weft::SecureMeshingResult axialCertified =
        generateFixture("cylinder", axial);
    CHECK(axialCertified);
    CHECK(axialCertified.value &&
          axialCertified.value->certified.triangles.size() > 0);
    CHECK(axialCertified.validation.complete());
    bool sawInterior = false;
    if (axialCertified.value) {
        for (const weft::CertifiedVertex& vertex :
             axialCertified.value->certified.vertices) {
            if (vertex.cylinderInterior) {
                sawInterior = true;
                CHECK(vertex.cylinderInterior->axialRing == 1);
            }
        }
    }
    CHECK(sawInterior);
}

void testTemplateChainSumConsumer() {
    TemporaryStep boxStep("box");
    const weft::ImportedModel box =
        weft::importStepSecure(boxStep.path().string());
    const weft::StableId edgeA{weft::StableIdKind::Edge, 1};
    const weft::StableId edgeB{weft::StableIdKind::Edge, 2};
    const weft::StableId edgeC{weft::StableIdKind::Edge, 3};
    const weft::StableId edgeD{weft::StableIdKind::Edge, 4};

    weft::SecureMeshingConfiguration baseline = configuration();
    baseline.exactEdgeIntervalCounts[edgeA] = 2;
    baseline.exactEdgeIntervalCounts[edgeB] = 3;
    const weft::SecureMeshingResult withoutSum =
        weft::generateSecureMesh(box, baseline);
    checkSuccessfulResult(withoutSum);
    CHECK(withoutSum.value &&
          withoutSum.value->generation.edgeDivisions.at(3) == 1);

    weft::SecureMeshingConfiguration withSum = baseline;
    withSum.edgeChainSums.push_back({{edgeA, edgeB}, {edgeC}});
    const weft::SecureMeshingResult first =
        weft::generateSecureMesh(box, withSum);
    const weft::SecureMeshingResult replay =
        weft::generateSecureMesh(box, withSum);
    checkSuccessfulResult(first);
    checkSuccessfulResult(replay);
    CHECK(hasCoverage(first.validation,
                      "secure_pipeline.chain_sum_requested"));
    CHECK(hasCoverage(first.validation, "secure_pipeline.chain_sum_solved"));
    CHECK(hasCoverage(first.validation,
                      "secure_pipeline.chain_sum_consumed"));
    CHECK(first.value && first.value->generation.edgeDivisions.at(1) == 2);
    CHECK(first.value && first.value->generation.edgeDivisions.at(2) == 3);
    CHECK(first.value && first.value->generation.edgeDivisions.at(3) == 5);
    CHECK(sampleFacesForEdge(first, edgeC).size() == 6);
    CHECK(first.value && replay.value &&
          first.value->certified.topologyFingerprint ==
              replay.value->certified.topologyFingerprint);

    weft::SecureMeshingConfiguration coupled = withSum;
    coupled.sampling.maximumSegmentCount = 64;
    coupled.edgeChainSums.push_back({{edgeC}, {edgeD}});
    const weft::SecureMeshingResult coupledResult =
        weft::generateSecureMesh(box, coupled);
    const weft::SecureMeshingResult coupledReplay =
        weft::generateSecureMesh(box, coupled);
    checkSuccessfulResult(coupledResult);
    checkSuccessfulResult(coupledReplay);
    CHECK(coupledResult.value &&
          coupledResult.value->generation.edgeDivisions.at(3) == 5);
    CHECK(coupledResult.value &&
          coupledResult.value->generation.edgeDivisions.at(4) == 5);
    CHECK(coupledResult.value && coupledReplay.value &&
          coupledResult.value->certified.topologyFingerprint ==
              coupledReplay.value->certified.topologyFingerprint);

    weft::SecureMeshingConfiguration conflict = withSum;
    conflict.exactEdgeIntervalCounts[edgeC] = 4;
    const weft::SecureMeshingResult conflicting =
        weft::generateSecureMesh(box, conflict);
    CHECK(!conflicting);
    CHECK(conflicting.failure &&
          conflicting.failure->code == "interval.sum_infeasible");

    weft::SecureMeshingConfiguration parity = withSum;
    parity.requireEvenEdgeIntervals.insert(edgeC);
    parity.exactEdgeIntervalCounts[edgeC] = 5;
    const weft::SecureMeshingResult parityConflict =
        weft::generateSecureMesh(box, parity);
    CHECK(!parityConflict);
    CHECK(parityConflict.failure &&
          parityConflict.failure->code == "interval.exact_parity_conflict");

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
        weft::SecureMeshingConfiguration belowMinimum = configuration();
        belowMinimum.exactEdgeIntervalCounts[circleEdges[0]] = 8;
        belowMinimum.edgeChainSums.push_back(
            {{circleEdges[0]}, {circleEdges[1]}});
        const weft::SecureMeshingResult minimumRefusal =
            weft::generateSecureMesh(cylinder, belowMinimum);
        CHECK(!minimumRefusal);
        CHECK(minimumRefusal.failure &&
              minimumRefusal.failure->code ==
                  "interval.exact_below_minimum");
    }

    weft::SecureMeshingConfiguration invalidSum = configuration();
    invalidSum.edgeChainSums.push_back(
        {{{weft::StableIdKind::Edge, 999999}}, {edgeA}});
    const weft::SecureMeshingResult invalid =
        weft::generateSecureMesh(box, invalidSum);
    CHECK(!invalid);
    CHECK(invalid.failure &&
          invalid.failure->code == "secure_pipeline.chain_sum_invalid");

    const weft::ReconnaissanceReport boxRecon = weft::reconnoitre(box);
    weft::IntervalProblem problem;
    for (const weft::EdgeTopologyRecord& topology :
         box.working->snapshot.edgeTopology) {
        std::uint32_t count = 1;
        std::optional<std::uint32_t> exact;
        if (topology.id == edgeA) {
            count = 2;
            exact = 2;
        } else if (topology.id == edgeB) {
            count = 3;
            exact = 3;
        }
        problem.variables.push_back(
            {{weft::StableIdKind::Boundary, topology.id.ordinal},
             static_cast<double>(count), count, false, exact});
    }
    problem.sums.push_back(
        {{{weft::StableIdKind::Boundary, 1},
          {weft::StableIdKind::Boundary, 2}},
         {{weft::StableIdKind::Boundary, 3}}});
    const weft::IntervalSolveResult solved =
        weft::solveIntervals(problem, withSum.sampling);
    CHECK(solved);
    const weft::CanonicalBoundaryBuildResult boundaries =
        weft::buildCanonicalBoundaries(box, boxRecon, *solved.solution);
    CHECK(boundaries);
    if (solved && boundaries) {
        weft::CanonicalBoundarySet tampered = *boundaries.value;
        CHECK(!tampered.boundaries.empty());
        if (!tampered.boundaries.empty()) {
            tampered.boundaries.front().intervalCount += 1;
            const auto refused = weft::certifySolvedIntervalConsumption(
                *solved.solution, tampered);
            CHECK(refused.has_value());
            CHECK(refused &&
                  refused->code ==
                      "secure_pipeline.interval_consumption_mismatch");
        }
        if (first.value) {
            weft::MeshingResult meshTamper = *first.value;
            const auto edgeKey = static_cast<int>(edgeC.ordinal);
            CHECK(meshTamper.generation.edgeDivisions.contains(edgeKey));
            meshTamper.generation.edgeDivisions[edgeKey] =
                meshTamper.generation.edgeDivisions[edgeKey] + 1;
            const auto refused = weft::certifySolvedIntervalConsumption(
                *solved.solution, *boundaries.value, &meshTamper);
            CHECK(refused.has_value());
            CHECK(refused &&
                  refused->code ==
                      "secure_pipeline.interval_consumption_mismatch");
        }
    }
}

void testNamedLodReporting() {
    const weft::SecureMeshingResult result = generateFixture("cylinder");
    checkSuccessfulResult(result);
    CHECK(result.value);
    if (!result.value) return;
    const auto& effects = result.value->generation.namedLodEffects;
    CHECK(effects.contains("cylinderAxialIntervals"));
    CHECK(effects.contains("chordTolerance"));
    CHECK(effects.contains("selectedOutput"));
    CHECK(effects.at("selectedOutput").find("modeling.") == 0 ||
          effects.at("selectedOutput") == "certified");
}

void testSecureCacheInvalidation() {
    const weft::SecureMeshingResult ok = generateFixture("box");
    checkSuccessfulResult(ok);
    CHECK(ok.value);
    if (!ok.value) return;

    weft::SecureCacheKey key;
    key.sourceSha256 = "abc";
    key.recipeFingerprint = "recipe-1";
    key.settingsFingerprint = "settings-1";
    key.implementationVersion = weft::kSecureImplementationVersion;
    key.certificateFingerprint = ok.value->certified.topologyFingerprint;

    const weft::SecureCacheLookupResult hit =
        weft::lookupSecureCache(key, key, &*ok.value);
    CHECK(hit.hit);
    CHECK(!hit.failure);

    weft::SecureCacheKey other = key;
    other.settingsFingerprint = "settings-2";
    const weft::SecureCacheLookupResult miss =
        weft::lookupSecureCache(key, other, &*ok.value);
    CHECK(!miss.hit);
    CHECK(miss.failure && miss.failure->code == "cache.key_mismatch");

    weft::SecureCacheKey badVersion = key;
    badVersion.implementationVersion = "weft-secure-0";
    const weft::SecureCacheLookupResult version =
        weft::lookupSecureCache(key, badVersion, &*ok.value);
    CHECK(!version.hit);
    CHECK(version.failure &&
          version.failure->code == "cache.implementation_mismatch");

    weft::MeshingResult tamperedPayload = *ok.value;
    tamperedPayload.certified.topologyFingerprint = "deadbeefdeadbeef";
    const weft::SecureCacheLookupResult corrupt =
        weft::lookupSecureCache(key, key, &tamperedPayload);
    CHECK(!corrupt.hit);
    CHECK(corrupt.failure &&
          corrupt.failure->code == "cache.certificate_mismatch");

    const weft::SecureCacheLookupResult empty =
        weft::lookupSecureCache(key, key, nullptr);
    CHECK(!empty.hit);
    CHECK(empty.failure && empty.failure->code == "cache.entry_corrupt");
}

void testCertifiedAdmissionGate() {
    const weft::SecureMeshingResult ok = generateFixture("box");
    checkSuccessfulResult(ok);
    CHECK(ok.value);
    if (!ok.value) return;
    const weft::CertifiedAdmissionResult admitted =
        weft::admitCertifiedMeshingResult(*ok.value);
    CHECK(admitted);
    CHECK(admitted.selectedOutput.has_value());

    weft::MeshingResult tampered = *ok.value;
    tampered.validation.checks.clear();
    const weft::CertifiedAdmissionResult incomplete =
        weft::admitCertifiedMeshingResult(tampered);
    CHECK(!incomplete);
    CHECK(incomplete.failure &&
          incomplete.failure->code == "admission.certificate_incomplete");

    const weft::CertifiedAdmissionResult stale =
        weft::admitCertifiedMeshingResult(*ok.value, 2, 1);
    CHECK(!stale);
    CHECK(stale.failure &&
          stale.failure->code == "admission.stale_generation");

    weft::MeshingResult unexplained = *ok.value;
    unexplained.modeling.provenance =
        weft::ModelingProvenanceKind::CertifiedFloorAlias;
    unexplained.modeling.aliasesCertified = false;
    unexplained.modeling.safeFloorReason = std::nullopt;
    const weft::CertifiedAdmissionResult badModeling =
        weft::admitCertifiedMeshingResult(unexplained);
    CHECK(!badModeling);
}

void testApexCone() {
    TemporaryStep step("cone");
    weft::writeStep(weft::makeFixture("cone"), step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    checkSuccessfulResult(result);
    CHECK(result.value && result.value->certified.triangles.size() >= 8);
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("cone.chord_bound.face_", 0) == 0 &&
                coverage.complete();
        }));
}

void testPartialCylinder() {
    TemporaryStep step("partial_cylinder");
    weft::writeStep(weft::makeFixture("partial_cylinder"),
                    step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    checkSuccessfulResult(result);
    CHECK(result.value &&
          result.value->certified.triangles.size() > 0);

    // Malformed request: force unequal open-arc counts through an exact
    // edge override on one rim arc.
    weft::ReconnaissanceReport reconnaissance = weft::reconnoitre(imported);
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Curve ||
            record.familyCode != "circle") {
            continue;
        }
        const weft::EdgeTopologyRecord* topology = nullptr;
        for (const weft::EdgeTopologyRecord& edge :
             imported.working->snapshot.edgeTopology) {
            if (edge.id == record.subjectId) {
                topology = &edge;
                break;
            }
        }
        if (!topology || !topology->lowerVertex || !topology->upperVertex ||
            *topology->lowerVertex == *topology->upperVertex) {
            continue;
        }
        weft::SecureMeshingConfiguration mismatched = configuration();
        mismatched.exactEdgeIntervalCounts[record.subjectId] = 7;
        const weft::SecureMeshingResult refused =
            weft::generateSecureMesh(imported, mismatched);
        CHECK(!refused);
        CHECK(refused.failure);
        break;
    }
}

}  // namespace

int main() {
    try {
        testPlanarBox();
        testFullCylinderDeterminism();
        testConnectedThroughHole();
        testCrossPlatformM3Determinism();
        testExactEdgeIntervals();
        testUnsupportedAndConfigurationRefusals();
        testTemplateChainSumConsumer();
        testCertifiedAdmissionGate();
        testSecureCacheInvalidation();
        testNamedLodReporting();
        testPartialCylinder();
        testApexCone();
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
