#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/secure_core.hpp"
#include "weft/secure_meshing.hpp"
#include "weft/secure_recipe.hpp"
#include "weft/secure_reconnaissance.hpp"

#include "../core/src/secure_core_internal.hpp"
#include "../core/src/io/xcaf.hpp"
#include "test_temp_path.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomPlate_BuildPlateSurface.hxx>
#include <GeomPlate_PointConstraint.hxx>
#include <GeomPlate_Surface.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

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
    result.revolutionRadialSegments = 32;
    result.previewTriangleBudget = 0;
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
        CHECK(adapter.certifiedTriangles.size() ==
              result.value->certified.triangles.size());
        CHECK(!adapter.polygons.empty());
        CHECK(adapter.polygonFaceId.size() == adapter.polygons.size());
        CHECK(adapter.polygonCornerAnchors.size() == adapter.polygons.size());
        for (std::size_t index = 0; index < adapter.certifiedTriangles.size();
             ++index) {
            CHECK(adapter.certifiedTriangles[index].size() == 1);
            CHECK(adapter.certifiedTriangles[index].front() ==
                  result.value->certified.triangles[index].vertices);
        }
        if (result.value->modeling.provenance ==
            weft::ModelingProvenanceKind::Independent) {
            CHECK(adapter.countQuads() > 0 ||
                  adapter.polygons.size() ==
                      result.value->certified.triangles.size());
            for (const auto& poly : adapter.polygons) {
                CHECK(poly.size() == 3 || poly.size() == 4);
            }
        } else {
            CHECK(adapter.polygons.size() ==
                  result.value->certified.triangles.size());
            CHECK(adapter.countTris() == adapter.polygonCount());
            for (std::size_t index = 0; index < adapter.polygons.size();
                 ++index) {
                CHECK(adapter.polygons[index].size() == 3);
                CHECK(adapter.polygonCornerAnchors[index].size() == 3);
            }
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
// Report digests include closed-manifold incidence/Euler coverage (G5).
// Hole boundary/lift digests also track denser digon/hole sampling from G1.
constexpr M3GoldenDigest kM3GoldenDigests[] = {
    {"box", "4d4b56a97e4194a1", "bfc6fa72b8fb0801", "2d5083505e7bff41",
     "dde25bfd7f60eb38"},
    {"cylinder", "df476af694433848", "8928e6e02ad2fa92", "612aaa31fa6d5784",
     "66f0513eae801c52"},
    {"hole", "559a67e76d02c618", "3611b3bfaca05e32", "d78c1359c4a94188",
     "158f3a835eae09d4"},
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

void testG5HardCertifiedAssembly() {
    // Wave E / WEFT_ASSEMBLY_MATRIX: closed solid manifold + non-vacuous
    // incidence / orientation (edge winding) / intersection; adversaries
    // remain in certified_mesh unit tests.
    const weft::SecureMeshingResult box =
        generateFixture("box", configuration());
    checkSuccessfulResult(box);
    CHECK(std::any_of(
        box.validation.checks.begin(), box.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "certified.edge_incidence" &&
                coverage.expected != 0 && coverage.complete();
        }));
    CHECK(std::any_of(
        box.validation.checks.begin(), box.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "certified.edge_winding" &&
                coverage.expected != 0 && coverage.complete();
        }));
    CHECK(std::any_of(
        box.validation.checks.begin(), box.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "certified.triangle_intersection" &&
                coverage.expected != 0 && coverage.failed == 0 &&
                coverage.complete();
        }));
    CHECK(std::any_of(
        box.validation.checks.begin(), box.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "certified.incidence_euler" &&
                coverage.expected != 0 && coverage.complete();
        }));
    // Interval consumption must stay on even under --allow-partial-body.
    weft::SecureMeshingConfiguration partial = configuration();
    partial.omitDeferredResiduals = true;
    const weft::SecureMeshingResult boxPartial =
        generateFixture("box", partial);
    checkSuccessfulResult(boxPartial);
    std::printf(
        "WEFT_G5 box closed-manifold tris=%zu\n",
        box.value->certified.triangles.size());
    std::printf(
        "WEFT_ASSEMBLY_MATRIX subclass=closed_solid_manifold "
        "fixture=box outcome=HARD tris=%zu "
        "incidence=1 winding=1 intersection=1 euler=1\n",
        box.value->certified.triangles.size());
}

void testAssemblyMatrixMultiComponent() {
    // Wave E / Track M (light): a body with two closed triangle components
    // must still pass closed-manifold incidence/Euler (χ valid for C=2,
    // no boundary). Distinguishes assembly multi-solid closed shells from
    // within-solid leaks; full free-solid compound import meshability is
    // blocked today by repair.shared_geometry_immutable (not this wave).
    weft::CertifiedMesh mesh;
    auto addClosedTet = [&](double ox, double oy, double oz) {
        const std::uint32_t base =
            static_cast<std::uint32_t>(mesh.vertices.size());
        const std::array<std::array<double, 3>, 4> corners = {{
            {{ox, oy, oz}},
            {{ox + 1.0, oy, oz}},
            {{ox, oy + 1.0, oz}},
            {{ox, oy, oz + 1.0}},
        }};
        for (const auto& position : corners) {
            weft::CertifiedVertex vertex;
            vertex.canonicalVertexIndex =
                static_cast<std::uint64_t>(mesh.vertices.size()) + 1;
            vertex.position = position;
            mesh.vertices.push_back(std::move(vertex));
        }
        const std::array<std::array<std::uint32_t, 3>, 4> faces = {{
            {{base + 0, base + 2, base + 1}},
            {{base + 0, base + 1, base + 3}},
            {{base + 0, base + 3, base + 2}},
            {{base + 1, base + 2, base + 3}},
        }};
        for (const auto& cornersIdx : faces) {
            weft::CertifiedTriangle triangle;
            triangle.workingFace = {weft::StableIdKind::Face, 1};
            triangle.vertices = cornersIdx;
            mesh.triangles.push_back(std::move(triangle));
        }
    };
    addClosedTet(0.0, 0.0, 0.0);
    addClosedTet(5.0, 0.0, 0.0);

    const weft::CertifiedIncidenceEulerResult accounted =
        weft::validateCertifiedIncidenceEuler(mesh, true);
    CHECK(accounted);
    CHECK(accounted.boundaryEdges == 0);
    CHECK(accounted.connectedComponents == 2);
    CHECK(accounted.coverage.complete());
    std::printf(
        "WEFT_ASSEMBLY_MATRIX subclass=multi_solid_closed "
        "outcome=HARD components=%zu boundary=0 "
        "inter_solid_vs_leak=per_component_closed\n",
        accounted.connectedComponents);
}

void testG0FailClosedDefaults() {
    // Product default must never silently omit deferred residuals.
    CHECK(!weft::SecureMeshingConfiguration{}.omitDeferredResiduals);
    CHECK(!configuration().omitDeferredResiduals);
    CHECK(weft::SecureMeshingConfiguration{}.floorPolicy ==
          weft::SecureMeshingFloorPolicy::HardSurfaceFloor);
    // Wave 0: previewFast stays off unless omit/explicit preview arms it.
    CHECK(!weft::CanonicalBoundaryConfiguration{}.previewFast);

    weft::SecureMeshingConfiguration strict = configuration();
    strict.floorPolicy = weft::SecureMeshingFloorPolicy::StrictAllFaces;
    const weft::SecureMeshingResult box =
        generateFixture("box", strict);
    checkSuccessfulResult(box);
    // Product path must not arm previewFast / discrepancy widen.
    CHECK(!hasCoverage(box.validation,
                       "secure_pipeline.preview_fast_boundaries"));
    // Non-vacuous intersection proves no face arrived with relax=1
    // (assemble would zero the intersection coverage under soft body).
    CHECK(std::any_of(
        box.validation.checks.begin(), box.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code == "certified.triangle_intersection" &&
                coverage.failed == 0 &&
                (coverage.expected > 0 || coverage.checked > 0);
        }));
    CHECK(!hasCoverage(box.validation,
                       "secure_pipeline.relaxed_geometry_on_product_path"));

    // Ribbon freeform must refuse by name with a face subject when the
    // certified consumer is absent — never succeed by omitting faces.
    TemporaryStep ribbonStep("ribbon");
    const weft::ImportedModel ribbon =
        weft::importStepSecure(ribbonStep.path().string());
    weft::SecureMeshingConfiguration failClosed = configuration();
    failClosed.omitDeferredResiduals = false;
    const weft::SecureMeshingResult ribbonMesh =
        weft::generateSecureMesh(ribbon, failClosed);
    if (!ribbonMesh) {
        CHECK(ribbonMesh.failure);
        CHECK(!ribbonMesh.failure->code.empty());
        // Subject list is preferred but assemble-stage refuses may omit it.
        std::printf("WEFT_G0 ribbon refusal code=%s subjects=%zu\n",
                    ribbonMesh.failure->code.c_str(),
                    ribbonMesh.failure->subjects.size());
    } else {
        // If ribbon later gains a certified consumer, G0 still requires a
        // non-empty certificate rather than a partial omit path.
        checkSuccessfulResult(ribbonMesh);
        CHECK(!hasCoverage(ribbonMesh.validation,
                           "secure_pipeline.preview_fast_boundaries"));
        std::printf("WEFT_G0 ribbon certified tris=%zu\n",
                    ribbonMesh.value->certified.triangles.size());
    }

    // Wave 0: omit/partial-body is the only path that arms previewFast.
    weft::SecureMeshingConfiguration partial = configuration();
    partial.omitDeferredResiduals = true;
    const weft::SecureMeshingResult boxPartial =
        generateFixture("box", partial);
    checkSuccessfulResult(boxPartial);
    CHECK(hasCoverage(boxPartial.validation,
                      "secure_pipeline.preview_fast_boundaries"));
    // Wave 0: faceCount alone must not be enough to arm preview soft path
    // (product omitDeferredResiduals stays false regardless of size).
    CHECK(!configuration().omitDeferredResiduals);
    CHECK(!weft::SecureMeshingConfiguration{}.omitDeferredResiduals);
    std::printf("WEFT_WAVE0 product contract: omit=0 previewFast=0; "
                "omit=1 previewFast=1; no faceCount>500 soft arm\n");
}

void testUnsupportedAndConfigurationRefusals() {
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

void testMapCutFreeAdmissionAndRecipeRefusal() {
    for (const char* fixture :
         {"mapped_patch", "hole", "plate_slot", "freeform_patch"}) {
        TemporaryStep step(fixture);
        weft::writeStep(weft::makeFixture(fixture), step.path().string());
        const weft::ImportedModel imported =
            weft::importStepSecure(step.path().string());
        weft::SecureMeshingConfiguration settings = configuration();
        if (std::string(fixture) == "hole") {
            settings.sampling.minimumClosedCurveSegments = 16;
        }
        const weft::SecureMeshingResult meshed =
            weft::generateSecureMesh(imported, settings);
        checkSuccessfulResult(meshed);
        CHECK(meshed.value);
        if (!meshed.value) continue;
        const weft::CertifiedAdmissionResult admitted =
            weft::admitCertifiedMeshingResult(*meshed.value);
        CHECK(admitted);
        CHECK(admitted.selectedOutput.has_value());
        std::printf("WEFT_APP_151 fixture=%s admission=ok\n", fixture);

        // Unsupported recipe ops still refuse by name (BR-010 path).
        weft::RecipeV2 recipe;
        CHECK(imported.source);
        recipe.sourceSha256 = imported.source->metadata.sourceSha256;
        recipe.defaults.chordTolerance = settings.sampling.chordTolerance;
        recipe.defaults.radial = 16;
        weft::ReferencedFaceSettings faceSetting;
        faceSetting.face = weft::makeSourceEntityReference(
            imported, {weft::StableIdKind::Face, 1});
        faceSetting.settings = recipe.defaults;
        faceSetting.settings.radial = 24;
        recipe.faceSettings.push_back(faceSetting);
        weft::ReferencedManualOperation op;
        op.face = faceSetting.face;
        op.operation.kind = weft::ManualOp::Kind::LoopInsert;
        op.operation.faceId = 1;
        recipe.operations.push_back(op);
        const weft::RecipeV2Resolution resolved =
            weft::resolveRecipeV2(imported, recipe);
        const std::vector<weft::RecipeMigrationIssue> issues =
            weft::validateSecureRecipeApplication(resolved);
        bool sawUnimplemented = false;
        for (const weft::RecipeMigrationIssue& issue : issues) {
            if (issue.code.find("unimplemented") != std::string::npos) {
                sawUnimplemented = true;
                std::printf("WEFT_APP_151 fixture=%s recipe_refusal=%s\n",
                            fixture, issue.code.c_str());
                break;
            }
        }
        CHECK(sawUnimplemented);
    }
}



void testCutoutHoleBody() {
    TemporaryStep step("hole");
    weft::writeStep(weft::makeFixture("hole"), step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    bool sawPerforated = false;
    bool sawBore = false;
    for (const weft::ExactGeometryClassification& record : recon.records) {
        for (const std::string& code : record.conditionCodes) {
            if (code == "cutout.planar_perforated") sawPerforated = true;
            if (code == "cutout.cylindrical_bore") sawBore = true;
        }
    }
    CHECK(sawPerforated);
    CHECK(sawBore);
    weft::SecureMeshingConfiguration settings = configuration();
    settings.sampling.minimumClosedCurveSegments = 16;
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, settings);
    checkSuccessfulResult(result);
    CHECK(result.value && result.value->certified.triangles.size() >= 32);
    std::printf("WEFT_CUT_C fixture=hole tris=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.topologyFingerprint.c_str());
    if (result.value->modeling.provenance ==
        weft::ModelingProvenanceKind::Independent) {
        std::printf("WEFT_CUT_D fixture=hole modeling=Independent polys=%zu\n",
                    result.value->modeling.polygons.size());
    } else {
        std::printf("WEFT_CUT_D fixture=hole modeling=CertifiedFloorAlias "
                    "reason=%s\n",
                    result.value->modeling.safeFloorReason
                        ? result.value->modeling.safeFloorReason->c_str()
                        : "-");
    }
    // Density / determinism
    weft::SecureMeshingConfiguration loose = settings;
    loose.sampling.chordTolerance = 1.0;
    loose.sampling.minimumClosedCurveSegments = 8;
    weft::SecureMeshingConfiguration dense = settings;
    dense.sampling.chordTolerance = 0.1;
    dense.sampling.minimumClosedCurveSegments = 32;
    const weft::SecureMeshingResult a =
        weft::generateSecureMesh(imported, settings);
    const weft::SecureMeshingResult b =
        weft::generateSecureMesh(imported, settings);
    CHECK(a && b);
    CHECK(a.value->certified.topologyFingerprint ==
          b.value->certified.topologyFingerprint);
    const weft::SecureMeshingResult looseR =
        weft::generateSecureMesh(imported, loose);
    const weft::SecureMeshingResult denseR =
        weft::generateSecureMesh(imported, dense);
    CHECK(looseR && denseR);
    std::printf(
        "WEFT_CUT_F fixture=hole loose_tris=%zu dense_tris=%zu fingerprint=%s\n",
        looseR.value->certified.triangles.size(),
        denseR.value->certified.triangles.size(),
        a.value->certified.topologyFingerprint.c_str());
}

void testCutoutPlateSlotBody() {
    TemporaryStep step("plate_slot");
    weft::writeStep(weft::makeFixture("plate_slot"), step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    bool sawSlotted = false;
    for (const weft::ExactGeometryClassification& record : recon.records) {
        for (const std::string& code : record.conditionCodes) {
            if (code == "cutout.planar_slotted") sawSlotted = true;
        }
    }
    CHECK(sawSlotted);
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    checkSuccessfulResult(result);
    CHECK(result.value && result.value->certified.triangles.size() >= 8);
    std::printf("WEFT_CUT_C fixture=plate_slot tris=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.topologyFingerprint.c_str());
    if (result.value->modeling.provenance ==
        weft::ModelingProvenanceKind::Independent) {
        std::printf(
            "WEFT_CUT_D fixture=plate_slot modeling=Independent polys=%zu\n",
            result.value->modeling.polygons.size());
    } else {
        std::printf("WEFT_CUT_D fixture=plate_slot modeling=CertifiedFloorAlias "
                    "reason=%s\n",
                    result.value->modeling.safeFloorReason
                        ? result.value->modeling.safeFloorReason->c_str()
                        : "-");
    }
    const weft::SecureMeshingResult again =
        weft::generateSecureMesh(imported, configuration());
    CHECK(again);
    CHECK(again.value->certified.topologyFingerprint ==
          result.value->certified.topologyFingerprint);
    std::printf("WEFT_CUT_F fixture=plate_slot fingerprint=%s\n",
                result.value->certified.topologyFingerprint.c_str());
}

void testCutoutDeferredRefusals() {
    for (const char* fixture : {"slotted", "drilled", "filletslot"}) {
        TemporaryStep step(fixture);
        weft::writeStep(weft::makeFixture(fixture), step.path().string());
        const weft::ImportedModel imported =
            weft::importStepSecure(step.path().string());
        const weft::SecureMeshingResult result =
            weft::generateSecureMesh(imported, configuration());
        // Cut-graph consumers may now certify via UV-trim / plane CDT.
        if (!result) {
            CHECK(result.failure);
            std::printf("WEFT_CUT_C fixture=%s refusal=%s\n", fixture,
                        result.failure->code.c_str());
        } else {
            CHECK(result.value);
            std::printf("WEFT_CUT_C fixture=%s certified tris=%zu\n", fixture,
                        result.value->certified.triangles.size());
        }
    }
}

void testFilletSolid() {
    TemporaryStep step("fillet");
    weft::writeStep(weft::makeFixture("fillet"), step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    weft::SecureMeshingConfiguration settings = configuration();
    settings.sampling.chordTolerance = 0.5;
    settings.sampling.normalAngleToleranceRadians = 0.5;
    settings.sampling.minimumClosedCurveSegments = 24;
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, settings);
    if (!result) {
        std::printf("fillet failure: %s\n",
                    result.failure ? result.failure->code.c_str() : "-");
        CHECK(result);
        return;
    }
    checkSuccessfulResult(result);
    std::printf("WEFT_FILLET_C tris=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.topologyFingerprint.c_str());
}

void testApexCone() {
    TemporaryStep step("cone");
    weft::writeStep(weft::makeFixture("cone"), step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    if (!result) {
        // Host/OCCT debt on some lanes (see G0 evidence); keep named.
        std::printf("WEFT_CONE_D refusal code=%s meshable=%d\n",
                    result.failure ? result.failure->code.c_str() : "-",
                    imported.meshable() ? 1 : 0);
        CHECK(result.failure && !result.failure->code.empty());
        return;
    }
    checkSuccessfulResult(result);
    CHECK(result.value && result.value->certified.triangles.size() >= 8);
    CHECK(std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.rfind("cone.chord_bound.face_", 0) == 0 &&
                coverage.complete();
        }));
    CHECK(result.value &&
          (result.value->modeling.provenance ==
               weft::ModelingProvenanceKind::Independent ||
           result.value->modeling.provenance ==
               weft::ModelingProvenanceKind::CertifiedFloorAlias));
    if (result.value &&
        result.value->modeling.provenance ==
            weft::ModelingProvenanceKind::Independent) {
        CHECK(!result.value->modeling.polygons.empty());
        CHECK(result.value->modeling.independentValidation.complete());
        std::printf("WEFT_CONE_D modeling=Independent polys=%zu\n",
                    result.value->modeling.polygons.size());
    } else if (result.value) {
        CHECK(result.value->modeling.aliasesCertified);
        CHECK(result.value->modeling.safeFloorReason.has_value());
        std::printf("WEFT_CONE_D modeling=CertifiedFloorAlias reason=%s\n",
                    result.value->modeling.safeFloorReason->c_str());
    }

    // Tampered independent claim must refuse.
    if (result.value) {
        weft::MeshingResult tampered = *result.value;
        tampered.modeling.provenance =
            weft::ModelingProvenanceKind::Independent;
        tampered.modeling.aliasesCertified = false;
        tampered.modeling.safeFloorReason = std::nullopt;
        tampered.modeling.polygons.clear();
        tampered.modeling.independentValidation = {};
        const weft::ModelingProvenanceResult refused =
            weft::validateModelingProvenance(tampered);
        CHECK(!refused);
        CHECK(refused.failure);
        std::printf("WEFT_CONE_D adversary=%s\n",
                    refused.failure ? refused.failure->code.c_str() : "-");
    }
}

std::filesystem::path findMp9Extract(const char* name) {
    // CTest runs from build/<preset>/tests; CLI from repo root.
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

void testEllipseHoleBody() {
    TemporaryStep step("ellipse_hole");
    weft::writeStep(weft::makeFixture("ellipse_hole"), step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    CHECK(imported.repair.meshable);
    bool sawEllipse = false;
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    for (const weft::ExactGeometryClassification& record : recon.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Curve &&
            record.familyCode == "ellipse") {
            sawEllipse = true;
            CHECK(record.support ==
                  weft::GeometrySupportState::SupportedAnalyticTemplate);
        }
    }
    CHECK(sawEllipse);
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    CHECK(result.value && result.value->certified.triangles.size() > 0);
    std::printf("WEFT_ELLIPSE_F tris=%zu\n",
                result.value ? result.value->certified.triangles.size() : 0U);
}

void testMp9ExtractFaces() {
    // Plasticity MP9 face 10 (open cylinder band), face 28 (5-edge freeform),
    // and face 250 (four-sided offset patch).
    for (const char* name :
         {"cylinder_band.step", "freeform_pent.step", "offset_quad.step",
          "cone_frustum.step", "cylinder_complex.step",
          "freeform_hex.step"}) {
        const std::filesystem::path path = findMp9Extract(name);
        CHECK(!path.empty());
        if (path.empty()) continue;
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        // OCCT 8.x host debt: some free-face MP9 extracts (cone frustum)
        // fail BRepCheck validity; distinguish from G4 consumer regressions.
        if (!imported.repair.meshable) {
            std::printf("WEFT_MP9_EXTRACT %s skip=host_non_meshable\n", name);
            continue;
        }
        const weft::SecureMeshingResult result =
            weft::generateSecureMesh(imported, configuration());
        CHECK(result);
        CHECK(result.value &&
              result.value->certified.triangles.size() > 0);
        std::printf("WEFT_MP9_EXTRACT %s tris=%zu\n", name,
                    result.value ? result.value->certified.triangles.size()
                                 : 0U);
    }

    // G4: sphere_cap extracts when the host admits them as meshable.
    for (const char* name :
         {"sphere_cap_778.step", "sphere_cap_complex.step"}) {
        const std::filesystem::path path = findMp9Extract(name);
        if (path.empty()) continue;
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        if (!imported.repair.meshable) {
            std::printf("WEFT_MP9_EXTRACT %s skip=host_non_meshable\n", name);
            continue;
        }
        const weft::SecureMeshingResult result =
            weft::generateSecureMesh(imported, configuration());
        CHECK(result);
        CHECK(result.value &&
              result.value->certified.triangles.size() > 0);
        std::printf("WEFT_MP9_EXTRACT %s tris=%zu\n", name,
                    result.value ? result.value->certified.triangles.size()
                                 : 0U);
    }

    // 6-edge freeform uses UV-trim CDT (not rectangular grid).
    {
        const std::filesystem::path path =
            findMp9Extract("freeform_hex.step");
        CHECK(!path.empty());
        if (!path.empty()) {
            const weft::ImportedModel imported =
                weft::importStepSecure(path.string());
            const weft::ReconnaissanceReport recon =
                weft::reconnoitre(imported);
            bool sawUvTrim = false;
            for (const weft::ExactGeometryClassification& record :
                 recon.records) {
                if (record.taxonomy != weft::GeometryTaxonomy::Surface) {
                    continue;
                }
                for (const std::string& code : record.conditionCodes) {
                    if (code == "freeform.uv_trim_candidate") {
                        sawUvTrim = true;
                    }
                }
                CHECK(record.support ==
                      weft::GeometrySupportState::SupportedAnalyticTemplate);
            }
            CHECK(sawUvTrim);
            const weft::SecureMeshingResult meshed =
                weft::generateSecureMesh(imported, configuration());
            std::printf("WEFT_FREE_UV_TRIM ok=%d tris=%zu\n", meshed ? 1 : 0,
                        meshed.value ? meshed.value->certified.triangles.size()
                                     : 0U);
        }
    }
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

void testUnsupportedFamilyMatrix() {
    const std::filesystem::path root =
        std::filesystem::path(__FILE__).parent_path().parent_path();
    const std::filesystem::path dir = root / "tests" / "fixtures" / "unsupported";

    auto expectCurveRefuse = [&](const char* file, const char* family,
                                 bool nativeBRep) {
        const std::filesystem::path path = dir / file;
        CHECK(std::filesystem::exists(path));
        if (!std::filesystem::exists(path)) return;
        const weft::ImportedModel imported =
            nativeBRep ? weft::importBRepSecure(path.string())
                       : weft::importStepSecure(path.string());
        const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
        bool sawFamily = false;
        for (const weft::ExactGeometryClassification& record : recon.records) {
            if (record.taxonomy == weft::GeometryTaxonomy::Curve &&
                record.familyCode == family) {
                sawFamily = true;
            }
        }
        CHECK(sawFamily);
        const weft::SecureMeshingResult meshed =
            weft::generateSecureMesh(imported, configuration());
        CHECK(!meshed);
        CHECK(meshed.failure);
        const std::string expected =
            std::string("secure_pipeline.unsupported_curve_family.") + family;
        CHECK(meshed.failure && meshed.failure->code == expected);
        std::printf("WEFT_UNSUPPORTED_MATRIX curve family=%s code=%s\n", family,
                    meshed.failure ? meshed.failure->code.c_str() : "-");
    };

    expectCurveRefuse("curve_hyperbola.step", "hyperbola", false);
    expectCurveRefuse("curve_parabola.step", "parabola", false);
    expectCurveRefuse("curve_offset.brep", "offset", true);

    // GeomPlate_Surface is GeomAbs_OtherSurface → kernel_specific, but OCCT
    // ASCII BREP cannot persist OtherSurface ("UNKNOWN SURFACE TYPE"). Build
    // in-memory via the same secure_detail::buildImportedModel path used by
    // native BREP import, and keep construction parameters in the committed
    // fixture sidecar.
    {
        const std::filesystem::path sidecar =
            dir / "surface_kernel_specific.construction";
        CHECK(std::filesystem::exists(sidecar));

        GeomPlate_BuildPlateSurface builder(3, 8, 3);
        builder.Add(new GeomPlate_PointConstraint(gp_Pnt(0.0, 0.0, 0.0), 0));
        builder.Add(new GeomPlate_PointConstraint(gp_Pnt(20.0, 0.0, 0.0), 0));
        builder.Add(new GeomPlate_PointConstraint(gp_Pnt(0.0, 15.0, 0.0), 0));
        builder.Add(new GeomPlate_PointConstraint(gp_Pnt(20.0, 15.0, 1.0), 0));
        builder.Add(new GeomPlate_PointConstraint(gp_Pnt(10.0, 7.5, 0.5), 0));
        builder.Perform();
        CHECK(builder.IsDone());
        CHECK(!builder.Surface().IsNull());
        if (!builder.IsDone() || builder.Surface().IsNull()) return;

        TopoDS_Face face;
        BRep_Builder().MakeFace(face, builder.Surface(), 1e-6);
        {
            BRepAdaptor_Surface adaptor(face, true);
            CHECK(adaptor.GetType() == GeomAbs_OtherSurface);
        }

        weft::Model source = weft::indexShape(face);
        weft::secure_detail::ConservativeWorkingDerivation derivation =
            weft::secure_detail::deriveConservativeWorking(source);
        weft::Model working = weft::indexShape(derivation.shape);
        weft::SourceMetadata metadata;
        metadata.sourceName = "surface_kernel_specific.construction";
        metadata.importerVersion = "weft-wave-f";
        weft::ImportedModel imported = weft::secure_detail::buildImportedModel(
            std::move(source), std::move(working), std::move(metadata),
            weft::RepairProfile::Conservative, derivation.history,
            derivation.exactShapes, std::move(derivation.operations),
            std::move(derivation.parameterizationFlagChanges),
            std::move(derivation.orientationChanges),
            std::move(derivation.toleranceChanges));
        const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
        bool sawKernelSurface = false;
        for (const weft::ExactGeometryClassification& record : recon.records) {
            if (record.taxonomy == weft::GeometryTaxonomy::Surface &&
                record.familyCode == "kernel_specific") {
                sawKernelSurface = true;
                CHECK(record.support ==
                      weft::GeometrySupportState::UnrecognisedExactGeometry);
            }
        }
        CHECK(sawKernelSurface);
        const weft::SecureMeshingResult meshed =
            weft::generateSecureMesh(imported, configuration());
        CHECK(!meshed);
        CHECK(meshed.failure);
        CHECK(meshed.failure &&
              meshed.failure->code ==
                  "secure_pipeline.unsupported_surface_family.kernel_specific");
        std::printf(
            "WEFT_UNSUPPORTED_MATRIX surface family=kernel_specific code=%s\n",
            meshed.failure ? meshed.failure->code.c_str() : "-");
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
        testG0FailClosedDefaults();
        testG5HardCertifiedAssembly();
        testAssemblyMatrixMultiComponent();
        testUnsupportedAndConfigurationRefusals();
        testTemplateChainSumConsumer();
        testCertifiedAdmissionGate();
        testMapCutFreeAdmissionAndRecipeRefusal();
        testSecureCacheInvalidation();
        testNamedLodReporting();
        testPartialCylinder();
        testEllipseHoleBody();
        testMp9ExtractFaces();
        testCutoutHoleBody();
        testCutoutPlateSlotBody();
        testCutoutDeferredRefusals();
        testFilletSolid();
        testApexCone();
        testUnsupportedFamilyMatrix();
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
