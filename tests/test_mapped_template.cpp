#include "weft/fixture.hpp"
#include "weft/mapped_template.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"

#include "test_temp_path.hpp"

#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Shape.hxx>

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

int countFaceEdges(const TopoDS_Shape& shape) {
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    return edges.Extent();
}

void testFreeformUvGridBody() {
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_freeform_template", ".step");
    const TopoDS_Shape shape = weft::makeFixture("freeform_patch");
    const int edgeCount = countFaceEdges(shape);
    CHECK(edgeCount == 5);
    std::printf("WEFT_FREE_A fixture=freeform_patch edge_count=%d\n",
                edgeCount);
    weft::writeStep(shape, path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    bool sawFree = false;
    for (const auto& record : recon.records) {
        for (const auto& code : record.conditionCodes) {
            if (code == "freeform.uv_grid_candidate") sawFree = true;
        }
    }
    CHECK(sawFree);
    std::printf("WEFT_FREE_A uv_grid_candidate=%d\n", (int)sawFree);

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
    // Distinct from four-sided mapped_patch fingerprint.
    CHECK(result.value->certified.topologyFingerprint !=
          "c6d2c3643ca44049");
    if (result.value->modeling.provenance ==
        weft::ModelingProvenanceKind::Independent) {
        CHECK(!result.value->modeling.polygons.empty());
        std::printf("WEFT_FREE_D modeling=Independent polys=%zu\n",
                    result.value->modeling.polygons.size());
    } else {
        CHECK(result.value->modeling.aliasesCertified);
        std::printf("WEFT_FREE_D modeling=CertifiedFloorAlias reason=%s\n",
                    result.value->modeling.safeFloorReason
                        ? result.value->modeling.safeFloorReason->c_str()
                        : "-");
    }
    std::printf("WEFT_FREE_B boundaries_via_mesh=ok\n");
    std::printf("WEFT_FREE_C tris=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.topologyFingerprint.c_str());

    const weft::SecureMeshingResult again =
        weft::generateSecureMesh(imported, configuration());
    CHECK(again);
    CHECK(again.value->certified.topologyFingerprint ==
          result.value->certified.topologyFingerprint);
    weft::SecureMeshingConfiguration loose = configuration();
    loose.sampling.chordTolerance = 4.0;
    weft::SecureMeshingConfiguration dense = configuration();
    dense.sampling.chordTolerance = 0.5;
    const weft::SecureMeshingResult looseR =
        weft::generateSecureMesh(imported, loose);
    const weft::SecureMeshingResult denseR =
        weft::generateSecureMesh(imported, dense);
    CHECK(looseR && denseR);
    std::printf(
        "WEFT_FREE_F loose_tris=%zu dense_tris=%zu fingerprint=%s\n",
        looseR.value->certified.triangles.size(),
        denseR.value->certified.triangles.size(),
        result.value->certified.topologyFingerprint.c_str());

    {
        const std::filesystem::path widePath =
            weft::test::uniqueTempPath("weft_freeform_wide", ".step");
        weft::writeStep(weft::makeFixture("ribbonnotch"), widePath.string());
        const weft::ImportedModel wide =
            weft::importStepSecure(widePath.string());
        const weft::SecureMeshingResult wideMesh =
            weft::generateSecureMesh(wide, configuration());
        CHECK(!wideMesh);
        CHECK(wideMesh.failure);
        std::printf("WEFT_FREE_C wider_refusal=%s\n",
                    wideMesh.failure->code.c_str());
        std::error_code ignoredWide;
        std::filesystem::remove(widePath, ignoredWide);
    }

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
    CHECK(result.value->modeling.provenance ==
              weft::ModelingProvenanceKind::Independent ||
          result.value->modeling.provenance ==
              weft::ModelingProvenanceKind::CertifiedFloorAlias);
    if (result.value->modeling.provenance ==
        weft::ModelingProvenanceKind::Independent) {
        CHECK(!result.value->modeling.polygons.empty());
        std::printf("WEFT_MAP_D modeling=Independent polys=%zu\n",
                    result.value->modeling.polygons.size());
    } else {
        CHECK(result.value->modeling.aliasesCertified);
        std::printf("WEFT_MAP_D modeling=CertifiedFloorAlias reason=%s\n",
                    result.value->modeling.safeFloorReason
                        ? result.value->modeling.safeFloorReason->c_str()
                        : "-");
    }
    std::printf("WEFT_MAP_C tris=%zu verts=%zu fingerprint=%s\n",
                result.value->certified.triangles.size(),
                result.value->certified.vertices.size(),
                result.value->certified.topologyFingerprint.c_str());

    weft::SecureMeshingConfiguration loose = configuration();
    loose.sampling.chordTolerance = 4.0;
    loose.sampling.minimumClosedCurveSegments = 8;
    weft::SecureMeshingConfiguration dense = configuration();
    dense.sampling.chordTolerance = 0.5;
    dense.sampling.minimumClosedCurveSegments = 32;
    const weft::SecureMeshingResult again =
        weft::generateSecureMesh(imported, configuration());
    CHECK(again);
    CHECK(again.value->certified.topologyFingerprint ==
          result.value->certified.topologyFingerprint);
    const weft::SecureMeshingResult looseR =
        weft::generateSecureMesh(imported, loose);
    const weft::SecureMeshingResult denseR =
        weft::generateSecureMesh(imported, dense);
    CHECK(looseR && denseR);
    std::printf(
        "WEFT_MAP_F loose_tris=%zu dense_tris=%zu fingerprint=%s\n",
        looseR.value->certified.triangles.size(),
        denseR.value->certified.triangles.size(),
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
