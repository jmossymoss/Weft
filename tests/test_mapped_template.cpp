#include "weft/fixture.hpp"
#include "weft/mapped_template.hpp"
#include "weft/model.hpp"
#include "weft/secure_meshing.hpp"

#include "test_temp_path.hpp"

#include <TopExp.hxx>
#include <TopoDS_Shape.hxx>
#include <NCollection_IndexedMap.hxx>
#include <TopTools_ShapeMapHasher.hxx>

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
    settings.revolutionRadialSegments = 32;
    settings.previewTriangleBudget = 0;
    settings.omitDeferredResiduals = false;
    return settings;
}

bool hasCoverage(const weft::ValidationCertificate& certificate,
                 const std::string& code) {
    return std::any_of(
        certificate.checks.begin(), certificate.checks.end(),
        [&](const weft::ValidationCoverage& coverage) {
            return coverage.code == code;
        });
}

int countFaceEdges(const TopoDS_Shape& shape) {
    NCollection_IndexedMap<TopoDS_Shape, TopTools_ShapeMapHasher> edges;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    return edges.Extent();
}

std::filesystem::path findMp9Extract(const char* name) {
    const std::filesystem::path candidates[] = {
        std::filesystem::path("tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("../tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("../../tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("../../../tests/fixtures/mp9_extracts") / name,
        std::filesystem::path("fixtures/mp9_extracts") / name,
    };
    for (const auto& path : candidates) {
        if (std::filesystem::exists(path)) return path;
    }
    return {};
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
        if (!wideMesh) {
            CHECK(wideMesh.failure);
            std::printf("WEFT_FREE_C wider_refusal=%s\n",
                        wideMesh.failure->code.c_str());
        } else {
            CHECK(wideMesh.value);
            std::printf("WEFT_FREE_C wider_certified tris=%zu\n",
                        wideMesh.value->certified.triangles.size());
        }
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

void testG3Mp9CrashFace136NoAv() {
    // MP9 body AV locus (bspline annulus, freeform.uv_grid_candidate): must
    // certify or named-refuse — never ACCESS_VIOLATION in curved UV CDT.
    const std::filesystem::path path =
        findMp9Extract("crash_face_136_r0.step");
    CHECK(!path.empty());
    if (path.empty()) return;
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    CHECK(imported.repair.meshable);
    if (!imported.repair.meshable) return;
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        std::printf("WEFT_G3_CRASH136 refuse=%s\n",
                    result.failure ? result.failure->code.c_str() : "?");
        return;
    }
    CHECK(result.value && result.value->certified.triangles.size() > 0);
    std::printf("WEFT_G3_CRASH136 tris=%zu\n",
                result.value->certified.triangles.size());
}

void testG3Face33Orientation() {
    // MP9 body assemble refused certified.triangle_orientation_invalid on
    // face 33 (bspline / mapped.four_sided). Extract must hard-certify with
    // windingsMatchOrientedFaceNormal (no Reversed double-flip / no soft).
    const std::filesystem::path path =
        findMp9Extract("face_33_orient.step");
    CHECK(!path.empty());
    if (path.empty()) return;
    const weft::ImportedModel imported =
        weft::importStepSecure(path.string());
    CHECK(imported.repair.meshable);
    if (!imported.repair.meshable) return;
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        std::printf("WEFT_G3_FACE33 fail=%s\n",
                    result.failure ? result.failure->code.c_str() : "?");
        return;
    }
    CHECK(result.validation.complete());
    CHECK(result.value && result.value->certified.triangles.size() > 0);
    std::printf("WEFT_G3_FACE33 tris=%zu complete=%d\n",
                result.value->certified.triangles.size(),
                result.validation.complete() ? 1 : 0);
}

void testG3Face103MappedDensify() {
    // MP9 assemble next refuse after face 33: chord_bound → residual UV-trim
    // against-N. Densify retry must take lattice +N / windingsMatch.
    const std::filesystem::path path =
        findMp9Extract("face_103_orient.step");
    CHECK(!path.empty());
    if (path.empty()) return;
    const weft::ImportedModel imported =
        weft::importStepSecure(path.string());
    CHECK(imported.repair.meshable);
    if (!imported.repair.meshable) return;
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        std::printf("WEFT_G3_FACE103 fail=%s\n",
                    result.failure ? result.failure->code.c_str() : "?");
        return;
    }
    CHECK(result.value && result.value->certified.triangles.size() > 0);
    std::printf("WEFT_G3_FACE103 tris=%zu\n",
                result.value->certified.triangles.size());
}

void testG3Face107HardOrient() {
    // MP9 body pinned certified.triangle_orientation_invalid on face 107
    // after soft UV-trim residual (windingsMatch=0). Extract + product
    // defaults must hard-certify (lattice or UV-trim +N, never residual).
    const std::filesystem::path path =
        findMp9Extract("face_107_orient.step");
    CHECK(!path.empty());
    if (path.empty()) return;
    const weft::ImportedModel imported =
        weft::importStepSecure(path.string());
    CHECK(imported.repair.meshable);
    if (!imported.repair.meshable) return;
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    CHECK(result);
    if (!result) {
        std::printf("WEFT_G3_FACE107 fail=%s\n",
                    result.failure ? result.failure->code.c_str() : "?");
        return;
    }
    CHECK(result.validation.complete());
    CHECK(result.value && result.value->certified.triangles.size() > 0);
    const bool residualSoft = std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.find("uv_trim_orientation") !=
                       std::string::npos &&
                   coverage.failed == 0 && coverage.skipped > 0;
        });
    CHECK(!residualSoft);
    CHECK(!hasCoverage(result.validation,
                       "secure_pipeline.preview_fast_boundaries"));
    CHECK(!hasCoverage(result.validation,
                       "secure_pipeline.relaxed_geometry_on_product_path"));
    std::printf("WEFT_G3_FACE107 tris=%zu complete=%d\n",
                result.value->certified.triangles.size(),
                result.validation.complete() ? 1 : 0);
}

void testG3Mp9FreeformExtracts() {
    // Meshable Plasticity extracts: Independent modelling where topology
    // allows; hard seam refuse routes to UV-trim (no soft seam skip).
    for (const char* name : {"freeform_pent.step", "freeform_mapped_fallback.step",
                             "bspline_139.step", "offset_quad.step",
                             "freeform_hex.step", "freeform_135.step",
                             "freeform_137.step", "freeform_138.step",
                             "freeform_186.step", "freeform_399.step",
                             "freeform_553.step"}) {
        const std::filesystem::path path = findMp9Extract(name);
        CHECK(!path.empty());
        if (path.empty()) continue;
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        CHECK(imported.repair.meshable);
        if (!imported.repair.meshable) {
            std::printf("WEFT_G3_EXTRACT %s meshable=0\n", name);
            continue;
        }
        const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
        bool sawMapped = false;
        bool sawUvGrid = false;
        bool sawUvTrim = false;
        for (const weft::ExactGeometryClassification& record : recon.records) {
            if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
            for (const std::string& code : record.conditionCodes) {
                if (code == "mapped.four_sided_candidate") sawMapped = true;
                if (code == "freeform.uv_grid_candidate") sawUvGrid = true;
                if (code == "freeform.uv_trim_candidate") sawUvTrim = true;
            }
        }
        const weft::SecureMeshingResult result =
            weft::generateSecureMesh(imported, configuration());
        // Named refuse when no single winding hard-orients (never soft
        // residual / never AV). Codes must stay family-scoped.
        const bool namedHardRefuse =
            !result && result.failure &&
            (result.failure->code == "mapped.uv_trim_orientation_unresolved" ||
             result.failure->code == "mapped.orientation_unresolved" ||
             result.failure->code ==
                 "freeform.uv_trim_orientation_unresolved") &&
            (std::string(name) == "freeform_mapped_fallback.step" ||
             std::string(name) == "bspline_139.step");
        if (namedHardRefuse) {
            std::printf("WEFT_G3_EXTRACT %s named_refuse=%s\n", name,
                        result.failure->code.c_str());
            continue;
        }
        CHECK(result);
        if (!result) {
            std::printf("WEFT_G3_EXTRACT %s fail=%s\n", name,
                        result.failure ? result.failure->code.c_str() : "?");
            continue;
        }
        CHECK(result.value && result.value->certified.triangles.size() > 0);
        const bool independent =
            result.value->modeling.provenance ==
            weft::ModelingProvenanceKind::Independent;
        std::size_t quads = 0;
        if (independent) {
            for (const auto& poly : result.value->modeling.polygons) {
                if (poly.vertices.size() == 4) ++quads;
            }
        }
        // Seam unmatched must not appear as a silent soft success — either
        // the lattice certified or UV-trim consumed the refuse.
        const bool seamSoft = std::any_of(
            result.validation.checks.begin(), result.validation.checks.end(),
            [](const weft::ValidationCoverage& coverage) {
                return coverage.code.find("mapped.seam_sample_unmatched") !=
                    std::string::npos &&
                    coverage.failed == 0 && coverage.skipped > 0;
            });
        CHECK(!seamSoft);
        std::printf(
            "WEFT_G3_EXTRACT %s tris=%zu independent=%d quads=%zu "
            "mapped=%d uv_grid=%d uv_trim=%d\n",
            name, result.value->certified.triangles.size(),
            independent ? 1 : 0, quads, sawMapped ? 1 : 0, sawUvGrid ? 1 : 0,
            sawUvTrim ? 1 : 0);
        if (std::string(name) == "freeform_pent.step") {
            CHECK(independent);
            CHECK(quads > 0);
        }
        if (std::string(name) == "freeform_hex.step") {
            CHECK(sawUvTrim);
        }
        if (std::string(name) == "freeform_135.step") {
            CHECK(sawUvTrim);
            CHECK(independent);
            CHECK(quads > 0);
        }
        if (std::string(name) == "freeform_137.step" ||
            std::string(name) == "freeform_138.step") {
            CHECK(sawUvTrim);
            CHECK(independent);
            CHECK(quads > 0);
        }
        if (std::string(name) == "freeform_186.step") {
            CHECK(sawUvTrim);
            CHECK(independent);
            CHECK(quads > 0);
        }
        if (std::string(name) == "freeform_399.step") {
            CHECK(sawUvTrim);
            CHECK(independent);
            CHECK(quads > 0);
        }
    }
}

int meshExtractOrNamedRefuse(const char* name, const char* marker) {
    const std::filesystem::path path = findMp9Extract(name);
    CHECK(!path.empty());
    if (path.empty()) return 0;
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    CHECK(imported.repair.meshable);
    if (!imported.repair.meshable) {
        std::printf("%s %s meshable=0\n", marker, name);
        return 0;
    }
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    if (!result) {
        CHECK(result.failure);
        const std::string& code =
            result.failure ? result.failure->code : std::string();
        const bool named =
            code.rfind("mapped.", 0) == 0 || code.rfind("freeform.", 0) == 0 ||
            code.rfind("secure_pipeline.", 0) == 0 ||
            code.rfind("extrusion.", 0) == 0 || code.rfind("offset.", 0) == 0 ||
            code.rfind("revolution.", 0) == 0 ||
            code.rfind("trim_assembly.", 0) == 0;
        CHECK(named);
        std::printf("%s %s named_refuse=%s\n", marker, name, code.c_str());
        return named ? 1 : 0;
    }
    CHECK(result.value && result.value->certified.triangles.size() > 0);
    CHECK(result.validation.complete());
    const bool residualSoft = std::any_of(
        result.validation.checks.begin(), result.validation.checks.end(),
        [](const weft::ValidationCoverage& coverage) {
            return coverage.code.find("uv_trim_orientation") !=
                       std::string::npos &&
                   coverage.failed == 0 && coverage.skipped > 0;
        });
    CHECK(!residualSoft);
    CHECK(!hasCoverage(result.validation,
                       "secure_pipeline.preview_fast_boundaries"));
    CHECK(!hasCoverage(result.validation,
                       "secure_pipeline.relaxed_geometry_on_product_path"));
    std::printf("%s %s tris=%zu complete=1\n", marker, name,
                result.value->certified.triangles.size());
    return 1;
}

void testExtrusionOffsetFixtures() {
    // Wave D: extrusion_quad fixture + offset_quad extract share UV/mapped.
    {
        const std::filesystem::path path =
            weft::test::uniqueTempPath("weft_extrusion_quad", ".step");
        weft::writeStep(weft::makeFixture("extrusion_quad"), path.string());
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
        bool sawExtrusion = false;
        bool sawMappedOrUv = false;
        for (const weft::ExactGeometryClassification& record : recon.records) {
            if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
            if (record.familyCode == "extrusion") sawExtrusion = true;
            for (const std::string& code : record.conditionCodes) {
                if (code == "mapped.four_sided_candidate" ||
                    code == "freeform.uv_trim_candidate") {
                    sawMappedOrUv = true;
                }
            }
        }
        CHECK(sawExtrusion);
        CHECK(sawMappedOrUv);
        const weft::SecureMeshingResult result =
            weft::generateSecureMesh(imported, configuration());
        CHECK(result);
        if (result) {
            CHECK(result.value && result.value->certified.triangles.size() > 0);
            std::printf("WEFT_MAPPED_MATRIX extrusion_quad tris=%zu\n",
                        result.value->certified.triangles.size());
        } else {
            std::printf("WEFT_MAPPED_MATRIX extrusion_quad fail=%s\n",
                        result.failure ? result.failure->code.c_str() : "?");
        }
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }
    CHECK(meshExtractOrNamedRefuse("offset_quad.step",
                                   "WEFT_MAPPED_MATRIX") > 0);
}

void testRevolutionNgonUvPromote() {
    // Wave D gap close: revolution n≠4 gets freeform.uv_trim_candidate and
    // hard-certifies (or named-refuses) — never Deferred-only.
    const std::filesystem::path path =
        weft::test::uniqueTempPath("weft_revolution_ngon", ".step");
    weft::writeStep(weft::makeFixture("revolution_ngon"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport recon = weft::reconnoitre(imported);
    bool sawRevolution = false;
    bool sawUvTrim = false;
    bool sawAttempted = false;
    bool supported = false;
    for (const weft::ExactGeometryClassification& record : recon.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
        if (record.familyCode != "revolution") continue;
        sawRevolution = true;
        if (record.support ==
            weft::GeometrySupportState::SupportedAnalyticTemplate) {
            supported = true;
        }
        for (const std::string& code : record.conditionCodes) {
            if (code == "freeform.uv_trim_candidate") sawUvTrim = true;
            if (code == "revolution.uv_trim_attempted") sawAttempted = true;
        }
    }
    CHECK(sawRevolution);
    CHECK(sawUvTrim);
    CHECK(sawAttempted);
    CHECK(supported);
    const weft::SecureMeshingResult result =
        weft::generateSecureMesh(imported, configuration());
    if (!result) {
        CHECK(result.failure);
        const std::string& code =
            result.failure ? result.failure->code : std::string();
        const bool named = code.rfind("mapped.", 0) == 0 ||
                           code.rfind("freeform.", 0) == 0 ||
                           code.rfind("revolution.", 0) == 0 ||
                           code.rfind("trim_assembly.", 0) == 0 ||
                           code.rfind("secure_pipeline.", 0) == 0;
        CHECK(named);
        std::printf(
            "WEFT_FREEFORM_MATRIX revolution_ngon named_refuse=%s "
            "uv_promote=1\n",
            code.c_str());
    } else {
        CHECK(result.value && result.value->certified.triangles.size() > 0);
        std::printf(
            "WEFT_FREEFORM_MATRIX revolution_ngon tris=%zu uv_promote=1\n",
            result.value->certified.triangles.size());
    }
    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testMappedFreeformMatrix() {
    // Wave D lock: general mapped/UV orient on existing extracts — no
    // face-id special cases. Prints WEFT_MAPPED_MATRIX + WEFT_FREEFORM_MATRIX.
    const char* mappedMarker = "WEFT_MAPPED_MATRIX";
    const char* freeformMarker = "WEFT_FREEFORM_MATRIX";
    const char* orientExtracts[] = {"face_33_orient.step",
                                    "face_103_orient.step",
                                    "face_107_orient.step"};
    const char* freeformExtracts[] = {
        "freeform_pent.step", "freeform_hex.step",
        "freeform_mapped_fallback.step", "bspline_139.step",
        "offset_quad.step", "crash_face_136_r0.step",
        "freeform_135.step", "freeform_137.step", "freeform_138.step",
        "freeform_186.step", "freeform_399.step", "freeform_553.step"};
    std::size_t mappedLocked = 0;
    for (const char* name : orientExtracts) {
        if (meshExtractOrNamedRefuse(name, mappedMarker) > 0) ++mappedLocked;
    }
    CHECK(mappedLocked == 3);
    std::size_t freeformLocked = 0;
    for (const char* name : freeformExtracts) {
        if (meshExtractOrNamedRefuse(name, freeformMarker) > 0) {
            ++freeformLocked;
        }
    }
    CHECK(freeformLocked == 12);
    testExtrusionOffsetFixtures();
    testRevolutionNgonUvPromote();
    std::printf(
        "%s locked=%zu/3 general_orient=1\n"
        "%s locked=%zu/12 +extrusion_quad +revolution_ngon "
        "revolution_uv_promote=1 periodic_band_pent=freeform_135 "
        "periodic_band_hex_period1=freeform_137 "
        "periodic_band_pent_period1=freeform_138 "
        "v_periodic_tiny_period=freeform_186 "
        "u_periodic_circular_caps=freeform_399 "
        "convex_simple_iso_lattice=freeform_553\n",
        mappedMarker, mappedLocked, freeformMarker, freeformLocked);
}

}  // namespace

int main() {
    try {
        testFreeformUvGridBody();
        testRibbonMappedBody();
        testG3Mp9CrashFace136NoAv();
        testG3Face33Orientation();
        testG3Face103MappedDensify();
        testG3Face107HardOrient();
        testG3Mp9FreeformExtracts();
        testMappedFreeformMatrix();
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
