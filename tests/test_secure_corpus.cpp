#include "weft/fixture.hpp"
#include "weft/secure_core.hpp"
#include "weft/secure_meshing.hpp"

#include "test_temp_path.hpp"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#ifndef WEFT_SECURE_FIXTURE_DIR
#error "WEFT_SECURE_FIXTURE_DIR must identify the frozen fixture snapshot"
#endif

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

struct CorpusCase {
    const char* relativePath;
    bool shouldImport;
    const char* expectedFailure;
};

void verifySuccess(const std::filesystem::path& path) {
    const weft::ImportedModel first = weft::importStepSecure(path.string());
    const weft::ImportedModel second = weft::importStepSecure(path.string());
    CHECK(first.source != nullptr);
    CHECK(first.working != nullptr);
    CHECK(first.repair.identity);
    CHECK(first.repair.correspondenceComplete);
    CHECK(first.repair.sourceFaces > 0);
    CHECK(first.repair.sourceEdges > 0);
    CHECK(first.source && first.source->metadata.sourceSha256.size() == 64);
    CHECK(first.repair.sourceShapeSha256.size() == 64);
    CHECK(first.repair.workingShapeSha256.size() == 64);
    CHECK(first.source->metadata.sourceSha256 ==
          second.source->metadata.sourceSha256);
    CHECK(first.repair.sourceShapeSha256 ==
          second.repair.sourceShapeSha256);
    CHECK(first.repair.workingShapeSha256 ==
          second.repair.workingShapeSha256);
    CHECK(first.repair.sourceFaces == second.repair.sourceFaces);
    CHECK(first.repair.sourceEdges == second.repair.sourceEdges);
    CHECK(first.meshable() == second.meshable());
    CHECK(first.repair.sourceValid == second.repair.sourceValid);
    CHECK(first.repair.workingValid == second.repair.workingValid);
    if (!first.meshable()) {
        const auto hasDiagnostic = [&](const std::string& code) {
            for (const weft::ImportDiagnostic& diagnostic :
                 first.diagnostics.events) {
                if (diagnostic.code == code) return true;
            }
            return false;
        };
        CHECK(hasDiagnostic("import.source.invalid"));
        CHECK(hasDiagnostic("import.working.invalid"));
        CHECK(hasDiagnostic("import.working.non_meshable"));
    }
}

void verifyFailure(const std::filesystem::path& path,
                   const std::string& expectedCode) {
    for (int attempt = 0; attempt < 2; ++attempt) {
        try {
            (void)weft::importStepSecure(path.string());
            std::printf("unexpected import success: %s\n",
                        path.string().c_str());
            ++failures;
        } catch (const weft::SecureImportError& error) {
            if (error.code() != expectedCode) {
                std::printf("wrong refusal for %s: expected %s, got %s (%s)\n",
                            path.string().c_str(), expectedCode.c_str(),
                            error.code().c_str(), error.what());
                ++failures;
            }
        }
    }
}

}  // namespace

int main() {
    const std::filesystem::path root(WEFT_SECURE_FIXTURE_DIR);
    const std::vector<CorpusCase> cases = {
        {"baselines/solid.box.step", true, ""},
        {"derived/assembly.nist_ctc_01.mirrored_mapped_item.step", false,
         "import.transform.unresolved_representation_loss"},
        {"derived/assembly.nist_ctc_01.scaled_mapped_item.step", false,
         "import.transform.unresolved_representation_loss"},
        {"derived/corrupt.step.syntax_failure.step", false,
         "import.step.transfer_failed"},
        {"external/freecad_as1_autocad2000_ap214.step", true, ""},
        {"external/nist_ctc_01_ap242_e1.step", true, ""},
        {"external/plasticity26_startup_analytic.step", true, ""},
        {"external/plasticity26_startup_analytic_214.step", true, ""},
        {"external/plasticity26_startup_analytic_242.step", true, ""},
    };

    CHECK(std::filesystem::is_directory(root));
    std::size_t importSuccess = 0;
    std::size_t importRefusal = 0;
    std::size_t meshCertified = 0;
    std::size_t meshNamedRefusal = 0;
    std::size_t meshInspectableOnly = 0;

    for (const CorpusCase& corpusCase : cases) {
        const std::filesystem::path path = root / corpusCase.relativePath;
        CHECK(std::filesystem::is_regular_file(path));
        if (!std::filesystem::is_regular_file(path)) continue;
        if (corpusCase.shouldImport) {
            // Preserve the original import contract checks for meshable
            // subjects; inspectable-only imports are classified without
            // requiring a specific diagnostic vocabulary.
            const weft::ImportedModel imported =
                weft::importStepSecure(path.string());
            CHECK(imported.source != nullptr);
            CHECK(imported.working != nullptr);
            ++importSuccess;
            if (!imported.meshable()) {
                ++meshInspectableOnly;
                std::printf(
                    "COVERAGE subject=%s terminal=inspectable_only\n",
                    corpusCase.relativePath);
                continue;
            }
            verifySuccess(path);
            const weft::SecureMeshingResult meshed =
                weft::generateSecureMesh(imported);
            if (meshed && meshed.value &&
                meshed.value->validation.complete()) {
                ++meshCertified;
                std::printf(
                    "COVERAGE subject=%s terminal=certified "
                    "fingerprint=%s\n",
                    corpusCase.relativePath,
                    meshed.value->certified.topologyFingerprint.c_str());
            } else {
                ++meshNamedRefusal;
                const std::string code = meshed.failure
                    ? meshed.failure->code
                    : "secure_pipeline.unknown_refusal";
                std::printf(
                    "COVERAGE subject=%s terminal=named_refusal code=%s\n",
                    corpusCase.relativePath, code.c_str());
                CHECK(meshed.failure.has_value());
            }
        } else {
            verifyFailure(path, corpusCase.expectedFailure);
            ++importRefusal;
            std::printf(
                "COVERAGE subject=%s terminal=import_refusal code=%s\n",
                corpusCase.relativePath, corpusCase.expectedFailure);
        }
    }

    const char* fixtures[] = {"box", "cylinder", "partial_cylinder", "hole",
                              "cone", "sphere"};
    constexpr std::size_t kFixtureExtra =
        sizeof(fixtures) / sizeof(fixtures[0]);
    for (const char* fixture : fixtures) {
        const std::filesystem::path path = weft::test::uniqueTempPath(
            std::string("weft_coverage_") + fixture, ".step");
        weft::writeStep(weft::makeFixture(fixture), path.string());
        const weft::ImportedModel imported =
            weft::importStepSecure(path.string());
        ++importSuccess;
        weft::SecureMeshingResult meshed;
        if (std::string(fixture) == "cone" || std::string(fixture) == "sphere") {
            weft::SecureMeshingConfiguration settings;
            settings.sampling.chordTolerance = 0.25;
            settings.sampling.normalAngleToleranceRadians = 0.35;
            settings.sampling.minimumClosedCurveSegments =
                std::string(fixture) == "sphere" ? 16 : 8;
            meshed = weft::generateSecureMesh(imported, settings);
        } else {
            meshed = weft::generateSecureMesh(imported);
        }
        if (meshed && meshed.value && meshed.value->validation.complete()) {
            ++meshCertified;
            std::printf(
                "COVERAGE subject=fixture:%s terminal=certified "
                "fingerprint=%s\n",
                fixture,
                meshed.value->certified.topologyFingerprint.c_str());
        } else {
            ++meshNamedRefusal;
            const std::string code = meshed.failure
                ? meshed.failure->code
                : "secure_pipeline.unknown_refusal";
            std::printf(
                "COVERAGE subject=fixture:%s terminal=named_refusal code=%s\n",
                fixture, code.c_str());
            CHECK(meshed.failure.has_value());
        }
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }

    const std::size_t caseCount = cases.size();
    std::printf(
        "COVERAGE_TOTALS import_success=%zu import_refusal=%zu "
        "mesh_certified=%zu mesh_named_refusal=%zu mesh_inspectable_only=%zu "
        "case_total=%zu fixture_extra=%zu\n",
        importSuccess, importRefusal, meshCertified, meshNamedRefusal,
        meshInspectableOnly, caseCount, kFixtureExtra);
    CHECK(importRefusal + (importSuccess - kFixtureExtra) == caseCount);
    CHECK(meshCertified + meshNamedRefusal + meshInspectableOnly ==
          importSuccess);

    if (failures == 0) {
        std::printf(
            "secure committed STEP corpus checks passed (%zu import success "
            "incl fixtures, %zu import refusal); coverage baseline classified\n",
            importSuccess, importRefusal);
        return EXIT_SUCCESS;
    }
    std::printf("%d secure committed STEP corpus failure(s)\n", failures);
    return EXIT_FAILURE;
}
