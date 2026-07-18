#include "weft/secure_core.hpp"

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
    for (const CorpusCase& corpusCase : cases) {
        const std::filesystem::path path = root / corpusCase.relativePath;
        CHECK(std::filesystem::is_regular_file(path));
        if (!std::filesystem::is_regular_file(path)) continue;
        if (corpusCase.shouldImport) {
            verifySuccess(path);
        } else {
            verifyFailure(path, corpusCase.expectedFailure);
        }
    }

    if (failures == 0) {
        std::printf("secure committed STEP corpus checks passed (6 success, 3 refusal)\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d secure committed STEP corpus failure(s)\n", failures);
    return EXIT_FAILURE;
}
