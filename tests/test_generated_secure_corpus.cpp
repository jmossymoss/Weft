#include "fixture_generator.hpp"

#include "weft/secure_core.hpp"
#include "weft/secure_reconnaissance.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

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

class TemporaryCorpus {
public:
    TemporaryCorpus() {
        path_ = weft::test::uniqueTempPath(
            "weft_generated_secure_corpus");
        if (!std::filesystem::create_directory(path_)) {
            throw std::runtime_error("failed to create temporary corpus");
        }
    }

    ~TemporaryCorpus() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

}  // namespace

int main() {
    try {
        TemporaryCorpus corpus;
        cad_mesher::fixtures::GenerationFailure generationFailure;
        const bool generated = cad_mesher::fixtures::generate_all(
            corpus.path(), generationFailure);
        if (!generated) {
            std::printf("fixture generation failed: %s: %s\n",
                        generationFailure.code.c_str(),
                        generationFailure.message.c_str());
            return EXIT_FAILURE;
        }

        std::vector<std::filesystem::path> stepFiles;
        for (const auto& entry :
             std::filesystem::recursive_directory_iterator(corpus.path())) {
            if (entry.is_regular_file() && entry.path().extension() == ".step") {
                stepFiles.push_back(entry.path());
            }
        }
        std::sort(stepFiles.begin(), stepFiles.end());
        CHECK(cad_mesher::fixtures::fixture_ids().size() == 77);
        CHECK(stepFiles.size() == 77);

        std::size_t meshable = 0;
        std::size_t inspectableOnly = 0;
        std::size_t classifiedSubjects = 0;
        for (const std::filesystem::path& path : stepFiles) {
            try {
                const weft::ImportedModel imported =
                    weft::importStepSecure(path.string());
                CHECK(imported.source != nullptr);
                CHECK(imported.working != nullptr);
                CHECK(imported.repair.identity);
                CHECK(imported.repair.correspondenceComplete);
                CHECK(imported.source &&
                      imported.source->metadata.sourceSha256.size() == 64);
                CHECK(imported.repair.sourceShapeSha256.size() == 64);
                CHECK(imported.repair.workingShapeSha256.size() == 64);
                const weft::ReconnaissanceReport reconnaissance =
                    weft::reconnoitre(imported);
                CHECK(reconnaissance.complete);
                CHECK(reconnaissance.expectedSubjects ==
                      reconnaissance.checkedSubjects);
                classifiedSubjects += reconnaissance.checkedSubjects;
                if (imported.meshable()) {
                    ++meshable;
                } else {
                    ++inspectableOnly;
                    CHECK(imported.diagnostics.hasErrors());
                }
            } catch (const weft::SecureImportError& error) {
                std::printf("generated fixture refused: %s: %s (%s)\n",
                            path.filename().string().c_str(),
                            error.code().c_str(), error.what());
                ++failures;
            }
        }
        CHECK(meshable + inspectableOnly == 77);
        CHECK(classifiedSubjects > 0);
        std::printf("generated secure corpus: 77 imported, %zu meshable, "
                    "%zu inspectable-only, %zu classified subjects\n",
                    meshable, inspectableOnly, classifiedSubjects);
    } catch (const std::exception& error) {
        std::printf("generated secure corpus exception: %s\n", error.what());
        ++failures;
    }

    if (failures == 0) return EXIT_SUCCESS;
    std::printf("%d generated secure corpus failure(s)\n", failures);
    return EXIT_FAILURE;
}
