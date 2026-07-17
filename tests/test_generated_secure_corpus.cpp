#include "fixture_generator.hpp"

#include "weft/secure_core.hpp"
#include "weft/secure_reconnaissance.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <set>
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
                if (path.filename() ==
                    "assembly.box.nested_repeated_instances.step") {
                    const auto& assembly = imported.source->snapshot.model.assembly;
                    const std::size_t resolvedLeaves =
                        static_cast<std::size_t>(std::count_if(
                            assembly.begin(), assembly.end(),
                            [](const weft::AssemblyNode& node) {
                                return node.solidId > 0;
                            }));
                    std::set<int> resolvedSolidIds;
                    for (const weft::AssemblyNode& node : assembly) {
                        if (node.solidId > 0) {
                            resolvedSolidIds.insert(node.solidId);
                        }
                    }
                    std::printf(
                        "nested assembly adapter: %zu expanded nodes, %zu resolved leaves, %zu distinct leaf IDs, %d indexed solids\n",
                        assembly.size(), resolvedLeaves, resolvedSolidIds.size(),
                        imported.source->snapshot.model.solids.Extent());
                    CHECK(assembly.size() == 9);
                    CHECK(resolvedLeaves == 6);
                    CHECK(resolvedSolidIds.size() == 6);
                    if (assembly.size() == 9) {
                        CHECK(assembly[0].parent == -1);
                        CHECK(assembly[0].isAssembly);
                        CHECK(assembly[0].sourceComponent.empty());
                        CHECK(!assembly[0].sourceDefinition.empty());
                        CHECK(assembly[1].parent == 0);
                        CHECK(assembly[5].parent == 0);
                        CHECK(assembly[1].isAssembly);
                        CHECK(assembly[5].isAssembly);
                        CHECK(assembly[1].sourceDefinition ==
                              assembly[5].sourceDefinition);
                        CHECK(assembly[1].sourceComponent !=
                              assembly[5].sourceComponent);
                        CHECK(assembly[2].parent == 1);
                        CHECK(assembly[3].parent == 1);
                        CHECK(assembly[4].parent == 1);
                        CHECK(assembly[6].parent == 5);
                        CHECK(assembly[7].parent == 5);
                        CHECK(assembly[8].parent == 5);
                        CHECK(assembly[2].sourceDefinition ==
                              assembly[8].sourceDefinition);
                        CHECK(assembly[2].sourceComponent ==
                              assembly[6].sourceComponent);
                        CHECK(assembly[3].sourceComponent ==
                              assembly[7].sourceComponent);
                        CHECK(assembly[4].sourceComponent ==
                              assembly[8].sourceComponent);
                        CHECK(assembly[1].localTransform[13] == 10.0);
                        CHECK(assembly[5].localTransform[12] == 30.0);
                        for (const weft::AssemblyNode& node : assembly) {
                            CHECK(!node.sourceDefinition.empty());
                            CHECK(node.localTransform[15] == 1.0);
                            CHECK(node.transform[15] == 1.0);
                            if (!node.isAssembly) {
                                CHECK(node.solidIds.size() == 1);
                                CHECK(node.solidId == node.solidIds.front());
                            }
                        }
                    }
                }
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
