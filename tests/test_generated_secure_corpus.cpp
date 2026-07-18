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
        std::size_t namedImportRefusals = 0;
        std::size_t classifiedSubjects = 0;
        for (const std::filesystem::path& path : stepFiles) {
            try {
                const weft::ImportedModel imported =
                    weft::importStepSecure(path.string());
                CHECK(imported.source != nullptr);
                CHECK(imported.working != nullptr);
                const bool certifiedToleranceOnly =
                    !imported.repair.identity &&
                    imported.repair.correspondenceComplete &&
                    imported.repair.workingValid &&
                    imported.repair.meshable &&
                    imported.repair.sourceShapeSha256 !=
                        imported.repair.workingShapeSha256 &&
                    !imported.repair.toleranceChanges.empty() &&
                    imported.repair.parameterizationFlagChanges.empty() &&
                    imported.repair.orientationChanges.empty() &&
                    imported.repair.representationChanges.empty() &&
                    imported.repair.topologyCardinalityChanges.empty() &&
                    std::all_of(
                        imported.repair.toleranceChanges.begin(),
                        imported.repair.toleranceChanges.end(),
                        [](const weft::ToleranceChange& change) {
                            return change.expectedPcurveUses != 0 &&
                                change.checkedPcurveUses ==
                                    change.expectedPcurveUses &&
                                change.after > change.before &&
                                change.maximumDiscrepancy == change.after;
                        }) &&
                    std::any_of(
                        imported.repair.operations.begin(),
                        imported.repair.operations.end(),
                        [](const weft::RepairOperation& operation) {
                            return operation.code ==
                                "repair.tolerance_envelope";
                        });
                CHECK(imported.repair.identity || certifiedToleranceOnly);
                CHECK(imported.repair.correspondenceComplete);
                CHECK(imported.source &&
                      imported.source->metadata.sourceSha256.size() == 64);
                CHECK(imported.repair.sourceShapeSha256.size() == 64);
                CHECK(imported.repair.workingShapeSha256.size() == 64);
                if (imported.repair.identity) {
                    CHECK(imported.repair.sourceShapeSha256 ==
                          imported.repair.workingShapeSha256);
                    CHECK(imported.repair.toleranceChanges.empty());
                    CHECK(imported.repair.representationChanges.empty());
                    CHECK(imported.repair.topologyCardinalityChanges.empty());
                } else {
                    CHECK(certifiedToleranceOnly);
                }
                if (imported.source && imported.working) {
                    CHECK(!imported.source->snapshot.model.shape.IsPartner(
                        imported.working->snapshot.model.shape));
                    for (const auto& [sourceId, sourceShape] :
                         imported.source->snapshot.topology.exactShapes) {
                        const auto workingShape =
                            imported.working->snapshot.topology.exactShapes.find(
                                sourceId);
                        CHECK(workingShape != imported.working->snapshot.topology
                                                  .exactShapes.end());
                        if (workingShape != imported.working->snapshot.topology
                                                .exactShapes.end()) {
                            CHECK(!sourceShape.IsPartner(workingShape->second));
                        }
                    }
                }
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
                                CHECK(!node.exactUse.IsNull());
                                CHECK(node.solidIds.size() == 1);
                                CHECK(node.solidId == node.solidIds.front());
                            }
                        }
                    }

                    const weft::TopologyAccount& topology =
                        imported.source->snapshot.topology;
                    const weft::TopologyAccountValidation topologyValidation =
                        weft::validateTopologyAccount(topology);
                    CHECK(topologyValidation.complete());
                    const auto occurrenceCount =
                        [&](weft::StableIdKind kind) {
                            return static_cast<std::size_t>(std::count_if(
                                topology.occurrences.begin(),
                                topology.occurrences.end(),
                                [kind](const weft::TopologyOccurrence& occurrence) {
                                    return occurrence.id.kind == kind;
                                }));
                        };
                    const auto uniqueCount = [&](weft::StableIdKind kind) {
                        return static_cast<std::size_t>(std::count_if(
                            topology.uniqueEntityIds.begin(),
                            topology.uniqueEntityIds.end(),
                            [kind](weft::StableId entity) {
                                return entity.kind == kind;
                            }));
                    };
                    CHECK(topology.assemblies.size() == 2);
                    CHECK(topology.instances.size() == 8);
                    CHECK(topology.assemblyRoots.size() == 1);
                    CHECK(topology.topologyRoots.size() == 1);
                    CHECK(occurrenceCount(weft::StableIdKind::Compound) == 1);
                    CHECK(occurrenceCount(weft::StableIdKind::Solid) == 6);
                    CHECK(occurrenceCount(weft::StableIdKind::Shell) == 6);
                    CHECK(occurrenceCount(weft::StableIdKind::Face) == 36);
                    CHECK(occurrenceCount(weft::StableIdKind::Wire) == 36);
                    CHECK(occurrenceCount(weft::StableIdKind::Edge) == 144);
                    CHECK(occurrenceCount(weft::StableIdKind::Vertex) == 288);
                    CHECK(topology.coedges.size() == 144);
                    CHECK(uniqueCount(weft::StableIdKind::Compound) == 1);
                    CHECK(uniqueCount(weft::StableIdKind::Solid) == 1);
                    CHECK(uniqueCount(weft::StableIdKind::Shell) == 1);
                    CHECK(uniqueCount(weft::StableIdKind::Face) == 6);
                    CHECK(uniqueCount(weft::StableIdKind::Wire) == 6);
                    CHECK(uniqueCount(weft::StableIdKind::Edge) == 12);
                    CHECK(uniqueCount(weft::StableIdKind::Vertex) == 8);
                    CHECK(topology.exactShapes.size() ==
                          topology.occurrences.size());
                    CHECK(imported.correspondence.topologyOccurrenceRecords.size() ==
                          topology.occurrences.size());
                    CHECK(std::all_of(
                        imported.correspondence.topologyOccurrenceRecords.begin(),
                        imported.correspondence.topologyOccurrenceRecords.end(),
                        [](const weft::CorrespondenceRecord& record) {
                            return record.sourceId.valid() &&
                                record.workingIds.size() == 1 &&
                                record.relation ==
                                    weft::CorrespondenceRelation::Identity;
                        }));

                    weft::TopologyAccount missingInstanceRoots = topology;
                    const auto leaf = std::find_if(
                        missingInstanceRoots.instances.begin(),
                        missingInstanceRoots.instances.end(),
                        [](const weft::InstanceRecord& instance) {
                            return !instance.targetAssembly &&
                                !instance.topologyRoots.empty();
                        });
                    CHECK(leaf != missingInstanceRoots.instances.end());
                    if (leaf != missingInstanceRoots.instances.end()) {
                        leaf->topologyRoots.clear();
                        const auto tampered = weft::validateTopologyAccount(
                            missingInstanceRoots);
                        CHECK(!tampered.complete());
                        CHECK(std::find(
                                  tampered.failureCodes.begin(),
                                  tampered.failureCodes.end(),
                                  "topology.instance.topology_roots_missing") !=
                              tampered.failureCodes.end());
                    }
                }
                if (path.filename() ==
                    "solid.fillet.junction_t_y_x.step") {
                    const weft::TopologyAccount& topology =
                        imported.source->snapshot.topology;
                    CHECK(weft::validateTopologyAccount(topology).complete());
                    std::set<std::string> nonBodyDefinitions;
                    for (const weft::AssemblyNode& node :
                         imported.source->snapshot.model.assembly) {
                        if (!node.isAssembly && node.solidIds.empty()) {
                            nonBodyDefinitions.insert(node.sourceDefinition);
                        }
                    }
                    CHECK(!nonBodyDefinitions.empty());
                    bool sawNonBodyLeafRoot = false;
                    for (const weft::InstanceRecord& instance :
                         topology.instances) {
                        if (instance.targetAssembly) continue;
                        CHECK(!instance.topologyRoots.empty());
                        if (nonBodyDefinitions.contains(
                                instance.sourceDefinition) &&
                            !instance.topologyRoots.empty()) {
                            sawNonBodyLeafRoot = true;
                        }
                        for (weft::StableId root : instance.topologyRoots) {
                            const auto occurrence = std::find_if(
                                topology.occurrences.begin(),
                                topology.occurrences.end(),
                                [root](const weft::TopologyOccurrence& item) {
                                    return item.id == root;
                                });
                            CHECK(occurrence != topology.occurrences.end());
                        }
                    }
                    CHECK(sawNonBodyLeafRoot);
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
                // OCCT version differences may refuse a fixture during
                // processing-disabled transfer. Count only stable named
                // import.* codes so the 77-fixture total still reconciles.
                CHECK(error.code().rfind("import.", 0) == 0);
                if (error.code().rfind("import.", 0) != 0) {
                    std::printf(
                        "generated fixture refused without import.* code: "
                        "%s: %s (%s)\n",
                        path.filename().string().c_str(),
                        error.code().c_str(), error.what());
                    ++failures;
                    continue;
                }
                ++namedImportRefusals;
            }
        }
        CHECK(meshable + inspectableOnly + namedImportRefusals == 77);
        CHECK(classifiedSubjects > 0);
        std::printf(
            "generated secure corpus: %zu meshable, %zu inspectable-only, "
            "%zu named-import-refusals, %zu classified subjects\n",
            meshable, inspectableOnly, namedImportRefusals,
            classifiedSubjects);
    } catch (const std::exception& error) {
        std::printf("generated secure corpus exception: %s\n", error.what());
        ++failures;
    }

    if (failures == 0) return EXIT_SUCCESS;
    std::printf("%d generated secure corpus failure(s)\n", failures);
    return EXIT_FAILURE;
}
