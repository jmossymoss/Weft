#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/secure_recipe.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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

class TemporaryFiles {
public:
    TemporaryFiles()
        : step_(weft::test::uniqueTempPath(
              "weft_secure_recipe_box", ".step")),
          recipe_(weft::test::uniqueTempPath(
              "weft_secure_recipe_roundtrip", ".recipe")),
          malformed_(weft::test::uniqueTempPath(
              "weft_secure_recipe_malformed", ".recipe")) {
        weft::writeStep(weft::makeFixture("box"), step_.string());
    }

    ~TemporaryFiles() {
        std::error_code ignored;
        std::filesystem::remove(step_, ignored);
        std::filesystem::remove(recipe_, ignored);
        std::filesystem::remove(malformed_, ignored);
    }

    const std::filesystem::path& step() const noexcept { return step_; }
    const std::filesystem::path& recipe() const noexcept { return recipe_; }
    const std::filesystem::path& malformed() const noexcept {
        return malformed_;
    }

private:
    std::filesystem::path step_;
    std::filesystem::path recipe_;
    std::filesystem::path malformed_;
};

bool hasIssue(const std::vector<weft::RecipeMigrationIssue>& issues,
              const std::string& code,
              weft::RecipeIssueSeverity severity) {
    return std::any_of(
        issues.begin(), issues.end(), [&](const auto& issue) {
            return issue.code == code && issue.severity == severity;
        });
}

weft::Recipe makeLegacyRecipe() {
    weft::Recipe legacy;
    legacy.pipeline = weft::MeshPipeline::PrimitiveCompiler;
    legacy.settings.defaults.radial = 20;
    legacy.settings.defaults.chordTolerance = 0.12345678901234566;
    legacy.settings.defaults.cellCap = 4321;
    weft::FaceMeshSettings face = legacy.settings.defaults;
    face.radial = 24;
    face.cellCap = 7890;
    legacy.settings.perFace[1] = face;
    legacy.settings.perEdge[1] = 7;
    weft::ManualOp operation;
    operation.kind = weft::ManualOp::Kind::LoopInsert;
    operation.faceId = 1;
    operation.u = 0.25;
    operation.v = 0.75;
    operation.t = 0.4;
    legacy.ops.push_back(operation);
    return legacy;
}

void testReferencesMigrationResolutionAndPersistence(
    const weft::ImportedModel& imported,
    const std::filesystem::path& recipePath) {
    const weft::SourceEntityReference face =
        weft::makeSourceEntityReference(
            imported, {weft::StableIdKind::Face, 1});
    const weft::SourceEntityReference edge =
        weft::makeSourceEntityReference(
            imported, {weft::StableIdKind::Edge, 1});
    CHECK(face.valid());
    CHECK(edge.valid());
    CHECK(face.geometricFingerprint.rfind("face-v1|", 0) == 0);
    CHECK(edge.geometricFingerprint.rfind("edge-v1|", 0) == 0);

    const weft::RecipeV2MigrationResult migrated =
        weft::migrateRecipeV1(imported, makeLegacyRecipe());
    CHECK(migrated.complete());
    CHECK(hasIssue(migrated.issues, "recipe_v1.pipeline_ignored",
                   weft::RecipeIssueSeverity::Warning));
    CHECK(migrated.recipe.sourceSha256.size() == 64);
    CHECK(migrated.recipe.faceSettings.size() == 1);
    CHECK(migrated.recipe.edgeSettings.size() == 1);
    CHECK(migrated.recipe.operations.size() == 1);
    CHECK(migrated.recipe.faceSettings.front().face.geometricFingerprint ==
          face.geometricFingerprint);
    CHECK(migrated.recipe.edgeSettings.front().edge.geometricFingerprint ==
          edge.geometricFingerprint);

    const weft::RecipeV2Resolution resolved =
        weft::resolveRecipeV2(imported, migrated.recipe);
    CHECK(resolved.complete());
    CHECK(resolved.settings.defaults.radial == 20);
    CHECK(resolved.settings.perFace.size() == 1);
    CHECK(resolved.settings.perFace.at(1).radial == 24);
    CHECK(resolved.settings.perEdge.at(1) == 7);
    CHECK(resolved.operations.size() == 1);
    CHECK(resolved.operations.front().faceId == 1);
    CHECK(resolved.operations.front().kind ==
          weft::ManualOp::Kind::LoopInsert);
    const std::vector<weft::RecipeMigrationIssue> applicationIssues =
        weft::validateSecureRecipeApplication(resolved);
    CHECK(hasIssue(applicationIssues,
                   "secure_recipe.application.face_settings_unimplemented",
                   weft::RecipeIssueSeverity::Conflict));
    CHECK(hasIssue(applicationIssues,
                   "secure_recipe.application.edge_settings_unimplemented",
                   weft::RecipeIssueSeverity::Conflict));
    CHECK(hasIssue(applicationIssues,
                   "secure_recipe.application.operations_unimplemented",
                   weft::RecipeIssueSeverity::Conflict));

    weft::saveRecipeV2(migrated.recipe, recipePath.string());
    const weft::RecipeV2 loaded =
        weft::loadRecipeV2(recipePath.string());
    CHECK(loaded.sourceSha256 == migrated.recipe.sourceSha256);
    CHECK(loaded.defaults.radial == 20);
    CHECK(loaded.defaults.chordTolerance ==
          migrated.recipe.defaults.chordTolerance);
    CHECK(loaded.defaults.cellCap == 4321);
    CHECK(loaded.faceSettings.size() == 1);
    CHECK(loaded.edgeSettings.size() == 1);
    CHECK(loaded.operations.size() == 1);
    CHECK(loaded.faceSettings.front().face.geometricFingerprint ==
          face.geometricFingerprint);
    const weft::RecipeV2Resolution roundTrip =
        weft::resolveRecipeV2(imported, loaded);
    CHECK(roundTrip.complete());
    CHECK(roundTrip.settings.perFace.at(1).radial == 24);
    CHECK(roundTrip.settings.perFace.at(1).cellCap == 7890);
    CHECK(roundTrip.settings.perEdge.at(1) == 7);

    weft::RecipeV2 invalidSave = loaded;
    invalidSave.sourceSha256 = "not-a-hash";
    bool invalidSaveRejected = false;
    try {
        weft::saveRecipeV2(invalidSave, recipePath.string());
    } catch (const std::exception&) {
        invalidSaveRejected = true;
    }
    CHECK(invalidSaveRejected);
    CHECK(weft::loadRecipeV2(recipePath.string()).sourceSha256 ==
          loaded.sourceSha256);
}

void testChangedSourceAndConflictRoutes(const weft::ImportedModel& imported) {
    const weft::RecipeV2MigrationResult migrated =
        weft::migrateRecipeV1(imported, makeLegacyRecipe());
    CHECK(migrated.complete());

    weft::RecipeV2 changedHash = migrated.recipe;
    changedHash.sourceSha256 = std::string(64, '0');
    const weft::RecipeV2Resolution relocated =
        weft::resolveRecipeV2(imported, changedHash);
    CHECK(relocated.complete());
    CHECK(hasIssue(relocated.issues, "recipe.source_hash_changed",
                   weft::RecipeIssueSeverity::Warning));
    CHECK(hasIssue(relocated.issues, "recipe.reference.relocated",
                   weft::RecipeIssueSeverity::Warning));

    weft::RecipeV2 missing = migrated.recipe;
    missing.faceSettings.front().face.geometricFingerprint =
        "face-v1|missing";
    const weft::RecipeV2Resolution missingResult =
        weft::resolveRecipeV2(imported, missing);
    CHECK(!missingResult.complete());
    CHECK(hasIssue(missingResult.issues, "recipe.reference.missing",
                   weft::RecipeIssueSeverity::Conflict));
    CHECK(missingResult.settings.perFace.empty());

    weft::RecipeV2 duplicate = migrated.recipe;
    duplicate.faceSettings.push_back(duplicate.faceSettings.front());
    const weft::RecipeV2Resolution duplicateResult =
        weft::resolveRecipeV2(imported, duplicate);
    CHECK(!duplicateResult.complete());
    CHECK(hasIssue(duplicateResult.issues,
                   "recipe.face.duplicate_working_target",
                   weft::RecipeIssueSeverity::Conflict));
}

void testWorkingRecipeCapture(const weft::ImportedModel& imported) {
    weft::Recipe working = makeLegacyRecipe();
    working.pipeline = weft::MeshPipeline::Legacy;
    const weft::RecipeV2MigrationResult captured =
        weft::captureRecipeV2(imported, working);
    CHECK(captured.complete());
    CHECK(captured.recipe.faceSettings.size() == 1);
    CHECK(captured.recipe.edgeSettings.size() == 1);
    CHECK(captured.recipe.operations.size() == 1);
    CHECK(captured.recipe.faceSettings.front().face.sourceId.ordinal == 1);

    // Prove capture uses the correspondence map rather than assuming source
    // and working ordinals are equal.
    weft::ImportedModel swapped = imported;
    weft::CorrespondenceRecord* faceOne = nullptr;
    weft::CorrespondenceRecord* faceTwo = nullptr;
    for (weft::CorrespondenceRecord& record :
         swapped.correspondence.records) {
        if (record.sourceId.kind != weft::StableIdKind::Face) continue;
        if (record.sourceId.ordinal == 1) faceOne = &record;
        if (record.sourceId.ordinal == 2) faceTwo = &record;
    }
    CHECK(faceOne != nullptr);
    CHECK(faceTwo != nullptr);
    if (faceOne && faceTwo) {
        std::swap(faceOne->workingIds, faceTwo->workingIds);
        faceOne->relation = weft::CorrespondenceRelation::Modified;
        faceTwo->relation = weft::CorrespondenceRelation::Modified;
        weft::Recipe onWorkingTwo;
        onWorkingTwo.settings.perFace[2] = working.settings.perFace.at(1);
        const weft::RecipeV2MigrationResult reverseMapped =
            weft::captureRecipeV2(swapped, onWorkingTwo);
        CHECK(reverseMapped.complete());
        CHECK(reverseMapped.recipe.faceSettings.size() == 1);
        if (!reverseMapped.recipe.faceSettings.empty()) {
            CHECK(reverseMapped.recipe.faceSettings.front()
                      .face.sourceId.ordinal == 1);
        }
        const weft::RecipeV2Resolution resolved =
            weft::resolveRecipeV2(swapped, reverseMapped.recipe);
        CHECK(resolved.complete());
        CHECK(resolved.settings.perFace.contains(2));
    }

    weft::ImportedModel missing = imported;
    missing.correspondence.records.erase(
        std::remove_if(
            missing.correspondence.records.begin(),
            missing.correspondence.records.end(),
            [](const weft::CorrespondenceRecord& record) {
                return record.sourceId.kind == weft::StableIdKind::Face &&
                    record.sourceId.ordinal == 1;
            }),
        missing.correspondence.records.end());
    weft::Recipe missingWorking;
    missingWorking.settings.perFace[1] = working.settings.perFace.at(1);
    const weft::RecipeV2MigrationResult refused =
        weft::captureRecipeV2(missing, missingWorking);
    CHECK(!refused.complete());
    CHECK(hasIssue(refused.issues,
                   "recipe_capture.correspondence.missing",
                   weft::RecipeIssueSeverity::Conflict));
}

void testV1RefusalsAndMalformedFile(
    const weft::ImportedModel& imported,
    const std::filesystem::path& malformedPath) {
    weft::Recipe worldOperation;
    weft::ManualOp operation;
    operation.kind = weft::ManualOp::Kind::DeletePoly;
    worldOperation.ops.push_back(operation);
    const weft::RecipeV2MigrationResult world =
        weft::migrateRecipeV1(imported, worldOperation);
    CHECK(!world.complete());
    CHECK(hasIssue(world.issues,
                   "recipe_v1.world_space_operation_unmigratable",
                   weft::RecipeIssueSeverity::Conflict));
    CHECK(world.recipe.operations.empty());

    weft::Recipe invalidReference;
    invalidReference.settings.perEdge[999999] = 4;
    const weft::RecipeV2MigrationResult invalid =
        weft::migrateRecipeV1(imported, invalidReference);
    CHECK(!invalid.complete());
    CHECK(hasIssue(invalid.issues, "recipe_v1.reference.invalid",
                   weft::RecipeIssueSeverity::Conflict));

    {
        std::ofstream malformed(malformedPath, std::ios::binary);
        malformed << "weft-recipe 2\n"
                  << "source-sha256 \"abc\"\n"
                  << "edge 1 \"\" 0\n";
    }
    bool rejected = false;
    try {
        (void)weft::loadRecipeV2(malformedPath.string());
    } catch (const std::exception& error) {
        rejected = std::string(error.what()).find(":3:") !=
            std::string::npos;
    }
    CHECK(rejected);

    {
        std::ofstream malformed(malformedPath, std::ios::binary);
        malformed << "weft-recipe 2\n"
                  << "source-sha256 \"abc\"\n";
    }
    bool invalidHashRejected = false;
    try {
        (void)weft::loadRecipeV2(malformedPath.string());
    } catch (const std::exception& error) {
        invalidHashRejected =
            std::string(error.what()).find("invalid source SHA-256") !=
            std::string::npos;
    }
    CHECK(invalidHashRejected);

    weft::Recipe globalsOnly;
    globalsOnly.settings.defaults.radial = 32;
    const weft::RecipeV2MigrationResult safeMigration =
        weft::migrateRecipeV1(imported, globalsOnly);
    const weft::RecipeV2Resolution safeResolution =
        weft::resolveRecipeV2(imported, safeMigration.recipe);
    CHECK(safeMigration.complete());
    CHECK(safeResolution.complete());
    CHECK(weft::validateSecureRecipeApplication(safeResolution).empty());
}

}  // namespace

int main() {
    try {
        TemporaryFiles files;
        const weft::ImportedModel imported =
            weft::importStepSecure(files.step().string());
        testReferencesMigrationResolutionAndPersistence(
            imported, files.recipe());
        testChangedSourceAndConflictRoutes(imported);
        testWorkingRecipeCapture(imported);
        testV1RefusalsAndMalformedFile(imported, files.malformed());
    } catch (const std::exception& error) {
        std::printf("FAIL secure-recipe exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("secure recipe v2 checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d secure recipe failure(s)\n", failures);
    return EXIT_FAILURE;
}
