#pragma once

#include "weft/recipe.hpp"
#include "weft/secure_core.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace weft {

struct SourceEntityReference {
    StableId sourceId;
    std::string geometricFingerprint;

    bool valid() const noexcept {
        return sourceId.valid() &&
            (sourceId.kind == StableIdKind::Face ||
             sourceId.kind == StableIdKind::Edge) &&
            !geometricFingerprint.empty();
    }
};

enum class RecipeIssueSeverity {
    Warning,
    Conflict,
};

struct RecipeMigrationIssue {
    RecipeIssueSeverity severity = RecipeIssueSeverity::Conflict;
    std::string code;
    std::string message;
    std::optional<SourceEntityReference> reference;
};

struct ReferencedFaceSettings {
    SourceEntityReference face;
    FaceMeshSettings settings;
};

struct ReferencedEdgeSettings {
    SourceEntityReference edge;
    int count = 0;
};

struct ReferencedManualOperation {
    ManualOp operation;
    std::optional<SourceEntityReference> face;
    std::optional<SourceEntityReference> edgeA;
    std::optional<SourceEntityReference> edgeB;
};

struct RecipeV2 {
    std::string sourceSha256;
    FaceMeshSettings defaults;
    double densityScale = 1.0;
    double weldTolerance = 1e-6;
    std::vector<ReferencedFaceSettings> faceSettings;
    std::vector<ReferencedEdgeSettings> edgeSettings;
    std::vector<ReferencedManualOperation> operations;
};

struct RecipeV2MigrationResult {
    RecipeV2 recipe;
    std::vector<RecipeMigrationIssue> issues;

    bool complete() const noexcept;
};

struct RecipeV2Resolution {
    GenerationSettings settings;
    std::vector<ManualOp> operations;
    std::vector<RecipeMigrationIssue> issues;

    bool complete() const noexcept;
};

// Fingerprints are normalized to model scale and source coordinates. They are
// deliberately strict: ambiguity or geometric drift becomes a conflict, never
// a best-effort reassignment.
SourceEntityReference makeSourceEntityReference(
    const ImportedModel& imported, StableId sourceId);

RecipeV2MigrationResult migrateRecipeV1(
    const ImportedModel& imported, const Recipe& legacy);

// Capture the application's current working-shape recipe back onto immutable
// source references. This is intentionally separate from v1 migration: app
// edits name working entities, whereas historical recipe v1 files name source
// import ordinals.
RecipeV2MigrationResult captureRecipeV2(
    const ImportedModel& imported, const Recipe& workingRecipe);

RecipeV2Resolution resolveRecipeV2(
    const ImportedModel& imported, const RecipeV2& recipe);

// Current rollout gate: representation-independent global radial/axial,
// chord, and normal-angle controls, plus one certified per-face cylinder
// `axial` override (WP-041). Resolved v2 data remains inspectable, but these
// issues block generation/export.
std::vector<RecipeMigrationIssue> validateSecureRecipeApplication(
    const RecipeV2Resolution& resolution);

// When resolution carries exactly one axial-only per-face override for a
// cylinder working face, returns that axial count. Otherwise nullopt.
std::optional<std::uint32_t> certifiedPerFaceCylinderAxial(
    const RecipeV2Resolution& resolution);

void saveRecipeV2(const RecipeV2& recipe, const std::string& path);
RecipeV2 loadRecipeV2(const std::string& path);
int recipeFileVersion(const std::string& path);

}  // namespace weft
