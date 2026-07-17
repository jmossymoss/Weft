#include "weft/secure_recipe.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace weft {

namespace {

bool hasConflict(const std::vector<RecipeMigrationIssue>& issues) {
    return std::any_of(
        issues.begin(), issues.end(), [](const RecipeMigrationIssue& issue) {
            return issue.severity == RecipeIssueSeverity::Conflict;
        });
}

bool validSha256(const std::string& value) {
    return value.size() == 64 &&
        std::all_of(value.begin(), value.end(), [](char c) {
            return std::isxdigit(static_cast<unsigned char>(c)) != 0;
        });
}

void addIssue(std::vector<RecipeMigrationIssue>& issues,
              RecipeIssueSeverity severity, std::string code,
              std::string message,
              std::optional<SourceEntityReference> reference = std::nullopt) {
    issues.push_back(
        {severity, std::move(code), std::move(message), std::move(reference)});
}

struct ModelNormalization {
    double scale = 1.0;
    std::array<double, 3> center{};
};

ModelNormalization normalization(const Model& model) {
    Bnd_Box box;
    BRepBndLib::Add(model.shape, box);
    if (box.IsVoid()) return {};
    double x0 = 0.0;
    double y0 = 0.0;
    double z0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double z1 = 0.0;
    box.Get(x0, y0, z0, x1, y1, z1);
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    ModelNormalization result;
    result.scale = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(result.scale) || result.scale <= 1e-12) {
        result.scale = 1.0;
    }
    result.center = {
        0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)};
    return result;
}

std::int64_t quantize(double value) {
    if (!std::isfinite(value)) {
        throw std::runtime_error(
            "cannot fingerprint non-finite geometric evidence");
    }
    return static_cast<std::int64_t>(std::llround(value * 1.0e9));
}

std::string faceFingerprint(const Model& model, const Analysis& analysis,
                            const ModelNormalization& normal, int faceId) {
    GProp_GProps properties;
    const TopoDS_Face face = TopoDS::Face(model.faces(faceId));
    BRepGProp::SurfaceProperties(face, properties);
    const gp_Pnt center = properties.CentreOfMass();
    const FaceInfo& info = analysis.faces[faceId - 1];
    const BRepAdaptor_Surface surface(face, true);
    std::ostringstream stream;
    stream << "face-v1|surface=" << static_cast<int>(surface.GetType())
           << "|area="
           << quantize(properties.Mass() / (normal.scale * normal.scale))
           << "|center="
           << quantize((center.X() - normal.center[0]) / normal.scale) << ','
           << quantize((center.Y() - normal.center[1]) / normal.scale) << ','
           << quantize((center.Z() - normal.center[2]) / normal.scale)
           << "|radius=" << quantize(info.radius / normal.scale)
           << "|edges=" << info.edgeIds.size();
    return stream.str();
}

std::string edgeFingerprint(const Model& model, const Analysis& analysis,
                            const ModelNormalization& normal, int edgeId) {
    GProp_GProps properties;
    const TopoDS_Edge edge = TopoDS::Edge(model.edges(edgeId));
    BRepGProp::LinearProperties(edge, properties);
    const gp_Pnt center = properties.CentreOfMass();
    const EdgeInfo& info = analysis.edges[edgeId - 1];
    const BRepAdaptor_Curve curve(edge);
    std::ostringstream stream;
    stream << "edge-v1|curve=" << static_cast<int>(curve.GetType())
           << "|length=" << quantize(properties.Mass() / normal.scale)
           << "|center="
           << quantize((center.X() - normal.center[0]) / normal.scale) << ','
           << quantize((center.Y() - normal.center[1]) / normal.scale) << ','
           << quantize((center.Z() - normal.center[2]) / normal.scale)
           << "|owners=" << info.faceIds.size();
    return stream.str();
}

struct ReferenceCatalog {
    std::map<StableId, SourceEntityReference> byId;
    std::map<std::pair<StableIdKind, std::string>, std::vector<StableId>>
        byFingerprint;
};

ReferenceCatalog buildCatalog(const ImportedModel& imported) {
    if (!imported.source) {
        throw std::runtime_error("recipe references require a source B-rep");
    }
    const Model& model = imported.source->snapshot.model;
    const Analysis analysis = analyze(model);
    const ModelNormalization normal = normalization(model);
    ReferenceCatalog catalog;
    auto add = [&](StableId id, std::string fingerprint) {
        SourceEntityReference reference{id, std::move(fingerprint)};
        catalog.byFingerprint[{id.kind, reference.geometricFingerprint}]
            .push_back(id);
        catalog.byId.emplace(id, std::move(reference));
    };
    for (int faceId = 1; faceId <= model.faceCount(); ++faceId) {
        add({StableIdKind::Face, static_cast<std::uint64_t>(faceId)},
            faceFingerprint(model, analysis, normal, faceId));
    }
    for (int edgeId = 1; edgeId <= model.edgeCount(); ++edgeId) {
        add({StableIdKind::Edge, static_cast<std::uint64_t>(edgeId)},
            edgeFingerprint(model, analysis, normal, edgeId));
    }
    return catalog;
}

SourceEntityReference referenceFor(const ReferenceCatalog& catalog,
                                   StableId sourceId) {
    const auto found = catalog.byId.find(sourceId);
    if (found == catalog.byId.end()) {
        throw std::runtime_error("source recipe reference is out of range");
    }
    return found->second;
}

std::optional<SourceEntityReference> referenceForWorking(
    const ImportedModel& imported, const ReferenceCatalog& catalog,
    StableId workingId, std::vector<RecipeMigrationIssue>& issues) {
    std::vector<StableId> candidates;
    for (const CorrespondenceRecord& record : imported.correspondence.records) {
        if (!record.sourceId.valid() || record.sourceId.kind != workingId.kind ||
            record.workingIds.size() != 1 ||
            record.workingIds.front() != workingId ||
            (record.relation != CorrespondenceRelation::Identity &&
             record.relation != CorrespondenceRelation::Modified)) {
            continue;
        }
        candidates.push_back(record.sourceId);
    }
    if (candidates.empty()) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "recipe_capture.correspondence.missing",
                 "working entity has no bijective source correspondence");
        return std::nullopt;
    }
    if (candidates.size() != 1) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "recipe_capture.correspondence.ambiguous",
                 "working entity maps to multiple source entities");
        return std::nullopt;
    }
    try {
        return referenceFor(catalog, candidates.front());
    } catch (const std::exception& error) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "recipe_capture.reference.invalid",
                 "source reference could not be fingerprinted: " +
                     std::string(error.what()));
        return std::nullopt;
    }
}

struct ResolvedReference {
    std::optional<StableId> workingId;
};

ResolvedReference resolveReference(
    const ImportedModel& imported, const RecipeV2& recipe,
    const ReferenceCatalog& catalog, const SourceEntityReference& reference,
    std::vector<RecipeMigrationIssue>& issues) {
    if (!reference.valid()) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "recipe.reference.invalid",
                 "recipe reference is missing a source ID or fingerprint",
                 reference);
        return {};
    }

    StableId sourceId = reference.sourceId;
    const auto exact = catalog.byId.find(sourceId);
    const bool sourceHashMatches = imported.source &&
        recipe.sourceSha256 == imported.source->metadata.sourceSha256;
    const bool exactFingerprint = exact != catalog.byId.end() &&
        exact->second.geometricFingerprint == reference.geometricFingerprint;
    if (!sourceHashMatches || !exactFingerprint) {
        const auto candidates = catalog.byFingerprint.find(
            {reference.sourceId.kind, reference.geometricFingerprint});
        const std::size_t candidateCount =
            candidates == catalog.byFingerprint.end()
            ? 0U
            : candidates->second.size();
        if (candidateCount == 0) {
            addIssue(issues, RecipeIssueSeverity::Conflict,
                     "recipe.reference.missing",
                     "no source entity has the recorded geometric fingerprint",
                     reference);
            return {};
        }
        if (candidateCount != 1) {
            addIssue(issues, RecipeIssueSeverity::Conflict,
                     "recipe.reference.ambiguous",
                     "multiple source entities have the recorded geometric "
                     "fingerprint",
                     reference);
            return {};
        }
        sourceId = candidates->second.front();
        addIssue(issues, RecipeIssueSeverity::Warning,
                 "recipe.reference.relocated",
                 "source ordinal changed; the unique geometric fingerprint "
                 "was used",
                 reference);
    }

    const CorrespondenceRecord* correspondence =
        imported.correspondence.find(sourceId);
    if (!correspondence || correspondence->workingIds.empty()) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "recipe.correspondence.missing",
                 "source entity has no working-shape correspondence",
                 reference);
        return {};
    }
    if (correspondence->workingIds.size() != 1 ||
        (correspondence->relation != CorrespondenceRelation::Identity &&
         correspondence->relation != CorrespondenceRelation::Modified)) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "recipe.correspondence.non_bijective",
                 "source entity does not map bijectively to one working entity",
                 reference);
        return {};
    }
    const StableId workingId = correspondence->workingIds.front();
    if (workingId.kind != sourceId.kind || !workingId.valid()) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "recipe.correspondence.kind_mismatch",
                 "working correspondence has the wrong entity kind",
                 reference);
        return {};
    }
    return {workingId};
}

void addMigrationReferenceIssue(
    std::vector<RecipeMigrationIssue>& issues, StableId sourceId,
    const std::exception& error) {
    addIssue(issues, RecipeIssueSeverity::Conflict,
             "recipe_v1.reference.invalid",
             "legacy reference could not be migrated: " +
                 std::string(error.what()),
             SourceEntityReference{sourceId, {}});
}

SourceEntityReference parsedReference(StableIdKind kind,
                                      std::uint64_t ordinal,
                                      std::string fingerprint) {
    SourceEntityReference reference{{kind, ordinal}, std::move(fingerprint)};
    if (!reference.valid()) {
        throw std::runtime_error("invalid source entity reference");
    }
    return reference;
}

}  // namespace

bool RecipeV2MigrationResult::complete() const noexcept {
    return !hasConflict(issues);
}

bool RecipeV2Resolution::complete() const noexcept {
    return !hasConflict(issues);
}

SourceEntityReference makeSourceEntityReference(
    const ImportedModel& imported, StableId sourceId) {
    return referenceFor(buildCatalog(imported), sourceId);
}

RecipeV2MigrationResult migrateRecipeV1(
    const ImportedModel& imported, const Recipe& legacy) {
    RecipeV2MigrationResult result;
    result.recipe.sourceSha256 = imported.source
        ? imported.source->metadata.sourceSha256
        : std::string();
    result.recipe.defaults = legacy.settings.defaults;
    result.recipe.densityScale = legacy.settings.densityScale;
    result.recipe.weldTolerance = legacy.settings.weldTolerance;
    const ReferenceCatalog catalog = buildCatalog(imported);

    addIssue(result.issues, RecipeIssueSeverity::Warning,
             "recipe_v1.pipeline_ignored",
             "the legacy pipeline selector is not part of recipe v2");

    for (const auto& [faceId, settings] : legacy.settings.perFace) {
        const StableId sourceId{
            StableIdKind::Face, static_cast<std::uint64_t>(faceId)};
        try {
            result.recipe.faceSettings.push_back(
                {referenceFor(catalog, sourceId), settings});
        } catch (const std::exception& error) {
            addMigrationReferenceIssue(result.issues, sourceId, error);
        }
    }

    std::map<int, int> edgeCounts = legacy.settings.perEdge;
    for (const auto& [edgeId, count] : legacy.compiler.perEdge) {
        const auto existing = edgeCounts.find(edgeId);
        if (existing != edgeCounts.end() && existing->second != count) {
            addIssue(result.issues, RecipeIssueSeverity::Conflict,
                     "recipe_v1.edge_count_conflict",
                     "legacy and compiler edge counts disagree for edge " +
                         std::to_string(edgeId));
            continue;
        }
        edgeCounts[edgeId] = count;
    }
    for (const auto& [edgeId, count] : edgeCounts) {
        const StableId sourceId{
            StableIdKind::Edge, static_cast<std::uint64_t>(edgeId)};
        try {
            result.recipe.edgeSettings.push_back(
                {referenceFor(catalog, sourceId), count});
        } catch (const std::exception& error) {
            addMigrationReferenceIssue(result.issues, sourceId, error);
        }
    }

    for (const ManualOp& operation : legacy.ops) {
        ReferencedManualOperation migrated;
        migrated.operation = operation;
        try {
            if (operation.kind == ManualOp::Kind::LoopInsert ||
                operation.kind == ManualOp::Kind::NudgeVertex) {
                migrated.face = referenceFor(
                    catalog,
                    {StableIdKind::Face,
                     static_cast<std::uint64_t>(operation.faceId)});
            } else if (operation.kind == ManualOp::Kind::Bridge) {
                migrated.edgeA = referenceFor(
                    catalog,
                    {StableIdKind::Edge,
                     static_cast<std::uint64_t>(operation.edgeA)});
                migrated.edgeB = referenceFor(
                    catalog,
                    {StableIdKind::Edge,
                     static_cast<std::uint64_t>(operation.edgeB)});
            } else if (operation.kind == ManualOp::Kind::FillLoop) {
                migrated.edgeA = referenceFor(
                    catalog,
                    {StableIdKind::Edge,
                     static_cast<std::uint64_t>(operation.edgeA)});
            } else {
                addIssue(result.issues, RecipeIssueSeverity::Conflict,
                         "recipe_v1.world_space_operation_unmigratable",
                         "world-space mesh surgery is not a source-anchored "
                         "recipe v2 operation");
                continue;
            }
            result.recipe.operations.push_back(std::move(migrated));
        } catch (const std::exception& error) {
            addIssue(result.issues, RecipeIssueSeverity::Conflict,
                     "recipe_v1.operation_reference_invalid",
                     "manual operation reference could not be migrated: " +
                         std::string(error.what()));
        }
    }
    return result;
}

RecipeV2MigrationResult captureRecipeV2(
    const ImportedModel& imported, const Recipe& workingRecipe) {
    RecipeV2MigrationResult result;
    result.recipe.sourceSha256 = imported.source
        ? imported.source->metadata.sourceSha256
        : std::string();
    result.recipe.defaults = workingRecipe.settings.defaults;
    result.recipe.densityScale = workingRecipe.settings.densityScale;
    result.recipe.weldTolerance = workingRecipe.settings.weldTolerance;
    const ReferenceCatalog catalog = buildCatalog(imported);

    for (const auto& [faceId, settings] : workingRecipe.settings.perFace) {
        const auto reference = referenceForWorking(
            imported, catalog,
            {StableIdKind::Face, static_cast<std::uint64_t>(faceId)},
            result.issues);
        if (reference) {
            result.recipe.faceSettings.push_back({*reference, settings});
        }
    }
    for (const auto& [edgeId, count] : workingRecipe.settings.perEdge) {
        const auto reference = referenceForWorking(
            imported, catalog,
            {StableIdKind::Edge, static_cast<std::uint64_t>(edgeId)},
            result.issues);
        if (reference) {
            result.recipe.edgeSettings.push_back({*reference, count});
        }
    }

    for (const ManualOp& operation : workingRecipe.ops) {
        ReferencedManualOperation captured;
        captured.operation = operation;
        if (operation.kind == ManualOp::Kind::LoopInsert ||
            operation.kind == ManualOp::Kind::NudgeVertex) {
            captured.face = referenceForWorking(
                imported, catalog,
                {StableIdKind::Face,
                 static_cast<std::uint64_t>(operation.faceId)},
                result.issues);
            if (!captured.face) continue;
        } else if (operation.kind == ManualOp::Kind::Bridge ||
                   operation.kind == ManualOp::Kind::FillLoop) {
            captured.edgeA = referenceForWorking(
                imported, catalog,
                {StableIdKind::Edge,
                 static_cast<std::uint64_t>(operation.edgeA)},
                result.issues);
            if (!captured.edgeA) continue;
            if (operation.kind == ManualOp::Kind::Bridge) {
                captured.edgeB = referenceForWorking(
                    imported, catalog,
                    {StableIdKind::Edge,
                     static_cast<std::uint64_t>(operation.edgeB)},
                    result.issues);
                if (!captured.edgeB) continue;
            }
        } else {
            addIssue(result.issues, RecipeIssueSeverity::Conflict,
                     "recipe_capture.operation.not_source_anchored",
                     "world-space mesh surgery cannot be saved as recipe v2");
            continue;
        }
        result.recipe.operations.push_back(std::move(captured));
    }
    return result;
}

RecipeV2Resolution resolveRecipeV2(
    const ImportedModel& imported, const RecipeV2& recipe) {
    RecipeV2Resolution result;
    result.settings.defaults = recipe.defaults;
    result.settings.densityScale = recipe.densityScale;
    result.settings.weldTolerance = recipe.weldTolerance;
    const ReferenceCatalog catalog = buildCatalog(imported);

    if (!imported.source ||
        imported.source->metadata.sourceSha256 != recipe.sourceSha256) {
        addIssue(result.issues, RecipeIssueSeverity::Warning,
                 "recipe.source_hash_changed",
                 "source hash changed; every reference requires a unique "
                 "geometric fingerprint match");
    }

    std::set<std::uint64_t> usedFaces;
    for (const ReferencedFaceSettings& item : recipe.faceSettings) {
        const ResolvedReference resolved = resolveReference(
            imported, recipe, catalog, item.face, result.issues);
        if (!resolved.workingId) continue;
        const std::uint64_t ordinal = resolved.workingId->ordinal;
        if (!usedFaces.insert(ordinal).second) {
            addIssue(result.issues, RecipeIssueSeverity::Conflict,
                     "recipe.face.duplicate_working_target",
                     "multiple recipe face settings resolve to one working "
                     "face",
                     item.face);
            continue;
        }
        result.settings.perFace[static_cast<int>(ordinal)] = item.settings;
    }

    std::set<std::uint64_t> usedEdges;
    for (const ReferencedEdgeSettings& item : recipe.edgeSettings) {
        if (item.count < 1) {
            addIssue(result.issues, RecipeIssueSeverity::Conflict,
                     "recipe.edge.count_invalid",
                     "edge subdivision count must be positive", item.edge);
            continue;
        }
        const ResolvedReference resolved = resolveReference(
            imported, recipe, catalog, item.edge, result.issues);
        if (!resolved.workingId) continue;
        const std::uint64_t ordinal = resolved.workingId->ordinal;
        if (!usedEdges.insert(ordinal).second) {
            addIssue(result.issues, RecipeIssueSeverity::Conflict,
                     "recipe.edge.duplicate_working_target",
                     "multiple recipe edge settings resolve to one working "
                     "edge",
                     item.edge);
            continue;
        }
        result.settings.perEdge[static_cast<int>(ordinal)] = item.count;
    }

    for (const ReferencedManualOperation& item : recipe.operations) {
        ManualOp operation = item.operation;
        bool valid = true;
        if (operation.kind == ManualOp::Kind::LoopInsert ||
            operation.kind == ManualOp::Kind::NudgeVertex) {
            if (!item.face) {
                addIssue(result.issues, RecipeIssueSeverity::Conflict,
                         "recipe.operation.face_reference_missing",
                         "face-anchored operation has no source reference");
                valid = false;
            } else {
                const ResolvedReference resolved = resolveReference(
                    imported, recipe, catalog, *item.face, result.issues);
                if (resolved.workingId) {
                    operation.faceId =
                        static_cast<int>(resolved.workingId->ordinal);
                } else {
                    valid = false;
                }
            }
        } else if (operation.kind == ManualOp::Kind::Bridge ||
                   operation.kind == ManualOp::Kind::FillLoop) {
            if (!item.edgeA) {
                addIssue(result.issues, RecipeIssueSeverity::Conflict,
                         "recipe.operation.edge_reference_missing",
                         "edge-anchored operation has no first source edge");
                valid = false;
            } else {
                const ResolvedReference resolved = resolveReference(
                    imported, recipe, catalog, *item.edgeA, result.issues);
                if (resolved.workingId) {
                    operation.edgeA =
                        static_cast<int>(resolved.workingId->ordinal);
                } else {
                    valid = false;
                }
            }
            if (operation.kind == ManualOp::Kind::Bridge) {
                if (!item.edgeB) {
                    addIssue(result.issues, RecipeIssueSeverity::Conflict,
                             "recipe.operation.edge_reference_missing",
                             "bridge operation has no second source edge");
                    valid = false;
                } else {
                    const ResolvedReference resolved = resolveReference(
                        imported, recipe, catalog, *item.edgeB,
                        result.issues);
                    if (resolved.workingId) {
                        operation.edgeB =
                            static_cast<int>(resolved.workingId->ordinal);
                    } else {
                        valid = false;
                    }
                }
            }
        } else {
            addIssue(result.issues, RecipeIssueSeverity::Conflict,
                     "recipe.operation.not_source_anchored",
                     "recipe v2 cannot resolve world-space mesh surgery");
            valid = false;
        }
        if (valid) result.operations.push_back(std::move(operation));
    }
    return result;
}

std::vector<RecipeMigrationIssue> validateSecureRecipeApplication(
    const RecipeV2Resolution& resolution) {
    std::vector<RecipeMigrationIssue> issues;
    if (!resolution.complete()) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "secure_recipe.application.unresolved",
                 "recipe resolution has conflicts");
        return issues;
    }
    if (!resolution.settings.perFace.empty()) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "secure_recipe.application.face_settings_unimplemented",
                 "per-face certified-template settings are not implemented");
    }
    if (!resolution.operations.empty()) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "secure_recipe.application.operations_unimplemented",
                 "certified surface-anchored editing is not implemented");
    }

    const FaceMeshSettings baseline;
    const FaceMeshSettings& value = resolution.settings.defaults;
    const bool unsupportedDefaults =
        value.gridU != baseline.gridU || value.gridV != baseline.gridV ||
        value.cap != baseline.cap ||
        value.filletLoops != baseline.filletLoops ||
        value.filletHold != baseline.filletHold ||
        value.junctionRings != baseline.junctionRings ||
        value.quadDominant != baseline.quadDominant ||
        value.pureTriFloor != baseline.pureTriFloor ||
        value.minimal != baseline.minimal || value.exclude != baseline.exclude ||
        value.forceMesher != baseline.forceMesher ||
        value.linkRims != baseline.linkRims ||
        value.minSize != baseline.minSize ||
        value.relativeDeviation != baseline.relativeDeviation ||
        value.weldTolerance != baseline.weldTolerance ||
        value.squareCollar != baseline.squareCollar ||
        value.coonsRotate != baseline.coonsRotate ||
        value.boundary != baseline.boundary ||
        value.adaptive != baseline.adaptive ||
        value.cellCap != baseline.cellCap;
    if (unsupportedDefaults) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "secure_recipe.application.defaults_unimplemented",
                 "legacy modelling defaults cannot drive certified topology");
    }

    const GenerationSettings settingsBaseline;
    if (resolution.settings.weldTolerance !=
            settingsBaseline.weldTolerance ||
        resolution.settings.densityScale !=
            settingsBaseline.densityScale ||
        resolution.settings.parallelMeshing !=
            settingsBaseline.parallelMeshing ||
        resolution.settings.conformBorders !=
            settingsBaseline.conformBorders ||
        resolution.settings.canonicalEdgeContracts !=
            settingsBaseline.canonicalEdgeContracts ||
        resolution.settings.finalizeMesh != settingsBaseline.finalizeMesh ||
        resolution.settings.decoupleSeams !=
            settingsBaseline.decoupleSeams) {
        addIssue(issues, RecipeIssueSeverity::Conflict,
                 "secure_recipe.application.generation_controls_unimplemented",
                 "legacy generation controls cannot drive the secure pipeline");
    }
    return issues;
}

void saveRecipeV2(const RecipeV2& recipe, const std::string& path) {
    if (!validSha256(recipe.sourceSha256)) {
        throw std::runtime_error(
            "cannot save recipe v2 without a valid source SHA-256");
    }
    if (!std::isfinite(recipe.densityScale) || recipe.densityScale <= 0.0 ||
        !std::isfinite(recipe.weldTolerance) || recipe.weldTolerance < 0.0) {
        throw std::runtime_error(
            "cannot save recipe v2 with invalid global controls");
    }
    for (const ReferencedFaceSettings& item : recipe.faceSettings) {
        if (!item.face.valid()) {
            throw std::runtime_error(
                "cannot save recipe v2 with an invalid face reference");
        }
    }
    for (const ReferencedEdgeSettings& item : recipe.edgeSettings) {
        if (!item.edge.valid() || item.count < 1) {
            throw std::runtime_error(
                "cannot save recipe v2 with an invalid edge record");
        }
    }
    for (const ReferencedManualOperation& item : recipe.operations) {
        const ManualOp::Kind kind = item.operation.kind;
        if (kind == ManualOp::Kind::LoopInsert ||
            kind == ManualOp::Kind::NudgeVertex) {
            if (!item.face || !item.face->valid()) {
                throw std::runtime_error(
                    "cannot save recipe v2 with an invalid face operation");
            }
        } else if (kind == ManualOp::Kind::Bridge) {
            if (!item.edgeA || !item.edgeA->valid() || !item.edgeB ||
                !item.edgeB->valid()) {
                throw std::runtime_error(
                    "cannot save recipe v2 with an invalid bridge operation");
            }
        } else if (kind == ManualOp::Kind::FillLoop) {
            if (!item.edgeA || !item.edgeA->valid() || item.edgeB) {
                throw std::runtime_error(
                    "cannot save recipe v2 with an invalid fill operation");
            }
        } else {
            throw std::runtime_error(
                "cannot save non-source-anchored operation in recipe v2");
        }
    }
    std::ofstream output(path, std::ios::binary);
    if (!output) {
        throw std::runtime_error("cannot open recipe v2 for writing: " + path);
    }
    output.imbue(std::locale::classic());
    output << "weft-recipe 2\n";
    output << "source-sha256 " << std::quoted(recipe.sourceSha256) << "\n";
    output << "default " << formatSettingsList(recipe.defaults) << "\n";
    output << std::setprecision(17);
    output << "scale " << recipe.densityScale << "\n";
    output << "weld " << recipe.weldTolerance << "\n";
    for (const ReferencedFaceSettings& item : recipe.faceSettings) {
        output << "face " << item.face.sourceId.ordinal << ' '
               << std::quoted(item.face.geometricFingerprint) << ' '
               << formatSettingsList(item.settings) << "\n";
    }
    for (const ReferencedEdgeSettings& item : recipe.edgeSettings) {
        output << "edge " << item.edge.sourceId.ordinal << ' '
               << std::quoted(item.edge.geometricFingerprint) << ' '
               << item.count << "\n";
    }
    for (const ReferencedManualOperation& item : recipe.operations) {
        const ManualOp& operation = item.operation;
        if (operation.kind == ManualOp::Kind::LoopInsert ||
            operation.kind == ManualOp::Kind::NudgeVertex) {
            if (!item.face) {
                throw std::runtime_error(
                    "cannot save face operation without source reference");
            }
            output << "op-face " << static_cast<int>(operation.kind) << ' '
                   << item.face->sourceId.ordinal << ' '
                   << std::quoted(item.face->geometricFingerprint) << ' '
                   << operation.u << ' ' << operation.v << ' ' << operation.t
                   << ' ' << operation.u2 << ' ' << operation.v2 << "\n";
        } else if (operation.kind == ManualOp::Kind::Bridge ||
                   operation.kind == ManualOp::Kind::FillLoop) {
            if (!item.edgeA ||
                (operation.kind == ManualOp::Kind::Bridge && !item.edgeB)) {
                throw std::runtime_error(
                    "cannot save edge operation without source references");
            }
            output << "op-edge " << static_cast<int>(operation.kind) << ' '
                   << item.edgeA->sourceId.ordinal << ' '
                   << std::quoted(item.edgeA->geometricFingerprint) << ' '
                   << (item.edgeB ? item.edgeB->sourceId.ordinal : 0U) << ' '
                   << std::quoted(item.edgeB
                                      ? item.edgeB->geometricFingerprint
                                      : std::string())
                   << ' ' << operation.twist << ' ' << operation.twistA << ' '
                   << operation.twistSide << ' ' << operation.spans << "\n";
        } else {
            throw std::runtime_error(
                "cannot save non-source-anchored operation in recipe v2");
        }
    }
    output.close();
    if (!output) {
        throw std::runtime_error("failed to write recipe v2: " + path);
    }
}

RecipeV2 loadRecipeV2(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open recipe v2: " + path);
    std::string header;
    int version = 0;
    input >> header >> version;
    if (header != "weft-recipe" || version != 2) {
        throw std::runtime_error("not a weft recipe v2: " + path);
    }
    std::string line;
    std::getline(input, line);
    RecipeV2 recipe;
    int lineNumber = 1;
    while (std::getline(input, line)) {
        ++lineNumber;
        if (line.empty() || line.front() == '#') continue;
        std::istringstream stream(line);
        stream.imbue(std::locale::classic());
        std::string kind;
        stream >> kind;
        try {
            if (kind == "source-sha256") {
                stream >> std::quoted(recipe.sourceSha256);
            } else if (kind == "default") {
                std::string settings;
                stream >> settings;
                applySettingsList(recipe.defaults, settings);
            } else if (kind == "scale") {
                stream >> recipe.densityScale;
                if (!stream || !std::isfinite(recipe.densityScale) ||
                    recipe.densityScale <= 0.0) {
                    throw std::runtime_error("invalid density scale");
                }
            } else if (kind == "weld") {
                stream >> recipe.weldTolerance;
                if (!stream || !std::isfinite(recipe.weldTolerance) ||
                    recipe.weldTolerance < 0.0) {
                    throw std::runtime_error("invalid weld tolerance");
                }
            } else if (kind == "face") {
                std::uint64_t ordinal = 0;
                std::string fingerprint;
                std::string settings;
                stream >> ordinal >> std::quoted(fingerprint) >> settings;
                FaceMeshSettings value = recipe.defaults;
                applySettingsList(value, settings);
                recipe.faceSettings.push_back(
                    {parsedReference(StableIdKind::Face, ordinal,
                                     std::move(fingerprint)),
                     value});
            } else if (kind == "edge") {
                std::uint64_t ordinal = 0;
                std::string fingerprint;
                int count = 0;
                stream >> ordinal >> std::quoted(fingerprint) >> count;
                if (!stream || count < 1) {
                    throw std::runtime_error("invalid edge count");
                }
                recipe.edgeSettings.push_back(
                    {parsedReference(StableIdKind::Edge, ordinal,
                                     std::move(fingerprint)),
                     count});
            } else if (kind == "op-face") {
                int operationKind = -1;
                std::uint64_t ordinal = 0;
                std::string fingerprint;
                ReferencedManualOperation item;
                stream >> operationKind >> ordinal >> std::quoted(fingerprint)
                       >> item.operation.u >> item.operation.v
                       >> item.operation.t >> item.operation.u2
                       >> item.operation.v2;
                if (!stream ||
                    (operationKind !=
                         static_cast<int>(ManualOp::Kind::LoopInsert) &&
                     operationKind !=
                         static_cast<int>(ManualOp::Kind::NudgeVertex))) {
                    throw std::runtime_error("invalid face operation");
                }
                item.operation.kind =
                    static_cast<ManualOp::Kind>(operationKind);
                item.face = parsedReference(
                    StableIdKind::Face, ordinal, std::move(fingerprint));
                recipe.operations.push_back(std::move(item));
            } else if (kind == "op-edge") {
                int operationKind = -1;
                std::uint64_t ordinalA = 0;
                std::uint64_t ordinalB = 0;
                std::string fingerprintA;
                std::string fingerprintB;
                ReferencedManualOperation item;
                stream >> operationKind >> ordinalA
                       >> std::quoted(fingerprintA) >> ordinalB
                       >> std::quoted(fingerprintB) >> item.operation.twist
                       >> item.operation.twistA >> item.operation.twistSide
                       >> item.operation.spans;
                if (!stream ||
                    (operationKind !=
                         static_cast<int>(ManualOp::Kind::Bridge) &&
                     operationKind !=
                         static_cast<int>(ManualOp::Kind::FillLoop))) {
                    throw std::runtime_error("invalid edge operation");
                }
                item.operation.kind =
                    static_cast<ManualOp::Kind>(operationKind);
                item.edgeA = parsedReference(
                    StableIdKind::Edge, ordinalA, std::move(fingerprintA));
                if (item.operation.kind == ManualOp::Kind::Bridge) {
                    item.edgeB = parsedReference(
                        StableIdKind::Edge, ordinalB,
                        std::move(fingerprintB));
                } else if (ordinalB != 0 || !fingerprintB.empty()) {
                    throw std::runtime_error(
                        "fill operation has an unexpected second edge");
                }
                recipe.operations.push_back(std::move(item));
            } else {
                throw std::runtime_error("unknown recipe v2 record: " + kind);
            }
            if (!stream) throw std::runtime_error("malformed record");
        } catch (const std::exception& error) {
            throw std::runtime_error(
                path + ":" + std::to_string(lineNumber) + ": " +
                error.what());
        }
    }
    if (!validSha256(recipe.sourceSha256)) {
        throw std::runtime_error(
            "recipe v2 has an invalid source SHA-256: " + path);
    }
    return recipe;
}

int recipeFileVersion(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open recipe: " + path);
    std::string header;
    int version = 0;
    input >> header >> version;
    if (header != "weft-recipe" || (version != 1 && version != 2)) {
        throw std::runtime_error("unsupported weft recipe header: " + path);
    }
    return version;
}

}  // namespace weft
