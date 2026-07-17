#include "weft/fixture.hpp"
#include "weft/io/system.hpp"
#include "weft/model.hpp"
#include "weft/secure_core.hpp"
#include "weft/secure_reconnaissance.hpp"

#include "../core/src/secure_core_internal.hpp"
#include "../core/src/io/xcaf.hpp"

#include "test_temp_path.hpp"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <ShapeProcess.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {

#ifndef WEFT_SECURE_FIXTURE_DIR
#error "WEFT_SECURE_FIXTURE_DIR must identify the frozen fixture snapshot"
#endif

int failures = 0;

#define CHECK(condition)                                                  \
    do {                                                                  \
        if (!(condition)) {                                               \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__,          \
                        #condition);                                      \
            ++failures;                                                   \
        }                                                                 \
    } while (false)

const weft::CoedgeRecord* firstExactCoedge(const weft::BRepSnapshot& snapshot) {
    const auto found = std::find_if(
        snapshot.coedges.begin(), snapshot.coedges.end(),
        [](const weft::CoedgeRecord& coedge) {
            return !coedge.pcurveRepresentations.empty();
        });
    return found == snapshot.coedges.end() ? nullptr : &*found;
}

std::filesystem::path nativeFixturePath(std::string_view directory,
                                        std::string_view filename) {
    return std::filesystem::path(WEFT_SECURE_FIXTURE_DIR) / directory /
        filename;
}

TopoDS_Shape readNativeFixture(std::string_view directory,
                               std::string_view filename) {
    const std::filesystem::path path =
        nativeFixturePath(directory, filename);
    TopoDS_Shape sourceShape;
    BRep_Builder builder;
    if (!BRepTools::Read(sourceShape, path.string().c_str(), builder) ||
        sourceShape.IsNull()) {
        throw std::runtime_error("failed to read native repair fixture: " +
                                 path.string());
    }
    return sourceShape;
}

weft::ImportedModel deriveNativeRepair(
    const TopoDS_Shape& sourceShape, std::string_view sourceName) {
    weft::Model source = weft::indexShape(sourceShape);
    weft::secure_detail::ConservativeWorkingDerivation derivation =
        weft::secure_detail::deriveConservativeWorking(source);
    weft::Model working = weft::indexShape(derivation.shape);
    weft::SourceMetadata metadata;
    metadata.sourceName = std::string(sourceName);
    metadata.importerVersion = "weft-native-repair-fixture-0.1";
    return weft::secure_detail::buildImportedModel(
        std::move(source), std::move(working), std::move(metadata),
        weft::RepairProfile::Conservative, derivation.history,
        derivation.exactShapes, std::move(derivation.operations),
        std::move(derivation.parameterizationFlagChanges),
        std::move(derivation.orientationChanges),
        std::move(derivation.toleranceChanges));
}

weft::ImportedModel importNativeRepairFixture(std::string_view filename) {
    return weft::importBRepSecure(
        nativeFixturePath("derived", filename).string(),
        weft::RepairProfile::Conservative);
}

bool hasDiagnostic(const weft::ImportedModel& imported,
                   std::string_view code) {
    return std::any_of(
        imported.diagnostics.events.begin(), imported.diagnostics.events.end(),
        [code](const weft::ImportDiagnostic& diagnostic) {
            return diagnostic.code == code;
        });
}

void testConservativeIdentity(const std::filesystem::path& path) {
    const weft::ImportedModel imported =
        weft::importStepSecure(path.string(), weft::RepairProfile::Conservative);

    CHECK(imported.source != nullptr);
    CHECK(imported.working != nullptr);
    CHECK(imported.sourceEvaluator != nullptr);
    CHECK(imported.workingEvaluator != nullptr);
    if (!imported.source || !imported.working || !imported.sourceEvaluator) return;

    CHECK(imported.source->metadata.sourceName == path.filename().string());
    CHECK(imported.source->metadata.sourceSha256.size() == 64);
    CHECK(imported.source->metadata.sourceByteLength ==
          std::filesystem::file_size(path));
    CHECK(imported.source->metadata.lengthUnitMm.has_value());
    if (imported.source->metadata.lengthUnitMm) {
        CHECK(*imported.source->metadata.lengthUnitMm ==
              imported.source->snapshot.model.lengthUnitMm);
    }
    CHECK(imported.source->metadata.stepSchema.has_value());
    CHECK(std::find(imported.source->metadata.effectiveTranslatorConfiguration.begin(),
                    imported.source->metadata.effectiveTranslatorConfiguration.end(),
                    "XDE shape processing: disabled") !=
          imported.source->metadata.effectiveTranslatorConfiguration.end());

    CHECK(imported.repair.profile == weft::RepairProfile::Conservative);
    CHECK(imported.repair.sourceValid);
    CHECK(imported.repair.workingValid);
    CHECK(imported.repair.sourceTopologyComplete);
    CHECK(imported.repair.workingTopologyComplete);
    CHECK(imported.repair.correspondenceComplete);
    CHECK(imported.repair.identity);
    CHECK(imported.repair.meshable);
    CHECK(imported.correspondence.topologyComplete);
    CHECK(imported.repair.sourceShapeSha256.size() == 64);
    CHECK(imported.repair.sourceShapeSha256 ==
          imported.repair.workingShapeSha256);
    CHECK(imported.repair.toleranceChanges.empty());
    CHECK(imported.repair.representationChanges.empty());
    CHECK(imported.repair.topologyCardinalityChanges.empty());
    CHECK(!imported.repair.validationEvidence.empty());
    CHECK(std::all_of(imported.repair.validationEvidence.begin(),
                      imported.repair.validationEvidence.end(),
                      [](const weft::RepairValidationEvidence& evidence) {
                          return evidence.complete();
                      }));
    const auto evidence = [&](std::string_view code) {
        const auto found = std::find_if(
            imported.repair.validationEvidence.begin(),
            imported.repair.validationEvidence.end(),
            [code](const weft::RepairValidationEvidence& candidate) {
                return candidate.code == code;
            });
        return found == imported.repair.validationEvidence.end()
            ? nullptr
            : &*found;
    };
    for (std::string_view code : {
             "repair.topology.occurrences",
             "repair.topology.coedges",
             "repair.topology_occurrence_correspondence"}) {
        const weft::RepairValidationEvidence* checked = evidence(code);
        CHECK(checked != nullptr);
        if (checked) {
            CHECK(checked->expected > 0);
            CHECK(checked->checked == checked->expected);
        }
    }
    for (std::string_view code : {
             "repair.topology.assemblies", "repair.topology.instances"}) {
        const weft::RepairValidationEvidence* checked = evidence(code);
        CHECK(checked != nullptr);
        if (checked) {
            CHECK(checked->expected == 0);
            CHECK(checked->checked == 0);
            CHECK(checked->complete());
        }
    }
    CHECK(!imported.diagnostics.hasErrors());
    CHECK(!imported.source->snapshot.model.shape.IsSame(
        imported.working->snapshot.model.shape));
    CHECK(!imported.source->snapshot.model.shape.IsPartner(
        imported.working->snapshot.model.shape));
    CHECK(imported.repair.sourceFaces == imported.repair.workingFaces);
    CHECK(imported.repair.sourceEdges == imported.repair.workingEdges);
    CHECK(imported.repair.sourceFaces == 3);
    CHECK(!imported.correspondence.records.empty());
    CHECK(std::all_of(imported.correspondence.records.begin(),
                      imported.correspondence.records.end(),
                      [](const weft::CorrespondenceRecord& record) {
                          return record.sourceId.valid() &&
                                 record.workingIds.size() == 1 &&
                                 record.relation ==
                                     weft::CorrespondenceRelation::Identity;
                      }));

    const weft::TopologyAccount& sourceTopology =
        imported.source->snapshot.topology;
    const weft::TopologyAccountValidation sourceTopologyValidation =
        weft::validateTopologyAccount(sourceTopology);
    const weft::TopologyAccountValidation workingTopologyValidation =
        weft::validateTopologyAccount(imported.working->snapshot.topology);
    CHECK(sourceTopologyValidation.complete());
    CHECK(workingTopologyValidation.complete());
    const weft::TopologyAccountCheck* occurrenceCheck =
        sourceTopologyValidation.find("topology.occurrences");
    const weft::TopologyAccountCheck* coedgeCheck =
        sourceTopologyValidation.find("topology.coedges");
    CHECK(occurrenceCheck != nullptr);
    CHECK(coedgeCheck != nullptr);
    if (occurrenceCheck) {
        CHECK(occurrenceCheck->expected > 0);
        CHECK(occurrenceCheck->checked == occurrenceCheck->expected);
    }
    if (coedgeCheck) {
        CHECK(coedgeCheck->expected > 0);
        CHECK(coedgeCheck->checked == coedgeCheck->expected);
    }
    CHECK(sourceTopology.exactShapes.size() ==
          sourceTopology.occurrences.size());
    for (const auto& [sourceId, sourceShape] : sourceTopology.exactShapes) {
        const auto workingShape =
            imported.working->snapshot.topology.exactShapes.find(sourceId);
        CHECK(workingShape !=
              imported.working->snapshot.topology.exactShapes.end());
        if (workingShape !=
            imported.working->snapshot.topology.exactShapes.end()) {
            CHECK(!sourceShape.IsPartner(workingShape->second));
        }
    }
    CHECK(imported.correspondence.topologyOccurrenceRecords.size() ==
          sourceTopology.occurrences.size());
    CHECK(std::all_of(
        imported.correspondence.topologyOccurrenceRecords.begin(),
        imported.correspondence.topologyOccurrenceRecords.end(),
        [](const weft::CorrespondenceRecord& record) {
            return record.sourceId.valid() && record.workingIds.size() == 1 &&
                record.relation == weft::CorrespondenceRelation::Identity;
        }));

    const auto hasFailure = [](const weft::TopologyAccountValidation& validation,
                               std::string_view code) {
        return std::find(validation.failureCodes.begin(),
                         validation.failureCodes.end(), code) !=
            validation.failureCodes.end();
    };
    const weft::TopologyAccountValidation emptyValidation =
        weft::validateTopologyAccount({});
    CHECK(!emptyValidation.complete());
    CHECK(hasFailure(emptyValidation, "topology.occurrence_account.empty"));

    if (!sourceTopology.occurrences.empty()) {
        weft::TopologyAccount missingShape = sourceTopology;
        missingShape.exactShapes.erase(missingShape.occurrences.front().id);
        const weft::TopologyAccountValidation missingShapeValidation =
            weft::validateTopologyAccount(missingShape);
        CHECK(!missingShapeValidation.complete());
        CHECK(hasFailure(missingShapeValidation,
                         "topology.occurrence.shape_missing"));
        const weft::TopologyAccountCheck* checked =
            missingShapeValidation.find("topology.occurrences");
        CHECK(checked != nullptr);
        if (checked) {
            CHECK(checked->expected > 0);
            CHECK(checked->checked == checked->expected);
            CHECK(checked->failed > 0);
        }

        weft::TopologyAccount duplicateOccurrence = sourceTopology;
        duplicateOccurrence.occurrences.push_back(
            duplicateOccurrence.occurrences.front());
        const weft::TopologyAccountValidation duplicateValidation =
            weft::validateTopologyAccount(duplicateOccurrence);
        CHECK(!duplicateValidation.complete());
        CHECK(hasFailure(duplicateValidation,
                         "topology.occurrence.id_duplicate"));
    }
    if (!sourceTopology.coedges.empty()) {
        weft::TopologyAccount missingCoedge = sourceTopology;
        missingCoedge.coedges.pop_back();
        const weft::TopologyAccountValidation missingCoedgeValidation =
            weft::validateTopologyAccount(missingCoedge);
        CHECK(!missingCoedgeValidation.complete());
        CHECK(hasFailure(missingCoedgeValidation,
                         "topology.coedge.wire_coverage_invalid"));
    }

    const weft::CoedgeRecord* coedge =
        firstExactCoedge(imported.source->snapshot);
    CHECK(coedge != nullptr);
    if (!coedge) return;

    const auto domain = imported.sourceEvaluator->curveDomain(coedge->edgeId);
    const auto workingDomain =
        imported.workingEvaluator->curveDomain(coedge->edgeId);
    CHECK(static_cast<bool>(domain));
    CHECK(static_cast<bool>(workingDomain));
    if (!domain || !workingDomain || !domain.value->lower ||
        !domain.value->upper) {
        return;
    }
    CHECK(domain.value->lower == workingDomain.value->lower);
    CHECK(domain.value->upper == workingDomain.value->upper);
    const double parameter =
        (*domain.value->lower + *domain.value->upper) * 0.5;

    const auto curve =
        imported.sourceEvaluator->evaluateCurve(coedge->edgeId, parameter);
    const auto workingCurve =
        imported.workingEvaluator->evaluateCurve(coedge->edgeId, parameter);
    CHECK(static_cast<bool>(curve));
    CHECK(static_cast<bool>(workingCurve));
    if (curve && workingCurve) {
        CHECK(curve.value->position == workingCurve.value->position);
        CHECK(curve.value->firstDerivative ==
              workingCurve.value->firstDerivative);
    }
    const auto firstVertex = std::find_if(
        imported.source->snapshot.occurrences.begin(),
        imported.source->snapshot.occurrences.end(),
        [](const weft::TopologyOccurrence& occurrence) {
            return occurrence.id.kind == weft::StableIdKind::Vertex;
        });
    CHECK(firstVertex != imported.source->snapshot.occurrences.end());
    if (firstVertex != imported.source->snapshot.occurrences.end()) {
        const auto vertex = imported.sourceEvaluator->evaluateVertex(
            firstVertex->id);
        const auto workingVertex = imported.workingEvaluator->evaluateVertex(
            firstVertex->id);
        CHECK(static_cast<bool>(vertex));
        CHECK(static_cast<bool>(workingVertex));
        CHECK(vertex && vertex.value->vertexId == firstVertex->id);
        if (vertex && workingVertex) {
            CHECK(vertex.value->position == workingVertex.value->position);
        }
    }
    const weft::PcurveRef pcurveRef = coedge->pcurveRepresentations.front();
    const auto pcurve =
        imported.sourceEvaluator->evaluatePcurve(pcurveRef, parameter);
    const auto workingPcurve =
        imported.workingEvaluator->evaluatePcurve(pcurveRef, parameter);
    CHECK(static_cast<bool>(pcurve));
    CHECK(static_cast<bool>(workingPcurve));
    if (pcurve && workingPcurve) {
        CHECK(pcurve.value->uv == workingPcurve.value->uv);
        CHECK(pcurve.value->firstDerivative ==
              workingPcurve.value->firstDerivative);
    }
    const auto composed = imported.sourceEvaluator->evaluateCurveOnSurface(
        pcurveRef, parameter);
    const auto workingComposed =
        imported.workingEvaluator->evaluateCurveOnSurface(pcurveRef,
                                                          parameter);
    CHECK(static_cast<bool>(composed));
    CHECK(static_cast<bool>(workingComposed));
    if (composed) CHECK(composed.value->discrepancy <= 1e-7);
    if (composed && workingComposed) {
        CHECK(composed.value->curvePosition ==
              workingComposed.value->curvePosition);
        CHECK(composed.value->surfacePosition ==
              workingComposed.value->surfacePosition);
        CHECK(composed.value->discrepancy ==
              workingComposed.value->discrepancy);
    }
    if (pcurve) {
        const auto surface = imported.sourceEvaluator->evaluateSurface(
            coedge->faceId, pcurve.value->uv);
        const auto workingSurface = imported.workingEvaluator->evaluateSurface(
            coedge->faceId, pcurve.value->uv);
        CHECK(static_cast<bool>(surface));
        CHECK(static_cast<bool>(workingSurface));
        if (surface && workingSurface) {
            CHECK(surface.value->position == workingSurface.value->position);
            CHECK(surface.value->derivativeU ==
                  workingSurface.value->derivativeU);
            CHECK(surface.value->derivativeV ==
                  workingSurface.value->derivativeV);
            CHECK(surface.value->unitNormal ==
                  workingSurface.value->unitNormal);
        }
    }

    const auto invalid = imported.sourceEvaluator->evaluateCurve(
        {weft::StableIdKind::Face, 1}, 0.0);
    CHECK(!invalid);
    CHECK(invalid.failure.has_value());
    if (invalid.failure) {
        CHECK(invalid.failure->code == "geometry.edge_not_found");
    }
}

void testCompatibilityIsAudited(const std::filesystem::path& path) {
    const weft::ImportedModel imported =
        weft::importStepSecure(path.string(), weft::RepairProfile::Compatibility);
    const weft::ImportedModel conservative =
        weft::importStepSecure(path.string(), weft::RepairProfile::Conservative);
    CHECK(imported.source != nullptr);
    CHECK(imported.working != nullptr);
    CHECK(imported.repair.profile == weft::RepairProfile::Compatibility);
    CHECK(imported.repair.sourceTopologyComplete);
    CHECK(imported.repair.workingTopologyComplete);
    CHECK(!imported.correspondence.topologyComplete);
    CHECK(!imported.repair.correspondenceComplete);
    CHECK(!imported.repair.identity);
    CHECK(!imported.repair.meshable);
    CHECK(imported.source && imported.source->metadata.sourceSha256.size() == 64);
    if (imported.source && imported.working) {
        CHECK(!imported.source->snapshot.model.shape.IsSame(
            imported.working->snapshot.model.shape));
    }
    CHECK(imported.repair.sourceShapeSha256 ==
          conservative.repair.sourceShapeSha256);
    CHECK(std::any_of(imported.repair.operations.begin(),
                      imported.repair.operations.end(),
                      [](const weft::RepairOperation& operation) {
                          return operation.code ==
                                 "repair.compatibility_pipeline";
                      }));
    CHECK(std::any_of(
        imported.diagnostics.events.begin(), imported.diagnostics.events.end(),
        [](const weft::ImportDiagnostic& diagnostic) {
            return diagnostic.code ==
                "import.topology_correspondence.incomplete";
        }));
}

void testReadFailureIsNamed(const std::filesystem::path& path) {
    try {
        (void)weft::importStepSecure(
            (path.string() + ".does-not-exist"),
            weft::RepairProfile::Conservative);
        CHECK(false);
    } catch (const weft::SecureImportError& error) {
        CHECK(error.code() == "import.step.read_failed");
    }
}

void testSourceSnapshotPreventsPathReplacement() {
    const std::filesystem::path path = weft::test::uniqueTempPath(
        "weft_secure_core_snapshot", ".step");
    weft::writeStep(weft::makeFixture("cylinder"), path.string());

    weft::io::System system;
    weft::io::bootstrapIo(system);
    std::unique_ptr<weft::io::Reader> reader =
        system.createReader(weft::io::Format::Step);
    CHECK(reader != nullptr);
    if (!reader) return;
    const bool readSnapshot = reader->readFile(path.string());
    CHECK(readSnapshot);
    if (!readSnapshot) return;

    // Replace the path after parse. The already-read source bytes must remain
    // the bytes transferred and hashed by this reader.
    weft::writeStep(weft::makeFixture("box"), path.string());
    const weft::ImportedModel snapshot =
        reader->transferSecure(weft::RepairProfile::Conservative);
    const weft::ImportedModel fresh =
        weft::importStepSecure(path.string(), weft::RepairProfile::Conservative);
    CHECK(snapshot.repair.sourceFaces == 3);
    CHECK(fresh.repair.sourceFaces == 6);
    CHECK(snapshot.source->metadata.sourceSha256 !=
          fresh.source->metadata.sourceSha256);

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testNativeBRepSecureImport() {
    constexpr std::string_view kRepairArtifactSha256 =
        "c400ae0bb2102165a07673bb0df8ba28a17e998c84daf1ade7b4bc95a963ba1e";
    constexpr std::string_view kBoxArtifactSha256 =
        "bdf9bdd95e8b40373f26c929e855528deadd4808756537c4803e3d78062be461";
    const std::filesystem::path repairFixture = nativeFixturePath(
        "derived", "corrupt.edge.sameparameter_samerange_false.brep");
    const std::filesystem::path boxFixture = nativeFixturePath(
        "baselines", "baseline.pathology.box.brep");
    const std::filesystem::path path = weft::test::uniqueTempPath(
        "weft_secure_native_snapshot", ".brep");
    std::error_code filesystemError;
    std::filesystem::copy_file(
        repairFixture, path, std::filesystem::copy_options::overwrite_existing,
        filesystemError);
    CHECK(!filesystemError);
    if (filesystemError) return;

    weft::io::System system;
    weft::io::bootstrapIo(system);
    std::unique_ptr<weft::io::Reader> reader =
        system.createReader(weft::io::Format::Brep);
    CHECK(reader != nullptr);
    if (!reader) return;
    CHECK(reader->readFile(path.string()));

    // Replacing the path cannot change either the parsed shape or its recorded
    // source digest because both came from the reader's retained byte snapshot.
    filesystemError.clear();
    std::filesystem::copy_file(
        boxFixture, path, std::filesystem::copy_options::overwrite_existing,
        filesystemError);
    CHECK(!filesystemError);
    if (filesystemError) return;
    const weft::ImportedModel snapshot =
        reader->transferSecure(weft::RepairProfile::Conservative);
    const weft::ImportedModel fresh = weft::importBRepSecure(
        path.string(), weft::RepairProfile::Conservative);
    CHECK(snapshot.source != nullptr);
    CHECK(fresh.source != nullptr);
    CHECK(snapshot.repair.sourceFaces == 3);
    CHECK(fresh.repair.sourceFaces == 6);
    if (snapshot.source && fresh.source) {
        CHECK(snapshot.source->metadata.sourceSha256 ==
              kRepairArtifactSha256);
        CHECK(snapshot.source->metadata.sourceByteLength ==
              std::filesystem::file_size(repairFixture));
        CHECK(snapshot.source->metadata.importerVersion ==
              "weft-secure-brep-0.1");
        CHECK(!snapshot.source->metadata.lengthUnitMm.has_value());
        CHECK(!snapshot.source->metadata.stepSchema.has_value());
        CHECK(std::find(
                  snapshot.source->metadata.effectiveTranslatorConfiguration.begin(),
                  snapshot.source->metadata.effectiveTranslatorConfiguration.end(),
                  "OCCT ASCII B-rep stream parse from immutable byte snapshot") !=
              snapshot.source->metadata.effectiveTranslatorConfiguration.end());
        CHECK(fresh.source->metadata.sourceSha256 == kBoxArtifactSha256);
        CHECK(snapshot.source->metadata.sourceSha256 !=
              fresh.source->metadata.sourceSha256);
    }
    CHECK(snapshot.repair.workingValid);
    CHECK(snapshot.repair.meshable);
    CHECK(snapshot.repair.parameterizationFlagChanges.size() == 1);
    CHECK(hasDiagnostic(snapshot, "import.brep.length_unit_unspecified"));
    CHECK(fresh.repair.identity);
    CHECK(fresh.repair.meshable);

    const weft::ImportedModel resolvedUnit = weft::importBRepSecure(
        path.string(), weft::RepairProfile::Conservative, 1.0);
    CHECK(resolvedUnit.source != nullptr);
    CHECK(resolvedUnit.repair.identity);
    CHECK(resolvedUnit.repair.meshable);
    if (resolvedUnit.source) {
        CHECK(resolvedUnit.source->metadata.lengthUnitMm.has_value());
        CHECK(resolvedUnit.source->metadata.lengthUnitExplicitlyResolved);
        if (resolvedUnit.source->metadata.lengthUnitMm) {
            CHECK(*resolvedUnit.source->metadata.lengthUnitMm == 1.0);
            CHECK(resolvedUnit.source->snapshot.model.lengthUnitMm == 1.0);
        }
        CHECK(resolvedUnit.working != nullptr);
        if (resolvedUnit.working) {
            CHECK(resolvedUnit.working->snapshot.model.lengthUnitMm == 1.0);
        }
        CHECK(std::find(
                  resolvedUnit.source->metadata.effectiveTranslatorConfiguration
                      .begin(),
                  resolvedUnit.source->metadata.effectiveTranslatorConfiguration
                      .end(),
                  "length unit: explicitly resolved to millimetres by caller") !=
              resolvedUnit.source->metadata.effectiveTranslatorConfiguration
                  .end());
    }
    CHECK(hasDiagnostic(resolvedUnit, "import.brep.length_unit_resolved"));
    CHECK(!hasDiagnostic(resolvedUnit, "import.brep.length_unit_unspecified"));
    CHECK(std::any_of(
        resolvedUnit.repair.validationEvidence.begin(),
        resolvedUnit.repair.validationEvidence.end(),
        [](const weft::RepairValidationEvidence& evidence) {
            return evidence.code == "repair.shared_geometry_immutable" &&
                evidence.complete() && evidence.expected != 0;
        }));

    try {
        (void)weft::importBRepSecure(
            path.string(), weft::RepairProfile::Conservative, 0.0);
        CHECK(false);
    } catch (const weft::SecureImportError& error) {
        CHECK(error.code() == "import.brep.length_unit_invalid");
    }
    try {
        (void)weft::importBRepSecure(
            path.string(), weft::RepairProfile::Conservative, -2.0);
        CHECK(false);
    } catch (const weft::SecureImportError& error) {
        CHECK(error.code() == "import.brep.length_unit_invalid");
    }

    const weft::ImportedModel compatibility = weft::importBRepSecure(
        path.string(), weft::RepairProfile::Compatibility);
    CHECK(compatibility.source != nullptr);
    CHECK(compatibility.working != nullptr);
    CHECK(compatibility.repair.profile ==
          weft::RepairProfile::Compatibility);
    CHECK(!compatibility.repair.meshable);
    CHECK(std::any_of(
        compatibility.repair.operations.begin(),
        compatibility.repair.operations.end(),
        [](const weft::RepairOperation& operation) {
            return operation.code == "repair.compatibility_pipeline";
        }));
    if (compatibility.source && compatibility.working) {
        CHECK(compatibility.source->metadata.sourceSha256 ==
              kBoxArtifactSha256);
        CHECK(!compatibility.source->snapshot.model.shape.IsSame(
            compatibility.working->snapshot.model.shape));
    }

    const std::filesystem::path malformed = weft::test::uniqueTempPath(
        "weft_secure_native_malformed", ".brep");
    {
        std::ofstream output(malformed, std::ios::binary);
        output << "not an OCCT ASCII B-rep\n";
    }
    try {
        (void)weft::importBRepSecure(
            malformed.string(), weft::RepairProfile::Conservative);
        CHECK(false);
    } catch (const weft::SecureImportError& error) {
        CHECK(error.code() == "import.brep.read_failed");
    }

    std::unique_ptr<weft::io::Reader> failedStateReader =
        system.createReader(weft::io::Format::Brep);
    CHECK(failedStateReader != nullptr);
    if (failedStateReader) {
        const bool readValid = failedStateReader->readFile(path.string());
        CHECK(readValid);
        bool malformedAccepted = false;
        try {
            malformedAccepted = failedStateReader->readFile(
                malformed.string());
        } catch (const Standard_Failure&) {
            // A parser exception is allowed, but the retained shape must still
            // have been cleared before it escaped.
        }
        CHECK(!malformedAccepted);
        try {
            (void)failedStateReader->transferSecure(
                weft::RepairProfile::Conservative);
            CHECK(false);
        } catch (const weft::SecureImportError& error) {
            CHECK(error.code() == "import.brep.no_source_shape");
        }
    }

    try {
        (void)weft::importBRepSecure(
            malformed.string() + ".missing",
            weft::RepairProfile::Conservative);
        CHECK(false);
    } catch (const weft::SecureImportError& error) {
        CHECK(error.code() == "import.brep.read_failed");
    }

    std::unique_ptr<weft::io::Reader> unreadIges =
        system.createReader(weft::io::Format::Iges);
    CHECK(unreadIges != nullptr);
    if (unreadIges) {
        try {
            (void)unreadIges->transferSecure(
                weft::RepairProfile::Conservative);
            CHECK(false);
        } catch (const weft::SecureImportError& error) {
            CHECK(error.code() == "import.iges.no_source_shape");
        }
    }

    const std::filesystem::path igesPath = weft::test::uniqueTempPath(
        "weft_secure_iges_box", ".igs");
    {
        IGESControl_Controller::Init();
        IGESControl_Writer writer("MM", 1);
        CHECK(writer.AddShape(BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape()));
        writer.ComputeModel();
        CHECK(writer.Write(igesPath.string().c_str()));
    }
    const weft::ImportedModel iges = weft::importIgesSecure(
        igesPath.string(), weft::RepairProfile::Conservative);
    CHECK(iges.source != nullptr);
    CHECK(iges.working != nullptr);
    CHECK(iges.repair.identity);
    CHECK(iges.repair.meshable);
    CHECK(iges.repair.correspondenceComplete);
    if (iges.source) {
        CHECK(iges.source->metadata.sourceSha256.size() == 64);
        CHECK(iges.source->metadata.sourceByteLength ==
              std::filesystem::file_size(igesPath));
        CHECK(iges.source->metadata.importerVersion == "weft-secure-iges-0.1");
        CHECK(std::find(
                  iges.source->metadata.effectiveTranslatorConfiguration.begin(),
                  iges.source->metadata.effectiveTranslatorConfiguration.end(),
                  "IGES shape processing: disabled") !=
              iges.source->metadata.effectiveTranslatorConfiguration.end());
    }

    std::unique_ptr<weft::io::Reader> igesSnapshotReader =
        system.createReader(weft::io::Format::Iges);
    CHECK(igesSnapshotReader != nullptr);
    if (igesSnapshotReader) {
        CHECK(igesSnapshotReader->readFile(igesPath.string()));
        {
            IGESControl_Controller::Init();
            IGESControl_Writer writer("MM", 1);
            CHECK(writer.AddShape(
                BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()));
            writer.ComputeModel();
            CHECK(writer.Write(igesPath.string().c_str()));
        }
        const weft::ImportedModel igesSnapshot =
            igesSnapshotReader->transferSecure(
                weft::RepairProfile::Conservative);
        const weft::ImportedModel igesFresh = weft::importIgesSecure(
            igesPath.string(), weft::RepairProfile::Conservative);
        CHECK(igesSnapshot.source != nullptr);
        CHECK(igesFresh.source != nullptr);
        if (igesSnapshot.source && igesFresh.source) {
            CHECK(igesSnapshot.source->metadata.sourceSha256 ==
                  iges.source->metadata.sourceSha256);
            CHECK(igesSnapshot.source->metadata.sourceSha256 !=
                  igesFresh.source->metadata.sourceSha256);
        }
        CHECK(igesSnapshot.repair.identity);
        CHECK(igesFresh.repair.identity);
    }

    std::error_code igesIgnored;
    std::filesystem::remove(igesPath, igesIgnored);

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
    std::filesystem::remove(malformed, ignored);
}

void testMultipleFreeRootOccurrences() {
    const std::filesystem::path path = weft::test::uniqueTempPath(
        "weft_secure_core_multiple_free_roots", ".step");
    Handle(TDocStd_Document) document =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(document->Main());
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    CHECK(!shapeTool.IsNull());
    if (shapeTool.IsNull()) return;

    const TDF_Label first = shapeTool->AddShape(
        BRepPrimAPI_MakeBox(2.0, 3.0, 4.0).Shape(), false, false);
    const TDF_Label second = shapeTool->AddShape(
        BRepPrimAPI_MakeBox(gp_Pnt(10.0, 0.0, 0.0), 5.0, 6.0, 7.0)
            .Shape(),
        false, false);
    TDataStd_Name::Set(first, TCollection_ExtendedString("free_part_first"));
    TDataStd_Name::Set(second, TCollection_ExtendedString("free_part_second"));

    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    const ShapeProcess::OperationsFlags noShapeProcessing;
    writer.SetShapeProcessFlags(noShapeProcessing);
    writer.ChangeWriter().SetShapeProcessFlags(noShapeProcessing);
    const bool transferred = writer.Transfer(document, STEPControl_AsIs);
    CHECK(transferred);
    if (!transferred) return;
    const bool written =
        writer.Write(path.string().c_str()) == IFSelect_RetDone;
    CHECK(written);
    if (!written) return;

    const weft::ImportedModel imported = weft::importStepSecure(
        path.string(), weft::RepairProfile::Conservative);
    CHECK(imported.repair.identity);
    CHECK(imported.correspondence.topologyComplete);
    CHECK(imported.source != nullptr);
    if (!imported.source) return;

    const weft::Model& model = imported.source->snapshot.model;
    const weft::TopologyAccount& topology =
        imported.source->snapshot.topology;
    CHECK(model.assembly.size() == 2);
    CHECK(topology.assemblies.empty());
    CHECK(topology.assemblyRoots.empty());
    CHECK(topology.instances.size() == 2);
    CHECK(weft::validateTopologyAccount(topology).complete());
    std::set<weft::StableId> instanceIds;
    for (const weft::InstanceRecord& instance : topology.instances) {
        CHECK(instance.parentId ==
              (weft::StableId{weft::StableIdKind::Model, 1}));
        CHECK(instance.sourceComponent.empty());
        CHECK(!instance.sourceDefinition.empty());
        CHECK(instance.topologyRoots.size() == 1);
        CHECK(instanceIds.insert(instance.id).second);
    }
    for (const weft::AssemblyNode& node : model.assembly) {
        CHECK(node.parent == -1);
        CHECK(!node.isAssembly);
        CHECK(!node.exactUse.IsNull());
    }

    std::error_code ignored;
    std::filesystem::remove(path, ignored);
}

void testBoundedParameterizationRepair() {
    const weft::ImportedModel repaired = importNativeRepairFixture(
        "corrupt.edge.sameparameter_samerange_false.brep");
    CHECK(repaired.source != nullptr);
    CHECK(repaired.working != nullptr);
    CHECK(!repaired.repair.sourceValid);
    CHECK(repaired.repair.workingValid);
    CHECK(repaired.repair.correspondenceComplete);
    CHECK(repaired.correspondence.topologyComplete);
    CHECK(!repaired.repair.identity);
    CHECK(repaired.repair.meshable);
    CHECK(repaired.repair.sourceShapeSha256 !=
          repaired.repair.workingShapeSha256);
    CHECK(repaired.repair.toleranceChanges.empty());
    CHECK(repaired.repair.representationChanges.empty());
    CHECK(repaired.repair.topologyCardinalityChanges.empty());
    CHECK(repaired.repair.parameterizationFlagChanges.size() == 1);
    CHECK(std::any_of(
        repaired.repair.validationEvidence.begin(),
        repaired.repair.validationEvidence.end(),
        [](const weft::RepairValidationEvidence& evidence) {
            return evidence.code == "repair.shared_geometry_immutable" &&
                evidence.complete() && evidence.expected != 0;
        }));
    CHECK(std::any_of(
        repaired.repair.validationEvidence.begin(),
        repaired.repair.validationEvidence.end(),
        [](const weft::RepairValidationEvidence& evidence) {
            return evidence.code ==
                       "repair.representation_copy_on_write_reconciliation" &&
                evidence.complete() && evidence.expected == 0;
        }));
    CHECK(!repaired.diagnostics.hasErrors());
    CHECK(hasDiagnostic(repaired, "import.source.invalid"));
    CHECK(!hasDiagnostic(
        repaired, "import.repair.same_parameter_range_unproven"));
    if (repaired.source) {
        CHECK(repaired.source->metadata.sourceName ==
              "corrupt.edge.sameparameter_samerange_false.brep");
        CHECK(repaired.source->metadata.sourceSha256 ==
              "c400ae0bb2102165a07673bb0df8ba28a17e998c84daf1ade7b4bc95a963ba1e");
        CHECK(repaired.source->metadata.importerVersion ==
              "weft-secure-brep-0.1");
    }
    if (repaired.source && repaired.working) {
        CHECK(!repaired.source->snapshot.model.shape.IsPartner(
            repaired.working->snapshot.model.shape));
        for (const auto& [sourceId, sourceShape] :
             repaired.source->snapshot.topology.exactShapes) {
            const auto workingShape =
                repaired.working->snapshot.topology.exactShapes.find(
                    sourceId);
            CHECK(workingShape !=
                  repaired.working->snapshot.topology.exactShapes.end());
            if (workingShape !=
                repaired.working->snapshot.topology.exactShapes.end()) {
                CHECK(!sourceShape.IsPartner(workingShape->second));
            }
        }
    }

    if (repaired.source && repaired.working &&
        repaired.repair.parameterizationFlagChanges.size() == 1) {
        const weft::ParameterizationFlagChange& change =
            repaired.repair.parameterizationFlagChanges.front();
        CHECK(change.sourceEdge == change.workingEdge);
        CHECK(!change.sourceSameParameter);
        CHECK(!change.sourceSameRange);
        CHECK(change.workingSameParameter);
        CHECK(change.workingSameRange);
        CHECK(change.expectedPcurveUses > 0);
        CHECK(change.checkedPcurveUses == change.expectedPcurveUses);
        CHECK(change.maximumDiscrepancy == 0.0);
        CHECK(change.maximumDiscrepancy <= change.toleranceEnvelope);
        const weft::CorrespondenceRecord* correspondence =
            repaired.correspondence.find(change.sourceEdge);
        CHECK(correspondence != nullptr);
        if (correspondence) {
            CHECK(correspondence->relation ==
                  weft::CorrespondenceRelation::Modified);
            CHECK(correspondence->workingIds.size() == 1);
        }
        const auto evidence = std::find_if(
            repaired.repair.validationEvidence.begin(),
            repaired.repair.validationEvidence.end(),
            [](const weft::RepairValidationEvidence& item) {
                return item.code ==
                    "repair.same_parameter_range_reconciliation";
            });
        CHECK(evidence != repaired.repair.validationEvidence.end());
        if (evidence != repaired.repair.validationEvidence.end()) {
            CHECK(evidence->expected > 0);
            CHECK(evidence->complete());
        }
        const TopoDS_Edge sourceEdge = TopoDS::Edge(
            repaired.source->snapshot.model.edges(
                static_cast<int>(change.sourceEdge.ordinal)));
        const TopoDS_Edge workingEdge = TopoDS::Edge(
            repaired.working->snapshot.model.edges(
                static_cast<int>(change.workingEdge.ordinal)));
        CHECK(!BRep_Tool::SameParameter(sourceEdge));
        CHECK(!BRep_Tool::SameRange(sourceEdge));
        CHECK(BRep_Tool::SameParameter(workingEdge));
        CHECK(BRep_Tool::SameRange(workingEdge));

        const auto coedge = std::find_if(
            repaired.source->snapshot.coedges.begin(),
            repaired.source->snapshot.coedges.end(),
            [&](const weft::CoedgeRecord& item) {
                return item.edgeId == change.sourceEdge &&
                    !item.pcurveRepresentations.empty();
            });
        CHECK(coedge != repaired.source->snapshot.coedges.end());
        if (coedge != repaired.source->snapshot.coedges.end()) {
            const auto domain = repaired.workingEvaluator->curveDomain(
                change.workingEdge);
            CHECK(domain && domain.value->lower && domain.value->upper);
            if (domain && domain.value->lower && domain.value->upper) {
                const double parameter =
                    (*domain.value->lower + *domain.value->upper) * 0.5;
                const weft::PcurveRef reference =
                    coedge->pcurveRepresentations.front();
                const auto sourceEvaluation =
                    repaired.sourceEvaluator->evaluateCurveOnSurface(
                        reference, parameter);
                const auto workingEvaluation =
                    repaired.workingEvaluator->evaluateCurveOnSurface(
                        reference, parameter);
                CHECK(!sourceEvaluation);
                CHECK(sourceEvaluation.failure.has_value());
                if (sourceEvaluation.failure) {
                    CHECK(sourceEvaluation.failure->code ==
                          "geometry.curve_on_surface_parameter_unproven");
                }
                CHECK(static_cast<bool>(workingEvaluation));
                if (workingEvaluation) {
                    CHECK(workingEvaluation.value->discrepancy <=
                          change.toleranceEnvelope);
                }
            }
        }
    }

    CHECK(std::any_of(
        repaired.repair.operations.begin(), repaired.repair.operations.end(),
        [](const weft::RepairOperation& operation) {
            return operation.code == "repair.same_parameter_range_flags";
        }));
    CHECK(std::all_of(
        repaired.correspondence.topologyOccurrenceRecords.begin(),
        repaired.correspondence.topologyOccurrenceRecords.end(),
        [](const weft::CorrespondenceRecord& record) {
            return record.sourceId.valid() &&
                record.workingIds.size() == 1 &&
                record.relation == weft::CorrespondenceRelation::Modified;
        }));

    const TopoDS_Shape tamperedSourceShape = readNativeFixture(
        "derived", "corrupt.edge.sameparameter_samerange_false.brep");
    weft::Model tamperedSource = weft::indexShape(tamperedSourceShape);
    weft::secure_detail::ConservativeWorkingDerivation tamperedDerivation =
        weft::secure_detail::deriveConservativeWorking(tamperedSource);
    CHECK(tamperedDerivation.parameterizationFlagChanges.size() == 1);
    if (tamperedDerivation.parameterizationFlagChanges.size() == 1) {
        tamperedDerivation.parameterizationFlagChanges.front()
            .checkedPcurveUses = 0;
    }
    weft::Model tamperedWorking =
        weft::indexShape(tamperedDerivation.shape);
    weft::SourceMetadata tamperedMetadata;
    tamperedMetadata.sourceName = "tampered-parameterization-certificate";
    const weft::ImportedModel tampered =
        weft::secure_detail::buildImportedModel(
            std::move(tamperedSource), std::move(tamperedWorking),
            std::move(tamperedMetadata),
            weft::RepairProfile::Conservative, tamperedDerivation.history,
            tamperedDerivation.exactShapes,
            std::move(tamperedDerivation.operations),
            std::move(tamperedDerivation.parameterizationFlagChanges),
            std::move(tamperedDerivation.orientationChanges),
            std::move(tamperedDerivation.toleranceChanges));
    CHECK(tampered.repair.workingValid);
    CHECK(!tampered.repair.meshable);
    CHECK(hasDiagnostic(tampered, "import.repair.validation_incomplete"));

    TopoDS_Shape seamShape = readNativeFixture(
        "baselines", "face.cylinder.seam_dual_pcurve.brep");
    const weft::Model seamSource = weft::indexShape(seamShape);
    TopoDS_Edge seamEdge;
    for (int edgeIndex = 1;
         edgeIndex <= seamSource.edges.Extent() && seamEdge.IsNull();
         ++edgeIndex) {
        const TopoDS_Edge edge =
            TopoDS::Edge(seamSource.edges(edgeIndex));
        const int ownerIndex = seamSource.edgeToFaces.FindIndex(edge);
        if (ownerIndex <= 0) continue;
        for (const TopoDS_Shape& owner :
             seamSource.edgeToFaces.FindFromIndex(ownerIndex)) {
            if (owner.ShapeType() == TopAbs_FACE &&
                BRep_Tool::IsClosed(edge, TopoDS::Face(owner))) {
                seamEdge = edge;
                break;
            }
        }
    }
    CHECK(!seamEdge.IsNull());
    if (!seamEdge.IsNull()) {
        BRep_Builder builder;
        builder.SameRange(seamEdge, false);
        builder.SameParameter(seamEdge, false);
        const weft::ImportedModel seamRepaired = deriveNativeRepair(
            seamShape, "face.cylinder.seam_dual_pcurve.false_flags.brep");
        CHECK(!seamRepaired.repair.sourceValid);
        CHECK(seamRepaired.repair.workingValid);
        CHECK(seamRepaired.repair.meshable);
        CHECK(seamRepaired.correspondence.topologyComplete);
        CHECK(seamRepaired.repair.parameterizationFlagChanges.size() == 1);
        if (seamRepaired.repair.parameterizationFlagChanges.size() == 1) {
            const weft::ParameterizationFlagChange& change =
                seamRepaired.repair.parameterizationFlagChanges.front();
            CHECK(change.expectedPcurveUses == 2);
            CHECK(change.checkedPcurveUses == 2);
            CHECK(change.maximumDiscrepancy <= change.toleranceEnvelope);
        }
    }

    const weft::ImportedModel rangeMismatch = importNativeRepairFixture(
        "corrupt.edge.range_mismatch.brep");
    CHECK(!rangeMismatch.repair.workingValid);
    CHECK(!rangeMismatch.repair.meshable);
    CHECK(rangeMismatch.repair.identity);
    CHECK(rangeMismatch.repair.parameterizationFlagChanges.empty());
    CHECK(hasDiagnostic(
        rangeMismatch, "import.repair.same_parameter_range_unproven"));
    CHECK(hasDiagnostic(rangeMismatch, "import.working.non_meshable"));

    const weft::ImportedModel beyondTolerance = importNativeRepairFixture(
        "corrupt.edge.pcurve_disagreement_beyond_tolerance.brep");
    CHECK(!beyondTolerance.repair.sourceValid);
    CHECK(beyondTolerance.repair.workingValid);
    CHECK(beyondTolerance.repair.meshable);
    CHECK(!beyondTolerance.repair.identity);
    CHECK(beyondTolerance.repair.toleranceChanges.size() == 1);
    CHECK(beyondTolerance.repair.parameterizationFlagChanges.empty());
    CHECK(std::any_of(
        beyondTolerance.repair.operations.begin(),
        beyondTolerance.repair.operations.end(),
        [](const weft::RepairOperation& operation) {
            return operation.code == "repair.tolerance_envelope";
        }));
    if (beyondTolerance.repair.toleranceChanges.size() == 1) {
        const weft::ToleranceChange& change =
            beyondTolerance.repair.toleranceChanges.front();
        CHECK(change.after > change.before);
        CHECK(change.expectedPcurveUses > 0);
        CHECK(change.checkedPcurveUses == change.expectedPcurveUses);
        CHECK(change.maximumDiscrepancy == change.after);
        CHECK(change.maximumDiscrepancy > change.before);
    }
    const auto envelopeEvidence = std::find_if(
        beyondTolerance.repair.validationEvidence.begin(),
        beyondTolerance.repair.validationEvidence.end(),
        [](const weft::RepairValidationEvidence& item) {
            return item.code == "repair.tolerance_envelope_reconciliation";
        });
    CHECK(envelopeEvidence !=
          beyondTolerance.repair.validationEvidence.end());
    if (envelopeEvidence !=
        beyondTolerance.repair.validationEvidence.end()) {
        CHECK(envelopeEvidence->expected > 0);
        CHECK(envelopeEvidence->complete());
    }
    CHECK(!hasDiagnostic(beyondTolerance, "import.working.invalid"));
    CHECK(!hasDiagnostic(beyondTolerance, "import.working.non_meshable"));

    weft::Model tolTamperSource = weft::indexShape(readNativeFixture(
        "derived", "corrupt.edge.pcurve_disagreement_beyond_tolerance.brep"));
    weft::secure_detail::ConservativeWorkingDerivation tolTamperDerivation =
        weft::secure_detail::deriveConservativeWorking(tolTamperSource);
    CHECK(tolTamperDerivation.toleranceChanges.size() == 1);
    if (tolTamperDerivation.toleranceChanges.size() == 1) {
        tolTamperDerivation.toleranceChanges.front().checkedPcurveUses = 0;
    }
    weft::Model tolTamperWorking =
        weft::indexShape(tolTamperDerivation.shape);
    weft::SourceMetadata tolTamperMetadata;
    tolTamperMetadata.sourceName = "tampered-tolerance-certificate";
    const weft::ImportedModel tolTampered =
        weft::secure_detail::buildImportedModel(
            std::move(tolTamperSource), std::move(tolTamperWorking),
            std::move(tolTamperMetadata),
            weft::RepairProfile::Conservative, tolTamperDerivation.history,
            tolTamperDerivation.exactShapes,
            std::move(tolTamperDerivation.operations),
            std::move(tolTamperDerivation.parameterizationFlagChanges),
            std::move(tolTamperDerivation.orientationChanges),
            std::move(tolTamperDerivation.toleranceChanges));
    CHECK(tolTampered.repair.workingValid);
    CHECK(!tolTampered.repair.meshable);
    CHECK(hasDiagnostic(tolTampered, "import.repair.validation_incomplete"));

    const weft::ImportedModel withinTolerance = importNativeRepairFixture(
        "corrupt.edge.pcurve_disagreement_within_tolerance.brep");
    CHECK(withinTolerance.repair.sourceValid);
    CHECK(withinTolerance.repair.workingValid);
    CHECK(withinTolerance.repair.identity);
    CHECK(withinTolerance.repair.meshable);
    CHECK(withinTolerance.repair.toleranceChanges.empty());
    CHECK(!std::any_of(
        withinTolerance.repair.operations.begin(),
        withinTolerance.repair.operations.end(),
        [](const weft::RepairOperation& operation) {
            return operation.code == "repair.tolerance_envelope";
        }));
}

void testFaceAdjacencyOrientationRepair() {
    const weft::ImportedModel repaired = importNativeRepairFixture(
        "corrupt.orientation.inverted_shell_face.brep");
    CHECK(repaired.source != nullptr);
    CHECK(repaired.working != nullptr);
    CHECK(!repaired.repair.sourceValid);
    CHECK(repaired.repair.workingValid);
    CHECK(repaired.repair.correspondenceComplete);
    CHECK(repaired.correspondence.topologyComplete);
    CHECK(!repaired.repair.identity);
    CHECK(repaired.repair.meshable);
    CHECK(repaired.repair.sourceShapeSha256 !=
          repaired.repair.workingShapeSha256);
    CHECK(repaired.repair.toleranceChanges.empty());
    CHECK(repaired.repair.representationChanges.empty());
    CHECK(repaired.repair.topologyCardinalityChanges.empty());
    CHECK(!repaired.repair.orientationChanges.empty());
    CHECK(!repaired.diagnostics.hasErrors());
    CHECK(hasDiagnostic(repaired, "import.source.invalid"));
    CHECK(std::any_of(
        repaired.repair.operations.begin(), repaired.repair.operations.end(),
        [](const weft::RepairOperation& operation) {
            return operation.code == "repair.orientation_face_adjacency";
        }));
    const auto evidence = std::find_if(
        repaired.repair.validationEvidence.begin(),
        repaired.repair.validationEvidence.end(),
        [](const weft::RepairValidationEvidence& item) {
            return item.code == "repair.orientation_reconciliation";
        });
    CHECK(evidence != repaired.repair.validationEvidence.end());
    if (evidence != repaired.repair.validationEvidence.end()) {
        CHECK(evidence->expected > 0);
        CHECK(evidence->complete());
    }
    for (const weft::OrientationChange& change :
         repaired.repair.orientationChanges) {
        CHECK(change.sourceId == change.workingId);
        CHECK(change.sourceOrientation != change.workingOrientation);
        CHECK(change.manifoldEdgesChecked > 0);
        CHECK(change.polarityChecks > 0);
    }
    CHECK(std::all_of(
        repaired.correspondence.topologyOccurrenceRecords.begin(),
        repaired.correspondence.topologyOccurrenceRecords.end(),
        [](const weft::CorrespondenceRecord& record) {
            return record.sourceId.valid() &&
                record.workingIds.size() == 1 &&
                (record.relation == weft::CorrespondenceRelation::Identity ||
                 record.relation == weft::CorrespondenceRelation::Modified);
        }));

    weft::Model tamperedSource = weft::indexShape(readNativeFixture(
        "derived", "corrupt.orientation.inverted_shell_face.brep"));
    weft::secure_detail::ConservativeWorkingDerivation tamperedDerivation =
        weft::secure_detail::deriveConservativeWorking(tamperedSource);
    CHECK(!tamperedDerivation.orientationChanges.empty());
    if (!tamperedDerivation.orientationChanges.empty()) {
        tamperedDerivation.orientationChanges.front().manifoldEdgesChecked =
            0;
    }
    weft::Model tamperedWorking =
        weft::indexShape(tamperedDerivation.shape);
    weft::SourceMetadata tamperedMetadata;
    tamperedMetadata.sourceName = "tampered-orientation-certificate";
    const weft::ImportedModel tampered =
        weft::secure_detail::buildImportedModel(
            std::move(tamperedSource), std::move(tamperedWorking),
            std::move(tamperedMetadata),
            weft::RepairProfile::Conservative, tamperedDerivation.history,
            tamperedDerivation.exactShapes,
            std::move(tamperedDerivation.operations),
            std::move(tamperedDerivation.parameterizationFlagChanges),
            std::move(tamperedDerivation.orientationChanges),
            std::move(tamperedDerivation.toleranceChanges));
    CHECK(tampered.repair.workingValid);
    CHECK(!tampered.repair.meshable);
    CHECK(hasDiagnostic(tampered, "import.repair.validation_incomplete"));

    const TopoDS_Shape inverted = readNativeFixture(
        "derived", "corrupt.orientation.inverted_shell_face.brep");
    const TopoDS_Shape box = readNativeFixture(
        "baselines", "baseline.pathology.box.brep");
    BRep_Builder compoundBuilder;
    TopoDS_Compound compound;
    compoundBuilder.MakeCompound(compound);
    compoundBuilder.Add(compound, inverted);
    compoundBuilder.Add(compound, box);
    const std::filesystem::path multiBodyPath = weft::test::uniqueTempPath(
        "weft_secure_multi_body_orientation", ".brep");
    CHECK(BRepTools::Write(compound, multiBodyPath.string().c_str()));
    const weft::ImportedModel multiBody = weft::importBRepSecure(
        multiBodyPath.string(), weft::RepairProfile::Conservative);
    CHECK(multiBody.source != nullptr);
    CHECK(multiBody.working != nullptr);
    CHECK(multiBody.repair.sourceFaces > repaired.repair.sourceFaces);
    CHECK(multiBody.repair.sourceEdges > repaired.repair.sourceEdges);
    CHECK(!multiBody.repair.sourceValid);
    CHECK(multiBody.repair.workingValid);
    CHECK(multiBody.repair.meshable);
    CHECK(multiBody.repair.correspondenceComplete);
    CHECK(!multiBody.repair.identity);
    CHECK(!multiBody.repair.orientationChanges.empty());
    CHECK(std::any_of(
        multiBody.repair.operations.begin(), multiBody.repair.operations.end(),
        [](const weft::RepairOperation& operation) {
            return operation.code == "repair.orientation_face_adjacency";
        }));
    CHECK(std::any_of(
        multiBody.repair.validationEvidence.begin(),
        multiBody.repair.validationEvidence.end(),
        [](const weft::RepairValidationEvidence& evidence) {
            return evidence.code == "repair.shared_geometry_immutable" &&
                evidence.complete() && evidence.expected != 0;
        }));
    std::error_code multiBodyIgnored;
    std::filesystem::remove(multiBodyPath, multiBodyIgnored);

    const weft::ImportedModel wireInconsistent = importNativeRepairFixture(
        "corrupt.wire.inconsistent_orientation.brep");
    CHECK(wireInconsistent.repair.orientationChanges.empty());
    CHECK(!std::any_of(
        wireInconsistent.repair.operations.begin(),
        wireInconsistent.repair.operations.end(),
        [](const weft::RepairOperation& operation) {
            return operation.code == "repair.orientation_face_adjacency";
        }));
}

void testTotalReconnaissance(const std::filesystem::path& cylinderPath) {
    const weft::ImportedModel cylinder = weft::importStepSecure(
        cylinderPath.string(), weft::RepairProfile::Conservative);
    const weft::ReconnaissanceReport report = weft::reconnoitre(cylinder);
    CHECK(report.complete);
    CHECK(report.expectedSubjects == report.checkedSubjects);
    CHECK(report.records.size() ==
          static_cast<std::size_t>(cylinder.working->snapshot.model.faceCount() +
                                   cylinder.working->snapshot.model.edgeCount()));
    CHECK(report.regions.size() == 3);
    std::set<weft::StableId> classifiedSubjects;

    int planes = 0;
    int cylinders = 0;
    for (const weft::ExactGeometryClassification& record : report.records) {
        CHECK(record.subjectId.valid());
        CHECK(classifiedSubjects.insert(record.subjectId).second);
        CHECK(record.sourceSubjects.size() == 1);
        CHECK(!record.familyCode.empty());
        CHECK(record.familyCode != "kernel_specific");
        if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
        if (record.familyCode == "plane") {
            ++planes;
            CHECK(record.support ==
                  weft::GeometrySupportState::SupportedAnalyticTemplate);
            CHECK(record.strategyOrReasonCode == "strategy.surface.plane");
        }
        if (record.familyCode == "cylinder") {
            ++cylinders;
            CHECK(record.support ==
                  weft::GeometrySupportState::SupportedAnalyticTemplate);
            CHECK(record.strategyOrReasonCode == "strategy.surface.cylinder");
        }
    }
    CHECK(planes == 2);
    CHECK(cylinders == 1);

    const std::filesystem::path boxPath = weft::test::uniqueTempPath(
        "weft_secure_core_recon_box", ".step");
    weft::writeStep(weft::makeFixture("box"), boxPath.string());
    const weft::ImportedModel box = weft::importStepSecure(
        boxPath.string(), weft::RepairProfile::Conservative);
    const weft::ReconnaissanceReport boxReport = weft::reconnoitre(box);
    CHECK(boxReport.complete);
    const bool hasMissingStoredPcurve = std::any_of(
        box.working->snapshot.coedges.begin(), box.working->snapshot.coedges.end(),
        [](const weft::CoedgeRecord& coedge) {
            return coedge.pcurveRepresentations.empty();
        });
    CHECK(hasMissingStoredPcurve);
    for (const weft::ExactGeometryClassification& record : boxReport.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Surface) {
            CHECK(record.familyCode == "plane");
            CHECK(record.support ==
                  weft::GeometrySupportState::SupportedAnalyticTemplate);
        }
    }
    std::error_code ignored;
    std::filesystem::remove(boxPath, ignored);
}

}  // namespace

int main() {
    const std::filesystem::path path = weft::test::uniqueTempPath(
        "weft_secure_core_cylinder", ".step");
    try {
        weft::writeStep(weft::makeFixture("cylinder"), path.string());
        testConservativeIdentity(path);
        testCompatibilityIsAudited(path);
        testReadFailureIsNamed(path);
        testSourceSnapshotPreventsPathReplacement();
        testNativeBRepSecureImport();
        testMultipleFreeRootOccurrences();
        testBoundedParameterizationRepair();
        testFaceAdjacencyOrientationRepair();
        testTotalReconnaissance(path);
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    } catch (const std::exception& error) {
        std::printf("FAIL secure-core exception: %s\n", error.what());
        ++failures;
    }

    if (failures == 0) {
        std::printf("secure-core contract checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d secure-core failure(s)\n", failures);
    return EXIT_FAILURE;
}
