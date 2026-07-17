#include "weft/fixture.hpp"
#include "weft/io/system.hpp"
#include "weft/model.hpp"
#include "weft/secure_core.hpp"
#include "weft/secure_reconnaissance.hpp"

#include "test_temp_path.hpp"

#include <BRepPrimAPI_MakeBox.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <ShapeProcess.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <set>
#include <string>
#include <string_view>

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

const weft::CoedgeRecord* firstExactCoedge(const weft::BRepSnapshot& snapshot) {
    const auto found = std::find_if(
        snapshot.coedges.begin(), snapshot.coedges.end(),
        [](const weft::CoedgeRecord& coedge) {
            return !coedge.pcurveRepresentations.empty();
        });
    return found == snapshot.coedges.end() ? nullptr : &*found;
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
    CHECK(reader->readFile(path.string()));

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
        testMultipleFreeRootOccurrences();
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
