#include "weft/fixture.hpp"
#include "weft/io/system.hpp"
#include "weft/model.hpp"
#include "weft/secure_core.hpp"
#include "weft/secure_reconnaissance.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <set>
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
    CHECK(imported.repair.correspondenceComplete);
    CHECK(imported.repair.identity);
    CHECK(imported.repair.meshable);
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
                          return evidence.expected > 0 && evidence.complete();
                      }));
    CHECK(!imported.diagnostics.hasErrors());
    CHECK(imported.source->snapshot.model.shape.IsSame(
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

    const weft::CoedgeRecord* coedge =
        firstExactCoedge(imported.source->snapshot);
    CHECK(coedge != nullptr);
    if (!coedge) return;

    const auto domain = imported.sourceEvaluator->curveDomain(coedge->edgeId);
    CHECK(static_cast<bool>(domain));
    if (!domain || !domain.value->lower || !domain.value->upper) return;
    const double parameter =
        (*domain.value->lower + *domain.value->upper) * 0.5;

    const auto curve =
        imported.sourceEvaluator->evaluateCurve(coedge->edgeId, parameter);
    CHECK(static_cast<bool>(curve));
    const weft::PcurveRef pcurveRef = coedge->pcurveRepresentations.front();
    const auto pcurve =
        imported.sourceEvaluator->evaluatePcurve(pcurveRef, parameter);
    CHECK(static_cast<bool>(pcurve));
    const auto composed = imported.sourceEvaluator->evaluateCurveOnSurface(
        pcurveRef, parameter);
    CHECK(static_cast<bool>(composed));
    if (composed) CHECK(composed.value->discrepancy <= 1e-7);
    if (pcurve) {
        const auto surface = imported.sourceEvaluator->evaluateSurface(
            coedge->faceId, pcurve.value->uv);
        CHECK(static_cast<bool>(surface));
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
