#include "weft/canonical_boundary.hpp"
#include "weft/fixture.hpp"
#include "weft/model.hpp"

#include "test_temp_path.hpp"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
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

weft::IntervalSolution intervalsFor(const weft::ImportedModel& imported,
                                    bool omitLast = false) {
    weft::IntervalProblem problem;
    const std::size_t count = imported.working->snapshot.edgeTopology.size();
    for (std::size_t index = 0; index < count; ++index) {
        if (omitLast && index + 1 == count) break;
        const weft::StableId edge =
            imported.working->snapshot.edgeTopology[index].id;
        problem.variables.push_back(
            {{weft::StableIdKind::Boundary, edge.ordinal}, 4.0, 1, false});
    }
    const weft::IntervalSolveResult solved = weft::solveIntervals(problem);
    CHECK(solved);
    return solved ? *solved.solution : weft::IntervalSolution{};
}

void verifyCanonicalModel(const std::string& fixtureName,
                          const std::filesystem::path& path,
                          bool expectPlanarProjection,
                          bool expectStoredPcurve) {
    weft::writeStep(weft::makeFixture(fixtureName), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance,
                                       intervalsFor(imported));
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("canonical failure: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return;
    }

    const weft::CanonicalBoundarySet& set = *built.value;
    CHECK(set.validation.complete());
    CHECK(set.boundaries.size() ==
          imported.working->snapshot.edgeTopology.size());
    CHECK(set.validation.expectedEdges == set.validation.checkedEdges);
    CHECK(set.validation.expectedSamples == set.validation.checkedSamples);
    CHECK(set.validation.expectedUvUses == set.validation.checkedUvUses);

    bool sawPlanarProjection = false;
    bool sawStoredPcurve = false;
    for (const weft::EdgeTopologyRecord& topology :
         imported.working->snapshot.edgeTopology) {
        const weft::CanonicalBoundary* boundary = set.find(topology.id);
        CHECK(boundary != nullptr);
        if (!boundary) continue;
        CHECK((boundary->boundaryId ==
               weft::StableId{weft::StableIdKind::Boundary,
                              topology.id.ordinal}));
        CHECK(!boundary->samples.empty());
        if (topology.lowerVertex) {
            CHECK(boundary->samples.front().canonicalVertexIndex ==
                  topology.lowerVertex->ordinal - 1U);
        }
        if (!boundary->closed && topology.upperVertex) {
            CHECK(boundary->samples.back().canonicalVertexIndex ==
                  topology.upperVertex->ordinal - 1U);
        }
        for (std::size_t index = 0; index < boundary->samples.size(); ++index) {
            const weft::CanonicalBoundarySample& sample =
                boundary->samples[index];
            CHECK(sample.id.valid());
            CHECK(sample.id.boundary == boundary->boundaryId);
            CHECK(sample.id.ordinal == index);
            CHECK(sample.workingEdge == topology.id);
            CHECK(sample.sourceEdge.has_value());
            CHECK(!sample.faceUses.empty());
            CHECK(sample.canonicalVertexIndex < set.canonicalVertexCount);
            for (const weft::CoedgeUvUse& use : sample.faceUses) {
                CHECK(use.sourceFace.has_value());
                CHECK(std::isfinite(use.uv[0]));
                CHECK(std::isfinite(use.uv[1]));
                CHECK(std::isfinite(use.liftedUv[0]));
                CHECK(std::isfinite(use.liftedUv[1]));
                CHECK(std::isfinite(
                    use.measuredCurveOnSurfaceDiscrepancy));
                CHECK(use.measuredCurveOnSurfaceDiscrepancy <= 1e-3);
                sawPlanarProjection = sawPlanarProjection ||
                    use.mappingKind ==
                        weft::BoundaryUvMappingKind::DerivedPlanarProjection;
                sawStoredPcurve = sawStoredPcurve ||
                    use.mappingKind ==
                        weft::BoundaryUvMappingKind::StoredPcurve;
            }
        }
    }
    CHECK(!expectPlanarProjection || sawPlanarProjection);
    CHECK(!expectStoredPcurve || sawStoredPcurve);

    const weft::CanonicalBoundaryBuildResult missing =
        weft::buildCanonicalBoundaries(imported, reconnaissance,
                                       intervalsFor(imported, true));
    CHECK(!missing);
    CHECK(missing.failure &&
          missing.failure->code == "boundary.interval_count_missing");
}

std::vector<std::array<double, 3>> ring(double z, double phase,
                                        double direction,
                                        int jitterIndex = -1) {
    constexpr std::uint32_t count = 12;
    constexpr double twoPi = 6.283185307179586476925286766559;
    std::vector<std::array<double, 3>> result;
    for (std::uint32_t index = 0; index < count; ++index) {
        double angle = phase + direction * static_cast<double>(index) *
            twoPi / static_cast<double>(count);
        if (static_cast<int>(index) == jitterIndex) angle += 0.4;
        result.push_back({std::cos(angle), std::sin(angle), z});
    }
    return result;
}

void testAzimuthRegistration() {
    constexpr double twoPi = 6.283185307179586476925286766559;
    const auto lower = ring(0.0, 0.0, 1.0);
    const auto aligned = weft::azimuthRegistration(
        lower, ring(1.0, 0.0, 1.0));
    CHECK(aligned);
    if (aligned) {
        for (std::uint32_t index = 0; index < 12; ++index) {
            CHECK((*aligned.permutation)[index] == index);
        }
    }

    const auto rotated = weft::azimuthRegistration(
        lower, ring(1.0, 3.0 * twoPi / 12.0, 1.0));
    CHECK(rotated);
    if (rotated) {
        CHECK((*rotated.permutation)[0] == 9);
        CHECK((*rotated.permutation)[3] == 0);
    }

    const auto reflected = weft::azimuthRegistration(
        lower, ring(1.0, 0.0, -1.0));
    CHECK(!reflected);
    CHECK(reflected.failure &&
          reflected.failure->code ==
              "boundary.azimuth_registration_failed");

    const auto nonUniform = weft::azimuthRegistration(
        lower, ring(1.0, 0.0, 1.0, 4));
    CHECK(!nonUniform);

    const auto invalidReference = weft::azimuthRegistration(
        ring(0.0, 0.0, 1.0, 4), ring(1.0, 0.0, 1.0));
    CHECK(!invalidReference);
}

void testPartialPeriodicCurveIsOpen(const std::filesystem::path& path) {
    const gp_Pnt centre(0.0, 0.0, 0.0);
    const gp_Pnt start(10.0, 0.0, 0.0);
    const gp_Pnt middle(7.0710678118654755, 7.0710678118654755, 0.0);
    const gp_Pnt end(0.0, 10.0, 0.0);
    const TopoDS_Edge first = BRepBuilderAPI_MakeEdge(centre, start);
    const Handle(Geom_TrimmedCurve) arc =
        GC_MakeArcOfCircle(start, middle, end).Value();
    const TopoDS_Edge curved = BRepBuilderAPI_MakeEdge(arc);
    const TopoDS_Edge last = BRepBuilderAPI_MakeEdge(end, centre);
    BRepBuilderAPI_MakeWire wire;
    wire.Add(first);
    wire.Add(curved);
    wire.Add(last);
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(wire.Wire());
    weft::writeStep(face, path.string());

    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance,
                                       intervalsFor(imported));
    CHECK(built);
    if (!built) return;

    const weft::ExactGeometryClassification* circle = nullptr;
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Curve &&
            record.familyCode == "circle") {
            circle = &record;
            break;
        }
    }
    CHECK(circle != nullptr);
    if (!circle) return;
    CHECK(!circle->parameterDomains.empty());
    CHECK(circle->parameterDomains.front().periodic);
    const weft::CanonicalBoundary* boundary =
        built.value->find(circle->subjectId);
    CHECK(boundary != nullptr);
    if (!boundary) return;
    CHECK(!boundary->closed);
    CHECK(boundary->samples.size() == 5);
    CHECK(boundary->samples.front().canonicalVertexIndex !=
          boundary->samples.back().canonicalVertexIndex);
}

}  // namespace

int main() {
    const std::filesystem::path boxPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_box", ".step");
    const std::filesystem::path cylinderPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_cylinder", ".step");
    const std::filesystem::path partialArcPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_partial_arc", ".step");
    try {
        verifyCanonicalModel("box", boxPath, true, false);
        verifyCanonicalModel("cylinder", cylinderPath, true, true);
        testAzimuthRegistration();
        testPartialPeriodicCurveIsOpen(partialArcPath);
    } catch (const std::exception& error) {
        std::printf("FAIL canonical-boundary exception: %s\n", error.what());
        ++failures;
    }
    std::error_code ignored;
    std::filesystem::remove(boxPath, ignored);
    std::filesystem::remove(cylinderPath, ignored);
    std::filesystem::remove(partialArcPath, ignored);

    if (failures == 0) {
        std::printf("canonical boundary checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d canonical boundary failure(s)\n", failures);
    return EXIT_FAILURE;
}
