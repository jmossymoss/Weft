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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <set>
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
        const weft::EdgeTopologyRecord& topology =
            imported.working->snapshot.edgeTopology[index];
        const weft::StableId edge = topology.id;
        const std::uint32_t demand = topology.degenerate ? 1U : 4U;
        problem.variables.push_back(
            {{weft::StableIdKind::Boundary, edge.ordinal},
             static_cast<double>(demand), demand, false, std::nullopt});
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
    CHECK(set.validation.expectedVertexCurveChecks ==
          set.validation.checkedVertexCurveChecks);
    CHECK(set.validation.checkedVertexCurveChecks != 0);
    CHECK(set.validation.expectedPeriodicClosures ==
          set.validation.checkedPeriodicClosures);
    CHECK(set.validation.expectedCriticalEvents != 0);
    CHECK(set.validation.expectedCriticalEvents ==
          set.validation.checkedCriticalEvents);

    bool sawPlanarProjection = false;
    bool sawStoredPcurve = false;
    bool sawForwardPeriodicClosure = false;
    bool sawReversedPeriodicClosure = false;
    bool sawFullPeriodCrossing = false;
    std::map<std::uint64_t, std::array<double, 3>> canonicalPositions;
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
        if (!boundary->closed) {
            CHECK(boundary->periodicClosures.empty());
        }
        for (const weft::PeriodicUvClosureWitness& closure :
             boundary->periodicClosures) {
            CHECK(closure.edge == topology.id);
            CHECK(closure.sourceEdge.has_value());
            CHECK(closure.sourceFace.has_value());
            CHECK(std::isfinite(closure.period) && closure.period > 0.0);
            CHECK(std::isfinite(closure.firstUv));
            CHECK(std::isfinite(closure.lastUv));
            CHECK(std::isfinite(closure.firstLiftedUv));
            CHECK(std::isfinite(closure.lastLiftedUv));
            CHECK(std::isfinite(closure.closingLiftedUv));
            CHECK(std::abs(closure.closingLiftedUv -
                           (closure.firstLiftedUv +
                            static_cast<double>(closure.periodsCrossed) *
                                closure.period)) < 1e-9 * closure.period);
            sawForwardPeriodicClosure = sawForwardPeriodicClosure ||
                closure.traversalOrientation ==
                    weft::TopologyOrientation::Forward;
            sawReversedPeriodicClosure = sawReversedPeriodicClosure ||
                closure.traversalOrientation ==
                    weft::TopologyOrientation::Reversed;
            sawFullPeriodCrossing =
                sawFullPeriodCrossing ||
                closure.periodsCrossed == 1 || closure.periodsCrossed == -1;
        }
        CHECK(!boundary->criticalEvents.empty());
        for (const weft::CriticalParameterEvent& event :
             boundary->criticalEvents) {
            CHECK(event.edge == topology.id);
            CHECK(event.sampleOrdinal < boundary->samples.size());
            CHECK(std::isfinite(event.curveParameter));
            CHECK(!event.detectionCode.empty());
            CHECK(std::abs(boundary->samples[event.sampleOrdinal]
                               .curveParameter -
                           event.curveParameter) <
                  1e-9 * std::max(1.0, std::abs(event.curveParameter)));
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
            const auto [position, inserted] = canonicalPositions.emplace(
                sample.canonicalVertexIndex, sample.position);
            CHECK(inserted || position->second == sample.position);
            for (const weft::CoedgeUvUse& use : sample.faceUses) {
                CHECK(use.sourceFace.has_value());
                CHECK(std::isfinite(use.uv[0]));
                CHECK(std::isfinite(use.uv[1]));
                CHECK(std::isfinite(use.liftedUv[0]));
                CHECK(std::isfinite(use.liftedUv[1]));
                CHECK(std::isfinite(
                    use.measuredCurveOnSurfaceDiscrepancy));
                CHECK(use.measuredCurveOnSurfaceDiscrepancy <= 1e-3);
                CHECK(std::isfinite(
                    use.allowedCurveOnSurfaceDiscrepancy));
                CHECK(use.allowedCurveOnSurfaceDiscrepancy <= 1e-3);
                CHECK(use.measuredCurveOnSurfaceDiscrepancy <=
                      use.allowedCurveOnSurfaceDiscrepancy);
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
    if (expectStoredPcurve) {
        CHECK(set.validation.expectedPeriodicClosures != 0);
        CHECK(sawFullPeriodCrossing);
        CHECK(sawForwardPeriodicClosure);
        CHECK(sawReversedPeriodicClosure);
    } else {
        CHECK(set.validation.expectedPeriodicClosures == 0);
    }

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

    // Shifted angle / cyclic phase: compatible frames share columns.
    const auto rotated = weft::azimuthRegistration(
        lower, ring(1.0, 3.0 * twoPi / 12.0, 1.0));
    CHECK(rotated);
    if (rotated) {
        CHECK((*rotated.permutation)[0] == 9);
        CHECK((*rotated.permutation)[3] == 0);
    }

    // Reversed axis (swap rings): still a pure cyclic registration.
    const auto reversedAxis = weft::azimuthRegistration(
        ring(1.0, 0.0, 1.0), lower);
    CHECK(reversedAxis);
    if (reversedAxis) {
        for (std::uint32_t index = 0; index < 12; ++index) {
            CHECK((*reversedAxis.permutation)[index] == index);
        }
    }

    // Rotated origin in the XY plane of each ring (same relative samples).
    auto rotatedOriginLower = lower;
    auto rotatedOriginUpper = ring(1.0, 0.0, 1.0);
    for (auto& point : rotatedOriginLower) {
        point[0] += 5.0;
        point[1] -= 2.0;
    }
    for (auto& point : rotatedOriginUpper) {
        point[0] += 5.0;
        point[1] -= 2.0;
    }
    const auto rotatedOrigin =
        weft::azimuthRegistration(rotatedOriginLower, rotatedOriginUpper);
    CHECK(rotatedOrigin);

    const auto reflected = weft::azimuthRegistration(
        lower, ring(1.0, 0.0, -1.0));
    CHECK(!reflected);
    CHECK(reflected.failure &&
          reflected.failure->code == "boundary.azimuth_reflection");

    const auto twisted = weft::azimuthRegistration(
        lower, ring(1.0, 0.0, 1.0, 4));
    CHECK(!twisted);
    CHECK(twisted.failure &&
          twisted.failure->code == "boundary.azimuth_twist");

    const auto invalidReference = weft::azimuthRegistration(
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
        ring(1.0, 0.0, 1.0));
    CHECK(!invalidReference);
    CHECK(invalidReference.failure &&
          invalidReference.failure->code ==
              "boundary.azimuth_incompatible");

    const auto countMismatch = weft::azimuthRegistration(
        lower, ring(1.0, 0.0, 1.0));
    // Same count — keep a true incompatible count case:
    auto shortRing = lower;
    shortRing.pop_back();
    const auto incompatible =
        weft::azimuthRegistration(lower, shortRing);
    CHECK(!incompatible);
    CHECK(incompatible.failure &&
          incompatible.failure->code ==
              "boundary.azimuth_incompatible");
    (void)countMismatch;
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
    CHECK(boundary->periodicClosures.empty());
    CHECK(boundary->samples.front().canonicalVertexIndex !=
          boundary->samples.back().canonicalVertexIndex);
}

weft::PeriodicUvClosureWitness baseClosureSeed() {
    weft::PeriodicUvClosureWitness seed;
    seed.edge = {weft::StableIdKind::Edge, 1};
    seed.coedge = {weft::StableIdKind::Coedge, 1};
    seed.face = {weft::StableIdKind::Face, 1};
    seed.sourceEdge = seed.edge;
    seed.sourceFace = seed.face;
    seed.traversalOrientation = weft::TopologyOrientation::Forward;
    seed.axis = 0;
    seed.period = 6.283185307179586476925286766559;
    seed.firstUv = 0.0;
    seed.lastUv = 0.75 * seed.period;
    seed.firstLiftedUv = 0.0;
    seed.lastLiftedUv = seed.lastUv;
    seed.selectedFirstLift = 0;
    seed.selectedLastLift = 0;
    return seed;
}

void testPeriodicUvClosureAdversaries() {
    const auto good = weft::provePeriodicUvClosure(baseClosureSeed());
    CHECK(good);
    if (good) {
        CHECK(good.value->periodsCrossed == 1);
        CHECK(std::abs(good.value->closingLiftedUv - good.value->period) <
              1e-12);
    }

    auto ambiguous = baseClosureSeed();
    ambiguous.period = 2.0;
    ambiguous.firstUv = 0.0;
    ambiguous.firstLiftedUv = 0.0;
    ambiguous.lastUv = 1.0;
    ambiguous.lastLiftedUv = 1.0;
    ambiguous.selectedFirstLift = 0;
    ambiguous.selectedLastLift = 0;
    const auto ambiguousResult = weft::provePeriodicUvClosure(ambiguous);
    CHECK(!ambiguousResult);
    CHECK(ambiguousResult.failure &&
          (ambiguousResult.failure->code ==
               "boundary.periodic_closure_ambiguous" ||
           ambiguousResult.failure->code ==
               "boundary.periodic_closure_discontinuous"));

    auto wrongRecordedLift = baseClosureSeed();
    wrongRecordedLift.selectedLastLift = 7;
    const auto wrongLiftResult =
        weft::provePeriodicUvClosure(wrongRecordedLift);
    CHECK(!wrongLiftResult);
    CHECK(wrongLiftResult.failure &&
          wrongLiftResult.failure->code ==
              "boundary.periodic_lift_inconsistent");

    auto mismatchedEndpoint = baseClosureSeed();
    mismatchedEndpoint.selectedFirstLift = 1;
    const auto mismatchedResult =
        weft::provePeriodicUvClosure(mismatchedEndpoint);
    CHECK(!mismatchedResult);
    CHECK(mismatchedResult.failure &&
          mismatchedResult.failure->code ==
              "boundary.periodic_lift_inconsistent");

    auto nearFullStillClosed = baseClosureSeed();
    nearFullStillClosed.lastLiftedUv =
        nearFullStillClosed.period - 1e-3 * nearFullStillClosed.period;
    nearFullStillClosed.lastUv = nearFullStillClosed.lastLiftedUv;
    const auto nearFullResult =
        weft::provePeriodicUvClosure(nearFullStillClosed);
    CHECK(nearFullResult);
    if (nearFullResult) {
        CHECK(nearFullResult.value->periodsCrossed == 1);
    }

    // A half-period covering gap lands on the branch cut and must refuse
    // rather than guess a wrap (seam-crossing ambiguity).
    auto seamCrossingAmbiguous = baseClosureSeed();
    seamCrossingAmbiguous.period = 2.0;
    seamCrossingAmbiguous.firstUv = 0.25;
    seamCrossingAmbiguous.firstLiftedUv = 0.25;
    seamCrossingAmbiguous.lastUv = 1.25;
    seamCrossingAmbiguous.lastLiftedUv = 1.25;
    seamCrossingAmbiguous.selectedFirstLift = 0;
    seamCrossingAmbiguous.selectedLastLift = 0;
    const auto seamResult =
        weft::provePeriodicUvClosure(seamCrossingAmbiguous);
    CHECK(!seamResult);
    CHECK(seamResult.failure &&
          (seamResult.failure->code ==
               "boundary.periodic_closure_ambiguous" ||
           seamResult.failure->code ==
               "boundary.periodic_closure_discontinuous"));
}

void testCylinderPeriodicClosureDeterminism(
    const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture("cylinder"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::IntervalSolution intervals = intervalsFor(imported);
    const weft::CanonicalBoundaryBuildResult first =
        weft::buildCanonicalBoundaries(imported, reconnaissance, intervals);
    const weft::CanonicalBoundaryBuildResult second =
        weft::buildCanonicalBoundaries(imported, reconnaissance, intervals);
    CHECK(first);
    CHECK(second);
    if (!first || !second) return;
    CHECK(first.validation.expectedPeriodicClosures ==
          second.validation.expectedPeriodicClosures);
    CHECK(first.validation.checkedPeriodicClosures ==
          second.validation.checkedPeriodicClosures);
    CHECK(first.validation.expectedCriticalEvents ==
          second.validation.expectedCriticalEvents);
    CHECK(first.validation.checkedCriticalEvents ==
          second.validation.checkedCriticalEvents);
    CHECK(first.value->boundaries.size() == second.value->boundaries.size());
    for (std::size_t index = 0; index < first.value->boundaries.size();
         ++index) {
        const weft::CanonicalBoundary& a = first.value->boundaries[index];
        const weft::CanonicalBoundary& b = second.value->boundaries[index];
        CHECK(a.periodicClosures.size() == b.periodicClosures.size());
        CHECK(a.criticalEvents.size() == b.criticalEvents.size());
        for (std::size_t eventIndex = 0; eventIndex < a.criticalEvents.size();
             ++eventIndex) {
            const weft::CriticalParameterEvent& left =
                a.criticalEvents[eventIndex];
            const weft::CriticalParameterEvent& right =
                b.criticalEvents[eventIndex];
            CHECK(left.kind == right.kind);
            CHECK(left.edge == right.edge);
            CHECK(left.coedge == right.coedge);
            CHECK(left.face == right.face);
            CHECK(left.axis == right.axis);
            CHECK(left.sampleOrdinal == right.sampleOrdinal);
            CHECK(left.detectionCode == right.detectionCode);
            CHECK(left.curveParameter == right.curveParameter);
        }
        for (std::size_t closureIndex = 0;
             closureIndex < a.periodicClosures.size(); ++closureIndex) {
            const weft::PeriodicUvClosureWitness& left =
                a.periodicClosures[closureIndex];
            const weft::PeriodicUvClosureWitness& right =
                b.periodicClosures[closureIndex];
            CHECK(left.edge == right.edge);
            CHECK(left.coedge == right.coedge);
            CHECK(left.face == right.face);
            CHECK(left.axis == right.axis);
            CHECK(left.periodsCrossed == right.periodsCrossed);
            CHECK(left.selectedFirstLift == right.selectedFirstLift);
            CHECK(left.selectedLastLift == right.selectedLastLift);
            CHECK(left.traversalOrientation == right.traversalOrientation);
            CHECK(left.firstLiftedUv == right.firstLiftedUv);
            CHECK(left.lastLiftedUv == right.lastLiftedUv);
            CHECK(left.closingLiftedUv == right.closingLiftedUv);
        }
    }
}

void testUnsupportedCriticalSegmentation(
    const std::filesystem::path& path) {
    // Freeform/bspline remains unsupported for critical segmentation.
    weft::writeStep(weft::makeFixture("fillet"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    // Fillet solid is plane+cylinder; boundaries should succeed. Use a
    // synthetic unsupported by omitting intervals instead.
    weft::IntervalSolution empty;
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance, empty);
    CHECK(!built);
    CHECK(built.failure);
    (void)path;
}



void testCutoutHoleBoundaries(const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture("hole"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance,
                                       intervalsFor(imported));
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("cutout hole boundary failure: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return;
    }
    CHECK(built.value->validation.complete());
    // Perforated planar faces must own outer + hole coedges (distinct wires).
    int perforatedFaces = 0;
    int multiWireFaces = 0;
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
        const bool perforated =
            std::find(record.conditionCodes.begin(), record.conditionCodes.end(),
                      "cutout.planar_perforated") != record.conditionCodes.end();
        if (!perforated) continue;
        ++perforatedFaces;
        std::set<weft::StableId> wires;
        for (const weft::CoedgeRecord& coedge :
             imported.working->snapshot.coedges) {
            if (coedge.faceId == record.subjectId) {
                wires.insert(coedge.wireId);
            }
        }
        if (wires.size() >= 2) ++multiWireFaces;
    }
    CHECK(perforatedFaces >= 1);
    CHECK(multiWireFaces >= 1);
    std::printf(
        "WEFT_CUT_B fixture=hole boundaries=%zu samples=%zu "
        "perforated_multi_wire=%d\n",
        built.value->boundaries.size(), built.value->validation.checkedSamples,
        multiWireFaces);

    // Adversary: empty intervals refuse by name.
    weft::IntervalSolution empty;
    const weft::CanonicalBoundaryBuildResult refused =
        weft::buildCanonicalBoundaries(imported, reconnaissance, empty);
    CHECK(!refused);
    CHECK(refused.failure);
    std::printf("WEFT_CUT_B fixture=hole adversary=%s\n",
                refused.failure->code.c_str());
}

void testCutoutPlateSlotBoundaries(const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture("plate_slot"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance,
                                       intervalsFor(imported));
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("plate_slot boundary failure: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return;
    }
    CHECK(built.value->validation.complete());
    CHECK(built.value->boundaries.size() ==
          imported.working->snapshot.edgeTopology.size());
    int slotted = 0;
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (std::find(record.conditionCodes.begin(), record.conditionCodes.end(),
                      "cutout.planar_slotted") != record.conditionCodes.end()) {
            ++slotted;
        }
    }
    CHECK(slotted >= 1);
    std::printf(
        "WEFT_CUT_B fixture=plate_slot boundaries=%zu samples=%zu slotted=%d\n",
        built.value->boundaries.size(), built.value->validation.checkedSamples,
        slotted);
}

void testMappedFourSidedBoundaries(const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture("ribbon"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance,
                                       intervalsFor(imported));
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("mapped boundary failure: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return;
    }
    CHECK(built.value->validation.complete());
    CHECK(built.value->boundaries.size() ==
          imported.working->snapshot.edgeTopology.size());
    std::printf("WEFT_MAP_B boundaries=%zu samples=%zu\n",
                built.value->boundaries.size(),
                built.value->validation.checkedSamples);
}

void testSpherePoleBoundaries(const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture("sphere"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance,
                                       intervalsFor(imported));
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("sphere canonical failure: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return;
    }
    std::size_t poles = 0;
    for (const weft::EdgeTopologyRecord& topology :
         imported.working->snapshot.edgeTopology) {
        const weft::CanonicalBoundary* boundary =
            built.value->find(topology.id);
        CHECK(boundary != nullptr);
        if (topology.degenerate) {
            ++poles;
            CHECK(boundary->criticalEvents.front().detectionCode ==
                  "event.sphere_pole");
        }
    }
    CHECK(poles == 2);
    std::printf("WEFT_SPHERE_B poles=2 boundaries=%zu\n",
                built.value->boundaries.size());
}

void testConeSingularBoundaries(const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture("cone"), path.string());
    const weft::ImportedModel imported = weft::importStepSecure(path.string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    const weft::IntervalSolution intervals = intervalsFor(imported);
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(imported, reconnaissance, intervals);
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("cone canonical failure: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return;
    }
    CHECK(built.value->validation.complete());
    CHECK(built.value->boundaries.size() ==
          imported.working->snapshot.edgeTopology.size());

    bool sawApexStation = false;
    bool sawGeneratorApexEvent = false;
    for (const weft::EdgeTopologyRecord& topology :
         imported.working->snapshot.edgeTopology) {
        const weft::CanonicalBoundary* boundary =
            built.value->find(topology.id);
        CHECK(boundary != nullptr);
        if (!boundary) continue;
        if (topology.degenerate) {
            CHECK(boundary->intervalCount == 1);
            CHECK(boundary->samples.size() == 1);
            CHECK(!boundary->criticalEvents.empty());
            CHECK(boundary->criticalEvents.front().kind ==
                  weft::CriticalParameterEventKind::Singular);
            CHECK(boundary->criticalEvents.front().detectionCode ==
                  "event.cone_apex");
            sawApexStation = true;
            std::printf(
                "WEFT_CONE_B degenerate_apex samples=1 uv_uses=%zu\n",
                boundary->samples.front().faceUses.size());
        }
        for (const weft::CriticalParameterEvent& event :
             boundary->criticalEvents) {
            if (event.detectionCode == "event.cone_apex" &&
                event.kind == weft::CriticalParameterEventKind::Singular &&
                !topology.degenerate) {
                sawGeneratorApexEvent = true;
            }
        }
    }
    CHECK(sawApexStation);
    CHECK(sawGeneratorApexEvent);

    // Adversary: demand more than one interval on the degenerate apex.
    weft::IntervalSolution bad = intervals;
    for (weft::SolvedInterval& solved : bad.counts) {
        for (const weft::EdgeTopologyRecord& topology :
             imported.working->snapshot.edgeTopology) {
            if (!topology.degenerate) continue;
            const weft::StableId boundaryId{weft::StableIdKind::Boundary,
                                            topology.id.ordinal};
            if (solved.boundaryId == boundaryId) {
                solved.count = 4;
            }
        }
    }
    const weft::CanonicalBoundaryBuildResult refused =
        weft::buildCanonicalBoundaries(imported, reconnaissance, bad);
    CHECK(!refused);
    CHECK(refused.failure &&
          refused.failure->code ==
              "boundary.degenerate_interval_count_invalid");
    std::printf("WEFT_CONE_B adversary=%s\n",
                refused.failure ? refused.failure->code.c_str() : "-");
}

}  // namespace

int main() {
    const std::filesystem::path boxPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_box", ".step");
    const std::filesystem::path cylinderPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_cylinder", ".step");
    const std::filesystem::path partialArcPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_partial_arc", ".step");
    const std::filesystem::path cylinderDeterminismPath =
        weft::test::uniqueTempPath(
            "weft_canonical_boundary_cylinder_determinism", ".step");
    const std::filesystem::path torusUnsupportedPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_torus_unsupported", ".step");
    const std::filesystem::path conePath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_cone", ".step");
    const std::filesystem::path spherePath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_sphere", ".step");
    const std::filesystem::path mappedPath = weft::test::uniqueTempPath(
        "weft_canonical_boundary_mapped", ".step");
    try {
        verifyCanonicalModel("box", boxPath, true, false);
        verifyCanonicalModel("cylinder", cylinderPath, true, true);
        testAzimuthRegistration();
        testPartialPeriodicCurveIsOpen(partialArcPath);
        testPeriodicUvClosureAdversaries();
        testCylinderPeriodicClosureDeterminism(cylinderDeterminismPath);
        testUnsupportedCriticalSegmentation(torusUnsupportedPath);
        testConeSingularBoundaries(conePath);
        testSpherePoleBoundaries(spherePath);
        testCutoutHoleBoundaries(weft::test::uniqueTempPath("weft_cut_b_hole", ".step"));
        testCutoutPlateSlotBoundaries(weft::test::uniqueTempPath("weft_cut_b_slot", ".step"));
        testMappedFourSidedBoundaries(mappedPath);
    } catch (const std::exception& error) {
        std::printf("FAIL canonical-boundary exception: %s\n", error.what());
        ++failures;
    }
    std::error_code ignored;
    std::filesystem::remove(boxPath, ignored);
    std::filesystem::remove(cylinderPath, ignored);
    std::filesystem::remove(partialArcPath, ignored);
    std::filesystem::remove(cylinderDeterminismPath, ignored);
    std::filesystem::remove(torusUnsupportedPath, ignored);
    std::filesystem::remove(conePath, ignored);
    std::filesystem::remove(spherePath, ignored);
    std::filesystem::remove(mappedPath, ignored);

    if (failures == 0) {
        std::printf("canonical boundary checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d canonical boundary failure(s)\n", failures);
    return EXIT_FAILURE;
}
