#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/planar_cdt.hpp"
#include "weft/planar_trim_assembly.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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

class TemporaryStep {
public:
    explicit TemporaryStep(const std::string& stem)
        : path_(weft::test::uniqueTempPath(stem, ".step")) {}

    ~TemporaryStep() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

struct PreparedFixture {
    weft::ImportedModel imported;
    weft::ReconnaissanceReport reconnaissance;
    weft::CanonicalBoundarySet boundaries;
};

std::optional<PreparedFixture> prepare(const std::string& fixture,
                                       const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture(fixture), path.string());
    PreparedFixture prepared;
    prepared.imported = weft::importStepSecure(path.string());
    prepared.reconnaissance = weft::reconnoitre(prepared.imported);
    CHECK(prepared.reconnaissance.complete);
    if (!prepared.reconnaissance.complete) return std::nullopt;

    weft::IntervalProblem intervals;
    for (const weft::EdgeTopologyRecord& edge :
         prepared.imported.working->snapshot.edgeTopology) {
        const weft::ExactGeometryClassification* classification =
            prepared.reconnaissance.find(edge.id);
        const bool circle = classification &&
            classification->familyCode == "circle";
        intervals.variables.push_back(
            {{weft::StableIdKind::Boundary, edge.id.ordinal},
             circle ? 16.0 : 2.0, 1, false, std::nullopt});
    }
    const weft::IntervalSolveResult solved = weft::solveIntervals(intervals);
    CHECK(solved);
    if (!solved) return std::nullopt;
    const weft::CanonicalBoundaryBuildResult built =
        weft::buildCanonicalBoundaries(
            prepared.imported, prepared.reconnaissance, *solved.solution);
    CHECK(built);
    if (!built) {
        if (built.failure) {
            std::printf("boundary preparation failed: %s: %s\n",
                        built.failure->code.c_str(),
                        built.failure->message.c_str());
        }
        return std::nullopt;
    }
    prepared.boundaries = *built.value;
    return prepared;
}

bool completeAssemblyEvidence(const weft::PlanarTrimAssemblyResult& result) {
    return result.evidence.size() == 6 &&
        std::all_of(
            result.evidence.begin(), result.evidence.end(),
            [](const weft::PlanarTrimAssemblyEvidence& item) {
                return item.complete();
            });
}

std::vector<weft::StableId> planarFaces(
    const weft::ReconnaissanceReport& reconnaissance) {
    std::vector<weft::StableId> faces;
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy == weft::GeometryTaxonomy::Surface &&
            record.familyCode == "plane" &&
            record.support ==
                weft::GeometrySupportState::SupportedAnalyticTemplate) {
            faces.push_back(record.subjectId);
        }
    }
    return faces;
}

void testBoxFaces(const weft::PlanarCdtBackend& cdt) {
    TemporaryStep step("weft_trim_assembly_box");
    const auto prepared = prepare("box", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;
    const std::vector<weft::StableId> faces =
        planarFaces(prepared->reconnaissance);
    CHECK(faces.size() == 6);
    for (weft::StableId face : faces) {
        const weft::PlanarTrimAssemblyResult assembled =
            weft::assemblePlanarTrimDomain(
                prepared->imported, prepared->reconnaissance,
                prepared->boundaries, face);
        if (!assembled && assembled.failure) {
            std::printf("box face %llu assembly failed: %s: %s\n",
                        static_cast<unsigned long long>(face.ordinal),
                        assembled.failure->code.c_str(),
                        assembled.failure->message.c_str());
        }
        CHECK(assembled);
        CHECK(completeAssemblyEvidence(assembled));
        CHECK(assembled.validation);
        CHECK(assembled.value && assembled.value->loops.size() == 1);
        if (!assembled.value) continue;

        std::size_t sharedCorners = 0;
        for (const weft::PlanarTrimVertex& vertex :
             assembled.value->loops.front().vertices) {
            if (vertex.boundaryUses.size() == 2) ++sharedCorners;
        }
        CHECK(sharedCorners == 4);
        const weft::PlanarCdtResult triangulated =
            cdt.triangulate(*assembled.value);
        CHECK(triangulated);
    }
}

void testCylinderCapsAndWallRefusal(const weft::PlanarCdtBackend& cdt) {
    TemporaryStep step("weft_trim_assembly_cylinder");
    const auto prepared = prepare("cylinder", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;

    std::size_t caps = 0;
    std::optional<weft::StableId> wall;
    for (const weft::ExactGeometryClassification& record :
         prepared->reconnaissance.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
        if (record.familyCode == "plane") {
            const weft::PlanarTrimAssemblyResult assembled =
                weft::assemblePlanarTrimDomain(
                    prepared->imported, prepared->reconnaissance,
                    prepared->boundaries, record.subjectId);
            CHECK(assembled);
            CHECK(assembled.value && assembled.value->loops.size() == 1);
            if (assembled.value) {
                CHECK(assembled.value->loops.front().vertices.size() == 16);
                CHECK(cdt.triangulate(*assembled.value));
            }
            ++caps;
        } else if (record.familyCode == "cylinder") {
            wall = record.subjectId;
        }
    }
    CHECK(caps == 2);
    CHECK(wall.has_value());
    if (wall) {
        const weft::PlanarTrimAssemblyResult refused =
            weft::assemblePlanarTrimDomain(
                prepared->imported, prepared->reconnaissance,
                prepared->boundaries, *wall);
        CHECK(!refused);
        CHECK(refused.failure &&
              refused.failure->code == "trim_assembly.face_not_planar");
    }
}

void testPerforatedPlanarFace(const weft::PlanarCdtBackend& cdt) {
    TemporaryStep step("weft_trim_assembly_hole");
    const auto prepared = prepare("hole", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;

    bool sawPerforated = false;
    for (weft::StableId face : planarFaces(prepared->reconnaissance)) {
        const weft::PlanarTrimAssemblyResult assembled =
            weft::assemblePlanarTrimDomain(
                prepared->imported, prepared->reconnaissance,
                prepared->boundaries, face);
        CHECK(assembled);
        if (!assembled.value || assembled.value->loops.size() < 2) continue;
        sawPerforated = true;
        CHECK(assembled.value->loops.front().declaredRole ==
              weft::PlanarTrimLoopRole::Outer);
        for (std::size_t index = 1; index < assembled.value->loops.size();
             ++index) {
            CHECK(assembled.value->loops[index].declaredRole ==
                  weft::PlanarTrimLoopRole::Hole);
        }
        const weft::PlanarCdtResult triangulated =
            cdt.triangulate(*assembled.value);
        CHECK(triangulated);
        CHECK(triangulated.trimValidation);
        CHECK(triangulated.value &&
              triangulated.value->boundaryLoops.size() ==
                  assembled.value->loops.size());
    }
    CHECK(sawPerforated);
}

weft::CanonicalBoundary* mutableBoundary(
    weft::CanonicalBoundarySet& set, weft::StableId edge) {
    const auto found = std::find_if(
        set.boundaries.begin(), set.boundaries.end(),
        [edge](const weft::CanonicalBoundary& boundary) {
            return boundary.edge == edge;
        });
    return found == set.boundaries.end() ? nullptr : &*found;
}

void testTamperedJunctionUvRefuses() {
    TemporaryStep step("weft_trim_assembly_tamper");
    const auto prepared = prepare("box", step.path());
    CHECK(prepared.has_value());
    if (!prepared) return;
    const weft::StableId face = planarFaces(prepared->reconnaissance).front();

    std::vector<const weft::CoedgeRecord*> coedges;
    for (const weft::CoedgeRecord& coedge :
         prepared->imported.working->snapshot.coedges) {
        if (coedge.faceId == face) coedges.push_back(&coedge);
    }
    std::sort(coedges.begin(), coedges.end(),
              [](const weft::CoedgeRecord* left,
                 const weft::CoedgeRecord* right) {
                  return left->ordinalInWire < right->ordinalInWire;
              });
    CHECK(coedges.size() == 4);
    if (coedges.size() != 4) return;

    weft::CanonicalBoundarySet tampered = prepared->boundaries;
    const weft::CoedgeRecord& second = *coedges[1];
    weft::CanonicalBoundary* boundary =
        mutableBoundary(tampered, second.edgeId);
    CHECK(boundary != nullptr);
    if (!boundary) return;
    const std::size_t sampleIndex =
        second.orientation == weft::TopologyOrientation::Forward
        ? 0
        : boundary->samples.size() - 1;
    bool changed = false;
    for (weft::CoedgeUvUse& use : boundary->samples[sampleIndex].faceUses) {
        if (use.face == face && use.coedge == second.id) {
            use.liftedUv[0] += 1e-12;
            changed = true;
        }
    }
    CHECK(changed);
    const weft::PlanarTrimAssemblyResult refused =
        weft::assemblePlanarTrimDomain(
            prepared->imported, prepared->reconnaissance, tampered, face);
    CHECK(!refused);
    CHECK(refused.failure &&
          refused.failure->code == "trim_assembly.vertex_uv_mismatch");
}

}  // namespace

int main() {
    try {
        const auto cdt = weft::makeExactLawsonReferencePlanarCdtBackend();
        CHECK(cdt != nullptr);
        if (cdt) {
            testBoxFaces(*cdt);
            testCylinderCapsAndWallRefusal(*cdt);
            testPerforatedPlanarFace(*cdt);
            testTamperedJunctionUvRefuses();
        }
    } catch (const std::exception& error) {
        std::printf("FAIL trim-assembly exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("planar trim assembly checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d planar trim assembly failure(s)\n", failures);
    return EXIT_FAILURE;
}
