#include "weft/certified_mesh.hpp"
#include "weft/cylinder_template.hpp"
#include "weft/fixture.hpp"
#include "weft/interval_solver.hpp"
#include "weft/model.hpp"
#include "weft/planar_trim_assembly.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
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

class TemporaryStep {
public:
    TemporaryStep()
        : path_(weft::test::uniqueTempPath(
              "weft_secure_cylinder_template", ".step")) {}

    ~TemporaryStep() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

struct PreparedCylinder {
    weft::ImportedModel imported;
    weft::ReconnaissanceReport reconnaissance;
    weft::IntervalSolution intervals;
    weft::CanonicalBoundarySet boundaries;
    weft::StableId cylinderFace;
    std::vector<weft::StableId> faces;
};

weft::IntervalSolveResult solveCounts(
    const weft::ImportedModel& imported,
    const weft::ReconnaissanceReport& reconnaissance,
    std::uint32_t firstCircleCount, std::uint32_t secondCircleCount,
    std::uint32_t lineCount) {
    weft::IntervalProblem problem;
    std::size_t circleIndex = 0;
    for (const weft::EdgeTopologyRecord& edge :
         imported.working->snapshot.edgeTopology) {
        const weft::ExactGeometryClassification* classification =
            reconnaissance.find(edge.id);
        const bool circle = classification &&
            classification->familyCode == "circle";
        std::uint32_t count = lineCount;
        if (circle) {
            count = circleIndex++ == 0 ? firstCircleCount
                                       : secondCircleCount;
        }
        problem.variables.push_back(
            {{weft::StableIdKind::Boundary, edge.id.ordinal},
             static_cast<double>(count), count, false, std::nullopt});
    }
    return weft::solveIntervals(problem);
}

std::optional<PreparedCylinder> prepare(
    const std::filesystem::path& path, std::uint32_t firstCircleCount,
    std::uint32_t secondCircleCount, std::uint32_t lineCount) {
    PreparedCylinder prepared;
    prepared.imported = weft::importStepSecure(path.string());
    prepared.reconnaissance = weft::reconnoitre(prepared.imported);
    CHECK(prepared.reconnaissance.complete);
    const weft::IntervalSolveResult solved = solveCounts(
        prepared.imported, prepared.reconnaissance, firstCircleCount,
        secondCircleCount, lineCount);
    CHECK(solved);
    if (!solved) return std::nullopt;
    prepared.intervals = *solved.solution;
    const weft::CanonicalBoundaryBuildResult boundaries =
        weft::buildCanonicalBoundaries(
            prepared.imported, prepared.reconnaissance,
            prepared.intervals);
    CHECK(boundaries);
    if (!boundaries) return std::nullopt;
    prepared.boundaries = *boundaries.value;
    for (const weft::ExactGeometryClassification& record :
         prepared.reconnaissance.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
        prepared.faces.push_back(record.subjectId);
        if (record.familyCode == "cylinder") {
            prepared.cylinderFace = record.subjectId;
        }
    }
    CHECK(prepared.cylinderFace.valid());
    CHECK(prepared.faces.size() == 3);
    if (!prepared.cylinderFace.valid()) return std::nullopt;
    return prepared;
}

weft::CylinderWallConfiguration acceptedConfiguration() {
    weft::CylinderWallConfiguration configuration;
    configuration.maximumChordDeviation = 0.02;
    configuration.maximumNormalDeviationRadians = 0.06;
    return configuration;
}

void testFullCylinderBody(const PreparedCylinder& prepared) {
    const weft::CylinderWallResult wall = weft::buildFullCylinderWall(
        prepared.imported, prepared.reconnaissance, prepared.boundaries,
        prepared.cylinderFace, acceptedConfiguration());
    if (wall.failure) {
        std::printf("cylinder wall refusal: %s: %s\n",
                    wall.failure->code.c_str(),
                    wall.failure->message.c_str());
    }
    CHECK(wall);
    CHECK(!wall.failure);
    CHECK(wall.validation.size() == 6);
    for (const weft::CylinderWallValidationEvidence& evidence :
         wall.validation) {
        CHECK(evidence.complete());
        CHECK(evidence.failed == 0);
        CHECK(evidence.skipped == 0);
    }
    CHECK(wall.value && wall.value->vertices.size() == 128);
    CHECK(wall.value && wall.value->triangles.size() == 128);
    CHECK(wall.value && wall.value->boundaryLoops.size() == 2);
    if (!wall.value) return;
    for (const weft::PlanarCdtTriangle& triangle : wall.value->triangles) {
        CHECK(triangle.cornerUv.has_value());
    }

    const auto cdt = weft::makeExactLawsonReferencePlanarCdtBackend();
    std::vector<weft::PlanarCdtMesh> faces;
    faces.push_back(*wall.value);
    for (weft::StableId face : prepared.faces) {
        if (face == prepared.cylinderFace) continue;
        const weft::PlanarTrimAssemblyResult trim =
            weft::assemblePlanarTrimDomain(
                prepared.imported, prepared.reconnaissance,
                prepared.boundaries, face);
        CHECK(trim);
        if (!trim.value) return;
        const weft::PlanarCdtResult triangulated =
            cdt->triangulate(*trim.value);
        CHECK(triangulated);
        if (!triangulated.value) return;
        faces.push_back(*triangulated.value);
    }
    CHECK(faces.size() == 3);

    const weft::CertifiedMeshAssemblyResult certified =
        weft::assembleCertifiedBoundaryMesh(
            prepared.imported, prepared.boundaries, faces,
            prepared.faces);
    CHECK(certified);
    CHECK(certified.validation.complete());
    CHECK(certified.value && certified.value->vertices.size() == 128);
    CHECK(certified.value && certified.value->triangles.size() == 252);
    CHECK(certified.value &&
          certified.value->topologyFingerprint.size() == 16);
    if (!certified.value) return;

    std::vector<weft::PlanarCdtMesh> missingRepresentation = faces;
    auto missingWall = std::find_if(
        missingRepresentation.begin(), missingRepresentation.end(),
        [&](const weft::PlanarCdtMesh& face) {
            return face.workingFace == prepared.cylinderFace;
        });
    CHECK(missingWall != missingRepresentation.end());
    if (missingWall != missingRepresentation.end() &&
        !missingWall->vertices.empty() &&
        !missingWall->vertices.front().boundaryUses.empty()) {
        missingWall->vertices.front().boundaryUses.front()
            .representation.reset();
        const weft::CertifiedMeshAssemblyResult refused =
            weft::assembleCertifiedBoundaryMesh(
                prepared.imported, prepared.boundaries,
                missingRepresentation, prepared.faces);
        CHECK(!refused);
        CHECK(refused.failure &&
              refused.failure->code ==
                  "certified.boundary_provenance_invalid");
    }

    std::vector<weft::PlanarCdtMesh> wrongPeriodicUv = faces;
    auto uvWall = std::find_if(
        wrongPeriodicUv.begin(), wrongPeriodicUv.end(),
        [&](const weft::PlanarCdtMesh& face) {
            return face.workingFace == prepared.cylinderFace;
        });
    CHECK(uvWall != wrongPeriodicUv.end());
    if (uvWall != wrongPeriodicUv.end() && !uvWall->triangles.empty() &&
        uvWall->triangles.front().cornerUv) {
        (*uvWall->triangles.front().cornerUv)[0][0] += 0.2;
        const weft::CertifiedMeshAssemblyResult refused =
            weft::assembleCertifiedBoundaryMesh(
                prepared.imported, prepared.boundaries, wrongPeriodicUv,
                prepared.faces);
        CHECK(!refused);
        CHECK(refused.failure &&
              refused.failure->code == "certified.vertex_off_surface");
    }

    std::reverse(faces.begin(), faces.end());
    const weft::CertifiedMeshAssemblyResult reordered =
        weft::assembleCertifiedBoundaryMesh(
            prepared.imported, prepared.boundaries, faces,
            prepared.faces);
    CHECK(reordered);
    CHECK(reordered.value &&
          reordered.value->topologyFingerprint ==
              certified.value->topologyFingerprint);

    const weft::MeshingResult meshing =
        weft::makeCertifiedFloorMeshingResult(
            *certified.value, certified.validation, {},
            "structured modeling cylinder is not yet independently certified");
    CHECK(meshing.modeling.aliasesCertified);
    CHECK(meshing.modeling.safeFloorReason.has_value());
    CHECK(meshing.modeling.polygons.size() ==
          meshing.certified.triangles.size());
}

void testNamedRefusals(const std::filesystem::path& path,
                       const PreparedCylinder& prepared) {
    weft::CanonicalBoundarySet reflected = prepared.boundaries;
    std::size_t circleCount = 0;
    for (weft::CanonicalBoundary& boundary : reflected.boundaries) {
        const weft::ExactGeometryClassification* classification =
            prepared.reconnaissance.find(boundary.edge);
        if (!classification || classification->familyCode != "circle" ||
            !boundary.closed) {
            continue;
        }
        if (++circleCount == 2) {
            std::reverse(boundary.samples.begin(), boundary.samples.end());
        }
    }
    CHECK(circleCount == 2);
    const weft::CylinderWallResult reflectedRefusal =
        weft::buildFullCylinderWall(
            prepared.imported, prepared.reconnaissance, reflected,
            prepared.cylinderFace, acceptedConfiguration());
    CHECK(!reflectedRefusal);
    CHECK(reflectedRefusal.failure &&
          reflectedRefusal.failure->code ==
              "boundary.azimuth_reflection");

    const auto mismatched = prepare(path, 32, 64, 1);
    CHECK(mismatched.has_value());
    if (mismatched) {
        const weft::CylinderWallResult result =
            weft::buildFullCylinderWall(
                mismatched->imported, mismatched->reconnaissance,
                mismatched->boundaries, mismatched->cylinderFace,
                acceptedConfiguration());
        CHECK(!result);
        CHECK(result.failure &&
              result.failure->code == "cylinder.rim_count_mismatch");
    }

    const auto axial = prepare(path, 64, 64, 2);
    CHECK(axial.has_value());
    if (axial) {
        const weft::CylinderWallResult missingProvenance =
            weft::buildFullCylinderWall(
                axial->imported, axial->reconnaissance,
                axial->boundaries, axial->cylinderFace,
                acceptedConfiguration());
        CHECK(!missingProvenance);
        CHECK(missingProvenance.failure &&
              missingProvenance.failure->code ==
                  "cylinder.axial_samples_require_interior_provenance");

        weft::CylinderWallConfiguration withRings = acceptedConfiguration();
        withRings.axialIntervals = 2;
        const weft::CylinderWallResult certified =
            weft::buildFullCylinderWall(
                axial->imported, axial->reconnaissance, axial->boundaries,
                axial->cylinderFace, withRings);
        CHECK(certified);
        CHECK(certified.value &&
              certified.value->vertices.size() == 64 * 3);
        CHECK(certified.value &&
              certified.value->triangles.size() == 64 * 4);
        std::size_t interiorVertices = 0;
        if (certified.value) {
            for (const weft::PlanarTrimVertex& vertex :
                 certified.value->vertices) {
                if (vertex.cylinderInterior) ++interiorVertices;
            }
        }
        CHECK(interiorVertices == 64);
    }

    weft::CylinderWallConfiguration chord = acceptedConfiguration();
    chord.maximumChordDeviation = 1e-10;
    const weft::CylinderWallResult chordRefusal =
        weft::buildFullCylinderWall(
            prepared.imported, prepared.reconnaissance,
            prepared.boundaries, prepared.cylinderFace, chord);
    CHECK(!chordRefusal);
    CHECK(chordRefusal.failure &&
          chordRefusal.failure->code == "cylinder.chord_bound_exceeded");

    weft::CylinderWallConfiguration normal = acceptedConfiguration();
    normal.maximumChordDeviation = 1.0;
    normal.maximumNormalDeviationRadians = 1e-4;
    const weft::CylinderWallResult normalRefusal =
        weft::buildFullCylinderWall(
            prepared.imported, prepared.reconnaissance,
            prepared.boundaries, prepared.cylinderFace, normal);
    CHECK(!normalRefusal);
    CHECK(normalRefusal.failure &&
          normalRefusal.failure->code == "cylinder.normal_bound_exceeded");

    const auto plane = std::find_if(
        prepared.faces.begin(), prepared.faces.end(),
        [&](weft::StableId face) { return face != prepared.cylinderFace; });
    CHECK(plane != prepared.faces.end());
    if (plane != prepared.faces.end()) {
        const weft::CylinderWallResult wrongFace =
            weft::buildFullCylinderWall(
                prepared.imported, prepared.reconnaissance,
                prepared.boundaries, *plane, acceptedConfiguration());
        CHECK(!wrongFace);
        CHECK(wrongFace.failure &&
              wrongFace.failure->code ==
                  "cylinder.face_unsupported");
    }
}

}  // namespace

int main() {
    TemporaryStep step;
    try {
        weft::writeStep(weft::makeFixture("cylinder"),
                        step.path().string());
        const std::optional<PreparedCylinder> prepared =
            prepare(step.path(), 64, 64, 1);
        CHECK(prepared.has_value());
        if (prepared) {
            testFullCylinderBody(*prepared);
            testNamedRefusals(step.path(), *prepared);
        }
    } catch (const std::exception& error) {
        std::printf("FAIL cylinder-template exception: %s\n", error.what());
        ++failures;
    }

    if (failures == 0) {
        std::printf("certified cylinder template checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d cylinder template failure(s)\n", failures);
    return EXIT_FAILURE;
}
