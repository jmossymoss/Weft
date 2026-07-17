#include "weft/certified_mesh.hpp"
#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/planar_trim_assembly.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <set>
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
    TemporaryStep()
        : path_(weft::test::uniqueTempPath(
              "weft_certified_mesh_box", ".step")) {}

    ~TemporaryStep() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

struct PreparedBox {
    weft::ImportedModel imported;
    weft::ReconnaissanceReport reconnaissance;
    weft::CanonicalBoundarySet boundaries;
    std::vector<weft::StableId> faces;
    std::vector<weft::PlanarCdtMesh> faceMeshes;
};

std::optional<PreparedBox> prepareBox(const std::filesystem::path& path) {
    weft::writeStep(weft::makeFixture("box"), path.string());
    PreparedBox prepared;
    prepared.imported = weft::importStepSecure(path.string());
    prepared.reconnaissance = weft::reconnoitre(prepared.imported);
    CHECK(prepared.reconnaissance.complete);
    if (!prepared.reconnaissance.complete) return std::nullopt;

    weft::IntervalProblem intervalProblem;
    for (const weft::EdgeTopologyRecord& edge :
         prepared.imported.working->snapshot.edgeTopology) {
        intervalProblem.variables.push_back(
            {{weft::StableIdKind::Boundary, edge.id.ordinal},
             2.0, 1, false});
    }
    const weft::IntervalSolveResult intervals =
        weft::solveIntervals(intervalProblem);
    CHECK(intervals);
    if (!intervals) return std::nullopt;
    const weft::CanonicalBoundaryBuildResult boundaries =
        weft::buildCanonicalBoundaries(
            prepared.imported, prepared.reconnaissance,
            *intervals.solution);
    CHECK(boundaries);
    if (!boundaries) return std::nullopt;
    prepared.boundaries = *boundaries.value;

    const auto cdt = weft::makeExactLawsonReferencePlanarCdtBackend();
    for (const weft::ExactGeometryClassification& record :
         prepared.reconnaissance.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Surface) continue;
        CHECK(record.familyCode == "plane");
        const weft::PlanarTrimAssemblyResult trim =
            weft::assemblePlanarTrimDomain(
                prepared.imported, prepared.reconnaissance,
                prepared.boundaries, record.subjectId);
        CHECK(trim);
        if (!trim.value) return std::nullopt;
        const weft::PlanarCdtResult faceMesh = cdt->triangulate(*trim.value);
        CHECK(faceMesh);
        if (!faceMesh.value) return std::nullopt;
        prepared.faces.push_back(record.subjectId);
        prepared.faceMeshes.push_back(*faceMesh.value);
    }
    CHECK(prepared.faces.size() == 6);
    return prepared;
}

const weft::ValidationCoverage* coverage(
    const weft::CertifiedMeshAssemblyResult& result,
    const std::string& code) {
    const auto found = std::find_if(
        result.validation.checks.begin(), result.validation.checks.end(),
        [&](const weft::ValidationCoverage& item) {
            return item.code == code;
        });
    return found == result.validation.checks.end() ? nullptr : &*found;
}

void verifyBoxMesh(const weft::CertifiedMeshAssemblyResult& result) {
    if (!result && result.failure) {
        std::printf("certified box failure: %s: %s\n",
                    result.failure->code.c_str(),
                    result.failure->message.c_str());
    }
    CHECK(result);
    CHECK(!result.failure);
    CHECK(result.validation.complete());
    CHECK(result.validation.checks.size() == 8);
    if (!result.value) return;
    CHECK(result.value->vertices.size() == 20);
    CHECK(result.value->triangles.size() == 36);
    CHECK(result.value->topologyFingerprint.size() == 16);

    std::set<std::uint64_t> canonicalIndices;
    std::size_t cornerVertices = 0;
    std::size_t edgeInteriorVertices = 0;
    for (const weft::CertifiedVertex& vertex : result.value->vertices) {
        CHECK(canonicalIndices.insert(vertex.canonicalVertexIndex).second);
        if (vertex.provenance.size() == 6) {
            ++cornerVertices;
        } else if (vertex.provenance.size() == 2) {
            ++edgeInteriorVertices;
        } else {
            CHECK(false);
        }
    }
    CHECK(cornerVertices == 8);
    CHECK(edgeInteriorVertices == 12);

    const weft::ValidationCoverage* incidence =
        coverage(result, "certified.edge_incidence");
    const weft::ValidationCoverage* winding =
        coverage(result, "certified.edge_winding");
    CHECK(incidence && incidence->expected == 54);
    CHECK(incidence && incidence->checked == 54);
    CHECK(winding && winding->expected == 54);
    CHECK(winding && winding->checked == 54);

    const weft::MeshingResult meshing =
        weft::makeCertifiedFloorMeshingResult(
            *result.value, result.validation,
            {}, std::string("no structured template proven"));
    CHECK(meshing.validation.complete());
    CHECK(meshing.modeling.aliasesCertified);
    CHECK(meshing.modeling.safeFloorReason &&
          *meshing.modeling.safeFloorReason ==
              "no structured template proven");
    CHECK(meshing.modeling.vertices.size() ==
          meshing.certified.vertices.size());
    CHECK(meshing.modeling.polygons.size() ==
          meshing.certified.triangles.size());
    CHECK(std::all_of(
        meshing.modeling.polygons.begin(), meshing.modeling.polygons.end(),
        [](const weft::ModelingPolygon& polygon) {
            return polygon.vertices.size() == 3;
        }));
}

void testClosedBoxAndDeterminism(const PreparedBox& prepared) {
    const weft::CertifiedMeshAssemblyResult first =
        weft::assembleCertifiedPlanarMesh(
            prepared.imported, prepared.boundaries,
            prepared.faceMeshes, prepared.faces);
    verifyBoxMesh(first);

    std::vector<weft::PlanarCdtMesh> reversedMeshes = prepared.faceMeshes;
    std::reverse(reversedMeshes.begin(), reversedMeshes.end());
    std::vector<weft::StableId> reversedFaces = prepared.faces;
    std::reverse(reversedFaces.begin(), reversedFaces.end());
    const weft::CertifiedMeshAssemblyResult second =
        weft::assembleCertifiedPlanarMesh(
            prepared.imported, prepared.boundaries,
            reversedMeshes, reversedFaces);
    verifyBoxMesh(second);
    CHECK(first.value && second.value &&
          first.value->topologyFingerprint ==
              second.value->topologyFingerprint);
    CHECK(first.value && second.value &&
          first.value->vertices.size() == second.value->vertices.size());
    CHECK(first.value && second.value &&
          first.value->triangles.size() == second.value->triangles.size());
}

void testMissingAndDuplicateFaceRefuse(const PreparedBox& prepared) {
    std::vector<weft::PlanarCdtMesh> missing = prepared.faceMeshes;
    missing.pop_back();
    const weft::CertifiedMeshAssemblyResult missingResult =
        weft::assembleCertifiedPlanarMesh(
            prepared.imported, prepared.boundaries, missing, prepared.faces);
    CHECK(!missingResult);
    CHECK(missingResult.failure &&
          missingResult.failure->code == "certified.face_mesh_missing");
    const weft::ValidationCoverage* faceCoverage =
        coverage(missingResult, "certified.face_coverage");
    CHECK(faceCoverage && faceCoverage->expected == 6);
    CHECK(faceCoverage && faceCoverage->checked == 6);
    CHECK(faceCoverage && faceCoverage->failed == 1);

    std::vector<weft::PlanarCdtMesh> duplicate = prepared.faceMeshes;
    duplicate.push_back(duplicate.front());
    const weft::CertifiedMeshAssemblyResult duplicateResult =
        weft::assembleCertifiedPlanarMesh(
            prepared.imported, prepared.boundaries,
            duplicate, prepared.faces);
    CHECK(!duplicateResult);
    CHECK(duplicateResult.failure &&
          duplicateResult.failure->code == "certified.duplicate_face_mesh");

    const std::vector<weft::StableId> noFaces;
    const std::vector<weft::PlanarCdtMesh> noMeshes;
    const weft::CertifiedMeshAssemblyResult empty =
        weft::assembleCertifiedPlanarMesh(
            prepared.imported, prepared.boundaries, noMeshes, noFaces);
    CHECK(!empty);
    CHECK(empty.failure &&
          empty.failure->code == "certified.expected_face_set_empty");
}

void testTamperedCanonicalPositionRefuses(const PreparedBox& prepared) {
    weft::CanonicalBoundarySet tampered = prepared.boundaries;
    CHECK(!tampered.boundaries.empty());
    CHECK(!tampered.boundaries.front().samples.empty());
    if (tampered.boundaries.empty() ||
        tampered.boundaries.front().samples.empty()) {
        return;
    }
    tampered.boundaries.front().samples.front().position[0] += 1e-9;
    const weft::CertifiedMeshAssemblyResult result =
        weft::assembleCertifiedPlanarMesh(
            prepared.imported, tampered,
            prepared.faceMeshes, prepared.faces);
    CHECK(!result);
    CHECK(result.failure &&
          result.failure->code == "certified.vertex_position_mismatch");
}

void testTamperedTriangleRefuses(const PreparedBox& prepared) {
    std::vector<weft::PlanarCdtMesh> tampered = prepared.faceMeshes;
    CHECK(!tampered.empty() && !tampered.front().triangles.empty());
    if (tampered.empty() || tampered.front().triangles.empty()) return;
    std::swap(tampered.front().triangles.front().vertices[1],
              tampered.front().triangles.front().vertices[2]);
    const weft::CertifiedMeshAssemblyResult result =
        weft::assembleCertifiedPlanarMesh(
            prepared.imported, prepared.boundaries,
            tampered, prepared.faces);
    CHECK(!result);
    CHECK(result.failure &&
          result.failure->code ==
              "certified.triangle_orientation_invalid");
}

}  // namespace

int main() {
    try {
        TemporaryStep step;
        const std::optional<PreparedBox> prepared = prepareBox(step.path());
        CHECK(prepared.has_value());
        if (prepared) {
            testClosedBoxAndDeterminism(*prepared);
            testMissingAndDuplicateFaceRefuse(*prepared);
            testTamperedCanonicalPositionRefuses(*prepared);
            testTamperedTriangleRefuses(*prepared);
        }
    } catch (const std::exception& error) {
        std::printf("FAIL certified-mesh exception: %s\n", error.what());
        ++failures;
    }
    if (failures == 0) {
        std::printf("certified planar mesh assembly checks passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d certified mesh failure(s)\n", failures);
    return EXIT_FAILURE;
}
