#include "weft/certified_mesh.hpp"
#include "weft/fixture.hpp"
#include "weft/model.hpp"
#include "weft/planar_trim_assembly.hpp"

#include "test_temp_path.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
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
    explicit TemporaryStep(
        const std::string& stem = "weft_certified_mesh_box")
        : path_(weft::test::uniqueTempPath(
              stem, ".step")) {}

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
             2.0, 1, false, std::nullopt});
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
    CHECK(result.validation.checks.size() == 9);
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
    const weft::ValidationCoverage* intersections =
        coverage(result, "certified.triangle_intersection");
    CHECK(incidence && incidence->expected == 54);
    CHECK(incidence && incidence->checked == 54);
    CHECK(winding && winding->expected == 54);
    CHECK(winding && winding->checked == 54);
    CHECK(intersections && intersections->expected == 36 * 35 / 2);
    CHECK(intersections && intersections->checked == intersections->expected);
    CHECK(intersections && intersections->failed == 0);
    CHECK(intersections && intersections->complete());

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

weft::CertifiedMesh makeSyntheticMesh(
    std::vector<std::array<double, 3>> positions,
    std::vector<std::array<std::uint32_t, 3>> triangles) {
    weft::CertifiedMesh mesh;
    mesh.vertices.reserve(positions.size());
    for (std::size_t index = 0; index < positions.size(); ++index) {
        weft::CertifiedVertex vertex;
        vertex.canonicalVertexIndex = static_cast<std::uint64_t>(index);
        vertex.position = positions[index];
        mesh.vertices.push_back(std::move(vertex));
    }
    mesh.triangles.reserve(triangles.size());
    for (const auto& corners : triangles) {
        weft::CertifiedTriangle triangle;
        triangle.workingFace = {weft::StableIdKind::Face, 1};
        triangle.vertices = corners;
        mesh.triangles.push_back(std::move(triangle));
    }
    return mesh;
}

void testTriangleIntersectionAdversaries() {
    const weft::CertifiedMesh legalAdjacent = makeSyntheticMesh(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {0.0, 1.0, 0.0},
         {1.0, 1.0, 0.0}},
        {{{0, 1, 2}, {1, 3, 2}}});
    const weft::CertifiedTriangleIntersectionResult adjacent =
        weft::validateCertifiedTriangleIntersections(legalAdjacent);
    CHECK(adjacent);
    CHECK(adjacent.coverage.expected == 1);
    CHECK(adjacent.coverage.checked == 1);
    CHECK(adjacent.coverage.failed == 0);

    const weft::CertifiedMesh legalVertex = makeSyntheticMesh(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {0.0, 1.0, 0.0},
         {-1.0, 0.0, 1.0},
         {-1.0, 1.0, 0.0}},
        {{{0, 1, 2}, {0, 3, 4}}});
    CHECK(weft::validateCertifiedTriangleIntersections(legalVertex));

    const weft::CertifiedMesh nearContact = makeSyntheticMesh(
        {{0.0, 0.0, 0.0},
         {1.0, 0.0, 0.0},
         {0.0, 1.0, 0.0},
         {0.0, 0.0, 1e-9},
         {1.0, 0.0, 1e-9},
         {0.0, 1.0, 1e-9}},
        {{{0, 1, 2}, {3, 4, 5}}});
    CHECK(weft::validateCertifiedTriangleIntersections(nearContact));

    const weft::CertifiedMesh stabbing = makeSyntheticMesh(
        {{0.0, 0.0, 0.0},
         {2.0, 0.0, 0.0},
         {0.0, 2.0, 0.0},
         {0.5, 0.5, -1.0},
         {0.5, 0.5, 1.0},
         {1.5, -0.5, 0.0}},
        {{{0, 1, 2}, {3, 4, 5}}});
    const weft::CertifiedTriangleIntersectionResult proper =
        weft::validateCertifiedTriangleIntersections(stabbing);
    CHECK(!proper);
    CHECK(proper.failure &&
          proper.failure->code == "certified.triangle_proper_intersection");
    CHECK(proper.coverage.failed == 1);

    const weft::CertifiedMesh overlapping = makeSyntheticMesh(
        {{0.0, 0.0, 0.0},
         {2.0, 0.0, 0.0},
         {0.0, 2.0, 0.0},
         {0.5, 0.5, 0.0},
         {1.5, 0.5, 0.0},
         {0.5, 1.5, 0.0}},
        {{{0, 1, 2}, {3, 4, 5}}});
    const weft::CertifiedTriangleIntersectionResult coplanar =
        weft::validateCertifiedTriangleIntersections(overlapping);
    CHECK(!coplanar);
    CHECK(coplanar.failure &&
          coplanar.failure->code == "certified.triangle_coplanar_overlap");

    const weft::CertifiedMesh duplicate = makeSyntheticMesh(
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
        {{{0, 1, 2}, {0, 2, 1}}});
    const weft::CertifiedTriangleIntersectionResult dup =
        weft::validateCertifiedTriangleIntersections(duplicate);
    CHECK(!dup);
    CHECK(dup.failure &&
          dup.failure->code == "certified.triangle_duplicate");

    const weft::CertifiedMesh single = makeSyntheticMesh(
        {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}},
        {{{0, 1, 2}}});
    const weft::CertifiedTriangleIntersectionResult vacuous =
        weft::validateCertifiedTriangleIntersections(single);
    CHECK(vacuous);
    CHECK(vacuous.coverage.expected == 0);
    CHECK(vacuous.coverage.checked == 0);
}

void testPerforatedPlanarFaceProduct() {
    TemporaryStep step("weft_certified_mesh_hole_face");
    weft::writeStep(weft::makeFixture("hole"), step.path().string());
    const weft::ImportedModel imported =
        weft::importStepSecure(step.path().string());
    const weft::ReconnaissanceReport reconnaissance =
        weft::reconnoitre(imported);
    CHECK(reconnaissance.complete);

    weft::IntervalProblem intervalProblem;
    for (const weft::EdgeTopologyRecord& edge :
         imported.working->snapshot.edgeTopology) {
        const weft::ExactGeometryClassification* classification =
            reconnaissance.find(edge.id);
        const bool circle = classification &&
            classification->familyCode == "circle";
        intervalProblem.variables.push_back(
            {{weft::StableIdKind::Boundary, edge.id.ordinal},
             circle ? 16.0 : 2.0, 1, false, std::nullopt});
    }
    const weft::IntervalSolveResult intervals =
        weft::solveIntervals(intervalProblem);
    CHECK(intervals);
    if (!intervals) return;
    const weft::CanonicalBoundaryBuildResult boundaries =
        weft::buildCanonicalBoundaries(
            imported, reconnaissance, *intervals.solution);
    CHECK(boundaries);
    if (!boundaries) return;

    const auto cdt = weft::makeExactLawsonReferencePlanarCdtBackend();
    std::optional<weft::PlanarCdtMesh> perforated;
    for (const weft::ExactGeometryClassification& record :
         reconnaissance.records) {
        if (record.taxonomy != weft::GeometryTaxonomy::Surface ||
            record.familyCode != "plane") {
            continue;
        }
        const weft::PlanarTrimAssemblyResult trim =
            weft::assemblePlanarTrimDomain(
                imported, reconnaissance, *boundaries.value,
                record.subjectId);
        CHECK(trim);
        if (!trim.value || trim.value->loops.size() < 2) continue;
        const weft::PlanarCdtResult triangulated =
            cdt->triangulate(*trim.value);
        CHECK(triangulated);
        if (triangulated.value) perforated = *triangulated.value;
        break;
    }
    CHECK(perforated.has_value());
    if (!perforated) return;

    const std::vector<weft::PlanarCdtMesh> faceMeshes{*perforated};
    const std::vector<weft::StableId> expectedFaces{
        perforated->workingFace};
    weft::CertifiedMeshAssemblyConfiguration openFace;
    openFace.requireClosedManifold = false;
    const weft::CertifiedMeshAssemblyResult certified =
        weft::assembleCertifiedPlanarMesh(
            imported, *boundaries.value, faceMeshes, expectedFaces,
            openFace);
    CHECK(certified);
    CHECK(certified.validation.complete());
    CHECK(certified.value &&
          certified.value->triangles.size() ==
              perforated->triangles.size());

    const weft::CertifiedMeshAssemblyResult closedRefusal =
        weft::assembleCertifiedPlanarMesh(
            imported, *boundaries.value, faceMeshes, expectedFaces);
    CHECK(!closedRefusal);
    CHECK(closedRefusal.failure &&
          closedRefusal.failure->code ==
              "certified.edge_incidence_invalid");
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
            testPerforatedPlanarFaceProduct();
        }
        testTriangleIntersectionAdversaries();
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
