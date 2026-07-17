#include "weft/certified_mesh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

namespace weft {
namespace {

enum CheckIndex : std::size_t {
    FaceCoverage = 0,
    BoundaryProvenance,
    CanonicalIdentity,
    TriangleProvenance,
    TriangleGeometry,
    EdgeIncidence,
    EdgeWinding,
    Fingerprint,
};

struct Edge {
    std::uint32_t lower = 0;
    std::uint32_t upper = 0;

    auto operator<=>(const Edge&) const = default;
};

struct DirectedEdgeUse {
    bool lowerToUpper = false;
};

Edge edge(std::uint32_t first, std::uint32_t second) {
    return first < second ? Edge{first, second} : Edge{second, first};
}

void setFailure(CertifiedMeshAssemblyResult& result, std::string code,
                std::string message, std::vector<StableId> subjects) {
    if (!result.failure) {
        result.failure = CertifiedMeshAssemblyFailure{
            std::move(code), std::move(message), std::move(subjects)};
    }
}

std::optional<TopologyOrientation> faceOrientation(
    const BRepSnapshot& snapshot, StableId face) {
    const auto found = std::find_if(
        snapshot.occurrences.begin(), snapshot.occurrences.end(),
        [face](const TopologyOccurrence& occurrence) {
            return occurrence.id == face;
        });
    return found == snapshot.occurrences.end()
        ? std::nullopt
        : std::optional<TopologyOrientation>(found->orientation);
}

bool exactPositionEqual(const std::array<double, 3>& first,
                        const std::array<double, 3>& second) {
    return first == second;
}

const CanonicalBoundarySample* resolveBoundarySample(
    const CanonicalBoundarySet& boundaries,
    const PlanarTrimBoundaryUse& use) {
    const CanonicalBoundary* boundary = boundaries.find(use.workingEdge);
    if (!boundary || boundary->boundaryId != use.sample.boundary ||
        use.sample.ordinal >= boundary->samples.size()) {
        return nullptr;
    }
    const CanonicalBoundarySample& sample =
        boundary->samples[use.sample.ordinal];
    return sample.id == use.sample && sample.workingEdge == use.workingEdge &&
        sample.sourceEdge == use.sourceEdge
        ? &sample
        : nullptr;
}

bool resolvesFaceUse(const CanonicalBoundarySample& sample,
                     const CertifiedVertexUse& use) {
    return std::any_of(
        sample.faceUses.begin(), sample.faceUses.end(),
        [&](const CoedgeUvUse& faceUse) {
            return faceUse.face == use.workingFace &&
                faceUse.sourceFace == use.sourceFace &&
                faceUse.coedge == use.boundary.coedge &&
                faceUse.liftedUv == use.boundary.uv &&
                faceUse.measuredCurveOnSurfaceDiscrepancy ==
                    use.boundary.measuredCurveOnSurfaceDiscrepancy &&
                faceUse.allowedCurveOnSurfaceDiscrepancy ==
                    use.boundary.allowedCurveOnSurfaceDiscrepancy;
        });
}

bool sameProvenance(const CertifiedVertexUse& first,
                    const CertifiedVertexUse& second) {
    return first.workingFace == second.workingFace &&
        first.sourceFace == second.sourceFace &&
        first.boundary.sample == second.boundary.sample &&
        first.boundary.workingEdge == second.boundary.workingEdge &&
        first.boundary.sourceEdge == second.boundary.sourceEdge &&
        first.boundary.coedge == second.boundary.coedge &&
        first.boundary.uv == second.boundary.uv &&
        first.boundary.measuredCurveOnSurfaceDiscrepancy ==
            second.boundary.measuredCurveOnSurfaceDiscrepancy &&
        first.boundary.allowedCurveOnSurfaceDiscrepancy ==
            second.boundary.allowedCurveOnSurfaceDiscrepancy;
}

StableId optionalId(const std::optional<StableId>& value) {
    return value.value_or(StableId{});
}

bool provenanceLess(const CertifiedVertexUse& first,
                    const CertifiedVertexUse& second) {
    return std::tuple(first.workingFace, optionalId(first.sourceFace),
                      first.boundary.workingEdge,
                      optionalId(first.boundary.sourceEdge),
                      first.boundary.coedge,
                      first.boundary.sample.boundary,
                      first.boundary.sample.ordinal, first.boundary.uv,
                      first.boundary.measuredCurveOnSurfaceDiscrepancy,
                      first.boundary.allowedCurveOnSurfaceDiscrepancy) <
        std::tuple(second.workingFace, optionalId(second.sourceFace),
                   second.boundary.workingEdge,
                   optionalId(second.boundary.sourceEdge),
                   second.boundary.coedge,
                   second.boundary.sample.boundary,
                   second.boundary.sample.ordinal, second.boundary.uv,
                   second.boundary.measuredCurveOnSurfaceDiscrepancy,
                   second.boundary.allowedCurveOnSurfaceDiscrepancy);
}

double squaredDistance(std::array<double, 3> first,
                       std::array<double, 3> second) {
    const double x = first[0] - second[0];
    const double y = first[1] - second[1];
    const double z = first[2] - second[2];
    return x * x + y * y + z * z;
}

std::array<double, 3> triangleNormal(
    const std::array<double, 3>& a, const std::array<double, 3>& b,
    const std::array<double, 3>& c) {
    const std::array<double, 3> ab{b[0] - a[0], b[1] - a[1],
                                   b[2] - a[2]};
    const std::array<double, 3> ac{c[0] - a[0], c[1] - a[1],
                                   c[2] - a[2]};
    return {ab[1] * ac[2] - ab[2] * ac[1],
            ab[2] * ac[0] - ab[0] * ac[2],
            ab[0] * ac[1] - ab[1] * ac[0]};
}

double dot(const std::array<double, 3>& first,
           const std::array<double, 3>& second) {
    return first[0] * second[0] + first[1] * second[1] +
        first[2] * second[2];
}

class Fnv1a64 {
public:
    void add(std::uint64_t value) {
        for (unsigned byte = 0; byte < 8; ++byte) {
            state_ ^= static_cast<std::uint8_t>(value >> (byte * 8U));
            state_ *= 1099511628211ULL;
        }
    }

    std::string finish() const {
        std::ostringstream stream;
        stream << std::hex << std::setfill('0') << std::setw(16) << state_;
        return stream.str();
    }

private:
    std::uint64_t state_ = 14695981039346656037ULL;
};

std::string fingerprintOf(const CertifiedMesh& mesh) {
    Fnv1a64 digest;
    digest.add(mesh.vertices.size());
    for (const CertifiedVertex& vertex : mesh.vertices) {
        digest.add(vertex.canonicalVertexIndex);
        for (double coordinate : vertex.position) {
            digest.add(std::bit_cast<std::uint64_t>(coordinate));
        }
    }
    digest.add(mesh.triangles.size());
    for (const CertifiedTriangle& triangle : mesh.triangles) {
        digest.add(static_cast<std::uint64_t>(triangle.workingFace.kind));
        digest.add(triangle.workingFace.ordinal);
        const StableId source = optionalId(triangle.sourceFace);
        digest.add(static_cast<std::uint64_t>(source.kind));
        digest.add(source.ordinal);
        for (std::uint32_t vertex : triangle.vertices) digest.add(vertex);
    }
    return digest.finish();
}

}  // namespace

bool ValidationCertificate::complete() const noexcept {
    return !checks.empty() &&
        std::all_of(checks.begin(), checks.end(),
                    [](const ValidationCoverage& coverage) {
                        return coverage.complete();
                    });
}

ModelingMesh makeCertifiedFloorModelingMesh(
    const CertifiedMesh& certified,
    std::optional<std::string> safeFloorReason) {
    ModelingMesh modeling;
    modeling.vertices = certified.vertices;
    modeling.aliasesCertified = true;
    modeling.safeFloorReason = std::move(safeFloorReason);
    modeling.polygons.reserve(certified.triangles.size());
    for (const CertifiedTriangle& triangle : certified.triangles) {
        modeling.polygons.push_back(
            {triangle.workingFace, triangle.sourceFace,
             {triangle.vertices.begin(), triangle.vertices.end()}});
    }
    return modeling;
}

MeshingResult makeCertifiedFloorMeshingResult(
    CertifiedMesh certified,
    ValidationCertificate validation,
    GenerationReport generation,
    std::optional<std::string> safeFloorReason) {
    MeshingResult result;
    result.modeling = makeCertifiedFloorModelingMesh(
        certified, std::move(safeFloorReason));
    result.certified = std::move(certified);
    result.generation = std::move(generation);
    result.validation = std::move(validation);
    return result;
}

CertifiedMeshAssemblyResult assembleCertifiedPlanarMesh(
    const ImportedModel& imported,
    const CanonicalBoundarySet& boundaries,
    std::span<const PlanarCdtMesh> faceMeshes,
    std::span<const StableId> expectedWorkingFaces,
    const CertifiedMeshAssemblyConfiguration& configuration) {
    CertifiedMeshAssemblyResult result;
    result.validation.checks = {
        {"certified.face_coverage", expectedWorkingFaces.size()},
        {"certified.boundary_provenance", 0},
        {"certified.canonical_vertex_identity", 0},
        {"certified.triangle_provenance", 0},
        {"certified.triangle_geometry", 0},
        {"certified.edge_incidence", 0},
        {"certified.edge_winding", 0},
        {"certified.fingerprint", 1},
    };
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator || !boundaries.validation.complete()) {
        ++result.validation.checks[FaceCoverage].failed;
        setFailure(result, "certified.prerequisite_incomplete",
                   "certified assembly requires a meshable working model and complete boundaries",
                   {});
        return result;
    }
    if (!std::isfinite(configuration.maximumVertexSurfaceDiscrepancy) ||
        configuration.maximumVertexSurfaceDiscrepancy < 0.0) {
        ++result.validation.checks[FaceCoverage].failed;
        setFailure(result, "certified.invalid_configuration",
                   "the vertex-on-surface discrepancy limit is invalid", {});
        return result;
    }

    std::set<StableId> expectedFaces;
    for (StableId face : expectedWorkingFaces) {
        if (face.kind != StableIdKind::Face || !face.valid() ||
            !expectedFaces.insert(face).second) {
            ++result.validation.checks[FaceCoverage].failed;
            setFailure(result, "certified.expected_face_set_invalid",
                       "expected working faces must be unique valid face IDs",
                       {face});
            return result;
        }
    }
    if (expectedFaces.empty()) {
        ++result.validation.checks[FaceCoverage].failed;
        setFailure(result, "certified.expected_face_set_empty",
                   "a certified body cannot be assembled without expected faces",
                   {});
        return result;
    }
    std::map<StableId, const PlanarCdtMesh*> meshesByFace;
    for (const PlanarCdtMesh& mesh : faceMeshes) {
        if (!meshesByFace.emplace(mesh.workingFace, &mesh).second) {
            ++result.validation.checks[FaceCoverage].failed;
            setFailure(result, "certified.duplicate_face_mesh",
                       "a working face has more than one certified face mesh",
                       {mesh.workingFace});
            return result;
        }
    }
    ValidationCoverage& faceCoverage =
        result.validation.checks[FaceCoverage];
    for (StableId face : expectedFaces) {
        ++faceCoverage.checked;
        if (!meshesByFace.contains(face)) {
            ++faceCoverage.failed;
            setFailure(result, "certified.face_mesh_missing",
                       "an expected working face has no certified mesh", {face});
        }
    }
    for (const auto& [face, mesh] : meshesByFace) {
        (void)mesh;
        if (!expectedFaces.contains(face)) {
            ++faceCoverage.failed;
            setFailure(result, "certified.unexpected_face_mesh",
                       "a certified face mesh is outside the expected face set",
                       {face});
        }
    }
    if (faceCoverage.failed != 0) return result;

    std::map<std::uint64_t, CertifiedVertex> verticesByCanonicalIndex;
    ValidationCoverage& boundaryProvenance =
        result.validation.checks[BoundaryProvenance];
    ValidationCoverage& identity =
        result.validation.checks[CanonicalIdentity];
    for (const auto& [face, meshPointer] : meshesByFace) {
        const PlanarCdtMesh& faceMesh = *meshPointer;
        if (faceMesh.workingFace != face ||
            faceMesh.sourceFace == std::nullopt) {
            ++identity.failed;
            setFailure(result, "certified.face_provenance_invalid",
                       "a planar face mesh lacks source/working face provenance",
                       {face});
            return result;
        }
        identity.expected += faceMesh.vertices.size();
        for (const PlanarTrimVertex& localVertex : faceMesh.vertices) {
            ++identity.checked;
            boundaryProvenance.expected += localVertex.boundaryUses.size();
            if (localVertex.boundaryUses.empty() ||
                localVertex.canonicalVertexIndex ==
                    InvalidCanonicalVertexIndex ||
                localVertex.canonicalVertexIndex >=
                    boundaries.canonicalVertexCount) {
                ++identity.failed;
                setFailure(result, "certified.canonical_vertex_invalid",
                           "a face vertex has no valid canonical identity",
                           {face});
                return result;
            }

            std::optional<std::array<double, 3>> resolvedPosition;
            std::vector<CertifiedVertexUse> resolvedUses;
            for (const PlanarTrimBoundaryUse& boundaryUse :
                 localVertex.boundaryUses) {
                ++boundaryProvenance.checked;
                const CanonicalBoundarySample* sample =
                    resolveBoundarySample(boundaries, boundaryUse);
                CertifiedVertexUse certifiedUse{
                    face, faceMesh.sourceFace, boundaryUse};
                if (!sample ||
                    sample->canonicalVertexIndex !=
                        localVertex.canonicalVertexIndex ||
                    !resolvesFaceUse(*sample, certifiedUse)) {
                    ++boundaryProvenance.failed;
                    setFailure(result, "certified.boundary_provenance_invalid",
                               "a face vertex does not resolve through its canonical sample and coedge UV use",
                               {face, boundaryUse.workingEdge,
                                boundaryUse.coedge});
                    return result;
                }
                if (resolvedPosition &&
                    !exactPositionEqual(*resolvedPosition, sample->position)) {
                    ++identity.failed;
                    setFailure(result, "certified.vertex_position_mismatch",
                               "incident canonical samples disagree exactly in 3D",
                               {face, boundaryUse.workingEdge});
                    return result;
                }
                resolvedPosition = sample->position;
                resolvedUses.push_back(std::move(certifiedUse));
            }

            auto [found, inserted] = verticesByCanonicalIndex.emplace(
                localVertex.canonicalVertexIndex,
                CertifiedVertex{localVertex.canonicalVertexIndex,
                                *resolvedPosition, {}});
            CertifiedVertex& global = found->second;
            if (!inserted &&
                !exactPositionEqual(global.position, *resolvedPosition)) {
                ++identity.failed;
                setFailure(result, "certified.vertex_position_mismatch",
                           "one canonical vertex resolves to different 3D positions",
                           {face});
                return result;
            }
            for (CertifiedVertexUse& use : resolvedUses) {
                const bool duplicate = std::any_of(
                    global.provenance.begin(), global.provenance.end(),
                    [&](const CertifiedVertexUse& existing) {
                        return sameProvenance(existing, use);
                    });
                if (!duplicate) global.provenance.push_back(std::move(use));
            }
        }
    }

    CertifiedMesh mesh;
    if (verticesByCanonicalIndex.size() >
        static_cast<std::size_t>(
            std::numeric_limits<std::uint32_t>::max())) {
        ++identity.failed;
        setFailure(result, "certified.vertex_index_overflow",
                   "the certified mesh exceeds the global index width", {});
        return result;
    }
    mesh.vertices.reserve(verticesByCanonicalIndex.size());
    std::map<std::uint64_t, std::uint32_t> canonicalToGlobal;
    for (auto& [canonicalIndex, vertex] : verticesByCanonicalIndex) {
        std::sort(vertex.provenance.begin(), vertex.provenance.end(),
                  provenanceLess);
        const std::uint32_t globalIndex =
            static_cast<std::uint32_t>(mesh.vertices.size());
        canonicalToGlobal.emplace(canonicalIndex, globalIndex);
        mesh.vertices.push_back(std::move(vertex));
    }

    ValidationCoverage& triangleProvenance =
        result.validation.checks[TriangleProvenance];
    ValidationCoverage& triangleGeometry =
        result.validation.checks[TriangleGeometry];
    for (const auto& [face, meshPointer] : meshesByFace) {
        const PlanarCdtMesh& faceMesh = *meshPointer;
        const std::optional<TopologyOrientation> orientation =
            faceOrientation(imported.working->snapshot, face);
        if (!orientation ||
            (*orientation != TopologyOrientation::Forward &&
             *orientation != TopologyOrientation::Reversed)) {
            ++triangleProvenance.failed;
            setFailure(result, "certified.face_orientation_invalid",
                       "the working face has no orientable topology occurrence",
                       {face});
            return result;
        }
        triangleProvenance.expected += faceMesh.triangles.size();
        triangleGeometry.expected += faceMesh.triangles.size();
        for (const PlanarCdtTriangle& localTriangle : faceMesh.triangles) {
            ++triangleProvenance.checked;
            ++triangleGeometry.checked;
            if (localTriangle.workingFace != face ||
                localTriangle.sourceFace != faceMesh.sourceFace) {
                ++triangleProvenance.failed;
                setFailure(result, "certified.triangle_provenance_invalid",
                           "a face triangle has inconsistent face provenance",
                           {face});
                return result;
            }
            CertifiedTriangle triangle;
            triangle.workingFace = face;
            triangle.sourceFace = faceMesh.sourceFace;
            std::array<std::uint32_t, 3> orientedLocalIndices =
                localTriangle.vertices;
            for (std::size_t corner = 0; corner < 3; ++corner) {
                const std::uint32_t localIndex =
                    localTriangle.vertices[corner];
                if (localIndex >= faceMesh.vertices.size()) {
                    ++triangleProvenance.failed;
                    setFailure(result, "certified.triangle_vertex_invalid",
                               "a face triangle references a missing local vertex",
                               {face});
                    return result;
                }
                const PlanarTrimVertex& localVertex =
                    faceMesh.vertices[localIndex];
                triangle.vertices[corner] = canonicalToGlobal.at(
                    localVertex.canonicalVertexIndex);
                triangle.cornerUv[corner] = localVertex.uv;
            }
            if (*orientation == TopologyOrientation::Reversed) {
                std::swap(triangle.vertices[1], triangle.vertices[2]);
                std::swap(triangle.cornerUv[1], triangle.cornerUv[2]);
                std::swap(orientedLocalIndices[1], orientedLocalIndices[2]);
            }

            const std::array<double, 3>& a =
                mesh.vertices[triangle.vertices[0]].position;
            const std::array<double, 3>& b =
                mesh.vertices[triangle.vertices[1]].position;
            const std::array<double, 3>& c =
                mesh.vertices[triangle.vertices[2]].position;
            const std::array<double, 3> normal = triangleNormal(a, b, c);
            const double normalSquared = dot(normal, normal);
            if (!std::isfinite(normalSquared) || !(normalSquared > 0.0)) {
                ++triangleGeometry.failed;
                setFailure(result, "certified.triangle_degenerate",
                           "a certified triangle has zero or non-finite 3D area",
                           {face});
                return result;
            }
            const EvaluationResult<SurfaceEvaluation> surface =
                imported.workingEvaluator->evaluateSurface(
                    face, triangle.cornerUv[0]);
            if (!surface || !surface.value->unitNormal) {
                ++triangleGeometry.failed;
                setFailure(result, "certified.surface_normal_unavailable",
                           "the exact planar surface has no evaluable normal",
                           {face});
                return result;
            }
            // GeometryEvaluator already applies TopoDS face orientation to
            // unitNormal. The triangle was swapped above for the same face
            // orientation, so direct positive alignment is required.
            if (!(dot(normal, *surface.value->unitNormal) > 0.0)) {
                ++triangleGeometry.failed;
                setFailure(result, "certified.triangle_orientation_invalid",
                           "a triangle winding disagrees with the oriented B-rep face normal",
                           {face});
                return result;
            }
            for (std::size_t corner = 0; corner < 3; ++corner) {
                const EvaluationResult<SurfaceEvaluation> evaluated =
                    imported.workingEvaluator->evaluateSurface(
                        face, triangle.cornerUv[corner]);
                const PlanarTrimVertex& localVertex =
                    faceMesh.vertices[orientedLocalIndices[corner]];
                double allowed =
                    configuration.maximumVertexSurfaceDiscrepancy;
                for (const PlanarTrimBoundaryUse& use :
                     localVertex.boundaryUses) {
                    allowed = std::min(
                        allowed,
                        use.allowedCurveOnSurfaceDiscrepancy);
                }
                const double maximumSquared = allowed * allowed;
                if (!evaluated ||
                    squaredDistance(
                        mesh.vertices[triangle.vertices[corner]].position,
                        evaluated.value->position) > maximumSquared) {
                    ++triangleGeometry.failed;
                    setFailure(result, "certified.vertex_off_surface",
                               "a triangle vertex exceeds the exact planar surface discrepancy limit",
                               {face});
                    return result;
                }
            }
            mesh.triangles.push_back(std::move(triangle));
        }
    }
    std::sort(mesh.triangles.begin(), mesh.triangles.end(),
              [](const CertifiedTriangle& first,
                 const CertifiedTriangle& second) {
                  return std::tie(first.workingFace, first.vertices,
                                  first.cornerUv) <
                      std::tie(second.workingFace, second.vertices,
                               second.cornerUv);
              });

    std::map<Edge, std::vector<DirectedEdgeUse>> edgeUses;
    for (const CertifiedTriangle& triangle : mesh.triangles) {
        for (std::size_t local = 0; local < 3; ++local) {
            const std::uint32_t first = triangle.vertices[local];
            const std::uint32_t second = triangle.vertices[(local + 1) % 3];
            const Edge key = edge(first, second);
            edgeUses[key].push_back({first == key.lower});
        }
    }
    ValidationCoverage& incidence =
        result.validation.checks[EdgeIncidence];
    ValidationCoverage& winding = result.validation.checks[EdgeWinding];
    incidence.expected = edgeUses.size();
    for (const auto& [key, uses] : edgeUses) {
        (void)key;
        ++incidence.checked;
        const bool incidenceValid = configuration.requireClosedManifold
            ? uses.size() == 2
            : (uses.size() == 1 || uses.size() == 2);
        if (!incidenceValid) {
            ++incidence.failed;
            setFailure(result, "certified.edge_incidence_invalid",
                       "a global triangle edge has invalid manifold incidence",
                       {});
        }
        if (uses.size() == 2) {
            ++winding.expected;
            ++winding.checked;
            if (uses[0].lowerToUpper == uses[1].lowerToUpper) {
                ++winding.failed;
                setFailure(result, "certified.edge_winding_conflict",
                           "two triangles traverse a shared edge in the same direction",
                           {});
            }
        }
    }
    if (incidence.failed != 0 || winding.failed != 0) return result;

    mesh.topologyFingerprint = fingerprintOf(mesh);
    ValidationCoverage& fingerprint = result.validation.checks[Fingerprint];
    ++fingerprint.checked;
    if (mesh.topologyFingerprint.size() != 16) {
        ++fingerprint.failed;
        setFailure(result, "certified.fingerprint_failed",
                   "deterministic topology fingerprint generation failed", {});
        return result;
    }
    if (!result.validation.complete()) {
        setFailure(result, "certified.validation_incomplete",
                   "certified mesh validation coverage is incomplete", {});
        return result;
    }
    result.value = std::move(mesh);
    return result;
}

}  // namespace weft
