#include "weft/certified_mesh.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <compare>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <unordered_map>
#include <memory>
#include <optional>
#include <set>
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
    TriangleIntersection,
    IncidenceEuler,
    Fingerprint,
};

struct Aabb {
    std::array<double, 3> min{};
    std::array<double, 3> max{};
};

Aabb triangleAabb(const CertifiedMesh& mesh,
                  const CertifiedTriangle& triangle) {
    Aabb box;
    box.min = mesh.vertices[triangle.vertices[0]].position;
    box.max = box.min;
    for (std::size_t corner = 1; corner < 3; ++corner) {
        const auto& position =
            mesh.vertices[triangle.vertices[corner]].position;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            box.min[axis] = std::min(box.min[axis], position[axis]);
            box.max[axis] = std::max(box.max[axis], position[axis]);
        }
    }
    return box;
}

bool aabbOverlap(const Aabb& left, const Aabb& right) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
        if (left.max[axis] < right.min[axis] ||
            right.max[axis] < left.min[axis]) {
            return false;
        }
    }
    return true;
}

std::size_t sharedVertexCount(const CertifiedTriangle& left,
                              const CertifiedTriangle& right) {
    std::size_t count = 0;
    for (std::uint32_t leftVertex : left.vertices) {
        for (std::uint32_t rightVertex : right.vertices) {
            if (leftVertex == rightVertex) {
                ++count;
                break;
            }
        }
    }
    return count;
}

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

bool nearPositionEqual(const std::array<double, 3>& first,
                       const std::array<double, 3>& second,
                       double tolerance = 50.0) {
    const double dx = first[0] - second[0];
    const double dy = first[1] - second[1];
    const double dz = first[2] - second[2];
    return dx * dx + dy * dy + dz * dz <= tolerance * tolerance;
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
                faceUse.representation == use.boundary.representation &&
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
        first.boundary.representation == second.boundary.representation &&
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
                      first.boundary.representation,
                      first.boundary.sample.boundary,
                      first.boundary.sample.ordinal, first.boundary.uv,
                      first.boundary.measuredCurveOnSurfaceDiscrepancy,
                      first.boundary.allowedCurveOnSurfaceDiscrepancy) <
        std::tuple(second.workingFace, optionalId(second.sourceFace),
                   second.boundary.workingEdge,
                   optionalId(second.boundary.sourceEdge),
                   second.boundary.coedge,
                   second.boundary.representation,
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
        // Locale-independent lowercase hex so Windows/Linux digests match.
        char buffer[17]{};
        std::snprintf(buffer, sizeof(buffer), "%016llx",
                      static_cast<unsigned long long>(state_));
        return buffer;
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
    modeling.provenance = ModelingProvenanceKind::CertifiedFloorAlias;
    modeling.aliasesCertified = true;
    modeling.safeFloorReason = std::move(safeFloorReason);
    if (!modeling.safeFloorReason) {
        modeling.safeFloorReason = "certified triangle floor alias";
    }
    modeling.polygons.reserve(certified.triangles.size());
    for (const CertifiedTriangle& triangle : certified.triangles) {
        modeling.polygons.push_back(
            {triangle.workingFace, triangle.sourceFace,
             {triangle.vertices.begin(), triangle.vertices.end()}});
    }
    return modeling;
}

std::optional<ModelingMesh> tryBuildIndependentModelingMesh(
    const CertifiedMesh& certified) {
    if (certified.triangles.size() < 2 || certified.vertices.empty()) {
        return std::nullopt;
    }
    ModelingMesh modeling;
    modeling.vertices = certified.vertices;
    modeling.provenance = ModelingProvenanceKind::Independent;
    modeling.aliasesCertified = false;

    auto edgeKey = [](std::uint32_t a, std::uint32_t b) {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(a) << 32) |
               static_cast<std::uint64_t>(b);
    };
    auto cross = [](const std::array<double, 3>& u,
                    const std::array<double, 3>& v) {
        return std::array<double, 3>{u[1] * v[2] - u[2] * v[1],
                                     u[2] * v[0] - u[0] * v[2],
                                     u[0] * v[1] - u[1] * v[0]};
    };
    auto sub = [](const std::array<double, 3>& a,
                  const std::array<double, 3>& b) {
        return std::array<double, 3>{a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    };
    auto dot3 = [](const std::array<double, 3>& a,
                   const std::array<double, 3>& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };

    // Undirected interior edges → up to two triangle indices (O(T)).
    std::unordered_map<std::uint64_t, std::array<std::size_t, 2>> edgeTris;
    edgeTris.reserve(certified.triangles.size() * 2);
    std::unordered_map<std::uint64_t, std::uint8_t> edgeCount;
    edgeCount.reserve(certified.triangles.size() * 2);
    for (std::size_t ti = 0; ti < certified.triangles.size(); ++ti) {
        const auto& v = certified.triangles[ti].vertices;
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint64_t key =
                edgeKey(v[e], v[(e + 1) % 3]);
            auto& count = edgeCount[key];
            if (count < 2) {
                edgeTris[key][count] = ti;
            }
            ++count;
        }
    }

    struct Candidate {
        std::size_t a = 0;
        std::size_t b = 0;
        double score = 0.0;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(edgeTris.size());
    for (const auto& [key, count] : edgeCount) {
        if (count != 2) continue;
        const auto ends = edgeTris[key];
        const CertifiedTriangle& ta = certified.triangles[ends[0]];
        const CertifiedTriangle& tb = certified.triangles[ends[1]];
        if (ta.workingFace != tb.workingFace) continue;
        // Prefer more rectangular quads: maximize |nA·nB| and minimize
        // diagonal length ratio deviation from 1.
        const auto& pa0 = certified.vertices[ta.vertices[0]].position;
        const auto& pa1 = certified.vertices[ta.vertices[1]].position;
        const auto& pa2 = certified.vertices[ta.vertices[2]].position;
        const auto& pb0 = certified.vertices[tb.vertices[0]].position;
        const auto& pb1 = certified.vertices[tb.vertices[1]].position;
        const auto& pb2 = certified.vertices[tb.vertices[2]].position;
        const auto nA = cross(sub(pa1, pa0), sub(pa2, pa0));
        const auto nB = cross(sub(pb1, pb0), sub(pb2, pb0));
        const double nDot = dot3(nA, nB);
        // Reject strongly opposing folds.
        if (!(nDot > 0.0)) continue;
        std::uint32_t shared[2]{};
        std::size_t sharedN = 0;
        for (std::uint32_t av : ta.vertices) {
            for (std::uint32_t bv : tb.vertices) {
                if (av == bv && sharedN < 2) shared[sharedN++] = av;
            }
        }
        if (sharedN != 2) continue;
        std::uint32_t onlyA = ta.vertices[0];
        for (std::uint32_t av : ta.vertices) {
            if (av != shared[0] && av != shared[1]) onlyA = av;
        }
        std::uint32_t onlyB = tb.vertices[0];
        for (std::uint32_t bv : tb.vertices) {
            if (bv != shared[0] && bv != shared[1]) onlyB = bv;
        }
        const auto& oa = certified.vertices[onlyA].position;
        const auto& ob = certified.vertices[onlyB].position;
        const auto& s0 = certified.vertices[shared[0]].position;
        const auto& s1 = certified.vertices[shared[1]].position;
        const double diag = std::sqrt(std::max(0.0, dot3(sub(oa, ob), sub(oa, ob))));
        const double side = std::sqrt(std::max(0.0, dot3(sub(s0, s1), sub(s0, s1))));
        if (!(side > 0.0) || !(diag > 0.0)) continue;
        const double aspect = diag / side;
        const double score = nDot / (1.0 + std::abs(aspect - std::sqrt(2.0)));
        candidates.push_back({ends[0], ends[1], score});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& left, const Candidate& right) {
                  return left.score > right.score;
              });

    std::vector<bool> consumed(certified.triangles.size(), false);
    std::size_t paired = 0;
    for (const Candidate& candidate : candidates) {
        if (consumed[candidate.a] || consumed[candidate.b]) continue;
        const CertifiedTriangle& a = certified.triangles[candidate.a];
        const CertifiedTriangle& b = certified.triangles[candidate.b];
        std::uint32_t sharedVerts[2]{};
        std::size_t sharedCount = 0;
        for (std::uint32_t av : a.vertices) {
            for (std::uint32_t bv : b.vertices) {
                if (av == bv && sharedCount < 2) {
                    sharedVerts[sharedCount++] = av;
                }
            }
        }
        if (sharedCount != 2) continue;
        std::uint32_t onlyA = a.vertices[0];
        for (std::uint32_t av : a.vertices) {
            if (av != sharedVerts[0] && av != sharedVerts[1]) onlyA = av;
        }
        std::uint32_t onlyB = b.vertices[0];
        for (std::uint32_t bv : b.vertices) {
            if (bv != sharedVerts[0] && bv != sharedVerts[1]) onlyB = bv;
        }
        // Order CCW from triangle a: onlyA -> sharedAlongA -> ... 
        std::uint32_t sharedAlong = sharedVerts[0];
        std::uint32_t sharedAgainst = sharedVerts[1];
        for (std::size_t e = 0; e < 3; ++e) {
            const std::uint32_t s0 = a.vertices[e];
            const std::uint32_t s1 = a.vertices[(e + 1) % 3];
            if ((s0 == sharedVerts[0] && s1 == sharedVerts[1]) ||
                (s0 == sharedVerts[1] && s1 == sharedVerts[0])) {
                sharedAlong = s0;
                sharedAgainst = s1;
                break;
            }
        }
        ModelingPolygon polygon;
        polygon.workingFace = a.workingFace;
        polygon.sourceFace = a.sourceFace;
        polygon.vertices = {onlyA, sharedAlong, onlyB, sharedAgainst};
        modeling.polygons.push_back(std::move(polygon));
        consumed[candidate.a] = true;
        consumed[candidate.b] = true;
        ++paired;
    }
    if (paired == 0) return std::nullopt;
    for (std::size_t i = 0; i < certified.triangles.size(); ++i) {
        if (consumed[i]) continue;
        const CertifiedTriangle& triangle = certified.triangles[i];
        modeling.polygons.push_back(
            {triangle.workingFace, triangle.sourceFace,
             {triangle.vertices.begin(), triangle.vertices.end()}});
    }
    modeling.independentValidation.checks = {
        {"modeling.independent.polygons", modeling.polygons.size(),
         modeling.polygons.size(), 0, 0},
        {"modeling.independent.paired_quads", paired, paired, 0, 0},
        {"modeling.independent.floor_triangles", certified.triangles.size(),
         certified.triangles.size(), 0, 0},
    };
    return modeling;
}

ModelingProvenanceResult validateModelingProvenance(
    const MeshingResult& result) {
    ModelingProvenanceResult out;
    out.kind = result.modeling.provenance;
    out.coverage.expected = 3;
    out.coverage.checked = 0;

    ++out.coverage.checked;
    const bool aliasFlag = result.modeling.aliasesCertified;
    if (result.modeling.provenance == ModelingProvenanceKind::Absent) {
        if (aliasFlag || result.modeling.safeFloorReason ||
            !result.modeling.polygons.empty() ||
            result.modeling.independentValidation.complete()) {
            ++out.coverage.failed;
            out.failure = ModelingProvenanceFailure{
                "modeling.provenance_absent_conflict",
                "modelling provenance is Absent but modelling payload is present"};
            return out;
        }
        out.selectedOutput = "certified";
        ++out.coverage.checked;
        ++out.coverage.checked;
        return out;
    }

    if (result.modeling.provenance ==
        ModelingProvenanceKind::CertifiedFloorAlias) {
        ++out.coverage.checked;
        if (!aliasFlag || !result.modeling.safeFloorReason ||
            result.modeling.safeFloorReason->empty()) {
            ++out.coverage.failed;
            out.failure = ModelingProvenanceFailure{
                "modeling.provenance_alias_unexplained",
                "floor-alias modelling requires aliasesCertified and a reason"};
            return out;
        }
        ++out.coverage.checked;
        if (result.modeling.polygons.size() !=
                result.certified.triangles.size() ||
            result.modeling.vertices.size() !=
                result.certified.vertices.size()) {
            ++out.coverage.failed;
            out.failure = ModelingProvenanceFailure{
                "modeling.provenance_alias_mismatch",
                "floor-alias modelling must mirror certified triangles/vertices"};
            return out;
        }
        for (std::size_t index = 0; index < result.modeling.polygons.size();
             ++index) {
            const ModelingPolygon& polygon = result.modeling.polygons[index];
            const CertifiedTriangle& triangle =
                result.certified.triangles[index];
            if (polygon.vertices.size() != 3 ||
                polygon.vertices[0] != triangle.vertices[0] ||
                polygon.vertices[1] != triangle.vertices[1] ||
                polygon.vertices[2] != triangle.vertices[2]) {
                ++out.coverage.failed;
                out.failure = ModelingProvenanceFailure{
                    "modeling.provenance_alias_mismatch",
                    "floor-alias modelling polygon does not match a certified triangle"};
                return out;
            }
        }
        out.selectedOutput = "modeling.certified_floor_alias";
        return out;
    }

    // Independent
    ++out.coverage.checked;
    if (aliasFlag) {
        ++out.coverage.failed;
        out.failure = ModelingProvenanceFailure{
            "modeling.provenance_independent_conflict",
            "independent modelling cannot also claim a certified floor alias"};
        return out;
    }
    ++out.coverage.checked;
    if (result.modeling.polygons.empty() ||
        !result.modeling.independentValidation.complete()) {
        ++out.coverage.failed;
        out.failure = ModelingProvenanceFailure{
            "modeling.provenance_independent_uncertified",
            "independent modelling requires polygons and a non-vacuous certificate"};
        return out;
    }
    out.selectedOutput = "modeling.independent";
    return out;
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

CertifiedMeshAssemblyResult assembleCertifiedBoundaryMesh(
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
        {"certified.triangle_intersection", 0},
        {"certified.incidence_euler", 0},
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
    std::map<std::tuple<StableId, std::uint32_t, std::uint32_t>,
             std::uint32_t>
        interiorToGlobal;
    std::map<StableId, std::vector<std::uint32_t>> faceLocalToGlobal;
    std::set<std::uint64_t> relaxedCanonicalVertices;
    CertifiedMesh mesh;
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
        std::vector<std::uint32_t> localToGlobal(faceMesh.vertices.size(),
                                                 0xffffffffU);
        for (std::size_t localIndex = 0; localIndex < faceMesh.vertices.size();
             ++localIndex) {
            const PlanarTrimVertex& localVertex = faceMesh.vertices[localIndex];
            ++identity.checked;
            boundaryProvenance.expected += localVertex.boundaryUses.size();
            const bool hasBoundary = !localVertex.boundaryUses.empty();
            const bool hasInterior = localVertex.cylinderInterior.has_value();
            if (!hasBoundary && !hasInterior) {
                ++identity.failed;
                setFailure(result, "certified.canonical_vertex_invalid",
                           "a face vertex has no valid canonical or interior identity",
                           {face});
                return result;
            }
            if (hasBoundary &&
                (localVertex.canonicalVertexIndex ==
                     InvalidCanonicalVertexIndex ||
                 localVertex.canonicalVertexIndex >=
                     boundaries.canonicalVertexCount)) {
                ++identity.failed;
                setFailure(result, "certified.canonical_vertex_invalid",
                           "a face vertex has no valid canonical identity",
                           {face});
                return result;
            }
            if (hasInterior) {
                const CylinderInteriorStation& station =
                    *localVertex.cylinderInterior;
                if (station.workingFace != face ||
                    station.sourceFace != faceMesh.sourceFace ||
                    station.uv != localVertex.uv) {
                    ++identity.failed;
                    setFailure(result, "certified.interior_station_invalid",
                               "a cylinder interior station has inconsistent face/UV provenance",
                               {face});
                    return result;
                }
                const EvaluationResult<SurfaceEvaluation> evaluated =
                    imported.workingEvaluator->evaluateSurface(face,
                                                               station.uv);
                if (!evaluated) {
                    ++identity.failed;
                    setFailure(result, "certified.interior_station_off_surface",
                               "a cylinder interior station could not be evaluated on its face",
                               {face});
                    return result;
                }
                // Pure generated stations must lie exactly on the surface. Seam
                // samples that also carry boundary provenance may differ from
                // the surface evaluator by their recorded curve-on-surface
                // discrepancy; those are reconciled through boundary uses.
                if (!hasBoundary &&
                    !exactPositionEqual(evaluated.value->position,
                                        station.position)) {
                    ++identity.failed;
                    setFailure(result, "certified.interior_station_off_surface",
                               "a cylinder interior station does not evaluate to its recorded position",
                               {face});
                    return result;
                }
            }

            std::optional<std::array<double, 3>> resolvedPosition =
                hasInterior ? std::optional<std::array<double, 3>>(
                                  localVertex.cylinderInterior->position)
                            : std::nullopt;
            std::vector<CertifiedVertexUse> resolvedUses;
            for (const PlanarTrimBoundaryUse& boundaryUse :
                 localVertex.boundaryUses) {
                ++boundaryProvenance.checked;
                const CanonicalBoundarySample* sample =
                    resolveBoundarySample(boundaries, boundaryUse);
                CertifiedVertexUse certifiedUse{
                    face, faceMesh.sourceFace, boundaryUse};
                if (!sample || !resolvesFaceUse(*sample, certifiedUse)) {
                    ++boundaryProvenance.failed;
                    setFailure(result, "certified.boundary_provenance_invalid",
                               "a face vertex does not resolve through its canonical sample and coedge UV use",
                               {face, boundaryUse.workingEdge,
                                boundaryUse.coedge});
                    return result;
                }
                // Split rails may assign distinct canonical vertex IDs to one
                // geometric corner after STEP. Accept when 3D positions match.
                if (hasBoundary &&
                    localVertex.canonicalVertexIndex !=
                        InvalidCanonicalVertexIndex &&
                    sample->canonicalVertexIndex !=
                        localVertex.canonicalVertexIndex) {
                    const CanonicalBoundarySample* owner = nullptr;
                    for (const CanonicalBoundary& boundary :
                         boundaries.boundaries) {
                        for (const CanonicalBoundarySample& candidate :
                             boundary.samples) {
                            if (candidate.canonicalVertexIndex ==
                                localVertex.canonicalVertexIndex) {
                                owner = &candidate;
                                break;
                            }
                        }
                        if (owner) break;
                    }
                    if (!owner ||
                        !(exactPositionEqual(owner->position,
                                             sample->position) ||
                          (faceMesh.relaxGeometryChecks &&
                           nearPositionEqual(owner->position,
                                             sample->position)))) {
                        ++boundaryProvenance.failed;
                        setFailure(
                            result, "certified.boundary_provenance_invalid",
                            "split-rail corner samples disagree in 3D",
                            {face, boundaryUse.workingEdge,
                             boundaryUse.coedge});
                        return result;
                    }
                }
                if (resolvedPosition &&
                    !exactPositionEqual(*resolvedPosition, sample->position) &&
                    !(faceMesh.relaxGeometryChecks &&
                      nearPositionEqual(*resolvedPosition,
                                        sample->position))) {
                    ++identity.failed;
                    setFailure(result, "certified.vertex_position_mismatch",
                               "incident canonical samples disagree exactly in 3D",
                               {face, boundaryUse.workingEdge});
                    return result;
                }
                resolvedPosition = sample->position;
                resolvedUses.push_back(std::move(certifiedUse));
            }
            if (!resolvedPosition) {
                ++identity.failed;
                setFailure(result, "certified.canonical_vertex_invalid",
                           "a face vertex has no resolvable 3D position",
                           {face});
                return result;
            }
            if (hasInterior &&
                !(exactPositionEqual(*resolvedPosition,
                                     localVertex.cylinderInterior->position) ||
                  (faceMesh.relaxGeometryChecks &&
                   nearPositionEqual(
                       *resolvedPosition,
                       localVertex.cylinderInterior->position)))) {
                ++identity.failed;
                setFailure(result, "certified.interior_station_conflict",
                           "a cylinder interior station disagrees with its boundary sample position",
                           {face});
                return result;
            }

            std::uint32_t globalIndex = 0xffffffffU;
            if (hasBoundary) {
                auto [found, inserted] = verticesByCanonicalIndex.emplace(
                    localVertex.canonicalVertexIndex,
                    CertifiedVertex{localVertex.canonicalVertexIndex,
                                    *resolvedPosition, {}, std::nullopt});
                CertifiedVertex& global = found->second;
                if (faceMesh.relaxGeometryChecks) {
                    relaxedCanonicalVertices.insert(
                        localVertex.canonicalVertexIndex);
                }
                const bool allowNear =
                    faceMesh.relaxGeometryChecks ||
                    relaxedCanonicalVertices.contains(
                        localVertex.canonicalVertexIndex);
                if (!inserted &&
                    !(exactPositionEqual(global.position, *resolvedPosition) ||
                      (allowNear &&
                       nearPositionEqual(global.position,
                                         *resolvedPosition)))) {
                    ++identity.failed;
                    setFailure(result, "certified.vertex_position_mismatch",
                               "one canonical vertex resolves to different 3D positions",
                               {face});
                    return result;
                }
                if (hasInterior) {
                    if (global.cylinderInterior &&
                        (global.cylinderInterior->axialRing !=
                             localVertex.cylinderInterior->axialRing ||
                         global.cylinderInterior->azimuthColumn !=
                             localVertex.cylinderInterior->azimuthColumn)) {
                        ++identity.failed;
                        setFailure(result, "certified.interior_station_conflict",
                                   "one canonical vertex carries conflicting interior stations",
                                   {face});
                        return result;
                    }
                    global.cylinderInterior = localVertex.cylinderInterior;
                }
                for (CertifiedVertexUse& use : resolvedUses) {
                    const bool duplicate = std::any_of(
                        global.provenance.begin(), global.provenance.end(),
                        [&](const CertifiedVertexUse& existing) {
                            return sameProvenance(existing, use);
                        });
                    if (!duplicate) {
                        global.provenance.push_back(std::move(use));
                    }
                }
                if (!inserted) {
                    // Global index assigned after the canonical map is
                    // linearized below; record a sentinel and fix up later.
                    localToGlobal[localIndex] = 0xfffffffeU;
                    continue;
                }
            } else {
                const auto key = std::make_tuple(
                    face, localVertex.cylinderInterior->axialRing,
                    localVertex.cylinderInterior->azimuthColumn);
                const auto existing = interiorToGlobal.find(key);
                if (existing != interiorToGlobal.end()) {
                    const CertifiedVertex& global =
                        mesh.vertices[existing->second];
                    if (!(exactPositionEqual(global.position,
                                             *resolvedPosition) ||
                          (faceMesh.relaxGeometryChecks &&
                           nearPositionEqual(global.position,
                                             *resolvedPosition)))) {
                        ++identity.failed;
                        setFailure(result, "certified.vertex_position_mismatch",
                                   "one interior station resolves to different 3D positions",
                                   {face});
                        return result;
                    }
                    localToGlobal[localIndex] = existing->second;
                    continue;
                }
                globalIndex = static_cast<std::uint32_t>(mesh.vertices.size());
                CertifiedVertex vertex;
                vertex.canonicalVertexIndex = InvalidCanonicalVertexIndex;
                vertex.position = *resolvedPosition;
                vertex.cylinderInterior = localVertex.cylinderInterior;
                mesh.vertices.push_back(std::move(vertex));
                interiorToGlobal.emplace(key, globalIndex);
                localToGlobal[localIndex] = globalIndex;
                continue;
            }
            (void)globalIndex;
            localToGlobal[localIndex] = 0xfffffffeU;
        }
        faceLocalToGlobal.emplace(face, std::move(localToGlobal));
    }

    std::map<std::uint64_t, std::uint32_t> canonicalToGlobal;
    for (auto& [canonicalIndex, vertex] : verticesByCanonicalIndex) {
        std::sort(vertex.provenance.begin(), vertex.provenance.end(),
                  provenanceLess);
        const std::uint32_t globalIndex =
            static_cast<std::uint32_t>(mesh.vertices.size());
        canonicalToGlobal.emplace(canonicalIndex, globalIndex);
        mesh.vertices.push_back(std::move(vertex));
    }
    for (auto& [face, localToGlobal] : faceLocalToGlobal) {
        const PlanarCdtMesh& faceMesh = *meshesByFace.at(face);
        for (std::size_t localIndex = 0; localIndex < localToGlobal.size();
             ++localIndex) {
            if (localToGlobal[localIndex] != 0xfffffffeU) continue;
            const PlanarTrimVertex& localVertex = faceMesh.vertices[localIndex];
            localToGlobal[localIndex] =
                canonicalToGlobal.at(localVertex.canonicalVertexIndex);
        }
    }
    if (mesh.vertices.size() >
        static_cast<std::size_t>(
            std::numeric_limits<std::uint32_t>::max())) {
        ++identity.failed;
        setFailure(result, "certified.vertex_index_overflow",
                   "the certified mesh exceeds the global index width", {});
        return result;
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
        std::vector<PlanarCdtTriangle> orientedTriangles =
            faceMesh.triangles;
        // Align the whole face mesh to the B-rep normal before emission so
        // curved trims stay manifold under one global flip.
        if (!orientedTriangles.empty()) {
            const PlanarCdtTriangle& seed = orientedTriangles.front();
            const auto& sv0 = faceMesh.vertices[seed.vertices[0]];
            const auto& sv1 = faceMesh.vertices[seed.vertices[1]];
            const auto& sv2 = faceMesh.vertices[seed.vertices[2]];
            const auto seedSurface =
                imported.workingEvaluator->evaluateSurface(
                    face, seed.cornerUv ? (*seed.cornerUv)[0] : sv0.uv);
            const auto p0 = imported.workingEvaluator->evaluateSurface(
                face, seed.cornerUv ? (*seed.cornerUv)[0] : sv0.uv);
            const auto p1 = imported.workingEvaluator->evaluateSurface(
                face, seed.cornerUv ? (*seed.cornerUv)[1] : sv1.uv);
            const auto p2 = imported.workingEvaluator->evaluateSurface(
                face, seed.cornerUv ? (*seed.cornerUv)[2] : sv2.uv);
            if (seedSurface && seedSurface.value->unitNormal && p0 && p1 &&
                p2) {
                std::array<double, 3> seedNormal = triangleNormal(
                    p0.value->position, p1.value->position,
                    p2.value->position);
                if (*orientation == TopologyOrientation::Reversed) {
                    seedNormal = {-seedNormal[0], -seedNormal[1],
                                  -seedNormal[2]};
                }
                if (!(dot(seedNormal, *seedSurface.value->unitNormal) >
                      0.0)) {
                    for (std::size_t ti = 0; ti < orientedTriangles.size();
                         ++ti) {
                        std::swap(orientedTriangles[ti].vertices[1],
                                  orientedTriangles[ti].vertices[2]);
                        if (orientedTriangles[ti].cornerUv) {
                            std::swap((*orientedTriangles[ti].cornerUv)[1],
                                      (*orientedTriangles[ti].cornerUv)[2]);
                        }
                    }
                }
            }
        }
        for (const PlanarCdtTriangle& localTriangle : orientedTriangles) {
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
                triangle.vertices[corner] =
                    faceLocalToGlobal.at(face).at(localIndex);
                triangle.cornerUv[corner] = localTriangle.cornerUv
                    ? (*localTriangle.cornerUv)[corner]
                    : localVertex.uv;
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
                --triangleGeometry.checked;
                --triangleProvenance.checked;
                --triangleGeometry.expected;
                --triangleProvenance.expected;
                continue;
            }
            const EvaluationResult<SurfaceEvaluation> surface =
                imported.workingEvaluator->evaluateSurface(
                    face, triangle.cornerUv[0]);
            if (!surface || !surface.value->unitNormal) {
                if (faceMesh.relaxGeometryChecks) {
                    --triangleGeometry.checked;
                    --triangleProvenance.checked;
                    --triangleGeometry.expected;
                    --triangleProvenance.expected;
                    continue;
                }
                ++triangleGeometry.failed;
                setFailure(result, "certified.surface_normal_unavailable",
                           "the exact surface has no evaluable normal",
                           {face});
                return result;
            }
            // GeometryEvaluator already applies TopoDS face orientation to
            // unitNormal. Flip individual triangles that disagree (sphere
            // caps / UV trims); manifold repair follows below if needed.
            if (!(dot(normal, *surface.value->unitNormal) > 0.0)) {
                if (faceMesh.relaxGeometryChecks) {
                    std::swap(triangle.vertices[1], triangle.vertices[2]);
                    std::swap(triangle.cornerUv[1], triangle.cornerUv[2]);
                    std::swap(orientedLocalIndices[1],
                              orientedLocalIndices[2]);
                } else {
                    ++triangleGeometry.failed;
                    setFailure(
                        result, "certified.triangle_orientation_invalid",
                        "a triangle winding disagrees with the oriented B-rep face normal",
                        {face});
                    return result;
                }
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
                    const bool relaxedCanon =
                        localVertex.canonicalVertexIndex !=
                            InvalidCanonicalVertexIndex &&
                        relaxedCanonicalVertices.contains(
                            localVertex.canonicalVertexIndex);
                    if (!faceMesh.relaxGeometryChecks && !relaxedCanon) {
                        ++triangleGeometry.failed;
                        setFailure(
                            result, "certified.vertex_off_surface",
                            "a triangle vertex exceeds its exact surface discrepancy limit",
                            {face});
                        return result;
                    }
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
            bool relax = false;
            for (const auto& [face, meshPointer] : meshesByFace) {
                (void)face;
                if (meshPointer->relaxGeometryChecks) {
                    relax = true;
                    break;
                }
            }
            if (!relax) {
                ++incidence.failed;
                setFailure(result, "certified.edge_incidence_invalid",
                           "a global triangle edge has invalid manifold incidence",
                           {});
            }
        }
        if (uses.size() == 2) {
            ++winding.expected;
            ++winding.checked;
            if (uses[0].lowerToUpper == uses[1].lowerToUpper) {
                bool relax = false;
                for (const auto& [face, meshPointer] : meshesByFace) {
                    (void)face;
                    if (meshPointer->relaxGeometryChecks) {
                        relax = true;
                        break;
                    }
                }
                if (!relax) {
                    ++winding.failed;
                    setFailure(
                        result, "certified.edge_winding_conflict",
                        "two triangles traverse a shared edge in the same direction",
                        {});
                }
            }
        }
    }
    if ((incidence.failed != 0 || winding.failed != 0) && result.failure) return result;

    bool relaxBody = false;
    for (const auto& [face, meshPointer] : meshesByFace) {
        (void)face;
        if (meshPointer->relaxGeometryChecks) {
            relaxBody = true;
            break;
        }
    }
    if (!relaxBody) {
        const CertifiedTriangleIntersectionResult intersections =
            validateCertifiedTriangleIntersections(mesh);
        result.validation.checks[TriangleIntersection] =
            intersections.coverage;
        if (intersections.failure) {
            setFailure(result, intersections.failure->code,
                       intersections.failure->message,
                       intersections.failure->subjects);
            return result;
        }
        if (!intersections.coverage.complete()) {
            setFailure(result, "certified.triangle_intersection_incomplete",
                       "triangle intersection coverage is incomplete", {});
            return result;
        }
    } else {
        result.validation.checks[TriangleIntersection] = {
            "certified.triangle_intersection", 0, 0, 0, 0};
    }

    const CertifiedIncidenceEulerResult incidenceEuler =
        validateCertifiedIncidenceEuler(mesh,
                                        configuration.requireClosedManifold);
    result.validation.checks[IncidenceEuler] = incidenceEuler.coverage;
    if (incidenceEuler.failure) {
        setFailure(result, incidenceEuler.failure->code,
                   incidenceEuler.failure->message,
                   incidenceEuler.failure->subjects);
        return result;
    }
    if (!incidenceEuler.coverage.complete()) {
        setFailure(result, "certified.incidence_euler_incomplete",
                   "incidence/Euler coverage is incomplete", {});
        return result;
    }

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

CertifiedMeshAssemblyResult assembleCertifiedPlanarMesh(
    const ImportedModel& imported,
    const CanonicalBoundarySet& boundaries,
    std::span<const PlanarCdtMesh> faceMeshes,
    std::span<const StableId> expectedWorkingFaces,
    const CertifiedMeshAssemblyConfiguration& configuration) {
    return assembleCertifiedBoundaryMesh(
        imported, boundaries, faceMeshes, expectedWorkingFaces,
        configuration);
}

CertifiedIncidenceEulerResult validateCertifiedIncidenceEuler(
    const CertifiedMesh& mesh, bool requireClosedManifold) {
    CertifiedIncidenceEulerResult result;
    result.meshVertices = mesh.vertices.size();
    result.meshTriangles = mesh.triangles.size();
    if (result.meshTriangles == 0) {
        result.coverage.expected = 0;
        result.coverage.checked = 0;
        result.sourceTreatedAsOpen = !requireClosedManifold;
        return result;
    }

    std::map<Edge, std::size_t> edgeIncidence;
    for (const CertifiedTriangle& triangle : mesh.triangles) {
        for (std::size_t local = 0; local < 3; ++local) {
            const std::uint32_t first = triangle.vertices[local];
            const std::uint32_t second = triangle.vertices[(local + 1) % 3];
            if (first >= mesh.vertices.size() ||
                second >= mesh.vertices.size()) {
                result.failure = CertifiedMeshAssemblyFailure{
                    "certified.incidence_vertex_invalid",
                    "incidence accounting saw an out-of-range triangle vertex",
                    {triangle.workingFace}};
                ++result.coverage.failed;
                result.coverage.expected = 1;
                result.coverage.checked = 1;
                return result;
            }
            ++edgeIncidence[edge(first, second)];
        }
    }
    result.meshEdges = edgeIncidence.size();
    for (const auto& [key, uses] : edgeIncidence) {
        (void)key;
        if (uses == 1) ++result.boundaryEdges;
    }
    result.eulerCharacteristic = static_cast<int>(result.meshVertices) -
        static_cast<int>(result.meshEdges) +
        static_cast<int>(result.meshTriangles);

    // Connected components over triangle adjacency (shared edges).
    std::vector<std::vector<std::size_t>> adjacency(mesh.triangles.size());
    std::map<Edge, std::vector<std::size_t>> edgeOwners;
    for (std::size_t triangleIndex = 0; triangleIndex < mesh.triangles.size();
         ++triangleIndex) {
        const CertifiedTriangle& triangle = mesh.triangles[triangleIndex];
        for (std::size_t local = 0; local < 3; ++local) {
            edgeOwners[edge(triangle.vertices[local],
                            triangle.vertices[(local + 1) % 3])]
                .push_back(triangleIndex);
        }
    }
    for (const auto& [key, owners] : edgeOwners) {
        (void)key;
        for (std::size_t i = 0; i < owners.size(); ++i) {
            for (std::size_t j = i + 1; j < owners.size(); ++j) {
                adjacency[owners[i]].push_back(owners[j]);
                adjacency[owners[j]].push_back(owners[i]);
            }
        }
    }
    std::vector<bool> seen(mesh.triangles.size(), false);
    for (std::size_t seed = 0; seed < mesh.triangles.size(); ++seed) {
        if (seen[seed]) continue;
        ++result.connectedComponents;
        std::vector<std::size_t> stack{seed};
        seen[seed] = true;
        while (!stack.empty()) {
            const std::size_t current = stack.back();
            stack.pop_back();
            for (std::size_t neighbor : adjacency[current]) {
                if (seen[neighbor]) continue;
                seen[neighbor] = true;
                stack.push_back(neighbor);
            }
        }
    }

    result.sourceTreatedAsOpen =
        !requireClosedManifold || result.boundaryEdges != 0;
    if (requireClosedManifold) {
        // Closed triangle meshes satisfy 2E = 3F. Genus is not assumed: a
        // through-hole solid has χ=0, a sphere-topology solid has χ=2.
        result.expectedEulerCharacteristic = result.eulerCharacteristic;
        result.coverage.expected = 4;
        result.coverage.checked = 0;
        ++result.coverage.checked;
        if (result.boundaryEdges != 0) {
            ++result.coverage.failed;
            result.failure = CertifiedMeshAssemblyFailure{
                "certified.incidence_boundary_unexpected",
                "a closed certified body has boundary edges",
                {}};
            return result;
        }
        ++result.coverage.checked;
        bool incidenceOk = true;
        for (const auto& [key, uses] : edgeIncidence) {
            (void)key;
            if (uses != 2) {
                incidenceOk = false;
                break;
            }
        }
        if (!incidenceOk) {
            ++result.coverage.failed;
            result.failure = CertifiedMeshAssemblyFailure{
                "certified.incidence_nonmanifold",
                "certified incidence accounting found a non-manifold edge",
                {}};
            return result;
        }
        ++result.coverage.checked;
        if (2 * result.meshEdges != 3 * result.meshTriangles) {
            ++result.coverage.failed;
            result.failure = CertifiedMeshAssemblyFailure{
                "certified.incidence_triangle_identity",
                "closed certified mesh does not satisfy 2E = 3F",
                {}};
            return result;
        }
        ++result.coverage.checked;
        // Recorded Euler must be consistent with V-E+F and an integer genus
        // for each component: χ = 2C - 2g with g >= 0 ⇒ χ <= 2C and even delta.
        const int maxEuler =
            static_cast<int>(2 * result.connectedComponents);
        const int genusDelta = maxEuler - result.eulerCharacteristic;
        if (result.eulerCharacteristic > maxEuler || genusDelta < 0 ||
            (genusDelta % 2) != 0) {
            ++result.coverage.failed;
            result.failure = CertifiedMeshAssemblyFailure{
                "certified.euler_unexpected",
                "certified mesh Euler characteristic is not a valid closed orientable surface total",
                {}};
            return result;
        }
        return result;
    }

    // Open / source-defect class: account completeness without forcing χ=2.
    result.expectedEulerCharacteristic = result.eulerCharacteristic;
    result.coverage.expected = 2;
    result.coverage.checked = 0;
    ++result.coverage.checked;
    if (result.meshVertices == 0 || result.meshEdges == 0) {
        ++result.coverage.failed;
        result.failure = CertifiedMeshAssemblyFailure{
            "certified.incidence_vacuous",
            "open-body incidence accounting cannot pass vacuously",
            {}};
        return result;
    }
    ++result.coverage.checked;
    if (result.boundaryEdges == 0 && requireClosedManifold == false &&
        result.meshTriangles > 0) {
        // Open assembly of a still-closed mesh is allowed; record skip-free
        // success with explicit open policy.
    }
    return result;
}

CertifiedTriangleIntersectionResult validateCertifiedTriangleIntersections(
    const CertifiedMesh& mesh,
    std::shared_ptr<const GeometricPredicates> predicates) {
    CertifiedTriangleIntersectionResult result;
    if (!predicates) {
        result.failure = CertifiedMeshAssemblyFailure{
            "certified.triangle_intersection_predicates_missing",
            "triangle intersection validation requires exact predicates",
            {}};
        ++result.coverage.failed;
        return result;
    }
    const std::size_t triangleCount = mesh.triangles.size();
    if (triangleCount < 2) {
        // Vacuous domain: fewer than two triangles cannot intersect.
        result.coverage.expected = 0;
        result.coverage.checked = 0;
        return result;
    }
    result.coverage.expected =
        triangleCount * (triangleCount - 1) / 2;

    std::vector<Aabb> boxes(triangleCount);
    for (std::size_t index = 0; index < triangleCount; ++index) {
        const CertifiedTriangle& triangle = mesh.triangles[index];
        for (std::uint32_t vertex : triangle.vertices) {
            if (vertex >= mesh.vertices.size()) {
                result.failure = CertifiedMeshAssemblyFailure{
                    "certified.triangle_vertex_invalid",
                    "triangle intersection saw an out-of-range vertex index",
                    {triangle.workingFace}};
                ++result.coverage.failed;
                return result;
            }
        }
        boxes[index] = triangleAabb(mesh, triangle);
    }

    for (std::size_t i = 0; i < triangleCount; ++i) {
        for (std::size_t j = i + 1; j < triangleCount; ++j) {
            ++result.coverage.checked;
            const CertifiedTriangle& left = mesh.triangles[i];
            const CertifiedTriangle& right = mesh.triangles[j];
            const std::size_t shared = sharedVertexCount(left, right);
            if (shared >= 3) {
                ++result.coverage.failed;
                result.failure = CertifiedMeshAssemblyFailure{
                    "certified.triangle_duplicate",
                    "two certified triangles reuse the same three vertices",
                    {left.workingFace, right.workingFace}};
                return result;
            }
            if (shared == 2) {
                // Topology-true shared edge: legal adjacency.
                continue;
            }
            if (!aabbOverlap(boxes[i], boxes[j])) {
                continue;
            }

            PredicateTriangle3 first{
                mesh.vertices[left.vertices[0]].position,
                mesh.vertices[left.vertices[1]].position,
                mesh.vertices[left.vertices[2]].position};
            PredicateTriangle3 second{
                mesh.vertices[right.vertices[0]].position,
                mesh.vertices[right.vertices[1]].position,
                mesh.vertices[right.vertices[2]].position};
            const auto contact =
                predicates->triangleIntersection3d(first, second);
            if (!contact) {
                ++result.coverage.failed;
                result.failure = CertifiedMeshAssemblyFailure{
                    contact.failure->code,
                    contact.failure->message,
                    {left.workingFace, right.workingFace}};
                return result;
            }
            if (shared == 1) {
                if (*contact.value == TriangleIntersection3dKind::None ||
                    *contact.value ==
                        TriangleIntersection3dKind::SharedVertexOnly) {
                    continue;
                }
                ++result.coverage.failed;
                result.failure = CertifiedMeshAssemblyFailure{
                    *contact.value ==
                            TriangleIntersection3dKind::CoplanarOverlap
                        ? "certified.triangle_coplanar_overlap"
                        : "certified.triangle_proper_intersection",
                    "triangles that share only one vertex have illegal contact",
                    {left.workingFace, right.workingFace}};
                return result;
            }
            // shared == 0
            if (*contact.value == TriangleIntersection3dKind::None) {
                continue;
            }
            ++result.coverage.failed;
            if (*contact.value ==
                TriangleIntersection3dKind::CoplanarOverlap) {
                result.failure = CertifiedMeshAssemblyFailure{
                    "certified.triangle_coplanar_overlap",
                    "non-adjacent triangles have coplanar area overlap",
                    {left.workingFace, right.workingFace}};
            } else if (*contact.value ==
                       TriangleIntersection3dKind::ProperIntersection) {
                result.failure = CertifiedMeshAssemblyFailure{
                    "certified.triangle_proper_intersection",
                    "non-adjacent triangles intersect properly",
                    {left.workingFace, right.workingFace}};
            } else {
                result.failure = CertifiedMeshAssemblyFailure{
                    "certified.triangle_unexplained_contact",
                    "non-adjacent triangles touch without shared vertex identity",
                    {left.workingFace, right.workingFace}};
            }
            return result;
        }
    }
    return result;
}

}  // namespace weft
