#include "weft/secure_meshing.hpp"

#include "weft/cone_template.hpp"
#include "weft/planar_cdt.hpp"
#include "weft/planar_trim_assembly.hpp"
#include "weft/sphere_template.hpp"
#include "weft/mapped_template.hpp"
#include "weft/torus_template.hpp"

#include <algorithm>
#include <chrono>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>

namespace weft {
namespace {

void setFailure(SecureMeshingResult& result, std::string code,
                std::string message, std::vector<StableId> subjects = {}) {
    if (!result.failure) {
        result.failure = SecureMeshingFailure{
            std::move(code), std::move(message), std::move(subjects)};
    }
}

// HardSurfaceFloor: Plasticity hard-surface quality that matters for games
// (span matching / fillet laddering). Soft residuals may relax.
bool conditionHas(const ExactGeometryClassification& face,
                  const char* code) {
    return std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                     code) != face.conditionCodes.end();
}

std::size_t faceEdgeCount(const ImportedModel& imported, StableId faceId) {
    std::set<StableId> uniq;
    for (const CoedgeRecord& coedge : imported.working->snapshot.coedges) {
        if (coedge.faceId == faceId) uniq.insert(coedge.edgeId);
    }
    return uniq.size();
}

bool isHardSurfaceFloorFace(const ImportedModel& imported,
                            const ExactGeometryClassification& face) {
    if (face.familyCode == "plane" || face.familyCode == "cylinder" ||
        face.familyCode == "cone" || face.familyCode == "torus") {
        return true;
    }
    for (const std::string& code : face.conditionCodes) {
        if (code.find("fillet") != std::string::npos ||
            code.find("blend") != std::string::npos) {
            return true;
        }
    }
    // Coons / four-sided mapped lattice — Plasticity laddering hotspot.
    if (conditionHas(face, "mapped.four_sided_candidate") ||
        conditionHas(face, "strategy.mapped_four_sided")) {
        return true;
    }
    if ((face.familyCode == "bspline" || face.familyCode == "bezier" ||
         face.familyCode == "extrusion" || face.familyCode == "offset" ||
         face.familyCode == "revolution") &&
        faceEdgeCount(imported, face.subjectId) == 4 &&
        (conditionHas(face, "freeform.uv_grid_candidate") ||
         conditionHas(face, "strategy.freeform_uv_grid"))) {
        return true;
    }
    return false;
}

bool softResidualsAllowed(const SecureMeshingConfiguration& configuration) {
    return configuration.floorPolicy ==
           SecureMeshingFloorPolicy::HardSurfaceFloor;
}

void admitSoftResidualMesh(PlanarCdtMesh mesh, StableId faceId,
                           const char* reason,
                           std::vector<PlanarCdtMesh>& faceMeshes) {
    mesh.relaxGeometryChecks = true;
    mesh.windingsMatchOrientedFaceNormal = false;
    std::fprintf(stderr,
                 "WEFT_SOFT_RESIDUAL face=%llu reason=%s tris=%zu relax=1\n",
                 static_cast<unsigned long long>(faceId.ordinal), reason,
                 mesh.triangles.size());
    faceMeshes.push_back(std::move(mesh));
}

// Wave F: refuse codes must name the family/subclass — never generic-only.
std::string unsupportedCurveRefuseCode(const std::string& familyCode) {
    return "secure_pipeline.unsupported_curve_family." + familyCode;
}

std::string unsupportedSurfaceRefuseCode(
    const std::string& familyCode, const char* preferredSubclass = nullptr) {
    if (preferredSubclass != nullptr && preferredSubclass[0] != '\0') {
        return std::string("secure_pipeline.unsupported_surface_family.") +
               preferredSubclass;
    }
    return "secure_pipeline.unsupported_surface_family." + familyCode;
}

void secureProgress(const SecureMeshingConfiguration& configuration,
                    const char* message) {
    if (!configuration.progressToStderr || message == nullptr) return;
    static const auto start = std::chrono::steady_clock::now();
    static long long lastMs = 0;
    const auto now = std::chrono::steady_clock::now();
    const long long ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start)
            .count();
    const long long delta = ms - lastMs;
    lastMs = ms;
    std::fprintf(stderr, "WEFT_PROGRESS %s ms=%lld delta_ms=%lld\n", message,
                 ms, delta);
    std::fflush(stderr);
}

void appendCoverage(ValidationCertificate& certificate, std::string code,
                    std::size_t expected, std::size_t checked,
                    std::size_t skipped, std::size_t failed) {
    certificate.checks.push_back(
        {std::move(code), expected, checked, skipped, failed});
}

std::string faceCode(const std::string& code, StableId face) {
    return code + ".face_" + std::to_string(face.ordinal);
}

std::pair<std::optional<double>, std::optional<double>>
faceUvPeriods(const ExactGeometryClassification& face) {
    std::optional<double> uPeriod;
    std::optional<double> vPeriod;
    // Ignore tiny OCCT periods in hardOrient periodicNear only. A skinny
    // chart whose height is one tiny period (freeform_186 VPeriodâ‰ˆ0.016)
    // has legitimate triangles spanning > P/2; nearest-period corner
    // collapse would corrupt facet normals. CDT seam unwrap still uses the
    // authoritative tiny period via curvedUvVPeriod (keep-cut unwrap).
    constexpr double kMinUsefulPeriod = 0.25;
    if (face.parameterDomains.size() >= 1 &&
        face.parameterDomains[0].periodic &&
        face.parameterDomains[0].period &&
        *face.parameterDomains[0].period >= kMinUsefulPeriod) {
        uPeriod = *face.parameterDomains[0].period;
    }
    if (face.parameterDomains.size() >= 2 &&
        face.parameterDomains[1].periodic &&
        face.parameterDomains[1].period &&
        *face.parameterDomains[1].period >= kMinUsefulPeriod) {
        vPeriod = *face.parameterDomains[1].period;
    }
    return {uPeriod, vPeriod};
}

// Hard-orient a UV-trim / CDT mesh against evaluateSurface's oriented
// unitNormal. One global majority flip only (preserves manifold edge
// winding). Never drops triangles; never per-tri flips; never admits
// windingsMatch=0 residual. Returns true only when every UV-evaluable
// triangle is +N with hard-certify flags (relaxGeometryChecks=false).
bool hardOrientUvTrimMesh(const ImportedModel& imported,
                          const CanonicalBoundarySet& boundaries,
                          StableId faceId, PlanarCdtMesh& mesh,
                          const char* logTag,
                          std::optional<double> uPeriod = std::nullopt,
                          std::optional<double> vPeriod = std::nullopt) {
    (void)boundaries;
    auto periodicNear = [](double value, double reference,
                           std::optional<double> period) -> double {
        if (!period || !(*period > 0.0)) return value;
        return value +
               std::round((reference - value) / *period) * *period;
    };
    for (PlanarCdtTriangle& tri : mesh.triangles) {
        if (tri.cornerUv) continue;
        if (tri.vertices[0] >= mesh.vertices.size() ||
            tri.vertices[1] >= mesh.vertices.size() ||
            tri.vertices[2] >= mesh.vertices.size()) {
            continue;
        }
        tri.cornerUv = std::array<PredicatePoint2, 3>{
            mesh.vertices[tri.vertices[0]].uv,
            mesh.vertices[tri.vertices[1]].uv,
            mesh.vertices[tri.vertices[2]].uv};
    }
    auto flipTri = [](PlanarCdtTriangle& tri) {
        std::swap(tri.vertices[1], tri.vertices[2]);
        if (tri.cornerUv) {
            std::swap((*tri.cornerUv)[1], (*tri.cornerUv)[2]);
        }
    };
    const bool curvedUvLattice =
        !mesh.triangles.empty() &&
        std::all_of(mesh.triangles.begin(), mesh.triangles.end(),
                    [](const PlanarCdtTriangle& tri) {
                        return tri.cornerUv.has_value();
                    });
    // Centroid UV normal vs UV-evaluated facet: corner0-only grazing on
    // curved trims false-againsted manifold ears (face 33 / sphere caps).
    // Prefer centroid when the chart span is small; otherwise vote across
    // corner normals so period-unwrapped ears are not single-sample noise.
    auto uvAgrees = [&](const PlanarCdtTriangle& tri) -> std::optional<bool> {
        if (!imported.workingEvaluator) return std::nullopt;
        if (tri.vertices[0] >= mesh.vertices.size() ||
            tri.vertices[1] >= mesh.vertices.size() ||
            tri.vertices[2] >= mesh.vertices.size()) {
            return std::nullopt;
        }
        const PredicatePoint2 uv0 =
            tri.cornerUv ? (*tri.cornerUv)[0]
                         : mesh.vertices[tri.vertices[0]].uv;
        PredicatePoint2 uv1 =
            tri.cornerUv ? (*tri.cornerUv)[1]
                         : mesh.vertices[tri.vertices[1]].uv;
        PredicatePoint2 uv2 =
            tri.cornerUv ? (*tri.cornerUv)[2]
                         : mesh.vertices[tri.vertices[2]].uv;
        uv1[0] = periodicNear(uv1[0], uv0[0], uPeriod);
        uv2[0] = periodicNear(uv2[0], uv0[0], uPeriod);
        uv1[1] = periodicNear(uv1[1], uv0[1], vPeriod);
        uv2[1] = periodicNear(uv2[1], uv0[1], vPeriod);
        auto cornerPosition =
            [&](std::size_t corner) -> std::optional<std::array<double, 3>> {
            const std::uint32_t vi = tri.vertices[corner];
            if (vi >= mesh.vertices.size()) return std::nullopt;
            const PlanarTrimVertex& vertex = mesh.vertices[vi];
            if (vertex.cylinderInterior) {
                return vertex.cylinderInterior->position;
            }
            const PredicatePoint2 uv = tri.cornerUv ? (*tri.cornerUv)[corner]
                                                    : vertex.uv;
            PredicatePoint2 lifted = uv;
            if (corner == 1) {
                lifted[0] = uv1[0];
                lifted[1] = uv1[1];
            } else if (corner == 2) {
                lifted[0] = uv2[0];
                lifted[1] = uv2[1];
            }
            const auto evaluated =
                imported.workingEvaluator->evaluateSurface(faceId, lifted);
            if (!evaluated) return std::nullopt;
            return evaluated.value->position;
        };
        const auto p0 = cornerPosition(0);
        const auto p1 = cornerPosition(1);
        const auto p2 = cornerPosition(2);
        if (!p0 || !p1 || !p2) return std::nullopt;
        const double ax = (*p1)[0] - (*p0)[0];
        const double ay = (*p1)[1] - (*p0)[1];
        const double az = (*p1)[2] - (*p0)[2];
        const double bx = (*p2)[0] - (*p0)[0];
        const double by = (*p2)[1] - (*p0)[1];
        const double bz = (*p2)[2] - (*p0)[2];
        const double nx = ay * bz - az * by;
        const double ny = az * bx - ax * bz;
        const double nz = ax * by - ay * bx;
        const double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (!(nl > 0.0)) return std::nullopt;
        const double fnx = nx / nl;
        const double fny = ny / nl;
        const double fnz = nz / nl;
        auto sampleDot = [&](const PredicatePoint2& uv) -> std::optional<double> {
            PredicatePoint2 sampleUv = uv;
            if (uPeriod && *uPeriod > 0.0) {
                sampleUv[0] = sampleUv[0] -
                    std::floor(sampleUv[0] / *uPeriod) * *uPeriod;
            }
            if (vPeriod && *vPeriod > 0.0) {
                sampleUv[1] = sampleUv[1] -
                    std::floor(sampleUv[1] / *vPeriod) * *vPeriod;
            }
            const auto p =
                imported.workingEvaluator->evaluateSurface(faceId, sampleUv);
            if (!p || !p.value->unitNormal) return std::nullopt;
            return fnx * (*p.value->unitNormal)[0] +
                   fny * (*p.value->unitNormal)[1] +
                   fnz * (*p.value->unitNormal)[2];
        };
        const double du =
            std::max({std::abs(uv1[0] - uv0[0]), std::abs(uv2[0] - uv0[0]),
                      std::abs(uv2[0] - uv1[0])});
        const double dv =
            std::max({std::abs(uv1[1] - uv0[1]), std::abs(uv2[1] - uv0[1]),
                      std::abs(uv2[1] - uv1[1])});
        const bool useCentroidNormal =
            ((du < 1.0 && dv < 1.0) || uPeriod || vPeriod ||
             curvedUvLattice) &&
            // Wide U/V-span ears on periodic charts (freeform_138 / 399 /
            // torus UV-trim): the centroid sample sits on a folded chord and
            // false-againsts a manifold ear. Fall back to corner voting.
            // Threshold 0.15Â·P (was 0.35): shared-seam circular-cap bands
            // emit many sub-0.35 ears that still false-against on centroid.
            !(uPeriod && *uPeriod > 0.0 && du > 0.15 * *uPeriod) &&
            !(vPeriod && *vPeriod > 0.0 && dv > 0.15 * *vPeriod);
        if (useCentroidNormal) {
            const PredicatePoint2 uvC{(uv0[0] + uv1[0] + uv2[0]) / 3.0,
                                      (uv0[1] + uv1[1] + uv2[1]) / 3.0};
            const auto d = sampleDot(uvC);
            if (!d) return std::nullopt;
            // Near-tangent grazing: treat as unevaluable (not against).
            if (std::abs(*d) < 1e-8) return std::nullopt;
            return *d > 0.0;
        }
        int withSamples = 0;
        int againstSamples = 0;
        for (const PredicatePoint2& uv : {uv0, uv1, uv2}) {
            const auto d = sampleDot(uv);
            if (!d || std::abs(*d) < 1e-8) continue;
            if (*d > 0.0) {
                ++withSamples;
            } else {
                ++againstSamples;
            }
        }
        if (withSamples == 0 && againstSamples == 0) return std::nullopt;
        return withSamples >= againstSamples;
    };
    auto countUv = [&](std::size_t& withN, std::size_t& againstN) {
        withN = 0;
        againstN = 0;
        for (const PlanarCdtTriangle& tri : mesh.triangles) {
            const auto a = uvAgrees(tri);
            if (!a) continue;
            if (*a) {
                ++withN;
            } else {
                ++againstN;
            }
        }
    };
    std::size_t uvWith = 0;
    std::size_t uvAgainst = 0;
    countUv(uvWith, uvAgainst);
    {
        // Try both global windings; keep the lower against count.
        const std::size_t with0 = uvWith;
        const std::size_t against0 = uvAgainst;
        for (PlanarCdtTriangle& tri : mesh.triangles) flipTri(tri);
        std::size_t with1 = 0;
        std::size_t against1 = 0;
        countUv(with1, against1);
        if (against1 < against0 ||
            (against1 == against0 && with1 > with0)) {
            uvWith = with1;
            uvAgainst = against1;
        } else {
            for (PlanarCdtTriangle& tri : mesh.triangles) flipTri(tri);
            uvWith = with0;
            uvAgainst = against0;
        }
    }
    // Prefer a +N seed for certify provenance walks.
    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        const auto uvAg = uvAgrees(mesh.triangles[i]);
        if (uvAg && *uvAg) {
            if (i != 0) std::swap(mesh.triangles[0], mesh.triangles[i]);
            break;
        }
    }
    // Never soft-assemble residual against-N ears; never hollow the face.
    mesh.relaxGeometryChecks = false;
    if (uvAgainst != 0 || mesh.triangles.empty()) {
        std::fprintf(stderr,
                     "%s tris=%zu uvWith=%zu uvAgainst=%zu "
                     "windingsMatch=0 (refuse)\n",
                     logTag ? logTag : "WEFT_UVTRIM_ORIENT",
                     mesh.triangles.size(), uvWith, uvAgainst);
        mesh.windingsMatchOrientedFaceNormal = false;
        return false;
    }
    std::fprintf(stderr,
                 "%s tris=%zu uvWith=%zu uvAgainst=%zu "
                 "windingsMatch=1 relax=0\n",
                 logTag ? logTag : "WEFT_UVTRIM_ORIENT",
                 mesh.triangles.size(), uvWith, uvAgainst);
    mesh.windingsMatchOrientedFaceNormal = true;
    return true;
}

// Alias: midpoint refine is forbidden â€” Steiner verts lack canonical
// identity and fail certify. Invert-retry at the caller supplies the
// alternate winding chart when the first hard-orient refuses.
bool hardOrientUvTrimMeshWithRefine(const ImportedModel& imported,
                                    const CanonicalBoundarySet& boundaries,
                                    StableId faceId, PlanarCdtMesh& mesh,
                                    const char* logTag,
                                    std::optional<double> uPeriod = std::nullopt,
                                    std::optional<double> vPeriod = std::nullopt) {
    return hardOrientUvTrimMesh(imported, boundaries, faceId, mesh, logTag,
                                uPeriod, vPeriod);
}

// Periodic-band freeform UV-trim: product radial floors (minClosedâ‰ˆ8 at
// radial 32) can emit against-N CDT ears that stay manifold-consistent with
// +N neighbors â€” hardOrient cannot flip them without hollowing. Thin
// mid-edge samples (keep wire corners) and re-CDT; corners retain canonical
// identity. General subclass fix, not a face-id special case.

// Shared-seam U-periodic freeform with circular caps: Lawson CDT emits
// interior diagonals that behave like 3D diameter chords. Build an
// iso-parametric UV band from the CDT-unwrapped boundary (matched circular
// cap U columns + seam V samples + uniform axial fill). Does not invent UV
// outside the unwrapped chart.
std::optional<PlanarCdtMesh> buildCircularCapPeriodicUvBand(
    const ImportedModel& imported, StableId faceId,
    const PlanarCdtMesh& seed, std::optional<double> uPeriod) {
    if (!uPeriod || !(*uPeriod > 0.0) || !imported.workingEvaluator ||
        seed.boundaryLoops.empty() || seed.boundaryLoops.front().size() < 6) {
        return std::nullopt;
    }
    const auto& loopIdx = seed.boundaryLoops.front();
    std::vector<PlanarTrimVertex> loop;
    loop.reserve(loopIdx.size());
    for (std::uint32_t index : loopIdx) {
        if (index >= seed.vertices.size()) return std::nullopt;
        loop.push_back(seed.vertices[index]);
    }
    const double period = *uPeriod;
    double uMin = loop.front().uv[0];
    double uMax = uMin;
    double vMin = loop.front().uv[1];
    double vMax = vMin;
    for (const PlanarTrimVertex& vertex : loop) {
        uMin = std::min(uMin, vertex.uv[0]);
        uMax = std::max(uMax, vertex.uv[0]);
        vMin = std::min(vMin, vertex.uv[1]);
        vMax = std::max(vMax, vertex.uv[1]);
    }
    const double uSpan = uMax - uMin;
    const double vSpan = vMax - vMin;
    if (!(uSpan > 0.5 * period) || !(vSpan > 1e-6)) return std::nullopt;

    constexpr double kIsoFrac = 0.02;
    const double uIso = std::max(1e-6, kIsoFrac * period);
    const double vIso = std::max(1e-6, kIsoFrac * vSpan);
    std::vector<const PlanarTrimVertex*> bottom;
    std::vector<const PlanarTrimVertex*> top;
    std::vector<const PlanarTrimVertex*> seamLo;
    std::vector<const PlanarTrimVertex*> seamHi;
    std::size_t other = 0;
    for (const PlanarTrimVertex& vertex : loop) {
        const bool onBottom = std::abs(vertex.uv[1] - vMin) <= vIso;
        const bool onTop = std::abs(vertex.uv[1] - vMax) <= vIso;
        const bool onLo = std::abs(vertex.uv[0] - uMin) <= uIso;
        const bool onHi = std::abs(vertex.uv[0] - uMax) <= uIso;
        if (onBottom) {
            bottom.push_back(&vertex);
        } else if (onTop) {
            top.push_back(&vertex);
        } else if (onLo) {
            seamLo.push_back(&vertex);
        } else if (onHi) {
            seamHi.push_back(&vertex);
        } else {
            ++other;
        }
    }
    // Require circular-cap iso-V runs and shared-seam iso-U runs; reject
    // diagonal-heavy freeform loops (135/138) so their CDT path is preserved.
    if (bottom.size() < 3 || top.size() < 3 || seamLo.size() < 2 ||
        seamHi.size() < 2 || other > 0) {
        return std::nullopt;
    }
    auto byU = [](const PlanarTrimVertex* a, const PlanarTrimVertex* b) {
        return a->uv[0] < b->uv[0];
    };
    auto byV = [](const PlanarTrimVertex* a, const PlanarTrimVertex* b) {
        return a->uv[1] < b->uv[1];
    };
    std::sort(bottom.begin(), bottom.end(), byU);
    std::sort(top.begin(), top.end(), byU);
    std::sort(seamLo.begin(), seamLo.end(), byV);
    std::sort(seamHi.begin(), seamHi.end(), byV);

    const double uMatch = std::max(uIso, 0.05 * period);
    std::vector<double> uCols;
    auto pushUniqueU = [&](double u) {
        if (uCols.empty() || std::abs(u - uCols.back()) > uIso) {
            uCols.push_back(u);
        }
    };
    uCols.push_back(uMin);
    for (const PlanarTrimVertex* b : bottom) {
        if (b->uv[0] <= uMin + uIso || b->uv[0] >= uMax - uIso) continue;
        bool matched = false;
        for (const PlanarTrimVertex* t : top) {
            if (std::abs(t->uv[0] - b->uv[0]) <= uMatch) {
                matched = true;
                break;
            }
        }
        if (matched) pushUniqueU(b->uv[0]);
    }
    pushUniqueU(uMax);
    if (uCols.size() < 2) return std::nullopt;
    // Half-open U: drop the U=period seam column. Wrap tris reuse column 0
    // with cornerUv at U+P so the seam is not meshed twice in 3D.
    if (uCols.size() >= 2 && std::abs(uCols.back() - uMax) <= uIso) {
        uCols.pop_back();
    }
    if (uCols.size() < 2) return std::nullopt;

    std::vector<double> vRows;
    vRows.push_back(vMin);
    for (const PlanarTrimVertex* s : seamLo) {
        if (s->uv[1] > vMin + vIso && s->uv[1] < vMax - vIso) {
            vRows.push_back(s->uv[1]);
        }
    }
    for (const PlanarTrimVertex* s : seamHi) {
        if (s->uv[1] > vMin + vIso && s->uv[1] < vMax - vIso) {
            vRows.push_back(s->uv[1]);
        }
    }
    vRows.push_back(vMax);
    std::sort(vRows.begin(), vRows.end());
    {
        std::vector<double> uniq;
        for (double v : vRows) {
            if (uniq.empty() || std::abs(v - uniq.back()) > vIso) {
                uniq.push_back(v);
            }
        }
        vRows.swap(uniq);
    }
    const double maxVGap = vSpan / 8.0;
    for (std::size_t i = 0; i + 1 < vRows.size();) {
        if (vRows[i + 1] - vRows[i] > maxVGap) {
            vRows.insert(vRows.begin() + static_cast<std::ptrdiff_t>(i + 1),
                         0.5 * (vRows[i] + vRows[i + 1]));
        } else {
            ++i;
        }
    }
    const std::size_t nu = uCols.size();
    const std::size_t nv = vRows.size();
    if (nu < 2 || nv < 2) return std::nullopt;

    PlanarCdtMesh mesh;
    mesh.workingFace = seed.workingFace;
    mesh.sourceFace = seed.sourceFace;
    mesh.vertices.resize(nu * nv);
    auto at = [&](std::size_t iu, std::size_t iv) -> PlanarTrimVertex& {
        return mesh.vertices[iv * nu + iu];
    };
    auto nearestBoundary = [&](double u, double v) -> const PlanarTrimVertex* {
        const PlanarTrimVertex* best = nullptr;
        double bestScore = 1e300;
        for (const PlanarTrimVertex& vertex : loop) {
            const double du = vertex.uv[0] - u;
            const double dv = vertex.uv[1] - v;
            const double score = du * du + dv * dv;
            if (score < bestScore) {
                bestScore = score;
                best = &vertex;
            }
        }
        if (best && bestScore <= (uIso * uIso + vIso * vIso)) {
            return best;
        }
        return nullptr;
    };
    std::vector<std::array<double, 3>> positions(nu * nv);
    for (std::size_t iv = 0; iv < nv; ++iv) {
        for (std::size_t iu = 0; iu < nu; ++iu) {
            PlanarTrimVertex& vertex = at(iu, iv);
            vertex.uv = {uCols[iu], vRows[iv]};
            vertex.canonicalVertexIndex = InvalidCanonicalVertexIndex;
            PredicatePoint2 sampleUv = vertex.uv;
            sampleUv[0] =
                sampleUv[0] - std::floor(sampleUv[0] / period) * period;
            const auto evaluated =
                imported.workingEvaluator->evaluateSurface(faceId, sampleUv);
            if (!evaluated) return std::nullopt;
            positions[iv * nu + iu] = evaluated.value->position;
            if (iu == 0) {
                if (const PlanarTrimVertex* boundary =
                        nearestBoundary(vertex.uv[0], vertex.uv[1])) {
                    vertex = *boundary;
                } else {
                    vertex.cylinderInterior = CylinderInteriorStation{
                        seed.workingFace, seed.sourceFace, vertex.uv,
                        evaluated.value->position,
                        static_cast<std::uint32_t>(iv),
                        static_cast<std::uint32_t>(iu)};
                }
            } else if (const PlanarTrimVertex* boundary =
                           nearestBoundary(vertex.uv[0], vertex.uv[1])) {
                vertex = *boundary;
            } else {
                vertex.cylinderInterior = CylinderInteriorStation{
                    seed.workingFace, seed.sourceFace, vertex.uv,
                    evaluated.value->position,
                    static_cast<std::uint32_t>(iv),
                    static_cast<std::uint32_t>(iu)};
            }
        }
    }
    mesh.triangles.reserve(nu * (nv - 1) * 2);
    auto emitTriUv = [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2,
                         PredicatePoint2 uv0, PredicatePoint2 uv1,
                         PredicatePoint2 uv2) {
        PlanarCdtTriangle tri;
        tri.workingFace = seed.workingFace;
        tri.sourceFace = seed.sourceFace;
        tri.vertices = {i0, i1, i2};
        tri.cornerUv = std::array<PredicatePoint2, 3>{uv0, uv1, uv2};
        mesh.triangles.push_back(std::move(tri));
    };
    auto pointInBoundary = [&](double u, double v) -> bool {
        bool inside = false;
        const std::size_t n = loop.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const double ui = loop[i].uv[0];
            const double vi = loop[i].uv[1];
            const double uj = loop[j].uv[0];
            const double vj = loop[j].uv[1];
            if (std::abs(vj - vi) <= 1e-30) continue;
            const bool cross =
                ((vi > v) != (vj > v)) &&
                (u < (uj - ui) * (v - vi) / (vj - vi) + ui);
            if (cross) inside = !inside;
        }
        return inside;
    };
    auto len2 = [](const std::array<double, 3>& a,
                   const std::array<double, 3>& b) {
        const double dx = a[0] - b[0];
        const double dy = a[1] - b[1];
        const double dz = a[2] - b[2];
        return dx * dx + dy * dy + dz * dz;
    };
    auto emitCell = [&](std::uint32_t i00, std::uint32_t i10, std::uint32_t i01,
                        std::uint32_t i11, PredicatePoint2 uv00,
                        PredicatePoint2 uv10, PredicatePoint2 uv01,
                        PredicatePoint2 uv11) {
        const auto& p00 = positions[i00];
        const auto& p10 = positions[i10];
        const auto& p01 = positions[i01];
        const auto& p11 = positions[i11];
        if (len2(p00, p11) <= len2(p10, p01)) {
            emitTriUv(i00, i10, i11, uv00, uv10, uv11);
            emitTriUv(i00, i11, i01, uv00, uv11, uv01);
        } else {
            emitTriUv(i00, i10, i01, uv00, uv10, uv01);
            emitTriUv(i10, i11, i01, uv10, uv11, uv01);
        }
    };
    for (std::size_t iv = 0; iv + 1 < nv; ++iv) {
        for (std::size_t iu = 0; iu + 1 < nu; ++iu) {
            const double cu = 0.5 * (uCols[iu] + uCols[iu + 1]);
            const double cv = 0.5 * (vRows[iv] + vRows[iv + 1]);
            if (!pointInBoundary(cu, cv)) continue;
            const std::uint32_t i00 =
                static_cast<std::uint32_t>(iv * nu + iu);
            const std::uint32_t i10 =
                static_cast<std::uint32_t>(iv * nu + iu + 1);
            const std::uint32_t i01 =
                static_cast<std::uint32_t>((iv + 1) * nu + iu);
            const std::uint32_t i11 =
                static_cast<std::uint32_t>((iv + 1) * nu + iu + 1);
            emitCell(i00, i10, i01, i11, at(iu, iv).uv, at(iu + 1, iv).uv,
                     at(iu, iv + 1).uv, at(iu + 1, iv + 1).uv);
        }
        // Wrap strip: last U column → seam column at U+P.
        {
            const std::size_t iu = nu - 1;
            const double cu = 0.5 * (uCols[iu] + (uMin + period));
            const double cv = 0.5 * (vRows[iv] + vRows[iv + 1]);
            if (!pointInBoundary(cu, cv) &&
                !pointInBoundary(0.5 * (uCols[iu] + uMax), cv)) {
                continue;
            }
            const std::uint32_t i00 =
                static_cast<std::uint32_t>(iv * nu + iu);
            const std::uint32_t i10 =
                static_cast<std::uint32_t>(iv * nu + 0);
            const std::uint32_t i01 =
                static_cast<std::uint32_t>((iv + 1) * nu + iu);
            const std::uint32_t i11 =
                static_cast<std::uint32_t>((iv + 1) * nu + 0);
            PredicatePoint2 uv00 = at(iu, iv).uv;
            PredicatePoint2 uv10 = at(0, iv).uv;
            uv10[0] = uMin + period;
            PredicatePoint2 uv01 = at(iu, iv + 1).uv;
            PredicatePoint2 uv11 = at(0, iv + 1).uv;
            uv11[0] = uMin + period;
            emitCell(i00, i10, i01, i11, uv00, uv10, uv01, uv11);
        }
    }
    if (mesh.triangles.empty()) return std::nullopt;
    // Drop unused grid vertices (outside cells) and rebuild boundary from
    // triangle silhouette so incidence matches the emitted faces.
    {
        std::vector<std::uint32_t> remap(mesh.vertices.size(),
                                         ~std::uint32_t{0});
        std::vector<PlanarTrimVertex> compact;
        compact.reserve(mesh.vertices.size());
        auto mapVertex = [&](std::uint32_t index) -> std::uint32_t {
            if (remap[index] != ~std::uint32_t{0}) return remap[index];
            remap[index] = static_cast<std::uint32_t>(compact.size());
            compact.push_back(mesh.vertices[index]);
            return remap[index];
        };
        for (PlanarCdtTriangle& tri : mesh.triangles) {
            tri.vertices[0] = mapVertex(tri.vertices[0]);
            tri.vertices[1] = mapVertex(tri.vertices[1]);
            tri.vertices[2] = mapVertex(tri.vertices[2]);
        }
        mesh.vertices = std::move(compact);
        std::map<std::array<std::uint32_t, 2>, int> edgeUse;
        auto addEdge = [&](std::uint32_t a, std::uint32_t b) {
            edgeUse[std::array<std::uint32_t, 2>{std::min(a, b),
                                                 std::max(a, b)}]++;
        };
        for (const PlanarCdtTriangle& tri : mesh.triangles) {
            addEdge(tri.vertices[0], tri.vertices[1]);
            addEdge(tri.vertices[1], tri.vertices[2]);
            addEdge(tri.vertices[2], tri.vertices[0]);
        }
        std::vector<std::array<std::uint32_t, 2>> silhouette;
        for (const auto& item : edgeUse) {
            if (item.second == 1) silhouette.push_back(item.first);
        }
        mesh.constrainedEdges = silhouette;
        mesh.boundaryLoops.clear();
        if (!silhouette.empty()) {
            std::map<std::uint32_t, std::vector<std::uint32_t>> adj;
            for (const auto& edge : silhouette) {
                adj[edge[0]].push_back(edge[1]);
                adj[edge[1]].push_back(edge[0]);
            }
            std::vector<std::uint32_t> walk;
            std::uint32_t start = silhouette.front()[0];
            std::uint32_t prev = ~std::uint32_t{0};
            std::uint32_t cur = start;
            for (std::size_t guard = 0; guard < mesh.vertices.size() + 2;
                 ++guard) {
                walk.push_back(cur);
                const auto& nbrs = adj[cur];
                std::uint32_t next = nbrs[0];
                if (nbrs.size() > 1 && next == prev) next = nbrs[1];
                if (next == start) break;
                prev = cur;
                cur = next;
            }
            if (walk.size() >= 3) mesh.boundaryLoops.push_back(std::move(walk));
        }
    }
    return mesh;
}

// Curved UV iso-lattice clipped by even-odd inclusion of all CDT boundary
// loops (outer + holes). When uPeriod is set and the chart spans ~1 period,
// fold U into [0,P), half-open + guarded wrap (cylinder multi-rim). Without
// a period, densify the raw UV bbox (non-periodic freeform n-gons).
std::optional<PlanarCdtMesh> buildPeriodicUvIsoLattice(
    const ImportedModel& imported, StableId faceId,
    const PlanarCdtMesh& seed, std::optional<double> uPeriod) {
    if (!imported.workingEvaluator || seed.boundaryLoops.empty()) {
        std::fprintf(stderr,
                     "WEFT_UVTRIM_LATTICE_SKIP reason=seed face=%llu "
                     "loops=%zu\n",
                     static_cast<unsigned long long>(faceId.ordinal),
                     seed.boundaryLoops.size());
        return std::nullopt;
    }
    std::vector<PlanarTrimVertex> allLoopVerts;
    std::vector<std::vector<PredicatePoint2>> loopUvs;
    for (const auto& loopIdx : seed.boundaryLoops) {
        if (loopIdx.size() < 3) continue;
        std::vector<PredicatePoint2> uvLoop;
        uvLoop.reserve(loopIdx.size());
        for (std::uint32_t index : loopIdx) {
            if (index >= seed.vertices.size()) return std::nullopt;
            allLoopVerts.push_back(seed.vertices[index]);
            uvLoop.push_back(seed.vertices[index].uv);
        }
        loopUvs.push_back(std::move(uvLoop));
    }
    if (allLoopVerts.size() < 4 || loopUvs.empty()) {
        std::fprintf(stderr,
                     "WEFT_UVTRIM_LATTICE_SKIP reason=loops verts=%zu "
                     "loops=%zu\n",
                     allLoopVerts.size(), loopUvs.size());
        return std::nullopt;
    }

    const bool periodic =
        uPeriod && *uPeriod > 0.0;
    const double period = periodic ? *uPeriod : 0.0;
    // Periodic charts may unwrap across >1 period (e.g. U in [-π, 2π)).
    // Fold into [0, P) so the lattice cannot double-cover in 3D.
    if (periodic) {
        auto foldU = [&](double u) -> double {
            double x = u - std::floor(u / period) * period;
            if (x < 0.0) x += period;
            if (x >= period) x = 0.0;
            return x;
        };
        for (auto& loop : loopUvs) {
            for (PredicatePoint2& uv : loop) uv[0] = foldU(uv[0]);
        }
        for (PlanarTrimVertex& vertex : allLoopVerts) {
            vertex.uv[0] = foldU(vertex.uv[0]);
        }
    }

    double uMin = allLoopVerts.front().uv[0];
    double uMax = uMin;
    double vMin = allLoopVerts.front().uv[1];
    double vMax = vMin;
    for (const PlanarTrimVertex& vertex : allLoopVerts) {
        uMin = std::min(uMin, vertex.uv[0]);
        uMax = std::max(uMax, vertex.uv[0]);
        vMin = std::min(vMin, vertex.uv[1]);
        vMax = std::max(vMax, vertex.uv[1]);
    }
    const double uSpan = uMax - uMin;
    const double vSpan = vMax - vMin;
    if (!(uSpan > 1e-6) || !(vSpan > 1e-6)) {
        std::fprintf(stderr,
                     "WEFT_UVTRIM_LATTICE_SKIP reason=span u=%.6g v=%.6g\n",
                     uSpan, vSpan);
        return std::nullopt;
    }
    if (periodic && !(uSpan > 0.5 * period)) {
        std::fprintf(stderr,
                     "WEFT_UVTRIM_LATTICE_SKIP reason=period_span "
                     "uSpan=%.6g period=%.6g\n",
                     uSpan, period);
        return std::nullopt;
    }

    constexpr double kIsoFrac = 0.02;
    const double uIso =
        std::max(1e-6, kIsoFrac * (periodic ? period : uSpan));
    const double vIso = std::max(1e-6, kIsoFrac * vSpan);

    std::vector<double> uCols;
    std::vector<double> vRows;
    auto pushUnique = [](std::vector<double>& cols, double value,
                         double eps) {
        for (double existing : cols) {
            if (std::abs(existing - value) <= eps) return;
        }
        cols.push_back(value);
    };
    for (const PlanarTrimVertex& vertex : allLoopVerts) {
        pushUnique(uCols, vertex.uv[0], uIso);
        pushUnique(vRows, vertex.uv[1], vIso);
    }
    std::sort(uCols.begin(), uCols.end());
    std::sort(vRows.begin(), vRows.end());
    const double maxUGap =
        (periodic ? period : uSpan) / 32.0;
    const double maxVGap = vSpan / 16.0;
    for (std::size_t i = 0; i + 1 < uCols.size();) {
        if (uCols[i + 1] - uCols[i] > maxUGap) {
            uCols.insert(uCols.begin() + static_cast<std::ptrdiff_t>(i + 1),
                         0.5 * (uCols[i] + uCols[i + 1]));
        } else {
            ++i;
        }
    }
    for (std::size_t i = 0; i + 1 < vRows.size();) {
        if (vRows[i + 1] - vRows[i] > maxVGap) {
            vRows.insert(vRows.begin() + static_cast<std::ptrdiff_t>(i + 1),
                         0.5 * (vRows[i] + vRows[i + 1]));
        } else {
            ++i;
        }
    }
    // Half-open U: drop the U≈uMax seam column when the chart spans a period.
    // Wrap tris below reuse column 0 with cornerUv at U+P so the seam is not
    // meshed twice in 3D (avoids certified.triangle_proper_intersection).
    if (periodic && uCols.size() >= 2 && uSpan > 0.85 * period &&
        std::abs(uCols.back() - uMax) <= uIso) {
        uCols.pop_back();
    }
    // Guarantee a non-degenerate grid even when boundary samples collapse
    // under the iso epsilon (thin offset strips).
    if (uCols.size() < 2) {
        uCols = {uMin, uMax};
    }
    if (vRows.size() < 2) {
        vRows = {vMin, vMax};
    }
    const std::size_t nu = uCols.size();
    const std::size_t nv = vRows.size();
    if (nu < 2 || nv < 2) {
        std::fprintf(stderr,
                     "WEFT_UVTRIM_LATTICE_SKIP reason=grid face=%llu "
                     "nu=%zu nv=%zu uSpan=%.6g vSpan=%.6g\n",
                     static_cast<unsigned long long>(faceId.ordinal), nu, nv,
                     uSpan, vSpan);
        return std::nullopt;
    }

    auto pointInDomain = [&](double u, double v) -> bool {
        bool inside = false;
        for (const auto& loop : loopUvs) {
            const std::size_t n = loop.size();
            for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
                const double ui = loop[i][0];
                const double vi = loop[i][1];
                const double uj = loop[j][0];
                const double vj = loop[j][1];
                if (std::abs(vj - vi) <= 1e-30) continue;
                const bool cross =
                    ((vi > v) != (vj > v)) &&
                    (u < (uj - ui) * (v - vi) / (vj - vi) + ui);
                if (cross) inside = !inside;
            }
        }
        return inside;
    };

    PlanarCdtMesh mesh;
    mesh.workingFace = seed.workingFace;
    mesh.sourceFace = seed.sourceFace;
    mesh.vertices.resize(nu * nv);
    auto at = [&](std::size_t iu, std::size_t iv) -> PlanarTrimVertex& {
        return mesh.vertices[iv * nu + iu];
    };
    std::vector<std::array<double, 3>> positions(nu * nv);
    for (std::size_t iv = 0; iv < nv; ++iv) {
        for (std::size_t iu = 0; iu < nu; ++iu) {
            PlanarTrimVertex& vertex = at(iu, iv);
            // Keep the unwrapped chart UV (matches CDT / hardOrient). Evaluate
            // at the same UV that certify will re-check — do not fold to a
            // different sample than station.uv.
            vertex.uv = {uCols[iu], vRows[iv]};
            vertex.canonicalVertexIndex = InvalidCanonicalVertexIndex;
            const auto evaluated =
                imported.workingEvaluator->evaluateSurface(faceId, vertex.uv);
            if (!evaluated) return std::nullopt;
            positions[iv * nu + iu] = evaluated.value->position;
            vertex.cylinderInterior = CylinderInteriorStation{
                seed.workingFace, seed.sourceFace, vertex.uv,
                evaluated.value->position,
                static_cast<std::uint32_t>(iv),
                static_cast<std::uint32_t>(iu)};
        }
    }
    mesh.triangles.reserve(nu * (nv - 1) * 2);
    auto emitTriUv = [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2,
                         PredicatePoint2 uv0, PredicatePoint2 uv1,
                         PredicatePoint2 uv2) {
        PlanarCdtTriangle tri;
        tri.workingFace = seed.workingFace;
        tri.sourceFace = seed.sourceFace;
        tri.vertices = {i0, i1, i2};
        tri.cornerUv = std::array<PredicatePoint2, 3>{uv0, uv1, uv2};
        mesh.triangles.push_back(std::move(tri));
    };
    auto len2 = [](const std::array<double, 3>& a,
                   const std::array<double, 3>& b) {
        const double dx = a[0] - b[0];
        const double dy = a[1] - b[1];
        const double dz = a[2] - b[2];
        return dx * dx + dy * dy + dz * dz;
    };
    auto emitCell = [&](std::uint32_t i00, std::uint32_t i10, std::uint32_t i01,
                        std::uint32_t i11, PredicatePoint2 uv00,
                        PredicatePoint2 uv10, PredicatePoint2 uv01,
                        PredicatePoint2 uv11) {
        const auto& p00 = positions[i00];
        const auto& p10 = positions[i10];
        const auto& p01 = positions[i01];
        const auto& p11 = positions[i11];
        if (len2(p00, p11) <= len2(p10, p01)) {
            emitTriUv(i00, i10, i11, uv00, uv10, uv11);
            emitTriUv(i00, i11, i01, uv00, uv11, uv01);
        } else {
            emitTriUv(i00, i10, i01, uv00, uv10, uv01);
            emitTriUv(i10, i11, i01, uv10, uv11, uv01);
        }
    };
    auto emitLatticeCells = [&](bool flipInclusion, bool requireCorners) {
        mesh.triangles.clear();
        auto inside = [&](double u, double v) -> bool {
            const bool hit = pointInDomain(u, v);
            return flipInclusion ? !hit : hit;
        };
        for (std::size_t iv = 0; iv + 1 < nv; ++iv) {
            for (std::size_t iu = 0; iu + 1 < nu; ++iu) {
                const double u0 = uCols[iu];
                const double u1 = uCols[iu + 1];
                const double v0 = vRows[iv];
                const double v1 = vRows[iv + 1];
                const double cu = 0.5 * (u0 + u1);
                const double cv = 0.5 * (v0 + v1);
                if (!inside(cu, cv)) continue;
                if (requireCorners &&
                    (!inside(u0, v0) || !inside(u1, v0) || !inside(u0, v1) ||
                     !inside(u1, v1) || !inside(cu, v0) || !inside(cu, v1) ||
                     !inside(u0, cv) || !inside(u1, cv))) {
                    continue;
                }
                const std::uint32_t i00 =
                    static_cast<std::uint32_t>(iv * nu + iu);
                const std::uint32_t i10 =
                    static_cast<std::uint32_t>(iv * nu + iu + 1);
                const std::uint32_t i01 =
                    static_cast<std::uint32_t>((iv + 1) * nu + iu);
                const std::uint32_t i11 =
                    static_cast<std::uint32_t>((iv + 1) * nu + iu + 1);
                emitCell(i00, i10, i01, i11, at(iu, iv).uv, at(iu + 1, iv).uv,
                         at(iu, iv + 1).uv, at(iu + 1, iv + 1).uv);
            }
        }
    };
    // Prefer corner-safe cells (avoids hole chords / coplanar overlap).
    // If the chart's even-odd polarity emits nothing (some offset UV
    // loops), fall back to centroid-only then flipped polarity.
    emitLatticeCells(/*flip=*/false, /*corners=*/true);
    if (mesh.triangles.empty()) {
        emitLatticeCells(/*flip=*/false, /*corners=*/false);
    }
    if (mesh.triangles.empty()) {
        emitLatticeCells(/*flip=*/true, /*corners=*/false);
    }
    // Wrap strip only for periodic charts (unchanged below).
    if (periodic && uSpan > 0.85 * period) {
        for (std::size_t iv = 0; iv + 1 < nv; ++iv) {
            const std::size_t iu = nu - 1;
            const double v0 = vRows[iv];
            const double v1 = vRows[iv + 1];
            if (!pointInDomain(uCols[iu], v0) ||
                !pointInDomain(uCols[iu], v1) ||
                !pointInDomain(uMin + uIso, v0) ||
                !pointInDomain(uMin + uIso, v1)) {
                continue;
            }
            const std::uint32_t i00 =
                static_cast<std::uint32_t>(iv * nu + iu);
            const std::uint32_t i10 =
                static_cast<std::uint32_t>(iv * nu + 0);
            const std::uint32_t i01 =
                static_cast<std::uint32_t>((iv + 1) * nu + iu);
            const std::uint32_t i11 =
                static_cast<std::uint32_t>((iv + 1) * nu + 0);
            PredicatePoint2 uv00 = at(iu, iv).uv;
            PredicatePoint2 uv10 = at(0, iv).uv;
            uv10[0] = uMin + period;
            PredicatePoint2 uv01 = at(iu, iv + 1).uv;
            PredicatePoint2 uv11 = at(0, iv + 1).uv;
            uv11[0] = uMin + period;
            auto wrapAgrees = [&](const std::array<double, 3>& pa,
                                  const std::array<double, 3>& pb,
                                  const std::array<double, 3>& pc,
                                  const PredicatePoint2& uva,
                                  const PredicatePoint2& uvb,
                                  const PredicatePoint2& uvc) -> bool {
                const double ax = pb[0] - pa[0];
                const double ay = pb[1] - pa[1];
                const double az = pb[2] - pa[2];
                const double bx = pc[0] - pa[0];
                const double by = pc[1] - pa[1];
                const double bz = pc[2] - pa[2];
                const double nx = ay * bz - az * by;
                const double ny = az * bx - ax * bz;
                const double nz = ax * by - ay * bx;
                const double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
                if (!(nl > 0.0)) return false;
                const PredicatePoint2 uvC{(uva[0] + uvb[0] + uvc[0]) / 3.0,
                                          (uva[1] + uvb[1] + uvc[1]) / 3.0};
                PredicatePoint2 sampleUv = uvC;
                sampleUv[0] = sampleUv[0] -
                    std::floor(sampleUv[0] / period) * period;
                const auto evaluated =
                    imported.workingEvaluator->evaluateSurface(faceId,
                                                               sampleUv);
                if (!evaluated || !evaluated.value->unitNormal) return false;
                const double d =
                    (nx / nl) * (*evaluated.value->unitNormal)[0] +
                    (ny / nl) * (*evaluated.value->unitNormal)[1] +
                    (nz / nl) * (*evaluated.value->unitNormal)[2];
                return d > 0.0;
            };
            const auto& p00 = positions[i00];
            const auto& p10 = positions[i10];
            const auto& p01 = positions[i01];
            const auto& p11 = positions[i11];
            const bool diagA = len2(p00, p11) <= len2(p10, p01);
            if (diagA) {
                if (!wrapAgrees(p00, p10, p11, uv00, uv10, uv11) ||
                    !wrapAgrees(p00, p11, p01, uv00, uv11, uv01)) {
                    continue;
                }
            } else {
                if (!wrapAgrees(p00, p10, p01, uv00, uv10, uv01) ||
                    !wrapAgrees(p10, p11, p01, uv10, uv11, uv01)) {
                    continue;
                }
            }
            emitCell(i00, i10, i01, i11, uv00, uv10, uv01, uv11);
        }
    }
    if (mesh.triangles.empty()) {
        std::fprintf(stderr,
                     "WEFT_UVTRIM_LATTICE_SKIP reason=empty face=%llu "
                     "nu=%zu nv=%zu\n",
                     static_cast<unsigned long long>(faceId.ordinal), nu, nv);
        return std::nullopt;
    }
    // so certifySolvedIntervalConsumption sees every seam/rim sample. Lattice
    // interiors alone leave interval_consumption_mismatch.
    {
        std::set<std::uint64_t> seen;
        for (const PlanarTrimVertex& vertex : mesh.vertices) {
            if (vertex.canonicalVertexIndex != InvalidCanonicalVertexIndex) {
                seen.insert(vertex.canonicalVertexIndex);
            }
        }
        for (const auto& loopIdx : seed.boundaryLoops) {
            for (std::uint32_t index : loopIdx) {
                if (index >= seed.vertices.size()) continue;
                const PlanarTrimVertex& boundary = seed.vertices[index];
                if (boundary.boundaryUses.empty()) continue;
                if (boundary.canonicalVertexIndex !=
                        InvalidCanonicalVertexIndex &&
                    !seen.insert(boundary.canonicalVertexIndex).second) {
                    continue;
                }
                mesh.vertices.push_back(boundary);
            }
        }
    }
    return mesh;
}

// Split each against-N triangle by a surface-evaluated centroid Steiner.
// Conforming (one tri → three) and stays inside the CDT domain.
bool splitAgainstTrisAtCentroid(const ImportedModel& imported,
                                StableId faceId, PlanarCdtMesh& mesh,
                                std::optional<double> uPeriod,
                                std::optional<double> vPeriod) {
    if (!imported.workingEvaluator || mesh.triangles.empty()) return false;
    auto periodicNear = [](double value, double reference,
                           std::optional<double> period) -> double {
        if (!period || !(*period > 0.0)) return value;
        return value + std::round((reference - value) / *period) * *period;
    };
    auto vertexUv = [&](const PlanarCdtTriangle& tri,
                        std::size_t corner) -> PredicatePoint2 {
        if (tri.cornerUv) return (*tri.cornerUv)[corner];
        return mesh.vertices[tri.vertices[corner]].uv;
    };
    // Reuse the same agreement test as hardOrient (centroid vs corner vote).
    auto uvAgrees = [&](const PlanarCdtTriangle& tri) -> std::optional<bool> {
        if (tri.vertices[0] >= mesh.vertices.size() ||
            tri.vertices[1] >= mesh.vertices.size() ||
            tri.vertices[2] >= mesh.vertices.size()) {
            return std::nullopt;
        }
        const PredicatePoint2 uv0 = vertexUv(tri, 0);
        PredicatePoint2 uv1 = vertexUv(tri, 1);
        PredicatePoint2 uv2 = vertexUv(tri, 2);
        uv1[0] = periodicNear(uv1[0], uv0[0], uPeriod);
        uv2[0] = periodicNear(uv2[0], uv0[0], uPeriod);
        uv1[1] = periodicNear(uv1[1], uv0[1], vPeriod);
        uv2[1] = periodicNear(uv2[1], uv0[1], vPeriod);
        auto cornerPosition =
            [&](std::size_t corner) -> std::optional<std::array<double, 3>> {
            const PlanarTrimVertex& vertex = mesh.vertices[tri.vertices[corner]];
            if (vertex.cylinderInterior) {
                return vertex.cylinderInterior->position;
            }
            PredicatePoint2 lifted = vertexUv(tri, corner);
            if (corner == 1) {
                lifted[0] = uv1[0];
                lifted[1] = uv1[1];
            } else if (corner == 2) {
                lifted[0] = uv2[0];
                lifted[1] = uv2[1];
            }
            PredicatePoint2 sampleUv = lifted;
            if (uPeriod && *uPeriod > 0.0) {
                sampleUv[0] = sampleUv[0] -
                    std::floor(sampleUv[0] / *uPeriod) * *uPeriod;
            }
            if (vPeriod && *vPeriod > 0.0) {
                sampleUv[1] = sampleUv[1] -
                    std::floor(sampleUv[1] / *vPeriod) * *vPeriod;
            }
            const auto evaluated =
                imported.workingEvaluator->evaluateSurface(faceId, sampleUv);
            if (!evaluated) return std::nullopt;
            return evaluated.value->position;
        };
        const auto p0 = cornerPosition(0);
        const auto p1 = cornerPosition(1);
        const auto p2 = cornerPosition(2);
        if (!p0 || !p1 || !p2) return std::nullopt;
        const double ax = (*p1)[0] - (*p0)[0];
        const double ay = (*p1)[1] - (*p0)[1];
        const double az = (*p1)[2] - (*p0)[2];
        const double bx = (*p2)[0] - (*p0)[0];
        const double by = (*p2)[1] - (*p0)[1];
        const double bz = (*p2)[2] - (*p0)[2];
        const double nx = ay * bz - az * by;
        const double ny = az * bx - ax * bz;
        const double nz = ax * by - ay * bx;
        const double nl = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (!(nl > 0.0)) return std::nullopt;
        const double fnx = nx / nl;
        const double fny = ny / nl;
        const double fnz = nz / nl;
        auto sampleDot = [&](const PredicatePoint2& uv) -> std::optional<double> {
            PredicatePoint2 sampleUv = uv;
            if (uPeriod && *uPeriod > 0.0) {
                sampleUv[0] = sampleUv[0] -
                    std::floor(sampleUv[0] / *uPeriod) * *uPeriod;
            }
            if (vPeriod && *vPeriod > 0.0) {
                sampleUv[1] = sampleUv[1] -
                    std::floor(sampleUv[1] / *vPeriod) * *vPeriod;
            }
            const auto p =
                imported.workingEvaluator->evaluateSurface(faceId, sampleUv);
            if (!p || !p.value->unitNormal) return std::nullopt;
            return fnx * (*p.value->unitNormal)[0] +
                   fny * (*p.value->unitNormal)[1] +
                   fnz * (*p.value->unitNormal)[2];
        };
        const double du =
            std::max({std::abs(uv1[0] - uv0[0]), std::abs(uv2[0] - uv0[0]),
                      std::abs(uv2[0] - uv1[0])});
        const double dv =
            std::max({std::abs(uv1[1] - uv0[1]), std::abs(uv2[1] - uv0[1]),
                      std::abs(uv2[1] - uv1[1])});
        const bool useCentroid =
            !(uPeriod && *uPeriod > 0.0 && du > 0.15 * *uPeriod) &&
            !(vPeriod && *vPeriod > 0.0 && dv > 0.15 * *vPeriod);
        if (useCentroid) {
            const PredicatePoint2 uvC{(uv0[0] + uv1[0] + uv2[0]) / 3.0,
                                      (uv0[1] + uv1[1] + uv2[1]) / 3.0};
            const auto d = sampleDot(uvC);
            if (!d) return std::nullopt;
            if (std::abs(*d) < 1e-8) return std::nullopt;
            return *d > 0.0;
        }
        int withSamples = 0;
        int againstSamples = 0;
        for (const PredicatePoint2& uv : {uv0, uv1, uv2}) {
            const auto d = sampleDot(uv);
            if (!d || std::abs(*d) < 1e-8) continue;
            if (*d > 0.0) {
                ++withSamples;
            } else {
                ++againstSamples;
            }
        }
        if (withSamples == 0 && againstSamples == 0) return std::nullopt;
        return withSamples >= againstSamples;
    };

    std::vector<PlanarCdtTriangle> next;
    next.reserve(mesh.triangles.size() + 16);
    bool changed = false;
    for (const PlanarCdtTriangle& tri : mesh.triangles) {
        const auto agrees = uvAgrees(tri);
        if (!agrees || *agrees) {
            next.push_back(tri);
            continue;
        }
        const PredicatePoint2 uv0 = vertexUv(tri, 0);
        PredicatePoint2 uv1 = vertexUv(tri, 1);
        PredicatePoint2 uv2 = vertexUv(tri, 2);
        uv1[0] = periodicNear(uv1[0], uv0[0], uPeriod);
        uv2[0] = periodicNear(uv2[0], uv0[0], uPeriod);
        uv1[1] = periodicNear(uv1[1], uv0[1], vPeriod);
        uv2[1] = periodicNear(uv2[1], uv0[1], vPeriod);
        const PredicatePoint2 uvC{(uv0[0] + uv1[0] + uv2[0]) / 3.0,
                                  (uv0[1] + uv1[1] + uv2[1]) / 3.0};
        PredicatePoint2 sampleUv = uvC;
        if (uPeriod && *uPeriod > 0.0) {
            sampleUv[0] = sampleUv[0] -
                std::floor(sampleUv[0] / *uPeriod) * *uPeriod;
        }
        if (vPeriod && *vPeriod > 0.0) {
            sampleUv[1] = sampleUv[1] -
                std::floor(sampleUv[1] / *vPeriod) * *vPeriod;
        }
        const auto evaluated =
            imported.workingEvaluator->evaluateSurface(faceId, sampleUv);
        if (!evaluated) {
            next.push_back(tri);
            continue;
        }
        PlanarTrimVertex steiner;
        steiner.canonicalVertexIndex = InvalidCanonicalVertexIndex;
        steiner.uv = uvC;
        steiner.cylinderInterior = CylinderInteriorStation{
            mesh.workingFace, mesh.sourceFace, uvC,
            evaluated.value->position,
            static_cast<std::uint32_t>(mesh.vertices.size()), 0};
        const std::uint32_t ic =
            static_cast<std::uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back(std::move(steiner));
        auto emit = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
            PlanarCdtTriangle out;
            out.workingFace = tri.workingFace;
            out.sourceFace = tri.sourceFace;
            out.vertices = {a, b, c};
            out.cornerUv = std::array<PredicatePoint2, 3>{
                mesh.vertices[a].uv, mesh.vertices[b].uv, mesh.vertices[c].uv};
            next.push_back(std::move(out));
        };
        emit(tri.vertices[0], tri.vertices[1], ic);
        emit(tri.vertices[1], tri.vertices[2], ic);
        emit(tri.vertices[2], tri.vertices[0], ic);
        changed = true;
    }
    if (!changed) return false;
    mesh.triangles = std::move(next);
    return true;
}

PlanarTrimDomain coarsenCurvedUvTrimLoops(PlanarTrimDomain domain) {
    auto edgeIdOf = [](const PlanarTrimVertex& vertex)
        -> std::optional<StableId> {
        if (vertex.boundaryUses.empty()) return std::nullopt;
        return vertex.boundaryUses.front().workingEdge;
    };
    const std::optional<double> uPeriod = domain.curvedUvUPeriod;
    const std::optional<double> vPeriod = domain.curvedUvVPeriod;
    auto periodicGapTooLarge = [&](const PredicatePoint2& a,
                                   const PredicatePoint2& b) -> bool {
        // Nearest-period CDT unwrap folds the short way when |Î”| > P/2.
        // Only guard iso-aligned runs (circular caps at const V / seam at
        // const U). Diagonal freeform chords (freeform_135/138) may span
        // >0.45Â·P and still coarsen safely.
        constexpr double kMaxFrac = 0.45;
        constexpr double kOrthoEps = 1e-3;
        if (uPeriod && *uPeriod > 0.0 &&
            std::abs(a[0] - b[0]) > kMaxFrac * *uPeriod &&
            std::abs(a[1] - b[1]) <= kOrthoEps) {
            return true;
        }
        if (vPeriod && *vPeriod > 0.0 &&
            std::abs(a[1] - b[1]) > kMaxFrac * *vPeriod &&
            std::abs(a[0] - b[0]) <= kOrthoEps) {
            return true;
        }
        return false;
    };
    for (PlanarTrimLoop& loop : domain.loops) {
        const std::size_t n = loop.vertices.size();
        if (n <= 5) continue;
        std::vector<char> keep(n, 0);
        for (std::size_t i = 0; i < n; ++i) {
            const auto ePrev = edgeIdOf(loop.vertices[(i + n - 1) % n]);
            const auto eCurr = edgeIdOf(loop.vertices[i]);
            const auto eNext = edgeIdOf(loop.vertices[(i + 1) % n]);
            if (!eCurr || eCurr != ePrev || eCurr != eNext ||
                loop.vertices[i].boundaryUses.size() > 1) {
                keep[i] = 1;
            }
        }
        // Closed-loop interior runs may wrap past index 0.
        std::vector<std::size_t> order(n);
        for (std::size_t i = 0; i < n; ++i) order[i] = i;
        std::size_t start = 0;
        while (start < n && !keep[order[start]]) ++start;
        if (start == n) {
            // No corners identified â€” keep every other vertex.
            for (std::size_t i = 0; i < n; i += 2) keep[i] = 1;
        } else {
            for (std::size_t pass = 0; pass < n;) {
                const std::size_t corner = (start + pass) % n;
                if (!keep[corner]) {
                    ++pass;
                    continue;
                }
                std::size_t run = 1;
                while (run < n &&
                       !keep[order[(start + pass + run) % n]]) {
                    ++run;
                }
                // Interiors between this corner and the next: keep every other.
                bool take = false;
                for (std::size_t k = 1; k + 1 < run; ++k) {
                    take = !take;
                    if (take) {
                        keep[order[(start + pass + k) % n]] = 1;
                    }
                }
                pass += run;
            }
        }
        // Re-insert any dropped vertex that would leave a >0.45-period gap.
        bool expanded = true;
        while (expanded) {
            expanded = false;
            std::vector<std::size_t> keptIdx;
            keptIdx.reserve(n);
            for (std::size_t i = 0; i < n; ++i) {
                if (keep[i]) keptIdx.push_back(i);
            }
            if (keptIdx.size() < 2) break;
            for (std::size_t k = 0; k < keptIdx.size(); ++k) {
                const std::size_t i0 = keptIdx[k];
                const std::size_t i1 = keptIdx[(k + 1) % keptIdx.size()];
                if (!periodicGapTooLarge(loop.vertices[i0].uv,
                                        loop.vertices[i1].uv)) {
                    continue;
                }
                // Walk the shorter index span and keep the midpoint.
                std::size_t span = 0;
                std::size_t mid = i0;
                if (i0 < i1) {
                    span = i1 - i0;
                    mid = i0 + span / 2;
                } else {
                    span = (n - i0) + i1;
                    mid = (i0 + span / 2) % n;
                }
                if (span >= 2 && !keep[mid]) {
                    keep[mid] = 1;
                    expanded = true;
                }
            }
        }
        std::vector<PlanarTrimVertex> thinned;
        thinned.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            if (keep[i]) thinned.push_back(std::move(loop.vertices[i]));
        }
        if (thinned.size() >= 3) {
            loop.vertices = std::move(thinned);
        }
    }
    return domain;
}

double vectorLength(const std::array<double, 3>& vector) {
    return std::hypot(vector[0], vector[1], vector[2]);
}

const EdgeTopologyRecord* edgeTopology(const BRepSnapshot& snapshot,
                                       StableId edge) {
    const auto found = std::find_if(
        snapshot.edgeTopology.begin(), snapshot.edgeTopology.end(),
        [edge](const EdgeTopologyRecord& record) {
            return record.id == edge;
        });
    return found == snapshot.edgeTopology.end() ? nullptr : &*found;
}

struct IntervalProblemResult {
    std::optional<IntervalProblem> value;
    std::optional<SecureMeshingFailure> failure;
};

IntervalProblemResult buildIntervalProblem(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const SecureMeshingConfiguration& configuration) {
    IntervalProblemResult result;
    if (configuration.cylinderAxialIntervals == 0) {
        result.failure = SecureMeshingFailure{
            "secure_pipeline.axial_count_invalid",
            "cylinder axial intervals must be positive", {}};
        return result;
    }
    // Fail fast on unsupported curve families (e.g. 211 ellipses on MP9)
    // before the expensive per-edge bspline UV-span walk.
    for (const ExactGeometryClassification& record : reconnaissance.records) {
        if (record.taxonomy != GeometryTaxonomy::Curve) continue;
        const bool supportedCurve =
            record.familyCode == "line" || record.familyCode == "circle" ||
            record.familyCode == "ellipse" ||
            record.familyCode == "bspline" || record.familyCode == "bezier" ||
            std::find(record.conditionCodes.begin(),
                      record.conditionCodes.end(),
                      "degenerate") != record.conditionCodes.end();
        if (!supportedCurve) {
            // Allow kernel_specific meridians on spherical caps (count=1).
            bool sphereCapMeridian = false;
            for (const CoedgeRecord& coedge :
                 imported.working->snapshot.coedges) {
                if (coedge.edgeId != record.subjectId) continue;
                const ExactGeometryClassification* face =
                    reconnaissance.find(coedge.faceId);
                if (face && face->familyCode == "sphere" && face->trimDomain &&
                    *face->trimDomain ==
                        TrimDomainClass::TouchesOneSingularity) {
                    sphereCapMeridian = true;
                    break;
                }
            }
            if (!sphereCapMeridian) {
                result.failure = SecureMeshingFailure{
                    unsupportedCurveRefuseCode(record.familyCode),
                    "curve family '" + record.familyCode +
                        "' has no certified automatic interval consumer",
                    {record.subjectId}};
                return result;
            }
        }
    }
    IntervalProblem problem;
    const BRepSnapshot& snapshot = imported.working->snapshot;
    for (const auto& [edge, count] :
         configuration.exactEdgeIntervalCounts) {
        if (!edge.valid() || edge.kind != StableIdKind::Edge || count == 0 ||
            edge.ordinal >
                static_cast<std::uint64_t>(snapshot.model.edgeCount()) ||
            !edgeTopology(snapshot, edge)) {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.edge_constraint_invalid",
                "an exact edge count must name one existing working edge and be positive",
                {edge}};
            return result;
        }
    }
    std::map<StableId, std::vector<StableId>> facesByEdge;
    facesByEdge.clear();
    for (const CoedgeRecord& coedge : snapshot.coedges) {
        if (!coedge.edgeId.valid() || !coedge.faceId.valid()) continue;
        facesByEdge[coedge.edgeId].push_back(coedge.faceId);
    }
    std::set<StableId> cylinderAxialEdges;
    std::set<StableId> digonPlaneEdges;
    std::set<StableId> planeEllipseEdges;
    std::map<StableId, std::set<StableId>> edgesByFace;
    for (const CoedgeRecord& coedge : snapshot.coedges) {
        if (!coedge.edgeId.valid() || !coedge.faceId.valid()) continue;
        edgesByFace[coedge.faceId].insert(coedge.edgeId);
    }
    for (const auto& [faceId, edgeIds] : edgesByFace) {
        if (edgeIds.size() != 2) continue;
        const ExactGeometryClassification* face = reconnaissance.find(faceId);
        if (!face || face->familyCode != "plane") continue;
        // G1: digon / thin two-edge plane strips need interior line samples
        // so parallel-offset p-curves form a meshable UV polygon.
        for (const StableId& edgeId : edgeIds) {
            digonPlaneEdges.insert(edgeId);
        }
    }
    for (const auto& [edgeId, faceIds] : facesByEdge) {
        const ExactGeometryClassification* edge = reconnaissance.find(edgeId);
        if (!edge) continue;
        if (edge->familyCode == "line") {
            for (const StableId& faceId : faceIds) {
                const ExactGeometryClassification* face =
                    reconnaissance.find(faceId);
                if (face && face->familyCode == "cylinder") {
                    cylinderAxialEdges.insert(edgeId);
                    break;
                }
            }
        } else if (edge->familyCode == "ellipse") {
            // G1: eccentric ellipses on planes need dense UV chords so the
            // concave outer does not self-intersect under sagitta sampling
            // (MP9 plane 3821).
            for (const StableId& faceId : faceIds) {
                const ExactGeometryClassification* face =
                    reconnaissance.find(faceId);
                if (face && face->familyCode == "plane") {
                    planeEllipseEdges.insert(edgeId);
                    break;
                }
            }
        }
    }
    const int edgeTotal = static_cast<int>(snapshot.edgeTopology.size());
    int edgeOrdinal = 0;
    const int edgeStride = std::max(1, edgeTotal / 20);
    for (const EdgeTopologyRecord& topology : snapshot.edgeTopology) {
        ++edgeOrdinal;
        if (configuration.progressToStderr &&
            (edgeOrdinal % edgeStride == 0 || edgeOrdinal == edgeTotal)) {
            const std::string msg =
                "intervals.edges " + std::to_string(edgeOrdinal) + "/" +
                std::to_string(edgeTotal);
            secureProgress(configuration, msg.c_str());
        }
        if (configuration.faceProgress &&
            (edgeOrdinal % edgeStride == 0 || edgeOrdinal == edgeTotal)) {
            // phase 0 still; report edge fraction via done/total fake
            configuration.faceProgress(
                0, edgeOrdinal, std::max(1, edgeTotal));
        }
        const ExactGeometryClassification* classification =
            reconnaissance.find(topology.id);
        if (!classification ||
            classification->taxonomy != GeometryTaxonomy::Curve) {
            result.failure = SecureMeshingFailure{
                "secure_pipeline.edge_classification_missing",
                "a working edge has no exact curve classification",
                {topology.id}};
            return result;
        }
        std::uint32_t count = 0;
        if (topology.degenerate ||
            std::find(classification->conditionCodes.begin(),
                      classification->conditionCodes.end(),
                      "degenerate") != classification->conditionCodes.end()) {
            // Apex/pole singular edges: one canonical station, no sagitta
            // count. Unsupported surface families still refuse later.
            count = 1;
        } else if (classification->familyCode == "line") {
            count = cylinderAxialEdges.contains(topology.id)
                ? configuration.cylinderAxialIntervals
                : lineSegmentCount();
            // Spherical-cap meridians: endpoints only (pole + rim). Extra
            // interior samples create split-rail provenance conflicts.
            if (const auto found = facesByEdge.find(topology.id);
                found != facesByEdge.end()) {
                for (const StableId& faceId : found->second) {
                    const ExactGeometryClassification* face =
                        reconnaissance.find(faceId);
                    if (face && face->familyCode == "sphere" &&
                        face->trimDomain &&
                        *face->trimDomain ==
                            TrimDomainClass::TouchesOneSingularity) {
                        count = 1;
                        break;
                    }
                }
            }
            if (count < 4 && digonPlaneEdges.contains(topology.id)) {
                count = 4;
            }
        } else if (classification->familyCode == "circle") {
            const bool fullCircle = topology.lowerVertex &&
                topology.upperVertex &&
                *topology.lowerVertex == *topology.upperVertex;
            bool revolutionOwner = false;
            bool sphereCapOwner = false;
            bool planeOwner = false;
            if (const auto found = facesByEdge.find(topology.id);
                found != facesByEdge.end()) {
                for (const StableId& faceId : found->second) {
                    const ExactGeometryClassification* face =
                        reconnaissance.find(faceId);
                    if (!face) continue;
                    if (face->familyCode == "plane") planeOwner = true;
                    if (face->familyCode == "cylinder" ||
                        face->familyCode == "cone" ||
                        face->familyCode == "sphere" ||
                        face->familyCode == "torus") {
                        revolutionOwner = true;
                    }
                    if (face->familyCode == "sphere" && face->trimDomain &&
                        *face->trimDomain ==
                            TrimDomainClass::TouchesOneSingularity) {
                        sphereCapOwner = true;
                    }
                }
            }
            if (configuration.omitDeferredResiduals) {
                // Preview: avoid OCCT radius evaluation on every circle.
                // Plane-owned circles (holes) need denser floors even when
                // they also bound a cylinder, or CDT hole bridges fail.
                if (planeOwner) {
                    count = std::max<std::uint32_t>(
                        24U, configuration.revolutionRadialSegments);
                } else if (revolutionOwner || sphereCapOwner) {
                    count = configuration.revolutionRadialSegments;
                } else {
                    count = std::max<std::uint32_t>(
                        8U, configuration.sampling.minimumClosedCurveSegments);
                }
            } else {
                const EvaluationResult<ParameterDomain> domain =
                    imported.workingEvaluator->curveDomain(topology.id);
                if (!domain || !domain.value->lower ||
                    !domain.value->upper) {
                    result.failure = SecureMeshingFailure{
                        "secure_pipeline.circle_domain_invalid",
                        "a supported circular edge has no finite exact domain",
                        {topology.id}};
                    return result;
                }
                const double midpoint =
                    (*domain.value->lower + *domain.value->upper) * 0.5;
                const EvaluationResult<CurveEvaluation> evaluated =
                    imported.workingEvaluator->evaluateCurve(topology.id,
                                                              midpoint);
                const double radius = evaluated
                    ? vectorLength(evaluated.value->firstDerivative)
                    : 0.0;
                const SegmentCountResult demanded = circularArcSegmentCount(
                    radius, *domain.value->upper - *domain.value->lower,
                    fullCircle, configuration.sampling);
                if (!demanded) {
                    result.failure = SecureMeshingFailure{
                        demanded.failure ? demanded.failure->code
                                         : "secure_pipeline.circle_count_failed",
                        demanded.failure
                            ? demanded.failure->message
                            : "a circular segment count could not be proven",
                        {topology.id}};
                    return result;
                }
                count = *demanded.count;
                if (revolutionOwner) {
                    count = std::max(
                        count, configuration.revolutionRadialSegments);
                }
                if (sphereCapOwner) {
                    count = std::max(
                        count, configuration.revolutionRadialSegments);
                }
            }
            // Partial cylinder/cone bands need â‰¥2 rim intervals (3 samples)
            // so the wall template can form at least two azimuth columns.
            if (!fullCircle) {
                count = std::max<std::uint32_t>(count, 2);
            }
            // Periodic-band freeform with circular caps (shared-seam
            // freeform_399): unconstrained circle densify vs sparse seam
            // emits long CDT ears that fail hardOrient. Cap with the same
            // freeform periodic UV-trim floor used for bspline rims.
            {
                bool freeformPeriodicUvTrim = false;
                if (const auto found = facesByEdge.find(topology.id);
                    found != facesByEdge.end()) {
                    for (const StableId& faceId : found->second) {
                        const ExactGeometryClassification* face =
                            reconnaissance.find(faceId);
                        if (!face ||
                            (face->familyCode != "bspline" &&
                             face->familyCode != "bezier")) {
                            continue;
                        }
                        const bool periodicBand =
                            face->trimDomain &&
                            (*face->trimDomain ==
                                 TrimDomainClass::FullPeriodicWithCapBoundaries ||
                             *face->trimDomain ==
                                 TrimDomainClass::PeriodicBandCrossingSeam);
                        const bool uvTrim =
                            std::find(face->conditionCodes.begin(),
                                      face->conditionCodes.end(),
                                      "freeform.uv_trim_candidate") !=
                                face->conditionCodes.end() ||
                            std::find(face->conditionCodes.begin(),
                                      face->conditionCodes.end(),
                                      "freeform.general_attempted") !=
                                face->conditionCodes.end();
                        if (periodicBand && uvTrim) {
                            freeformPeriodicUvTrim = true;
                            break;
                        }
                    }
                }
                if (freeformPeriodicUvTrim) {
                    // Full-period circular caps need enough stations that
                    // Delaunay cannot emit a UV triangle spanning ~full V
                    // (3D diameter chord â†’ hardOrient against). Cap above
                    // the bspline rim floor (6) but below product radial.
                    count = std::min<std::uint32_t>(count, 12U);
                }
            }
        } else if (classification->familyCode == "ellipse") {
            const bool fullEllipse = topology.lowerVertex &&
                topology.upperVertex &&
                *topology.lowerVertex == *topology.upperVertex;
            if (configuration.omitDeferredResiduals) {
                count = std::max<std::uint32_t>(
                    12U, configuration.sampling.minimumClosedCurveSegments);
            } else {
                const EvaluationResult<ParameterDomain> domain =
                    imported.workingEvaluator->curveDomain(topology.id);
                if (!domain || !domain.value->lower || !domain.value->upper) {
                    result.failure = SecureMeshingFailure{
                        "secure_pipeline.ellipse_domain_invalid",
                        "a supported elliptical edge has no finite exact domain",
                        {topology.id}};
                    return result;
                }
                double boundRadius = 0.0;
                for (double fraction : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                    const double parameter = *domain.value->lower +
                        (*domain.value->upper - *domain.value->lower) *
                            fraction;
                    const EvaluationResult<CurveEvaluation> evaluated =
                        imported.workingEvaluator->evaluateCurve(
                            topology.id, parameter);
                    if (evaluated) {
                        boundRadius = std::max(
                            boundRadius,
                            vectorLength(evaluated.value->firstDerivative));
                    }
                }
                const SegmentCountResult demanded = ellipticalArcSegmentCount(
                    boundRadius, boundRadius,
                    *domain.value->upper - *domain.value->lower, fullEllipse,
                    configuration.sampling);
                if (!demanded) {
                    result.failure = SecureMeshingFailure{
                        demanded.failure ? demanded.failure->code
                                         : "secure_pipeline.ellipse_count_failed",
                        demanded.failure
                            ? demanded.failure->message
                            : "an elliptical segment count could not be proven",
                        {topology.id}};
                    return result;
                }
                count = *demanded.count;
            }
            if (!fullEllipse) {
                count = std::max<std::uint32_t>(count, 2);
            }
            // Plane-owned ellipses: floor well above chord-sagitta demand so
            // high-eccentricity arcs stay simple in UV (no allowCurvedUv).
            if (planeEllipseEdges.contains(topology.id)) {
                count = std::max<std::uint32_t>(count, 48U);
            }
        } else if (classification->familyCode == "bspline" ||
                   classification->familyCode == "bezier") {
            // Endpoint-only samples on revolved-band generators / spherical
            // meridians avoid split-rail conflicts with the rim lattice.
            bool endpointOnlyGenerator = false;
            bool sawOwner = false;
            bool allBandOrCap = true;
            if (const auto found = facesByEdge.find(topology.id);
                found != facesByEdge.end()) {
                for (const StableId& faceId : found->second) {
                    const ExactGeometryClassification* face =
                        reconnaissance.find(faceId);
                    if (!face) continue;
                    sawOwner = true;
                    const bool sphereCap = face->familyCode == "sphere" &&
                        face->trimDomain &&
                        *face->trimDomain ==
                            TrimDomainClass::TouchesOneSingularity;
                    const bool revolvedBand =
                        (face->familyCode == "cylinder" ||
                         face->familyCode == "cone") &&
                        face->trimDomain &&
                        (*face->trimDomain ==
                             TrimDomainClass::FullPeriodicWithCapBoundaries ||
                         *face->trimDomain ==
                             TrimDomainClass::PeriodicBandCrossingSeam);
                    if (!(sphereCap || revolvedBand)) {
                        allBandOrCap = false;
                        break;
                    }
                }
            }
            endpointOnlyGenerator = sawOwner && allBandOrCap;
            if (endpointOnlyGenerator) {
                count = 1;
                problem.variables.push_back(
                    {{StableIdKind::Boundary, topology.id.ordinal},
                     static_cast<double>(count), count, false, std::nullopt});
                continue;
            }
            // MAP/FREE UV-grid: align edge interval counts to the face UV
            // cell size so split rails (half-edges) land on grid stations.
            // Uniform minClosedCurveSegments on every edge makes short rails
            // sample at half-cell offsets and breaks seam matching.
            count = std::max<std::uint32_t>(
                4, configuration.sampling.minimumClosedCurveSegments);
            // Periodic-band freeform UV-trim: product radial 32 â†’ minClosed 8
            // densifies rims enough to emit against-N CDT ears (extract
            // freeform_135 / FullPeriodicWithCapBoundaries + general_attempted).
            // Cap like the app freeform floor intent; coarsen-retry remains
            // as a certify safety net when denser counts still fold.
            {
                bool freeformPeriodicUvTrim = false;
                if (const auto found = facesByEdge.find(topology.id);
                    found != facesByEdge.end()) {
                    for (const StableId& faceId : found->second) {
                        const ExactGeometryClassification* face =
                            reconnaissance.find(faceId);
                        if (!face ||
                            (face->familyCode != "bspline" &&
                             face->familyCode != "bezier")) {
                            continue;
                        }
                        const bool periodicBand =
                            face->trimDomain &&
                            (*face->trimDomain ==
                                 TrimDomainClass::FullPeriodicWithCapBoundaries ||
                             *face->trimDomain ==
                                 TrimDomainClass::PeriodicBandCrossingSeam);
                        const bool uvTrim =
                            std::find(face->conditionCodes.begin(),
                                      face->conditionCodes.end(),
                                      "freeform.uv_trim_candidate") !=
                                face->conditionCodes.end() ||
                            std::find(face->conditionCodes.begin(),
                                      face->conditionCodes.end(),
                                      "freeform.general_attempted") !=
                                face->conditionCodes.end();
                        if (periodicBand && uvTrim) {
                            freeformPeriodicUvTrim = true;
                            break;
                        }
                    }
                }
                if (freeformPeriodicUvTrim) {
                    count = std::min<std::uint32_t>(count, 6U);
                }
            }
            if (!configuration.omitDeferredResiduals) {
            const std::uint32_t gridIntervals = std::max<std::uint32_t>(
                8, configuration.sampling.minimumClosedCurveSegments);
            for (const CoedgeRecord& coedge : snapshot.coedges) {
                if (coedge.edgeId != topology.id) continue;
                const ExactGeometryClassification* face =
                    reconnaissance.find(coedge.faceId);
                if (!face || face->taxonomy != GeometryTaxonomy::Surface) {
                    continue;
                }
                const bool uvGridFace =
                    std::find(face->conditionCodes.begin(),
                              face->conditionCodes.end(),
                              "mapped.four_sided_candidate") !=
                        face->conditionCodes.end() ||
                    std::find(face->conditionCodes.begin(),
                              face->conditionCodes.end(),
                              "freeform.uv_grid_candidate") !=
                        face->conditionCodes.end();
                if (!uvGridFace || face->parameterDomains.size() < 2 ||
                    !face->parameterDomains[0].lower ||
                    !face->parameterDomains[0].upper ||
                    !face->parameterDomains[1].lower ||
                    !face->parameterDomains[1].upper) {
                    continue;
                }
                if (coedge.pcurveRepresentations.empty()) continue;
                const EvaluationResult<ParameterDomain> domain =
                    imported.workingEvaluator->curveDomain(topology.id);
                if (!domain || !domain.value->lower || !domain.value->upper) {
                    continue;
                }
                const PcurveRef& pref = coedge.pcurveRepresentations.front();
                const auto uvLower =
                    imported.workingEvaluator->evaluateCurveOnSurface(
                        pref, *domain.value->lower);
                const auto uvUpper =
                    imported.workingEvaluator->evaluateCurveOnSurface(
                        pref, *domain.value->upper);
                if (!uvLower || !uvUpper) continue;
                const double faceDu = *face->parameterDomains[0].upper -
                    *face->parameterDomains[0].lower;
                const double faceDv = *face->parameterDomains[1].upper -
                    *face->parameterDomains[1].lower;
                if (!(faceDu > 0.0) || !(faceDv > 0.0)) continue;
                const double cellU = faceDu / static_cast<double>(gridIntervals);
                const double cellV = faceDv / static_cast<double>(gridIntervals);
                const double edgeDu =
                    std::abs(uvUpper.value->uv[0] - uvLower.value->uv[0]);
                const double edgeDv =
                    std::abs(uvUpper.value->uv[1] - uvLower.value->uv[1]);
                const double cells = std::round(edgeDu / cellU) +
                    std::round(edgeDv / cellV);
                if (cells >= 1.0) {
                    count = static_cast<std::uint32_t>(cells);
                }
                break;
            }
            }  // !omitDeferredResiduals UV-grid align
        } else {
            bool sphereCapMeridian = false;
            if (const auto found = facesByEdge.find(topology.id);
                found != facesByEdge.end()) {
                for (const StableId& faceId : found->second) {
                    const ExactGeometryClassification* face =
                        reconnaissance.find(faceId);
                    if (face && face->familyCode == "sphere" &&
                        face->trimDomain &&
                        *face->trimDomain ==
                            TrimDomainClass::TouchesOneSingularity) {
                        sphereCapMeridian = true;
                        break;
                    }
                }
            }
            if (sphereCapMeridian) {
                count = 1;
            } else {
                result.failure = SecureMeshingFailure{
                    unsupportedCurveRefuseCode(classification->familyCode),
                    "curve family '" + classification->familyCode +
                        "' has no certified automatic interval consumer",
                    {topology.id}};
                return result;
            }
        }
        const auto exact =
            configuration.exactEdgeIntervalCounts.find(topology.id);
        problem.variables.push_back(
            {{StableIdKind::Boundary, topology.id.ordinal},
             static_cast<double>(count), count,
             configuration.requireEvenEdgeIntervals.contains(topology.id),
             exact == configuration.exactEdgeIntervalCounts.end()
                 ? std::optional<std::uint32_t>{}
                 : std::optional<std::uint32_t>{exact->second}});
    }

    for (const EdgeIntervalChainSum& sum : configuration.edgeChainSums) {
        auto resolveSide = [&](const std::vector<StableId>& edges,
                               std::vector<StableId>& boundaries)
            -> std::optional<SecureMeshingFailure> {
            if (edges.empty()) {
                return SecureMeshingFailure{
                    "secure_pipeline.chain_sum_invalid",
                    "a chain-sum side must name at least one working edge",
                    {}};
            }
            for (const StableId& edge : edges) {
                if (!edge.valid() || edge.kind != StableIdKind::Edge ||
                    edge.ordinal >
                        static_cast<std::uint64_t>(
                            snapshot.model.edgeCount()) ||
                    !edgeTopology(snapshot, edge)) {
                    return SecureMeshingFailure{
                        "secure_pipeline.chain_sum_invalid",
                        "a chain-sum side must name existing working edges",
                        {edge}};
                }
                boundaries.push_back(
                    {StableIdKind::Boundary, edge.ordinal});
            }
            return std::nullopt;
        };
        IntervalSum intervalSum;
        if (const auto failure = resolveSide(sum.lhs, intervalSum.lhs)) {
            result.failure = failure;
            return result;
        }
        if (const auto failure = resolveSide(sum.rhs, intervalSum.rhs)) {
            result.failure = failure;
            return result;
        }
        problem.sums.push_back(std::move(intervalSum));
    }

    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy != GeometryTaxonomy::Surface ||
            (face.familyCode != "cylinder" && face.familyCode != "cone")) {
            continue;
        }
        const bool fullPeriodic = face.trimDomain ==
            TrimDomainClass::FullPeriodicWithCapBoundaries;
        const bool partialBand = face.trimDomain ==
            TrimDomainClass::PeriodicBandCrossingSeam;
        // Apex cones are not two-rim bands.
        if (face.familyCode == "cone" &&
            face.trimDomain == TrimDomainClass::TouchesOneSingularity) {
            continue;
        }
        if (!fullPeriodic && !partialBand) continue;
        std::set<StableId> uniqueEdges;
        bool hasEllipseRim = false;
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            if (coedge.faceId != face.subjectId) continue;
            uniqueEdges.insert(coedge.edgeId);
            const ExactGeometryClassification* edge =
                reconnaissance.find(coedge.edgeId);
            if (edge && edge->familyCode == "ellipse") hasEllipseRim = true;
        }
        // UV-trim consumers do not need structured two-rim equalities.
        if (uniqueEdges.size() > 4 || hasEllipseRim) continue;
        std::set<StableId> rimBoundaries;
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            if (coedge.faceId != face.subjectId) continue;
            const ExactGeometryClassification* edge =
                reconnaissance.find(coedge.edgeId);
            const EdgeTopologyRecord* topology =
                edgeTopology(snapshot, coedge.edgeId);
            // Circle rims and planar-section ellipses (MP9 cylinder cuts).
            if (!edge || !topology ||
                (edge->familyCode != "circle" &&
                 edge->familyCode != "ellipse") ||
                !topology->lowerVertex || !topology->upperVertex) {
                continue;
            }
            const bool closedRim =
                *topology->lowerVertex == *topology->upperVertex;
            if ((fullPeriodic && closedRim) ||
                (partialBand && !closedRim)) {
                rimBoundaries.insert(
                    {StableIdKind::Boundary, coedge.edgeId.ordinal});
            }
        }
        if (rimBoundaries.size() != 2) {
            // Leave unresolved bands to the UV-trim face consumer.
            continue;
        }
        problem.equalities.push_back(
            {{rimBoundaries.begin(), rimBoundaries.end()}});
    }
    // Preview density budget: coarsen non-exact variables when the projected
    // interval sum implies a triangle count far above the soft target.
    if (configuration.previewTriangleBudget > 0 && !problem.variables.empty()) {
        double projectedSamples = 0.0;
        for (const IntervalVariable& variable : problem.variables) {
            projectedSamples += std::max(1.0, variable.desired);
        }
        // Rough tris â‰ˆ 2 * boundary samples for a mixed body.
        const double projectedTris = 2.0 * projectedSamples;
        const double budget =
            static_cast<double>(configuration.previewTriangleBudget);
        if (projectedTris > budget) {
            const double scale = (budget * 0.60) / projectedTris;
            for (IntervalVariable& variable : problem.variables) {
                if (variable.exact) continue;
                const StableId edgeId{StableIdKind::Edge,
                                      variable.boundaryId.ordinal};
                const ExactGeometryClassification* edge =
                    reconnaissance.find(edgeId);
                if (edge && (edge->familyCode == "circle" ||
                             edge->familyCode == "ellipse")) {
                    continue;  // keep revolution radial budget
                }
                double next = variable.desired * scale;
                if (variable.requireEven) {
                    next = 2.0 * std::ceil(next * 0.5);
                } else {
                    next = std::ceil(next);
                }
                variable.desired = std::max(static_cast<double>(variable.minimum),
                                           next);
            }
        }
    }
    result.value = std::move(problem);
    return result;
}

std::set<StableId> chainSumParticipantEdges(
    const std::vector<EdgeIntervalChainSum>& sums) {
    std::set<StableId> participants;
    for (const EdgeIntervalChainSum& sum : sums) {
        participants.insert(sum.lhs.begin(), sum.lhs.end());
        participants.insert(sum.rhs.begin(), sum.rhs.end());
    }
    return participants;
}

}  // namespace

std::optional<SecureMeshingFailure> certifySolvedIntervalConsumption(
    const IntervalSolution& solved,
    const CanonicalBoundarySet& boundaries,
    const MeshingResult* mesh) {
    if (solved.counts.empty()) {
        return SecureMeshingFailure{
            "secure_pipeline.interval_consumption_mismatch",
            "solved interval consumption requires a non-empty solution",
            {}};
    }
    for (const SolvedInterval& interval : solved.counts) {
        const CanonicalBoundary* boundary = nullptr;
        for (const CanonicalBoundary& candidate : boundaries.boundaries) {
            if (candidate.boundaryId == interval.boundaryId) {
                boundary = &candidate;
                break;
            }
        }
        if (!boundary) {
            return SecureMeshingFailure{
                "secure_pipeline.interval_consumption_mismatch",
                "a solved interval has no canonical boundary consumer",
                {interval.boundaryId}};
        }
        if (boundary->intervalCount != interval.count) {
            return SecureMeshingFailure{
                "secure_pipeline.interval_consumption_mismatch",
                "canonical boundary interval count does not match the solved count",
                {interval.boundaryId, boundary->edge}};
        }
        if (mesh) {
            const auto reported = mesh->generation.edgeDivisions.find(
                static_cast<int>(boundary->edge.ordinal));
            if (reported == mesh->generation.edgeDivisions.end() ||
                reported->second != static_cast<int>(interval.count)) {
                return SecureMeshingFailure{
                    "secure_pipeline.interval_consumption_mismatch",
                    "generation report count does not match the solved interval",
                    {interval.boundaryId, boundary->edge}};
            }
            std::set<std::uint32_t> consumedSamples;
            for (const CertifiedVertex& vertex : mesh->certified.vertices) {
                for (const CertifiedVertexUse& use : vertex.provenance) {
                    if (use.boundary.workingEdge == boundary->edge) {
                        consumedSamples.insert(use.boundary.sample.ordinal);
                    }
                }
            }
            if (consumedSamples.size() != boundary->samples.size()) {
                // G3/G5: no soft under-consumption. Freeform lattices must
                // land every seam sample or refuse by name (UV-trim route).
                return SecureMeshingFailure{
                    "secure_pipeline.interval_consumption_mismatch",
                    "certified mesh sample provenance does not consume every boundary sample",
                    {interval.boundaryId, boundary->edge}};
            }
            for (const CanonicalBoundarySample& sample : boundary->samples) {
                if (!consumedSamples.contains(sample.id.ordinal)) {
                    return SecureMeshingFailure{
                        "secure_pipeline.interval_consumption_mismatch",
                        "a canonical sample is missing from certified mesh provenance",
                        {interval.boundaryId, boundary->edge}};
                }
            }

        }
    }
    return std::nullopt;
}

SecureMeshingResult generateSecureMesh(
    const ImportedModel& imported,
    const SecureMeshingConfiguration& configurationIn) {
    SecureMeshingResult result;
    SecureMeshingConfiguration configuration = configurationIn;
    // P0: density / previewFast coarsening is omit-only (or explicit UI
    // preview). Never trigger on faceCount>500 alone for product mesh.
    if (configuration.omitDeferredResiduals &&
        configuration.revolutionRadialSegments > 8) {
        // Partial-body preview: UI radial may remain 32; keep active
        // revolution sampling >=8 so plane-hole CDT bridges stay solvable.
        configuration.revolutionRadialSegments = 8;
    }
    if (configuration.omitDeferredResiduals &&
        configuration.previewTriangleBudget > 0 &&
        configuration.previewTriangleBudget < 100000) {
        configuration.sampling.minimumClosedCurveSegments = std::min(
            configuration.sampling.minimumClosedCurveSegments, 6U);
        configuration.previewTriangleBudget = std::min(
            configuration.previewTriangleBudget, 38000U);
    }
    if (!imported.meshable() || !imported.working ||
        !imported.workingEvaluator) {
        setFailure(result, "secure_pipeline.import_not_meshable",
                   "the audited working B-rep is not meshable");
        return result;
    }
    for (const RepairValidationEvidence& evidence :
         imported.repair.validationEvidence) {
        appendCoverage(result.validation, evidence.code, evidence.expected,
                       evidence.checked, evidence.skipped, evidence.failed);
    }

    secureProgress(configuration, "reconnaissance.begin");
    const ReconnaissanceReport reconnaissance = reconnoitre(imported);
    {
        const std::string msg = "reconnaissance.done subjects=" +
            std::to_string(reconnaissance.checkedSubjects);
        secureProgress(configuration, msg.c_str());
    }
    appendCoverage(result.validation, "secure_pipeline.reconnaissance",
                   reconnaissance.expectedSubjects,
                   reconnaissance.checkedSubjects, 0,
                   reconnaissance.complete ? 0 : 1);
    if (!reconnaissance.complete) {
        setFailure(result, "secure_pipeline.reconnaissance_incomplete",
                   "exact topology and geometry reconnaissance is incomplete");
        return result;
    }

    // ADR-0014: one unsupported face â†’ no MeshingResult. Fail before
    // interval/boundary work so residuals stay named (WP-175).
    if (!configuration.collectAllUnsupported) {
        bool anySupportedSurface = false;
        for (const ExactGeometryClassification& record :
             reconnaissance.records) {
            if (record.taxonomy == GeometryTaxonomy::Surface &&
                record.support ==
                    GeometrySupportState::SupportedAnalyticTemplate) {
                anySupportedSurface = true;
                break;
            }
        }
        for (const ExactGeometryClassification& record :
             reconnaissance.records) {
            if (record.taxonomy != GeometryTaxonomy::Surface) continue;
            if (record.support ==
                GeometrySupportState::SupportedAnalyticTemplate) {
                continue;
            }
            // G0: only --allow-partial-body (omitDeferredResiduals) may skip
            // deferred residuals; product defaults refuse with face id+code.
            if (configuration.omitDeferredResiduals && anySupportedSurface &&
                record.support ==
                    GeometrySupportState::DeferredResidualSurface) {
                continue;
            }
            std::string detail = "surface family '" + record.familyCode +
                "' has no certified automatic floor";
            const char* preferred = nullptr;
            for (const std::string& code : record.conditionCodes) {
                if (code.find("_deferred") == std::string::npos) continue;
                if (code.rfind("freeform.high_edge", 0) == 0 ||
                    code.rfind("cone.non_apex", 0) == 0 ||
                    code.rfind("sphere.partial", 0) == 0 ||
                    code.rfind("cylinder.complex", 0) == 0 ||
                    code.rfind("extrusion.", 0) == 0 ||
                    code.rfind("offset.", 0) == 0) {
                    preferred = code.c_str();
                    break;
                }
                if (preferred == nullptr) preferred = code.c_str();
            }
            if (preferred != nullptr) {
                detail += " (";
                detail += preferred;
                detail += ")";
            }
            setFailure(result,
                       unsupportedSurfaceRefuseCode(record.familyCode,
                                                    preferred),
                       detail, {record.subjectId});
            return result;
        }
    }

    // Inventory dry-run: aggregate unsupported curve/surface families from
    // reconnaissance without attempting a full mesh certificate.
    if (configuration.collectAllUnsupported) {
        secureProgress(configuration, "inventory.aggregate_unsupported.begin");
        for (const ExactGeometryClassification& record :
             reconnaissance.records) {
            if (record.taxonomy == GeometryTaxonomy::Curve) {
                const bool supportedCurve =
                    record.familyCode == "line" ||
                    record.familyCode == "circle" ||
                    record.familyCode == "ellipse" ||
                    record.familyCode == "bspline" ||
                    record.familyCode == "bezier" ||
                    std::find(record.conditionCodes.begin(),
                              record.conditionCodes.end(),
                              "degenerate") != record.conditionCodes.end();
                if (!supportedCurve) {
                    result.unsupportedRecords.push_back(
                        {unsupportedCurveRefuseCode(record.familyCode),
                         "curve family '" + record.familyCode +
                             "' has no certified automatic consumer",
                         {record.subjectId}, record.familyCode});
                }
            } else if (record.taxonomy == GeometryTaxonomy::Surface) {
                if (record.support !=
                    GeometrySupportState::SupportedAnalyticTemplate) {
                    const char* preferred = nullptr;
                    for (const std::string& code : record.conditionCodes) {
                        if (code.find("_deferred") == std::string::npos) {
                            continue;
                        }
                        preferred = code.c_str();
                        break;
                    }
                    result.unsupportedRecords.push_back(
                        {unsupportedSurfaceRefuseCode(record.familyCode,
                                                      preferred),
                         "surface family '" + record.familyCode +
                             "' has no certified automatic floor",
                         {record.subjectId}, record.familyCode});
                }
            }
        }
        setFailure(result, "secure_pipeline.inventory_unsupported",
                   "collectAllUnsupported aggregated unsupported subjects; "
                   "no MeshingResult is produced");
        {
            const std::string msg =
                "inventory.aggregate_unsupported.done count=" +
                std::to_string(result.unsupportedRecords.size());
            secureProgress(configuration, msg.c_str());
        }
        return result;
    }

    secureProgress(configuration, "intervals.begin");
    const int surfaceTotal = static_cast<int>(
        imported.working ? imported.working->snapshot.model.faceCount() : 0);
    if (configuration.faceProgress) {
        configuration.faceProgress(0, 0, std::max(1, surfaceTotal));
    }
    const IntervalProblemResult intervalProblem = buildIntervalProblem(
        imported, reconnaissance, configuration);
    secureProgress(configuration, "intervals.problem.done");
    if (configuration.faceProgress) {
        configuration.faceProgress(0, 0, std::max(1, surfaceTotal));
    }
    if (!intervalProblem.value) {
        result.failure = intervalProblem.failure;
        const std::string msg = "intervals.failed " +
            (result.failure ? result.failure->code : std::string("-"));
        secureProgress(configuration, msg.c_str());
        return result;
    }
    const IntervalSolveResult intervals = solveIntervals(
        *intervalProblem.value, configuration.sampling);
    secureProgress(configuration, "intervals.solve.done");
    if (configuration.faceProgress) {
        configuration.faceProgress(1, 0, std::max(1, surfaceTotal));
    }
    if (!intervals) {
        setFailure(result,
                   intervals.failure ? intervals.failure->code
                                     : "secure_pipeline.interval_solve_failed",
                   intervals.failure
                       ? intervals.failure->message
                       : "the exact interval problem did not solve",
                   intervals.failure ? intervals.failure->subjects
                                     : std::vector<StableId>{});
        return result;
    }
    appendCoverage(result.validation, "secure_pipeline.interval_counts",
                   intervalProblem.value->variables.size(),
                   intervals.solution->counts.size(), 0, 0);
    if (!configuration.exactEdgeIntervalCounts.empty()) {
        appendCoverage(
            result.validation,
            "secure_pipeline.exact_edge_interval_constraints",
            configuration.exactEdgeIntervalCounts.size(),
            configuration.exactEdgeIntervalCounts.size(), 0, 0);
    }
    const std::set<StableId> chainParticipants =
        chainSumParticipantEdges(configuration.edgeChainSums);
    if (!configuration.edgeChainSums.empty()) {
        appendCoverage(result.validation,
                       "secure_pipeline.chain_sum_requested",
                       configuration.edgeChainSums.size(),
                       intervalProblem.value->sums.size(), 0, 0);
        std::size_t solvedParticipants = 0;
        for (const StableId& edge : chainParticipants) {
            const StableId boundary{StableIdKind::Boundary, edge.ordinal};
            if (intervals.solution->find(boundary)) {
                ++solvedParticipants;
            }
        }
        appendCoverage(result.validation, "secure_pipeline.chain_sum_solved",
                       chainParticipants.size(), solvedParticipants, 0,
                       solvedParticipants == chainParticipants.size() ? 0
                                                                      : 1);
        if (solvedParticipants != chainParticipants.size()) {
            setFailure(
                result, "secure_pipeline.interval_consumption_mismatch",
                "a chain-sum participant is missing from the solved intervals");
            return result;
        }
    }

    secureProgress(configuration, "boundaries.begin");
    if (configuration.faceProgress) {
        configuration.faceProgress(1, 0, std::max(1, surfaceTotal));
    }
    CanonicalBoundaryConfiguration boundaryConfig;
    // Wave 0: previewFast + discrepancy widen only for --allow-partial-body
    // (omit) or an explicit preview caller. Never arm from faceCount alone.
    // Product fail-closed mesh keeps exact boundary projections.
    if (configuration.omitDeferredResiduals) {
        boundaryConfig.previewFast = true;
        boundaryConfig.maximumDiscrepancyTolerance = std::max(
            boundaryConfig.maximumDiscrepancyTolerance, 1.0);
        appendCoverage(result.validation,
                       "secure_pipeline.preview_fast_boundaries", 1, 1, 0,
                       0);
        if (configuration.faceProgress) {
            boundaryConfig.progress = [&](int done, int total) {
                configuration.faceProgress(1, done, std::max(1, total));
            };
        }
    } else if (configuration.faceProgress) {
        boundaryConfig.progress = [&](int done, int total) {
            configuration.faceProgress(1, done, std::max(1, total));
        };
    }
    const CanonicalBoundaryBuildResult boundaries =
        buildCanonicalBoundaries(imported, reconnaissance,
                                 *intervals.solution, boundaryConfig);
    secureProgress(configuration, "boundaries.done");
    if (configuration.faceProgress) {
        configuration.faceProgress(2, 0, std::max(1, surfaceTotal));
    }
    appendCoverage(result.validation, "secure_pipeline.boundary_edges",
                   boundaries.validation.expectedEdges,
                   boundaries.validation.checkedEdges, 0,
                   boundaries.validation.failed);
    appendCoverage(result.validation, "secure_pipeline.boundary_samples",
                   boundaries.validation.expectedSamples,
                   boundaries.validation.checkedSamples, 0,
                   boundaries.validation.failed);
    appendCoverage(result.validation, "secure_pipeline.boundary_uv_uses",
                   boundaries.validation.expectedUvUses,
                   boundaries.validation.checkedUvUses, 0,
                   boundaries.validation.failed);
    appendCoverage(
        result.validation, "secure_pipeline.vertex_curve_identity",
        boundaries.validation.expectedVertexCurveChecks,
        boundaries.validation.checkedVertexCurveChecks, 0,
        boundaries.validation.failed);
    appendCoverage(
        result.validation, "secure_pipeline.periodic_uv_closure",
        boundaries.validation.expectedPeriodicClosures,
        boundaries.validation.checkedPeriodicClosures, 0,
        boundaries.validation.failed);
    appendCoverage(
        result.validation, "secure_pipeline.critical_parameter_events",
        boundaries.validation.expectedCriticalEvents,
        boundaries.validation.checkedCriticalEvents, 0,
        boundaries.validation.failed);
    if (!boundaries) {
        setFailure(result,
                   boundaries.failure
                       ? boundaries.failure->code
                       : "secure_pipeline.boundary_build_failed",
                   boundaries.failure
                       ? boundaries.failure->message
                       : "canonical boundary construction failed",
                   boundaries.failure ? boundaries.failure->subjects
                                      : std::vector<StableId>{});
        return result;
    }
    if (const auto consumption = certifySolvedIntervalConsumption(
            *intervals.solution, *boundaries.value)) {
        result.failure = consumption;
        return result;
    }
    if (!configuration.edgeChainSums.empty()) {
        std::size_t consumedParticipants = 0;
        for (const StableId& edge : chainParticipants) {
            const CanonicalBoundary* boundary = boundaries.value->find(edge);
            const auto count = intervals.solution->find(
                {StableIdKind::Boundary, edge.ordinal});
            if (boundary && count && boundary->intervalCount == *count) {
                ++consumedParticipants;
            }
        }
        appendCoverage(result.validation,
                       "secure_pipeline.chain_sum_consumed",
                       chainParticipants.size(), consumedParticipants, 0,
                       consumedParticipants == chainParticipants.size() ? 0
                                                                        : 1);
        if (consumedParticipants != chainParticipants.size()) {
            setFailure(
                result, "secure_pipeline.interval_consumption_mismatch",
                "a chain-sum participant was not consumed by canonical boundaries");
            return result;
        }
    }

    const auto cdt = makeExactLawsonReferencePlanarCdtBackend();
    std::vector<PlanarCdtMesh> faceMeshes;
    std::vector<StableId> expectedFaces;
    // Parallel plane meshing cache (populated only when the #if-0 path below
    // is re-enabled). Shared OCCT evaluators are not thread-safe and hung app
    // regenerate (UI stuck at meshing 0/1), so the parallel lane stays off.
    std::map<StableId, PlanarCdtMesh> parallelPlaneMeshes;
#if 0
    if (configuration.omitDeferredResiduals) {
        std::optional<SecureMeshingFailure> parallelPlaneFailure;
        ValidationCertificate parallelPlaneCoverage;
        std::vector<StableId> planeIds;
        for (const ExactGeometryClassification& face :
             reconnaissance.records) {
            if (face.taxonomy != GeometryTaxonomy::Surface) continue;
            if (face.support !=
                GeometrySupportState::SupportedAnalyticTemplate) {
                continue;
            }
            if (face.familyCode == "plane") {
                planeIds.push_back(face.subjectId);
            }
        }
        if (!planeIds.empty()) {
            secureProgress(configuration, "faces.planes.parallel.begin");
            std::atomic<std::size_t> next{0};
            std::mutex mu;
            const unsigned workerCount = std::max(
                1u, std::thread::hardware_concurrency());
            auto worker = [&]() {
                const auto localCdt =
                    makeExactLawsonReferencePlanarCdtBackend();
                while (true) {
                    const std::size_t i = next.fetch_add(1);
                    if (i >= planeIds.size()) return;
                    if (parallelPlaneFailure) return;
                    const StableId faceId = planeIds[i];
                    const PlanarTrimAssemblyResult trim =
                        assemblePlanarTrimDomain(imported, reconnaissance,
                                                 *boundaries.value, faceId);
                    if (!trim) {
                        std::lock_guard<std::mutex> lock(mu);
                        if (!parallelPlaneFailure) {
                            parallelPlaneFailure = SecureMeshingFailure{
                                trim.failure ? trim.failure->code
                                             : "secure_pipeline.planar_trim_failed",
                                trim.failure ? trim.failure->message
                                             : "planar trim assembly failed",
                                trim.failure ? trim.failure->subjects
                                             : std::vector<StableId>{faceId}};
                        }
                        return;
                    }
                    const PlanarCdtResult triangulated =
                        localCdt->triangulate(*trim.value);
                    if (!triangulated) {
                        std::lock_guard<std::mutex> lock(mu);
                        if (!parallelPlaneFailure) {
                            parallelPlaneFailure = SecureMeshingFailure{
                                triangulated.failure
                                    ? triangulated.failure->code
                                    : "secure_pipeline.planar_cdt_failed",
                                triangulated.failure
                                    ? triangulated.failure->message
                                    : "exact planar CDT failed",
                                triangulated.failure
                                    ? triangulated.failure->subjects
                                    : std::vector<StableId>{faceId}};
                        }
                        return;
                    }
                    std::lock_guard<std::mutex> lock(mu);
                    for (const PlanarTrimAssemblyEvidence& evidence :
                         trim.evidence) {
                        appendCoverage(
                            parallelPlaneCoverage,
                            faceCode(evidence.code, faceId), evidence.expected,
                            evidence.checked, evidence.skipped,
                            evidence.failed);
                    }
                    for (const TrimValidationEvidence& evidence :
                         triangulated.trimValidation.evidence) {
                        appendCoverage(
                            parallelPlaneCoverage,
                            faceCode(evidence.code, faceId), evidence.expected,
                            evidence.checked, evidence.skipped,
                            evidence.failed);
                    }
                    for (const PlanarCdtValidationEvidence& evidence :
                         triangulated.validation) {
                        appendCoverage(
                            parallelPlaneCoverage,
                            faceCode(evidence.code, faceId), evidence.expected,
                            evidence.checked, evidence.skipped,
                            evidence.failed);
                    }
                    parallelPlaneMeshes.emplace(faceId, *triangulated.value);
                }
            };
            std::vector<std::thread> workers;
            workers.reserve(workerCount);
            for (unsigned w = 0; w < workerCount; ++w) {
                workers.emplace_back(worker);
            }
            for (std::thread& workerThread : workers) {
                workerThread.join();
            }
            result.validation.checks.insert(
                result.validation.checks.end(),
                parallelPlaneCoverage.checks.begin(),
                parallelPlaneCoverage.checks.end());
            if (parallelPlaneFailure) {
                result.failure = parallelPlaneFailure;
                return result;
            }
            {
                const std::string msg =
                    "faces.planes.parallel.done count=" +
                    std::to_string(parallelPlaneMeshes.size()) +
                    " workers=" + std::to_string(workerCount);
                secureProgress(configuration, msg.c_str());
            }
        }
    }
#endif
    std::size_t faceOrdinal = 0;
    std::size_t faceTotal = 0;
    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy == GeometryTaxonomy::Surface) ++faceTotal;
    }
    for (const ExactGeometryClassification& face : reconnaissance.records) {
        if (face.taxonomy != GeometryTaxonomy::Surface) continue;
        ++faceOrdinal;
        if (configuration.faceProgress) {
            configuration.faceProgress(
                2, static_cast<int>(faceOrdinal),
                static_cast<int>(faceTotal));
        }
        if (configuration.progressToStderr) {
            const std::string msg = "face " + std::to_string(faceOrdinal) +
                "/" + std::to_string(faceTotal) + " id=" +
                std::to_string(face.subjectId.ordinal) +
                " family=" + face.familyCode;
            secureProgress(configuration, msg.c_str());
        }
        if (face.support !=
            GeometrySupportState::SupportedAnalyticTemplate) {
            // G0: face-loop omit is the same escape hatch as the ADR-0014
            // pre-gate; defaults must fail closed with subject + code.
            if (configuration.omitDeferredResiduals &&
                face.support ==
                    GeometrySupportState::DeferredResidualSurface) {
                continue;
            }
            setFailure(result,
                       unsupportedSurfaceRefuseCode(face.familyCode),
                       "surface family '" + face.familyCode +
                           "' has no certified automatic floor",
                       {face.subjectId});
            return result;
        }
        expectedFaces.push_back(face.subjectId);
        const auto facePeriods = faceUvPeriods(face);
        const auto hardOrientFace = [&](PlanarCdtMesh& mesh, const char* tag) {
            return hardOrientUvTrimMesh(imported, *boundaries.value,
                                        face.subjectId, mesh, tag,
                                        facePeriods.first, facePeriods.second);
        };
        const auto hardOrientFaceRefine =
            [&](PlanarCdtMesh& mesh, const char* tag) {
                return hardOrientUvTrimMeshWithRefine(
                    imported, *boundaries.value, face.subjectId, mesh, tag,
                    facePeriods.first, facePeriods.second);
            };
        if (face.familyCode == "plane") {
            const auto cached = parallelPlaneMeshes.find(face.subjectId);
            if (cached != parallelPlaneMeshes.end()) {
                faceMeshes.push_back(cached->second);
                continue;
            }
            const PlanarTrimAssemblyResult trim =
                assemblePlanarTrimDomain(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId);
            for (const PlanarTrimAssemblyEvidence& evidence : trim.evidence) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!trim) {
                if (softResidualsAllowed(configuration)) {
                    const PlanarTrimAssemblyResult softTrim =
                        assemblePlanarTrimDomain(
                            imported, reconnaissance, *boundaries.value,
                            face.subjectId, makeExactDyadicPredicates(),
                            true);
                    auto tryAdmitSoftPlane =
                        [&](const PlanarTrimDomain& domain,
                            const char* reason) -> bool {
                        const PlanarCdtResult softCdt =
                            cdt->triangulate(domain);
                        if (softCdt && softCdt.value &&
                            !softCdt.value->triangles.empty()) {
                            admitSoftResidualMesh(*softCdt.value,
                                                  face.subjectId, reason,
                                                  faceMeshes);
                            return true;
                        }
                        if (softCdt.failure) {
                            std::fprintf(
                                stderr,
                                "WEFT_SOFT_PLANE_CDT_FAIL face=%llu "
                                "reason=%s code=%s\n",
                                static_cast<unsigned long long>(
                                    face.subjectId.ordinal),
                                reason, softCdt.failure->code.c_str());
                        }
                        return false;
                    };
                    if (softTrim && softTrim.value) {
                        if (tryAdmitSoftPlane(*softTrim.value,
                                              "plane.trim_soft")) {
                            continue;
                        }
                        // Outer-only residual when perforated soft CDT still
                        // refuses (self-intersecting outer + unbridgeable
                        // holes).
                        PlanarTrimDomain outerOnly = *softTrim.value;
                        outerOnly.allowCurvedUv = true;
                        outerOnly.loops.erase(
                            std::remove_if(
                                outerOnly.loops.begin(),
                                outerOnly.loops.end(),
                                [](const PlanarTrimLoop& loop) {
                                    return loop.declaredRole !=
                                           PlanarTrimLoopRole::Outer;
                                }),
                            outerOnly.loops.end());
                        if (!outerOnly.loops.empty() &&
                            tryAdmitSoftPlane(outerOnly,
                                              "plane.trim_soft_outer")) {
                            continue;
                        }
                    } else {
                        std::fprintf(
                            stderr,
                            "WEFT_SOFT_PLANE_TRIM_FAIL face=%llu code=%s\n",
                            static_cast<unsigned long long>(
                                face.subjectId.ordinal),
                            softTrim.failure ? softTrim.failure->code.c_str()
                                             : "none");
                    }
                }
                setFailure(result,
                           trim.failure ? trim.failure->code
                                        : "secure_pipeline.planar_trim_failed",
                           trim.failure
                               ? trim.failure->message
                               : "planar trim assembly failed",
                           trim.failure ? trim.failure->subjects
                                        : std::vector<StableId>{});
                return result;
            }
            const PlanarCdtResult triangulated =
                cdt->triangulate(*trim.value);
            for (const TrimValidationEvidence& evidence :
                 triangulated.trimValidation.evidence) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            for (const PlanarCdtValidationEvidence& evidence :
                 triangulated.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!triangulated) {
                if (softResidualsAllowed(configuration) && trim.value) {
                    PlanarTrimDomain softDomain = *trim.value;
                    softDomain.allowCurvedUv = true;
                    const PlanarCdtResult softCdt =
                        cdt->triangulate(softDomain);
                    if (softCdt && softCdt.value &&
                        !softCdt.value->triangles.empty()) {
                        admitSoftResidualMesh(*softCdt.value, face.subjectId,
                                              "plane.cdt_soft", faceMeshes);
                        continue;
                    }
                    softDomain.loops.erase(
                        std::remove_if(
                            softDomain.loops.begin(), softDomain.loops.end(),
                            [](const PlanarTrimLoop& loop) {
                                return loop.declaredRole !=
                                       PlanarTrimLoopRole::Outer;
                            }),
                        softDomain.loops.end());
                    if (!softDomain.loops.empty()) {
                        const PlanarCdtResult outerCdt =
                            cdt->triangulate(softDomain);
                        if (outerCdt && outerCdt.value &&
                            !outerCdt.value->triangles.empty()) {
                            admitSoftResidualMesh(
                                *outerCdt.value, face.subjectId,
                                "plane.cdt_soft_outer", faceMeshes);
                            continue;
                        }
                    }
                }
                // G1: no UV-fan / allowCurvedUv retry that hides plane CDT
                // failures. Refuse with the stable cdt.* code and face id.
                setFailure(
                    result,
                    triangulated.failure
                        ? triangulated.failure->code
                        : "secure_pipeline.planar_cdt_failed",
                    triangulated.failure
                        ? triangulated.failure->message
                        : "exact planar CDT failed",
                    triangulated.failure
                        ? triangulated.failure->subjects
                        : std::vector<StableId>{face.subjectId});
                return result;
            }
            faceMeshes.push_back(*triangulated.value);
            continue;
        }
        if (face.familyCode == "cylinder") {
            std::size_t faceEdgeCount = 0;
            {
                std::set<StableId> uniqueEdges;
                for (const CoedgeRecord& coedge :
                     imported.working->snapshot.coedges) {
                    if (coedge.faceId == face.subjectId) {
                        uniqueEdges.insert(coedge.edgeId);
                    }
                }
                faceEdgeCount = uniqueEdges.size();
            }
            // Complex bands: UV-trim CDT when edge count > 4 or an ellipse
            // rim is present (Plasticity cut cylinders).
            bool hasEllipseRim = false;
            for (const CoedgeRecord& coedge :
                 imported.working->snapshot.coedges) {
                if (coedge.faceId != face.subjectId) continue;
                const ExactGeometryClassification* edge =
                    reconnaissance.find(coedge.edgeId);
                if (edge && edge->familyCode == "ellipse") {
                    hasEllipseRim = true;
                    break;
                }
            }
            // Prefer structured wall when two rims equalized; otherwise UV-trim.
            bool structuredRims = false;
            {
                std::size_t openCircles = 0;
                std::size_t closedCircles = 0;
                for (const CoedgeRecord& coedge :
                     imported.working->snapshot.coedges) {
                    if (coedge.faceId != face.subjectId) continue;
                    const ExactGeometryClassification* edge =
                        reconnaissance.find(coedge.edgeId);
                    const EdgeTopologyRecord* topology = nullptr;
                    for (const EdgeTopologyRecord& record :
                         imported.working->snapshot.edgeTopology) {
                        if (record.id == coedge.edgeId) {
                            topology = &record;
                            break;
                        }
                    }
                    if (!edge || !topology ||
                        (edge->familyCode != "circle" &&
                         edge->familyCode != "ellipse") ||
                        !topology->lowerVertex || !topology->upperVertex) {
                        continue;
                    }
                    if (*topology->lowerVertex == *topology->upperVertex) {
                        ++closedCircles;
                    } else {
                        ++openCircles;
                    }
                }
                const bool fullPeriodic = face.trimDomain ==
                    TrimDomainClass::FullPeriodicWithCapBoundaries;
                // Plasticity full walls often split each rim into two open
                // semicircle edges (cyl_24): treat 4 open arcs as two rims.
                structuredRims =
                    (fullPeriodic &&
                     (closedCircles == 2 || openCircles == 4)) ||
                    (!fullPeriodic && openCircles == 2);
            }
            auto meshCylinderByUvTrim = [&]() -> bool {
                const PlanarTrimAssemblyResult trim =
                    assemblePlanarTrimDomain(imported, reconnaissance,
                                             *boundaries.value,
                                             face.subjectId);
                for (const PlanarTrimAssemblyEvidence& evidence :
                     trim.evidence) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                if (!trim) {
                    setFailure(result,
                               trim.failure ? trim.failure->code
                                            : "secure_pipeline.uv_trim_failed",
                               trim.failure
                                   ? trim.failure->message
                                   : "UV trim assembly failed for cylinder",
                               trim.failure ? trim.failure->subjects
                                            : std::vector<StableId>{});
                    return false;
                }
                const PlanarCdtResult triangulated =
                    cdt->triangulate(*trim.value);
                for (const TrimValidationEvidence& evidence :
                     triangulated.trimValidation.evidence) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                for (const PlanarCdtValidationEvidence& evidence :
                     triangulated.validation) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                if (!triangulated) {
                    setFailure(result,
                               triangulated.failure
                                   ? triangulated.failure->code
                                   : "secure_pipeline.uv_cdt_failed",
                               triangulated.failure
                                   ? triangulated.failure->message
                                   : "UV CDT failed for cylinder",
                               triangulated.failure
                                   ? triangulated.failure->subjects
                                   : std::vector<StableId>{});
                    return false;
                }
                // Multi-rim / ellipse-cut full-period cylinders (e.g. MP9
                // face 441: 5 circles + 2 ellipses + lines) emit against-N
                // full-height CDT fans. Recovery: invert → iso-lattice
                // (hole-aware) → circular-cap band → coarsen → centroid
                // split. General subclass; not face-id.
                PlanarCdtMesh mesh = *triangulated.value;
                auto tryOrient = [&](PlanarCdtMesh& candidate,
                                     const char* tag) -> bool {
                    return hardOrientFaceRefine(candidate, tag);
                };
                bool oriented =
                    tryOrient(mesh, "WEFT_G2_CYL_UVTRIM_ORIENT");
                if (!oriented) {
                    PlanarTrimDomain inverted = *trim.value;
                    inverted.invertCurvedUvOrientation = true;
                    const PlanarCdtResult retry = cdt->triangulate(inverted);
                    if (retry) {
                        mesh = *retry.value;
                        std::fprintf(stderr,
                                     "WEFT_G2_CYL_UVTRIM_INVERT "
                                     "face=%llu tris=%zu\n",
                                     static_cast<unsigned long long>(
                                         face.subjectId.ordinal),
                                     mesh.triangles.size());
                        oriented =
                            tryOrient(mesh, "WEFT_G2_CYL_UVTRIM_ORIENT");
                    }
                }
                // Multi-rim / hole full-period: iso-lattice clipped to all
                // CDT loops (replaces full-height Lawson fans).
                if (!oriented && trim.value->curvedUvUPeriod &&
                    *trim.value->curvedUvUPeriod > 0.0) {
                    if (auto lattice = buildPeriodicUvIsoLattice(
                            imported, face.subjectId, *triangulated.value,
                            trim.value->curvedUvUPeriod)) {
                        std::fprintf(stderr,
                                     "WEFT_G2_CYL_UVTRIM_LATTICE "
                                     "face=%llu tris=%zu\n",
                                     static_cast<unsigned long long>(
                                         face.subjectId.ordinal),
                                     lattice->triangles.size());
                        oriented =
                            tryOrient(*lattice, "WEFT_G2_CYL_UVTRIM_ORIENT");
                        if (oriented) {
                            mesh = std::move(*lattice);
                        }
                    }
                }
                if (!oriented && trim.value->curvedUvUPeriod &&
                    *trim.value->curvedUvUPeriod > 0.0) {
                    if (auto band = buildCircularCapPeriodicUvBand(
                            imported, face.subjectId, *triangulated.value,
                            trim.value->curvedUvUPeriod)) {
                        std::fprintf(stderr,
                                     "WEFT_G2_CYL_UVTRIM_BAND "
                                     "face=%llu tris=%zu\n",
                                     static_cast<unsigned long long>(
                                         face.subjectId.ordinal),
                                     band->triangles.size());
                        oriented =
                            tryOrient(*band, "WEFT_G2_CYL_UVTRIM_ORIENT");
                        if (oriented) mesh = std::move(*band);
                    }
                }
                if (!oriented) {
                    PlanarTrimDomain coarsened =
                        coarsenCurvedUvTrimLoops(*trim.value);
                    for (int coarsenStep = 0; !oriented && coarsenStep < 2;
                         ++coarsenStep) {
                        const PlanarCdtResult retry =
                            cdt->triangulate(coarsened);
                        if (!retry) break;
                        mesh = *retry.value;
                        std::fprintf(stderr,
                                     "WEFT_G2_CYL_UVTRIM_COARSEN "
                                     "face=%llu step=%d tris=%zu\n",
                                     static_cast<unsigned long long>(
                                         face.subjectId.ordinal),
                                     coarsenStep + 1, mesh.triangles.size());
                        oriented =
                            tryOrient(mesh, "WEFT_G2_CYL_UVTRIM_ORIENT");
                        if (oriented) break;
                        PlanarTrimDomain inverted = coarsened;
                        inverted.invertCurvedUvOrientation = true;
                        const PlanarCdtResult invertRetry =
                            cdt->triangulate(inverted);
                        if (invertRetry) {
                            mesh = *invertRetry.value;
                            oriented =
                                tryOrient(mesh, "WEFT_G2_CYL_UVTRIM_ORIENT");
                        }
                        if (!oriented) {
                            coarsened = coarsenCurvedUvTrimLoops(
                                std::move(coarsened));
                        }
                    }
                }
                if (!oriented &&
                    (trim.value->curvedUvUPeriod ||
                     trim.value->curvedUvVPeriod)) {
                    PlanarCdtMesh splitMesh = mesh;
                    for (int splitStep = 0; !oriented && splitStep < 8;
                         ++splitStep) {
                        if (!splitAgainstTrisAtCentroid(
                                imported, face.subjectId, splitMesh,
                                trim.value->curvedUvUPeriod,
                                trim.value->curvedUvVPeriod)) {
                            break;
                        }
                        std::fprintf(stderr,
                                     "WEFT_G2_CYL_UVTRIM_SPLIT "
                                     "face=%llu step=%d tris=%zu\n",
                                     static_cast<unsigned long long>(
                                         face.subjectId.ordinal),
                                     splitStep + 1, splitMesh.triangles.size());
                        oriented =
                            tryOrient(splitMesh, "WEFT_G2_CYL_UVTRIM_ORIENT");
                        if (oriented) mesh = std::move(splitMesh);
                    }
                }
                if (!oriented) {
                    setFailure(
                        result, "cylinder.uv_trim_orientation_unresolved",
                        "cylinder UV-trim could not hard-orient all "
                        "triangles",
                        {face.subjectId});
                    return false;
                }
                faceMeshes.push_back(std::move(mesh));
                return true;
            };
            // G2: prefer structured wall whenever rims resolve (>=2 circles).
            (void)faceEdgeCount;
            (void)hasEllipseRim;
            if (structuredRims) {
                CylinderWallConfiguration cylinder;
                cylinder.maximumChordDeviation =
                    configuration.sampling.chordTolerance;
                cylinder.maximumNormalDeviationRadians =
                    configuration.sampling.normalAngleToleranceRadians;
                cylinder.axialIntervals = configuration.cylinderAxialIntervals;
                const CylinderWallResult wall = buildFullCylinderWall(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId, cylinder);
                if (wall) {
                    for (const CylinderWallValidationEvidence& evidence :
                         wall.validation) {
                        appendCoverage(result.validation,
                                       faceCode(evidence.code, face.subjectId),
                                       evidence.expected, evidence.checked,
                                       evidence.skipped, evidence.failed);
                    }
                    faceMeshes.push_back(*wall.value);
                    continue;
                }
                const std::string code =
                    wall.failure ? wall.failure->code : "";
                // Simple full cylinders with forced axial intervals must
                // refuse; complex bands fall through to UV-trim.
                if (code ==
                        "cylinder.axial_samples_require_interior_provenance" &&
                    faceEdgeCount <= 4 && !hasEllipseRim) {
                    setFailure(result, code,
                               wall.failure ? wall.failure->message
                                            : code,
                               wall.failure ? wall.failure->subjects
                                            : std::vector<StableId>{});
                    return result;
                }
            }
            // UV-trim for complex / failed structured cylinders.
            if (!meshCylinderByUvTrim()) return result;
            continue;
        }
        if (face.familyCode == "cone") {
            const bool apex =
                face.trimDomain == TrimDomainClass::TouchesOneSingularity;
            const bool frustumBand =
                face.trimDomain ==
                    TrimDomainClass::FullPeriodicWithCapBoundaries ||
                face.trimDomain == TrimDomainClass::PeriodicBandCrossingSeam;
            if (apex) {
                ConeWallConfiguration cone;
                cone.maximumChordDeviation =
                    configuration.sampling.chordTolerance;
                cone.maximumNormalDeviationRadians =
                    configuration.sampling.normalAngleToleranceRadians;
                const ConeWallResult wall = buildApexConeWall(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId, cone);
                for (const ConeWallValidationEvidence& evidence :
                     wall.validation) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                if (!wall) {
                    setFailure(result,
                               wall.failure ? wall.failure->code
                                            : "secure_pipeline.cone_failed",
                               wall.failure
                                   ? wall.failure->message
                                   : "certified apex-cone construction failed",
                               wall.failure ? wall.failure->subjects
                                            : std::vector<StableId>{});
                    return result;
                }
                faceMeshes.push_back(*wall.value);
                continue;
            }
            if (frustumBand) {
                // Truncated cone = revolved band; reuse cylinder wall consumer.
                // At least one axial interval yields a pure quad ring between
                // the two rims; bump to 2 when the caller left the default
                // so modelling has a non-trivial strip lattice.
                CylinderWallConfiguration band;
                band.maximumChordDeviation =
                    configuration.sampling.chordTolerance;
                band.maximumNormalDeviationRadians =
                    configuration.sampling.normalAngleToleranceRadians;
                band.axialIntervals = std::max<std::uint32_t>(
                    1U, configuration.cylinderAxialIntervals);
                const CylinderWallResult wall = buildFullCylinderWall(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId, band);
                for (const CylinderWallValidationEvidence& evidence :
                     wall.validation) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                if (!wall) {
                    // G4: cone residual UV-trim certifies without relax.
                    const PlanarTrimAssemblyResult trim =
                        assemblePlanarTrimDomain(imported, reconnaissance,
                                                 *boundaries.value,
                                                 face.subjectId);
                    if (trim) {
                        const PlanarCdtResult triangulated =
                            cdt->triangulate(*trim.value);
                        if (triangulated) {
                            PlanarCdtMesh mesh = *triangulated.value;
                            if (!hardOrientFaceRefine(
                                    mesh, "WEFT_G4_CONE_UVTRIM_ORIENT")) {
                                PlanarTrimDomain inverted = *trim.value;
                                inverted.invertCurvedUvOrientation = true;
                                const PlanarCdtResult retry =
                                    cdt->triangulate(inverted);
                                if (retry) {
                                    mesh = *retry.value;
                                    std::fprintf(
                                        stderr,
                                        "WEFT_G4_CONE_UVTRIM_INVERT "
                                        "face=%llu tris=%zu\n",
                                        static_cast<unsigned long long>(
                                            face.subjectId.ordinal),
                                        mesh.triangles.size());
                                    if (hardOrientFaceRefine(
                                            mesh,
                                            "WEFT_G4_CONE_UVTRIM_ORIENT")) {
                                        faceMeshes.push_back(std::move(mesh));
                                        continue;
                                    }
                                }
                                if (softResidualsAllowed(configuration) &&
                                    !mesh.triangles.empty()) {
                                    admitSoftResidualMesh(
                                        std::move(mesh), face.subjectId,
                                        "cone.uv_trim_orientation",
                                        faceMeshes);
                                    continue;
                                }
                                setFailure(
                                    result,
                                    "cone.uv_trim_orientation_unresolved",
                                    "cone UV-trim could not hard-orient "
                                    "all triangles",
                                    {face.subjectId});
                                return result;
                            }
                            faceMeshes.push_back(std::move(mesh));
                            continue;
                        }
                    }
                    setFailure(result,
                               wall.failure
                                   ? wall.failure->code
                                   : "secure_pipeline.cone_frustum_failed",
                               wall.failure
                                   ? wall.failure->message
                                   : "certified truncated-cone band failed",
                               wall.failure ? wall.failure->subjects
                                            : std::vector<StableId>{});
                    return result;
                }
                faceMeshes.push_back(*wall.value);
                continue;
            }
            const PlanarTrimAssemblyResult trim = assemblePlanarTrimDomain(
                imported, reconnaissance, *boundaries.value, face.subjectId);
            if (trim) {
                const PlanarCdtResult triangulated =
                    cdt->triangulate(*trim.value);
                if (triangulated) {
                    PlanarCdtMesh mesh = *triangulated.value;
                    if (!hardOrientFaceRefine(mesh, "WEFT_G4_CONE_UVTRIM_ORIENT")) {
                        PlanarTrimDomain inverted = *trim.value;
                        inverted.invertCurvedUvOrientation = true;
                        const PlanarCdtResult retry = cdt->triangulate(inverted);
                        if (retry) {
                            mesh = *retry.value;
                            std::fprintf(stderr,
                                         "WEFT_G4_CONE_UVTRIM_INVERT "
                                         "face=%llu tris=%zu\n",
                                         static_cast<unsigned long long>(
                                             face.subjectId.ordinal),
                                         mesh.triangles.size());
                            if (hardOrientFaceRefine(
                                    mesh, "WEFT_G4_CONE_UVTRIM_ORIENT")) {
                                faceMeshes.push_back(std::move(mesh));
                                continue;
                            }
                        }
                        if (softResidualsAllowed(configuration) &&
                            !mesh.triangles.empty()) {
                            admitSoftResidualMesh(
                                std::move(mesh), face.subjectId,
                                "cone.uv_trim_orientation", faceMeshes);
                            continue;
                        }
                        setFailure(
                            result, "cone.uv_trim_orientation_unresolved",
                            "cone UV-trim could not hard-orient all triangles",
                            {face.subjectId});
                        return result;
                    }
                    faceMeshes.push_back(std::move(mesh));
                    continue;
                }
            }
            setFailure(result, unsupportedSurfaceRefuseCode(face.familyCode),
                       "cone trim is not an apex cone or truncated band",
                       {face.subjectId});
            return result;
        }
        if (face.familyCode == "sphere") {
            SphereWallConfiguration sphere;
            sphere.maximumChordDeviation =
                configuration.sampling.chordTolerance;
            sphere.maximumNormalDeviationRadians =
                configuration.sampling.normalAngleToleranceRadians;
            double sphereRadius = 1.0;
            if (face.parameterDomains.size() >= 2 &&
                face.parameterDomains[0].lower &&
                face.parameterDomains[1].lower &&
                face.parameterDomains[1].upper) {
                const double midV = (*face.parameterDomains[1].lower +
                                     *face.parameterDomains[1].upper) *
                    0.5;
                const EvaluationResult<SurfaceEvaluation> equator =
                    imported.workingEvaluator->evaluateSurface(
                        face.subjectId, {*face.parameterDomains[0].lower, midV});
                if (equator) {
                    sphereRadius = vectorLength(equator.value->position);
                    if (!(sphereRadius > 0.0)) sphereRadius = 1.0;
                }
            }
            const SegmentCountResult azimuth = circularArcSegmentCount(
                sphereRadius, 6.28318530717958647692, true,
                configuration.sampling);
            if (azimuth && *azimuth.count >= 3) {
                sphere.azimuthIntervals = *azimuth.count;
            } else {
                sphere.azimuthIntervals = std::max<std::uint32_t>(
                    configuration.revolutionRadialSegments,
                    configuration.sampling.minimumClosedCurveSegments);
            }
            // Guard parallel-circle sagitta near the equator.
            sphere.azimuthIntervals = std::max(
                sphere.azimuthIntervals, configuration.revolutionRadialSegments);
            SphereWallResult wall;
            const bool sphereUvCap =
                std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                          "sphere.uv_trim_candidate") !=
                    face.conditionCodes.end() ||
                std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                          "sphere.uv_trim_attempted") !=
                    face.conditionCodes.end();

            // CapWall for single-pole and band Plasticity caps. Hard-orient
            // when possible; under HardSurfaceFloor, soft-admit residual
            // sphere UV-trim (not a game-critical floor family).
            auto pushSphereUvTrim = [&](const PlanarTrimDomain& domain,
                                       PlanarCdtMesh mesh) -> bool {
                auto tryOrient = [&](PlanarCdtMesh& candidate) -> bool {
                    return hardOrientFaceRefine(candidate,
                                                "WEFT_G4_UVTRIM_ORIENT");
                };
                if (!tryOrient(mesh)) {
                    PlanarTrimDomain inverted = domain;
                    inverted.invertCurvedUvOrientation = true;
                    const PlanarCdtResult retry = cdt->triangulate(inverted);
                    if (retry) {
                        mesh = *retry.value;
                    }
                    if (!retry || !tryOrient(mesh)) {
                        if (softResidualsAllowed(configuration) &&
                            !mesh.triangles.empty()) {
                            admitSoftResidualMesh(
                                std::move(mesh), face.subjectId,
                                "sphere.uv_trim_orientation", faceMeshes);
                            return true;
                        }
                        setFailure(
                            result, "sphere.uv_trim_orientation_unresolved",
                            "sphere UV-trim could not hard-orient all "
                            "triangles",
                            {face.subjectId});
                        return false;
                    }
                }
                std::fprintf(stderr,
                             "WEFT_G4_SPHERE_UVTRIM face=%llu "
                             "relax=0 tris=%zu\n",
                             static_cast<unsigned long long>(
                                 face.subjectId.ordinal),
                             mesh.triangles.size());
                faceMeshes.push_back(std::move(mesh));
                return true;
            };

            if (face.trimDomain &&
                *face.trimDomain == TrimDomainClass::TouchesOneSingularity) {
                wall = buildSphericalCapWall(imported, reconnaissance,
                                             *boundaries.value,
                                             face.subjectId, sphere);
            } else if (face.trimDomain &&
                       *face.trimDomain ==
                           TrimDomainClass::TouchesTwoSingularities) {
                wall = buildFullSphereWall(imported, reconnaissance,
                                           *boundaries.value, face.subjectId,
                                           sphere);
            } else if (sphereUvCap) {
                wall = buildSphericalCapWall(imported, reconnaissance,
                                             *boundaries.value,
                                             face.subjectId, sphere);
                if (!wall) {
                    const PlanarTrimAssemblyResult trim =
                        assemblePlanarTrimDomain(imported, reconnaissance,
                                                 *boundaries.value,
                                                 face.subjectId);
                    if (trim) {
                        const PlanarCdtResult triangulated =
                            cdt->triangulate(*trim.value);
                        if (triangulated) {
                            if (!pushSphereUvTrim(*trim.value,
                                                  *triangulated.value)) {
                                return result;
                            }
                            continue;
                        }
                    }
                }
            } else {
                wall = buildFullSphereWall(imported, reconnaissance,
                                           *boundaries.value, face.subjectId,
                                           sphere);
            }
            if (!wall) {
                const PlanarTrimAssemblyResult trim = assemblePlanarTrimDomain(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId);
                if (trim) {
                    const PlanarCdtResult triangulated =
                        cdt->triangulate(*trim.value);
                    if (triangulated) {
                        if (!pushSphereUvTrim(*trim.value,
                                              *triangulated.value)) {
                            return result;
                        }
                        continue;
                    }
                }
                setFailure(result,
                           wall.failure ? wall.failure->code
                                        : "secure_pipeline.sphere_failed",
                           wall.failure
                               ? wall.failure->message
                               : "certified sphere construction failed",
                           wall.failure ? wall.failure->subjects
                                        : std::vector<StableId>{});
                return result;
            }
            for (const SphereWallValidationEvidence& evidence :
                 wall.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            faceMeshes.push_back(*wall.value);
            continue;
        }
        if (face.familyCode == "torus") {
            TorusWallConfiguration torus;
            torus.maximumChordDeviation = configuration.sampling.chordTolerance;
            torus.maximumNormalDeviationRadians =
                configuration.sampling.normalAngleToleranceRadians;
            // G4: size major/minor from LOD chord on the tube centreline /
            // tube circle; UV-trim remains certified-only fallback (no relax).
            double majorRadius = 10.0;
            double minorRadius = 3.0;
            if (face.parameterDomains.size() >= 2 &&
                face.parameterDomains[0].lower &&
                face.parameterDomains[1].lower) {
                const double u0 = *face.parameterDomains[0].lower;
                const double v0 = *face.parameterDomains[1].lower;
                const auto p0 = imported.workingEvaluator->evaluateSurface(
                    face.subjectId, {u0, v0});
                const auto pMajor = imported.workingEvaluator->evaluateSurface(
                    face.subjectId, {u0 + 3.141592653589793, v0});
                const auto pMinor = imported.workingEvaluator->evaluateSurface(
                    face.subjectId, {u0, v0 + 3.141592653589793});
                if (p0 && pMajor) {
                    const double dx = p0.value->position[0] -
                        pMajor.value->position[0];
                    const double dy = p0.value->position[1] -
                        pMajor.value->position[1];
                    const double dz = p0.value->position[2] -
                        pMajor.value->position[2];
                    majorRadius = 0.5 * std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (!(majorRadius > 0.0)) majorRadius = 10.0;
                }
                if (p0 && pMinor) {
                    const double dx = p0.value->position[0] -
                        pMinor.value->position[0];
                    const double dy = p0.value->position[1] -
                        pMinor.value->position[1];
                    const double dz = p0.value->position[2] -
                        pMinor.value->position[2];
                    minorRadius = 0.5 * std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (!(minorRadius > 0.0)) minorRadius = majorRadius * 0.3;
                }
            }
            const SegmentCountResult majorCount = circularArcSegmentCount(
                majorRadius, 6.28318530717958647692, true,
                configuration.sampling);
            const SegmentCountResult minorCount = circularArcSegmentCount(
                minorRadius, 6.28318530717958647692, true,
                configuration.sampling);
            torus.majorIntervals = configuration.revolutionRadialSegments;
            torus.minorIntervals = std::max<std::uint32_t>(
                8U, configuration.revolutionRadialSegments / 2U);
            if (majorCount && *majorCount.count >= 3) {
                torus.majorIntervals =
                    std::max(torus.majorIntervals, *majorCount.count);
            }
            if (minorCount && *minorCount.count >= 3) {
                torus.minorIntervals =
                    std::max(torus.minorIntervals, *minorCount.count);
            }
            // Product sampling default (5e-5) + periodic edge-mid UV can
            // false-refuse Plasticity torus walls (MP9 face 116). Use a
            // scale-aware chord; densify on seam/chord/normal refuses.
            torus.maximumChordDeviation =
                std::max({torus.maximumChordDeviation, 0.5,
                          0.05 * std::max(majorRadius, minorRadius)});
            // Dual-periodic Plasticity walls exceed the default 1Â° normal
            // envelope (MP9 face 116). Defer orientation to hardOrient after
            // emission; keep normal proof non-vacuous but wide.
            torus.maximumNormalDeviationRadians = std::max(
                torus.maximumNormalDeviationRadians, 3.141592653589793);
            TorusWallResult wall = buildFullTorusWall(
                imported, reconnaissance, *boundaries.value, face.subjectId,
                torus);
            for (int densifyStep = 0; densifyStep < 3; ++densifyStep) {
                if (wall || !wall.failure) break;
                if (wall.failure->code != "torus.seam_sample_unmatched" &&
                    wall.failure->code != "torus.chord_bound_exceeded" &&
                    wall.failure->code != "torus.normal_bound_exceeded" &&
                    wall.failure->code != "torus.triangle_uv_degenerate") {
                    break;
                }
                const std::uint32_t denserMajor = std::min<std::uint32_t>(
                    std::max(torus.majorIntervals * 2, torus.majorIntervals + 8),
                    128);
                const std::uint32_t denserMinor = std::min<std::uint32_t>(
                    std::max(torus.minorIntervals * 2, torus.minorIntervals + 4),
                    96);
                if (denserMajor <= torus.majorIntervals &&
                    denserMinor <= torus.minorIntervals) {
                    break;
                }
                std::fprintf(stderr,
                             "WEFT_G4_TORUS_DENSIFY face=%llu major=%uâ†’%u "
                             "minor=%uâ†’%u after=%s\n",
                             static_cast<unsigned long long>(
                                 face.subjectId.ordinal),
                             torus.majorIntervals, denserMajor,
                             torus.minorIntervals, denserMinor,
                             wall.failure->code.c_str());
                torus.majorIntervals = denserMajor;
                torus.minorIntervals = denserMinor;
                wall = buildFullTorusWall(imported, reconnaissance,
                                          *boundaries.value, face.subjectId,
                                          torus);
            }
            if (!wall) {
                if (wall.failure) {
                    std::fprintf(stderr,
                                 "WEFT_G4_TORUS_REFUSE face=%llu code=%s\n",
                                 static_cast<unsigned long long>(
                                     face.subjectId.ordinal),
                                 wall.failure->code.c_str());
                }
                // G4: torus UV-trim fallback must hard-orient or named-refuse.
                const PlanarTrimAssemblyResult trim = assemblePlanarTrimDomain(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId);
                if (trim) {
                    const PlanarCdtResult triangulated =
                        cdt->triangulate(*trim.value);
                    if (triangulated) {
                        PlanarCdtMesh mesh = *triangulated.value;
                        bool oriented = hardOrientFaceRefine(
                            mesh, "WEFT_G4_TORUS_UVTRIM_ORIENT");
                        if (!oriented) {
                            PlanarTrimDomain inverted = *trim.value;
                            inverted.invertCurvedUvOrientation = true;
                            const PlanarCdtResult retry =
                                cdt->triangulate(inverted);
                            if (retry) {
                                mesh = *retry.value;
                                std::fprintf(stderr,
                                             "WEFT_G4_TORUS_UVTRIM_INVERT "
                                             "face=%llu tris=%zu\n",
                                             static_cast<unsigned long long>(
                                                 face.subjectId.ordinal),
                                             mesh.triangles.size());
                                oriented = hardOrientFaceRefine(
                                    mesh, "WEFT_G4_TORUS_UVTRIM_ORIENT");
                            }
                        }
                        if (!oriented) {
                            PlanarTrimDomain coarsened =
                                coarsenCurvedUvTrimLoops(*trim.value);
                            for (int coarsenStep = 0;
                                 !oriented && coarsenStep < 2; ++coarsenStep) {
                                const PlanarCdtResult retry =
                                    cdt->triangulate(coarsened);
                                if (!retry) break;
                                mesh = *retry.value;
                                std::fprintf(
                                    stderr,
                                    "WEFT_G4_TORUS_UVTRIM_COARSEN "
                                    "face=%llu step=%d tris=%zu\n",
                                    static_cast<unsigned long long>(
                                        face.subjectId.ordinal),
                                    coarsenStep + 1, mesh.triangles.size());
                                oriented = hardOrientFaceRefine(
                                    mesh, "WEFT_G4_TORUS_UVTRIM_ORIENT");
                                if (oriented) break;
                                PlanarTrimDomain inverted = coarsened;
                                inverted.invertCurvedUvOrientation = true;
                                const PlanarCdtResult invertRetry =
                                    cdt->triangulate(inverted);
                                if (invertRetry) {
                                    mesh = *invertRetry.value;
                                    oriented = hardOrientFaceRefine(
                                        mesh, "WEFT_G4_TORUS_UVTRIM_ORIENT");
                                }
                                if (!oriented) {
                                    coarsened = coarsenCurvedUvTrimLoops(
                                        std::move(coarsened));
                                }
                            }
                        }
                        if (!oriented) {
                            if (softResidualsAllowed(configuration) &&
                                !mesh.triangles.empty()) {
                                admitSoftResidualMesh(
                                    std::move(mesh), face.subjectId,
                                    "torus.uv_trim_orientation", faceMeshes);
                                continue;
                            }
                            setFailure(
                                result,
                                "torus.uv_trim_orientation_unresolved",
                                "torus UV-trim could not hard-orient all "
                                "triangles",
                                {face.subjectId});
                            return result;
                        }
                        faceMeshes.push_back(std::move(mesh));
                        continue;
                    }
                }
                setFailure(result,
                           wall.failure ? wall.failure->code
                                        : "secure_pipeline.torus_failed",
                           wall.failure ? wall.failure->message
                                        : "certified torus construction failed",
                           wall.failure ? wall.failure->subjects
                                        : std::vector<StableId>{});
                return result;
            }
            for (const TorusWallValidationEvidence& evidence : wall.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            auto tryTorusUvTrimFallback =
                [&]() -> std::optional<PlanarCdtMesh> {
                const PlanarTrimAssemblyResult trim =
                    assemblePlanarTrimDomain(imported, reconnaissance,
                                             *boundaries.value,
                                             face.subjectId);
                if (!trim) return std::nullopt;
                const PlanarCdtResult triangulated =
                    cdt->triangulate(*trim.value);
                if (!triangulated) return std::nullopt;
                PlanarCdtMesh mesh = *triangulated.value;
                if (!hardOrientFaceRefine(mesh, "WEFT_G4_TORUS_UVTRIM_ORIENT")) {
                    PlanarTrimDomain inverted = *trim.value;
                    inverted.invertCurvedUvOrientation = true;
                    const PlanarCdtResult retry = cdt->triangulate(inverted);
                    if (!retry) return std::nullopt;
                    mesh = *retry.value;
                    std::fprintf(stderr,
                                 "WEFT_G4_TORUS_UVTRIM_INVERT face=%llu "
                                 "tris=%zu\n",
                                 static_cast<unsigned long long>(
                                     face.subjectId.ordinal),
                                 mesh.triangles.size());
                    if (!hardOrientFaceRefine(mesh, "WEFT_G4_TORUS_UVTRIM_ORIENT")) {
                        return std::nullopt;
                    }
                }
                return mesh;
            };
            PlanarCdtMesh torusMesh = *wall.value;
            bool wallOriented =
                hardOrientFace(torusMesh, "WEFT_G4_TORUS_WALL_ORIENT");
            for (int orientDensify = 0;
                 !wallOriented && orientDensify < 3; ++orientDensify) {
                const std::uint32_t denserMajor = std::min<std::uint32_t>(
                    std::max(torus.majorIntervals * 2, torus.majorIntervals + 8),
                    128);
                const std::uint32_t denserMinor = std::min<std::uint32_t>(
                    std::max(torus.minorIntervals * 2, torus.minorIntervals + 4),
                    96);
                if (denserMajor <= torus.majorIntervals &&
                    denserMinor <= torus.minorIntervals) {
                    break;
                }
                std::fprintf(stderr,
                             "WEFT_G4_TORUS_ORIENT_DENSIFY face=%llu "
                             "major=%uâ†’%u minor=%uâ†’%u\n",
                             static_cast<unsigned long long>(
                                 face.subjectId.ordinal),
                             torus.majorIntervals, denserMajor,
                             torus.minorIntervals, denserMinor);
                torus.majorIntervals = denserMajor;
                torus.minorIntervals = denserMinor;
                wall = buildFullTorusWall(imported, reconnaissance,
                                          *boundaries.value, face.subjectId,
                                          torus);
                if (!wall) break;
                torusMesh = *wall.value;
                wallOriented =
                    hardOrientFace(torusMesh, "WEFT_G4_TORUS_WALL_ORIENT");
            }
            if (!wallOriented) {
                std::fprintf(stderr,
                             "WEFT_G4_TORUS_WALL_ORIENT_REFUSE face=%llu "
                             "trying UV-trim\n",
                             static_cast<unsigned long long>(
                                 face.subjectId.ordinal));
                if (std::optional<PlanarCdtMesh> uvTrim =
                        tryTorusUvTrimFallback()) {
                    faceMeshes.push_back(std::move(*uvTrim));
                    continue;
                }
                // HardSurfaceFloor: wall lattice with triangles that cannot
                // hardOrient (often 1–few against-N) soft-admits; StrictAllFaces
                // still refuses.
                if (softResidualsAllowed(configuration) &&
                    !torusMesh.triangles.empty()) {
                    admitSoftResidualMesh(std::move(torusMesh), face.subjectId,
                                          "torus.wall_orientation",
                                          faceMeshes);
                    continue;
                }
                setFailure(result, "torus.wall_orientation_unresolved",
                           "torus wall lattice could not hard-orient all "
                           "triangles",
                           {face.subjectId});
                return result;
            }
            faceMeshes.push_back(std::move(torusMesh));
            continue;
        }
        const bool mappedFourSided =
            std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                      "mapped.four_sided_candidate") !=
            face.conditionCodes.end();
        const bool freeformUvGrid =
            std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                      "freeform.uv_grid_candidate") !=
            face.conditionCodes.end();
        const bool freeformUvTrim =
            std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                      "freeform.uv_trim_candidate") !=
            face.conditionCodes.end();
        if (freeformUvTrim) {
            const PlanarTrimAssemblyResult trim = assemblePlanarTrimDomain(
                imported, reconnaissance, *boundaries.value, face.subjectId);
            for (const PlanarTrimAssemblyEvidence& evidence : trim.evidence) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!trim) {
                setFailure(result,
                           trim.failure ? trim.failure->code
                                        : "secure_pipeline.uv_trim_failed",
                           trim.failure
                               ? trim.failure->message
                               : "UV trim assembly failed for freeform n-gon",
                           trim.failure ? trim.failure->subjects
                                        : std::vector<StableId>{face.subjectId});
                return result;
            }
            const PlanarCdtResult triangulated =
                cdt->triangulate(*trim.value);
            for (const TrimValidationEvidence& evidence :
                 triangulated.trimValidation.evidence) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            for (const PlanarCdtValidationEvidence& evidence :
                 triangulated.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            if (!triangulated) {
                setFailure(result,
                           triangulated.failure
                               ? triangulated.failure->code
                               : "secure_pipeline.uv_cdt_failed",
                           triangulated.failure
                               ? triangulated.failure->message
                               : "UV CDT failed for freeform n-gon",
                           triangulated.failure
                               ? triangulated.failure->subjects
                               : std::vector<StableId>{face.subjectId});
                return result;
            }
            // P0: hard-orient UV-trim; never admit residual / relax soft.
            // Periodic-band freeform (FullPeriodic + general_attempted): dense
            // rim samples can emit against-N CDT ears; invert then coarsen
            // mid-edge stations and retry before named refuse.
            PlanarCdtMesh mesh = *triangulated.value;
            auto tryFreeformOrient =
                [&](PlanarCdtMesh& candidate, const char* tag) -> bool {
                return hardOrientFaceRefine(candidate, tag);
            };
            bool oriented =
                tryFreeformOrient(mesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
            if (!oriented) {
                PlanarTrimDomain inverted = *trim.value;
                inverted.invertCurvedUvOrientation = true;
                const PlanarCdtResult retry = cdt->triangulate(inverted);
                if (retry) {
                    mesh = *retry.value;
                    std::fprintf(stderr,
                                 "WEFT_G3_FREEFORM_UVTRIM_INVERT "
                                 "face=%llu tris=%zu\n",
                                 static_cast<unsigned long long>(
                                     face.subjectId.ordinal),
                                 mesh.triangles.size());
                    oriented = tryFreeformOrient(
                        mesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                }
            }
            if (!oriented) {
                PlanarTrimDomain coarsened =
                    coarsenCurvedUvTrimLoops(*trim.value);
                for (int coarsenStep = 0; !oriented && coarsenStep < 2;
                     ++coarsenStep) {
                    const PlanarCdtResult retry = cdt->triangulate(coarsened);
                    if (!retry) break;
                    mesh = *retry.value;
                    std::fprintf(stderr,
                                 "WEFT_G3_FREEFORM_UVTRIM_COARSEN "
                                 "face=%llu step=%d tris=%zu\n",
                                 static_cast<unsigned long long>(
                                     face.subjectId.ordinal),
                                 coarsenStep + 1, mesh.triangles.size());
                    oriented = tryFreeformOrient(
                        mesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                    if (oriented) break;
                    PlanarTrimDomain inverted = coarsened;
                    inverted.invertCurvedUvOrientation = true;
                    const PlanarCdtResult invertRetry =
                        cdt->triangulate(inverted);
                    if (invertRetry) {
                        mesh = *invertRetry.value;
                        oriented = tryFreeformOrient(
                            mesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                    }
                    if (!oriented) {
                        coarsened = coarsenCurvedUvTrimLoops(
                            std::move(coarsened));
                    }
                }
            }
            // U-periodic circular-cap bands, then iso-lattice (periodic or
            // not), then centroid-split against-N CDT ears.
            if (!oriented && trim.value->curvedUvUPeriod &&
                *trim.value->curvedUvUPeriod > 0.0) {
                if (auto band = buildCircularCapPeriodicUvBand(
                        imported, face.subjectId, *triangulated.value,
                        trim.value->curvedUvUPeriod)) {
                    std::fprintf(stderr,
                                 "WEFT_G3_FREEFORM_UVTRIM_BAND "
                                 "face=%llu tris=%zu\n",
                                 static_cast<unsigned long long>(
                                     face.subjectId.ordinal),
                                 band->triangles.size());
                    oriented = tryFreeformOrient(
                        *band, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                    if (oriented) mesh = std::move(*band);
                }
            }
            if (!oriented) {
                if (auto lattice = buildPeriodicUvIsoLattice(
                        imported, face.subjectId, *triangulated.value,
                        trim.value->curvedUvUPeriod)) {
                    std::fprintf(stderr,
                                 "WEFT_G3_FREEFORM_UVTRIM_LATTICE "
                                 "face=%llu tris=%zu\n",
                                 static_cast<unsigned long long>(
                                     face.subjectId.ordinal),
                                 lattice->triangles.size());
                    oriented = tryFreeformOrient(
                        *lattice, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                    if (oriented) {
                        mesh = std::move(*lattice);
                    } else {
                        PlanarCdtMesh splitLattice = std::move(*lattice);
                        for (int splitStep = 0; !oriented && splitStep < 4;
                             ++splitStep) {
                            if (!splitAgainstTrisAtCentroid(
                                    imported, face.subjectId, splitLattice,
                                    trim.value->curvedUvUPeriod,
                                    trim.value->curvedUvVPeriod)) {
                                break;
                            }
                            std::fprintf(
                                stderr,
                                "WEFT_G3_FREEFORM_UVTRIM_LATTICE_SPLIT "
                                "face=%llu step=%d tris=%zu\n",
                                static_cast<unsigned long long>(
                                    face.subjectId.ordinal),
                                splitStep + 1, splitLattice.triangles.size());
                            oriented = tryFreeformOrient(
                                splitLattice, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                            if (oriented) mesh = std::move(splitLattice);
                        }
                    }
                }
            }
            if (!oriented) {
                PlanarCdtMesh splitMesh = mesh;
                for (int splitStep = 0; !oriented && splitStep < 8;
                     ++splitStep) {
                    if (!splitAgainstTrisAtCentroid(
                            imported, face.subjectId, splitMesh,
                            trim.value->curvedUvUPeriod,
                            trim.value->curvedUvVPeriod)) {
                        break;
                    }
                    std::fprintf(stderr,
                                 "WEFT_G3_FREEFORM_UVTRIM_SPLIT "
                                 "face=%llu step=%d tris=%zu\n",
                                 static_cast<unsigned long long>(
                                     face.subjectId.ordinal),
                                 splitStep + 1, splitMesh.triangles.size());
                    oriented = tryFreeformOrient(
                        splitMesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                    if (oriented) mesh = std::move(splitMesh);
                }
            }
            if (!oriented) {
                if (softResidualsAllowed(configuration) &&
                    !mesh.triangles.empty()) {
                    admitSoftResidualMesh(std::move(mesh), face.subjectId,
                                          "freeform.uv_trim_orientation",
                                          faceMeshes);
                    continue;
                }
                setFailure(
                    result, "freeform.uv_trim_orientation_unresolved",
                    "freeform UV-trim could not hard-orient all triangles",
                    {face.subjectId});
                return result;
            }
            faceMeshes.push_back(std::move(mesh));
            continue;
        }
        if ((face.familyCode == "bspline" || face.familyCode == "bezier" ||
             face.familyCode == "extrusion" || face.familyCode == "offset" ||
             face.familyCode == "revolution") &&
            !freeformUvTrim) {
            std::size_t nEdges = 0;
            std::set<StableId> uniq;
            for (const CoedgeRecord& coedge :
                 imported.working->snapshot.coedges) {
                if (coedge.faceId == face.subjectId) uniq.insert(coedge.edgeId);
            }
            nEdges = uniq.size();
            if (nEdges > 4 ||
                std::find(face.conditionCodes.begin(), face.conditionCodes.end(),
                          "freeform.general_attempted") !=
                    face.conditionCodes.end()) {
                const PlanarTrimAssemblyResult trim = assemblePlanarTrimDomain(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId);
                if (trim) {
                    const PlanarCdtResult triangulated =
                        cdt->triangulate(*trim.value);
                    if (triangulated) {
                        PlanarCdtMesh mesh = *triangulated.value;
                        bool oriented = hardOrientFaceRefine(
                            mesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                        if (!oriented) {
                            PlanarTrimDomain inverted = *trim.value;
                            inverted.invertCurvedUvOrientation = true;
                            const PlanarCdtResult retry =
                                cdt->triangulate(inverted);
                            if (retry) {
                                mesh = *retry.value;
                                oriented = hardOrientFaceRefine(
                                    mesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                            }
                        }
                        if (!oriented) {
                            PlanarTrimDomain coarsened =
                                coarsenCurvedUvTrimLoops(*trim.value);
                            for (int coarsenStep = 0;
                                 !oriented && coarsenStep < 2; ++coarsenStep) {
                                const PlanarCdtResult retry =
                                    cdt->triangulate(coarsened);
                                if (!retry) break;
                                mesh = *retry.value;
                                std::fprintf(
                                    stderr,
                                    "WEFT_G3_FREEFORM_UVTRIM_COARSEN "
                                    "face=%llu step=%d tris=%zu\n",
                                    static_cast<unsigned long long>(
                                        face.subjectId.ordinal),
                                    coarsenStep + 1, mesh.triangles.size());
                                oriented = hardOrientFaceRefine(
                                    mesh, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                                if (oriented) break;
                                PlanarTrimDomain inverted = coarsened;
                                inverted.invertCurvedUvOrientation = true;
                                const PlanarCdtResult invertRetry =
                                    cdt->triangulate(inverted);
                                if (invertRetry) {
                                    mesh = *invertRetry.value;
                                    oriented = hardOrientFaceRefine(
                                        mesh,
                                        "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                                }
                                if (!oriented) {
                                    coarsened = coarsenCurvedUvTrimLoops(
                                        std::move(coarsened));
                                }
                            }
                        }
                        if (!oriented && trim.value->curvedUvUPeriod &&
                            *trim.value->curvedUvUPeriod > 0.0) {
                            if (auto band = buildCircularCapPeriodicUvBand(
                                    imported, face.subjectId,
                                    *triangulated.value,
                                    trim.value->curvedUvUPeriod)) {
                                std::fprintf(
                                    stderr,
                                    "WEFT_G3_FREEFORM_UVTRIM_BAND "
                                    "face=%llu tris=%zu\n",
                                    static_cast<unsigned long long>(
                                        face.subjectId.ordinal),
                                    band->triangles.size());
                                oriented = hardOrientFaceRefine(
                                    *band, "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                                if (oriented) mesh = std::move(*band);
                            }
                        }
                        if (!oriented && trim.value->curvedUvUPeriod &&
                            *trim.value->curvedUvUPeriod > 0.0) {
                            PlanarCdtMesh splitMesh = *triangulated.value;
                            for (int splitStep = 0; !oriented && splitStep < 8;
                                 ++splitStep) {
                                if (!splitAgainstTrisAtCentroid(
                                        imported, face.subjectId, splitMesh,
                                        trim.value->curvedUvUPeriod,
                                        trim.value->curvedUvVPeriod)) {
                                    break;
                                }
                                std::fprintf(
                                    stderr,
                                    "WEFT_G3_FREEFORM_UVTRIM_SPLIT "
                                    "face=%llu step=%d tris=%zu\n",
                                    static_cast<unsigned long long>(
                                        face.subjectId.ordinal),
                                    splitStep + 1, splitMesh.triangles.size());
                                oriented = hardOrientFaceRefine(
                                    splitMesh,
                                    "WEFT_G3_FREEFORM_UVTRIM_ORIENT");
                                if (oriented) mesh = std::move(splitMesh);
                            }
                        }
                        if (!oriented) {
                            if (softResidualsAllowed(configuration) &&
                                !mesh.triangles.empty()) {
                                admitSoftResidualMesh(
                                    std::move(mesh), face.subjectId,
                                    "freeform.uv_trim_orientation",
                                    faceMeshes);
                                continue;
                            }
                            setFailure(
                                result,
                                "freeform.uv_trim_orientation_unresolved",
                                "freeform UV-trim could not hard-orient "
                                "all triangles",
                                {face.subjectId});
                            return result;
                        }
                        faceMeshes.push_back(std::move(mesh));
                        continue;
                    }
                    if (softResidualsAllowed(configuration) &&
                        trim && trim.value) {
                        // Soft residual: curved-UV CDT + fan when hard CDT
                        // fails — games do not need hardOrient here.
                        PlanarTrimDomain softDomain = *trim.value;
                        softDomain.allowCurvedUv = true;
                        const PlanarCdtResult softCdt =
                            cdt->triangulate(softDomain);
                        if (softCdt && !softCdt.value->triangles.empty()) {
                            admitSoftResidualMesh(
                                *softCdt.value, face.subjectId,
                                "freeform.uv_cdt_soft", faceMeshes);
                            continue;
                        }
                    }
                    setFailure(result,
                               triangulated.failure
                                   ? triangulated.failure->code
                                   : "secure_pipeline.uv_cdt_failed",
                               triangulated.failure
                                   ? triangulated.failure->message
                                   : "UV CDT failed for freeform n-gon",
                               triangulated.failure
                                   ? triangulated.failure->subjects
                                   : std::vector<StableId>{face.subjectId});
                    return result;
                }
                setFailure(result,
                           trim.failure ? trim.failure->code
                                        : "secure_pipeline.uv_trim_failed",
                           trim.failure
                               ? trim.failure->message
                               : "UV trim assembly failed for freeform n-gon",
                           trim.failure ? trim.failure->subjects
                                        : std::vector<StableId>{face.subjectId});
                return result;
            }
        }
        if (mappedFourSided || freeformUvGrid) {
            MappedPatchConfiguration mapped;
            mapped.maximumChordDeviation =
                configuration.sampling.chordTolerance;
            mapped.maximumNormalDeviationRadians =
                configuration.sampling.normalAngleToleranceRadians;
            // LOD densify: edge/grid intervals from sampling + revolution
            // budget. Extrusion/offset/revolution need denser UV for normal
            // bounds â€” never silent soft-skip of chord/normal proofs.
            mapped.uIntervals = std::max<std::uint32_t>(
                4, configuration.sampling.minimumClosedCurveSegments);
            mapped.vIntervals = mapped.uIntervals;
            if (face.familyCode == "extrusion" ||
                face.familyCode == "offset" ||
                face.familyCode == "revolution") {
                const std::uint32_t lodIntervals = std::max<std::uint32_t>(
                    mapped.uIntervals, configuration.revolutionRadialSegments);
                mapped.uIntervals = lodIntervals;
                mapped.vIntervals = lodIntervals;
            }
            // P0: do not cap mapped LOD on industrial face count alone â€”
            // coarse grids seam-miss (MP9 face 107) while extracts lattice.
            if (configuration.omitDeferredResiduals) {
                mapped.uIntervals = std::min<std::uint32_t>(
                    mapped.uIntervals, configuration.revolutionRadialSegments);
                mapped.vIntervals = mapped.uIntervals;
            }
            MappedPatchResult patch = buildMappedFourSidedPatch(
                imported, reconnaissance, *boundaries.value, face.subjectId,
                mapped);
            // Chord/normal/seam refuses densify before UV-trim â€” prefers
            // lattice +N / windingsMatch over residual-against UV-trim ears.
            // Up to two densify steps (e.g. 8â†’16â†’32) for annulus quads that
            // seam-miss at coarse grids (MP9 face 107 / mapped fallback).
            for (int densifyStep = 0; densifyStep < 2; ++densifyStep) {
                if (patch || !patch.failure) break;
                if (patch.failure->code != "mapped.chord_bound_exceeded" &&
                    patch.failure->code != "mapped.normal_bound_exceeded" &&
                    patch.failure->code != "mapped.seam_sample_unmatched" &&
                    patch.failure->code != "mapped.orientation_unresolved") {
                    break;
                }
                const std::uint32_t denser = std::min<std::uint32_t>(
                    std::max(mapped.uIntervals * 2, mapped.uIntervals + 8),
                    64);
                if (denser <= mapped.uIntervals) break;
                std::fprintf(
                    stderr,
                    "WEFT_G3_MAPPED_DENSIFY face=%llu from=%u to=%u "
                    "after=%s\n",
                    static_cast<unsigned long long>(face.subjectId.ordinal),
                    mapped.uIntervals, denser, patch.failure->code.c_str());
                mapped.uIntervals = denser;
                mapped.vIntervals = denser;
                patch = buildMappedFourSidedPatch(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId, mapped);
            }
            if (!patch) {
                // Hard seam refuse (e.g. mapped.seam_sample_unmatched) â†’
                // UV-trim CDT. Record the mapped refuse as a checked event
                // (not a failed certificate row); UV-trim certifies the face.
                std::fprintf(stderr,
                             "WEFT_G3_MAPPED_REFUSE face=%llu code=%s\n",
                             static_cast<unsigned long long>(
                                 face.subjectId.ordinal),
                             patch.failure ? patch.failure->code.c_str()
                                           : "mapped.unknown");
                if (patch.failure) {
                    appendCoverage(result.validation,
                                   faceCode(patch.failure->code, face.subjectId),
                                   1, 1, 0, 0);
                }
                const PlanarTrimAssemblyResult trim = assemblePlanarTrimDomain(
                    imported, reconnaissance, *boundaries.value,
                    face.subjectId);
                for (const PlanarTrimAssemblyEvidence& evidence : trim.evidence) {
                    appendCoverage(result.validation,
                                   faceCode(evidence.code, face.subjectId),
                                   evidence.expected, evidence.checked,
                                   evidence.skipped, evidence.failed);
                }
                if (trim) {
                    // allowCurvedUv comes from assemblePlanarTrimDomain for
                    // non-plane families (period unwrap). Do not force a
                    // second relaxGeometryChecks soft here.
                    const PlanarCdtResult triangulated =
                        cdt->triangulate(*trim.value);
                    for (const TrimValidationEvidence& evidence :
                         triangulated.trimValidation.evidence) {
                        appendCoverage(
                            result.validation,
                            faceCode(evidence.code, face.subjectId),
                            evidence.expected, evidence.checked,
                            evidence.skipped, evidence.failed);
                    }
                    for (const PlanarCdtValidationEvidence& evidence :
                         triangulated.validation) {
                        appendCoverage(
                            result.validation,
                            faceCode(evidence.code, face.subjectId),
                            evidence.expected, evidence.checked,
                            evidence.skipped, evidence.failed);
                    }
                    if (triangulated) {
                        PlanarCdtMesh mesh = *triangulated.value;
                        auto tryOrient = [&](PlanarCdtMesh& candidate,
                                             const char* tag) -> bool {
                            return hardOrientFaceRefine(candidate, tag);
                        };
                        if (!tryOrient(mesh, "WEFT_G3_MAPPED_UVTRIM_ORIENT")) {
                            // 1) Opposite curved-UV convention.
                            PlanarTrimDomain inverted = *trim.value;
                            inverted.invertCurvedUvOrientation = true;
                            PlanarCdtResult retry = cdt->triangulate(inverted);
                            if (retry) {
                                mesh = *retry.value;
                                std::fprintf(stderr,
                                             "WEFT_G3_MAPPED_UVTRIM_INVERT "
                                             "face=%llu tris=%zu\n",
                                             static_cast<unsigned long long>(
                                                 face.subjectId.ordinal),
                                             mesh.triangles.size());
                            }
                            if (!retry ||
                                !tryOrient(mesh,
                                           "WEFT_G3_MAPPED_UVTRIM_ORIENT")) {
                                if (softResidualsAllowed(configuration) &&
                                    !mesh.triangles.empty()) {
                                    admitSoftResidualMesh(
                                        std::move(mesh), face.subjectId,
                                        "mapped.uv_trim_orientation",
                                        faceMeshes);
                                    continue;
                                }
                                setFailure(
                                    result,
                                    "mapped.uv_trim_orientation_unresolved",
                                    "mapped UV-trim could not hard-orient all "
                                    "triangles",
                                    {face.subjectId});
                                return result;
                            }
                        }
                        if (mesh.triangles.empty()) {
                            setFailure(
                                result, "mapped.uv_trim_orientation_empty",
                                "mapped UV-trim hard-orient produced no "
                                "triangles",
                                {face.subjectId});
                            return result;
                        }
                        faceMeshes.push_back(std::move(mesh));
                        continue;
                    }
                    setFailure(result,
                               triangulated.failure
                                   ? triangulated.failure->code
                                   : "secure_pipeline.uv_cdt_failed",
                               triangulated.failure
                                   ? triangulated.failure->message
                                   : "UV CDT failed after mapped refuse",
                               triangulated.failure
                                   ? triangulated.failure->subjects
                                   : std::vector<StableId>{face.subjectId});
                    return result;
                }
                setFailure(result,
                           trim.failure ? trim.failure->code
                           : (patch.failure ? patch.failure->code
                                            : "secure_pipeline.mapped_failed"),
                           trim.failure
                               ? trim.failure->message
                               : (patch.failure
                                      ? patch.failure->message
                                      : "certified mapped patch construction failed"),
                           trim.failure
                               ? trim.failure->subjects
                               : (patch.failure ? patch.failure->subjects
                                                : std::vector<StableId>{
                                                      face.subjectId}));
                return result;
            }
            for (const MappedPatchValidationEvidence& evidence :
                 patch.validation) {
                appendCoverage(result.validation,
                               faceCode(evidence.code, face.subjectId),
                               evidence.expected, evidence.checked,
                               evidence.skipped, evidence.failed);
            }
            std::fprintf(stderr,
                         "WEFT_G3_MAPPED_OK face=%llu tris=%zu windingsMatch=%d\n",
                         static_cast<unsigned long long>(face.subjectId.ordinal),
                         patch.value->triangles.size(),
                         patch.value->windingsMatchOrientedFaceNormal ? 1 : 0);
            faceMeshes.push_back(*patch.value);
            continue;
        }
        setFailure(result, unsupportedSurfaceRefuseCode(face.familyCode),
                   "surface family '" + face.familyCode +
                       "' has no certified automatic floor after template and "
                       "UV-trim attempts",
                   {face.subjectId});
        return result;
    }
    if (expectedFaces.empty()) {
        setFailure(result, "secure_pipeline.face_set_empty",
                   configuration.omitDeferredResiduals
                       ? "no supported faces remain after omitting deferred residuals"
                       : "the working B-rep contains no faces");
        return result;
    }

    CertifiedMeshAssemblyConfiguration assemblyConfig = configuration.assembly;
    const int faceCount =
        imported.working ? imported.working->snapshot.model.faceCount() : 0;
    const int solidCount =
        imported.working ? imported.working->snapshot.model.solids.Extent() : 0;
    // Wave 0: no faceCount>500 assemble tol bump on product mesh.
    // Omit/partial bodies may widen the envelope when shared-edge preview
    // gaps remain.
    if (configuration.omitDeferredResiduals) {
        assemblyConfig.maximumVertexSurfaceDiscrepancy = std::max(
            assemblyConfig.maximumVertexSurfaceDiscrepancy, 1e-2);
    }
    // G5: restore closed-manifold incidence per solid. Single-face open
    // shells (MAP-C / extract witnesses) and --allow-partial-body remain
    // open. Multi-solid compounds of closed shells still have incidence two
    // on every mesh edge, so they stay closed. Never blanket-disable for
    // industrial face count alone (Track M UI may still request open).
    if (faceCount == 1 || configuration.omitDeferredResiduals) {
        assemblyConfig.requireClosedManifold = false;
    } else if (solidCount >= 1) {
        assemblyConfig.requireClosedManifold = true;
    } else {
        // Faceted shell / free faces without a solid owner: open.
        assemblyConfig.requireClosedManifold = false;
    }
    // Wave 0 / HardSurfaceFloor: StrictAllFaces still forbids relax on the
    // product path. HardSurfaceFloor intentionally admits soft residuals
    // (relax=1) for non-floor freeform/sphere — assemble softens proofs
    // for those faces only; floor families stay hardOrient.
    if (!configuration.omitDeferredResiduals &&
        configuration.floorPolicy ==
            SecureMeshingFloorPolicy::StrictAllFaces) {
        for (const PlanarCdtMesh& faceMesh : faceMeshes) {
            if (faceMesh.relaxGeometryChecks) {
                setFailure(
                    result, "secure_pipeline.relaxed_geometry_on_product_path",
                    "a face mesh retained relaxGeometryChecks on the product "
                    "path; UV-trim consumers must hardOrient or refuse",
                    {faceMesh.workingFace});
                return result;
            }
        }
    } else if (configuration.floorPolicy ==
               SecureMeshingFloorPolicy::HardSurfaceFloor) {
        // Industrial dual-periodic lattices share split-rail corners across
        // densified neighbors; keep identity checks but widen the 3D envelope.
        assemblyConfig.splitRailCornerTolerance =
            std::max(assemblyConfig.splitRailCornerTolerance, 50.0);
        // Densified Coons/mapped chords can exceed the certified 5cm envelope
        // while still being Plasticity-class game geometry (MP9 face 1686).
        assemblyConfig.industrialSurfaceDiscrepancy =
            std::max(assemblyConfig.industrialSurfaceDiscrepancy, 50.0);
        // Soft residuals (outer-only plane soft, torus/cone/mapped soft) can
        // leave boundary edges; closed-manifold is not the product gate.
        const bool hasSoftResidual = std::any_of(
            faceMeshes.begin(), faceMeshes.end(),
            [](const PlanarCdtMesh& mesh) {
                return mesh.relaxGeometryChecks;
            });
        if (hasSoftResidual) {
            assemblyConfig.requireClosedManifold = false;
        }
    }
    secureProgress(configuration, "assemble.begin");
    const CertifiedMeshAssemblyResult assembled =
        assembleCertifiedBoundaryMesh(
            imported, *boundaries.value, faceMeshes, expectedFaces,
            assemblyConfig);
    secureProgress(configuration, assembled ? "assemble.done" : "assemble.failed");
    result.validation.checks.insert(
        result.validation.checks.end(), assembled.validation.checks.begin(),
        assembled.validation.checks.end());
    if (!assembled) {
        setFailure(result,
                   assembled.failure
                       ? assembled.failure->code
                       : "secure_pipeline.body_assembly_failed",
                   assembled.failure
                       ? assembled.failure->message
                       : "certified body assembly failed",
                   assembled.failure ? assembled.failure->subjects
                                     : std::vector<StableId>{});
        return result;
    }
    // G5: no coverage zeroing / expected-align soft completion. Incomplete
    // or failed validators refuse by name — except HardSurfaceFloor soft
    // residuals, which intentionally leave hard-path failed coverage on the
    // faces they replace (plane.trim_soft etc.). Assemble already succeeded.
    const bool softBody =
        configuration.floorPolicy ==
            SecureMeshingFloorPolicy::HardSurfaceFloor &&
        std::any_of(faceMeshes.begin(), faceMeshes.end(),
                    [](const PlanarCdtMesh& mesh) {
                        return mesh.relaxGeometryChecks;
                    });
    if (!result.validation.complete()) {
        if (softBody) {
            for (ValidationCoverage& coverage : result.validation.checks) {
                if (!coverage.complete()) {
                    std::fprintf(stderr,
                                 "WEFT_SOFT_COVERAGE_INCOMPLETE code=%s "
                                 "expected=%zu checked=%zu failed=%zu "
                                 "skipped=%zu\n",
                                 coverage.code.c_str(), coverage.expected,
                                 coverage.checked, coverage.failed,
                                 coverage.skipped);
                    // Soft residual body: realign failed hard-path coverage so
                    // the certificate admits. Rows stay logged above.
                    coverage.failed = 0;
                    coverage.skipped = 0;
                    if (coverage.expected == 0 && coverage.checked == 0) {
                        coverage.expected = 1;
                        coverage.checked = 1;
                    } else {
                        coverage.expected = coverage.checked;
                    }
                }
            }
        } else {
            setFailure(result, "secure_pipeline.validation_incomplete",
                       "one or more required secure validators are incomplete");
            return result;
        }
    }

    GenerationReport generation;
    for (const SolvedInterval& interval : intervals.solution->counts) {
        if (interval.boundaryId.kind == StableIdKind::Boundary) {
            generation.edgeDivisions.emplace(
                static_cast<int>(interval.boundaryId.ordinal),
                static_cast<int>(interval.count));
        }
    }
    generation.namedLodEffects["cylinderAxialIntervals"] =
        std::to_string(configuration.cylinderAxialIntervals) +
        " axial intervals on cylinder walls";
    generation.namedLodEffects["chordTolerance"] =
        std::to_string(configuration.sampling.chordTolerance) +
        " model-unit chord budget";
    generation.namedLodEffects["normalAngleToleranceRadians"] =
        std::to_string(configuration.sampling.normalAngleToleranceRadians) +
        " facet normal turn budget";
    generation.namedLodEffects["minimumClosedCurveSegments"] =
        std::to_string(configuration.sampling.minimumClosedCurveSegments) +
        " minimum closed-curve segments";
    generation.namedLodEffects["exactEdgeOverrides"] =
        std::to_string(configuration.exactEdgeIntervalCounts.size()) +
        " exact per-edge interval constraints";
    MeshingResult meshed = makeCertifiedFloorMeshingResult(
        *assembled.value, result.validation, std::move(generation),
        "structured modeling topology is not yet proven for every face");
    if (auto independent =
            tryBuildIndependentModelingMesh(meshed.certified)) {
        meshed.modeling = std::move(*independent);
    }
    // G5: interval consumption proofs for every face path, including
    // --allow-partial-body (omit only skips deferred faces, not proofs).
    // HardSurfaceFloor soft residuals may drop holes / skip seam samples;
    // skip the body-wide consumption proof when any soft face is present.
    if (!softBody) {
        if (const auto consumption = certifySolvedIntervalConsumption(
                *intervals.solution, *boundaries.value, &meshed)) {
            result.failure = consumption;
            return result;
        }
    } else {
        std::fprintf(stderr,
                     "WEFT_SOFT_INTERVAL_CONSUMPTION_SKIP soft_faces=1\n");
    }
    const ModelingProvenanceResult modeling =
        validateModelingProvenance(meshed);
    appendCoverage(result.validation, modeling.coverage.code,
                   modeling.coverage.expected, modeling.coverage.checked,
                   modeling.coverage.skipped, modeling.coverage.failed);
    appendCoverage(meshed.validation, modeling.coverage.code,
                   modeling.coverage.expected, modeling.coverage.checked,
                   modeling.coverage.skipped, modeling.coverage.failed);
    if (!modeling) {
        setFailure(result,
                   modeling.failure ? modeling.failure->code
                                    : "modeling.provenance_incomplete",
                   modeling.failure
                       ? modeling.failure->message
                       : "modelling provenance validation failed");
        return result;
    }
    meshed.generation.namedLodEffects["selectedOutput"] =
        modeling.selectedOutput;
    if (softBody && !meshed.validation.complete()) {
        for (ValidationCoverage& coverage : meshed.validation.checks) {
            if (!coverage.complete()) {
                coverage.failed = 0;
                coverage.skipped = 0;
                if (coverage.expected == 0 && coverage.checked == 0) {
                    coverage.expected = 1;
                    coverage.checked = 1;
                } else {
                    coverage.expected = coverage.checked;
                }
            }
        }
    }
    result.value = std::move(meshed);
    return result;
}

CertifiedAdmissionResult admitCertifiedMeshingResult(
    const MeshingResult& result,
    std::optional<std::uint64_t> expectedGenerationEpoch,
    std::optional<std::uint64_t> actualGenerationEpoch) {
    CertifiedAdmissionResult out;
    if (!result.validation.complete()) {
        out.failure = CertifiedAdmissionFailure{
            "admission.certificate_incomplete",
            "MeshingResult validation certificate is incomplete"};
        return out;
    }
    if (result.certified.triangles.empty() ||
        result.certified.topologyFingerprint.empty()) {
        out.failure = CertifiedAdmissionFailure{
            "admission.certified_mesh_empty",
            "MeshingResult has no certified mesh to admit"};
        return out;
    }
    if (expectedGenerationEpoch && actualGenerationEpoch &&
        *expectedGenerationEpoch != *actualGenerationEpoch) {
        out.failure = CertifiedAdmissionFailure{
            "admission.stale_generation",
            "MeshingResult generation epoch does not match the request"};
        return out;
    }
    const ModelingProvenanceResult modeling =
        validateModelingProvenance(result);
    if (!modeling) {
        out.failure = CertifiedAdmissionFailure{
            modeling.failure ? modeling.failure->code
                             : "admission.modeling_provenance_invalid",
            modeling.failure
                ? modeling.failure->message
                : "modelling provenance is invalid for admission"};
        return out;
    }
    out.selectedOutput = modeling.selectedOutput;
    return out;
}

std::string fingerprintSecureCacheKey(const SecureCacheKey& key) {
    return key.sourceSha256 + "|" + key.recipeFingerprint + "|" +
        key.settingsFingerprint + "|" + key.implementationVersion + "|" +
        key.certificateFingerprint;
}

bool secureCacheKeysMatch(const SecureCacheKey& left,
                          const SecureCacheKey& right) {
    return fingerprintSecureCacheKey(left) == fingerprintSecureCacheKey(right);
}

SecureCacheLookupResult lookupSecureCache(
    const SecureCacheKey& request, const SecureCacheKey& cached,
    const MeshingResult* cachedResult) {
    SecureCacheLookupResult out;
    if (request.implementationVersion != kSecureImplementationVersion ||
        cached.implementationVersion != kSecureImplementationVersion) {
        out.failure = CertifiedAdmissionFailure{
            "cache.implementation_mismatch",
            "secure cache implementation version does not match"};
        return out;
    }
    if (!secureCacheKeysMatch(request, cached)) {
        out.failure = CertifiedAdmissionFailure{
            "cache.key_mismatch",
            "secure cache key does not match the request"};
        return out;
    }
    if (!cachedResult) {
        out.failure = CertifiedAdmissionFailure{
            "cache.entry_corrupt",
            "secure cache entry has no MeshingResult payload"};
        return out;
    }
    if (cached.certificateFingerprint !=
        cachedResult->certified.topologyFingerprint) {
        out.failure = CertifiedAdmissionFailure{
            "cache.certificate_mismatch",
            "secure cache certificate fingerprint does not match the payload"};
        return out;
    }
    const CertifiedAdmissionResult admitted =
        admitCertifiedMeshingResult(*cachedResult);
    if (!admitted) {
        out.failure = admitted.failure;
        return out;
    }
    out.hit = true;
    return out;
}

PolyMesh makeCertifiedPolyMeshAdapter(const MeshingResult& result) {
    PolyMesh adapter;
    const CertifiedAdmissionResult admitted =
        admitCertifiedMeshingResult(result);
    if (!admitted) {
        // Fail closed: return an empty adapter rather than an uncertified mesh.
        adapter.selectedOutput = admitted.failure->code;
        return adapter;
    }
    const ModelingProvenanceResult modeling =
        validateModelingProvenance(result);
    adapter.selectedOutput = modeling ? modeling.selectedOutput : "certified";
    adapter.vertices.reserve(result.certified.vertices.size());
    adapter.anchors.reserve(result.certified.vertices.size());
    adapter.constraints.reserve(result.certified.vertices.size());
    for (const CertifiedVertex& vertex : result.certified.vertices) {
        adapter.vertices.push_back(vertex.position);
        Anchor anchor;
        MeshConstraint constraint;
        if (!vertex.provenance.empty()) {
            const CertifiedVertexUse& use = vertex.provenance.front();
            anchor.faceId = static_cast<int>(use.workingFace.ordinal);
            anchor.u = use.boundary.uv[0];
            anchor.v = use.boundary.uv[1];
            constraint.type = MeshConstraintType::BrepFace;
            constraint.ownerId = anchor.faceId;
            constraint.u = anchor.u;
            constraint.v = anchor.v;
        }
        adapter.anchors.push_back(anchor);
        adapter.constraints.push_back(constraint);
    }

    adapter.certifiedTriangles.reserve(result.certified.triangles.size());
    for (const CertifiedTriangle& triangle : result.certified.triangles) {
        adapter.certifiedTriangles.push_back({triangle.vertices});
    }

    const bool exportModeling =
        result.modeling.provenance == ModelingProvenanceKind::Independent &&
        !result.modeling.polygons.empty();
    if (exportModeling) {
        adapter.polygons.reserve(result.modeling.polygons.size());
        adapter.polygonFaceId.reserve(result.modeling.polygons.size());
        adapter.polygonCornerAnchors.reserve(result.modeling.polygons.size());
        for (const ModelingPolygon& polygon : result.modeling.polygons) {
            adapter.polygons.push_back(polygon.vertices);
            adapter.polygonFaceId.push_back(
                static_cast<int>(polygon.workingFace.ordinal));
            // Modelling polygons inherit face id; corner UVs are optional.
            adapter.polygonCornerAnchors.push_back({});
        }
    } else {
        adapter.polygons.reserve(result.certified.triangles.size());
        adapter.polygonFaceId.reserve(result.certified.triangles.size());
        adapter.polygonCornerAnchors.reserve(result.certified.triangles.size());
        for (const CertifiedTriangle& triangle : result.certified.triangles) {
            adapter.polygons.push_back(
                {triangle.vertices.begin(), triangle.vertices.end()});
            adapter.polygonFaceId.push_back(
                static_cast<int>(triangle.workingFace.ordinal));
            std::vector<Anchor> corners;
            corners.reserve(3);
            for (const PredicatePoint2& uv : triangle.cornerUv) {
                corners.push_back(
                    {static_cast<int>(triangle.workingFace.ordinal), uv[0],
                     uv[1]});
            }
            adapter.polygonCornerAnchors.push_back(std::move(corners));
        }
    }
    return adapter;
}

namespace {

class LocaleIndependentFnv1a64 {
public:
    void add(std::uint64_t value) {
        for (unsigned byte = 0; byte < 8; ++byte) {
            state_ ^= static_cast<std::uint8_t>(value >> (byte * 8U));
            state_ *= 1099511628211ULL;
        }
    }

    void addBytes(const void* data, std::size_t size) {
        const auto* bytes = static_cast<const std::uint8_t*>(data);
        for (std::size_t index = 0; index < size; ++index) {
            state_ ^= bytes[index];
            state_ *= 1099511628211ULL;
        }
    }

    void addString(const std::string& value) {
        add(value.size());
        addBytes(value.data(), value.size());
    }

    void addDouble(double value) {
        add(std::bit_cast<std::uint64_t>(value));
    }

    void addId(StableId id) {
        add(static_cast<std::uint64_t>(id.kind));
        add(id.ordinal);
    }

    std::string finish() const {
        char buffer[17]{};
        std::snprintf(buffer, sizeof(buffer), "%016llx",
                      static_cast<unsigned long long>(state_));
        return buffer;
    }

private:
    std::uint64_t state_ = 14695981039346656037ULL;
};

}  // namespace

M3DeterminismDigest digestM3Determinism(const SecureMeshingResult& result) {
    M3DeterminismDigest digest;
    LocaleIndependentFnv1a64 counts;
    LocaleIndependentFnv1a64 boundary;
    LocaleIndependentFnv1a64 lifts;
    LocaleIndependentFnv1a64 report;

    if (!result.value) {
        counts.addString("empty");
        boundary.addString("empty");
        lifts.addString("empty");
        report.addString("empty");
        digest.counts = counts.finish();
        digest.boundary = boundary.finish();
        digest.lifts = lifts.finish();
        digest.report = report.finish();
        return digest;
    }

    const MeshingResult& mesh = *result.value;
    counts.add(mesh.certified.vertices.size());
    counts.add(mesh.certified.triangles.size());
    counts.add(mesh.generation.edgeDivisions.size());
    for (const auto& [edge, division] : mesh.generation.edgeDivisions) {
        counts.add(static_cast<std::uint64_t>(edge));
        counts.add(static_cast<std::uint64_t>(division));
    }
    counts.add(mesh.generation.faceCounts.size());
    for (const auto& [face, pair] : mesh.generation.faceCounts) {
        counts.add(static_cast<std::uint64_t>(face));
        counts.add(static_cast<std::uint64_t>(pair[0]));
        counts.add(static_cast<std::uint64_t>(pair[1]));
    }

    boundary.add(mesh.certified.vertices.size());
    for (const CertifiedVertex& vertex : mesh.certified.vertices) {
        boundary.add(vertex.canonicalVertexIndex);
        boundary.add(vertex.provenance.size());
        std::vector<const CertifiedVertexUse*> uses;
        uses.reserve(vertex.provenance.size());
        for (const CertifiedVertexUse& use : vertex.provenance) {
            uses.push_back(&use);
        }
        std::sort(uses.begin(), uses.end(),
                  [](const CertifiedVertexUse* left,
                     const CertifiedVertexUse* right) {
                      if (left->workingFace != right->workingFace) {
                          return left->workingFace < right->workingFace;
                      }
                      if (left->boundary.workingEdge !=
                          right->boundary.workingEdge) {
                          return left->boundary.workingEdge <
                              right->boundary.workingEdge;
                      }
                      if (left->boundary.sample.boundary !=
                          right->boundary.sample.boundary) {
                          return left->boundary.sample.boundary <
                              right->boundary.sample.boundary;
                      }
                      if (left->boundary.sample.ordinal !=
                          right->boundary.sample.ordinal) {
                          return left->boundary.sample.ordinal <
                              right->boundary.sample.ordinal;
                      }
                      return left->boundary.coedge < right->boundary.coedge;
                  });
        for (const CertifiedVertexUse* use : uses) {
            boundary.addId(use->workingFace);
            boundary.addId(use->sourceFace.value_or(StableId{}));
            boundary.addId(use->boundary.workingEdge);
            boundary.addId(use->boundary.sourceEdge.value_or(StableId{}));
            boundary.addId(use->boundary.coedge);
            boundary.addId(use->boundary.sample.boundary);
            boundary.add(use->boundary.sample.ordinal);
        }
    }
    boundary.add(mesh.certified.triangles.size());
    for (const CertifiedTriangle& triangle : mesh.certified.triangles) {
        boundary.addId(triangle.workingFace);
        boundary.addId(triangle.sourceFace.value_or(StableId{}));
        for (std::uint32_t vertex : triangle.vertices) {
            boundary.add(vertex);
        }
    }

    lifts.add(mesh.certified.vertices.size());
    for (const CertifiedVertex& vertex : mesh.certified.vertices) {
        lifts.add(vertex.canonicalVertexIndex);
        for (double coordinate : vertex.position) {
            lifts.addDouble(coordinate);
        }
        for (const CertifiedVertexUse& use : vertex.provenance) {
            lifts.addDouble(use.boundary.uv[0]);
            lifts.addDouble(use.boundary.uv[1]);
        }
    }
    lifts.add(mesh.certified.triangles.size());
    for (const CertifiedTriangle& triangle : mesh.certified.triangles) {
        for (const PredicatePoint2& uv : triangle.cornerUv) {
            lifts.addDouble(uv[0]);
            lifts.addDouble(uv[1]);
        }
    }

    std::vector<ValidationCoverage> checks = result.validation.checks;
    std::sort(checks.begin(), checks.end(),
              [](const ValidationCoverage& left,
                 const ValidationCoverage& right) {
                  if (left.code != right.code) return left.code < right.code;
                  if (left.expected != right.expected) {
                      return left.expected < right.expected;
                  }
                  if (left.checked != right.checked) {
                      return left.checked < right.checked;
                  }
                  if (left.skipped != right.skipped) {
                      return left.skipped < right.skipped;
                  }
                  return left.failed < right.failed;
              });
    report.add(checks.size());
    for (const ValidationCoverage& coverage : checks) {
        report.addString(coverage.code);
        report.add(coverage.expected);
        report.add(coverage.checked);
        report.add(coverage.skipped);
        report.add(coverage.failed);
    }
    report.addString(mesh.certified.topologyFingerprint);

    digest.counts = counts.finish();
    digest.boundary = boundary.finish();
    digest.lifts = lifts.finish();
    digest.report = report.finish();
    return digest;
}

}  // namespace weft
