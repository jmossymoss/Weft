#include "weft/planar_trim_assembly.hpp"

#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace weft {
namespace {

enum EvidenceIndex : std::size_t {
    FaceEvidence = 0,
    WireEvidence,
    CoedgeEvidence,
    SampleEvidence,
    JunctionEvidence,
    ValidationEvidence,
};

void setFailure(PlanarTrimAssemblyResult& result, std::string code,
                std::string message, std::vector<StableId> subjects) {
    if (!result.failure) {
        result.failure = PlanarTrimAssemblyFailure{
            std::move(code), std::move(message), std::move(subjects)};
    }
}

std::optional<StableId> uniqueSourceForWorking(
    const ImportedModel& imported, StableId working) {
    std::optional<StableId> source;
    for (const CorrespondenceRecord& record : imported.correspondence.records) {
        if (std::find(record.workingIds.begin(), record.workingIds.end(),
                      working) == record.workingIds.end()) {
            continue;
        }
        if (source) return std::nullopt;
        source = record.sourceId;
    }
    return source;
}

PredicateResult<PlanarTrimLoopOrientation> orientationOf(
    const PlanarTrimLoop& loop, const GeometricPredicates& predicates) {
    const auto extreme = std::min_element(
        loop.vertices.begin(), loop.vertices.end(),
        [](const PlanarTrimVertex& left, const PlanarTrimVertex& right) {
            if (left.uv[0] != right.uv[0]) return left.uv[0] < right.uv[0];
            return left.uv[1] < right.uv[1];
        });
    const std::size_t index =
        static_cast<std::size_t>(extreme - loop.vertices.begin());
    const PredicatePoint2 previous =
        loop.vertices[(index + loop.vertices.size() - 1) %
                      loop.vertices.size()]
            .uv;
    const PredicatePoint2 current = loop.vertices[index].uv;
    const PredicatePoint2 next =
        loop.vertices[(index + 1) % loop.vertices.size()].uv;
    const PredicateResult<ExactSign> turn =
        predicates.orient2d(previous, current, next);
    if (!turn) {
        return {std::nullopt, turn.failure};
    }
    if (*turn.value == ExactSign::Zero) {
        // Industrial UV loops can have collinear extremes; scan for any turn.
        for (std::size_t i = 0; i < loop.vertices.size(); ++i) {
            const PredicatePoint2 a =
                loop.vertices[(i + loop.vertices.size() - 1) %
                              loop.vertices.size()]
                    .uv;
            const PredicatePoint2 b = loop.vertices[i].uv;
            const PredicatePoint2 c =
                loop.vertices[(i + 1) % loop.vertices.size()].uv;
            const PredicateResult<ExactSign> alt = predicates.orient2d(a, b, c);
            if (alt && *alt.value != ExactSign::Zero) {
                PredicateResult<PlanarTrimLoopOrientation> result;
                result.value = *alt.value == ExactSign::Positive
                    ? PlanarTrimLoopOrientation::CounterClockwise
                    : PlanarTrimLoopOrientation::Clockwise;
                return result;
            }
        }
        PredicateResult<PlanarTrimLoopOrientation> result;
        result.value = PlanarTrimLoopOrientation::CounterClockwise;
        return result;
    }
    PredicateResult<PlanarTrimLoopOrientation> result;
    result.value = *turn.value == ExactSign::Positive
        ? PlanarTrimLoopOrientation::CounterClockwise
        : PlanarTrimLoopOrientation::Clockwise;
    return result;
}

bool sameUv(PredicatePoint2 first, PredicatePoint2 second) {
    return first[0] == second[0] && first[1] == second[1];
}

bool nearUv(PredicatePoint2 first, PredicatePoint2 second,
            std::optional<double> uPeriod) {
    constexpr double kEps = 1e-3;
    double du = std::abs(first[0] - second[0]);
    if (uPeriod && *uPeriod > 0.0 && du > 0.5 * *uPeriod) {
        du = std::abs(du - *uPeriod);
    }
    return du <= kEps && std::abs(first[1] - second[1]) <= kEps;
}

bool mergeVertexUses(PlanarTrimVertex& retained,
                     PlanarTrimVertex candidate,
                     PlanarTrimAssemblyResult& result, StableId face,
                     StableId wire, bool allowNearUv,
                     std::optional<double> uPeriod) {
    // Planes require exact UV identity. Curved UV trims may disagree by lift
    // noise / periodic seam wraps; when the canonical vertex identity matches,
    // prefer topology over UV equality (sphere poles/seams).
    // Planes: exact UV. Curved: near-UV or same canonical. Far-3D uses are
    // pruned after the loop so cylinder seam unwrap cannot create split-rails.
    const bool sameCanon =
        retained.canonicalVertexIndex == candidate.canonicalVertexIndex;
    const bool uvCompatible =
        sameUv(retained.uv, candidate.uv) ||
        (allowNearUv && nearUv(retained.uv, candidate.uv, uPeriod)) ||
        (allowNearUv && sameCanon);
    if (!uvCompatible) {
        setFailure(result, "trim_assembly.vertex_uv_mismatch",
                   "incident canonical boundary samples disagree exactly in lifted UV",
                   {face, wire});
        return false;
    }
    for (PlanarTrimBoundaryUse& use : candidate.boundaryUses) {
        const bool duplicate = std::any_of(
            retained.boundaryUses.begin(), retained.boundaryUses.end(),
            [&](const PlanarTrimBoundaryUse& existing) {
                return existing.sample == use.sample;
            });
        if (duplicate) {
            // Curved UV trims can revisit the same sample at a closed
            // parallel; skip the duplicate rather than fail closed.
            if (allowNearUv) continue;
            setFailure(result, "trim_assembly.duplicate_boundary_use",
                       "a canonical boundary sample occurs twice at one loop vertex",
                       {face, wire, use.workingEdge});
            return false;
        }
        retained.boundaryUses.push_back(std::move(use));
    }
    return true;
}

bool appendOrMerge(PlanarTrimLoop& loop, PlanarTrimVertex candidate,
                   PlanarTrimAssemblyResult& result, StableId face,
                   StableId wire, bool allowNearUv,
                   std::optional<double> uPeriod) {
    if (loop.vertices.empty() ||
        loop.vertices.back().canonicalVertexIndex !=
            candidate.canonicalVertexIndex) {
        loop.vertices.push_back(std::move(candidate));
        return true;
    }
    return mergeVertexUses(loop.vertices.back(), std::move(candidate), result,
                           face, wire, allowNearUv, uPeriod);
}

}  // namespace

PlanarTrimAssemblyResult assemblePlanarTrimDomain(
    const ImportedModel& imported,
    const ReconnaissanceReport& reconnaissance,
    const CanonicalBoundarySet& boundaries,
    StableId workingFace,
    std::shared_ptr<const GeometricPredicates> predicates) {
    PlanarTrimAssemblyResult result;
    result.evidence = {
        {"trim_assembly.face", 1},
        {"trim_assembly.wires", 0},
        {"trim_assembly.coedges", 0},
        {"trim_assembly.samples", 0},
        {"trim_assembly.junctions", 0},
        {"trim_assembly.validation", 1},
    };

    PlanarTrimAssemblyEvidence& faceEvidence = result.evidence[FaceEvidence];
    ++faceEvidence.checked;
    if (!imported.meshable() || !imported.working || !imported.source ||
        !imported.correspondence.complete) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.working_model_not_meshable",
                   "planar trim assembly requires a certified meshable working model",
                   {workingFace});
        return result;
    }
    if (!reconnaissance.complete || !boundaries.validation.complete()) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.prerequisite_incomplete",
                   "reconnaissance and canonical boundaries must be complete",
                   {workingFace});
        return result;
    }
    if (!predicates || !predicates->exactForFiniteDoubleInputs()) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.predicate_backend_not_exact",
                   "planar trim assembly requires exact finite-double predicates",
                   {workingFace});
        return result;
    }
    if (workingFace.kind != StableIdKind::Face || !workingFace.valid() ||
        workingFace.ordinal > static_cast<std::uint64_t>(
            imported.working->snapshot.model.faces.Extent())) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.face_not_found",
                   "the working face ID does not resolve", {workingFace});
        return result;
    }
    const ExactGeometryClassification* classification =
        reconnaissance.find(workingFace);
    // Plane faces and UV-parameterized surfaces (cylinder/cone/sphere/
    // freeform) may assemble a trim domain from canonical lifted UV.
    const bool uvSurface =
        classification &&
        classification->taxonomy == GeometryTaxonomy::Surface &&
        classification->support ==
            GeometrySupportState::SupportedAnalyticTemplate &&
        (classification->familyCode == "plane" ||
         classification->familyCode == "cylinder" ||
         classification->familyCode == "cone" ||
         classification->familyCode == "sphere" ||
         classification->familyCode == "bspline" ||
         classification->familyCode == "bezier" ||
         classification->familyCode == "extrusion" ||
         classification->familyCode == "offset" ||
         classification->familyCode == "torus");
    if (!uvSurface) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.face_not_uv_surface",
                   "only a proven supported UV surface may use trim assembly",
                   {workingFace});
        return result;
    }
    const std::optional<StableId> sourceFace =
        uniqueSourceForWorking(imported, workingFace);
    if (!sourceFace || sourceFace->kind != StableIdKind::Face) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.source_face_missing",
                   "the working face does not resolve to exactly one source face",
                   {workingFace});
        return result;
    }

    const BRepSnapshot& snapshot = imported.working->snapshot;
    std::map<StableId, std::vector<const CoedgeRecord*>> wireCoedges;
    for (const CoedgeRecord& coedge : snapshot.coedges) {
        if (coedge.faceId == workingFace) {
            wireCoedges[coedge.wireId].push_back(&coedge);
        }
    }
    if (wireCoedges.empty()) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.face_has_no_wires",
                   "the planar face has no coedge-owned wires", {workingFace});
        return result;
    }
    for (auto& [wire, coedges] : wireCoedges) {
        (void)wire;
        std::sort(coedges.begin(), coedges.end(),
                  [](const CoedgeRecord* left, const CoedgeRecord* right) {
                      return left->ordinalInWire < right->ordinalInWire;
                  });
    }

    result.evidence[WireEvidence].expected = wireCoedges.size();
    for (const auto& [wire, coedges] : wireCoedges) {
        (void)wire;
        result.evidence[CoedgeEvidence].expected += coedges.size();
        result.evidence[JunctionEvidence].expected += coedges.size();
        for (const CoedgeRecord* coedge : coedges) {
            const CanonicalBoundary* boundary =
                boundaries.find(coedge->edgeId);
            if (boundary) {
                result.evidence[SampleEvidence].expected +=
                    boundary->samples.size();
            }
        }
    }

    StableId outerWireId;
    try {
        const TopoDS_Face face = TopoDS::Face(
            snapshot.model.faces(static_cast<int>(workingFace.ordinal)));
        const TopoDS_Wire outer = BRepTools::OuterWire(face);
        std::vector<StableId> faceOccurrences;
        for (const TopologyOccurrence& occurrence :
             snapshot.topology.occurrences) {
            if (occurrence.id.kind != StableIdKind::Face) continue;
            const auto shape =
                snapshot.topology.exactShapes.find(occurrence.id);
            if (shape == snapshot.topology.exactShapes.end()) continue;
            if (snapshot.model.faces.FindIndex(shape->second) ==
                static_cast<int>(workingFace.ordinal)) {
                faceOccurrences.push_back(occurrence.id);
            }
        }
        if (faceOccurrences.size() != 1) {
            ++faceEvidence.failed;
            setFailure(
                result,
                faceOccurrences.size() > 1
                    ? "trim_assembly.face_alias_collapse"
                    : "trim_assembly.face_occurrence_missing",
                faceOccurrences.size() > 1
                    ? "multiple topology face occurrences collapse onto one "
                      "meshing face id"
                    : "the meshing face has no topology face occurrence",
                {workingFace});
            return result;
        }
        const auto faceOccurrence = std::find_if(
            snapshot.topology.occurrences.begin(),
            snapshot.topology.occurrences.end(),
            [&](const TopologyOccurrence& occurrence) {
                return occurrence.id == faceOccurrences.front();
            });
        if (faceOccurrence != snapshot.topology.occurrences.end() &&
            !outer.IsNull()) {
            for (StableId child : faceOccurrence->childIds) {
                if (child.kind != StableIdKind::Wire) continue;
                const auto wireShape =
                    snapshot.topology.exactShapes.find(child);
                if (wireShape != snapshot.topology.exactShapes.end() &&
                    wireShape->second.IsSame(outer)) {
                    outerWireId = child;
                    break;
                }
            }
        }
    } catch (const Standard_Failure&) {
        outerWireId = {};
    }
    if (!outerWireId.valid() || !wireCoedges.contains(outerWireId)) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.outer_wire_unresolved",
                   "topology did not resolve one outer wire occurrence for the "
                   "face",
                   {workingFace});
        return result;
    }

    PlanarTrimDomain domain;
    domain.face = workingFace;
    domain.sourceFace = sourceFace;
    domain.loops.reserve(wireCoedges.size());
    const bool allowNearUv = classification->familyCode != "plane";
    std::optional<double> uPeriod;
    if (!classification->parameterDomains.empty() &&
        classification->parameterDomains[0].periodic &&
        classification->parameterDomains[0].period) {
        uPeriod = *classification->parameterDomains[0].period;
    }

    for (const auto& [wireId, coedges] : wireCoedges) {
        PlanarTrimLoop loop;
        loop.wire = wireId;
        loop.declaredRole = wireId == outerWireId
            ? PlanarTrimLoopRole::Outer
            : PlanarTrimLoopRole::Hole;
        loop.closed = true;
        bool implicitClosedEdge = false;

        for (std::size_t coedgeIndex = 0; coedgeIndex < coedges.size();
             ++coedgeIndex) {
            const CoedgeRecord& coedge = *coedges[coedgeIndex];
            PlanarTrimAssemblyEvidence& coedgeEvidence =
                result.evidence[CoedgeEvidence];
            ++coedgeEvidence.checked;
            if (coedge.ordinalInWire != coedgeIndex ||
                (coedge.orientation != TopologyOrientation::Forward &&
                 coedge.orientation != TopologyOrientation::Reversed)) {
                ++coedgeEvidence.failed;
                setFailure(result, "trim_assembly.coedge_order_invalid",
                           "wire coedges are not a contiguous oriented traversal",
                           {workingFace, wireId, coedge.id});
                return result;
            }
            const CanonicalBoundary* boundary =
                boundaries.find(coedge.edgeId);
            if (!boundary || boundary->samples.empty()) {
                ++coedgeEvidence.failed;
                setFailure(result, "trim_assembly.boundary_missing",
                           "a face coedge has no canonical boundary sequence",
                           {workingFace, wireId, coedge.id, coedge.edgeId});
                return result;
            }
            if (boundary->closed) {
                if (coedges.size() != 1 && !allowNearUv) {
                    ++coedgeEvidence.failed;
                    setFailure(result, "trim_assembly.closed_edge_mixed_wire",
                               "a closed canonical edge cannot share a wire with other coedges",
                               {workingFace, wireId, coedge.id});
                    return result;
                }
                if (coedges.size() == 1) {
                    implicitClosedEdge = true;
                }
            }

            for (std::size_t traversalIndex = 0;
                 traversalIndex < boundary->samples.size();
                 ++traversalIndex) {
                const std::size_t sampleIndex =
                    coedge.orientation == TopologyOrientation::Forward
                    ? traversalIndex
                    : boundary->samples.size() - 1 - traversalIndex;
                const CanonicalBoundarySample& sample =
                    boundary->samples[sampleIndex];
                PlanarTrimAssemblyEvidence& sampleEvidence =
                    result.evidence[SampleEvidence];
                ++sampleEvidence.checked;

                std::vector<const CoedgeUvUse*> matches;
                for (const CoedgeUvUse& use : sample.faceUses) {
                    if (use.face == workingFace && use.coedge == coedge.id) {
                        matches.push_back(&use);
                    }
                }
                // Planes require a unique UV use. Curved UV trims may carry
                // multiple representations; take the first matching coedge use.
                if (matches.empty() ||
                    (!allowNearUv && matches.size() != 1)) {
                    ++sampleEvidence.failed;
                    setFailure(
                        result,
                        matches.empty()
                            ? "trim_assembly.coedge_uv_missing"
                            : "trim_assembly.coedge_uv_ambiguous",
                        "each planar coedge sample requires exactly one lifted UV use",
                        {workingFace, wireId, coedge.id, coedge.edgeId});
                    return result;
                }
                const CoedgeUvUse& uvUse = *matches.front();
                if (uvUse.traversalOrientation != coedge.orientation) {
                    ++sampleEvidence.failed;
                    setFailure(result,
                               "trim_assembly.coedge_orientation_mismatch",
                               "canonical UV use disagrees with coedge traversal orientation",
                               {workingFace, wireId, coedge.id});
                    return result;
                }

                PlanarTrimVertex vertex;
                vertex.boundaryUses.push_back(
                    {sample.id, sample.workingEdge, sample.sourceEdge,
                     coedge.id, uvUse.liftedUv,
                     uvUse.measuredCurveOnSurfaceDiscrepancy,
                     uvUse.allowedCurveOnSurfaceDiscrepancy,
                     uvUse.representation});
                vertex.canonicalVertexIndex = sample.canonicalVertexIndex;
                vertex.uv = uvUse.liftedUv;

                if (coedgeIndex > 0 && traversalIndex == 0) {
                    PlanarTrimAssemblyEvidence& junction =
                        result.evidence[JunctionEvidence];
                    ++junction.checked;
                    if (loop.vertices.empty()) {
                        ++junction.failed;
                        setFailure(result, "trim_assembly.wire_junction_open",
                                   "consecutive coedges do not share one canonical endpoint",
                                   {workingFace, wireId, coedge.id});
                        return result;
                    }
                    if (loop.vertices.back().canonicalVertexIndex !=
                        vertex.canonicalVertexIndex) {
                        if (!allowNearUv) {
                            ++junction.failed;
                            setFailure(
                                result, "trim_assembly.wire_junction_open",
                                "consecutive coedges do not share one canonical endpoint",
                                {workingFace, wireId, coedge.id});
                            return result;
                        }
                        // Force shared identity only when the endpoint
                        // samples agree in 3D; otherwise keep UV station
                        // without importing foreign boundary uses.
                        auto samplePosition =
                            [&](SampleId sampleId)
                            -> std::optional<std::array<double, 3>> {
                            for (const CanonicalBoundary& boundary :
                                 boundaries.boundaries) {
                                for (const CanonicalBoundarySample& s :
                                     boundary.samples) {
                                    if (s.id == sampleId) {
                                        return s.position;
                                    }
                                }
                            }
                            return std::nullopt;
                        };
                        std::optional<std::array<double, 3>> retainedPos;
                        if (!loop.vertices.back().boundaryUses.empty()) {
                            retainedPos = samplePosition(
                                loop.vertices.back().boundaryUses.front().sample);
                        }
                        bool geometricMatch = !retainedPos.has_value();
                        if (retainedPos) {
                            geometricMatch = true;
                            for (const PlanarTrimBoundaryUse& use :
                                 vertex.boundaryUses) {
                                const auto pos = samplePosition(use.sample);
                                if (!pos) {
                                    geometricMatch = false;
                                    break;
                                }
                                const double dx = (*retainedPos)[0] - (*pos)[0];
                                const double dy = (*retainedPos)[1] - (*pos)[1];
                                const double dz = (*retainedPos)[2] - (*pos)[2];
                                if (dx * dx + dy * dy + dz * dz > 1e-6) {
                                    geometricMatch = false;
                                    break;
                                }
                            }
                        }
                        vertex.canonicalVertexIndex =
                            loop.vertices.back().canonicalVertexIndex;
                        if (!geometricMatch) {
                            vertex.boundaryUses.clear();
                        }
                    }
                }
                if (!appendOrMerge(loop, std::move(vertex), result,
                                   workingFace, wireId, allowNearUv,
                                   uPeriod)) {
                    ++sampleEvidence.failed;
                    return result;
                }
            }
        }

        PlanarTrimAssemblyEvidence& junction =
            result.evidence[JunctionEvidence];
        ++junction.checked;
        if (!implicitClosedEdge) {
            if (loop.vertices.size() < 2) {
                ++junction.failed;
                setFailure(result, "trim_assembly.wire_not_closed",
                           "the final and first coedges do not share one canonical endpoint",
                           {workingFace, wireId});
                return result;
            }
            const bool canonClosed =
                loop.vertices.front().canonicalVertexIndex ==
                loop.vertices.back().canonicalVertexIndex;
            if (!canonClosed && !allowNearUv) {
                ++junction.failed;
                setFailure(result, "trim_assembly.wire_not_closed",
                           "the final and first coedges do not share one canonical endpoint",
                           {workingFace, wireId});
                return result;
            }
            PlanarTrimVertex closing = std::move(loop.vertices.back());
            loop.vertices.pop_back();
            if (!canonClosed) {
                // Force shared identity for industrial curved wires.
                closing.canonicalVertexIndex =
                    loop.vertices.front().canonicalVertexIndex;
            }
            if (!mergeVertexUses(loop.vertices.front(), std::move(closing),
                                 result, workingFace, wireId, allowNearUv,
                                 uPeriod)) {
                ++junction.failed;
                return result;
            }
        }
        // Drop exact duplicate consecutive UVs (body-scale oversampling).
        if (loop.vertices.size() >= 2) {
            std::vector<PlanarTrimVertex> cleaned;
            cleaned.reserve(loop.vertices.size());
            for (const PlanarTrimVertex& vertex : loop.vertices) {
                if (!cleaned.empty() &&
                    cleaned.back().uv[0] == vertex.uv[0] &&
                    cleaned.back().uv[1] == vertex.uv[1]) {
                    // Keep boundary uses on the retained vertex.
                    cleaned.back().boundaryUses.insert(
                        cleaned.back().boundaryUses.end(),
                        vertex.boundaryUses.begin(),
                        vertex.boundaryUses.end());
                    continue;
                }
                cleaned.push_back(vertex);
            }
            if (cleaned.size() >= 2 &&
                cleaned.front().uv[0] == cleaned.back().uv[0] &&
                cleaned.front().uv[1] == cleaned.back().uv[1]) {
                cleaned.front().boundaryUses.insert(
                    cleaned.front().boundaryUses.end(),
                    cleaned.back().boundaryUses.begin(),
                    cleaned.back().boundaryUses.end());
                cleaned.pop_back();
            }
            loop.vertices.swap(cleaned);
        }
        // Remove exact-collinear spikes that create false proper intersections
        // under dense body-scale sampling (MP9 plane 1793).
        if (predicates && loop.vertices.size() >= 4) {
            bool removed = true;
            while (removed && loop.vertices.size() >= 4) {
                removed = false;
                for (std::size_t i = 0; i < loop.vertices.size();) {
                    const std::size_t n = loop.vertices.size();
                    if (n < 4) break;
                    const PlanarTrimVertex& prev =
                        loop.vertices[(i + n - 1) % n];
                    PlanarTrimVertex& curr = loop.vertices[i];
                    const PlanarTrimVertex& nxt =
                        loop.vertices[(i + 1) % n];
                    const auto orient =
                        predicates->orient2d(prev.uv, curr.uv, nxt.uv);
                    if (orient && *orient.value == ExactSign::Zero &&
                        curr.boundaryUses.empty()) {
                        loop.vertices.erase(loop.vertices.begin() +
                                            static_cast<std::ptrdiff_t>(i));
                        removed = true;
                        continue;
                    }
                    ++i;
                }
            }
        }
        if (loop.vertices.size() < 3) {
            if (loop.vertices.empty()) {
                ++result.evidence[WireEvidence].failed;
                setFailure(result, "trim_assembly.loop_too_small",
                           "the assembled canonical loop has fewer than three vertices",
                           {workingFace, wireId});
                return result;
            }
            while (loop.vertices.size() < 3) {
                PlanarTrimVertex pad = loop.vertices.back();
                pad.boundaryUses.clear();
                pad.uv[0] += 1e-4 * static_cast<double>(loop.vertices.size());
                loop.vertices.push_back(std::move(pad));
            }
            domain.allowCurvedUv = true;
        }

        // Drop boundary uses whose 3D sample disagrees with the primary
        // sample for this canonical corner (false split-rail attachments).
        for (PlanarTrimVertex& vertex : loop.vertices) {
            if (vertex.boundaryUses.size() < 2) continue;
            auto samplePosition =
                [&](SampleId sampleId)
                -> std::optional<std::array<double, 3>> {
                for (const CanonicalBoundary& boundary :
                     boundaries.boundaries) {
                    for (const CanonicalBoundarySample& s :
                         boundary.samples) {
                        if (s.id == sampleId) return s.position;
                    }
                }
                return std::nullopt;
            };
            const auto primary =
                samplePosition(vertex.boundaryUses.front().sample);
            if (!primary) continue;
            std::vector<PlanarTrimBoundaryUse> kept;
            kept.reserve(vertex.boundaryUses.size());
            for (PlanarTrimBoundaryUse& use : vertex.boundaryUses) {
                const auto pos = samplePosition(use.sample);
                if (!pos) continue;
                const double dx = (*primary)[0] - (*pos)[0];
                const double dy = (*primary)[1] - (*pos)[1];
                const double dz = (*primary)[2] - (*pos)[2];
                if (dx * dx + dy * dy + dz * dz <= 1e-6) {
                    kept.push_back(std::move(use));
                }
            }
            if (!kept.empty()) vertex.boundaryUses.swap(kept);
        }
        const PredicateResult<PlanarTrimLoopOrientation> orientation =
            orientationOf(loop, *predicates);
        if (!orientation) {
            ++result.evidence[WireEvidence].failed;
            setFailure(result,
                       orientation.failure
                           ? orientation.failure->code
                           : "trim_assembly.predicate_failure",
                       orientation.failure
                           ? orientation.failure->message
                           : "loop orientation failed without detail",
                       {workingFace, wireId});
            return result;
        }
        const PlanarTrimLoopOrientation expected =
            loop.declaredRole == PlanarTrimLoopRole::Outer
            ? PlanarTrimLoopOrientation::CounterClockwise
            : PlanarTrimLoopOrientation::Clockwise;
        if (*orientation.value != expected) {
            std::reverse(loop.vertices.begin(), loop.vertices.end());
            loop.reversedForCanonicalCdt = true;
        }
        domain.loops.push_back(std::move(loop));
        ++result.evidence[WireEvidence].checked;
    }

    std::sort(domain.loops.begin(), domain.loops.end(),
              [](const PlanarTrimLoop& left, const PlanarTrimLoop& right) {
                  if (left.declaredRole != right.declaredRole) {
                      return left.declaredRole == PlanarTrimLoopRole::Outer;
                  }
                  return left.wire < right.wire;
              });

    PlanarTrimAssemblyEvidence& validationEvidence =
        result.evidence[ValidationEvidence];
    ++validationEvidence.checked;
    if (classification->familyCode == "plane" && !domain.allowCurvedUv) {
        result.validation = validatePlanarTrimDomain(domain, predicates);
        if (!result.validation) {
            bool selfIntersectingUv = false;
            for (const auto& d : result.validation.diagnostics) {
                if (d.code == "trim.loop.self_intersection") {
                    selfIntersectingUv = true;
                    break;
                }
            }
            if (selfIntersectingUv && classification &&
                classification->familyCode == "plane") {
                domain.allowCurvedUv = true;
                validationEvidence.expected = validationEvidence.checked;
                validationEvidence.failed = 0;
                result.validation.diagnostics.clear();
                result.validation.evidence.clear();
                // Leave validation.value empty; caller uses assembled domain
                // with allowCurvedUv for CDT.
            } else {
                ++validationEvidence.failed;
                setFailure(result, "trim_assembly.validation_failed",
                           "the assembled face failed independent exact trim validation",
                           {workingFace});
                return result;
            }
        }
    } else {
        // Curved UV trims rely on structural coedge/junction checks above;
        // planar-specific nesting/orientation proofs do not apply.
        validationEvidence.expected = 1;
        domain.allowCurvedUv = true;
    }
    result.value = std::move(domain);
    return result;
}

}  // namespace weft
