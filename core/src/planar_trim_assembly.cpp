#include "weft/planar_trim_assembly.hpp"

#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>

#include <algorithm>
#include <array>
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
        PredicateResult<PlanarTrimLoopOrientation> failure;
        failure.failure = PredicateFailure{
            "trim_assembly.degenerate_orientation",
            "the assembled loop has a zero exact extreme turn"};
        return failure;
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

bool mergeVertexUses(PlanarTrimVertex& retained,
                     PlanarTrimVertex candidate,
                     PlanarTrimAssemblyResult& result, StableId face,
                     StableId wire) {
    if (!sameUv(retained.uv, candidate.uv)) {
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
                   StableId wire) {
    if (loop.vertices.empty() ||
        loop.vertices.back().canonicalVertexIndex !=
            candidate.canonicalVertexIndex) {
        loop.vertices.push_back(std::move(candidate));
        return true;
    }
    return mergeVertexUses(loop.vertices.back(), std::move(candidate), result,
                           face, wire);
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
    if (!classification || classification->taxonomy != GeometryTaxonomy::Surface ||
        classification->familyCode != "plane" ||
        classification->support !=
            GeometrySupportState::SupportedAnalyticTemplate) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.face_not_planar",
                   "only a proven supported planar face may use planar trim assembly",
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
        ShapeMap wires;
        TopExp::MapShapes(snapshot.model.shape, TopAbs_WIRE, wires);
        const int outerIndex = outer.IsNull() ? 0 : wires.FindIndex(outer);
        if (outerIndex > 0) {
            outerWireId = {StableIdKind::Wire,
                           static_cast<std::uint64_t>(outerIndex)};
        }
    } catch (const Standard_Failure&) {
        outerWireId = {};
    }
    if (!outerWireId.valid() || !wireCoedges.contains(outerWireId)) {
        ++faceEvidence.failed;
        setFailure(result, "trim_assembly.outer_wire_unresolved",
                   "OCCT topology did not resolve one outer wire for the face",
                   {workingFace});
        return result;
    }

    PlanarTrimDomain domain;
    domain.face = workingFace;
    domain.sourceFace = sourceFace;
    domain.loops.reserve(wireCoedges.size());

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
                if (coedges.size() != 1) {
                    ++coedgeEvidence.failed;
                    setFailure(result, "trim_assembly.closed_edge_mixed_wire",
                               "a closed canonical edge cannot share a wire with other coedges",
                               {workingFace, wireId, coedge.id});
                    return result;
                }
                implicitClosedEdge = true;
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
                if (matches.size() != 1) {
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
                    if (loop.vertices.empty() ||
                        loop.vertices.back().canonicalVertexIndex !=
                            vertex.canonicalVertexIndex) {
                        ++junction.failed;
                        setFailure(result, "trim_assembly.wire_junction_open",
                                   "consecutive coedges do not share one canonical endpoint",
                                   {workingFace, wireId, coedge.id});
                        return result;
                    }
                }
                if (!appendOrMerge(loop, std::move(vertex), result,
                                   workingFace, wireId)) {
                    ++sampleEvidence.failed;
                    return result;
                }
            }
        }

        PlanarTrimAssemblyEvidence& junction =
            result.evidence[JunctionEvidence];
        ++junction.checked;
        if (!implicitClosedEdge) {
            if (loop.vertices.size() < 2 ||
                loop.vertices.front().canonicalVertexIndex !=
                    loop.vertices.back().canonicalVertexIndex) {
                ++junction.failed;
                setFailure(result, "trim_assembly.wire_not_closed",
                           "the final and first coedges do not share one canonical endpoint",
                           {workingFace, wireId});
                return result;
            }
            PlanarTrimVertex closing = std::move(loop.vertices.back());
            loop.vertices.pop_back();
            if (!mergeVertexUses(loop.vertices.front(), std::move(closing),
                                 result, workingFace, wireId)) {
                ++junction.failed;
                return result;
            }
        }
        if (loop.vertices.size() < 3) {
            ++result.evidence[WireEvidence].failed;
            setFailure(result, "trim_assembly.loop_too_small",
                       "the assembled canonical loop has fewer than three vertices",
                       {workingFace, wireId});
            return result;
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
    result.validation = validatePlanarTrimDomain(domain, predicates);
    if (!result.validation) {
        ++validationEvidence.failed;
        setFailure(result, "trim_assembly.validation_failed",
                   "the assembled face failed independent exact trim validation",
                   {workingFace});
        return result;
    }
    result.value = std::move(domain);
    return result;
}

}  // namespace weft
