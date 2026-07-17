#include "secure_core_internal.hpp"

#include "io/xcaf.hpp"

#include <Adaptor3d_CurveOnSurface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <GeomLib_CheckCurveOnSurface.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace weft::secure_detail {
namespace {

class TopologyIdentityCopier {
public:
    TopologyIdentityCopier()
        : history_(new BRepTools_History()) {}

    TopoDS_Shape copy(const TopoDS_Shape& source) {
        if (source.IsNull()) return {};
        const void* key = source.TShape().get();
        const auto existing = copies_.find(key);
        if (existing != copies_.end()) {
            TopoDS_Shape result = source;
            result.TShape(existing->second.TShape());
            recordHistory(source, result);
            return result;
        }

        TopoDS_Shape prototype = source.EmptyCopied();
        prototype.Orientation(TopAbs_FORWARD);
        prototype.Location(TopLoc_Location());
        prototype.Free(true);
        for (TopoDS_Iterator child(source, false, false); child.More();
             child.Next()) {
            builder_.Add(prototype, copy(child.Value()));
        }
        prototype.Free(source.Free());
        prototype.Modified(source.Modified());
        prototype.Checked(source.Checked());
        prototype.Orientable(source.Orientable());
        prototype.Closed(source.Closed());
        prototype.Infinite(source.Infinite());
        prototype.Convex(source.Convex());
        copies_.emplace(key, prototype);
        exactShapes_.bind(source, prototype);

        TopoDS_Shape result = source;
        result.TShape(prototype.TShape());
        recordHistory(source, result);
        return result;
    }

    const Handle(BRepTools_History)& history() const noexcept {
        return history_;
    }

    const ExactShapeDerivationMap& exactShapes() const noexcept {
        return exactShapes_;
    }

private:
    void recordHistory(const TopoDS_Shape& source,
                       const TopoDS_Shape& working) {
        if (!BRepTools_History::IsSupportedType(source)) return;
        const ShapeList& existing = history_->Modified(source);
        if (std::none_of(existing.begin(), existing.end(),
                         [&](const TopoDS_Shape& candidate) {
                             return candidate.IsSame(working);
                         })) {
            history_->AddModified(source, working);
        }
    }

    BRep_Builder builder_;
    std::unordered_map<const void*, TopoDS_Shape> copies_;
    Handle(BRepTools_History) history_;
    ExactShapeDerivationMap exactShapes_;
};

struct ParameterizationProof {
    std::size_t expectedPcurveUses = 0;
    std::size_t checkedPcurveUses = 0;
    double maximumDiscrepancy = 0.0;
    double toleranceEnvelope = 0.0;
};

bool exactRange(double first, double last, double candidateFirst,
                double candidateLast) {
    return std::isfinite(first) && std::isfinite(last) &&
        std::isfinite(candidateFirst) && std::isfinite(candidateLast) &&
        first < last && first == candidateFirst && last == candidateLast;
}

std::optional<ParameterizationProof> proveExistingParameterization(
    const Model& source, const TopoDS_Edge& edge) {
    if (BRep_Tool::Degenerated(edge) ||
        BRep_Tool::SameParameter(edge) || BRep_Tool::SameRange(edge)) {
        return std::nullopt;
    }

    double curveFirst = 0.0;
    double curveLast = 0.0;
    const Handle(Geom_Curve) curve =
        BRep_Tool::Curve(edge, curveFirst, curveLast);
    if (curve.IsNull() || !std::isfinite(curveFirst) ||
        !std::isfinite(curveLast) || !(curveFirst < curveLast)) {
        return std::nullopt;
    }
    const Handle(BRepAdaptor_Curve) curveAdaptor =
        new BRepAdaptor_Curve(edge);

    const int ownersIndex = source.edgeToFaces.FindIndex(edge);
    if (ownersIndex <= 0) return std::nullopt;
    const ShapeList& owners = source.edgeToFaces.FindFromIndex(ownersIndex);
    if (owners.IsEmpty()) return std::nullopt;
    ShapeMap uniqueOwners;
    for (const TopoDS_Shape& owner : owners) uniqueOwners.Add(owner);
    if (uniqueOwners.IsEmpty()) return std::nullopt;

    ParameterizationProof proof;
    proof.toleranceEnvelope = BRep_Tool::Tolerance(edge);
    if (!std::isfinite(proof.toleranceEnvelope) ||
        proof.toleranceEnvelope < 0.0) {
        return std::nullopt;
    }

    try {
        for (int ownerIndex = 1; ownerIndex <= uniqueOwners.Extent();
             ++ownerIndex) {
            const TopoDS_Shape& owner = uniqueOwners(ownerIndex);
            if (owner.ShapeType() != TopAbs_FACE) return std::nullopt;
            const TopoDS_Face face = TopoDS::Face(owner);
            const bool seam = BRep_Tool::IsClosed(edge, face);
            proof.expectedPcurveUses += seam ? 2U : 1U;
            const Handle(BRepAdaptor_Surface) surfaceAdaptor =
                new BRepAdaptor_Surface(face, false);

            const auto checkStoredPcurve =
                [&](TopoDS_Edge orientedEdge) {
                    double pcurveFirst = 0.0;
                    double pcurveLast = 0.0;
                    bool stored = false;
                    const Handle(Geom2d_Curve) pcurve =
                        BRep_Tool::CurveOnSurface(
                            orientedEdge, face, pcurveFirst, pcurveLast,
                            &stored);
                    if (!stored || pcurve.IsNull() ||
                        !exactRange(curveFirst, curveLast, pcurveFirst,
                                    pcurveLast)) {
                        return false;
                    }
                    const Handle(BRepAdaptor_Curve2d) pcurveAdaptor =
                        new BRepAdaptor_Curve2d(orientedEdge, face);
                    const Handle(Adaptor3d_CurveOnSurface)
                        curveOnSurface = new Adaptor3d_CurveOnSurface(
                            pcurveAdaptor, surfaceAdaptor);
                    GeomLib_CheckCurveOnSurface check(curveAdaptor);
                    check.SetParallel(false);
                    check.Perform(curveOnSurface);
                    if (!check.IsDone() ||
                        !std::isfinite(check.MaxDistance()) ||
                        check.MaxDistance() > proof.toleranceEnvelope) {
                        return false;
                    }
                    proof.maximumDiscrepancy = std::max(
                        proof.maximumDiscrepancy, check.MaxDistance());
                    ++proof.checkedPcurveUses;
                    return true;
                };
            if (!checkStoredPcurve(edge)) return std::nullopt;
            if (seam) {
                TopoDS_Edge reversed = edge;
                reversed.Reverse();
                if (!checkStoredPcurve(reversed)) return std::nullopt;
            }
        }
    } catch (const Standard_Failure&) {
        return std::nullopt;
    }

    if (proof.expectedPcurveUses == 0 ||
        proof.checkedPcurveUses != proof.expectedPcurveUses) {
        return std::nullopt;
    }
    return proof;
}

std::string proofDetail(const ParameterizationProof& proof) {
    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "set existing SameRange and SameParameter flags after checking "
           << proof.checkedPcurveUses
           << " stored p-curve use(s); maximum discrepancy="
           << proof.maximumDiscrepancy
           << ", source tolerance envelope=" << proof.toleranceEnvelope;
    return detail.str();
}

Handle(BRepTools_History) composeCopyRepairHistory(
    const TopoDS_Shape& sourceShape, const BRepBuilderAPI_Copy& copier,
    const Handle(BRepTools_History)& repairHistory) {
    Handle(BRepTools_History) composed = new BRepTools_History();
    ShapeMap sourceShapes;
    sourceShapes.Add(sourceShape);
    TopExp::MapShapes(sourceShape, sourceShapes);
    for (int index = 1; index <= sourceShapes.Extent(); ++index) {
        const TopoDS_Shape& source = sourceShapes(index);
        if (!BRepTools_History::IsSupportedType(source)) continue;
        const TopoDS_Shape copied = copier.ModifiedShape(source);
        if (copied.IsNull() ||
            (!repairHistory.IsNull() && repairHistory->IsRemoved(copied))) {
            composed->Remove(source);
            continue;
        }
        bool mapped = false;
        if (!repairHistory.IsNull()) {
            for (const TopoDS_Shape& finalShape :
                 repairHistory->Modified(copied)) {
                composed->AddModified(source, finalShape);
                mapped = true;
            }
            for (const TopoDS_Shape& finalShape :
                 repairHistory->Generated(copied)) {
                composed->AddGenerated(source, finalShape);
                mapped = true;
            }
        }
        if (!mapped) composed->AddModified(source, copied);
    }
    return composed;
}

std::string toleranceDetail(const ToleranceChange& change,
                            double toleranceEnvelope) {
    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "raised the working vertex tolerance from " << change.before
           << " to the measured incidence gap " << change.after
           << " within the incident-edge evidence envelope "
           << toleranceEnvelope;
    return detail.str();
}

// Raises only working vertex tolerances, only up to the measured incidence
// gap, and only when that gap stays within the largest stored tolerance of
// the incident source edges — the evidence envelope. Anything beyond
// refuses by name and leaves the working copy untouched.
void reconcileVertexTolerances(const Model& source,
                               ConservativeWorkingDerivation& derivation) {
    EdgeFaceMap vertexToEdges;
    TopExp::MapShapesAndAncestors(source.shape, TopAbs_VERTEX, TopAbs_EDGE,
                                  vertexToEdges);
    BRep_Builder builder;
    for (int vertexIndex = 1; vertexIndex <= vertexToEdges.Extent();
         ++vertexIndex) {
        const TopoDS_Shape& vertexShape = vertexToEdges.FindKey(vertexIndex);
        const StableId vertexId{StableIdKind::Vertex,
                                static_cast<std::uint64_t>(vertexIndex)};
        VertexToleranceEvidence evidence;
        double storedTolerance = 0.0;
        try {
            storedTolerance =
                BRep_Tool::Tolerance(TopoDS::Vertex(vertexShape));
            evidence = measureVertexToleranceEvidence(source, vertexShape);
        } catch (const Standard_Failure&) {
            continue;
        }
        if (!evidence.measured || !std::isfinite(evidence.requiredGap) ||
            evidence.requiredGap <= storedTolerance) {
            continue;
        }
        if (!std::isfinite(evidence.toleranceEnvelope) ||
            evidence.requiredGap > evidence.toleranceEnvelope) {
            derivation.refusals.push_back(
                {"repair.tolerance.gap_beyond_envelope", {vertexId},
                 "the measured vertex incidence gap exceeds every stored "
                 "tolerance of its incident source edges"});
            continue;
        }
        const TopoDS_Shape workingVertex =
            derivation.exactShapes.mapped(vertexShape);
        if (workingVertex.IsNull()) {
            throw std::logic_error(
                "tolerance reconciliation lost its working derivation");
        }
        builder.UpdateVertex(TopoDS::Vertex(workingVertex),
                             evidence.requiredGap);
        if (BRep_Tool::Tolerance(TopoDS::Vertex(vertexShape)) !=
                storedTolerance ||
            BRep_Tool::Tolerance(TopoDS::Vertex(workingVertex)) <
                evidence.requiredGap) {
            throw std::logic_error(
                "tolerance reconciliation violated source/working isolation");
        }
        ToleranceChange change{vertexId, vertexId, storedTolerance,
                               BRep_Tool::Tolerance(
                                   TopoDS::Vertex(workingVertex))};
        derivation.operations.push_back(
            {"repair.tolerance_reconciliation", {vertexId}, {vertexId},
             toleranceDetail(change, evidence.toleranceEnvelope)});
        derivation.toleranceReconciliations.push_back(change);
    }
}

}  // namespace

VertexToleranceEvidence measureVertexToleranceEvidence(
    const Model& model, const TopoDS_Shape& vertex) {
    VertexToleranceEvidence evidence;
    if (vertex.IsNull() || vertex.ShapeType() != TopAbs_VERTEX) {
        return evidence;
    }
    EdgeFaceMap vertexToEdges;
    TopExp::MapShapesAndAncestors(model.shape, TopAbs_VERTEX, TopAbs_EDGE,
                                  vertexToEdges);
    const int index = vertexToEdges.FindIndex(vertex);
    if (index <= 0) return evidence;
    const TopoDS_Vertex vertexShape =
        TopoDS::Vertex(vertexToEdges.FindKey(index));
    const gp_Pnt position = BRep_Tool::Pnt(vertexShape);
    ShapeMap uniqueEdges;
    for (const TopoDS_Shape& owner : vertexToEdges.FindFromIndex(index)) {
        uniqueEdges.Add(owner);
    }
    for (int edgeIndex = 1; edgeIndex <= uniqueEdges.Extent(); ++edgeIndex) {
        const TopoDS_Edge edge = TopoDS::Edge(uniqueEdges(edgeIndex));
        if (BRep_Tool::Degenerated(edge)) continue;
        double first = 0.0;
        double last = 0.0;
        const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);
        if (curve.IsNull()) continue;
        double parameter = 0.0;
        try {
            parameter = BRep_Tool::Parameter(vertexShape, edge);
        } catch (const Standard_Failure&) {
            continue;
        }
        if (!std::isfinite(parameter)) continue;
        const double gap = position.Distance(curve->Value(parameter));
        if (!std::isfinite(gap)) continue;
        evidence.measured = true;
        evidence.requiredGap = std::max(evidence.requiredGap, gap);
        evidence.toleranceEnvelope = std::max(
            evidence.toleranceEnvelope, BRep_Tool::Tolerance(edge));
    }
    return evidence;
}

ConservativeWorkingDerivation deriveConservativeWorking(
    const Model& source) {
    ConservativeWorkingDerivation derivation;
    TopologyIdentityCopier copier;
    derivation.shape = copier.copy(source.shape);
    derivation.history = copier.history();
    derivation.exactShapes = copier.exactShapes();

    Model working = indexShape(derivation.shape);
    if (source.edges.Extent() != working.edges.Extent()) {
        throw std::logic_error(
            "topology identity copy changed the unique edge count");
    }

    BRep_Builder builder;
    for (int edgeIndex = 1; edgeIndex <= source.edges.Extent(); ++edgeIndex) {
        const TopoDS_Edge sourceEdge =
            TopoDS::Edge(source.edges(edgeIndex));
        const TopoDS_Edge workingEdge =
            TopoDS::Edge(working.edges(edgeIndex));
        if (!derivation.exactShapes.maps(sourceEdge, workingEdge)) continue;
        const std::optional<ParameterizationProof> proof =
            proveExistingParameterization(source, sourceEdge);
        if (!proof) continue;

        builder.SameRange(workingEdge, true);
        builder.SameParameter(workingEdge, true);
        if (BRep_Tool::SameRange(sourceEdge) ||
            BRep_Tool::SameParameter(sourceEdge) ||
            !BRep_Tool::SameRange(workingEdge) ||
            !BRep_Tool::SameParameter(workingEdge)) {
            throw std::logic_error(
                "bounded parameterization repair violated source/working isolation");
        }

        const StableId edgeId{StableIdKind::Edge,
                              static_cast<std::uint64_t>(edgeIndex)};
        derivation.operations.push_back(
            {"repair.same_parameter_range_flags", {edgeId}, {edgeId},
             proofDetail(*proof)});
        derivation.parameterizationFlagChanges.push_back(
            {edgeId,
             edgeId,
             false,
             false,
             true,
             true,
             proof->expectedPcurveUses,
             proof->checkedPcurveUses,
             proof->maximumDiscrepancy,
             proof->toleranceEnvelope});
    }
    repairShellOrientations(source, derivation);
    reconcileVertexTolerances(source, derivation);
    return derivation;
}

CompatibilityWorkingDerivation deriveCompatibilityWorking(
    const Model& source) {
    CompatibilityWorkingDerivation derivation;
    BRepBuilderAPI_Copy copier(source.shape, true, false);
    Handle(BRepTools_History) repairHistory;
    derivation.shape = healWithHistory(copier.Shape(), repairHistory);
    derivation.history = composeCopyRepairHistory(
        source.shape, copier, repairHistory);
    derivation.operations.push_back({
        "repair.compatibility_pipeline", {}, {},
        "historical Weft healing pipeline applied to a geometry-deep working copy after immutable source capture"});
    return derivation;
}

}  // namespace weft::secure_detail
