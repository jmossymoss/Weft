#include "secure_core_internal.hpp"

#include "io/xcaf.hpp"

#include <Adaptor3d_CurveOnSurface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <GeomLib_CheckCurveOnSurface.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopAbs.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp.hxx>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

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

struct CurveOnSurfaceProof {
    std::size_t expectedPcurveUses = 0;
    std::size_t checkedPcurveUses = 0;
    double maximumDiscrepancy = 0.0;
    double sourceTolerance = 0.0;
};

bool exactRange(double first, double last, double candidateFirst,
                double candidateLast) {
    return std::isfinite(first) && std::isfinite(last) &&
        std::isfinite(candidateFirst) && std::isfinite(candidateLast) &&
        first < last && first == candidateFirst && last == candidateLast;
}

std::optional<CurveOnSurfaceProof> measureStoredCurveOnSurface(
    const Model& source, const TopoDS_Edge& edge) {
    if (BRep_Tool::Degenerated(edge)) return std::nullopt;

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

    CurveOnSurfaceProof proof;
    proof.sourceTolerance = BRep_Tool::Tolerance(edge);
    if (!std::isfinite(proof.sourceTolerance) ||
        proof.sourceTolerance < 0.0) {
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
#if OCC_VERSION_HEX >= 0x070800
                    check.SetParallel(false);
#endif
                    check.Perform(curveOnSurface);
                    if (!check.IsDone() ||
                        !std::isfinite(check.MaxDistance()) ||
                        check.MaxDistance() < 0.0) {
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

std::optional<CurveOnSurfaceProof> proveExistingParameterization(
    const Model& source, const TopoDS_Edge& edge) {
    if (BRep_Tool::SameParameter(edge) || BRep_Tool::SameRange(edge)) {
        return std::nullopt;
    }
    std::optional<CurveOnSurfaceProof> proof =
        measureStoredCurveOnSurface(source, edge);
    if (!proof) return std::nullopt;
    if (proof->maximumDiscrepancy > proof->sourceTolerance) {
        return std::nullopt;
    }
    return proof;
}

std::optional<CurveOnSurfaceProof> proveToleranceEnvelopeRaise(
    const Model& source, const TopoDS_Edge& edge) {
    std::optional<CurveOnSurfaceProof> proof =
        measureStoredCurveOnSurface(source, edge);
    if (!proof) return std::nullopt;
    if (!(proof->maximumDiscrepancy > proof->sourceTolerance)) {
        return std::nullopt;
    }
    return proof;
}

std::string parameterizationProofDetail(const CurveOnSurfaceProof& proof) {
    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "set existing SameRange and SameParameter flags after checking "
           << proof.checkedPcurveUses
           << " stored p-curve use(s); maximum discrepancy="
           << proof.maximumDiscrepancy
           << ", source tolerance envelope=" << proof.sourceTolerance;
    return detail.str();
}

std::string toleranceProofDetail(const CurveOnSurfaceProof& proof) {
    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "raise working edge tolerance to measured curve-on-surface "
           << "maximum after checking " << proof.checkedPcurveUses
           << " stored p-curve use(s); maximum discrepancy="
           << proof.maximumDiscrepancy
           << ", source tolerance=" << proof.sourceTolerance;
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

bool workingShapeIsValid(const TopoDS_Shape& shape) {
    // BRepCheck_Analyzer mutates Checked flags on the input shape. Probe a
    // disposable geometry-deep copy so identity digests stay stable.
    BRepBuilderAPI_Copy probe(shape, true, false);
    if (probe.Shape().IsNull()) return false;
    return BRepCheck_Analyzer(probe.Shape(), true).IsValid();
}

TopologyOrientation topologyOrientationOf(TopAbs_Orientation orientation) {
    switch (orientation) {
        case TopAbs_FORWARD: return TopologyOrientation::Forward;
        case TopAbs_REVERSED: return TopologyOrientation::Reversed;
        case TopAbs_INTERNAL: return TopologyOrientation::Internal;
        case TopAbs_EXTERNAL: return TopologyOrientation::External;
    }
    return TopologyOrientation::Forward;
}

struct FaceEdgeUse {
    std::size_t faceIndex = 0;
    TopAbs_Orientation composedOrientation = TopAbs_FORWARD;
};

struct ShellOrientationSolution {
    std::vector<TopoDS_Face> faces;
    std::vector<TopAbs_Orientation> sourceOrientations;
    std::vector<TopAbs_Orientation> solvedOrientations;
    std::size_t manifoldEdgesChecked = 0;
    bool polarityFlipped = false;
};

enum class ShellOrientationRefusal {
    None,
    Open,
    NonManifold,
    NonOrientable,
    Unsupported,
};

std::optional<ShellOrientationSolution> solveClosedShellOrientation(
    const TopoDS_Shell& shell);

ImportDiagnostic makeNamedRefusal(std::string code, std::string message) {
    return {{}, std::move(code), DiagnosticSeverity::Error,
            {{StableIdKind::Model, 1}}, std::move(message)};
}

void recordOrientationRefusal(ConservativeWorkingDerivation& derivation,
                              ShellOrientationRefusal refusal) {
    switch (refusal) {
        case ShellOrientationRefusal::Open:
            derivation.namedRefusals.push_back(makeNamedRefusal(
                "import.repair.orientation_open_shell",
                "face-adjacency orientation repair refuses open shells"));
            break;
        case ShellOrientationRefusal::NonManifold:
            derivation.namedRefusals.push_back(makeNamedRefusal(
                "import.repair.orientation_non_manifold",
                "face-adjacency orientation repair refuses non-manifold "
                "edge/face incidence"));
            break;
        case ShellOrientationRefusal::NonOrientable:
            derivation.namedRefusals.push_back(makeNamedRefusal(
                "import.repair.orientation_non_orientable",
                "face-adjacency orientation repair refuses non-orientable "
                "shell parity"));
            break;
        case ShellOrientationRefusal::Unsupported:
            derivation.namedRefusals.push_back(makeNamedRefusal(
                "import.repair.orientation_unsupported",
                "face-adjacency orientation repair refuses unsupported shell "
                "structure"));
            break;
        case ShellOrientationRefusal::None:
            break;
    }
}

ShellOrientationRefusal classifyShellOrientationRefusal(
    const TopoDS_Shell& shell) {
    if (shell.IsNull()) return ShellOrientationRefusal::Unsupported;
    if (!shell.Closed()) return ShellOrientationRefusal::Open;

    std::size_t faceCount = 0;
    std::unordered_map<const void*, std::size_t> usesByEdge;
    for (TopoDS_Iterator faceIt(shell, false, false); faceIt.More();
         faceIt.Next()) {
        if (faceIt.Value().ShapeType() != TopAbs_FACE) {
            return ShellOrientationRefusal::Unsupported;
        }
        const TopoDS_Face face = TopoDS::Face(faceIt.Value());
        if (face.Orientation() != TopAbs_FORWARD &&
            face.Orientation() != TopAbs_REVERSED) {
            return ShellOrientationRefusal::Unsupported;
        }
        ++faceCount;
        for (TopoDS_Iterator wireIt(face, false, false); wireIt.More();
             wireIt.Next()) {
            if (wireIt.Value().ShapeType() != TopAbs_WIRE) continue;
            for (TopoDS_Iterator edgeIt(wireIt.Value(), false, false);
                 edgeIt.More(); edgeIt.Next()) {
                if (edgeIt.Value().ShapeType() != TopAbs_EDGE) continue;
                const TopoDS_Edge edge = TopoDS::Edge(edgeIt.Value());
                if (BRep_Tool::Degenerated(edge)) continue;
                if (edge.Orientation() != TopAbs_FORWARD &&
                    edge.Orientation() != TopAbs_REVERSED) {
                    return ShellOrientationRefusal::Unsupported;
                }
                ++usesByEdge[edge.TShape().get()];
            }
        }
    }
    if (faceCount < 2 || usesByEdge.empty()) {
        return ShellOrientationRefusal::Unsupported;
    }
    bool sawOpen = false;
    bool sawNonManifold = false;
    for (const auto& [edgeKey, uses] : usesByEdge) {
        (void)edgeKey;
        if (uses < 2) sawOpen = true;
        if (uses > 2) sawNonManifold = true;
    }
    if (sawNonManifold) return ShellOrientationRefusal::NonManifold;
    if (sawOpen) return ShellOrientationRefusal::Open;

    // Closed two-manifold but parity / polarity may still fail.
    if (!solveClosedShellOrientation(shell)) {
        return ShellOrientationRefusal::NonOrientable;
    }
    return ShellOrientationRefusal::None;
}

std::optional<ShellOrientationSolution> solveClosedShellOrientation(
    const TopoDS_Shell& shell) {
    if (shell.IsNull() || !shell.Closed()) return std::nullopt;

    ShellOrientationSolution solution;
    for (TopoDS_Iterator faceIt(shell, false, false); faceIt.More();
         faceIt.Next()) {
        if (faceIt.Value().ShapeType() != TopAbs_FACE) return std::nullopt;
        const TopoDS_Face face = TopoDS::Face(faceIt.Value());
        if (face.Orientation() != TopAbs_FORWARD &&
            face.Orientation() != TopAbs_REVERSED) {
            return std::nullopt;
        }
        solution.faces.push_back(face);
        solution.sourceOrientations.push_back(face.Orientation());
        solution.solvedOrientations.push_back(face.Orientation());
    }
    if (solution.faces.size() < 2) return std::nullopt;

    std::unordered_map<const void*, std::vector<FaceEdgeUse>> usesByEdge;
    for (std::size_t faceIndex = 0; faceIndex < solution.faces.size();
         ++faceIndex) {
        const TopoDS_Face& face = solution.faces[faceIndex];
        for (TopoDS_Iterator wireIt(face, false, false); wireIt.More();
             wireIt.Next()) {
            if (wireIt.Value().ShapeType() != TopAbs_WIRE) continue;
            for (TopoDS_Iterator edgeIt(wireIt.Value(), false, false);
                 edgeIt.More(); edgeIt.Next()) {
                if (edgeIt.Value().ShapeType() != TopAbs_EDGE) continue;
                const TopoDS_Edge edge = TopoDS::Edge(edgeIt.Value());
                if (BRep_Tool::Degenerated(edge)) continue;
                if (edge.Orientation() != TopAbs_FORWARD &&
                    edge.Orientation() != TopAbs_REVERSED) {
                    return std::nullopt;
                }
                usesByEdge[edge.TShape().get()].push_back(
                    {faceIndex,
                     TopAbs::Compose(face.Orientation(),
                                     edge.Orientation())});
            }
        }
    }
    if (usesByEdge.empty()) return std::nullopt;

    std::vector<std::vector<std::pair<std::size_t, bool>>> adjacency(
        solution.faces.size());
    for (const auto& [edgeKey, uses] : usesByEdge) {
        (void)edgeKey;
        if (uses.size() != 2) return std::nullopt;
        ++solution.manifoldEdgesChecked;
        const FaceEdgeUse& first = uses[0];
        const FaceEdgeUse& second = uses[1];
        const bool sameSense =
            first.composedOrientation == second.composedOrientation;
        adjacency[first.faceIndex].push_back(
            {second.faceIndex, sameSense});
        adjacency[second.faceIndex].push_back(
            {first.faceIndex, sameSense});
    }

    std::vector<char> visited(solution.faces.size(), 0);
    std::vector<char> reverseRelative(solution.faces.size(), 0);
    for (std::size_t root = 0; root < solution.faces.size(); ++root) {
        if (visited[root]) continue;
        std::vector<std::size_t> stack;
        stack.push_back(root);
        visited[root] = 1;
        reverseRelative[root] = 0;
        while (!stack.empty()) {
            const std::size_t faceIndex = stack.back();
            stack.pop_back();
            for (const auto& [neighbor, needsRelativeReverse] :
                 adjacency[faceIndex]) {
                const char required =
                    static_cast<char>(reverseRelative[faceIndex] ^
                                      (needsRelativeReverse ? 1 : 0));
                if (!visited[neighbor]) {
                    visited[neighbor] = 1;
                    reverseRelative[neighbor] = required;
                    stack.push_back(neighbor);
                    continue;
                }
                if (reverseRelative[neighbor] != required) {
                    return std::nullopt;
                }
            }
        }
    }

    for (std::size_t faceIndex = 0; faceIndex < solution.faces.size();
         ++faceIndex) {
        if (reverseRelative[faceIndex]) {
            solution.solvedOrientations[faceIndex] = TopAbs::Reverse(
                solution.sourceOrientations[faceIndex]);
        }
    }
    return solution;
}

bool solidNeedsOppositePolarity(const TopoDS_Solid& solid) {
    GProp_GProps props;
    BRepGProp::VolumeProperties(solid, props);
    const double mass = props.Mass();
    if (!std::isfinite(mass) || mass <= 0.0) return true;

    // OCCT 7.9 has been observed to SIGSEGV inside PerformInfinitePoint on
    // some imported face pcurves (Geom2dAdaptor_Curve::D1). Mass sign remains
    // the hard gate above; skip the infinite-point probe before OCCT 8.
#if OCC_VERSION_HEX < 0x080000
    (void)solid;
    return false;
#else
    try {
        BRepClass3d_SolidClassifier classifier(solid);
        classifier.PerformInfinitePoint(1.0e-7);
        return classifier.State() == TopAbs_IN;
    } catch (const Standard_Failure&) {
        return false;
    }
#endif
}

TopoDS_Shell rebuildShell(const ShellOrientationSolution& solution) {
    BRep_Builder builder;
    TopoDS_Shell shell;
    builder.MakeShell(shell);
    for (std::size_t faceIndex = 0; faceIndex < solution.faces.size();
         ++faceIndex) {
        TopoDS_Face face = solution.faces[faceIndex];
        face.Orientation(solution.solvedOrientations[faceIndex]);
        builder.Add(shell, face);
    }
    shell.Closed(true);
    shell.Orientation(TopAbs_FORWARD);
    return shell;
}

TopoDS_Solid rebuildSolidWithShell(const TopoDS_Shell& shell) {
    BRep_Builder builder;
    TopoDS_Solid solid;
    builder.MakeSolid(solid);
    builder.Add(solid, shell);
    return solid;
}

bool replaceSolidInWorkingRoot(TopoDS_Shape& root,
                               const TopoDS_Solid& oldSolid,
                               const TopoDS_Solid& newSolid,
                               ConservativeWorkingDerivation* derivation,
                               const TopoDS_Shape& sourceRoot) {
    if (root.IsNull() || oldSolid.IsNull() || newSolid.IsNull()) {
        return false;
    }
    if (root.ShapeType() == TopAbs_SOLID) {
        if (!root.IsPartner(oldSolid) && !root.IsSame(oldSolid)) {
            return false;
        }
        TopoDS_Shape replacement = root;
        replacement.TShape(newSolid.TShape());
        root = replacement;
        return true;
    }
    if (root.ShapeType() != TopAbs_COMPOUND &&
        root.ShapeType() != TopAbs_COMPSOLID) {
        return false;
    }

    const TopoDS_Shape previousRoot = root;
    BRep_Builder builder;
    TopoDS_Shape rebuilt = root.EmptyCopied();
    bool replaced = false;
    for (TopoDS_Iterator it(root, false, false); it.More(); it.Next()) {
        const TopoDS_Shape& child = it.Value();
        if (child.ShapeType() == TopAbs_SOLID &&
            (child.IsPartner(oldSolid) || child.IsSame(oldSolid))) {
            TopoDS_Shape replacement = child;
            replacement.TShape(newSolid.TShape());
            builder.Add(rebuilt, replacement);
            replaced = true;
            continue;
        }
        builder.Add(rebuilt, child);
    }
    if (!replaced) return false;
    rebuilt.Orientation(root.Orientation());
    rebuilt.Location(root.Location());
    root = rebuilt;
    if (derivation != nullptr && !sourceRoot.IsNull() &&
        derivation->exactShapes.mapsPartner(sourceRoot, previousRoot)) {
        derivation->exactShapes.rebind(sourceRoot, root);
        if (!derivation->history.IsNull()) {
            derivation->history->AddModified(sourceRoot, root);
        }
    }
    return true;
}

bool applyRootSolidOrientationRepair(
    ConservativeWorkingDerivation& derivation, const Model& source,
    const Model& working, const TopoDS_Solid& sourceSolid,
    TopoDS_Solid workingSolid) {
    TopoDS_Shell sourceShell;
    TopoDS_Shell workingShell;
    int shellCount = 0;
    for (TopoDS_Iterator it(workingSolid, false, false); it.More();
         it.Next()) {
        if (it.Value().ShapeType() != TopAbs_SHELL) return false;
        workingShell = TopoDS::Shell(it.Value());
        ++shellCount;
    }
    if (shellCount != 1 || workingShell.IsNull() || !workingShell.Closed()) {
        return false;
    }
    for (TopoDS_Iterator it(sourceSolid, false, false); it.More();
         it.Next()) {
        if (it.Value().ShapeType() != TopAbs_SHELL) return false;
        sourceShell = TopoDS::Shell(it.Value());
    }
    if (sourceShell.IsNull()) return false;

    const TopAbs_Orientation shellSourceOrientation =
        workingShell.Orientation();
    std::vector<TopAbs_Orientation> localSourceOrientations;
    TopoDS_Shell effectiveShell;
    BRep_Builder shellBuilder;
    shellBuilder.MakeShell(effectiveShell);
    for (TopoDS_Iterator faceIt(workingShell, false, false); faceIt.More();
         faceIt.Next()) {
        if (faceIt.Value().ShapeType() != TopAbs_FACE) return false;
        TopoDS_Face face = TopoDS::Face(faceIt.Value());
        localSourceOrientations.push_back(face.Orientation());
        face.Orientation(TopAbs::Compose(shellSourceOrientation,
                                         face.Orientation()));
        shellBuilder.Add(effectiveShell, face);
    }
    effectiveShell.Closed(workingShell.Closed());
    effectiveShell.Orientation(TopAbs_FORWARD);

    std::optional<ShellOrientationSolution> solution =
        solveClosedShellOrientation(effectiveShell);
    if (!solution) return false;
    if (localSourceOrientations.size() != solution->faces.size()) {
        return false;
    }

    bool faceChanged = false;
    for (std::size_t faceIndex = 0; faceIndex < solution->faces.size();
         ++faceIndex) {
        if (localSourceOrientations[faceIndex] !=
            solution->solvedOrientations[faceIndex]) {
            faceChanged = true;
            break;
        }
    }
    const bool shellNeedsNormalize =
        shellSourceOrientation != TopAbs_FORWARD;
    // Refuse polarity-only mutation: whole-solid polarity without an
    // adjacency or shell-occurrence change is BR-015 territory.
    if (!faceChanged && !shellNeedsNormalize) {
        return true;
    }

    TopoDS_Shell repairedShell = rebuildShell(*solution);
    TopoDS_Solid repairedSolid = rebuildSolidWithShell(repairedShell);
    std::size_t polarityChecks = 1;
    if (solidNeedsOppositePolarity(repairedSolid)) {
        for (TopAbs_Orientation& orientation :
             solution->solvedOrientations) {
            orientation = TopAbs::Reverse(orientation);
        }
        repairedShell = rebuildShell(*solution);
        repairedSolid = rebuildSolidWithShell(repairedShell);
        solution->polarityFlipped = true;
        if (solidNeedsOppositePolarity(repairedSolid)) return false;
    }
    // Commit only when the rebuilt solid is independently valid. A solver
    // proposal that leaves other pathologies in place must not rewrite
    // identity copies of non-orientation failures.
    if (!workingShapeIsValid(repairedSolid)) {
        return false;
    }

    const int solidIndex = source.solids.FindIndex(sourceSolid);
    if (solidIndex <= 0) return false;
    const StableId solidId{StableIdKind::Solid,
                           static_cast<std::uint64_t>(solidIndex)};

    std::size_t faceChangeCount = 0;
    for (std::size_t faceIndex = 0; faceIndex < solution->faces.size();
         ++faceIndex) {
        if (localSourceOrientations[faceIndex] ==
            solution->solvedOrientations[faceIndex]) {
            continue;
        }
        const int faceIndexInModel =
            working.faces.FindIndex(solution->faces[faceIndex]);
        if (faceIndexInModel <= 0) return false;
        const StableId faceId{StableIdKind::Face,
                              static_cast<std::uint64_t>(faceIndexInModel)};
        derivation.orientationChanges.push_back(
            {faceId, faceId,
             topologyOrientationOf(localSourceOrientations[faceIndex]),
             topologyOrientationOf(solution->solvedOrientations[faceIndex]),
             solution->manifoldEdgesChecked, polarityChecks});
        ++faceChangeCount;
    }
    if (faceChangeCount == 0 &&
        shellSourceOrientation != TopAbs_FORWARD) {
        // Shell-only absorption still mutates occurrence orientations through
        // the rebuilt forward shell; record the solid subject as witness.
        derivation.orientationChanges.push_back(
            {solidId, solidId,
             topologyOrientationOf(shellSourceOrientation),
             TopologyOrientation::Forward, solution->manifoldEdgesChecked,
             polarityChecks});
    }

    if (!replaceSolidInWorkingRoot(derivation.shape, workingSolid,
                                   repairedSolid, &derivation,
                                   source.shape)) {
        return false;
    }

    derivation.exactShapes.rebind(sourceSolid, repairedSolid);
    derivation.exactShapes.rebind(sourceShell, repairedShell);
    if (!derivation.history.IsNull()) {
        derivation.history->AddModified(sourceSolid, repairedSolid);
        derivation.history->AddModified(sourceShell, repairedShell);
    }

    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << "face-adjacency orientation repair on closed solid; "
           << "manifold edges checked=" << solution->manifoldEdgesChecked
           << ", polarity checks=" << polarityChecks
           << ", face orientation changes=" << faceChangeCount
           << ", shell normalized="
           << (shellSourceOrientation != TopAbs_FORWARD ? "yes" : "no");
    derivation.operations.push_back(
        {"repair.orientation_face_adjacency", {solidId}, {solidId},
         detail.str()});
    return true;
}

int countFreeEdges(const TopoDS_Shape& shape) {
    EdgeFaceMap edgeToFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);
    int freeEdges = 0;
    for (int index = 1; index <= edgeToFaces.Extent(); ++index) {
        const TopoDS_Edge edge = TopoDS::Edge(edgeToFaces.FindKey(index));
        if (BRep_Tool::Degenerated(edge)) continue;
        if (edgeToFaces(index).Extent() < 2) ++freeEdges;
    }
    return freeEdges;
}

bool repairBoundedSewing(ConservativeWorkingDerivation& derivation,
                         const Model& source) {
    if (derivation.shape.IsNull()) return false;
    const TopAbs_ShapeEnum rootType = derivation.shape.ShapeType();
    // Only open face compounds/shells. Never sew an already-closed solid:
    // legacy sew can demote solids and destroy StableId alignment.
    if (rootType != TopAbs_COMPOUND && rootType != TopAbs_SHELL) {
        return false;
    }
    if (source.solids.Extent() != 0) return false;
    if (source.faces.Extent() < 2) return false;

    const int freeBefore = countFreeEdges(derivation.shape);
    if (freeBefore < 2) return false;

    constexpr double kSewToleranceMm = 1.0e-4;
    BRepBuilderAPI_Sewing sewer(kSewToleranceMm, true, true, true, false);
    if (rootType == TopAbs_COMPOUND) {
        bool addedFace = false;
        for (TopoDS_Iterator it(derivation.shape, false, false); it.More();
             it.Next()) {
            if (it.Value().ShapeType() == TopAbs_FACE) {
                sewer.Add(it.Value());
                addedFace = true;
            } else if (it.Value().ShapeType() == TopAbs_SHELL) {
                sewer.Add(it.Value());
                addedFace = true;
            }
        }
        if (!addedFace) return false;
    } else {
        sewer.Add(derivation.shape);
    }
    sewer.Perform();
    const TopoDS_Shape sewed = sewer.SewedShape();
    if (sewed.IsNull()) return false;

    ShapeMap sourceFaces;
    TopExp::MapShapes(derivation.shape, TopAbs_FACE, sourceFaces);
    ShapeMap sewedFaces;
    TopExp::MapShapes(sewed, TopAbs_FACE, sewedFaces);
    if (sourceFaces.Extent() == 0 ||
        sourceFaces.Extent() != sewedFaces.Extent()) {
        return false;
    }

    Handle(BRepTools_History) sewHistory = new BRepTools_History();
    for (int sourceIndex = 1; sourceIndex <= source.faces.Extent();
         ++sourceIndex) {
        const TopoDS_Face sourceFace = TopoDS::Face(source.faces(sourceIndex));
        const TopoDS_Shape workingFace =
            derivation.exactShapes.mapped(sourceFace);
        if (workingFace.IsNull() || workingFace.ShapeType() != TopAbs_FACE) {
            return false;
        }
        TopoDS_Shape mapped = workingFace;
        if (sewer.IsModified(workingFace)) {
            mapped = sewer.Modified(workingFace);
        }
        if (mapped.IsNull() || mapped.ShapeType() != TopAbs_FACE ||
            !sewedFaces.Contains(mapped)) {
            return false;
        }
        sewHistory->AddModified(sourceFace, mapped);
        derivation.exactShapes.rebind(sourceFace, mapped);

        // Pair face-local wires by iterator order after the face rebind.
        std::vector<TopoDS_Shape> sourceWires;
        std::vector<TopoDS_Shape> mappedWires;
        for (TopoDS_Iterator it(sourceFace, false, false); it.More();
             it.Next()) {
            if (it.Value().ShapeType() == TopAbs_WIRE) {
                sourceWires.push_back(it.Value());
            }
        }
        for (TopoDS_Iterator it(mapped, false, false); it.More(); it.Next()) {
            if (it.Value().ShapeType() == TopAbs_WIRE) {
                mappedWires.push_back(it.Value());
            }
        }
        if (sourceWires.size() != mappedWires.size()) return false;
        for (std::size_t wireIndex = 0; wireIndex < sourceWires.size();
             ++wireIndex) {
            // Wires are not a BRepTools_History supported family; exact-shape
            // rebind alone feeds shapesCorrespond for topology matching.
            derivation.exactShapes.rebind(sourceWires[wireIndex],
                                          mappedWires[wireIndex]);
        }
    }

    ShapeMap sewedEdges;
    TopExp::MapShapes(sewed, TopAbs_EDGE, sewedEdges);
    ShapeMap sewedVertices;
    TopExp::MapShapes(sewed, TopAbs_VERTEX, sewedVertices);

    auto rebindEdgeAndVertices = [&](const TopoDS_Edge& sourceEdge,
                                     const TopoDS_Shape& targetEdge) {
        if (targetEdge.IsNull() || targetEdge.ShapeType() != TopAbs_EDGE) {
            return;
        }
        sewHistory->AddModified(sourceEdge, targetEdge);
        derivation.exactShapes.rebind(sourceEdge, targetEdge);
        TopoDS_Vertex sourceV1;
        TopoDS_Vertex sourceV2;
        TopExp::Vertices(sourceEdge, sourceV1, sourceV2);
        TopoDS_Vertex mappedV1;
        TopoDS_Vertex mappedV2;
        TopExp::Vertices(TopoDS::Edge(targetEdge), mappedV1, mappedV2);
        auto rebindVertex = [&](const TopoDS_Vertex& sourceVertex,
                                const TopoDS_Vertex& preferred,
                                const TopoDS_Vertex& alternate) {
            if (sourceVertex.IsNull()) return;
            TopoDS_Vertex target = preferred;
            if (target.IsNull() ||
                (!alternate.IsNull() &&
                 BRep_Tool::Pnt(sourceVertex).Distance(
                     BRep_Tool::Pnt(preferred)) >
                     BRep_Tool::Pnt(sourceVertex).Distance(
                         BRep_Tool::Pnt(alternate)))) {
                target = alternate;
            }
            if (target.IsNull()) return;
            sewHistory->AddModified(sourceVertex, target);
            derivation.exactShapes.rebind(sourceVertex, target);
        };
        rebindVertex(sourceV1, mappedV1, mappedV2);
        rebindVertex(sourceV2, mappedV2, mappedV1);
    };

    // Contiguous couples name the free-boundary sections absorbed into each
    // shared seam. Prefer this over ModifiedSubShape alone: absorbed mates are
    // not always reported as modified subshapes, and ordinal StableIds shift.
    for (int coupleIndex = 1; coupleIndex <= sewer.NbContigousEdges();
         ++coupleIndex) {
        const TopoDS_Edge shared = sewer.ContigousEdge(coupleIndex);
        if (shared.IsNull() || !sewedEdges.Contains(shared)) continue;
        const NCollection_List<TopoDS_Shape>& sections =
            sewer.ContigousEdgeCouple(coupleIndex);
        for (NCollection_List<TopoDS_Shape>::Iterator sectionIt(sections);
             sectionIt.More(); sectionIt.Next()) {
            TopoDS_Shape section = sectionIt.Value();
            if (section.IsNull() || section.ShapeType() != TopAbs_EDGE) {
                continue;
            }
            TopoDS_Edge boundary = TopoDS::Edge(section);
            if (sewer.IsSectionBound(boundary)) {
                boundary = sewer.SectionToBoundary(boundary);
            }
            for (int sourceIndex = 1; sourceIndex <= source.edges.Extent();
                 ++sourceIndex) {
                const TopoDS_Edge sourceEdge =
                    TopoDS::Edge(source.edges(sourceIndex));
                const TopoDS_Shape workingEdge =
                    derivation.exactShapes.mapped(sourceEdge);
                if (workingEdge.IsNull()) continue;
                if (workingEdge.IsPartner(boundary) ||
                    workingEdge.IsPartner(section) ||
                    sourceEdge.IsPartner(boundary) ||
                    sourceEdge.IsPartner(section) ||
                    workingEdge.IsSame(boundary) ||
                    workingEdge.IsSame(section)) {
                    rebindEdgeAndVertices(sourceEdge, shared);
                }
            }
        }
    }

    for (int sourceIndex = 1; sourceIndex <= source.edges.Extent();
         ++sourceIndex) {
        const TopoDS_Edge sourceEdge = TopoDS::Edge(source.edges(sourceIndex));
        const TopoDS_Shape workingEdge =
            derivation.exactShapes.mapped(sourceEdge);
        if (workingEdge.IsNull() || workingEdge.ShapeType() != TopAbs_EDGE) {
            continue;
        }
        if (sewedEdges.Contains(workingEdge)) {
            rebindEdgeAndVertices(sourceEdge, workingEdge);
            continue;
        }
        TopoDS_Shape mapped = workingEdge;
        if (sewer.IsModifiedSubShape(workingEdge)) {
            mapped = sewer.ModifiedSubShape(workingEdge);
        }
        if (!mapped.IsNull() && mapped.ShapeType() == TopAbs_EDGE &&
            sewedEdges.Contains(mapped)) {
            rebindEdgeAndVertices(sourceEdge, mapped);
        }
    }

    // Any source vertex still outside the sewed body snaps to the nearest
    // sewed vertex within the sew tolerance envelope.
    {
        ShapeMap sourceVertices;
        TopExp::MapShapes(source.shape, TopAbs_VERTEX, sourceVertices);
        for (int index = 1; index <= sourceVertices.Extent(); ++index) {
            const TopoDS_Vertex sourceVertex =
                TopoDS::Vertex(sourceVertices(index));
            const TopoDS_Shape current =
                derivation.exactShapes.mapped(sourceVertex);
            if (!current.IsNull() && current.ShapeType() == TopAbs_VERTEX &&
                sewedVertices.Contains(current)) {
                continue;
            }
            const gp_Pnt sourcePoint = BRep_Tool::Pnt(sourceVertex);
            double best = std::numeric_limits<double>::infinity();
            TopoDS_Vertex bestVertex;
            for (int sewedIndex = 1; sewedIndex <= sewedVertices.Extent();
                 ++sewedIndex) {
                const TopoDS_Vertex candidate =
                    TopoDS::Vertex(sewedVertices(sewedIndex));
                const double distance =
                    sourcePoint.Distance(BRep_Tool::Pnt(candidate));
                if (distance < best) {
                    best = distance;
                    bestVertex = candidate;
                }
            }
            if (!bestVertex.IsNull() && std::isfinite(best) &&
                best <= kSewToleranceMm * 10.0) {
                sewHistory->AddModified(sourceVertex, bestVertex);
                derivation.exactShapes.rebind(sourceVertex, bestVertex);
            }
        }
    }

    const int freeAfter = countFreeEdges(sewed);
    if (!(freeAfter < freeBefore)) return false;

    if (!derivation.history.IsNull()) {
        derivation.history->Merge(*sewHistory);
    } else {
        derivation.history = sewHistory;
    }
    derivation.exactShapes.rebind(source.shape, sewed);
    derivation.shape = sewed;

    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << "bounded sewing of open face compound/shell; free edges "
           << freeBefore << " -> " << freeAfter
           << ", faces preserved=" << sourceFaces.Extent()
           << ", sew tolerance mm=" << kSewToleranceMm;
    derivation.operations.push_back(
        {"repair.sewing_one_to_one", {}, {}, detail.str()});
    return true;
}

void repairRootSolidOrientations(ConservativeWorkingDerivation& derivation,
                                 const Model& source,
                                 const Model& working) {
    if (derivation.shape.IsNull() || source.solids.Extent() == 0 ||
        source.solids.Extent() != working.solids.Extent()) {
        return;
    }
    const TopAbs_ShapeEnum rootType = derivation.shape.ShapeType();
    if (rootType != TopAbs_SOLID && rootType != TopAbs_COMPOUND &&
        rootType != TopAbs_COMPSOLID) {
        return;
    }
    if (rootType == TopAbs_SOLID && source.solids.Extent() != 1) {
        return;
    }
    // Only attempt orientation repair when the isolated working body is
    // already invalid. Valid identity imports must remain byte-stable.
    if (workingShapeIsValid(derivation.shape)) return;

    for (int solidIndex = 1; solidIndex <= source.solids.Extent();
         ++solidIndex) {
        const TopoDS_Solid sourceSolid =
            TopoDS::Solid(source.solids(solidIndex));
        const TopoDS_Shape mapped =
            derivation.exactShapes.mapped(sourceSolid);
        if (mapped.IsNull() || mapped.ShapeType() != TopAbs_SOLID) {
            continue;
        }
        const TopoDS_Solid workingSolid = TopoDS::Solid(mapped);
        if (workingShapeIsValid(workingSolid)) {
            continue;
        }
        if (applyRootSolidOrientationRepair(derivation, source, working,
                                            sourceSolid, workingSolid)) {
            continue;
        }
        TopoDS_Shell workingShell;
        int shellCount = 0;
        for (TopoDS_Iterator it(workingSolid, false, false); it.More();
             it.Next()) {
            if (it.Value().ShapeType() != TopAbs_SHELL) {
                shellCount = -1;
                break;
            }
            workingShell = TopoDS::Shell(it.Value());
            ++shellCount;
        }
        if (shellCount == 1 && !workingShell.IsNull()) {
            recordOrientationRefusal(
                derivation, classifyShellOrientationRefusal(workingShell));
        } else {
            recordOrientationRefusal(derivation,
                                     ShellOrientationRefusal::Unsupported);
        }
    }
}

}  // namespace

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

    {
        const std::string msg = "working.identity_copy.done edges=" +
            std::to_string(source.edges.Extent());
        importProgress(msg.c_str());
    }

    BRep_Builder builder;
    const int edgeCount = source.edges.Extent();
    const int paramStride = std::max(1, edgeCount / 20);
    for (int edgeIndex = 1; edgeIndex <= edgeCount; ++edgeIndex) {
        if (edgeIndex % paramStride == 0) {
            const std::string msg = "working.param_loop edge=" +
                std::to_string(edgeIndex) + "/" + std::to_string(edgeCount);
            importProgress(msg.c_str());
        }
        const TopoDS_Edge sourceEdge =
            TopoDS::Edge(source.edges(edgeIndex));
        const TopoDS_Edge workingEdge =
            TopoDS::Edge(working.edges(edgeIndex));
        if (!derivation.exactShapes.maps(sourceEdge, workingEdge)) continue;
        const std::optional<CurveOnSurfaceProof> proof =
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
             parameterizationProofDetail(*proof)});
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
             proof->sourceTolerance});
    }
    importProgress("working.param_loop.done");

    importProgress("working.validity_check.begin");
    const bool workingInitiallyValid = workingShapeIsValid(derivation.shape);
    importProgress(workingInitiallyValid
                       ? "working.validity_check.done valid=1"
                       : "working.validity_check.done valid=0");
    if (!workingInitiallyValid) {
        // Tolerance-envelope fallback. The live working shape is invalid; raise
        // only the edges whose stored curve-on-surface discrepancy exceeds the
        // source tolerance, and re-check whole-shape validity ONLY after an
        // applied raise (each raise is the only thing that can flip validity).
        // Re-checking on every skipped edge previously deep-copied and
        // BRepCheck'd the entire model per edge — O(edges x model) — which is
        // the large-model (MP9) import stall.
        const int toleranceStride = std::max(1, edgeCount / 20);
        for (int edgeIndex = 1; edgeIndex <= edgeCount; ++edgeIndex) {
            if (edgeIndex % toleranceStride == 0) {
                const std::string msg = "working.tolerance_fallback edge=" +
                    std::to_string(edgeIndex) + "/" +
                    std::to_string(edgeCount);
                importProgress(msg.c_str());
            }
            const TopoDS_Edge sourceEdge =
                TopoDS::Edge(source.edges(edgeIndex));
            const TopoDS_Edge workingEdge =
                TopoDS::Edge(working.edges(edgeIndex));
            if (!derivation.exactShapes.maps(sourceEdge, workingEdge)) {
                continue;
            }
            const std::optional<CurveOnSurfaceProof> proof =
                proveToleranceEnvelopeRaise(source, sourceEdge);
            if (!proof) continue;

            const double before = BRep_Tool::Tolerance(workingEdge);
            const double after = proof->maximumDiscrepancy;
            if (!(after > before) || !std::isfinite(after)) continue;

            // Trial the raise on a disposable deep copy first. Failed
            // UpdateEdge attempts can leave vertex-tolerance residue on the
            // live working shape even after an edge revert.
            BRepBuilderAPI_Copy probeCopy(derivation.shape, true, false);
            if (probeCopy.Shape().IsNull()) continue;
            Model probeModel = indexShape(probeCopy.Shape());
            if (probeModel.edges.Extent() < edgeIndex) continue;
            const TopoDS_Edge probeEdge =
                TopoDS::Edge(probeModel.edges(edgeIndex));
            builder.UpdateEdge(probeEdge, after);
            if (!workingShapeIsValid(probeCopy.Shape())) {
                continue;
            }

            builder.UpdateEdge(workingEdge, after);
            if (BRep_Tool::Tolerance(sourceEdge) != proof->sourceTolerance ||
                BRep_Tool::Tolerance(workingEdge) != after) {
                throw std::logic_error(
                    "bounded tolerance repair violated source/working isolation");
            }
            if (!workingShapeIsValid(derivation.shape)) {
                throw std::logic_error(
                    "tolerance probe validated but live working shape did not");
            }

            const StableId edgeId{StableIdKind::Edge,
                                  static_cast<std::uint64_t>(edgeIndex)};
            derivation.operations.push_back(
                {"repair.tolerance_envelope", {edgeId}, {edgeId},
                 toleranceProofDetail(*proof)});
            derivation.toleranceChanges.push_back(
                {edgeId, edgeId, before, after, proof->expectedPcurveUses,
                 proof->checkedPcurveUses, proof->maximumDiscrepancy});

            // An applied raise just made the live shape valid (probe proved it
            // and the live re-check above passed) — no further edges needed.
            break;
        }
        importProgress("working.tolerance_fallback.done");
    }

    // Re-index after tolerance mutations so sewing and orientation repairs
    // see current working faces/edges if the root shape stayed partner-identical.
    working = indexShape(derivation.shape);
    importProgress("working.sewing.begin");
    repairBoundedSewing(derivation, source);
    importProgress("working.sewing.done");
    working = indexShape(derivation.shape);
    importProgress("working.orientation.begin");
    repairRootSolidOrientations(derivation, source, working);
    importProgress("working.orientation.done");
    return derivation;
}

CompatibilityWorkingDerivation deriveCompatibilityWorking(
    const Model& source) {
    CompatibilityWorkingDerivation derivation;
    BRepBuilderAPI_Copy copier(source.shape, true, false);
    const TopoDS_Shape copiedRoot = copier.Shape();

    // Bind every unique source TShape through the deep copy first. Valid copies
    // keep this map and skip the historical sew/ShapeFix pass so StableId
    // occurrence ordinals stay one-to-one. Invalid copies still heal, and the
    // map is rebound only through one-to-one Modified/Generated results.
    ShapeMap sourceShapes;
    sourceShapes.Add(source.shape);
    TopExp::MapShapes(source.shape, sourceShapes);
    for (int index = 1; index <= sourceShapes.Extent(); ++index) {
        const TopoDS_Shape& sourceShape = sourceShapes(index);
        const TopoDS_Shape copied = copier.ModifiedShape(sourceShape);
        if (copied.IsNull()) continue;
        const TopoDS_Shape existing = derivation.exactShapes.mapped(sourceShape);
        if (!existing.IsNull()) {
            if (!existing.IsPartner(copied) && !existing.IsSame(copied)) {
                ++derivation.multiWaySplitCount;
            }
            continue;
        }
        derivation.exactShapes.bind(sourceShape, copied);
    }

    Handle(BRepTools_History) repairHistory;
    if (workingShapeIsValid(copiedRoot)) {
        derivation.shape = copiedRoot;
        derivation.history =
            composeCopyRepairHistory(source.shape, copier, repairHistory);
        derivation.operations.push_back({
            "repair.compatibility_pipeline", {}, {},
            "geometry-deep working copy retained without sew/ShapeFix because "
            "the copy is already BRepCheck-valid"});
        return derivation;
    }

    derivation.shape = healWithHistory(copiedRoot, repairHistory);
    derivation.history = composeCopyRepairHistory(
        source.shape, copier, repairHistory);

    ShapeMap healedFaces;
    ShapeMap healedEdges;
    ShapeMap healedVertices;
    ShapeMap healedShells;
    ShapeMap healedSolids;
    TopExp::MapShapes(derivation.shape, TopAbs_FACE, healedFaces);
    TopExp::MapShapes(derivation.shape, TopAbs_EDGE, healedEdges);
    TopExp::MapShapes(derivation.shape, TopAbs_VERTEX, healedVertices);
    TopExp::MapShapes(derivation.shape, TopAbs_SHELL, healedShells);
    TopExp::MapShapes(derivation.shape, TopAbs_SOLID, healedSolids);

    auto imageInResult = [&](const TopoDS_Shape& candidate) -> TopoDS_Shape {
        if (candidate.IsNull()) return {};
        switch (candidate.ShapeType()) {
            case TopAbs_FACE:
                if (healedFaces.Contains(candidate)) return candidate;
                for (int i = 1; i <= healedFaces.Extent(); ++i) {
                    if (healedFaces(i).IsPartner(candidate)) {
                        return healedFaces(i);
                    }
                }
                break;
            case TopAbs_EDGE:
                if (healedEdges.Contains(candidate)) return candidate;
                for (int i = 1; i <= healedEdges.Extent(); ++i) {
                    if (healedEdges(i).IsPartner(candidate)) {
                        return healedEdges(i);
                    }
                }
                break;
            case TopAbs_VERTEX:
                if (healedVertices.Contains(candidate)) return candidate;
                for (int i = 1; i <= healedVertices.Extent(); ++i) {
                    if (healedVertices(i).IsPartner(candidate)) {
                        return healedVertices(i);
                    }
                }
                break;
            case TopAbs_SHELL:
                if (healedShells.Contains(candidate)) return candidate;
                for (int i = 1; i <= healedShells.Extent(); ++i) {
                    if (healedShells(i).IsPartner(candidate)) {
                        return healedShells(i);
                    }
                }
                break;
            case TopAbs_SOLID:
                if (healedSolids.Contains(candidate)) return candidate;
                for (int i = 1; i <= healedSolids.Extent(); ++i) {
                    if (healedSolids(i).IsPartner(candidate)) {
                        return healedSolids(i);
                    }
                }
                break;
            default:
                if (derivation.shape.IsPartner(candidate) ||
                    derivation.shape.IsSame(candidate)) {
                    return derivation.shape;
                }
                break;
        }
        return {};
    };

    auto uniqueSameTypeInResult = [&](TopAbs_ShapeEnum type) -> TopoDS_Shape {
        ShapeMap map;
        TopExp::MapShapes(derivation.shape, type, map);
        if (map.Extent() == 1) return map(1);
        return {};
    };

    ExactShapeDerivationMap healedShapes;
    for (int index = 1; index <= sourceShapes.Extent(); ++index) {
        const TopoDS_Shape& sourceShape = sourceShapes(index);
        const TopoDS_Shape copied = copier.ModifiedShape(sourceShape);
        if (copied.IsNull()) continue;
        if (!repairHistory.IsNull() && repairHistory->IsRemoved(copied)) {
            continue;
        }
        TopoDS_Shape finalShape = copied;
        bool mapped = false;
        if (!repairHistory.IsNull()) {
            const auto& modified = repairHistory->Modified(copied);
            if (modified.Size() == 1) {
                finalShape = modified.First();
                mapped = true;
            } else if (modified.Size() > 1) {
                // Prefer a unique partner-identical image; otherwise keep the
                // first same-type result that still appears in the healed body.
                TopoDS_Shape chosen;
                bool ambiguous = false;
                for (const TopoDS_Shape& candidate : modified) {
                    const TopoDS_Shape inResult = imageInResult(candidate);
                    if (inResult.IsNull()) continue;
                    if (chosen.IsNull()) {
                        chosen = inResult;
                    } else if (!chosen.IsPartner(inResult)) {
                        ambiguous = true;
                        break;
                    }
                }
                if (!ambiguous && !chosen.IsNull()) {
                    finalShape = chosen;
                    mapped = true;
                } else {
                    ++derivation.multiWaySplitCount;
                    continue;
                }
            }
            if (!mapped) {
                const auto& generated = repairHistory->Generated(copied);
                if (generated.Size() == 1) {
                    finalShape = generated.First();
                    mapped = true;
                } else if (generated.Size() > 1) {
                    ++derivation.multiWaySplitCount;
                    continue;
                }
            }
        }

        TopoDS_Shape inResult = imageInResult(finalShape);
        if (inResult.IsNull() && sourceShape.ShapeType() == TopAbs_SOLID) {
            // Sewing often demotes a solid to a single shell.
            inResult = imageInResult(finalShape);
            if (inResult.IsNull()) {
                const TopoDS_Shape shell = uniqueSameTypeInResult(TopAbs_SHELL);
                if (!shell.IsNull() && healedSolids.Extent() == 0) {
                    inResult = shell;
                }
            }
        }
        if (inResult.IsNull() && sourceShape.ShapeType() == TopAbs_SHELL &&
            finalShape.ShapeType() == TopAbs_SOLID) {
            inResult = imageInResult(finalShape);
        }
        if (inResult.IsNull()) {
            inResult = imageInResult(copied);
        }
        if (inResult.IsNull() &&
            (derivation.shape.IsPartner(finalShape) ||
             derivation.shape.IsSame(finalShape) ||
             derivation.shape.IsPartner(copied))) {
            inResult = derivation.shape;
        }
        if (inResult.IsNull()) continue;
        const TopoDS_Shape existing = healedShapes.mapped(sourceShape);
        if (!existing.IsNull() && !existing.IsPartner(inResult) &&
            !existing.IsSame(inResult)) {
            ++derivation.multiWaySplitCount;
            continue;
        }
        healedShapes.rebind(sourceShape, inResult);
    }

    // Wires are not a BRepTools_History family. Pair face-local wires after
    // face images are known, matching the conservative sew rebind.
    for (int faceIndex = 1; faceIndex <= source.faces.Extent(); ++faceIndex) {
        const TopoDS_Face sourceFace = TopoDS::Face(source.faces(faceIndex));
        const TopoDS_Shape mappedFace = healedShapes.mapped(sourceFace);
        if (mappedFace.IsNull() || mappedFace.ShapeType() != TopAbs_FACE) {
            continue;
        }
        std::vector<TopoDS_Shape> sourceWires;
        std::vector<TopoDS_Shape> mappedWires;
        for (TopoDS_Iterator it(sourceFace, false, false); it.More();
             it.Next()) {
            if (it.Value().ShapeType() == TopAbs_WIRE) {
                sourceWires.push_back(it.Value());
            }
        }
        for (TopoDS_Iterator it(mappedFace, false, false); it.More();
             it.Next()) {
            if (it.Value().ShapeType() == TopAbs_WIRE) {
                mappedWires.push_back(it.Value());
            }
        }
        if (sourceWires.size() != mappedWires.size()) continue;
        for (std::size_t wireIndex = 0; wireIndex < sourceWires.size();
             ++wireIndex) {
            healedShapes.rebind(sourceWires[wireIndex], mappedWires[wireIndex]);
        }
    }

    // Single-shell demotion/rebuild: claim the healed shell from the source
    // shell (or sole solid) when history left it unbound.
    if (healedShells.Extent() == 1) {
        const TopoDS_Shape healedShell = healedShells(1);
        ShapeMap sourceShells;
        TopExp::MapShapes(source.shape, TopAbs_SHELL, sourceShells);
        if (sourceShells.Extent() == 1) {
            const TopoDS_Shape current =
                healedShapes.mapped(sourceShells(1));
            if (current.IsNull() || imageInResult(current).IsNull()) {
                healedShapes.rebind(sourceShells(1), healedShell);
            }
        } else if (source.solids.Extent() == 1) {
            const TopoDS_Shape current =
                healedShapes.mapped(source.solids(1));
            if (current.IsNull() || imageInResult(current).IsNull() ||
                current.ShapeType() != TopAbs_SHELL) {
                healedShapes.rebind(source.solids(1), healedShell);
            }
        }
    }

    // Vertex snap: any source vertex whose copy was absorbed still maps to the
    // nearest healed vertex within a heal-scale envelope.
    constexpr double kHealVertexTolMm = 1.0e-3;
    for (int index = 1; index <= sourceShapes.Extent(); ++index) {
        const TopoDS_Shape& sourceShape = sourceShapes(index);
        if (sourceShape.ShapeType() != TopAbs_VERTEX) continue;
        const TopoDS_Shape current = healedShapes.mapped(sourceShape);
        if (!current.IsNull() && !imageInResult(current).IsNull()) continue;
        const TopoDS_Vertex sourceVertex = TopoDS::Vertex(sourceShape);
        const gp_Pnt sourcePoint = BRep_Tool::Pnt(sourceVertex);
        double best = std::numeric_limits<double>::infinity();
        TopoDS_Vertex bestVertex;
        for (int healedIndex = 1; healedIndex <= healedVertices.Extent();
             ++healedIndex) {
            const TopoDS_Vertex candidate =
                TopoDS::Vertex(healedVertices(healedIndex));
            const double distance =
                sourcePoint.Distance(BRep_Tool::Pnt(candidate));
            if (distance < best) {
                best = distance;
                bestVertex = candidate;
            }
        }
        if (!bestVertex.IsNull() && std::isfinite(best) &&
            best <= kHealVertexTolMm) {
            healedShapes.rebind(sourceShape, bestVertex);
        }
    }

    derivation.exactShapes = std::move(healedShapes);
    if (derivation.multiWaySplitCount > 0) {
        derivation.namedRefusals.push_back(makeNamedRefusal(
            "import.heal.multi_way_split",
            "compatibility heal produced a multi-way Modified/Generated image "
            "without a unique partner correspondence"));
    }
    derivation.operations.push_back({
        "repair.compatibility_pipeline", {}, {},
        "historical Weft healing pipeline applied to a geometry-deep working "
        "copy after immutable source capture"});
    return derivation;
}

}  // namespace weft::secure_detail
