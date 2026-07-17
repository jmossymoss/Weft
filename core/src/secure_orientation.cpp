#include "secure_core_internal.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace weft::secure_detail {
namespace {

constexpr int kUnassignedParity = 0;

struct FaceUse {
    // Stored child value inside the shell TShape list: relative orientation
    // and relative location, exactly as BRep_Builder keeps it.
    TopoDS_Shape storedFace;
    // The same face with orientation and location composed from the indexed
    // solid, used for exact edge identity and the polarity trial.
    TopoDS_Shape composedFace;
    int faceMapIndex = 0;
    // Current effective orientation sign within the solid frame: +1 when the
    // composed use is FORWARD, -1 when REVERSED.
    int currentSign = 0;
    // Target effective orientation sign selected by the parity/polarity
    // proof; 0 until assigned.
    int targetSign = kUnassignedParity;
};

struct EdgeUse {
    std::size_t faceUse = 0;
    // Direction of this edge traversal with the owning face taken FORWARD:
    // +1 for FORWARD, -1 for REVERSED.
    int forwardDirection = 0;
};

struct ParityConstraint {
    std::size_t firstFaceUse = 0;
    std::size_t secondFaceUse = 0;
    // +1 when the two target signs must be equal, -1 when opposite.
    int relation = 0;
};

struct ShellAnalysis {
    TopoDS_Shape storedShell;
    TopoDS_Shape composedShell;
    std::vector<FaceUse> faceUses;
    std::vector<ParityConstraint> constraints;
    std::size_t distinctManifoldEdges = 0;
    bool parityViolated = false;
    std::optional<std::string> scopeRefusalCode;
};

struct PolarityMeasurement {
    double signedVolume = 0.0;
    bool infinitePointOutside = false;
    bool nativeValid = false;
};

int orientationSign(TopAbs_Orientation orientation) {
    if (orientation == TopAbs_FORWARD) return 1;
    if (orientation == TopAbs_REVERSED) return -1;
    return 0;
}

using EdgeUseMap =
    std::unordered_map<TopoDS_Shape, std::vector<EdgeUse>,
                       TopTools_ShapeMapHasher, TopTools_ShapeMapHasher>;

// Scans one shell's face uses and shared-edge adjacency into `analysis`,
// which must already carry composedShell/storedShell. Returns early with a
// scope refusal code when the shell leaves the bounded repair envelope.
void scanShellParity(const Model& source, ShellAnalysis& analysis) {
    EdgeUseMap edgeUses;
    ShapeMap distinctFaces;
    TopoDS_Iterator composedFaces(analysis.composedShell, true, true);
    TopoDS_Iterator storedFaces(analysis.storedShell, false, false);
    for (; composedFaces.More() && storedFaces.More();
         composedFaces.Next(), storedFaces.Next()) {
        const TopoDS_Shape& composedFace = composedFaces.Value();
        if (composedFace.ShapeType() != TopAbs_FACE) {
            analysis.scopeRefusalCode =
                "repair.orientation.non_face_shell_child";
            return;
        }
        FaceUse use;
        use.storedFace = storedFaces.Value();
        use.composedFace = composedFace;
        use.faceMapIndex = source.faces.FindIndex(composedFace);
        use.currentSign = orientationSign(composedFace.Orientation());
        if (use.faceMapIndex <= 0 || use.currentSign == 0) {
            analysis.scopeRefusalCode =
                "repair.orientation.face_occurrence_not_two_sided";
            return;
        }
        if (distinctFaces.Contains(composedFace)) {
            analysis.scopeRefusalCode =
                "repair.orientation.repeated_face_occurrence";
            return;
        }
        distinctFaces.Add(composedFace);
        const std::size_t faceUseIndex = analysis.faceUses.size();
        TopoDS_Shape forwardFace = composedFace;
        forwardFace.Orientation(TopAbs_FORWARD);
        for (TopExp_Explorer edges(forwardFace, TopAbs_EDGE); edges.More();
             edges.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(edges.Current());
            if (BRep_Tool::Degenerated(edge)) continue;
            const int direction = orientationSign(edge.Orientation());
            if (direction == 0) {
                analysis.scopeRefusalCode =
                    "repair.orientation.edge_use_not_two_sided";
                return;
            }
            edgeUses[edge].push_back({faceUseIndex, direction});
        }
        analysis.faceUses.push_back(std::move(use));
    }
    if (composedFaces.More() || storedFaces.More()) {
        throw std::logic_error(
            "cumulative and stored shell traversals disagree");
    }

    // The hash iteration order over edges is not deterministic, so record
    // manifold defects as counts and pick the named refusal by fixed
    // priority afterwards.
    std::size_t openEdges = 0;
    std::size_t nonmanifoldEdges = 0;
    for (const auto& [edge, uses] : edgeUses) {
        ++analysis.distinctManifoldEdges;
        if (uses.size() == 1) {
            ++openEdges;
            continue;
        }
        if (uses.size() > 2) {
            ++nonmanifoldEdges;
            continue;
        }
        const EdgeUse& first = uses.front();
        const EdgeUse& second = uses.back();
        const int firstCurrent =
            first.forwardDirection *
            analysis.faceUses[first.faceUse].currentSign;
        const int secondCurrent =
            second.forwardDirection *
            analysis.faceUses[second.faceUse].currentSign;
        if (firstCurrent == secondCurrent) analysis.parityViolated = true;
        if (first.faceUse == second.faceUse) continue;  // seam: no constraint
        analysis.constraints.push_back(
            {first.faceUse, second.faceUse,
             first.forwardDirection == second.forwardDirection ? -1 : 1});
    }
    if (openEdges > 0) {
        analysis.scopeRefusalCode = "repair.orientation.shell_open";
    } else if (nonmanifoldEdges > 0) {
        analysis.scopeRefusalCode =
            "repair.orientation.shell_nonmanifold_edge";
    }
}

// Collects the shell structure of one indexed solid. Returns nothing when
// the solid has no shell children at all; multi-shell solids are scanned for
// parity violations only, so a defect still refuses by name even though its
// repair is out of the bounded scope.
std::optional<ShellAnalysis> analyseShell(const Model& source,
                                          const TopoDS_Shape& solid) {
    ShellAnalysis analysis;
    std::vector<std::pair<TopoDS_Shape, TopoDS_Shape>> shells;
    TopoDS_Iterator composedChildren(solid, true, true);
    TopoDS_Iterator storedChildren(solid, false, false);
    for (; composedChildren.More() && storedChildren.More();
         composedChildren.Next(), storedChildren.Next()) {
        if (composedChildren.Value().ShapeType() != TopAbs_SHELL) {
            analysis.scopeRefusalCode =
                "repair.orientation.non_shell_solid_child";
            continue;
        }
        shells.emplace_back(composedChildren.Value(),
                            storedChildren.Value());
    }
    if (shells.empty()) return std::nullopt;
    if (shells.size() > 1) {
        for (const auto& [composedShell, storedShell] : shells) {
            if (orientationSign(composedShell.Orientation()) == 0) continue;
            ShellAnalysis scan;
            scan.composedShell = composedShell;
            scan.storedShell = storedShell;
            scanShellParity(source, scan);
            if (scan.parityViolated) analysis.parityViolated = true;
        }
        analysis.scopeRefusalCode = "repair.orientation.multi_shell_solid";
        return analysis;
    }
    analysis.composedShell = shells.front().first;
    analysis.storedShell = shells.front().second;
    if (orientationSign(solid.Orientation()) != 1) {
        analysis.scopeRefusalCode =
            "repair.orientation.solid_occurrence_not_forward";
    }
    if (orientationSign(analysis.composedShell.Orientation()) == 0) {
        analysis.scopeRefusalCode =
            "repair.orientation.shell_occurrence_not_two_sided";
        return analysis;
    }
    scanShellParity(source, analysis);
    return analysis;
}

// Two-colours the face-adjacency graph. Returns false when the constraints
// admit no consistent assignment (a non-orientable winding).
bool solveParity(ShellAnalysis& analysis, std::string& refusalCode) {
    std::vector<std::vector<std::pair<std::size_t, int>>> adjacency(
        analysis.faceUses.size());
    for (const ParityConstraint& constraint : analysis.constraints) {
        adjacency[constraint.firstFaceUse].push_back(
            {constraint.secondFaceUse, constraint.relation});
        adjacency[constraint.secondFaceUse].push_back(
            {constraint.firstFaceUse, constraint.relation});
    }
    std::deque<std::size_t> pending;
    analysis.faceUses.front().targetSign = 1;
    pending.push_back(0);
    while (!pending.empty()) {
        const std::size_t current = pending.front();
        pending.pop_front();
        const int currentSign = analysis.faceUses[current].targetSign;
        for (const auto& [neighbour, relation] : adjacency[current]) {
            const int required = currentSign * relation;
            int& neighbourSign = analysis.faceUses[neighbour].targetSign;
            if (neighbourSign == kUnassignedParity) {
                neighbourSign = required;
                pending.push_back(neighbour);
            } else if (neighbourSign != required) {
                refusalCode = "repair.orientation.non_orientable";
                return false;
            }
        }
    }
    if (std::any_of(analysis.faceUses.begin(), analysis.faceUses.end(),
                    [](const FaceUse& use) {
                        return use.targetSign == kUnassignedParity;
                    })) {
        refusalCode = "repair.orientation.adjacency_disconnected";
        return false;
    }
    return true;
}

PolarityMeasurement measurePolarity(const ShellAnalysis& analysis,
                                    int globalSign) {
    BRep_Builder builder;
    TopoDS_Shell trialShell;
    builder.MakeShell(trialShell);
    for (const FaceUse& use : analysis.faceUses) {
        TopoDS_Shape oriented = use.composedFace;
        oriented.Orientation(use.targetSign * globalSign > 0
                                 ? TopAbs_FORWARD
                                 : TopAbs_REVERSED);
        builder.Add(trialShell, oriented);
    }
    trialShell.Closed(true);
    TopoDS_Solid trialSolid;
    builder.MakeSolid(trialSolid);
    builder.Add(trialSolid, trialShell);

    PolarityMeasurement measurement;
    GProp_GProps properties;
    BRepGProp::VolumeProperties(trialSolid, properties);
    measurement.signedVolume = properties.Mass();
    BRepClass3d_SolidClassifier classifier(trialSolid);
    classifier.PerformInfinitePoint(Precision::Confusion());
    measurement.infinitePointOutside = classifier.State() == TopAbs_OUT;
    measurement.nativeValid = BRepCheck_Analyzer(trialSolid, true).IsValid();
    return measurement;
}

std::string repairDetail(const ShellOrientationRepair& repair) {
    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "restored coherent face adjacency over "
           << repair.checkedManifoldEdges << " manifold edge(s) by flipping "
           << repair.flippedFaces.size() << " of " << repair.shellFaceUses
           << " face occurrence(s)"
           << (repair.shellOccurrenceReversed
                   ? " and the shell occurrence"
                   : "")
           << "; independent polarity: signed volume="
           << repair.signedVolume << ", infinite point outside="
           << (repair.infinitePointOutside ? "true" : "false");
    return detail.str();
}

// Rebuilds one working parent's stored child list, flipping the requested
// child ordinals between FORWARD and REVERSED. TShapes, child order, flags,
// locations, geometry, and tolerances are untouched.
void flipStoredChildren(const TopoDS_Shape& workingParent,
                        const std::vector<bool>& flipAtOrdinal) {
    std::vector<TopoDS_Shape> children;
    for (TopoDS_Iterator child(workingParent, false, false); child.More();
         child.Next()) {
        children.push_back(child.Value());
    }
    if (children.size() != flipAtOrdinal.size()) {
        throw std::logic_error(
            "orientation repair child list diverged from its analysis");
    }
    // BRep_Builder re-expresses components relative to the handle's
    // orientation and location, so mutate through a FORWARD identity handle
    // and the stored child values round-trip unchanged.
    TopoDS_Shape mutableParent = workingParent;
    mutableParent.Orientation(TopAbs_FORWARD);
    mutableParent.Location(TopLoc_Location());
    const bool wasFree = mutableParent.Free();
    mutableParent.Free(true);
    BRep_Builder builder;
    for (const TopoDS_Shape& child : children) {
        builder.Remove(mutableParent, child);
    }
    for (std::size_t ordinal = 0; ordinal < children.size(); ++ordinal) {
        TopoDS_Shape adjusted = children[ordinal];
        if (flipAtOrdinal[ordinal]) {
            adjusted.Orientation(adjusted.Orientation() == TopAbs_FORWARD
                                     ? TopAbs_REVERSED
                                     : TopAbs_FORWARD);
        }
        builder.Add(mutableParent, adjusted);
    }
    mutableParent.Free(wasFree);
}

}  // namespace

void repairShellOrientations(const Model& source,
                             ConservativeWorkingDerivation& derivation) {
    // Shell definitions used by more than one indexed solid cannot be
    // repaired per occurrence, because the flip mutates the shared TShape
    // child list once for every use.
    std::unordered_map<const void*, int> shellDefinitionUses;
    for (int solidIndex = 1; solidIndex <= source.solids.Extent();
         ++solidIndex) {
        for (TopoDS_Iterator child(source.solids(solidIndex), false, false);
             child.More(); child.Next()) {
            if (child.Value().ShapeType() == TopAbs_SHELL) {
                ++shellDefinitionUses[child.Value().TShape().get()];
            }
        }
    }

    for (int solidIndex = 1; solidIndex <= source.solids.Extent();
         ++solidIndex) {
        const StableId solidId{StableIdKind::Solid,
                               static_cast<std::uint64_t>(solidIndex)};
        const TopoDS_Shape solid = source.solids(solidIndex);
        std::optional<ShellAnalysis> analysis;
        try {
            analysis = analyseShell(source, solid);
        } catch (const Standard_Failure&) {
            derivation.refusals.push_back(
                {"repair.orientation.analysis_failure", {solidId},
                 "OCCT failed while collecting shell adjacency evidence"});
            continue;
        }
        // Solids that are not even shell-structured, and shells whose
        // face adjacency is already coherent, are not repair candidates;
        // their validity remains BRepCheck's verdict.
        if (!analysis) continue;
        if (!analysis->parityViolated) continue;

        const auto refuse = [&](std::string code, std::string detail) {
            derivation.refusals.push_back(
                {std::move(code), {solidId}, std::move(detail)});
        };
        if (analysis->scopeRefusalCode) {
            refuse(*analysis->scopeRefusalCode,
                   "incoherent shell is outside the bounded conservative "
                   "orientation repair scope");
            continue;
        }
        if (shellDefinitionUses[analysis->storedShell.TShape().get()] > 1) {
            refuse("repair.orientation.shared_shell_definition",
                   "incoherent shell definition is used by more than one "
                   "solid");
            continue;
        }
        std::string parityRefusal;
        if (!solveParity(*analysis, parityRefusal)) {
            refuse(std::move(parityRefusal),
                   "no coherent two-manifold parity assignment exists");
            continue;
        }

        int globalSign = 0;
        PolarityMeasurement accepted;
        try {
            for (int candidate : {1, -1}) {
                const PolarityMeasurement measurement =
                    measurePolarity(*analysis, candidate);
                if (std::isfinite(measurement.signedVolume) &&
                    measurement.signedVolume > 0.0 &&
                    measurement.infinitePointOutside &&
                    measurement.nativeValid) {
                    globalSign = candidate;
                    accepted = measurement;
                    break;
                }
            }
        } catch (const Standard_Failure&) {
            refuse("repair.orientation.analysis_failure",
                   "OCCT failed while measuring candidate polarity");
            continue;
        }
        if (globalSign == 0) {
            refuse("repair.orientation.polarity_unproven",
                   "neither coherent assignment produced a native-valid "
                   "positive-volume solid with the infinite point outside");
            continue;
        }

        // Realize the chosen effective orientations with the fewest
        // occurrence flips: either keep the shell occurrence and flip the
        // disagreeing faces, or flip the shell occurrence and flip the
        // complement.
        std::vector<bool> flipKeepingShell(analysis->faceUses.size());
        std::size_t flipsKeepingShell = 0;
        for (std::size_t ordinal = 0; ordinal < analysis->faceUses.size();
             ++ordinal) {
            const FaceUse& use = analysis->faceUses[ordinal];
            flipKeepingShell[ordinal] =
                use.currentSign != use.targetSign * globalSign;
            if (flipKeepingShell[ordinal]) ++flipsKeepingShell;
        }
        const bool reverseShell =
            analysis->faceUses.size() - flipsKeepingShell < flipsKeepingShell;
        std::vector<bool> faceFlips = std::move(flipKeepingShell);
        if (reverseShell) faceFlips.flip();
        if (!reverseShell &&
            std::none_of(faceFlips.begin(), faceFlips.end(),
                         [](bool flip) { return flip; })) {
            refuse("repair.orientation.analysis_failure",
                   "parity violation produced an empty occurrence flip set");
            continue;
        }

        const TopoDS_Shape workingShell =
            derivation.exactShapes.mapped(analysis->storedShell);
        const TopoDS_Shape workingSolid = derivation.exactShapes.mapped(solid);
        if (workingShell.IsNull() || workingSolid.IsNull()) {
            throw std::logic_error(
                "orientation repair lost its source-to-working derivation");
        }
        flipStoredChildren(workingShell, faceFlips);
        if (reverseShell) {
            std::vector<bool> solidChildFlips;
            for (TopoDS_Iterator child(workingSolid, false, false);
                 child.More(); child.Next()) {
                solidChildFlips.push_back(
                    child.Value().IsPartner(workingShell));
            }
            flipStoredChildren(workingSolid, solidChildFlips);
        }

        ShellOrientationRepair repair;
        repair.sourceSolid = solidId;
        repair.workingSolid = solidId;
        repair.shellOccurrenceReversed = reverseShell;
        repair.shellFaceUses = analysis->faceUses.size();
        repair.expectedManifoldEdges = analysis->distinctManifoldEdges;
        repair.checkedManifoldEdges = analysis->distinctManifoldEdges;
        repair.signedVolume = accepted.signedVolume;
        repair.infinitePointOutside = accepted.infinitePointOutside;
        for (std::size_t ordinal = 0; ordinal < faceFlips.size(); ++ordinal) {
            if (!faceFlips[ordinal]) continue;
            repair.flippedFaces.push_back(
                {StableIdKind::Face,
                 static_cast<std::uint64_t>(
                     analysis->faceUses[ordinal].faceMapIndex)});
        }
        std::sort(repair.flippedFaces.begin(), repair.flippedFaces.end());

        // The immutable source must not have observed the flip, and the
        // working stored orientations must now realize exactly the proof.
        TopoDS_Iterator sourceCheck(analysis->storedShell, false, false);
        TopoDS_Iterator workingCheck(workingShell, false, false);
        for (std::size_t ordinal = 0;
             sourceCheck.More() && workingCheck.More();
             sourceCheck.Next(), workingCheck.Next(), ++ordinal) {
            const bool flipped = ordinal < faceFlips.size() &&
                                 faceFlips[ordinal];
            if ((sourceCheck.Value().Orientation() !=
                 workingCheck.Value().Orientation()) != flipped ||
                sourceCheck.Value().Orientation() !=
                    analysis->faceUses[ordinal].storedFace.Orientation()) {
                throw std::logic_error(
                    "orientation repair violated source/working isolation");
            }
        }
        if (sourceCheck.More() || workingCheck.More()) {
            throw std::logic_error(
                "orientation repair changed the shell face cardinality");
        }

        std::vector<StableId> subjects;
        subjects.push_back(solidId);
        subjects.insert(subjects.end(), repair.flippedFaces.begin(),
                        repair.flippedFaces.end());
        derivation.operations.push_back(
            {"repair.face_adjacency_orientation", subjects, subjects,
             repairDetail(repair)});
        derivation.shellOrientationRepairs.push_back(std::move(repair));
    }
}

}  // namespace weft::secure_detail
