#include "weft/meshers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace weft {

std::vector<double> clusteredParams(int divisions, double hold) {
    const int count = std::max(1, divisions);
    hold = std::min(0.95, std::max(0.0, hold));
    std::vector<double> params(count + 1);
    for (int i = 0; i <= count; ++i) {
        const double x = double(i) / count;
        // Monotonic for hold < 1: slope 1-hold at the ends, 1+hold mid-span,
        // so intervals shrink near the creases and grow in the middle.
        params[i] =
            x - hold * std::sin(2.0 * M_PI * x) / (2.0 * M_PI);
    }
    params.front() = 0.0;
    params.back() = 1.0;
    return params;
}

const char* mesherKindName(MesherKind kind) {
    switch (kind) {
        case MesherKind::RevolutionGrid: return "revolution-grid";
        case MesherKind::DiskCap: return "disk-cap";
        case MesherKind::PlanarGrid: return "parametric-grid";
        case MesherKind::CoonsGrid: return "coons-grid";
        case MesherKind::RingJunction: return "ring-junction";
        case MesherKind::QuadDominant: return "quad-dominant";
        case MesherKind::MinimalNGon: return "minimal-ngon";
        case MesherKind::Fallback: return "contract-floor";
        case MesherKind::AnnulusRing: return "annulus-ring";
        case MesherKind::PlateWeb: return "plate-web";
        case MesherKind::QuadFill: return "quad-fill";
        case MesherKind::RailLadder: return "rail-ladder";
        case MesherKind::RibbonSweep: return "ribbon-sweep";
        case MesherKind::DomeCap: return "dome-cap";
    }
    return "fallback-tri";
}

const char* faceBuildClassName(FaceBuildClass c) {
    switch (c) {
        case FaceBuildClass::Built: return "built";
        case FaceBuildClass::PlannedFloor: return "planned-floor";
        case FaceBuildClass::MesherFailed: return "mesher-failed";
        case FaceBuildClass::BorderContract: return "border-contract";
        case FaceBuildClass::FoldHeal: return "fold-heal";
        case FaceBuildClass::SelfCheck: return "self-check";
        case FaceBuildClass::DensityOverride: return "density-override";
        case FaceBuildClass::Raw: return "raw";
        case FaceBuildClass::Empty: return "empty";
        case FaceBuildClass::Unknown: return "unknown";
    }
    return "unknown";
}

FaceBuildClass classifyFaceBuild(int build, const std::string& cause) {
    if (build == 0) return FaceBuildClass::Built;
    if (build == 1) return FaceBuildClass::Raw;
    if (build == -1) return FaceBuildClass::Empty;
    auto has = [&](const char* needle) {
        return cause.find(needle) != std::string::npos;
    };
    // Order matters: an override demote also mentions "contract floor", and a
    // fold self-heal is a fold class even though it lands on the same floor.
    if (has("planned contract floor")) return FaceBuildClass::PlannedFloor;
    if (has("radial override")) return FaceBuildClass::DensityOverride;
    if (has("fold")) return FaceBuildClass::FoldHeal;
    if (has("self-check")) return FaceBuildClass::SelfCheck;
    if (has("border contract")) return FaceBuildClass::BorderContract;
    if (has("sphere fold")) return FaceBuildClass::FoldHeal;
    if (has("contract floor unavailable") || has("threw")) {
        return FaceBuildClass::MesherFailed;
    }
    if (has("failed")) return FaceBuildClass::MesherFailed;
    return FaceBuildClass::Unknown;
}

StructureSummary summarizeStructure(const GenerationReport& report) {
    StructureSummary s;
    for (const auto& [fid, how] : report.faceBuild) {
        ++s.total;
        std::string cause;
        auto cit = report.faceBuildCause.find(fid);
        if (cit != report.faceBuildCause.end()) cause = cit->second;
        const FaceBuildClass c = classifyFaceBuild(how, cause);
        ++s.byClass[c];
        switch (c) {
            case FaceBuildClass::Built:
                ++s.structured;
                break;
            case FaceBuildClass::PlannedFloor:
                ++s.plannedFloor;
                break;
            case FaceBuildClass::Raw:
                ++s.raw;
                break;
            case FaceBuildClass::Empty:
                ++s.empty;
                break;
            default:
                ++s.failedFloor;
                ++s.failedByCause[cause.empty() ? "(no cause)" : cause];
                break;
        }
    }
    return s;
}

std::string formatStructure(const GenerationReport& report) {
    const StructureSummary s = summarizeStructure(report);
    if (!s.total) return {};
    char line[256];
    std::snprintf(line, sizeof(line),
                  "  structure: faces=%d structured=%d planned-floor=%d "
                  "failed-floor=%d raw=%d empty=%d retention=%.4f\n",
                  s.total, s.structured, s.plannedFloor, s.failedFloor, s.raw,
                  s.empty, s.retention());
    std::string out = line;
    if (!s.failedByCause.empty()) {
        out += "    failed-floor by cause:";
        for (const auto& [cause, n] : s.failedByCause) {
            std::snprintf(line, sizeof(line), " %s=%d", cause.c_str(), n);
            out += line;
        }
        out += '\n';
    }
    return out;
}

std::string formatBuildDemotions(const GenerationReport& report) {
    int floor = 0, raw = 0, empty = 0;
    std::vector<int> floorFaces, rawFaces, emptyFaces;
    for (const auto& [fid, how] : report.faceBuild) {
        if (how == 2) {
            ++floor;
            floorFaces.push_back(fid);
        } else if (how == 1) {
            ++raw;
            rawFaces.push_back(fid);
        } else if (how == -1) {
            ++empty;
            emptyFaces.push_back(fid);
        }
    }
    if (!floor && !raw && !empty) return {};

    std::string out;
    char line[256];
    std::snprintf(line, sizeof(line),
                  "  demoted: %d to contract floor, %d to raw "
                  "triangulation, %d emitted nothing\n",
                  floor, raw, empty);
    out += line;

    auto appendFaces = [&](const char* label, const std::vector<int>& ids) {
        if (ids.empty()) return;
        out += "    ";
        out += label;
        out += ":";
        for (int fid : ids) {
            auto cit = report.faceBuildCause.find(fid);
            if (cit != report.faceBuildCause.end() && !cit->second.empty()) {
                std::snprintf(line, sizeof(line), " %d(%s)", fid,
                              cit->second.c_str());
            } else {
                std::snprintf(line, sizeof(line), " %d", fid);
            }
            out += line;
        }
        out += '\n';
    };
    appendFaces("floor face ids", floorFaces);
    appendFaces("raw face ids", rawFaces);
    appendFaces("empty face ids", emptyFaces);
    return out;
}

std::string formatDensityOwnership(const GenerationReport& report) {
    if (report.edgeDivisions.empty() && report.densityConflicts.empty()) {
        return {};
    }
    std::string out;
    char line[256];
    std::snprintf(line, sizeof(line), "  density-matched edges: %zu",
                  report.edgeDivisions.size());
    out += line;
    if (!report.edgeDivisions.empty()) {
        out += ':';
        for (const auto& [eid, div] : report.edgeDivisions) {
            auto oit = report.edgeDivisionOwner.find(eid);
            if (oit != report.edgeDivisionOwner.end() &&
                !oit->second.empty()) {
                std::snprintf(line, sizeof(line), " #%d=%d(%s)", eid, div,
                              oit->second.c_str());
            } else {
                std::snprintf(line, sizeof(line), " #%d=%d", eid, div);
            }
            out += line;
        }
    }
    out += '\n';

    if (!report.densityConflicts.empty()) {
        std::snprintf(line, sizeof(line),
                      "  density ownership conflicts: %zu\n",
                      report.densityConflicts.size());
        out += line;
        for (const auto& c : report.densityConflicts) {
            std::snprintf(line, sizeof(line), "    edge #%d=%d (%s)",
                          c.edgeId, c.solved, c.reason.c_str());
            out += line;
            if (!c.faceProposals.empty()) {
                out += " proposals:";
                for (const auto& [fid, n] : c.faceProposals) {
                    if (fid == 0) {
                        std::snprintf(line, sizeof(line), " pin=%d", n);
                    } else {
                        std::snprintf(line, sizeof(line), " f%d=%d", fid, n);
                    }
                    out += line;
                }
            }
            out += '\n';
        }
    }
    return out;
}

}  // namespace weft
