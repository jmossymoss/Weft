#include "weft/topology_signature.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace weft {
namespace {

std::string basenameLabel(const std::string& path) {
    if (path.empty()) return {};
    const auto slash = path.find_last_of("/\\");
    if (slash == std::string::npos) return path;
    return path.substr(slash + 1);
}

std::string formatDouble(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.9g", v);
    return buf;
}

// Stable 64-bit FNV-1a over bytes; hex-encoded.
std::string fnv1aHex(const std::string& data) {
    uint64_t h = 14695981039346656037ull;
    for (unsigned char c : data) {
        h ^= c;
        h *= 1099511628211ull;
    }
    char buf[17];
    std::snprintf(buf, sizeof buf, "%016llx",
                  static_cast<unsigned long long>(h));
    return buf;
}

size_t countFolded(const Model& model, const PolyMesh& mesh) {
    const auto mask = foldedPolys(model, mesh);
    return static_cast<size_t>(
        std::count(mask.begin(), mask.end(), uint8_t{1}));
}

std::string joinIds(const std::vector<int>& ids) {
    std::string out;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) out += ',';
        out += std::to_string(ids[i]);
    }
    return out;
}

void appendKv(std::ostringstream& oss, const std::string& key,
              const std::string& value) {
    oss << key << '=' << value << '\n';
}

void appendKv(std::ostringstream& oss, const std::string& key, size_t value) {
    oss << key << '=' << value << '\n';
}

void appendKv(std::ostringstream& oss, const std::string& key, int value) {
    oss << key << '=' << value << '\n';
}

}  // namespace

std::string formatTopologySignature(const PolyMesh& mesh, const Model& model,
                                    const GenerationReport& report,
                                    const ValidationReport& validation,
                                    const TopologySignatureInfo& info) {
    std::ostringstream oss;
    oss << "# weft-topology-signature 1\n";
    appendKv(oss, "schema", kTopologySignatureSchema);

    appendKv(oss, "vertices", mesh.vertexCount());
    appendKv(oss, "polygons", mesh.polygonCount());
    appendKv(oss, "quads", mesh.countQuads());
    appendKv(oss, "tris", mesh.countTris());
    appendKv(oss, "ngons", mesh.countNgons());

    appendKv(oss, "open_edges", validation.openEdges);
    appendKv(oss, "non_manifold_edges", validation.nonManifoldEdges);
    appendKv(oss, "winding_conflicts", validation.windingConflicts);
    appendKv(oss, "degenerate_polygons", validation.degeneratePolygons);
    appendKv(oss, "sliver_polygons", validation.sliverPolygons);
    appendKv(oss, "folded_polygons", countFolded(model, mesh));
    appendKv(oss, "watertight", validation.watertight() ? 1 : 0);

    std::vector<int> rawFaces, emptyFaces, floorFaces;
    for (const auto& [fid, how] : report.faceBuild) {
        if (how == 1) rawFaces.push_back(fid);
        else if (how == -1) emptyFaces.push_back(fid);
        else if (how == 2) floorFaces.push_back(fid);
    }
    std::sort(rawFaces.begin(), rawFaces.end());
    std::sort(emptyFaces.begin(), emptyFaces.end());
    std::sort(floorFaces.begin(), floorFaces.end());
    appendKv(oss, "raw_count", rawFaces.size());
    appendKv(oss, "empty_count", emptyFaces.size());
    appendKv(oss, "floor_count", floorFaces.size());
    appendKv(oss, "raw_faces", joinIds(rawFaces));
    appendKv(oss, "empty_faces", joinIds(emptyFaces));
    appendKv(oss, "floor_faces", joinIds(floorFaces));

    std::map<std::string, int> kindCounts;
    for (const auto& [fid, kind] : report.faceMesher) {
        ++kindCounts[mesherKindName(kind)];
    }
    for (const auto& [name, count] : kindCounts) {
        appendKv(oss, "kind." + name, count);
    }

    // Per-face routing + build status (sorted by face id).
    std::vector<int> faceIds;
    faceIds.reserve(report.faceMesher.size());
    for (const auto& [fid, _] : report.faceMesher) faceIds.push_back(fid);
    std::sort(faceIds.begin(), faceIds.end());
    for (int fid : faceIds) {
        const auto kit = report.faceMesher.find(fid);
        if (kit == report.faceMesher.end()) continue;
        appendKv(oss, "face." + std::to_string(fid) + ".kind",
                 mesherKindName(kit->second));
        int build = 0;
        const auto bit = report.faceBuild.find(fid);
        if (bit != report.faceBuild.end()) build = bit->second;
        appendKv(oss, "face." + std::to_string(fid) + ".build", build);
        const auto fcit = report.faceFeatureClass.find(fid);
        if (fcit != report.faceFeatureClass.end()) {
            appendKv(oss, "face." + std::to_string(fid) + ".feature",
                     featureClassName(fcit->second));
        }
        const auto chit = report.faceChartKind.find(fid);
        if (chit != report.faceChartKind.end()) {
            appendKv(oss, "face." + std::to_string(fid) + ".chart",
                     chartKindName(chit->second));
        }
        const auto cit = report.faceBuildCause.find(fid);
        if (cit != report.faceBuildCause.end() && !cit->second.empty() &&
            build != 0) {
            appendKv(oss, "face." + std::to_string(fid) + ".cause",
                     cit->second);
        }
    }

    // Density-matched edge divisions (sorted).
    for (const auto& [eid, div] : report.edgeDivisions) {
        appendKv(oss, "edge." + std::to_string(eid) + ".div", div);
    }

    // Anchored-vertex summary: counts + quantized UV hash (not raw floats).
    size_t anchored = 0;
    size_t unanchored = 0;
    std::map<int, size_t> faceHist;
    std::vector<std::string> qTokens;
    qTokens.reserve(mesh.anchors.size());
    const double tol = kTopologySignatureAnchorTol;
    for (const auto& a : mesh.anchors) {
        if (a.faceId <= 0) {
            ++unanchored;
            continue;
        }
        ++anchored;
        ++faceHist[a.faceId];
        const long long qu =
            static_cast<long long>(std::llround(a.u / tol));
        const long long qv =
            static_cast<long long>(std::llround(a.v / tol));
        qTokens.push_back(std::to_string(a.faceId) + ':' +
                          std::to_string(qu) + ':' + std::to_string(qv));
    }
    std::sort(qTokens.begin(), qTokens.end());
    std::string hist;
    for (const auto& [fid, n] : faceHist) {
        if (!hist.empty()) hist += ',';
        hist += std::to_string(fid) + ':' + std::to_string(n);
    }
    std::string qcat;
    for (size_t i = 0; i < qTokens.size(); ++i) {
        if (i) qcat += ';';
        qcat += qTokens[i];
    }
    appendKv(oss, "anchors.total", mesh.anchors.size());
    appendKv(oss, "anchors.anchored", anchored);
    appendKv(oss, "anchors.unanchored", unanchored);
    appendKv(oss, "anchors.face_hist", hist);
    appendKv(oss, "anchors.uv_tol", formatDouble(tol));
    appendKv(oss, "anchors.uv_qhash", fnv1aHex(qcat));

    // Informational (ignored by policy compare).
    const std::string label = basenameLabel(info.inputLabel);
    if (!label.empty()) appendKv(oss, "info.input", label);
    appendKv(oss, "info.open_on_input_boundary",
             validation.openEdgesOnInputBoundary);
    appendKv(oss, "info.input_boundary_edges", validation.inputBoundaryEdges);
    appendKv(oss, "info.input_non_manifold_edges",
             validation.inputNonManifoldEdges);
    if (validation.deviationSamples) {
        appendKv(oss, "info.max_deviation",
                 formatDouble(validation.maxDeviation));
        appendKv(oss, "info.mean_deviation",
                 formatDouble(validation.meanDeviation));
        appendKv(oss, "info.deviation_samples", validation.deviationSamples);
    }
    appendKv(oss, "info.note",
             "byte-identical OBJ floats not required; "
             "anchors compared via quantized uv_qhash");

    return oss.str();
}

std::vector<std::string> topologySignaturePolicyLines(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') continue;
        if (line.rfind("info.", 0) == 0) continue;
        // Normalize CRLF.
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines.push_back(line);
    }
    std::sort(lines.begin(), lines.end());
    return lines;
}

bool topologySignaturesEqual(const std::string& a, const std::string& b,
                             std::string* diffOut) {
    const auto la = topologySignaturePolicyLines(a);
    const auto lb = topologySignaturePolicyLines(b);
    if (la == lb) return true;
    if (!diffOut) return false;

    std::ostringstream diff;
    diff << "topology signature policy mismatch\n";
    size_t i = 0, j = 0;
    size_t shown = 0;
    constexpr size_t kMax = 32;
    while ((i < la.size() || j < lb.size()) && shown < kMax) {
        if (i < la.size() && j < lb.size() && la[i] == lb[j]) {
            ++i;
            ++j;
            continue;
        }
        if (j >= lb.size() || (i < la.size() && la[i] < lb[j])) {
            diff << "- " << la[i++] << '\n';
            ++shown;
        } else if (i >= la.size() || (j < lb.size() && lb[j] < la[i])) {
            diff << "+ " << lb[j++] << '\n';
            ++shown;
        } else {
            diff << "- " << la[i++] << '\n';
            diff << "+ " << lb[j++] << '\n';
            shown += 2;
        }
    }
    if (i < la.size() || j < lb.size()) {
        diff << "... (" << (la.size() - i) << " remaining left, "
             << (lb.size() - j) << " remaining right)\n";
    }
    *diffOut = diff.str();
    return false;
}

}  // namespace weft
