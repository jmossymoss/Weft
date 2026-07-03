#include "weft/remap.hpp"

#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <vector>

namespace weft {

namespace {

struct FaceSig {
    SurfaceType type = SurfaceType::Other;
    double area = 0.0;
    double radius = 0.0;
    gp_Pnt centroid;
};

struct EdgeSig {
    double length = 0.0;
    gp_Pnt center;
};

std::vector<FaceSig> faceSignatures(const Model& model,
                                    const Analysis& analysis) {
    std::vector<FaceSig> sigs(model.faceCount() + 1);
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(TopoDS::Face(model.faces(fid)), props);
        sigs[fid].area = props.Mass();
        sigs[fid].centroid = props.CentreOfMass();
        sigs[fid].type = analysis.faces[fid - 1].type;
        sigs[fid].radius = analysis.faces[fid - 1].radius;
    }
    return sigs;
}

std::vector<EdgeSig> edgeSignatures(const Model& model) {
    std::vector<EdgeSig> sigs(model.edgeCount() + 1);
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        GProp_GProps props;
        BRepGProp::LinearProperties(TopoDS::Edge(model.edges(eid)), props);
        sigs[eid].length = props.Mass();
        sigs[eid].center = props.CentreOfMass();
    }
    return sigs;
}

double modelScale(const Model& model) {
    Bnd_Box box;
    BRepBndLib::Add(model.shape, box);
    if (box.IsVoid()) return 1.0;
    double x0, y0, z0, x1, y1, z1;
    box.Get(x0, y0, z0, x1, y1, z1);
    double d = std::sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) +
                         (z1 - z0) * (z1 - z0));
    return d > 1e-12 ? d : 1.0;
}

double relDiff(double a, double b) {
    double m = std::max(std::abs(a), std::abs(b));
    return m > 1e-12 ? std::abs(a - b) / m : 0.0;
}

// Greedy unique best-first assignment between two signature sets. `score`
// returns the match cost or a negative value to reject the pair outright;
// ties prefer identical ids (unchanged re-exports map to identity), then
// the smallest id shift (duplicate features — twin bores — stay in file
// order instead of swapping arbitrarily).
std::map<int, int> assign(int oldCount, int newCount,
                          const std::function<double(int, int)>& score) {
    struct Cand {
        double s;
        int oldId, newId;
    };
    std::vector<Cand> cands;
    for (int o = 1; o <= oldCount; ++o) {
        for (int n = 1; n <= newCount; ++n) {
            double s = score(o, n);
            if (s < 0) continue;
            s += (o == n ? 0.0 : 1e-6) + 1e-9 * std::abs(o - n);
            cands.push_back({s, o, n});
        }
    }
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.s < b.s; });
    std::map<int, int> map;
    std::set<int> usedNew;
    for (const Cand& c : cands) {
        if (map.count(c.oldId) || usedNew.count(c.newId)) continue;
        map[c.oldId] = c.newId;
        usedNew.insert(c.newId);
    }
    return map;
}

}  // namespace

Recipe remapRecipe(const Recipe& recipe, const Model& oldModel,
                   const Analysis& oldAnalysis, const Model& newModel,
                   const Analysis& newAnalysis, RemapReport* report) {
    const std::vector<FaceSig> oldF = faceSignatures(oldModel, oldAnalysis);
    const std::vector<FaceSig> newF = faceSignatures(newModel, newAnalysis);
    const std::vector<EdgeSig> oldE = edgeSignatures(oldModel);
    const std::vector<EdgeSig> newE = edgeSignatures(newModel);
    const double scale = modelScale(newModel);

    // Face match: same surface type, centroid within a fraction of the
    // model, area in the same ballpark (an edit next door shifts both a
    // little — a different feature shifts them a lot).
    std::map<int, int> faceMap = assign(
        oldModel.faceCount(), newModel.faceCount(), [&](int o, int n) {
            const FaceSig& a = oldF[o];
            const FaceSig& b = newF[n];
            if (a.type != b.type) return -1.0;
            double centroid = a.centroid.Distance(b.centroid) / scale;
            double area = relDiff(a.area, b.area);
            double radius = relDiff(a.radius, b.radius);
            if (centroid > 0.25 || area > 0.5 || radius > 0.25) return -1.0;
            return centroid + area + radius;
        });
    std::map<int, int> edgeMap = assign(
        oldModel.edgeCount(), newModel.edgeCount(), [&](int o, int n) {
            const EdgeSig& a = oldE[o];
            const EdgeSig& b = newE[n];
            double center = a.center.Distance(b.center) / scale;
            double length = relDiff(a.length, b.length);
            if (center > 0.2 || length > 0.5) return -1.0;
            return center + length;
        });

    auto faceFor = [&](int fid) {
        auto it = faceMap.find(fid);
        return it == faceMap.end() ? 0 : it->second;
    };
    auto edgeFor = [&](int eid) {
        auto it = edgeMap.find(eid);
        return it == edgeMap.end() ? 0 : it->second;
    };

    Recipe out;
    out.settings = recipe.settings;  // defaults + global knobs carry over
    out.settings.perFace.clear();
    out.settings.perEdge.clear();
    int facesDropped = 0, edgesDropped = 0, opsDropped = 0;
    for (const auto& [fid, s] : recipe.settings.perFace) {
        if (int n = faceFor(fid)) out.settings.perFace[n] = s;
        else ++facesDropped;
    }
    for (const auto& [eid, count] : recipe.settings.perEdge) {
        if (int n = edgeFor(eid)) out.settings.perEdge[n] = count;
        else ++edgesDropped;
    }
    for (ManualOp op : recipe.ops) {
        if (op.kind == ManualOp::Kind::Bridge) {
            int a = edgeFor(op.edgeA), b = edgeFor(op.edgeB);
            if (!a || !b) {
                ++opsDropped;
                continue;
            }
            op.edgeA = a;
            op.edgeB = b;
        } else if (op.kind == ManualOp::Kind::FillLoop) {
            int a = edgeFor(op.edgeA);
            if (!a) {
                ++opsDropped;
                continue;
            }
            op.edgeA = a;
        } else {  // LoopInsert / NudgeVertex anchor to a face
            int n = faceFor(op.faceId);
            if (!n) {
                ++opsDropped;
                continue;
            }
            op.faceId = n;
        }
        out.ops.push_back(op);
    }

    if (report) {
        report->faceMap = std::move(faceMap);
        report->edgeMap = std::move(edgeMap);
        report->facesDropped = facesDropped;
        report->edgesDropped = edgesDropped;
        report->opsDropped = opsDropped;
    }
    return out;
}

}  // namespace weft
