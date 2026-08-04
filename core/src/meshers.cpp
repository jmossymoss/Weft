// Accuracy-branch generate(): Pixyz-inspired sag/angle/length tessellation.
// Structured meshers live in core/src/legacy/meshers_structured.cpp (not built).

#include "weft/meshers.hpp"
#include "weft/analysis.hpp"
#include "weft/mesh.hpp"
#include "weft/model.hpp"
#include "mesher_sampling.hpp"
#include "mesher_trace.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Surface.hxx>
#include <Precision.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <array>
#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <unordered_map>
#include <vector>

namespace weft {
namespace {

struct SamplePoint {
    gp_Pnt p;
    double t = 0.0;
};

double effectiveSag(const FaceMeshSettings& s, double modelDiagonal) {
    double sag = std::max(1e-9, s.chordTolerance);
    if (s.relativeDeviation && modelDiagonal > 0.0) {
        const double ratio = s.minSize > 0.0 ? s.minSize : 0.0003;
        sag = std::min(sag, modelDiagonal * ratio);
    }
    return sag;
}

double modelDiagonalOf(const Model& model) {
    if (model.shape.IsNull()) return 0.0;
    Bnd_Box bb;
    BRepBndLib::Add(model.shape, bb);
    if (bb.IsVoid()) return 0.0;
    double x0, y0, z0, x1, y1, z1;
    bb.Get(x0, y0, z0, x1, y1, z1);
    return gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1));
}

// Circle/cylinder chord → segment count: h = r (1 - cos(θ/2)).
int circleDivisions(double radius, double sag, double angleDeg) {
    radius = std::max(radius, 1e-9);
    sag = std::max(sag, 1e-12);
    int fromSag = 3;
    if (sag < 2.0 * radius) {
        const double ratio = std::clamp(1.0 - sag / radius, -1.0, 1.0);
        const double theta = 2.0 * std::acos(ratio);  // radians per segment
        if (theta > 1e-9) {
            fromSag = std::max(3, int(std::ceil(2.0 * M_PI / theta)));
        } else {
            fromSag = 256;
        }
    } else {
        fromSag = 3;  // sag bigger than diameter → very coarse
    }
    int fromAngle = 3;
    if (angleDeg > 0.0) {
        fromAngle = std::max(3, int(std::ceil(360.0 / angleDeg)));
    }
    return std::clamp(std::max(fromSag, fromAngle), 3, 512);
}

std::vector<SamplePoint> sampleEdgeCurve(const TopoDS_Edge& edge, double sag,
                                         double angleDeg, double maxLength) {
    std::vector<SamplePoint> out;
    if (BRep_Tool::Degenerated(edge)) return out;
    BRepAdaptor_Curve curve(edge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    if (!(last > first)) return out;

    const double angleTol =
        angleDeg > 0.0 ? angleDeg * M_PI / 180.0 : 1e9;
    int n = mesher_detail::stableDeflectionCount(curve, angleTol, sag);
    n = std::max(1, n);

    // Circles/arcs: closed-form sag/angle count is authoritative so analytic
    // UV grids and shared rim samples stay in lockstep.
    try {
        if (curve.GetType() == GeomAbs_Circle) {
            const double r = curve.Circle().Radius();
            const double span = std::abs(last - first);
            const int full = circleDivisions(r, sag, angleDeg);
            const int forSpan =
                std::max(1, int(std::ceil(full * (span / (2.0 * M_PI)))));
            n = forSpan;
        }
    } catch (...) {
    }

    auto appendParam = [&](double t) {
        SamplePoint sp;
        sp.t = t;
        try {
            sp.p = curve.Value(t);
        } catch (...) {
            return;
        }
        if (!out.empty() && maxLength > 0.0) {
            const double dist = out.back().p.Distance(sp.p);
            if (dist > maxLength * 1.001) {
                const int splits =
                    std::max(1, int(std::ceil(dist / maxLength)));
                for (int i = 1; i < splits; ++i) {
                    const double ti =
                        out.back().t + (t - out.back().t) * (double(i) / splits);
                    SamplePoint mid;
                    mid.t = ti;
                    try {
                        mid.p = curve.Value(ti);
                        out.push_back(mid);
                    } catch (...) {
                    }
                }
            }
        }
        if (out.empty() || out.back().p.Distance(sp.p) > 1e-12) {
            out.push_back(sp);
        }
    };

    for (int i = 0; i <= n; ++i) {
        appendParam(first + (last - first) * (double(i) / double(n)));
    }
    return out;
}

struct MeshBuilder {
    PolyMesh mesh;
    // Exact vertex index keyed by EdgeId sample slot for seam sharing.
    // Key: (edgeId << 32) | sampleIndex  (sampleIndex along FORWARD edge param)
    std::map<uint64_t, uint32_t> edgeSlot;

    // Spatial cell key — must NOT XOR-fold axes (that collides opposite
    // points on a circle, e.g. 45° with 225° at the same |x|=|y|).
    struct CellKey {
        int64_t x = 0, y = 0, z = 0;
        bool operator==(const CellKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };
    struct CellKeyHash {
        size_t operator()(const CellKey& k) const {
            size_t h = std::hash<int64_t>{}(k.x);
            h ^= std::hash<int64_t>{}(k.y) + 0x9e3779b97f4a7c15ULL +
                 (h << 6) + (h >> 2);
            h ^= std::hash<int64_t>{}(k.z) + 0x9e3779b97f4a7c15ULL +
                 (h << 6) + (h >> 2);
            return h;
        }
    };
    std::unordered_map<CellKey, uint32_t, CellKeyHash> weld;

    static CellKey quantize(const gp_Pnt& p, double tol) {
        const double s = 1.0 / std::max(tol, 1e-12);
        return {std::llround(p.X() * s), std::llround(p.Y() * s),
                std::llround(p.Z() * s)};
    }

    uint32_t addVertex(const gp_Pnt& p, const Anchor& a, double weldTol) {
        const CellKey key = quantize(p, weldTol);
        auto it = weld.find(key);
        if (it != weld.end()) {
            // Prefer an existing anchored vertex; keep first anchor.
            return it->second;
        }
        const uint32_t idx = uint32_t(mesh.vertices.size());
        mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
        mesh.anchors.push_back(a);
        weld.emplace(key, idx);
        return idx;
    }

    uint32_t addEdgeSample(int edgeId, int sampleIndex, const gp_Pnt& p,
                           const Anchor& a, double weldTol) {
        const uint64_t slot =
            (uint64_t(uint32_t(edgeId)) << 32) | uint32_t(sampleIndex);
        auto it = edgeSlot.find(slot);
        if (it != edgeSlot.end()) return it->second;
        const uint32_t idx = addVertex(p, a, weldTol);
        edgeSlot.emplace(slot, idx);
        return idx;
    }

    void addPolygon(std::vector<uint32_t> idxs, int faceId) {
        if (idxs.size() < 3) return;
        std::vector<uint32_t> clean;
        clean.reserve(idxs.size());
        for (uint32_t i : idxs) {
            if (clean.empty() || clean.back() != i) clean.push_back(i);
        }
        if (clean.size() >= 3 && clean.front() == clean.back()) clean.pop_back();
        if (clean.size() < 3) return;
        mesh.polygons.push_back(std::move(clean));
        mesh.polygonFaceId.push_back(faceId);
    }
};

Anchor anchorOnFace(const TopoDS_Face& face, int faceId, const gp_Pnt& p) {
    Anchor a{faceId, 0.0, 0.0};
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    if (surf.IsNull()) return a;
    try {
        GeomAPI_ProjectPointOnSurf projector(p, surf);
        if (projector.NbPoints() >= 1) {
            double u = 0, v = 0;
            projector.LowerDistanceParameters(u, v);
            a.u = u;
            a.v = v;
        }
    } catch (...) {
    }
    return a;
}

// Per-edge polyline in FORWARD parameter order (Model edge orientation).
struct EdgeSamples {
    std::vector<SamplePoint> forward;  // first→last of Model edge
};

using EdgeSampleMap = std::map<int, EdgeSamples>;

FaceMeshSettings mergeEdgeSettings(const Model& model, int edgeId,
                                   const GenerationSettings& settings,
                                   const Analysis& analysis) {
    // Tightest sag / angle / length across adjacent faces (finest wins).
    FaceMeshSettings best = settings.defaults;
    best.chordTolerance = 1e9;
    best.angleToleranceDeg = -1;
    best.maxLength = -1;
    bool any = false;
    if (edgeId >= 1 && edgeId <= int(analysis.edges.size())) {
        for (int fid : analysis.edges[edgeId - 1].faceIds) {
            const FaceMeshSettings& fs = settings.forFace(fid);
            if (fs.exclude) continue;
            any = true;
            best.chordTolerance =
                std::min(best.chordTolerance, fs.chordTolerance);
            if (fs.angleToleranceDeg > 0.0) {
                if (best.angleToleranceDeg < 0.0) {
                    best.angleToleranceDeg = fs.angleToleranceDeg;
                } else {
                    best.angleToleranceDeg =
                        std::min(best.angleToleranceDeg, fs.angleToleranceDeg);
                }
            }
            if (fs.maxLength > 0.0) {
                if (best.maxLength < 0.0) {
                    best.maxLength = fs.maxLength;
                } else {
                    best.maxLength = std::min(best.maxLength, fs.maxLength);
                }
            }
            best.relativeDeviation =
                best.relativeDeviation || fs.relativeDeviation;
            if (fs.minSize > 0.0) {
                best.minSize = best.minSize > 0.0
                                   ? std::min(best.minSize, fs.minSize)
                                   : fs.minSize;
            }
        }
    }
    if (!any) best = settings.defaults;
    (void)model;
    return best;
}

EdgeSampleMap buildEdgeSamples(const Model& model, const Analysis& analysis,
                               const GenerationSettings& settings,
                               double modelDiag) {
    EdgeSampleMap map;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        const FaceMeshSettings fs =
            mergeEdgeSettings(model, eid, settings, analysis);
        const double sag = effectiveSag(fs, modelDiag);
        map[eid].forward =
            sampleEdgeCurve(edge, sag, fs.angleToleranceDeg, fs.maxLength);
    }
    return map;
}

// Walk a wire using shared edge samples. Returns 3D points + owning edge slots.
struct WireVert {
    gp_Pnt p;
    int edgeId = 0;
    int sampleIndex = 0;  // index in FORWARD samples; -1 if not from map
};

std::vector<WireVert> walkWire(const TopoDS_Wire& wire, const Model& model,
                               const EdgeSampleMap& edges) {
    std::vector<WireVert> verts;
    // Map TopoDS_Edge → EdgeId
    TopTools_IndexedMapOfShape edgeMap;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        edgeMap.Add(model.edges(eid));
    }

    // TopExp_Explorer does not follow wire contour order — use WireExplorer.
    for (BRepTools_WireExplorer ex(wire); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        const TopoDS_Edge fwd = TopoDS::Edge(edge.Oriented(TopAbs_FORWARD));
        int eid = edgeMap.FindIndex(fwd);
        if (eid <= 0) {
            eid = edgeMap.FindIndex(edge);
        }
        if (eid <= 0) {
            for (int i = 1; i <= model.edgeCount(); ++i) {
                if (edge.IsSame(TopoDS::Edge(model.edges(i)))) {
                    eid = i;
                    break;
                }
            }
        }
        if (eid <= 0) continue;
        auto it = edges.find(eid);
        if (it == edges.end() || it->second.forward.size() < 2) continue;

        const auto& fwdSamples = it->second.forward;
        const bool reversed = (edge.Orientation() == TopAbs_REVERSED);
        const int n = int(fwdSamples.size());
        // Skip first sample of subsequent edges (shared vertex).
        const int start = verts.empty() ? 0 : 1;
        for (int k = start; k < n; ++k) {
            const int idx = reversed ? (n - 1 - k) : k;
            // When reversed, sampleIndex still refers to FORWARD index.
            WireVert wv;
            wv.p = fwdSamples[idx].p;
            wv.edgeId = eid;
            wv.sampleIndex = idx;
            verts.push_back(wv);
        }
    }
    if (verts.size() >= 2 &&
        verts.front().p.Distance(verts.back().p) < 1e-9) {
        verts.pop_back();
    }
    return verts;
}

bool isPlanarFace(const TopoDS_Face& face) {
    return BRepAdaptor_Surface(face).GetType() == GeomAbs_Plane;
}

// 2D segment intersection (proper cross, not shared endpoints).
bool segmentsCross2d(const gp_Pnt2d& a, const gp_Pnt2d& b, const gp_Pnt2d& c,
                     const gp_Pnt2d& d) {
    auto orient = [](const gp_Pnt2d& p, const gp_Pnt2d& q, const gp_Pnt2d& r) {
        const double v = (q.X() - p.X()) * (r.Y() - p.Y()) -
                         (q.Y() - p.Y()) * (r.X() - p.X());
        if (std::abs(v) < 1e-14) return 0;
        return v > 0.0 ? 1 : -1;
    };
    const int o1 = orient(a, b, c);
    const int o2 = orient(a, b, d);
    const int o3 = orient(c, d, a);
    const int o4 = orient(c, d, b);
    if (o1 == 0 || o2 == 0 || o3 == 0 || o4 == 0) return false;
    return o1 != o2 && o3 != o4;
}

struct IndexedRingVert {
    WireVert wv;
    uint32_t idx = 0;
    gp_Pnt2d uv;
};

std::vector<IndexedRingVert> indexRing(const std::vector<WireVert>& ring,
                                       const TopoDS_Face& face, int faceId,
                                       double weldTol, MeshBuilder& out) {
    std::vector<IndexedRingVert> indexed;
    indexed.reserve(ring.size());
    for (const WireVert& wv : ring) {
        IndexedRingVert ir;
        ir.wv = wv;
        const Anchor a = anchorOnFace(face, faceId, wv.p);
        if (wv.edgeId > 0 && wv.sampleIndex >= 0) {
            ir.idx = out.addEdgeSample(wv.edgeId, wv.sampleIndex, wv.p, a,
                                       weldTol);
        } else {
            ir.idx = out.addVertex(wv.p, a, weldTol);
        }
        ir.uv = gp_Pnt2d(a.u, a.v);
        indexed.push_back(ir);
    }
    return indexed;
}

// Merge holes into outer via non-crossing keyhole bridges (doubled verts).
std::vector<IndexedRingVert> mergeHolesKeyhole(
    std::vector<IndexedRingVert> outer,
    std::vector<std::vector<IndexedRingVert>> holes) {
    auto maxU = [](const std::vector<IndexedRingVert>& ring) {
        size_t best = 0;
        for (size_t i = 1; i < ring.size(); ++i) {
            if (ring[i].uv.X() > ring[best].uv.X()) best = i;
        }
        return best;
    };
    std::sort(holes.begin(), holes.end(),
              [&](const auto& a, const auto& b) {
                  return a[maxU(a)].uv.X() > b[maxU(b)].uv.X();
              });

    for (size_t h = 0; h < holes.size(); ++h) {
        const auto& hole = holes[h];
        if (hole.size() < 3) continue;
        const size_t m = maxU(hole);
        const gp_Pnt2d& M = hole[m].uv;

        auto crossesAny = [&](const gp_Pnt2d& from, const gp_Pnt2d& to) {
            auto crossesRing = [&](const std::vector<IndexedRingVert>& ring) {
                for (size_t i = 0; i < ring.size(); ++i) {
                    if (segmentsCross2d(from, to, ring[i].uv,
                                        ring[(i + 1) % ring.size()].uv)) {
                        return true;
                    }
                }
                return false;
            };
            if (crossesRing(outer) || crossesRing(hole)) return true;
            for (size_t j = h + 1; j < holes.size(); ++j) {
                if (crossesRing(holes[j])) return true;
            }
            return false;
        };

        size_t bestP = outer.size();
        double bestD = 1e300;
        for (size_t p = 0; p < outer.size(); ++p) {
            const double d = M.SquareDistance(outer[p].uv);
            if (d >= bestD) continue;
            if (crossesAny(M, outer[p].uv)) continue;
            bestD = d;
            bestP = p;
        }
        if (bestP == outer.size()) {
            // Pathological: leave hole unmerged (caller may emit separately).
            continue;
        }

        std::vector<IndexedRingVert> merged;
        merged.reserve(outer.size() + hole.size() + 2);
        merged.insert(merged.end(), outer.begin(), outer.begin() + bestP + 1);
        for (size_t k = 0; k <= hole.size(); ++k) {
            merged.push_back(hole[(m + k) % hole.size()]);
        }
        merged.insert(merged.end(), outer.begin() + bestP, outer.end());
        outer = std::move(merged);
    }
    return outer;
}

void meshPlanarFace(const TopoDS_Face& face, int faceId, const Model& model,
                    const EdgeSampleMap& edgeSamples, double weldTol,
                    MeshBuilder& out) {
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);
    if (outerWire.IsNull()) return;

    auto outerRing = walkWire(outerWire, model, edgeSamples);
    if (outerRing.size() < 3) return;

    std::vector<std::vector<WireVert>> holeRings;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        if (w.IsSame(outerWire)) continue;
        auto hole = walkWire(w, model, edgeSamples);
        if (hole.size() >= 3) holeRings.push_back(std::move(hole));
    }

    auto indexedOuter = indexRing(outerRing, face, faceId, weldTol, out);
    std::vector<std::vector<IndexedRingVert>> indexedHoles;
    indexedHoles.reserve(holeRings.size());
    for (const auto& h : holeRings) {
        indexedHoles.push_back(indexRing(h, face, faceId, weldTol, out));
    }

    std::vector<IndexedRingVert> ring = indexedOuter;
    if (!indexedHoles.empty()) {
        ring = mergeHolesKeyhole(std::move(indexedOuter),
                                 std::move(indexedHoles));
    }

    std::vector<uint32_t> idxs;
    idxs.reserve(ring.size());
    for (const auto& ir : ring) idxs.push_back(ir.idx);

    BRepAdaptor_Surface surf(face);
    gp_Dir n = surf.Plane().Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    if (idxs.size() >= 3) {
        const auto& A = out.mesh.vertices[idxs[0]];
        const auto& B = out.mesh.vertices[idxs[1]];
        const auto& C = out.mesh.vertices[idxs[2]];
        const gp_Vec ab(B[0] - A[0], B[1] - A[1], B[2] - A[2]);
        const gp_Vec ac(C[0] - A[0], C[1] - A[1], C[2] - A[2]);
        if (ab.Crossed(ac).Dot(gp_Vec(n)) < 0.0) {
            std::reverse(idxs.begin(), idxs.end());
        }
    }
    out.addPolygon(std::move(idxs), faceId);
}

// All shared-edge samples on a face outer+hole wires.
std::vector<WireVert> faceBoundarySamples(const TopoDS_Face& face,
                                          const Model& model,
                                          const EdgeSampleMap& edgeSamples) {
    std::vector<WireVert> all;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        auto ring = walkWire(TopoDS::Wire(wx.Current()), model, edgeSamples);
        all.insert(all.end(), ring.begin(), ring.end());
    }
    return all;
}

// Ear-clip a UV ring into triangles (indices into `ring`).
std::vector<std::array<size_t, 3>> earclipUv(
    const std::vector<IndexedRingVert>& ring) {
    std::vector<std::array<size_t, 3>> tris;
    const size_t n = ring.size();
    if (n < 3) return tris;
    auto cross = [&](size_t o, size_t a, size_t b) {
        const double ox = ring[o].uv.X(), oy = ring[o].uv.Y();
        return (ring[a].uv.X() - ox) * (ring[b].uv.Y() - oy) -
               (ring[a].uv.Y() - oy) * (ring[b].uv.X() - ox);
    };
    auto d2 = [&](size_t a, size_t b) {
        const double dx = ring[a].uv.X() - ring[b].uv.X();
        const double dy = ring[a].uv.Y() - ring[b].uv.Y();
        return dx * dx + dy * dy;
    };
    double span = 0;
    for (const auto& ir : ring) {
        span = std::max({span, std::abs(ir.uv.X()), std::abs(ir.uv.Y())});
    }
    const double eps = 1e-12 * std::max(1.0, span * span);
    auto inside = [&](size_t a, size_t b, size_t c, size_t p) {
        return cross(a, b, p) > eps && cross(b, c, p) > eps &&
               cross(c, a, p) > eps;
    };
    // Ensure CCW in UV for ear tests.
    double area2 = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto& a = ring[i].uv;
        const auto& b = ring[(i + 1) % n].uv;
        area2 += a.X() * b.Y() - b.X() * a.Y();
    }
    std::vector<size_t> idx(n);
    for (size_t i = 0; i < n; ++i) idx[i] = i;
    if (area2 < 0) std::reverse(idx.begin(), idx.end());

    size_t guard = 3 * n * n + 16;
    while (idx.size() > 3 && guard-- > 0) {
        size_t bestK = idx.size();
        double bestQ = -1.0;
        for (size_t k = 0; k < idx.size(); ++k) {
            const size_t ip = idx[(k + idx.size() - 1) % idx.size()];
            const size_t ic = idx[k];
            const size_t in = idx[(k + 1) % idx.size()];
            const double a2 = cross(ip, ic, in);
            if (a2 <= eps) continue;
            const double s = d2(ip, ic) + d2(ic, in) + d2(in, ip);
            const double q = s > 1e-300 ? a2 / s : 0.0;
            if (q <= bestQ) continue;
            bool blocked = false;
            for (size_t other : idx) {
                if (other == ip || other == ic || other == in) continue;
                if (d2(other, ip) < eps || d2(other, ic) < eps ||
                    d2(other, in) < eps) {
                    continue;
                }
                if (inside(ip, ic, in, other)) {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
            bestK = k;
            bestQ = q;
        }
        if (bestK >= idx.size()) {
            for (size_t k = 1; k + 1 < idx.size(); ++k) {
                tris.push_back({idx[0], idx[k], idx[k + 1]});
            }
            return tris;
        }
        const size_t ip = idx[(bestK + idx.size() - 1) % idx.size()];
        const size_t ic = idx[bestK];
        const size_t in = idx[(bestK + 1) % idx.size()];
        tris.push_back({ip, ic, in});
        idx.erase(idx.begin() + static_cast<long>(bestK));
    }
    if (idx.size() == 3) tris.push_back({idx[0], idx[1], idx[2]});
    return tris;
}

// Watertight path for trimmed faces: boundary = shared edge samples only.
bool meshBoundaryLoops(const TopoDS_Face& face, int faceId, const Model& model,
                       const EdgeSampleMap& edgeSamples, double weldTol,
                       MeshBuilder& out) {
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);
    if (outerWire.IsNull()) return false;
    auto outerRing = walkWire(outerWire, model, edgeSamples);
    if (outerRing.size() < 3) return false;

    std::vector<std::vector<WireVert>> holeRings;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        if (w.IsSame(outerWire)) continue;
        auto hole = walkWire(w, model, edgeSamples);
        if (hole.size() >= 3) holeRings.push_back(std::move(hole));
    }

    auto indexedOuter = indexRing(outerRing, face, faceId, weldTol, out);
    std::vector<std::vector<IndexedRingVert>> indexedHoles;
    for (const auto& h : holeRings) {
        indexedHoles.push_back(indexRing(h, face, faceId, weldTol, out));
    }
    std::vector<IndexedRingVert> ring = indexedOuter;
    if (!indexedHoles.empty()) {
        ring = mergeHolesKeyhole(std::move(indexedOuter),
                                 std::move(indexedHoles));
    }
    if (ring.size() < 3) return false;

    // Prefer a cylinder/cone quad strip when the outer wire is two equal rims.
    BRepAdaptor_Surface surf(face);
    const GeomAbs_SurfaceType ty = surf.GetType();
    if ((ty == GeomAbs_Cylinder || ty == GeomAbs_Cone) && holeRings.empty()) {
        struct RimEdge {
            int eid = 0;
            bool closed = false;
            std::vector<WireVert> samples;
            double vMean = 0;
        };
        std::map<int, RimEdge> circ;
        TopTools_IndexedMapOfShape edgeMap;
        for (int eid = 1; eid <= model.edgeCount(); ++eid) {
            edgeMap.Add(model.edges(eid));
        }
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            int eid = edgeMap.FindIndex(edge.Oriented(TopAbs_FORWARD));
            if (eid <= 0) eid = edgeMap.FindIndex(edge);
            if (eid <= 0) continue;
            BRepAdaptor_Curve c(edge);
            if (c.GetType() != GeomAbs_Circle) continue;
            auto it = edgeSamples.find(eid);
            if (it == edgeSamples.end() || it->second.forward.size() < 2) {
                continue;
            }
            RimEdge& re = circ[eid];
            re.eid = eid;
            re.closed = BRep_Tool::IsClosed(TopoDS::Edge(model.edges(eid)));
            re.samples.clear();
            for (int si = 0; si < int(it->second.forward.size()); ++si) {
                WireVert wv;
                wv.p = it->second.forward[si].p;
                wv.edgeId = eid;
                wv.sampleIndex = si;
                re.samples.push_back(wv);
            }
            double vs = 0;
            int nv = 0;
            for (const WireVert& wv : re.samples) {
                const Anchor a = anchorOnFace(face, faceId, wv.p);
                vs += a.v;
                ++nv;
            }
            re.vMean = nv ? vs / nv : 0;
        }
        if (circ.size() == 2) {
            auto it = circ.begin();
            RimEdge a = it->second;
            ++it;
            RimEdge b = it->second;
            if (a.vMean > b.vMean) std::swap(a, b);
            auto sortU = [&](RimEdge& r) {
                std::vector<std::pair<double, WireVert>> keyed;
                keyed.reserve(r.samples.size());
                for (const WireVert& wv : r.samples) {
                    const Anchor an = anchorOnFace(face, faceId, wv.p);
                    keyed.push_back({an.u, wv});
                }
                std::sort(keyed.begin(), keyed.end(),
                          [](const auto& x, const auto& y) {
                              return x.first < y.first;
                          });
                r.samples.clear();
                for (auto& kv : keyed) r.samples.push_back(kv.second);
            };
            sortU(a);
            sortU(b);
            // Align start ends (one rim may run opposite U after sort).
            if (!a.samples.empty() && !b.samples.empty() &&
                a.samples.front().p.Distance(b.samples.front().p) >
                    a.samples.front().p.Distance(b.samples.back().p)) {
                std::reverse(b.samples.begin(), b.samples.end());
            }
            if (a.samples.size() == b.samples.size() && a.samples.size() >= 3 &&
                a.closed && b.closed) {
                // Full-period drum only. Open fillet arcs fall through to
                // boundary earclip so wire order (not U-sort) matches planars.
                const size_t n = a.samples.size();
                std::vector<uint32_t> ia(n), ib(n);
                for (size_t i = 0; i < n; ++i) {
                    const Anchor aa = anchorOnFace(face, faceId, a.samples[i].p);
                    const Anchor ab = anchorOnFace(face, faceId, b.samples[i].p);
                    ia[i] = out.addEdgeSample(a.samples[i].edgeId,
                                              a.samples[i].sampleIndex,
                                              a.samples[i].p, aa, weldTol);
                    ib[i] = out.addEdgeSample(b.samples[i].edgeId,
                                              b.samples[i].sampleIndex,
                                              b.samples[i].p, ab, weldTol);
                }
                const bool closed = a.closed && b.closed;
                const size_t seg = closed ? n : (n > 0 ? n - 1 : 0);
                const bool faceReversed = face.Orientation() == TopAbs_REVERSED;
                int emitted = 0;
                for (size_t i = 0; i < seg; ++i) {
                    const size_t j = closed ? (i + 1) % n : (i + 1);
                    // Orient with surface normal at mid-cell.
                    gp_Pnt p;
                    gp_Vec du, dv;
                    const Anchor am = anchorOnFace(
                        face, faceId,
                        gp_Pnt(0.25 * (a.samples[i].p.X() + a.samples[j].p.X() +
                                       b.samples[i].p.X() + b.samples[j].p.X()),
                               0.25 * (a.samples[i].p.Y() + a.samples[j].p.Y() +
                                       b.samples[i].p.Y() + b.samples[j].p.Y()),
                               0.25 * (a.samples[i].p.Z() + a.samples[j].p.Z() +
                                       b.samples[i].p.Z() + b.samples[j].p.Z())));
                    surf.D1(am.u, am.v, p, du, dv);
                    gp_Vec sn = du.Crossed(dv);
                    if (faceReversed) sn.Reverse();
                    const auto& A = out.mesh.vertices[ia[i]];
                    const auto& B = out.mesh.vertices[ia[j]];
                    const auto& C = out.mesh.vertices[ib[j]];
                    const gp_Vec ab(B[0] - A[0], B[1] - A[1], B[2] - A[2]);
                    const gp_Vec ac(C[0] - A[0], C[1] - A[1], C[2] - A[2]);
                    const bool flip = sn.Magnitude() > 1e-12 &&
                                      ab.Crossed(ac).Dot(sn) < 0.0;
                    if (!flip) {
                        out.addPolygon({ia[i], ia[j], ib[j], ib[i]}, faceId);
                    } else {
                        out.addPolygon({ia[i], ib[i], ib[j], ia[j]}, faceId);
                    }
                    ++emitted;
                }
                if (emitted > 0) return true;
            }
        }
    }

    // General trimmed face: triangulate boundary loop in UV.
    auto tris = earclipUv(ring);
    if (tris.empty()) return false;
    BRepAdaptor_Surface srf(face);
    const bool faceReversed = face.Orientation() == TopAbs_REVERSED;
    int emitted = 0;
    for (const auto& t : tris) {
        const uint32_t ia = ring[t[0]].idx;
        const uint32_t ib = ring[t[1]].idx;
        const uint32_t ic = ring[t[2]].idx;
        const double um =
            (ring[t[0]].uv.X() + ring[t[1]].uv.X() + ring[t[2]].uv.X()) / 3.0;
        const double vm =
            (ring[t[0]].uv.Y() + ring[t[1]].uv.Y() + ring[t[2]].uv.Y()) / 3.0;
        gp_Pnt p;
        gp_Vec du, dv;
        srf.D1(um, vm, p, du, dv);
        gp_Vec sn = du.Crossed(dv);
        if (faceReversed) sn.Reverse();
        const auto& A = out.mesh.vertices[ia];
        const auto& B = out.mesh.vertices[ib];
        const auto& C = out.mesh.vertices[ic];
        const gp_Vec ab(B[0] - A[0], B[1] - A[1], B[2] - A[2]);
        const gp_Vec ac(C[0] - A[0], C[1] - A[1], C[2] - A[2]);
        if (sn.Magnitude() > 1e-12 && ab.Crossed(ac).Dot(sn) < 0.0) {
            out.addPolygon({ia, ic, ib}, faceId);
        } else {
            out.addPolygon({ia, ib, ic}, faceId);
        }
        ++emitted;
    }
    return emitted > 0;
}

// Sphere / torus: UV quads when the domain is a simple rectangle; otherwise
// fall through to boundary-loop fill.
bool meshAnalyticUvGrid(const TopoDS_Face& face, int faceId, double sag,
                        double angleDeg, double maxLength, double weldTol,
                        MeshBuilder& out) {
    BRepAdaptor_Surface surf(face);
    const GeomAbs_SurfaceType ty = surf.GetType();
    if (ty != GeomAbs_Sphere && ty != GeomAbs_Torus) return false;

    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    if (!(u1 > u0) || !(v1 > v0)) return false;

    double radiusU = 0.0;
    double radiusV = 0.0;
    try {
        if (ty == GeomAbs_Sphere) {
            radiusU = surf.Sphere().Radius();
            radiusV = radiusU;
        } else {
            radiusU = surf.Torus().MajorRadius() + surf.Torus().MinorRadius();
            radiusV = surf.Torus().MinorRadius();
        }
    } catch (...) {
        return false;
    }
    if (!(radiusU > 1e-12) || !(radiusV > 1e-12)) return false;

    const double uSpan = u1 - u0;
    const double vSpan = v1 - v0;
    int nu = circleDivisions(radiusU, sag, angleDeg);
    nu = std::max(3, int(std::ceil(nu * (uSpan / (2.0 * M_PI)) - 1e-9)));
    int nv = circleDivisions(radiusV, sag, angleDeg);
    nv = std::max(1, int(std::ceil(nv * (std::abs(vSpan) / (2.0 * M_PI)) -
                                   1e-9)));
    nu = std::clamp(nu, 3, 256);
    nv = std::clamp(nv, 1, 128);

    BRepTopAdaptor_FClass2d classifier(face, Precision::Confusion());
    // Require almost-full rectangle coverage — otherwise boundary fill.
    int inside = 0, total = nu * nv;
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            const double um = u0 + uSpan * ((double(i) + 0.5) / nu);
            const double vm = v0 + vSpan * ((double(j) + 0.5) / nv);
            if (classifier.Perform(gp_Pnt2d(um, vm)) != TopAbs_OUT) ++inside;
        }
    }
    if (total > 0 && double(inside) / double(total) < 0.92) return false;

    const bool faceReversed = face.Orientation() == TopAbs_REVERSED;
    std::vector<std::vector<uint32_t>> grid(nv + 1,
                                            std::vector<uint32_t>(nu + 1, ~0u));
    auto ensure = [&](int i, int j) -> uint32_t {
        if (grid[j][i] != ~0u) return grid[j][i];
        const double u = u0 + uSpan * (double(i) / nu);
        const double v = v0 + vSpan * (double(j) / nv);
        gp_Pnt p;
        surf.D0(u, v, p);
        grid[j][i] = out.addVertex(p, Anchor{faceId, u, v}, weldTol);
        return grid[j][i];
    };

    int emitted = 0;
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            const double um = u0 + uSpan * ((double(i) + 0.5) / nu);
            const double vm = v0 + vSpan * ((double(j) + 0.5) / nv);
            if (classifier.Perform(gp_Pnt2d(um, vm)) == TopAbs_OUT) continue;
            const uint32_t ia = ensure(i, j);
            const uint32_t ib = ensure(i + 1, j);
            const uint32_t ic = ensure(i + 1, j + 1);
            const uint32_t id = ensure(i, j + 1);
            gp_Pnt p;
            gp_Vec du, dv;
            surf.D1(um, vm, p, du, dv);
            gp_Vec sn = du.Crossed(dv);
            if (faceReversed) sn.Reverse();
            const auto& A = out.mesh.vertices[ia];
            const auto& B = out.mesh.vertices[ib];
            const auto& C = out.mesh.vertices[ic];
            const gp_Vec ab(B[0] - A[0], B[1] - A[1], B[2] - A[2]);
            const gp_Vec ac(C[0] - A[0], C[1] - A[1], C[2] - A[2]);
            const bool flip =
                sn.Magnitude() > 1e-12 && ab.Crossed(ac).Dot(sn) < 0.0;
            if (!flip) out.addPolygon({ia, ib, ic, id}, faceId);
            else out.addPolygon({ia, id, ic, ib}, faceId);
            ++emitted;
        }
    }
    return emitted > 0;
}


}  // namespace

void applyQualityPreset(FaceMeshSettings& s, QualityPreset preset) {
    // Pixyz-style: maxSag + maxAngle. Angle densifies small-radius fillets
    // when sag alone would leave them coarse (Pixyz docs / SDK examples).
    s.maxLength = -1;
    switch (preset) {
        case QualityPreset::VeryHigh:
            s.chordTolerance = 0.01;
            s.angleToleranceDeg = 10.0;
            break;
        case QualityPreset::High:
            s.chordTolerance = 0.1;
            s.angleToleranceDeg = 15.0;
            break;
        case QualityPreset::Medium:
            s.chordTolerance = 0.2;
            s.angleToleranceDeg = 20.0;
            break;
        case QualityPreset::Low:
            s.chordTolerance = 1.0;
            s.angleToleranceDeg = 40.0;
            break;
    }
}

PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settingsIn, GenerationReport* report,
                  GenerationCache* /*cache*/) {
    const GenerationSettings& settings = settingsIn;
    MeshBuilder builder;
    const double diag = modelDiagonalOf(model);
    const double weldTol = std::max(settings.weldTolerance, 1e-9);

    if (report) {
        report->cacheHits = 0;
        report->cacheMisses = model.faceCount();
        report->faceMesher.clear();
        report->faceBuild.clear();
        report->faceBuildCause.clear();
        report->faceFeatureClass.clear();
        report->faceChartKind.clear();
        report->edgeDivisions.clear();
        report->remeshedFaces.clear();
    }

    const EdgeSampleMap edgeSamples =
        buildEdgeSamples(model, analysis, settings, diag);
    if (report) {
        for (const auto& [eid, es] : edgeSamples) {
            // Divisions = segments = samples - 1
            report->edgeDivisions[eid] =
                std::max(1, int(es.forward.size()) - 1);
        }
    }

    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const FaceMeshSettings& fs = settings.forFace(fid);
        if (fs.exclude) continue;

        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const double sag = effectiveSag(fs, diag);

        if (report) {
            const int idx = fid - 1;
            if (idx >= 0 && idx < int(analysis.faces.size())) {
                report->faceFeatureClass[fid] = analysis.faces[idx].featureClass;
                report->faceChartKind[fid] = analysis.faces[idx].chartKind;
            }
            report->remeshedFaces.push_back(fid);
        }

        if (isPlanarFace(face)) {
            meshPlanarFace(face, fid, model, edgeSamples, weldTol, builder);
            if (report) {
                report->faceMesher[fid] = MesherKind::MinimalNGon;
                report->faceBuild[fid] = 0;
            }
            continue;
        }

        // Sphere/torus rectangular UV → quad grid; else / cylinders →
        // boundary-loop fill (shared edge slots → watertight with planars).
        if (meshAnalyticUvGrid(face, fid, sag, fs.angleToleranceDeg,
                               fs.maxLength, weldTol, builder)) {
            if (report) {
                report->faceMesher[fid] = MesherKind::RevolutionGrid;
                report->faceBuild[fid] = 0;
                report->faceBuildCause[fid] = "accuracy-uv-grid";
            }
            continue;
        }
        if (meshBoundaryLoops(face, fid, model, edgeSamples, weldTol,
                              builder)) {
            if (report) {
                report->faceMesher[fid] = MesherKind::RevolutionGrid;
                report->faceBuild[fid] = 0;
                report->faceBuildCause[fid] = "accuracy-boundary";
            }
            continue;
        }
        if (report) {
            report->faceMesher[fid] = MesherKind::Fallback;
            report->faceBuild[fid] = 0;
            report->faceBuildCause[fid] = "accuracy-empty";
        }
    }

    PolyMesh mesh = std::move(builder.mesh);
    if (settings.finalizeMesh) {
        weldVertices(mesh, weldTol);
    }
    return mesh;
}

}  // namespace weft
