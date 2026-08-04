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

#include <algorithm>
#include <cmath>
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

    // Circles/arcs: also enforce closed-form sag count so Medium≠High.
    try {
        if (curve.GetType() == GeomAbs_Circle) {
            const double r = curve.Circle().Radius();
            const double span = std::abs(last - first);
            const int full = circleDivisions(r, sag, angleDeg);
            const int forSpan =
                std::max(1, int(std::ceil(full * (span / (2.0 * M_PI)))));
            n = std::max(n, forSpan);
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
    std::unordered_map<int64_t, uint32_t> weld;

    static int64_t quantize(const gp_Pnt& p, double tol) {
        const double s = 1.0 / std::max(tol, 1e-12);
        return (int64_t(std::llround(p.X() * s)) * 73856093) ^
               (int64_t(std::llround(p.Y() * s)) * 19349663) ^
               (int64_t(std::llround(p.Z() * s)) * 83492791);
    }

    uint32_t addVertex(const gp_Pnt& p, const Anchor& a, double weldTol) {
        const int64_t key = quantize(p, weldTol);
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

    for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        const TopoDS_Edge fwd = TopoDS::Edge(edge.Oriented(TopAbs_FORWARD));
        int eid = edgeMap.FindIndex(fwd);
        if (eid <= 0) {
            // Try without reorientation
            eid = edgeMap.FindIndex(edge);
        }
        // Same geometry may be stored as the model's edge shape.
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

void meshPlanarFace(const TopoDS_Face& face, int faceId, const Model& model,
                    const EdgeSampleMap& edgeSamples, double weldTol,
                    MeshBuilder& out) {
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return;

    auto emitRing = [&](const TopoDS_Wire& wire) {
        auto ring = walkWire(wire, model, edgeSamples);
        if (ring.size() < 3) return;
        std::vector<uint32_t> idxs;
        idxs.reserve(ring.size());
        for (const WireVert& wv : ring) {
            const Anchor a = anchorOnFace(face, faceId, wv.p);
            if (wv.edgeId > 0 && wv.sampleIndex >= 0) {
                idxs.push_back(out.addEdgeSample(wv.edgeId, wv.sampleIndex,
                                                  wv.p, a, weldTol));
            } else {
                idxs.push_back(out.addVertex(wv.p, a, weldTol));
            }
        }
        // Orient to plane normal (respecting face orientation).
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
    };

    emitRing(outer);
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        if (w.IsSame(outer)) continue;
        emitRing(w);
    }
}

// Closed cylinder / cone / sphere / torus revolution band as UV quads/tris.
bool meshAnalyticRevolution(const TopoDS_Face& face, int faceId, double sag,
                            double angleDeg, double maxLength, double weldTol,
                            MeshBuilder& out) {
    BRepAdaptor_Surface surf(face);
    const GeomAbs_SurfaceType ty = surf.GetType();
    if (ty != GeomAbs_Cylinder && ty != GeomAbs_Cone && ty != GeomAbs_Sphere &&
        ty != GeomAbs_Torus) {
        return false;
    }

    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    if (!(u1 > u0) || !(v1 > v0)) return false;

    // Radius for circumferential sag (U for cylinder/cone/sphere/torus).
    double radius = 0.0;
    try {
        if (ty == GeomAbs_Cylinder) radius = surf.Cylinder().Radius();
        else if (ty == GeomAbs_Cone) {
            // Mid-span radius along V.
            const double vm = 0.5 * (v0 + v1);
            gp_Pnt p;
            gp_Vec du, dv;
            surf.D1(u0, vm, p, du, dv);
            radius = du.Magnitude();  // |∂r/∂u| ≈ radius at that V
        } else if (ty == GeomAbs_Sphere) radius = surf.Sphere().Radius();
        else if (ty == GeomAbs_Torus) {
            // Minor radius drives fillet-like density; also check major.
            radius = surf.Torus().MinorRadius();
            const double major = surf.Torus().MajorRadius();
            // Circumferential around tube uses minor; around ring uses major.
            // U is usually the major angle on OCCT tori.
            (void)major;
        }
    } catch (...) {
        return false;
    }
    if (!(radius > 1e-12)) return false;

    const double uSpan = u1 - u0;
    const double vSpan = v1 - v0;
    int nu = circleDivisions(radius, sag, angleDeg);
    // Scale by partial period (open cylinder band / partial sphere).
    nu = std::max(3, int(std::ceil(nu * (uSpan / (2.0 * M_PI)))));

    // Axial / polar: straight generators need 1; curved V uses sag on a
    // mid-U isocurve sample.
    int nv = 1;
    {
        const double angleTol =
            angleDeg > 0.0 ? angleDeg * M_PI / 180.0 : 1e9;
        std::vector<gp_Pnt> probe;
        const int probeN = 32;
        for (int i = 0; i <= probeN; ++i) {
            const double v = v0 + vSpan * (double(i) / probeN);
            gp_Pnt p;
            surf.D0(0.5 * (u0 + u1), v, p);
            probe.push_back(p);
        }
        // Count segments needed for sag along that polyline.
        nv = 1;
        for (int div = 1; div < 256; ++div) {
            bool ok = true;
            for (int i = 0; i < div && ok; ++i) {
                const double a = double(i) / div;
                const double b = double(i + 1) / div;
                const double m = 0.5 * (a + b);
                auto at = [&](double t) {
                    const double idx = t * probeN;
                    const int i0 = std::min(probeN - 1, std::max(0, int(idx)));
                    const double f = idx - i0;
                    const int i1 = std::min(probeN, i0 + 1);
                    return gp_Pnt(
                        probe[i0].X() + (probe[i1].X() - probe[i0].X()) * f,
                        probe[i0].Y() + (probe[i1].Y() - probe[i0].Y()) * f,
                        probe[i0].Z() + (probe[i1].Z() - probe[i0].Z()) * f);
                };
                const gp_Pnt pa = at(a), pb = at(b), pm = at(m);
                const gp_Pnt midChord(0.5 * (pa.X() + pb.X()),
                                      0.5 * (pa.Y() + pb.Y()),
                                      0.5 * (pa.Z() + pb.Z()));
                if (pm.Distance(midChord) > sag) ok = false;
                if (maxLength > 0.0 && pa.Distance(pb) > maxLength) ok = false;
                (void)angleTol;
            }
            if (ok) {
                nv = div;
                break;
            }
            nv = div;
        }
    }
    nu = std::clamp(nu, 3, 512);
    nv = std::clamp(nv, 1, 256);

    // Trim test: only emit cells whose center is ON the face.
    BRepTopAdaptor_FClass2d classifier(face, Precision::Confusion());
    // Prefer geometric normal vs triangle winding to decide flip — TopAbs
    // alone still left folded cells on some cylinders.
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
            const double um =
                u0 + uSpan * ((double(i) + 0.5) / nu);
            const double vm =
                v0 + vSpan * ((double(j) + 0.5) / nv);
            const TopAbs_State st = classifier.Perform(gp_Pnt2d(um, vm));
            if (st == TopAbs_OUT) continue;

            const uint32_t ia = ensure(i, j);
            const uint32_t ib = ensure(i + 1, j);
            const uint32_t ic = ensure(i + 1, j + 1);
            const uint32_t id = ensure(i, j + 1);

            // Orient triangle to match surface normal at cell center.
            gp_Pnt p;
            gp_Vec du, dv;
            surf.D1(um, vm, p, du, dv);
            gp_Vec sn = du.Crossed(dv);
            if (faceReversed) sn.Reverse();
            auto triNormal = [&](uint32_t a, uint32_t b, uint32_t c) {
                const auto& A = out.mesh.vertices[a];
                const auto& B = out.mesh.vertices[b];
                const auto& C = out.mesh.vertices[c];
                const gp_Vec ab(B[0] - A[0], B[1] - A[1], B[2] - A[2]);
                const gp_Vec ac(C[0] - A[0], C[1] - A[1], C[2] - A[2]);
                return ab.Crossed(ac);
            };
            gp_Vec n0 = triNormal(ia, ib, ic);
            bool flipCell = false;
            if (sn.Magnitude() > 1e-12 && n0.Magnitude() > 1e-12) {
                flipCell = sn.Dot(n0) < 0.0;
            } else {
                flipCell = faceReversed;
            }
            if (!flipCell) {
                out.addPolygon({ia, ib, ic}, faceId);
                out.addPolygon({ia, ic, id}, faceId);
            } else {
                out.addPolygon({ia, ic, ib}, faceId);
                out.addPolygon({ia, id, ic}, faceId);
            }
            ++emitted;
        }
    }
    return emitted > 0;
}

// Generic freeform: trimmed UV grid with face classifier.
void meshFreeformFace(const TopoDS_Face& face, int faceId, double sag,
                      double angleDeg, double maxLength, double weldTol,
                      MeshBuilder& out) {
    BRepAdaptor_Surface surf(face);
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    if (!(u1 > u0) || !(v1 > v0)) return;

    const double uSpan = u1 - u0;
    const double vSpan = v1 - v0;
    gp_Pnt c00, c10, c01;
    surf.D0(u0, v0, c00);
    surf.D0(u1, v0, c10);
    surf.D0(u0, v1, c01);
    const double uLen = std::max(c00.Distance(c10), 1e-9);
    const double vLen = std::max(c00.Distance(c01), 1e-9);

    // Initial density from sag as chord on the longer side, then refine.
    int nu = std::clamp(int(std::ceil(uLen / std::max(sag, 1e-6))), 1, 256);
    int nv = std::clamp(int(std::ceil(vLen / std::max(sag, 1e-6))), 1, 256);

    const double angleTol =
        angleDeg > 0.0 ? angleDeg * M_PI / 180.0 : 1e9;

    auto cellFails = [&](int nuCur, int nvCur) {
        for (int i = 0; i < nuCur; ++i) {
            for (int j = 0; j < nvCur; ++j) {
                const double ua = u0 + uSpan * (double(i) / nuCur);
                const double ub = u0 + uSpan * (double(i + 1) / nuCur);
                const double va = v0 + vSpan * (double(j) / nvCur);
                const double vb = v0 + vSpan * (double(j + 1) / nvCur);
                const double um = 0.5 * (ua + ub);
                const double vm = 0.5 * (va + vb);
                gp_Pnt p00, p10, p01, p11, pExact;
                gp_Vec du, dv;
                surf.D1(ua, va, p00, du, dv);
                surf.D0(ub, va, p10);
                surf.D0(ua, vb, p01);
                surf.D0(ub, vb, p11);
                surf.D1(um, vm, pExact, du, dv);
                const gp_Pnt pm((p00.XYZ() + p10.XYZ() + p01.XYZ() + p11.XYZ()) *
                                0.25);
                if (pm.Distance(pExact) > sag) return true;
                if (maxLength > 0.0 &&
                    (p00.Distance(p10) > maxLength ||
                     p00.Distance(p01) > maxLength)) {
                    return true;
                }
                gp_Vec nA = du.Crossed(dv);
                gp_Vec du2, dv2;
                gp_Pnt tmp;
                surf.D1(ub, vb, tmp, du2, dv2);
                gp_Vec nB = du2.Crossed(dv2);
                if (nA.Magnitude() > 1e-12 && nB.Magnitude() > 1e-12 &&
                    nA.Angle(nB) > angleTol) {
                    return true;
                }
            }
        }
        return false;
    };

    for (int pass = 0; pass < 6; ++pass) {
        if (!cellFails(nu, nv)) break;
        if (nu < 512) nu = std::min(512, nu * 2);
        if (nv < 512) nv = std::min(512, nv * 2);
    }

    BRepTopAdaptor_FClass2d classifier(face, Precision::Confusion());
    const bool faceReversed = face.Orientation() == TopAbs_REVERSED;
    std::vector<std::vector<uint32_t>> grid(nv + 1,
                                            std::vector<uint32_t>(nu + 1, ~0u));
    auto ensure = [&](int i, int j) {
        if (grid[j][i] != ~0u) return grid[j][i];
        const double u = u0 + uSpan * (double(i) / nu);
        const double v = v0 + vSpan * (double(j) / nv);
        gp_Pnt p;
        surf.D0(u, v, p);
        grid[j][i] = out.addVertex(p, Anchor{faceId, u, v}, weldTol);
        return grid[j][i];
    };

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
            const gp_Vec n0 = ab.Crossed(ac);
            const bool flipCell =
                (sn.Magnitude() > 1e-12 && n0.Magnitude() > 1e-12)
                    ? (sn.Dot(n0) < 0.0)
                    : faceReversed;
            if (!flipCell) {
                out.addPolygon({ia, ib, ic}, faceId);
                out.addPolygon({ia, ic, id}, faceId);
            } else {
                out.addPolygon({ia, ic, ib}, faceId);
                out.addPolygon({ia, id, ic}, faceId);
            }
        }
    }
}

}  // namespace

void applyQualityPreset(FaceMeshSettings& s, QualityPreset preset) {
    s.angleToleranceDeg = -1;
    s.maxLength = -1;
    switch (preset) {
        case QualityPreset::VeryHigh:
            s.chordTolerance = 0.01;
            break;
        case QualityPreset::High:
            s.chordTolerance = 0.1;
            break;
        case QualityPreset::Medium:
            s.chordTolerance = 0.2;
            break;
        case QualityPreset::Low:
            s.chordTolerance = 1.0;
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

        const bool analytic = meshAnalyticRevolution(
            face, fid, sag, fs.angleToleranceDeg, fs.maxLength, weldTol,
            builder);
        if (analytic) {
            if (report) {
                report->faceMesher[fid] = MesherKind::RevolutionGrid;
                report->faceBuild[fid] = 0;
                report->faceBuildCause[fid] = "accuracy-analytic";
            }
            continue;
        }

        meshFreeformFace(face, fid, sag, fs.angleToleranceDeg, fs.maxLength,
                         weldTol, builder);
        if (report) {
            report->faceMesher[fid] = MesherKind::Fallback;
            report->faceBuild[fid] = 0;
            report->faceBuildCause[fid] = "accuracy-freeform";
        }
    }

    PolyMesh mesh = std::move(builder.mesh);
    if (settings.finalizeMesh) {
        weldVertices(mesh, weldTol);
    }
    return mesh;
}

}  // namespace weft
