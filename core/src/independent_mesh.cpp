#include "weft/independent_mesh.hpp"

#include "mesher_sampling.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <IMeshTools_Parameters.hxx>
#include <Poly_Triangulation.hxx>
#include <ShapeAnalysis_Surface.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <mutex>
#include <vector>

namespace weft {
namespace {

struct EdgeSamples {
    std::vector<gp_Pnt> pts;  // n+1 points, first to last along forward curve
};

double faceChord(const TopoDS_Face& face, const FaceMeshSettings& s) {
    double defl = std::max(1e-9, s.chordTolerance);
    if (!s.relativeDeviation) return defl;
    Bnd_Box bb;
    BRepBndLib::Add(face, bb);
    if (bb.IsVoid()) return defl;
    double x0, y0, z0, x1, y1, z1;
    bb.Get(x0, y0, z0, x1, y1, z1);
    const double diag = gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1));
    return std::max(1e-9, s.chordTolerance * 0.05 * diag);
}

int curveSegmentCount(const TopoDS_Edge& edge, double angleRad,
                      double chord) {
    if (BRep_Tool::Degenerated(edge)) return 1;
    double f = 0, l = 0;
    Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f, l);
    if (c3.IsNull() || !(l > f)) return 1;
    GeomAdaptor_Curve gc(c3, f, l);
    if (gc.GetType() == GeomAbs_Line) return 1;
    return std::clamp(
        mesher_detail::stableDeflectionCount(gc, angleRad, chord), 1, 256);
}

bool edgeIsClosedCurve(const TopoDS_Edge& edge) {
    if (BRep_Tool::Degenerated(edge)) return false;
    double f = 0, l = 0;
    Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f, l);
    if (c3.IsNull()) return false;
    GeomAdaptor_Curve gc(c3, f, l);
    if (gc.GetType() == GeomAbs_Line) return false;
    return c3->Value(f).Distance(c3->Value(l)) < 1e-9;
}

bool edgeIsLine(const TopoDS_Edge& edge) {
    if (BRep_Tool::Degenerated(edge)) return true;
    BRepAdaptor_Curve ac(edge);
    return ac.GetType() == GeomAbs_Line;
}

int wireCount(const TopoDS_Face& face) {
    int n = 0;
    for (TopExp_Explorer w(face, TopAbs_WIRE); w.More(); w.Next()) ++n;
    return n;
}

bool isPlanarFace(const TopoDS_Face& face) {
    try {
        BRepAdaptor_Surface surf(face);
        return surf.GetType() == GeomAbs_Plane;
    } catch (const Standard_Failure&) {
        return false;
    }
}

Anchor projectAnchor(const TopoDS_Face& face, int faceId, const gp_Pnt& p) {
    Anchor a;
    a.faceId = faceId;
    try {
        Handle(Geom_Surface) gs = BRep_Tool::Surface(face);
        if (gs.IsNull()) return a;
        ShapeAnalysis_Surface sas(gs);
        const gp_Pnt2d uv = sas.ValueOfUV(p, 1e-6);
        a.u = uv.X();
        a.v = uv.Y();
    } catch (const Standard_Failure&) {
    }
    return a;
}

std::vector<gp_Pnt> sampleEdgeForward(const TopoDS_Edge& edge, int n) {
    std::vector<gp_Pnt> out;
    if (n < 1) n = 1;
    double f = 0, l = 0;
    Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f, l);
    if (c3.IsNull()) {
        TopoDS_Vertex v1, v2;
        TopExp::Vertices(edge, v1, v2);
        if (!v1.IsNull()) out.push_back(BRep_Tool::Pnt(v1));
        if (!v2.IsNull() && !v1.IsSame(v2)) out.push_back(BRep_Tool::Pnt(v2));
        if (out.size() == 1) out.push_back(out.front());
        return out;
    }
    out.reserve(size_t(n) + 1);
    for (int i = 0; i <= n; ++i) {
        const double t = f + (l - f) * (double(i) / double(n));
        out.push_back(c3->Value(t));
    }
    return out;
}

void appendPolygon(PolyMesh& mesh, int faceId, const TopoDS_Face& face,
                   std::vector<gp_Pnt> pts, bool reverse) {
    if (pts.size() < 3) return;
    const double tol2 = 1e-16;
    std::vector<gp_Pnt> cleaned;
    cleaned.reserve(pts.size());
    for (const gp_Pnt& p : pts) {
        if (!cleaned.empty() && cleaned.back().SquareDistance(p) < tol2) {
            continue;
        }
        cleaned.push_back(p);
    }
    if (cleaned.size() >= 2 &&
        cleaned.front().SquareDistance(cleaned.back()) < 1e-16) {
        cleaned.pop_back();
    }
    if (cleaned.size() < 3) return;
    if (reverse) std::reverse(cleaned.begin(), cleaned.end());
    std::vector<uint32_t> poly;
    poly.reserve(cleaned.size());
    for (const gp_Pnt& p : cleaned) {
        poly.push_back(uint32_t(mesh.vertices.size()));
        mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
        mesh.anchors.push_back(projectAnchor(face, faceId, p));
    }
    mesh.polygons.push_back(std::move(poly));
    mesh.polygonFaceId.push_back(faceId);
}

std::vector<gp_Pnt> orientedEdgeSamples(
    const TopoDS_Edge& e, const Model& model,
    const std::vector<std::vector<gp_Pnt>>& samples) {
    const int eid = model.edges.FindIndex(e);
    std::vector<gp_Pnt> samp;
    if (eid >= 1 && eid < int(samples.size()) && !samples[eid].empty()) {
        samp = samples[eid];
    } else {
        samp = sampleEdgeForward(e, 1);
    }
    if (e.Orientation() == TopAbs_REVERSED) {
        std::reverse(samp.begin(), samp.end());
    }
    return samp;
}

std::vector<gp_Pnt> collectWireLoop(const TopoDS_Wire& wire,
                                    const TopoDS_Face& face,
                                    const Model& model,
                                    const std::vector<std::vector<gp_Pnt>>& samples) {
    std::vector<gp_Pnt> loop;
    for (BRepTools_WireExplorer ex(wire, face); ex.More(); ex.Next()) {
        std::vector<gp_Pnt> samp = orientedEdgeSamples(ex.Current(), model, samples);
        if (samp.size() < 2) continue;
        const size_t start = loop.empty() ? 0 : 1;
        for (size_t i = start; i < samp.size(); ++i) loop.push_back(samp[i]);
    }
    if (loop.size() >= 2 && loop.front().SquareDistance(loop.back()) < 1e-16) {
        loop.pop_back();
    }
    return loop;
}

gp_Pnt2d unwrapUv(gp_Pnt2d uv, const gp_Pnt2d& prev,
                  const BRepAdaptor_Surface& surf) {
    if (surf.IsUPeriodic()) {
        const double period = surf.UPeriod();
        if (period > 0.0) {
            while (uv.X() - prev.X() > 0.5 * period) uv.SetX(uv.X() - period);
            while (prev.X() - uv.X() > 0.5 * period) uv.SetX(uv.X() + period);
        }
    }
    if (surf.IsVPeriodic()) {
        const double period = surf.VPeriod();
        if (period > 0.0) {
            while (uv.Y() - prev.Y() > 0.5 * period) uv.SetY(uv.Y() - period);
            while (prev.Y() - uv.Y() > 0.5 * period) uv.SetY(uv.Y() + period);
        }
    }
    return uv;
}

gp_Pnt2d uvOnFace(const TopoDS_Face& face, const gp_Pnt& p) {
    try {
        Handle(Geom_Surface) gs = BRep_Tool::Surface(face);
        if (gs.IsNull()) return gp_Pnt2d(0, 0);
        ShapeAnalysis_Surface sas(gs);
        return sas.ValueOfUV(p, 1e-6);
    } catch (const Standard_Failure&) {
        return gp_Pnt2d(0, 0);
    }
}

struct SampleLoop {
    std::vector<gp_Pnt> p3;
    std::vector<gp_Pnt2d> uv;
};

SampleLoop collectWireLoopUv(const TopoDS_Wire& wire, const TopoDS_Face& face,
                             const Model& model,
                             const std::vector<std::vector<gp_Pnt>>& samples) {
    SampleLoop loop;
    BRepAdaptor_Surface surf(face);
    for (BRepTools_WireExplorer ex(wire, face); ex.More(); ex.Next()) {
        const TopoDS_Edge e = ex.Current();
        std::vector<gp_Pnt> p3 = orientedEdgeSamples(e, model, samples);
        std::vector<gp_Pnt2d> uv;
        double f2 = 0, l2 = 0;
        Handle(Geom2d_Curve) pc;
        try {
            pc = BRep_Tool::CurveOnSurface(e, face, f2, l2);
        } catch (const Standard_Failure&) {
            pc.Nullify();
        }
        const bool degenerated = BRep_Tool::Degenerated(e);
        if (degenerated) {
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(e, v1, v2);
            gp_Pnt pole(0, 0, 0);
            if (!v1.IsNull()) pole = BRep_Tool::Pnt(v1);
            else if (!v2.IsNull()) pole = BRep_Tool::Pnt(v2);
            int n = std::max(1, int(p3.size()) - 1);
            if (n < 8 && !pc.IsNull() && l2 > f2) n = 8;
            p3.clear();
            uv.clear();
            p3.reserve(size_t(n) + 1);
            uv.reserve(size_t(n) + 1);
            for (int i = 0; i <= n; ++i) {
                p3.push_back(pole);
                if (!pc.IsNull() && l2 > f2) {
                    const double t = f2 + (l2 - f2) * (double(i) / double(n));
                    uv.push_back(pc->Value(t));
                } else {
                    uv.push_back(uvOnFace(face, pole));
                }
            }
            if (e.Orientation() == TopAbs_REVERSED) {
                std::reverse(uv.begin(), uv.end());
            }
        } else if (!pc.IsNull() && l2 > f2 && p3.size() >= 2) {
            const int n = int(p3.size()) - 1;
            uv.reserve(p3.size());
            for (int i = 0; i <= n; ++i) {
                const double t = f2 + (l2 - f2) * (double(i) / double(n));
                uv.push_back(pc->Value(t));
            }
            if (e.Orientation() == TopAbs_REVERSED) {
                std::reverse(uv.begin(), uv.end());
            }
        } else {
            uv.reserve(p3.size());
            for (const gp_Pnt& p : p3) uv.push_back(uvOnFace(face, p));
        }
        if (p3.size() < 2 || uv.size() != p3.size()) continue;
        const size_t start = loop.p3.empty() ? 0 : 1;
        for (size_t i = start; i < p3.size(); ++i) {
            gp_Pnt2d uvi = uv[i];
            if (!loop.uv.empty()) uvi = unwrapUv(uvi, loop.uv.back(), surf);
            else if (i > 0) uvi = unwrapUv(uvi, uv[i - 1], surf);
            if (!loop.p3.empty() &&
                loop.p3.back().SquareDistance(p3[i]) < 1e-16 &&
                loop.uv.back().SquareDistance(uvi) < 1e-16) {
                continue;
            }
            loop.p3.push_back(p3[i]);
            loop.uv.push_back(uvi);
        }
    }
    if (loop.p3.size() >= 2 &&
        loop.p3.front().SquareDistance(loop.p3.back()) < 1e-16 &&
        loop.uv.front().SquareDistance(loop.uv.back()) < 1e-16) {
        loop.p3.pop_back();
        loop.uv.pop_back();
    }
    return loop;
}

double uvSignedArea(const std::vector<gp_Pnt2d>& uv) {
    double a = 0;
    const size_t n = uv.size();
    for (size_t i = 0; i < n; ++i) {
        const gp_Pnt2d& p = uv[i];
        const gp_Pnt2d& q = uv[(i + 1) % n];
        a += p.X() * q.Y() - q.X() * p.Y();
    }
    return 0.5 * a;
}

void reverseLoop(SampleLoop& loop) {
    std::reverse(loop.p3.begin(), loop.p3.end());
    std::reverse(loop.uv.begin(), loop.uv.end());
}

void alignLoopUv(SampleLoop& loop, const SampleLoop& ref,
                 const BRepAdaptor_Surface& surf) {
    if (loop.uv.empty() || ref.uv.empty()) return;
    gp_Pnt2d shift(0, 0);
    double best = 1e300;
    for (const gp_Pnt2d& r : ref.uv) {
        const gp_Pnt2d u = unwrapUv(loop.uv[0], r, surf);
        const double d = u.SquareDistance(r);
        if (d < best) {
            best = d;
            shift = gp_Pnt2d(u.X() - loop.uv[0].X(), u.Y() - loop.uv[0].Y());
        }
    }
    if (shift.X() == 0.0 && shift.Y() == 0.0) return;
    for (gp_Pnt2d& p : loop.uv) {
        p.SetX(p.X() + shift.X());
        p.SetY(p.Y() + shift.Y());
    }
}

double uvCross(const gp_Pnt2d& o, const gp_Pnt2d& a, const gp_Pnt2d& b) {
    return (a.X() - o.X()) * (b.Y() - o.Y()) - (a.Y() - o.Y()) * (b.X() - o.X());
}

bool earClipUv(const std::vector<gp_Pnt2d>& uv, const std::vector<uint32_t>& idx,
               std::vector<std::array<uint32_t, 3>>& tris) {
    const size_t n = uv.size();
    tris.clear();
    if (n < 3 || idx.size() != n) return false;
    double span = 0;
    for (const gp_Pnt2d& p : uv) {
        span = std::max({span, std::abs(p.X()), std::abs(p.Y())});
    }
    const double eps = 1e-12 * std::max(1.0, span * span);
    std::vector<size_t> ring(n);
    for (size_t i = 0; i < n; ++i) ring[i] = i;
    auto insideTri = [&](size_t a, size_t b, size_t c, size_t p) {
        return uvCross(uv[a], uv[b], uv[p]) > eps &&
               uvCross(uv[b], uv[c], uv[p]) > eps &&
               uvCross(uv[c], uv[a], uv[p]) > eps;
    };
    size_t guard = 3 * n * n + 16;
    while (ring.size() > 3 && guard-- > 0) {
        size_t bestK = ring.size();
        double bestQ = -1.0;
        for (size_t k = 0; k < ring.size(); ++k) {
            const size_t ip = ring[(k + ring.size() - 1) % ring.size()];
            const size_t ic = ring[k];
            const size_t in = ring[(k + 1) % ring.size()];
            const double a2 = uvCross(uv[ip], uv[ic], uv[in]);
            if (a2 <= eps) continue;
            const double d2ab = uv[ip].SquareDistance(uv[ic]);
            const double d2bc = uv[ic].SquareDistance(uv[in]);
            const double d2ca = uv[in].SquareDistance(uv[ip]);
            const double s = d2ab + d2bc + d2ca;
            const double q = s > 1e-300 ? a2 / s : 0.0;
            if (q <= bestQ) continue;
            bool blocked = false;
            for (size_t other : ring) {
                if (other == ip || other == ic || other == in) continue;
                if (uv[other].SquareDistance(uv[ip]) < eps ||
                    uv[other].SquareDistance(uv[ic]) < eps ||
                    uv[other].SquareDistance(uv[in]) < eps) {
                    continue;
                }
                if (insideTri(ip, ic, in, other)) {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
            bestK = k;
            bestQ = q;
        }
        if (bestK >= ring.size()) {
            tris.clear();
            return false;
        }
        const size_t ip = ring[(bestK + ring.size() - 1) % ring.size()];
        const size_t ic = ring[bestK];
        const size_t in = ring[(bestK + 1) % ring.size()];
        tris.push_back({idx[ip], idx[ic], idx[in]});
        ring.erase(ring.begin() + long(bestK));
    }
    if (ring.size() != 3) {
        tris.clear();
        return false;
    }
    tris.push_back({idx[ring[0]], idx[ring[1]], idx[ring[2]]});
    return true;
}

void emitUvTris(PolyMesh& mesh, int faceId, bool flip,
                const std::vector<std::array<uint32_t, 3>>& tris) {
    for (const auto& t : tris) {
        if (t[0] == t[1] || t[1] == t[2] || t[2] == t[0]) continue;
        if (flip) mesh.polygons.push_back({t[0], t[2], t[1]});
        else mesh.polygons.push_back({t[0], t[1], t[2]});
        mesh.polygonFaceId.push_back(faceId);
    }
}

std::vector<uint32_t> emitVerts(PolyMesh& mesh, int faceId,
                                const TopoDS_Face& face,
                                const std::vector<gp_Pnt>& pts) {
    std::vector<uint32_t> idx;
    idx.reserve(pts.size());
    for (const gp_Pnt& p : pts) {
        idx.push_back(uint32_t(mesh.vertices.size()));
        mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
        mesh.anchors.push_back(projectAnchor(face, faceId, p));
    }
    return idx;
}

bool meshSampledPlanar(const TopoDS_Face& face, int faceId, const Model& model,
                       const std::vector<std::vector<gp_Pnt>>& samples,
                       PolyMesh& mesh) {
    if (!isPlanarFace(face)) return false;
    TopoDS_Wire outerW = BRepTools::OuterWire(face);
    if (outerW.IsNull()) return false;
    std::vector<gp_Pnt> outer = collectWireLoop(outerW, face, model, samples);
    if (outer.size() < 3) return false;
    std::vector<std::vector<gp_Pnt>> holes;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        if (w.IsSame(outerW)) continue;
        std::vector<gp_Pnt> hole = collectWireLoop(w, face, model, samples);
        if (hole.size() >= 3) holes.push_back(std::move(hole));
    }
    const size_t before = mesh.polygonCount();
    if (holes.empty()) {
        appendPolygon(mesh, faceId, face, std::move(outer), false);
        return mesh.polygonCount() > before;
    }
    // Keyhole n-gon: one polygon, holes bridged with repeated indices so
    // weld does not collapse the bore.
    std::vector<uint32_t> outerIdx = emitVerts(mesh, faceId, face, outer);
    std::vector<uint32_t> poly = outerIdx;
    for (const auto& hole : holes) {
        std::vector<uint32_t> holeIdx = emitVerts(mesh, faceId, face, hole);
        double best = 1e300;
        size_t oi = 0, hi = 0;
        for (size_t i = 0; i < outerIdx.size(); ++i) {
            const auto& A = mesh.vertices[outerIdx[i]];
            for (size_t j = 0; j < holeIdx.size(); ++j) {
                const auto& B = mesh.vertices[holeIdx[j]];
                const double dx = A[0] - B[0], dy = A[1] - B[1],
                             dz = A[2] - B[2];
                const double d = dx * dx + dy * dy + dz * dz;
                if (d < best) {
                    best = d;
                    oi = i;
                    hi = j;
                }
            }
        }
        std::vector<uint32_t> next;
        next.reserve(poly.size() + holeIdx.size() + 2);
        for (size_t i = 0; i <= oi; ++i) next.push_back(poly[i]);
        for (size_t k = 0; k < holeIdx.size(); ++k) {
            next.push_back(holeIdx[(hi + k) % holeIdx.size()]);
        }
        next.push_back(holeIdx[hi]);
        next.push_back(poly[oi]);
        for (size_t i = oi + 1; i < poly.size(); ++i) next.push_back(poly[i]);
        poly = std::move(next);
        outerIdx = poly;
    }
    if (poly.size() < 3) return false;
    mesh.polygons.push_back(std::move(poly));
    mesh.polygonFaceId.push_back(faceId);
    return true;
}

gp_Pnt nearestSample(const gp_Pnt& p,
                     const std::vector<gp_Pnt>& pts, double& distOut) {
    distOut = 1e300;
    gp_Pnt best = p;
    for (const gp_Pnt& q : pts) {
        const double d = p.SquareDistance(q);
        if (d < distOut) {
            distOut = d;
            best = q;
        }
    }
    distOut = distOut < 1e299 ? std::sqrt(distOut) : 1e300;
    return best;
}

bool meshUvFill(const TopoDS_Face& face, int faceId, const FaceMeshSettings& s,
                const Model& model,
                const std::vector<std::vector<gp_Pnt>>& samples,
                PolyMesh& mesh) {
    (void)s;
    TopoDS_Wire outerW = BRepTools::OuterWire(face);
    if (outerW.IsNull()) return false;
    SampleLoop outer = collectWireLoopUv(outerW, face, model, samples);
    if (outer.p3.size() < 3) return false;
    std::vector<SampleLoop> holes;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        if (w.IsSame(outerW)) continue;
        SampleLoop hole = collectWireLoopUv(w, face, model, samples);
        if (hole.p3.size() >= 3) holes.push_back(std::move(hole));
    }
    BRepAdaptor_Surface surf(face);
    for (SampleLoop& hole : holes) alignLoopUv(hole, outer, surf);
    if (uvSignedArea(outer.uv) < 0) reverseLoop(outer);
    for (SampleLoop& hole : holes) {
        if (uvSignedArea(hole.uv) > 0) reverseLoop(hole);
    }

    const size_t v0 = mesh.vertices.size();
    const size_t a0 = mesh.anchors.size();
    const size_t p0 = mesh.polygonCount();
    auto rollback = [&] {
        mesh.vertices.resize(v0);
        mesh.anchors.resize(a0);
        mesh.polygons.resize(p0);
        mesh.polygonFaceId.resize(p0);
    };

    std::vector<uint32_t> outerIdx = emitVerts(mesh, faceId, face, outer.p3);
    for (size_t i = 0; i < outerIdx.size() && i < outer.uv.size(); ++i) {
        mesh.anchors[outerIdx[i]].u = outer.uv[i].X();
        mesh.anchors[outerIdx[i]].v = outer.uv[i].Y();
    }
    std::vector<uint32_t> ringIdx = outerIdx;
    std::vector<gp_Pnt2d> ringUv = outer.uv;
    for (const SampleLoop& hole : holes) {
        std::vector<uint32_t> holeIdx = emitVerts(mesh, faceId, face, hole.p3);
        for (size_t i = 0; i < holeIdx.size() && i < hole.uv.size(); ++i) {
            mesh.anchors[holeIdx[i]].u = hole.uv[i].X();
            mesh.anchors[holeIdx[i]].v = hole.uv[i].Y();
        }
        double best = 1e300;
        size_t oi = 0, hi = 0;
        for (size_t i = 0; i < ringUv.size(); ++i) {
            for (size_t j = 0; j < hole.uv.size(); ++j) {
                const double d = ringUv[i].SquareDistance(hole.uv[j]);
                if (d < best) {
                    best = d;
                    oi = i;
                    hi = j;
                }
            }
        }
        std::vector<uint32_t> nextIdx;
        std::vector<gp_Pnt2d> nextUv;
        nextIdx.reserve(ringIdx.size() + holeIdx.size() + 2);
        nextUv.reserve(nextIdx.capacity());
        for (size_t i = 0; i <= oi; ++i) {
            nextIdx.push_back(ringIdx[i]);
            nextUv.push_back(ringUv[i]);
        }
        for (size_t k = 0; k < holeIdx.size(); ++k) {
            const size_t j = (hi + k) % holeIdx.size();
            nextIdx.push_back(holeIdx[j]);
            nextUv.push_back(hole.uv[j]);
        }
        nextIdx.push_back(holeIdx[hi]);
        nextUv.push_back(hole.uv[hi]);
        nextIdx.push_back(ringIdx[oi]);
        nextUv.push_back(ringUv[oi]);
        for (size_t i = oi + 1; i < ringIdx.size(); ++i) {
            nextIdx.push_back(ringIdx[i]);
            nextUv.push_back(ringUv[i]);
        }
        ringIdx = std::move(nextIdx);
        ringUv = std::move(nextUv);
    }
    if (ringIdx.size() < 3) {
        rollback();
        return false;
    }

    std::vector<std::array<uint32_t, 3>> tris;
    if (!earClipUv(ringUv, ringIdx, tris)) {
        tris.clear();
        for (size_t k = 1; k + 1 < ringIdx.size(); ++k) {
            tris.push_back({ringIdx[0], ringIdx[k], ringIdx[k + 1]});
        }
        if (tris.empty()) {
            rollback();
            return false;
        }
    }

    const bool flip = face.Orientation() == TopAbs_REVERSED;
    emitUvTris(mesh, faceId, flip, tris);
    if (mesh.polygonCount() == p0) {
        rollback();
        return false;
    }
    return true;
}

bool meshDrumGrid(const TopoDS_Face& face, int faceId,
                  const FaceMeshSettings& s, const std::vector<int>& edgeN,
                  const Model& model,
                  const std::vector<std::vector<gp_Pnt>>& samples,
                  PolyMesh& mesh) {
    if (wireCount(face) != 1) return false;
    BRepAdaptor_Surface surf(face);
    const GeomAbs_SurfaceType ty = surf.GetType();
    if (ty != GeomAbs_Cylinder && ty != GeomAbs_Cone &&
        ty != GeomAbs_SurfaceOfRevolution && ty != GeomAbs_Sphere) {
        return false;
    }
    int nEdges = 0;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) ++nEdges;
    if (nEdges < 3 || nEdges > 4) return false;
    int nu = std::max(3, s.radial);
    int nv = std::max(1, s.axial);
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const int eid = model.edges.FindIndex(ex.Current());
        if (eid < 1 || eid >= int(edgeN.size())) continue;
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (edgeIsClosedCurve(edge)) nu = std::max(nu, edgeN[eid]);
        else if (edgeIsLine(edge)) nv = std::max(nv, edgeN[eid]);
    }
    if (ty == GeomAbs_Sphere) {
        nv = std::max(nv, std::max(4, nu / 2));
    }
    double u0 = 0, u1 = 1, v0 = 0, v1 = 1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    if (!(std::abs(u1 - u0) > 1e-12) || !(std::abs(v1 - v0) > 1e-12)) {
        return false;
    }
    const bool uWrap = surf.IsUPeriodic() && std::abs((u1 - u0) - surf.UPeriod()) < 1e-4 * std::max(1.0, surf.UPeriod());
    const bool vWrap = surf.IsVPeriodic() && std::abs((v1 - v0) - surf.VPeriod()) < 1e-4 * std::max(1.0, surf.VPeriod());
    const int nU = uWrap ? nu : nu;
    const int nV = vWrap ? nv : nv;
    const int cols = uWrap ? nU : nU + 1;
    const int rows = vWrap ? nV : nV + 1;
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    std::vector<uint32_t> idx(size_t(rows) * size_t(cols));
    for (int j = 0; j < rows; ++j) {
        const double v = vWrap ? v0 + (v1 - v0) * (double(j) / double(nV))
                               : v0 + (v1 - v0) * (double(j) / double(std::max(1, nV)));
        for (int i = 0; i < cols; ++i) {
            const double u = uWrap ? u0 + (u1 - u0) * (double(i) / double(nU))
                                   : u0 + (u1 - u0) * (double(i) / double(std::max(1, nU)));
            gp_Pnt p;
            try {
                p = surf.Value(u, v);
            } catch (const Standard_Failure&) {
                return false;
            }
            idx[size_t(j) * size_t(cols) + size_t(i)] =
                uint32_t(mesh.vertices.size());
            mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
            mesh.anchors.push_back({faceId, u, v});
        }
    }
    std::vector<gp_Pnt> rim;
    double rimSpace = 1e300;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        const int eid = model.edges.FindIndex(edge);
        if (eid < 1 || eid >= int(samples.size())) continue;
        if (!edgeIsClosedCurve(edge) && !edgeIsLine(edge)) {
            // keep both; rims are usually closed
        }
        const auto& pts = samples[eid];
        for (size_t k = 0; k < pts.size(); ++k) {
            rim.push_back(pts[k]);
            if (k > 0) rimSpace = std::min(rimSpace, pts[k - 1].Distance(pts[k]));
        }
    }
    const double snap = (rimSpace < 1e299) ? 0.45 * rimSpace : 0.0;
    if (snap > 0.0 && !rim.empty()) {
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < cols; ++i) {
                const bool border = (j == 0 || j == rows - 1 || i == 0 ||
                                     i == cols - 1);
                if (!border) continue;
                const uint32_t vi = idx[size_t(j) * size_t(cols) + size_t(i)];
                gp_Pnt p(mesh.vertices[vi][0], mesh.vertices[vi][1],
                         mesh.vertices[vi][2]);
                double dist = 0;
                const gp_Pnt q = nearestSample(p, rim, dist);
                if (dist <= snap) {
                    mesh.vertices[vi] = {q.X(), q.Y(), q.Z()};
                }
            }
        }
    }
    auto at = [&](int i, int j) -> uint32_t {
        if (uWrap) i = (i + nU) % nU;
        if (vWrap) j = (j + nV) % nV;
        return idx[size_t(j) * size_t(cols) + size_t(i)];
    };
    const int iMax = uWrap ? nU : nU;
    const int jMax = vWrap ? nV : nV;
    const size_t before = mesh.polygonCount();
    for (int j = 0; j < jMax; ++j) {
        for (int i = 0; i < iMax; ++i) {
            uint32_t a = at(i, j);
            uint32_t b = at(i + 1, j);
            uint32_t c = at(i + 1, j + 1);
            uint32_t d = at(i, j + 1);
            if (flip) mesh.polygons.push_back({a, d, c, b});
            else mesh.polygons.push_back({a, b, c, d});
            mesh.polygonFaceId.push_back(faceId);
        }
    }
    return mesh.polygonCount() > before;
}

bool meshTransfinite4(const TopoDS_Face& face, int faceId, const Model& model,
                      const std::vector<std::vector<gp_Pnt>>& samples,
                      PolyMesh& mesh) {
    if (wireCount(face) != 1) return false;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    std::vector<std::vector<gp_Pnt>> sides;
    for (BRepTools_WireExplorer ex(outer, face); ex.More(); ex.Next()) {
        std::vector<gp_Pnt> samp =
            orientedEdgeSamples(ex.Current(), model, samples);
        if (samp.size() < 2) continue;
        sides.push_back(std::move(samp));
    }
    if (sides.size() != 4) return false;
    auto same3 = [](const gp_Pnt& a, const gp_Pnt& b) {
        return a.SquareDistance(b) < 1e-16;
    };
    const gp_Pnt c0 = sides[0].front();
    const gp_Pnt c1 = sides[0].back();
    const gp_Pnt c2 = sides[2].front();
    const gp_Pnt c3 = sides[2].back();
    int uniq = 1;
    if (!same3(c1, c0)) ++uniq;
    if (!same3(c2, c0) && !same3(c2, c1)) ++uniq;
    if (!same3(c3, c0) && !same3(c3, c1) && !same3(c3, c2)) ++uniq;
    if (uniq < 3) return false;
    const int nu = std::max(int(sides[0].size()), int(sides[2].size())) - 1;
    const int nv = std::max(int(sides[1].size()), int(sides[3].size())) - 1;
    if (nu < 1 || nv < 1) return false;
    auto pickPnt = [](const std::vector<gp_Pnt>& side, int i, int nDst) {
        const int nSrc = int(side.size()) - 1;
        if (nSrc <= 0) return side.front();
        const int j = nDst <= 0 ? 0 : (i * nSrc + nDst / 2) / nDst;
        return side[size_t(std::clamp(j, 0, nSrc))];
    };
    BRepAdaptor_Surface surf(face);
    auto uvOf = [&](const gp_Pnt& p) {
        try {
            Handle(Geom_Surface) gs = BRep_Tool::Surface(face);
            ShapeAnalysis_Surface sas(gs);
            return sas.ValueOfUV(p, 1e-6);
        } catch (const Standard_Failure&) {
            return gp_Pnt2d(0, 0);
        }
    };
    std::vector<std::vector<gp_Pnt2d>> uv(4);
    for (int s = 0; s < 4; ++s) {
        uv[s].reserve(sides[s].size());
        for (const gp_Pnt& p : sides[s]) uv[s].push_back(uvOf(p));
    }
    auto pickUv = [](const std::vector<gp_Pnt2d>& side, int i, int nDst) {
        const int nSrc = int(side.size()) - 1;
        if (nSrc <= 0) return side.front();
        const int j = nDst <= 0 ? 0 : (i * nSrc + nDst / 2) / nDst;
        return side[size_t(std::clamp(j, 0, nSrc))];
    };
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    const int cols = nu + 1;
    const int rows = nv + 1;
    std::vector<uint32_t> idx(size_t(rows) * size_t(cols));
    const gp_Pnt2d p00 = pickUv(uv[0], 0, nu);
    const gp_Pnt2d p10 = pickUv(uv[0], nu, nu);
    const gp_Pnt2d p11 = pickUv(uv[1], nv, nv);
    const gp_Pnt2d p01 = pickUv(uv[2], nu, nu);
    for (int j = 0; j < rows; ++j) {
        const double t = double(j) / double(nv);
        for (int i = 0; i < cols; ++i) {
            const double s = double(i) / double(nu);
            gp_Pnt p;
            if (j == 0) p = pickPnt(sides[0], i, nu);
            else if (i == nu) p = pickPnt(sides[1], j, nv);
            else if (j == nv) p = pickPnt(sides[2], nu - i, nu);
            else if (i == 0) p = pickPnt(sides[3], nv - j, nv);
            else {
                const gp_Pnt2d c0u = pickUv(uv[0], i, nu);
                const gp_Pnt2d c1u = pickUv(uv[2], nu - i, nu);
                const gp_Pnt2d d0u = pickUv(uv[3], nv - j, nv);
                const gp_Pnt2d d1u = pickUv(uv[1], j, nv);
                const double uu =
                    (1 - t) * c0u.X() + t * c1u.X() + (1 - s) * d0u.X() +
                    s * d1u.X() - (1 - s) * (1 - t) * p00.X() -
                    s * (1 - t) * p10.X() - (1 - s) * t * p01.X() -
                    s * t * p11.X();
                const double vv =
                    (1 - t) * c0u.Y() + t * c1u.Y() + (1 - s) * d0u.Y() +
                    s * d1u.Y() - (1 - s) * (1 - t) * p00.Y() -
                    s * (1 - t) * p10.Y() - (1 - s) * t * p01.Y() -
                    s * t * p11.Y();
                try {
                    p = surf.Value(uu, vv);
                } catch (const Standard_Failure&) {
                    return false;
                }
            }
            idx[size_t(j) * size_t(cols) + size_t(i)] =
                uint32_t(mesh.vertices.size());
            mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
            mesh.anchors.push_back(projectAnchor(face, faceId, p));
        }
    }
    const size_t before = mesh.polygonCount();
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            const uint32_t a = idx[size_t(j) * size_t(cols) + size_t(i)];
            const uint32_t b = idx[size_t(j) * size_t(cols) + size_t(i + 1)];
            const uint32_t c = idx[size_t(j + 1) * size_t(cols) + size_t(i + 1)];
            const uint32_t d = idx[size_t(j + 1) * size_t(cols) + size_t(i)];
            if (flip) mesh.polygons.push_back({a, d, c, b});
            else mesh.polygons.push_back({a, b, c, d});
            mesh.polygonFaceId.push_back(faceId);
        }
    }
    return mesh.polygonCount() > before;
}

void meshFaceOcct(const TopoDS_Face& face, int faceId,
                  const FaceMeshSettings& s, double radius,
                  FeatureClass feature, const Model& model,
                  const std::vector<int>& edgeN,
                  const std::vector<std::vector<gp_Pnt>>& samples,
                  PolyMesh& mesh) {
    double defl = faceChord(face, s);
    double angleRad = std::max(1.0, s.angleToleranceDeg) * M_PI / 180.0;
    int maxN = 1;
    std::vector<gp_Pnt> border;
    double spacing = 1e300;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const int eid = model.edges.FindIndex(ex.Current());
        if (eid < 1 || eid >= int(edgeN.size())) continue;
        maxN = std::max(maxN, edgeN[eid]);
        if (eid < int(samples.size())) {
            const auto& pts = samples[eid];
            for (size_t i = 0; i < pts.size(); ++i) {
                border.push_back(pts[i]);
                if (i > 0) {
                    spacing = std::min(spacing, pts[i - 1].Distance(pts[i]));
                }
            }
        }
    }
    if (maxN > 1) {
        const double span = 2.0 * M_PI / double(maxN);
        angleRad = std::min(angleRad, span);
        if (radius > 1e-12) {
            defl = std::max(defl, radius * (1.0 - std::cos(span * 0.5)));
        }
    }
    if (feature == FeatureClass::FilletStrip && s.filletLoops > 0 &&
        radius > 1e-12) {
        const double span = M_PI / double(std::max(1, s.filletLoops));
        defl = std::max(defl, radius * (1.0 - std::cos(span * 0.5)));
    }

    Handle(Poly_Triangulation) tri;
    TopLoc_Location loc;
    {
        static std::mutex occtMeshMutex;
        std::lock_guard<std::mutex> lock(occtMeshMutex);
        BRepBuilderAPI_Copy copier(face, Standard_False, Standard_False);
        const TopoDS_Face copy = TopoDS::Face(copier.Shape());
        BRepTools::Clean(copy);
        IMeshTools_Parameters mp;
        mp.Deflection = defl;
        mp.Angle = angleRad;
        mp.Relative = Standard_False;
        if (s.minSize > 0) mp.MinSize = s.minSize;
        mp.InParallel = Standard_False;
        BRepMesh_IncrementalMesh mesher(copy, mp);
        tri = BRep_Tool::Triangulation(copy, loc);
    }
    if (tri.IsNull() || tri->NbTriangles() < 1) return;

    const bool flip = face.Orientation() == TopAbs_REVERSED;
    const bool hasUV = tri->HasUVNodes();
    const gp_Trsf trsf = loc.Transformation();
    const uint32_t base = uint32_t(mesh.vertices.size());
    const double snap = (spacing < 1e299) ? 0.45 * spacing : 0.0;
    for (int i = 1; i <= tri->NbNodes(); ++i) {
        gp_Pnt p = tri->Node(i).Transformed(trsf);
        if (snap > 0.0 && !border.empty()) {
            double dist = 0;
            const gp_Pnt q = nearestSample(p, border, dist);
            if (dist <= snap) p = q;
        }
        mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
        Anchor a;
        a.faceId = faceId;
        if (hasUV) {
            const gp_Pnt2d uv = tri->UVNode(i);
            a.u = uv.X();
            a.v = uv.Y();
        }
        mesh.anchors.push_back(a);
    }
    for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int n1, n2, n3;
        tri->Triangle(i).Get(n1, n2, n3);
        if (flip) std::swap(n2, n3);
        mesh.polygons.push_back({base + uint32_t(n1 - 1),
                                 base + uint32_t(n2 - 1),
                                 base + uint32_t(n3 - 1)});
        mesh.polygonFaceId.push_back(faceId);
    }
}

}  // namespace

PolyMesh meshIndependent(const Model& model, const Analysis& analysis,
                         const GenerationSettings& settings,
                         GenerationReport* report) {
    PolyMesh mesh;
    const int faceN = model.faceCount();
    const int edgeNmax = model.edgeCount();
    std::vector<int> edgeN(size_t(edgeNmax) + 1, 1);

    auto angleOf = [&](int fid) {
        return std::max(1.0, settings.forFace(fid).angleToleranceDeg) * M_PI /
               180.0;
    };

    for (int eid = 1; eid <= edgeNmax; ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        double chord = 1e300;
        double angle = 1e300;
        int minCurve = 1;
        const auto& eFaces = (eid <= int(analysis.edges.size()))
                                 ? analysis.edges[size_t(eid) - 1].faceIds
                                 : std::vector<int>{};
        if (eFaces.empty()) {
            chord = settings.defaults.chordTolerance;
            angle = angleOf(0);
            minCurve = settings.defaults.minCurvedSegments;
        } else {
            for (int fid : eFaces) {
                const FaceMeshSettings& s = settings.forFace(fid);
                TopoDS_Face face = TopoDS::Face(model.faces(fid));
                chord = std::min(chord, faceChord(face, s));
                angle = std::min(angle, angleOf(fid));
                minCurve = std::max(minCurve, s.minCurvedSegments);
            }
        }
        if (!(chord < 1e299)) chord = settings.defaults.chordTolerance;
        if (!(angle < 1e299)) angle = angleOf(0);
        int n = curveSegmentCount(edge, angle, chord);
        if (edgeIsClosedCurve(edge)) n = std::max(n, std::clamp(minCurve, 1, 256));
        auto pin = settings.perEdge.find(eid);
        if (pin != settings.perEdge.end()) n = std::max(1, pin->second);
        edgeN[eid] = n;
    }

    for (int fid = 1; fid <= faceN; ++fid) {
        if (settings.forFace(fid).exclude) continue;
        const FaceMeshSettings& s = settings.forFace(fid);
        FeatureClass fc = FeatureClass::Freeform;
        if (fid <= int(analysis.faces.size())) {
            fc = analysis.faces[size_t(fid) - 1].featureClass;
        }
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
            const int eid = model.edges.FindIndex(ex.Current());
            if (eid < 1) continue;
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            if (settings.perEdge.count(eid)) continue;
            int n = edgeN[eid];
            if (fc == FeatureClass::Drum || fc == FeatureClass::SphereCap) {
                if (edgeIsClosedCurve(edge)) n = std::max(n, std::max(3, s.radial));
                else if (edgeIsLine(edge)) n = std::max(n, std::max(1, s.axial));
            }
            if ((fc == FeatureClass::HolePlate ||
                 fc == FeatureClass::BossJunction) &&
                edgeIsClosedCurve(edge)) {
                n = std::max(n, std::max(3, s.radial));
            }
            edgeN[eid] = n;
        }
    }

    if (report) {
        for (int eid = 1; eid <= edgeNmax; ++eid) {
            report->edgeDivisions[eid] = edgeN[eid];
        }
    }

    std::vector<std::vector<gp_Pnt>> samples(size_t(edgeNmax) + 1);
    for (int eid = 1; eid <= edgeNmax; ++eid) {
        samples[eid] =
            sampleEdgeForward(TopoDS::Edge(model.edges(eid)), edgeN[eid]);
    }

    for (int fid = 1; fid <= faceN; ++fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        if (s.exclude) {
            if (report) report->remeshedFaces.push_back(fid);
            continue;
        }
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        FeatureClass fc = FeatureClass::Freeform;
        ChartKind ck = ChartKind::FreeTrim;
        double radius = 0.0;
        if (fid <= int(analysis.faces.size())) {
            fc = analysis.faces[size_t(fid) - 1].featureClass;
            ck = analysis.faces[size_t(fid) - 1].chartKind;
            radius = analysis.faces[size_t(fid) - 1].radius;
        }
        const size_t polysBefore = mesh.polygonCount();
        const size_t vertsBefore = mesh.vertices.size();
        MesherKind kind = MesherKind::Fallback;
        bool ngon = false;
        auto rollbackFace = [&] {
            mesh.vertices.resize(vertsBefore);
            mesh.anchors.resize(vertsBefore);
            mesh.polygons.resize(polysBefore);
            mesh.polygonFaceId.resize(polysBefore);
        };
        try {
            if (s.minimal &&
                meshSampledPlanar(face, fid, model, samples, mesh)) {
                ngon = true;
                kind = MesherKind::MinimalNGon;
            } else if ((ck == ChartKind::FullPeriod ||
                        ck == ChartKind::Pole) &&
                       meshDrumGrid(face, fid, s, edgeN, model, samples,
                                    mesh)) {
                kind = MesherKind::RevolutionGrid;
            } else if (meshTransfinite4(face, fid, model, samples, mesh)) {
                kind = MesherKind::CoonsGrid;
            } else if (meshUvFill(face, fid, s, model, samples, mesh)) {
                kind = MesherKind::PlateWeb;
            } else {
                meshFaceOcct(face, fid, s, radius, fc, model, edgeN, samples,
                             mesh);
            }
        } catch (const Standard_Failure&) {
            rollbackFace();
            try {
                meshFaceOcct(face, fid, s, radius, fc, model, edgeN, samples,
                             mesh);
            } catch (const Standard_Failure&) {
                rollbackFace();
            }
        }
        const bool empty = mesh.polygonCount() == polysBefore;
        if (report) {
            report->faceMesher[fid] = kind;
            report->faceFeatureClass[fid] = fc;
            report->faceChartKind[fid] = ck;
            report->faceBuild[fid] = empty ? -1 : 0;
            if (empty) report->faceBuildCause[fid] = "independent tessellation empty";
            report->remeshedFaces.push_back(fid);
            ++report->cacheMisses;
            if (ngon) {
                report->faceCounts[fid] = {
                    int(mesh.polygons.back().size()), 1};
            }
        }
    }

    const double weld = std::max(settings.weldTolerance, 1e-4);
    weldVertices(mesh, weld);
    return mesh;
}

}  // namespace weft
