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
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_Surface.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
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
    double t = 0.0;  // curve parameter
};

double effectiveSag(const FaceMeshSettings& s, double modelDiagonal) {
    double sag = std::max(1e-9, s.chordTolerance);
    if (s.relativeDeviation && modelDiagonal > 0.0) {
        // Pixyz-style: min(maxSag, diag * sagRatio). relativeDeviation reuses
        // chordTolerance as maxSag and minSize as sagRatio when > 0.
        const double ratio = s.minSize > 0.0 ? s.minSize : 0.0003;
        sag = std::min(sag, modelDiagonal * ratio);
    }
    return sag;
}

double modelDiagonal(const Model& model) {
    if (model.shape.IsNull()) return 0.0;
    double x0, y0, z0, x1, y1, z1;
    Bnd_Box bb;
    BRepBndLib::Add(model.shape, bb);
    if (bb.IsVoid()) return 0.0;
    bb.Get(x0, y0, z0, x1, y1, z1);
    return gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1));
}

// Sample a 3D edge curve to sag / optional angle / optional maxLength.
std::vector<SamplePoint> sampleEdgeCurve(const TopoDS_Edge& edge, double sag,
                                         double angleDeg, double maxLength) {
    std::vector<SamplePoint> out;
    if (BRep_Tool::Degenerated(edge)) return out;
    BRepAdaptor_Curve curve(edge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    if (!(last > first)) return out;

    const double angleTol =
        angleDeg > 0.0 ? angleDeg * M_PI / 180.0 : 1e9;  // off ≈ unconstrained
    const int count =
        mesher_detail::stableDeflectionCount(curve, angleTol, sag);
    const int n = std::max(1, count);

    auto push = [&](double t) {
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
                // Split long chords until under maxLength.
                const int splits = std::max(1, int(std::ceil(dist / maxLength)));
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
        out.push_back(sp);
    };

    for (int i = 0; i <= n; ++i) {
        push(first + (last - first) * (double(i) / double(n)));
    }
    return out;
}

struct MeshBuilder {
    PolyMesh mesh;
    std::unordered_map<int64_t, uint32_t> weld;  // quantized key -> index

    static int64_t quantize(const gp_Pnt& p, double tol) {
        const double s = 1.0 / std::max(tol, 1e-12);
        const int64_t x = int64_t(std::llround(p.X() * s));
        const int64_t y = int64_t(std::llround(p.Y() * s));
        const int64_t z = int64_t(std::llround(p.Z() * s));
        return (x * 73856093) ^ (y * 19349663) ^ (z * 83492791);
    }

    uint32_t addVertex(const gp_Pnt& p, const Anchor& a, double weldTol) {
        const int64_t key = quantize(p, weldTol);
        auto it = weld.find(key);
        if (it != weld.end()) return it->second;
        const uint32_t idx = uint32_t(mesh.vertices.size());
        mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
        mesh.anchors.push_back(a);
        weld.emplace(key, idx);
        return idx;
    }

    void addPolygon(std::vector<uint32_t> idxs, int faceId) {
        if (idxs.size() < 3) return;
        // Drop consecutive duplicates from weld.
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

bool isPlanarFace(const TopoDS_Face& face) {
    return BRepAdaptor_Surface(face).GetType() == GeomAbs_Plane;
}

// Project a 3D point onto the face surface for UV anchors.
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

// Ordered samples along one wire (outer or hole), skipping micro-edges.
std::vector<gp_Pnt> sampleWire(const TopoDS_Wire& wire, const TopoDS_Face& face,
                               double sag, double angleDeg, double maxLength) {
    std::vector<gp_Pnt> pts;
    for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        auto samples = sampleEdgeCurve(edge, sag, angleDeg, maxLength);
        // Edges may be reversed relative to the wire.
        if (edge.Orientation() == TopAbs_REVERSED) {
            std::reverse(samples.begin(), samples.end());
        }
        for (size_t i = 0; i < samples.size(); ++i) {
            // Skip first point of subsequent edges (shared vertex).
            if (!pts.empty() && i == 0) continue;
            pts.push_back(samples[i].p);
        }
    }
    if (pts.size() >= 2 && pts.front().Distance(pts.back()) < sag * 0.5) {
        pts.pop_back();
    }
    (void)face;
    return pts;
}

void meshPlanarFace(const TopoDS_Face& face, int faceId, double sag,
                    double angleDeg, double maxLength, double weldTol,
                    MeshBuilder& out) {
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return;
    auto ring = sampleWire(outer, face, sag, angleDeg, maxLength);
    if (ring.size() < 3) return;

    std::vector<uint32_t> idxs;
    idxs.reserve(ring.size());
    for (const gp_Pnt& p : ring) {
        idxs.push_back(out.addVertex(p, anchorOnFace(face, faceId, p), weldTol));
    }
    // Face orientation.
    if (face.Orientation() == TopAbs_REVERSED) {
        std::reverse(idxs.begin(), idxs.end());
    }
    out.addPolygon(std::move(idxs), faceId);

    // Holes: emit as separate n-gons for now (bridge/cleanup later).
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        const TopoDS_Wire w = TopoDS::Wire(wx.Current());
        if (w.IsSame(outer)) continue;
        auto hole = sampleWire(w, face, sag, angleDeg, maxLength);
        if (hole.size() < 3) continue;
        std::vector<uint32_t> hidxs;
        for (const gp_Pnt& p : hole) {
            hidxs.push_back(
                out.addVertex(p, anchorOnFace(face, faceId, p), weldTol));
        }
        if (face.Orientation() == TopAbs_REVERSED) {
            std::reverse(hidxs.begin(), hidxs.end());
        }
        out.addPolygon(std::move(hidxs), faceId);
    }
}

// Coarse adaptive UV tessellation for curved faces (not OCCT BRepMesh).
void meshCurvedFace(const TopoDS_Face& face, int faceId, double sag,
                    double angleDeg, double maxLength, double weldTol,
                    MeshBuilder& out) {
    BRepAdaptor_Surface surf(face);
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    if (!(u1 > u0) || !(v1 > v0)) {
        // Degenerate parametric domain — fall back to outer-wire fan.
        meshPlanarFace(face, faceId, sag, angleDeg, maxLength, weldTol, out);
        return;
    }

    const double angleTol =
        angleDeg > 0.0 ? angleDeg * M_PI / 180.0 : 1e9;

    auto eval = [&](double u, double v, gp_Pnt& p, gp_Vec& du, gp_Vec& dv) {
        surf.D1(u, v, p, du, dv);
    };

    // Start with a coarse grid sized from edge-length / sag heuristics.
    const double uSpan = u1 - u0;
    const double vSpan = v1 - v0;
    gp_Pnt c00, c10, c01;
    gp_Vec d;
    eval(u0, v0, c00, d, d);
    eval(u1, v0, c10, d, d);
    eval(u0, v1, c01, d, d);
    const double uLen = c00.Distance(c10);
    const double vLen = c00.Distance(c01);
    int nu = std::max(1, int(std::ceil(uLen / std::max(sag * 4.0, 1e-6))));
    int nv = std::max(1, int(std::ceil(vLen / std::max(sag * 4.0, 1e-6))));
    nu = std::min(nu, 64);
    nv = std::min(nv, 64);

    // Refine: double resolution while any cell midpoint fails sag/angle.
    for (int pass = 0; pass < 4; ++pass) {
        bool needRefine = false;
        for (int i = 0; i < nu && !needRefine; ++i) {
            for (int j = 0; j < nv && !needRefine; ++j) {
                const double ua = u0 + uSpan * (double(i) / nu);
                const double ub = u0 + uSpan * (double(i + 1) / nu);
                const double va = v0 + vSpan * (double(j) / nv);
                const double vb = v0 + vSpan * (double(j + 1) / nv);
                const double um = 0.5 * (ua + ub);
                const double vm = 0.5 * (va + vb);
                gp_Pnt p00, p10, p01, p11, pm, pExact;
                gp_Vec du, dv;
                eval(ua, va, p00, du, dv);
                eval(ub, va, p10, du, dv);
                eval(ua, vb, p01, du, dv);
                eval(ub, vb, p11, du, dv);
                eval(um, vm, pExact, du, dv);
                pm = gp_Pnt((p00.XYZ() + p10.XYZ() + p01.XYZ() + p11.XYZ()) *
                            0.25);
                if (pm.Distance(pExact) > sag) {
                    needRefine = true;
                    break;
                }
                gp_Vec n0 = du.Crossed(dv);
                if (n0.Magnitude() > 1e-12) {
                    gp_Vec du2, dv2;
                    gp_Pnt tmp;
                    eval(ua, va, tmp, du2, dv2);
                    gp_Vec nA = du2.Crossed(dv2);
                    eval(ub, vb, tmp, du2, dv2);
                    gp_Vec nB = du2.Crossed(dv2);
                    if (nA.Magnitude() > 1e-12 && nB.Magnitude() > 1e-12 &&
                        nA.Angle(nB) > angleTol) {
                        needRefine = true;
                        break;
                    }
                }
                if (maxLength > 0.0) {
                    if (p00.Distance(p10) > maxLength ||
                        p00.Distance(p01) > maxLength) {
                        needRefine = true;
                        break;
                    }
                }
            }
        }
        if (!needRefine) break;
        nu = std::min(128, nu * 2);
        nv = std::min(128, nv * 2);
    }

    // Emit triangles for the UV grid (full rectangle — trim holes ignored in
    // this base pass; outer-wire sampling still used for planar faces).
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    std::vector<std::vector<uint32_t>> grid(nv + 1,
                                            std::vector<uint32_t>(nu + 1));
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            const double u = u0 + uSpan * (double(i) / nu);
            const double v = v0 + vSpan * (double(j) / nv);
            gp_Pnt p;
            gp_Vec du, dv;
            eval(u, v, p, du, dv);
            Anchor a{faceId, u, v};
            grid[j][i] = out.addVertex(p, a, weldTol);
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            uint32_t a = grid[j][i];
            uint32_t b = grid[j][i + 1];
            uint32_t c = grid[j + 1][i + 1];
            uint32_t d = grid[j + 1][i];
            if (!flip) {
                out.addPolygon({a, b, c}, faceId);
                out.addPolygon({a, c, d}, faceId);
            } else {
                out.addPolygon({a, c, b}, faceId);
                out.addPolygon({a, d, c}, faceId);
            }
        }
    }
}

}  // namespace

void applyQualityPreset(FaceMeshSettings& s, QualityPreset preset) {
    s.angleToleranceDeg = -1;  // Pixyz presets leave angle off
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
    GenerationSettings settings = settingsIn;
    MeshBuilder builder;
    const double diag = modelDiagonal(model);
    const double weldTol = std::max(settings.weldTolerance, 1e-9);

    if (report) {
        report->cacheHits = 0;
        report->cacheMisses = model.faceCount();
        report->faceMesher.clear();
        report->faceBuild.clear();
        report->faceBuildCause.clear();
        report->faceFeatureClass.clear();
        report->faceChartKind.clear();
    }

    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const FaceMeshSettings& fs = settings.forFace(fid);
        if (fs.exclude) continue;

        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const double sag = effectiveSag(fs, diag);
        const double angle = fs.angleToleranceDeg;
        const double maxLen = fs.maxLength;

        if (report) {
            const int idx = fid - 1;
            if (idx >= 0 && idx < int(analysis.faces.size())) {
                report->faceFeatureClass[fid] = analysis.faces[idx].featureClass;
                report->faceChartKind[fid] = analysis.faces[idx].chartKind;
            }
            report->remeshedFaces.push_back(fid);
        }

        if (isPlanarFace(face)) {
            meshPlanarFace(face, fid, sag, angle, maxLen, weldTol, builder);
            if (report) {
                report->faceMesher[fid] = MesherKind::MinimalNGon;
                report->faceBuild[fid] = 0;
            }
        } else {
            meshCurvedFace(face, fid, sag, angle, maxLen, weldTol, builder);
            if (report) {
                report->faceMesher[fid] = MesherKind::Fallback;
                report->faceBuild[fid] = 0;
                report->faceBuildCause[fid] = "accuracy-uv-grid";
            }
        }
    }

    PolyMesh mesh = std::move(builder.mesh);
    if (settings.finalizeMesh) {
        weldVertices(mesh, weldTol);
    }
    return mesh;
}

}  // namespace weft
