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

bool meshPlanarNgon(const TopoDS_Face& face, int faceId, const Model& model,
                    const std::vector<std::vector<gp_Pnt>>& samples,
                    PolyMesh& mesh) {
    if (!isPlanarFace(face)) return false;
    if (wireCount(face) != 1) return false;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    std::vector<gp_Pnt> loop;
    for (BRepTools_WireExplorer ex(outer, face); ex.More(); ex.Next()) {
        const TopoDS_Edge e = ex.Current();
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
        if (samp.size() < 2) continue;
        const size_t start = loop.empty() ? 0 : 1;
        for (size_t i = start; i < samp.size(); ++i) loop.push_back(samp[i]);
    }
    const size_t before = mesh.polygonCount();
    appendPolygon(mesh, faceId, face, std::move(loop), false);
    return mesh.polygonCount() > before;
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

bool meshDrumGrid(const TopoDS_Face& face, int faceId,
                  const FaceMeshSettings& s, const std::vector<int>& edgeN,
                  const Model& model, PolyMesh& mesh) {
    BRepAdaptor_Surface surf(face);
    const GeomAbs_SurfaceType ty = surf.GetType();
    if (ty != GeomAbs_Cylinder && ty != GeomAbs_Cone &&
        ty != GeomAbs_SurfaceOfRevolution && ty != GeomAbs_Torus) {
        return false;
    }
    int nu = std::max(3, s.radial);
    int nv = std::max(1, s.axial);
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const int eid = model.edges.FindIndex(ex.Current());
        if (eid < 1 || eid >= int(edgeN.size())) continue;
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (edgeIsClosedCurve(edge)) nu = std::max(nu, edgeN[eid]);
        else if (edgeIsLine(edge)) nv = std::max(nv, edgeN[eid]);
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
        MesherKind kind = MesherKind::Fallback;
        bool ngon = false;
        if (s.minimal && meshPlanarNgon(face, fid, model, samples, mesh)) {
            ngon = true;
            kind = MesherKind::MinimalNGon;
        } else if (fc == FeatureClass::Drum &&
                   meshDrumGrid(face, fid, s, edgeN, model, mesh)) {
            kind = MesherKind::RevolutionGrid;
        } else {
            meshFaceOcct(face, fid, s, radius, fc, model, edgeN, samples, mesh);
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
