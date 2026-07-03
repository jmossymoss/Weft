#include "weft/analysis.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp_Face.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_ListOfShape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace weft {

const char* surfaceTypeName(SurfaceType t) {
    switch (t) {
        case SurfaceType::Plane: return "plane";
        case SurfaceType::Cylinder: return "cylinder";
        case SurfaceType::Cone: return "cone";
        case SurfaceType::Sphere: return "sphere";
        case SurfaceType::Torus: return "torus";
        case SurfaceType::Revolution: return "revolution";
        case SurfaceType::Extrusion: return "extrusion";
        case SurfaceType::BSpline: return "bspline";
        case SurfaceType::Bezier: return "bezier";
        case SurfaceType::Offset: return "offset";
        case SurfaceType::Other: return "other";
    }
    return "other";
}

const char* edgeConvexityName(EdgeConvexity c) {
    switch (c) {
        case EdgeConvexity::Convex: return "convex";
        case EdgeConvexity::Concave: return "concave";
        case EdgeConvexity::Smooth: return "smooth";
        case EdgeConvexity::Boundary: return "boundary";
    }
    return "boundary";
}

static SurfaceType classifySurface(const TopoDS_Face& face, double& radiusOut) {
    BRepAdaptor_Surface surf(face);
    radiusOut = 0.0;
    switch (surf.GetType()) {
        case GeomAbs_Plane: return SurfaceType::Plane;
        case GeomAbs_Cylinder:
            radiusOut = surf.Cylinder().Radius();
            return SurfaceType::Cylinder;
        case GeomAbs_Cone:
            radiusOut = surf.Cone().RefRadius();
            return SurfaceType::Cone;
        case GeomAbs_Sphere:
            radiusOut = surf.Sphere().Radius();
            return SurfaceType::Sphere;
        case GeomAbs_Torus:
            radiusOut = surf.Torus().MajorRadius();
            return SurfaceType::Torus;
        case GeomAbs_SurfaceOfRevolution: return SurfaceType::Revolution;
        case GeomAbs_SurfaceOfExtrusion: return SurfaceType::Extrusion;
        case GeomAbs_BSplineSurface: return SurfaceType::BSpline;
        case GeomAbs_BezierSurface: return SurfaceType::Bezier;
        case GeomAbs_OffsetSurface: return SurfaceType::Offset;
        default: return SurfaceType::Other;
    }
}

// Orientation of `edge` as it appears in `face`'s wires (FORWARD/REVERSED),
// needed to orient the edge tangent consistently for the convexity test.
static TopAbs_Orientation edgeOrientationInFace(const TopoDS_Edge& edge,
                                                const TopoDS_Face& face) {
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        if (ex.Current().IsSame(edge)) return ex.Current().Orientation();
    }
    return TopAbs_FORWARD;
}

// Outward face normal at the edge midpoint, via the edge's pcurve on the face.
// BRepGProp_Face accounts for face orientation. Returns false if the edge has
// no pcurve on the face or the normal is degenerate.
static bool faceNormalAtEdgeMid(const TopoDS_Edge& edge, const TopoDS_Face& face,
                                double midParam, gp_Dir& normalOut) {
    double f = 0, l = 0;
    Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, f, l);
    if (pcurve.IsNull()) return false;
    gp_Pnt2d uv = pcurve->Value(midParam);

    gp_Pnt p;
    gp_Vec n;
    BRepGProp_Face props(face);
    props.Normal(uv.X(), uv.Y(), p, n);
    if (n.Magnitude() < 1e-12) return false;
    normalOut = gp_Dir(n);
    return true;
}

static constexpr double kSmoothToleranceDeg = 0.5;

Analysis analyze(const Model& model) {
    Analysis a;
    a.faces.resize(model.faceCount());
    a.edges.resize(model.edgeCount());

    // Object structure for the outliner: faces grouped per solid; shells
    // outside any solid count as their own objects; anything left over
    // (free faces) becomes one final group.
    std::vector<bool> grouped(model.faceCount() + 1, false);
    auto collect = [&](const TopoDS_Shape& obj) {
        std::vector<int> fids;
        for (TopExp_Explorer fx(obj, TopAbs_FACE); fx.More(); fx.Next()) {
            int fid = model.faces.FindIndex(fx.Current());
            if (fid > 0 && !grouped[fid]) {
                grouped[fid] = true;
                fids.push_back(fid);
            }
        }
        if (!fids.empty()) a.solidFaces.push_back(std::move(fids));
    };
    for (TopExp_Explorer sx(model.shape, TopAbs_SOLID); sx.More(); sx.Next()) {
        collect(sx.Current());
    }
    for (TopExp_Explorer sx(model.shape, TopAbs_SHELL, TopAbs_SOLID);
         sx.More(); sx.Next()) {
        collect(sx.Current());
    }
    {
        std::vector<int> loose;
        for (int fid = 1; fid <= model.faceCount(); ++fid) {
            if (!grouped[fid]) loose.push_back(fid);
        }
        if (!loose.empty()) a.solidFaces.push_back(std::move(loose));
    }

    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        FaceInfo& info = a.faces[fid - 1];
        info.id = fid;
        info.type = classifySurface(face, info.radius);
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
            int eid = model.edges.FindIndex(ex.Current());
            if (eid > 0 &&
                std::find(info.edgeIds.begin(), info.edgeIds.end(), eid) ==
                    info.edgeIds.end()) {
                info.edgeIds.push_back(eid);
            }
        }
    }

    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        EdgeInfo& info = a.edges[eid - 1];
        info.id = eid;
        if (!BRep_Tool::Degenerated(edge)) {
            double f, l;
            if (!BRep_Tool::Curve(edge, f, l).IsNull()) {
                BRepAdaptor_Curve c(edge);
                info.length = GCPnts_AbscissaPoint::Length(c);
            }
        }

        const TopTools_ListOfShape& owners = model.edgeToFaces.FindFromKey(edge);
        for (const TopoDS_Shape& s : owners) {
            int fid = model.faces.FindIndex(s);
            if (fid > 0 &&
                std::find(info.faceIds.begin(), info.faceIds.end(), fid) ==
                    info.faceIds.end()) {
                info.faceIds.push_back(fid);
            }
        }

        if (info.faceIds.size() < 2) {
            info.convexity = EdgeConvexity::Boundary;
            continue;
        }

        const TopoDS_Face face1 = TopoDS::Face(model.faces(info.faceIds[0]));
        const TopoDS_Face face2 = TopoDS::Face(model.faces(info.faceIds[1]));

        double f = 0, l = 0;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
        if (curve.IsNull()) {
            info.convexity = EdgeConvexity::Boundary;
            continue;
        }
        double mid = 0.5 * (f + l);

        gp_Dir n1, n2;
        if (!faceNormalAtEdgeMid(edge, face1, mid, n1) ||
            !faceNormalAtEdgeMid(edge, face2, mid, n2)) {
            info.convexity = EdgeConvexity::Boundary;
            continue;
        }

        info.dihedralDeg = n1.Angle(n2) * 180.0 / M_PI;
        if (info.dihedralDeg < kSmoothToleranceDeg) {
            info.convexity = EdgeConvexity::Smooth;
            continue;
        }

        gp_Pnt p;
        gp_Vec d1;
        curve->D1(mid, p, d1);
        if (edgeOrientationInFace(edge, face1) == TopAbs_REVERSED) d1.Reverse();

        // With t oriented as in face1's wire and outward normals n1/n2, the
        // triple product sign separates convex from concave dihedrals.
        double s = gp_Vec(n1).Crossed(gp_Vec(n2)).Dot(d1);
        info.convexity = s > 0 ? EdgeConvexity::Convex : EdgeConvexity::Concave;
    }

    // Fillets: cylindrical/toroidal faces stitched in by tangent joins.
    // Holes: full cylindrical bores, i.e. the surface's natural outward
    // normal is flipped so material lies outside the cylinder.
    for (FaceInfo& f : a.faces) {
        if (f.type != SurfaceType::Cylinder && f.type != SurfaceType::Torus) {
            continue;
        }
        int smooth = 0;
        for (int eid : f.edgeIds) {
            if (a.edges[eid - 1].convexity == EdgeConvexity::Smooth) ++smooth;
        }
        f.isFillet = smooth >= 2;

        if (f.type == SurfaceType::Cylinder) {
            const TopoDS_Face face = TopoDS::Face(model.faces(f.id));
            BRepAdaptor_Surface surf(face);
            f.isHole = surf.IsUClosed() &&
                       face.Orientation() == TopAbs_REVERSED;
        }
    }

    // Adjacency: two faces are neighbors when they share an edge.
    for (const EdgeInfo& e : a.edges) {
        for (int f1 : e.faceIds) {
            for (int f2 : e.faceIds) {
                if (f1 == f2) continue;
                auto& nbrs = a.faces[f1 - 1].neighborFaceIds;
                if (std::find(nbrs.begin(), nbrs.end(), f2) == nbrs.end()) {
                    nbrs.push_back(f2);
                }
            }
        }
    }

    return a;
}

}  // namespace weft
