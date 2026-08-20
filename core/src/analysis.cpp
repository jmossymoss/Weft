#include "weft/analysis.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <TopExp_Explorer.hxx>
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

const char* chartKindName(ChartKind k) {
    switch (k) {
        case ChartKind::FreeTrim: return "free-trim";
        case ChartKind::Pole: return "pole";
        case ChartKind::FullPeriod: return "full-period";
        case ChartKind::IsoBand: return "iso-band";
        case ChartKind::GeometricCap: return "geometric-cap";
    }
    return "free-trim";
}

const char* featureClassName(FeatureClass c) {
    switch (c) {
        case FeatureClass::Freeform: return "freeform";
        case FeatureClass::Drum: return "drum";
        case FeatureClass::SphereCap: return "sphere-cap";
        case FeatureClass::FilletStrip: return "fillet-strip";
        case FeatureClass::HolePlate: return "hole-plate";
        case FeatureClass::BossJunction: return "boss-junction";
        case FeatureClass::PlanarPanel: return "planar-panel";
    }
    return "freeform";
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

static LoopSignature classifyLoop(const TopoDS_Face& face) {
    LoopSignature sig;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    std::vector<gp_Pnt> outerPts;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        ++sig.wireCount;
        const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
        const bool isOuter = !outer.IsNull() && wire.IsSame(outer);
        for (TopExp_Explorer ex(wire, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(edge)) {
                ++sig.degEdgeCount;
                continue;
            }
            ++sig.realEdgeCount;
            if (!isOuter) continue;
            double f = 0, l = 0;
            Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
            if (c.IsNull()) continue;
            for (int i = 0; i < 8; ++i) {
                outerPts.push_back(
                    c->Value(f + (l - f) * (i + 0.5) / 8.0));
            }
        }
    }
    if (outerPts.size() >= 4) {
        gp_XYZ c(0, 0, 0);
        for (const gp_Pnt& p : outerPts) c += p.XYZ();
        c /= double(outerPts.size());
        double rMin = 1e300, rMax = 0;
        for (const gp_Pnt& p : outerPts) {
            const double r = p.XYZ().Subtracted(c).Modulus();
            rMin = std::min(rMin, r);
            rMax = std::max(rMax, r);
        }
        if (rMin > 1e-12) sig.outerRoundness = rMax / rMin;
    }
    return sig;
}

// Mirror of the mesher sphere chart probe: true when RevolutionGrid owns the
// UV chart; false for geometric caps (bullet tips, corner balls, half-domes)
// that need disk rings or a four-sided patch instead.
//
// RevolutionGrid's lattice is periodic by construction: it wraps column
// `nu - 1` back onto column `0` across its whole trimmed u range. So it owns
// a sphere patch only where u genuinely CLOSES. A degenerate edge says the
// chart TOUCHES a pole, and a collapsing iso says one side of it is a pole;
// neither says the chart goes AROUND that pole, so neither can stand in for
// closure. A patch trimmed to part of the period is bounded by meridians —
// mp9_Edited's instanced rib corner balls are spherical octants (quarter
// period, one pole edge, three real sides) and its half-domes are lunes
// (half period, two pole edges). Wrapping either welds its own opposite
// meridian onto itself, which is why the corner balls lost the border
// contract on the meridian they never sampled, or folded.
static bool sphereHasUvPoleChart(const TopoDS_Face& face,
                                 const BRepAdaptor_Surface& surf) {
    if (surf.GetType() != GeomAbs_Sphere) return false;
    Handle(Geom_Surface) S = BRep_Tool::Surface(face);
    double umin = 0, umax = 0, vmin = 0, vmax = 0;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    // Trimmed boolean spheres often report IsUClosed()=false on the adaptor,
    // so the period comes from the geometry rather than from that flag.
    const double period =
        (S && !S.IsNull() && S->IsUPeriodic()) ? S->UPeriod() : 2.0 * M_PI;
    return (umax - umin) >= 0.999 * period;
}

static ChartKind classifyChart(const TopoDS_Face& face, SurfaceType type,
                               const LoopSignature& loop) {
    BRepAdaptor_Surface surf(face);
    double umin = 0, umax = 0, vmin = 0, vmax = 0;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double uspan = umax - umin;
    Handle(Geom_Surface) S = BRep_Tool::Surface(face);

    if (type == SurfaceType::Sphere) {
        if (!sphereHasUvPoleChart(face, surf)) return ChartKind::GeometricCap;
        if (loop.degEdgeCount > 0) return ChartKind::Pole;
        if (S && !S.IsNull() && S->IsUPeriodic() &&
            uspan >= 0.999 * S->UPeriod()) {
            return ChartKind::FullPeriod;
        }
        if (surf.IsUClosed() && uspan >= 0.999 * 2.0 * M_PI) {
            return ChartKind::FullPeriod;
        }
        return ChartKind::Pole;
    }

    const bool analyticDrum = type == SurfaceType::Cylinder ||
                              type == SurfaceType::Cone ||
                              type == SurfaceType::Revolution ||
                              type == SurfaceType::Torus;
    if (analyticDrum) {
        if ((S && !S.IsNull() && S->IsUPeriodic() &&
             uspan >= 0.999 * S->UPeriod()) ||
            (surf.IsUClosed() && uspan >= 0.999 * 2.0 * M_PI) ||
            (type == SurfaceType::Sphere)) {
            return ChartKind::FullPeriod;
        }
        if (uspan >= 1.0) return ChartKind::IsoBand;
        return ChartKind::FreeTrim;
    }
    return ChartKind::FreeTrim;
}

static int priorityFor(FeatureClass fc, SurfaceType type) {
    // Product order: cylinder → sphere → hemisphere → box → torus → curves →
    // cuts. Hemisphere shares sphere-cap priority.
    switch (fc) {
        case FeatureClass::Drum:
            return type == SurfaceType::Cylinder ? 100 : 95;
        case FeatureClass::SphereCap:
            return 90;
        case FeatureClass::FilletStrip:
            return 70;
        case FeatureClass::BossJunction:
            return 60;
        case FeatureClass::HolePlate:
            return 50;
        case FeatureClass::PlanarPanel:
            return 40;
        case FeatureClass::Freeform:
            return type == SurfaceType::BSpline || type == SurfaceType::Bezier
                       ? 20
                       : 10;
    }
    return 10;
}

// Narrow constant-radius blend vs wide false-fillet drum (foam half-drums
// are isFillet but must stay Drum × IsoBand for open-band columns).
static bool isNarrowFilletStrip(const TopoDS_Face& face, double radius) {
    BRepAdaptor_Surface surf(face);
    const double vSpan =
        surf.LastVParameter() - surf.FirstVParameter();
    const double uSpan =
        surf.LastUParameter() - surf.FirstUParameter();
    if (radius <= 1e-9) return true;
    return vSpan <= 1.8 * radius || uSpan <= 1.9;
}

static FeatureClass classifyFeature(const TopoDS_Face& face, SurfaceType type,
                                    bool isFillet, bool isHole, ChartKind chart,
                                    const LoopSignature& loop,
                                    double radius) {
    if (type == SurfaceType::Sphere) return FeatureClass::SphereCap;

    if (isFillet && (type == SurfaceType::Cylinder ||
                     type == SurfaceType::Torus ||
                     type == SurfaceType::Cone)) {
        if (isNarrowFilletStrip(face, radius)) return FeatureClass::FilletStrip;
        // Wide tangent drum wrongly flagged as fillet — keep as Drum.
        return FeatureClass::Drum;
    }

    if (type == SurfaceType::Cylinder || type == SurfaceType::Cone ||
        type == SurfaceType::Revolution) {
        (void)isHole;
        (void)chart;
        return FeatureClass::Drum;
    }

    if (type == SurfaceType::Torus) {
        return isFillet && isNarrowFilletStrip(face, radius)
                   ? FeatureClass::FilletStrip
                   : FeatureClass::Freeform;
    }

    if (type == SurfaceType::Plane) {
        if (loop.wireCount >= 2) {
            // Roundish outer + inner wires → hole plate; a single compact
            // annular web with a cylindrical neighbor is often a boss
            // junction — treat multi-wire planes with round outer as
            // boss-junction when outerRoundness is circle-like.
            if (loop.outerRoundness <= 1.35 && loop.wireCount == 2) {
                return FeatureClass::BossJunction;
            }
            return FeatureClass::HolePlate;
        }
        return FeatureClass::PlanarPanel;
    }

    return FeatureClass::Freeform;
}

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
        info.loop = classifyLoop(face);
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

        const auto& owners = model.edgeToFaces.FindFromKey(edge);
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

    // AD-5: chart + feature class + priority (once per face).
    for (FaceInfo& f : a.faces) {
        const TopoDS_Face face = TopoDS::Face(model.faces(f.id));
        f.chartKind = classifyChart(face, f.type, f.loop);
        f.featureClass = classifyFeature(face, f.type, f.isFillet, f.isHole,
                                         f.chartKind, f.loop, f.radius);
        // Wide false-fillets reclassified as Drum keep an iso/full chart.
        if (f.featureClass == FeatureClass::Drum &&
            f.chartKind == ChartKind::FreeTrim &&
            (f.type == SurfaceType::Cylinder || f.type == SurfaceType::Cone ||
             f.type == SurfaceType::Revolution)) {
            f.chartKind = classifyChart(face, f.type, f.loop);
        }
        f.priority = priorityFor(f.featureClass, f.type);
    }

    // Flat CSR adjacency for generate() — no TopExp on the interactive path.
    a.topology.buildFromAnalysis(a);
    a.topology.fillFaceAreas(model);

    return a;
}

}  // namespace weft
