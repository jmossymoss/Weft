#include "weft/meshers.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <ElCLib.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace weft {

const char* mesherKindName(MesherKind k) {
    switch (k) {
        case MesherKind::CylinderGrid: return "cylinder-grid";
        case MesherKind::DiskCap: return "disk-cap";
        case MesherKind::PlanarGrid: return "parametric-grid";
        case MesherKind::Fallback: return "fallback-tri";
    }
    return "fallback-tri";
}

namespace {

class MeshBuilder {
public:
    explicit MeshBuilder(PolyMesh& mesh) : mesh_(mesh) {}

    uint32_t addVertex(const gp_Pnt& p) {
        mesh_.vertices.push_back({p.X(), p.Y(), p.Z()});
        return static_cast<uint32_t>(mesh_.vertices.size() - 1);
    }

    void addPolygon(std::vector<uint32_t> indices, int faceId, bool flip) {
        if (flip) std::reverse(indices.begin(), indices.end());
        mesh_.polygons.push_back(std::move(indices));
        mesh_.polygonFaceId.push_back(faceId);
    }

private:
    PolyMesh& mesh_;
};

bool isFullRevolutionCylinder(const BRepAdaptor_Surface& surf) {
    if (surf.GetType() != GeomAbs_Cylinder) return false;
    return std::abs((surf.LastUParameter() - surf.FirstUParameter()) - 2.0 * M_PI) <
           1e-7;
}

// Full-revolution cylindrical side face: exact radial x axial quad grid.
// u is the angle, v runs along the axis; the seam column is shared by wrap.
void meshCylinderGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                      int faceId, const FaceMeshSettings& s, MeshBuilder& out) {
    const int nu = std::max(3, s.radial);
    const int nv = std::max(1, s.axial);
    const double u0 = surf.FirstUParameter();
    const double v0 = surf.FirstVParameter();
    const double du = 2.0 * M_PI / nu;
    const double dv = (surf.LastVParameter() - v0) / nv;
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    std::vector<uint32_t> ring((nu) * (nv + 1));
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            ring[j * nu + i] = out.addVertex(surf.Value(u0 + i * du, v0 + j * dv));
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            int i2 = (i + 1) % nu;
            out.addPolygon({ring[j * nu + i], ring[j * nu + i2],
                            ring[(j + 1) * nu + i2], ring[(j + 1) * nu + i]},
                           faceId, flip);
        }
    }
}

// A planar face bounded by exactly one full-circle edge (a cylinder cap).
bool boundingCircle(const TopoDS_Face& face, gp_Circ& circOut) {
    int edgeCount = 0;
    TopoDS_Edge only;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        ++edgeCount;
        only = TopoDS::Edge(ex.Current());
    }
    if (edgeCount != 1) return false;

    double f = 0, l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(only, f, l);
    Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
    if (circle.IsNull()) return false;
    if (std::abs((l - f) - 2.0 * M_PI) > 1e-7) return false;
    circOut = circle->Circ();
    return true;
}

// Outward normal of a planar face (accounts for face orientation).
gp_Vec planarFaceNormal(const TopoDS_Face& face, const BRepAdaptor_Surface& surf) {
    gp_Vec n(surf.Plane().Axis().Direction());
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    return n;
}

void meshDiskCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const gp_Circ& circ, int faceId, const FaceMeshSettings& s,
                 MeshBuilder& out) {
    const int n = std::max(3, s.radial);
    // Ring points come from the circle's own parametrization so they land on
    // the same positions as an adjacent cylinder side sharing this circle;
    // the weld pass then stitches the two faces watertight.
    std::vector<uint32_t> ring(n);
    std::vector<gp_Pnt> pts(n);
    for (int i = 0; i < n; ++i) {
        pts[i] = ElCLib::Value(i * 2.0 * M_PI / n, circ);
        ring[i] = out.addVertex(pts[i]);
    }

    // Ring order follows circle parametrization, which is unrelated to the
    // face's outward side; orient by comparing the ring's normal to the face's.
    gp_Vec ringNormal =
        gp_Vec(pts[0], pts[1]).Crossed(gp_Vec(pts[1], pts[2]));
    const bool flip = ringNormal.Dot(planarFaceNormal(face, surf)) < 0;

    if (s.cap == CapStyle::NGon) {
        out.addPolygon(ring, faceId, flip);
    } else {
        uint32_t center = out.addVertex(circ.Location());
        for (int i = 0; i < n; ++i) {
            out.addPolygon({center, ring[i], ring[(i + 1) % n]}, faceId, flip);
        }
    }
}

// UV grid over the face's parametric bounds. Only valid when the grid lies
// inside the trim boundary, which we verify by classifying every node and
// cell center. Conforming a grid to arbitrary trim curves is the hard
// problem (plan §7.1) and is out of scope for the spike.
bool tryMeshParametricGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                           int faceId, const FaceMeshSettings& s, MeshBuilder& out) {
    if (surf.IsUPeriodic() || surf.IsVPeriodic()) return false;

    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const int nu = std::max(1, s.gridU);
    const int nv = std::max(1, s.gridV);
    const double du = (umax - umin) / nu;
    const double dv = (vmax - vmin) / nv;
    const double tol = BRep_Tool::Tolerance(face);

    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            gp_Pnt2d node(umin + i * du, vmin + j * dv);
            BRepClass_FaceClassifier cls(face, node, tol);
            if (cls.State() == TopAbs_OUT) return false;
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            gp_Pnt2d center(umin + (i + 0.5) * du, vmin + (j + 0.5) * dv);
            BRepClass_FaceClassifier cls(face, center, tol);
            if (cls.State() != TopAbs_IN) return false;
        }
    }

    const bool flip = face.Orientation() == TopAbs_REVERSED;
    std::vector<uint32_t> grid((nu + 1) * (nv + 1));
    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            grid[j * (nu + 1) + i] =
                out.addVertex(surf.Value(umin + i * du, vmin + j * dv));
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            out.addPolygon({grid[j * (nu + 1) + i], grid[j * (nu + 1) + i + 1],
                            grid[(j + 1) * (nu + 1) + i + 1],
                            grid[(j + 1) * (nu + 1) + i]},
                           faceId, flip);
        }
    }
    return true;
}

// Last resort: OCCT chord-tolerance triangulation of the single face.
void meshFallback(const TopoDS_Face& face, int faceId, const FaceMeshSettings& s,
                  MeshBuilder& out) {
    BRepMesh_IncrementalMesh mesher(face, s.chordTolerance);
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) return;

    const bool flip = face.Orientation() == TopAbs_REVERSED;
    std::vector<uint32_t> verts(tri->NbNodes());
    for (int i = 1; i <= tri->NbNodes(); ++i) {
        verts[i - 1] = out.addVertex(tri->Node(i).Transformed(loc.Transformation()));
    }
    for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int a, b, c;
        tri->Triangle(i).Get(a, b, c);
        out.addPolygon({verts[a - 1], verts[b - 1], verts[c - 1]}, faceId, flip);
    }
}

}  // namespace

PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings, GenerationReport* report) {
    PolyMesh mesh;
    MeshBuilder out(mesh);

    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FaceMeshSettings& s = settings.forFace(fid);
        BRepAdaptor_Surface surf(face);
        MesherKind kind = MesherKind::Fallback;

        gp_Circ circ;
        if (isFullRevolutionCylinder(surf)) {
            meshCylinderGrid(face, surf, fid, s, out);
            kind = MesherKind::CylinderGrid;
        } else if (surf.GetType() == GeomAbs_Plane && boundingCircle(face, circ)) {
            meshDiskCap(face, surf, circ, fid, s, out);
            kind = MesherKind::DiskCap;
        } else if (tryMeshParametricGrid(face, surf, fid, s, out)) {
            kind = MesherKind::PlanarGrid;
        } else {
            meshFallback(face, fid, s, out);
        }
        if (report) report->faceMesher[fid] = kind;
    }

    weldVertices(mesh, settings.weldTolerance);
    return mesh;
}

}  // namespace weft
