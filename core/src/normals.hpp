// Private helper shared by the exporters (OBJ, glTF): exact CAD normals
// per (vertex, B-rep face) pair. Not installed; include from core/src only.
#pragma once

#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Surface.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <array>
#include <map>

namespace weft {
namespace detail {

// Exact surface normal of `fid` at a mesh vertex: through the vertex's
// anchor when it belongs to that face, otherwise by projecting the point
// onto the face (border vertices keep the anchor of whichever face
// emitted them first). Returns false at poles/apexes (no unique normal).
inline bool cadNormal(const PolyMesh& mesh, const Model& model, uint32_t idx,
                      int fid, std::map<int, BRepAdaptor_Surface>& cache,
                      std::array<double, 3>& out) {
    if (fid < 1 || fid > model.faceCount()) return false;
    const TopoDS_Face face = TopoDS::Face(model.faces(fid));
    auto it = cache.find(fid);
    if (it == cache.end()) {
        it = cache.emplace(fid, BRepAdaptor_Surface(face)).first;
    }
    BRepAdaptor_Surface& surf = it->second;

    double u = 0, v = 0;
    const Anchor& a = mesh.anchors[idx];
    if (a.faceId == fid) {
        u = a.u;
        v = a.v;
    } else {
        Handle(Geom_Surface) hs = BRep_Tool::Surface(face);
        if (hs.IsNull()) return false;
        gp_Pnt p(mesh.vertices[idx][0], mesh.vertices[idx][1],
                 mesh.vertices[idx][2]);
        GeomAPI_ProjectPointOnSurf proj(p, hs);
        if (proj.NbPoints() < 1) return false;
        proj.LowerDistanceParameters(u, v);
    }
    gp_Pnt p;
    gp_Vec du, dv;
    surf.D1(u, v, p, du, dv);
    gp_Vec n = du.Crossed(dv);
    if (n.Magnitude() < 1e-14) return false;
    n.Normalize();
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    out = {n.X(), n.Y(), n.Z()};
    return true;
}

}  // namespace detail
}  // namespace weft
