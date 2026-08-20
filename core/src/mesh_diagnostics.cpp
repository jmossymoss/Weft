#include "weft/meshers.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <memory>
#include <vector>

namespace weft {

std::vector<uint8_t> foldedPolys(const Model& model, const PolyMesh& mesh) {
    std::vector<uint8_t> folded(mesh.polygons.size(), 0);
    // Surface adaptors are built lazily per face; polygons arrive grouped
    // by face so in practice each face is built once.
    int currentFace = 0;
    std::unique_ptr<BRepAdaptor_Surface> surface;
    double orientation = 1.0;
    for (size_t polygonIndex = 0; polygonIndex < mesh.polygons.size();
         ++polygonIndex) {
        const int faceId = mesh.polygonFaceId[polygonIndex];
        if (faceId <= 0 || faceId > model.faceCount()) continue;
        const auto& polygon = mesh.polygons[polygonIndex];

        if (faceId != currentFace) {
            const TopoDS_Face& face = TopoDS::Face(model.faces(faceId));
            surface = std::make_unique<BRepAdaptor_Surface>(face);
            orientation =
                face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
            currentFace = faceId;
        }

        // Prefer Newell from surface-evaluated anchor points when the
        // polygon is fully (or nearly) UV-anchored on this face. Weld /
        // micro-edge collapse can drift mesh vertices off the surface so
        // a mesh-space Newell opposes the CAD normal while the intended
        // UV winding is still correct (mp9_Edited #133/#3025). Fall back
        // to mesh positions when anchors are sparse.
        std::vector<gp_Pnt> ring;
        ring.reserve(polygon.size());
        int anchored = 0;
        for (uint32_t vertexIndex : polygon) {
            if (vertexIndex < mesh.anchors.size() &&
                mesh.anchors[vertexIndex].faceId == faceId && surface) {
                const Anchor& an = mesh.anchors[vertexIndex];
                ring.push_back(surface->Value(an.u, an.v));
                ++anchored;
            } else if (vertexIndex < mesh.vertices.size()) {
                const auto& v = mesh.vertices[vertexIndex];
                ring.push_back(gp_Pnt(v[0], v[1], v[2]));
            }
        }
        if (ring.size() < 3) continue;
        // Prefer the surface ring whenever at least 3 face anchors exist
        // (enough for a Newell). Otherwise keep mesh-space Newell so
        // unanchored edit geometry still flags real folds.
        if (anchored < 3) {
            ring.clear();
            for (uint32_t vertexIndex : polygon) {
                if (vertexIndex >= mesh.vertices.size()) continue;
                const auto& v = mesh.vertices[vertexIndex];
                ring.push_back(gp_Pnt(v[0], v[1], v[2]));
            }
        }

        // Newell normal: robust winding normal for any planar-ish polygon.
        double nx = 0.0, ny = 0.0, nz = 0.0;
        for (size_t i = 0; i < ring.size(); ++i) {
            const gp_Pnt& a = ring[i];
            const gp_Pnt& b = ring[(i + 1) % ring.size()];
            nx += (a.Y() - b.Y()) * (a.Z() + b.Z());
            ny += (a.Z() - b.Z()) * (a.X() + b.X());
            nz += (a.X() - b.X()) * (a.Y() + b.Y());
        }
        const double normalLength = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (normalLength < 1e-14) continue;

        // Every vertex anchored on this face votes. Sampling each UV avoids
        // averaging across periodic seams onto the opposite side of a surface.
        int votes = 0;
        for (uint32_t vertexIndex : polygon) {
            if (vertexIndex >= mesh.anchors.size()) continue;
            const Anchor& anchor = mesh.anchors[vertexIndex];
            if (anchor.faceId != faceId) continue;
            gp_Pnt point;
            gp_Vec du, dv;
            surface->D1(anchor.u, anchor.v, point, du, dv);
            const gp_Vec surfaceNormal = du.Crossed(dv);
            const double surfaceLength = surfaceNormal.Magnitude();
            if (surfaceLength < 1e-14) continue;
            const double dot =
                orientation *
                (surfaceNormal.X() * nx + surfaceNormal.Y() * ny +
                 surfaceNormal.Z() * nz) /
                (surfaceLength * normalLength);
            if (dot > 0.1) ++votes;
            else if (dot < -0.1) --votes;
        }
        if (votes < 0) folded[polygonIndex] = 1;
    }
    return folded;
}

}  // namespace weft
