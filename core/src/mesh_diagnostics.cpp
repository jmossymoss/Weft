#include "weft/meshers.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <memory>

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

        // Newell normal: robust winding normal for any planar-ish polygon.
        double nx = 0.0, ny = 0.0, nz = 0.0;
        for (size_t i = 0; i < polygon.size(); ++i) {
            const auto& a = mesh.vertices[polygon[i]];
            const auto& b = mesh.vertices[polygon[(i + 1) % polygon.size()]];
            nx += (a[1] - b[1]) * (a[2] + b[2]);
            ny += (a[2] - b[2]) * (a[0] + b[0]);
            nz += (a[0] - b[0]) * (a[1] + b[1]);
        }
        const double normalLength = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (normalLength < 1e-14) continue;

        if (faceId != currentFace) {
            const TopoDS_Face& face = TopoDS::Face(model.faces(faceId));
            surface = std::make_unique<BRepAdaptor_Surface>(face);
            orientation =
                face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
            currentFace = faceId;
        }

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
