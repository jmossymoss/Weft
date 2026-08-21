#include "gpu_proxy.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

#include <algorithm>
#include <cmath>

namespace weft_app {

GpuProxyGrid gpuProxyGrid(weft::MesherKind kind,
                          const weft::FaceMeshSettings& s,
                          const weft::FaceMeshSettings& lastAdopted,
                          std::array<int, 2> lastSolved, int faceAcross,
                          bool isFillet) {
    using MK = weft::MesherKind;
    GpuProxyGrid g;
    std::array<int, 2> n{1, 1};
    auto manual = [&](int u, int v) {
        n = {std::max(1, u), std::max(1, v)};
    };
    const int stripAcross = isFillet && faceAcross > 0 ? faceAcross : 0;
    switch (kind) {
        case MK::RevolutionGrid:
        case MK::DomeCap:
            if (stripAcross && kind == MK::RevolutionGrid) {
                if (stripAcross == 1) {
                    manual(s.filletLoops, s.radial);
                } else {
                    manual(s.radial, s.filletLoops);
                }
            } else {
                manual(s.radial, s.axial);
            }
            g.drawLattice = true;
            break;
        case MK::PlanarGrid:
        case MK::CoonsGrid:
            if (stripAcross) {
                if (stripAcross == 1) {
                    manual(s.filletLoops, s.gridU);
                } else {
                    manual(s.gridU, s.filletLoops);
                }
            } else {
                manual(s.gridU, s.gridV);
            }
            if (kind == MK::CoonsGrid && (s.coonsRotate & 1)) {
                std::swap(n[0], n[1]);
            }
            g.drawLattice = true;
            break;
        default:
            g.drawLattice = false;
            break;
    }
    if (g.drawLattice && s.adaptive) {
        if (lastSolved[0] > 0) n[0] = lastSolved[0];
        if (lastSolved[1] > 0) n[1] = lastSolved[1];
        const double chordScale = std::sqrt(std::clamp(
            lastAdopted.chordTolerance / std::max(1e-9, s.chordTolerance),
            0.0625, 16.0));
        const double angleScale = std::clamp(
            lastAdopted.angleToleranceDeg /
                std::max(1.0, s.angleToleranceDeg),
            0.25, 4.0);
        const double scale = std::max(chordScale, angleScale);
        n[0] = std::max(1, int(std::lround(n[0] * scale)));
        n[1] = std::max(1, int(std::lround(n[1] * scale)));
    }
    g.u = float(std::clamp(n[0], 1, 256));
    g.v = float(std::clamp(n[1], 1, 256));
    return g;
}

bool gpuProxyActive(FaceFidelity fidelity, bool faceDirty, bool forceShow) {
    if (forceShow) return true;
    if (faceDirty) return true;
    return fidelity == FaceFidelity::Queued ||
           fidelity == FaceFidelity::Baking ||
           fidelity == FaceFidelity::LowPolyProxy;
}

ProxyUvBox faceProxyUvBox(const weft::Model& model, int faceId) {
    ProxyUvBox box;
    if (faceId < 1 || faceId > model.faceCount()) return box;
    try {
        const TopoDS_Face face = TopoDS::Face(model.faces(faceId));
        const BRepAdaptor_Surface surf(face);
        box.surfU0 = surf.FirstUParameter();
        box.surfU1 = surf.LastUParameter();
        box.surfV0 = surf.FirstVParameter();
        box.surfV1 = surf.LastVParameter();
        box.uPeriodic = surf.IsUPeriodic();
        box.vPeriodic = surf.IsVPeriodic();
        double umin = 0, umax = 0, vmin = 0, vmax = 0;
        BRepTools::UVBounds(face, umin, umax, vmin, vmax);
        if (!(std::isfinite(umax - umin) && std::abs(umax - umin) > 1e-12) ||
            !(std::isfinite(vmax - vmin) && std::abs(vmax - vmin) > 1e-12)) {
            umin = box.surfU0;
            umax = box.surfU1;
            vmin = box.surfV0;
            vmax = box.surfV1;
        }
        box.u0 = umin;
        box.u1 = umax;
        box.v0 = vmin;
        box.v1 = vmax;
        box.valid =
            std::isfinite(box.u1 - box.u0) && std::abs(box.u1 - box.u0) > 1e-12 &&
            std::isfinite(box.v1 - box.v0) && std::abs(box.v1 - box.v0) > 1e-12;
    } catch (const Standard_Failure&) {
        box.valid = false;
    }
    return box;
}

std::array<double, 2> normalizeProxyUv(const ProxyUvBox& box, double u,
                                       double v) {
    return {(u - box.u0) / (box.u1 - box.u0),
            (v - box.v0) / (box.v1 - box.v0)};
}

}  // namespace weft_app
