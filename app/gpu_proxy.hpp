// GPU isoline preview for weft_app density edits.
//
// The overlay is a UV lattice on the CAD face, not a second mesher.
// Counts come from the same semantic knobs generate() reads; UV is
// normalized to the trimmed face box so isolines match the CPU grid
// instead of the underlying surface domain.
#pragma once

#include "bake_queue.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"

#include <array>

namespace weft_app {

struct GpuProxyGrid {
    float u = 1.0f;
    float v = 1.0f;
    bool drawLattice = false;
};

// Map artist knobs onto CAD U/V isolines. Non-lattice meshers return
// drawLattice=false so the overlay does not promise a rectangular grid
// the CPU mesher will not emit.
GpuProxyGrid gpuProxyGrid(weft::MesherKind kind,
                          const weft::FaceMeshSettings& settings,
                          const weft::FaceMeshSettings& lastAdopted,
                          std::array<int, 2> lastSolved,
                          int faceAcross, bool isFillet);

// Preview stays visible while this face is dirty, queued, or baking so
// an intermediate adopt of another face cannot hide it.
bool gpuProxyActive(FaceFidelity fidelity, bool faceDirty, bool forceShow);

struct ProxyUvBox {
    double u0 = 0.0, u1 = 1.0, v0 = 0.0, v1 = 1.0;
    double surfU0 = 0.0, surfU1 = 1.0, surfV0 = 0.0, surfV1 = 1.0;
    bool uPeriodic = false;
    bool vPeriodic = false;
    bool valid = false;
};

// Trimmed-face UV box (BRepTools::UVBounds), not the surface First/Last
// domain. A cylinder sector then gets N isolines across the visible
// patch instead of N isolines across the full 2π parent surface.
ProxyUvBox faceProxyUvBox(const weft::Model& model, int faceId);

std::array<double, 2> normalizeProxyUv(const ProxyUvBox& box, double u,
                                       double v);

}  // namespace weft_app
