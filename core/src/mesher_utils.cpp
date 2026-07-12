#include "weft/meshers.hpp"

#include <algorithm>
#include <cmath>

namespace weft {

std::vector<double> clusteredParams(int divisions, double hold) {
    const int count = std::max(1, divisions);
    hold = std::min(0.95, std::max(0.0, hold));
    std::vector<double> params(count + 1);
    for (int i = 0; i <= count; ++i) {
        const double x = double(i) / count;
        // Monotonic for hold < 1: slope 1-hold at the ends, 1+hold mid-span,
        // so intervals shrink near the creases and grow in the middle.
        params[i] =
            x - hold * std::sin(2.0 * M_PI * x) / (2.0 * M_PI);
    }
    params.front() = 0.0;
    params.back() = 1.0;
    return params;
}

const char* mesherKindName(MesherKind kind) {
    switch (kind) {
        case MesherKind::RevolutionGrid: return "revolution-grid";
        case MesherKind::DiskCap: return "disk-cap";
        case MesherKind::PlanarGrid: return "parametric-grid";
        case MesherKind::CoonsGrid: return "coons-grid";
        case MesherKind::RingJunction: return "ring-junction";
        case MesherKind::QuadDominant: return "quad-dominant";
        case MesherKind::MinimalNGon: return "minimal-ngon";
        case MesherKind::Fallback: return "contract-floor";
        case MesherKind::AnnulusRing: return "annulus-ring";
        case MesherKind::PlateWeb: return "plate-web";
        case MesherKind::QuadFill: return "quad-fill";
        case MesherKind::RailLadder: return "rail-ladder";
        case MesherKind::RibbonSweep: return "ribbon-sweep";
        case MesherKind::DomeCap: return "dome-cap";
    }
    return "fallback-tri";
}

}  // namespace weft
