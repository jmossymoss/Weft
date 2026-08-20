// Lock-free 3D edge curve pool (ETL at analyze).
//
// Mirrors GeometryPool: extract Line/Circle/NURBS (or dense polyline
// fallback) once, then sample without BRepAdaptor_Curve on the mesh hot
// path. AD-1: still feeds the same border-contract samplers.
//
#pragma once

#include <cstdint>
#include <vector>

#include <gp_Ax2.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace weft {

struct Model;

enum class CurvePrimKind : uint8_t {
    None = 0,
    Line,
    Circle,
    Nurb,
    Polyline,  // dense chord samples for exotic curves
};

struct CurvePrimitive {
    uint32_t edgeId = 0;
    CurvePrimKind kind = CurvePrimKind::None;
    double f = 0, l = 1;  // native 3D-curve parameter range
    double length = 0;    // 3D arc length (mm)

    // Line
    gp_Pnt origin;
    gp_Vec direction;  // unit

    // Circle
    gp_Ax2 circlePos;
    double radius = 0;

    // NURBS curve
    int degree = 0;
    int nPoles = 0;
    bool rational = false;
    std::vector<gp_Pnt> poles;
    std::vector<double> weights;
    std::vector<double> knots;  // full sequence

    // Polyline fallback / arc-length table (analyze-time only growth)
    std::vector<gp_Pnt> samples;     // including endpoints
    std::vector<double> cumulArc;    // size == samples; cumulArc.back()==length
};

struct CurvePool {
    // Index 0 unused; edgeId 1..N
    std::vector<CurvePrimitive> byEdge;

    bool empty() const { return byEdge.size() <= 1; }
    void clear() { byEdge.clear(); }

    void extractFromModel(const Model& model);

    // t in [0,1] along FORWARD parameter (f + t*(l-f)). Lock-free.
    bool value(uint32_t edgeId, double t, gp_Pnt& out) const;

    // Even arc-length fractions in [0,1], n+1 stations (endpoints included).
    // Writes into `out` (cleared first). Uses precomputed cumulArc — no OCCT.
    // Returns false → caller falls back to uniform param fractions.
    bool evenArcFractions(uint32_t edgeId, int n,
                          std::vector<double>& out) const;
};

}  // namespace weft
