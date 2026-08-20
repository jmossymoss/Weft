#include "weft/curve_pool.hpp"

#include "weft/model.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <ElCLib.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Standard_Failure.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>

#include <algorithm>
#include <cmath>

namespace weft {
namespace {

constexpr int kMaxDeg = 16;
constexpr int kPolySamples = 64;

int findSpan(int nPoles, int p, double u, const double* U) {
    if (u >= U[nPoles]) return nPoles - 1;
    if (u <= U[p]) return p;
    int low = p, high = nPoles, mid = (low + high) / 2;
    while (u < U[mid] || u >= U[mid + 1]) {
        if (u < U[mid]) high = mid;
        else low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

void basisFuns(int i, double u, int p, const double* U, double* N) {
    double left[kMaxDeg + 1], right[kMaxDeg + 1];
    N[0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - U[i + 1 - j];
        right[j] = U[i + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            const double denom = right[r + 1] + left[j - r];
            const double temp = denom > 1e-30 ? N[r] / denom : 0.0;
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
}

bool evalNurbCurve(const CurvePrimitive& c, double u, gp_Pnt& out) {
    if (c.nPoles < 1 || c.degree < 0 || c.degree > kMaxDeg) return false;
    if (int(c.knots.size()) != c.nPoles + c.degree + 1) return false;
    const double* U = c.knots.data();
    if (u < U[c.degree]) u = U[c.degree];
    if (u > U[c.nPoles]) u = U[c.nPoles];
    const int span = findSpan(c.nPoles, c.degree, u, U);
    double N[kMaxDeg + 1];
    basisFuns(span, u, c.degree, U, N);
    double x = 0, y = 0, z = 0, wsum = 0;
    for (int j = 0; j <= c.degree; ++j) {
        const int ip = span - c.degree + j;
        const gp_Pnt& P = c.poles[size_t(ip)];
        const double w = c.rational ? c.weights[size_t(ip)] : 1.0;
        x += N[j] * P.X() * w;
        y += N[j] * P.Y() * w;
        z += N[j] * P.Z() * w;
        wsum += N[j] * w;
    }
    if (std::abs(wsum) < 1e-30) return false;
    const double inv = 1.0 / wsum;
    out.SetCoord(x * inv, y * inv, z * inv);
    return true;
}

void fillPolylineTable(CurvePrimitive& c) {
    const int n = kPolySamples;
    c.samples.resize(size_t(n) + 1);
    c.cumulArc.resize(size_t(n) + 1);
    c.cumulArc[0] = 0.0;
    for (int i = 0; i <= n; ++i) {
        const double t = double(i) / double(n);
        const double u = c.f + t * (c.l - c.f);
        gp_Pnt p;
        if (c.kind == CurvePrimKind::Nurb) {
            if (!evalNurbCurve(c, u, p)) p = c.poles.empty() ? gp_Pnt() : c.poles.front();
        } else if (c.kind == CurvePrimKind::Line) {
            p = c.origin.Translated(c.direction.Multiplied(t * c.length));
        } else if (c.kind == CurvePrimKind::Circle) {
            p = ElCLib::CircleValue(u, c.circlePos, c.radius);
        } else {
            p = c.origin;
        }
        c.samples[size_t(i)] = p;
        if (i > 0) {
            c.cumulArc[size_t(i)] =
                c.cumulArc[size_t(i) - 1] +
                c.samples[size_t(i) - 1].Distance(p);
        }
    }
    c.length = c.cumulArc.back();
}

}  // namespace

void CurvePool::extractFromModel(const Model& model) {
    clear();
    const int nE = model.edgeCount();
    if (nE < 1) return;
    byEdge.assign(size_t(nE) + 1, CurvePrimitive{});

    for (int eid = 1; eid <= nE; ++eid) {
        CurvePrimitive& c = byEdge[size_t(eid)];
        c.edgeId = uint32_t(eid);
        try {
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(edge)) {
                c.kind = CurvePrimKind::None;
                continue;
            }
            double f = 0, l = 0;
            Handle(Geom_Curve) gc = BRep_Tool::Curve(edge, f, l);
            if (gc.IsNull() || !(l > f)) {
                c.kind = CurvePrimKind::None;
                continue;
            }
            c.f = f;
            c.l = l;

            Handle(Geom_Line) line = Handle(Geom_Line)::DownCast(gc);
            Handle(Geom_Circle) circ = Handle(Geom_Circle)::DownCast(gc);
            Handle(Geom_BSplineCurve) bs =
                Handle(Geom_BSplineCurve)::DownCast(gc);
            Handle(Geom_BezierCurve) bz =
                Handle(Geom_BezierCurve)::DownCast(gc);

            if (!line.IsNull()) {
                c.kind = CurvePrimKind::Line;
                c.origin = line->Value(f);
                gp_Pnt end = line->Value(l);
                c.direction = gp_Vec(c.origin, end);
                c.length = c.direction.Magnitude();
                if (c.length > 1e-15) c.direction.Divide(c.length);
                else c.direction = gp_Vec(1, 0, 0);
            } else if (!circ.IsNull()) {
                c.kind = CurvePrimKind::Circle;
                c.circlePos = circ->Position();
                c.radius = circ->Radius();
                try {
                    BRepAdaptor_Curve ac(edge);
                    c.length = GCPnts_AbscissaPoint::Length(ac);
                } catch (...) {
                    c.length = c.radius * std::abs(l - f);
                }
                fillPolylineTable(c);  // arc-length table for even fractions
            } else if (!bs.IsNull()) {
                c.kind = CurvePrimKind::Nurb;
                c.degree = bs->Degree();
                c.nPoles = bs->NbPoles();
                c.rational = bs->IsRational();
                c.poles.resize(size_t(c.nPoles));
                if (c.rational) c.weights.resize(size_t(c.nPoles));
                for (int i = 1; i <= c.nPoles; ++i) {
                    c.poles[size_t(i - 1)] = bs->Pole(i);
                    if (c.rational) c.weights[size_t(i - 1)] = bs->Weight(i);
                }
                const TColStd_Array1OfReal& seq = bs->KnotSequence();
                c.knots.resize(size_t(seq.Length()));
                for (int i = seq.Lower(); i <= seq.Upper(); ++i) {
                    c.knots[size_t(i - seq.Lower())] = seq.Value(i);
                }
                fillPolylineTable(c);
            } else if (!bz.IsNull()) {
                c.kind = CurvePrimKind::Nurb;
                c.degree = bz->Degree();
                c.nPoles = bz->NbPoles();
                c.rational = bz->IsRational();
                c.poles.resize(size_t(c.nPoles));
                if (c.rational) c.weights.resize(size_t(c.nPoles));
                for (int i = 1; i <= c.nPoles; ++i) {
                    c.poles[size_t(i - 1)] = bz->Pole(i);
                    if (c.rational) c.weights[size_t(i - 1)] = bz->Weight(i);
                }
                c.knots.assign(size_t(c.degree + 1), 0.0);
                c.knots.insert(c.knots.end(), size_t(c.degree + 1), 1.0);
                // Remap f,l into [0,1] bezier domain for eval
                c.f = 0;
                c.l = 1;
                fillPolylineTable(c);
            } else {
                // Generic: dense polyline from adaptor (analyze-time only).
                c.kind = CurvePrimKind::Polyline;
                BRepAdaptor_Curve ac(edge);
                c.samples.resize(size_t(kPolySamples) + 1);
                c.cumulArc.resize(size_t(kPolySamples) + 1);
                c.cumulArc[0] = 0.0;
                for (int i = 0; i <= kPolySamples; ++i) {
                    const double u = f + (l - f) * double(i) / kPolySamples;
                    c.samples[size_t(i)] = ac.Value(u);
                    if (i > 0) {
                        c.cumulArc[size_t(i)] =
                            c.cumulArc[size_t(i) - 1] +
                            c.samples[size_t(i) - 1].Distance(
                                c.samples[size_t(i)]);
                    }
                }
                c.length = c.cumulArc.back();
            }
        } catch (const Standard_Failure&) {
            c.kind = CurvePrimKind::None;
        }
    }
}

bool CurvePool::value(uint32_t edgeId, double t, gp_Pnt& out) const {
    if (edgeId == 0 || edgeId >= byEdge.size()) return false;
    const CurvePrimitive& c = byEdge[edgeId];
    t = std::clamp(t, 0.0, 1.0);
    switch (c.kind) {
        case CurvePrimKind::Line:
            out = c.origin.Translated(c.direction.Multiplied(t * c.length));
            return true;
        case CurvePrimKind::Circle: {
            const double u = c.f + t * (c.l - c.f);
            out = ElCLib::CircleValue(u, c.circlePos, c.radius);
            return true;
        }
        case CurvePrimKind::Nurb: {
            const double u = c.f + t * (c.l - c.f);
            return evalNurbCurve(c, u, out);
        }
        case CurvePrimKind::Polyline: {
            if (c.samples.size() < 2) return false;
            const double target = t * c.length;
            // Binary search on cumulArc
            size_t lo = 0, hi = c.cumulArc.size() - 1;
            while (hi - lo > 1) {
                const size_t mid = (lo + hi) / 2;
                if (c.cumulArc[mid] < target) lo = mid;
                else hi = mid;
            }
            const double seg = c.cumulArc[hi] - c.cumulArc[lo];
            const double a =
                seg > 1e-15 ? (target - c.cumulArc[lo]) / seg : 0.0;
            const gp_Pnt& A = c.samples[lo];
            const gp_Pnt& B = c.samples[hi];
            out.SetCoord(A.X() + a * (B.X() - A.X()),
                         A.Y() + a * (B.Y() - A.Y()),
                         A.Z() + a * (B.Z() - A.Z()));
            return true;
        }
        default:
            return false;
    }
}

bool CurvePool::evenArcFractions(uint32_t edgeId, int n,
                                 std::vector<double>& out) const {
    out.clear();
    if (edgeId == 0 || edgeId >= byEdge.size() || n < 1) return false;
    const CurvePrimitive& c = byEdge[edgeId];
    // Analytic line/circle: param ∝ arc — uniform fractions are exact.
    if (c.kind == CurvePrimKind::Line || c.kind == CurvePrimKind::Circle) {
        return false;  // caller keeps uniform phasedT (byte-identical)
    }
    if (c.cumulArc.size() < 2 || c.length < 1e-15) return false;
    out.resize(size_t(n) + 1);
    out.front() = 0.0;
    out.back() = 1.0;
    for (int i = 1; i < n; ++i) {
        const double target = c.length * double(i) / double(n);
        size_t lo = 0, hi = c.cumulArc.size() - 1;
        while (hi - lo > 1) {
            const size_t mid = (lo + hi) / 2;
            if (c.cumulArc[mid] < target) lo = mid;
            else hi = mid;
        }
        const double seg = c.cumulArc[hi] - c.cumulArc[lo];
        const double a =
            seg > 1e-15 ? (target - c.cumulArc[lo]) / seg : 0.0;
        const double t0 = double(lo) / double(c.samples.size() - 1);
        const double t1 = double(hi) / double(c.samples.size() - 1);
        out[size_t(i)] = t0 + a * (t1 - t0);
    }
    return true;
}

}  // namespace weft
