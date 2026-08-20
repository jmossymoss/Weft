#include "weft/geometry_pool.hpp"

#include "weft/analysis.hpp"
#include "weft/model.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <ElSLib.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>

#include <TColStd_Array1OfReal.hxx>
#include <cmath>

namespace weft {

void GeometryPool::clear() {
    planes.clear();
    cylinders.clear();
    cones.clear();
    spheres.clear();
    tori.clear();
    nurbs.clear();
    byFace.clear();
}

void GeometryPool::extractFromModel(const Model& model,
                                    const Analysis& analysis) {
    clear();
    const int nF = model.faceCount();
    if (nF < 1) return;
    byFace.assign(size_t(nF) + 1, FaceGeomSlot{});
    planes.reserve(size_t(nF) / 2 + 8);
    cylinders.reserve(size_t(nF) / 4 + 8);
    cones.reserve(64);
    spheres.reserve(64);
    tori.reserve(64);
    nurbs.reserve(size_t(nF) / 3 + 8);

    for (int fid = 1; fid <= nF; ++fid) {
        try {
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            BRepAdaptor_Surface surf(face, Standard_True);
            double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
            BRepTools::UVBounds(face, u0, u1, v0, v1);

            FaceGeomSlot slot;
            switch (surf.GetType()) {
                case GeomAbs_Plane: {
                    PlanePrimitive p;
                    p.faceId = uint32_t(fid);
                    p.frame = surf.Plane().Position();
                    p.u0 = u0;
                    p.u1 = u1;
                    p.v0 = v0;
                    p.v1 = v1;
                    slot.kind = GeomPrimKind::Plane;
                    slot.index = uint32_t(planes.size());
                    planes.push_back(std::move(p));
                    break;
                }
                case GeomAbs_Cylinder: {
                    CylinderPrimitive c;
                    c.faceId = uint32_t(fid);
                    c.frame = surf.Cylinder().Position();
                    c.radius = surf.Cylinder().Radius();
                    c.u0 = u0;
                    c.u1 = u1;
                    c.v0 = v0;
                    c.v1 = v1;
                    slot.kind = GeomPrimKind::Cylinder;
                    slot.index = uint32_t(cylinders.size());
                    cylinders.push_back(std::move(c));
                    break;
                }
                case GeomAbs_Cone: {
                    ConePrimitive c;
                    c.faceId = uint32_t(fid);
                    c.frame = surf.Cone().Position();
                    c.radius = surf.Cone().RefRadius();
                    c.semiAngle = surf.Cone().SemiAngle();
                    c.u0 = u0;
                    c.u1 = u1;
                    c.v0 = v0;
                    c.v1 = v1;
                    slot.kind = GeomPrimKind::Cone;
                    slot.index = uint32_t(cones.size());
                    cones.push_back(std::move(c));
                    break;
                }
                case GeomAbs_Sphere: {
                    SpherePrimitive s;
                    s.faceId = uint32_t(fid);
                    s.frame = surf.Sphere().Position();
                    s.radius = surf.Sphere().Radius();
                    s.u0 = u0;
                    s.u1 = u1;
                    s.v0 = v0;
                    s.v1 = v1;
                    slot.kind = GeomPrimKind::Sphere;
                    slot.index = uint32_t(spheres.size());
                    spheres.push_back(std::move(s));
                    break;
                }
                case GeomAbs_Torus: {
                    TorusPrimitive t;
                    t.faceId = uint32_t(fid);
                    t.frame = surf.Torus().Position();
                    t.majorRadius = surf.Torus().MajorRadius();
                    t.minorRadius = surf.Torus().MinorRadius();
                    t.u0 = u0;
                    t.u1 = u1;
                    t.v0 = v0;
                    t.v1 = v1;
                    slot.kind = GeomPrimKind::Torus;
                    slot.index = uint32_t(tori.size());
                    tori.push_back(std::move(t));
                    break;
                }
                case GeomAbs_BSplineSurface:
                case GeomAbs_BezierSurface: {
                    NurbPrimitive n;
                    n.faceId = uint32_t(fid);
                    n.u0 = u0;
                    n.u1 = u1;
                    n.v0 = v0;
                    n.v1 = v1;
                    Handle(Geom_Surface) gs = surf.Surface().Surface();
                    Handle(Geom_BSplineSurface) bs =
                        Handle(Geom_BSplineSurface)::DownCast(gs);
                    Handle(Geom_BezierSurface) bz =
                        Handle(Geom_BezierSurface)::DownCast(gs);
                    if (!bs.IsNull()) {
                        n.uDegree = bs->UDegree();
                        n.vDegree = bs->VDegree();
                        n.nU = bs->NbUPoles();
                        n.nV = bs->NbVPoles();
                        n.rational = bs->IsURational() || bs->IsVRational();
                        n.poles.resize(size_t(n.nU) * size_t(n.nV));
                        if (n.rational) {
                            n.weights.resize(n.poles.size());
                        }
                        for (int iv = 1; iv <= n.nV; ++iv) {
                            for (int iu = 1; iu <= n.nU; ++iu) {
                                const size_t k =
                                    size_t(iv - 1) * size_t(n.nU) +
                                    size_t(iu - 1);
                                n.poles[k] = bs->Pole(iu, iv);
                                if (n.rational) {
                                    n.weights[k] = bs->Weight(iu, iv);
                                }
                            }
                        }
                        // Full knot sequences (multiplicities expanded) —
                        // length = nPoles + degree + 1 for de Boor.
                        const TColStd_Array1OfReal& uSeq = bs->UKnotSequence();
                        const TColStd_Array1OfReal& vSeq = bs->VKnotSequence();
                        n.uKnots.resize(size_t(uSeq.Length()));
                        n.vKnots.resize(size_t(vSeq.Length()));
                        for (int i = uSeq.Lower(); i <= uSeq.Upper(); ++i) {
                            n.uKnots[size_t(i - uSeq.Lower())] = uSeq.Value(i);
                        }
                        for (int i = vSeq.Lower(); i <= vSeq.Upper(); ++i) {
                            n.vKnots[size_t(i - vSeq.Lower())] = vSeq.Value(i);
                        }
                    } else if (!bz.IsNull()) {
                        n.uDegree = bz->UDegree();
                        n.vDegree = bz->VDegree();
                        n.nU = bz->NbUPoles();
                        n.nV = bz->NbVPoles();
                        n.rational = bz->IsURational() || bz->IsVRational();
                        n.poles.resize(size_t(n.nU) * size_t(n.nV));
                        if (n.rational) n.weights.resize(n.poles.size());
                        for (int iv = 1; iv <= n.nV; ++iv) {
                            for (int iu = 1; iu <= n.nU; ++iu) {
                                const size_t k =
                                    size_t(iv - 1) * size_t(n.nU) +
                                    size_t(iu - 1);
                                n.poles[k] = bz->Pole(iu, iv);
                                if (n.rational) {
                                    n.weights[k] = bz->Weight(iu, iv);
                                }
                            }
                        }
                        // Bezier: open knots [0..1] with multiplicity deg+1
                        n.uKnots.assign(size_t(n.uDegree + 1), 0.0);
                        n.uKnots.insert(n.uKnots.end(), size_t(n.uDegree + 1),
                                        1.0);
                        n.vKnots.assign(size_t(n.vDegree + 1), 0.0);
                        n.vKnots.insert(n.vKnots.end(), size_t(n.vDegree + 1),
                                        1.0);
                    } else {
                        slot.kind = GeomPrimKind::Other;
                        byFace[size_t(fid)] = slot;
                        continue;
                    }
                    slot.kind = GeomPrimKind::Nurb;
                    slot.index = uint32_t(nurbs.size());
                    nurbs.push_back(std::move(n));
                    break;
                }
                default:
                    slot.kind = GeomPrimKind::Other;
                    break;
            }
            byFace[size_t(fid)] = slot;
        } catch (const Standard_Failure&) {
            byFace[size_t(fid)] = FaceGeomSlot{};
        }
    }
}

namespace {

constexpr int kMaxDeg = 16;  // stack workspace; higher → fall back to false

// The NURBS Book A2.1: find span index i with U[i] <= u < U[i+1].
int findSpan(int nPoles, int p, double u, const double* U, int /*nKnots*/) {
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

// The NURBS Book A2.2: non-zero basis functions N[0..p] for span i.
void basisFuns(int i, double u, int p, const double* U, double* N) {
    double left[kMaxDeg + 1];
    double right[kMaxDeg + 1];
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

// Evaluate a 1D B-spline / NURBS curve into homogeneous 4-vector (x,y,z,w).
// polesW: consecutive (x,y,z,w) for the p+1 poles in the span.
void deBoorCurve1D(int span, double u, int p, const double* U,
                   const double* polesW /* (p+1)*4 */, double* outW) {
    double N[kMaxDeg + 1];
    basisFuns(span, u, p, U, N);
    outW[0] = outW[1] = outW[2] = outW[3] = 0.0;
    for (int j = 0; j <= p; ++j) {
        const double* pw = polesW + size_t(j) * 4;
        outW[0] += N[j] * pw[0];
        outW[1] += N[j] * pw[1];
        outW[2] += N[j] * pw[2];
        outW[3] += N[j] * pw[3];
    }
}

bool evalNurbSurface(const NurbPrimitive& n, double u, double v, gp_Pnt& out) {
    if (n.nU < 1 || n.nV < 1 || n.poles.empty()) return false;
    if (n.uDegree < 0 || n.vDegree < 0 || n.uDegree > kMaxDeg ||
        n.vDegree > kMaxDeg) {
        return false;
    }
    // Knot length must be nPoles + degree + 1.
    if (int(n.uKnots.size()) != n.nU + n.uDegree + 1 ||
        int(n.vKnots.size()) != n.nV + n.vDegree + 1) {
        return false;
    }
    const double* U = n.uKnots.data();
    const double* V = n.vKnots.data();
    // Clamp into the open interval of the knot vector.
    if (u < U[n.uDegree]) u = U[n.uDegree];
    if (u > U[n.nU]) u = U[n.nU];
    if (v < V[n.vDegree]) v = V[n.vDegree];
    if (v > V[n.nV]) v = V[n.nV];

    const int us = findSpan(n.nU, n.uDegree, u, U, int(n.uKnots.size()));
    const int vs = findSpan(n.nV, n.vDegree, v, V, int(n.vKnots.size()));

    // Temp: for each of (vDegree+1) rows, evaluate the u-curve → homogeneous.
    double rowW[(kMaxDeg + 1) * 4];
    double uPoles[(kMaxDeg + 1) * 4];
    for (int l = 0; l <= n.vDegree; ++l) {
        const int iv = vs - n.vDegree + l;
        for (int k = 0; k <= n.uDegree; ++k) {
            const int iu = us - n.uDegree + k;
            const size_t pk = size_t(iv) * size_t(n.nU) + size_t(iu);
            const gp_Pnt& P = n.poles[pk];
            const double w =
                n.rational ? n.weights[pk] : 1.0;
            uPoles[size_t(k) * 4 + 0] = P.X() * w;
            uPoles[size_t(k) * 4 + 1] = P.Y() * w;
            uPoles[size_t(k) * 4 + 2] = P.Z() * w;
            uPoles[size_t(k) * 4 + 3] = w;
        }
        deBoorCurve1D(us, u, n.uDegree, U, uPoles, rowW + size_t(l) * 4);
    }
    double outW[4];
    deBoorCurve1D(vs, v, n.vDegree, V, rowW, outW);
    if (std::abs(outW[3]) < 1e-30) return false;
    const double inv = 1.0 / outW[3];
    out.SetCoord(outW[0] * inv, outW[1] * inv, outW[2] * inv);
    return true;
}

}  // namespace

bool GeometryPool::value(uint32_t faceId, double u, double v,
                         gp_Pnt& out) const {
    if (faceId == 0 || faceId >= byFace.size()) return false;
    const FaceGeomSlot& s = byFace[faceId];
    switch (s.kind) {
        case GeomPrimKind::Plane: {
            if (s.index >= planes.size()) return false;
            const PlanePrimitive& p = planes[s.index];
            ElSLib::PlaneD0(u, v, p.frame, out);
            return true;
        }
        case GeomPrimKind::Cylinder: {
            if (s.index >= cylinders.size()) return false;
            const CylinderPrimitive& c = cylinders[s.index];
            ElSLib::CylinderD0(u, v, c.frame, c.radius, out);
            return true;
        }
        case GeomPrimKind::Cone: {
            if (s.index >= cones.size()) return false;
            const ConePrimitive& c = cones[s.index];
            ElSLib::ConeD0(u, v, c.frame, c.radius, c.semiAngle, out);
            return true;
        }
        case GeomPrimKind::Sphere: {
            if (s.index >= spheres.size()) return false;
            const SpherePrimitive& sp = spheres[s.index];
            ElSLib::SphereD0(u, v, sp.frame, sp.radius, out);
            return true;
        }
        case GeomPrimKind::Torus: {
            if (s.index >= tori.size()) return false;
            const TorusPrimitive& t = tori[s.index];
            ElSLib::TorusD0(u, v, t.frame, t.majorRadius, t.minorRadius, out);
            return true;
        }
        case GeomPrimKind::Nurb: {
            if (s.index >= nurbs.size()) return false;
            return evalNurbSurface(nurbs[s.index], u, v, out);
        }
        default:
            return false;
    }
}

}  // namespace weft
