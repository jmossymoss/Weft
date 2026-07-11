// probe93: which parameter direction is ANGULAR on freeform patches —
// ring test (constant radius+height about a fitted axis) on iso-v rows
// and iso-u columns separately, per face.
#include "weft/analysis.hpp"
#include "weft/model.hpp"
#include <BRepAdaptor_Surface.hxx>
#include <TopoDS.hxx>
#include <cstdio>
#include <cmath>
#include <algorithm>
static bool ringsOk(const BRepAdaptor_Surface& surf, bool uAngular,
                    double* arcOut) {
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    constexpr int NR = 5, NA = 12;
    gp_XYZ P[NR][NA + 1];
    gp_XYZ lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
    for (int j = 0; j < NR; ++j) {
        for (int i = 0; i <= NA; ++i) {
            const double a = double(i) / NA, r = double(j) / (NR - 1);
            const double u = uAngular ? u0 + (u1 - u0) * a
                                      : u0 + (u1 - u0) * r;
            const double v = uAngular ? v0 + (v1 - v0) * r
                                      : v0 + (v1 - v0) * a;
            P[j][i] = surf.Value(u, v).XYZ();
            lo.SetX(std::min(lo.X(), P[j][i].X()));
            lo.SetY(std::min(lo.Y(), P[j][i].Y()));
            lo.SetZ(std::min(lo.Z(), P[j][i].Z()));
            hi.SetX(std::max(hi.X(), P[j][i].X()));
            hi.SetY(std::max(hi.Y(), P[j][i].Y()));
            hi.SetZ(std::max(hi.Z(), P[j][i].Z()));
        }
    }
    const double scale = (hi - lo).Modulus();
    if (scale < 1e-12) return false;
    gp_XYZ C[NR];
    for (int j = 0; j < NR; ++j) {
        C[j] = gp_XYZ(0, 0, 0);
        for (int i = 0; i <= NA; ++i) C[j] += P[j][i];
        C[j] /= double(NA + 1);
    }
    gp_XYZ dir = C[NR - 1] - C[0];
    if (dir.Modulus() < 1e-6 * scale) {
        gp_XYZ n(0, 0, 0);
        for (int i = 0; i < NA; ++i) {
            n += (P[0][i] - C[0]).Crossed(P[0][i + 1] - C[0]);
        }
        dir = n;
    }
    if (dir.Modulus() < 1e-12) return false;
    dir.Normalize();
    double arcMax = 0;
    for (int j = 0; j < NR; ++j) {
        double rMin = 1e300, rMax = -1e300, hMin = 1e300, hMax = -1e300;
        for (int i = 0; i <= NA; ++i) {
            const gp_XYZ d = P[j][i] - C[0];
            const double h = d.Dot(dir);
            const double r = (d - dir * h).Modulus();
            rMin = std::min(rMin, r);
            rMax = std::max(rMax, r);
            hMin = std::min(hMin, h);
            hMax = std::max(hMax, h);
        }
        if (rMax - rMin > 5e-4 * scale || hMax - hMin > 5e-4 * scale) {
            if (arcOut) *arcOut = -std::max(rMax - rMin, hMax - hMin) / scale;
            return false;
        }
        // Subtended angle of the ring (first-mid-last chord estimate).
        const gp_XYZ d0 = P[j][0] - C[0];
        const double h0 = d0.Dot(dir);
        const gp_XYZ r0 = d0 - dir * h0;
        const gp_XYZ dN = P[j][NA] - C[0];
        const gp_XYZ rN = dN - dir * (dN.Dot(dir));
        if (r0.Modulus() > 1e-9 && rN.Modulus() > 1e-9) {
            double cosA = r0.Dot(rN) / (r0.Modulus() * rN.Modulus());
            cosA = std::clamp(cosA, -1.0, 1.0);
            arcMax = std::max(arcMax, std::acos(cosA));
        }
    }
    if (arcOut) *arcOut = arcMax;
    return true;
}
int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    int uAng = 0, vAng = 0, none = 0, both = 0;
    for (int fid = 1; fid <= m.faceCount(); ++fid) {
        const TopoDS_Face f = TopoDS::Face(m.faces(fid));
        BRepAdaptor_Surface sa(f);
        if (sa.GetType() != GeomAbs_BSplineSurface &&
            sa.GetType() != GeomAbs_BezierSurface &&
            sa.GetType() != GeomAbs_OffsetSurface &&
            sa.GetType() != GeomAbs_SurfaceOfExtrusion) {
            continue;
        }
        double arcU = 0, arcV = 0;
        const bool u = ringsOk(sa, true, &arcU);
        const bool v = ringsOk(sa, false, &arcV);
        if (u && v) ++both;
        else if (u) ++uAng;
        else if (v) ++vAng;
        else ++none;
        if (fid >= 89 && fid <= 111) {
            std::printf("face %d: u=%d(%.4f) v=%d(%.4f)\n",
                        fid, u ? 1 : 0, arcU, v ? 1 : 0, arcV);
        }
    }
    std::printf("summary: uAngular=%d vAngular=%d both=%d neither=%d\n",
                uAng, vAng, both, none);
    return 0;
}
