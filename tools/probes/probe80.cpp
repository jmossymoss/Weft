// Old (GCPnts_TangentialDeflection) vs new (stable integral) curvature
// floor per edge of a face — find where the swap moved a count.
#include "weft/analysis.hpp"
#include "weft/model.hpp"
#include <BRep_Tool.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <algorithm>
#include <cmath>
#include <cstdio>

static int stableCount(const Adaptor3d_Curve& c, double angTol, double chordTol) {
    const double f = c.FirstParameter(), l = c.LastParameter();
    if (!(l > f)) return 1;
    constexpr int kSamples = 33;
    double S[kSamples] = {0}, T[kSamples] = {0};
    gp_Pnt prevP; double prevK = 0.0; bool prevOk = false;
    for (int i = 0; i < kSamples; ++i) {
        const double t = f + (l - f) * i / double(kSamples - 1);
        gp_Pnt P; double k = 0.0; bool have = true;
        try {
            gp_Vec D1, D2; c.D2(t, P, D1, D2);
            const double d1 = D1.Magnitude();
            k = d1 > 1e-12 ? D1.Crossed(D2).Magnitude() / (d1*d1*d1) : 0.0;
        } catch (...) { try { P = c.Value(t); } catch (...) { have = false; } }
        if (i > 0) { S[i] = S[i-1]; T[i] = T[i-1]; }
        if (!have) continue;
        if (prevOk) {
            const double ds = P.Distance(prevP);
            S[i] = S[i-1] + ds;
            T[i] = T[i-1] + 0.5 * (k + prevK) * ds;
        }
        prevP = P; prevK = k; prevOk = true;
    }
    const double aTol = std::max(angTol, 1e-3) * (1.0 + 1e-9);
    const double cTol8 = 8.0 * std::max(chordTol, 1e-12) * (1.0 + 1e-9);
    auto at = [&](const double* A, double x) {
        const double u = x * (kSamples - 1);
        const int i = std::min(kSamples - 2, std::max(0, int(u)));
        return A[i] + (A[i + 1] - A[i]) * (u - i);
    };
    for (int n = 1; n < 256; ++n) {
        bool ok = true;
        for (int i = 0; i < n && ok; ++i) {
            const double x0 = i / double(n), x1 = (i + 1) / double(n);
            const double dT = at(T, x1) - at(T, x0);
            if (dT > aTol) ok = false;
            const double dS = at(S, x1) - at(S, x0);
            if (dT * dS > cTol8) ok = false;
        }
        if (ok) return n;
    }
    return 256;
}

int main(int argc, char** argv) {
    weft::Model m = weft::loadStep(argv[1]);
    int fid = atoi(argv[2]);
    weft::Analysis a = weft::analyze(m);
    for (int eid : a.faces[fid - 1].edgeIds) {
        const TopoDS_Edge E = TopoDS::Edge(m.edges(eid));
        if (BRep_Tool::Degenerated(E)) continue;
        double f, l;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(E, f, l);
        if (c3.IsNull()) continue;
        GeomAdaptor_Curve gc(c3, f, l);
        int oldN = -1;
        try {
            GCPnts_TangentialDeflection td(gc, M_PI / 3.0, 1e6, 2);
            oldN = std::clamp(td.NbPoints() - 1, 1, 32);
        } catch (...) {}
        int newN = std::clamp(stableCount(gc, M_PI / 3.0, 1e6), 1, 32);
        std::printf("edge #%d type %d: old floor %d new floor %d%s\n", eid,
                    int(gc.GetType()), oldN, newN,
                    oldN != newN ? "  <-- MOVED" : "");
    }
    return 0;
}
