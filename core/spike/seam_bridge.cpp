// Seam-absorption spike (decoupled-core de-risk).
//
// The make-or-break primitive for a decoupled mesher: bridge two ordered
// loops of ARBITRARY, DIFFERENT vertex counts (N outer, M inner) into ONE
// ring of cells — quads where the counts line up, grouped n-gons where they
// don't — that is watertight (every interior edge shared by 2 cells), manifold,
// and fold-free (all cells same winding, none degenerate). If this holds for
// wild count ratios and shapes, the whole decoupled architecture is viable:
// every face meshes its interior at its OWN count and absorbs each border with
// this bridge; neighbours weld because they share the exact border samples.
//
// Self-contained (no OCCT): synthesizes loops, bridges, validates. Build:
//   g++ -O2 -std=c++17 core/spike/seam_bridge.cpp -o /tmp/seam_bridge && /tmp/seam_bridge
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <vector>

using V3 = std::array<double, 3>;
struct Mesh {
    std::vector<V3> verts;
    std::vector<std::vector<int>> polys;
};

static V3 sub(const V3& a, const V3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
static V3 cross(const V3& a, const V3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}
static double dot(const V3& a, const V3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static double norm(const V3& a) { return std::sqrt(dot(a, a)); }

// ---- the primitive under test -------------------------------------------
// Bridge outer loop O (N verts) to inner loop I (M verts), same winding,
// roughly concentric. One ring of cells. A monotone merge-walk by loop
// fraction: advance whichever side is "behind"; emit a quad when both sides
// step, an n-gon when one side absorbs several of the other's steps. Vertices
// are given as global indices into a shared vertex array.
static void bridgeLoops(const std::vector<int>& O, const std::vector<int>& I,
                        Mesh& out) {
    const int N = (int)O.size(), M = (int)I.size();
    if (N < 3 || M < 3) return;
    // Iterate the COARSER loop's edges; each maps to a contiguous run of the
    // finer loop's vertices, and the pair becomes ONE cell — a quad when the
    // run is a single edge, a grouped n-gon when it absorbs several. Winding is
    // always outer-forward then inner-backward, so it stays consistent whether
    // the outer or the inner loop is the coarse one (fixes the inverted case).
    auto emit = [&](std::vector<int> cell) {
        std::vector<int> clean;
        for (int idx : cell)
            if (clean.empty() || clean.back() != idx) clean.push_back(idx);
        if (clean.size() > 1 && clean.front() == clean.back()) clean.pop_back();
        if ((int)clean.size() >= 3) out.polys.push_back(std::move(clean));
    };
    if (M <= N) {  // inner coarse: one cell per inner edge, absorbing outer run
        for (int c = 0; c < M; ++c) {
            const int oa = (int)std::llround((long double)c * N / M);
            const int ob = (int)std::llround((long double)(c + 1) * N / M);
            std::vector<int> cell;
            for (int k = oa; k <= ob; ++k) cell.push_back(O[k % N]);  // fwd
            cell.push_back(I[(c + 1) % M]);
            cell.push_back(I[c % M]);
            emit(std::move(cell));
        }
    } else {  // outer coarse: one cell per outer edge, absorbing inner run
        for (int c = 0; c < N; ++c) {
            const int ia = (int)std::llround((long double)c * M / N);
            const int ib = (int)std::llround((long double)(c + 1) * M / N);
            std::vector<int> cell;
            cell.push_back(O[c % N]);
            cell.push_back(O[(c + 1) % N]);
            for (int k = ib; k >= ia; --k) cell.push_back(I[k % M]);  // back
            emit(std::move(cell));
        }
    }
}

// ---- validation ----------------------------------------------------------
struct Report {
    int cells = 0, quads = 0, tris = 0, ngons = 0;
    int interiorOpen = 0;   // interior edges used by !=2 cells (real cracks)
    int nonManifold = 0;    // edges used by >2 cells
    int folds = 0;          // cells whose winding opposes the ring normal
    int degenerate = 0;
};

static Report validate(const Mesh& m, const std::vector<int>& outerLoop,
                       const std::vector<int>& innerLoop, const V3& axis) {
    Report r;
    // Boundary edges = the outer + inner loops (expected to be open).
    std::map<std::pair<int, int>, int> boundary;
    auto addLoop = [&](const std::vector<int>& L) {
        for (size_t k = 0; k < L.size(); ++k) {
            int a = L[k], b = L[(k + 1) % L.size()];
            boundary[{std::min(a, b), std::max(a, b)}]++;
        }
    };
    addLoop(outerLoop); addLoop(innerLoop);
    std::map<std::pair<int, int>, int> use;
    for (const auto& p : m.polys) {
        r.cells++;
        if (p.size() == 3) r.tris++; else if (p.size() == 4) r.quads++;
        else r.ngons++;
        // winding vs axis (Newell normal)
        V3 nrm{0, 0, 0};
        for (size_t k = 0; k < p.size(); ++k) {
            const V3& a = m.verts[p[k]];
            const V3& b = m.verts[p[(k + 1) % p.size()]];
            nrm[0] += (a[1] - b[1]) * (a[2] + b[2]);
            nrm[1] += (a[2] - b[2]) * (a[0] + b[0]);
            nrm[2] += (a[0] - b[0]) * (a[1] + b[1]);
        }
        double a2 = norm(nrm);
        if (a2 < 1e-9) { r.degenerate++; continue; }
        if (dot(nrm, axis) < 0) r.folds++;
        for (size_t k = 0; k < p.size(); ++k) {
            int a = p[k], b = p[(k + 1) % p.size()];
            use[{std::min(a, b), std::max(a, b)}]++;
        }
    }
    for (const auto& [e, n] : use) {
        if (n > 2) { r.nonManifold++; continue; }
        if (n == 1 && !boundary.count(e)) r.interiorOpen++;  // real crack
    }
    return r;
}

// Build two concentric loops on a plane (circle radius rO/rI) with N/M verts,
// optionally with a radial "wobble" so it isn't a perfect circle (tests that
// the bridge doesn't rely on exact concentricity), and a z tilt.
static void makeCase(int N, int M, double rO, double rI, double wobble,
                     double tilt, Mesh& m, std::vector<int>& O,
                     std::vector<int>& I) {
    auto ring = [&](int n, double rad, double z, std::vector<int>& L) {
        for (int k = 0; k < n; ++k) {
            double t = 2 * M_PI * k / n;
            double rr = rad * (1.0 + wobble * std::sin(3 * t));
            m.verts.push_back({rr * std::cos(t), rr * std::sin(t),
                               z + tilt * std::cos(t)});
            L.push_back((int)m.verts.size() - 1);
        }
    };
    ring(N, rO, 0.0, O);
    ring(M, rI, 0.0, I);
}

int main() {
    struct Case { int N, M; double rO, rI, wob, tilt; const char* name; };
    std::vector<Case> cases = {
        {32, 32, 10, 8, 0, 0, "32:32 equal"},
        {32, 16, 10, 8, 0, 0, "32:16 2:1"},
        {32, 6, 10, 8, 0, 0, "32:6  ~5:1"},
        {100, 7, 10, 8, 0, 0, "100:7 wild"},
        {6, 32, 10, 8, 0, 0, "6:32  inverted"},
        {31, 17, 10, 8, 0, 0, "31:17 coprime"},
        {48, 5, 10, 2, 0, 0, "48:5  deep well"},
        {32, 16, 10, 8, 0.25, 0, "32:16 wobble"},
        {32, 16, 10, 8, 0, 3.0, "32:16 tilt"},
        {24, 24, 10, 9.9, 0, 0, "24:24 thin"},
        {13, 40, 10, 8, 0.2, 1.5, "13:40 wobble+tilt"},
        {256, 3, 10, 5, 0, 0, "256:3 extreme"},
    };
    int pass = 0;
    printf("%-22s %6s %5s %5s %5s | %8s %5s %5s %5s\n", "case", "cells",
           "quad", "ngon", "tri", "openInt", "nonMf", "fold", "degen");
    for (auto& c : cases) {
        Mesh m; std::vector<int> O, I;
        makeCase(c.N, c.M, c.rO, c.rI, c.wob, c.tilt, m, O, I);
        bridgeLoops(O, I, m);
        Report r = validate(m, O, I, {0, 0, 1});
        bool ok = r.interiorOpen == 0 && r.nonManifold == 0 && r.folds == 0 &&
                  r.degenerate == 0 && r.cells > 0;
        pass += ok;
        printf("%-22s %6d %5d %5d %5d | %8d %5d %5d %5d  %s\n", c.name, r.cells,
               r.quads, r.ngons, r.tris, r.interiorOpen, r.nonManifold,
               r.folds, r.degenerate, ok ? "OK" : "FAIL");
    }
    printf("\n%d/%d cases clean (watertight interior, manifold, no folds)\n",
           pass, (int)cases.size());
    return pass == (int)cases.size() ? 0 : 1;
}
