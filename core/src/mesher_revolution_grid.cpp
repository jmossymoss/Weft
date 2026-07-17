#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// One rim sample: azimuth u, height v, and the exact 3D curve point.
struct RevRimPt {
    double u;
    double v;
    gp_Pnt p;
};

// Closed transition strip between two full-circle rings whose azimuthal
// counts differ: monotone circular grouping by cumulative arc angle — quads
// where the counts advance together, a grouped n-gon where the dense ring
// contributes extra points, distributed around the whole ring. Winding
// matches the lattice cells (lower row forward, upper row backward). Factored
// out of meshRevolutionGrid so the annulus-body path shares the exact code
// (existing callers stay byte-identical).
void emitClosedRimStrip(MeshBuilder& out, int faceId, bool flip, double period,
                        const std::vector<uint32_t>& loI,
                        const std::vector<double>& loU,
                        const std::vector<uint32_t>& hiI,
                        const std::vector<double>& hiU) {
    const int nl = int(loI.size()), nh = int(hiI.size());
    if (nl < 3 || nh < 3) return;
    const bool loSparse = nl <= nh;
    const std::vector<uint32_t>& S = loSparse ? loI : hiI;
    const std::vector<double>& sU = loSparse ? loU : hiU;
    const std::vector<uint32_t>& D = loSparse ? hiI : loI;
    const std::vector<double>& dU = loSparse ? hiU : loU;
    const int ns = int(S.size()), nd = int(D.size());
    // Reference angle measured ALONG each ring's own order (cumulative
    // short-step deltas), not the raw wrapped u: a rim sampled in wire-chain
    // order winds once around but starts mid-circle (one seam wrap) and a
    // scalloped rim adds tiny local backsteps — a raw wrapped-u comparison
    // then misplaces the pairing. The cumulative angle is monotone from each
    // ring's first sample and closes at one period.
    auto cumAngle = [&](const std::vector<double>& U) {
        std::vector<double> c(U.size() + 1, 0.0);
        for (size_t i = 1; i <= U.size(); ++i) {
            double d = U[i % U.size()] - U[i - 1];
            d -= period * std::round(d / period);
            if (d < 0) d = 0;  // seam wrap / tiny scallop backsteps
            c[i] = c[i - 1] + d;
        }
        return c;
    };
    const std::vector<double> dCum = cumAngle(dU);  // size nd+1
    const std::vector<double> sCum = cumAngle(sU);  // size ns+1
    double off = sU[0] - dU[0];  // sparse start ahead of dense start
    off -= period * std::round(off / period);
    if (off < 0) off += period;
    auto dAt = [&](int i) {
        int w = ((i % nd) + nd) % nd;
        return dCum[w] + period * std::floor(double(i) / nd);
    };
    // m[k] = unwrapped dense index paired with sparse k, monotone, closing
    // after exactly one full turn.
    std::vector<int> m(ns + 1);
    double bd = 1e300;
    for (int i = 0; i < nd; ++i) {
        double d = std::abs(dCum[i] - off);
        d = std::min(d, period - d);
        if (d < bd) { bd = d; m[0] = i; }
    }
    for (int k = 1; k < ns; ++k) {
        double t = off + sCum[k];  // target angle from dense start
        int best = m[k - 1];
        double bestD = std::abs(dAt(best) - t);
        for (int i = m[k - 1] + 1; i <= m[0] + nd; ++i) {
            double d = std::abs(dAt(i) - t);
            if (d < bestD) { bestD = d; best = i; }
            if (dAt(i) > t + period / nd) break;
        }
        m[k] = best;
    }
    m[ns] = m[0] + nd;
    for (int k = 0; k < ns; ++k) {
        std::vector<uint32_t> ring2;
        if (loSparse) {
            ring2 = {S[k], S[(k + 1) % ns]};
            for (int i = m[k + 1]; i >= m[k]; --i) {
                ring2.push_back(D[((i % nd) + nd) % nd]);
            }
        } else {
            for (int i = m[k]; i <= m[k + 1]; ++i) {
                ring2.push_back(D[((i % nd) + nd) % nd]);
            }
            ring2.push_back(S[(k + 1) % ns]);
            ring2.push_back(S[k]);
        }
        ring2.erase(std::unique(ring2.begin(), ring2.end()), ring2.end());
        if (ring2.size() > 1 && ring2.front() == ring2.back()) {
            ring2.pop_back();
        }
        if (ring2.size() < 3) continue;
        out.addPolygon(std::move(ring2), faceId, flip);
    }
}

// Closed transition strip paired by 3D ARC FRACTION (meshAnnulusCRing's
// metric), not azimuth. A notched rim runs its samples DOWN a near-vertical
// wall and back up: in azimuth those samples pile at one angle, so an
// azimuth-paired strip collapses them into folded slivers. Arc fraction spends
// the wall's real length as loop distance, so the wall's samples pair one-to-
// one with the mating ring's own climb — quads up the wall, grouped n-gons
// only where one ring genuinely has more points along the same arc.
void emitClosedRimStripArc(MeshBuilder& out, int faceId, bool flip,
                           const std::vector<uint32_t>& loI,
                           const std::vector<gp_Pnt>& loP,
                           const std::vector<uint32_t>& hiI,
                           const std::vector<gp_Pnt>& hiP) {
    const int nl = int(loI.size()), nh = int(hiI.size());
    if (nl < 3 || nh < 3) return;
    const bool loSparse = nl <= nh;
    const std::vector<uint32_t>& S = loSparse ? loI : hiI;
    const std::vector<gp_Pnt>& sP = loSparse ? loP : hiP;
    const std::vector<uint32_t>& D = loSparse ? hiI : loI;
    const std::vector<gp_Pnt>& dP = loSparse ? hiP : loP;
    const int ns = int(S.size()), nd = int(D.size());
    // Cumulative loop distance (closing back to index 0), normalised to [0,1].
    auto cumFrac = [](const std::vector<gp_Pnt>& P) {
        std::vector<double> c(P.size() + 1, 0.0);
        for (size_t i = 1; i <= P.size(); ++i) {
            c[i] = c[i - 1] + P[i - 1].Distance(P[i % P.size()]);
        }
        const double per = c.back() > 1e-12 ? c.back() : 1.0;
        for (double& x : c) x /= per;
        return c;  // c[0]=0, c[n]=1, monotone
    };
    const std::vector<double> dCum = cumFrac(dP);  // size nd+1
    const std::vector<double> sCum = cumFrac(sP);  // size ns+1
    // Align sparse[0] to its nearest dense vertex in 3D; that is the loop
    // origin so the two rings' fractions are comparable.
    int base = 0;
    double best = 1e300;
    for (int i = 0; i < nd; ++i) {
        const double dd = dP[i].Distance(sP[0]);
        if (dd < best) { best = dd; base = i; }
    }
    const double off = dCum[base];  // dense-loop fraction of the origin
    auto dAt = [&](int i) {
        int w = ((i % nd) + nd) % nd;
        return dCum[w] + std::floor(double(i) / nd);
    };
    std::vector<int> m(ns + 1);
    m[0] = base;
    for (int k = 1; k < ns; ++k) {
        const double t = off + sCum[k];  // target fraction from the origin
        int bestI = m[k - 1];
        double bestD = std::abs(dAt(bestI) - t);
        for (int i = m[k - 1] + 1; i <= base + nd; ++i) {
            const double d = std::abs(dAt(i) - t);
            if (d < bestD) { bestD = d; bestI = i; }
            if (dAt(i) > t + 1.0 / nd) break;
        }
        m[k] = bestI;
    }
    m[ns] = base + nd;
    for (int k = 0; k < ns; ++k) {
        std::vector<uint32_t> ring2;
        if (loSparse) {
            ring2 = {S[k], S[(k + 1) % ns]};
            for (int i = m[k + 1]; i >= m[k]; --i) {
                ring2.push_back(D[((i % nd) + nd) % nd]);
            }
        } else {
            for (int i = m[k]; i <= m[k + 1]; ++i) {
                ring2.push_back(D[((i % nd) + nd) % nd]);
            }
            ring2.push_back(S[(k + 1) % ns]);
            ring2.push_back(S[k]);
        }
        ring2.erase(std::unique(ring2.begin(), ring2.end()), ring2.end());
        if (ring2.size() > 1 && ring2.front() == ring2.back()) {
            ring2.pop_back();
        }
        if (ring2.size() < 3) continue;
        out.addPolygon(std::move(ring2), faceId, flip);
    }
}

// Merge one OPEN smooth arc of a ribbon: lo run L[0..nl-1], hi run H[0..nh-1],
// endpoints already paired (L[0]~H[0], L[nl-1]~H[nh-1]). Walks both by
// cumulative arc fraction along the segment and, at each step, spends one edge
// of the locally sparser side while absorbing the run of the denser side into
// a single polygon: quads where the densities match, evenly grouped n-gons
// where they differ, no stray triangles. Winding: lo forward, hi backward.
static void emitRibbonSegment(MeshBuilder& out, int faceId, bool flip,
                              const std::vector<uint32_t>& L,
                              const std::vector<gp_Pnt>& LP,
                              const std::vector<uint32_t>& H,
                              const std::vector<gp_Pnt>& HP) {
    const int nl = int(L.size()), nh = int(H.size());
    if (nl < 2 || nh < 2) return;
    auto cum = [](const std::vector<gp_Pnt>& P) {
        std::vector<double> c(P.size(), 0.0);
        for (size_t i = 1; i < P.size(); ++i)
            c[i] = c[i - 1] + P[i - 1].Distance(P[i]);
        const double per = c.back() > 1e-12 ? c.back() : 1.0;
        for (double& x : c) x /= per;
        return c;  // c[0]=0, c[nl-1]=1
    };
    const std::vector<double> fL = cum(LP), fH = cum(HP);
    const double tol = 0.5 / std::max(nl, nh);
    int ia = 0, ib = 0, guard = 0;
    while ((ia < nl - 1 || ib < nh - 1) && guard++ < 2 * (nl + nh) + 8) {
        const int aStart = ia, bStart = ib;
        const bool aCan = ia < nl - 1, bCan = ib < nh - 1;
        if (aCan && bCan) {
            const double nfa = fL[ia + 1], nfb = fH[ib + 1];
            if (nfa >= nfb - tol && nfb >= nfa - tol) {
                ia++; ib++;  // aligned -> quad
            } else if (nfa < nfb) {
                // lo denser: spend one hi edge, absorb the lo run to it.
                ib++;
                while (ia < nl - 1 && fL[ia + 1] <= nfb + tol) ia++;
            } else {
                // hi denser: spend one lo edge, absorb the hi run to it.
                ia++;
                while (ib < nh - 1 && fH[ib + 1] <= nfa + tol) ib++;
            }
        } else if (aCan) {
            ia = nl - 1;
        } else {
            ib = nh - 1;
        }
        std::vector<uint32_t> poly;
        poly.reserve((ia - aStart) + (ib - bStart) + 2);
        for (int s = aStart; s <= ia; ++s) poly.push_back(L[s]);
        for (int s = ib; s >= bStart; --s) poly.push_back(H[s]);
        poly.erase(std::unique(poly.begin(), poly.end()), poly.end());
        if (poly.size() > 1 && poly.front() == poly.back()) poly.pop_back();
        if (poly.size() >= 3) out.addPolygon(std::move(poly), faceId, flip);
    }
}

// Symmetric ribbon between two closed rings whose sampling density varies
// LOCALLY (the foam notch is sampled FINER than the uniform shoulder along its
// rounded floor, yet COARSER along its flat top) and which BOTH carry the same
// near-vertical WALLS. A single sparse->dense assignment (emitClosedRimStripArc)
// can only group the denser ring into the sparser one's edges; where the
// nominally-sparse ring is the locally denser one it collapses into stray
// triangles/folded slivers. Here the caller supplies each ring's wall edge
// indices (`wL`/`wH`, found from the u,v profile where a plain 3D length test
// can't tell a wall from a wide flat-top chord); the walls are paired
// one-to-one (a clean vertical quad each) and the smooth arcs BETWEEN walls are
// merged by arc fraction. Pairing walls explicitly keeps them aligned even when
// the two loops' total perimeters differ (which, under a pure global
// arc-fraction walk, drifts the walls out of step and folds a cell across one
// at high column counts). `loI`/`hiI` are the two rings in the same rotational
// sense; winding matches the body lattice (lo forward, hi backward).
void emitClosedRibbonMerge(MeshBuilder& out, int faceId, bool flip,
                           const std::vector<uint32_t>& loI,
                           const std::vector<gp_Pnt>& loP,
                           const std::vector<uint32_t>& hiI,
                           const std::vector<gp_Pnt>& hiP,
                           const std::vector<int>& wL,
                           const std::vector<int>& wH) {
    const int na = int(loI.size()), nb = int(hiI.size());
    if (na < 3 || nb < 3) return;

    // Fall back to a plain global arc-fraction walk when the walls don't match
    // up (or there are none) — a smooth ribbon that path handles cleanly.
    auto globalMerge = [&]() {
        auto cumFrac = [](const std::vector<gp_Pnt>& P) {
            const int n = int(P.size());
            std::vector<double> c(n + 1, 0.0);
            for (int i = 1; i <= n; ++i)
                c[i] = c[i - 1] + P[i - 1].Distance(P[i % n]);
            const double per = c[n] > 1e-12 ? c[n] : 1.0;
            for (double& x : c) x /= per;
            return c;
        };
        const std::vector<double> ca = cumFrac(loP), cb = cumFrac(hiP);
        int a0 = 0;
        double best = 1e300;
        for (int i = 0; i < na; ++i) {
            const double d = loP[i].Distance(hiP[0]);
            if (d < best) { best = d; a0 = i; }
        }
        auto fa = [&](int s) {
            int idx = a0 + s;
            double turns = 0;
            while (idx >= na) { idx -= na; turns += 1.0; }
            return ca[idx] + turns - ca[a0];
        };
        auto fb = [&](int s) { return cb[s]; };
        const double tol = 0.5 / std::max(na, nb);
        int ia = 0, ib = 0, guard = 0;
        while ((ia < na || ib < nb) && guard++ < 2 * (na + nb) + 8) {
            const int aStart = ia, bStart = ib;
            const bool aCan = ia < na, bCan = ib < nb;
            if (aCan && bCan) {
                const double nfa = fa(ia + 1), nfb = fb(ib + 1);
                if (nfa >= nfb - tol && nfb >= nfa - tol) {
                    ia++; ib++;
                } else if (nfa < nfb) {
                    ib++;
                    while (ia < na && fa(ia + 1) <= fb(ib) + tol) ia++;
                } else {
                    ia++;
                    while (ib < nb && fb(ib + 1) <= fa(ia) + tol) ib++;
                }
            } else if (aCan) {
                ia = na;
            } else {
                ib = nb;
            }
            std::vector<uint32_t> poly;
            for (int s = aStart; s <= ia; ++s) poly.push_back(loI[(a0 + s) % na]);
            for (int s = ib; s >= bStart; --s) poly.push_back(hiI[s % nb]);
            poly.erase(std::unique(poly.begin(), poly.end()), poly.end());
            if (poly.size() > 1 && poly.front() == poly.back()) poly.pop_back();
            if (poly.size() >= 3) out.addPolygon(std::move(poly), faceId, flip);
        }
    };
    if (wL.empty() || wL.size() != wH.size()) {
        globalMerge();
        return;
    }
    const int W = int(wL.size());
    // Correspond the two rings' walls: rotate the hi wall list so hi wall r
    // pairs with the lo wall nearest in 3D (by the wall edge's midpoint),
    // preserving cyclic order.
    auto wallMid = [](const std::vector<gp_Pnt>& P, int i) {
        return gp_Pnt((P[i].XYZ() + P[(i + 1) % P.size()].XYZ()) / 2.0);
    };
    int rot = 0;
    double bestSum = 1e300;
    for (int r = 0; r < W; ++r) {
        double sum = 0;
        for (int k = 0; k < W; ++k)
            sum += wallMid(loP, wL[k]).Distance(wallMid(hiP, wH[(k + r) % W]));
        if (sum < bestSum) { bestSum = sum; rot = r; }
    }
    // Emit each wall as its own quad, and merge the smooth arc that follows it
    // (up to the next wall) as an open segment.
    for (int k = 0; k < W; ++k) {
        const int lw = wL[k];
        const int hw = wH[(k + rot) % W];
        // Wall quad: lo bottom->top, hi top->bottom (lo forward, hi backward).
        {
            std::vector<uint32_t> quad = {loI[lw], loI[(lw + 1) % na],
                                          hiI[(hw + 1) % nb], hiI[hw]};
            quad.erase(std::unique(quad.begin(), quad.end()), quad.end());
            if (quad.size() > 1 && quad.front() == quad.back()) quad.pop_back();
            if (quad.size() >= 3) out.addPolygon(std::move(quad), faceId, flip);
        }
        // Smooth arc from this wall's top to the next wall's bottom.
        const int lwNext = wL[(k + 1) % W];
        const int hwNext = wH[(k + 1 + rot) % W];
        std::vector<uint32_t> L;
        std::vector<gp_Pnt> LP;
        for (int i = (lw + 1) % na;; i = (i + 1) % na) {
            L.push_back(loI[i]);
            LP.push_back(loP[i]);
            if (i == lwNext) break;
        }
        std::vector<uint32_t> H;
        std::vector<gp_Pnt> HP;
        for (int j = (hw + 1) % nb;; j = (j + 1) % nb) {
            H.push_back(hiI[j]);
            HP.push_back(hiP[j]);
            if (j == hwNext) break;
        }
        emitRibbonSegment(out, faceId, flip, L, LP, H, HP);
    }
}

// Mesh a tall mismatched two-rim revolution band the ANNULUS way (modelled on
// meshAnnulusCRing). The strip reconcile in meshRevolutionGrid drives the
// interior off the SPARSE rim and bridges the dense rim with one full-height
// diagonal strip — which makes `radial` dead (the body stays at the sparse
// count) and shears the body. Instead:
//   * the DENSE (flat) rim drives clean STRAIGHT columns (nu = its count, so
//     `radial` densifies the whole body),
//   * interior rings are HORIZONTAL (constant v) — the body reads as quad
//     rings and `axial` adds more,
//   * the count reduction (dense->sparse) AND the sparse rim's height
//     variation (a notched shoulder) are both absorbed in ONE short band
//     adjacent to that rim, via arc-fraction grouped n-gons distributed around
//     the ring (emitClosedRimStrip) — never a full-height diagonal strip.
// Watertight because BOTH rims are emitted as their exact solved samples; only
// the face-private interior rings carry the dense column count.
bool meshRevolutionAnnulusBody(const BRepAdaptor_Surface& surf, int faceId,
                               const std::vector<RevRimPt>& denseRim,
                               const std::vector<RevRimPt>& notchRim,
                               int nvBody, bool flip, MeshBuilder& out) {
    const int nu = int(denseRim.size());
    if (nu < 3 || notchRim.size() < 3 || nvBody < 1) return false;
    const double period = surf.LastUParameter() - surf.FirstUParameter();

    // Dense rim height (flat) and the notch rim's v-extent.
    double vDense = 0;
    for (const RevRimPt& r : denseRim) vDense += r.v;
    vDense /= nu;
    double vNmin = 1e300, vNmax = -1e300;
    for (const RevRimPt& r : notchRim) {
        vNmin = std::min(vNmin, r.v);
        vNmax = std::max(vNmax, r.v);
    }
    const double u0 = surf.FirstUParameter();
    // The notch rim rises toward the dense rim (denseBelow) or hangs below it.
    // sgn points from the body toward the notch.
    const bool denseBelow = vDense <= vNmin;
    const double sgn = denseBelow ? 1.0 : -1.0;
    const double notchRange = std::max(1e-9, vNmax - vNmin);
    // `band` is the short height of the final reduction cells: the shoulder
    // ring sits `band` inside the notch's own profile so that closing band
    // (dense->sparse count change) never goes degenerate at the notch's
    // nearest point.
    const double band = std::max(1e-6, 0.15 * notchRange);

    // Column azimuths follow the dense rim (so it welds one-to-one through the
    // horizontal lattice), unwrapped monotone from its first sample.
    std::vector<double> colU(nu);
    colU[0] = denseRim[0].u;
    for (int i = 1; i < nu; ++i) {
        double d = denseRim[i].u - denseRim[i - 1].u;
        d -= period * std::round(d / period);
        if (d < 0) d = 0;
        colU[i] = colU[i - 1] + d;
    }

    // Does the notch rim carry NEAR-VERTICAL WALLS that the uniform shoulder
    // can't sample one-to-one? A wall is two consecutive rim samples whose
    // azimuthal gap is far smaller than a shoulder column's spacing but whose
    // v-jump is a fair fraction of the band — the foam barrel's notch leaps
    // ~46 in v over ~0 azimuth. Only THAT pathology needs the wall-aware
    // profile + symmetric reduction below; a smoothly scalloped notch (every
    // other annulus body: nasty_cheese's shallow dishes, weldment's chamfer
    // ring) reconciles cleanly the ordinary way and MUST stay byte-identical.
    const double shoulderStep = period / nu;
    double minNotchGap = 1e300, maxNotchVJump = 0;
    for (size_t i = 0; i < notchRim.size(); ++i) {
        const RevRimPt& a = notchRim[i];
        const RevRimPt& b = notchRim[(i + 1) % notchRim.size()];
        double du = b.u - a.u;
        du -= period * std::round(du / period);
        const double dv = std::abs(b.v - a.v);
        if (std::abs(du) < 0.5 * shoulderStep) {
            minNotchGap = std::min(minNotchGap, std::abs(du));
            maxNotchVJump = std::max(maxNotchVJump, dv);
        }
    }
    const bool notchHasWalls =
        minNotchGap < 0.5 * shoulderStep && maxNotchVJump > 0.15 * notchRange;
    // How far is the notch OVERSAMPLED past the uniform shoulder? Count the
    // most rim samples that fall inside any single shoulder-column-wide
    // azimuth window. 1-2 is an ordinary rim (or a lone wall pair) the classic
    // arc-fraction strip reconciles cleanly; >=3 means a genuinely finer-
    // sampled arc (the foam notch's rounded floor packs 5 per column) that the
    // classic strip collapses into folded slivers. Only a notch that BOTH
    // carries a steep wall AND is oversampled that far needs the wall-aware
    // symmetric reduction; everything else stays byte-identical.
    int maxInWin = 0;
    {
        std::vector<double> nuw;
        nuw.reserve(notchRim.size());
        for (const RevRimPt& r : notchRim)
            nuw.push_back(r.u - period * std::floor((r.u - u0) / period));
        for (double c : nuw) {
            int cnt = 0;
            for (double x : nuw) {
                double d = x - c;
                d -= period * std::round(d / period);
                if (std::abs(d) <= shoulderStep) ++cnt;
            }
            maxInWin = std::max(maxInWin, cnt);
        }
    }
    const bool useSymmetric = notchHasWalls && maxInWin >= 3;

    // Notch v-profile v(u): the sparse rim's height as a function of azimuth,
    // so a dense column can read the notch shape at its own azimuth. When the
    // notch carries near-vertical walls it is built by UNWRAPPING the rim in
    // WIRE (loop) order rather than u-sorting the samples: a wall is two
    // samples that share a u but jump ~46 in v; a u-sort tie-breaks that pair
    // by v, and where the LOWER sample sorts ahead of the upper one the
    // piecewise-linear notchVAt ramps from the previous top sample down across
    // the whole azimuthal gap — smearing one wall into a long diagonal (the
    // fold's root cause). Walking the loop keeps each wall a zero-width
    // (u-constant) step so both walls stay SHARP. Without walls the classic
    // u-sort is kept verbatim so every other annulus body is byte-identical.
    std::vector<std::pair<double, double>> prof;
    prof.reserve(notchRim.size() + 1);
    double profLo = 0, profHi = 0;
    if (useSymmetric) {
        double uAcc =
            notchRim[0].u - period * std::floor((notchRim[0].u - u0) / period);
        prof.push_back({uAcc, notchRim[0].v});
        for (size_t i = 1; i < notchRim.size(); ++i) {
            double d = notchRim[i].u - notchRim[i - 1].u;
            d -= period * std::round(d / period);  // shortest step
            uAcc += d;
            prof.push_back({uAcc, notchRim[i].v});
        }
        // Normalise to overall-increasing u (reverse a clockwise wire).
        if (prof.size() >= 2 && prof.back().first < prof.front().first) {
            std::reverse(prof.begin(), prof.end());
        }
        profLo = prof.front().first;
        profHi = prof.back().first;
    } else {
        for (const RevRimPt& r : notchRim) {
            double uw = r.u - period * std::floor((r.u - u0) / period);
            prof.push_back({uw, r.v});
        }
        std::sort(prof.begin(), prof.end());
    }
    auto notchVAt = [&](double u) -> double {
        if (prof.size() == 1) return prof[0].second;
        if (useSymmetric) {
            // Loop-order profile: bring u into [profLo, profLo + period).
            double uu = u - period * std::floor((u - profLo) / period);
            double ua, va, ub, vb;
            if (uu <= profHi) {
                size_t hi = 0;
                while (hi < prof.size() && prof[hi].first < uu) ++hi;
                if (hi == 0) hi = 1;
                ua = prof[hi - 1].first; va = prof[hi - 1].second;
                ub = prof[hi].first;     vb = prof[hi].second;
            } else {
                ua = profHi;          va = prof.back().second;
                ub = profLo + period; vb = prof.front().second;
            }
            const double s = ub - ua;
            const double t = s > 1e-12 ? (uu - ua) / s : 0.0;
            return va + (vb - va) * std::clamp(t, 0.0, 1.0);
        }
        // Classic u-sorted profile (unchanged).
        double uu = u - period * std::floor((u - u0) / period);
        size_t hi = 0;
        while (hi < prof.size() && prof[hi].first < uu) ++hi;
        double ua, va, ub, vb;
        if (hi == 0) {
            ua = prof.back().first - period; va = prof.back().second;
            ub = prof.front().first;         vb = prof.front().second;
        } else if (hi == prof.size()) {
            ua = prof.back().first;          va = prof.back().second;
            ub = prof.front().first + period; vb = prof.front().second;
        } else {
            ua = prof[hi - 1].first; va = prof[hi - 1].second;
            ub = prof[hi].first;     vb = prof[hi].second;
        }
        const double s = ub - ua;
        const double t = s > 1e-12 ? (uu - ua) / s : 0.0;
        return va + (vb - va) * std::clamp(t, 0.0, 1.0);
    };

    // Per-column TOP profile: each dense column rises to the notch's own
    // v-profile at that azimuth (pulled `band` back into the body so the final
    // reduction band has a short, uniform strip). This is the SHOULDER ring.
    // The vertical columns follow the notch shape and reach the top boundary —
    // there is no flat ceiling that stops the body short of the feature edge.
    // Clamp the top to stay a body-band above the dense rim even where the
    // notch dips close to it (no degenerate/inverted body cells).
    std::vector<double> topV(nu);
    for (int i = 0; i < nu; ++i) {
        double v = notchVAt(colU[i]) - sgn * band;
        if (denseBelow) v = std::max(v, vDense + band);
        else            v = std::min(v, vDense - band);
        topV[i] = v;
    }

    // Body rings: row 0 is the dense rim's EXACT samples (flat, shared edge);
    // rows 1..nvBody march each column from vDense up to its own top profile
    // topV[i]. A single band (default axial) already spans the FULL height,
    // bottom rim to notched top; `axial` adds intermediate rows. Because the
    // count is constant (nu columns throughout) this is a clean one-to-one
    // quad lattice — the notch's vertical walls just tilt the top edge of the
    // affected cells, they do not fold.
    const int bodyRows = nvBody + 1;
    std::vector<std::vector<uint32_t>> ring(bodyRows);
    for (int j = 0; j < bodyRows; ++j) {
        ring[j].resize(nu);
        const double t = double(j) / nvBody;
        for (int i = 0; i < nu; ++i) {
            if (j == 0) {
                ring[j][i] = out.addVertex(denseRim[i].p,
                                           {faceId, denseRim[i].u, vDense});
            } else {
                const double v = vDense + (topV[i] - vDense) * t;
                const gp_Pnt p = surf.Value(colU[i], v);
                ring[j][i] = out.addVertex(p, {faceId, colU[i], v});
            }
        }
    }
    for (int j = 0; j + 1 < bodyRows; ++j) {
        for (int i = 0; i < nu; ++i) {
            const int i2 = (i + 1) % nu;
            out.addPolygon({ring[j][i], ring[j][i2], ring[j + 1][i2],
                            ring[j + 1][i]},
                           faceId, flip);
        }
    }

    // The profiled top ring IS the last body row (dense count) — the shoulder
    // the single reduction band pairs down to the sparse notch rim.
    std::vector<uint32_t>& shoulder = ring[bodyRows - 1];
    std::vector<gp_Pnt> shoulderP(nu);
    for (int i = 0; i < nu; ++i) shoulderP[i] = surf.Value(colU[i], topV[i]);

    // The notch rim's EXACT samples (its own solved count) and points.
    std::vector<uint32_t> notchId(notchRim.size());
    std::vector<gp_Pnt> notchP(notchRim.size());
    for (size_t i = 0; i < notchRim.size(); ++i) {
        notchP[i] = notchRim[i].p;
        notchId[i] =
            out.addVertex(notchRim[i].p, {faceId, notchRim[i].u, notchRim[i].v});
    }

    // One short reduction band from the profiled shoulder (dense, uniform
    // count) to the notch rim (sparse, non-uniform count). For a WALLED notch
    // (foam) a SYMMETRIC arc-fraction merge groups whichever ring is locally
    // denser into the other's edges, so the notch's finely-sampled floor and
    // its coarse flat top both reconcile as evenly grouped n-gons (no stray
    // triangles) and its near-vertical walls pair one-to-one (no folded
    // slivers). A smoothly scalloped notch keeps the classic arc-fraction
    // strip verbatim (byte-identical to before). Winding matches the lattice
    // (the notch on the high side is the upper row).
    if (useSymmetric) {
        // Wall edges from the u,v profile: a jump in v across one edge of a
        // fair fraction of the band. The shoulder's walls are the columns that
        // straddle the notch's own walls (topV steps there); the notch's walls
        // are its near-vertical rim segments. A 3D length test can't find the
        // notch walls (its wide flat-top chords are as long), so pass them
        // explicitly. Both rings carry the same 2 walls (mirrored).
        const double vWall = 0.4 * notchRange;
        std::vector<int> shoulderWalls, notchWalls;
        for (int i = 0; i < nu; ++i)
            if (std::abs(topV[(i + 1) % nu] - topV[i]) > vWall)
                shoulderWalls.push_back(i);
        for (size_t j = 0; j < notchRim.size(); ++j)
            if (std::abs(notchRim[(j + 1) % notchRim.size()].v -
                         notchRim[j].v) > vWall)
                notchWalls.push_back(int(j));
        if (denseBelow) {
            emitClosedRibbonMerge(out, faceId, flip, shoulder, shoulderP,
                                  notchId, notchP, shoulderWalls, notchWalls);
        } else {
            emitClosedRibbonMerge(out, faceId, flip, notchId, notchP, shoulder,
                                  shoulderP, notchWalls, shoulderWalls);
        }
    } else if (denseBelow) {
        emitClosedRimStripArc(out, faceId, flip, shoulder, shoulderP, notchId,
                              notchP);
    } else {
        emitClosedRimStripArc(out, faceId, flip, notchId, notchP, shoulder,
                              shoulderP);
    }
    return true;
}


bool meshRevolutionGrid(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        const Model& model, const std::vector<int>& rimEdges,
                        const std::vector<int>& solvedEdge, int faceId,
                        int nu, int nv, MeshBuilder& out,
                        const std::vector<double>* vRowsOpt,
                        const std::vector<int>* rimLowOpt,
                        std::array<int, 2>* built,
                        bool dedupeDriveRim,
                        bool decoupleSeams) {
    nu = std::max(3, nu);
    nv = std::max(1, nv);
    const double v0 = surf.FirstVParameter();
    const double v1 = surf.LastVParameter();
    const double vspan = std::max(1e-12, v1 - v0);
    const double period = surf.LastUParameter() - surf.FirstUParameter();
    const bool vWrap = surf.IsVClosed();
    // Explicit row positions (insert faces put rows exactly at each slot
    // band's v-extents so deleted cells stay strictly interior and the
    // staircase closes). Only meaningful for open-v bands.
    const std::vector<double>* vRows =
        (!vWrap && vRowsOpt && vRowsOpt->size() >= 2) ? vRowsOpt : nullptr;
    if (vRows) nv = int(vRows->size()) - 1;
    if (vWrap) nv = std::max(3, nv);  // a wrapped ring of <3 rows is flat
    // Pole-to-pole band (full sphere / both-ends-closed revolve): both
    // end rows collapse to a point, so nv==1 would emit zero polygons.
    // The added interior ring is face-private (poles are degenerate
    // edges, the seam is internal), so no border sampling changes.
    if (!vWrap && !vRows && nv < 2) {
        auto rowDegenerate = [&](double v) {
            const gp_Pnt p0 = surf.Value(surf.FirstUParameter(), v);
            for (int i = 1; i < 8; ++i) {
                double u = surf.FirstUParameter() + period * i / 8.0;
                if (surf.Value(u, v).Distance(p0) > 1e-9) return false;
            }
            return true;
        };
        if (rowDegenerate(v0) && rowDegenerate(v1)) nv = 2;
    }
    double dv = (v1 - v0) / nv;
    int rows = vWrap ? nv : nv + 1;
    auto rowV = [&](int j) {
        return vRows ? (*vRows)[std::min<size_t>(j, vRows->size() - 1)]
                     : v0 + j * dv;
    };
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    // Rim rows sample the rim EDGE CURVES (like every chain mesher), so
    // multi-arc rims keep their joint vertices and neighbours weld
    // bit-identically; interior rows interpolate each column's u between
    // the two rims (wrap-shortest), twisting gently if the rims' origins
    // differ. Falls back to plain uniform rings when there are no usable
    // rims (full tori) or the two rims disagree in count.
    struct RimPt {
        double u;
        double v;
        gp_Pnt p;
    };
    std::vector<RimPt> rim[2];
    // Chain indices that are B-rep edge JUNCTIONS (each edge's first
    // sample). Under decoupled seams these are contract points — three
    // faces meet at a rim corner, so the pure lattice must emit them
    // even when its own column count skips past.
    std::array<std::set<size_t>, 2> rimCorner;
    // Rim rows are built in WIRE CHAIN ORDER, not sorted by u: a
    // countersunk bore's rim is arcs joined by short v-steps, and a
    // u-sort interleaves the step samples between arc samples — the
    // emitted row zigzags and consecutive border samples lose their
    // direct polygon edge (contract violation, observed on 1797609in
    // face 37). Walking the wire keeps every chain adjacent by
    // construction; only the overall circular direction is normalized.
    const std::set<int> rimSet(rimEdges.begin(), rimEdges.end());
    std::set<int> rimSeen;
    auto sampleRimEdge = [&](const TopoDS_Edge& edge, int eid) {
        double f2, l2, f3, l3;
        Handle(Geom2d_Curve) pc =
            BRep_Tool::CurveOnSurface(edge, face, f2, l2);
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        if (pc.IsNull() || c3.IsNull()) return;
        rimSeen.insert(eid);
        gp_Pnt2d mid = pc->Value((f2 + l2) / 2);
        // Chain membership beats nearest-end: a deep saddle rim wanders
        // past the band middle but still belongs to its chain.
        int side;
        if (rimLowOpt) {
            side = std::find(rimLowOpt->begin(), rimLowOpt->end(), eid) !=
                           rimLowOpt->end()
                       ? 0
                       : 1;
        } else {
            side = std::abs(mid.Y() - v0) < std::abs(mid.Y() - v1) ? 0 : 1;
        }
        int n = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
        if (n < 1) n = nu;
        const bool rev = edge.Orientation() == TopAbs_REVERSED;
        const double ph = closedEdgePhase(edge, model);
        rimCorner[side].insert(rim[side].size());
        for (double t : edgeSampleFractions(eid, n, ph, rev,
                                            /*includeLast=*/false, nullptr,
                                            &model)) {
            gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
            gp_Pnt p = c3->Value(f3 + (l3 - f3) * t);
            double u = uv.X();
            u -= period * std::floor((u - surf.FirstUParameter()) / period);
            rim[side].push_back({u, uv.Y(), p});
        }
    };
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face);
             we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            if (BRep_Tool::Degenerated(edge)) continue;
            const int eid = model.edges.FindIndex(edge);
            if (eid < 1 || !rimSet.count(eid) || rimSeen.count(eid)) {
                continue;
            }
            if (BRep_Tool::IsClosed(edge, face)) continue;  // seam
            sampleRimEdge(edge, eid);
        }
    }
    // Sloppy wires make BRepTools_WireExplorer drop edges silently —
    // append any rim edge it missed (order degrades locally, borders
    // stay complete).
    for (int eid : rimEdges) {
        if (eid < 1 || eid > model.edgeCount() || rimSeen.count(eid)) {
            continue;
        }
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        sampleRimEdge(edge, eid);
    }
    // Normalize both rims to the same circular direction (ascending u
    // overall) without disturbing chain adjacency.
    for (int k = 0; k < 2; ++k) {
        if (rim[k].size() < 2) continue;
        double turn = 0;
        for (size_t i = 0; i + 1 < rim[k].size(); ++i) {
            double d = rim[k][i + 1].u - rim[k][i].u;
            d -= period * std::round(d / period);
            turn += d;
        }
        if (turn < 0) {
            std::reverse(rim[k].begin(), rim[k].end());
            std::set<size_t> flipped;
            for (size_t c : rimCorner[k]) {
                flipped.insert(rim[k].size() - 1 - c);
            }
            rimCorner[k] = std::move(flipped);
        }
    }

    // Decoupled seams, mismatched rims: NO transition strips, NO
    // irreconcilable bail — the band emits a PURE uniform lattice at its
    // own solved count. Rim rows are the rim chains resampled at the
    // lattice's nu column azimuths, PLUS every B-rep edge junction
    // (corner) on the chain: corners are contract points (three faces
    // meet there) and they partition the row so every emitted rim
    // segment lies within ONE B-rep edge — which is what lets the
    // post-weld stitcher close each seam against the neighbours' own
    // counts. Non-column corners ride inside the boundary cells as
    // extra polygon vertices (quad -> 5-gon, the absorption pattern).
    // Only u-monotone rims that wind one full period qualify — a rim
    // that doubles back in azimuth keeps the strict machinery below.
    if (decoupleSeams && !vWrap && !vRows &&
        rim[0].size() >= 3 && rim[1].size() >= 3 &&
        rim[0].size() != rim[1].size()) {
        struct StitchRow {
            std::vector<uint32_t> col;    // one vertex per lattice column
            std::vector<double> colV;     // that vertex's v (for columns)
            // Non-column corner verts per column gap, ascending azimuth.
            std::vector<std::vector<uint32_t>> extra;
            bool ok = false;
        };
        const double phase = rim[0].front().u;
        auto buildRow = [&](const std::vector<RimPt>& R,
                            const std::set<size_t>& corners) -> StitchRow {
            StitchRow row;
            const size_t n = R.size();
            std::vector<double> uu(n + 1);
            uu[0] = R[0].u;
            for (size_t k = 1; k <= n; ++k) {
                double d = R[k % n].u - R[k - 1].u;
                d -= period * std::round(d / period);
                if (d < -1e-6 * period) return row;  // doubles back
                // An iso-azimuth run (same u, real v step) is a slit or
                // notch SIDE bundled into the rim chain, not rim
                // material — v-interpolated columns would span the gap
                // and the line border would never be emitted (nasty
                // face 6: full-height slit lines, 4 permanent opens).
                if (std::abs(d) < 1e-6 * period &&
                    std::abs(R[k % n].v - R[k - 1].v) > 0.02 * vspan) {
                    return row;
                }
                uu[k] = uu[k - 1] + std::max(0.0, d);
            }
            // Must wind exactly one full turn to be a closed ring.
            if (std::abs(uu[n] - uu[0] - period) > 0.05 * period) {
                return row;
            }
            // Column azimuths in this chain's unwrapped frame.
            std::vector<double> cu(nu);
            for (int i = 0; i < nu; ++i) {
                double t = phase + period * i / double(nu);
                cu[i] = uu[0] +
                        std::fmod(t - uu[0] + 4.0 * period, period);
            }
            // Nearest corner to each column (circular distance): within
            // the snap band the corner BECOMES the column vertex, so no
            // sliver edge separates them.
            const double snap = 0.15 * period / double(nu);
            std::vector<int> colCorner(nu, -1);
            std::vector<char> consumed(n, 0);
            for (int i = 0; i < nu; ++i) {
                double best = snap;
                for (size_t c : corners) {
                    double d = std::abs(uu[c] - cu[i]);
                    d = std::min(d, period - std::min(d, period));
                    if (d < best) {
                        best = d;
                        colCorner[i] = int(c);
                    }
                }
                if (colCorner[i] >= 0) consumed[colCorner[i]] = 1;
            }
            row.col.resize(nu);
            row.colV.resize(nu);
            for (int i = 0; i < nu; ++i) {
                if (colCorner[i] >= 0) {
                    const RimPt& C = R[colCorner[i]];
                    row.col[i] = out.addVertex(C.p, {faceId, C.u, C.v});
                    row.colV[i] = C.v;
                    continue;
                }
                const double target = cu[i];
                size_t k = std::upper_bound(uu.begin(), uu.end(), target) -
                           uu.begin();
                k = std::clamp<size_t>(k, 1, n);
                const RimPt& A = R[k - 1];
                const RimPt& B = R[k % n];
                const double span = std::max(1e-12, uu[k] - uu[k - 1]);
                const double t =
                    std::clamp((target - uu[k - 1]) / span, 0.0, 1.0);
                const double vI = A.v + t * (B.v - A.v);
                double uI = target;
                uI -= period *
                      std::floor((uI - surf.FirstUParameter()) / period);
                const gp_Pnt p = surf.Value(uI, vI);
                row.col[i] = out.addVertex(p, {faceId, uI, vI});
                row.colV[i] = vI;
            }
            // Remaining corners ride the cell whose azimuth gap holds
            // them.
            row.extra.assign(nu, {});
            std::vector<std::pair<double, size_t>> loose;
            for (size_t c : corners) {
                if (!consumed[c]) loose.push_back({uu[c], c});
            }
            std::sort(loose.begin(), loose.end());
            for (const auto& [ucRaw, c] : loose) {
                // Shift into [cu[0], cu[0] + period).
                const double uc =
                    cu[0] +
                    std::fmod(ucRaw - cu[0] + 4.0 * period, period);
                int gap = int((uc - cu[0]) / (period / double(nu)));
                gap = std::clamp(gap, 0, nu - 1);
                const RimPt& C = R[c];
                row.extra[gap].push_back(
                    out.addVertex(C.p, {faceId, C.u, C.v}));
            }
            row.ok = true;
            return row;
        };
        StitchRow lo = buildRow(rim[0], rimCorner[0]);
        StitchRow hi = lo.ok ? buildRow(rim[1], rimCorner[1]) : StitchRow{};
        if (lo.ok && hi.ok) {
            dbg("revgrid face %d: stitch pure lattice nu=%d nv=%d "
                "(rim rows at column azimuths + %zu/%zu corners)",
                faceId, nu, nv, rimCorner[0].size(), rimCorner[1].size());
            // Interior rings between the two rim rows, columns straight
            // in azimuth, v lerped between the rims' column profiles.
            std::vector<std::vector<uint32_t>> ringS(nv + 1);
            ringS[0] = lo.col;
            ringS[nv] = hi.col;
            for (int j = 1; j < nv; ++j) {
                ringS[j].resize(nu);
                const double w = double(j) / double(nv);
                for (int i = 0; i < nu; ++i) {
                    double uI = phase + period * i / double(nu);
                    uI -= period * std::floor(
                                       (uI - surf.FirstUParameter()) /
                                       period);
                    const double vI =
                        lo.colV[i] + (hi.colV[i] - lo.colV[i]) * w;
                    ringS[j][i] =
                        out.addVertex(surf.Value(uI, vI), {faceId, uI, vI});
                }
            }
            for (int j = 0; j < nv; ++j) {
                for (int i = 0; i < nu; ++i) {
                    const int i2 = (i + 1) % nu;
                    std::vector<uint32_t> poly;
                    poly.push_back(ringS[j][i]);
                    if (j == 0) {
                        for (uint32_t v : lo.extra[i]) poly.push_back(v);
                    }
                    poly.push_back(ringS[j][i2]);
                    poly.push_back(ringS[j + 1][i2]);
                    if (j == nv - 1) {
                        const auto& ex = hi.extra[i];
                        for (auto it = ex.rbegin(); it != ex.rend(); ++it) {
                            poly.push_back(*it);
                        }
                    }
                    poly.push_back(ringS[j + 1][i]);
                    poly.erase(std::unique(poly.begin(), poly.end()),
                               poly.end());
                    if (poly.size() > 1 && poly.front() == poly.back()) {
                        poly.pop_back();
                    }
                    if (poly.size() < 3) continue;
                    out.addPolygon(std::move(poly), faceId, flip);
                }
            }
            if (built) *built = {nu, nv};
            return true;
        }
    }
    // The rim samples OWN the border contract — they are what the
    // neighbouring faces emit on the shared edges, and a border row may
    // never be re-spaced (doctrine). When the rims disagree with the
    // requested radial count, the rims win: if they agree with each
    // other (or there is only one), the whole lattice follows them; if
    // the two rims themselves differ, the interior keeps the requested
    // density and each rim is stitched to its neighbouring uniform ring
    // by a closed transition strip of quads/5-gons below.
    const int nRim0 = int(rim[0].size());
    const int nRim1 = int(rim[1].size());
    const bool rim0ok = !vWrap && nRim0 >= 3;
    const bool rim1ok = !vWrap && nRim1 >= 3;
    if (rim0ok && nRim0 != nu && (!rim1ok || nRim1 == nRim0)) {
        nu = nRim0;
    } else if (!rim0ok && rim1ok && nRim1 != nu) {
        nu = nRim1;
    }
    dbg("revgrid face %d: nu=%d nv=%d rims=%d/%d wrap=%d rimEdges=%zu",
        faceId, nu, nv, nRim0, nRim1, vWrap ? 1 : 0, rimEdges.size());
    // A single-rim SPHERE patch whose open end collapses to a pole reads as a
    // smooth dome — Plasticity caps such a corner blend with one disk n-gon,
    // not a tri fan. So on this exact case suppress the pole vertex and close
    // the innermost real ring with a single n-gon (done in the emission below).
    // Gated tightly: only GeomAbs_Sphere with exactly one usable rim (rim0ok
    // XOR rim1ok). A cone's apex is a genuine SHARP tip (its fixture asserts
    // the fan, and a flat cap would lose the point), and the rimless full
    // sphere fixture (rims=0/0) has two poles and no single rim — both are
    // excluded here and keep fanning byte-identically.
    const bool sphereCapPole =
        surf.GetType() == GeomAbs_Sphere && (rim0ok != rim1ok);
    // Mismatched-but-usable rims: rather than demote the whole face to the
    // contract floor (a tri soup), keep each rim's exact samples and absorb
    // the count difference in a transition strip. On an analytic revolution
    // surface (cylinder/cone/torus) the interior is a straight-column grid
    // whose columns are rulings and emitClosedStrip bridges each rim to it
    // (the non-chained path below). This only holds where the band is tall
    // enough that the strip cells stay convex; on a THIN tube the strip
    // degenerates into folded lunes (the case this bail originally guarded),
    // so measure the band height against the rim's azimuthal chord and fall
    // back to the floor when too thin.
    bool stripReconcile = false;
    // Carried out of the reconcile test for the interior-row bump below.
    double reconBandH = 0;     // band height (v0->v1 chord)
    double reconAzStep = 0;    // driving rim's mean azimuthal chord
    bool reconFrayStrip = false;  // the flat strip frays (needs row bump)
    // Deduped drive-rim azimuths (set only when the drive rim carries
    // ISO-AZIMUTH runs — notch walls whose samples stack at one angle).
    // Interior columns then use one column per DISTINCT azimuth and the
    // drive rim welds through the closed strip like the sparse rim,
    // instead of a 1:1 index weld that turns every stacked run into
    // zero-width folded columns (foam body face 183's top-rim bite).
    std::vector<double> reconColU;
    // Which rim welds one-to-one to the interior (drives its azimuths). A
    // strip bridges the OTHER rim. A strip over a rim that JUMPS in v (the
    // foam body's mess-side rim leaps ~46 between adjacent samples) shears
    // into folded slivers, so the wavier rim must drive (weld to its exact
    // points, no strip over it) and the flatter rim — clean at any count —
    // takes the strip. -1 means "denser drives" (both rims equally flat).
    int driveSide = -1;
    if (rim0ok && rim1ok && nRim0 != nRim1) {
        const GeomAbs_SurfaceType st = surf.GetType();
        const bool analyticRev = st == GeomAbs_Cylinder ||
                                 st == GeomAbs_Cone || st == GeomAbs_Torus;
        if (analyticRev) {
            const double u0i = surf.FirstUParameter();
            double bandH = 0;
            for (int i = 0; i < 12; ++i) {
                double u = u0i + period * i / 12.0;
                bandH += surf.Value(u, v0).Distance(surf.Value(u, v1));
            }
            bandH /= 12.0;
            auto meanChord = [&](const std::vector<RimPt>& R) {
                if (R.size() < 2) return 1e300;
                double s = 0;
                for (size_t i = 0; i < R.size(); ++i) {
                    s += R[i].p.Distance(R[(i + 1) % R.size()].p);
                }
                return s / R.size();
            };
            auto vRange = [&](const std::vector<RimPt>& R) {
                double lo = 1e300, hi = -1e300;
                for (const RimPt& r : R) {
                    lo = std::min(lo, r.v);
                    hi = std::max(hi, r.v);
                }
                return hi - lo;
            };
            // The transition triangle at each extra dense sample rises the
            // band height over half a rim chord (the widest pairing gap);
            // it stays convex while the band is a fair fraction of that.
            const double reach =
                0.5 * std::max(meanChord(rim[0]), meanChord(rim[1]));
            auto maxStep = [&](const std::vector<RimPt>& R) {
                double m = 0;
                for (size_t i = 0; i < R.size(); ++i) {
                    m = std::max(m, std::abs(R[(i + 1) % R.size()].v - R[i].v));
                }
                return m;
            };
            // Pick the drive rim: the wavier one welds one-to-one (a strip
            // over its v-jumps would shear into folds), the flatter one
            // takes the transition strip. When both rims are equally flat,
            // -1 keeps the old "denser drives" so its many samples weld
            // rather than pile into one strip.
            const double vr0 = vRange(rim[0]), vr1 = vRange(rim[1]);
            // Boolean-split analytic rims commonly carry a small pcurve
            // wobble even though they are visually a level ring. Two percent
            // rejected MP9 face 1310 at 3.9% and replaced a clean cylindrical
            // transition with a dense contract web. Five percent still
            // excludes genuinely wavy saddle rims while accepting this mild
            // trim noise.
            const double flatV = std::max(1e-6, 0.05 * bandH);
            if (std::abs(vr0 - vr1) > flatV) driveSide = vr0 > vr1 ? 0 : 1;
            // A strip can only stay convex over a FLAT rim: the drive rim
            // absorbs its wander by welding, but the stripped rim (the
            // flatter of the two) must itself be near-flat, or its own
            // v-wander shears the strip cells into folds. So the reconcile
            // holds only when a genuinely flat rim exists to strip — both
            // rims wavy (nasty_cheese's pipe-saddle bands) is irreconcilable
            // and takes the contract floor, as before. The band must also be
            // tall enough that the transition triangles stay convex (reach).
            const double stripVr = std::min(vr0, vr1);
            stripReconcile = bandH >= 0.35 * reach && stripVr <= flatV;
            const int driveCount =
                driveSide >= 0 ? int(rim[driveSide].size())
                               : std::max(nRim0, nRim1);
            const int stripCount =
                driveSide >= 0 ? int(rim[driveSide ^ 1].size())
                               : std::min(nRim0, nRim1);
            reconBandH = bandH;
            const std::vector<RimPt>& driveRim =
                driveSide >= 0 ? rim[driveSide]
                               : (nRim0 >= nRim1 ? rim[0] : rim[1]);
            reconAzStep = meanChord(driveRim);
            // The interior welds one-to-one to the drive rim, so it inherits
            // the drive rim's v profile. When that rim carries a steep local
            // v-JUMP — the foam body's mess-side rim leaps ~46 (0.4 * band
            // height) between two adjacent samples — the lattice cells across
            // the jump collapse into transition triangles, and a coarse band
            // (a manual radial that samples the rim sparsely, concentrating
            // the whole jump into one step) can't spread them out. A taller
            // interior lattice out-numbers those triangles and keeps quads
            // dominant. A rim that only wanders GRADUALLY (face 4's dense
            // 278-sample blend: max step 0.12 * band height, the default
            // body's 38-sample rim: 0.13) makes no triangles and must NOT
            // trip the bump, so gate on the step, not the total wander.
            const double driveStep = maxStep(driveRim);
            reconFrayStrip = driveStep > 0.25 * bandH;
            dbg("revgrid face %d: RECDIAG bandH=%g reach=%g stripVr=%g "
                "vr0=%g vr1=%g drive=%d(%d) strip=%d driveStep=%g fray=%d",
                faceId, bandH, reach, stripVr, vr0, vr1, driveSide,
                driveCount, stripCount, driveStep, reconFrayStrip ? 1 : 0);
        }
        if (!stripReconcile) {
            dbg("revgrid face %d: rim totals %d/%d irreconcilable", faceId,
                nRim0, nRim1);
            return false;
        }
        dbg("revgrid face %d: rim totals %d/%d -> transition strip", faceId,
            nRim0, nRim1);
        // ANNULUS-BODY route (tight gate): the strip reconcile above must pick
        // a WAVY rim to drive (driveSide >= 0), and that wavy rim is the
        // SPARSE one (drives fewer columns than the flat rim it strips). That
        // is exactly the pathology on the foam body — the interior collapses
        // to the sparse count so `radial` is dead and the flat dense rim rides
        // one full-height diagonal strip. Every other reconciled band drives
        // off the DENSER rim (radial already works) and is untouched here. The
        // wavy/sparse rim must carry a genuine notch (a fair slice of the band
        // height) with room left for a horizontal body. When it matches, run
        // the body the annulus way: dense flat rim drives clean columns,
        // horizontal interior rings, one short distributed-n-gon reduction
        // band at the notched rim.
        {
            const int driveCount =
                driveSide >= 0 ? int(rim[driveSide].size())
                               : std::max(nRim0, nRim1);
            const int stripCount =
                driveSide >= 0 ? int(rim[driveSide ^ 1].size())
                               : std::min(nRim0, nRim1);
            double notchRange = 0;
            if (driveSide >= 0) {
                double lo = 1e300, hi = -1e300;
                for (const RimPt& r : rim[driveSide]) {
                    lo = std::min(lo, r.v);
                    hi = std::max(hi, r.v);
                }
                notchRange = hi - lo;
            }
            const bool annulusBody =
                !vWrap && driveSide >= 0 && driveCount < stripCount &&
                reconBandH > 1e-9 && notchRange >= 0.15 * reconBandH &&
                notchRange <= 0.85 * reconBandH;
            if (annulusBody) {
                std::vector<RevRimPt> denseRim, notchRim;
                denseRim.reserve(rim[driveSide ^ 1].size());
                notchRim.reserve(rim[driveSide].size());
                for (const RimPt& r : rim[driveSide ^ 1])
                    denseRim.push_back({r.u, r.v, r.p});
                for (const RimPt& r : rim[driveSide])
                    notchRim.push_back({r.u, r.v, r.p});
                const int nvBody = std::max(1, nv);
                dbg("revgrid face %d: ANNULUS-BODY nu=%d(dense) notch=%d "
                    "nvBody=%d notchRange=%g bandH=%g",
                    faceId, int(denseRim.size()), int(notchRim.size()), nvBody,
                    notchRange, reconBandH);
                const bool okAB = meshRevolutionAnnulusBody(
                    surf, faceId, denseRim, notchRim, nvBody, flip, out);
                // The body drives its columns off the DENSE rim, not the
                // sparse notch rim the counts table pre-sampled from uEdges[0]
                // — report the count actually built so displayed == actual.
                if (okAB && built) {
                    *built = {int(denseRim.size()), nvBody};
                }
                return okAB;
            }
        }
        // Interior azimuthal count equals the DRIVE rim's: its columns sit
        // at that rim's own azimuths, so it welds to the interior through a
        // clean one-to-one lattice (quads) and only the other rim needs a
        // transition strip. The drive rim is the wavier one (a strip over
        // its v-jumps folds); with both flat, the denser rim drives.
        nu = driveSide >= 0 ? int(rim[driveSide].size())
                            : std::max(nRim0, nRim1);
        // A drive rim with ISO-AZIMUTH runs (a notch bitten into the rim:
        // wall samples stacked at one angle) cannot weld 1:1 — every
        // stacked run becomes a zero-width column and the cells along it
        // fold (foam face 183). Collapse runs to one interior column per
        // DISTINCT azimuth; the drive rim then bridges through the closed
        // strip below, whose angle pairing strings each wall run into one
        // absorber n-gon at its own azimuth. Only engages when runs exist
        // and the rim is not scalloped — every other reconcile keeps the
        // 1:1 weld byte-identically.
        if (driveSide >= 0 && dedupeDriveRim) {
            // FOLD-TRIGGERED REPAIR, never a default route: the caller
            // only sets dedupeDriveRim after the 1:1 weld actually
            // folded (self-heal tournament). No static threshold can
            // separate the two regimes — foam face 183's bite walls
            // (3-7 stacked samples, 40% of the band) fold 1:1, while a
            // countersunk bore's v-steps and a micro pin's spiral rim
            // (1797609in face 62) weld 1:1 cleanly and only get WORSE
            // deduped — so the tournament judges by result instead.
            const std::vector<RimPt>& R = rim[driveSide];
            const double azEps = 1e-6 * period;
            std::vector<double> ded;
            ded.reserve(R.size());
            bool scalloped = false;
            for (size_t i = 0; i < R.size(); ++i) {
                if (ded.empty()) {
                    ded.push_back(R[i].u);
                    continue;
                }
                double d = R[i].u - R[i - 1].u;
                d -= period * std::round(d / period);
                if (d < -1e-7) {
                    scalloped = true;
                    break;
                }
                if (d <= azEps) continue;  // stacked wall sample
                ded.push_back(ded.back() + d);
            }
            if (!scalloped && int(ded.size()) >= 8 &&
                ded.size() < R.size()) {
                reconColU = std::move(ded);
                nu = int(reconColU.size());
                dbg("revgrid face %d: drive rim %zu -> %d distinct "
                    "azimuths (iso-azimuth runs strip-bridged)",
                    faceId, R.size(), nu);
            }
        }
        // The strip design bridges EACH rim to an interior ring; with
        // nv==1 there is no interior and the two mismatched rims would
        // bridge directly, twisting where their samples don't line up.
        // Force at least one interior row.
        int nvFloor = 2;
        // A fraying strip needs the one-to-one lattice to out-number it, so
        // give the tall band enough interior rows: a cell aspect near 2:1
        // (v-height : azimuth-width) from the band height, capped so a
        // genuinely tall body doesn't over-mesh. Only fires when the drive
        // rim carries a steep v-jump (reconFrayStrip), so clean reconciles
        // (the default foam body, every fixture) are untouched.
        if (reconFrayStrip && reconAzStep > 1e-9) {
            int aspectRows =
                int(std::lround(reconBandH / (2.0 * reconAzStep)));
            nvFloor = std::max(nvFloor, std::min(aspectRows, 8));
        }
        if (!vWrap && !vRows && nv < nvFloor) {
            nv = nvFloor;
            dv = (v1 - v0) / nv;
            rows = nv + 1;
        }
    }

    std::vector<std::vector<uint32_t>> ring(rows);
    const bool chained = !vWrap && int(rim[0].size()) == nu &&
                         (rim[1].empty() || int(rim[1].size()) == nu);
    if (chained) {
        // Column u at each rim (missing rim mirrors the other).
        const std::vector<RimPt>& A = rim[0];
        const std::vector<RimPt>& B = rim[1].empty() ? rim[0] : rim[1];
        // When the far rim is missing (cone apex, pole), the loft's far
        // v is the band bound, not a mirror of the near rim's v.
        const bool mirrorB = rim[1].empty();
        double vFarMean = 0;
        for (const RimPt& r : A) vFarMean += r.v;
        vFarMean /= std::max<size_t>(1, A.size());
        const double vFar =
            std::abs(vFarMean - v0) <= std::abs(vFarMean - v1) ? v1 : v0;
        // Rotational alignment of B to A (wrap-shortest total delta).
        int bestOff = 0;
        double bestSum = 1e300;
        for (int off = 0; off < nu; ++off) {
            double sum = 0;
            for (int i = 0; i < nu; ++i) {
                double d = B[(i + off) % nu].u - A[i].u;
                d -= period * std::round(d / period);
                sum += d * d;
            }
            if (sum < bestSum) {
                bestSum = sum;
                bestOff = off;
            }
        }
        // Interior rows ease across rim v-STEPS (a countersunk bore's
        // rim arcs sit at different heights): a row that inherits the
        // raw step per column twists against its neighbour and folds.
        // A short circular moving average spreads the step over a few
        // columns; the rim rows themselves stay exact, so the cells
        // touching a rim absorb what remains of the step.
        std::vector<double> vSmA(nu), vSmB(nu);
        std::vector<double> uSmA(nu), uSmB(nu);
        {
            // Unwrap each rim's u into a monotone sequence first so the
            // window average never mixes branches across the seam.
            std::vector<double> uwA(nu), uwB(nu);
            for (int i = 0; i < nu; ++i) {
                double ua = A[i].u;
                double ub = B[(i + bestOff) % nu].u;
                if (i) {
                    ua -= period * std::round((ua - uwA[i - 1]) / period);
                    ub -= period * std::round((ub - uwB[i - 1]) / period);
                }
                uwA[i] = ua;
                uwB[i] = ub;
            }
            const int win = std::max(1, nu / 12);
            for (int i = 0; i < nu; ++i) {
                double sa = 0, sb = 0, su = 0, sv = 0;
                for (int k = -win; k <= win; ++k) {
                    const int ia = ((i + k) % nu + nu) % nu;
                    sa += A[ia].v;
                    sb += mirrorB ? vFar : B[(ia + bestOff) % nu].v;
                    // Column targets need the unwrapped branch nearest
                    // sample i, not the wrapped raw value.
                    double ua = uwA[ia], ub = uwB[ia];
                    ua -= period * std::round((ua - uwA[i]) / period);
                    ub -= period * std::round((ub - uwB[i]) / period);
                    su += ua;
                    sv += ub;
                }
                vSmA[i] = sa / (2 * win + 1);
                vSmB[i] = sb / (2 * win + 1);
                uSmA[i] = su / (2 * win + 1);
                uSmB[i] = sv / (2 * win + 1);
            }
        }
        for (int j = 0; j < rows; ++j) {
            double v = rowV(j);
            double w = (v - v0) / vspan;
            std::vector<gp_Pnt> pts(nu);
            std::vector<double> us(nu), vs(nu);
            for (int i = 0; i < nu; ++i) {
                // Interior columns interpolate between SMOOTHED rim
                // u's: a rim step stacks several samples at one angle,
                // and raw targets pinch every interior column into that
                // line (the fold fan on countersunk bores). Rim rows
                // keep their exact points below.
                double uA = j == 0 ? A[i].u : uSmA[i];
                double uB = (j == nv && !rim[1].empty())
                                ? B[(i + bestOff) % nu].u
                                : uSmB[i];
                double dU = uB - uA;
                dU -= period * std::round(dU / period);
                us[i] = uA + dU * w;
                // Loft v per column: WAVY rims (pipe-saddle weld
                // curves) carry their own v at each sample, and the
                // interior rows must follow them or the fixed-v rings
                // cross the rims. Flat rims reduce to the old uniform
                // spacing exactly. Explicit vRows (insert bands) keep
                // absolute positions — inserts only plan on flat rims.
                vs[i] = vRows ? v : vSmA[i] + (vSmB[i] - vSmA[i]) * w;
                if (j == 0) pts[i] = A[i].p;  // exact curve points
                else if (j == nv && !rim[1].empty())
                    pts[i] = B[(i + bestOff) % nu].p;
                else pts[i] = surf.Value(us[i], vs[i]);
            }
            bool degenerate = true;
            for (int i = 1; i < nu && degenerate; ++i) {
                degenerate = pts[i].Distance(pts[0]) <= 1e-9;
            }
            if (degenerate && sphereCapPole) {
                // Suppress the pole vertex: its former fan band is skipped and
                // the adjacent real ring is capped by one n-gon below.
                ring[j].clear();
            } else if (degenerate) {
                ring[j].assign(nu,
                               out.addVertex(pts[0], {faceId, us[0], vs[0]}));
            } else {
                ring[j].resize(nu);
                for (int i = 0; i < nu; ++i) {
                    ring[j][i] =
                        out.addVertex(pts[i], {faceId, us[i], vs[i]});
                }
            }
        }
    }
    std::vector<double> ringU[2];  // rim-row u positions (bridge path)
    std::vector<double> colU;      // interior column azimuths (bridge path)
    if (!chained) {
        const double du = period / nu;
        const double u0 = surf.FirstUParameter();
        // When reconciling mismatched rims, the interior rings must follow
        // each rim's v(u) profile so the columns stay straight rulings and
        // no constant-v ring crosses a WAVY rim (saddle cuts) or a
        // scalloped rim's teeth — a crossing folds the transition strip.
        // Each rim's samples give v as a function of azimuth; the interior
        // column at azimuth u_i lofts v between the two profiles.
        std::vector<std::pair<double, double>> prof[2];  // (wrapped u, v)
        if (stripReconcile) {
            for (int k = 0; k < 2; ++k) {
                prof[k].reserve(rim[k].size());
                for (const RimPt& r : rim[k]) {
                    double uw =
                        r.u - period * std::floor((r.u - u0) / period);
                    prof[k].push_back({uw, r.v});
                }
                std::sort(prof[k].begin(), prof[k].end());
            }
        }
        auto vAtU = [&](int k, double u) -> double {
            const auto& P = prof[k];
            if (P.empty()) return v0;
            if (P.size() == 1) return P[0].second;
            double uu = u - period * std::floor((u - u0) / period);
            // First profile sample with u >= uu; the pair straddling uu is
            // (lo, hi), wrapping the ends across the seam.
            size_t hi = 0;
            while (hi < P.size() && P[hi].first < uu) ++hi;
            double u1, v1v, u0v, vv0;
            if (hi == 0) {
                u0v = P.back().first - period;
                vv0 = P.back().second;
                u1 = P.front().first;
                v1v = P.front().second;
            } else if (hi == P.size()) {
                u0v = P.back().first;
                vv0 = P.back().second;
                u1 = P.front().first + period;
                v1v = P.front().second;
            } else {
                u0v = P[hi - 1].first;
                vv0 = P[hi - 1].second;
                u1 = P[hi].first;
                v1v = P[hi].second;
            }
            double span = u1 - u0v;
            double t = span > 1e-12 ? (uu - u0v) / span : 0.0;
            return vv0 + (v1v - vv0) * std::clamp(t, 0.0, 1.0);
        };
        // Interior column azimuths. Reconciled bands place them at the
        // DRIVE rim's own samples (nu == that rim's count), so the interior
        // ring next to that rim pairs it one-to-one by index — a clean
        // lattice with no twist — while the other rim takes the strip. The
        // drive rim is the wavier one (driveSide) or, both flat, the denser
        // one. Other non-chained bands keep the uniform ruling positions.
        int denseSide = stripReconcile
                            ? (driveSide >= 0 ? driveSide
                                              : (nRim1 >= nRim0 ? 1 : 0))
                            : -1;
        // A SCALLOPED dense rim (radial step edges) backsteps in azimuth;
        // pinning the interior to its samples then folds the aligned lattice
        // at every tooth, and such a rim is already near-uniform in azimuth,
        // so fall back to uniform columns and let its (evenly spaced) samples
        // pair the uniform interior directly. A CLUSTERED but monotone dense
        // rim (a saddle cut) instead needs its own azimuths, or the uniform
        // lattice twists.
        if (denseSide >= 0) {
            const auto& R = rim[denseSide];
            for (size_t i = 0; i + 1 < R.size(); ++i) {
                double d = R[i + 1].u - R[i].u;
                d -= period * std::round(d / period);
                if (d < -1e-7) { denseSide = -2; break; }  // scalloped
            }
        }
        colU.resize(nu);
        if (!reconColU.empty() && int(reconColU.size()) == nu) {
            // Deduped drive-rim azimuths (iso-azimuth runs collapsed);
            // already unwrapped monotone from the rim's first sample.
            colU = reconColU;
        } else if (denseSide >= 0 && int(rim[denseSide].size()) == nu) {
            // Dense-rim azimuths, unwrapped monotone from its first sample.
            const auto& R = rim[denseSide];
            colU[0] = R[0].u;
            for (int i = 1; i < nu; ++i) {
                double d = R[i].u - R[i - 1].u;
                d -= period * std::round(d / period);
                if (d < 0) d = 0;
                colU[i] = colU[i - 1] + d;
            }
        } else {
            for (int i = 0; i < nu; ++i) colU[i] = u0 + i * du;
        }
        for (int j = 0; j < rows; ++j) {
            double v = rowV(j);
            // Rim rows with usable samples are the EXACT rim points; the
            // strips below stitch them to the interior.
            const int side = (j == 0 && rim0ok)          ? 0
                             : (j == rows - 1 && rim1ok) ? 1
                                                         : -1;
            if (side >= 0) {
                const auto& R = rim[side];
                ring[j].resize(R.size());
                for (size_t i = 0; i < R.size(); ++i) {
                    ring[j][i] =
                        out.addVertex(R[i].p, {faceId, R[i].u, v});
                    ringU[side].push_back(R[i].u);
                }
                continue;
            }
            const double w =
                (stripReconcile && rows > 1) ? double(j) / (rows - 1) : 0.0;
            std::vector<gp_Pnt> pts(nu);
            std::vector<double> vcol(nu, v);
            bool degenerate = true;
            for (int i = 0; i < nu; ++i) {
                const double ui = colU[i];
                if (stripReconcile) {
                    vcol[i] = vAtU(0, ui) * (1 - w) + vAtU(1, ui) * w;
                }
                pts[i] = surf.Value(ui, vcol[i]);
                if (i > 0 && pts[i].Distance(pts[0]) > 1e-9) {
                    degenerate = false;
                }
            }
            if (degenerate && sphereCapPole) {
                // Suppress the pole vertex: its former fan band is skipped and
                // the adjacent real ring is capped by one n-gon below.
                ring[j].clear();
            } else if (degenerate) {
                ring[j].assign(nu, out.addVertex(pts[0], {faceId, colU[0], v}));
            } else {
                ring[j].resize(nu);
                for (int i = 0; i < nu; ++i) {
                    ring[j][i] = out.addVertex(
                        pts[i], {faceId, colU[i], vcol[i]});
                }
            }
        }
    }

    for (int j = 0; j < nv; ++j) {
        const std::vector<uint32_t>& lo = ring[j];
        const std::vector<uint32_t>& hi = ring[(j + 1) % rows];
        if (lo.size() != hi.size()) continue;  // strip-bridged pair
        for (int i = 0; i < nu; ++i) {
            int i2 = (i + 1) % nu;
            std::vector<uint32_t> quad{lo[i], lo[i2], hi[i2], hi[i]};
            // Collapse repeats now so degenerate rows emit clean triangles.
            quad.erase(std::unique(quad.begin(), quad.end()), quad.end());
            if (quad.size() > 1 && quad.front() == quad.back()) quad.pop_back();
            if (quad.size() < 3) continue;
            out.addPolygon(std::move(quad), faceId, flip);
        }
    }

    // Close each suppressed pole (single-rim sphere dome) with one n-gon over
    // the innermost real ring, wound to match the lattice cells its fan band
    // replaced: a low pole (row 0) fanned the ring in reverse, a high pole
    // forward. The cleared pole ring left its band unmeshed above, so this is
    // the only polygon spanning it — watertight, no pole vertex, one clean cap.
    if (sphereCapPole) {
        for (int j = 0; j < rows; ++j) {
            if (!ring[j].empty()) continue;  // not a suppressed pole
            const int jn = (j == 0) ? 1 : rows - 2;
            if (jn < 0 || jn >= rows || int(ring[jn].size()) < 3) continue;
            std::vector<uint32_t> cap = ring[jn];
            if (j == 0) std::reverse(cap.begin(), cap.end());
            cap.erase(std::unique(cap.begin(), cap.end()), cap.end());
            if (cap.size() > 1 && cap.front() == cap.back()) cap.pop_back();
            if (cap.size() < 3) continue;
            out.addPolygon(std::move(cap), faceId, flip);
        }
    }

    // Closed transition strips between an exact rim row and its neighbouring
    // ring when their counts differ — the shared emitClosedRimStrip: quads
    // where the counts advance together, a grouped n-gon where the dense ring
    // contributes extra points, distributed around the ring. Winding matches
    // the lattice cells (lower row forward, upper row backward).
    if (!chained) {
        const double du = period / nu;
        const double u0 = surf.FirstUParameter();
        // Interior column azimuths: the dense-rim positions when reconciling
        // (colU was filled during ring construction), else uniform rulings.
        if (colU.empty()) {
            colU.resize(nu);
            for (int i = 0; i < nu; ++i) colU[i] = u0 + i * du;
        }
        if (rim0ok && rim1ok && rows == 2) {
            // No interior ring at all: bridge rim to rim directly.
            emitClosedRimStrip(out, faceId, flip, period, ring[0], ringU[0],
                               ring[1], ringU[1]);
        } else {
            // Only a rim whose count differs from the interior takes a
            // strip; a reconciled band's denser rim equals the interior and
            // welds through the aligned lattice above.
            if (rim0ok && ring[0].size() != ring[1].size()) {
                emitClosedRimStrip(out, faceId, flip, period, ring[0], ringU[0],
                                   ring[1], colU);
            }
            if (rim1ok && ring[rows - 1].size() != ring[rows - 2].size()) {
                emitClosedRimStrip(out, faceId, flip, period, ring[rows - 2],
                                   colU, ring[rows - 1], ringU[1]);
            }
        }
    }
    // The interior column count `nu` may have been raised to the drive rim's
    // sample count (strip reconcile) and the row count `nv` floored — report
    // what was actually built, which equals the passed request on the common
    // matched/single-rim path.
    if (built) *built = {nu, nv};
    return true;
}

// A full revolution band with interior slot/hole wires: mesh the plain
// grid, remove the cells the wires cover, and web the staircase to the
// wires' exact border sampling (3D edge curves at solved counts — the
// same contract the slot's wall faces sample, so the weld closes it).
bool meshRevolutionInsert(const TopoDS_Face& face,
                          const BRepAdaptor_Surface& surf, const Model& model,
                          const FacePlan& plan,
                          const std::vector<int>& solvedEdge, int faceId,
                          int nu, int nv, MeshBuilder& out,
                          const PinnedEdges* pins,
                          bool decoupleSeams) {
    // Row alignment (rows exactly at each band's v-extents) is what
    // makes the staircase close; a v-closed surface ignores explicit
    // rows, so refuse and let the face take the contract floor.
    if (surf.IsVClosed()) return false;
    // A wire whose solved counts can't even form a triangle would leave
    // its hole open; refuse up front and let the face fall back whole.
    for (const auto& wire : plan.insertWires) {
        int total = 0;
        for (int eid : wire) {
            total += eid > 0 && eid < (int)solvedEdge.size() &&
                             solvedEdge[eid] > 0
                         ? solvedEdge[eid]
                         : 8;
        }
        if (total < 3) return false;
    }
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double du = (u1 - u0) / std::max(3, nu);
    const double vspan = std::max(1e-12, v1 - v0);

    // UV bbox per wire. u grows by most of a cell so sliver cells go
    // too (u wraps, columns always exist on both sides); v stays EXACT —
    // the grid below places rows precisely at these extents, so deleted
    // cells are strictly interior and the staircase closes by
    // construction (a full-height deletion used to clip the rims and
    // leave the whole slot unwebbed, silently).
    struct Box { double u0, u1, v0, v1; };
    std::vector<Box> boxes;
    for (const auto& wire : plan.insertWires) {
        Box b{1e300, -1e300, 1e300, -1e300};
        for (int eid : wire) {
            double f, l;
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f, l);
            if (pc.IsNull()) continue;
            for (int k = 0; k <= 16; ++k) {
                gp_Pnt2d uv = pc->Value(f + (l - f) * k / 16.0);
                b.u0 = std::min(b.u0, uv.X());
                b.u1 = std::max(b.u1, uv.X());
                b.v0 = std::min(b.v0, uv.Y());
                b.v1 = std::max(b.v1, uv.Y());
            }
            // The web below samples this pcurve at the SOLVED count with
            // the edge's phase — different positions than the uniform
            // sweep above. The box's v-extents become grid rows, so they
            // must bound THOSE samples: a polyline extremum past the row
            // pokes the hole ring through the staircase, the keyhole
            // ring self-intersects, and the ear-clip dead-ends into a
            // folded fan (slotted tube: 58/181 inverted web cells).
            const int wn = eid > 0 && eid < (int)solvedEdge.size() &&
                                   solvedEdge[eid] > 0
                               ? solvedEdge[eid]
                               : 8;
            const double ph = closedEdgePhase(edge, model);
            for (int k = 0; k <= wn; ++k) {
                const double t = phasedT(k, wn, ph, false);
                gp_Pnt2d uv = pc->Value(f + (l - f) * t);
                b.u0 = std::min(b.u0, uv.X());
                b.u1 = std::max(b.u1, uv.X());
                b.v0 = std::min(b.v0, uv.Y());
                b.v1 = std::max(b.v1, uv.Y());
            }
        }
        if (b.u0 > b.u1) return false;  // no usable pcurves on this wire
        b.u0 -= 0.6 * du; b.u1 += 0.6 * du;
        // Rows sit at the box's v-extents; the ring's own extreme verts
        // would then lie ON the staircase (tangent webs ear-clip into
        // folded fans). Push the rows just past the ring so the web is
        // a strict annulus.
        const double vPad = 0.04 * std::max(1e-12, b.v1 - b.v0);
        b.v0 -= vPad;
        b.v1 += vPad;
        boxes.push_back(b);
    }
    // Row layout: rims plus every band extent. A wire too close to a rim
    // can't be banded — plan-time margins should have excluded it.
    std::vector<double> vRows{v0, v1};
    for (const Box& b : boxes) {
        if (b.v0 <= v0 + 0.01 * vspan || b.v1 >= v1 - 0.01 * vspan) {
            return false;
        }
        vRows.push_back(b.v0);
        vRows.push_back(b.v1);
    }
    std::sort(vRows.begin(), vRows.end());
    vRows.erase(std::unique(vRows.begin(), vRows.end(),
                            [&](double a, double c) {
                                return c - a < 1e-7 * vspan;
                            }),
                vRows.end());
    if (vRows.size() < 3 ||
        std::abs(vRows.back() - v1) > 1e-7 * vspan) {
        return false;
    }

    PolyMesh grid;
    {
        MeshBuilder tmp(grid);
        bool built;
        if (plan.castellated) {
            // Notched rim AND interior slots on one wall (torture's
            // muzzle): the base lattice comes from the pinned
            // boolean-cut notch mesher, subdivided at the slot rows,
            // and the carve below webs the slots exactly as on a
            // plain-rim wall. The interior levels exclude the rims
            // (vRows carries v0/v1 too).
            std::vector<double> levels(vRows.begin() + 1, vRows.end() - 1);
            built = meshRevolutionRimNotch(face, surf, model, plan.rimLow,
                                           plan.rimHigh, plan.plainRimEdge,
                                           solvedEdge, faceId, nu, nv, tmp,
                                           pins, &levels);
        } else {
            built = meshRevolutionGrid(
                face, surf, model, plan.uEdges, solvedEdge, faceId, nu,
                int(vRows.size()) - 1, tmp, &vRows,
                plan.rimLow.empty() ? nullptr : &plan.rimLow, nullptr,
                false, decoupleSeams);
        }
        if (!built) return false;
    }

    const double period = u1 - u0;
    auto covered = [&](const std::vector<uint32_t>& poly) {
        double cu = 0, cv = 0, u0ref = 0; int n = 0;
        for (uint32_t idx : poly) {
            const Anchor& a = grid.anchors[idx];
            if (a.faceId != faceId) return false;
            double u = a.u;
            if (!n) {
                u0ref = u;
            } else {
                // Seam cells mix u0 and u0+period anchors; unwrap
                // against the first corner or the center lands
                // mid-period and the wrong cells are deleted.
                u -= period * std::round((u - u0ref) / period);
            }
            cu += u; cv += a.v; ++n;
        }
        if (!n) return false;
        cu /= n; cv /= n;
        cu -= period * std::floor((cu - u0) / period);
        for (const Box& b : boxes) {
            if (cu >= b.u0 && cu <= b.u1 && cv >= b.v0 && cv <= b.v1) {
                return true;
            }
        }
        return false;
    };

    // Directed boundary edges before/after deletion; the difference is the
    // staircase around the removed region.
    auto directedBoundary = [](const PolyMesh& m, const std::vector<char>& keep) {
        std::map<std::pair<uint32_t, uint32_t>, int> use;
        for (size_t p = 0; p < m.polygons.size(); ++p) {
            if (!keep[p]) continue;
            const auto& poly = m.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                ++use[{std::min(a, b), std::max(a, b)}];
            }
        }
        std::map<uint32_t, uint32_t> next;  // directed open edges a->b
        for (size_t p = 0; p < m.polygons.size(); ++p) {
            if (!keep[p]) continue;
            const auto& poly = m.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                if (use[{std::min(a, b), std::max(a, b)}] == 1) next[a] = b;
            }
        }
        return next;
    };
    std::vector<char> all(grid.polygons.size(), 1);
    std::vector<char> keep(grid.polygons.size(), 1);
    bool any = false;
    for (size_t p = 0; p < grid.polygons.size(); ++p) {
        if (covered(grid.polygons[p])) { keep[p] = 0; any = true; }
    }
    // Wires present but nothing deleted: the intact grid would cover the
    // holes and every wall border would dangle. Refuse visibly.
    if (!any) {
        dbg("insert face %d: no covered cells", faceId);
        return false;
    }
    auto before = directedBoundary(grid, all);
    auto after = directedBoundary(grid, keep);

    // New staircase loops = directed open edges present now, absent
    // before. Extract and VALIDATE them BEFORE emitting anything: an
    // unclosed chain means the deletion clipped the outer boundary and
    // the hole could never be webbed — fail the whole face while it is
    // still un-emitted, so the planner's fallback stays contract-clean.
    std::map<uint32_t, uint32_t> stair;
    for (const auto& [a, b] : after) {
        auto it = before.find(a);
        if (it == before.end() || it->second != b) stair[a] = b;
    }
    std::vector<std::vector<uint32_t>> loops;
    while (!stair.empty()) {
        std::vector<uint32_t> loop;
        uint32_t start = stair.begin()->first, cur = start;
        bool closedLoop = false;
        while (loop.size() <= grid.vertices.size()) {
            auto it = stair.find(cur);
            if (it == stair.end()) break;
            loop.push_back(cur);
            cur = it->second;
            stair.erase(it);
            if (cur == start) { closedLoop = true; break; }
        }
        if (!closedLoop || loop.size() < 3) {
            dbg("insert face %d: staircase loop open (size %zu)", faceId,
                loop.size());
            return false;
        }
        loops.push_back(std::move(loop));
    }
    if (loops.empty()) {
        dbg("insert face %d: no staircase loops", faceId);
        return false;
    }
    // Every wire must belong to a loop or its hole stays open.
    std::vector<std::vector<size_t>> loopWires(loops.size());
    {
        std::vector<char> assigned(boxes.size(), 0);
        for (size_t li = 0; li < loops.size(); ++li) {
            double lu0 = 1e300, lu1 = -1e300, lv0 = 1e300, lv1 = -1e300;
            for (uint32_t idx : loops[li]) {
                lu0 = std::min(lu0, grid.anchors[idx].u);
                lu1 = std::max(lu1, grid.anchors[idx].u);
                lv0 = std::min(lv0, grid.anchors[idx].v);
                lv1 = std::max(lv1, grid.anchors[idx].v);
            }
            for (size_t w = 0; w < boxes.size(); ++w) {
                if (assigned[w]) continue;
                double cu = (boxes[w].u0 + boxes[w].u1) / 2;
                double cv = (boxes[w].v0 + boxes[w].v1) / 2;
                if (cu >= lu0 && cu <= lu1 && cv >= lv0 && cv <= lv1) {
                    loopWires[li].push_back(w);
                    assigned[w] = 1;
                }
            }
        }
        for (char a : assigned) {
            if (!a) return false;
        }
        // And every loop needs at least one wire, or it has no lid.
        for (const auto& lw : loopWires) {
            if (lw.empty()) return false;
        }
    }

    // Topology validated. Build the whole result LOCALLY first — the
    // webs can still fail (ear-clip on a degenerate keyhole ring), and
    // a partially emitted face is a guaranteed leak. `out` receives the
    // part only after every web proved complete.
    PolyMesh webbedMesh;
    MeshBuilder wb(webbedMesh);
    std::vector<uint32_t> remap(grid.vertices.size(), UINT32_MAX);
    auto emitVert = [&](uint32_t i) {
        if (remap[i] == UINT32_MAX) {
            remap[i] = wb.addVertex(gp_Pnt(grid.vertices[i][0],
                                           grid.vertices[i][1],
                                           grid.vertices[i][2]),
                                    grid.anchors[i]);
        }
        return remap[i];
    };
    for (size_t p = 0; p < grid.polygons.size(); ++p) {
        if (!keep[p]) continue;
        std::vector<uint32_t> poly;
        poly.reserve(grid.polygons[p].size());
        for (uint32_t idx : grid.polygons[p]) poly.push_back(emitVert(idx));
        wb.addPolygon(std::move(poly), faceId, false);
    }

    const double rScale =
        std::max(1e-6, surf.Value((u0 + u1) / 2, (v0 + v1) / 2)
                           .Distance(surf.Value((u0 + u1) / 2 + 1e-3,
                                                (v0 + v1) / 2)) /
                           1e-3);

    // One web per staircase loop, splicing in every wire assigned to it
    // (nearby slots can merge into one staircase): keyhole ear-clip.
    for (size_t li = 0; li < loops.size(); ++li) {
        const std::vector<uint32_t>& loop = loops[li];
        const std::vector<size_t>& inLoop = loopWires[li];

        // Working ring: reversed staircase (it bounds the remaining mesh)
        // in synthetic planar coords + output vertex ids.
        std::vector<std::array<double, 3>> ringPts;
        std::vector<uint32_t> ringIds;
        {
            std::vector<uint32_t> outer(loop.rbegin(), loop.rend());
            for (uint32_t idx : outer) {
                ringPts.push_back({grid.anchors[idx].u * rScale,
                                   grid.anchors[idx].v, 0.0});
                ringIds.push_back(emitVert(idx));
            }
        }
        auto ringArea = [&]() {
            double a2 = 0;
            for (size_t i = 0; i < ringPts.size(); ++i) {
                const auto& p1 = ringPts[i];
                const auto& p2 = ringPts[(i + 1) % ringPts.size()];
                a2 += p1[0] * p2[1] - p2[0] * p1[1];
            }
            return a2;
        };
        const double outerSign = ringArea();

        for (size_t w : inLoop) {
            // Wire polyline: each edge sampled on its 3D curve at the
            // solved count (the same contract its wall faces sample), uv
            // through the pcurve; pieces chained by nearest endpoints.
            struct WPt { gp_Pnt p; double u, v; };
            std::vector<std::vector<WPt>> pieces;
            for (int eid : plan.insertWires[w]) {
                int n = eid > 0 && eid < (int)solvedEdge.size() &&
                                solvedEdge[eid] > 0
                            ? solvedEdge[eid]
                            : 8;
                const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
                double f, l;
                Handle(Geom2d_Curve) pc =
                    BRep_Tool::CurveOnSurface(edge, face, f, l);
                if (pc.IsNull()) continue;
                BRepAdaptor_Curve c(edge);
                const double f3 = c.FirstParameter(), l3 = c.LastParameter();
                const double ph = closedEdgePhase(edge, model);
                std::vector<WPt> piece;
                for (double t : edgeSampleFractions(
                         eid, n, ph, false, /*includeLast=*/true, nullptr,
                         &model)) {
                    gp_Pnt2d uv = pc->Value(f + t * (l - f));
                    piece.push_back(
                        {c.Value(f3 + t * (l3 - f3)), uv.X(), uv.Y()});
                }
                pieces.push_back(std::move(piece));
            }
            if (pieces.empty()) continue;
            std::vector<WPt> hole = pieces[0];
            std::vector<char> used(pieces.size(), 0);
            used[0] = 1;
            for (size_t step = 1; step < pieces.size(); ++step) {
                double bd = 1e300; size_t bi = 0; bool rev = false;
                for (size_t k = 0; k < pieces.size(); ++k) {
                    if (used[k]) continue;
                    double dF = hole.back().p.Distance(pieces[k].front().p);
                    double dB = hole.back().p.Distance(pieces[k].back().p);
                    if (dF < bd) { bd = dF; bi = k; rev = false; }
                    if (dB < bd) { bd = dB; bi = k; rev = true; }
                }
                used[bi] = 1;
                std::vector<WPt> pc2 = pieces[bi];
                if (rev) std::reverse(pc2.begin(), pc2.end());
                hole.insert(hole.end(), pc2.begin() + 1, pc2.end());
            }
            if (hole.size() > 1 &&
                hole.front().p.Distance(hole.back().p) < 1e-9) {
                hole.pop_back();
            }
            if (hole.size() < 3) continue;

            double aHole = 0;
            for (size_t i = 0; i < hole.size(); ++i) {
                const WPt& p1 = hole[i];
                const WPt& p2 = hole[(i + 1) % hole.size()];
                aHole += p1.u * rScale * p2.v - p2.u * rScale * p1.v;
            }
            std::vector<WPt> h = hole;
            if (outerSign * aHole > 0) std::reverse(h.begin(), h.end());

            // Splice this hole into the working ring at the nearest pair.
            size_t bo = 0, bh = 0; double bd = 1e300;
            for (size_t i = 0; i < ringPts.size(); ++i) {
                for (size_t j = 0; j < h.size(); ++j) {
                    double dx = ringPts[i][0] - h[j].u * rScale;
                    double dy = ringPts[i][1] - h[j].v;
                    double d = dx * dx + dy * dy;
                    if (d < bd) { bd = d; bo = i; bh = j; }
                }
            }
            std::vector<std::array<double, 3>> np;
            std::vector<uint32_t> ni;
            for (size_t i = 0; i <= bo; ++i) {
                np.push_back(ringPts[i]);
                ni.push_back(ringIds[i]);
            }
            std::vector<uint32_t> holeIds(h.size(), UINT32_MAX);
            auto holeId = [&](size_t j) {
                if (holeIds[j] == UINT32_MAX) {
                    holeIds[j] = wb.addVertex(
                        h[j].p, Anchor{faceId, h[j].u, h[j].v});
                }
                return holeIds[j];
            };
            for (size_t j = 0; j <= h.size(); ++j) {
                size_t k = (bh + j) % h.size();
                np.push_back({h[k].u * rScale, h[k].v, 0.0});
                ni.push_back(holeId(k));
            }
            for (size_t i = bo; i < ringPts.size(); ++i) {
                np.push_back(ringPts[i]);
                ni.push_back(ringIds[i]);
            }
            ringPts = std::move(np);
            ringIds = std::move(ni);
        }

        std::vector<uint32_t> ringIdx(ringPts.size());
        for (size_t i = 0; i < ringIdx.size(); ++i) ringIdx[i] = i;
        size_t emitted = 0;
        for (const auto& t : triangulatePoly(ringPts, ringIdx)) {
            uint32_t a = ringIds[t[0]], b = ringIds[t[1]],
                     c = ringIds[t[2]];
            if (a == b || b == c || a == c) continue;
            wb.addPolygon({a, b, c}, faceId, false);
            ++emitted;
        }
        // A complete ear-clip of a keyhole ring yields exactly V-2
        // triangles (bridge duplicates included). Anything less means
        // the web has an internal hole — fail the face un-emitted.
        if (emitted + 2 < ringPts.size()) return false;
    }

    // Every web complete: splat the local result into the real builder.
    std::vector<uint32_t> outMap(webbedMesh.vertices.size());
    for (uint32_t i = 0; i < webbedMesh.vertices.size(); ++i) {
        outMap[i] = out.addVertex(gp_Pnt(webbedMesh.vertices[i][0],
                                         webbedMesh.vertices[i][1],
                                         webbedMesh.vertices[i][2]),
                                  webbedMesh.anchors[i]);
    }
    for (const auto& poly : webbedMesh.polygons) {
        std::vector<uint32_t> mapped;
        mapped.reserve(poly.size());
        for (uint32_t idx : poly) mapped.push_back(outMap[idx]);
        out.addPolygon(std::move(mapped), faceId, false);
    }
    return true;
}


}  // namespace weft::mesher_impl
