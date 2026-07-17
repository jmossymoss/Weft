#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// ---------------------------------------------------------------------------
// Trim corridor: a conservative, separate answer for a single concave trim
// which is topologically just two long rails plus two local end closures.
//
// This is intentionally *not* another permissive Coons fallback.  The four
// Coons sides may legitimately cross on these trims even though the usable
// topology is obvious (the MP9 optic/grip strips were the motivating class).
// We therefore prove the narrower statement we actually need: two complete
// boundary chains advance monotonically along one UV axis, arc-length paired
// rungs stay inside the face, and every prospective body cell keeps one UV
// orientation.  Only then may a direct rail strip own the face.  Border count
// disagreement is never fanned across the face: the density solve aligns the
// opposite rail-chain totals, while the two remaining chains close locally.

struct TrimCorridorPatch {
    bool ok = false;
    bool axisU = true;
    std::vector<int> railA;    // sampled-ring indices, low -> high
    std::vector<int> railB;    // sampled-ring indices, low -> high
    std::vector<int> capLow;   // exact boundary chain between rail starts
    std::vector<int> capHigh;  // exact boundary chain between rail ends
    std::vector<int> railAEdges;
    std::vector<int> railBEdges;
    std::vector<int> capLowEdges;
    std::vector<int> capHighEdges;
    double score = -1e300;
};

std::vector<int> corridorRingArc(int n, int from, int to, int step = 1) {
    std::vector<int> out;
    if (n < 1 || from < 0 || from >= n || to < 0 || to >= n) return out;
    int k = from;
    for (int guard = 0; guard <= n; ++guard) {
        out.push_back(k);
        if (k == to) return out;
        k = (k + step + n) % n;
    }
    return {};
}

std::vector<int> corridorArcEdges(const std::vector<int>& arc,
                                  const std::vector<int>& sampleEdges,
                                  int n) {
    std::vector<int> edges;
    if (arc.size() < 2 || int(sampleEdges.size()) != n) return edges;
    for (size_t i = 0; i + 1 < arc.size(); ++i) {
        const int a = arc[i], b = arc[i + 1];
        int eid = 0;
        if ((a + 1) % n == b) {
            eid = sampleEdges[a];
        } else if ((b + 1) % n == a) {
            eid = sampleEdges[b];
        } else {
            return {};
        }
        if (eid < 1) return {};
        if (edges.empty() || edges.back() != eid) edges.push_back(eid);
    }
    return edges;
}

bool sameCorridorEdgeSet(const std::vector<int>& a,
                         const std::vector<int>& b) {
    if (a.size() != b.size()) return false;
    std::multiset<int> aa(a.begin(), a.end()), bb(b.begin(), b.end());
    return aa == bb;
}

struct CorridorSample {
    gp_Pnt2d uv;
    gp_Pnt p;
};

CorridorSample sampleCorridorArc(const std::vector<int>& rail, double t,
                                 const std::vector<gp_Pnt>& p,
                                 const std::vector<gp_Pnt2d>& uv) {
    if (rail.empty()) return {};
    if (rail.size() == 1 || t <= 0) return {uv[rail.front()], p[rail.front()]};
    if (t >= 1) return {uv[rail.back()], p[rail.back()]};
    std::vector<double> d(rail.size(), 0.0);
    for (size_t i = 1; i < rail.size(); ++i) {
        d[i] = d[i - 1] + p[rail[i - 1]].Distance(p[rail[i]]);
    }
    const double total = d.back();
    if (total <= 1e-14) return {uv[rail.front()], p[rail.front()]};
    const double want = total * t;
    size_t i = 0;
    while (i + 1 < d.size() && d[i + 1] < want) ++i;
    if (i + 1 >= d.size()) return {uv[rail.back()], p[rail.back()]};
    const double span = d[i + 1] - d[i];
    const double f = span > 1e-14 ? (want - d[i]) / span : 0.0;
    const gp_Pnt2d a = uv[rail[i]], b = uv[rail[i + 1]];
    return {gp_Pnt2d(a.X() * (1.0 - f) + b.X() * f,
                     a.Y() * (1.0 - f) + b.Y() * f),
            gp_Pnt(p[rail[i]].XYZ() * (1.0 - f) +
                   p[rail[i + 1]].XYZ() * f)};
}

// Find the strongest proven corridor.  `axisHint` is -1 at planning time and
// 0/1 at build time.  The optional expected rails make the build reproduce
// the exact edge-chain decision made by planning even if solved sampling
// changes the number of points on individual edges.
bool findTrimCorridor(const TopoDS_Face& face,
                      const std::vector<gp_Pnt>& p,
                      const std::vector<gp_Pnt2d>& uv,
                      const std::vector<int>& corners,
                      const std::vector<int>& sampleEdges, int axisHint,
                      const std::array<std::vector<int>, 2>* expectedRails,
                      TrimCorridorPatch& best) {
    best = {};
    const int n = int(p.size());
    const int nc = int(corners.size());
    if (n < 8 || nc < 6 || nc > 20 || int(uv.size()) != n ||
        int(sampleEdges.size()) != n) {
        return false;
    }
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const gp_Pnt2d& q : uv) {
        u0 = std::min(u0, q.X()); u1 = std::max(u1, q.X());
        v0 = std::min(v0, q.Y()); v1 = std::max(v1, q.Y());
    }
    if (u1 - u0 <= 1e-12 || v1 - v0 <= 1e-12) return false;

    struct Meta {
        int i = 0, j = 0, k = 0, l = 0, axis = 0;
        bool reverse = false;
        double score = -1e300;
    };
    std::vector<Meta> candidates;
    auto coord = [&](int idx, int axis) {
        return axis == 0 ? uv[idx].X() : uv[idx].Y();
    };
    auto arcLength = [&](const std::vector<int>& a) {
        double d = 0.0;
        for (size_t x = 1; x < a.size(); ++x) {
            d += p[a[x - 1]].Distance(p[a[x]]);
        }
        return d;
    };
    auto monotone = [&](const std::vector<int>& a, int axis, double range) {
        if (a.size() < 3) return false;
        double furthest = coord(a.front(), axis);
        double back = 0.0;
        for (size_t x = 1; x < a.size(); ++x) {
            const double q = coord(a[x], axis);
            if (q < furthest) back += furthest - q;
            if (q < furthest - 0.035 * range) return false;
            furthest = std::max(furthest, q);
        }
        return back <= 0.10 * range;
    };
    auto primaryRange = [&](const std::vector<int>& a, int axis) {
        double lo = 1e300, hi = -1e300;
        for (int x : a) {
            lo = std::min(lo, coord(x, axis));
            hi = std::max(hi, coord(x, axis));
        }
        return hi - lo;
    };
    auto evaluateCheap = [&](int i, int j, int k, int l, int axis) {
        const double loAll = axis == 0 ? u0 : v0;
        const double hiAll = axis == 0 ? u1 : v1;
        const double range = hiAll - loAll;
        // Reject the overwhelming majority of four-cut combinations before
        // constructing paths: rail starts must live near one axis extreme and
        // rail ends near the other (or the exact reverse).
        const double di = coord(i, axis), dj = coord(j, axis);
        const double dk = coord(k, axis), dl = coord(l, axis);
        const bool increasing = std::max(di, dl) <= loAll + 0.24 * range &&
                                std::min(dj, dk) >= hiAll - 0.24 * range;
        const bool decreasing = std::min(di, dl) >= hiAll - 0.24 * range &&
                                std::max(dj, dk) <= loAll + 0.24 * range;
        if (!increasing && !decreasing) return;

        std::vector<int> rawA = corridorRingArc(n, i, j, +1);
        std::vector<int> rawB = corridorRingArc(n, k, l, +1);
        std::vector<int> capJK = corridorRingArc(n, j, k, +1);
        std::vector<int> capLI = corridorRingArc(n, l, i, +1);
        if (rawA.size() < 3 || rawB.size() < 3 || capJK.size() < 2 ||
            capLI.size() < 2) {
            return;
        }
        std::vector<int> a = rawA;
        std::vector<int> b(rawB.rbegin(), rawB.rend());
        std::vector<int> low = capLI, high = capJK;
        if (decreasing) {
            std::reverse(a.begin(), a.end());
            std::reverse(b.begin(), b.end());
            low = capJK;
            high = capLI;
        }
        if (!monotone(a, axis, range) || !monotone(b, axis, range)) return;
        if (primaryRange(a, axis) < 0.70 * range ||
            primaryRange(b, axis) < 0.70 * range) {
            return;
        }
        // End closures stay local in the primary direction.  They may be
        // broad across the corridor (a flat grip end), but may not become a
        // hidden third rail.
        if (primaryRange(low, axis) > 0.32 * range ||
            primaryRange(high, axis) > 0.32 * range) {
            return;
        }
        const double la = arcLength(a), lb = arcLength(b);
        const double lc0 = arcLength(low), lc1 = arcLength(high);
        if (std::min(la, lb) <= 1e-9 ||
            std::max(la, lb) > 3.2 * std::min(la, lb)) {
            return;
        }
        if (std::max(lc0, lc1) > 1.6 * std::max(la, lb) ||
            la + lb < 0.65 * (lc0 + lc1)) {
            return;
        }

        std::vector<int> ae = corridorArcEdges(a, sampleEdges, n);
        std::vector<int> be = corridorArcEdges(b, sampleEdges, n);
        if (ae.empty() || be.empty()) return;
        if (expectedRails &&
            !(sameCorridorEdgeSet(ae, (*expectedRails)[0]) &&
              sameCorridorEdgeSet(be, (*expectedRails)[1])) &&
            !(sameCorridorEdgeSet(ae, (*expectedRails)[1]) &&
              sameCorridorEdgeSet(be, (*expectedRails)[0]))) {
            return;
        }

        double minW = 1e300, maxW = 0.0, sumW = 0.0;
        double meanAlign = 0.0, minAlign = 1.0;
        int alignN = 0;
        double sign = 0.0, minArea = 1e300;
        constexpr int probes = 16;
        CorridorSample pa0 = sampleCorridorArc(a, 0.0, p, uv);
        CorridorSample pb0 = sampleCorridorArc(b, 0.0, p, uv);
        for (int s = 0; s < probes; ++s) {
            const double f0 = double(s) / probes;
            const double f1 = double(s + 1) / probes;
            const CorridorSample aa = sampleCorridorArc(a, f0, p, uv);
            const CorridorSample ab = sampleCorridorArc(a, f1, p, uv);
            const CorridorSample ba = sampleCorridorArc(b, f0, p, uv);
            const CorridorSample bb = sampleCorridorArc(b, f1, p, uv);
            if (s > 0 && s + 1 < probes) {
                const double w = aa.p.Distance(ba.p);
                minW = std::min(minW, w);
                maxW = std::max(maxW, w);
                sumW += w;
            }
            gp_Vec ta(aa.p, ab.p), tb(ba.p, bb.p);
            if (ta.Magnitude() > 1e-10 && tb.Magnitude() > 1e-10) {
                const double d = ta.Dot(tb) /
                    (ta.Magnitude() * tb.Magnitude());
                meanAlign += d;
                minAlign = std::min(minAlign, d);
                ++alignN;
            }
            const std::array<gp_Pnt2d, 4> q = {aa.uv, ab.uv, bb.uv, ba.uv};
            double a2 = 0.0;
            for (int z = 0; z < 4; ++z) {
                a2 += q[z].X() * q[(z + 1) % 4].Y() -
                      q[(z + 1) % 4].X() * q[z].Y();
            }
            const double epsA = 1e-9 * (u1 - u0) * (v1 - v0);
            if (std::abs(a2) <= epsA) return;
            if (sign == 0.0) sign = a2;
            if (a2 * sign <= 0.0) return;
            minArea = std::min(minArea, std::abs(a2));
            pa0 = ab; pb0 = bb;
        }
        (void)pa0; (void)pb0;
        if (!std::isfinite(minW) || minW <= 1e-8 || maxW > 7.0 * minW) {
            return;
        }
        if (alignN == 0) return;
        meanAlign /= alignN;
        // The rails must advance alongside one another.  This is the key
        // distinction between a real corridor and the tempting but wrong
        // split which swallows a short end wall into each rail: that split
        // still looks axis-monotone, but its paired tangents turn across one
        // another and bunch many rungs into the corner.
        if (meanAlign < 0.55 || minAlign < -0.25) return;
        const double meanW = sumW / std::max(1, probes - 2);
        const double balance = std::abs(la - lb) / std::max(la, lb);
        const double widthVar = (maxW - minW) / std::max(1e-12, meanW);
        const double crossRange = axis == 0 ? (v1 - v0) : (u1 - u0);
        auto normalizedCrossTravel = [&](const std::vector<int>& rail) {
            double d = 0.0;
            for (size_t z = 1; z < rail.size(); ++z) {
                const double x0 = axis == 0 ? uv[rail[z - 1]].Y()
                                             : uv[rail[z - 1]].X();
                const double x1 = axis == 0 ? uv[rail[z]].Y()
                                             : uv[rail[z]].X();
                d += std::abs(x1 - x0);
            }
            return d / std::max(1e-12, crossRange);
        };
        const double crossDrift = normalizedCrossTravel(a) +
                                  normalizedCrossTravel(b);
        // Prefer the simplest pair of genuinely longitudinal rails.  Cap
        // length is deliberately absent from the score: swallowing a short
        // end wall into a rail makes the remaining cap look cheaper but is
        // exactly what creates a corner fan.  Cross-axis drift and a very
        // small edge-piece tie-break keep those end walls in the closures.
        const double score = 8.0 - 2.5 * balance - 1.8 * widthVar +
                             3.5 * meanAlign - 2.0 * crossDrift -
                             0.20 * double(ae.size() + be.size());
        candidates.push_back({i, j, k, l, axis, decreasing, score});
    };

    for (int axis = 0; axis < 2; ++axis) {
        if (axisHint >= 0 && axis != axisHint) continue;
        for (int ia = 0; ia < nc; ++ia)
            for (int ib = ia + 1; ib < nc; ++ib)
                for (int ic = ib + 1; ic < nc; ++ic)
                    for (int id = ic + 1; id < nc; ++id) {
                        const int c0 = corners[ia], c1 = corners[ib];
                        const int c2 = corners[ic], c3 = corners[id];
                        evaluateCheap(c0, c1, c2, c3, axis);
                        evaluateCheap(c1, c2, c3, c0, axis);
                    }
    }
    if (candidates.empty()) return false;
    std::sort(candidates.begin(), candidates.end(),
              [](const Meta& a, const Meta& b) { return a.score > b.score; });

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) return false;
    // Only the best few cheap candidates pay for BRep face classification.
    // A valid corridor has a decisive rail/cap split; dozens of near ties are
    // evidence that this is not the narrow topology class handled here.
    const int testN = std::min<int>(16, candidates.size());
    for (int ci = 0; ci < testN; ++ci) {
        const Meta& m = candidates[ci];
        std::vector<int> rawA = corridorRingArc(n, m.i, m.j, +1);
        std::vector<int> rawB = corridorRingArc(n, m.k, m.l, +1);
        std::vector<int> capJK = corridorRingArc(n, m.j, m.k, +1);
        std::vector<int> capLI = corridorRingArc(n, m.l, m.i, +1);
        std::vector<int> a = rawA;
        std::vector<int> b(rawB.rbegin(), rawB.rend());
        std::vector<int> low = capLI, high = capJK;
        if (m.reverse) {
            std::reverse(a.begin(), a.end());
            std::reverse(b.begin(), b.end());
            low = capJK;
            high = capLI;
        }
        // Dense rungs prove the entire body corridor, including its physical
        // ends.  A trim that needs an out-of-face end chord is not this narrow
        // topology class and must stay on the exact-border fallback.
        bool inside = true;
        for (int s = 0; s <= 24 && inside; ++s) {
            const double f = double(s) / 24.0;
            const CorridorSample aa = sampleCorridorArc(a, f, p, uv);
            const CorridorSample bb = sampleCorridorArc(b, f, p, uv);
            for (int x = 1; x < 5; ++x) {
                const double t = double(x) / 5.0;
                const gp_Pnt2d q(aa.uv.X() * (1.0 - t) + bb.uv.X() * t,
                                 aa.uv.Y() * (1.0 - t) + bb.uv.Y() * t);
                BRepClass_FaceClassifier cls(
                    const_cast<TopoDS_Face&>(face), q, 1e-7);
                if (cls.State() == TopAbs_OUT) {
                    inside = false;
                    break;
                }
            }
        }
        if (!inside) continue;

        TrimCorridorPatch got;
        got.ok = true;
        got.axisU = m.axis == 0;
        got.railA = std::move(a);
        got.railB = std::move(b);
        got.capLow = std::move(low);
        got.capHigh = std::move(high);
        got.railAEdges = corridorArcEdges(got.railA, sampleEdges, n);
        got.railBEdges = corridorArcEdges(got.railB, sampleEdges, n);
        got.capLowEdges = corridorArcEdges(got.capLow, sampleEdges, n);
        got.capHighEdges = corridorArcEdges(got.capHigh, sampleEdges, n);
        got.score = m.score;
        if (got.railAEdges.empty() || got.railBEdges.empty()) continue;
        best = std::move(got);
        return true;
    }
    return false;
}

// A separate four-chain recognizer for broad shallow panels and curved neck
// transitions.  Unlike findTrimCorridor(), this class is defined by the UV
// chart itself: two opposite boundary chains must traverse almost the complete
// U or V range, the other two chains must be local end closures, and a dense
// ruled lattice between matched rail stations must remain inside with one
// Jacobian orientation.  Keeping it separate lets these many-piece STEP trims
// qualify without weakening the accepted corridor/Coons predicates.
bool findSectionStrip(const TopoDS_Face& face,
                      const std::vector<gp_Pnt>& p,
                      const std::vector<gp_Pnt2d>& uv,
                      const std::vector<int>& corners,
                      const std::vector<int>& sampleEdges, int axisHint,
                      const std::array<std::vector<int>, 2>* expectedRails,
                      TrimCorridorPatch& best) {
    best = {};
    const int n = int(p.size()), nc = int(corners.size());
    if (n < 8 || nc < 8 || nc > 24 || int(uv.size()) != n ||
        int(sampleEdges.size()) != n) {
        return false;
    }
    double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    for (const gp_Pnt2d& q : uv) {
        u0 = std::min(u0, q.X()); u1 = std::max(u1, q.X());
        v0 = std::min(v0, q.Y()); v1 = std::max(v1, q.Y());
    }
    const double span[2] = {u1 - u0, v1 - v0};
    if (span[0] <= 1e-12 || span[1] <= 1e-12) return false;

    struct Candidate {
        std::vector<int> a, b, low, high;
        std::vector<int> ae, be;
        int axis = 0;
        double score = -1e300;
    };
    std::vector<Candidate> candidates;
    auto coord = [&](int idx, int axis) {
        return axis == 0 ? uv[idx].X() : uv[idx].Y();
    };
    auto arcPrefix = [&](const std::vector<int>& arc) {
        std::vector<double> d(arc.size(), 0.0);
        for (size_t i = 1; i < arc.size(); ++i) {
            d[i] = d[i - 1] + p[arc[i - 1]].Distance(p[arc[i]]);
        }
        return d;
    };
    auto arcSample = [&](const std::vector<int>& arc,
                         const std::vector<double>& d, double t) {
        if (arc.size() == 1 || t <= 0.0) {
            return CorridorSample{uv[arc.front()], p[arc.front()]};
        }
        if (t >= 1.0 || d.back() <= 1e-14) {
            return CorridorSample{uv[arc.back()], p[arc.back()]};
        }
        const double want = d.back() * t;
        const auto it = std::lower_bound(d.begin(), d.end(), want);
        size_t z = it == d.begin() ? 0 : size_t(it - d.begin() - 1);
        if (z + 1 >= arc.size()) z = arc.size() - 2;
        const double spanD = d[z + 1] - d[z];
        const double f = spanD > 1e-14 ? (want - d[z]) / spanD : 0.0;
        const gp_Pnt2d a = uv[arc[z]], b = uv[arc[z + 1]];
        return CorridorSample{
            gp_Pnt2d(a.X() * (1.0 - f) + b.X() * f,
                     a.Y() * (1.0 - f) + b.Y() * f),
            gp_Pnt(p[arc[z]].XYZ() * (1.0 - f) +
                   p[arc[z + 1]].XYZ() * f)};
    };
    auto monotone = [&](const std::vector<int>& rail, int axis,
                        double& coverage, double& crossDrift) {
        if (rail.size() < 2) return false;
        const double d = coord(rail.back(), axis) - coord(rail.front(), axis);
        coverage = std::abs(d) / span[axis];
        if (coverage < 0.72) return false;
        const double sign = d >= 0.0 ? 1.0 : -1.0;
        double back = 0.0, crossLo = 1e300, crossHi = -1e300;
        for (size_t i = 0; i < rail.size(); ++i) {
            const double c = coord(rail[i], 1 - axis);
            crossLo = std::min(crossLo, c);
            crossHi = std::max(crossHi, c);
            if (i == 0) continue;
            const double step = sign *
                (coord(rail[i], axis) - coord(rail[i - 1], axis));
            if (step < -0.04 * span[axis]) return false;
            if (step < 0.0) back -= step;
        }
        if (back > 0.10 * span[axis]) return false;
        crossDrift = (crossHi - crossLo) / span[1 - axis];
        return crossDrift <= 0.34;
    };
    auto capLike = [&](const std::vector<int>& cap, int axis) {
        if (cap.size() < 2) return false;
        double axisLo = 1e300, axisHi = -1e300;
        double crossLo = 1e300, crossHi = -1e300;
        for (int idx : cap) {
            axisLo = std::min(axisLo, coord(idx, axis));
            axisHi = std::max(axisHi, coord(idx, axis));
            crossLo = std::min(crossLo, coord(idx, 1 - axis));
            crossHi = std::max(crossHi, coord(idx, 1 - axis));
        }
        return axisHi - axisLo <= 0.36 * span[axis] &&
               crossHi - crossLo >= 0.45 * span[1 - axis];
    };
    auto consider = [&](int i, int j, int k, int l, int axis) {
        std::vector<int> a = corridorRingArc(n, i, j, +1);
        std::vector<int> high = corridorRingArc(n, j, k, +1);
        std::vector<int> rawB = corridorRingArc(n, k, l, +1);
        std::vector<int> low = corridorRingArc(n, l, i, +1);
        std::vector<int> b(rawB.rbegin(), rawB.rend());
        if (a.size() < 2 || b.size() < 2 || low.size() < 2 ||
            high.size() < 2) {
            return;
        }
        // Put both rails in the same longitudinal direction.
        const double da = coord(a.back(), axis) - coord(a.front(), axis);
        const double db = coord(b.back(), axis) - coord(b.front(), axis);
        if (da * db < 0.0) std::reverse(b.begin(), b.end());
        double coverA = 0.0, coverB = 0.0, driftA = 0.0, driftB = 0.0;
        if (!monotone(a, axis, coverA, driftA) ||
            !monotone(b, axis, coverB, driftB)) {
            return;
        }
        if (!capLike(low, axis) || !capLike(high, axis)) {
            return;
        }
        std::vector<int> ae = corridorArcEdges(a, sampleEdges, n);
        std::vector<int> be = corridorArcEdges(b, sampleEdges, n);
        if (ae.empty() || be.empty() || ae.size() > 12 || be.size() > 12) {
            return;
        }
        if (expectedRails &&
            !(sameCorridorEdgeSet(ae, (*expectedRails)[0]) &&
              sameCorridorEdgeSet(be, (*expectedRails)[1])) &&
            !(sameCorridorEdgeSet(ae, (*expectedRails)[1]) &&
              sameCorridorEdgeSet(be, (*expectedRails)[0]))) {
            return;
        }
        const std::vector<double> prefixA = arcPrefix(a);
        const std::vector<double> prefixB = arcPrefix(b);
        const double la = prefixA.back(), lb = prefixB.back();
        if (std::min(la, lb) <= 1e-9 ||
            std::max(la, lb) > 4.0 * std::min(la, lb)) {
            return;
        }
        double minW = 1e300, maxW = 0.0, align = 0.0;
        double referenceArea = 0.0;
        constexpr int probes = 20;
        for (int s = 0; s < probes; ++s) {
            const double f0 = double(s) / probes;
            const double f1 = double(s + 1) / probes;
            const CorridorSample a0s = arcSample(a, prefixA, f0);
            const CorridorSample a1s = arcSample(a, prefixA, f1);
            const CorridorSample b0s = arcSample(b, prefixB, f0);
            const CorridorSample b1s = arcSample(b, prefixB, f1);
            const double w = a0s.p.Distance(b0s.p);
            minW = std::min(minW, w); maxW = std::max(maxW, w);
            gp_Vec ta(a0s.p, a1s.p), tb(b0s.p, b1s.p);
            if (ta.Magnitude() > 1e-10 && tb.Magnitude() > 1e-10) {
                align += ta.Dot(tb) / (ta.Magnitude() * tb.Magnitude());
            }
            const std::array<gp_Pnt2d, 4> q = {
                a0s.uv, a1s.uv, b1s.uv, b0s.uv};
            double a2 = 0.0;
            for (int z = 0; z < 4; ++z) {
                a2 += q[z].X() * q[(z + 1) % 4].Y() -
                      q[(z + 1) % 4].X() * q[z].Y();
            }
            if (!std::isfinite(a2) || std::abs(a2) <=
                    1e-10 * span[0] * span[1]) {
                return;
            }
            if (referenceArea == 0.0) referenceArea = a2;
            if (a2 * referenceArea <= 0.0) {
                return;
            }
        }
        align /= probes;
        if (!std::isfinite(minW) || minW <= 1e-9 ||
            maxW > 5.0 * minW || align < 0.20) {
            return;
        }
        const double score = 6.0 * (coverA + coverB) + 2.0 * align -
                             2.0 * (driftA + driftB) -
                             0.12 * double(ae.size() + be.size());
        candidates.push_back({std::move(a), std::move(b), std::move(low),
                              std::move(high), std::move(ae), std::move(be),
                              axis, score});
    };

    auto nearEnd = [&](int idx, int axis, bool high) {
        const double lo = axis == 0 ? u0 : v0;
        const double hi = axis == 0 ? u1 : v1;
        const double x = coord(idx, axis);
        return high ? x >= hi - 0.18 * span[axis]
                    : x <= lo + 0.18 * span[axis];
    };
    auto endpointPattern = [&](int i, int j, int k, int l, int axis) {
        return (nearEnd(i, axis, false) && nearEnd(j, axis, true) &&
                nearEnd(k, axis, true) && nearEnd(l, axis, false)) ||
               (nearEnd(i, axis, true) && nearEnd(j, axis, false) &&
                nearEnd(k, axis, false) && nearEnd(l, axis, true));
    };
    for (int axis = 0; axis < 2; ++axis) {
        if (axisHint >= 0 && axis != axisHint) continue;
        for (int ia = 0; ia < nc; ++ia)
            for (int ib = ia + 1; ib < nc; ++ib)
                for (int ic = ib + 1; ic < nc; ++ic)
                    for (int id = ic + 1; id < nc; ++id) {
                        const int c0 = corners[ia], c1 = corners[ib];
                        const int c2 = corners[ic], c3 = corners[id];
                        // Cheap extrema test happens before any arc vectors
                        // or prefix tables are allocated.  Only a rectangle-
                        // like low/high/high/low split reaches consider().
                        if (endpointPattern(c0, c1, c2, c3, axis)) {
                            consider(c0, c1, c2, c3, axis);
                        }
                        if (endpointPattern(c1, c2, c3, c0, axis)) {
                            consider(c1, c2, c3, c0, axis);
                        }
                    }
    }
    if (candidates.empty()) return false;
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.score > b.score;
              });

    const double tol = std::max(1e-9, BRep_Tool::Tolerance(face));
    const int testN = std::min<int>(24, candidates.size());
    for (int ci = 0; ci < testN; ++ci) {
        Candidate& c = candidates[ci];
        const std::vector<double> da = arcPrefix(c.a);
        const std::vector<double> db = arcPrefix(c.b);
        std::array<bool, 25> safe{};
        // Dense body proof.  The exact, potentially concave trim chains at
        // the two physical ends are local closure cells, not straight ruled
        // rows.  Find one central contiguous interval of valid rungs, while
        // bounding each closure to at most about one third of the rail.
        for (int s = 0; s <= 24; ++s) {
            const double f = double(s) / 24.0;
            const CorridorSample a = arcSample(c.a, da, f);
            const CorridorSample b = arcSample(c.b, db, f);
            bool stationInside = true;
            for (int x = 1; x < 6; ++x) {
                const double t = double(x) / 6.0;
                const gp_Pnt2d q(a.uv.X() * (1.0 - t) + b.uv.X() * t,
                                 a.uv.Y() * (1.0 - t) + b.uv.Y() * t);
                BRepClass_FaceClassifier cls(
                    const_cast<TopoDS_Face&>(face), q, tol);
                if (cls.State() == TopAbs_OUT) {
                    stationInside = false;
                    break;
                }
            }
            safe[size_t(s)] = stationInside;
        }
        int bestStart = -1, bestEnd = -1;
        for (int s = 0; s <= 24;) {
            while (s <= 24 && !safe[size_t(s)]) ++s;
            const int start = s;
            while (s <= 24 && safe[size_t(s)]) ++s;
            const int end = s - 1;
            if (start <= 24 &&
                (bestStart < 0 || end - start > bestEnd - bestStart)) {
                bestStart = start;
                bestEnd = end;
            }
        }
        const bool centralBody = bestStart >= 0 && bestStart <= 8 &&
                                 bestEnd >= 16 && bestStart <= 12 &&
                                 bestEnd >= 12 &&
                                 bestEnd - bestStart + 1 >= 11;
        if (!centralBody) {
            continue;
        }
        best.ok = true;
        best.axisU = c.axis == 0;
        best.railA = std::move(c.a);
        best.railB = std::move(c.b);
        best.capLow = std::move(c.low);
        best.capHigh = std::move(c.high);
        best.railAEdges = std::move(c.ae);
        best.railBEdges = std::move(c.be);
        best.capLowEdges = corridorArcEdges(best.capLow, sampleEdges, n);
        best.capHighEdges = corridorArcEdges(best.capHigh, sampleEdges, n);
        best.score = c.score;
        if (best.capLowEdges.empty() || best.capHighEdges.empty()) continue;
        return true;
    }
    return false;
}

// Minimum number of across-corridor bands required by the surface itself.
// A narrow annular trim stays one quad strip; a broader curved grip panel may
// get a few true-surface scaffold rails rather than one chord spanning its
// whole width.  This is still a rail grid, never a face-centre fan.
int trimCorridorBandCount(const TopoDS_Face& face,
                          const TrimCorridorPatch& patch,
                          const std::vector<gp_Pnt>& p,
                          const std::vector<gp_Pnt2d>& uv,
                          double chordTolerance) {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) return 0;
    const double tol = std::max(1e-7, chordTolerance);
    for (int bands = 1; bands <= 12; ++bands) {
        bool ok = true;
        for (int s = 0; s <= 16 && ok; ++s) {
            const double f = double(s) / 16.0;
            const CorridorSample a =
                sampleCorridorArc(patch.railA, f, p, uv);
            const CorridorSample b =
                sampleCorridorArc(patch.railB, f, p, uv);
            for (int j = 0; j < bands; ++j) {
                const double t0 = double(j) / bands;
                const double t1 = double(j + 1) / bands;
                const double tm = 0.5 * (t0 + t1);
                auto at = [&](double t) {
                    const double u = a.uv.X() * (1.0 - t) + b.uv.X() * t;
                    const double v = a.uv.Y() * (1.0 - t) + b.uv.Y() * t;
                    return surface->Value(u, v);
                };
                const gp_Pnt p0 = at(t0), p1 = at(t1), pm = at(tm);
                const gp_Pnt chord((p0.XYZ() + p1.XYZ()) * 0.5);
                if (pm.Distance(chord) > tol) {
                    ok = false;
                    break;
                }
            }
        }
        if (ok) return bands;
    }
    return 0;
}

bool planTrimCorridor(const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf, const Model& model,
                      const FaceMeshSettings& settings, FacePlan& plan) {
    const GeomAbs_SurfaceType st = surf.GetType();
    if (st != GeomAbs_BSplineSurface && st != GeomAbs_BezierSurface &&
        st != GeomAbs_OffsetSurface && st != GeomAbs_OtherSurface) {
        return false;
    }
    if (surf.IsUPeriodic() || surf.IsVPeriodic()) return false;
    int wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
    }
    std::vector<gp_Pnt> p;
    std::vector<gp_Pnt2d> uv;
    std::vector<int> corners, sampleEdges;
    if (!sampleRibbonRing(face, model, nullptr, 16, p, uv, corners, nullptr,
                          &sampleEdges)) {
        return false;
    }
    TrimCorridorPatch patch;
    if (!findTrimCorridor(face, p, uv, corners, sampleEdges, -1, nullptr,
                          patch)) {
        return false;
    }
    const int bands = trimCorridorBandCount(
        face, patch, p, uv, settings.chordTolerance);
    if (bands < 1) return false;

    std::vector<int> loop;
    for (BRepTools_WireExplorer we(BRepTools::OuterWire(face), face);
         we.More(); we.Next()) {
        if (BRep_Tool::Degenerated(we.Current())) continue;
        const int eid = model.edges.FindIndex(we.Current());
        if (eid < 1) return false;
        loop.push_back(eid);
    }
    if (loop.size() != corners.size()) return false;

    plan = FacePlan();
    plan.kind = MesherKind::CoonsGrid;
    plan.constrains = true;
    plan.trimCorridor = true;
    plan.trimCorridorAxisU = patch.axisU;
    plan.trimCorridorBands = bands;
    plan.coonsSides[0] = patch.railAEdges;
    plan.coonsSides[2] = patch.railBEdges;
    plan.uEdges = patch.railAEdges;
    plan.uEdges.insert(plan.uEdges.end(), patch.railBEdges.begin(),
                       patch.railBEdges.end());
    plan.vEdges = patch.capLowEdges;
    plan.vEdges.insert(plan.vEdges.end(), patch.capHighEdges.begin(),
                       patch.capHighEdges.end());
    plan.loops = {std::move(loop)};  // per-edge proposals; never union chains
    auto edgeList = [](const std::vector<int>& edges) {
        std::string s;
        for (int e : edges) {
            if (!s.empty()) s += ',';
            s += std::to_string(e);
        }
        return s;
    };
    dbg("corridor plan rails [%s] / [%s], caps [%s] / [%s]",
        edgeList(patch.railAEdges).c_str(),
        edgeList(patch.railBEdges).c_str(),
        edgeList(patch.capLowEdges).c_str(),
        edgeList(patch.capHighEdges).c_str());
    return true;
}

bool planSectionStrip(const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf, const Model& model,
                      const FaceMeshSettings& settings, FacePlan& plan) {
    const GeomAbs_SurfaceType st = surf.GetType();
    if (st != GeomAbs_BSplineSurface && st != GeomAbs_BezierSurface &&
        st != GeomAbs_OffsetSurface && st != GeomAbs_OtherSurface) {
        return false;
    }
    if (surf.IsUPeriodic() || surf.IsVPeriodic()) return false;
    int wires = 0, edgePieces = 0;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
    }
    if (wires != 1) return false;
    for (BRepTools_WireExplorer we(outer, face); we.More(); we.Next()) {
        if (BRep_Tool::Degenerated(we.Current())) return false;
        ++edgePieces;
    }
    if (edgePieces < 12 || edgePieces > 24) return false;

    // Cheap curvature class gate before any corner-combination work.  The
    // transition path is for a genuinely bent but coherent chart: flat panels
    // already took the deviation-n-gon route, while a normal field turning
    // through more than ~80 degrees is not safe for ruled cross sections.
    {
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        if (surface.IsNull()) return false;
        double u0, u1, v0, v1;
        BRepTools::UVBounds(face, u0, u1, v0, v1);
        gp_Vec reference;
        bool haveReference = false;
        double worstDot = 1.0;
        int normals = 0;
        for (int j = 0; j < 5; ++j) {
            for (int i = 0; i < 5; ++i) {
                const gp_Pnt2d q(u0 + (u1 - u0) * (i + 0.5) / 5.0,
                                 v0 + (v1 - v0) * (j + 0.5) / 5.0);
                BRepClass_FaceClassifier cls(
                    const_cast<TopoDS_Face&>(face), q,
                    std::max(1e-9, BRep_Tool::Tolerance(face)));
                if (cls.State() == TopAbs_OUT) continue;
                gp_Pnt at;
                gp_Vec du, dv;
                try {
                    surface->D1(q.X(), q.Y(), at, du, dv);
                } catch (const Standard_Failure&) {
                    return false;
                }
                gp_Vec n = du.Crossed(dv);
                if (n.Magnitude() <= 1e-14) return false;
                n.Normalize();
                if (!haveReference) {
                    reference = n;
                    haveReference = true;
                }
                if (n.Dot(reference) < 0.0) n.Reverse();
                worstDot = std::min(worstDot, n.Dot(reference));
                ++normals;
            }
        }
        if (normals < 8 || worstDot > 0.97 || worstDot < 0.15) return false;
    }

    std::vector<gp_Pnt> p;
    std::vector<gp_Pnt2d> uv;
    std::vector<int> corners, sampleEdges;
    if (!sampleRibbonRing(face, model, nullptr, 16, p, uv, corners, nullptr,
                          &sampleEdges)) {
        return false;
    }
    TrimCorridorPatch patch;
    if (!findSectionStrip(face, p, uv, corners, sampleEdges, -1, nullptr,
                          patch)) {
        return false;
    }
    int bands = trimCorridorBandCount(face, patch, p, uv,
                                      settings.chordTolerance);
    if (bands < 1) return false;
    // Two interior sections keep a genuinely curved neck from collapsing
    // into one ruled chord even when midpoint sag is deceptive.
    bands = std::max(bands, 3);
    bands = std::min(bands, 12);

    std::vector<int> loop;
    for (BRepTools_WireExplorer we(outer, face); we.More(); we.Next()) {
        if (BRep_Tool::Degenerated(we.Current())) continue;
        const int eid = model.edges.FindIndex(we.Current());
        if (eid < 1) return false;
        loop.push_back(eid);
    }
    if (loop.size() != size_t(edgePieces)) return false;

    plan = FacePlan();
    plan.kind = MesherKind::CoonsGrid;
    plan.constrains = true;
    plan.sectionStrip = true;
    plan.sectionStripAxisU = patch.axisU;
    plan.sectionStripBands = bands;
    plan.coonsSides[0] = patch.railAEdges;
    plan.coonsSides[2] = patch.railBEdges;
    plan.uEdges = patch.railAEdges;
    plan.uEdges.insert(plan.uEdges.end(), patch.railBEdges.begin(),
                       patch.railBEdges.end());
    plan.vEdges = patch.capLowEdges;
    plan.vEdges.insert(plan.vEdges.end(), patch.capHighEdges.begin(),
                       patch.capHighEdges.end());
    plan.loops = {std::move(loop)};
    dbg("section strip: transition axis=%c rails %zu/%zu caps %zu/%zu "
        "bands=%d",
        patch.axisU ? 'u' : 'v',
        patch.railAEdges.size(), patch.railBEdges.size(),
        patch.capLowEdges.size(), patch.capHighEdges.size(), bands);
    return true;
}

// Deviation-flat panel. Some STEP writers retain a very shallow sculpt in a
// BSpline chart even though it is below the active tessellation tolerance.
// The generic relative-planarity heuristic can reject a large shallow panel,
// after which fallback draws a face-wide CDT/star. This probe is deliberately
// late and narrow: one broad single-wire freeform face, no periodic chart,
// coherent dense boundary/interior normals, and a slab bounded by the active
// absolute/relative deviation plus a CAD/model-tolerance floor.
bool planDeviationFlatPanel(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model,
                            const FaceMeshSettings& settings,
                            FacePlan& plan) {
    const GeomAbs_SurfaceType st = surf.GetType();
    if (st != GeomAbs_BSplineSurface && st != GeomAbs_BezierSurface &&
        st != GeomAbs_OffsetSurface && st != GeomAbs_OtherSurface) {
        return false;
    }
    if (surf.IsUPeriodic() || surf.IsVPeriodic()) return false;
    const double chordSetting = settings.chordTolerance;
    if (!(chordSetting > 1e-8) || !std::isfinite(chordSetting)) return false;

    int wires = 0, edgePieces = 0;
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
    }
    if (wires != 1) return false;
    double cadTolerance = std::max(1e-9, BRep_Tool::Tolerance(face));
    for (BRepTools_WireExplorer we(outer, face); we.More(); we.Next()) {
        const TopoDS_Edge edge = we.Current();
        if (BRep_Tool::Degenerated(edge)) return false;
        cadTolerance = std::max(cadTolerance, BRep_Tool::Tolerance(edge));
        for (TopExp_Explorer vx(edge, TopAbs_VERTEX); vx.More(); vx.Next()) {
            cadTolerance = std::max(
                cadTolerance,
                BRep_Tool::Tolerance(TopoDS::Vertex(vx.Current())));
        }
        ++edgePieces;
    }
    // A four-sided patch belongs to Coons; a very edge-rich outline is a
    // boolean web, not one readable panel.  The lower bound also prevents
    // this late path from stealing ordinary analytic-looking patches.
    if (edgePieces < 8 || edgePieces > 24) return false;

    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) return false;
    double u0, u1, v0, v1;
    BRepTools::UVBounds(face, u0, u1, v0, v1);
    if (!(u1 > u0) || !(v1 > v0)) return false;

    std::vector<gp_Pnt> points;
    std::vector<gp_Vec> normals;
    points.reserve(400);
    normals.reserve(200);
    const double faceTol = std::max(1e-9, BRep_Tool::Tolerance(face));
    auto addSurfaceProbe = [&](const gp_Pnt2d& uv) {
        gp_Pnt p;
        gp_Vec du, dv;
        try {
            surface->D1(uv.X(), uv.Y(), p, du, dv);
        } catch (const Standard_Failure&) {
            return false;
        }
        gp_Vec n = du.Crossed(dv);
        if (n.Magnitude() <= 1e-14) return false;
        n.Normalize();
        points.push_back(p);
        normals.push_back(n);
        return true;
    };

    // Exact trim boundary, sampled independently of the later density solve.
    // Any out-of-plane trim wiggle must count against the slab just as the
    // interior surface does.
    for (BRepTools_WireExplorer we(outer, face); we.More(); we.Next()) {
        const TopoDS_Edge edge = we.Current();
        double f = 0.0, l = 0.0;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f, l);
        if (pc.IsNull() || !std::isfinite(f) || !std::isfinite(l)) {
            return false;
        }
        for (int k = 0; k <= 12; ++k) {
            if (!addSurfaceProbe(pc->Value(f + (l - f) * k / 12.0))) {
                return false;
            }
        }
    }

    // Dense classified interior.  Unlike isGeometricallyFlat(), points in a
    // trimmed-away part of the underlying spline do not influence the result.
    constexpr int grid = 13;
    int insideCount = 0;
    for (int j = 0; j < grid; ++j) {
        for (int i = 0; i < grid; ++i) {
            const gp_Pnt2d uv(u0 + (u1 - u0) * (i + 0.5) / grid,
                              v0 + (v1 - v0) * (j + 0.5) / grid);
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), uv,
                                          faceTol);
            if (cls.State() == TopAbs_OUT) continue;
            if (!addSurfaceProbe(uv)) return false;
            ++insideCount;
        }
    }
    if (insideCount < 20 || points.size() < 32) return false;

    // Average consistently-oriented surface normals.  A shallow but rapidly
    // rippled sheet can fit a thin slab while still needing tessellation; the
    // normal-cone gate excludes it.
    gp_XYZ normalSum(0, 0, 0);
    gp_Vec reference = normals.front();
    for (gp_Vec n : normals) {
        if (n.Dot(reference) < 0.0) n.Reverse();
        normalSum += n.XYZ();
    }
    if (normalSum.Modulus() <= 1e-12) return false;
    gp_Dir panelNormal(normalSum);
    constexpr double minNormalDot = 0.985;  // about a 10-degree cone
    double worstNormalDot = 1.0;
    for (gp_Vec n : normals) {
        if (n.Dot(reference) < 0.0) n.Reverse();
        worstNormalDot =
            std::min(worstNormalDot, n.Dot(gp_Vec(panelNormal)));
    }
    if (worstNormalDot < minNormalDot) return false;

    gp_XYZ centre(0, 0, 0);
    for (const gp_Pnt& p : points) centre += p.XYZ();
    centre /= double(points.size());
    double lo = 1e300, hi = -1e300;
    double xLo = 1e300, xHi = -1e300;
    double yLo = 1e300, yHi = -1e300;
    double zLo = 1e300, zHi = -1e300;
    for (const gp_Pnt& p : points) {
        const double d = (p.XYZ() - centre).Dot(panelNormal.XYZ());
        lo = std::min(lo, d);
        hi = std::max(hi, d);
        xLo = std::min(xLo, p.X()); xHi = std::max(xHi, p.X());
        yLo = std::min(yLo, p.Y()); yHi = std::max(yHi, p.Y());
        zLo = std::min(zLo, p.Z()); zHi = std::max(zHi, p.Z());
    }
    // Conservative O(N) extent: the AABB diagonal bounds every pairwise
    // distance, so all scale-aware gates remain at least as strict as the
    // former quadratic diameter scan without adding hundreds of thousands
    // of distance evaluations on dense imported panels.
    const double dx = xHi - xLo, dy = yHi - yLo, dz = zHi - zLo;
    const double diameter = std::sqrt(dx * dx + dy * dy + dz * dz);
    // A broad panel, not a hairline transition strip.  The perimeter/area
    // aspect guard is intrinsic geometry and does not rely on model IDs.
    GProp_GProps sp, lp;
    try {
        BRepGProp::SurfaceProperties(face, sp);
        BRepGProp::LinearProperties(outer, lp);
    } catch (const Standard_Failure&) {
        return false;
    }
    const double area = sp.Mass(), perimeter = lp.Mass();
    if (!(area > 1e-10) || !(perimeter > 1e-8)) return false;
    const double width = 2.0 * area / perimeter;
    const double length = 0.5 * perimeter - width;
    if (!(width > 1e-8) || length > 8.0 * width) return false;
    if (!(diameter > 1e-8) || width < 0.04 * diameter) return false;

    // The boundary itself is exact and may occupy both sides of the best
    // plane.  The allowance is scale-aware and uses the same active deviation
    // meaning as fallback meshing: absolute chord, or 5% of face extent in
    // relative mode.  A 15% leeway accounts for a non-planar exact boundary;
    // CAD sewing tolerance and a tiny model-scale quantum form a floor so
    // imported tolerance noise cannot arbitrarily flip the classification.
    // Dense height plus the normal-cone check above remain the real geometry
    // gates; this is not a GeomAbs_Plane/type shortcut.
    const double slab = hi - lo;
    const double activeDeviation = settings.relativeDeviation
        ? std::max(1e-9, chordSetting * 0.05 * diameter)
        : chordSetting;
    const double toleranceFloor =
        std::max(8.0 * cadTolerance, 1e-6 * diameter);
    const double allowedSlab = toleranceFloor + 1.15 * activeDeviation;
    if (!std::isfinite(slab) || slab > allowedSlab) return false;

    FacePlan probe;
    if (!planMinimalPlanar(face, surf, model, probe,
                           /*requirePlane=*/false)) {
        return false;
    }
    plan = std::move(probe);
    plan.deviationFlatPanel = true;
    dbg("deviation-flat panel: %d-edge exact n-gon, slab %.6g <= %.6g "
        "(active %.6g + floor %.6g), aspect %.3g",
        edgePieces, slab, allowedSlab, activeDeviation, toleranceFloor,
        length / width);
    return true;
}

bool meshTrimCorridor(const TopoDS_Face& face, const Model& model,
                      const FacePlan& plan, int faceId,
                      const std::vector<int>& solvedEdge, int radialDefault,
                      MeshBuilder& out, const PinnedEdges* pins,
                      std::array<int, 2>* built) {
    const size_t firstPolygon = out.mesh().polygons.size();
    std::vector<gp_Pnt> p;
    std::vector<gp_Pnt2d> uv;
    std::vector<int> corners, sampleEdges;
    if (!sampleRibbonRing(face, model, &solvedEdge, radialDefault, p, uv,
                          corners, pins, &sampleEdges)) {
        return false;
    }
    const std::array<std::vector<int>, 2> expected = {
        plan.coonsSides[0], plan.coonsSides[2]};
    TrimCorridorPatch patch;
    const bool found = plan.sectionStrip
        ? findSectionStrip(face, p, uv, corners, sampleEdges,
                           plan.sectionStripAxisU ? 0 : 1, &expected, patch)
        : findTrimCorridor(face, p, uv, corners, sampleEdges,
                           plan.trimCorridorAxisU ? 0 : 1, &expected, patch);
    if (!found) {
        return false;
    }
    const int alongA = int(patch.railA.size()) - 1;
    const int alongB = int(patch.railB.size()) - 1;
    // This is the central contract of the planner.  Unequal rails would need
    // a transition polygon in the body, recreating the very station collapse
    // this path exists to remove.  Leave such a face to the exact-border floor
    // instead of silently weakening the result.
    if (alongA < 2 || alongA != alongB) {
        dbg("corridor face %d: rail totals %d/%d not aligned", faceId,
            alongA, alongB);
        return false;
    }
    const int bands = std::clamp(plan.sectionStrip
                                     ? plan.sectionStripBands
                                     : plan.trimCorridorBands,
                                 1, 12);
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) return false;
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    int bodyFirst = 0;
    int bodyLast = alongA;
    if (plan.sectionStrip) {
        // The section recognizer may intentionally leave short concave ends
        // to local exact-boundary closures.  Re-prove the usable rows using
        // the FINAL solved rail samples, not the planning approximation.
        std::vector<bool> safe(size_t(alongA + 1), false);
        const double faceTol = std::max(1e-9, BRep_Tool::Tolerance(face));
        for (int i = 0; i <= alongA; ++i) {
            const gp_Pnt2d& a = uv[patch.railA[i]];
            const gp_Pnt2d& b = uv[patch.railB[i]];
            bool inside = true;
            for (int x = 1; x < 12; ++x) {
                const double t = double(x) / 12.0;
                const gp_Pnt2d q(a.X() * (1.0 - t) + b.X() * t,
                                 a.Y() * (1.0 - t) + b.Y() * t);
                BRepClass_FaceClassifier cls(
                    const_cast<TopoDS_Face&>(face), q, faceTol);
                if (cls.State() == TopAbs_OUT) {
                    inside = false;
                    break;
                }
            }
            safe[size_t(i)] = inside;
        }
        int bestFirst = -1, bestLast = -1;
        for (int i = 0; i <= alongA;) {
            while (i <= alongA && !safe[size_t(i)]) ++i;
            const int first = i;
            while (i <= alongA && safe[size_t(i)]) ++i;
            const int last = i - 1;
            if (first <= alongA &&
                (bestFirst < 0 || last - first > bestLast - bestFirst)) {
                bestFirst = first;
                bestLast = last;
            }
        }
        const int middle = alongA / 2;
        const int minRows = std::max(
            3, int(std::ceil(0.40 * double(alongA + 1))));
        if (bestFirst < 0 || bestFirst > int(std::ceil(0.34 * alongA)) ||
            bestLast < int(std::floor(0.66 * alongA)) ||
            bestFirst > middle || bestLast < middle ||
            bestLast - bestFirst + 1 < minRows ||
            bestLast - bestFirst < 2) {
            dbg("section strip face %d: solved rows lack central safe run "
                "(%d..%d of %d)", faceId, bestFirst, bestLast, alongA);
            return false;
        }
        bodyFirst = bestFirst;
        bodyLast = bestLast;
    } else {
        // A one-segment B-rep cap cannot be subdivided by private scaffold
        // vertices.  Absorb the adjacent interval into one short transition
        // n-gon instead of collapsing every band onto the endpoint (a fan).
        if (patch.capLow.size() <= 2) bodyFirst = 1;
        if (patch.capHigh.size() <= 2) bodyLast = alongA - 1;
        if (bodyFirst > bodyLast) return false;
    }

    // Every boundary sample is exact and anchorless; interior scaffold rails
    // live on the true surface and retain face-UV anchors for editing.
    std::vector<uint32_t> boundary(p.size(), UINT32_MAX);
    auto boundaryId = [&](int idx) {
        if (boundary[idx] == UINT32_MAX) {
            boundary[idx] = out.addVertex(p[idx], {});
        }
        return boundary[idx];
    };
    std::vector<std::vector<uint32_t>> grid(
        alongA + 1, std::vector<uint32_t>(bands + 1, UINT32_MAX));
    std::vector<std::vector<gp_Pnt2d>> guv(
        alongA + 1, std::vector<gp_Pnt2d>(bands + 1));
    for (int i = 0; i <= alongA; ++i) {
        const int ia = patch.railA[i], ib = patch.railB[i];
        grid[i][0] = boundaryId(ia);
        grid[i][bands] = boundaryId(ib);
        guv[i][0] = uv[ia];
        guv[i][bands] = uv[ib];
        for (int j = 1; j < bands; ++j) {
            if (i < bodyFirst || i > bodyLast) continue;
            const double t = double(j) / bands;
            const double u = uv[ia].X() * (1.0 - t) + uv[ib].X() * t;
            const double v = uv[ia].Y() * (1.0 - t) + uv[ib].Y() * t;
            guv[i][j] = gp_Pnt2d(u, v);
            grid[i][j] = out.addVertex(surface->Value(u, v),
                                       Anchor{faceId, u, v});
        }
    }
    auto emitUv = [&](std::vector<uint32_t> ids,
                      std::vector<gp_Pnt2d> q) {
        std::vector<uint32_t> clean;
        std::vector<gp_Pnt2d> cq;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (!clean.empty() && clean.back() == ids[i]) continue;
            clean.push_back(ids[i]); cq.push_back(q[i]);
        }
        if (clean.size() > 1 && clean.front() == clean.back()) {
            clean.pop_back(); cq.pop_back();
        }
        if (clean.size() < 3) return false;
        // Fail closed on a self-touching or crossing UV outline.  These cells
        // are the topology that will be exported; a positive shoelace sum by
        // itself is insufficient because two crossing lobes can cancel into
        // a plausible signed area.
        const size_t count = clean.size();
        std::set<uint32_t> uniqueIds(clean.begin(), clean.end());
        if (uniqueIds.size() != count) return false;
        double uvLoX = 1e300, uvHiX = -1e300;
        double uvLoY = 1e300, uvHiY = -1e300;
        for (const gp_Pnt2d& x : cq) {
            uvLoX = std::min(uvLoX, x.X()); uvHiX = std::max(uvHiX, x.X());
            uvLoY = std::min(uvLoY, x.Y()); uvHiY = std::max(uvHiY, x.Y());
        }
        const double uvScale = std::max({uvHiX - uvLoX, uvHiY - uvLoY,
                                         1e-12});
        const double crossTol = 1e-12 * uvScale * uvScale;
        auto orient2 = [](const gp_Pnt2d& a, const gp_Pnt2d& b,
                          const gp_Pnt2d& c) {
            return (b.X() - a.X()) * (c.Y() - a.Y()) -
                   (b.Y() - a.Y()) * (c.X() - a.X());
        };
        auto onSegment = [&](const gp_Pnt2d& a, const gp_Pnt2d& b,
                             const gp_Pnt2d& x) {
            return std::abs(orient2(a, b, x)) <= crossTol &&
                   x.X() >= std::min(a.X(), b.X()) - 1e-10 * uvScale &&
                   x.X() <= std::max(a.X(), b.X()) + 1e-10 * uvScale &&
                   x.Y() >= std::min(a.Y(), b.Y()) - 1e-10 * uvScale &&
                   x.Y() <= std::max(a.Y(), b.Y()) + 1e-10 * uvScale;
        };
        auto intersects = [&](const gp_Pnt2d& a, const gp_Pnt2d& b,
                              const gp_Pnt2d& c, const gp_Pnt2d& d) {
            const double o1 = orient2(a, b, c), o2 = orient2(a, b, d);
            const double o3 = orient2(c, d, a), o4 = orient2(c, d, b);
            if (((o1 > crossTol && o2 < -crossTol) ||
                 (o1 < -crossTol && o2 > crossTol)) &&
                ((o3 > crossTol && o4 < -crossTol) ||
                 (o3 < -crossTol && o4 > crossTol))) {
                return true;
            }
            return (std::abs(o1) <= crossTol && onSegment(a, b, c)) ||
                   (std::abs(o2) <= crossTol && onSegment(a, b, d)) ||
                   (std::abs(o3) <= crossTol && onSegment(c, d, a)) ||
                   (std::abs(o4) <= crossTol && onSegment(c, d, b));
        };
        for (size_t i = 0; i < count; ++i) {
            const size_t in = (i + 1) % count;
            for (size_t j = i + 1; j < count; ++j) {
                const size_t jn = (j + 1) % count;
                if (i == j || in == j || jn == i) continue;
                if (intersects(cq[i], cq[in], cq[j], cq[jn])) return false;
            }
        }
        double area = 0.0;
        for (size_t i = 0; i < cq.size(); ++i) {
            area += cq[i].X() * cq[(i + 1) % cq.size()].Y() -
                    cq[(i + 1) % cq.size()].X() * cq[i].Y();
        }
        if (std::abs(area) <= 1e-14) return false;
        if (area < 0) {
            std::reverse(clean.begin(), clean.end());
            std::reverse(cq.begin(), cq.end());
        }
        // A valid UV loop can still collapse in 3D on a singular chart.  Its
        // Newell normal must be finite, non-trivial, and agree with the CAD
        // surface normal.  Unknown/degenerate is rejection, never success.
        gp_XYZ newell(0, 0, 0);
        gp_XYZ centreUv(0, 0, 0);
        double maxEdge2 = 0.0;
        const PolyMesh& mesh = out.mesh();
        for (size_t i = 0; i < count; ++i) {
            const auto& a = mesh.vertices[clean[i]];
            const auto& b = mesh.vertices[clean[(i + 1) % count]];
            newell += gp_XYZ(a[1] * b[2] - a[2] * b[1],
                             a[2] * b[0] - a[0] * b[2],
                             a[0] * b[1] - a[1] * b[0]);
            const double dx = b[0] - a[0], dy = b[1] - a[1];
            const double dz = b[2] - a[2];
            maxEdge2 = std::max(maxEdge2, dx * dx + dy * dy + dz * dz);
            centreUv += gp_XYZ(cq[i].X(), cq[i].Y(), 0.0);
        }
        centreUv /= double(count);
        if (!std::isfinite(newell.X()) || !std::isfinite(newell.Y()) ||
            !std::isfinite(newell.Z()) || maxEdge2 <= 1e-24 ||
            newell.Modulus() <= 1e-12 * maxEdge2) {
            return false;
        }
        gp_Pnt at;
        gp_Vec du, dv;
        try {
            surface->D1(centreUv.X(), centreUv.Y(), at, du, dv);
        } catch (const Standard_Failure&) {
            return false;
        }
        const gp_Vec cadNormal = du.Crossed(dv);
        if (cadNormal.Magnitude() <= 1e-16 ||
            gp_Vec(newell).Dot(cadNormal) <=
                1e-8 * newell.Modulus() * cadNormal.Magnitude()) {
            return false;
        }
        out.addPolygon(std::move(clean), faceId, flip);
        return true;
    };
    for (int i = bodyFirst; i < bodyLast; ++i) {
        for (int j = 0; j < bands; ++j) {
            if (!emitUv({grid[i][j], grid[i + 1][j],
                         grid[i + 1][j + 1], grid[i][j + 1]},
                        {guv[i][j], guv[i + 1][j],
                         guv[i + 1][j + 1], guv[i][j + 1]})) {
                return false;
            }
        }
    }
    auto orientedCap = [](const std::vector<int>& cap, int from, int to,
                          std::vector<int>& path) {
        path.clear();
        if (cap.empty()) return false;
        if (cap.front() == from && cap.back() == to) {
            path = cap;
            return true;
        }
        if (cap.front() == to && cap.back() == from) {
            path.assign(cap.rbegin(), cap.rend());
            return true;
        }
        return false;
    };
    auto closeLow = [&]() {
        if (bodyFirst == 0 && patch.capLow.size() <= 2) return true;
        std::vector<uint32_t> ids;
        std::vector<gp_Pnt2d> q;
        // A0 -> Afirst, across the proved body row, Bfirst -> B0,
        // then the exact cap B0 -> A0.
        for (int i = 0; i <= bodyFirst; ++i) {
            const int idx = patch.railA[i];
            ids.push_back(boundaryId(idx));
            q.push_back(uv[idx]);
        }
        for (int j = 1; j <= bands; ++j) {
            ids.push_back(grid[bodyFirst][j]);
            q.push_back(guv[bodyFirst][j]);
        }
        for (int i = bodyFirst - 1; i >= 0; --i) {
            const int idx = patch.railB[i];
            ids.push_back(boundaryId(idx));
            q.push_back(uv[idx]);
        }
        std::vector<int> cap;
        if (!orientedCap(patch.capLow, patch.railB.front(),
                         patch.railA.front(), cap)) {
            return false;
        }
        for (size_t i = 1; i < cap.size(); ++i) {
            ids.push_back(boundaryId(cap[i]));
            q.push_back(uv[cap[i]]);
        }
        return emitUv(std::move(ids), std::move(q));
    };
    auto closeHigh = [&]() {
        if (bodyLast == alongA && patch.capHigh.size() <= 2) return true;
        std::vector<uint32_t> ids;
        std::vector<gp_Pnt2d> q;
        // Alast -> Aend, exact cap Aend -> Bend, Bend -> Blast,
        // then back across the proved body row.
        for (int i = bodyLast; i <= alongA; ++i) {
            const int idx = patch.railA[i];
            ids.push_back(boundaryId(idx));
            q.push_back(uv[idx]);
        }
        std::vector<int> cap;
        if (!orientedCap(patch.capHigh, patch.railA.back(),
                         patch.railB.back(), cap)) {
            return false;
        }
        for (size_t i = 1; i < cap.size(); ++i) {
            ids.push_back(boundaryId(cap[i]));
            q.push_back(uv[cap[i]]);
        }
        for (int i = alongA - 1; i >= bodyLast; --i) {
            const int idx = patch.railB[i];
            ids.push_back(boundaryId(idx));
            q.push_back(uv[idx]);
        }
        for (int j = bands - 1; j >= 0; --j) {
            ids.push_back(grid[bodyLast][j]);
            q.push_back(guv[bodyLast][j]);
        }
        return emitUv(std::move(ids), std::move(q));
    };
    if (!closeLow() || !closeHigh()) {
        return false;
    }
    const std::vector<uint8_t> folds = foldedPolys(model, out.mesh());
    if (folds.size() != out.mesh().polygons.size()) return false;
    for (size_t i = firstPolygon; i < folds.size(); ++i) {
        if (folds[i]) {
            dbg("%s face %d: post-build fold census rejected polygon %zu",
                plan.sectionStrip ? "section strip" : "corridor", faceId,
                i - firstPolygon);
            return false;
        }
    }
    if (built) *built = {alongA, bands};
    dbg("%s face %d: %d equal rail spans x %d band(s), body %d..%d, "
        "caps %zu/%zu",
        plan.sectionStrip ? "section strip" : "corridor", faceId, alongA,
        bands, bodyFirst, bodyLast, patch.capLow.size(),
        patch.capHigh.size());
    dbg("%s face %d: post-build census 0 folds, 0 UV self-crossings",
        plan.sectionStrip ? "section strip" : "corridor", faceId);
    return true;
}


}  // namespace weft::mesher_impl
