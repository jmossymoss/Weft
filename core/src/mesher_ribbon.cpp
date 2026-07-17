#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// A band with exactly two sharp tips: crescents, lunes, tangent strips
// (a drill grazing a wall). Coons needs four corners and the webs fan
// these; the ladder pairs the two rails by arc fraction directly —
// quads rung by rung, grouped 5-gons absorbing count differences, the
// tips collapsing to triangles by construction.
bool planRailLadder(const TopoDS_Face& face, const Model& model,
                    FacePlan& plan) {
    int wires = 0;
    std::vector<TopoDS_Edge> order;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
        for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face);
             we.More(); we.Next()) {
            const TopoDS_Edge e = we.Current();
            if (BRep_Tool::Degenerated(e)) continue;
            double f, l;
            if (BRep_Tool::Curve(e, f, l).IsNull()) return false;
            if (model.edges.FindIndex(e) < 1) return false;
            order.push_back(e);
        }
    }
    if (wires != 1 || order.size() < 2 || order.size() > 64) return false;
    auto wireTangent = [&](const TopoDS_Edge& e, bool atWireEnd) {
        BRepAdaptor_Curve c(e);
        const bool rev = e.Orientation() == TopAbs_REVERSED;
        const double t = (atWireEnd != rev) ? c.LastParameter()
                                            : c.FirstParameter();
        gp_Pnt p;
        gp_Vec d;
        c.D1(t, p, d);
        if (rev) d.Reverse();
        return d;
    };
    int sharp = 0;
    for (size_t i = 0; i < order.size(); ++i) {
        gp_Vec a = wireTangent(order[i], true);
        gp_Vec b = wireTangent(order[(i + 1) % order.size()], false);
        if (a.Magnitude() < 1e-12 || b.Magnitude() < 1e-12) return false;
        if (a.Angle(b) > M_PI / 4.0) ++sharp;
    }
    if (sharp != 2) return false;
    plan.kind = MesherKind::RailLadder;
    plan.constrains = true;
    for (const TopoDS_Edge& e : order) {
        plan.uEdges.push_back(model.edges.FindIndex(e));
    }
    return true;
}

bool meshRailLadder(const TopoDS_Face& face, const Model& model, int faceId,
                    const std::vector<int>& solvedEdge, int radialDefault,
                    MeshBuilder& out, std::array<int, 2>* built) {
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
        return false;
    }
    if (rings.size() != 1) return false;
    const std::vector<gp_Pnt>& P = rings[0].p;
    const size_t N = P.size();
    if (N < 4) return false;
    // The two sharpest turns of the sampled outline are the tips.
    auto turn = [&](size_t i) {
        const gp_Pnt& a = P[(i + N - 1) % N];
        const gp_Pnt& b = P[i];
        const gp_Pnt& c = P[(i + 1) % N];
        gp_Vec u(a, b), v(b, c);
        if (u.Magnitude() < 1e-12 || v.Magnitude() < 1e-12) return 0.0;
        return u.Angle(v);
    };
    size_t t1 = 0, t2 = 0;
    double a1 = -1.0, a2 = -1.0;
    for (size_t i = 0; i < N; ++i) {
        const double a = turn(i);
        if (a > a1) {
            a2 = a1;
            t2 = t1;
            a1 = a;
            t1 = i;
        } else if (a > a2) {
            a2 = a;
            t2 = i;
        }
    }
    if (t1 == t2 || a2 < M_PI / 6.0) return false;  // no second tip
    const size_t lo = std::min(t1, t2), hi = std::max(t1, t2);
    std::vector<uint32_t> ids(N);
    for (size_t i = 0; i < N; ++i) ids[i] = out.addVertex(P[i], {});
    std::vector<uint32_t> A, B;
    std::vector<gp_Pnt> Ap, Bp;
    for (size_t i = lo;; i = (i + 1) % N) {
        A.push_back(ids[i]);
        Ap.push_back(P[i]);
        if (i == hi) break;
    }
    for (size_t i = hi;; i = (i + 1) % N) {
        B.push_back(ids[i]);
        Bp.push_back(P[i]);
        if (i == lo) break;
    }
    std::reverse(B.begin(), B.end());
    std::reverse(Bp.begin(), Bp.end());
    if (A.size() < 2 || B.size() < 2) return false;
    auto arcs = [](const std::vector<gp_Pnt>& pts) {
        std::vector<double> f(pts.size(), 0.0);
        for (size_t i = 1; i < pts.size(); ++i) {
            f[i] = f[i - 1] + pts[i].Distance(pts[i - 1]);
        }
        const double t = f.back() > 1e-12 ? f.back() : 1.0;
        for (double& x : f) x /= t;
        return f;
    };
    const bool aSparse = A.size() <= B.size();
    const std::vector<uint32_t>& S = aSparse ? A : B;
    const std::vector<uint32_t>& D = aSparse ? B : A;
    const std::vector<double> sf = arcs(aSparse ? Ap : Bp);
    const std::vector<double> df = arcs(aSparse ? Bp : Ap);
    const int m = int(S.size()) - 1;
    const int n = int(D.size()) - 1;
    std::vector<int> mp(m + 1);
    mp[0] = 0;
    mp[m] = n;
    for (int k = 1; k < m; ++k) {
        int j = mp[k - 1];
        while (j + 1 < n &&
               std::abs(df[j + 1] - sf[k]) <= std::abs(df[j] - sf[k])) {
            ++j;
        }
        mp[k] = j;
    }
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    const size_t polyBefore = out.mesh().polygons.size();
    for (int k = 0; k < m; ++k) {
        std::vector<uint32_t> ring2;
        if (aSparse) {
            ring2 = {S[k], S[k + 1]};
            for (int t = mp[k + 1]; t >= mp[k]; --t) ring2.push_back(D[t]);
        } else {
            for (int t = mp[k]; t <= mp[k + 1]; ++t) ring2.push_back(D[t]);
            ring2.push_back(S[k + 1]);
            ring2.push_back(S[k]);
        }
        ring2.erase(std::unique(ring2.begin(), ring2.end()), ring2.end());
        if (ring2.size() > 1 && ring2.front() == ring2.back()) {
            ring2.pop_back();
        }
        if (ring2.size() < 3) continue;
        out.addPolygon(std::move(ring2), faceId, flip);
    }
    // Primary = the number of rungs actually laddered (sparse-rail stations);
    // secondary = the dense rail's station count.
    if (built) {
        *built = {int(out.mesh().polygons.size() - polyBefore), n};
    }
    return true;
}

// ---------------------------------------------------------------------------
// Ribbon sweep: a long, thin, BENT strip (the flaregun grip / trigger-guard
// rails) whose flattened outline is non-convex, so coons rejects it and a
// transfinite blend would fold over the bend. Its two long rails are found
// by a robust anti-parallel pairing -- NOT the sharpest-corner pick, which
// the strips' 90-degree rail bends and their weak (~22 deg) notch corners
// both defeat -- then matched station-for-station and laddered into an even
// quad flow. The end caps, including notches, are webbed LOCALLY (never a
// global fan), so no inversion can propagate across the strip.

// Sample the outer wire into an ordered 3D/UV ring. `corners` receives the
// ring index at which each wire edge begins (the join candidates). When
// `solvedEdge` is null a fixed dense count per edge is used (planning-time
// geometry probe); otherwise the solved counts drive it (the border
// contract). Returns false on any unsampleable edge or a sloppy wire.
bool sampleRibbonRing(const TopoDS_Face& face, const Model& model,
                      const std::vector<int>* solvedEdge, int radialDefault,
                      std::vector<gp_Pnt>& P, std::vector<gp_Pnt2d>& UV,
                      std::vector<int>& corners,
                      const PinnedEdges* pins,
                      std::vector<int>* sampleEdges) {
    P.clear();
    UV.clear();
    corners.clear();
    if (sampleEdges) sampleEdges->clear();
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (outer.IsNull()) return false;
    int wireEdges = 0, rawEdges = 0;
    for (BRepTools_WireExplorer we(outer, face); we.More(); we.Next()) {
        ++wireEdges;
    }
    for (TopoDS_Iterator it(outer); it.More(); it.Next()) {
        if (it.Value().ShapeType() == TopAbs_EDGE &&
            !BRep_Tool::Degenerated(TopoDS::Edge(it.Value()))) {
            ++rawEdges;
        }
    }
    if (rawEdges > wireEdges) return false;  // sloppy wire: let quad-fill own it
    for (BRepTools_WireExplorer we(outer, face); we.More(); we.Next()) {
        const TopoDS_Edge edge = we.Current();
        if (BRep_Tool::Degenerated(edge)) continue;
        const int eid = model.edges.FindIndex(edge);
        int n = 0;
        if (solvedEdge && eid >= 1 && eid < int(solvedEdge->size())) {
            n = (*solvedEdge)[eid];
        }
        if (n < 1) {
            if (solvedEdge) {
                n = std::max(1, std::max(3, radialDefault) /
                                    std::max(1, wireEdges));
            } else {
                BRepAdaptor_Curve c(edge);
                const double len = GCPnts_AbscissaPoint::Length(c);
                n = std::clamp(int(len / 3.0) + 2, 2, 40);
            }
        }
        double f3, l3, f2, l2;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
        Handle(Geom2d_Curve) c2 = BRep_Tool::CurveOnSurface(edge, face, f2, l2);
        if (c3.IsNull() || c2.IsNull()) return false;
        const bool rev = edge.Orientation() == TopAbs_REVERSED;
        const double ph = closedEdgePhase(edge, model);
        corners.push_back(int(P.size()));
        for (double t : edgeSampleFractions(eid, n, ph, rev,
                                            /*includeLast=*/false, pins,
                                            &model)) {
            UV.push_back(c2->Value(f2 + (l2 - f2) * t));
            P.push_back(c3->Value(f3 + (l3 - f3) * t));
            if (sampleEdges) sampleEdges->push_back(eid);
        }
    }
    if (P.size() < 4 || corners.size() < 4) return false;
    // The ring stays in raw wire-traversal order: the rail finder works in
    // 3D, the sweep decides its winding from the surface normal, and the cap
    // arcs are addressed by corner adjacency -- all orientation-agnostic, so
    // no UV winding normalization (and its fragile index remap) is needed.
    return true;
}

struct RibbonRails {
    // Ring index ranges (into the sampled ring): railA walks forward
    // a0 -> a1, railB walks forward b0 -> b1. railA[0] and railB[1]'s
    // side are the SAME end cap (they are joined by end cap "one").
    int a0 = -1, a1 = -1, b0 = -1, b1 = -1;
    double railLen = 0, width = 0, aspect = 0, antiDot = 0;
    double score_ = -1e300;
    bool ok = false;
};

// Find the two long anti-parallel rails from the sampled ring. `corners`
// are the wire-edge join ring indices -- the only legal cut points, so the
// rails stay whole B-rep edge chains (border contract). Brute force over
// 4-corner splits: maximize rail length minus end length, gated on aspect
// ratio and anti-parallelism, so the two long sides win over the notched
// end caps regardless of how weak or spurious the corner turns are.
RibbonRails findRibbonRails(const std::vector<gp_Pnt>& P,
                            const std::vector<int>& corners) {
    RibbonRails best;
    const int N = int(P.size());
    const int n = int(corners.size());
    if (N < 4 || n < 4 || n > 40) return best;
    std::vector<double> cum(N + 1, 0.0);
    for (int i = 0; i < N; ++i) {
        cum[i + 1] = cum[i] + P[i].Distance(P[(i + 1) % N]);
    }
    const double perim = cum[N];
    auto arcLen = [&](int i, int j) {  // forward ring length from i to j
        double d = cum[j] - cum[i];
        if (j < i) d = perim - (cum[i] - cum[j]);
        return d;
    };
    auto ptAtFrac = [&](int i, int j, double f) {  // along ring i->j (fwd)
        const double target = arcLen(i, j) * f;
        double acc = 0;
        int k = i;
        while (true) {
            int kn = (k + 1) % N;
            double seg = P[k].Distance(P[kn]);
            if (acc + seg >= target || kn == j) {
                double u = seg > 1e-12 ? (target - acc) / seg : 0.0;
                u = std::clamp(u, 0.0, 1.0);
                return gp_Pnt(P[k].XYZ() * (1 - u) + P[kn].XYZ() * u);
            }
            acc += seg;
            k = kn;
            if (k == j) return P[j];
        }
    };
    // Evaluate one rail assignment (railA arc [i..j], railB arc [k..l]).
    // railB is traversed the opposite way around the loop, so railA at
    // fraction f pairs with railB at fraction 1-f; walked that way the two
    // rails advance ALONGSIDE the strip, so their local tangents should be
    // PARALLEL at every station. Absorbing an end cap into a rail makes that
    // rail veer across the strip, collapsing the alignment there -- so the
    // WORST-station alignment (not the mean) is what fences the rails off
    // from the caps, whatever the corner turns do.
    auto consider = [&](int i, int j, int k, int l) {
        const double la = arcLen(i, j), lc = arcLen(k, l);
        const double lb = arcLen(j, k), ld = arcLen(l, i);
        if (la < 1e-6 || lc < 1e-6) return;
        const double railLen = la + lc;
        if (railLen < 0.4 * perim) return;
        if (std::max(la, lc) > 4.0 * std::min(la, lc)) return;
        // Each end cap shorter than the longer rail (a cap, not a third rail).
        if (std::max(lb, ld) > std::max(la, lc)) return;
        const int S = 24;
        const double df = 0.5 / S;
        double width = 0, minW = 1e300, maxW = 0, meanAlign = 0;
        int wn = 0, an = 0;
        for (int s = 1; s < S; ++s) {
            const double f = double(s) / S;
            const gp_Pnt a0 = ptAtFrac(i, j, f);
            const gp_Pnt b0 = ptAtFrac(k, l, 1.0 - f);
            const double w = a0.Distance(b0);
            width += w;
            minW = std::min(minW, w);
            maxW = std::max(maxW, w);
            ++wn;
            gp_Vec dA(a0, ptAtFrac(i, j, std::min(1.0, f + df)));
            gp_Vec dB(b0, ptAtFrac(k, l, std::max(0.0, 1.0 - f - df)));
            if (dA.Magnitude() < 1e-9 || dB.Magnitude() < 1e-9) continue;
            meanAlign += dA.Dot(dB) / (dA.Magnitude() * dB.Magnitude());
            ++an;
        }
        if (wn == 0 || an == 0) return;
        width /= wn;
        meanAlign /= an;
        if (width < 1e-6) return;
        // The two rails must run alongside each other: aligned tangents on
        // average (a bent rail dips locally, so the MEAN, not the worst,
        // station) and a roughly CONSTANT gap. A rail that veers into an end
        // cap balloons the gap there -- the width-consistency gate is what
        // fences the rails off from the caps without punishing sharp bends.
        if (meanAlign < 0.5) return;
        if (maxW > 2.6 * minW) return;
        // The two cap junctions must have real width: if a rail END pinches
        // to the far rail, that rail has veered across the strip to swallow
        // a cap corner (the notch corners on the fixture / trigger guard),
        // which would sweep into a collinear degenerate. A genuine cap holds
        // the rails a finite gap apart.
        const double wCap0 = P[i].Distance(P[l]);
        const double wCap1 = P[j].Distance(P[k]);
        if (std::min(wCap0, wCap1) < 0.2 * width) return;
        const double aspect = std::min(la, lc) / width;
        if (aspect < 3.5) return;
        // The two cells that ABUT the caps must have real area. When a rail
        // swallows a short ~90-degree cap corner (e1/e11 on the grip's flat
        // top, the notch corners on the guard), that corner lies on the cap
        // line together with its neighbour, so the abutting cell goes
        // collinear -- a zero-area quad no sweep can rescue. Rejecting the
        // split here forces the honest one: the corner stays in the cap and
        // the cap webs. (a1..b0 is cap 2, b1..a0 is cap 1; rail A runs
        // i->j forward, rail B l->k backward.)
        auto cellArea = [&](int p0, int p1, int p2, int p3) {
            gp_XYZ n = gp_Vec(P[p0], P[p1]).Crossed(gp_Vec(P[p0], P[p2]))
                           .XYZ() +
                       gp_Vec(P[p0], P[p2]).Crossed(gp_Vec(P[p0], P[p3]))
                           .XYZ();
            return 0.5 * n.Modulus();
        };
        const double capCell1 = cellArea(i, (i + 1) % N, (l - 1 + N) % N, l);
        const double capCell2 = cellArea((j - 1 + N) % N, j, k, (k + 1) % N);
        if (std::min(capCell1, capCell2) < 0.02 * width * width) return;
        // Prefer EQUAL segment counts (a clean 1:1 ladder), then the longest
        // rails: a true rail cannot extend without swallowing a cap, which
        // the alignment/width gates fence off. The segment count is the
        // ring-index span (= sum of the rail edges' sample counts).
        const int segA = (j - i + N) % N, segC = (l - k + N) % N;
        const double score = railLen - 1000.0 * std::abs(segA - segC);
        if (!best.ok || score > best.score_) {
            best.a0 = i; best.a1 = j; best.b0 = k; best.b1 = l;
            best.railLen = railLen; best.width = width;
            best.aspect = aspect; best.antiDot = meanAlign; best.ok = true;
            best.score_ = score;
        }
    };
    for (int ia = 0; ia < n; ++ia)
        for (int ib = ia + 1; ib < n; ++ib)
            for (int ic = ib + 1; ic < n; ++ic)
                for (int id = ic + 1; id < n; ++id) {
                    const int c0 = corners[ia], c1 = corners[ib],
                              c2 = corners[ic], c3 = corners[id];
                    // Pairing 1: rails = (c0..c1),(c2..c3).
                    consider(c0, c1, c2, c3);
                    // Pairing 2: rails = (c1..c2),(c3..c0).
                    consider(c1, c2, c3, c0);
                }
    return best;
}

// Planning-time gate: is this face a long thin bent ribbon the sweep should
// own? Pure geometry (no solved counts), so it memoizes with the face and
// never disturbs density. A positive only redirects a face quad-fill would
// otherwise take -- the mesh path falls straight back to quad-fill on any
// doubt, so this stays permissive about the exact rail counts.
bool ribbonDetect(const TopoDS_Face& face, const Model& model) {
    // Single outer wire only (holes stay with quad-fill / plate-web).
    int wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
    }
    std::vector<gp_Pnt> P;
    std::vector<gp_Pnt2d> UV;
    std::vector<int> corners;
    if (!sampleRibbonRing(face, model, nullptr, 16, P, UV, corners)) {
        return false;
    }
    RibbonRails r = findRibbonRails(P, corners);
    if (r.ok) {
        dbg("ribbon detect: rails found, aspect %.1f width %.2f align %.2f",
            r.aspect, r.width, r.antiDot);
    }
    return r.ok;
}

// A rectangular NOTCH bitten into one END of the ribbon (a trigger-guard /
// grip end-slot): the two rails run whole from cap to cap, and the notched
// cap opens into a rail-mouth-back-mouth-rail chain. findRibbonRails would
// SWALLOW the notch walls into the rails and sweep the pocket shut (fill);
// this recovers the TRUE rails (the two long sides, notch NOT absorbed) plus
// the four notch-corner ring indices so the sweep can CUT the pocket, its
// walls rising from the cut, instead of covering it.
struct RibbonEndNotch {
    bool ok = false;
    int a0 = -1, a1 = -1, b0 = -1, b1 = -1;  // TRUE rail ring ranges
    int mouthTop = -1, backTop = -1;         // notch corners, rail-A side
    int backBot = -1, mouthBot = -1;         // notch corners, rail-B side
};

// The wire corner ring indices carry the join candidates; a simple ribbon
// with one end notch has EXACTLY eight (2 rails + near cap + 5 far-cap: two
// mouth stubs + three walls). The notch back shows as two consecutive REFLEX
// corners in the outline; everything else is derived by adjacency.
RibbonEndNotch findRibbonEndNotch(const std::vector<gp_Pnt>& P,
                                  const std::vector<int>& corners) {
    RibbonEndNotch nc;
    const int n = int(corners.size());
    if (n != 8) return nc;
    std::vector<gp_Pnt> cp(n);
    for (int i = 0; i < n; ++i) cp[i] = P[corners[i]];
    // Average outline normal (Newell over the corner polygon) — the plane the
    // reflex turns are measured against, robust to the strip's bend.
    gp_XYZ nrm(0, 0, 0);
    for (int i = 0; i < n; ++i) {
        const gp_XYZ& a = cp[i].XYZ();
        const gp_XYZ& b = cp[(i + 1) % n].XYZ();
        nrm += gp_XYZ(a.Y() * b.Z() - a.Z() * b.Y(),
                      a.Z() * b.X() - a.X() * b.Z(),
                      a.X() * b.Y() - a.Y() * b.X());
    }
    if (nrm.Modulus() < 1e-12) return nc;
    gp_Vec un(nrm);
    un.Multiply(1.0 / nrm.Modulus());
    std::vector<int> turn(n, 0);
    for (int i = 0; i < n; ++i) {
        gp_Vec e0(cp[(i + n - 1) % n], cp[i]);
        gp_Vec e1(cp[i], cp[(i + 1) % n]);
        if (e0.Magnitude() < 1e-9 || e1.Magnitude() < 1e-9) return nc;
        const double cr = e0.Crossed(e1).Dot(un);
        turn[i] = cr > 1e-9 ? 1 : (cr < -1e-9 ? -1 : 0);
    }
    int sum = 0;
    for (int t : turn) sum += t;
    const int convex = sum >= 0 ? 1 : -1;
    // Exactly one run of two consecutive reflex corners, its neighbours
    // convex — the notch back. More than one such run: not this class.
    int rStart = -1;
    for (int i = 0; i < n; ++i) {
        if (turn[i] == -convex && turn[(i + 1) % n] == -convex &&
            turn[(i + n - 1) % n] == convex && turn[(i + 2) % n] == convex) {
            if (rStart >= 0) return nc;
            rStart = i;
        }
    }
    if (rStart < 0) return nc;
    const int backTop = rStart, backBot = (rStart + 1) % n;
    const int mouthTop = (rStart + n - 1) % n, mouthBot = (rStart + 2) % n;
    const int railAend = (rStart + n - 2) % n, railBstart = (rStart + 3) % n;
    const int nearB = (railBstart + 1) % n, nearA = (railBstart + 2) % n;
    // Exactly two corners (the near cap) span the rails' far ends the long
    // way round: guarantees single-edge rails and one near cap (the tested
    // class). Anything richer falls back to the generic sweep.
    if ((railBstart + 3) % n != railAend) return nc;
    // Rectangular pocket: mouth and back comparable width, the two side
    // walls comparable depth and non-trivial. Loose ratios — only rule out a
    // spurious pair of reflex corners that is plainly not a rectangular bite.
    const double wMouth = cp[mouthTop].Distance(cp[mouthBot]);
    const double wBack = cp[backTop].Distance(cp[backBot]);
    const double dTop = cp[mouthTop].Distance(cp[backTop]);
    const double dBot = cp[mouthBot].Distance(cp[backBot]);
    if (wMouth < 1e-6 || wBack < 1e-6 || dTop < 1e-6 || dBot < 1e-6) return nc;
    if (std::max(wMouth, wBack) > 2.5 * std::min(wMouth, wBack)) return nc;
    if (std::max(dTop, dBot) > 2.5 * std::min(dTop, dBot)) return nc;
    // The two derived RAILS must be the dominant long sides -- otherwise the
    // reflex pair is not a notch back but an ordinary curved-band corner (a
    // partial-revolution barrel wall), whose short edges would masquerade as
    // rails and collapse the sweep. Rails run cap to cap; every other outline
    // segment (near cap, mouth stubs, notch walls) is a minor feature.
    const double railALen = cp[nearA].Distance(cp[railAend]);
    const double railBLen = cp[railBstart].Distance(cp[nearB]);
    double maxOther = 0;
    for (int i = 0; i < n; ++i) {
        if (i == nearA || i == railBstart) continue;  // the two rail segments
        maxOther = std::max(maxOther, cp[i].Distance(cp[(i + 1) % n]));
    }
    if (std::min(railALen, railBLen) < 1.8 * maxOther) return nc;
    nc.a0 = corners[nearA];
    nc.a1 = corners[railAend];
    nc.b0 = corners[railBstart];
    nc.b1 = corners[nearB];
    nc.mouthTop = corners[mouthTop];
    nc.backTop = corners[backTop];
    nc.backBot = corners[backBot];
    nc.mouthBot = corners[mouthBot];
    nc.ok = true;
    return nc;
}

// Planning-time gate: does this ribbon carry an end notch the cut path owns?
bool ribbonEndNotchDetect(const TopoDS_Face& face, const Model& model) {
    int wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;
    }
    std::vector<gp_Pnt> P;
    std::vector<gp_Pnt2d> UV;
    std::vector<int> corners;
    if (!sampleRibbonRing(face, model, nullptr, 16, P, UV, corners)) {
        return false;
    }
    if (!findRibbonRails(P, corners).ok) return false;
    return findRibbonEndNotch(P, corners).ok;
}

// Ribbon sweep mesher. Returns false (fall back to quad-fill) whenever the
// strip does not resolve to two equal-count rails with cap-webbed ends --
// never ships a fold or a leak.
bool meshRibbonSweep(const TopoDS_Face& face, const Model& model, int faceId,
                     const std::vector<int>& solvedEdge, int radialDefault,
                     MeshBuilder& out, std::array<int, 2>* built,
                     const PinnedEdges* pins) {
    const size_t polygonBegin = out.mesh().polygons.size();
    std::vector<gp_Pnt> P;
    std::vector<gp_Pnt2d> UV;
    std::vector<int> corners;
    if (!sampleRibbonRing(face, model, &solvedEdge, radialDefault, P, UV,
                          corners, pins)) {
        return false;
    }
    const int N = int(P.size());
    RibbonRails r = findRibbonRails(P, corners);
    if (!r.ok) return false;
    // A rectangular end notch: the rail finder swallowed its walls (the rails
    // veer inward to the pocket back). Recover the TRUE rails so the far cap
    // stays a notched cap the cut path can open, instead of a swept-shut fill.
    const RibbonEndNotch notch = findRibbonEndNotch(P, corners);
    if (notch.ok) {
        r.a0 = notch.a0;
        r.a1 = notch.a1;
        r.b0 = notch.b0;
        r.b1 = notch.b1;
    }
    // Rail A walks a0 -> a1 forward; rail B walks b0 -> b1 forward but pairs
    // in reverse (the strip is traversed the opposite way on the far rail),
    // so railBr[0] sits on the SAME end cap as railA[0].
    std::vector<int> railA, railBr;
    for (int k = r.a0;; k = (k + 1) % N) {
        railA.push_back(k);
        if (k == r.a1) break;
        if (int(railA.size()) > N) return false;
    }
    for (int k = r.b1;; k = (k + N - 1) % N) {
        railBr.push_back(k);
        if (k == r.b0) break;
        if (int(railBr.size()) > N) return false;
    }
    const int MA = int(railA.size()) - 1;
    const int MB = int(railBr.size()) - 1;
    if (MA < 2 || MB < 2) {
        // Too few rungs for a ladder: leave it to quad-fill.
        dbg("ribbon face %d: rails %d/%d too short -> quad-fill", faceId, MA,
            MB);
        return false;
    }
    // Fractional station index of a point's nearest projection onto a rail
    // polyline -- where a notch-back corner falls between two rail stations.
    auto railStation = [&](const std::vector<int>& rail, const gp_Pnt& t) {
        double bestD = 1e300, bestF = 0;
        for (int k = 0; k + 1 < int(rail.size()); ++k) {
            gp_Vec seg(P[rail[k]], P[rail[k + 1]]);
            const double L2 = seg.SquareMagnitude();
            double u = L2 > 1e-18
                           ? gp_Vec(P[rail[k]], t).Dot(seg) / L2
                           : 0.0;
            u = std::clamp(u, 0.0, 1.0);
            const gp_Pnt foot(P[rail[k]].XYZ() * (1 - u) +
                              P[rail[k + 1]].XYZ() * u);
            const double d = foot.Distance(t);
            if (d < bestD) {
                bestD = d;
                bestF = k + u;
            }
        }
        return bestF;
    };
    // End-notch cut: stop the full-width body at the last rail station BEFORE
    // the pocket back (so no body cell straddles the cut), then tile the
    // notched cap as two side bands + a back connector with the pocket open.
    int sA = -1, sB = -1;
    bool notchCut = false;
    if (notch.ok) {
        // The pocket walls must each be a SINGLE ring segment: the cut tiles
        // them corner-to-corner, so an interior wall sample (a fine solve on
        // the wall) would be dropped and the border would not weld. When they
        // are not single-segment the cut can't stand -- and a detected notch
        // must never be swept SHUT -- so bail (quad-fill / coons carries it).
        const bool wallsSingle =
            notch.mouthTop == (r.a1 + 1) % N &&
            notch.backTop == (notch.mouthTop + 1) % N &&
            notch.backBot == (notch.backTop + 1) % N &&
            notch.mouthBot == (notch.backBot + 1) % N &&
            r.b0 == (notch.mouthBot + 1) % N;
        sA = int(std::floor(railStation(railA, P[notch.backTop])));
        sB = int(std::floor(railStation(railBr, P[notch.backBot])));
        notchCut = wallsSingle && sA >= 1 && sB >= 1 && MA - sA >= 1 &&
                   MB - sB >= 1;
        if (!notchCut) {
            // A notch we cannot cleanly cut: never fill it. Hand the face
            // back so quad-fill (or, for a coons-plan face, coons) owns it.
            dbg("ribbon face %d: notch present but uncuttable "
                "(wallsSingle=%d sA=%d sB=%d MA=%d MB=%d) -> fall back",
                faceId, wallsSingle ? 1 : 0, sA, sB, MA, MB);
            return false;
        }
    }
    // The two end caps: cap "1" joins railA[0]=a0 to railBr[0]=b1 along the
    // ring arc b1 -> a0 (forward); cap "2" joins railA[M]=a1 to railBr[M]=b0
    // along a1 -> b0. A cap with no interior ring sample is a single segment
    // (a plain quad rung); a notched cap carries interior samples and gets a
    // local web.
    const bool cap1Simple = (r.a0 == (r.b1 + 1) % N);
    const bool cap2Simple = (r.b0 == (r.a1 + 1) % N);
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    // Border positions come from the exact shared 3D edge curves, so they
    // weld to the neighbour bit-for-bit.  Keep this face's UV provenance as
    // an anchor as well: on a thin folded ribbon, projecting a polygon
    // centroid can land on the far side of the surface and falsely report an
    // otherwise clean ladder as inverted.
    std::vector<uint32_t> vid(N, UINT32_MAX);
    auto pushUv = [&](int ring) {
        if (vid[ring] == UINT32_MAX) {
            vid[ring] = out.addVertex(
                P[ring], {faceId, UV[ring].X(), UV[ring].Y()});
        }
        return vid[ring];
    };
    Handle(Geom_Surface) S = BRep_Tool::Surface(face);
    if (S.IsNull()) return false;
    // Winding decided ONCE from 3D geometry, not per-polygon UV area: a
    // freeform chart can flip the sign of a thin cell's UV area even where
    // the 3D strip is perfectly regular, which would wind adjacent quads
    // oppositely (a duplicate directed edge = the self-check trips). A
    // representative body quad's Newell normal against the surface normal
    // fixes the whole strip's hand; every polygon is then emitted CAD-out.
    auto surfN = [&](const gp_Pnt2d& uv) {
        gp_Pnt p;
        gp_Vec du, dv;
        S->D1(uv.X(), uv.Y(), p, du, dv);
        gp_Vec n = du.Crossed(dv);
        if (flip) n.Reverse();
        return n;
    };
    auto newell = [&](const std::vector<int>& ring) {
        gp_XYZ n(0, 0, 0);
        for (size_t i = 0; i < ring.size(); ++i) {
            const gp_XYZ& a = P[ring[i]].XYZ();
            const gp_XYZ& b = P[ring[(i + 1) % ring.size()]].XYZ();
            n += gp_XYZ(a.Y() * b.Z() - a.Z() * b.Y(),
                        a.Z() * b.X() - a.X() * b.Z(),
                        a.X() * b.Y() - a.Y() * b.X());
        }
        return n;
    };
    // Winding vote over the strip, not one midpoint cell.  A sharp rail bend
    // can make the two independently sampled midpoint indices straddle
    // different stations; that one bow-tie then votes opposite to every
    // honest ladder cell and flips the whole sweep.  Match several local
    // segments by normalized arc fraction and let their CAD-normal agreement
    // choose the single hand used by the body and both caps.
    bool reverseAll = false;
    {
        auto arcTable = [&](const std::vector<int>& rail) {
            std::vector<double> arc(rail.size(), 0.0);
            for (size_t i = 1; i < rail.size(); ++i) {
                arc[i] = arc[i - 1] +
                         P[rail[i - 1]].Distance(P[rail[i]]);
            }
            return arc;
        };
        const std::vector<double> aArc = arcTable(railA);
        const std::vector<double> bArc = arcTable(railBr);
        auto segmentAt = [](const std::vector<double>& arc, double frac) {
            const double target = frac * std::max(1e-12, arc.back());
            int i = 0;
            while (i + 2 < int(arc.size()) && arc[i + 1] < target) ++i;
            return i;
        };
        double vote = 0.0;
        for (int s2 = 1; s2 < 8; ++s2) {
            const double frac = s2 / 8.0;
            const int ia = segmentAt(aArc, frac);
            const int ib = segmentAt(bArc, frac);
            const std::vector<int> refQuad = {
                railA[ia], railA[ia + 1], railBr[ib + 1], railBr[ib]};
            const gp_XYZ nq = newell(refQuad);
            gp_Pnt2d c(0, 0);
            for (int idx : refQuad) {
                c.SetX(c.X() + 0.25 * UV[idx].X());
                c.SetY(c.Y() + 0.25 * UV[idx].Y());
            }
            const gp_Vec ref = surfN(c);
            if (ref.Magnitude() <= 1e-12 || nq.Modulus() <= 1e-12) continue;
            vote += gp_Vec(nq).Dot(ref) /
                    (nq.Modulus() * ref.Magnitude());
        }
        reverseAll = vote < 0.0;
    }
    int localFoldSplits = 0;
    int localFoldDetected = 0;
    auto emit = [&](std::vector<uint32_t> poly) {
        // Collapse vertices that coincide within the weld tolerance (a sharp
        // reflex station can pinch a rung to zero width): a 4-gon becomes a
        // clean triangle instead of a zero-area quad, and the dropped edge
        // was zero length so the weld is unaffected.
        std::vector<uint32_t> dd;
        for (size_t i = 0; i < poly.size(); ++i) {
            const auto& a = out.mesh().vertices[poly[i]];
            const auto& b =
                out.mesh().vertices[poly[(i + 1) % poly.size()]];
            const double d = std::hypot(std::hypot(a[0] - b[0], a[1] - b[1]),
                                        a[2] - b[2]);
            if (d > 1e-7) dd.push_back(poly[i]);
        }
        if (dd.size() < 3) return;
        auto put = [&](std::vector<uint32_t> p) {
            if (reverseAll) std::reverse(p.begin(), p.end());
            out.addPolygon(std::move(p), faceId, /*flip=*/false);
        };
        auto pnt = [&](uint32_t v) {
            const auto& a = out.mesh().vertices[v];
            return gp_Pnt(a[0], a[1], a[2]);
        };
        auto triA = [&](uint32_t a, uint32_t b, uint32_t c) {
            return gp_Vec(pnt(a), pnt(b)).Crossed(gp_Vec(pnt(a), pnt(c)))
                .Magnitude();
        };
        auto polyArea = [&](const std::vector<uint32_t>& p) {
            gp_XYZ n(0, 0, 0);
            for (size_t i = 0; i + 1 < p.size(); ++i) {
                n += gp_Vec(pnt(p[0]), pnt(p[i])).Crossed(
                         gp_Vec(pnt(p[0]), pnt(p[i + 1]))).XYZ();
            }
            return 0.5 * n.Modulus();
        };
        // A quad whose area collapses is a bowtie (a rail veering across the
        // strip at a crease or a swallowed notch corner): split it along the
        // diagonal that keeps both triangles non-degenerate.
        if (dd.size() == 4) {
            const double refW =
                0.5 * (pnt(dd[0]).Distance(pnt(dd[3])) +
                       pnt(dd[1]).Distance(pnt(dd[2])));
            if (polyArea(dd) < 1e-3 * std::max(1e-9, refW) * refW) {
                const double d02 = std::min(triA(dd[0], dd[1], dd[2]),
                                            triA(dd[0], dd[2], dd[3]));
                const double d13 = std::min(triA(dd[1], dd[2], dd[3]),
                                            triA(dd[1], dd[3], dd[0]));
                if (std::max(d02, d13) > 1e-4 * refW * refW) {
                    if (d02 >= d13) {
                        put({dd[0], dd[1], dd[2]});
                        put({dd[0], dd[2], dd[3]});
                    } else {
                        put({dd[1], dd[2], dd[3]});
                        put({dd[1], dd[3], dd[0]});
                    }
                    return;
                }
                // Collinear on BOTH diagonals: a genuine zero-width fold where
                // the strip's edge runs parallel to the rung (the grip's flat
                // top, a notch corner). No triangulation has area -- MERGE the
                // dead cell into the previous polygon across their shared rung
                // so the flat sliver disappears into a valid neighbour instead
                // of shipping a zero-area quad.
                std::vector<uint32_t> cur = dd;
                if (reverseAll) std::reverse(cur.begin(), cur.end());
                PolyMesh& m = out.mesh();
                if (!m.polygons.empty() && m.polygonFaceId.back() == faceId) {
                    const std::vector<uint32_t>& prev = m.polygons.back();
                    const size_t na = prev.size(), nb = cur.size();
                    for (size_t i = 0; i < na; ++i) {
                        const uint32_t a = prev[i], an = prev[(i + 1) % na];
                        bool merged = false;
                        for (size_t j = 0; j < nb; ++j) {
                            if (cur[j] != an || cur[(j + 1) % nb] != a) {
                                continue;
                            }
                            std::vector<uint32_t> out2;
                            for (size_t k = 0; k < na; ++k) {
                                out2.push_back(prev[(i + 1 + k) % na]);
                            }
                            for (size_t k = 0; k < nb - 2; ++k) {
                                out2.push_back(cur[(j + 2 + k) % nb]);
                            }
                            if (polyArea(out2) > 1e-9) {
                                m.polygons.back() = std::move(out2);
                                merged = true;
                            }
                            break;
                        }
                        if (merged) return;
                    }
                }
            }
        }
        put(std::move(dd));
    };
    // Cap classification. An end that carries interior ring samples is either
    // a genuinely FLAT, convex rim -- closed as ONE n-gon (Plasticity-style),
    // with the rail-end rung staying a normal body cell -- or a NOTCHED end
    // that gets a local web. A flat rim's n-gon spans only the cap arc, so the
    // body owns the rung and the rim never drags interior rung vertices out of
    // plane.
    auto arcFwd = [&](int from, int to) {
        std::vector<int> pts;
        for (int k = from;; k = (k + 1) % N) {
            pts.push_back(k);
            if (k == to) break;
            if (int(pts.size()) > N) break;
        }
        return pts;
    };
    // A flat, convex rim: every vertex within a small fraction of the rim
    // perimeter of the best-fit plane, consistent turn sign in that plane.
    auto capFlat = [&](const std::vector<int>& pts) {
        if (pts.size() < 4) return false;
        gp_XYZ nrm = newell(pts);
        const double nmod = nrm.Modulus();
        if (nmod < 1e-12) return false;
        gp_Vec un(nrm);
        un.Multiply(1.0 / nmod);
        gp_XYZ cen(0, 0, 0);
        for (int idx : pts) cen += P[idx].XYZ();
        cen /= double(pts.size());
        gp_Pnt cp(cen);
        double perim = 0, dev = 0;
        const int np = int(pts.size());
        for (int i = 0; i < np; ++i) {
            dev = std::max(dev, std::abs(gp_Vec(cp, P[pts[i]]).Dot(un)));
            perim += P[pts[i]].Distance(P[pts[(i + 1) % np]]);
        }
        if (dev > 0.03 * perim) return false;
        int sign = 0;
        for (int i = 0; i < np; ++i) {
            gp_Vec e0(P[pts[(i + np - 1) % np]], P[pts[i]]);
            gp_Vec e1(P[pts[i]], P[pts[(i + 1) % np]]);
            const double cr = e0.Crossed(e1).Dot(un);
            const int s = cr > 1e-9 ? 1 : (cr < -1e-9 ? -1 : 0);
            if (s == 0) continue;
            if (sign == 0) sign = s;
            else if (s != sign) return false;
        }
        return true;
    };
    const std::vector<int> cap1Arc = cap1Simple ? std::vector<int>()
                                                : arcFwd(r.b1, r.a0);
    const std::vector<int> cap2Arc = cap2Simple ? std::vector<int>()
                                                : arcFwd(r.a1, r.b0);
    const bool cap1Flat = !cap1Simple && capFlat(cap1Arc);
    const bool cap2Flat = !cap2Simple && capFlat(cap2Arc);
    // Emit a flat rim as ONE n-gon in NATURAL ring order (b1->..->a0 for cap
    // 1, a1->..->b0 for cap 2). That order closes on the rail-end chord in the
    // direction OPPOSITE the body cell that owns that rung, so the two weld
    // manifold; routing it through emit() applies the strip's single hand
    // (reverseAll) exactly as the body does.
    auto emitFlatCap = [&](const std::vector<int>& arc) {
        std::vector<uint32_t> ng;
        for (int idx : arc) ng.push_back(pushUv(idx));
        emit(std::move(ng));
    };
    // Body: skip only the end interval a NOTCHED cap will web. A simple or
    // flat end keeps its rail-end rung as a body cell. The two rails can carry
    // DIFFERENT station counts (one B-rep edge subdivides finer than its
    // opposite side), so each rail keeps its own body span.
    int loA = (cap1Simple || cap1Flat) ? 0 : 1;
    int loB = (cap1Simple || cap1Flat) ? 0 : 1;
    int hiA = notchCut ? sA : ((cap2Simple || cap2Flat) ? MA : MA - 1);
    int hiB = notchCut ? sB : ((cap2Simple || cap2Flat) ? MB : MB - 1);
    // If both rails are monotone in the SAME surface coordinate but their
    // trims begin/end at different parameter values, the non-overlapping
    // prefixes belong to the end caps, not the body zipper.  Align the body
    // to the common parameter interval; the generalized cap boundary below
    // then includes every trimmed-off rail segment, so the face contract is
    // unchanged.  This prevents a short rail rung from being stretched
    // diagonally across a B-spline bend.
    int alignedParamAxis = -1;
    if (!notchCut) {
        BRepAdaptor_Surface alignSurf(face);
        auto alignedTrack = [&](const std::vector<int>& rail, bool useU,
                                double period) {
            std::vector<double> q(rail.size());
            q[0] = useU ? UV[rail[0]].X() : UV[rail[0]].Y();
            for (size_t i = 1; i < rail.size(); ++i) {
                q[i] = useU ? UV[rail[i]].X() : UV[rail[i]].Y();
                if (period > 0.0) {
                    q[i] -= period *
                            std::round((q[i] - q[i - 1]) / period);
                }
            }
            return q;
        };
        const std::array<double, 2> period = {
            alignSurf.IsUPeriodic() ? alignSurf.UPeriod() : 0.0,
            alignSurf.IsVPeriodic() ? alignSurf.VPeriod() : 0.0};
        const std::array<double, 2> span = {
            std::max(1e-12, alignSurf.LastUParameter() -
                                alignSurf.FirstUParameter()),
            std::max(1e-12, alignSurf.LastVParameter() -
                                alignSurf.FirstVParameter())};
        std::array<std::vector<double>, 2> trackA, trackB;
        double bestScore = 0.0;
        for (int axis = 0; axis < 2; ++axis) {
            trackA[axis] = alignedTrack(railA, axis == 0, period[axis]);
            trackB[axis] = alignedTrack(railBr, axis == 0, period[axis]);
            auto quality = [](const std::vector<double>& q) {
                double variation = 0.0;
                for (size_t i = 1; i < q.size(); ++i) {
                    variation += std::abs(q[i] - q[i - 1]);
                }
                const double delta = q.back() - q.front();
                return std::pair<double, double>{
                    delta, variation > 1e-12
                               ? std::abs(delta) / variation
                               : 0.0};
            };
            const auto [da, ma] = quality(trackA[axis]);
            const auto [db, mb] = quality(trackB[axis]);
            if (da * db <= 0.0 || std::min(ma, mb) < 0.92) continue;
            const double dir = da > 0.0 ? 1.0 : -1.0;
            const double a0 = dir * trackA[axis][loA];
            const double a1 = dir * trackA[axis][hiA];
            const double b0 = dir * trackB[axis][loB];
            const double b1 = dir * trackB[axis][hiB];
            const double overlap = std::min(a1, b1) - std::max(a0, b0);
            const double shorter = std::min(a1 - a0, b1 - b0);
            const double coverage = std::min(std::abs(da), std::abs(db)) /
                                    span[axis];
            const double score = std::min(ma, mb) * std::min(1.0, coverage);
            if (overlap > 0.70 * std::max(1e-12, shorter) &&
                coverage >= 0.20 && score > bestScore) {
                bestScore = score;
                alignedParamAxis = axis;
            }
        }
        if (alignedParamAxis >= 0) {
            const auto& a = trackA[alignedParamAxis];
            const auto& b = trackB[alignedParamAxis];
            const double dir = (a.back() - a.front()) > 0.0 ? 1.0 : -1.0;
            auto nearest = [&](const std::vector<double>& q, int first,
                               int last, double target) {
                int best = first;
                double d = std::abs(dir * q[first] - target);
                for (int i = first + 1; i <= last; ++i) {
                    const double di = std::abs(dir * q[i] - target);
                    if (di < d) {
                        d = di;
                        best = i;
                    }
                }
                return best;
            };
            const int oldLoA = loA, oldLoB = loB;
            const int oldHiA = hiA, oldHiB = hiB;
            if (!cap1Simple && !cap1Flat) {
                const double start =
                    std::max(dir * a[loA], dir * b[loB]);
                loA = nearest(a, loA, hiA, start);
                loB = nearest(b, loB, hiB, start);
            }
            if (!cap2Simple && !cap2Flat) {
                const double finish =
                    std::min(dir * a[hiA], dir * b[hiB]);
                hiA = nearest(a, loA, hiA, finish);
                hiB = nearest(b, loB, hiB, finish);
            }
            if (hiA - loA < 2 || hiB - loB < 2) {
                loA = oldLoA;
                loB = oldLoB;
                hiA = oldHiA;
                hiB = oldHiB;
                alignedParamAxis = -1;
            } else if (std::getenv("WEFT_FOLD_DEBUG")) {
                dbg("ribbon align f%d axis=%c A %d..%d -> %d..%d, "
                    "B %d..%d -> %d..%d",
                    faceId, alignedParamAxis == 0 ? 'u' : 'v', oldLoA,
                    oldHiA, loA, hiA, oldLoB, oldHiB, loB, hiB);
            }
        }
    }
    // Zip two rails (ring-index chains RA, RB, paired 1:1 at their ends) by
    // ARC LENGTH, not by index. Index pairing twists where a sharp reflex
    // crowds the samples on one rail (the flaregun grip creases: adjacent
    // rungs jump 10->18 wide and the cell between them collapses), and it
    // cannot pair rails of unequal count at all. Advancing whichever rail lags
    // in arc fraction keeps every cell square: equal, evenly-spread rails stay
    // a pure quad ladder; where one rail is finer its extra stations BATCH
    // into the cell as a grouped n-gon (a pentagon / hexagon whose flat side
    // runs along that rail), never a fanned triangle.
    int paramRailZips = 0;
    auto zipRailPair = [&](const std::vector<int>& RA,
                           const std::vector<int>& RB) {
        const int nA = int(RA.size()) - 1, nB = int(RB.size()) - 1;
        if (nA < 1 || nB < 1) return;
        std::vector<double> gA(nA + 1, 0), gB(nB + 1, 0);
        for (int i = 1; i <= nA; ++i)
            gA[i] = gA[i - 1] + P[RA[i - 1]].Distance(P[RA[i]]);
        for (int i = 1; i <= nB; ++i)
            gB[i] = gB[i - 1] + P[RB[i - 1]].Distance(P[RB[i]]);
        // On a trimmed parametric strip the two rails often share an exact
        // CAD running coordinate even when their 3D arc lengths bunch very
        // differently around a sharp bend. Pairing those rails by unrelated
        // arc fractions can cross two adjacent rungs. Prefer the common
        // monotone surface parameter when both rails prove it geometrically;
        // arbitrary/skew ribbons retain the historic arc-length zipper.
        struct ParamTrack {
            std::vector<double> value;
            double delta = 0.0;
            double variation = 0.0;
            double monotonicity = 0.0;
        };
        auto track = [&](const std::vector<int>& rail, bool useU,
                         double period) {
            ParamTrack t;
            t.value.resize(rail.size());
            t.value[0] = useU ? UV[rail[0]].X() : UV[rail[0]].Y();
            for (size_t i = 1; i < rail.size(); ++i) {
                double q = useU ? UV[rail[i]].X() : UV[rail[i]].Y();
                if (period > 0.0) {
                    q -= period *
                         std::round((q - t.value[i - 1]) / period);
                }
                t.value[i] = q;
                t.variation += std::abs(q - t.value[i - 1]);
            }
            t.delta = t.value.back() - t.value.front();
            if (t.variation > 1e-12) {
                t.monotonicity = std::abs(t.delta) / t.variation;
            }
            return t;
        };
        BRepAdaptor_Surface railSurf(face);
        const double pU = railSurf.IsUPeriodic() ? railSurf.UPeriod() : 0.0;
        const double pV = railSurf.IsVPeriodic() ? railSurf.VPeriod() : 0.0;
        const std::array<ParamTrack, 2> aTrack = {
            track(RA, true, pU), track(RA, false, pV)};
        const std::array<ParamTrack, 2> bTrack = {
            track(RB, true, pU), track(RB, false, pV)};
        const std::array<double, 2> domainSpan = {
            std::max(1e-12, railSurf.LastUParameter() -
                                railSurf.FirstUParameter()),
            std::max(1e-12, railSurf.LastVParameter() -
                                railSurf.FirstVParameter())};
        int paramAxis = -1;
        double paramScore = 0.0;
        for (int axis = 0; axis < 2; ++axis) {
            const double mono = std::min(aTrack[axis].monotonicity,
                                         bTrack[axis].monotonicity);
            if (aTrack[axis].delta * bTrack[axis].delta <= 0.0) continue;
            const double coverage =
                std::min(std::abs(aTrack[axis].delta),
                         std::abs(bTrack[axis].delta)) /
                domainSpan[axis];
            const double score = mono * std::min(1.0, coverage);
            if (mono >= 0.92 && coverage >= 0.20 && score > paramScore) {
                paramScore = score;
                paramAxis = axis;
            }
        }
        bool absoluteParamMetric = false;
        if (paramAxis >= 0) {
            const ParamTrack& ta = aTrack[paramAxis];
            const ParamTrack& tb = bTrack[paramAxis];
            if (std::getenv("WEFT_FOLD_DEBUG")) {
                dbg("ribbon param zip f%d axis=%c A %.4f..%.4f B %.4f..%.4f "
                    "mono %.3f/%.3f",
                    faceId, paramAxis == 0 ? 'u' : 'v', ta.value.front(),
                    ta.value.back(), tb.value.front(), tb.value.back(),
                    ta.monotonicity, tb.monotonicity);
            }
            const double dir = ta.delta > 0.0 ? 1.0 : -1.0;
            const double lo = std::min(dir * ta.value.front(),
                                       dir * tb.value.front());
            const double hi = std::max(dir * ta.value.back(),
                                       dir * tb.value.back());
            const double scale = 1.0 / std::max(1e-12, hi - lo);
            for (int i = 0; i <= nA; ++i) {
                gA[i] = (dir * ta.value[i] - lo) * scale;
            }
            for (int i = 0; i <= nB; ++i) {
                gB[i] = (dir * tb.value[i] - lo) * scale;
            }
            absoluteParamMetric = true;
            ++paramRailZips;
        }
        const double LA = std::max(1e-12, gA[nA]), LB = std::max(1e-12, gB[nB]);
        auto fa = [&](int i) {
            return absoluteParamMetric ? gA[i] : gA[i] / LA;
        };
        auto fb = [&](int i) {
            return absoluteParamMetric ? gB[i] : gB[i] / LB;
        };
        int ia = 0, ib = 0;
        while (ia < nA || ib < nB) {
            if (ia >= nA) {
                // Rail A is spent: the leftover B stations close onto A's last
                // vertex as ONE boundary n-gon instead of a triangle fan.
                std::vector<uint32_t> poly = {pushUv(RA[nA])};
                for (int k = nB; k >= ib; --k) poly.push_back(pushUv(RB[k]));
                emit(std::move(poly));
                ib = nB;
                continue;
            }
            if (ib >= nB) {
                std::vector<uint32_t> poly;
                for (int k = ia; k <= nA; ++k) poly.push_back(pushUv(RA[k]));
                poly.push_back(pushUv(RB[nB]));
                emit(std::move(poly));
                ia = nA;
                continue;
            }
            const double na = fa(ia + 1), nb = fb(ib + 1);
            const double sA = na - fa(ia), sB = nb - fb(ib);
            if (std::abs(na - nb) < 0.5 * std::min(sA, sB)) {
                emit({pushUv(RA[ia]), pushUv(RA[ia + 1]), pushUv(RB[ib + 1]),
                      pushUv(RB[ib])});
                ++ia;
                ++ib;
            } else if (na < nb) {
                // Rail A finer here: batch its stations before B's next
                // station into one polygon (a grouped n-gon, flat along A).
                int ea = ia + 1;
                while (ea < nA && fa(ea + 1) < nb) ++ea;
                std::vector<uint32_t> poly;
                for (int k = ia; k <= ea; ++k) poly.push_back(pushUv(RA[k]));
                poly.push_back(pushUv(RB[ib + 1]));
                poly.push_back(pushUv(RB[ib]));
                emit(std::move(poly));
                ia = ea;
                ++ib;
            } else {
                int eb = ib + 1;
                while (eb < nB && fb(eb + 1) < na) ++eb;
                std::vector<uint32_t> poly = {pushUv(RA[ia]),
                                              pushUv(RA[ia + 1])};
                for (int k = eb; k >= ib; --k) poly.push_back(pushUv(RB[k]));
                emit(std::move(poly));
                ++ia;
                ib = eb;
            }
        }
    };
    const size_t polyBeforeBody = out.mesh().polygons.size();
    {
        std::vector<int> bodyA(railA.begin() + loA, railA.begin() + hiA + 1);
        std::vector<int> bodyB(railBr.begin() + loB, railBr.begin() + hiB + 1);
        zipRailPair(bodyA, bodyB);
    }
    // The rungs laddered along the sweep body (excluding the end caps): the
    // honest primary count for a rail sweep, which the `rail density` knob
    // (radial) drives up and down. Counted before the end-notch cut so it is
    // the pure body-rung count.
    const int bodyRungs = int(out.mesh().polygons.size() - polyBeforeBody);
    // End-notch cut. The body stopped full-width at railA[sA]/railBr[sB] (the
    // last rung wholly BEFORE the pocket). The notched cap is then three
    // pieces welded onto that rung and the pocket's B-rep walls, the pocket
    // MOUTH left open (its wall faces rise from the cut):
    //   * a MIDDLE-BACK quad from the last body rung to the back wall,
    //   * a TOP band zipped from rail A's far stations to the top wall,
    //   * a BOTTOM band zipped from the bottom wall to rail B's far stations.
    // Every band edge is either a rail segment, an interior lattice row, or a
    // notch wall sampled at its solved count, so the cut welds watertight and
    // the flow stays an even ladder that simply parts around the slot.
    if (notchCut) {
        emit({pushUv(railA[sA]), pushUv(notch.backTop), pushUv(notch.backBot),
              pushUv(railBr[sB])});
        std::vector<int> topA(railA.begin() + sA, railA.end());
        std::vector<int> topB = {notch.backTop, notch.mouthTop};
        zipRailPair(topA, topB);
        std::vector<int> botA = {notch.backBot, notch.mouthBot};
        std::vector<int> botB(railBr.begin() + sB, railBr.end());
        zipRailPair(botA, botB);
    }
    // The cap-region boundary in ring order: base-A vertex, the cap arc, then
    // base-B vertex. The closing edge base-B -> base-A is the rail-end rung the
    // body's last cell owns, so any tiling of this boundary welds to the body.
    auto capBoundary = [&](bool nearCap) {
        std::vector<int> poly;
        if (nearCap) {  // cap 1: body base -> a0 -> cap arc -> body base
            poly.push_back(railA[loA]);
            for (int i = loA - 1; i >= 0; --i) {
                poly.push_back(railA[i]);
            }
            for (int k = (r.a0 + N - 1) % N;;
                 k = (k + N - 1) % N) {
                poly.push_back(k);
                if (k == r.b1) break;
            }
            for (int i = 1; i <= loB; ++i) {
                poly.push_back(railBr[i]);
            }
        } else {  // cap 2: body end -> a1 -> cap arc -> body end
            poly.push_back(railA[hiA]);
            for (int i = hiA + 1; i <= MA; ++i) {
                poly.push_back(railA[i]);
            }
            for (int k = (r.a1 + 1) % N;; k = (k + 1) % N) {
                poly.push_back(k);
                if (k == r.b0) break;
            }
            for (int i = MB - 1; i >= hiB; --i) {
                poly.push_back(railBr[i]);
            }
        }
        return poly;
    };
    // A cap whose boundary FOLDS BACK on itself is not a flat rim but a strip
    // continuation the rail finder stopped short of (the flaregun trigger
    // guard curls past where the two rails still run parallel). Split it at the
    // fold and zip the two halves into a quad ladder instead of fanning the
    // whole loop into triangles. The two base vertices are the body's last
    // rung, so the ladder welds to the body; the halves converge at the tip
    // into one closing triangle.
    auto zipFoldedCap = [&](const std::vector<int>& bp) -> bool {
        const int n = int(bp.size());
        if (n < 8) return false;
        std::vector<double> cum(n, 0);
        for (int i = 1; i < n; ++i)
            cum[i] = cum[i - 1] + P[bp[i - 1]].Distance(P[bp[i]]);
        const double L = cum[n - 1];
        const double baseW = P[bp[0]].Distance(P[bp[n - 1]]);
        // A hairpin's boundary is far longer than its base rung is wide.
        if (L < 3.0 * std::max(1e-9, baseW)) return false;
        // Tip: the mid-path vertex farthest from the base-rung midpoint.
        gp_Pnt baseMid((P[bp[0]].XYZ() + P[bp[n - 1]].XYZ()) * 0.5);
        int tIdx = -1;
        double best = -1;
        for (int i = 1; i + 1 < n; ++i) {
            const double f = cum[i] / L;
            if (f < 0.30 || f > 0.70) continue;
            const double d = P[bp[i]].Distance(baseMid);
            if (d > best) {
                best = d;
                tIdx = i;
            }
        }
        if (tIdx < 2 || tIdx > n - 3) return false;
        std::vector<int> R1(bp.begin(), bp.begin() + tIdx + 1);
        std::vector<int> R2(bp.rbegin(), bp.rbegin() + (n - tIdx));
        // The two halves must run ALONGSIDE each other (a genuine fold), not
        // diverge: their arc lengths are comparable and the gap at matched
        // fractions stays a bounded multiple of the base width.
        const double l1 = cum[tIdx], l2 = L - cum[tIdx];
        if (std::max(l1, l2) > 2.5 * std::min(l1, l2)) return false;
        auto ptAt = [&](const std::vector<int>& R, double f) {
            double tot = 0;
            for (size_t i = 0; i + 1 < R.size(); ++i)
                tot += P[R[i]].Distance(P[R[i + 1]]);
            const double target = tot * f;
            double acc = 0;
            for (size_t i = 0; i + 1 < R.size(); ++i) {
                const double seg = P[R[i]].Distance(P[R[i + 1]]);
                if (acc + seg >= target || i + 2 == R.size()) {
                    const double u = seg > 1e-12 ? (target - acc) / seg : 0.0;
                    return gp_Pnt(P[R[i]].XYZ() * (1 - u) +
                                  P[R[i + 1]].XYZ() * u);
                }
                acc += seg;
            }
            return P[R.back()];
        };
        double maxGap = 0, minGap = 1e300;
        for (int s = 1; s < 6; ++s) {
            const double f = s / 6.0;
            const double g = ptAt(R1, f).Distance(ptAt(R2, f));
            maxGap = std::max(maxGap, g);
            minGap = std::min(minGap, g);
        }
        if (maxGap > 3.0 * std::max(1e-9, minGap)) return false;
        zipRailPair(R1, R2);
        dbg("ribbon face %d: folded cap %d pts, tip@%d", faceId, n, tIdx);
        return true;
    };
    // Ear-clip fallback for a genuinely notched (non-flat, non-folded) end.
    auto webCap = [&](bool nearCap) -> bool {
        std::vector<int> poly = capBoundary(nearCap);
        if (poly.size() < 3) return false;
        std::vector<WebPoint> ring;
        for (int idx : poly) ring.push_back({UV[idx], pushUv(idx)});
        // Order the ring so its UV winding matches the body's hand: the body
        // quads came out CAD-out; ask triangulateWeb for the same by feeding
        // it a positively-wound ring and flip=reverseAll.
        double area = 0;
        for (size_t i = 0; i < ring.size(); ++i) {
            const gp_Pnt2d& p = ring[i].uv;
            const gp_Pnt2d& q = ring[(i + 1) % ring.size()].uv;
            area += p.X() * q.Y() - q.X() * p.Y();
        }
        if (area < 0) std::reverse(ring.begin(), ring.end());
        // triangulateWeb can refuse a tiny cap whose UV ring is nearly
        // degenerate (rail ends crowd the corner). A small cap is better
        // topology as ONE n-gon cell anyway — the artist's rule: allow
        // the n-gon, don't fan it — so keep the ids and emit that when
        // the web says no.
        std::vector<uint32_t> ids;
        for (const WebPoint& wp : ring) {
            if (ids.empty() || ids.back() != wp.vert) ids.push_back(wp.vert);
        }
        while (ids.size() > 1 && ids.front() == ids.back()) ids.pop_back();
        const bool okw = triangulateWeb(std::move(ring), {}, faceId,
                                        reverseAll, out);
        dbg("ribbon face %d: cap web (%s) %zu pts -> %s", faceId,
            nearCap ? "near" : "far", poly.size(), okw ? "ok" : "FAIL");
        if (!okw && ids.size() >= 3 && ids.size() <= 8) {
            out.addPolygon(std::move(ids), faceId, reverseAll);
            dbg("ribbon face %d: cap (%s) closed as one n-gon", faceId,
                nearCap ? "near" : "far");
            return true;
        }
        return okw;
    };
    auto closeCap = [&](bool nearCap, bool simple, bool flat,
                        const std::vector<int>& flatArc) -> bool {
        if (simple) return true;
        if (flat) {
            emitFlatCap(flatArc);
            return true;
        }
        if (zipFoldedCap(capBoundary(nearCap))) return true;
        return webCap(nearCap);
    };
    if (!closeCap(/*nearCap=*/true, cap1Simple, cap1Flat, cap1Arc))
        return false;
    // The end-notch cut owns cap 2 (the notched far end); its bands already
    // closed the pocket, so the generic cap close is skipped there.
    if (!notchCut &&
        !closeCap(/*nearCap=*/false, cap2Simple, cap2Flat, cap2Arc))
        return false;
    // One sharp bend can make an arc-fraction rail pairing cross for a single
    // station even though the surrounding ladder is valid.  Judge every
    // emitted quad against this face's retained UV anchors and replace only
    // an opposing quad with the diagonal whose two triangles both agree with
    // the CAD normal.  Shared border edges are untouched; unlike the contract
    // floor this cannot spread a local kink into a face-wide triangle fan.
    BRepAdaptor_Surface foldSurf(face);
    const double foldUPeriod = foldSurf.IsUPeriodic() ? foldSurf.UPeriod()
                                                       : 0.0;
    const double foldVPeriod = foldSurf.IsVPeriodic() ? foldSurf.VPeriod()
                                                       : 0.0;
    GeomAPI_ProjectPointOnSurf foldProj;
    foldProj.Init(gp_Pnt(0, 0, 0), S);
    auto agreement = [&](const std::vector<uint32_t>& poly) {
        gp_XYZ n(0, 0, 0);
        gp_XYZ cen(0, 0, 0);
        gp_Pnt2d uv(0, 0);
        int anchored = 0;
        double uRef = 0.0, vRef = 0.0;
        for (size_t i = 0; i < poly.size(); ++i) {
            const auto& a = out.mesh().vertices[poly[i]];
            const auto& b =
                out.mesh().vertices[poly[(i + 1) % poly.size()]];
            n += gp_XYZ(a[1] * b[2] - a[2] * b[1],
                        a[2] * b[0] - a[0] * b[2],
                        a[0] * b[1] - a[1] * b[0]);
            cen += gp_XYZ(a[0], a[1], a[2]);
            const Anchor& an = out.mesh().anchors[poly[i]];
            if (an.faceId == faceId) {
                double au = an.u, av = an.v;
                if (anchored == 0) {
                    uRef = au;
                    vRef = av;
                } else {
                    if (foldUPeriod > 0.0) {
                        au -= foldUPeriod *
                              std::round((au - uRef) / foldUPeriod);
                    }
                    if (foldVPeriod > 0.0) {
                        av -= foldVPeriod *
                              std::round((av - vRef) / foldVPeriod);
                    }
                }
                uv.SetX(uv.X() + au);
                uv.SetY(uv.Y() + av);
                ++anchored;
            }
        }
        // An unknown orientation is not evidence that a cell is safe.  A
        // degenerate polygon cannot be certified against the CAD normal, so
        // make every caller take the repair/demotion path.
        if (n.Modulus() <= 1e-16) return -1.0;
        if (anchored == int(poly.size())) {
            uv.SetX(uv.X() / anchored);
            uv.SetY(uv.Y() / anchored);
        } else {
            cen /= double(poly.size());
            foldProj.Perform(gp_Pnt(cen));
            if (!foldProj.IsDone() || foldProj.NbPoints() < 1) return -1.0;
            double pu = 0.0, pv = 0.0;
            foldProj.LowerDistanceParameters(pu, pv);
            uv.SetX(pu);
            uv.SetY(pv);
        }
        const gp_Vec sn = surfN(uv);
        if (sn.Magnitude() <= 1e-16) return -1.0;
        return gp_Vec(n).Dot(sn) / (n.Modulus() * sn.Magnitude());
    };
    const size_t polygonEnd = out.mesh().polygons.size();
    std::vector<size_t> dropLocalPolys;
    for (size_t pi = polygonBegin; pi < polygonEnd; ++pi) {
        const std::vector<uint32_t> quad = out.mesh().polygons[pi];
        if (quad.size() != 4 || agreement(quad) >= 0.0) continue;
        ++localFoldDetected;
        const bool bodyCell =
            pi >= polyBeforeBody &&
            pi < polyBeforeBody + size_t(bodyRungs);
        if (std::getenv("WEFT_FOLD_DEBUG")) {
            dbg("ribbon fold f%d poly=%zu region=%s uv "
                "(%.4f,%.4f) (%.4f,%.4f) (%.4f,%.4f) (%.4f,%.4f)",
                faceId, pi,
                bodyCell ? "body" : "cap",
                out.mesh().anchors[quad[0]].u,
                out.mesh().anchors[quad[0]].v,
                out.mesh().anchors[quad[1]].u,
                out.mesh().anchors[quad[1]].v,
                out.mesh().anchors[quad[2]].u,
                out.mesh().anchors[quad[2]].v,
                out.mesh().anchors[quad[3]].u,
                out.mesh().anchors[quad[3]].v);
        }
        // A curved end-cap sliver can be nearly planar in 3D yet still turn
        // past the face normal, making its closing chord invalid. Remove that
        // chord by merging the cap into the adjacent body rung when the
        // combined boundary is CAD-out. This keeps a single bounded n-gon and
        // every exterior border edge; no triangle fan or global floor.
        bool capMerged = false;
        if (!bodyCell) {
            for (size_t ni = polyBeforeBody;
                 ni < polyBeforeBody + size_t(bodyRungs) && !capMerged;
                 ++ni) {
                if (ni >= out.mesh().polygons.size()) break;
                const auto& body = out.mesh().polygons[ni];
                for (size_t i = 0; i < body.size() && !capMerged; ++i) {
                    const uint32_t a = body[i];
                    const uint32_t b = body[(i + 1) % body.size()];
                    for (size_t j = 0; j < quad.size(); ++j) {
                        if (quad[j] != b ||
                            quad[(j + 1) % quad.size()] != a) {
                            continue;
                        }
                        std::vector<uint32_t> merged;
                        for (size_t k = 0; k < body.size(); ++k) {
                            merged.push_back(body[(i + 1 + k) % body.size()]);
                        }
                        for (size_t k = 0; k < quad.size() - 2; ++k) {
                            merged.push_back(
                                quad[(j + 2 + k) % quad.size()]);
                        }
                        if (agreement(merged) > 0.0) {
                            out.mesh().polygons[ni] = std::move(merged);
                            out.mesh().polygons[pi].clear();
                            dropLocalPolys.push_back(pi);
                            ++localFoldSplits;
                            capMerged = true;
                        }
                        break;
                    }
                }
            }
        }
        if (capMerged) continue;
        // First preserve quads: some ribbons cross a very curved surface in
        // the width direction, so one ruled cell's Newell normal is a poor
        // approximation even though both boundary rails are sound. Add a
        // short CAD-evaluated mid-rail (or quarter rails) inside this rung.
        // No boundary vertex changes, and the result remains a local strip of
        // quads rather than a face-wide fallback fan.
        bool bandRepaired = false;
        const Anchor qa0 = out.mesh().anchors[quad[0]];
        const Anchor qa1 = out.mesh().anchors[quad[1]];
        const Anchor qb1 = out.mesh().anchors[quad[2]];
        const Anchor qb0 = out.mesh().anchors[quad[3]];
        if (qa0.faceId == faceId && qa1.faceId == faceId &&
            qb0.faceId == faceId && qb1.faceId == faceId) {
            auto uvLerp = [&](const Anchor& a, const Anchor& b, double t) {
                double bu = b.u, bv = b.v;
                if (foldUPeriod > 0.0) {
                    bu -= foldUPeriod *
                          std::round((bu - a.u) / foldUPeriod);
                }
                if (foldVPeriod > 0.0) {
                    bv -= foldVPeriod *
                          std::round((bv - a.v) / foldVPeriod);
                }
                return gp_Pnt2d(a.u + t * (bu - a.u),
                                a.v + t * (bv - a.v));
            };
            for (int bands : {2, 4, 8}) {
                const size_t vertexMark = out.mesh().vertices.size();
                std::vector<uint32_t> start(bands + 1), finish(bands + 1);
                start.front() = quad[0];
                finish.front() = quad[1];
                start.back() = quad[3];
                finish.back() = quad[2];
                for (int b = 1; b < bands; ++b) {
                    const double t = double(b) / bands;
                    const gp_Pnt2d us = uvLerp(qa0, qb0, t);
                    const gp_Pnt2d uf = uvLerp(qa1, qb1, t);
                    start[b] = out.addVertex(
                        S->Value(us.X(), us.Y()),
                        {faceId, us.X(), us.Y()});
                    finish[b] = out.addVertex(
                        S->Value(uf.X(), uf.Y()),
                        {faceId, uf.X(), uf.Y()});
                }
                std::vector<std::vector<uint32_t>> cells;
                double margin = 1e300;
                for (int b = 0; b < bands; ++b) {
                    cells.push_back({start[b], finish[b], finish[b + 1],
                                     start[b + 1]});
                    margin = std::min(margin, agreement(cells.back()));
                }
                if (margin > 0.0) {
                    out.mesh().polygons[pi] = cells.front();
                    for (int b = 1; b < bands; ++b) {
                        out.addPolygon(cells[b], faceId, false);
                    }
                    ++localFoldSplits;
                    bandRepaired = true;
                    break;
                }
                out.mesh().vertices.resize(vertexMark);
                out.mesh().anchors.resize(vertexMark);
            }
        }
        if (bandRepaired) continue;
        const std::array<std::vector<uint32_t>, 2> split02 = {
            std::vector<uint32_t>{quad[0], quad[1], quad[2]},
            std::vector<uint32_t>{quad[0], quad[2], quad[3]}};
        const std::array<std::vector<uint32_t>, 2> split13 = {
            std::vector<uint32_t>{quad[1], quad[2], quad[3]},
            std::vector<uint32_t>{quad[1], quad[3], quad[0]}};
        const std::array<std::array<std::vector<uint32_t>, 2>, 2> split = {
            split02, split13};
        int best = -1;
        double bestMargin = -1e300;
        for (int d = 0; d < 2; ++d) {
            const double margin =
                std::min(agreement(split[d][0]), agreement(split[d][1]));
            if (margin > bestMargin) {
                bestMargin = margin;
                best = d;
            }
        }
        if (best >= 0 && bestMargin > 0.0) {
            out.mesh().polygons[pi] = split[best][0];
            out.addPolygon(split[best][1], faceId, false);
            ++localFoldSplits;
            continue;
        }
        // Both rail-preserving subdivision and both honest diagonals failed.
        // Do not hide the fold behind a centre fan: the caller can demote the
        // complete face to a mesher that can certify its topology.
        dbg("ribbon face %d: unrepaired fold at local polygon %zu -> demote",
            faceId, pi - polygonBegin);
        return false;
    }
    std::sort(dropLocalPolys.begin(), dropLocalPolys.end());
    dropLocalPolys.erase(
        std::unique(dropLocalPolys.begin(), dropLocalPolys.end()),
        dropLocalPolys.end());
    for (auto it = dropLocalPolys.rbegin(); it != dropLocalPolys.rend(); ++it) {
        out.mesh().polygons.erase(out.mesh().polygons.begin() + *it);
        out.mesh().polygonFaceId.erase(out.mesh().polygonFaceId.begin() + *it);
    }
    // Repairs may add cells after polygonEnd or merge a cap into a body cell.
    // Re-certify the complete route result, including those replacements,
    // before allowing RibbonSweep to own the face.
    int finalFoldCount = 0;
    for (size_t pi = polygonBegin; pi < out.mesh().polygons.size(); ++pi) {
        const auto& poly = out.mesh().polygons[pi];
        if (poly.size() < 3 || agreement(poly) <= 0.0) ++finalFoldCount;
    }
    if (finalFoldCount != 0) {
        dbg("ribbon face %d: final fold census found %d cell(s) -> demote",
            faceId, finalFoldCount);
        return false;
    }
    // Quality gate: the sweep only earns the face when the RAILS carry it --
    // an even quad ladder with the caps a bounded quad/n-gon closure. When
    // triangles outnumber every clean cell (quads plus grouped n-gons) the
    // "rails" were spurious -- a loop around a notch the fold-zip could not
    // rescue -- and quad-fill's grid+pairing is the safer result; hand it back
    // rather than ship a tri-heavy strip.
    int nq = 0, nt = 0, nn = 0;
    for (const auto& p : out.mesh().polygons) {
        if (p.size() == 3) ++nt;
        else if (p.size() == 4) ++nq;
        else ++nn;
    }
    // A short ribbon can legitimately consist of only one or two grouped
    // body cells plus a small triangulated closure at either end.  Rejecting
    // that bounded cap web merely because it contains four triangles throws
    // away the rail-aligned result and replaces it with a face-wide contract
    // floor (dozens of unrelated triangles on MP9's grip transition).  Keep
    // the conservative gate for genuinely triangle-heavy false rails, but
    // retain a small closure when the sweep did build a body and at least two
    // clean grouped cells.  The budget covers two quad-sized cap webs; larger
    // webs still take the safer fallback path.
    const bool boundedCapWeb =
        bodyRungs > 0 && nt <= 8 && (nq + nn) >= 2;
    if (nt > nq + nn && !boundedCapWeb) {
        dbg("ribbon face %d: caps web-heavy (%d tri / %d quad / %d ngon) -> "
            "quad-fill",
            faceId, nt, nq, nn);
        return false;
    }
    if (nt > nq + nn) {
        dbg("ribbon face %d: retaining bounded cap web "
            "(%d tri / %d quad / %d ngon)",
            faceId, nt, nq, nn);
    }
    dbg("ribbon face %d: rails %d/%d, width %.2f aspect %.1f, caps %s/%s, "
        "reverse=%d param-zips=%d local-folds=%d local-splits=%d",
        faceId, MA, MB, r.width, r.aspect, cap1Simple ? "quad" : "web",
        cap2Simple ? "quad" : "web", reverseAll ? 1 : 0, paramRailZips,
        localFoldDetected,
        localFoldSplits);
    if (built) *built = {bodyRungs, 1};
    return true;
}


}  // namespace weft::mesher_impl
