#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// ---------------------------------------------------------------------------
// Density matching: a union-find over edges. Each parametric face requires
// all its u-edges to share one subdivision count (and v-edges another), and
// shared edges tie neighbouring faces' groups together. Every face proposes
// its own settings; a group resolves to the max proposal unless an explicit
// per-edge override pins it.


// A revolution band and the fillets/bands it blends into carry ONE column
// count across every shared rim (columns run barrel -> fillet -> fillet ->
// sibling band unbroken). So a per-face radial override on ONE band must reach
// the whole connected blend group: densify a band alone and its blends meet a
// sparser neighbour whose structured mesher (rail-ladder / revolution grid)
// can't reconcile the two rail counts and demotes to OCCT triangulation — the
// "a set mesher must never fall back" break. From each overridden band, collect
// the barrel unit two ways — walk the TANGENT blend network (through smooth
// fillet chains, stopping at but including sibling bands) and add each band's
// one-hop column-edge blend neighbours (the sharp-attached rounded corners) —
// then stamp the same radial override on every face reached. No-op when nothing
// is overridden, so the default corpus is untouched.
void propagateBandRadialToBlendGroup(const Analysis& analysis,
                                     const std::map<int, FacePlan>& plans,
                                     GenerationSettings& settings) {
    const FaceMeshSettings& dfl = settings.defaults;
    auto isColumnMesher = [](MesherKind k) {
        return k == MesherKind::RevolutionGrid || k == MesherKind::CoonsGrid ||
               k == MesherKind::RailLadder || k == MesherKind::RibbonSweep;
    };
    auto isBand = [&](int fid) {
        auto it = plans.find(fid);
        return it != plans.end() &&
               it->second.kind == MesherKind::RevolutionGrid &&
               !it->second.bandSides.empty();
    };
    // The face's COLUMN-carrying edges: the ones whose solved count a radial
    // change moves (uEdges + both rims). A neighbour sharing one of these
    // feels the count change and must follow, or its rails disagree and its
    // mesher falls back. The band SIDES (row count) are deliberately excluded.
    auto columnEdges = [&](int fid) -> std::vector<int> {
        std::vector<int> es;
        auto it = plans.find(fid);
        if (it == plans.end()) return es;
        const FacePlan& p = it->second;
        es.insert(es.end(), p.uEdges.begin(), p.uEdges.end());
        es.insert(es.end(), p.rimLow.begin(), p.rimLow.end());
        es.insert(es.end(), p.rimHigh.begin(), p.rimHigh.end());
        return es;
    };
    // Explicit radial override on a face (differs from the model default).
    auto radialOverride = [&](int fid) -> int {
        auto it = settings.perFace.find(fid);
        if (it == settings.perFace.end() || it->second.radial == dfl.radial) {
            return 0;
        }
        return it->second.radial;
    };
    std::vector<std::pair<int, int>> seeds;  // (band fid, radial)
    for (const auto& [fid, plan] : plans) {
        if (!isBand(fid)) continue;
        const int R = radialOverride(fid);
        if (R > 0) seeds.push_back({fid, R});
    }
    if (seeds.empty()) return;  // default path: nothing to propagate

    auto isBlendFillet = [&](int fid) {
        auto it = plans.find(fid);
        if (it == plans.end()) return false;
        const MesherKind k = it->second.kind;
        return k == MesherKind::RailLadder || k == MesherKind::RibbonSweep ||
               k == MesherKind::CoonsGrid;
    };
    std::map<int, int> target;     // grouped face -> radial the group carries
    std::map<int, int> driverPin;  // band driver edge -> forced column count
    for (const auto& [seedFid, R] : seeds) {
        std::set<int> group{seedFid};
        std::vector<int> frontier{seedFid};
        // Walk the TANGENT blend network: bands join their coaxial siblings
        // through smooth fillet chains (the barrel's two walls meet through
        // rounded tori). Expand through blends, stop at (but include) sibling
        // bands so the group stays the barrel unit and never runs the whole
        // coaxial stack.
        while (!frontier.empty()) {
            const int f = frontier.back();
            frontier.pop_back();
            if (f != seedFid && isBand(f)) continue;
            if (f < 1 || f > int(analysis.faces.size())) continue;
            for (int eid : analysis.faces[f - 1].edgeIds) {
                if (eid < 1 || eid > int(analysis.edges.size())) continue;
                if (analysis.edges[eid - 1].convexity != EdgeConvexity::Smooth) {
                    continue;
                }
                for (int nf : analysis.edges[eid - 1].faceIds) {
                    if (nf == f || group.count(nf)) continue;
                    auto it = plans.find(nf);
                    if (it == plans.end() || !isColumnMesher(it->second.kind)) {
                        continue;
                    }
                    group.insert(nf);
                    frontier.push_back(nf);
                }
            }
        }
        // A band also shares its RIM count with the little rounded corners that
        // sit on it across a SHARP edge (r=3 fillet cylinders meshed as rail
        // ladders). They aren't tangent, so the smooth walk misses them, yet a
        // rim-count bump breaks their ladder — pull in each band's one-hop
        // column-edge blend neighbours (no further expansion, so the group
        // can't leak down the next feature's blend chain).
        std::vector<int> bands;
        for (int g : group)
            if (isBand(g)) bands.push_back(g);
        for (int b : bands) {
            for (int eid : columnEdges(b)) {
                if (eid < 1 || eid > int(analysis.edges.size())) continue;
                for (int nf : analysis.edges[eid - 1].faceIds) {
                    if (nf != b && isBlendFillet(nf)) group.insert(nf);
                }
            }
        }
        for (int g : group) {
            auto it = target.find(g);
            target[g] = it == target.end() ? R : std::max(it->second, R);
        }
        // The grouped bands share ONE column count, not just one radial: their
        // wrap fractions differ slightly (0.924 vs 0.927), so radial*wrap can
        // round to DIFFERENT nu (18 vs 19 at radial 20) and the pinned cut rims
        // then disagree edge-for-edge and a band fails its border contract.
        // Force every band's driver to the group's max round(radial*wrap).
        int commonNu = 0;
        for (int b : bands) {
            const double wrap = plans.find(b)->second.bandWrapFrac;
            commonNu = std::max(
                commonNu, std::max(3, int(std::lround(std::max(3, R) * wrap))));
        }
        for (int b : bands) {
            const int drv = plans.find(b)->second.bandDriver;
            if (drv >= 1) {
                auto it = driverPin.find(drv);
                driverPin[drv] =
                    it == driverPin.end() ? commonNu
                                          : std::max(it->second, commonNu);
            }
        }
        dbg("blend-group: band %d radial %d propagated to %zu faces", seedFid,
            R, group.size());
    }
    // Stamp each grouped face with the larger of its own explicit radial and
    // the group target, so a lone override reaches its whole group whether it
    // raises OR lowers the count (the seed and its blends stay equal either
    // way) while a user who set several faces keeps the highest.
    for (const auto& [g, R] : target) {
        const int keep = radialOverride(g);  // this face's own explicit radial
        FaceMeshSettings& s = settings.perFace.count(g)
                                  ? settings.perFace[g]
                                  : (settings.perFace[g] = dfl);
        s.radial = std::max(R, keep);
    }
    for (const auto& [e, c] : driverPin) {
        auto it = settings.perEdge.find(e);
        settings.perEdge[e] = it == settings.perEdge.end()
                                  ? c
                                  : std::max(it->second, c);
    }
}

DensitySolution solveDensity(const Model& model, std::map<int, FacePlan>& plans,
                             const GenerationSettings& settings,
                             GenerationCache* cache) {
    DensitySolution sol(model.edgeCount());

    for (const auto& [fid, plan] : plans) {
        if (!plan.constrains) continue;
        // Loop-based plans (plate webs, minimal planar) never tie their
        // edges together: each hole/border edge solves on its own (the
        // bore through a hole drives that hole).
        if (!plan.loops.empty()) continue;
        // Orthogonal trims share one station lattice, but a short notch
        // step must not be unioned with the full-width rim (that would give
        // every tiny step the complete cylinder count). Their explicit
        // station pins reconcile the shared borders later.
        if (plan.orthogonalTrimGrid) continue;
        // Rims made of several edges (a T-junction interrupts one side's
        // circle) must match the opposite rim in TOTAL, not per edge —
        // uniting them would hand every arc the full circle's count and
        // double that rim. Their totals equalize after the solve.
        if (plan.linkRims &&
            plan.rimLow.size() <= 1 && plan.rimHigh.size() <= 1) {
            sol.groups.unite(plan.uEdges);
        }
        sol.groups.unite(plan.vEdges);
    }

    // Co-circular arcs are ONE ring to the eye: STEP kernels split
    // closed bores into half-cylinders, and when each half's rim arc
    // solves independently the two halves land on different counts and
    // the ring breaks visually at the split lines. Arcs lying on the
    // same circle (same centre, axis, radius) with near-equal spans
    // unite into one group — both halves then carry the same count and
    // their columns meet exactly at the seams. Proportional splits
    // (a quarter against a three-quarter arc) keep their own counts.
    {
        std::map<std::array<long long, 7>, std::vector<std::pair<int, double>>>
            rings;
        for (int eid = 1; eid <= model.edgeCount(); ++eid) {
            const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(e)) continue;
            double f, l;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
            if (c.IsNull()) continue;
            GeomAdaptor_Curve gc(c, f, l);
            if (gc.GetType() != GeomAbs_Circle) continue;
            if (l - f >= 2.0 * M_PI - 1e-9) continue;  // full circles solo
            const gp_Circ circ = gc.Circle();
            auto q = [](double v) { return llround(v * 1e6); };
            const gp_Pnt o = circ.Location();
            gp_Dir d = circ.Axis().Direction();
            if (d.Z() < 0 ||
                (d.Z() == 0 && (d.Y() < 0 || (d.Y() == 0 && d.X() < 0)))) {
                d.Reverse();  // sign-normalize so mirrored arcs meet
            }
            rings[{q(o.X()), q(o.Y()), q(o.Z()), q(d.X()), q(d.Y()),
                   q(d.Z()), q(circ.Radius())}]
                .push_back({eid, l - f});
        }
        for (auto& [key, arcs] : rings) {
            if (arcs.size() < 2) continue;
            double mn = 1e300, mx = 0.0;
            for (const auto& [eid, span] : arcs) {
                mn = std::min(mn, span);
                mx = std::max(mx, span);
            }
            if (mx > 1.3 * mn) continue;
            std::vector<int> ids;
            ids.reserve(arcs.size());
            for (const auto& [eid, span] : arcs) ids.push_back(eid);
            sol.groups.unite(ids);
        }
    }

    // A face with an explicit per-face override PINS its groups: the user
    // asked for that density by name, so it must not be silently outvoted
    // by neighbours' defaults. Groups touched only by defaulted faces
    // resolve to the max proposal as before; several overrides sharing a
    // group still resolve by max among themselves.
    std::map<int, int> facePinned;
    // Global budget knob: counts scale before the group solve (floors
    // reapply after), so one slider re-budgets the whole model.
    const double dScale = std::clamp(settings.densityScale, 0.05, 20.0);
    auto propose = [&](const std::vector<int>& edges, int count,
                       bool overridden) {
        if (edges.empty()) return;
        // Explicit counts stay EXACT — the budget knob never rescales a
        // number the user typed. Straight lines don't scale either:
        // extra spans along a ruling buy no fidelity (a cylinder's
        // axial count is the user's choice, not the budget's).
        bool curved = false;
        for (int e : edges) {
            double cf, cl;
            Handle(Geom_Curve) cc =
                BRep_Tool::Curve(TopoDS::Edge(model.edges(e)), cf, cl);
            if (cc.IsNull()) continue;
            GeomAdaptor_Curve gcc(cc, cf, cl);
            if (gcc.GetType() != GeomAbs_Line) {
                curved = true;
                break;
            }
        }
        if (!overridden && curved) {
            count = std::max(1, int(std::lround(count * dScale)));
        }
        int root = sol.groups.find(edges[0]);
        auto [it, inserted] = sol.groupCount.try_emplace(root, count);
        if (!inserted) it->second = std::max(it->second, count);
        if (overridden) {
            auto [pit, pIns] = facePinned.try_emplace(root, count);
            if (!pIns) pit->second = std::max(pit->second, count);
        }
    };
    // Curvature-adaptive proposals: an edge's count comes from tangential-
    // deflection sampling of its curve under the proposing face's chord +
    // angle tolerances — big arcs get more segments than small ones,
    // straight edges get 1. Memoized per (edge, tolerances) for this solve.
    // Model diagonal for the relative-deviation law, computed on first
    // use only: BRepBndLib::Add can warm OCCT triangulation caches and
    // change later fallback output, so profiles that never take the
    // relative path must never query the box.
    double adDiag = cache ? cache->modelDiagonal : -1.0;
    auto adModelDiag = [&]() {
        if (adDiag < 0) {
            adDiag = 1e-9;
            Bnd_Box bb;
            BRepBndLib::Add(model.shape, bb);
            if (!bb.IsVoid()) {
                double x0, y0, z0, x1, y1, z1;
                bb.Get(x0, y0, z0, x1, y1, z1);
                adDiag = std::max(
                    1e-9, gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1)));
            }
            if (cache) cache->modelDiagonal = adDiag;
        }
        return adDiag;
    };
    std::map<std::array<long long, 4>, int> localAdCache;
    auto& adCache = cache ? cache->adaptiveEdgeCounts : localAdCache;
    auto adaptiveCount = [&](int eid, const FaceMeshSettings& s) {
        const std::array<long long, 4> key = {
            eid,
            static_cast<long long>(s.chordTolerance * 1e9) * 2 +
                (s.relativeDeviation ? 1 : 0),
            static_cast<long long>(s.angleToleranceDeg * 1e6),
            static_cast<long long>(settings.defaults.angleToleranceDeg *
                                   1e6)};
        auto it = adCache.find(key);
        if (it != adCache.end()) return it->second;
        int n = 1;
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (!BRep_Tool::Degenerated(edge)) {
            double f, l;
            if (!BRep_Tool::Curve(edge, f, l).IsNull()) {
                BRepAdaptor_Curve c(edge);
                double ang =
                    std::max(1.0, s.angleToleranceDeg) * M_PI / 180.0;
                double chord = std::max(1e-9, s.chordTolerance);
                if (s.relativeDeviation) {
                    // RADIUS-SCALED density (artist request 2026-07-11).
                    // The chord budget is relative to the MODEL, not the
                    // edge: at the 0.1 default every edge may sag 0.1% of
                    // the bounding diagonal. Segment counts then grow as
                    // sqrt(radius) — a big barrel ring genuinely carries
                    // more segments than a small bore, a 0.8-radius edge
                    // round crosses in 2 instead of the angle floor's 4,
                    // and fillet-owned freeform rims stop out-sampling
                    // the primitives next to them (the old edge-extent
                    // basis gave a 500mm and a 5mm bore the SAME ring
                    // and pushed fillet bsplines through a 4x tighter
                    // gate — the demo read inverted, dense struts under
                    // a coarse barrel). Uniform scaling of the whole
                    // model still reproduces identical topology; only
                    // features WITHIN a model contrast now.
                    chord = std::max(
                        chord * 0.01 * adModelDiag(), 1e-9);
                    // Deviation is the master knob here: the 28-degree
                    // default no longer floors every ring at 13. A
                    // 60-degree kink guard still catches tangent breaks,
                    // and an angle the user TIGHTENED below the default
                    // is honored as before.
                    if (!(s.angleToleranceDeg <
                          settings.defaults.angleToleranceDeg - 1e-9)) {
                        ang = std::max(ang, 60.0 * M_PI / 180.0);
                    }
                }
                const bool closedLoop =
                    c.Value(c.FirstParameter())
                        .Distance(c.Value(c.LastParameter())) < 1e-9;
                if (s.relativeDeviation && closedLoop &&
                    c.GetType() != GeomAbs_Line) {
                    // Closed curved loops follow the RING'S SIZE — the
                    // unrolled radius len/2pi in closed form — not the
                    // worst local bend. A tilted strut's blend rim bends
                    // 2-3x tighter on its downhill side; per-interval
                    // counting would drive that collar as dense as a
                    // barrel twice its size and the artist's hierarchy
                    // (big barrel > strut collar > small bore) would
                    // collapse again.
                    double len = 0.0;
                    try {
                        len = GCPnts_AbscissaPoint::Length(c);
                    } catch (const Standard_Failure&) {
                    }
                    if (len > 1e-12) {
                        const double rEff = len / (2.0 * M_PI);
                        const double half =
                            std::acos(1.0 - std::min(chord / rEff, 1.0));
                        n = half > 1e-9 ? int(std::ceil(M_PI / half - 1e-9))
                                        : 256;
                        // The (possibly user-tightened) turn budget still
                        // bounds each segment's arc.
                        n = std::max(n, int(std::ceil(2.0 * M_PI / ang -
                                                      1e-9)));
                    }
                    n = std::clamp(n, 6, 256);
                } else {
                    try {
                        n = std::clamp(stableDeflectionCount(c, ang, chord),
                                       1, 256);
                    } catch (const Standard_Failure&) {
                    }
                    // Closed edges (full circles) keep a sane ring floor.
                    if (closedLoop) n = std::max(n, 6);
                }
            }
        }
        adCache[key] = n;
        return n;
    };
    std::map<std::array<long long, 4>, int> localSurfaceCounts;
    auto& surfaceCounts =
        cache ? cache->orthogonalSurfaceCounts : localSurfaceCounts;
    auto surfaceCount = [&](int fid, const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const FaceMeshSettings& s, bool alongU) {
        const std::array<long long, 4> key = {
            fid, alongU ? 1LL : 0LL,
            static_cast<long long>(s.chordTolerance * 1e9),
            s.relativeDeviation ? 1LL : 0LL};
        auto it = surfaceCounts.find(key);
        if (it != surfaceCounts.end()) return it->second;
        const int count =
            orthogonalSurfaceDivisionFloor(face, surf, s, alongU);
        surfaceCounts[key] = count;
        return count;
    };
    auto proposeSurface = [&](int eid, int count) {
        if (eid <= 0) return;
        count = std::max(1, int(std::lround(count * dScale)));
        const int root = sol.groups.find(eid);
        auto [it, inserted] = sol.groupCount.try_emplace(root, count);
        if (!inserted) it->second = std::max(it->second, count);
    };
    // A per-face COUNT the user typed (a density field that differs from the
    // model default) is AUTHORITATIVE: it REPLACES the adaptive curvature
    // floor for that face rather than acting as a floor under it. Without
    // this, raising gridU/axial in adaptive mode does nothing until the
    // number clears the curvature count (the coons-gridU / torus-axial dead
    // zone). Tolerance-only overrides (chord/angle) leave this false so they
    // keep refining adaptively.
    bool curCountOverride = false;  // set per-face in the loop below
    // Propose `flat` onto a set, or — adaptive — each edge's own
    // curvature count with `floorA` as the minimum.
    auto proposeSet = [&](const std::vector<int>& edges, int flat,
                          int floorA, bool adaptive,
                          const FaceMeshSettings& s, bool overridden) {
        if (!adaptive || curCountOverride) {
            propose(edges, flat, overridden);
            return;
        }
        for (int eid : edges) {
            // Floor applies AFTER scaling (propose scales): a ring floor
            // of 6 must survive a 0.5x budget.
            propose({eid}, std::max(int(std::lround(floorA / dScale)),
                                    adaptiveCount(eid, s)),
                    overridden);
        }
    };

    for (const auto& [fid, plan] : plans) {
        if (!plan.constrains) continue;
        const FaceMeshSettings& s = settings.forFace(fid);
        const bool overridden = settings.perFace.count(fid) > 0;
        // Did the user type an explicit COUNT on this face (vs only a
        // tolerance/flag)? If so its proposals are exact, not adaptive floors.
        const FaceMeshSettings& dfl = settings.defaults;
        // filletLoops deliberately NOT in this list: the across axis of
        // a blend strip is never adaptive (adU/adV below), so the loops
        // knob flows through without this hammer — and including it
        // meant a loops-only override KILLED the along axis's
        // adaptivity too, collapsing a 24-station edge round to
        // gridU's default 1 the moment the user touched loops.
        curCountOverride =
            overridden &&
            (s.gridU != dfl.gridU || s.gridV != dfl.gridV ||
             s.radial != dfl.radial || s.axial != dfl.axial);
        const bool gridUOverride =
            overridden && s.gridU != dfl.gridU;
        const bool gridVOverride =
            overridden && s.gridV != dfl.gridV;
        if (plan.orthogonalTrimGrid) {
            const TopoDS_Face of = TopoDS::Face(model.faces(fid));
            BRepAdaptor_Surface os(of);
            const double us = std::max(
                1e-12, os.LastUParameter() - os.FirstUParameter());
            const double vs = std::max(
                1e-12, os.LastVParameter() - os.FirstVParameter());
            const bool drum = plan.kind == MesherKind::RevolutionGrid;
            auto spanOf = [&](int eid, bool u) {
                double f, l;
                Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(
                    TopoDS::Edge(model.edges(eid)), of, f, l);
                if (pc.IsNull()) return 0.0;
                double lo = 1e300, hi = -1e300;
                for (int k = 0; k <= 8; ++k) {
                    const gp_Pnt2d p = pc->Value(f + (l - f) * k / 8.0);
                    const double x = u ? p.X() : p.Y();
                    lo = std::min(lo, x); hi = std::max(hi, x);
                }
                return std::max(0.0, hi - lo);
            };
            for (int e : plan.uEdges) {
                const double frac = spanOf(e, true) /
                    (drum ? 2.0 * M_PI : us);
                const int n = std::max(1, int(std::lround(
                    (drum ? std::max(3, s.radial) : std::max(1, s.gridU)) *
                    frac)));
                proposeSet({e}, n, 1, s.adaptive, s, overridden);
            }
            for (int e : plan.vEdges) {
                const double frac = spanOf(e, false) / vs;
                const int n = std::max(1, int(std::lround(
                    std::max(1, drum ? s.axial : s.gridV) * frac)));
                proposeSet({e}, n, 1, s.adaptive, s, overridden);
            }
            if (!drum && !plan.orthogonalLocalComb && s.adaptive) {
                const int surfaceU =
                    gridUOverride ? 1 : surfaceCount(fid, of, os, s, true);
                const int surfaceV =
                    gridVOverride ? 1 : surfaceCount(fid, of, os, s, false);
                if (!gridUOverride) {
                    proposeSurface(plan.orthogonalDriverU, surfaceU);
                }
                if (!gridVOverride) {
                    proposeSurface(plan.orthogonalDriverV, surfaceV);
                }
                if (surfaceU > 1 || surfaceV > 1) {
                    dbg("orthogonal surface floor face %d: u=%d v=%d",
                        fid, surfaceU, surfaceV);
                }
            }
            if (drum && plan.orthogonalDrumComb &&
                plan.orthogonalFeatureCount > 0 &&
                plan.orthogonalDriverU > 0) {
                // A repeated groove/castellation needs at least two angular
                // intervals per detected rail/cap piece. Below that, several
                // trim endpoints quantize into one primitive column and the
                // default-density mesh visibly drops groove spans. This is a
                // structural floor derived from the B-rep trim count, not an
                // MP9 id or a hidden radial=32 dependency. Like the closed
                // ring floor above, pre-divide by densityScale so the floor
                // survives the proposal scaler.
                const int featureFloor =
                    std::max(3, 2 * plan.orthogonalFeatureCount);
                const int proposal = overridden
                    ? featureFloor
                    : std::max(1, int(std::ceil(featureFloor / dScale)));
                propose({plan.orthogonalDriverU}, proposal, overridden);
            }
        } else if (!plan.loops.empty()) {
            // Explicit boundary control: a TOTAL vertex count around the
            // outer loop, distributed across its edges by arc length and
            // pinned — it drives the neighbouring walls' shared edges too.
            const bool boundarySet = s.boundary > 0 && !plan.loops[0].empty();
            if (boundarySet) {
                const std::vector<int>& outer = plan.loops[0];
                std::vector<double> lens(outer.size(), 1.0);
                double sum = 0;
                for (size_t i = 0; i < outer.size(); ++i) {
                    BRepAdaptor_Curve c(
                        TopoDS::Edge(model.edges(outer[i])));
                    lens[i] = std::max(1e-12,
                                       GCPnts_AbscissaPoint::Length(c));
                    sum += lens[i];
                }
                int total = std::max(int(outer.size()), s.boundary);
                int assigned = 0;
                for (size_t i = 0; i < outer.size(); ++i) {
                    int share =
                        i + 1 == outer.size()
                            ? std::max(1, total - assigned)
                            : std::max(1,
                                       int(std::floor(total * lens[i] / sum +
                                                      0.5)));
                    assigned += share;
                    propose({outer[i]}, share, /*overridden=*/true);
                }
            }
            // Every other border edge proposes independently. Plate webs
            // share the radial default out per loop (a one-edge hole
            // circle gets all of it; a bore's larger proposal still wins).
            // Minimal planar proposes the floor — flattest possible — and
            // lets the neighbours drive any edge that needs more; it must
            // never PIN a shared edge down, even as an explicit override.
            for (size_t li = boundarySet ? 1 : 0; li < plan.loops.size();
                 ++li) {
                const auto& loop = plan.loops[li];
                bool minimal = plan.kind == MesherKind::MinimalNGon;
                int per = minimal ? 1
                                  : std::max(1, std::max(3, s.radial) /
                                                    int(loop.size()));
                for (int eid : loop) {
                    if (minimal) {
                        propose({eid}, 1, false);
                    } else {
                        proposeSet({eid}, per, 1, s.adaptive, s, overridden);
                    }
                }
            }
        } else if (plan.kind == MesherKind::PlanarGrid ||
            plan.kind == MesherKind::CoonsGrid ||
            plan.kind == MesherKind::MinimalNGon ||
            plan.kind == MesherKind::RingJunction) {
            int nu, nv;
            if (plan.isFillet && (plan.kind == MesherKind::CoonsGrid ||
                                  plan.kind == MesherKind::PlanarGrid)) {
                // SEMANTIC knobs on blend strips (artist report
                // 2026-07-11): "fillet loops" is ALWAYS the across-the-
                // blend count and "grid u" is ALWAYS the along count,
                // whichever patch axis each lands on. Mirror-twin strips
                // rotate their coons sides by one (the wire starts on a
                // different edge), so the raw exposure made the same
                // geometric direction ride grid u on one twin and grid v
                // on its mirror. grid v is inert here (the app hides
                // it). Defaults are symmetric (gridU == gridV), so
                // unoverridden output is unchanged.
                nu = std::max(1, plan.acrossIsU ? s.filletLoops : s.gridU);
                nv = std::max(1, plan.acrossIsU ? s.gridU : s.filletLoops);
            } else {
                nu = std::max(1, plan.isFillet && plan.acrossIsU
                                     ? s.filletLoops : s.gridU);
                nv = std::max(1, plan.isFillet && !plan.acrossIsU
                                     ? s.filletLoops : s.gridV);
            }
            if (plan.kind == MesherKind::CoonsGrid &&
                plan.coonsPolePatch && !s.adaptive) {
                // A collapsed revolution cap is not a generic gridU=1
                // patch: its surviving long direction spans a real fraction
                // of a turn. Preserve that turn at the radial budget even in
                // the fast non-adaptive default, plus enough profile rows to
                // approach the pole smoothly. Adaptive mode already derives
                // these counts from curvature and remains untouched.
                const int around = std::max(
                    4, int(std::lround(std::max(3, s.radial) *
                                      plan.coonsPoleTurnFraction)));
                const int profile = std::max(3, (around + 1) / 2);
                if (plan.coonsPoleAroundIsU) {
                    nu = std::max(nu, around);
                    nv = std::max(nv, profile);
                } else {
                    nv = std::max(nv, around);
                    nu = std::max(nu, profile);
                }
            }
            // Hole cutouts demand lattice lines the border edges alone
            // would never propose (a straight edge proposes 1); the
            // floors size cells to the holes so lines run border to
            // border and the collar absorbs the ring locally.
            nu = std::max(nu, plan.insertMinU);
            nv = std::max(nv, plan.insertMinV);
            // Support loops across a blend stay a deliberate choice; the
            // other directions adapt to their edges' curvature.
            bool adU = s.adaptive && !(plan.isFillet && plan.acrossIsU);
            bool adV = s.adaptive && !(plan.isFillet && !plan.acrossIsU);
            proposeSet(plan.uEdges, nu, nu, adU, s, overridden);
            proposeSet(plan.vEdges, nv, nv, adV, s, overridden);
            // Ring junction: the concentric loops need the SAME angular count
            // on the inner circle and the outer rectangle row, but the circle
            // (a full bore) otherwise solves to its own adaptive ring count
            // and the border-contract rejects the n=2*(nu+nv) the junction
            // samples it at — demoting the face and dropping every loop. When
            // the user explicitly asked for ring loops, pin the circle (and
            // its bore neighbour) to the junction's angular count so the rings
            // survive watertight; the loop COUNT then rides junctionRings and
            // the angular resolution rides gridU/gridV. Gated on the explicit
            // override, so a defaulted ring junction keeps its historical path
            // and default output stays byte-identical.
            if (plan.kind == MesherKind::RingJunction &&
                plan.circleEdgeId > 0 && overridden &&
                s.junctionRings != settings.defaults.junctionRings) {
                propose({plan.circleEdgeId}, 2 * (nu + nv),
                        /*overridden=*/true);
            }
            // Chained Coons sides: every piece proposes on its own; the
            // chain pass below reconciles opposite sides by sum. But a
            // fillet's ACROSS side carries filletLoops as a CHAIN TOTAL,
            // not per edge — a k-edge across chain proposing filletLoops on
            // each piece would sum to k*filletLoops and mismatch the
            // opposite (single-edge) across side, pentagonating the strip.
            // Distribute the loop count over the across chain's pieces so
            // both across sides carry the same total.
            for (int sd = 0; sd < 4; ++sd) {
                const int sideTotal = sd % 2 == 0 ? nu : nv;
                const bool adSide = sd % 2 == 0 ? adU : adV;
                const std::vector<int>& sedges = plan.coonsSides[sd];
                const bool acrossSide =
                    plan.isFillet && ((plan.acrossIsU && sd % 2 == 0) ||
                                      (!plan.acrossIsU && sd % 2 == 1));
                if (acrossSide && sedges.size() > 1) {
                    const int k = int(sedges.size());
                    for (int i = 0; i < k; ++i) {
                        const int share =
                            sideTotal / k + (i < sideTotal % k ? 1 : 0);
                        proposeSet({sedges[i]}, std::max(1, share), 1, false,
                                   s, overridden);
                    }
                } else {
                    for (int e : sedges) {
                        proposeSet({e}, sideTotal, 1, adSide, s, overridden);
                    }
                }
            }
        } else if (plan.kind == MesherKind::AnnulusRing) {
            // Both loops are rings; they solve independently (their own
            // neighbours usually drive them).
            proposeSet(plan.uEdges, std::max(3, s.radial), 3, s.adaptive, s,
                       overridden);
            proposeSet(plan.vEdges, std::max(3, s.radial), 3, s.adaptive, s,
                       overridden);
        } else if (plan.kind == MesherKind::RailLadder &&
                   [&]() -> bool {
                       // Rail-ladder blend strips lying ON a revolution
                       // surface (slot-end roundings, junction beads):
                       // the radial that reaches them — usually stamped
                       // band-wide by the blend-group propagation — is
                       // counted around the NEIGHBOUR BAND's axis, not
                       // this little cylinder's own turn. Pinning any
                       // self-derived number on the shared arcs fought
                       // the band's pitch and wrapped the flaregun
                       // sleeve in a dense absorber band (verbatim 50
                       // first, own-axis share 15 after). So: edges
                       // shared with ANY revolution plan are the band's
                       // to count — propose nothing; the rest take the
                       // arc share of this surface's turn, UNPINNED, as
                       // a proposal adaptive/neighbours may top. Freeform
                       // strips (grip rails) return false and keep the
                       // verbatim rail-count semantics below.
                       const TopoDS_Face rf = TopoDS::Face(model.faces(fid));
                       BRepAdaptor_Surface rs(rf);
                       const GeomAbs_SurfaceType rt = rs.GetType();
                       if (rt != GeomAbs_Cylinder && rt != GeomAbs_Cone &&
                           rt != GeomAbs_Torus) {
                           return false;
                       }
                       auto sharedWithRevolution = [&](int e) {
                           const TopoDS_Edge E =
                               TopoDS::Edge(model.edges(e));
                           if (!model.edgeToFaces.Contains(E)) return false;
                           for (const TopoDS_Shape& fs :
                                model.edgeToFaces.FindFromKey(E)) {
                               const int nfid =
                                   model.faces.FindIndex(fs);
                               if (nfid == fid) continue;
                               auto pit = plans.find(nfid);
                               if (pit != plans.end() &&
                                   pit->second.kind ==
                                       MesherKind::RevolutionGrid) {
                                   return true;
                               }
                           }
                           return false;
                       };
                       for (int e : plan.uEdges) {
                           if (sharedWithRevolution(e)) continue;
                           double f, l;
                           Handle(Geom2d_Curve) pc =
                               BRep_Tool::CurveOnSurface(
                                   TopoDS::Edge(model.edges(e)), rf, f, l);
                           double eu0 = 1e300, eu1 = -1e300;
                           if (!pc.IsNull()) {
                               for (int k = 0; k <= 8; ++k) {
                                   const double uu =
                                       pc->Value(f + (l - f) * k / 8.0)
                                           .X();
                                   eu0 = std::min(eu0, uu);
                                   eu1 = std::max(eu1, uu);
                               }
                           }
                           const double frac =
                               eu1 > eu0 ? (eu1 - eu0) / (2.0 * M_PI)
                                         : 0.0;
                           const int flat = std::max(
                               1, int(std::lround(std::max(3, s.radial) *
                                                  frac)));
                           proposeSet({e}, flat, 1, s.adaptive, s,
                                      /*overridden=*/false);
                       }
                       return true;
                   }()) {
            // proposals already emitted per arc share above
        } else {  // revolution sides and disk caps subdivide rings radially
            if (!plan.bandSides.empty()) {
                // Open band: each rim edge proposes its own count and
                // stays its neighbours' contract — adaptive follows
                // curvature, flat mode takes the edge's share of the
                // radial dial (wrap-scaled so 'radial' keeps meaning
                // divisions per full turn). The chains never equalize
                // by SUM: the band's strips and notch webs absorb any
                // mismatch inside the face.
                const TopoDS_Face bandFace =
                    TopoDS::Face(model.faces(fid));
                for (int e : plan.uEdges) {
                    // The edge's wrap fraction -> its share of the radial dial.
                    // This is BOTH the flat-mode count AND the authoritative
                    // count a typed radial pins: with adaptive ON, a per-face
                    // count sets curCountOverride, and proposeSet then uses this
                    // flat value verbatim — so it must be the real wrap-scaled
                    // number, never the old `1` placeholder (that pinned the
                    // driver rim to 1, collapsed nu to 3, and dropped the band
                    // to the contract floor — the "ring came back" / triangle
                    // soup at radial 20+). Adaptive-with-no-override still
                    // follows curvature through proposeSet.
                    double f, l;
                    Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(
                        TopoDS::Edge(model.edges(e)), bandFace, f, l);
                    double eu0 = 1e300, eu1 = -1e300;
                    if (!pc.IsNull()) {
                        for (int k = 0; k <= 8; ++k) {
                            const double uu =
                                pc->Value(f + (l - f) * k / 8.0).X();
                            eu0 = std::min(eu0, uu);
                            eu1 = std::max(eu1, uu);
                        }
                    }
                    const double frac =
                        eu1 > eu0 ? (eu1 - eu0) / (2.0 * M_PI) : 0.0;
                    const int flat = std::max(
                        1, int(std::lround(std::max(3, s.radial) * frac)));
                    proposeSet({e}, flat, 1, s.adaptive, s, overridden);
                }
            } else if (!plan.linkRims && plan.uEdges.size() == 2) {
                // Unlinked rims: each ring solves on its own (pin per-edge
                // or via the rim fields to make them differ).
                proposeSet({plan.uEdges[0]}, std::max(3, s.radial), 3,
                           s.adaptive, s, overridden);
                proposeSet({plan.uEdges[1]}, std::max(3, s.radial), 3,
                           s.adaptive, s, overridden);
            } else {
                proposeSet(plan.uEdges, std::max(3, s.radial), 3, s.adaptive,
                           s, overridden);
            }
            // Explicit axial acts as the floor along the axis; profile
            // curvature (a vase wall) adds what it needs.
            proposeSet(plan.vEdges, std::max(1, s.axial),
                       std::max(1, s.axial), s.adaptive, s, overridden);
        }
    }
    for (const auto& [root, count] : facePinned) {
        sol.groupCount[root] = count;
        sol.pinnedRoots.insert(root);
    }

    // Explicit per-edge overrides pin their whole group (max if several),
    // winning over both defaults and per-face overrides.
    std::map<int, int> pinned;
    for (const auto& [eid, count] : settings.perEdge) {
        if (eid < 1 || eid > model.edgeCount()) continue;
        int root = sol.groups.find(eid);
        auto [it, inserted] = pinned.try_emplace(root, count);
        if (!inserted) it->second = std::max(it->second, count);
    }
    for (const auto& [root, count] : pinned) {
        sol.groupCount[root] = std::max(1, count);  // a pin of 0 is a leak
        sol.pinnedRoots.insert(root);
    }

    // Chained Coons: opposite sides must sample equal TOTALS. Chains
    // share rails with other chains, so one-shot bumps go stale — grow
    // the SMALLER side's last unpinned edge instead and iterate to a
    // fixpoint (growth is monotone, so it terminates). Anything still
    // unequal falls back at mesh time without breaking seams.
    {
        // Never grow edges that other PATTERN meshers depend on: a
        // revolution band's radial count must stay the sum of its rim
        // arcs, and ring junctions derive their circle from the plate.
        std::set<int> protectedRoots;
        for (const auto& [fid2, plan2] : plans) {
            if (plan2.kind != MesherKind::RevolutionGrid &&
                plan2.kind != MesherKind::DiskCap &&
                plan2.kind != MesherKind::RingJunction) {
                continue;
            }
            for (int e : plan2.uEdges) {
                protectedRoots.insert(sol.groups.find(e));
            }
            if (plan2.circleEdgeId > 0) {
                protectedRoots.insert(sol.groups.find(plan2.circleEdgeId));
            }
        }
        auto sideSum = [&](const std::vector<int>& sd) {
            int t = 0;
            for (int e : sd) t += std::max(1, sol.countFor(e, 1));
            return t;
        };
        auto grow = [&](const std::vector<int>& sd, int by) {
            for (auto it = sd.rbegin(); it != sd.rend(); ++it) {
                int root = sol.groups.find(*it);
                if (pinned.count(root) || protectedRoots.count(root)) {
                    continue;
                }
                sol.groupCount[root] =
                    std::max(1, sol.countFor(*it, 1)) + by;
                return true;
            }
            return false;
        };
        // COUNT DECOUPLING (handoff step 1): the old fixpoint grew the
        // lighter side of every chained coons patch until opposite totals
        // matched, which cascaded counts across shared rails (measured:
        // 37,808-poly defaults on one model, and a ~983k solved count on
        // a dirty assembly). Chained sides now keep their natural counts;
        // meshCoonsGrid arc-length-resamples the deficit rail for its
        // lattice and the post-weld seam absorber splices the neighbours'
        // extra border vertices in as small-edge n-gons — the way the
        // reference CAD export absorbs count mismatches.
        (void)sideSum;
        (void)grow;
    }

    // Ring junctions close the loop: the circle must take exactly one ring
    // vertex per boundary vertex, so its group count is DERIVED from the
    // boundary — 2*(nu+nv) — and propagates through the group to whatever
    // boss/bore shares that circle ("the plate drives the boss"). A face
    // whose circle is pinned to an incompatible count can't form the
    // pattern and demotes to fallback triangulation.
    for (auto& [fid, plan] : plans) {
        if (plan.kind != MesherKind::RingJunction) continue;
        int nu = sol.countFor(plan.uEdges[0], 1);
        int nv = sol.countFor(plan.vEdges[0], 1);
        int derived = 2 * (nu + nv);
        int root = sol.groups.find(plan.circleEdgeId);
        auto pin = pinned.find(root);
        if (pin != pinned.end() && pin->second != derived) {
            plan.kind = MesherKind::Fallback;
            plan.constrains = false;
            continue;
        }
        sol.groupCount[root] = derived;
    }

    return sol;
}

// Pin the base-arc edges of every full-wrap castellated rim to the plain
// rim's column azimuths (plus the notch-corner endpoints). The band then
// runs columns straight from the plain rim onto the cut rim with no
// transition strip and no phase break, and the neighbour annulus face —
// which samples these same edges — adopts the identical positions, so the
// shared border stays watertight by construction (the column-alignment
// contract). Runs after the density solve so nu (the plain rim's solved
// count) is final; both this pass and the mesher derive uk[] from the same
// plain-rim sampling, so their columns coincide.
void pinCastellatedRims(const Model& model,
                        const std::map<int, FacePlan>& plans,
                        const GenerationSettings& settings,
                        const std::vector<int>& solvedEdge,
                        DensitySolution& density, PinnedEdges& pins) {
    for (const auto& [fid, plan] : plans) {
        if (!plan.castellated || plan.plainRimEdge < 1) continue;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        BRepAdaptor_Surface surf(face);
        if (!surf.IsUClosed() || surf.IsVClosed()) continue;
        const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
        const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
        const double period = std::max(1e-12, u1 - u0);
        const double vspan = std::max(1e-12, v1 - v0);
        const int nu = std::max(
            3, std::max(solvedEdge[plan.plainRimEdge],
                        density.countFor(plan.plainRimEdge,
                                         settings.forFace(fid).radial)));
        const double pitch = period / nu;

        // Column azimuths from the plain rim (the mesher's own sampling).
        const bool lowPlain =
            plan.rimLow.size() == 1 && plan.rimLow[0] == plan.plainRimEdge;
        const bool highPlain =
            plan.rimHigh.size() == 1 && plan.rimHigh[0] == plan.plainRimEdge;
        if (lowPlain == highPlain) continue;
        const std::vector<int>& cutChain = lowPlain ? plan.rimHigh
                                                     : plan.rimLow;
        std::vector<double> uk;
        double vPlain = 0;
        {
            const TopoDS_Edge edge =
                TopoDS::Edge(model.edges(plan.plainRimEdge));
            double f2, l2;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f2, l2);
            if (pc.IsNull()) continue;
            const bool rev = edge.Orientation() == TopAbs_REVERSED;
            const double ph = closedEdgePhase(edge, model);
            for (int i = 0; i < nu; ++i) {
                const double t = phasedT(i, nu, ph, rev);
                gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * t);
                double u = uv.X();
                u -= period * std::floor((u - u0) / period);
                uk.push_back(u);
                vPlain += uv.Y();
            }
            vPlain /= nu;
        }
        if (int(uk.size()) != nu) continue;
        std::sort(uk.begin(), uk.end());
        // Per-edge pcurve box (u-span, v-range) + the wall azimuths (the
        // two notch corners). Classify: an azimuthal arc (base arc / floor)
        // spans u at ~constant v; a wall spans v at ~constant u.
        struct EdgeBox {
            int eid;
            std::vector<double> uSeq;  // unwrapped, monotone
            bool wall = false;
        };
        const int NS = 32;
        std::vector<EdgeBox> boxes;
        std::vector<double> cornerU;  // wall azimuths (the notch corners)
        for (int eid : cutChain) {
            if (eid < 1 || eid >= int(pins.size())) continue;
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(edge)) continue;
            double f2, l2;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f2, l2);
            if (pc.IsNull()) continue;
            EdgeBox box;
            box.eid = eid;
            box.uSeq.resize(NS + 1);
            double vLo = 1e300, vHi = -1e300, uPrev = 0;
            for (int k = 0; k <= NS; ++k) {
                gp_Pnt2d uv = pc->Value(f2 + (l2 - f2) * double(k) / NS);
                vLo = std::min(vLo, uv.Y());
                vHi = std::max(vHi, uv.Y());
                double u = uv.X();
                u -= period * std::floor((u - u0) / period);
                if (k > 0) u += period * std::round((uPrev - u) / period);
                box.uSeq[k] = u;
                uPrev = u;
            }
            const double uSpanE = std::abs(box.uSeq.back() - box.uSeq.front());
            box.wall = uSpanE < 0.5 * pitch && vHi - vLo > 0.1 * vspan;
            if (box.wall) {
                cornerU.push_back(0.5 * (box.uSeq.front() + box.uSeq.back()));
            }
            boxes.push_back(std::move(box));
        }
        // Only a column essentially COINCIDENT with a corner rides it (the
        // corner already owns that vertex); a column merely near a corner
        // keeps its own sample and the cap n-gon absorbs the thin gap, so
        // every uniform column stays present and straight.
        auto nearCorner = [&](double c) {
            for (double cu : cornerU) {
                double d = c - cu;
                d -= period * std::round(d / period);
                if (std::abs(d) < 0.02 * pitch) return true;
            }
            return false;
        };
        for (EdgeBox& box : boxes) {
            if (box.wall) {
                pins[box.eid] = {0.0, 1.0};  // single flat span
                continue;
            }
            const std::vector<double>& uSeq = box.uSeq;
            const double uLo = std::min(uSeq.front(), uSeq.back());
            const double uHi = std::max(uSeq.front(), uSeq.back());
            // Column azimuths within the arc span become samples; the two
            // notch corners (wall azimuths) reserve their own vertices, so a
            // column riding a corner is dropped rather than minting a sliver.
            // Guarding against the GLOBAL corners (not the edge endpoints)
            // keeps columns near the u-seam — where a base arc is split into
            // two edges with no corner between them.
            std::vector<double> fr;
            fr.push_back(0.0);
            for (double col : uk) {
                for (int kk = -1; kk <= 1; ++kk) {
                    const double c = col + kk * period;
                    if (c <= uLo + 1e-9 || c >= uHi - 1e-9) continue;
                    if (nearCorner(c)) continue;
                    double t = -1;
                    for (int s = 0; s < NS; ++s) {
                        const double a = uSeq[s], b = uSeq[s + 1];
                        if ((c - a) * (c - b) <= 0 && std::abs(b - a) > 1e-15) {
                            t = (double(s) + (c - a) / (b - a)) / NS;
                            break;
                        }
                    }
                    if (t > 1e-6 && t < 1 - 1e-6) fr.push_back(t);
                }
            }
            fr.push_back(1.0);
            std::sort(fr.begin(), fr.end());
            fr.erase(std::unique(fr.begin(), fr.end(),
                                 [](double a, double b) {
                                     return std::abs(a - b) < 1e-9;
                                 }),
                     fr.end());
            if (fr.size() < 2) continue;
            pins[box.eid] = std::move(fr);
        }
    }
}

// Pin fillet blend chains so columns continue THROUGH them. From each open
// revolution band's fillet-side rim, walk the coaxial coons blend strips
// rail-to-rail until the next revolution face, pinning every cross-rail arc
// to the band's column azimuths (the arc endpoints stay the castellation
// corners). The band's bottom transition strip then degenerates to quads and
// the barrel columns run straight into and through the blends. The chain's
// ENTRY rail (a band cut arc) fixes the column set the whole chain carries so
// a fillet's two rails always take equal counts (an arc catching more columns
// than its solved count raises that count instead of bailing, so the cut rim
// keeps grounding as radial climbs); a column landing right on a notch corner
// is skipped so the cut rim never gets a near-duplicate sample; only a rail
// catching FEWER columns than its own subdivisions is left uniform.
void pinFilletChains(const Model& model,
                     const std::map<int, FacePlan>& plans,
                     std::vector<int>& solvedEdge, PinnedEdges& pins) {
    // Azimuth about a revolution axis (loc O, unit dir D), with an in-plane
    // reference frame (r1, r2).
    auto frameOf = [](const gp_Ax1& ax, gp_Vec& r1, gp_Vec& r2) {
        const gp_Dir d = ax.Direction();
        gp_Vec dv(d);
        gp_Vec t = std::abs(d.X()) < 0.9 ? gp_Vec(1, 0, 0) : gp_Vec(0, 1, 0);
        r1 = t - dv * (t.Dot(dv));
        r1.Normalize();
        r2 = dv.Crossed(r1);
    };
    auto azimuth = [](const gp_Pnt& p, const gp_Pnt& o, const gp_Vec& r1,
                      const gp_Vec& r2) {
        gp_Vec w(o, p);
        return std::atan2(w.Dot(r2), w.Dot(r1));
    };
    // The two coaxial circular rails of a coons blend face, longest-span
    // first; empties when the face isn't a two-rail blend on this axis.
    auto railsOf = [&](int fid, const gp_Ax1& ax, const gp_Pnt& o,
                       const gp_Vec& r1, const gp_Vec& r2) {
        std::vector<int> rails;
        const TopoDS_Face F = TopoDS::Face(model.faces(fid));
        for (TopExp_Explorer ex(F, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(e)) continue;
            const int eid = model.edges.FindIndex(e);
            if (eid < 1) continue;
            double f, l;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
            if (c.IsNull()) continue;
            GeomAdaptor_Curve gc(c, f, l);
            if (gc.GetType() != GeomAbs_Circle) continue;
            // Coaxial with the band axis?
            gp_Ax1 ca = gc.Circle().Axis();
            if (1.0 - std::abs(ca.Direction().Dot(ax.Direction())) > 1e-4) {
                continue;
            }
            double amn = 1e300, amx = -1e300;
            for (int k = 0; k <= 8; ++k) {
                const double a =
                    azimuth(c->Value(f + (l - f) * k / 8.0), o, r1, r2);
                amn = std::min(amn, a);
                amx = std::max(amx, a);
            }
            if (amx - amn < 0.05) continue;  // an across edge, not a rail
            rails.push_back(eid);
        }
        return rails;
    };
    // Project a set of column azimuths onto one arc edge and pin it. Returns
    // the SUBSET of column azimuths that actually landed on the arc (empty if
    // the arc was left unpinned) so the chain walk can pin a fillet's opposite
    // rail to the identical columns.
    auto pinArc = [&](int eid, const std::vector<double>& cols,
                      const gp_Pnt& o, const gp_Vec& r1,
                      const gp_Vec& r2) -> std::vector<double> {
        if (eid < 1 || eid >= int(pins.size()) || !pins[eid].empty()) return {};
        const int want = eid < int(solvedEdge.size()) ? solvedEdge[eid] : 0;
        if (want < 2) return {};
        const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
        double f, l;
        Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
        if (c.IsNull()) return {};
        const int NS = 64;
        std::vector<double> aSeq(NS + 1);
        double aPrev = 0;
        for (int k = 0; k <= NS; ++k) {
            double a = azimuth(c->Value(f + (l - f) * double(k) / NS), o, r1,
                               r2);
            if (k > 0) a += 2 * M_PI * std::round((aPrev - a) / (2 * M_PI));
            aSeq[k] = a;
            aPrev = a;
        }
        const double aLo = std::min(aSeq.front(), aSeq.back());
        const double aHi = std::max(aSeq.front(), aSeq.back());
        // Raw interior hits: (t along arc, source column azimuth).
        std::vector<std::pair<double, double>> hit;
        for (double col : cols) {
            for (int kk = -1; kk <= 1; ++kk) {
                const double a = col + kk * 2 * M_PI;
                if (a <= aLo + 1e-6 || a >= aHi - 1e-6) continue;
                double t = -1;
                for (int s = 0; s < NS; ++s) {
                    const double x = aSeq[s], y = aSeq[s + 1];
                    if ((a - x) * (a - y) <= 0 && std::abs(y - x) > 1e-15) {
                        t = (double(s) + (a - x) / (y - x)) / NS;
                        break;
                    }
                }
                if (t > 1e-6 && t < 1 - 1e-6) hit.push_back({t, col});
            }
        }
        std::sort(hit.begin(), hit.end());
        // A column that lands within ~30% of a sample spacing of a notch corner
        // (t=0/1) or of the previous column would put a near-coincident sample
        // on the cut rim, whose zero-length edge takes the band non-manifold at
        // that exact count. Drop it (that lone column just isn't grounded).
        const double minGap = hit.empty() ? 1.0 : 0.3 / double(hit.size() + 1);
        std::vector<double> fr{0.0};
        std::vector<double> accepted;
        double last = 0.0;
        for (const auto& [t, col] : hit) {
            if (t < minGap || t > 1.0 - minGap || t < last + minGap) continue;
            fr.push_back(t);
            accepted.push_back(col);
            last = t;
        }
        fr.push_back(1.0);
        const int got = int(fr.size()) - 1;
        // Pin at the columns that actually cross this arc even when that
        // exceeds the arc's solved count: the band's cut rim then samples ON
        // every column and grounds at any radial (the ring stays gone as the
        // count climbs). A DEFICIT (got < want) still bails — pinning a
        // near-tangent rail sparse would starve the neighbour blend.
        if (got < want) return {};
        // Raise the arc's solved count to the pinned column count so the
        // neighbour blend's interior grid matches its now-denser border (else
        // the coons demotes to a zippered strip of triangles at that seam).
        if (got > want) solvedEdge[eid] = got;
        pins[eid] = std::move(fr);
        std::sort(accepted.begin(), accepted.end());
        return accepted;
    };
    auto faceAcross = [&](int eid, int notFid, MesherKind wantKind) {
        const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
        if (!model.edgeToFaces.Contains(e)) return 0;
        for (const TopoDS_Shape& s : model.edgeToFaces.FindFromKey(e)) {
            const int f2 = model.faces.FindIndex(s);
            if (f2 < 1 || f2 == notFid) continue;
            auto it = plans.find(f2);
            if (it != plans.end() && it->second.kind == wantKind) return f2;
        }
        return 0;
    };

    for (const auto& [fid, plan] : plans) {
        if (plan.bandSides.empty() || plan.kind != MesherKind::RevolutionGrid) {
            continue;
        }
        const TopoDS_Face bandFace = TopoDS::Face(model.faces(fid));
        BRepAdaptor_Surface surf(bandFace);
        if (surf.GetType() != GeomAbs_Cylinder) continue;
        const gp_Ax1 ax = surf.Cylinder().Axis();
        gp_Vec r1, r2;
        frameOf(ax, r1, r2);
        const gp_Pnt o = ax.Location();
        if (plan.bandDriver < 1) continue;
        // Column azimuths = the driver rim's own samples.
        std::vector<double> cols;
        {
            const int drv = plan.bandDriver;
            const int nu = drv < int(solvedEdge.size()) ? solvedEdge[drv] : 0;
            if (nu < 3) continue;
            const TopoDS_Edge de = TopoDS::Edge(model.edges(drv));
            double f, l;
            Handle(Geom_Curve) dc = BRep_Tool::Curve(de, f, l);
            if (dc.IsNull()) continue;
            const bool rev = de.Orientation() == TopAbs_REVERSED;
            const double ph = closedEdgePhase(de, model);
            for (int i = 0; i < nu; ++i) {
                const double t = phasedT(i, nu, ph, rev);
                cols.push_back(azimuth(dc->Value(f + (l - f) * t), o, r1, r2));
            }
            std::sort(cols.begin(), cols.end());
        }
        // Walk the blend chain from every fillet-side rim arc.
        for (int rimEid : plan.uEdges) {
            int cur = rimEid;
            int prevFid = fid;
            std::set<int> seen;
            // The band cut-rim arc (the chain's entry) fixes which columns the
            // whole chain carries; every downstream fillet rail pins to that
            // SAME set so a fillet's two rails never disagree in count (a
            // mismatch there demotes the coons to a strip of triangles).
            std::vector<double> chainCols = cols;
            while (cur >= 1 && !seen.count(cur)) {
                seen.insert(cur);
                const int F = faceAcross(cur, prevFid, MesherKind::CoonsGrid);
                if (F < 1) break;  // reached a non-coons neighbour
                std::vector<double> acc = pinArc(cur, chainCols, o, r1, r2);
                if (!acc.empty()) chainCols = std::move(acc);
                // The blend's OPPOSITE rail continues the chain.
                std::vector<int> rails = railsOf(F, ax, o, r1, r2);
                int nxt = 0;
                for (int e2 : rails) {
                    if (e2 != cur) { nxt = e2; break; }
                }
                if (nxt < 1) break;
                pinArc(nxt, chainCols, o, r1, r2);
                prevFid = F;
                cur = nxt;
            }
        }
    }
}


}  // namespace weft::mesher_impl
