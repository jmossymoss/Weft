#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// ---------------------------------------------------------------------------
// Border conformity (plan §7.1, first bite): freeform faces triangulate to
// chord tolerance, so their borders never agree with an analytic
// neighbour's solved divisions — T-junctions along every shared edge. Fix
// after meshing: for each edge where a fallback face meets a constraining
// analytic face, snap the fallback border chain onto the analytic vertex
// chain and insert any analytic verts the chain skips; the weld then fuses
// the seam exactly.

struct EdgeParamPoint {
    uint32_t vert;
    double param;
};

void conformFallbackBorders(PolyMesh& mesh, const Model& model,
                            const std::map<int, FacePlan>& plans,
                            const GenerationSettings& settings,
                            const std::vector<std::array<size_t, 2>>& range,
                            const std::vector<char>& fellBack) {
    // Conforming across BODIES splices the other solid's vertex ids into
    // this face's polygons — contact faces then fuse into non-manifold
    // sandwiches. Neighbours must share the owning solid.
    std::vector<int> faceSolid(model.faceCount() + 1, 0);
    {
        int solidId = 0;
        auto assign = [&](const TopoDS_Shape& obj) {
            ++solidId;
            for (TopExp_Explorer fx(obj, TopAbs_FACE); fx.More();
                 fx.Next()) {
                int f2 = model.faces.FindIndex(fx.Current());
                if (f2 > 0 && faceSolid[f2] == 0) faceSolid[f2] = solidId;
            }
        };
        for (TopExp_Explorer sx(model.shape, TopAbs_SOLID); sx.More();
             sx.Next()) {
            assign(sx.Current());
        }
        for (TopExp_Explorer sx(model.shape, TopAbs_SHELL, TopAbs_SOLID);
             sx.More(); sx.Next()) {
            assign(sx.Current());
        }
    }
    auto isFreeform = [&](int fid) {
        // A verified contract floor (fellBack == 2) has EXACT borders —
        // moving them tears its web triangles open.
        if (fid < int(fellBack.size()) && fellBack[fid] == 2) return false;
        MesherKind k = plans.at(fid).kind;
        // RibbonSweep samples every border directly from the shared 3D curve
        // at the pinned/solved fractions and passes the exact border contract
        // before it is accepted.  Treat that rail ladder as an authority too:
        // post-build snapping shears its sparse cap rungs and can fold an
        // otherwise clean short strip.
        if (k == MesherKind::RibbonSweep) return false;
        return (k == MesherKind::Fallback || k == MesherKind::QuadDominant ||
                k == MesherKind::AnnulusRing ||
                !plans.at(fid).loops.empty() ||
                (k == MesherKind::PlanarGrid &&
                 !plans.at(fid).constrains)) &&
               !settings.forFace(fid).exclude;
    };
    auto isAnalytic = [&](int fid) {
        MesherKind k = plans.at(fid).kind;
        return plans.at(fid).constrains && k != MesherKind::Fallback &&
               k != MesherKind::QuadDominant && !settings.forFace(fid).exclude;
    };

    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        if (!isFreeform(fid)) continue;

        // Topological border vertices of this face's sub-mesh.
        std::map<std::pair<uint32_t, uint32_t>, int> use;
        std::vector<size_t> facePolys;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != fid) continue;
            facePolys.push_back(p);
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                ++use[a < b ? std::make_pair(a, b) : std::make_pair(b, a)];
            }
        }
        std::set<uint32_t> borderVerts;
        for (const auto& [e, count] : use) {
            if (count == 1) {
                borderVerts.insert(e.first);
                borderVerts.insert(e.second);
            }
        }
        if (borderVerts.empty()) continue;

        for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE); ex.More();
             ex.Next()) {
            int eid = model.edges.FindIndex(ex.Current());
            if (eid < 1) continue;
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            // Degenerate edges (poles, apexes) carry no 3D curve; seam and
            // curveless edges can't anchor a chain either.
            if (BRep_Tool::Degenerated(edge)) continue;
            {
                double cf, cl;
                if (BRep_Tool::Curve(edge, cf, cl).IsNull()) continue;
            }
            // Contact edges in multi-body files carry 3+ faces; prefer an
            // ANALYTIC neighbour (its borders sample this very curve) over
            // whichever face happens to come last in the map — picking a
            // wrong-solid neighbour finds zero targets and the seam stays
            // open.
            int nfid = 0;
            if (model.edgeToFaces.Contains(ex.Current())) {
                for (const TopoDS_Shape& s :
                     model.edgeToFaces.FindFromKey(ex.Current())) {
                    int f2 = model.faces.FindIndex(s);
                    if (f2 == fid || f2 < 1) continue;
                    if (faceSolid[f2] != faceSolid[fid]) continue;
                    if (nfid < 1 || (!isAnalytic(nfid) && isAnalytic(f2))) {
                        nfid = f2;
                    }
                }
            }
            const bool analyticNb = nfid >= 1 && isAnalytic(nfid);
            // A pinned edge with no analytic driver (both sides freeform,
            // or the neighbour deleted) resamples this border to exactly
            // the pinned count with vertices ON the curve — count control
            // that keeps the curvature.
            auto pinIt = settings.perEdge.find(eid);
            const bool pinnedResample =
                !analyticNb && pinIt != settings.perEdge.end() &&
                pinIt->second >= 2;
            // Freeform-to-freeform seams: the DENSER side is the
            // authority (ties: lower id) and only the sparser side moves,
            // exactly like the analytic case — snapping never collapses
            // because the target chain has at least as many verts.
            const bool freeformSeam = !analyticNb && !pinnedResample &&
                                      nfid >= 1 && isFreeform(nfid);
            if (!analyticNb && !pinnedResample && !freeformSeam) continue;
            dbg("conform: face %d edge %d (%s)", fid, eid,
                analyticNb ? "analytic neighbour"
                : pinnedResample ? "pinned resample"
                                 : "freeform seam");

            BRepAdaptor_Curve curve(edge);
            const double f = curve.FirstParameter(), l = curve.LastParameter();
            const bool closed = curve.IsClosed();
            const double period = l - f;
            GCPnts_AbscissaPoint lenTool;
            double edgeLen = GCPnts_AbscissaPoint::Length(curve);
            (void)lenTool;

            // Coarse polyline of the edge: candidates farther from it
            // than the tolerance (plus the sampling slack) can't project
            // onto the curve, so the expensive Extrema never runs for the
            // bulk of a neighbour's vertices.
            std::array<gp_Pnt, 33> coarse;
            for (int i = 0; i < 33; ++i) {
                coarse[i] = curve.Value(f + (l - f) * i / 32.0);
            }
            const double slack = edgeLen / 16.0;

            // Exact distance/parameter on the curve for a point. The
            // coarse polyline seeds the answer — Extrema can return
            // nothing at all on tiny curves, and an endpoint-only
            // fallback then bunches every projection at the two ends,
            // scrambling the nearest-by-param snap.
            auto projectPnt = [&](const gp_Pnt& p, double tol,
                                  double* paramOut) -> bool {
                double bestD = 1e300;
                double bestT = f;
                for (int i = 0; i < 33; ++i) {
                    double d = p.Distance(coarse[i]);
                    if (d < bestD) {
                        bestD = d;
                        bestT = f + (l - f) * i / 32.0;
                    }
                }
                if (bestD > tol + slack) return false;
                try {
                    Extrema_ExtPC ext(p, curve);
                    if (ext.IsDone()) {
                        for (int i = 1; i <= ext.NbExt(); ++i) {
                            double d = std::sqrt(ext.SquareDistance(i));
                            if (d < bestD) {
                                bestD = d;
                                bestT = ext.Point(i).Parameter();
                            }
                        }
                    }
                } catch (...) {
                    // Extrema can fail on exotic curves; endpoint distances
                    // computed above still stand.
                }
                if (bestD > tol) return false;
                *paramOut = bestT;
                return true;
            };
            auto project = [&](uint32_t v, double tol,
                               double* paramOut) -> bool {
                return projectPnt(gp_Pnt(mesh.vertices[v][0],
                                         mesh.vertices[v][1],
                                         mesh.vertices[v][2]),
                                  tol, paramOut);
            };

            // The authoritative chain: the analytic side's verts on this
            // edge, or — for a pinned resample — fresh uniform samples on
            // the curve itself. Coons verts evaluate through the pcurve,
            // which only agrees with the 3D curve to the edge tolerance,
            // so include it.
            const double tolTarget =
                std::max(1e-6 * (1.0 + edgeLen),
                         10.0 * BRep_Tool::Tolerance(edge));
            // My border verts on this edge (needed up front: seams pick
            // the denser side as authority before any vertex moves).
            // Fallback/ring borders lie ON their curves, so capture them
            // tightly — a loose radius kidnaps verts that belong to
            // ADJACENT edges when a face is thinner than the slack (a
            // 0.19mm strip's interior verts are within 0.12 of every
            // edge around it) and snapping folds them onto the corners.
            // Only decimated borders (quad-dominant simplification,
            // unconstrained grids) genuinely sit off-curve and keep the
            // loose chord-scaled capture.
            const FacePlan& myPlan = plans.at(fid);
            const bool decimatedBorder =
                myPlan.kind == MesherKind::QuadDominant ||
                (myPlan.kind == MesherKind::Fallback &&
                 (myPlan.forceFallbackQuads >= 0
                      ? myPlan.forceFallbackQuads != 0
                      : settings.forFace(fid).quadDominant)) ||
                (myPlan.kind == MesherKind::PlanarGrid &&
                 !myPlan.constrains);
            const double tolMoverPre =
                decimatedBorder
                    ? std::max(
                          1e-6 * (1.0 + edgeLen),
                          std::max(settings.forFace(fid).chordTolerance,
                                   nfid >= 1
                                       ? settings.forFace(nfid)
                                             .chordTolerance
                                       : 0.0) *
                              1.2)
                    : tolTarget;
            std::map<uint32_t, double> movers;  // vert -> snapped param
            std::map<uint32_t, gp_Pnt> moverOrig;  // pre-snap positions
            for (uint32_t v : borderVerts) {
                double t;
                if (project(v, tolMoverPre, &t)) {
                    movers[v] = t;
                    moverOrig.emplace(v, gp_Pnt(mesh.vertices[v][0],
                                                mesh.vertices[v][1],
                                                mesh.vertices[v][2]));
                }
            }
            if (movers.empty()) continue;

            std::vector<EdgeParamPoint> targets;
            if (analyticNb || freeformSeam) {
                // Triangulation NODES on an edge lie exactly on its curve
                // (only chord midpoints sag), so a freeform authority uses
                // the same tight projection as an analytic one. A loose
                // tolerance here captured the neighbour's verts on OTHER
                // nearly-collinear edges as targets, and the insertion
                // step then dragged this border onto them (folds).
                for (size_t v = range[nfid][0]; v < range[nfid][1]; ++v) {
                    double t;
                    if (project(uint32_t(v), tolTarget, &t)) {
                        targets.push_back({uint32_t(v), t});
                    }
                }
            } else {
                const int n = pinIt->second;
                const int count = closed ? n : n + 1;
                for (int i = 0; i < count; ++i) {
                    double t = f + (l - f) * i / double(n);
                    gp_Pnt q = curve.Value(t);
                    uint32_t nv = uint32_t(mesh.vertices.size());
                    mesh.vertices.push_back({q.X(), q.Y(), q.Z()});
                    mesh.anchors.push_back({});
                    targets.push_back({nv, t});
                }
            }
            dbg("conform: face %d edge %d: %zu movers, %zu targets", fid,
                eid, movers.size(), targets.size());
            if (targets.size() < 2) continue;
            // Decoupled seams: conform must never work from an
            // INCOMPLETE target set. Stitch-mode resampled rims sit a
            // hair off the exact curve, so the tight target projection
            // captures only their corner verts and conform pulls a
            // solved ring down onto 2 points (mohne's hole caved to a
            // triangle). Completeness test: recount the neighbour's
            // verts with a loose (4x) band — if the tight set missed a
            // real fraction of them, the neighbour's border is off-
            // curve by stitch design, and the seam belongs to the
            // stitcher. (A complete-but-small set is legitimate: a
            // sloppy freeform border decimating onto a coarse analytic
            // contract keeps working — 1797609in needs exactly that.)
            if (settings.decoupleSeams &&
                (analyticNb || freeformSeam)) {
                size_t loose = 0;
                const double tolLoose =
                    std::max(tolTarget * 4.0, 0.03 * edgeLen);
                for (size_t v = range[nfid][0]; v < range[nfid][1]; ++v) {
                    double t;
                    if (project(uint32_t(v), tolLoose, &t)) {
                        ++loose;
                    }
                }
                if (targets.size() * 5 < loose * 4) continue;
            }
            // Already-welded seam: every mover sits on some target
            // (both faces sampled this edge at the same solved count,
            // so their borders are bit-identical). The loose capture
            // can still catch EXTRA targets on adjacent edges, and
            // pairing against those drags matched verts off the seam —
            // tearing a junction that was already exact. Nothing to
            // conform here. Known limitation: a raw-OCCT part whose
            // only border verts are the edge ENDPOINTS also reads as
            // aligned and skips the insertion splice — reachable only
            // when both floor retries failed (no corpus instance); the
            // seam absorber still gets a shot at such gaps.
            {
                const double wtol =
                    std::max(1e-6, settings.weldTolerance);
                bool aligned = true;
                for (const auto& [v, t] : movers) {
                    const auto& mv = mesh.vertices[v];
                    bool onTarget = false;
                    for (const auto& tg : targets) {
                        const auto& tv = mesh.vertices[tg.vert];
                        const double dx = mv[0] - tv[0], dy = mv[1] - tv[1],
                                     dz = mv[2] - tv[2];
                        if (dx * dx + dy * dy + dz * dz < wtol * wtol) {
                            onTarget = true;
                            break;
                        }
                    }
                    if (!onTarget) {
                        aligned = false;
                        break;
                    }
                }
                if (aligned) continue;
            }
            // Seam authority: only the sparser side conforms; the denser
            // (or equal-count lower-id) side keeps its chain.
            if (freeformSeam &&
                (targets.size() < movers.size() ||
                 (targets.size() == movers.size() && fid < nfid))) {
                continue;
            }

            std::sort(targets.begin(), targets.end(),
                      [](const EdgeParamPoint& a, const EdgeParamPoint& b) {
                          return a.param < b.param;
                      });

            auto paramGap = [&](double a, double b) {  // |a-b| wrap-aware
                double d = std::abs(a - b);
                return closed ? std::min(d, period - d) : d;
            };
            // Largest spacing between consecutive targets: a mover whose
            // nearest target is farther than this has NO partner on the
            // chain (the neighbour's matching vert sits off-curve on a
            // sloppy edge, or the chain doesn't reach the mover's end) —
            // snapping it anyway teleports it across the edge and folds
            // its polygons. Leave such movers where they are.
            double maxGap = 0.0;
            for (size_t i = 1; i < targets.size(); ++i) {
                maxGap = std::max(
                    maxGap, targets[i].param - targets[i - 1].param);
            }
            if (closed) {
                maxGap = std::max(
                    maxGap, period - (targets.back().param -
                                      targets.front().param));
            }

            // Equal counts: pair by RANK along the curve — a bijection.
            // Nearest-by-param can send two drifted movers to one target
            // and leave its neighbour unmatched, punching a hole in an
            // otherwise perfectly matched seam.
            if (movers.size() == targets.size() && targets.size() >= 2) {
                std::vector<std::pair<double, uint32_t>> mv;
                mv.reserve(movers.size());
                for (const auto& [v, t] : movers) mv.push_back({t, v});
                std::sort(mv.begin(), mv.end());
                const int nRank = int(mv.size());
                int bestShift = 0;
                if (closed) {
                    double bestCost = 1e300;
                    for (int sft = 0; sft < nRank; ++sft) {
                        double c = 0;
                        for (int i = 0; i < nRank; ++i) {
                            c += paramGap(targets[(i + sft) % nRank].param,
                                          mv[i].first);
                        }
                        if (c < bestCost) {
                            bestCost = c;
                            bestShift = sft;
                        }
                    }
                }
                for (int i = 0; i < nRank; ++i) {
                    const EdgeParamPoint& tgt =
                        targets[(i + bestShift) % nRank];
                    mesh.vertices[mv[i].second] = mesh.vertices[tgt.vert];
                    movers[mv[i].second] = tgt.param;
                }
                continue;
            }

            // Snap every mover to the nearest target (position + param).
            struct SnapPick {
                uint32_t v;
                const EdgeParamPoint* best;
                double dist;
            };
            std::vector<SnapPick> picks;
            for (auto& [v, t] : movers) {
                const EdgeParamPoint* best = &targets[0];
                for (const EdgeParamPoint& cand : targets) {
                    if (paramGap(cand.param, t) < paramGap(best->param, t)) {
                        best = &cand;
                    }
                }
                if (paramGap(best->param, t) > 0.75 * maxGap) {
                    dbg("conform: face %d edge %d vert %u kept (nearest "
                        "target %.4g away, max gap %.4g)",
                        fid, eid, v, paramGap(best->param, t), maxGap);
                    continue;
                }
                const auto& q = mesh.vertices[best->vert];
                picks.push_back(
                    {v, best,
                     moverOrig.at(v).Distance(gp_Pnt(q[0], q[1], q[2]))});
            }
            // (Injective snapping — one mover per target — was tried for
            // decimated borders and reverted: their zip against analytic
            // chains RELIES on many-to-one collapse; forcing uniqueness
            // exploded opens 10x across the sweep.)
            for (const SnapPick& s : picks) {
                mesh.vertices[s.v] = mesh.vertices[s.best->vert];
                movers[s.v] = s.best->param;
            }

            // Insert targets skipped between consecutive border movers so
            // the chains agree vertex-for-vertex.
            for (size_t p : facePolys) {
                std::vector<uint32_t>& poly = mesh.polygons[p];
                std::vector<uint32_t> ring;
                ring.reserve(poly.size() + 4);
                for (size_t i = 0; i < poly.size(); ++i) {
                    uint32_t u = poly[i], w = poly[(i + 1) % poly.size()];
                    ring.push_back(u);
                    auto mu = movers.find(u), mw = movers.find(w);
                    if (mu == movers.end() || mw == movers.end()) continue;
                    // Only true border segments take insertions — a
                    // triangulation also has interior chords whose both
                    // ends sit on the curve, and inserting into those
                    // duplicates the chain.
                    auto useIt = use.find(
                        u < w ? std::make_pair(u, w) : std::make_pair(w, u));
                    if (useIt == use.end() || useIt->second != 1) continue;
                    // Border segment on the edge: walk the shorter param
                    // arc from u to w, inserting the targets inside it.
                    double pu = mu->second, pw = mw->second;
                    if (paramGap(pu, pw) < 1e-12) continue;
                    bool forward = closed
                        ? std::fmod(pw - pu + period, period) <= period * 0.5
                        : pw > pu;
                    const double span = closed
                        ? std::fmod((forward ? pw - pu : pu - pw) + period,
                                    period)
                        : std::abs(pw - pu);
                    std::vector<const EdgeParamPoint*> between;
                    for (const EdgeParamPoint& cand : targets) {
                        double rel = closed
                            ? std::fmod((forward ? cand.param - pu
                                                 : pu - cand.param) + period,
                                        period)
                            : (forward ? cand.param - pu : pu - cand.param);
                        if (rel > 1e-12 && rel < span - 1e-12) {
                            between.push_back(&cand);
                        }
                    }
                    // A segment swallowing SEVERAL targets must actually
                    // LIE on this edge inside (pu,pw): a border segment of
                    // a DIFFERENT edge can still have both endpoints on
                    // this curve — the two ends of a nearly-closed arc are
                    // joined by its tiny closing edge — and inserting the
                    // chain there wraps the whole arc into that polygon a
                    // second time. Verified with the PRE-SNAP midpoint;
                    // one-or-two-target insertions skip the check (short
                    // spans put the midpoint near the boundary from sheer
                    // projection noise and were being starved).
                    if (between.size() >= 3) {
                        const gp_Pnt& a = moverOrig.at(u);
                        const gp_Pnt& b = moverOrig.at(w);
                        gp_Pnt mid((a.X() + b.X()) / 2, (a.Y() + b.Y()) / 2,
                                   (a.Z() + b.Z()) / 2);
                        double tm;
                        if (!projectPnt(mid,
                                        std::max(tolMoverPre,
                                                 0.3 * a.Distance(b)),
                                        &tm)) {
                            dbg("conform: face %d edge %d seg %u-%u skipped "
                                "(midpoint off curve)",
                                fid, eid, u, w);
                            continue;
                        }
                        double relm = closed
                            ? std::fmod((forward ? tm - pu : pu - tm) +
                                            period, period)
                            : (forward ? tm - pu : pu - tm);
                        if (relm < 0.1 * span || relm > 0.9 * span) {
                            dbg("conform: face %d edge %d seg %u-%u skipped "
                                "(midpoint rel %.3g of span %.4g)",
                                fid, eid, u, w, relm / span, span);
                            continue;
                        }
                    }
                    std::sort(between.begin(), between.end(),
                              [&](const EdgeParamPoint* a,
                                  const EdgeParamPoint* b) {
                                  auto key = [&](double t) {
                                      return closed
                                          ? std::fmod((forward ? t - pu
                                                               : pu - t) +
                                                          period, period)
                                          : (forward ? t - pu : pu - t);
                                  };
                                  return key(a->param) < key(b->param);
                              });
                    for (const EdgeParamPoint* c : between) {
                        ring.push_back(c->vert);
                    }
                }
                poly = std::move(ring);
            }
        }
    }
}

// Seam union v1 (the n-gon absorber, plan #37): purely topological
// T-junction healing. An OPEN directed edge (u,v) whose complement is a
// two-step path v->w->u on the neighbouring face means the neighbour
// sampled one extra vertex on the shared border; splicing w into (u,v)'s
// polygon turns a quad into a 5-gon with one short edge — exactly how
// Plasticity absorbs a denser neighbour — and the seam closes without
// moving or collapsing anything. Requiring the exact complement path
// (not curve proximity) makes sliver cross-talk impossible. Iterating
// lets chains of absorbed verts close multi-vert gaps one layer at a
// time. This pass is the contract that will let neighbouring faces
// disagree on border counts (strips vs fillet rings).
// Seam-twin fusion (decoupled seams, pre-stitch): two faces sampling a
// shared edge at the SAME curve params can still emit DISTINCT vertices
// — each side's lattice evaluates its border its own way, and the two
// samples of one contract point land a few percent of a pitch apart:
// past the weld, with nothing "between" for the stitcher to splice
// (nasty_cheese walls: both sides at t = 0, 0.113, 0.423, 0.733, 1 with
// interior verts 0.05 apart — 4 permanently open segments per face
// pair). Those twins ARE the same contract point; fusing them is the
// weld's semantic with a param-aware, seam-scoped tolerance the global
// weld could never afford. Pairs must be cross-side, mutually nearest
// in param, and within a small fraction of the local pitch in both
// param and 3D before they merge (union-find, lowest index wins).
void fuseSeamTwins(PolyMesh& mesh, const Model& model, double weldTol) {
    // Per-face boundary segments and vertex pitch (longest incident
    // boundary segment) — same qualification scaffolding as the
    // stitcher, rebuilt here because fusion must happen BEFORE the
    // stitcher reads the mesh.
    std::map<int, std::vector<size_t>> facePolys;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] > 0) {
            facePolys[mesh.polygonFaceId[p]].push_back(p);
        }
    }
    std::map<int, std::map<uint32_t, double>> facePitch;
    for (const auto& [fid, polys] : facePolys) {
        std::map<std::pair<uint32_t, uint32_t>, int> cnt;
        for (size_t p : polys) {
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                if (a > b) std::swap(a, b);
                ++cnt[{a, b}];
            }
        }
        auto& pitch = facePitch[fid];
        for (const auto& [seg, c] : cnt) {
            if (c != 1) continue;
            const auto& A = mesh.vertices[seg.first];
            const auto& B = mesh.vertices[seg.second];
            const double dx = A[0] - B[0], dy = A[1] - B[1],
                         dz = A[2] - B[2];
            const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
            // SHORTEST incident segment, not longest: fusion's scale
            // must respect the nearest distinct feature, and on a
            // hairline strip that is the strip's own width (max-pitch
            // fused foam's twin rails together — the across segment is
            // the honest bound; twins sit at a few PERCENT of a pitch).
            for (uint32_t v : {seg.first, seg.second}) {
                auto [it, fresh] = pitch.try_emplace(v, len);
                if (!fresh && len < it->second) it->second = len;
            }
        }
    }
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (const auto& v : mesh.vertices) {
        for (int c = 0; c < 3; ++c) {
            lo[c] = std::min(lo[c], v[c]);
            hi[c] = std::max(hi[c], v[c]);
        }
    }
    const double diag = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) +
                                  (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                  (hi[2] - lo[2]) * (hi[2] - lo[2]));
    const double tolCap = 0.01 * diag;
    // Dense polylines of every edge curve + per-face HOME attribution —
    // the same relative gate the stitcher uses. Without it, a hairline
    // strip's FAR rail qualified as "on" the near rail's curve (0.077
    // apart under a 0.24 band) and fused with the neighbour's twins
    // across the gap, collapsing the strip into 3-owner edges.
    constexpr int kSeg = 96;
    std::vector<std::vector<gp_Pnt>> curvePl(model.edgeCount() + 1);
    std::vector<double> curveLenAll(model.edgeCount() + 1, 0.0);
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        double cf, cl;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, cf, cl);
        if (c3.IsNull()) continue;
        auto& pl = curvePl[eid];
        pl.reserve(kSeg + 1);
        for (int k = 0; k <= kSeg; ++k) {
            pl.push_back(c3->Value(cf + (cl - cf) * k / double(kSeg)));
            if (k) curveLenAll[eid] += pl[k].Distance(pl[k - 1]);
        }
    }
    auto distToCurve = [&](const std::array<double, 3>& v, int eid) {
        const auto& pl = curvePl[eid];
        double best = 1e300;
        for (size_t k = 0; k + 1 < pl.size(); ++k) {
            const gp_XYZ a = pl[k].XYZ(), b = pl[k + 1].XYZ();
            const gp_XYZ ab = b - a;
            const gp_XYZ av(v[0] - a.X(), v[1] - a.Y(), v[2] - a.Z());
            const double ll = ab.SquareModulus();
            double t = ll > 1e-30 ? av.Dot(ab) / ll : 0.0;
            t = std::clamp(t, 0.0, 1.0);
            const gp_XYZ q = a + ab * t;
            const double d2 = gp_XYZ(v[0] - q.X(), v[1] - q.Y(), v[2] - q.Z())
                                  .SquareModulus();
            if (d2 < best) best = d2;
        }
        return std::sqrt(best);
    };
    std::map<int, std::map<uint32_t, double>> faceHome;
    for (const auto& [fid, polysUnused] : facePolys) {
        (void)polysUnused;
        std::vector<int> eids;
        for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE); ex.More();
             ex.Next()) {
            const int eid = model.edges.FindIndex(ex.Current());
            if (eid >= 1 && !curvePl[eid].empty()) eids.push_back(eid);
        }
        if (eids.empty()) continue;
        auto& home = faceHome[fid];
        for (const auto& [v, pitchUnused] : facePitch[fid]) {
            (void)pitchUnused;
            double dh = 1e300;
            for (int eid : eids) {
                dh = std::min(dh, distToCurve(mesh.vertices[v], eid));
            }
            home[v] = dh;
        }
    }
    // Union-find over fused twins.
    std::map<uint32_t, uint32_t> parent;
    std::function<uint32_t(uint32_t)> find = [&](uint32_t v) -> uint32_t {
        auto it = parent.find(v);
        if (it == parent.end() || it->second == v) return v;
        return it->second = find(it->second);
    };
    auto unite = [&](uint32_t a, uint32_t b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (b < a) std::swap(a, b);
        parent[b] = a;
    };
    int fused = 0;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        if (!model.edgeToFaces.Contains(edge)) continue;
        const auto& owners = model.edgeToFaces.FindFromKey(edge);
        if (owners.Extent() != 2) continue;
        const int fA = model.faces.FindIndex(owners.First());
        const int fB = model.faces.FindIndex(owners.Last());
        if (fA < 1 || fB < 1 || fA == fB) continue;
        if (curvePl[eid].empty()) continue;
        const std::vector<gp_Pnt>& cp = curvePl[eid];
        const double clen = curveLenAll[eid];
        if (clen < 1e-9) continue;
        const bool closedCurve =
            cp.front().Distance(cp.back()) <= std::max(weldTol, 1e-7 * clen);
        auto paramOf = [&](const std::array<double, 3>& v, double tol,
                           double& tOut) {
            const double tol2 = tol * tol;
            double best = tol2;
            bool hit = false;
            for (int k = 0; k < kSeg; ++k) {
                const gp_XYZ a = cp[k].XYZ(), b = cp[k + 1].XYZ();
                const gp_XYZ ab = b - a;
                const gp_XYZ av(v[0] - a.X(), v[1] - a.Y(), v[2] - a.Z());
                const double ll = ab.SquareModulus();
                double t = ll > 1e-30 ? av.Dot(ab) / ll : 0.0;
                t = std::clamp(t, 0.0, 1.0);
                const gp_XYZ q = a + ab * t;
                const double d2 =
                    gp_XYZ(v[0] - q.X(), v[1] - q.Y(), v[2] - q.Z())
                        .SquareModulus();
                if (d2 < best) {
                    best = d2;
                    tOut = (k + t) / double(kSeg);
                    hit = true;
                }
            }
            return hit;
        };
        // Collect each side's on-curve boundary verts (band + cap only —
        // fusion's own mutual-nearest + twin-distance rules are the
        // contamination gate here).
        struct SideVert {
            uint32_t v;
            double t;
            double pitch;
        };
        std::array<std::vector<SideVert>, 2> side;
        const int fids[2] = {fA, fB};
        for (int s2 = 0; s2 < 2; ++s2) {
            auto pit = facePitch.find(fids[s2]);
            if (pit == facePitch.end()) continue;
            const auto& home = faceHome[fids[s2]];
            for (const auto& [v, pv] : pit->second) {
                const double tolV =
                    std::max(weldTol * 4.0, std::min(tolCap, 0.25 * pv));
                double t;
                if (!paramOf(mesh.vertices[find(v)], tolV, t)) continue;
                // Home gate (same as the stitcher): this curve must be
                // (nearly) the vertex's nearest among its face's own
                // curves; the corner floor only near the curve's ends.
                auto hit = home.find(v);
                if (hit != home.end()) {
                    const double d = distToCurve(mesh.vertices[v], eid);
                    const bool nearEnd =
                        !closedCurve && (t * clen < 0.25 * pv ||
                                         (1.0 - t) * clen < 0.25 * pv);
                    double slack =
                        std::max(weldTol * 4.0, 0.5 * hit->second);
                    if (nearEnd) slack = std::max(slack, 0.05 * pv);
                    if (d > hit->second + slack) continue;
                }
                side[s2].push_back({v, t, pv});
            }
        }
        if (side[0].empty() || side[1].empty()) continue;
        // Cross-side twins: mutually nearest in param, within a small
        // fraction of the local pitch in param AND in 3D.
        auto paramDist = [&](double a, double b) {
            double d = std::abs(a - b);
            if (closedCurve) d = std::min(d, 1.0 - d);
            return d * clen;
        };
        auto nearestIn = [&](const std::vector<SideVert>& vs, double t) {
            int best = -1;
            double bd = 1e300;
            for (size_t i = 0; i < vs.size(); ++i) {
                const double d = paramDist(vs[i].t, t);
                if (d < bd) {
                    bd = d;
                    best = int(i);
                }
            }
            return best;
        };
        for (const SideVert& a : side[0]) {
            const int jb = nearestIn(side[1], a.t);
            if (jb < 0) continue;
            const SideVert& b = side[1][jb];
            if (find(a.v) == find(b.v)) continue;  // already one vertex
            const int ja = nearestIn(side[0], b.t);
            if (ja < 0 || side[0][ja].v != a.v) continue;  // not mutual
            const double pMin = std::max(1e-12, std::min(a.pitch, b.pitch));
            if (paramDist(a.t, b.t) > 0.15 * pMin) continue;
            const auto& P = mesh.vertices[find(a.v)];
            const auto& Q = mesh.vertices[find(b.v)];
            const double dx = P[0] - Q[0], dy = P[1] - Q[1],
                         dz = P[2] - Q[2];
            if (dx * dx + dy * dy + dz * dz > 0.0625 * pMin * pMin) {
                continue;  // > 25% of pitch apart: not the same point
            }
            unite(a.v, b.v);
            ++fused;
        }
    }
    if (parent.empty()) return;
    // Apply the remap; collapse consecutive repeats a merge created.
    for (auto& poly : mesh.polygons) {
        for (uint32_t& v : poly) v = find(v);
        poly.erase(std::unique(poly.begin(), poly.end()), poly.end());
        while (poly.size() > 1 && poly.front() == poly.back()) {
            poly.pop_back();
        }
    }
    // Drop polygons a fusion degenerated below a triangle.
    size_t w = 0;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygons[p].size() < 3) continue;
        if (w != p) {
            mesh.polygons[w] = std::move(mesh.polygons[p]);
            mesh.polygonFaceId[w] = mesh.polygonFaceId[p];
        }
        ++w;
    }
    mesh.polygons.resize(w);
    mesh.polygonFaceId.resize(w);
    dbg("stitch: fused %d seam twin pair(s)", fused);
}

// Decoupled-seams stitcher: the curve-guided T-junction closer. For every
// B-rep edge shared by exactly two faces, both sides' border vertices are
// located ON the edge curve (by proximity to a dense polyline of it),
// merged into one parameter-sorted chain, and each side's border polygon
// segments gain the union vertices they are missing (a quad with one
// inserted vertex becomes a 5-gon). Both sides then traverse the exact
// same vertex chain, so the seam is watertight REGARDLESS of what counts
// each face meshed at — the load-bearing pass of the decoupled-seams
// architecture, replacing forced count equality. Geometry never moves:
// only polygon connectivity gains vertices that already exist.
void stitchSeams(PolyMesh& mesh, const Model& model, double weldTol,
                 const std::map<int, FacePlan>& plans,
                 const GenerationSettings& settings,
                 DensitySolution& density) {
    // face -> polygon indices (only 2-owner edges are stitched).
    std::map<int, std::vector<size_t>> facePolys;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] > 0) {
            facePolys[mesh.polygonFaceId[p]].push_back(p);
        }
    }
    // Per-face BOUNDARY segments (undirected within-face count of 1):
    // only these may stitch — interior verts that merely pass near a
    // border curve must never be swallowed into a seam chain. Alongside,
    // each boundary vertex's longest incident boundary segment: the
    // vertex's own sampling pitch, which scales its on-curve acceptance
    // tolerance below.
    std::map<int, std::set<std::pair<uint32_t, uint32_t>>> faceBoundary;
    std::map<int, std::map<uint32_t, double>> facePitch;
    std::map<int, std::map<uint32_t, double>> facePitchMin;
    for (const auto& [fid, polys] : facePolys) {
        std::map<std::pair<uint32_t, uint32_t>, int> cnt;
        for (size_t p : polys) {
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                if (a > b) std::swap(a, b);
                ++cnt[{a, b}];
            }
        }
        auto& bset = faceBoundary[fid];
        auto& pitch = facePitch[fid];
        auto& pitchMin = facePitchMin[fid];
        for (const auto& [seg, c] : cnt) {
            if (c != 1) continue;
            bset.insert(seg);
            const auto& A = mesh.vertices[seg.first];
            const auto& B = mesh.vertices[seg.second];
            const double dx = A[0] - B[0], dy = A[1] - B[1],
                         dz = A[2] - B[2];
            const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
            for (uint32_t v : {seg.first, seg.second}) {
                auto [it, fresh] = pitch.try_emplace(v, len);
                if (!fresh && len > it->second) it->second = len;
                auto [it2, fresh2] = pitchMin.try_emplace(v, len);
                if (!fresh2 && len < it2->second) it2->second = len;
            }
        }
    }
    // Absolute drift ceiling: pitch-relative tolerances collapse when a
    // border's pitch exceeds the feature scale (a ribbon's 99-unit chord
    // put its 15% band over the OPPOSITE rail 6.7 away and swallowed it).
    // Real off-curve drift is resampling sagitta — foam's coarse can rim
    // sags 1.2 (0.5% of its diagonal) off its curve at pitch 7 — while
    // the swallowed ribbon rail sat 6.7 away (4.5% of flaregun's
    // diagonal). 1% of the diagonal separates the two.
    double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
    for (const auto& v : mesh.vertices) {
        for (int c = 0; c < 3; ++c) {
            lo[c] = std::min(lo[c], v[c]);
            hi[c] = std::max(hi[c], v[c]);
        }
    }
    const double diag = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) +
                                  (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                  (hi[2] - lo[2]) * (hi[2] - lo[2]));
    const double tolCap = 0.01 * diag;
    // Dense polylines of every edge curve, sampled once: the stitch loop
    // parameterizes against them, and the HOME attribution below compares
    // a vertex's distance to each of its face's own curves.
    constexpr int kSeg = 96;
    std::vector<std::vector<gp_Pnt>> curvePl(model.edgeCount() + 1);
    std::vector<double> curveLen(model.edgeCount() + 1, 0.0);
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        double cf, cl;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, cf, cl);
        if (c3.IsNull()) continue;
        auto& pl = curvePl[eid];
        pl.reserve(kSeg + 1);
        for (int k = 0; k <= kSeg; ++k) {
            pl.push_back(c3->Value(cf + (cl - cf) * k / double(kSeg)));
            if (k) curveLen[eid] += pl[k].Distance(pl[k - 1]);
        }
    }
    auto distToCurve = [&](const std::array<double, 3>& v, int eid) {
        const auto& pl = curvePl[eid];
        double best = 1e300;
        for (size_t k = 0; k + 1 < pl.size(); ++k) {
            const gp_XYZ a = pl[k].XYZ(), b = pl[k + 1].XYZ();
            const gp_XYZ ab = b - a;
            const gp_XYZ av(v[0] - a.X(), v[1] - a.Y(), v[2] - a.Z());
            const double ll = ab.SquareModulus();
            double t = ll > 1e-30 ? av.Dot(ab) / ll : 0.0;
            t = std::clamp(t, 0.0, 1.0);
            const gp_XYZ q = a + ab * t;
            const double d2 = gp_XYZ(v[0] - q.X(), v[1] - q.Y(), v[2] - q.Z())
                                  .SquareModulus();
            if (d2 < best) best = d2;
        }
        return std::sqrt(best);
    };
    // HOME attribution: for each face, every boundary vertex's distance to
    // the NEAREST of the face's own edge curves. A vertex may only join a
    // seam chain for a curve it is (nearly) closest to — the relative test
    // that absolute tolerances cannot express. This is what keeps a
    // hairline fillet strip's twin rail (foam: parallel lines 0.136 apart
    // against ~0.28 resampling drift) from being swallowed into the wrong
    // seam.
    std::map<int, std::map<uint32_t, double>> faceHome;
    for (const auto& [fid, polysUnused] : facePolys) {
        (void)polysUnused;
        std::vector<int> eids;
        for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE); ex.More();
             ex.Next()) {
            const int eid = model.edges.FindIndex(ex.Current());
            if (eid >= 1 && !curvePl[eid].empty()) eids.push_back(eid);
        }
        if (eids.empty()) continue;
        auto& home = faceHome[fid];
        for (const auto& [v, pitchUnused] : facePitch[fid]) {
            (void)pitchUnused;
            double dh = 1e300;
            for (int eid : eids) {
                dh = std::min(dh, distToCurve(mesh.vertices[v], eid));
            }
            home[v] = dh;
        }
    }
    struct LocalCombBridgeEdge {
        int edgeId = 0;
        int localFace = 0;
        int featureFace = 0;
        uint32_t seed = std::numeric_limits<uint32_t>::max();
    };
    std::vector<LocalCombBridgeEdge> localCombBridgeEdges;
    // The endpoint-bridge proof is still a research experiment.  Keep it
    // opt-in until the entire proposal is transactional: earlier versions
    // could reject the final bridge batch after already rewriting seam
    // polygons, which regressed the accepted MP9 handgrip topology.  The
    // ordinary shared-edge stitch path remains the production default.
    const bool enableExperimentalLocalCombBridge =
        std::getenv("WEFT_ENABLE_EXPERIMENTAL_LOCAL_COMB_BRIDGE") != nullptr;
    int spliced = 0;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;
        if (!model.edgeToFaces.Contains(edge)) continue;
        // `auto`, not the named list type: TopTools_ListOfShape is only
        // transitively declared on some OCCT header layouts (MSVC/vcpkg
        // 7.8 broke on the explicit name).
        const auto& owners = model.edgeToFaces.FindFromKey(edge);
        if (owners.Extent() != 2) continue;
        const int fA = model.faces.FindIndex(owners.First());
        const int fB = model.faces.FindIndex(owners.Last());
        if (fA < 1 || fB < 1 || fA == fB) continue;
        if (curvePl[eid].empty()) continue;
        const std::vector<gp_Pnt>& cp = curvePl[eid];
        const double clen = curveLen[eid];
        if (clen < 1e-9) continue;
        // Closed ring: params wrap, and there are no ends for corner
        // verts to live at.
        const bool isClosedPl =
            cp.front().Distance(cp.back()) <=
            std::max(weldTol, 1e-7 * clen);
        // On-curve tolerance: PITCH-scaled, not curve-length-scaled. A
        // lattice-resampled border vertex strays off the exact curve by
        // the sagitta of its own sampling pitch (~turn/8 of a segment's
        // turn angle, <= ~13% at the 60 deg/segment curvature floor) plus
        // whatever the input's own curve/surface disagreement adds
        // (teleporter carries borders ~19% of pitch off their curve) —
        // while a foreign border vertex one cell away sits a FULL pitch
        // off. 25% of the vertex's own pitch admits the real drift; the
        // HOME attribution + already-closed-segment guards carry the
        // contamination defense. (The old 0.4%-of-curve-length tolerance
        // failed both ways: too tight for coarse rims on short edges, and
        // on a long edge it grew past the cell size and swallowed
        // neighbouring seams' verts near corners.)
        auto paramOf = [&](const std::array<double, 3>& v, double tol,
                           double& tOut) {
            const double tol2 = tol * tol;
            double best = tol2;
            bool hit = false;
            for (int k = 0; k < kSeg; ++k) {
                const gp_XYZ a = cp[k].XYZ(), b = cp[k + 1].XYZ();
                const gp_XYZ ab = b - a;
                const gp_XYZ av(v[0] - a.X(), v[1] - a.Y(), v[2] - a.Z());
                const double ll = ab.SquareModulus();
                double t = ll > 1e-30 ? av.Dot(ab) / ll : 0.0;
                t = std::clamp(t, 0.0, 1.0);
                const gp_XYZ q = a + ab * t;
                const double d2 =
                    gp_XYZ(v[0] - q.X(), v[1] - q.Y(), v[2] - q.Z())
                        .SquareModulus();
                if (d2 < best) {
                    best = d2;
                    tOut = (k + t) / double(kSeg);
                    hit = true;
                }
            }
            return hit;
        };
        // Each side's on-curve verts, then the union chain.
        const char* dbgEidEnv = std::getenv("WEFT_STITCH_EID");
        const bool traceEid = dbgEidEnv && std::atoi(dbgEidEnv) == eid;
        std::map<uint32_t, double> tOf;  // vert -> curve param (union)
        std::array<std::set<uint32_t>, 2> sideVerts;
        const int fids[2] = {fA, fB};
        for (int s2 = 0; s2 < 2; ++s2) {
            auto bit = faceBoundary.find(fids[s2]);
            if (bit == faceBoundary.end()) continue;
            const auto& pitch = facePitch[fids[s2]];
            const auto& pitchMin = facePitchMin[fids[s2]];
            const auto& home = faceHome[fids[s2]];
            std::set<uint32_t> seen;
            for (const auto& [a, b] : bit->second) {
                for (uint32_t v : {a, b}) {
                    if (!seen.insert(v).second) continue;
                    auto pit = pitch.find(v);
                    const double pv =
                        pit != pitch.end() ? pit->second : 0.0;
                    auto pmt = pitchMin.find(v);
                    const double pvMin =
                        pmt != pitchMin.end() ? pmt->second : 0.0;
                    // 35%: coons rails' UV-interpolated resampling on
                    // curved bsplines drifts past the sagitta model
                    // (teleporter face 301: 27% of pitch off its rail
                    // curve). Foreign verts still sit a FULL pitch off;
                    // home attribution + the closed-segment guards are
                    // the real contamination defense.
                    const double tolV = std::max(
                        weldTol * 4.0, std::min(tolCap, 0.35 * pv));
                    double t;
                    if (!paramOf(mesh.vertices[v], tolV, t)) {
                        if (traceEid) {
                            dbg("stitch eid %d f%d v%u REJ band d=%.4g "
                                "tolV=%.4g pitch=%.4g",
                                eid, fids[s2], v,
                                distToCurve(mesh.vertices[v], eid), tolV,
                                pv);
                        }
                        continue;
                    }
                    // Home test: this curve must be (nearly) the
                    // vertex's nearest among its face's own curves.
                    // The pitch-scaled slack floor exists ONLY for true
                    // corner verts — which sit a hair off BOTH adjacent
                    // curves — and corners live at the curve's ENDS, so
                    // the floor is gated on end-proximity: an interior-
                    // param vertex gets no floor (a chamfer ring's far
                    // rim, one ring-width away at interior params,
                    // otherwise rides any pitch-scaled floor in).
                    auto hit = home.find(v);
                    if (hit != home.end()) {
                        const double d = distToCurve(mesh.vertices[v], eid);
                        const bool nearEnd =
                            !isClosedPl &&
                            (t * clen < 0.25 * pv ||
                             (1.0 - t) * clen < 0.25 * pv);
                        double slack = std::max(weldTol * 4.0,
                                                0.5 * hit->second);
                        if (nearEnd) {
                            slack = std::max(slack, 0.05 * pv);
                        }
                        if (d > hit->second + slack) {
                            if (traceEid) {
                                dbg("stitch eid %d f%d v%u REJ home "
                                    "d=%.4g dHome=%.4g slack=%.4g",
                                    eid, fids[s2], v, d, hit->second,
                                    slack);
                            }
                            continue;
                        }
                    }
                    sideVerts[s2].insert(v);
                    tOf[v] = t;
                }
            }
        }
        if (traceEid) {
            dbg("stitch eid %d: sides %zu/%zu (f%d/f%d)", eid,
                sideVerts[0].size(), sideVerts[1].size(), fA, fB);
            for (int s2 = 0; s2 < 2; ++s2) {
                auto bit = faceBoundary.find(fids[s2]);
                if (bit == faceBoundary.end()) continue;
                std::set<uint32_t> boundaryVerts;
                for (const auto& [a, b] : bit->second) {
                    boundaryVerts.insert(a);
                    boundaryVerts.insert(b);
                }
                for (int end = 0; end < 2; ++end) {
                    const gp_Pnt& q = end ? cp.back() : cp.front();
                    std::vector<std::pair<double, uint32_t>> nearest;
                    for (uint32_t v : boundaryVerts) {
                        const auto& p = mesh.vertices[v];
                        const double dx = p[0] - q.X();
                        const double dy = p[1] - q.Y();
                        const double dz = p[2] - q.Z();
                        nearest.push_back({std::sqrt(dx * dx + dy * dy +
                                                     dz * dz),
                                           v});
                    }
                    std::sort(nearest.begin(), nearest.end());
                    for (size_t k = 0; k < std::min<size_t>(4, nearest.size());
                         ++k) {
                        const auto [d, v] = nearest[k];
                        const auto pit = facePitch[fids[s2]].find(v);
                        const double pv = pit != facePitch[fids[s2]].end()
                                              ? pit->second
                                              : 0.0;
                        dbg("stitch eid %d f%d end%d candidate%zu v%u "
                            "dEnd=%.4g pitch=%.4g admitted=%d",
                            eid, fids[s2], end, k, v, d, pv,
                            sideVerts[s2].count(v) ? 1 : 0);
                    }
                }
            }
            dbg("stitch eid %d tolerances edge=%.4g faces=%.4g/%.4g "
                "tolCap=%.4g",
                eid, BRep_Tool::Tolerance(edge),
                BRep_Tool::Tolerance(TopoDS::Face(model.faces(fA))),
                BRep_Tool::Tolerance(TopoDS::Face(model.faces(fB))), tolCap);
        }
        const auto planA = plans.find(fA);
        const auto planB = plans.find(fB);
        const bool combA = planA != plans.end() &&
                           planA->second.orthogonalLocalComb;
        const bool combB = planB != plans.end() &&
                           planB->second.orthogonalLocalComb;
        if (enableExperimentalLocalCombBridge && combA != combB) {
            const int localSide = combA ? 0 : 1;
            const FacePlan& localPlan = combA ? planA->second : planB->second;
            const bool horizontalClosure =
                std::find(localPlan.uEdges.begin(), localPlan.uEdges.end(),
                          eid) != localPlan.uEdges.end();
            const bool pinned = settings.perEdge.count(eid) != 0 ||
                density.pinnedRoots.count(density.groups.find(eid)) != 0;
            const int solved = density.countFor(eid, 1);
            if (horizontalClosure && solved <= 3 && !pinned &&
                sideVerts[localSide].size() < 2) {
                uint32_t seed = std::numeric_limits<uint32_t>::max();
                double seedDistance = 1e300;
                for (int side = 0; side < 2; ++side) {
                    for (uint32_t v : sideVerts[side]) {
                        const auto& p = mesh.vertices[v];
                        const auto distanceTo = [&](const gp_Pnt& q) {
                            const double dx = p[0] - q.X();
                            const double dy = p[1] - q.Y();
                            const double dz = p[2] - q.Z();
                            return std::sqrt(dx * dx + dy * dy + dz * dz);
                        };
                        const double d = std::min(distanceTo(cp.front()),
                                                  distanceTo(cp.back()));
                        if (d < seedDistance) {
                            seedDistance = d;
                            seed = v;
                        }
                    }
                }
                if (seed != std::numeric_limits<uint32_t>::max()) {
                    localCombBridgeEdges.push_back(
                        {eid, fids[localSide], fids[1 - localSide], seed});
                }
                if (traceEid) {
                    dbg("stitch eid %d: defer local-comb endpoint bridge "
                        "(local f%d, feature f%d, solved %d, seed v%u)",
                        eid, fids[localSide], fids[1 - localSide], solved,
                        seed);
                }
                continue;
            }
        }
        if (sideVerts[0].empty() || sideVerts[1].empty()) continue;
        // Already agreeing (welded shared chain)? Nothing to do.
        if (sideVerts[0] == sideVerts[1]) continue;
        std::vector<std::pair<double, uint32_t>> chain;
        for (const auto& [v, t] : tOf) chain.push_back({t, v});
        std::sort(chain.begin(), chain.end());
        // Insert missing union verts into each side's border segments.
        for (int s2 = 0; s2 < 2; ++s2) {
            auto it = facePolys.find(fids[s2]);
            if (it == facePolys.end()) continue;
            const auto& bset = faceBoundary[fids[s2]];
            const auto& oset = faceBoundary[fids[1 - s2]];
            for (size_t p : it->second) {
                auto& poly = mesh.polygons[p];
                for (size_t i = 0; i < poly.size(); ++i) {
                    const uint32_t a = poly[i];
                    const uint32_t b = poly[(i + 1) % poly.size()];
                    if (!sideVerts[s2].count(a) || !sideVerts[s2].count(b)) {
                        continue;
                    }
                    // Only true BOUNDARY segments stitch — a diagonal or
                    // interior chord between two on-curve verts is not a
                    // seam.
                    const auto seg =
                        std::make_pair(std::min(a, b), std::max(a, b));
                    if (!bset.count(seg)) continue;
                    // A segment the OTHER face also traverses is a seam
                    // already closed — splicing a near-curve straggler
                    // from a neighbouring border into it is how corner
                    // contamination manufactured non-manifold edges.
                    if (oset.count(seg)) continue;
                    double ta = tOf[a], tb = tOf[b];
                    if (std::abs(ta - tb) < 1e-12) continue;
                    // The chord must LIE on the curve (not a cap corner
                    // whose two endpoints merely touch it): its midpoint
                    // stands off by the chord's own sagitta — up to ~29%
                    // of the chord at 120 deg of turn per segment on a
                    // coarse ring — while a corner chord cutting across
                    // the face sits far inside (and the home attribution
                    // + owner + boundary-segment gates already screen
                    // it).
                    const auto& A = mesh.vertices[a];
                    const auto& B = mesh.vertices[b];
                    const double abx = A[0] - B[0], aby = A[1] - B[1],
                                 abz = A[2] - B[2];
                    const double chordLen =
                        std::sqrt(abx * abx + aby * aby + abz * abz);
                    std::array<double, 3> mid{(A[0] + B[0]) / 2,
                                              (A[1] + B[1]) / 2,
                                              (A[2] + B[2]) / 2};
                    // A chord whose two endpoints sit AT the open
                    // curve's own terminals IS the whole edge (minimal
                    // n-gon plates take one segment per B-rep edge) —
                    // no midpoint test can vouch for it (a half-circle
                    // chord's sagitta is 50% of the chord), and none is
                    // needed: terminal-to-terminal leaves no ambiguity
                    // about which span to stitch.
                    double tm = 0.5 * (ta + tb);
                    bool fullEdge = false;
                    if (!isClosedPl) {
                        const double eTol =
                            std::max(weldTol * 8.0, 1e-5 * clen);
                        auto near3 = [&](const std::array<double, 3>& P,
                                         const gp_Pnt& Q) {
                            const double dx = P[0] - Q.X(),
                                         dy = P[1] - Q.Y(),
                                         dz = P[2] - Q.Z();
                            return dx * dx + dy * dy + dz * dz <
                                   eTol * eTol;
                        };
                        fullEdge = (near3(A, cp.front()) &&
                                    near3(B, cp.back())) ||
                                   (near3(A, cp.back()) &&
                                    near3(B, cp.front()));
                    }
                    // No tolCap here: a coarse ring's 27-unit chord has a
                    // 3-unit sagitta (legit, > any absolute cap), and the
                    // contamination gate is the ENDPOINT vetting above —
                    // a chord can't reach this test unless both ends
                    // passed the capped band + home checks.
                    if (!fullEdge &&
                        !paramOf(mid,
                                 std::max(weldTol * 4.0, 0.35 * chordLen),
                                 tm)) {
                        if (traceEid) {
                            dbg("stitch eid %d f%d seg v%u-v%u REJ mid "
                                "dMid=%.4g chord=%.4g",
                                eid, fids[s2], a, b,
                                distToCurve(mid, eid), chordLen);
                        }
                        continue;
                    }
                    if (traceEid) {
                        dbg("stitch eid %d f%d seg v%u-v%u ta=%.5f "
                            "tb=%.5f",
                            eid, fids[s2], a, b, ta, tb);
                    }
                    // Which verts lie between a and b? On a CLOSED curve
                    // params wrap, so "between" is ambiguous — a segment
                    // hugging the wrap point read as spanning the whole
                    // circle and swallowed every vertex of the ring
                    // (teleporter bores: one 4%-arc chord gained all 12
                    // of the far side's verts). The chord MIDPOINT's
                    // param (already located) picks the true arc: map
                    // every param to r = (t - ta) mod 1 and walk toward
                    // b on the side that contains the midpoint.
                    const bool closedCurve = isClosedPl;
                    auto relOf = [&](double t) {
                        double r = t - ta;
                        r -= std::floor(r);
                        return r;
                    };
                    const double rb = relOf(tb);
                    const double rm = relOf(tm);
                    // direct = walking a->b through ascending r covers
                    // the chord's own arc (contains the midpoint).
                    const bool direct = !closedCurve || rm <= rb;
                    std::vector<uint32_t> ins;  // ascending r
                    auto nearVert = [&](uint32_t v, uint32_t w) {
                        const auto& P = mesh.vertices[v];
                        const auto& Q = mesh.vertices[w];
                        const double dx = P[0] - Q[0], dy = P[1] - Q[1],
                                     dz = P[2] - Q[2];
                        return dx * dx + dy * dy + dz * dz <
                               weldTol * weldTol * 4.0;
                    };
                    for (const auto& [t, v] : chain) {
                        if (v == a || v == b) continue;
                        // A vertex this face's border ALREADY traverses
                        // must never be inserted a second time — the
                        // duplicate traversal is a non-manifold edge by
                        // construction (the other corner-contamination
                        // half).
                        if (sideVerts[s2].count(v)) continue;
                        if (closedCurve) {
                            const double r = relOf(t);
                            if (direct) {
                                if (r <= 1e-12 || r >= rb - 1e-12) continue;
                            } else {
                                if (r <= rb + 1e-12 || r >= 1.0 - 1e-12) {
                                    continue;
                                }
                            }
                        } else if (t <= std::min(ta, tb) + 1e-12 ||
                                   t >= std::max(ta, tb) - 1e-12) {
                            continue;
                        }
                        // A chain vert coincident with the segment's own
                        // endpoints (or the previous insertion) is the
                        // weld's near-duplicate — inserting it would
                        // traverse a zero-length seam segment twice.
                        if (nearVert(v, a) || nearVert(v, b)) continue;
                        ins.push_back(v);
                    }
                    if (ins.empty()) continue;
                    // Order along the walk a->b. chain is sorted by raw
                    // t; re-sort by r so a wrapped interval stays one
                    // monotone run, then flip when the walk descends.
                    std::sort(ins.begin(), ins.end(),
                              [&](uint32_t x, uint32_t y) {
                                  return (closedCurve ? relOf(tOf[x])
                                                      : tOf[x]) <
                                         (closedCurve ? relOf(tOf[y])
                                                      : tOf[y]);
                              });
                    const bool fwd =
                        closedCurve ? direct : tb > ta;
                    if (!fwd) std::reverse(ins.begin(), ins.end());
                    // Drop weld-coincident neighbours in FINAL order.
                    {
                        std::vector<uint32_t> dedup;
                        for (uint32_t v : ins) {
                            if (!dedup.empty() && nearVert(v, dedup.back())) {
                                continue;
                            }
                            dedup.push_back(v);
                        }
                        ins = std::move(dedup);
                    }
                    if (ins.empty()) continue;
                    if (std::getenv("WEFT_STITCH_DEBUG")) {
                        std::string s;
                        for (uint32_t v : ins) {
                            s += " v" + std::to_string(v);
                        }
                        dbg("stitch: eid %d face %d seg v%u-v%u gains%s",
                            eid, fids[s2], a, b, s.c_str());
                    }
                    poly.insert(poly.begin() + i + 1, ins.begin(),
                                ins.end());
                    for (uint32_t v : ins) sideVerts[s2].insert(v);
                    spliced += int(ins.size());
                    i += ins.size();  // continue after the insertion
                }
            }
        }
    }
    // A local-comb's short horizontal closures are deliberately not widened
    // into the global structural lattice.  At a CAD-vertex star that can
    // leave several already-meshed face borders forming one small open loop:
    // there is no honest local segment for the ordinary union-chain splice
    // above to rewrite.  Close only such proven loops, using existing border
    // vertices.  No vertex moves, no sample/count changes, and no existing
    // polygon is rewritten.
    int bridgePolygons = 0;
    if (!localCombBridgeEdges.empty()) {
        struct OpenEdge {
            uint32_t a = 0, b = 0;
            int face = 0;
        };
        using EdgeKey = std::pair<uint32_t, uint32_t>;
        auto edgeKey = [](uint32_t a, uint32_t b) {
            return std::make_pair(std::min(a, b), std::max(a, b));
        };
        std::map<EdgeKey, int> edgeUses;
        std::map<EdgeKey, std::vector<std::pair<uint32_t, uint32_t>>>
            edgeDirections;
        for (size_t pi = 0; pi < mesh.polygons.size(); ++pi) {
            const auto& poly = mesh.polygons[pi];
            for (size_t k = 0; k < poly.size(); ++k) {
                const uint32_t a = poly[k], b = poly[(k + 1) % poly.size()];
                ++edgeUses[edgeKey(a, b)];
                edgeDirections[edgeKey(a, b)].push_back({a, b});
            }
        }

        std::vector<OpenEdge> open;
        std::map<uint32_t, std::vector<size_t>> incident, outgoing, incoming;
        for (size_t pi = 0; pi < mesh.polygons.size(); ++pi) {
            const auto& poly = mesh.polygons[pi];
            for (size_t k = 0; k < poly.size(); ++k) {
                const uint32_t a = poly[k], b = poly[(k + 1) % poly.size()];
                if (edgeUses[edgeKey(a, b)] != 1) continue;
                const size_t oi = open.size();
                open.push_back({a, b, mesh.polygonFaceId[pi]});
                incident[a].push_back(oi);
                incident[b].push_back(oi);
                outgoing[a].push_back(oi);
                incoming[b].push_back(oi);
            }
        }

        struct OpenComponent {
            std::vector<size_t> edges;
            std::set<uint32_t> vertices;
        };
        std::vector<OpenComponent> components;
        std::vector<int> componentOfOpen(open.size(), -1);
        std::map<uint32_t, int> componentOfVertex;
        for (size_t first = 0; first < open.size(); ++first) {
            if (componentOfOpen[first] >= 0) continue;
            const int ci = static_cast<int>(components.size());
            components.push_back({});
            std::vector<size_t> stack{first};
            componentOfOpen[first] = ci;
            while (!stack.empty()) {
                const size_t oi = stack.back();
                stack.pop_back();
                components[ci].edges.push_back(oi);
                for (uint32_t v : {open[oi].a, open[oi].b}) {
                    components[ci].vertices.insert(v);
                    componentOfVertex[v] = ci;
                    for (size_t next : incident[v]) {
                        if (componentOfOpen[next] >= 0) continue;
                        componentOfOpen[next] = ci;
                        stack.push_back(next);
                    }
                }
            }
        }

        std::map<int, std::vector<size_t>> candidatesByComponent;
        bool candidateMappingOk = true;
        for (size_t i = 0; i < localCombBridgeEdges.size(); ++i) {
            const auto found = componentOfVertex.find(
                localCombBridgeEdges[i].seed);
            if (found == componentOfVertex.end()) {
                candidateMappingOk = false;
                break;
            }
            candidatesByComponent[found->second].push_back(i);
        }

        struct BridgeProposal {
            std::vector<uint32_t> ring;
            int face = 0;
        };
        struct SingleBridgeStar {
            size_t proposalIndex = 0;
            size_t candidateIndex = 0;
            std::vector<uint32_t> cycle;
            std::vector<int> cycleFaces;
        };
        struct SharedLocalChord {
            EdgeKey key{};
            int edgeId = 0;
            int faceA = 0;
            int faceB = 0;
            double first = 0.0;
            double last = 0.0;
            double parameterA = 0.0;
            double parameterB = 0.0;
            double tolerance = 0.0;
        };
        std::vector<BridgeProposal> proposals;
        std::vector<SingleBridgeStar> singleBridgeStars;
        std::set<int> selectedComponents;
        if (candidateMappingOk) {
            for (const auto& [ci, candidateIds] : candidatesByComponent) {
                if (ci < 0 || ci >= static_cast<int>(components.size()) ||
                    candidateIds.empty() || candidateIds.size() > 2) {
                    candidateMappingOk = false;
                    break;
                }
                const OpenComponent& component = components[ci];
                bool simpleDirectedCycle =
                    component.edges.size() == component.vertices.size() &&
                    component.edges.size() >= 3 &&
                    component.edges.size() <= 64;
                for (uint32_t v : component.vertices) {
                    simpleDirectedCycle &= outgoing[v].size() == 1 &&
                                           incoming[v].size() == 1;
                }
                if (!simpleDirectedCycle) {
                    candidateMappingOk = false;
                    break;
                }

                std::vector<uint32_t> cycle;
                std::vector<int> cycleFaces;
                uint32_t current = *component.vertices.begin();
                const uint32_t start = current;
                do {
                    if (cycle.size() > component.edges.size()) {
                        simpleDirectedCycle = false;
                        break;
                    }
                    cycle.push_back(current);
                    const OpenEdge& oe = open[outgoing[current].front()];
                    cycleFaces.push_back(oe.face);
                    current = oe.b;
                } while (current != start);
                if (!simpleDirectedCycle ||
                    cycle.size() != component.edges.size()) {
                    candidateMappingOk = false;
                    break;
                }

                if (candidateIds.size() == 1) {
                    const auto& candidate =
                        localCombBridgeEdges[candidateIds.front()];
                    if (std::find(cycleFaces.begin(), cycleFaces.end(),
                                  candidate.localFace) == cycleFaces.end()) {
                        candidateMappingOk = false;
                        break;
                    }
                    std::reverse(cycle.begin(), cycle.end());
                    std::reverse(cycleFaces.begin(), cycleFaces.end());
                    // Keep an unmodified directed copy for the owner-sector
                    // repartition below; the placeholder preserves the first
                    // edge-use simulation until exact BRep samples exist.
                    std::vector<uint32_t> directedCycle = cycle;
                    std::vector<int> directedFaces = cycleFaces;
                    std::reverse(directedCycle.begin(), directedCycle.end());
                    std::reverse(directedFaces.begin(), directedFaces.end());
                    singleBridgeStars.push_back(
                        {proposals.size(), candidateIds.front(),
                         std::move(directedCycle), std::move(directedFaces)});
                    proposals.push_back({std::move(cycle),
                                         candidate.localFace});
                    selectedComponents.insert(ci);
                    continue;
                }

                const auto& c0 = localCombBridgeEdges[candidateIds[0]];
                const auto& c1 = localCombBridgeEdges[candidateIds[1]];
                if (c0.localFace == c1.localFace ||
                    c0.featureFace == c1.featureFace) {
                    candidateMappingOk = false;
                    break;
                }
                const std::set<int> localFaces{c0.localFace, c1.localFace};
                const std::set<int> featureFaces{c0.featureFace,
                                                 c1.featureFace};
                std::vector<size_t> localJunctions, featureJunctions;
                for (size_t k = 0; k < cycle.size(); ++k) {
                    const int before =
                        cycleFaces[(k + cycle.size() - 1) % cycle.size()];
                    const int after = cycleFaces[k];
                    if (before != after && localFaces.count(before) &&
                        localFaces.count(after)) {
                        localJunctions.push_back(k);
                    }
                    if (before != after && featureFaces.count(before) &&
                        featureFaces.count(after)) {
                        featureJunctions.push_back(k);
                    }
                }
                if (localJunctions.size() != 1 ||
                    featureJunctions.size() != 1 ||
                    localJunctions.front() == featureJunctions.front()) {
                    candidateMappingOk = false;
                    break;
                }

                auto makeArc = [&](size_t from, size_t to) {
                    std::vector<uint32_t> ring;
                    std::set<int> arcFaces;
                    size_t k = from;
                    ring.push_back(cycle[k]);
                    while (k != to) {
                        arcFaces.insert(cycleFaces[k]);
                        k = (k + 1) % cycle.size();
                        ring.push_back(cycle[k]);
                    }
                    return std::make_pair(std::move(ring),
                                          std::move(arcFaces));
                };
                auto arc0 = makeArc(featureJunctions.front(),
                                    localJunctions.front());
                auto arc1 = makeArc(localJunctions.front(),
                                    featureJunctions.front());
                std::array<std::pair<std::vector<uint32_t>, std::set<int>>, 2>
                    arcs{std::move(arc0), std::move(arc1)};
                std::set<size_t> assigned;
                for (auto& arc : arcs) {
                    size_t match = candidateIds.size();
                    for (size_t k = 0; k < candidateIds.size(); ++k) {
                        const auto& candidate =
                            localCombBridgeEdges[candidateIds[k]];
                        if (arc.second.count(candidate.localFace) &&
                            arc.second.count(candidate.featureFace)) {
                            if (match != candidateIds.size()) {
                                match = candidateIds.size();
                                break;
                            }
                            match = k;
                        }
                    }
                    if (match == candidateIds.size() ||
                        !assigned.insert(match).second ||
                        arc.first.size() < 3) {
                        candidateMappingOk = false;
                        break;
                    }
                    std::reverse(arc.first.begin(), arc.first.end());
                    proposals.push_back(
                        {std::move(arc.first),
                         localCombBridgeEdges[candidateIds[match]].localFace});
                }
                if (!candidateMappingOk || assigned.size() != 2) break;
                selectedComponents.insert(ci);
            }
        }

        // Simulate every edge use before touching the mesh.  Existing loop
        // edges must gain exactly one opposite use.  A split two-face batch
        // may introduce one chord, but that chord must occur twice in
        // opposite directions within the same atomic proposal set.
        std::map<EdgeKey, int> coveredOpen;
        std::map<EdgeKey, std::vector<std::pair<uint32_t, uint32_t>>> newEdges;
        if (candidateMappingOk) {
            for (const BridgeProposal& proposal : proposals) {
                const std::set<uint32_t> unique(proposal.ring.begin(),
                                                proposal.ring.end());
                if (proposal.ring.size() < 3 ||
                    unique.size() != proposal.ring.size()) {
                    candidateMappingOk = false;
                    break;
                }
                for (size_t k = 0; k < proposal.ring.size(); ++k) {
                    const uint32_t a = proposal.ring[k];
                    const uint32_t b =
                        proposal.ring[(k + 1) % proposal.ring.size()];
                    const EdgeKey key = edgeKey(a, b);
                    const int uses = edgeUses[key];
                    if (uses == 1) {
                        const auto& directions = edgeDirections[key];
                        if (directions.size() != 1 ||
                            directions.front() != std::make_pair(b, a)) {
                            candidateMappingOk = false;
                            break;
                        }
                        ++coveredOpen[key];
                    } else if (uses == 0) {
                        newEdges[key].push_back({a, b});
                    } else {
                        candidateMappingOk = false;
                        break;
                    }
                }
                if (!candidateMappingOk) break;
            }
        }
        if (candidateMappingOk) {
            for (int ci : selectedComponents) {
                for (size_t oi : components[ci].edges) {
                    if (coveredOpen[edgeKey(open[oi].a, open[oi].b)] != 1) {
                        candidateMappingOk = false;
                        break;
                    }
                }
                if (!candidateMappingOk) break;
            }
        }
        if (candidateMappingOk) {
            for (const auto& [key, directions] : newEdges) {
                if (directions.size() != 2 ||
                    directions[0] !=
                        std::make_pair(directions[1].second,
                                       directions[1].first)) {
                    candidateMappingOk = false;
                    break;
                }
            }
        }

        // A two-face split introduces one synthetic chord between the two
        // local-comb owners.  Its current endpoints are reconstructed cell
        // corners and need not lie on the owners' exact shared BRep edge. Add
        // only the missing exact edge samples at the uniquely nearest bounded
        // parameters, then replace the chord by the shared connector chain.
        // Existing vertices, cells, columns and rib stations remain untouched.
        const size_t bridgeVertexBase = mesh.vertices.size();
        const size_t bridgeAnchorBase = mesh.anchors.size();
        const size_t bridgeConstraintBase = mesh.constraints.size();
        const bool haveBridgeConstraints =
            mesh.constraints.size() == mesh.vertices.size();
        std::map<EdgeKey, SharedLocalChord> sharedLocalChords;
        size_t pairedChordCount = 0;
        size_t extraStarProposals = 0;
        if (candidateMappingOk) {
            auto nearestBoundedParameter = [&](const TopoDS_Edge& edge,
                                               uint32_t vi,
                                               double& parameter,
                                               double& distance) {
                if (vi >= mesh.vertices.size()) return false;
                BRepAdaptor_Curve curve(edge);
                const double first = curve.FirstParameter();
                const double last = curve.LastParameter();
                if (!std::isfinite(first) || !std::isfinite(last) ||
                    last <= first)
                    return false;
                const auto& v = mesh.vertices[vi];
                const gp_Pnt p(v[0], v[1], v[2]);
                struct Candidate {
                    double distance = 0.0;
                    double parameter = 0.0;
                };
                std::vector<Candidate> candidates{
                    {p.Distance(curve.Value(first)), first},
                    {p.Distance(curve.Value(last)), last}};
                try {
                    Extrema_ExtPC extrema(p, curve);
                    if (extrema.IsDone()) {
                        for (int i = 1; i <= extrema.NbExt(); ++i) {
                            const double candidate =
                                extrema.Point(i).Parameter();
                            const double parameterTolerance = std::max(
                                Precision::PConfusion(),
                                1e-10 * std::abs(last - first));
                            if (candidate < first - parameterTolerance ||
                                candidate > last + parameterTolerance)
                                continue;
                            candidates.push_back(
                                {std::sqrt(extrema.SquareDistance(i)),
                                 candidate});
                        }
                    }
                } catch (...) {
                    // Exact endpoints remain bounded candidates.
                }
                const double parameterTolerance = std::max(
                    Precision::PConfusion(),
                    1e-10 * std::abs(last - first));
                std::sort(candidates.begin(), candidates.end(),
                          [](const Candidate& a, const Candidate& b) {
                              if (a.distance != b.distance)
                                  return a.distance < b.distance;
                              return a.parameter < b.parameter;
                          });
                std::vector<Candidate> distinct;
                for (const Candidate& candidate : candidates) {
                    bool duplicate = false;
                    for (const Candidate& kept : distinct) {
                        if (std::abs(candidate.parameter - kept.parameter) <=
                            parameterTolerance) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (!duplicate) distinct.push_back(candidate);
                }
                if (distinct.empty()) return false;
                const double uniquenessTolerance = std::max(
                    Precision::Confusion(),
                    1e-9 * std::max(1.0, distinct.front().distance));
                if (distinct.size() > 1 &&
                    distinct[1].distance - distinct[0].distance <=
                        uniquenessTolerance)
                    return false;
                parameter = std::clamp(distinct.front().parameter, first,
                                       last);
                distance = distinct.front().distance;
                return distance <= tolCap;
            };
            auto edgeOrientationOnFace = [&](int faceId, int edgeId,
                                             TopAbs_Orientation& orientation) {
                int occurrences = 0;
                for (TopExp_Explorer ex(model.faces(faceId), TopAbs_EDGE);
                     ex.More(); ex.Next()) {
                    if (model.edges.FindIndex(ex.Current()) != edgeId)
                        continue;
                    orientation = ex.Current().Orientation();
                    ++occurrences;
                }
                return occurrences == 1 &&
                       (orientation == TopAbs_FORWARD ||
                        orientation == TopAbs_REVERSED);
            };

            const auto syntheticChords = newEdges;
            for (const auto& [key, directions] : syntheticChords) {
                std::vector<std::pair<int, std::pair<uint32_t, uint32_t>>>
                    proposalUses;
                for (const BridgeProposal& proposal : proposals) {
                    for (size_t k = 0; k < proposal.ring.size(); ++k) {
                        const uint32_t a = proposal.ring[k];
                        const uint32_t b = proposal.ring[
                            (k + 1) % proposal.ring.size()];
                        if (edgeKey(a, b) == key)
                            proposalUses.push_back({proposal.face, {a, b}});
                    }
                }
                if (proposalUses.size() != 2 ||
                    proposalUses[0].first == proposalUses[1].first) {
                    candidateMappingOk = false;
                    break;
                }
                const int faceA = proposalUses[0].first;
                const int faceB = proposalUses[1].first;
                const auto planForA = plans.find(faceA);
                const auto planForB = plans.find(faceB);
                if (planForA == plans.end() || planForB == plans.end() ||
                    !planForA->second.orthogonalLocalComb ||
                    !planForB->second.orthogonalLocalComb) {
                    candidateMappingOk = false;
                    break;
                }

                std::set<int> possibleEdges;
                for (TopExp_Explorer ex(model.faces(faceA), TopAbs_EDGE);
                     ex.More(); ex.Next()) {
                    const int edgeId = model.edges.FindIndex(ex.Current());
                    if (edgeId < 1 || possibleEdges.count(edgeId)) continue;
                    const TopoDS_Edge edge =
                        TopoDS::Edge(model.edges(edgeId));
                    if (BRep_Tool::Degenerated(edge) ||
                        !model.edgeToFaces.Contains(edge))
                        continue;
                    const auto& owners = model.edgeToFaces.FindFromKey(edge);
                    if (owners.Extent() != 2) continue;
                    std::set<int> ownerFaces;
                    for (const TopoDS_Shape& owner : owners) {
                        ownerFaces.insert(model.faces.FindIndex(owner));
                    }
                    if (ownerFaces == std::set<int>{faceA, faceB})
                        possibleEdges.insert(edgeId);
                }

                if (possibleEdges.size() != 1) {
                    candidateMappingOk = false;
                    break;
                }
                const int edgeId = *possibleEdges.begin();
                const TopoDS_Edge edge = TopoDS::Edge(model.edges(edgeId));
                BRepAdaptor_Curve curve(edge);
                const double first = curve.FirstParameter();
                const double last = curve.LastParameter();
                const double tolerance = std::max(
                    {weldTol, BRep_Tool::Tolerance(edge),
                     BRep_Tool::Tolerance(
                         TopoDS::Face(model.faces(faceA))),
                     BRep_Tool::Tolerance(
                         TopoDS::Face(model.faces(faceB))),
                     Precision::Confusion()});
                double parameterA = 0.0, parameterB = 0.0;
                double distanceA = 0.0, distanceB = 0.0;
                if (!nearestBoundedParameter(edge, key.first, parameterA,
                                             distanceA) ||
                    !nearestBoundedParameter(edge, key.second, parameterB,
                                             distanceB) ||
                    std::abs(parameterA - parameterB) <=
                        std::max(Precision::PConfusion(),
                                 1e-10 * std::abs(last - first))) {
                    candidateMappingOk = false;
                    break;
                }

                double first2d = 0.0, last2d = 0.0;
                const TopoDS_Face anchorFace =
                    TopoDS::Face(model.faces(faceA));
                Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                    edge, anchorFace, first2d, last2d);
                if (pcurve.IsNull() || !BRep_Tool::SameParameter(edge)) {
                    candidateMappingOk = false;
                    break;
                }
                const double parameterTolerance = std::max(
                    Precision::PConfusion(),
                    1e-10 * std::abs(last2d - first2d));
                auto exactSample = [&](double parameter,
                                       uint32_t& vertex) {
                    if (parameter < first2d - parameterTolerance ||
                        parameter > last2d + parameterTolerance)
                        return false;
                    const gp_Pnt point = curve.Value(parameter);
                    std::set<uint32_t> exact;
                    for (int localFace : {faceA, faceB}) {
                        const auto boundary = faceBoundary.find(localFace);
                        if (boundary == faceBoundary.end()) continue;
                        for (const auto& segment : boundary->second) {
                            for (uint32_t vi : {segment.first,
                                                segment.second}) {
                                const auto& v = mesh.vertices[vi];
                                if (point.Distance(gp_Pnt(v[0], v[1],
                                                         v[2])) <=
                                    tolerance)
                                    exact.insert(vi);
                            }
                        }
                    }
                    if (exact.size() > 1) return false;
                    if (exact.size() == 1) {
                        vertex = *exact.begin();
                        return true;
                    }
                    const gp_Pnt2d uvPoint = pcurve->Value(parameter);
                    if (BRep_Tool::Surface(anchorFace)
                            ->Value(uvPoint.X(), uvPoint.Y())
                            .Distance(point) > tolerance)
                        return false;
                    vertex = static_cast<uint32_t>(mesh.vertices.size());
                    mesh.vertices.push_back(
                        {point.X(), point.Y(), point.Z()});
                    mesh.anchors.push_back(
                        {faceA, uvPoint.X(), uvPoint.Y()});
                    if (haveBridgeConstraints) {
                        mesh.constraints.push_back(
                            {MeshConstraintType::BrepEdge, edgeId,
                             parameter, 0.0, 0.0});
                    }
                    return true;
                };

                uint32_t exactA = 0, exactB = 0;
                if (!exactSample(parameterA, exactA) ||
                    !exactSample(parameterB, exactB) || exactA == exactB) {
                    candidateMappingOk = false;
                    break;
                }
                const EdgeKey exactKey = edgeKey(exactA, exactB);
                SharedLocalChord proof{exactKey, edgeId, faceA, faceB,
                                       first, last,
                                       exactA == exactKey.first
                                           ? parameterA
                                           : parameterB,
                                       exactB == exactKey.second
                                           ? parameterB
                                           : parameterA,
                                       tolerance};

                // Assign the one endpoint already anchored to a local owner
                // to that owner; the other endpoint belongs to the peer by
                // elimination.  Each main proposal keeps only its endpoint
                // and replaces the foreign endpoint with the exact BRep
                // sample.  A small owner-side transition then covers the one
                // omitted open edge.  Thus f38 never inherits f37's v461 (and
                // f107 never inherits f39's mirrored junction).
                const int anchorOfFirst =
                    key.first < mesh.anchors.size()
                        ? mesh.anchors[key.first].faceId
                        : 0;
                const int anchorOfSecond =
                    key.second < mesh.anchors.size()
                        ? mesh.anchors[key.second].faceId
                        : 0;
                const bool firstIsLocal =
                    anchorOfFirst == faceA || anchorOfFirst == faceB;
                const bool secondIsLocal =
                    anchorOfSecond == faceA || anchorOfSecond == faceB;
                if (firstIsLocal == secondIsLocal) {
                    candidateMappingOk = false;
                    break;
                }
                const uint32_t ownedEndpoint =
                    firstIsLocal ? key.first : key.second;
                const int ownedFace = firstIsLocal ? anchorOfFirst
                                                   : anchorOfSecond;
                if (ownedFace != faceA && ownedFace != faceB) {
                    candidateMappingOk = false;
                    break;
                }
                const int peerFace = ownedFace == faceA ? faceB : faceA;
                auto exactFor = [&](uint32_t vi) {
                    return vi == key.first ? exactA : exactB;
                };
                auto ownerFor = [&](uint32_t vi) {
                    return vi == ownedEndpoint ? ownedFace : peerFace;
                };
                std::vector<BridgeProposal> transitions;
                int transformedProposals = 0;
                for (BridgeProposal& proposal : proposals) {
                    if (proposal.face != faceA && proposal.face != faceB)
                        continue;
                    size_t chordIndex = proposal.ring.size();
                    int chordOccurrences = 0;
                    for (size_t k = 0; k < proposal.ring.size(); ++k) {
                        if (edgeKey(proposal.ring[k], proposal.ring[
                                (k + 1) % proposal.ring.size()]) == key) {
                            chordIndex = k;
                            ++chordOccurrences;
                        }
                    }
                    if (chordOccurrences == 0) continue;
                    if (chordOccurrences != 1 ||
                        chordIndex == proposal.ring.size()) {
                        candidateMappingOk = false;
                        break;
                    }
                    const uint32_t retained =
                        proposal.face == ownedFace
                            ? ownedEndpoint
                            : (ownedEndpoint == key.first ? key.second
                                                          : key.first);
                    const uint32_t foreign =
                        retained == key.first ? key.second : key.first;
                    if (proposal.ring[chordIndex] != retained ||
                        proposal.ring[(chordIndex + 1) %
                                      proposal.ring.size()] != foreign) {
                        candidateMappingOk = false;
                        break;
                    }
                    const size_t foreignIndex =
                        (chordIndex + 1) % proposal.ring.size();
                    const uint32_t successor = proposal.ring[
                        (foreignIndex + 1) % proposal.ring.size()];
                    const uint32_t exactRetained = exactFor(retained);
                    const uint32_t exactForeign = exactFor(foreign);
                    proposal.ring[foreignIndex] = exactForeign;
                    proposal.ring.insert(
                        proposal.ring.begin() + chordIndex + 1,
                        exactRetained);
                    transitions.push_back(
                        {{foreign, successor, exactForeign},
                         ownerFor(foreign)});
                    ++transformedProposals;
                }
                if (!candidateMappingOk || transitions.size() != 2 ||
                    transformedProposals != 2)
                    break;
                proposals.insert(proposals.end(),
                                 std::make_move_iterator(transitions.begin()),
                                 std::make_move_iterator(transitions.end()));

                for (const auto& [faceId, oldDirected] : proposalUses) {
                    (void)oldDirected;
                    TopAbs_Orientation orientation = TopAbs_EXTERNAL;
                    if (!edgeOrientationOnFace(faceId, edgeId,
                                               orientation)) {
                        candidateMappingOk = false;
                        break;
                    }
                    std::pair<uint32_t, uint32_t> directed{};
                    int occurrences = 0;
                    for (const BridgeProposal& proposal : proposals) {
                        if (proposal.face != faceId) continue;
                        for (size_t k = 0; k < proposal.ring.size(); ++k) {
                            const uint32_t a = proposal.ring[k];
                            const uint32_t b = proposal.ring[
                                (k + 1) % proposal.ring.size()];
                            if (edgeKey(a, b) == exactKey) {
                                directed = {a, b};
                                ++occurrences;
                            }
                        }
                    }
                    if (occurrences != 1) {
                        candidateMappingOk = false;
                        break;
                    }
                    auto parameterFor = [&](uint32_t vi) {
                        return vi == exactKey.first ? proof.parameterA
                                                    : proof.parameterB;
                    };
                    const bool increasing =
                        parameterFor(directed.second) >
                        parameterFor(directed.first);
                    if (increasing != (orientation == TopAbs_FORWARD)) {
                        candidateMappingOk = false;
                        break;
                    }
                }
                if (!candidateMappingOk) break;
                if (std::getenv("WEFT_STITCH_DEBUG")) {
                    dbg("stitch: local-comb chord e%d v%u-v%u -> exact "
                        "v%u-v%u for f%d/f%d (params %.12g/%.12g, "
                        "offsets %.6g/%.6g)",
                        edgeId, key.first, key.second, exactA, exactB,
                        faceA, faceB, parameterA, parameterB, distanceA,
                        distanceB);
                }
                sharedLocalChords.emplace(exactKey, std::move(proof));
                ++pairedChordCount;
            }

            // A single deferred edge can sit in a small CAD-vertex star whose
            // open loop contains both of its exact owners plus one owner run
            // on either side (e240).  Replace the cross-face placeholder cap
            // with one sector per existing face run, meeting only at the two
            // exact endpoints of the deferred BRep edge.  Components where an
            // exact owner has no run remain unchanged and must pass the strict
            // one-face certification or fail closed.
            for (const SingleBridgeStar& star : singleBridgeStars) {
                if (!candidateMappingOk) break;
                const LocalCombBridgeEdge& candidate =
                    localCombBridgeEdges[star.candidateIndex];
                if (star.cycle.size() < 3 ||
                    star.cycleFaces.size() != star.cycle.size() ||
                    star.proposalIndex >= proposals.size()) {
                    candidateMappingOk = false;
                    break;
                }
                struct FaceRun {
                    int face = 0;
                    std::vector<uint32_t> vertices;
                };
                std::vector<FaceRun> runs;
                size_t start = 0;
                bool foundBreak = false;
                for (size_t k = 0; k < star.cycle.size(); ++k) {
                    if (star.cycleFaces[(k + star.cycle.size() - 1) %
                                        star.cycle.size()] !=
                        star.cycleFaces[k]) {
                        start = k;
                        foundBreak = true;
                        break;
                    }
                }
                if (!foundBreak) continue;
                for (size_t step = 0; step < star.cycle.size(); ++step) {
                    const size_t k = (start + step) % star.cycle.size();
                    const int faceId = star.cycleFaces[k];
                    if (runs.empty() || runs.back().face != faceId) {
                        runs.push_back({faceId, {star.cycle[k]}});
                    }
                    runs.back().vertices.push_back(
                        star.cycle[(k + 1) % star.cycle.size()]);
                }
                int localRun = -1, featureRun = -1;
                for (size_t ri = 0; ri < runs.size(); ++ri) {
                    if (runs[ri].face == candidate.localFace) {
                        if (localRun >= 0) {
                            localRun = -2;
                            break;
                        }
                        localRun = static_cast<int>(ri);
                    }
                    if (runs[ri].face == candidate.featureFace) {
                        if (featureRun >= 0) {
                            featureRun = -2;
                            break;
                        }
                        featureRun = static_cast<int>(ri);
                    }
                }
                if (localRun < 0 || featureRun < 0 || runs.size() < 3)
                    continue;

                const TopoDS_Edge edge =
                    TopoDS::Edge(model.edges(candidate.edgeId));
                if (BRep_Tool::Degenerated(edge) ||
                    !BRep_Tool::SameParameter(edge) ||
                    !model.edgeToFaces.Contains(edge)) {
                    candidateMappingOk = false;
                    break;
                }
                std::set<int> exactOwners;
                for (const TopoDS_Shape& owner :
                     model.edgeToFaces.FindFromKey(edge)) {
                    exactOwners.insert(model.faces.FindIndex(owner));
                }
                if (exactOwners !=
                    std::set<int>{candidate.localFace,
                                  candidate.featureFace}) {
                    candidateMappingOk = false;
                    break;
                }
                BRepAdaptor_Curve curve(edge);
                const double first = curve.FirstParameter();
                const double last = curve.LastParameter();
                const double tolerance = std::max(
                    {weldTol, BRep_Tool::Tolerance(edge),
                     BRep_Tool::Tolerance(TopoDS::Face(
                         model.faces(candidate.localFace))),
                     BRep_Tool::Tolerance(TopoDS::Face(
                         model.faces(candidate.featureFace))),
                     Precision::Confusion()});
                const TopoDS_Face anchorFace = TopoDS::Face(
                    model.faces(candidate.localFace));
                double first2d = 0.0, last2d = 0.0;
                Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                    edge, anchorFace, first2d, last2d);
                if (pcurve.IsNull()) {
                    candidateMappingOk = false;
                    break;
                }
                auto exactEndpoint = [&](double parameter,
                                         uint32_t& vertex) {
                    const gp_Pnt point = curve.Value(parameter);
                    std::set<uint32_t> exact;
                    for (int owner : {candidate.localFace,
                                      candidate.featureFace}) {
                        const auto boundary = faceBoundary.find(owner);
                        if (boundary == faceBoundary.end()) continue;
                        for (const auto& segment : boundary->second) {
                            for (uint32_t vi : {segment.first,
                                                segment.second}) {
                                const auto& v = mesh.vertices[vi];
                                if (point.Distance(gp_Pnt(v[0], v[1],
                                                         v[2])) <=
                                    tolerance)
                                    exact.insert(vi);
                            }
                        }
                    }
                    if (exact.size() > 1) return false;
                    if (exact.size() == 1) {
                        vertex = *exact.begin();
                        return true;
                    }
                    const gp_Pnt2d uvPoint = pcurve->Value(parameter);
                    if (BRep_Tool::Surface(anchorFace)
                            ->Value(uvPoint.X(), uvPoint.Y())
                            .Distance(point) > tolerance)
                        return false;
                    vertex = static_cast<uint32_t>(mesh.vertices.size());
                    mesh.vertices.push_back(
                        {point.X(), point.Y(), point.Z()});
                    mesh.anchors.push_back(
                        {candidate.localFace, uvPoint.X(), uvPoint.Y()});
                    if (haveBridgeConstraints) {
                        mesh.constraints.push_back(
                            {MeshConstraintType::BrepEdge,
                             candidate.edgeId, parameter, 0.0, 0.0});
                    }
                    return true;
                };
                uint32_t exactFirst = 0, exactLast = 0;
                if (!exactEndpoint(first, exactFirst) ||
                    !exactEndpoint(last, exactLast) ||
                    exactFirst == exactLast) {
                    candidateMappingOk = false;
                    break;
                }
                TopAbs_Orientation localOrientation = TopAbs_EXTERNAL;
                TopAbs_Orientation featureOrientation = TopAbs_EXTERNAL;
                if (!edgeOrientationOnFace(candidate.localFace,
                                           candidate.edgeId,
                                           localOrientation) ||
                    !edgeOrientationOnFace(candidate.featureFace,
                                           candidate.edgeId,
                                           featureOrientation) ||
                    localOrientation == featureOrientation) {
                    candidateMappingOk = false;
                    break;
                }
                const uint32_t qFL =
                    localOrientation == TopAbs_FORWARD ? exactFirst
                                                       : exactLast;
                const uint32_t qLF =
                    localOrientation == TopAbs_FORWARD ? exactLast
                                                       : exactFirst;
                std::vector<char> onLocalToFeature(runs.size(), 0);
                for (size_t ri = (localRun + 1) % runs.size();
                     static_cast<int>(ri) != featureRun;
                     ri = (ri + 1) % runs.size()) {
                    onLocalToFeature[ri] = 1;
                }
                std::vector<BridgeProposal> sectors;
                for (size_t ri = 0; ri < runs.size(); ++ri) {
                    std::vector<uint32_t> ring = runs[ri].vertices;
                    std::reverse(ring.begin(), ring.end());
                    if (static_cast<int>(ri) == localRun) {
                        ring.push_back(qFL);
                        ring.push_back(qLF);
                    } else if (static_cast<int>(ri) == featureRun) {
                        ring.push_back(qLF);
                        ring.push_back(qFL);
                    } else {
                        ring.push_back(onLocalToFeature[ri] ? qLF : qFL);
                    }
                    ring.erase(std::unique(ring.begin(), ring.end()),
                               ring.end());
                    if (ring.size() > 1 && ring.front() == ring.back())
                        ring.pop_back();
                    if (ring.size() < 3) {
                        candidateMappingOk = false;
                        break;
                    }
                    sectors.push_back({std::move(ring), runs[ri].face});
                }
                if (!candidateMappingOk || sectors.size() != runs.size())
                    break;
                proposals[star.proposalIndex] = std::move(sectors.front());
                for (size_t si = 1; si < sectors.size(); ++si)
                    proposals.push_back(std::move(sectors[si]));
                extraStarProposals += sectors.size() - 1;
                const EdgeKey exactKey = edgeKey(exactFirst, exactLast);
                sharedLocalChords.emplace(
                    exactKey,
                    SharedLocalChord{
                        exactKey, candidate.edgeId, candidate.localFace,
                        candidate.featureFace, first, last,
                        exactFirst == exactKey.first ? first : last,
                        exactLast == exactKey.second ? last : first,
                        tolerance});
                if (std::getenv("WEFT_STITCH_DEBUG")) {
                    dbg("stitch: local-comb star e%d repartitioned into "
                        "%zu owner sectors at exact v%u-v%u",
                        candidate.edgeId, sectors.size(), exactFirst,
                        exactLast);
                }
            }
        }

        // Re-run the complete edge-use simulation after replacing each
        // synthetic chord by old-junction -> exact-edge -> old-junction.
        if (candidateMappingOk) {
            coveredOpen.clear();
            newEdges.clear();
            for (const BridgeProposal& proposal : proposals) {
                const std::set<uint32_t> unique(proposal.ring.begin(),
                                                proposal.ring.end());
                if (proposal.ring.size() < 3 ||
                    unique.size() != proposal.ring.size()) {
                    candidateMappingOk = false;
                    break;
                }
                for (size_t k = 0; k < proposal.ring.size(); ++k) {
                    const uint32_t a = proposal.ring[k];
                    const uint32_t b = proposal.ring[
                        (k + 1) % proposal.ring.size()];
                    const EdgeKey key = edgeKey(a, b);
                    const int uses = edgeUses[key];
                    if (uses == 1) {
                        const auto& directions = edgeDirections[key];
                        if (directions.size() != 1 ||
                            directions.front() != std::make_pair(b, a)) {
                            candidateMappingOk = false;
                            break;
                        }
                        ++coveredOpen[key];
                    } else if (uses == 0) {
                        newEdges[key].push_back({a, b});
                    } else {
                        candidateMappingOk = false;
                        break;
                    }
                }
                if (!candidateMappingOk) break;
            }
        }
        if (candidateMappingOk) {
            for (int ci : selectedComponents) {
                for (size_t oi : components[ci].edges) {
                    if (coveredOpen[edgeKey(open[oi].a, open[oi].b)] != 1) {
                        candidateMappingOk = false;
                        break;
                    }
                }
                if (!candidateMappingOk) break;
            }
        }
        if (candidateMappingOk) {
            for (const auto& [key, directions] : newEdges) {
                if (directions.size() != 2 ||
                    directions[0] !=
                        std::make_pair(directions[1].second,
                                       directions[1].first)) {
                    candidateMappingOk = false;
                    break;
                }
            }
        }
        if (std::getenv("WEFT_STITCH_DEBUG")) {
            dbg("stitch: local-comb bridge topology %s (%zu candidates, "
                "%zu proposals, %zu components, %zu new chords)",
                candidateMappingOk ? "accepted" : "rejected",
                localCombBridgeEdges.size(), proposals.size(),
                selectedComponents.size(), newEdges.size());
            for (const BridgeProposal& proposal : proposals) {
                dbg("stitch: local-comb bridge proposal f%d has %zu corners",
                    proposal.face, proposal.ring.size());
            }
        }

        auto certifyBridge = [&](const BridgeProposal& proposal) {
            auto reject = [&](const char* why) {
                if (std::getenv("WEFT_STITCH_DEBUG")) {
                    dbg("stitch: local-comb bridge f%d rejected: %s",
                        proposal.face, why);
                }
                return false;
            };
            if (proposal.face < 1 || proposal.face > model.faceCount())
                return reject("invalid face");
            const TopoDS_Face face = TopoDS::Face(model.faces(proposal.face));
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface.IsNull()) return reject("missing surface");
            BRepAdaptor_Surface sa(face);
            if (sa.IsUPeriodic() || sa.IsVPeriodic())
                return reject("periodic local face");
            const double uvSpan = std::max(
                {1.0, std::abs(sa.LastUParameter() - sa.FirstUParameter()),
                 std::abs(sa.LastVParameter() - sa.FirstVParameter())});
            const double uvTol = 1e-10 * uvSpan;
            const double uvAreaTol = uvTol * uvSpan;
            const double faceTol = BRep_Tool::Tolerance(face);
            std::vector<gp_Pnt2d> uv;
            uv.reserve(proposal.ring.size());
            auto provesExistingOwnBoundary = [&](uint32_t vi, double u,
                                                  double v) {
                if (vi >= mesh.anchors.size() ||
                    mesh.anchors[vi].faceId != proposal.face)
                    return false;
                bool usedByFace = false;
                for (size_t pi = 0; pi < mesh.polygons.size(); ++pi) {
                    if (mesh.polygonFaceId[pi] != proposal.face) continue;
                    if (std::find(mesh.polygons[pi].begin(),
                                  mesh.polygons[pi].end(), vi) !=
                        mesh.polygons[pi].end()) {
                        usedByFace = true;
                        break;
                    }
                }
                if (!usedByFace) return false;
                const bool onOpenFaceBoundary = std::any_of(
                    open.begin(), open.end(), [&](const OpenEdge& edgeUse) {
                        return edgeUse.face == proposal.face &&
                               (edgeUse.a == vi || edgeUse.b == vi);
                    });
                if (!onOpenFaceBoundary) return false;
                const auto& p = mesh.vertices[vi];
                const double surfaceTolerance = std::max(
                    {weldTol, faceTol, Precision::Confusion()});
                return surface->Value(u, v).Distance(
                           gp_Pnt(p[0], p[1], p[2])) <= surfaceTolerance;
            };
            std::map<uint32_t, gp_Pnt2d> sharedBoundaryUv;
            for (size_t k = 0; k < proposal.ring.size(); ++k) {
                const uint32_t a = proposal.ring[k];
                const uint32_t b =
                    proposal.ring[(k + 1) % proposal.ring.size()];
                const auto proofIt = sharedLocalChords.find(edgeKey(a, b));
                if (proofIt == sharedLocalChords.end()) continue;
                const SharedLocalChord& proof = proofIt->second;
                if ((proposal.face != proof.faceA &&
                     proposal.face != proof.faceB) ||
                    !BRep_Tool::SameParameter(
                        TopoDS::Edge(model.edges(proof.edgeId))))
                    return reject("invalid shared-edge chord owner");
                const TopoDS_Edge edge =
                    TopoDS::Edge(model.edges(proof.edgeId));
                double first2d = 0.0, last2d = 0.0;
                Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                    edge, face, first2d, last2d);
                if (pcurve.IsNull())
                    return reject("missing shared-edge pcurve");
                const double parameterTolerance = std::max(
                    Precision::PConfusion(),
                    1e-10 * std::abs(last2d - first2d));
                auto addSharedBoundaryUv = [&](uint32_t vi,
                                               double parameter) {
                    if (parameter < first2d - parameterTolerance ||
                        parameter > last2d + parameterTolerance)
                        return false;
                    const gp_Pnt2d point = pcurve->Value(parameter);
                    const auto& v = mesh.vertices[vi];
                    if (surface->Value(point.X(), point.Y())
                            .Distance(gp_Pnt(v[0], v[1], v[2])) >
                        proof.tolerance)
                        return false;
                    const auto [it, inserted] =
                        sharedBoundaryUv.emplace(vi, point);
                    return inserted ||
                           it->second.SquareDistance(point) <=
                               uvTol * uvTol;
                };
                const double parameterForA =
                    a == proof.key.first ? proof.parameterA
                                         : proof.parameterB;
                const double parameterForB =
                    b == proof.key.first ? proof.parameterA
                                         : proof.parameterB;
                if (!addSharedBoundaryUv(a, parameterForA) ||
                    !addSharedBoundaryUv(b, parameterForB))
                    return reject("shared-edge pcurve mismatch");
            }
            for (uint32_t vi : proposal.ring) {
                if (vi >= mesh.vertices.size()) return reject("invalid vertex");
                double u = 0.0, vv = 0.0;
                const auto sharedIt = sharedBoundaryUv.find(vi);
                const bool haveSharedBoundaryUv =
                    sharedIt != sharedBoundaryUv.end();
                bool haveLocalCornerUv = haveSharedBoundaryUv;
                if (haveSharedBoundaryUv) {
                    u = sharedIt->second.X();
                    vv = sharedIt->second.Y();
                }
                if (!haveLocalCornerUv && vi < mesh.anchors.size() &&
                    mesh.anchors[vi].faceId == proposal.face) {
                    u = mesh.anchors[vi].u;
                    vv = mesh.anchors[vi].v;
                    haveLocalCornerUv = true;
                }
                if (!haveLocalCornerUv) {
                    const auto& v = mesh.vertices[vi];
                    GeomAPI_ProjectPointOnSurf project(
                        gp_Pnt(v[0], v[1], v[2]), surface);
                    if (!project.IsDone() || project.NbPoints() < 1 ||
                        project.LowerDistance() > tolCap) {
                        if (std::getenv("WEFT_STITCH_DEBUG")) {
                            dbg("stitch: local-comb bridge f%d v%u "
                                "projection failed (anchor f%d, distance "
                                "%.6g, cap %.6g)",
                                proposal.face, vi,
                                vi < mesh.anchors.size()
                                    ? mesh.anchors[vi].faceId
                                    : 0,
                                project.IsDone() && project.NbPoints() > 0
                                    ? project.LowerDistance()
                                    : -1.0,
                                tolCap);
                        }
                        return reject("projection outside bridge band");
                    }
                    project.LowerDistanceParameters(u, vv);
                }
                BRepClass_FaceClassifier owner(
                    const_cast<TopoDS_Face&>(face), gp_Pnt2d(u, vv), faceTol);
                const bool haveOwnBoundaryProof =
                    !haveSharedBoundaryUv &&
                    provesExistingOwnBoundary(vi, u, vv);
                if (owner.State() == TopAbs_OUT && !haveSharedBoundaryUv &&
                    !haveOwnBoundaryProof) {
                    if (std::getenv("WEFT_STITCH_DEBUG")) {
                        dbg("stitch: local-comb bridge f%d v%u UV "
                            "(%.12g,%.12g) classified OUT (anchor f%d, "
                            "local-corner=%d)",
                            proposal.face, vi, u, vv,
                            vi < mesh.anchors.size()
                                ? mesh.anchors[vi].faceId
                                : 0,
                            haveLocalCornerUv ? 1 : 0);
                    }
                    return reject("corner projects outside local face");
                }
                if (owner.State() == TopAbs_OUT && haveSharedBoundaryUv &&
                    std::getenv("WEFT_STITCH_DEBUG")) {
                    dbg("stitch: local-comb bridge f%d v%u accepted by "
                        "shared BRep edge despite OUT pcurve classifier",
                        proposal.face, vi);
                }
                if (owner.State() == TopAbs_OUT && haveOwnBoundaryProof &&
                    std::getenv("WEFT_STITCH_DEBUG")) {
                    dbg("stitch: local-comb bridge f%d v%u accepted by "
                        "existing own-face open-boundary proof despite OUT "
                        "classifier",
                        proposal.face, vi);
                }
                uv.emplace_back(u, vv);
            }
            auto cross = [](const gp_Pnt2d& a, const gp_Pnt2d& b,
                            const gp_Pnt2d& c) {
                return (b.X() - a.X()) * (c.Y() - a.Y()) -
                       (b.Y() - a.Y()) * (c.X() - a.X());
            };
            auto onSegment = [&](const gp_Pnt2d& p, const gp_Pnt2d& a,
                                 const gp_Pnt2d& b) {
                const double len = std::sqrt(a.SquareDistance(b));
                if (len <= uvTol)
                    return p.SquareDistance(a) <= uvTol * uvTol;
                if (std::abs(cross(a, b, p)) > uvTol * len) return false;
                return (p.X() - a.X()) * (p.X() - b.X()) +
                           (p.Y() - a.Y()) * (p.Y() - b.Y()) <=
                       uvTol * uvTol;
            };
            for (size_t i = 0; i < uv.size(); ++i) {
                const size_t i2 = (i + 1) % uv.size();
                if (uv[i].SquareDistance(uv[i2]) <= uvTol * uvTol)
                    return reject("collapsed UV edge");
                for (size_t j = i + 1; j < uv.size(); ++j) {
                    const size_t j2 = (j + 1) % uv.size();
                    if (i2 == j || j2 == i) continue;
                    const double abC = cross(uv[i], uv[i2], uv[j]);
                    const double abD = cross(uv[i], uv[i2], uv[j2]);
                    const double cdA = cross(uv[j], uv[j2], uv[i]);
                    const double cdB = cross(uv[j], uv[j2], uv[i2]);
                    if ((abC * abD < 0.0 && cdA * cdB < 0.0) ||
                        onSegment(uv[i], uv[j], uv[j2]) ||
                        onSegment(uv[i2], uv[j], uv[j2]) ||
                        onSegment(uv[j], uv[i], uv[i2]) ||
                        onSegment(uv[j2], uv[i], uv[i2]))
                        return reject("non-simple UV ring");
                }
            }
            double twiceArea = 0.0, cu = 0.0, cv = 0.0;
            for (size_t k = 0; k < uv.size(); ++k) {
                const gp_Pnt2d& a = uv[k];
                const gp_Pnt2d& b = uv[(k + 1) % uv.size()];
                const double cr = a.X() * b.Y() - b.X() * a.Y();
                twiceArea += cr;
                cu += (a.X() + b.X()) * cr;
                cv += (a.Y() + b.Y()) * cr;
            }
            if (std::abs(twiceArea) <= 2.0 * uvAreaTol)
                return reject("zero UV area");
            cu /= 3.0 * twiceArea;
            cv /= 3.0 * twiceArea;
            BRepClass_FaceClassifier centerOwner(
                const_cast<TopoDS_Face&>(face), gp_Pnt2d(cu, cv), faceTol);
            if (centerOwner.State() == TopAbs_OUT)
                return reject("centroid outside local face");

            gp_XYZ nw(0, 0, 0);
            for (size_t k = 0; k < proposal.ring.size(); ++k) {
                const auto& a = mesh.vertices[proposal.ring[k]];
                const auto& b = mesh.vertices[
                    proposal.ring[(k + 1) % proposal.ring.size()]];
                nw += gp_XYZ(a[1] * b[2] - a[2] * b[1],
                             a[2] * b[0] - a[0] * b[2],
                             a[0] * b[1] - a[1] * b[0]);
            }
            gp_Pnt p; gp_Vec du, dv;
            sa.D1(cu, cv, p, du, dv);
            gp_Vec cadN = du.Crossed(dv);
            if (face.Orientation() == TopAbs_REVERSED) cadN.Reverse();
            if (gp_Vec(nw).Magnitude() <= 1e-16 ||
                cadN.Magnitude() <= 1e-16 || gp_Vec(nw).Dot(cadN) <= 0.0)
                return reject("CAD-normal disagreement");

            const auto triangles = triangulatePoly(mesh.vertices,
                                                   proposal.ring);
            if (triangles.size() != proposal.ring.size() - 2)
                return reject("uncertifiable triangulation");
            for (const auto& tri : triangles) {
                const gp_Pnt2d probe(
                    (uv[tri[0]].X() + uv[tri[1]].X() + uv[tri[2]].X()) / 3.0,
                    (uv[tri[0]].Y() + uv[tri[1]].Y() + uv[tri[2]].Y()) / 3.0);
                BRepClass_FaceClassifier triOwner(
                    const_cast<TopoDS_Face&>(face), probe, faceTol);
                if (triOwner.State() == TopAbs_OUT)
                    return reject("triangle probe outside local face");
                const auto& a = mesh.vertices[proposal.ring[tri[0]]];
                const auto& b = mesh.vertices[proposal.ring[tri[1]]];
                const auto& c = mesh.vertices[proposal.ring[tri[2]]];
                const gp_Vec ab(gp_Pnt(a[0], a[1], a[2]),
                                gp_Pnt(b[0], b[1], b[2]));
                const gp_Vec ac(gp_Pnt(a[0], a[1], a[2]),
                                gp_Pnt(c[0], c[1], c[2]));
                gp_Pnt tp; gp_Vec tdu, tdv;
                sa.D1(probe.X(), probe.Y(), tp, tdu, tdv);
                gp_Vec tCad = tdu.Crossed(tdv);
                if (face.Orientation() == TopAbs_REVERSED) tCad.Reverse();
                if (ab.Crossed(ac).Magnitude() <= 1e-16 ||
                    tCad.Magnitude() <= 1e-16 ||
                    ab.Crossed(ac).Dot(tCad) <= 0.0)
                    return reject("triangle CAD-normal disagreement");
            }
            return true;
        };

        if (candidateMappingOk) {
            for (const BridgeProposal& proposal : proposals) {
                if (!certifyBridge(proposal)) {
                    if (std::getenv("WEFT_STITCH_DEBUG")) {
                        dbg("stitch: local-comb bridge geometry rejected f%d "
                            "(%zu corners)", proposal.face,
                            proposal.ring.size());
                    }
                    candidateMappingOk = false;
                    break;
                }
            }
        }
        const size_t expectedBridgeProposalCount =
            localCombBridgeEdges.size() + 2 * pairedChordCount +
            extraStarProposals;
        if (candidateMappingOk &&
            proposals.size() == expectedBridgeProposalCount) {
            for (BridgeProposal& proposal : proposals) {
                mesh.polygons.push_back(std::move(proposal.ring));
                mesh.polygonFaceId.push_back(proposal.face);
                ++bridgePolygons;
            }
            mesh.polygonCornerAnchors.clear();
            refreshCertifiedTriangulations(mesh);
        } else {
            mesh.vertices.resize(bridgeVertexBase);
            mesh.anchors.resize(bridgeAnchorBase);
            mesh.constraints.resize(bridgeConstraintBase);
            if (std::getenv("WEFT_STITCH_DEBUG")) {
                dbg("stitch: local-comb endpoint bridge batch rejected");
            }
        }
    }
    if (spliced) dbg("stitch: %d seam vertex insertion(s)", spliced);
    if (bridgePolygons) {
        dbg("stitch: %d certified local-comb endpoint bridge polygon(s)",
            bridgePolygons);
    }
}

void unionSeams(PolyMesh& mesh, const Model& model, double weldTol) {
    (void)model;
    int total = 0;
    for (int pass = 0; pass < 8; ++pass) {
        std::map<std::pair<uint32_t, uint32_t>, size_t> polyOf;
        std::map<std::pair<uint32_t, uint32_t>, int> count;
        std::multimap<uint32_t, uint32_t> outOf;  // v -> w for edge (v,w)
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                auto key = std::make_pair(poly[i],
                                          poly[(i + 1) % poly.size()]);
                polyOf[key] = p;
                ++count[key];
                outOf.emplace(key.first, key.second);
            }
        }
        int spliced = 0;
        for (const auto& [e, c] : count) {
            if (c != 1 || count.count({e.second, e.first})) continue;
            const auto [u, v] = e;
            const auto& U = mesh.vertices[u];
            const auto& V = mesh.vertices[v];
            double ex = V[0] - U[0], ey = V[1] - U[1], ez = V[2] - U[2];
            double ee = ex * ex + ey * ey + ez * ez;
            if (ee < 1e-30) continue;
            // Curvature-tolerant on-segment test: the complement path of
            // an open edge on a CURVED border (cylinder rim, fillet rail)
            // lies on the arc, not the chord — its sagitta reaches 21% of
            // the chord at a 90-degree span, so an 8% chord slack silently
            // dropped every curved seam. Allow 25% perpendicular drift
            // plus an ellipse detour bound (|uw|+|wv| vs |uv|); the walk's
            // monotone parameter and the exact topological closure at u
            // remain the real gatekeepers.
            const double chord = std::sqrt(ee);
            const double slack = std::max(weldTol * 2.0, 0.25 * chord);
            auto onSegment = [&](uint32_t w, double tMax, double& tOut) {
                const auto& W = mesh.vertices[w];
                double px = W[0] - U[0], py = W[1] - U[1], pz = W[2] - U[2];
                double t = (px * ex + py * ey + pz * ez) / ee;
                if (t < -0.01 || t > tMax + 1e-9) return false;
                double dx = px - t * ex, dy = py - t * ey, dz = pz - t * ez;
                if (dx * dx + dy * dy + dz * dz > slack * slack) {
                    return false;
                }
                const double dU = std::sqrt(px * px + py * py + pz * pz);
                const double dV = std::sqrt(
                    (W[0] - V[0]) * (W[0] - V[0]) +
                    (W[1] - V[1]) * (W[1] - V[1]) +
                    (W[2] - V[2]) * (W[2] - V[2]));
                if (dU + dV > 1.35 * chord + 2.0 * weldTol) return false;
                tOut = t;
                return true;
            };
            // Complement path v -> w1 -> ... -> wk -> u on the denser
            // neighbour (multi-vertex gaps, handoff step 1): walk open
            // edges from v, each step landing ON the u-v segment with
            // strictly decreasing t, until an edge into u exists.
            std::vector<uint32_t> path;
            uint32_t cur = v;
            double tCur = 1.0;
            bool closed = false;
            for (int step = 0; step < 24 && !closed; ++step) {
                uint32_t nxt = UINT32_MAX;
                double tNxt = 0;
                bool ambiguous = false;
                for (auto it = outOf.lower_bound(cur);
                     it != outOf.end() && it->first == cur; ++it) {
                    const uint32_t w = it->second;
                    if (w == u && !path.empty()) {
                        if (count.count({w, u})) {}
                        // direct closure candidate handled below
                    }
                    if (w == u) {
                        if (!path.empty()) { nxt = u; tNxt = 0; }
                        continue;
                    }
                    double t;
                    if (!onSegment(w, tCur - 1e-9, t)) continue;
                    if (nxt != UINT32_MAX && nxt != u) {
                        ambiguous = true;  // two candidates: bail, safety
                        break;
                    }
                    if (nxt == UINT32_MAX || nxt == u) { nxt = w; tNxt = t; }
                }
                if (ambiguous || nxt == UINT32_MAX) break;
                if (nxt == u) { closed = true; break; }
                path.push_back(nxt);
                cur = nxt;
                tCur = tNxt;
                if (count.count({cur, u})) { closed = true; break; }
            }
            if (!closed || path.empty()) continue;
            // Splices add (u, p_k), reversed interiors, and (p_1, v):
            // none may already exist or the splice would open a
            // non-manifold edge instead of closing a seam.
            bool clash = count.count({u, path.back()}) ||
                         count.count({path.front(), v});
            for (size_t i = 0; i + 1 < path.size() && !clash; ++i) {
                clash = count.count({path[i + 1], path[i]}) > 0;
            }
            if (clash) continue;
            auto pit = polyOf.find(e);
            if (pit == polyOf.end()) continue;
            auto& poly = mesh.polygons[pit->second];
            for (size_t i = 0; i < poly.size(); ++i) {
                if (poly[i] == u && poly[(i + 1) % poly.size()] == v) {
                    // Ring runs u -> v; the complement ran v -> ... -> u,
                    // so insert the path REVERSED between them.
                    std::vector<uint32_t> rev(path.rbegin(), path.rend());
                    poly.insert(poly.begin() + i + 1, rev.begin(),
                                rev.end());
                    --count[{u, v}];
                    ++count[{u, rev.front()}];
                    for (size_t k = 0; k + 1 < rev.size(); ++k) {
                        ++count[{rev[k], rev[k + 1]}];
                    }
                    ++count[{rev.back(), v}];
                    spliced += int(rev.size());
                    break;
                }
            }
        }
        total += spliced;
        if (!spliced) break;
    }
    if (total) dbg("seam union: absorbed %d vert(s) into n-gons", total);
}

// The solved subdivision total around a face's OUTER loop: the exact number
// of boundary segments the boundary-driven meshers (quad-fill, plate-web,
// minimal planar) lay down, since samplePlanarRings samples each outer wire
// edge at solvedEdge[eid] segments (falling back to an even share of the
// radial default when an edge took no density share). This is the honest
// "primary count" for those meshers — they carry no interior grid count, only
// a boundary total — and mirrors samplePlanarRings' own outer-ring sampling
// so displayed == built. Report-only; drives no geometry.
int outerWireSolvedTotal(const TopoDS_Face& face, const Model& model,
                         const std::vector<int>& solvedEdge,
                         int radialDefault) {
    int total = 0;
    try {
        const TopoDS_Wire w = BRepTools::OuterWire(face);
        // Iterate the wire's edges directly (TopoDS_Iterator), not via
        // BRepTools_WireExplorer, which SILENTLY DROPS the edges of a sloppy
        // wire — samplePlanarRings falls back to exactly this raw-edge walk
        // when it detects the drop, so mirroring it keeps the total in step
        // with the boundary the mesh actually laid down. The per-edge fallback
        // share matches samplePlanarRings' sloppy-path divisor (raw count).
        int rawEdges = 0;
        for (TopoDS_Iterator it(w); it.More(); it.Next()) {
            if (it.Value().ShapeType() == TopAbs_EDGE &&
                !BRep_Tool::Degenerated(TopoDS::Edge(it.Value()))) {
                ++rawEdges;
            }
        }
        for (TopoDS_Iterator it(w); it.More(); it.Next()) {
            if (it.Value().ShapeType() != TopAbs_EDGE) continue;
            const TopoDS_Edge e = TopoDS::Edge(it.Value());
            if (BRep_Tool::Degenerated(e)) continue;
            const int eid = model.edges.FindIndex(e);
            int n = (eid >= 1 && eid < int(solvedEdge.size())) ? solvedEdge[eid]
                                                               : 0;
            if (n < 1) {
                n = std::max(1, std::max(3, radialDefault) /
                                    std::max(1, rawEdges));
            }
            total += n;
        }
    } catch (const Standard_Failure&) {
    }
    return total;
}


}  // namespace weft::mesher_impl
