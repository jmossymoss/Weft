#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// A freeform split rectangle has two segmented side rails. STEP pcurves for
// one logical cross-row can differ slightly on the opposing rails even though
// the B-rep junction is intentional. Pair only LEFT-vs-RIGHT rail levels;
// never merge two close levels on the same rail. This prevents pcurve noise
// from becoming a global sliver row without erasing a genuine narrow feature.
bool appendPairedSplitRows(const TopoDS_Face& face, const Model& model,
                           const FacePlan& plan, double vSpan,
                           std::vector<double>& rows) {
    std::array<std::vector<double>, 2> rail;
    BRepAdaptor_Surface surf(face);
    const double midU =
        0.5 * (surf.FirstUParameter() + surf.LastUParameter());
    for (int eid : plan.vEdges) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        double first = 0.0, last = 0.0;
        Handle(Geom2d_Curve) pc =
            BRep_Tool::CurveOnSurface(edge, face, first, last);
        if (pc.IsNull()) return false;
        const gp_Pnt2d a = pc->Value(first);
        const gp_Pnt2d b = pc->Value(last);
        const int side = 0.5 * (a.X() + b.X()) < midU ? 0 : 1;
        rail[side].push_back(a.Y());
        rail[side].push_back(b.Y());
    }
    const double logicalTol = std::max(1e-12, 2e-5 * vSpan);
    auto compact = [&](std::vector<double>& values) {
        std::sort(values.begin(), values.end());
        std::vector<double> out;
        for (double value : values) {
            if (out.empty() || std::abs(value - out.back()) > logicalTol) {
                out.push_back(value);
            }
        }
        values.swap(out);
    };
    compact(rail[0]);
    compact(rail[1]);
    if (rail[0].empty() || rail[1].empty()) return false;

    const double opposingTol = std::max(logicalTol, 3e-5 * vSpan);
    size_t left = 0, right = 0;
    while (left < rail[0].size() || right < rail[1].size()) {
        if (left == rail[0].size()) {
            rows.push_back(rail[1][right++]);
            continue;
        }
        if (right == rail[1].size()) {
            rows.push_back(rail[0][left++]);
            continue;
        }
        const double a = rail[0][left];
        const double b = rail[1][right];
        if (std::abs(a - b) <= opposingTol) {
            rows.push_back(0.5 * (a + b));
            ++left;
            ++right;
        } else if (a < b) {
            rows.push_back(a);
            ++left;
        } else {
            rows.push_back(b);
            ++right;
        }
    }
    return true;
}

void pinOrthogonalTrimGrids(const Model& model,
                            const std::map<int, FacePlan>& plans,
                            const GenerationSettings& settings,
                            const std::vector<int>& solvedEdge,
                            PinnedEdges& pins) {
    auto uniqueStations = [](std::vector<double>& a, double tol) {
        std::sort(a.begin(), a.end());
        a.erase(std::unique(a.begin(), a.end(), [&](double x, double y) {
                    return std::abs(x - y) <= tol;
                }), a.end());
    };
    for (const auto& [fid, plan] : plans) {
        if (!plan.orthogonalTrimGrid) continue;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        BRepAdaptor_Surface surf(face);
        const FaceMeshSettings& s = settings.forFace(fid);
        const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
        const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
        const double ut = 1e-9 * std::max(1.0, std::abs(u1 - u0));
        const double vt = 1e-9 * std::max(1.0, std::abs(v1 - v0));
        const int nu = plan.kind == MesherKind::RevolutionGrid
            ? std::max(1, plan.orthogonalDriverU > 0
                              ? solvedEdge[plan.orthogonalDriverU]
                              : int(std::lround(std::max(3, s.radial) *
                                                plan.bandWrapFrac)))
            : std::max(1, plan.orthogonalDriverU > 0
                              ? solvedEdge[plan.orthogonalDriverU] : s.gridU);
        const int nv = std::max(1, plan.orthogonalDriverV > 0
                                      ? solvedEdge[plan.orthogonalDriverV]
                                      : (plan.kind == MesherKind::RevolutionGrid
                                             ? s.axial : s.gridV));
        std::vector<double> U, V;
        for (int i = 0; i <= nu; ++i) U.push_back(u0 + (u1-u0)*i/nu);
        for (int j = 0; j <= nv; ++j) V.push_back(v0 + (v1-v0)*j/nv);
        auto collectEndpoints = [&](int eid, bool collectU,
                                    bool collectV) {
            const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
            double f, l;
            Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e, face, f, l);
            if (pc.IsNull()) return;
            const gp_Pnt2d a = pc->Value(f), b = pc->Value(l);
            if (collectU) {
                U.push_back(a.X()); U.push_back(b.X());
            }
            if (collectV) {
                V.push_back(a.Y()); V.push_back(b.Y());
            }
        };
        // Generic split rectangles use one Cartesian station table, hence
        // every boundary edge must be pinned where every trim endpoint's
        // row/column crosses it.  A comb is different: a groove cap owns its
        // station only inside that groove corridor.  Pinning the cap's V to
        // every distant rail/side is precisely what created full transverse
        // rings across the cylinder (and dense cross-lattices on freeform
        // grip combs).  Their builders retain the uniform primitive/base
        // stations here and consume each feature's own solved-edge samples
        // locally.
        if (!plan.orthogonalLocalComb && !plan.orthogonalDrumComb) {
            if (plan.kind == MesherKind::CoonsGrid) {
                // A non-comb freeform route is admitted only for a split
                // rectangle: exactly two horizontal rims and no interior U
                // step. Its segmented side rails contribute logical V rows,
                // while their small p-curve U excursions are boundary shape,
                // not model-wide columns. Promoting both endpoint axes made
                // micron-wide sliver columns on every opposing rail.
                if (!appendPairedSplitRows(face, model, plan,
                                           std::abs(v1 - v0), V)) {
                    for (int e : plan.vEdges) {
                        collectEndpoints(e, false, true);
                    }
                }
            } else {
                // Analytic drums may contain real orthogonal inset steps, so
                // retain their full Cartesian endpoint station table.
                for (int e : plan.orthogonalEdges) {
                    collectEndpoints(e, true, true);
                }
            }
        }
        uniqueStations(U, std::max(ut, 1e-4*std::abs(u1-u0)));
        const double vStationFraction =
            plan.kind == MesherKind::CoonsGrid ? 2e-5 : 1e-6;
        uniqueStations(V, std::max(vt,
                                  vStationFraction*std::abs(v1-v0)));

        auto pinEdge = [&](int eid, bool alongU) {
            const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
            double f, l;
            Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e, face, f, l);
            if (pc.IsNull()) return;
            auto coord = [&](double t, bool alongU) {
                const gp_Pnt2d p = pc->Value(f + (l-f)*t);
                return alongU ? p.X() : p.Y();
            };
            std::vector<double> fr{0.0, 1.0};
            if (!pins[eid].empty()) {
                fr.insert(fr.end(), pins[eid].begin(), pins[eid].end());
            }
            for (bool axis : {alongU}) {
                const auto& stations = axis ? U : V;
                const double a = coord(0, axis), b = coord(1, axis);
                const double lo = std::min(a,b), hi = std::max(a,b);
                const double tol = axis ? ut : vt;
                if (hi - lo <= tol) continue;
                for (double x : stations) {
                    if (x <= lo + tol || x >= hi - tol) continue;
                    double L = 0.0, R = 1.0;
                    for (int it = 0; it < 52; ++it) {
                        const double m = 0.5 * (L + R);
                        if ((coord(m, axis) < x) == (a < b)) L = m;
                        else R = m;
                    }
                    fr.push_back(0.5 * (L + R));
                }
            }
            std::sort(fr.begin(), fr.end());
            fr.erase(std::unique(fr.begin(), fr.end(), [](double x, double y) {
                         return std::abs(x-y) < 1e-10;
                     }), fr.end());
            pins[eid] = std::move(fr);
        };
        for (int e : plan.uEdges) pinEdge(e, true);
        for (int e : plan.vEdges) pinEdge(e, false);
    }
}
// Freeform comb trims (the long, alternating grip panels on imported CAD
// parts) are not a Cartesian grid.  Carrying every tooth endpoint through
// every V band turns one local trim event into a model-wide row/column
// product and creates the dense side fans that motivated this path.  Build
// each connected V slab against one shared absolute longitudinal phase:
// exact samples on the two local rails remain exact, while the structural
// columns keep the same U stations from the upper flare through every rib
// interval into the lower body.  Exact trim samples become corners of the
// local boundary n-gon instead of spawning short columns or triangle fans.
bool meshOrthogonalLocalComb(const TopoDS_Face& face,
                             const BRepAdaptor_Surface& surf,
                             const Model& model, const FacePlan& plan,
                             const std::vector<int>& solvedEdge, int faceId,
                             int nu, int nv, int cellCap, MeshBuilder& out,
                             const PinnedEdges* pins) {
    nu = std::max(1, nu);
    nv = std::max(1, nv);
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double us = std::max(1e-12, std::abs(u1 - u0));
    const double vs = std::max(1e-12, std::abs(v1 - v0));
    const double ut = 1e-9 * std::max(1.0, us);
    const double vt = 1e-9 * std::max(1.0, vs);
    const double stationTolU = std::max(ut, 1e-6 * us);
    const double stationTolV = std::max(vt, 1e-6 * vs);
    const double qU = std::max(1e-12, ut);
    const double qV = std::max(1e-12, vt);
    auto uvKey = [&](const gp_Pnt2d& p) {
        return std::make_pair(llround(p.X() / qU), llround(p.Y() / qV));
    };

    struct ExactSample {
        gp_Pnt2d uv;
        gp_Pnt2d sourceUv;
        gp_Pnt p;
        bool horizontal = false;
        double railV = 0.0;
    };
    std::vector<gp_Pnt2d> boundary;
    std::vector<bool> boundaryHorizontal;
    std::vector<ExactSample> samples;
    std::map<std::pair<long long, long long>, gp_Pnt> exactBoundary;
    std::vector<double> levels{v0, v1};
    bool havePreviousEdge = false;
    bool previousHorizontal = false;
    double previousRailV = 0.0;
    bool firstEdgeHorizontal = false;
    double lastRailV = 0.0;

    // Read the one trim wire in wire order.  Only genuinely longitudinal
    // samples become local rail stations; short tooth ends contribute V
    // levels instead, so they never acquire the count of a full tooth.
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face);
             we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            if (BRep_Tool::Degenerated(edge)) continue;
            const int eid = model.edges.FindIndex(edge);
            if (eid < 1) return false;
            double f, l, f3, l3;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f, l);
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
            if (pc.IsNull() || c3.IsNull()) return false;

            const int n = std::max(1, solvedEdge[eid]);
            const bool rev = edge.Orientation() == TopAbs_REVERSED;
            const std::vector<double> fractions = edgeSampleFractions(
                eid, n, 0.0, rev, false, pins, &model);
            if (fractions.empty()) return false;

            std::vector<std::pair<gp_Pnt2d, gp_Pnt>> edgeSamples;
            edgeSamples.reserve(fractions.size() + 1);
            double eu0 = 1e300, eu1 = -1e300;
            double ev0 = 1e300, ev1 = -1e300;
            for (double t : fractions) {
                const gp_Pnt2d uv = pc->Value(f + (l - f) * t);
                const gp_Pnt p = c3->Value(f3 + (l3 - f3) * t);
                edgeSamples.push_back({uv, p});
                eu0 = std::min(eu0, uv.X());
                eu1 = std::max(eu1, uv.X());
                ev0 = std::min(ev0, uv.Y());
                ev1 = std::max(ev1, uv.Y());
            }
            // Include the far endpoint for span classification and mandatory
            // V levels; the next wire edge still owns it in `boundary`.
            const gp_Pnt2d uvFirst = pc->Value(f);
            const gp_Pnt2d uvLast = pc->Value(l);
            eu0 = std::min({eu0, uvFirst.X(), uvLast.X()});
            eu1 = std::max({eu1, uvFirst.X(), uvLast.X()});
            ev0 = std::min({ev0, uvFirst.Y(), uvLast.Y()});
            ev1 = std::max({ev1, uvFirst.Y(), uvLast.Y()});
            const double du = (eu1 - eu0) / us;
            const double dv = (ev1 - ev0) / vs;
            const bool horizontal = du >= 0.02 && du >= 4.0 * dv;

            const double railV = 0.5 * (ev0 + ev1);
            if (!havePreviousEdge) firstEdgeHorizontal = horizontal;
            for (size_t sampleIndex = 0;
                 sampleIndex < edgeSamples.size(); ++sampleIndex) {
                const auto& [uv, p] = edgeSamples[sampleIndex];
                // A topologically horizontal trim edge owns one row even
                // when its STEP pcurve wiggles slightly in V.  Use that
                // canonical row in the UV partition, while retaining the
                // exact B-rep 3D sample at the canonical key.  This prevents
                // one physical rail sample from being admitted to both
                // neighbouring slabs without moving the exported border.
                gp_Pnt2d ownedUv(
                    uv.X(), horizontal ? railV : uv.Y());
                // The far endpoint of the preceding wire edge is owned by
                // this edge's first sample.  When that preceding edge is a
                // slightly sloped horizontal rail, its canonical row must
                // also own the shared topological junction.  Leaving the
                // adjoining side edge at its noisy endpoint V creates a
                // microscopic extra slab, so the rail interior samples are
                // skipped and the neighbouring planar wall sees an open
                // border.  Only the UV partition coordinate is snapped; the
                // exported point remains the exact shared B-rep vertex.
                if (sampleIndex == 0 && previousHorizontal &&
                    !horizontal) {
                    ownedUv.SetY(previousRailV);
                }
                boundary.push_back(ownedUv);
                boundaryHorizontal.push_back(horizontal);
                samples.push_back(
                    {ownedUv, uv, p, horizontal, railV});
            }
            havePreviousEdge = true;
            previousHorizontal = horizontal;
            previousRailV = railV;
            lastRailV = railV;
        }
        break;
    }
    if (boundary.size() < 3) return false;
    if (previousHorizontal && !firstEdgeHorizontal) {
        boundary.front().SetY(lastRailV);
        samples.front().uv.SetY(lastRailV);
    }
    // Build the partition levels from the canonicalized wire walk itself.
    // Every topological endpoint appears as the next edge's first sample, so
    // re-inserting the pre-snap min/max values would recreate the micro slabs
    // this canonicalization removes.
    for (const ExactSample& sample : samples) {
        exactBoundary[uvKey(sample.uv)] = sample.p;
        levels.push_back(sample.horizontal
                             ? sample.railV : sample.uv.Y());
    }

    // Sparse structural rows keep the large un-notched shoulders smooth.
    // Tooth endpoints and side-edge samples remain mandatory rows.
    for (int j = 1; j < nv; ++j)
        levels.push_back(v0 + (v1 - v0) * j / nv);
    std::sort(levels.begin(), levels.end());
    std::vector<double> compactLevels;
    for (double v : levels) {
        if (compactLevels.empty() ||
            std::abs(v - compactLevels.back()) > stationTolV)
            compactLevels.push_back(v);
    }
    levels.swap(compactLevels);
    if (levels.size() < 2) return false;

    struct Crossing { double u; size_t seg; };
    struct LocalSlab {
        gp_Pnt2d bl, br, tr, tl;
        double vb = 0.0, vt = 0.0;
    };
    std::vector<LocalSlab> slabs;
    const double tolF = BRep_Tool::Tolerance(face);
    auto onSegAtV = [&](size_t k, double v) {
        const gp_Pnt2d& a = boundary[k];
        const gp_Pnt2d& b = boundary[(k + 1) % boundary.size()];
        const double t = std::abs(b.Y() - a.Y()) > 1e-14
            ? std::clamp((v - a.Y()) / (b.Y() - a.Y()), 0.0, 1.0)
            : 0.0;
        return gp_Pnt2d(a.X() + (b.X() - a.X()) * t, v);
    };
    for (size_t j = 0; j + 1 < levels.size(); ++j) {
        const double vb = levels[j], vt2 = levels[j + 1];
        if (vt2 - vb <= vt) continue;
        const double vm = 0.5 * (vb + vt2);
        std::vector<Crossing> cross;
        for (size_t k = 0; k < boundary.size(); ++k) {
            const gp_Pnt2d& a = boundary[k];
            const gp_Pnt2d& b = boundary[(k + 1) % boundary.size()];
            if (boundaryHorizontal[k]) continue;
            if (std::abs(b.Y() - a.Y()) < 1e-14) continue;
            // Half-open crossing rule prevents a sampled trim vertex from
            // appearing twice when the scan line hits it exactly.
            const double lo = std::min(a.Y(), b.Y());
            const double hi = std::max(a.Y(), b.Y());
            if (vm < lo || vm >= hi) continue;
            const double t = (vm - a.Y()) / (b.Y() - a.Y());
            cross.push_back({a.X() + (b.X() - a.X()) * t, k});
        }
        std::sort(cross.begin(), cross.end(),
                  [](const Crossing& a, const Crossing& b) {
                      return a.u < b.u;
                  });
        std::vector<Crossing> uniqueCross;
        for (const Crossing& c : cross) {
            if (uniqueCross.empty() ||
                std::abs(c.u - uniqueCross.back().u) > ut)
                uniqueCross.push_back(c);
        }
        for (size_t k = 0; k + 1 < uniqueCross.size(); ++k) {
            if (uniqueCross[k + 1].u - uniqueCross[k].u <= ut) continue;
            BRepClass_FaceClassifier cls(
                const_cast<TopoDS_Face&>(face),
                gp_Pnt2d(0.5 * (uniqueCross[k].u + uniqueCross[k + 1].u),
                         vm),
                tolF);
            if (cls.State() == TopAbs_OUT) continue;
            const gp_Pnt2d bl = onSegAtV(uniqueCross[k].seg, vb);
            const gp_Pnt2d br = onSegAtV(uniqueCross[k + 1].seg, vb);
            const gp_Pnt2d tr = onSegAtV(uniqueCross[k + 1].seg, vt2);
            const gp_Pnt2d tl = onSegAtV(uniqueCross[k].seg, vt2);
            if (std::max(br.X() - bl.X(), tr.X() - tl.X()) <= ut) continue;
            slabs.push_back({bl, br, tr, tl, vb, vt2});
        }
    }
    if (slabs.empty()) return false;

    // cellCap is a budget, not permission to discard required trim samples.
    // It limits only the optional structural rails.  Dividing by two leaves
    // room for exact samples on the two B-rep rails of every slab.
    const int cap = std::max(16, cellCap);
    const int budgetCols = std::max(
        1, cap / std::max(1, 2 * static_cast<int>(slabs.size())));
    const int fullCols = std::max(1, std::min(nu, budgetCols));

    // A B-spline's U parameter is rarely proportional to distance.  Linear
    // U stations therefore bunch several otherwise global columns into the
    // narrow side corridor of this grip.  Solve the one shared station set
    // by arc length on the widest untrimmed slab, then reuse those exact U
    // values through every V band.  This keeps both phase continuity and
    // visibly even spans without making local trim endpoints persistent.
    const LocalSlab* metricSlab = &slabs.front();
    double metricWidth = -1.0;
    for (const LocalSlab& slab : slabs) {
        const double w = std::min(slab.br.X() - slab.bl.X(),
                                  slab.tr.X() - slab.tl.X());
        if (w > metricWidth) {
            metricWidth = w;
            metricSlab = &slab;
        }
    }
    const double metricV = 0.5 * (metricSlab->vb + metricSlab->vt);
    constexpr int kMetricSteps = 256;
    std::array<double, kMetricSteps + 1> metricU{};
    std::array<double, kMetricSteps + 1> metricLength{};
    gp_Pnt previous = surf.Value(u0, metricV);
    metricU[0] = u0;
    for (int k = 1; k <= kMetricSteps; ++k) {
        const double u = u0 + (u1 - u0) * double(k) / kMetricSteps;
        const gp_Pnt p = surf.Value(u, metricV);
        metricU[k] = u;
        metricLength[k] = metricLength[k - 1] + previous.Distance(p);
        previous = p;
    }
    std::vector<double> primaryU{u0};
    const double metricTotal = metricLength.back();
    for (int i = 1; i < fullCols; ++i) {
        if (metricTotal <= 1e-12) {
            primaryU.push_back(u0 + (u1 - u0) * double(i) / fullCols);
            continue;
        }
        const double target = metricTotal * double(i) / fullCols;
        const auto upper = std::lower_bound(metricLength.begin(),
                                            metricLength.end(), target);
        const int k = std::clamp(int(upper - metricLength.begin()),
                                 1, kMetricSteps);
        const double span = metricLength[k] - metricLength[k - 1];
        const double f = span > 1e-12
            ? (target - metricLength[k - 1]) / span : 0.0;
        primaryU.push_back(metricU[k - 1] +
                           (metricU[k] - metricU[k - 1]) * f);
    }
    primaryU.push_back(u1);

    std::map<std::pair<long long, long long>, uint32_t> verts;
    auto vertex = [&](const gp_Pnt2d& uv) {
        const auto key = uvKey(uv);
        auto found = verts.find(key);
        if (found != verts.end()) return found->second;
        gp_Pnt p = surf.Value(uv.X(), uv.Y());
        auto xb = exactBoundary.find(key);
        if (xb != exactBoundary.end()) {
            p = xb->second;
        } else {
            // Canonicalize a numerically reconstructed side crossing to its
            // exact shared-edge sample when both UV coordinates agree.
            double best = 1e300;
            for (const ExactSample& s : samples) {
                if (std::abs(s.uv.X() - uv.X()) > stationTolU ||
                    std::abs(s.uv.Y() - uv.Y()) > stationTolV)
                    continue;
                const double d = std::abs(s.uv.X() - uv.X()) / stationTolU +
                                 std::abs(s.uv.Y() - uv.Y()) / stationTolV;
                if (d < best) { best = d; p = s.p; }
            }
        }
        const uint32_t id = out.addVertex(p, {faceId, uv.X(), uv.Y()});
        verts.emplace(key, id);
        return id;
    };

    struct RailPoint { double f; gp_Pnt2d uv; };
    auto railSamples = [&](const gp_Pnt2d& left, const gp_Pnt2d& right,
                           double v, double bandHeight,
                           bool interiorTowardPositiveV) {
        std::vector<RailPoint> rail;
        const double width = right.X() - left.X();
        if (width <= ut) return rail;
        for (const ExactSample& s : samples) {
            if (!s.horizontal ||
                std::abs(s.railV - v) > 2.0 * stationTolV)
                continue;
            if (s.uv.X() < left.X() - stationTolU ||
                s.uv.X() > right.X() + stationTolU)
                continue;
            // A horizontal trim separates this face from its neighbour.  It
            // belongs to only the slab on the face-interior side; admitting
            // it to both adjacent slabs duplicates the same directed edge
            // and creates two overlapping layers.  Probe just inside the
            // candidate slab to select the one legitimate owner.
            const double probeStep = std::max(
                8.0 * stationTolV, 0.05 * bandHeight);
            const double probeV = s.sourceUv.Y() +
                (interiorTowardPositiveV ? probeStep : -probeStep);
            BRepClass_FaceClassifier owner(
                const_cast<TopoDS_Face&>(face),
                gp_Pnt2d(s.sourceUv.X(), probeV), tolF);
            if (owner.State() == TopAbs_OUT) continue;
            const double f = std::clamp(
                (s.uv.X() - left.X()) / width, 0.0, 1.0);
            rail.push_back({f, s.uv});
        }
        std::sort(rail.begin(), rail.end(),
                  [](const RailPoint& a, const RailPoint& b) {
                      return a.f < b.f;
                  });
        std::vector<RailPoint> uniqueRail;
        for (const RailPoint& p : rail) {
            if (uniqueRail.empty() ||
                std::abs(p.f - uniqueRail.back().f) > 1e-7)
                uniqueRail.push_back(p);
            else if (std::abs(p.uv.Y() - v) <
                     std::abs(uniqueRail.back().uv.Y() - v))
                uniqueRail.back() = p;
        }
        return uniqueRail;
    };

    const bool faceReversed = face.Orientation() == TopAbs_REVERSED;
    auto rejectLocalCell = [&](const char* reason) {
        dbg("orthogonal local comb face %d: partition rejected (%s)",
            faceId, reason);
        return false;
    };
    int emitted = 0;
    int emittedQuads = 0;
    int emittedNgons = 0;
    for (const LocalSlab& slab : slabs) {
        const double bandHeight = slab.vt - slab.vb;
        const std::vector<RailPoint> bottom =
            railSamples(slab.bl, slab.br, slab.vb, bandHeight, true);
        const std::vector<RailPoint> top =
            railSamples(slab.tl, slab.tr, slab.vt, bandHeight, false);

        auto pointAtU = [&](const std::vector<RailPoint>& exact,
                            double u, double v) {
            const RailPoint* closest = nullptr;
            double du = 1e300;
            for (const RailPoint& p : exact) {
                const double d = std::abs(p.uv.X() - u);
                if (d < du) { du = d; closest = &p; }
            }
            if (closest && du <= stationTolU) return closest->uv;
            return gp_Pnt2d(u, v);
        };

        struct ColumnCut { gp_Pnt2d bottom, top; };
        std::vector<ColumnCut> cuts{{slab.bl, slab.tl}};
        const double commonLeft = std::max(slab.bl.X(), slab.tl.X());
        const double commonRight = std::min(slab.br.X(), slab.tr.X());
        for (int i = 1; i < fullCols; ++i) {
            const double u = primaryU[i];
            if (u <= commonLeft + stationTolU ||
                u >= commonRight - stationTolU)
                continue;
            cuts.push_back({pointAtU(bottom, u, slab.vb),
                            pointAtU(top, u, slab.vt)});
        }
        cuts.push_back({slab.br, slab.tr});
        std::sort(cuts.begin(), cuts.end(), [](const ColumnCut& a,
                                               const ColumnCut& b) {
            return 0.5 * (a.bottom.X() + a.top.X()) <
                   0.5 * (b.bottom.X() + b.top.X());
        });

        auto appendUnique = [&](std::vector<gp_Pnt2d>& uv,
                                const gp_Pnt2d& p) {
            if (uv.empty() || uvKey(uv.back()) != uvKey(p)) uv.push_back(p);
        };
        auto signedArea = [](const std::vector<gp_Pnt2d>& ring) {
            double area = 0.0;
            for (size_t k = 0; k < ring.size(); ++k) {
                const gp_Pnt2d& a = ring[k];
                const gp_Pnt2d& b = ring[(k + 1) % ring.size()];
                area += a.X() * b.Y() - b.X() * a.Y();
            }
            return 0.5 * area;
        };
        auto buildCell = [&](const ColumnCut& left,
                             const ColumnCut& right) {
            std::vector<gp_Pnt2d> uv;
            appendUnique(uv, left.bottom);
            for (const RailPoint& p : bottom) {
                if (p.uv.X() > left.bottom.X() + stationTolU &&
                    p.uv.X() < right.bottom.X() - stationTolU)
                    appendUnique(uv, p.uv);
            }
            appendUnique(uv, right.bottom);
            appendUnique(uv, right.top);
            for (auto it = top.rbegin(); it != top.rend(); ++it) {
                if (it->uv.X() > left.top.X() + stationTolU &&
                    it->uv.X() < right.top.X() - stationTolU)
                    appendUnique(uv, it->uv);
            }
            appendUnique(uv, left.top);
            if (uv.size() > 1 && uvKey(uv.front()) == uvKey(uv.back()))
                uv.pop_back();
            return uv;
        };
        // A sloped exact rail can meet a column endpoint and appear twice in
        // the raw walk.  That creates a zero-area out-and-back excursion (the
        // old duplicate-directed-edge sliver), not a second region.  Strip
        // only a provably zero-area loop; two finite loops are ambiguous and
        // fail closed.  The retained loop is the actual cell boundary.
        const double zeroLoopArea = 1e-14 * std::max(1.0, us * vs);
        auto normalizeCell = [&](std::vector<gp_Pnt2d>& uv) {
            for (size_t guard = 0; guard <= uv.size() + 2; ++guard) {
                std::map<std::pair<long long, long long>, size_t> seen;
                bool foundRepeat = false;
                for (size_t i = 0; i < uv.size(); ++i) {
                    const auto key = uvKey(uv[i]);
                    const auto [it, inserted] = seen.emplace(key, i);
                    if (inserted) continue;
                    const size_t first = it->second;
                    std::vector<gp_Pnt2d> loopA(uv.begin() + first,
                                                uv.begin() + i);
                    std::vector<gp_Pnt2d> loopB(uv.begin() + i, uv.end());
                    loopB.insert(loopB.end(), uv.begin(), uv.begin() + first);
                    auto distinctKeys = [&](const auto& loop) {
                        std::set<std::pair<long long, long long>> keys;
                        for (const gp_Pnt2d& p : loop) keys.insert(uvKey(p));
                        return keys.size();
                    };
                    const bool hairpinA = distinctKeys(loopA) < 3;
                    const bool hairpinB = distinctKeys(loopB) < 3;
                    // Exactly one side of the repeated vertex must be a
                    // combinatorial A-B-A excursion.  Area-based guesses can
                    // discard a real neighbouring region, so two finite loops
                    // (or two empty loops) are never resolved here.
                    if (hairpinA == hairpinB) return false;
                    uv = hairpinA ? std::move(loopB) : std::move(loopA);
                    foundRepeat = true;
                    break;
                }
                if (!foundRepeat) return true;
            }
            return false;
        };
        auto properCross = [](const gp_Pnt2d& a, const gp_Pnt2d& b,
                              const gp_Pnt2d& c, const gp_Pnt2d& d) {
            auto cross = [](const gp_Pnt2d& p, const gp_Pnt2d& q,
                            const gp_Pnt2d& r) {
                return (q.X() - p.X()) * (r.Y() - p.Y()) -
                       (q.Y() - p.Y()) * (r.X() - p.X());
            };
            const double abC = cross(a, b, c), abD = cross(a, b, d);
            const double cdA = cross(c, d, a), cdB = cross(c, d, b);
            return abC * abD < 0.0 && cdA * cdB < 0.0;
        };
        auto simpleCell = [&](const std::vector<gp_Pnt2d>& uv) {
            std::set<std::pair<long long, long long>> unique;
            for (const gp_Pnt2d& p : uv) {
                if (!unique.insert(uvKey(p)).second) return false;
            }
            for (size_t i = 0; i < uv.size(); ++i) {
                const size_t i2 = (i + 1) % uv.size();
                for (size_t j = i + 1; j < uv.size(); ++j) {
                    const size_t j2 = (j + 1) % uv.size();
                    if (i2 == j || j2 == i) continue;
                    if (properCross(uv[i], uv[i2], uv[j], uv[j2]))
                        return false;
                }
            }
            return true;
        };
        // No triangles are emitted.  If trimming leaves a three-corner wedge,
        // remove its internal column separator and rebuild the union directly
        // from the two rail walks.  Boundary wedges merge inward; an interior
        // wedge deterministically merges left.  Rebuilding (rather than a
        // post-hoc polygon union) gives one ownership partition by construction.
        for (;;) {
            bool coarsened = false;
            for (size_t i = 0; i + 1 < cuts.size(); ++i) {
                std::vector<gp_Pnt2d> uv = buildCell(cuts[i], cuts[i + 1]);
                if (!normalizeCell(uv))
                    return rejectLocalCell("finite repeated UV loop");
                if (uv.size() >= 4) continue;
                if (cuts.size() <= 2)
                    return rejectLocalCell("unmergeable boundary wedge");
                const size_t separator = i == 0 ? 1 : i;
                cuts.erase(cuts.begin() + separator);
                coarsened = true;
                break;
            }
            if (!coarsened) break;
        }

        for (size_t i = 0; i + 1 < cuts.size(); ++i) {
            const ColumnCut& left = cuts[i];
            const ColumnCut& right = cuts[i + 1];
            if (right.bottom.X() - left.bottom.X() <= ut ||
                right.top.X() - left.top.X() <= ut)
                continue;
            std::vector<gp_Pnt2d> uv = buildCell(left, right);
            if (!normalizeCell(uv))
                return rejectLocalCell("finite repeated UV loop after merge");
            if (uv.size() < 4)
                return rejectLocalCell("triangle survived column coarsening");
            if (!simpleCell(uv))
                return rejectLocalCell("non-simple rebuilt UV cell");

            double twiceArea = 0.0, cu = 0.0, cv = 0.0;
            for (size_t k = 0; k < uv.size(); ++k) {
                const gp_Pnt2d& a = uv[k];
                const gp_Pnt2d& b = uv[(k + 1) % uv.size()];
                const double cr = a.X() * b.Y() - b.X() * a.Y();
                twiceArea += cr;
                cu += (a.X() + b.X()) * cr;
                cv += (a.Y() + b.Y()) * cr;
            }
            if (std::abs(twiceArea) <= 2.0 * zeroLoopArea)
                return rejectLocalCell("zero-area rebuilt UV cell");
            cu /= 3.0 * twiceArea;
            cv /= 3.0 * twiceArea;
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face),
                                         gp_Pnt2d(cu, cv), tolF);
            if (cls.State() == TopAbs_OUT) continue;

            std::vector<uint32_t> ids;
            ids.reserve(uv.size());
            for (const gp_Pnt2d& p : uv) ids.push_back(vertex(p));
            gp_XYZ nw(0, 0, 0);
            for (size_t k = 0; k < uv.size(); ++k) {
                const gp_XYZ a = surf.Value(uv[k].X(), uv[k].Y()).XYZ();
                const gp_XYZ b = surf.Value(uv[(k + 1) % uv.size()].X(),
                                             uv[(k + 1) % uv.size()].Y()).XYZ();
                nw += gp_XYZ(a.Y() * b.Z() - a.Z() * b.Y(),
                             a.Z() * b.X() - a.X() * b.Z(),
                             a.X() * b.Y() - a.Y() * b.X());
            }
            gp_Pnt sp; gp_Vec du, dv;
            surf.D1(cu, cv, sp, du, dv);
            gp_Vec expected = du.Crossed(dv);
            if (faceReversed) expected.Reverse();
            const double wholeDot = gp_Vec(nw).Dot(expected);
            if (gp_Vec(nw).Magnitude() <= 1e-16 ||
                expected.Magnitude() <= 1e-16 ||
                std::abs(wholeDot) <=
                    1e-14 * gp_Vec(nw).Magnitude() * expected.Magnitude())
                return rejectLocalCell("unknown CAD-normal agreement");
            // Certify the simple ring with a deterministic UV ear clipping.
            // Unlike a centroid fan, ears remain inside a concave cell, so a
            // valid re-entrant boundary is not mistaken for a fold.  Every
            // ear must agree with the CAD normal after output winding.
            const double hand = wholeDot > 0.0 ? 1.0 : -1.0;
            const double uvHand = signedArea(uv) > 0.0 ? 1.0 : -1.0;
            auto uvCross = [](const gp_Pnt2d& a, const gp_Pnt2d& b,
                              const gp_Pnt2d& c) {
                return (b.X() - a.X()) * (c.Y() - a.Y()) -
                       (b.Y() - a.Y()) * (c.X() - a.X());
            };
            auto pointInEar = [&](const gp_Pnt2d& a, const gp_Pnt2d& b,
                                  const gp_Pnt2d& c,
                                  const gp_Pnt2d& p) {
                return uvHand * uvCross(a, b, p) >= -zeroLoopArea &&
                       uvHand * uvCross(b, c, p) >= -zeroLoopArea &&
                       uvHand * uvCross(c, a, p) >= -zeroLoopArea;
            };
            auto certifiedEar = [&](size_t ia, size_t ib, size_t ic) {
                const gp_Pnt a = surf.Value(uv[ia].X(), uv[ia].Y());
                const gp_Pnt b = surf.Value(uv[ib].X(), uv[ib].Y());
                const gp_Pnt c = surf.Value(uv[ic].X(), uv[ic].Y());
                const gp_Vec ab(a, b), ac(a, c);
                const gp_Vec triN = ab.Crossed(ac);
                const gp_Pnt2d probe((uv[ia].X() + uv[ib].X() +
                                      uv[ic].X()) / 3.0,
                                     (uv[ia].Y() + uv[ib].Y() +
                                      uv[ic].Y()) / 3.0);
                gp_Pnt pp; gp_Vec pdu, pdv;
                surf.D1(probe.X(), probe.Y(), pp, pdu, pdv);
                gp_Vec cadN = pdu.Crossed(pdv);
                if (faceReversed) cadN.Reverse();
                return triN.Magnitude() > 1e-16 &&
                       cadN.Magnitude() > 1e-16 &&
                       hand * triN.Dot(cadN) > 0.0;
            };
            std::vector<size_t> ear(uv.size());
            std::iota(ear.begin(), ear.end(), size_t(0));
            while (ear.size() > 3) {
                bool clipped = false;
                for (size_t k = 0; k < ear.size(); ++k) {
                    const size_t ia = ear[(k + ear.size() - 1) % ear.size()];
                    const size_t ib = ear[k];
                    const size_t ic = ear[(k + 1) % ear.size()];
                    if (uvHand * uvCross(uv[ia], uv[ib], uv[ic]) <=
                        zeroLoopArea)
                        continue;
                    bool contains = false;
                    for (size_t m : ear) {
                        if (m == ia || m == ib || m == ic) continue;
                        if (pointInEar(uv[ia], uv[ib], uv[ic], uv[m])) {
                            contains = true;
                            break;
                        }
                    }
                    if (contains || !certifiedEar(ia, ib, ic)) continue;
                    ear.erase(ear.begin() + k);
                    clipped = true;
                    break;
                }
                if (!clipped)
                    return rejectLocalCell("uncertifiable UV ear");
            }
            if (ear.size() != 3 ||
                uvHand * uvCross(uv[ear[0]], uv[ear[1]], uv[ear[2]]) <=
                    zeroLoopArea ||
                !certifiedEar(ear[0], ear[1], ear[2]))
                return rejectLocalCell("folded final UV ear");
            if (wholeDot < 0.0)
                std::reverse(ids.begin(), ids.end());
            out.addPolygon(std::move(ids), faceId, false);
            if (uv.size() == 4) ++emittedQuads;
            else ++emittedNgons;
            ++emitted;
        }
    }
    // A short connector can lie wholly under the first cell of the adjacent
    // broad panel when a sloped trim endpoint is reconstructed at its exact
    // 3D sample.  A repeated *directed* edge is only a warning: adjacent or
    // folded cells can produce the same symptom.  Drop the smaller cell only
    // after its complete UV ring is proven contained in the larger one.
    // Ambiguous overlap fails the route so the caller can demote the face.
    int culledOverlaps = 0;
    {
        PolyMesh& pm = out.mesh();
        std::vector<char> drop(pm.polygons.size(), 0);
        auto uvRing = [&](size_t pi, std::vector<gp_Pnt2d>& ring) {
            ring.clear();
            const auto& poly = pm.polygons[pi];
            ring.reserve(poly.size());
            for (uint32_t vi : poly) {
                if (vi >= pm.anchors.size() ||
                    pm.anchors[vi].faceId != faceId) {
                    return false;
                }
                const Anchor& a = pm.anchors[vi];
                ring.emplace_back(a.u, a.v);
            }
            return ring.size() >= 3;
        };
        auto signedUvArea = [&](const std::vector<gp_Pnt2d>& ring) {
            double area = 0.0;
            for (size_t k = 0; k < ring.size(); ++k) {
                const gp_Pnt2d& a = ring[k];
                const gp_Pnt2d& b = ring[(k + 1) % ring.size()];
                area += a.X() * b.Y() - b.X() * a.Y();
            }
            return 0.5 * area;
        };
        const double uvTol = std::max({ut, vt, 1e-10});
        const double uvTol2 = uvTol * uvTol;
        const double areaTol = 1e-12 * std::max(1.0, us * vs);
        auto orient = [](const gp_Pnt2d& a, const gp_Pnt2d& b,
                         const gp_Pnt2d& c) {
            return (b.X() - a.X()) * (c.Y() - a.Y()) -
                   (b.Y() - a.Y()) * (c.X() - a.X());
        };
        auto onSegment = [&](const gp_Pnt2d& q, const gp_Pnt2d& a,
                             const gp_Pnt2d& b) {
            const double len = std::sqrt(a.SquareDistance(b));
            if (len <= uvTol) return q.SquareDistance(a) <= uvTol2;
            if (std::abs(orient(a, b, q)) > uvTol * len) return false;
            const double dot = (q.X() - a.X()) * (q.X() - b.X()) +
                               (q.Y() - a.Y()) * (q.Y() - b.Y());
            return dot <= uvTol2;
        };
        auto properCross = [&](const gp_Pnt2d& a, const gp_Pnt2d& b,
                               const gp_Pnt2d& c, const gp_Pnt2d& d) {
            const double abC = orient(a, b, c);
            const double abD = orient(a, b, d);
            const double cdA = orient(c, d, a);
            const double cdB = orient(c, d, b);
            return abC * abD < -areaTol * areaTol &&
                   cdA * cdB < -areaTol * areaTol;
        };
        auto simpleRing = [&](const std::vector<gp_Pnt2d>& ring) {
            for (size_t i = 0; i < ring.size(); ++i) {
                const size_t i2 = (i + 1) % ring.size();
                if (ring[i].SquareDistance(ring[i2]) <= uvTol2) return false;
                for (size_t j = i + 1; j < ring.size(); ++j) {
                    const size_t j2 = (j + 1) % ring.size();
                    if (i == j || i2 == j || j2 == i) continue;
                    if (ring[i].SquareDistance(ring[j]) <= uvTol2)
                        return false;
                    const bool hit =
                        properCross(ring[i], ring[i2], ring[j], ring[j2]) ||
                        onSegment(ring[i], ring[j], ring[j2]) ||
                        onSegment(ring[i2], ring[j], ring[j2]) ||
                        onSegment(ring[j], ring[i], ring[i2]) ||
                        onSegment(ring[j2], ring[i], ring[i2]);
                    if (hit) return false;
                }
            }
            return true;
        };
        // -1 outside, 0 on boundary, +1 strictly inside.
        auto classifyPoint = [&](const gp_Pnt2d& q,
                                 const std::vector<gp_Pnt2d>& ring) {
            bool inside = false;
            for (size_t i = 0, j = ring.size() - 1; i < ring.size();
                 j = i++) {
                const gp_Pnt2d& a = ring[j];
                const gp_Pnt2d& b = ring[i];
                if (onSegment(q, a, b)) return 0;
                if ((a.Y() > q.Y()) == (b.Y() > q.Y())) continue;
                const double x = a.X() +
                    (q.Y() - a.Y()) * (b.X() - a.X()) /
                    (b.Y() - a.Y());
                if (x > q.X() + uvTol) inside = !inside;
            }
            return inside ? 1 : -1;
        };
        auto polygonCentroid = [&](const std::vector<gp_Pnt2d>& ring,
                                   gp_Pnt2d& c) {
            double twiceArea = 0.0, cx = 0.0, cy = 0.0;
            for (size_t i = 0; i < ring.size(); ++i) {
                const gp_Pnt2d& a = ring[i];
                const gp_Pnt2d& b = ring[(i + 1) % ring.size()];
                const double cross = a.X() * b.Y() - b.X() * a.Y();
                twiceArea += cross;
                cx += (a.X() + b.X()) * cross;
                cy += (a.Y() + b.Y()) * cross;
            }
            if (std::abs(twiceArea) <= 2.0 * areaTol) return false;
            c.SetX(cx / (3.0 * twiceArea));
            c.SetY(cy / (3.0 * twiceArea));
            return true;
        };
        auto provenContained = [&](size_t innerIndex, size_t outerIndex,
                                   const char*& why) {
            auto reject = [&](const char* reason) {
                why = reason;
                return false;
            };
            std::vector<gp_Pnt2d> inner, outer;
            if (!uvRing(innerIndex, inner) || !uvRing(outerIndex, outer) ||
                !simpleRing(inner) || !simpleRing(outer)) {
                return reject("missing anchors or non-simple UV ring");
            }
            const double innerArea = signedUvArea(inner);
            const double outerArea = signedUvArea(outer);
            if (std::abs(innerArea) <= areaTol ||
                std::abs(outerArea) <= areaTol) {
                return reject("degenerate UV area");
            }
            if (innerArea * outerArea <= 0.0)
                return reject("opposed UV winding");
            if (std::abs(innerArea) >= std::abs(outerArea) - areaTol)
                return reject("no strict smaller cell");
            bool strict = false;
            for (const gp_Pnt2d& p : inner) {
                const int state = classifyPoint(p, outer);
                if (state < 0) return reject("inner vertex outside keeper");
                strict = strict || state > 0;
            }
            // Vertices alone are insufficient for a concave container: an
            // inner edge could leave and re-enter it between endpoints.
            for (size_t i = 0; i < inner.size(); ++i) {
                const gp_Pnt2d& a = inner[i];
                const gp_Pnt2d& b = inner[(i + 1) % inner.size()];
                const gp_Pnt2d mid(0.5 * (a.X() + b.X()),
                                   0.5 * (a.Y() + b.Y()));
                if (classifyPoint(mid, outer) < 0)
                    return reject("inner edge leaves keeper");
                for (size_t j = 0; j < outer.size(); ++j) {
                    if (properCross(a, b, outer[j],
                                    outer[(j + 1) % outer.size()])) {
                        return reject("UV boundaries cross");
                    }
                }
            }
            gp_Pnt2d ci, co;
            if (!polygonCentroid(inner, ci) || !polygonCentroid(outer, co) ||
                classifyPoint(ci, outer) < 0)
                return reject("centroid outside keeper");
            if (!strict) return reject("no strict interior sample");
            BRepClass_FaceClassifier innerOwner(
                const_cast<TopoDS_Face&>(face), ci, tolF);
            BRepClass_FaceClassifier outerOwner(
                const_cast<TopoDS_Face&>(face), co, tolF);
            if (innerOwner.State() == TopAbs_OUT ||
                outerOwner.State() == TopAbs_OUT) {
                return reject("centroid outside CAD face");
            }
            why = "contained";
            return true;
        };
        for (;;) {
            std::map<std::pair<uint32_t, uint32_t>, size_t> directed;
            bool found = false;
            for (size_t pi = 0; pi < pm.polygons.size() && !found; ++pi) {
                if (drop[pi]) continue;
                const auto& poly = pm.polygons[pi];
                for (size_t k = 0; k < poly.size(); ++k) {
                    const auto edge = std::make_pair(
                        poly[k], poly[(k + 1) % poly.size()]);
                    auto [it, inserted] = directed.emplace(edge, pi);
                    if (inserted || drop[it->second]) continue;
                    const size_t other = it->second;
                    std::vector<gp_Pnt2d> a, b;
                    if (!uvRing(pi, a) || !uvRing(other, b)) return false;
                    const size_t loser =
                        std::abs(signedUvArea(a)) < std::abs(signedUvArea(b))
                            ? pi : other;
                    const size_t keeper = loser == pi ? other : pi;
                    const char* overlapWhy = "unknown";
                    if (!provenContained(loser, keeper, overlapWhy)) {
                        dbg("orthogonal local comb face %d: ambiguous "
                            "directed-edge overlap (%s, cells %zu/%zu, "
                            "areas %.9g/%.9g) -> demote",
                            faceId, overlapWhy, loser, keeper,
                            loser == pi ? std::abs(signedUvArea(a))
                                        : std::abs(signedUvArea(b)),
                            keeper == pi ? std::abs(signedUvArea(a))
                                         : std::abs(signedUvArea(b)));
                        if (std::getenv("WEFT_LOCAL_COMB_DEBUG")) {
                            const auto dumpRing = [&](const char* label,
                                                      size_t index,
                                                      const std::vector<gp_Pnt2d>& ring) {
                                for (size_t ri = 0; ri < ring.size(); ++ri) {
                                    dbg("local comb f%d %s cell %zu uv[%zu]="
                                        "(%.12g,%.12g)", faceId, label,
                                        index, ri, ring[ri].X(), ring[ri].Y());
                                }
                            };
                            dumpRing("candidate-a", pi, a);
                            dumpRing("candidate-b", other, b);
                        }
                        return false;
                    }
                    drop[loser] = 1;
                    ++culledOverlaps;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }
        if (culledOverlaps) {
            const bool haveCorners =
                pm.polygonCornerAnchors.size() == pm.polygons.size();
            const bool haveCertified =
                pm.certifiedTriangles.size() == pm.polygons.size();
            std::vector<std::vector<uint32_t>> polygons;
            std::vector<int> faceIds;
            std::vector<std::vector<Anchor>> corners;
            std::vector<std::vector<std::array<uint32_t, 3>>> certified;
            polygons.reserve(pm.polygons.size() - culledOverlaps);
            faceIds.reserve(pm.polygons.size() - culledOverlaps);
            if (haveCorners) corners.reserve(pm.polygons.size() - culledOverlaps);
            if (haveCertified)
                certified.reserve(pm.polygons.size() - culledOverlaps);
            for (size_t pi = 0; pi < pm.polygons.size(); ++pi) {
                if (drop[pi]) continue;
                polygons.push_back(std::move(pm.polygons[pi]));
                faceIds.push_back(pm.polygonFaceId[pi]);
                if (haveCorners)
                    corners.push_back(std::move(pm.polygonCornerAnchors[pi]));
                if (haveCertified)
                    certified.push_back(std::move(pm.certifiedTriangles[pi]));
            }
            pm.polygons = std::move(polygons);
            pm.polygonFaceId = std::move(faceIds);
            if (haveCorners) pm.polygonCornerAnchors = std::move(corners);
            if (haveCertified) pm.certifiedTriangles = std::move(certified);
            emitted -= culledOverlaps;
        }
    }
    dbg("orthogonal local comb face %d: %zu slabs, %d structural cols, "
        "%d quads + %d boundary n-gons, %d covered slivers removed (cap %d)",
        faceId, slabs.size(), fullCols, emittedQuads, emittedNgons,
        culledOverlaps, cellCap);
    return emitted > 0;
}

bool meshOrthogonalTrimGrid(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model, const FacePlan& plan,
                            const std::vector<int>& solvedEdge, int faceId,
                            int nu, int nv, MeshBuilder& out,
                            const PinnedEdges* pins) {
    nu = std::max(1, nu); nv = std::max(1, nv);
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double ut = 1e-9 * std::max(1.0, std::abs(u1 - u0));
    const double vt = 1e-9 * std::max(1.0, std::abs(v1 - v0));
    std::vector<double> baseU, baseV;
    for (int i = 0; i <= nu; ++i)
        baseU.push_back(u0 + (u1 - u0) * i / nu);
    for (int j = 0; j <= nv; ++j)
        baseV.push_back(v0 + (v1 - v0) * j / nv);
    std::vector<double> U = baseU, V = baseV;
    auto addEdgeEndpoints = [&](int eid, bool addU, bool addV) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f, l);
        if (pc.IsNull()) return false;
        const gp_Pnt2d a = pc->Value(f), b = pc->Value(l);
        if (addU) {
            U.push_back(a.X()); U.push_back(b.X());
        }
        if (addV) {
            V.push_back(a.Y()); V.push_back(b.Y());
        }
        return true;
    };
    if (plan.kind == MesherKind::CoonsGrid) {
        // `planOrthogonalTrimGrid` proves that this non-comb freeform face is
        // a split rectangle with no interior U step. Only its segmented side
        // rails own additional row stations; their slight U drift must remain
        // local to the clipped boundary.
        if (!appendPairedSplitRows(face, model, plan,
                                   std::abs(v1 - v0), V)) {
            for (int e : plan.vEdges) {
                if (!addEdgeEndpoints(e, false, true)) return false;
            }
        }
    } else {
        for (int e : plan.orthogonalEdges) {
            // Repeated cylinder cuts are local trim events. Promoting every
            // slot endpoint into a full-height U column destroys the regular
            // angular rhythm with clusters of near-duplicate spans. Keep the
            // base circumferential stations uniform and let clipping plus the
            // assembly edge contract resolve the local cap topology.
            if (!addEdgeEndpoints(
                    e, !plan.orthogonalDrumComb, true)) {
                return false;
            }
        }
    }
    auto normalize = [](std::vector<double>& a, double tol) {
        std::sort(a.begin(), a.end());
        std::vector<double> b;
        for (double x : a) {
            if (b.empty() || std::abs(x - b.back()) > tol) b.push_back(x);
        }
        a.swap(b);
    };
    const double stationTolU = std::max(ut, 1e-4*std::abs(u1-u0));
    const double vStationFraction =
        plan.kind == MesherKind::CoonsGrid ? 2e-5 : 1e-6;
    const double stationTolV =
        std::max(vt, vStationFraction*std::abs(v1-v0));
    normalize(U, stationTolU);
    normalize(V, stationTolV);
    if (U.size() < 2 || V.size() < 2 ||
        (U.size() - 1) * (V.size() - 1) > 200000) return false;

    // Exact sampled trim polygon in wire order. Pins include every crossing
    // with the global station lines, so clipping creates no unsampled point
    // along a shared B-rep edge.
    const double qU = std::max(1e-12, ut), qV = std::max(1e-12, vt);
    auto uvKey = [&](const gp_Pnt2d& p) {
        return std::make_pair(llround(p.X()/qU), llround(p.Y()/qV));
    };
    std::vector<gp_Pnt2d> boundary;
    std::map<std::pair<long long,long long>, gp_Pnt> exactBoundary;
    std::vector<std::pair<gp_Pnt2d,gp_Pnt>> exactSamples;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), face);
             we.More(); we.Next()) {
            const TopoDS_Edge edge = we.Current();
            if (BRep_Tool::Degenerated(edge)) continue;
            const int eid = model.edges.FindIndex(edge);
            if (eid < 1) return false;
            double f, l, f3, l3;
            Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face,
                                                                 f, l);
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(edge, f3, l3);
            if (pc.IsNull() || c3.IsNull()) return false;
            const int n = std::max(1, solvedEdge[eid]);
            const bool rev = edge.Orientation() == TopAbs_REVERSED;
            for (double t : edgeSampleFractions(eid, n, 0.0, rev, false,
                                                pins, &model)) {
                gp_Pnt2d uv = pc->Value(f + (l-f)*t);
                auto snap = [](double x, const std::vector<double>& s,
                               double tol) {
                    auto it = std::lower_bound(s.begin(),s.end(),x);
                    double best=x, d=tol;
                    if (it!=s.end() && std::abs(*it-x)<=d) {
                        d=std::abs(*it-x); best=*it;
                    }
                    if (it!=s.begin()) {
                        --it;
                        if (std::abs(*it-x)<=d) best=*it;
                    }
                    return best;
                };
                uv.SetX(snap(uv.X(),U,stationTolU));
                uv.SetY(snap(uv.Y(),V,stationTolV));
                boundary.push_back(uv);
                const gp_Pnt ep = c3->Value(f3 + (l3-f3)*t);
                exactBoundary[uvKey(uv)] = ep;
                exactSamples.push_back({uv,ep});
            }
        }
        break;
    }
    if (boundary.size() < 3) return false;
    auto clipHalfPlane = [](const std::vector<gp_Pnt2d>& in, bool axisU,
                            double bound, bool keepGreater) {
        std::vector<gp_Pnt2d> out;
        if (in.empty()) return out;
        auto val = [&](const gp_Pnt2d& p) { return axisU ? p.X() : p.Y(); };
        auto inside = [&](const gp_Pnt2d& p) {
            return keepGreater ? val(p) >= bound - 1e-12
                               : val(p) <= bound + 1e-12;
        };
        gp_Pnt2d A = in.back(); bool aIn = inside(A);
        for (const gp_Pnt2d& B : in) {
            const bool bIn = inside(B);
            if (aIn != bIn) {
                const double av = val(A), bv = val(B);
                const double t = std::abs(bv-av) > 1e-15
                    ? std::clamp((bound-av)/(bv-av), 0.0, 1.0) : 0.0;
                out.emplace_back(A.X() + (B.X()-A.X())*t,
                                 A.Y() + (B.Y()-A.Y())*t);
            }
            if (bIn) out.push_back(B);
            A = B; aIn = bIn;
        }
        return out;
    };

    const bool faceReversed = face.Orientation() == TopAbs_REVERSED;
    const double tolF = BRep_Tool::Tolerance(face);
    std::map<std::pair<long long, long long>, uint32_t> verts;
    std::set<uint32_t> boundaryContractVertices;
    auto vertex = [&](const gp_Pnt2d& p) {
        const auto key = uvKey(p);
        auto xb = exactBoundary.find(key);
        bool boundaryContract = xb != exactBoundary.end();
        gp_Pnt p3 = xb != exactBoundary.end()
                        ? xb->second : surf.Value(p.X(), p.Y());
        if (xb == exactBoundary.end()) {
            double best = 1e300;
            for (const auto& [uv,ep] : exactSamples) {
                const double duv = std::abs(uv.X()-p.X())/stationTolU +
                                   std::abs(uv.Y()-p.Y())/stationTolV;
                if (std::abs(uv.X()-p.X()) <= stationTolU &&
                    std::abs(uv.Y()-p.Y()) <= stationTolV && duv < best) {
                    best = duv;
                    p3 = ep;
                    boundaryContract = true;
                }
            }
        }
        auto it = verts.find(key);
        if (it != verts.end()) {
            if (boundaryContract) boundaryContractVertices.insert(it->second);
            return it->second;
        }
        const uint32_t id = out.addVertex(p3, {faceId, p.X(), p.Y()});
        verts.emplace(key, id);
        if (boundaryContract) boundaryContractVertices.insert(id);
        return id;
    };
    int emitted = 0, tris = 0, quads = 0, ngons = 0;
    const size_t polyBegin = out.mesh().polygons.size();
    for (int j = 0; j + 1 < int(V.size()); ++j) {
        if (V[j + 1] - V[j] <= vt) continue;
        const double vb = V[j], vt2 = V[j+1], vm = 0.5*(vb+vt2);
        struct Crossing { double u; size_t seg; };
        std::vector<Crossing> cross;
        for (size_t k = 0; k < boundary.size(); ++k) {
            const gp_Pnt2d& a = boundary[k];
            const gp_Pnt2d& b = boundary[(k+1)%boundary.size()];
            if (std::abs(b.Y()-a.Y()) < 1e-14) continue;
            if (vm < std::min(a.Y(),b.Y()) || vm > std::max(a.Y(),b.Y()))
                continue;
            const double t = (vm-a.Y())/(b.Y()-a.Y());
            cross.push_back({a.X()+(b.X()-a.X())*t,k});
        }
        std::sort(cross.begin(), cross.end(),
                  [](const Crossing& a,const Crossing& b){return a.u<b.u;});
        auto onSegAtV = [&](size_t k, double v) {
            const gp_Pnt2d& a = boundary[k];
            const gp_Pnt2d& b = boundary[(k+1)%boundary.size()];
            const double t = std::abs(b.Y()-a.Y()) > 1e-14
                ? std::clamp((v-a.Y())/(b.Y()-a.Y()),0.0,1.0) : 0.0;
            return gp_Pnt2d(a.X()+(b.X()-a.X())*t,v);
        };
        std::vector<std::vector<gp_Pnt2d>> slabs;
        for (size_t k = 0; k + 1 < cross.size(); ++k) {
            if (cross[k+1].u-cross[k].u <= ut) continue;
            BRepClass_FaceClassifier cc(const_cast<TopoDS_Face&>(face),
                gp_Pnt2d(0.5*(cross[k].u+cross[k+1].u),vm),tolF);
            if (cc.State() == TopAbs_OUT) continue;
            slabs.push_back({onSegAtV(cross[k].seg,vb),
                             onSegAtV(cross[k+1].seg,vb),
                             onSegAtV(cross[k+1].seg,vt2),
                             onSegAtV(cross[k].seg,vt2)});
        }
        for (int i = 0; i + 1 < int(U.size()); ++i) {
            if (U[i + 1] - U[i] <= ut) continue;
            const double ul = U[i], ur = U[i+1];
            for (const auto& slab : slabs) {
            std::vector<gp_Pnt2d> poly = clipHalfPlane(slab,true,ul,true);
            poly = clipHalfPlane(poly,true,ur,false);
            if (poly.size() < 3) continue;
            std::vector<gp_Pnt2d> clean;
            std::set<std::pair<long long,long long>> cleanKeys;
            for (const gp_Pnt2d& p : poly) {
                if (!clean.empty() && p.Distance(clean.back()) <= 1e-10)
                    continue;
                if (cleanKeys.insert(uvKey(p)).second) clean.push_back(p);
            }
            if (clean.size() > 2 && clean.front().Distance(clean.back()) < 1e-10)
                clean.pop_back();
            if (clean.size() < 3) continue;
            double area = 0.0, cu = 0.0, cv = 0.0;
            for (size_t k = 0; k < clean.size(); ++k) {
                const gp_Pnt2d& a = clean[k];
                const gp_Pnt2d& b = clean[(k+1)%clean.size()];
                area += a.X()*b.Y() - b.X()*a.Y();
                cu += a.X(); cv += a.Y();
            }
            if (std::abs(area) < 1e-14) continue;
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face),
                gp_Pnt2d(cu/clean.size(), cv/clean.size()), tolF);
            if (cls.State() == TopAbs_OUT) continue;
            std::vector<uint32_t> ids;
            ids.reserve(clean.size());
            for (const gp_Pnt2d& p : clean) ids.push_back(vertex(p));
            gp_XYZ nw(0,0,0);
            for (size_t k = 0; k < clean.size(); ++k) {
                const gp_XYZ a = surf.Value(clean[k].X(), clean[k].Y()).XYZ();
                const gp_XYZ b = surf.Value(clean[(k+1)%clean.size()].X(),
                                             clean[(k+1)%clean.size()].Y()).XYZ();
                nw += gp_XYZ(a.Y()*b.Z()-a.Z()*b.Y(),
                             a.Z()*b.X()-a.X()*b.Z(),
                             a.X()*b.Y()-a.Y()*b.X());
            }
            gp_Pnt sp; gp_Vec du, dv;
            surf.D1(cu/clean.size(), cv/clean.size(), sp, du, dv);
            gp_Vec expected = du.Crossed(dv);
            if (faceReversed) expected.Reverse();
            if (gp_Vec(nw).Dot(expected) < 0) std::reverse(ids.begin(), ids.end());
            out.addPolygon(std::move(ids), faceId, false);
            if (clean.size() == 3) ++tris;
            else if (clean.size() == 4) ++quads;
            else ++ngons;
            ++emitted;
            }
        }
    }
    // The fine clipping table above contains every trim endpoint so each
    // groove cap is cut exactly.  On a repeated-groove drum those endpoints
    // are LOCAL feature stations, not global topology.  Collapse a fine-grid
    // edge whenever it lies off the uniform primitive rows/columns and the
    // two cells on its sides form one rectangular quad.  At a groove rail or
    // cap one side is absent/curved, so the station remains exactly where the
    // feature owns it; everywhere else the long primitive span is restored.
    // This is the same local-region doctrine as the open-band castellation
    // builder, applied conservatively after exact polygon clipping.
    int localizedEdges = 0;
    // Uniform cylinder stations are the primary topology contract. The old
    // localizer removed interior stations to chase larger quads, but that
    // broke the shared slot-wall and end-transition chains. Keep it available
    // only for diagnostics until it can prove that it preserves every B-rep
    // boundary route.
    const bool enableDrumLocalizer =
        std::getenv("WEFT_ENABLE_DRUM_LOCALIZER") != nullptr;
    if (plan.orthogonalDrumComb && enableDrumLocalizer) {
        PolyMesh& pm = out.mesh();
        const double colTolU = std::max(ut, 2e-8 * std::abs(u1-u0));
        const double colTolV = std::max(vt, 2e-8 * std::abs(v1-v0));
        auto onBase = [](double x, const std::vector<double>& base,
                         double tol) {
            auto it = std::lower_bound(base.begin(), base.end(), x);
            if (it != base.end() && std::abs(*it-x) <= tol) return true;
            return it != base.begin() && std::abs(*std::prev(it)-x) <= tol;
        };
        auto uvArea = [&](const std::vector<uint32_t>& p) {
            double a = 0.0;
            for (size_t i = 0; i < p.size(); ++i) {
                const Anchor& A = pm.anchors[p[i]];
                const Anchor& B = pm.anchors[p[(i+1)%p.size()]];
                a += A.u*B.v - B.u*A.v;
            }
            return a;
        };
        for (;;) {
            struct Use { size_t poly; uint32_t a, b; };
            std::map<std::pair<uint32_t,uint32_t>, std::vector<Use>> uses;
            for (size_t pi = polyBegin; pi < pm.polygons.size(); ++pi) {
                const auto& p = pm.polygons[pi];
                if (p.size() < 3 || p.size() > 4) continue;
                for (size_t k = 0; k < p.size(); ++k) {
                    const uint32_t a = p[k], b = p[(k+1)%p.size()];
                    uses[{std::min(a,b),std::max(a,b)}].push_back({pi,a,b});
                }
            }
            bool changed = false;
            std::set<size_t> claimed;
            for (const auto& [shared, use] : uses) {
                if (use.size() != 2 || use[0].poly == use[1].poly) continue;
                const Anchor& A = pm.anchors[shared.first];
                const Anchor& B = pm.anchors[shared.second];
                if (A.faceId != faceId || B.faceId != faceId) continue;
                const bool constU = std::abs(A.u-B.u) <= colTolU;
                const bool constV = std::abs(A.v-B.v) <= colTolV;
                if (!constU && !constV) continue;
                if (constU && onBase(0.5*(A.u+B.u),baseU,colTolU)) continue;
                if (constV && onBase(0.5*(A.v+B.v),baseV,colTolV)) continue;

                const size_t pa = use[0].poly, pb = use[1].poly;
                if (claimed.count(pa) || claimed.count(pb) ||
                    pm.polygons[pa].empty() || pm.polygons[pb].empty()) {
                    continue;
                }
                std::map<std::pair<uint32_t,uint32_t>, int> count;
                std::map<uint32_t,std::vector<uint32_t>> adjacency;
                for (size_t pi : {pa,pb}) {
                    const auto& p = pm.polygons[pi];
                    for (size_t k = 0; k < p.size(); ++k) {
                        const uint32_t x=p[k], y=p[(k+1)%p.size()];
                        ++count[{std::min(x,y),std::max(x,y)}];
                    }
                }
                for (const auto& [e,c] : count) {
                    if (c != 1) continue;
                    adjacency[e.first].push_back(e.second);
                    adjacency[e.second].push_back(e.first);
                }
                if (adjacency.size() < 4 ||
                    std::any_of(adjacency.begin(), adjacency.end(),
                                [](const auto& kv){return kv.second.size()!=2;})) {
                    continue;
                }
                std::vector<uint32_t> ring;
                uint32_t start = adjacency.begin()->first, prev = UINT32_MAX;
                uint32_t cur = start;
                do {
                    ring.push_back(cur);
                    const auto& n = adjacency[cur];
                    const uint32_t next = n[0] == prev ? n[1] : n[0];
                    prev = cur; cur = next;
                } while (cur != start && ring.size() <= adjacency.size()+1);
                if (cur != start || ring.size() != adjacency.size()) continue;

                bool reduced = true;
                while (reduced && ring.size() > 4) {
                    reduced = false;
                    for (size_t k = 0; k < ring.size(); ++k) {
                        // These vertices are the immutable B-rep edge sample
                        // contract shared with the neighbouring face. A comb
                        // station may be removed from the interior of a drum,
                        // but never from its trim boundary: doing so changes
                        // one side's edge segmentation and opens the slot,
                        // rim, or end-transition seam.
                        if (boundaryContractVertices.count(ring[k])) continue;
                        const Anchor& P = pm.anchors[ring[(k+ring.size()-1)%ring.size()]];
                        const Anchor& Q = pm.anchors[ring[k]];
                        const Anchor& R = pm.anchors[ring[(k+1)%ring.size()]];
                        const double cross=(Q.u-P.u)*(R.v-Q.v)-
                                           (Q.v-P.v)*(R.u-Q.u);
                        const double scale=std::hypot(Q.u-P.u,Q.v-P.v)*
                                           std::hypot(R.u-Q.u,R.v-Q.v);
                        if (std::abs(cross) <= 1e-9*std::max(1e-30,scale)) {
                            ring.erase(ring.begin()+k);
                            reduced = true;
                            break;
                        }
                    }
                }
                if (ring.size() != 4) continue;
                if (uvArea(ring) * uvArea(pm.polygons[pa]) < 0.0)
                    std::reverse(ring.begin(),ring.end());
                // Candidate must remain a simple UV quad and face the CAD
                // sheet. This is deliberately checked inside the builder,
                // before the global fold census, so localizing comb stations
                // cannot introduce a late sliver/fold after validation.
                auto orientUv = [&](uint32_t ia, uint32_t ib, uint32_t ic) {
                    const Anchor& a = pm.anchors[ia];
                    const Anchor& b = pm.anchors[ib];
                    const Anchor& c = pm.anchors[ic];
                    return (b.u-a.u)*(c.v-a.v) -
                           (b.v-a.v)*(c.u-a.u);
                };
                auto crosses = [&](uint32_t ia, uint32_t ib,
                                   uint32_t ic, uint32_t id) {
                    const double abC = orientUv(ia,ib,ic);
                    const double abD = orientUv(ia,ib,id);
                    const double cdA = orientUv(ic,id,ia);
                    const double cdB = orientUv(ic,id,ib);
                    return abC*abD < -1e-20 && cdA*cdB < -1e-20;
                };
                if (crosses(ring[0],ring[1],ring[2],ring[3]) ||
                    crosses(ring[1],ring[2],ring[3],ring[0])) {
                    continue;
                }
                double cu = 0.0, cv = 0.0;
                gp_XYZ nw(0,0,0);
                for (size_t k = 0; k < ring.size(); ++k) {
                    const Anchor& a = pm.anchors[ring[k]];
                    const Anchor& b = pm.anchors[ring[(k+1)%ring.size()]];
                    cu += a.u; cv += a.v;
                    const gp_XYZ pa3 = surf.Value(a.u,a.v).XYZ();
                    const gp_XYZ pb3 = surf.Value(b.u,b.v).XYZ();
                    nw += gp_XYZ(pa3.Y()*pb3.Z()-pa3.Z()*pb3.Y(),
                                 pa3.Z()*pb3.X()-pa3.X()*pb3.Z(),
                                 pa3.X()*pb3.Y()-pa3.Y()*pb3.X());
                }
                cu /= ring.size(); cv /= ring.size();
                gp_Pnt sp; gp_Vec du,dv;
                surf.D1(cu,cv,sp,du,dv);
                gp_Vec expected = du.Crossed(dv);
                if (faceReversed) expected.Reverse();
                if (nw.Modulus() <= 1e-16 || expected.Magnitude() <= 1e-16)
                    continue;
                if (gp_Vec(nw).Dot(expected) < 0.0) {
                    std::reverse(ring.begin(),ring.end());
                }
                pm.polygons[pa] = std::move(ring);
                pm.polygons[pb].clear();
                claimed.insert(pa);
                claimed.insert(pb);
                ++localizedEdges;
                changed = true;
            }
            if (!changed) break;
        }
        if (localizedEdges) {
            std::vector<std::vector<uint32_t>> polys;
            std::vector<int> owners;
            polys.reserve(pm.polygons.size()-localizedEdges);
            owners.reserve(pm.polygonFaceId.size()-localizedEdges);
            for (size_t i = 0; i < pm.polygons.size(); ++i) {
                if (pm.polygons[i].empty()) continue;
                polys.push_back(std::move(pm.polygons[i]));
                owners.push_back(i < pm.polygonFaceId.size()
                                     ? pm.polygonFaceId[i] : faceId);
            }
            pm.polygons = std::move(polys);
            pm.polygonFaceId = std::move(owners);
            emitted -= localizedEdges;
            tris = quads = ngons = 0;
            for (size_t i = polyBegin; i < pm.polygons.size(); ++i) {
                if (pm.polygons[i].size()==3) ++tris;
                else if (pm.polygons[i].size()==4) ++quads;
                else ++ngons;
            }
        }
    }

    // Clipping a non-convex trim exactly on a station line can leave a
    // numerically tiny cell whose Newell test chooses the opposite hand.
    // Propagate one consistent winding through shared lattice edges; each
    // connected component's seed was already oriented to the CAD normal.
    {
        PolyMesh& pm = out.mesh();
        struct Use { size_t poly; bool forward; };
        std::map<std::pair<uint32_t,uint32_t>, std::vector<Use>> uses;
        for (size_t pi = 0; pi < pm.polygons.size(); ++pi) {
            const auto& p = pm.polygons[pi];
            for (size_t k = 0; k < p.size(); ++k) {
                const uint32_t a = p[k], b = p[(k+1)%p.size()];
                uses[{std::min(a,b),std::max(a,b)}].push_back(
                    {pi, a < b});
            }
        }
        std::vector<std::vector<std::pair<size_t,bool>>> adj(pm.polygons.size());
        for (const auto& [edge, u] : uses) {
            if (u.size() != 2) continue;
            const bool same = u[0].forward == u[1].forward;
            adj[u[0].poly].push_back({u[1].poly, same});
            adj[u[1].poly].push_back({u[0].poly, same});
        }
        std::vector<int> parity(pm.polygons.size(), -1);
        for (size_t root = 0; root < parity.size(); ++root) {
            if (parity[root] >= 0) continue;
            parity[root] = 0;
            std::vector<size_t> todo{root};
            while (!todo.empty()) {
                const size_t a = todo.back(); todo.pop_back();
                for (const auto& [b, toggle] : adj[a]) {
                    const int want = parity[a] ^ int(toggle);
                    if (parity[b] < 0) { parity[b] = want; todo.push_back(b); }
                }
            }
        }
        for (size_t i = 0; i < parity.size(); ++i)
            if (parity[i]) std::reverse(pm.polygons[i].begin(),
                                        pm.polygons[i].end());
    }
    dbg("orthogonal grid face %d: %zux%zu fine stations, %d cells (%d tri, "
        "%d quad, %d ngon), localized %d feature edges", faceId, U.size(),
        V.size(), emitted, tris, quads, ngons, localizedEdges);
    return emitted > 0;
}

}  // namespace weft::mesher_impl
