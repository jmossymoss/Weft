#include "mesher_internal.hpp"

namespace weft {
using mesher_impl::DensitySolution;
using mesher_impl::FacePlan;
using mesher_impl::MeshBuilder;
using mesher_impl::PinnedEdges;
using mesher_impl::applyOrthogonalEdgeContracts;
using mesher_impl::closedEdgePhase;
using mesher_impl::conformFallbackBorders;
using mesher_impl::dbg;
using mesher_impl::edgeSampleFractions;
using mesher_impl::faceWithinDeflection;
using mesher_impl::fuseSeamTwins;
using mesher_impl::isClosedRevolution;
using mesher_impl::isGeometricClosedRevolution;
using mesher_impl::meshAnnulusCRing;
using mesher_impl::meshAnnulusRing;
using mesher_impl::meshContractFallback;
using mesher_impl::meshCoonsGrid;
using mesher_impl::meshDiskCap;
using mesher_impl::meshDomeCap;
using mesher_impl::meshFallback;
using mesher_impl::meshMinimalNGon;
using mesher_impl::meshMinimalPlanar;
using mesher_impl::meshOrthogonalLocalComb;
using mesher_impl::meshOrthogonalTrimGrid;
using mesher_impl::meshParametricGrid;
using mesher_impl::meshPlateWeb;
using mesher_impl::meshQuadFill;
using mesher_impl::meshRailLadder;
using mesher_impl::meshRevolutionGrid;
using mesher_impl::meshRevolutionInsert;
using mesher_impl::meshRevolutionOpenBand;
using mesher_impl::meshRevolutionRimNotch;
using mesher_impl::meshRevolutionTaper;
using mesher_impl::meshRibbonSweep;
using mesher_impl::meshRingJunction;
using mesher_impl::meshRingLattice;
using mesher_impl::meshSurfaceCapFan;
using mesher_impl::meshTrimCorridor;
using mesher_impl::outerWireSolvedTotal;
using mesher_impl::pairPartTris;
using mesher_impl::pinCastellatedRims;
using mesher_impl::pinFilletChains;
using mesher_impl::pinOrthogonalTrimGrids;
using mesher_impl::planFace;
using mesher_impl::propagateBandRadialToBlendGroup;
using mesher_impl::revolutionUPhase;
using mesher_impl::ringAnchorAngle;
using mesher_impl::solveDensity;
using mesher_impl::stableDeflectionCount;
using mesher_impl::stitchSeams;
using mesher_impl::unionSeams;

namespace {
bool sameFaceSettings(const FaceMeshSettings& a,
                      const FaceMeshSettings& b) {
    return a.radial == b.radial && a.axial == b.axial &&
           a.gridU == b.gridU && a.gridV == b.gridV && a.cap == b.cap &&
           a.chordTolerance == b.chordTolerance &&
           a.angleToleranceDeg == b.angleToleranceDeg &&
           a.filletLoops == b.filletLoops &&
           a.filletHold == b.filletHold &&
           a.junctionRings == b.junctionRings &&
           a.quadDominant == b.quadDominant &&
           a.pureTriFloor == b.pureTriFloor && a.minimal == b.minimal &&
           a.exclude == b.exclude && a.forceMesher == b.forceMesher &&
           a.linkRims == b.linkRims && a.minSize == b.minSize &&
           a.relativeDeviation == b.relativeDeviation &&
           a.weldTolerance == b.weldTolerance &&
           a.squareCollar == b.squareCollar &&
           a.coonsRotate == b.coonsRotate && a.boundary == b.boundary &&
           a.adaptive == b.adaptive && a.cellCap == b.cellCap;
}

struct CachedFacePlan {
    FaceMeshSettings effective;
    FaceMeshSettings defaults;
    bool explicitFace = false;
    bool decoupleSeams = false;
    FacePlan plan;
};

struct FacePlanCache {
    std::map<int, CachedFacePlan> faces;
};

using CornerCell = std::tuple<long long, long long, long long>;
struct CachedCorner {
    gp_Pnt at;
    double tol = 0.0;
    gp_Pnt target;
};
struct CachedUvCorner {
    double u = 0.0;
    double v = 0.0;
    double uTol = 0.0;
    double vTol = 0.0;
    gp_Pnt target;
};
struct CornerRepairCache {
    std::vector<CachedCorner> corners;
    std::map<CornerCell, std::vector<size_t>> grid;
    // Indexed by FaceId. Anchored legacy vertices may snap only when their
    // face UV proves that they are a topological B-rep corner.
    std::vector<std::vector<CachedUvCorner>> faceCorners;
    double cell = 1e-7;
    double structuredNoiseTol = 1e-7;
    int microEdges = 0;
};

}  // namespace

PolyMesh generate(const Model& model, const Analysis& analysis,
                  const GenerationSettings& settingsIn, GenerationReport* report,
                  GenerationCache* cache) {
    const bool timingEnabled = std::getenv("WEFT_TIMINGS") != nullptr;
    auto timingLast = std::chrono::steady_clock::now();
    const auto timingBegin = timingLast;
    auto timingCheckpoint = [&](const char* stage) {
        if (!timingEnabled) return;
        const auto now = std::chrono::steady_clock::now();
        const auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - timingLast)
                               .count();
        const auto total = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now - timingBegin)
                               .count();
        std::fprintf(stderr, "timing: %-20s %6lld ms (%6lld total)\n", stage,
                     static_cast<long long>(delta),
                     static_cast<long long>(total));
        timingLast = now;
    };
    // Local mutable copy: a per-face radial override on a revolution band is
    // propagated across its connected blend group (below) so the whole barrel
    // densifies as one unit instead of stranding a neighbour at the old count.
    GenerationSettings settings = settingsIn;
    dbg("generate: begin (%d faces, %d edges, parallel=%d, conform=%d)",
        model.faceCount(), model.edgeCount(), settings.parallelMeshing ? 1 : 0,
        settings.conformBorders ? 1 : 0);
    std::shared_ptr<FacePlanCache> planCache;
    if (cache) {
        if (cache->facePlans) {
            planCache =
                std::static_pointer_cast<FacePlanCache>(cache->facePlans);
        } else {
            planCache = std::make_shared<FacePlanCache>();
            cache->facePlans = planCache;
        }
    }
    std::map<int, FacePlan> plans;
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const FaceMeshSettings& effective = settings.forFace(fid);
        const bool explicitFace = settings.perFace.count(fid) != 0;
        FacePlan plan;
        bool reusedPlan = false;
        if (planCache) {
            auto it = planCache->faces.find(fid);
            if (it != planCache->faces.end() &&
                it->second.explicitFace == explicitFace &&
                it->second.decoupleSeams == settings.decoupleSeams &&
                sameFaceSettings(it->second.effective, effective) &&
                sameFaceSettings(it->second.defaults, settings.defaults)) {
                plan = it->second.plan;
                reusedPlan = true;
            }
        }
        if (!reusedPlan) {
            plan = planFace(fid, model, analysis, settings, cache);
        }
        if (!reusedPlan && effective.exclude) {
            // Deleted faces neither mesh nor constrain their neighbours'
            // densities — their borders become free boundary loops.
            plan.kind = MesherKind::Fallback;
            plan.constrains = false;
        }
        if (!reusedPlan && planCache) {
            planCache->faces[fid] = {effective, settings.defaults,
                                     explicitFace, settings.decoupleSeams,
                                     plan};
        }
        plans.emplace(fid, std::move(plan));
    }
    dbg("generate: plans done");
    timingCheckpoint("face planning");
    propagateBandRadialToBlendGroup(analysis, plans, settings);

    DensitySolution density = solveDensity(model, plans, settings, cache);
    timingCheckpoint("density proposals");
    // Curvature floor, every mode: a curved edge solved below its turn
    // angle collapses to chords — observed as two bracket-bend
    // quarter-pipes flattening into the SAME plane strip and weld-fusing
    // non-manifold. One segment per ~60 degrees of turn is the least
    // that keeps distinct geometry distinct; explicit counts (per-edge
    // pins AND per-face overrides) win outright — 16 radial segments
    // means exactly 16.
    //
    // The floor is applied per density GROUP, not per edge: raising one
    // member of a matched group above its siblings breaks the equality
    // the meshers rely on (two faces then sample the same border at
    // different counts — an open seam by construction). Groups that
    // carry a solved count take the raise through groupCount so
    // countFor readers (the per-face count tables) agree with the flat
    // solvedEdge table below; proposal-less groups keep countFor's
    // fallback semantics and are floored uniformly in the table only.
    std::map<int, int> floorOfRoot;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        if (settings.perEdge.count(eid)) continue;
        const int root = density.groups.find(eid);
        if (density.pinnedRoots.count(root)) continue;
        const TopoDS_Edge E = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(E)) continue;
        double f, l;
        Handle(Geom_Curve) c3 = BRep_Tool::Curve(E, f, l);
        if (c3.IsNull()) continue;
        GeomAdaptor_Curve gc(c3, f, l);
        if (gc.GetType() == GeomAbs_Line) continue;
        try {
            int floorN = 0;
            if (cache) {
                auto fit = cache->curvatureFloors.find(eid);
                if (fit != cache->curvatureFloors.end()) floorN = fit->second;
            }
            if (floorN == 0) {
                floorN = std::clamp(
                    stableDeflectionCount(gc, M_PI / 3.0, 1e6), 1, 32);
                if (cache) cache->curvatureFloors[eid] = floorN;
            }
            auto [it, inserted] = floorOfRoot.try_emplace(root, floorN);
            if (!inserted && it->second < floorN) it->second = floorN;
        } catch (const Standard_Failure&) {
        }
    }
    for (const auto& [root, floorN] : floorOfRoot) {
        auto it = density.groupCount.find(root);
        if (it != density.groupCount.end() && it->second < floorN) {
            it->second = floorN;
        }
    }
    // Flat per-edge count table: lets meshers consume per-edge counts from
    // worker threads (union-find lookups path-compress, so countFor can't
    // run concurrently).
    std::vector<int> solvedEdge(model.edgeCount() + 1, 0);
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        solvedEdge[eid] = density.countFor(eid, 0);
        auto it = floorOfRoot.find(density.groups.find(eid));
        if (it != floorOfRoot.end() && solvedEdge[eid] < it->second) {
            solvedEdge[eid] = it->second;  // proposal-less group: uniform
        }
    }
    timingCheckpoint("curvature floors");
    auto geometryEdgeLength = [&](int eid) {
        if (cache) {
            auto it = cache->edgeLengths.find(eid);
            if (it != cache->edgeLengths.end()) return it->second;
        }
        double len = 0.0;
        try {
            const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
            BRepAdaptor_Curve c(e);
            len = GCPnts_AbscissaPoint::Length(c);
        } catch (const Standard_Failure&) {
        }
        if (cache) cache->edgeLengths[eid] = len;
        return len;
    };
    // Wire floor: a closed wire sampled at fewer than 3 border vertices
    // cannot bound any polygon; rail ladders need 4 so their two tips split
    // the outline into two real rails. Per-face and per-edge pins are user
    // requests, not permission to create impossible topology, so this hard
    // floor clamps them too. Raise the wire's longest edge through its whole
    // density group so both bordering faces read the same count everywhere.
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        for (TopExp_Explorer wx(model.faces(fid), TopAbs_WIRE); wx.More();
             wx.Next()) {
            int total = 0;
            std::vector<int> wireEdges;
            const int minimum =
                plans.at(fid).kind == MesherKind::RailLadder ? 4 : 3;
            for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More();
                 ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                if (BRep_Tool::Degenerated(e)) continue;
                const int eid = model.edges.FindIndex(e);
                if (eid < 1) continue;
                wireEdges.push_back(eid);
                total += std::max(0, solvedEdge[eid]);
            }
            // Nearly every valid wire already clears the hard floor. The old
            // loop integrated every CAD edge's arc length before discovering
            // that no repair was needed; on MP9 that was thousands of costly
            // GCPnts solves after every edit. Only rank edges on the rare wire
            // that actually needs a bump.
            if (total == 0 || total >= minimum || wireEdges.empty()) continue;
            int bumpEid = 0;
            double bumpLen = -1.0;
            for (int eid : wireEdges) {
                const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
                double cf, cl;
                if (BRep_Tool::Curve(e, cf, cl).IsNull()) continue;
                const double len = geometryEdgeLength(eid);
                if (len > bumpLen) {
                    bumpLen = len;
                    bumpEid = eid;
                }
            }
            if (bumpEid == 0) continue;
            dbg("density: wire on face %d totals %d samples, edge %d "
                "raised by %d",
                fid, total, bumpEid, minimum - total);
            const int target = solvedEdge[bumpEid] + minimum - total;
            const int root = density.groups.find(bumpEid);
            auto it = density.groupCount.find(root);
            if (it != density.groupCount.end() && it->second < target) {
                it->second = target;
            }
            for (int e = 1; e <= model.edgeCount(); ++e) {
                if (density.groups.find(e) == root && solvedEdge[e] < target) {
                    solvedEdge[e] = target;
                }
            }
        }
    }
    timingCheckpoint("wire floors");
    // Annulus containment floor: on a plate with holes, the outer ring's
    // chords cut INSIDE the true boundary — if a chord sags deeper than
    // the clearance to a hole, the sampled hole protrudes through the
    // sampled outer polygon and no web can triangulate it (a thin
    // annular plate whose hole ring densified through a neighbour's
    // radial edit while the outer ring stayed coarse demoted to raw
    // triangulation exactly this way). Raise each outer edge until its
    // sag stays under half the clearance. Hole rings need nothing: a
    // hole's chords sag INTO the hole, away from the outer boundary.
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        int wireCount = 0;
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            ++wireCount;
        }
        if (wireCount < 2) continue;
        const TopoDS_Wire outerW = BRepTools::OuterWire(face);
        if (outerW.IsNull()) continue;
        // Clearance: nearest approach between the outer wire and any
        // hole wire, sampled coarsely (exact enough for a floor).
        std::vector<gp_Pnt> outPts, holePts;
        auto sampleWirePts = [&](const TopoDS_Shape& w,
                                 std::vector<gp_Pnt>& pts) {
            for (TopExp_Explorer ex(w, TopAbs_EDGE); ex.More(); ex.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(ex.Current());
                if (BRep_Tool::Degenerated(e)) continue;
                double f, l;
                if (BRep_Tool::Curve(e, f, l).IsNull()) continue;
                BRepAdaptor_Curve c(e);
                for (int k = 0; k <= 16; ++k) {
                    pts.push_back(c.Value(f + (l - f) * k / 16.0));
                }
            }
        };
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            if (wx.Current().IsSame(outerW)) {
                sampleWirePts(wx.Current(), outPts);
            } else {
                sampleWirePts(wx.Current(), holePts);
            }
        }
        if (outPts.empty() || holePts.empty()) continue;
        double gap2 = 1e300;
        for (const gp_Pnt& p : outPts) {
            for (const gp_Pnt& q : holePts) {
                gap2 = std::min(gap2, p.SquareDistance(q));
            }
        }
        const double allow = 0.5 * std::sqrt(gap2);
        if (!(allow > 1e-9)) continue;
        for (TopExp_Explorer ex(outerW, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(e)) continue;
            const int eid = model.edges.FindIndex(e);
            if (eid < 1) continue;
            // Unlike the other floors this one overrides pins too: an
            // explicit count that makes the sampled hole protrude
            // through the sampled outer boundary cannot be honoured —
            // no web can triangulate that region, and the only escapes
            // are raw triangulation or an open seam. Overrides are
            // clamped before they can break a neighbour (the plan's
            // override-safety rule); the raise is logged.
            const int root = density.groups.find(eid);
            double f, l;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(e, f, l);
            if (c3.IsNull()) continue;
            GeomAdaptor_Curve gc(c3, f, l);
            if (gc.GetType() == GeomAbs_Line) continue;
            int n = std::max(1, solvedEdge[eid]);
            auto sagOk = [&](int nn) {
                for (int k = 0; k < nn; ++k) {
                    const double t0 = f + (l - f) * k / double(nn);
                    const double t1 = f + (l - f) * (k + 1) / double(nn);
                    const gp_Pnt a = gc.Value(t0), b = gc.Value(t1);
                    const gp_Pnt m = gc.Value(0.5 * (t0 + t1));
                    const gp_Pnt c(0.5 * (a.X() + b.X()),
                                   0.5 * (a.Y() + b.Y()),
                                   0.5 * (a.Z() + b.Z()));
                    if (m.Distance(c) > allow) return false;
                }
                return true;
            };
            int target = n;
            while (target < 256 && !sagOk(target)) target *= 2;
            target = std::min(target, 256);
            if (target <= n) continue;
            dbg("density: face %d outer edge %d sag floor %d -> %d "
                "(clearance %.3g)",
                fid, eid, n, target, allow);
            auto git = density.groupCount.find(root);
            if (git != density.groupCount.end() && git->second < target) {
                git->second = target;
            }
            for (int e2 = 1; e2 <= model.edgeCount(); ++e2) {
                if (density.groups.find(e2) == root &&
                    solvedEdge[e2] < target) {
                    solvedEdge[e2] = target;
                }
            }
        }
    }
    // Strip pitch floor (adaptive only): a long thin strip — a fillet
    // segment, a grip ribbon, a rail band — whose length direction is
    // near-straight solves to 1-2 stations under curvature-adaptive
    // counts and meshes as monster slats (observed 23:1 cells on the
    // flaregun trigger strip: 10.3-long rails at 2 against half-circle
    // caps at 7). Curvature is the wrong ruler along a strip: the rungs
    // must march by ARC LENGTH relative to the strip's width, so the
    // quad flow reads evenly along the whole feature — and, because
    // every segment of a chained fillet applies the same width-derived
    // pitch, rung spacing stays uniform across large regions instead of
    // bunching at bends. Only strip-shaped plans (coons strips, ribbon
    // sweeps, rail ladders) take the floor; width comes from the robust
    // isoperimetric estimate w = 2A/L (exact for long rectangles), and
    // every non-degenerate outline edge floors to ceil(len / (K*w)) —
    // across-edges (len ~ w) floor to 1, a no-op. Explicit pins win;
    // non-adaptive faces are the user's manual counts and stay alone,
    // which also keeps the flat-count default profile byte-identical.
    //
    // Groups the pitch floor must not touch: open-band and castellated
    // revolution rims emit their chain edges through strip/web passes
    // that can't honour an arbitrary raised count — pitching a straight
    // rim edge (which no other floor ever raises; the curvature floor
    // skips lines) demoted two flaregun bands to the contract floor.
    timingCheckpoint("annulus floors");
    std::set<int> pitchFragileRoots;
    for (const auto& [fid, plan] : plans) {
        if (plan.kind != MesherKind::RevolutionGrid) continue;
        if (plan.bandSides.empty() && !plan.castellated) continue;
        for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE); ex.More();
             ex.Next()) {
            const int eid = model.edges.FindIndex(ex.Current());
            if (eid >= 1) pitchFragileRoots.insert(density.groups.find(eid));
        }
    }
    // Hairline gauge: strips narrower than 0.1% of the model are seam
    // shims, not visible flow — rungs there are pure pathology (a
    // 0.008-wide teleporter sliver pitched to 24 stations handed the
    // coons untangler 15 inverted cells it couldn't recover).
    double modelDiag = 0.0;
    bool needsStripPitch = false;
    for (const auto& [fid, plan] : plans) {
        const bool stripKind = plan.kind == MesherKind::RibbonSweep ||
                               plan.kind == MesherKind::RailLadder ||
                               plan.kind == MesherKind::CoonsGrid;
        if (stripKind && plan.constrains && settings.forFace(fid).adaptive) {
            needsStripPitch = true;
            break;
        }
    }
    // BRepBndLib::Add can populate OCCT triangulation caches. Do not even
    // query the box when the non-adaptive profile will skip every pitch floor:
    // merely warming that cache changed later fallback output on complex parts.
    if (needsStripPitch) {
        if (cache && cache->modelDiagonal >= 0.0) {
            modelDiag = cache->modelDiagonal;
        } else {
            Bnd_Box bb;
            BRepBndLib::Add(model.shape, bb);
            if (!bb.IsVoid()) {
                double x0, y0, z0, x1, y1, z1;
                bb.Get(x0, y0, z0, x1, y1, z1);
                modelDiag =
                    gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1));
            }
            if (cache) cache->modelDiagonal = modelDiag;
        }
    }
    for (const auto& [fid, plan] : plans) {
        const bool stripKind = plan.kind == MesherKind::RibbonSweep ||
                               plan.kind == MesherKind::RailLadder ||
                               plan.kind == MesherKind::CoonsGrid;
        if (!stripKind || !plan.constrains) continue;
        if (!settings.forFace(fid).adaptive) continue;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        double area = 0, perim = 0;
        if (cache) {
            auto ait = cache->faceAreas.find(fid);
            if (ait != cache->faceAreas.end()) area = ait->second;
            auto pit = cache->faceOuterPerimeters.find(fid);
            if (pit != cache->faceOuterPerimeters.end()) perim = pit->second;
        }
        try {
            if (!(area > 0.0)) {
                GProp_GProps sp;
                BRepGProp::SurfaceProperties(face, sp);
                area = sp.Mass();
                if (cache) cache->faceAreas[fid] = area;
            }
            if (!(perim > 0.0)) {
                GProp_GProps lp;
                BRepGProp::LinearProperties(BRepTools::OuterWire(face), lp);
                perim = lp.Mass();
                if (cache) cache->faceOuterPerimeters[fid] = perim;
            }
        } catch (const Standard_Failure&) {
            continue;
        }
        if (!(area > 1e-12) || !(perim > 1e-9)) continue;
        const double w = 2.0 * area / perim;      // strip width estimate
        const double along = 0.5 * perim - w;     // strip length estimate
        if (!(w > 1e-9) || along < 3.0 * w) continue;  // not a strip
        if (w < 1e-3 * modelDiag) continue;            // hairline shim
        // Rung pitch: one station per 2 strip-widths, riding the density
        // dial like every adaptive proposal. (1.5 was tried and pushed a
        // neighbour band past its contract; 2.0 keeps every planned
        // mesher building while the flow already reads even.)
        const double pitch =
            2.0 * w / std::max(0.05, settings.densityScale);
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(e)) continue;
            const int eid = model.edges.FindIndex(e);
            if (eid < 1 || settings.perEdge.count(eid)) continue;
            const int root = density.groups.find(eid);
            if (density.pinnedRoots.count(root)) continue;
            if (pitchFragileRoots.count(root)) continue;
            double cf, cl;
            if (BRep_Tool::Curve(e, cf, cl).IsNull()) continue;
            BRepAdaptor_Curve c(e);
            const double len = geometryEdgeLength(eid);
            // The artist's rule (2026-07-11): a STRAIGHT rail carries no
            // stations by default — "along the blend" is 1 unless the
            // fillet actually curves along its length, and then the
            // curvature LAW sizes it (the pocket ring's 9 stations),
            // not this flow floor. The pitch floor predates the
            // radius-scaled law; it survives only for curved rails.
            if (c.GetType() == GeomAbs_Line) continue;
            {
                const gp_Pnt A = c.Value(c.FirstParameter());
                const gp_Pnt B = c.Value(c.LastParameter());
                const gp_Pnt M = c.Value(0.5 * (c.FirstParameter() +
                                                c.LastParameter()));
                const gp_XYZ ab = B.XYZ() - A.XYZ();
                const double ab2 = ab.SquareModulus();
                double sag;
                if (ab2 > 1e-24) {
                    const gp_XYZ am = M.XYZ() - A.XYZ();
                    const double t =
                        std::clamp(am.Dot(ab) / ab2, 0.0, 1.0);
                    sag = (am - ab * t).Modulus();
                } else {
                    sag = len;  // closed loop: genuinely curved
                }
                if (sag < 1e-4 * len) continue;  // straight bspline rail
            }
            // Hairline slivers bound the rung count instead of exploding
            // it: stations ~ len/width goes quadratic on a 0.1-wide rail
            // (observed 56 rungs on a cosmetic sliver), so each edge is
            // capped at 24 stations from this floor.
            const int target = std::min(
                24, int(std::ceil(len / std::max(1e-9, pitch))));
            if (target <= solvedEdge[eid]) continue;
            dbg("density: face %d strip edge %d pitch floor %d -> %d "
                "(len %.3g, width %.3g)",
                fid, eid, solvedEdge[eid], target, len, w);
            auto git = density.groupCount.find(root);
            if (git != density.groupCount.end() && git->second < target) {
                git->second = target;
            }
            for (int e2 = 1; e2 <= model.edgeCount(); ++e2) {
                if (density.groups.find(e2) == root &&
                    solvedEdge[e2] < target) {
                    solvedEdge[e2] = target;
                }
            }
        }
    }
    // Section-strip intrinsic rail floor.  Its geometric proof deliberately
    // permits each concave end closure to occupy up to one third of the rail,
    // while the final solved-row census requires three safe central rows (two
    // actual body intervals).  Two curvature-driven spans therefore cannot
    // ever build even though the face is valid: they provide only the single
    // midpoint row and make output depend on unrelated neighbour pressure.
    // Six longitudinal spans are the smallest context-independent sampling
    // that always exposes rows at 1/3, 1/2, and 2/3.  Raise only adaptive,
    // unpinned rail groups; explicit author counts and fragile revolution
    // groups remain authoritative and will demote honestly if too coarse.
    constexpr int sectionStripMinSpans = 6;
    for (const auto& [fid, plan] : plans) {
        if (!plan.sectionStrip || plan.kind != MesherKind::CoonsGrid) continue;
        const FaceMeshSettings& fs = settings.forFace(fid);
        if (!fs.adaptive || fs.exclude) continue;
        for (int side : {0, 2}) {
            const std::vector<int>& rail = plan.coonsSides[side];
            if (rail.empty()) continue;
            auto railTotal = [&]() {
                int total = 0;
                for (int eid : rail) {
                    if (eid < 1 || eid >= int(solvedEdge.size())) return -1;
                    total += std::max(0, solvedEdge[eid]);
                }
                return total;
            };
            const int before = railTotal();
            if (before < 1 || before >= sectionStripMinSpans) continue;
            for (int guard = 0; guard < sectionStripMinSpans * 2; ++guard) {
                if (railTotal() >= sectionStripMinSpans) break;
                int bestEdge = 0;
                double bestPitch = -1.0;
                for (int eid : rail) {
                    if (eid < 1 || eid > model.edgeCount() ||
                        settings.perEdge.count(eid)) {
                        continue;
                    }
                    const int root = density.groups.find(eid);
                    if (density.pinnedRoots.count(root) ||
                        pitchFragileRoots.count(root)) {
                        continue;
                    }
                    const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
                    if (BRep_Tool::Degenerated(edge)) continue;
                    const double len = geometryEdgeLength(eid);
                    if (!(len > 1e-12)) continue;
                    const double pitch =
                        len / std::max(1, solvedEdge[eid]);
                    if (pitch > bestPitch) {
                        bestPitch = pitch;
                        bestEdge = eid;
                    }
                }
                if (bestEdge == 0) break;
                const int root = density.groups.find(bestEdge);
                const int target = solvedEdge[bestEdge] + 1;
                auto git = density.groupCount.find(root);
                if (git != density.groupCount.end() && git->second < target) {
                    git->second = target;
                }
                for (int eid = 1; eid <= model.edgeCount(); ++eid) {
                    if (density.groups.find(eid) == root &&
                        solvedEdge[eid] < target) {
                        solvedEdge[eid] = target;
                    }
                }
            }
            const int after = railTotal();
            if (after > before) {
                dbg("density: face %d section rail %d floor %d -> %d "
                    "spans", fid, side / 2, before, after);
            }
        }
    }
    // Rail-station ALIGNMENT (adaptive coons strips): opposite chained
    // sides match by SUM, but their stations sit at whatever arc
    // fractions the per-piece counts imply — unequal piece densities put
    // station k at different fractions on the two rails, and since the
    // lattice connects station k to station k (the continuity the
    // fillet loops crossing the band NEED), every rung shears
    // diagonally. The artist's requirement is BOTH: continuous AND
    // perpendicular — so align the FRACTIONS themselves: raise
    // per-piece counts toward an arc-proportional distribution at a
    // common total N' (monotone raises only, so every floor and pin
    // survives; N' is capped so one dense curvy piece cannot explode
    // the strip; raises apply through the density groups so neighbours
    // follow coherently). Rungs then come out straight and continuous
    // with no mesher re-pairing at all.
    timingCheckpoint("strip pitch floors");
    bool railAlignAny = false;
    for (const auto& [fid2, plan2] : plans) {
        (void)plan2;
        if (settings.forFace(fid2).adaptive) {
            railAlignAny = true;
            break;
        }
    }
    if (railAlignAny) {
        std::set<int> alignedRoots;  // first-come: don't re-move a group
        for (const auto& [fid, plan] : plans) {
            if (plan.kind != MesherKind::CoonsGrid) continue;
            if (!settings.forFace(fid).adaptive) continue;
            if (settings.forFace(fid).exclude) continue;
            const bool matchedLocalRails =
                plan.trimCorridor || plan.sectionStrip;
            bool anyChain = matchedLocalRails;
            for (int i = 0; i < 4; ++i) {
                if (plan.coonsSides[i].size() > 1) anyChain = true;
            }
            if (!anyChain) continue;
            auto sideEdges = [&](int i) {
                std::vector<int> v = plan.coonsSides[i];
                if (v.empty()) {
                    const auto& src = (i % 2 == 0) ? plan.uEdges
                                                   : plan.vEdges;
                    const int idx = i / 2;
                    if (int(src.size()) > idx && src[idx] > 0) {
                        v.push_back(src[idx]);
                    }
                }
                return v;
            };
            for (int pr = 0; pr < 2; ++pr) {
                std::vector<int> A = sideEdges(pr);
                std::vector<int> C = sideEdges(pr + 2);
                if (A.empty() || C.empty()) continue;
                if (A.size() < 2 && C.size() < 2 &&
                    !matchedLocalRails) {
                    continue;
                }
                auto chainLens = [&](const std::vector<int>& ch,
                                     std::vector<double>& L) {
                    double total = 0;
                    L.clear();
                    for (int eid : ch) {
                        if (eid < 1 || eid > model.edgeCount()) return -1.0;
                        const TopoDS_Edge e =
                            TopoDS::Edge(model.edges(eid));
                        if (BRep_Tool::Degenerated(e)) return -1.0;
                        const double l = geometryEdgeLength(eid);
                        if (l <= 1e-12) return -1.0;
                        L.push_back(l);
                        total += l;
                    }
                    return total;
                };
                std::vector<double> LA, LC;
                const double totA = chainLens(A, LA);
                const double totC = chainLens(C, LC);
                if (totA <= 0 || totC <= 0) continue;
                auto sumOf = [&](const std::vector<int>& ch) {
                    int n = 0;
                    for (int eid : ch) {
                        if (eid >= int(solvedEdge.size()) ||
                            solvedEdge[eid] < 1) {
                            return -1;
                        }
                        n += solvedEdge[eid];
                    }
                    return n;
                };
                const int NA = sumOf(A), NC = sumOf(C);
                // Unequal sums align too — both rails land on the same
                // arc-proportional total, which ALSO equalizes them (the
                // SUM repair below then no-ops for this pair).
                if (NA < 2 || NC < 2) continue;
                const int N0 = std::max(NA, NC);
                bool blocked = false;
                for (const std::vector<int>* ch : {&A, &C}) {
                    for (int eid : *ch) {
                        const int root = density.groups.find(eid);
                        if (density.pinnedRoots.count(root) ||
                            alignedRoots.count(root)) {
                            blocked = true;
                        }
                    }
                }
                if (blocked) continue;
                // Common total: every piece's current count must fit
                // under the arc-proportional line, on both rails.
                double need = N0;
                auto needOf = [&](const std::vector<int>& ch,
                                  const std::vector<double>& L,
                                  double tot) {
                    for (size_t i = 0; i < ch.size(); ++i) {
                        need = std::max(
                            need, solvedEdge[ch[i]] * tot / L[i]);
                    }
                };
                needOf(A, LA, totA);
                needOf(C, LC, totC);
                int Np = int(std::ceil(need - 1e-9));
                const int cap =
                    std::min(64, std::max(N0 + 4, int(2.5 * N0)));
                if (Np > cap) continue;  // one hot piece; not worth it
                // Largest-remainder arc-proportional targets at Np,
                // clamped up to current; iterate the common total until
                // both rails carry it exactly.
                auto targetsOf = [&](const std::vector<int>& ch,
                                     const std::vector<double>& L,
                                     double tot, int total,
                                     std::vector<int>& t) {
                    t.assign(ch.size(), 0);
                    std::vector<std::pair<double, size_t>> rem;
                    int used = 0;
                    for (size_t i = 0; i < ch.size(); ++i) {
                        const double ideal = total * L[i] / tot;
                        t[i] = std::max(1, int(ideal));
                        rem.push_back({ideal - int(ideal), i});
                        used += t[i];
                    }
                    std::sort(rem.rbegin(), rem.rend());
                    for (size_t k = 0; used < total && k < rem.size();
                         ++k, ++used) {
                        ++t[rem[k].second];
                    }
                    int sum = 0;
                    for (size_t i = 0; i < ch.size(); ++i) {
                        t[i] = std::max(t[i], solvedEdge[ch[i]]);
                        sum += t[i];
                    }
                    return sum;
                };
                std::vector<int> tA, tC;
                bool ok = false;
                for (int iter = 0; iter < 4; ++iter) {
                    const int sA = targetsOf(A, LA, totA, Np, tA);
                    const int sC = targetsOf(C, LC, totC, Np, tC);
                    if (sA == Np && sC == Np) {
                        ok = true;
                        break;
                    }
                    Np = std::max(sA, sC);
                    if (Np > cap) break;
                }
                if (!ok) continue;
                // Apply through the groups (monotone raises only).
                auto raise = [&](const std::vector<int>& ch,
                                 const std::vector<int>& t) {
                    for (size_t i = 0; i < ch.size(); ++i) {
                        const int root = density.groups.find(ch[i]);
                        alignedRoots.insert(root);
                        auto git = density.groupCount.find(root);
                        if (git != density.groupCount.end() &&
                            git->second < t[i]) {
                            git->second = t[i];
                        }
                        for (int e2 = 1; e2 <= model.edgeCount(); ++e2) {
                            if (density.groups.find(e2) == root &&
                                solvedEdge[e2] < t[i]) {
                                solvedEdge[e2] = t[i];
                            }
                        }
                    }
                };
                dbg("density: face %d rail align pair %d: %d/%d -> %d "
                    "stations (arc-proportional)",
                    fid, pr, NA, NC, Np);
                raise(A, tA);
                raise(C, tC);
            }
        }
    }

    // Chained-coons SUM repair: opposite chained sides of a coons patch
    // match by SUM of their per-edge counts — an equality the solver's
    // chain pass establishes BEFORE the floors above run. Any post-solve
    // raise on a chain member (curvature floor, containment, the strip
    // pitch) skews the patch: the grid absorbs the mismatch as diagonal
    // cells and folds (unterlaf's 310-long wall chains folded 2 cells
    // per face exactly this way). Re-balance by raising the lighter
    // side's longest unpinned edge until the sums meet again. The constraint
    // graph is not always consistent: several faces can share the same groups
    // in a cycle with a fixed offset (nasty_cheese has a four-face +10 cycle).
    // Monotone raises then pump that cycle on every pass and make the topology
    // dramatically worse. Treat the repair as a transaction: keep it only if
    // the complete graph reaches a fixpoint; otherwise restore the safe
    // post-floor counts and let the existing local transition strips absorb
    // the mismatches.
    timingCheckpoint("rail alignment");
    bool hasAdaptiveFaces = false;
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        if (settings.forFace(fid).adaptive) {
            hasAdaptiveFaces = true;
            break;
        }
    }
    // Decoupled-seams experiment: skip the global sum repair entirely and
    // let the post-weld unionSeams splice absorb chain mismatches.
    if (hasAdaptiveFaces && !settings.decoupleSeams) {
        const std::vector<int> beforeChainRepair = solvedEdge;
        const auto groupCountBeforeChainRepair = density.groupCount;
        int brepOpenEdges = 0;
        int brepEdges = 0;
        for (int eid = 1; eid <= model.edgeCount(); ++eid) {
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(edge)) continue;
            ++brepEdges;
            if (!model.edgeToFaces.Contains(edge) ||
                model.edgeToFaces.FindFromKey(edge).Extent() < 2) {
                ++brepOpenEdges;
            }
        }
        // A mostly-open authored sheet (tork: 833/1226 naked B-rep edges) has
        // no closed-solid flow to equalize globally. Raising its shared groups
        // only distorts already-broken patches, so leave its local strip
        // transitions alone just as the importer's dropped-face cap does.
        const bool chainRepairEligible =
            brepEdges > 0 && brepOpenEdges * 3 < brepEdges;
        bool chainRepairConverged = !chainRepairEligible;
        if (!chainRepairEligible) {
            dbg("density: chained-coons sum repair skipped on open sheet "
                "(%d/%d naked B-rep edges)",
                brepOpenEdges, brepEdges);
        }
        for (int pass = 0; pass < (chainRepairEligible ? 16 : 0); ++pass) {
            bool changed = false;
            for (const auto& [fid, plan] : plans) {
                if (plan.kind != MesherKind::CoonsGrid) continue;
                for (int pr = 0; pr < 2; ++pr) {
                    const auto& A = plan.coonsSides[pr];
                    const auto& B = plan.coonsSides[pr + 2];
                    if (A.empty() || B.empty()) continue;
                    if (A.size() == 1 && B.size() == 1 &&
                        !(plan.trimCorridor || plan.sectionStrip)) {
                        continue;  // ordinary Coons single edges are grouped
                    }
                    long tA = 0, tB = 0;
                    for (int e : A) tA += std::max(0, solvedEdge[e]);
                    for (int e : B) tB += std::max(0, solvedEdge[e]);
                    if (tA == tB || tA == 0 || tB == 0) continue;
                    const auto& light = tA < tB ? A : B;
                    const long deficit = std::labs(tA - tB);
                    int bumpEid = 0;
                    double bumpLen = -1.0;
                    for (int e : light) {
                        if (settings.perEdge.count(e)) continue;
                        if (density.pinnedRoots.count(
                                density.groups.find(e))) {
                            continue;
                        }
                        const TopoDS_Edge E = TopoDS::Edge(model.edges(e));
                        if (BRep_Tool::Degenerated(E)) continue;
                        double cf, cl;
                        if (BRep_Tool::Curve(E, cf, cl).IsNull()) continue;
                        const double len = geometryEdgeLength(e);
                        if (len > bumpLen) {
                            bumpLen = len;
                            bumpEid = e;
                        }
                    }
                    if (bumpEid == 0) continue;
                    const int target = std::min<long>(
                        256, solvedEdge[bumpEid] + deficit);
                    if (target <= solvedEdge[bumpEid]) continue;
                    dbg("density: face %d coons chain sum %ld != %ld, "
                        "edge %d raised to %d",
                        fid, tA, tB, bumpEid, target);
                    const int root = density.groups.find(bumpEid);
                    auto git = density.groupCount.find(root);
                    if (git != density.groupCount.end() &&
                        git->second < target) {
                        git->second = target;
                    }
                    for (int e2 = 1; e2 <= model.edgeCount(); ++e2) {
                        if (density.groups.find(e2) == root &&
                            solvedEdge[e2] < target) {
                            solvedEdge[e2] = target;
                        }
                    }
                    changed = true;
                }
            }
            if (!changed) {
                chainRepairConverged = true;
                break;
            }
        }
        if (!chainRepairConverged) {
            solvedEdge = beforeChainRepair;
            density.groupCount = groupCountBeforeChainRepair;
            dbg("density: chained-coons sum repair did not converge; "
                "rolled back transaction");
        }
    }
    timingCheckpoint("chain sum repair");
    // Revolution rim SUM constraint: when a T-junction splits one rim of
    // a closed band into k edges while the other stays a full circle,
    // the totals must agree or the band needs a transition strip — and
    // on thin fillet tubes those strip cells degenerate into folded
    // lunes. Raise the lighter rim (through its whole density group, so
    // stacked bands and cones follow) until the totals meet. Raises are
    // monotone and capped, so the fixpoint terminates.
    // (Skipped under the decoupled-seams experiment: the mismatch then
    // surfaces at the weld and unionSeams splices it.)
    if (!settings.decoupleSeams) {
        auto raiseGroup = [&](int eid, int target) {
            target = std::min(target, 256);
            const int root = density.groups.find(eid);
            for (int e = 1; e <= model.edgeCount(); ++e) {
                if (density.groups.find(e) == root &&
                    solvedEdge[e] < target) {
                    solvedEdge[e] = target;
                }
            }
        };
        // Lower a whole shared-edge group to a simple-density target (never
        // below 1). Used to collapse a castellated boolean rim's freeform
        // over-sampling to clean spans; the group is shared only with
        // analytic body strips, which mesh exactly at low counts.
        auto capGroup = [&](int eid, int target) {
            target = std::max(1, target);
            const int root = density.groups.find(eid);
            for (int e = 1; e <= model.edgeCount(); ++e) {
                if (density.groups.find(e) == root &&
                    solvedEdge[e] > target) {
                    solvedEdge[e] = target;
                }
            }
        };
        for (int pass = 0; pass < 16; ++pass) {
            bool changed = false;
            for (const auto& [fid, plan] : plans) {
                if (plan.kind != MesherKind::RevolutionGrid) continue;
                // Full-wrap castellated rims are the closed-band analog:
                // the plain rim drives the columns and the notch is
                // boolean-cut, so raising the plain rim to the castellated
                // total would only over-mesh (and re-arm the irreconcilable
                // bail) — skip them exactly like multi-edge open bands.
                if (plan.castellated) continue;
                const auto& lo = plan.rimLow;
                const auto& hi = plan.rimHigh;
                if (lo.empty() || hi.empty()) continue;
                if (!plan.bandSides.empty()) {
                    // Open bands with CHAINED rims never equalize: their
                    // columns come from the flat rim alone and the
                    // chain's total is absorbed by the strip/webs —
                    // raising the plain rim to the chain's sum is what
                    // made the radial dial dead on partial barrel walls.
                    // But a rerouted DRUM (one edge per rim) welds its
                    // columns to BOTH rims 1:1 — unequal rims force a
                    // hair-thin hug-row bridge that folds on shallow
                    // segments (foam's countersink quarters, 16 x 1-cell
                    // folds under the census gate). Equalize those, and
                    // let the raise chain through shared rims so stacked
                    // co-axial segments carry continuous columns.
                    if (lo.size() == 1 && hi.size() == 1) {
                        const int a = solvedEdge[lo[0]];
                        const int b = solvedEdge[hi[0]];
                        const auto& small = a < b ? lo : hi;
                        if (a != b &&
                            !density.pinnedRoots.count(
                                density.groups.find(small[0]))) {
                            raiseGroup(small[0], std::max(a, b));
                            changed = true;
                            dbg("density: face %d drum rims %d/%d "
                                "equalized",
                                fid, a, b);
                        }
                    }
                    continue;
                }
                if (lo.size() == 1 && hi.size() == 1) continue;
                long tLo = 0, tHi = 0;
                for (int e : lo) tLo += solvedEdge[e];
                for (int e : hi) tHi += solvedEdge[e];
                if (tLo == tHi) continue;
                // Only a lone closed rim gets raised to the arcs'
                // total. Spreading a deficit across a multi-edge chain
                // pumps shared chain edges back and forth between the
                // faces that share them (both pipe walls carry the
                // same saddle edges) and never converges to sane
                // counts — an unresolved mismatch takes the transition
                // strip instead.
                const auto& small = tLo < tHi ? lo : hi;
                const auto& large = tLo < tHi ? hi : lo;
                const long deficit = std::labs(tHi - tLo);
                if (small.size() != 1) {
                    // A small boolean/T-junction split can leave two short
                    // rim chains just one or two stations apart. The
                    // revolution mesher cannot build that mismatch and used
                    // to demote an otherwise ordinary cone/cylinder to the
                    // contract floor (MP9 face 906: 7 versus 6). Repair only
                    // this tightly bounded case, choosing a group absent
                    // from the opposite rim so the raise cannot cancel out.
                    if (small.size() <= 4 && large.size() <= 4 &&
                        deficit <= 2) {
                        std::set<int> oppositeRoots;
                        for (int e : large) {
                            oppositeRoots.insert(density.groups.find(e));
                        }
                        for (int e : small) {
                            const int root = density.groups.find(e);
                            if (oppositeRoots.count(root) ||
                                density.pinnedRoots.count(root)) {
                                continue;
                            }
                            raiseGroup(e, solvedEdge[e] + int(deficit));
                            changed = true;
                            dbg("density: face %d split rim totals %ld/%ld "
                                "equalized on edge %d",
                                fid, tLo, tHi, e);
                            break;
                        }
                    }
                    continue;
                }
                // A rim split into MANY edges is a castellated boolean rim
                // (foam's top ring: 74 feature arcs, each a short bspline
                // intersection curve the freeform chord gate over-samples),
                // not a genuine few-way T-junction. Raising the lone clean
                // opposite rim to that inflated sum shatters the whole band
                // into one spanning column per feature arc — exactly the
                // "segment loops to support the booleans" pathology. In game
                // topology, leave the clean rim clean and let the transition
                // strip carry the mismatch. (A real T-junction splits a rim
                // into a handful of arcs, so the threshold stays well clear.)
                // Skip only the castellated-boolean-rim pathology: the heavy
                // rim is split into MANY short arcs (>=24) AND its total dwarfs
                // the clean rim (>=8x) because those arcs are bspline boolean
                // cuts the freeform chord gate over-samples. A genuine few-way
                // T-junction (a handful of arcs, totals within a small factor)
                // still equalizes so its thin transition strip can't fold.
                const long heavy = std::max(tLo, tHi);
                const long light = std::max<long>(1, std::min(tLo, tHi));
                if (settings.defaults.minimal && large.size() >= 24 &&
                    heavy >= 8 * light) {
                    // Collapse the castellated rim's over-sampled arcs to
                    // simple density (each short analytic-boundary arc needs
                    // ~1 segment), so the band meshes as clean spans instead
                    // of one spanning column per arc. Shared only with the
                    // analytic body strips, which stay exact at count 1;
                    // user-pinned rings keep their explicit count.
                    for (int e : large) {
                        if (density.pinnedRoots.count(density.groups.find(e))) {
                            continue;
                        }
                        if (solvedEdge[e] > 1) {
                            capGroup(e, 1);
                            changed = true;
                        }
                    }
                    continue;
                }
                // A user-pinned ring never gets raised behind their
                // back — the mismatch stays visible (strip or floor).
                if (density.pinnedRoots.count(
                        density.groups.find(small[0]))) {
                    continue;
                }
                raiseGroup(small[0], solvedEdge[small[0]] + int(deficit));
                changed = true;
                dbg("density: face %d rim totals %ld/%ld equalized", fid,
                    tLo, tHi);
            }
            if (!changed) break;
        }
    }
    timingCheckpoint("rim sum repair");
    if (const char* dumpE = getenv("WEFT_EDGE_DEBUG")) {
        std::stringstream ss(dumpE);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            const int e = atoi(tok.c_str());
            if (e >= 1 && e <= model.edgeCount()) {
                dbg("solvedEdge[%d] = %d", e, solvedEdge[e]);
            }
        }
    }
    dbg("generate: density solved");
    timingCheckpoint("density repairs");

    // Pin castellated rims' base arcs to their column azimuths (the
    // column-alignment contract): the notch band and the neighbour annulus
    // both emit these exact positions, so columns run straight to the cut
    // rim with no inset strip and no top-to-bottom phase break.
    PinnedEdges pinnedEdge(model.edgeCount() + 1);
    pinCastellatedRims(model, plans, settings, solvedEdge, density,
                       pinnedEdge);
    // Fillet flow-through: pin blend-chain cross-rails to the band columns
    // so columns run barrel -> fillet -> fillet -> lower band unbroken.
    pinFilletChains(model, plans, solvedEdge, pinnedEdge);
    // Support-loop hold changes the sampling fractions along a fillet's
    // across edges. Pin those fractions model-wide so fallback/planar
    // neighbours sample the same shared edge instead of staying uniform and
    // opening a seam. Chained sides require chain-global parameterization and
    // keep their existing uniform contract for now.
    for (const auto& [fid, plan] : plans) {
        const FaceMeshSettings& fs = settings.forFace(fid);
        if (!plan.isFillet || fs.filletHold <= 0.0) continue;
        bool chained = false;
        for (const auto& side : plan.coonsSides) chained |= !side.empty();
        if (chained) continue;
        const auto& across = plan.acrossIsU ? plan.uEdges : plan.vEdges;
        for (int eid : across) {
            if (eid < 1 || eid >= int(pinnedEdge.size()) ||
                !pinnedEdge[eid].empty()) {
                continue;
            }
            const int n = std::max(1, solvedEdge[eid]);
            pinnedEdge[eid] = clusteredParams(n, fs.filletHold);
        }
    }
    // One station set owns every side of an orthogonal trim. Propagate the
    // intersections onto the shared B-rep edges so neighbouring faces emit
    // the same points; otherwise a clean quad lattice would merely hide
    // T-junctions along its border.
    pinOrthogonalTrimGrids(model, plans, settings, solvedEdge, pinnedEdge);

    // Research-backed boundary ownership, rollout R1: freeze the FINAL legacy
    // count/pin decisions into one immutable sequence per ordinary shared
    // line/circle B-rep edge before any face worker starts.  Freeform edges
    // stay on their existing even-arc path until UV/sample-id migration.
    // edgeSampleFractions() now
    // adapts to this table whenever a caller requests that exact count and
    // phase; special counts remain on the legacy path.  This deliberately
    // changes ownership before it changes any primitive mesher topology.
    const bool enableCanonicalEdgeContracts =
        settings.canonicalEdgeContracts ||
        std::getenv("WEFT_USE_CANONICAL_EDGE_TABLE");
    int canonicalSharedEdges = 0;
    int canonicalSharedSamples = 0;
    if (enableCanonicalEdgeContracts) {
        std::vector<double> canonicalEdgePhase(model.edgeCount() + 1, 0.0);
        for (int eid = 1; eid <= model.edgeCount(); ++eid) {
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(edge) ||
                !model.edgeToFaces.Contains(edge) ||
                model.edgeToFaces.FindFromKey(edge).Extent() != 2) {
                continue;
            }
            canonicalEdgePhase[eid] = closedEdgePhase(edge, model);
        }
        const std::shared_ptr<const CanonicalEdgeTable> canonicalEdgeTable =
            mesher_detail::buildCanonicalEdgeTable(
                model, solvedEdge, pinnedEdge, canonicalEdgePhase);
        pinnedEdge.canonical = canonicalEdgeTable;
        for (const CanonicalEdgePlan& edge : canonicalEdgeTable->edgePlans) {
            canonicalSharedEdges += edge.edgeId > 0 ? 1 : 0;
            if (edge.edgeId > 0) {
                canonicalSharedSamples +=
                    static_cast<int>(edge.samples.size());
            }
        }
        dbg("generate: %d shared edges own canonical sample sequences",
            canonicalSharedEdges);
    }

    // Resolve every face's division counts up front (union-find lookups
    // path-compress, so they must not run concurrently) — after this the
    // per-face meshing is embarrassingly parallel.
    const int faceN = model.faceCount();
    std::vector<std::array<int, 3>> counts(faceN + 1, {0, 0, 0});
    // The primary/secondary counts each face ACTUALLY built, filled in at
    // mesh time by the meshers whose built count can differ from the `counts`
    // table above — the annulus body drives its columns off the dense rim
    // (not the sparse notch rim uEdges[0] pre-samples), and the boundary /
    // rail meshers carry no interior grid count at all (they left {0,0}).
    // -1 = "not reported, fall back to counts[]". Report-only: never keys the
    // cache and never drives a mesher, so no mesh output moves.
    std::vector<std::array<int, 2>> builtCounts(faceN + 1, {-1, -1});
    for (int fid = 1; fid <= faceN; ++fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        const FacePlan& plan = plans.at(fid);
        auto solved = [&](const std::vector<int>& edges, int fallback) {
            if (edges.empty()) return fallback;
            // Combine countFor (group counts, pins, per-face defaults
            // for unproposed edges like seams) with solvedEdge (which
            // additionally carries the curvature floor and the rim-
            // total raises the group solve doesn't see).
            return std::max(solvedEdge[edges[0]],
                            density.countFor(edges[0], fallback));
        };
        switch (plan.kind) {
            case MesherKind::RevolutionGrid:
            case MesherKind::DiskCap: {
                if (plan.orthogonalTrimGrid) {
                    const int radialWrap = std::max(
                        1, int(std::lround(std::max(3, s.radial) *
                                           plan.bandWrapFrac)));
                    const int nuO = plan.orthogonalDriverU > 0
                        ? std::max(solvedEdge[plan.orthogonalDriverU],
                                   density.countFor(plan.orthogonalDriverU,
                                                    radialWrap))
                        : radialWrap;
                    const int nvO = plan.orthogonalDriverV > 0
                        ? std::max(solvedEdge[plan.orthogonalDriverV],
                                   density.countFor(plan.orthogonalDriverV,
                                                    std::max(1, s.axial)))
                        : std::max(1, s.axial);
                    counts[fid] = {std::max(1, nuO), std::max(1, nvO), 0};
                    break;
                }
                if (plan.castellated) {
                    // Full-wrap castellated rim: the plain rim drives the
                    // column count (the notch is cut, never counted); the
                    // castellated chain's arcs weld through a strip.
                    const int nuP = std::max(
                        solvedEdge[plan.plainRimEdge],
                        density.countFor(plan.plainRimEdge, s.radial));
                    counts[fid] = {std::max(3, nuP), std::max(1, s.axial), 0};
                    break;
                }
                if (!plan.bandSides.empty()) {
                    // Open band: nu follows the flat full-span rim's
                    // solved count (which follows radial/adaptive/
                    // density like any revolution rim) — never the
                    // castellated chain's total. uEdges[0] would be
                    // the castellated chain's first arc.
                    const int radialWrap = std::max(
                        3, int(std::lround(std::max(3, s.radial) *
                                           plan.bandWrapFrac)));
                    int nuB = radialWrap;
                    if (plan.bandDriver > 0) {
                        nuB = std::max(
                            solvedEdge[plan.bandDriver],
                            density.countFor(plan.bandDriver, radialWrap));
                    }
                    counts[fid] = {nuB, solved(plan.vEdges, s.axial), 0};
                    break;
                }
                int nuA = solved(plan.uEdges, s.radial);
                int nuB = nuA;
                if (!plan.linkRims && plan.uEdges.size() == 2) {
                    nuB = std::max(
                        solvedEdge[plan.uEdges[1]],
                        density.countFor(plan.uEdges[1], s.radial));
                }
                counts[fid] = {nuA, solved(plan.vEdges, s.axial), nuB};
                break;
            }
            case MesherKind::PlanarGrid:
            case MesherKind::CoonsGrid: {
                if (plan.orthogonalTrimGrid) {
                    counts[fid] = {
                        plan.orthogonalDriverU > 0
                            ? std::max(solvedEdge[plan.orthogonalDriverU],
                                       density.countFor(
                                           plan.orthogonalDriverU,
                                           std::max(1, s.gridU)))
                            : std::max(1, s.gridU),
                        plan.orthogonalDriverV > 0
                            ? std::max(solvedEdge[plan.orthogonalDriverV],
                                       density.countFor(
                                           plan.orthogonalDriverV,
                                           std::max(1, s.gridV)))
                            : std::max(1, s.gridV),
                        0};
                    break;
                }
                int defU = plan.isFillet && plan.acrossIsU ? s.filletLoops
                                                           : s.gridU;
                int defV = plan.isFillet && !plan.acrossIsU ? s.filletLoops
                                                            : s.gridV;
                counts[fid] = {plan.constrains ? solved(plan.uEdges, defU)
                                               : std::max(1, defU),
                               plan.constrains ? solved(plan.vEdges, defV)
                                               : std::max(1, defV),
                               0};
                // Chained Coons meshes from per-edge counts: fold them
                // into the cache key so density edits regenerate.
                int chainHash = 0;
                for (int sd = 0; sd < 4; ++sd) {
                    for (int e : plan.coonsSides[sd]) {
                        chainHash = chainHash * 31 +
                                    density.countFor(e, 1) * (sd + 1);
                    }
                }
                if (chainHash) counts[fid][2] = chainHash;
                break;
            }
            case MesherKind::MinimalNGon:
            case MesherKind::RingJunction:
                counts[fid] = {solved(plan.uEdges, s.gridU),
                               solved(plan.vEdges, s.gridV), 0};
                break;
            case MesherKind::AnnulusRing:
                counts[fid] = {solved(plan.uEdges, s.radial),
                               solved(plan.vEdges, s.radial), 0};
                break;
            case MesherKind::DomeCap: {
                // Meridian count = total base-loop samples (the base rim's
                // edges solve like a revolution rim); latitude ring count =
                // the polar meridian's solved count (axial + curvature),
                // floored so a squat dome still reads as a hemisphere.
                int nAz = 0;
                for (int e : plan.uEdges) {
                    if (e >= 1 && e < int(solvedEdge.size())) nAz += solvedEdge[e];
                }
                const int nLat = plan.vEdges.empty()
                                     ? std::max(2, s.axial)
                                     : std::max(2, solved(plan.vEdges, s.axial));
                counts[fid] = {std::max(3, nAz), nLat, 0};
                break;
            }
            default:
                break;
        }
    }

    // The "solved nu/nv per face" report line moved AFTER meshing (below the
    // parallel mesh loop): several meshers only settle their true built count
    // at mesh time (the annulus body's dense-rim columns, the boundary/rail
    // meshers' totals), so reporting the pre-mesh `counts` table here would
    // show the sparse/zero placeholder instead of what was actually built.

    // Per-face pathology guard (see FaceMeshSettings::cellCap). A face's
    // cell count should track its surface area; a face carrying far more
    // cells than its area-share of the model is a sizing pathology, not
    // detail. Cap total cells at a generous multiple of the face's fair
    // area-share of a reference global budget, scaled by the density dial
    // (mesh cell area ~ 1/density^2, so the cell budget scales as
    // density^2). Floored so a small high-curvature face keeps its detail
    // and never trips the guard; the guard only ever bites gross outliers.
    std::vector<int> faceCellCap(faceN + 1, 0);
    {
        // Reference whole-model cell budget at density 1.0: a face's fair
        // ceiling is its area-share of this, so only a genuinely large
        // face is allowed a large grid. Deliberately generous (~2x a
        // Plasticity-parity model's total) — it is a pathology guard, not
        // a density target. Cells scale with 1/spacing^2, so the budget
        // scales with density^2. Floored so a small high-curvature face
        // keeps its detail and never trips the guard; big faces scale past
        // the floor by area. Both bounds verified to leave every fixture
        // and board face byte-identical (guard bites only gross outliers).
        const double kBudget = 26000.0;
        const double kFloor = 1200.0;
        const double dsc = std::clamp(settings.densityScale, 0.05, 20.0);
        std::vector<double> faceArea(faceN + 1, 0.0);
        double modelArea = 0.0;
        const bool cachedAreas =
            cache && cache->modelArea >= 0.0 &&
            int(cache->faceAreas.size()) == faceN;
        if (cachedAreas) {
            modelArea = cache->modelArea;
            for (int fid = 1; fid <= faceN; ++fid) {
                faceArea[fid] = cache->faceAreas.at(fid);
            }
        } else {
            for (int fid = 1; fid <= faceN; ++fid) {
                try {
                    GProp_GProps gp;
                    BRepGProp::SurfaceProperties(
                        TopoDS::Face(model.faces(fid)), gp);
                    faceArea[fid] = std::max(0.0, gp.Mass());
                } catch (const Standard_Failure&) {
                }
                modelArea += faceArea[fid];
            }
            if (cache) {
                cache->faceAreas.clear();
                for (int fid = 1; fid <= faceN; ++fid) {
                    cache->faceAreas[fid] = faceArea[fid];
                }
                cache->modelArea = modelArea;
            }
        }
        if (modelArea > 1e-12) {
            for (int fid = 1; fid <= faceN; ++fid) {
                const double frac = faceArea[fid] / modelArea;
                const double cap = kBudget * dsc * dsc * frac;
                faceCellCap[fid] =
                    (int)std::clamp(cap, kFloor, 1.0e8);
            }
        }
    }

    // Cache keys: everything that shapes a face's part. A hit skips the
    // (expensive) meshing entirely and reuses the stored part.
    std::vector<std::string> cacheKey(faceN + 1);
    std::vector<bool> cached(faceN + 1, false);
    for (int fid = 1; fid <= faceN; ++fid) {
        const FaceMeshSettings& s = settings.forFace(fid);
        const FacePlan& plan = plans.at(fid);
        char key[352];
        std::snprintf(
            key, sizeof key,
            "k%d c%d f%d a%d l%d q%d|%d,%d,%d|r%d x%d u%d v%d cap%d ch%.6g "
            "an%.6g fl%d fh%.6g jr%d qd%d mn%d ex%d ms%.6g rd%d sq%d cr%d "
            "ds%.4g pt%d wt%.6g ce%d",
            int(plan.kind), plan.constrains ? 1 : 0, plan.isFillet ? 1 : 0,
            plan.acrossIsU ? 1 : 0, plan.linkRims ? 1 : 0,
            plan.forceFallbackQuads, counts[fid][0], counts[fid][1],
            counts[fid][2], s.radial, s.axial, s.gridU, s.gridV, int(s.cap),
            s.chordTolerance, s.angleToleranceDeg, s.filletLoops,
            s.filletHold, s.junctionRings, s.quadDominant ? 1 : 0,
            s.minimal ? 1 : 0, s.exclude ? 1 : 0, s.minSize,
            s.relativeDeviation ? 1 : 0, s.squareCollar ? 1 : 0,
            plan.coonsRotate, settings.densityScale,
            s.pureTriFloor ? 1 : 0,
            s.weldTolerance,
            enableCanonicalEdgeContracts ? 1 : 0);
        cacheKey[fid] = key;
        if (plan.kind == MesherKind::AnnulusRing ||
            plan.kind == MesherKind::RailLadder ||
            plan.kind == MesherKind::DomeCap ||
            !plan.bandSides.empty() || !plan.loops.empty() ||
            plan.castellated) {
            // A castellated rim consumes the plain rim's count (columns)
            // AND every cut-chain edge's count (arc/wall/floor samples),
            // all of which live in uEdges — key them so a density edit
            // regenerates the part against re-meshed neighbours.
            for (int eid : plan.uEdges) {
                cacheKey[fid] += "u" + std::to_string(solvedEdge[eid]);
            }
            for (int eid : plan.vEdges) {
                cacheKey[fid] += "v" + std::to_string(solvedEdge[eid]);
            }
            // C-ring walls carry the across-ring row count.
            for (int eid : plan.cWalls) {
                cacheKey[fid] += "w" + std::to_string(solvedEdge[eid]);
            }
        }
        // Every solved count the part consumes must key the cache, or a
        // density edit on a slot border / chained side / corner stub
        // reuses a stale part against a re-meshed neighbour.
        for (const auto& wire : plan.insertWires) {
            for (int eid : wire) {
                cacheKey[fid] += "w" + std::to_string(solvedEdge[eid]);
            }
        }
        for (int side = 0; side < 4; ++side) {
            for (int eid : plan.coonsSides[side]) {
                cacheKey[fid] += "c" + std::to_string(solvedEdge[eid]);
            }
        }
        // Key EVERY border edge's solved count, for every kind: any
        // mesher that walks its wires (minimal n-gons, plate webs, the
        // floors) consumes counts that live in no plan list, and a
        // density edit that reaches such an edge through group
        // propagation must re-mesh the face — a stale part against a
        // re-meshed neighbour is an open seam (observed: a mohne radial
        // edit under the sweep's warm cache leaked exactly the edited
        // count per side).
        for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE); ex.More();
             ex.Next()) {
            int eid = model.edges.FindIndex(ex.Current());
            if (eid >= 1 && eid < int(solvedEdge.size())) {
                cacheKey[fid] += "e" + std::to_string(solvedEdge[eid]);
            }
        }
    }

    // Mesh every face into its own part, in parallel, then merge in face
    // order so the output is deterministic (identical to the serial order).
    std::vector<PolyMesh> parts(faceN + 1);
    // Faces whose planned mesher couldn't build: the part is a fallback
    // triangulation, and the PLAN must follow (conform treats structured
    // meshers as exact-border authorities — a fallback part isn't one).
    std::vector<char> fellBack(faceN + 1, 0);
    int cacheHits = 0;
    if (cache) {
        for (int fid = 1; fid <= faceN; ++fid) {
            auto it = cache->faces.find(fid);
            if (it != cache->faces.end() &&
                it->second.key == cacheKey[fid]) {
                parts[fid] = it->second.part;  // copy: merge mutates
                fellBack[fid] = it->second.fellBack;
                builtCounts[fid] = it->second.builtCounts;
                cached[fid] = true;
                ++cacheHits;
            }
        }
    }
    const int cacheMisses = faceN - cacheHits;
    timingCheckpoint("counts + cache keys");
    if (settings.progressTotal) {
        settings.progressTotal->store(cacheMisses, std::memory_order_relaxed);
    }
    // Border-contract oracle: does this part contain every border edge
    // of the face at its solved sampling (each consecutive pair of
    // 3D-curve samples present as a polygon edge)? Returns the first
    // offending edge id, 0 when clean. Seams (edges appearing twice in
    // the face's wires), degenerate and micro edges are exempt.
    auto borderContractViolation = [&](int fid,
                                       const PolyMesh& part) -> int {
        const TopoDS_Face F = TopoDS::Face(model.faces(fid));
        // Fixed quantum (the historical weld floor): the contract oracle
        // decides which mesher/fallback a face gets, so it must NOT move
        // with the user's weld knob — otherwise raising the global weld
        // silently re-plans faces. Welding proper happens post-plan.
        const double q = 1e-6;
        std::map<std::tuple<long long, long long, long long>,
                 std::vector<uint32_t>>
            cells;
        for (uint32_t vi = 0; vi < part.vertices.size(); ++vi) {
            const auto& P = part.vertices[vi];
            cells[{llround(P[0] / q), llround(P[1] / q),
                   llround(P[2] / q)}]
                .push_back(vi);
        }
        // ALL vertices coinciding with a sample: pre-weld parts hold
        // duplicate corner ids (one per side/strip), and the polygon
        // edge may hang off any of them.
        auto nearVerts = [&](const gp_Pnt& p) {
            std::vector<uint32_t> hits;
            const long long cx = llround(p.X() / q),
                            cy = llround(p.Y() / q),
                            cz = llround(p.Z() / q);
            for (long long dx = -1; dx <= 1; ++dx) {
                for (long long dy = -1; dy <= 1; ++dy) {
                    for (long long dz = -1; dz <= 1; ++dz) {
                        auto it = cells.find({cx + dx, cy + dy, cz + dz});
                        if (it == cells.end()) continue;
                        for (uint32_t vi : it->second) {
                            const auto& P = part.vertices[vi];
                            const double ddx = P[0] - p.X();
                            const double ddy = P[1] - p.Y();
                            const double ddz = P[2] - p.Z();
                            if (ddx * ddx + ddy * ddy + ddz * ddz <
                                q * q) {
                                hits.push_back(vi);
                            }
                        }
                    }
                }
            }
            return hits;
        };
        std::set<uint64_t> partEdges;
        for (const auto& poly : part.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                const uint32_t a = poly[i];
                const uint32_t b = poly[(i + 1) % poly.size()];
                partEdges.insert((uint64_t(std::min(a, b)) << 32) |
                                 std::max(a, b));
            }
        }
        std::map<int, int> occur;  // seams appear twice in the wires
        for (TopExp_Explorer ex(F, TopAbs_EDGE); ex.More(); ex.Next()) {
            int eid = model.edges.FindIndex(ex.Current());
            if (eid >= 1) ++occur[eid];
        }
        for (const auto& [eid, cnt] : occur) {
            if (cnt != 1) continue;  // seam: internal to this face
            const TopoDS_Edge E = TopoDS::Edge(model.edges(eid));
            if (BRep_Tool::Degenerated(E)) continue;
            const int n = eid < int(solvedEdge.size()) ? solvedEdge[eid]
                                                       : 0;
            if (n < 1) continue;
            double f, l;
            Handle(Geom_Curve) c3 = BRep_Tool::Curve(E, f, l);
            if (c3.IsNull()) continue;
            if (n == 1 &&
                c3->Value(f).Distance(c3->Value(l)) < 4.0 * q) {
                continue;  // micro edge: below weld resolution
            }
            // Corner gaps on sloppy CAD reach the EDGE tolerance (1e-4
            // and worse) and are healed later by corner
            // canonicalization — endpoint samples get that tolerance,
            // interior samples stay at weld exactness.
            // Recorded tolerances LIE on sloppy exports (observed: a
            // 1.4e-4 corner gap on an edge claiming 1e-6). Bound the
            // endpoint radius by the local sample spacing instead —
            // 40% of a step can never capture the wrong border sample.
            const double eTol = std::max(
                {q, BRep_Tool::Tolerance(E),
                 0.4 * c3->Value(f).Distance(c3->Value(l)) /
                     double(std::max(1, n))});
            auto nearVertsEnd = [&](const gp_Pnt& p) {
                std::vector<uint32_t> hits = nearVerts(p);
                if (!hits.empty() || eTol <= q) return hits;
                for (uint32_t vi = 0; vi < part.vertices.size(); ++vi) {
                    const auto& P = part.vertices[vi];
                    const double dx = P[0] - p.X(), dy = P[1] - p.Y(),
                                 dz = P[2] - p.Z();
                    if (dx * dx + dy * dy + dz * dz < eTol * eTol) {
                        hits.push_back(vi);
                    }
                }
                return hits;
            };
            // Read the already-frozen 3D boundary contract.  The spatial
            // lookup below remains a temporary compatibility oracle for face
            // builders that do not record sample ids yet, but it no longer
            // invents a third edge discretisation.
            std::vector<gp_Pnt> expected;
            const CanonicalEdgePlan* contract =
                pinnedEdge.canonical ? pinnedEdge.canonical->find(eid)
                                     : nullptr;
            if (contract) {
                expected.reserve(contract->samples.size());
                for (const CanonicalEdgeSample& sample : contract->samples) {
                    if (!sample.valid) {
                        expected.clear();
                        break;
                    }
                    expected.emplace_back(sample.position[0],
                                          sample.position[1],
                                          sample.position[2]);
                }
            }
            if (expected.empty()) {
                const double ph = closedEdgePhase(E, model);
                const std::vector<double> frac = edgeSampleFractions(
                    eid, n, ph, false, /*includeLast=*/true, &pinnedEdge,
                    &model);
                expected.reserve(frac.size());
                for (double t : frac) {
                    expected.push_back(c3->Value(f + (l - f) * t));
                }
            }
            const int m = int(expected.size()) - 1;
            if (m < 1) continue;
            auto sampleAt = [&](int i) {
                return expected[i];
            };
            std::vector<uint32_t> prev = nearVertsEnd(sampleAt(0));
            for (int i = 1; i <= m; ++i) {
                std::vector<uint32_t> cur = i == m
                                                ? nearVertsEnd(sampleAt(m))
                                                : nearVerts(sampleAt(i));
                bool linked = false;
                for (uint32_t a : prev) {
                    for (uint32_t b : cur) {
                        if (a != b &&
                            partEdges.count(
                                (uint64_t(std::min(a, b)) << 32) |
                                std::max(a, b))) {
                            linked = true;
                            break;
                        }
                    }
                    if (linked) break;
                }
                if (!linked) {
                    const gp_Pnt sp = sampleAt(i);
                    double bn = 1e300;
                    for (uint32_t vi = 0; vi < part.vertices.size(); ++vi) {
                        const auto& P = part.vertices[vi];
                        const double dx = P[0] - sp.X(), dy = P[1] - sp.Y(),
                                     dz = P[2] - sp.Z();
                        bn = std::min(bn, dx * dx + dy * dy + dz * dz);
                    }
                    dbg("contract check %d: edge %d sample %d/%d: %s "
                        "(nearest %.3g, eTol %.3g)",
                        fid, eid, i, m,
                        prev.empty() || cur.empty() ? "no vertex" : "no edge",
                        std::sqrt(bn), eTol);
                    return eid;
                }
                prev = std::move(cur);
            }
        }
        return 0;
    };

    // Repair isolated folded floor cells without imposing a replacement
    // lattice: insert one surface-anchored centre and fan only that polygon.
    // Borders and all neighbouring polygons remain untouched. Returns true
    // only when the resulting part has no inverted polygons.
    auto repairFloorFolds = [&](int fid, const TopoDS_Face& face,
                                PolyMesh& part) {
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        if (surface.IsNull()) return false;
        BRepAdaptor_Surface sa(face);
        const double up = sa.IsUPeriodic() ? sa.UPeriod() : 0.0;
        const double vp = sa.IsVPeriodic() ? sa.VPeriod() : 0.0;
        for (int pass = 0; pass < 3; ++pass) {
            const auto folded = foldedPolys(model, part);
            const int before = int(std::count(folded.begin(), folded.end(),
                                              uint8_t{1}));
            if (before == 0) return true;
            PolyMesh cand = part;
            bool changed = false;
            const size_t originalPolys = part.polygons.size();
            for (size_t pi = 0; pi < originalPolys; ++pi) {
                if (pi >= folded.size() || !folded[pi]) continue;
                const auto poly = part.polygons[pi];
                if (poly.size() < 3) continue;
                double cu = 0.0, cv = 0.0, uref = 0.0, vref = 0.0;
                bool anchored = true;
                for (size_t k = 0; k < poly.size(); ++k) {
                    const uint32_t vi = poly[k];
                    if (vi >= part.anchors.size() ||
                        part.anchors[vi].faceId != fid) {
                        anchored = false;
                        break;
                    }
                    double u = part.anchors[vi].u;
                    double v = part.anchors[vi].v;
                    if (k == 0) {
                        uref = u;
                        vref = v;
                    } else {
                        if (up > 0) u -= up * std::round((u - uref) / up);
                        if (vp > 0) v -= vp * std::round((v - vref) / vp);
                    }
                    cu += u;
                    cv += v;
                }
                if (!anchored) continue;
                cu /= poly.size();
                cv /= poly.size();
                const gp_Pnt cp = surface->Value(cu, cv);
                const uint32_t ci = uint32_t(cand.vertices.size());
                cand.vertices.push_back({cp.X(), cp.Y(), cp.Z()});
                cand.anchors.push_back({fid, cu, cv});
                const int pf = pi < cand.polygonFaceId.size()
                                   ? cand.polygonFaceId[pi]
                                   : fid;
                cand.polygons[pi] = {poly[0], poly[1], ci};
                for (size_t k = 1; k < poly.size(); ++k) {
                    cand.polygons.push_back(
                        {poly[k], poly[(k + 1) % poly.size()], ci});
                    cand.polygonFaceId.push_back(pf);
                }
                if (pi < cand.polygonFaceId.size()) {
                    cand.polygonFaceId[pi] = pf;
                }
                changed = true;
            }
            if (!changed) return false;
            const auto afterMask = foldedPolys(model, cand);
            const int after = int(std::count(afterMask.begin(),
                                             afterMask.end(), uint8_t{1}));
            if (after >= before) return false;
            part = std::move(cand);
            dbg("mesh face %d: floor fold repair %d -> %d", fid, before,
                after);
        }
        const auto folded = foldedPolys(model, part);
        return std::none_of(folded.begin(), folded.end(),
                            [](uint8_t v) { return v != 0; });
    };

    // One demotion path for every mesher failure: the contract floor
    // first (exact borders, cannot leak), verified; the raw OCCT
    // triangulation only when even that is unavailable.
    auto demote = [&](int fid, const TopoDS_Face& face,
                      const BRepAdaptor_Surface& surf,
                      const FaceMeshSettings& s, const char* why) {
        parts[fid] = PolyMesh();
        fellBack[fid] = 1;
        // Density settings reach demoted faces too: the floor's interior
        // refinement and the OCCT retry both honor the (budget-scaled)
        // deviation, so a failed mesher doesn't freeze the face's detail.
        FaceMeshSettings fsD = s;
        // A failed structured strategy has already proved that imposed flow
        // is unsafe. Keep its emergency floor as honest local triangles;
        // only an explicit quad-dominant request may pair them.
        if (!s.quadDominant) fsD.pureTriFloor = true;
        const double dscD = std::clamp(settings.densityScale, 0.05, 20.0);
        if (dscD != 1.0) {
            fsD.chordTolerance /= dscD * dscD;
            fsD.angleToleranceDeg =
                std::clamp(fsD.angleToleranceDeg / dscD, 1.0, 60.0);
        }
        // An explicit (tightened) angle tolerance makes the floor web's
        // interior split on facet-turn angle too, not just chord sag.
        const bool angleSplitD =
            settings.perFace.count(fid) &&
            s.angleToleranceDeg != settings.defaults.angleToleranceDeg;
        {
            MeshBuilder retry(parts[fid]);
            const bool built = meshContractFallback(face, model, fid,
                                                    solvedEdge, s.radial,
                                                    retry, &fsD, angleSplitD,
                                                    &pinnedEdge);
            const int floorBad =
                built ? borderContractViolation(fid, parts[fid]) : -1;
            int floorFolds = 0;
            if (built && floorBad == 0) {
                if (!repairFloorFolds(fid, face, parts[fid])) {
                    const auto folded = foldedPolys(model, parts[fid]);
                    floorFolds = int(std::count(folded.begin(), folded.end(),
                                                uint8_t{1}));
                }
            }
            if (built && floorBad == 0 && floorFolds == 0) {
                // Verified floor: borders are exact at the solved
                // counts, so conform must treat them as authority,
                // not as freeform movers to kidnap.
                fellBack[fid] = 2;
                dbg("mesh face %d: %s -> contract floor", fid, why);
                return;
            }
            dbg("mesh face %d: floor %s (edge %d, folds %d)", fid,
                built ? "rejected" : "failed to build", floorBad,
                floorFolds);
        }
        parts[fid] = PolyMesh();
        MeshBuilder retry(parts[fid]);
        meshFallback(face, surf, fid, fsD, retry);
        repairFloorFolds(fid, face, parts[fid]);
        dbg("mesh face %d: %s -> OCCT fallback (no contract floor)", fid,
            why);
    };

    auto meshFace = [&](int fid) {
        // Copy (not ref): inject this face's pathology-guard cell ceiling,
        // computed from area vs. the model above. Downstream qs/fsD copies
        // inherit it, so every interior densifier sees the same budget.
        FaceMeshSettings s = settings.forFace(fid);
        s.cellCap = faceCellCap[fid];
        if (s.exclude) return;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        const FacePlan& plan = plans.at(fid);
        dbg("mesh face %d: %s", fid, mesherKindName(plan.kind));
        BRepAdaptor_Surface surf(face);
        MeshBuilder out(parts[fid]);
        const int nu = counts[fid][0], nv = counts[fid][1];
        // An explicit per-face gridU/gridV (a count DIFFERING from the model
        // default) drives the quad-fill interior grid directly; 0 means "not
        // set" so the geometry/border spacing stands and default output is
        // unchanged.
        const bool ovFace = settings.perFace.count(fid) > 0;
        const int quadGridU =
            ovFace && s.gridU != settings.defaults.gridU ? s.gridU : 0;
        const int quadGridV =
            ovFace && s.gridV != settings.defaults.gridV ? s.gridV : 0;
        // A tightened angle tolerance drives the floor-web angle split.
        const bool angleSplit =
            ovFace &&
            s.angleToleranceDeg != settings.defaults.angleToleranceDeg;

        // Both rims' phases, each mapped to its own v end; a band with
        // one rim (or none) uses the same phase at both ends.
        auto revPhases = [&]() -> std::pair<double, double> {
            double p0 = surf.FirstUParameter(), p1 = p0;
            bool have0 = false, have1 = false;
            for (size_t k = 0; k < plan.uEdges.size() && k < 2; ++k) {
                bool atV1 = false;
                double ph =
                    revolutionUPhase(surf, model, plan.uEdges[k], &atV1);
                if (atV1 && !have1) { p1 = ph; have1 = true; }
                else if (!atV1 && !have0) { p0 = ph; have0 = true; }
            }
            if (have0 && !have1) p1 = p0;
            if (have1 && !have0) p0 = p1;
            return {p0, p1};
        };
        switch (plan.kind) {
            case MesherKind::RevolutionGrid:
                if (plan.orthogonalTrimGrid) {
                    const bool ok = plan.orthogonalLocalComb
                        ? meshOrthogonalLocalComb(face, surf, model, plan,
                                                  solvedEdge, fid, nu, nv,
                                                  s.cellCap, out, &pinnedEdge)
                        : meshOrthogonalTrimGrid(face, surf, model, plan,
                                                 solvedEdge, fid, nu, nv, out,
                                                 &pinnedEdge);
                    if (!ok) {
                        demote(fid, face, surf, s,
                               "orthogonal revolution grid failed");
                    }
                } else if (plan.castellated && !plan.insertWires.empty()) {
                    // A notched rim AND an interior slot on one wall
                    // (torture's muzzle): the rim-notch mesher cannot
                    // emit interior wires, so it was GUARANTEED to fail
                    // the border contract and land on the floor web —
                    // the patchwork the artist flagged. The insert path
                    // owns interior wires and samples the notched rim
                    // chain like any rim, so try it first; on failure
                    // the same floor catches it, no worse than before.
                    if (!meshRevolutionInsert(face, surf, model, plan,
                                              solvedEdge, fid, nu, nv,
                                              out, &pinnedEdge,
                                              settings.decoupleSeams)) {
                        demote(fid, face, surf, s,
                               "castellated insert failed");
                    }
                } else if (plan.castellated) {
                    // Full-wrap castellated rim: straight uniform lattice
                    // with the notch cut out and webbed.
                    if (!meshRevolutionRimNotch(face, surf, model,
                                                plan.rimLow, plan.rimHigh,
                                                plan.plainRimEdge, solvedEdge,
                                                fid, nu, nv, out,
                                                &pinnedEdge)) {
                        demote(fid, face, surf, s, "rim notch failed");
                    }
                } else if (!plan.insertWires.empty() &&
                           !plan.bandSides.empty()) {
                    // Partial-wrap wall with interior cutouts: straight
                    // columns, covered cells deleted, cutouts webbed as
                    // local collars.
                    if (!meshRevolutionOpenBand(face, surf, model,
                                                plan.uEdges, solvedEdge,
                                                fid, nu, nv,
                                                plan.bandSides, out,
                                                &pinnedEdge,
                                                &plan.insertWires)) {
                        demote(fid, face, surf, s,
                               "open band insert failed");
                    }
                } else if (!plan.insertWires.empty()) {
                    // Before the taper branch: a taper never cuts the
                    // slots out, so insert bands go first regardless of
                    // rim linkage.
                    if (!meshRevolutionInsert(face, surf, model, plan,
                                              solvedEdge, fid, nu, nv,
                                              out, nullptr,
                                              settings.decoupleSeams)) {
                        demote(fid, face, surf, s,
                               "revolution insert failed");
                    }
                } else if (!plan.linkRims && counts[fid][2] > 0 &&
                    counts[fid][2] != nu && !surf.IsVClosed()) {
                    // counts[0] belongs to uEdges[0]; find which v-end that
                    // rim sits at so the taper's rings land on their caps.
                    int nA = nu, nB = counts[fid][2];
                    BRepAdaptor_Curve rim(
                        TopoDS::Edge(model.edges(plan.uEdges[0])));
                    gp_Pnt pm = rim.Value(
                        (rim.FirstParameter() + rim.LastParameter()) / 2);
                    double d0 = 1e300, d1 = 1e300;
                    const double u0 = surf.FirstUParameter();
                    const double du =
                        (surf.LastUParameter() - u0) / 16.0;
                    for (int k = 0; k < 16; ++k) {
                        d0 = std::min(d0, pm.Distance(surf.Value(
                                              u0 + k * du,
                                              surf.FirstVParameter())));
                        d1 = std::min(d1, pm.Distance(surf.Value(
                                              u0 + k * du,
                                              surf.LastVParameter())));
                    }
                    if (d1 < d0) std::swap(nA, nB);
                    auto [p0, p1] = revPhases();
                    meshRevolutionTaper(face, surf, fid, nA, nB, p0, p1,
                                        out);
                } else if (!plan.bandSides.empty()) {
                    if (!meshRevolutionOpenBand(face, surf, model,
                                                plan.uEdges, solvedEdge,
                                                fid, nu, nv,
                                                plan.bandSides, out,
                                                &pinnedEdge)) {
                        demote(fid, face, surf, s, "open band failed");
                    }
                } else {
                    if (!meshRevolutionGrid(
                            face, surf, model, plan.uEdges, solvedEdge, fid,
                            nu, nv, out, nullptr,
                            plan.rimLow.empty() ? nullptr : &plan.rimLow,
                            &builtCounts[fid], false,
                            settings.decoupleSeams)) {
                        // Same floor the border-contract postcondition
                        // used to reach — but explicit, so the relaxed
                        // stitch mode can't ship the empty part as a
                        // hole in the output.
                        demote(fid, face, surf, s, "revolution grid failed");
                    }
                }
                break;
            case MesherKind::DiskCap: {
                double a0 = 0.0;
                if (!plan.uEdges.empty() &&
                    closedEdgePhase(
                        TopoDS::Edge(model.edges(plan.uEdges[0])), model) >
                        0.0) {
                    a0 = ringAnchorAngle(plan.circ);
                }
                meshDiskCap(face, surf, plan.circ, fid, nu, s.cap, out, a0);
                break;
            }
            case MesherKind::PlanarGrid: {
                // Support loops hug the creases on fillet strips.
                double holdU = plan.isFillet && plan.acrossIsU ? s.filletHold : 0;
                double holdV = plan.isFillet && !plan.acrossIsU ? s.filletHold : 0;
                meshParametricGrid(face, surf, fid, clusteredParams(nu, holdU),
                                   clusteredParams(nv, holdV), out);
                break;
            }
            case MesherKind::CoonsGrid: {
                if (plan.trimCorridor || plan.sectionStrip) {
                    builtCounts[fid] = {
                        outerWireSolvedTotal(face, model, solvedEdge,
                                             s.radial),
                        plan.sectionStrip ? plan.sectionStripBands
                                          : plan.trimCorridorBands};
                    if (!meshTrimCorridor(face, model, plan, fid, solvedEdge,
                                          s.radial, out, &pinnedEdge,
                                          &builtCounts[fid])) {
                        demote(fid, face, surf, s,
                               plan.sectionStrip ? "section strip failed"
                                                 : "trim corridor failed");
                    }
                    break;
                }
                if (plan.orthogonalTrimGrid) {
                    const bool ok = plan.orthogonalLocalComb
                        ? meshOrthogonalLocalComb(face, surf, model, plan,
                                                  solvedEdge, fid, nu, nv,
                                                  s.cellCap, out, &pinnedEdge)
                        : meshOrthogonalTrimGrid(face, surf, model, plan,
                                                 solvedEdge, fid, nu, nv, out,
                                                 &pinnedEdge);
                    if (!ok) {
                        demote(fid, face, surf, s,
                               "orthogonal surface grid failed");
                    }
                    break;
                }
                double holdU = plan.isFillet && plan.acrossIsU ? s.filletHold : 0;
                double holdV = plan.isFillet && !plan.acrossIsU ? s.filletHold : 0;
                // One-direction-closed blend rings (the demo strut
                // skirts): the coons patch's transfinite 3D blend folds
                // on these charts and the face drops to the contract
                // floor. The ring lattice evaluates every vertex on the
                // surface directly, so try it first — kept only when it
                // welds exactly AND never folds (Newell vs CAD normal
                // with periodic v unwrap); otherwise the historic coons
                // path runs untouched, byte-for-byte.
                if (plan.insertWires.empty() && surf.IsVClosed() &&
                    !surf.IsUClosed()) {
                    PolyMesh tmp;
                    MeshBuilder rb(tmp);
                    auto latticeFolds = [&](const PolyMesh& pm) {
                        Handle(Geom_Surface) S2 = BRep_Tool::Surface(face);
                        if (S2.IsNull()) return 0;
                        const double vper =
                            surf.IsVPeriodic()
                                ? surf.VPeriod()
                                : surf.LastVParameter() -
                                      surf.FirstVParameter();
                        const bool rev =
                            face.Orientation() == TopAbs_REVERSED;
                        int folds = 0;
                        for (const auto& poly : pm.polygons) {
                            if (poly.size() < 3) continue;
                            gp_XYZ nw(0, 0, 0);
                            double mu = 0, mv = 0, vref = 0;
                            int na = 0;
                            for (size_t i = 0; i < poly.size(); ++i) {
                                const auto& a2 = pm.vertices[poly[i]];
                                const auto& b2 =
                                    pm.vertices[poly[(i + 1) %
                                                     poly.size()]];
                                nw += gp_XYZ(
                                    a2[1] * b2[2] - a2[2] * b2[1],
                                    a2[2] * b2[0] - a2[0] * b2[2],
                                    a2[0] * b2[1] - a2[1] * b2[0]);
                                const Anchor& an = pm.anchors[poly[i]];
                                if (an.faceId == fid) {
                                    double vv = an.v;
                                    if (!na) {
                                        vref = vv;
                                    } else if (vper > 0) {
                                        // Wrap cells: unwrap v against
                                        // the first anchor so the mean
                                        // lands inside the cell.
                                        vv -= vper *
                                              std::round((vv - vref) /
                                                         vper);
                                    }
                                    mu += an.u;
                                    mv += vv;
                                    ++na;
                                }
                            }
                            if (!na || nw.Modulus() < 1e-16) continue;
                            gp_Pnt P2;
                            gp_Vec dU, dV;
                            S2->D1(mu / na, mv / na, P2, dU, dV);
                            gp_XYZ nS = dU.Crossed(dV).XYZ();
                            if (rev) nS.Reverse();
                            if (nS.Modulus() < 1e-16) continue;
                            if (nw.Dot(nS) < 0) ++folds;
                        }
                        return folds;
                    };
                    if (meshRingLattice(face, surf, model, fid, solvedEdge,
                                        &pinnedEdge, rb) &&
                        borderContractViolation(fid, tmp) == 0 &&
                        latticeFolds(tmp) == 0) {
                        parts[fid] = std::move(tmp);
                        dbg("mesh face %d: ring lattice (closed blend ring)",
                            fid);
                        break;
                    }
                    dbg("mesh face %d: ring lattice unusable -> coons", fid);
                }
                // A bent ribbon with an end notch: try the rail sweep's clean
                // notch cut first. Keep it only when it welds EXACTLY (no
                // border-contract violation) — a fine-railed solve. When the
                // solve is too coarse for the pocket walls to weld single-row,
                // the cut can't stand, so fall through to the untouched Coons
                // plan (the historic result, byte-for-byte).
                if (plan.tryRibbonNotch || plan.tryRibbonSweep) {
                    PolyMesh tmp;
                    MeshBuilder rb(tmp);
                    // The sweep must also not FOLD — a folded sweep that
                    // passes the contract would later lose the self-heal
                    // tournament to the FLOOR, which is worse than the
                    // coons this transaction replaces. Newell vs the CAD
                    // normal at the anchors' mean uv (strips are not
                    // periodic charts, so no unwrap needed here).
                    auto sweepFolds = [&]() {
                        Handle(Geom_Surface) S2 = BRep_Tool::Surface(face);
                        if (S2.IsNull()) return 0;
                        const bool rev =
                            face.Orientation() == TopAbs_REVERSED;
                        int folds = 0;
                        for (const auto& poly : tmp.polygons) {
                            if (poly.size() < 3) continue;
                            gp_XYZ nw(0, 0, 0);
                            double mu = 0, mv = 0;
                            int na = 0;
                            for (size_t i = 0; i < poly.size(); ++i) {
                                const auto& a2 = tmp.vertices[poly[i]];
                                const auto& b2 =
                                    tmp.vertices[poly[(i + 1) %
                                                      poly.size()]];
                                nw += gp_XYZ(
                                    a2[1] * b2[2] - a2[2] * b2[1],
                                    a2[2] * b2[0] - a2[0] * b2[2],
                                    a2[0] * b2[1] - a2[1] * b2[0]);
                                const Anchor& an = tmp.anchors[poly[i]];
                                if (an.faceId == fid) {
                                    mu += an.u;
                                    mv += an.v;
                                    ++na;
                                }
                            }
                            if (!na || nw.Modulus() < 1e-16) continue;
                            gp_Pnt P2;
                            gp_Vec dU, dV;
                            S2->D1(mu / na, mv / na, P2, dU, dV);
                            gp_XYZ nS = dU.Crossed(dV).XYZ();
                            if (rev) nS.Reverse();
                            if (nS.Modulus() < 1e-16) continue;
                            if (nw.Dot(nS) < 0) ++folds;
                        }
                        return folds;
                    };
                    // The sweep's border must also be the part's
                    // BOUNDARY: a cap web that re-traverses the strip's
                    // end border passes the contract (the samples appear
                    // as polygon edges — twice) yet welds non-manifold
                    // against the neighbour. Boundary length below the
                    // solved border total betrays it.
                    auto boundaryCovers = [&]() {
                        // Face border polylines, coarsely sampled.
                        std::vector<std::vector<gp_Pnt>> borders;
                        for (TopExp_Explorer ex2(face, TopAbs_EDGE);
                             ex2.More(); ex2.Next()) {
                            const TopoDS_Edge e2 =
                                TopoDS::Edge(ex2.Current());
                            if (BRep_Tool::Degenerated(e2)) continue;
                            double f2, l2;
                            Handle(Geom_Curve) c3 =
                                BRep_Tool::Curve(e2, f2, l2);
                            if (c3.IsNull()) continue;
                            auto& pl = borders.emplace_back();
                            for (int k = 0; k <= 32; ++k) {
                                pl.push_back(c3->Value(
                                    f2 + (l2 - f2) * k / 32.0));
                            }
                        }
                        std::map<std::pair<uint32_t, uint32_t>, int> cnt;
                        for (const auto& poly : tmp.polygons) {
                            for (size_t i = 0; i < poly.size(); ++i) {
                                uint32_t a2 = poly[i];
                                uint32_t b2 =
                                    poly[(i + 1) % poly.size()];
                                if (a2 > b2) std::swap(a2, b2);
                                ++cnt[{a2, b2}];
                            }
                        }
                        for (const auto& [seg, c] : cnt) {
                            if (c < 2) continue;
                            // Multi-use segment ON a border curve =
                            // over-traversal (welds non-manifold).
                            const auto& A2 = tmp.vertices[seg.first];
                            const auto& B2 = tmp.vertices[seg.second];
                            const gp_Pnt mid2((A2[0] + B2[0]) / 2,
                                              (A2[1] + B2[1]) / 2,
                                              (A2[2] + B2[2]) / 2);
                            const double dx2 = A2[0] - B2[0],
                                         dy2 = A2[1] - B2[1],
                                         dz2 = A2[2] - B2[2];
                            const double tol2 =
                                0.1 * std::sqrt(dx2 * dx2 + dy2 * dy2 +
                                                dz2 * dz2);
                            for (const auto& pl : borders) {
                                for (const gp_Pnt& q : pl) {
                                    if (q.Distance(mid2) < tol2) {
                                        return false;
                                    }
                                }
                            }
                        }
                        return true;
                    };
                    if (meshRibbonSweep(face, model, fid, solvedEdge, s.radial,
                                        rb, nullptr, &pinnedEdge) &&
                        borderContractViolation(fid, tmp) == 0 &&
                        sweepFolds() == 0 && boundaryCovers()) {
                        parts[fid] = std::move(tmp);
                        dbg("mesh face %d: rail sweep (over coons)%s", fid,
                            plan.tryRibbonNotch ? " [end-notch]"
                                                : " [chained strip]");
                        break;
                    }
                    dbg("mesh face %d: ribbon cut unweldable -> coons", fid);
                }
                if (!meshCoonsGrid(
                        face, model, fid, clusteredParams(nu, holdU),
                        clusteredParams(nv, holdV), plan.coonsRotate,
                        solvedEdge, out,
                        plan.insertWires.empty() ? nullptr
                                                 : &plan.insertWires,
                        std::max(0, s.junctionRings), &pinnedEdge,
                        s.cellCap, settings.decoupleSeams)) {
                    demote(fid, face, surf, s, "coons failed");
                }
                break;
            }
            case MesherKind::MinimalNGon:
                // Boundary-driven (both the planar-web and UV-n-gon paths): the
                // outer loop's solved sample total is the count the mesh
                // presents (no interior grid). Set before the call so a
                // contract-floor demote reports it too; raw-OCCT resets below.
                builtCounts[fid] = {
                    outerWireSolvedTotal(face, model, solvedEdge, s.radial), 0};
                if (!plan.loops.empty()) {
                    if (!meshMinimalPlanar(face, model, fid, solvedEdge,
                                           s.radial, out, &pinnedEdge)) {
                        demote(fid, face, surf, s, "minimal planar failed");
                    }
                } else {
                    meshMinimalNGon(face, surf, fid, nu, nv, out);
                }
                break;
            case MesherKind::RingJunction: {
                double a0 = 0.0;
                if (plan.circleEdgeId > 0 &&
                    closedEdgePhase(
                        TopoDS::Edge(model.edges(plan.circleEdgeId)),
                        model) > 0.0) {
                    a0 = ringAnchorAngle(plan.circ);
                }
                meshRingJunction(face, surf, plan.circ, fid, nu, nv,
                                 s.junctionRings, out, a0);
                break;
            }
            case MesherKind::DomeCap:
                // A UV hemisphere: latitude rings + straight meridians + a
                // pole fan. On any doubt it falls back to the contract floor
                // (exact borders, watertight by construction), so a dome the
                // walk can't express never leaks.
                if (!meshDomeCap(face, surf, model, plan, solvedEdge, fid, nv,
                                 out, &pinnedEdge)) {
                    demote(fid, face, surf, s, "dome cap failed");
                }
                break;
            case MesherKind::AnnulusRing:
                if (plan.cRing) {
                    if (!meshAnnulusCRing(face, model, fid, plan.uEdges,
                                          plan.vEdges, plan.cWalls, solvedEdge,
                                          s.radial, out, &pinnedEdge)) {
                        demote(fid, face, surf, s, "annulus c-ring failed");
                    }
                } else if (!meshAnnulusRing(face, model, fid, plan.uEdges,
                                            plan.vEdges, solvedEdge, s.radial,
                                            out)) {
                    demote(fid, face, surf, s, "annulus ring failed");
                }
                break;
            case MesherKind::PlateWeb:
                // Boundary-driven around the outer loop (plus collar rings at
                // each hole, a separate knob) — report the outer total. Set
                // before the call so a contract-floor demote (borders still
                // exact at the solved counts) reports it too; a raw-OCCT
                // demote resets it below.
                builtCounts[fid] = {
                    outerWireSolvedTotal(face, model, solvedEdge, s.radial), 0};
                if (!meshPlateWeb(face, surf, model, fid, solvedEdge,
                                  s.radial, s.junctionRings, s.squareCollar,
                                  out)) {
                    demote(fid, face, surf, s, "plate web failed");
                }
                break;
            case MesherKind::RibbonSweep: {
                // The rail sweep, or on any doubt (unequal rails, a fold, a
                // leak) the local exact-border floor. Never invent a global
                // Quad Fill lattice as an emergency substitute.
                if (meshRibbonSweep(face, model, fid, solvedEdge, s.radial,
                                    out, &builtCounts[fid], &pinnedEdge)) {
                    break;
                }
                demote(fid, face, surf, s, "ribbon sweep failed");
                break;
            }
            case MesherKind::QuadFill: {
                // The same budget scaling the fallback path applies, so
                // the density dial reaches quad-fill interiors too.
                FaceMeshSettings qs = s;
                const double qsc =
                    std::clamp(settings.densityScale, 0.05, 20.0);
                if (qsc != 1.0) {
                    qs.chordTolerance /= qsc * qsc;
                    qs.angleToleranceDeg =
                        std::clamp(qs.angleToleranceDeg / qsc, 1.0, 60.0);
                }
                // Boundary-driven: the outer loop's solved sample total is the
                // count the mesh presents (the interior grid tracks it but
                // carries no independent user count). Set before the call so a
                // contract-floor demote reports it too; raw-OCCT resets below.
                builtCounts[fid] = {
                    outerWireSolvedTotal(face, model, solvedEdge, s.radial), 0};
                if (!meshQuadFill(face, surf, model, fid, solvedEdge,
                                  s.radial, qs, out, quadGridU, quadGridV)) {
                    demote(fid, face, surf, s, "quad fill failed");
                } else if (!s.pureTriFloor) {
                    // Lift the CDT rim from a tri fan into quad-dominant
                    // flow: merge adjacent rim triangles into quads. Border
                    // edges are single-tri, so the outline never moves.
                    pairPartTris(parts[fid]);
                }
                break;
            }
            case MesherKind::RailLadder:
                if (!meshRailLadder(face, model, fid, solvedEdge, s.radial,
                                    out, &builtCounts[fid])) {
                    demote(fid, face, surf, s, "rail ladder failed");
                }
                break;
            case MesherKind::QuadDominant:
            case MesherKind::Fallback: {
                // The contract floor first: exact borders by
                // construction, so planned-fallback faces weld seamlessly
                // instead of relying on the conform pass to reconcile
                // OCCT's own discretization (observed: 23 opens on the
                // weldment where conform couldn't). Raw OCCT remains for
                // faces the floor cannot express.
                FaceMeshSettings fs = s;
                if (plan.forceFallbackQuads >= 0) {
                    fs.quadDominant = plan.forceFallbackQuads != 0;
                    fs.pureTriFloor = plan.forceFallbackQuads == 0;
                }
                // The global budget knob reaches triangulations too:
                // deflection error scales with the SQUARE of linear
                // density, angle linearly (already in the cache key).
                const double dsc =
                    std::clamp(settings.densityScale, 0.05, 20.0);
                if (dsc != 1.0) {
                    fs.chordTolerance /= dsc * dsc;
                    fs.angleToleranceDeg =
                        std::clamp(fs.angleToleranceDeg / dsc, 1.0, 60.0);
                }
                // The cap fan is the minimal single-wire answer — valid
                // only while the face stays inside the deviation budget;
                // a curved face takes the floor web, whose interior
                // REFINES to the same budget (that's what makes the
                // deviation / min-size / quad-dominant settings live on
                // floored faces).
                if (faceWithinDeflection(face, fs) &&
                    meshSurfaceCapFan(face, model, fid, solvedEdge,
                                      s.radial, out) &&
                    borderContractViolation(fid, parts[fid]) == 0) {
                    if (repairFloorFolds(fid, face, parts[fid])) {
                        fellBack[fid] = 2;  // exact borders: authority
                        break;
                    }
                }
                {
                    parts[fid] = PolyMesh();
                    MeshBuilder retryFloor(parts[fid]);
                    const bool builtFloor = meshContractFallback(
                        face, model, fid, solvedEdge, s.radial, retryFloor,
                        &fs, angleSplit, &pinnedEdge);
                    const bool exactFloor =
                        builtFloor &&
                        borderContractViolation(fid, parts[fid]) == 0;
                    if (exactFloor &&
                        repairFloorFolds(fid, face, parts[fid])) {
                        fellBack[fid] = 2;  // exact borders: authority
                        break;
                    }
                }
                parts[fid] = PolyMesh();
                MeshBuilder retryFb(parts[fid]);
                meshFallback(face, surf, fid, fs, retryFb);
                repairFloorFolds(fid, face, parts[fid]);
                break;
            }
        }
        // Under decoupled seams the border-contract postcondition below
        // is relaxed — but a mesher that emitted NOTHING is a hole in
        // the output, not a seam-count disagreement: it must still take
        // the floor (nasty_cheese shipped 45 drill walls as holes, 1615
        // open edges, before this net).
        if (settings.decoupleSeams && !fellBack[fid] &&
            plan.kind != MesherKind::Fallback &&
            plan.kind != MesherKind::QuadDominant &&
            parts[fid].polygons.empty()) {
            demote(fid, face, surf, s, "emitted nothing");
        }
        // Safety net: any directed edge repeated inside one face's part is
        // degenerate topology (it would leak non-manifold edges into the
        // weld). Throw the part away and triangulate honestly instead.
        if (plan.kind != MesherKind::Fallback &&
            plan.kind != MesherKind::QuadDominant) {
            std::set<std::pair<uint32_t, uint32_t>> seen;
            std::map<std::pair<uint32_t,uint32_t>, size_t> seenAt;
            bool sane = true;
            uint32_t repeatA = 0, repeatB = 0;
            size_t repeatPoly = 0;
            for (size_t pi = 0; pi < parts[fid].polygons.size(); ++pi) {
                const auto& poly = parts[fid].polygons[pi];
                for (size_t i = 0; i < poly.size() && sane; ++i) {
                    repeatA = poly[i];
                    repeatB = poly[(i + 1) % poly.size()];
                    repeatPoly = pi;
                    const auto directed = std::make_pair(repeatA, repeatB);
                    if (!seen.insert(directed).second) {
                        sane = false;
                    } else {
                        seenAt[directed] = pi;
                    }
                }
                if (!sane) break;
            }
            if (!sane) {
                if (plan.orthogonalTrimGrid) {
                    const Anchor& aa = parts[fid].anchors[repeatA];
                    const Anchor& ab = parts[fid].anchors[repeatB];
                    dbg("orthogonal face %d repeated directed edge %u->%u "
                        "uv (%.8g,%.8g)->(%.8g,%.8g) at polygon %zu "
                        "(first %zu)", fid, repeatA, repeatB, aa.u, aa.v,
                        ab.u, ab.v, repeatPoly, seenAt[{repeatA,repeatB}]);
                }
                demote(fid, face, surf, s, "self-check failed");
            }
        }
        // Border-contract postcondition: any face that cannot prove its
        // borders at the solved sampling demotes to the contract floor
        // VISIBLY — a silent contract break is a guaranteed open seam
        // after the weld. Fallback parts are exempt (they are the
        // floor's floor), as are deliberate clustered fillet holds.
        // Under decoupled seams the contract is no longer the law: faces
        // sample their borders at their own counts and the post-weld
        // splice closes the seams, so a mismatch is not a defect and
        // must not demote (that is what dumped the ribbon sweep onto
        // the floor in the first stitch experiment).
        if (!settings.decoupleSeams && !fellBack[fid] &&
            plan.kind != MesherKind::Fallback &&
            plan.kind != MesherKind::QuadDominant &&
            !plan.orthogonalTrimGrid &&
            !(plan.isFillet && s.filletHold > 0.0)) {
            const int bad = borderContractViolation(fid, parts[fid]);
            if (bad) {
                dbg("mesh face %d: border contract failed on edge %d (%s)",
                    fid, bad, mesherKindName(plan.kind));
                demote(fid, face, surf, s, "border contract failed");
            } else if (plan.sectionStrip) {
                dbg("section strip face %d: exact-border contract passed",
                    fid);
            }
        }
        // Fold postcondition: a part whose polygons largely oppose the
        // CAD normal has folded over itself (warped sliver patches:
        // nasty_cheese faces 46/321 carried 116 inverted cells EACH and
        // still passed every topological check). The floor's UV
        // triangulation cannot fold, so demote visibly. Bounded to
        // small parts — projection per polygon is not free.
        if (!fellBack[fid] && plan.kind != MesherKind::Fallback &&
            plan.kind != MesherKind::QuadDominant &&
            parts[fid].polygons.size() >= 8 &&
            parts[fid].polygons.size() <= 2000) {
            Handle(Geom_Surface) S = BRep_Tool::Surface(face);
            if (!S.IsNull()) {
                GeomAPI_ProjectPointOnSurf proj;
                proj.Init(gp_Pnt(0, 0, 0), S);
                const bool rev = face.Orientation() == TopAbs_REVERSED;
                // Inverted-cell census of ANY candidate part for this
                // face — the planned build and self-heal candidates are
                // judged by the same ruler.
                auto invertedCells = [&](const PolyMesh& part,
                                         std::vector<size_t>* which = nullptr)
                    -> std::pair<int, int> {
                int inverted = 0, tested = 0;
                for (size_t polyIndex = 0;
                     polyIndex < part.polygons.size(); ++polyIndex) {
                    const auto& poly = part.polygons[polyIndex];
                    if (poly.size() < 3) continue;
                    gp_XYZ nw(0, 0, 0), cen(0, 0, 0);
                    for (size_t i = 0; i < poly.size(); ++i) {
                        const auto& a = part.vertices[poly[i]];
                        const auto& b =
                            part.vertices[poly[(i + 1) % poly.size()]];
                        nw += gp_XYZ(a[1] * b[2] - a[2] * b[1],
                                     a[2] * b[0] - a[0] * b[2],
                                     a[0] * b[1] - a[1] * b[0]);
                        cen += gp_XYZ(a[0], a[1], a[2]);
                    }
                    cen /= double(poly.size());
                    if (nw.Modulus() < 1e-16) continue;
                    // Prefer the polygon's own UV provenance: projecting
                    // the 3D centroid is ambiguous on thin tubes (a
                    // fillet torus with a small minor radius — a coarse
                    // cell's centroid sags past the tube centre and
                    // projects onto the FAR side, flipping the reference
                    // normal and flagging perfectly good cells).
                    double pu = 0, pv = 0;
                    int anchored = 0;
                    double u0ref = 0, v0ref = 0;
                    // Offset surfaces LIE about u-periodicity the same
                    // way they lie about closure (IsUPeriodic()=false
                    // across a genuine full turn) — without the unwrap,
                    // every u-seam cell of a rerouted revolution wall
                    // averages across the wrap, reads inverted, and the
                    // self-heal trades clean columns for a floor web.
                    // A closed-revolution plan's own u-range IS the
                    // period.
                    const double uPeriod =
                        surf.IsUPeriodic()
                            ? surf.UPeriod()
                            : (plan.kind == MesherKind::RevolutionGrid &&
                                       (isClosedRevolution(surf) ||
                                        isGeometricClosedRevolution(surf))
                                   ? surf.LastUParameter() -
                                         surf.FirstUParameter()
                                   : 0.0);
                    // v unwraps too: on a doubly periodic surface (a
                    // full torus) the v-seam cells otherwise average
                    // across the wrap and read as inverted — false
                    // positives that made the self-heal tournament trade
                    // a perfect quad torus for a floor web.
                    const double vPeriod =
                        surf.IsVPeriodic() ? surf.VPeriod() : 0.0;
                    for (uint32_t vi : poly) {
                        const Anchor& an = part.anchors[vi];
                        if (an.faceId != fid) continue;
                        double au = an.u;
                        double av = an.v;
                        if (anchored == 0) {
                            u0ref = au;
                            v0ref = av;
                        } else {
                            // Unwrap seam-adjacent params onto the first
                            // vertex's branch.
                            if (uPeriod > 0) {
                                au -= uPeriod *
                                      std::round((au - u0ref) / uPeriod);
                            }
                            if (vPeriod > 0) {
                                av -= vPeriod *
                                      std::round((av - v0ref) / vPeriod);
                            }
                        }
                        pu += au;
                        pv += av;
                        ++anchored;
                    }
                    if (anchored == int(poly.size())) {
                        pu /= anchored;
                        pv /= anchored;
                    } else {
                        proj.Perform(gp_Pnt(cen));
                        if (!proj.IsDone() || proj.NbPoints() < 1) continue;
                        proj.LowerDistanceParameters(pu, pv);
                    }
                    gp_Pnt sp;
                    gp_Vec du, dv;
                    S->D1(pu, pv, sp, du, dv);
                    gp_Vec n = du.Crossed(dv);
                    if (n.Magnitude() < 1e-16) continue;
                    if (rev) n.Reverse();
                    ++tested;
                    if (gp_Vec(nw).Dot(n) < 0) {
                        ++inverted;
                        if (which) which->push_back(polyIndex);
                        if (std::getenv("WEFT_FOLD_DEBUG")) {
                            dbg("fold f%d: %zu-gon uv=(%.4f,%.4f) "
                                "cen=(%.3f,%.3f,%.3f)",
                                fid, poly.size(), pu, pv, cen.X(), cen.Y(),
                                cen.Z());
                        }
                    }
                }
                return {tested, inverted};
                };
                const auto [tested, inverted] = invertedCells(parts[fid]);
                if (tested >= 8 && inverted * 4 > tested) {
                    dbg("mesh face %d: fold check failed (%d/%d inverted, "
                        "%s)",
                        fid, inverted, tested, mesherKindName(plan.kind));
                    demote(fid, face, surf, s, "fold check failed");
                } else if (inverted > 0 && tested >= 8) {
                    // Self-heal tournament: a FEW folded cells sit below
                    // the demote threshold, ship broken, and stay broken
                    // — the artist's report. Build the contract floor as
                    // a CANDIDATE at the same exact borders (identical
                    // welds), and keep whichever part folds less. Only a
                    // strictly better candidate swaps in, so this can
                    // never regress a face.
                    int liveFolds = inverted;
                    // A folding closed revolution band gets a structured
                    // candidate FIRST: rebuild with the drive rim's
                    // iso-azimuth runs collapsed to single columns (the
                    // notch walls strip-bridged instead of welded 1:1 —
                    // foam face 183's bite). Kept only when strictly
                    // fewer folds at an exact contract, judged by the
                    // same ruler as the floor below; the floor then only
                    // wins if it beats THIS too. Structured columns beat
                    // a web whenever both are clean.
                    if (plan.kind == MesherKind::RevolutionGrid &&
                        plan.bandSides.empty() && !plan.castellated) {
                        PolyMesh cand2;
                        MeshBuilder cb2(cand2);
                        std::array<int, 2> bc2 = {-1, -1};
                        if (meshRevolutionGrid(
                                face, surf, model, plan.uEdges, solvedEdge,
                                fid, nu, nv, cb2, nullptr,
                                plan.rimLow.empty() ? nullptr
                                                    : &plan.rimLow,
                                &bc2, /*dedupeDriveRim=*/true,
                                settings.decoupleSeams) &&
                            borderContractViolation(fid, cand2) == 0) {
                            const auto [t2, i2] = invertedCells(cand2);
                            (void)t2;
                            if (i2 < liveFolds) {
                                dbg("mesh face %d: self-heal — %d folded "
                                    "-> %d with drive-rim dedupe",
                                    fid, liveFolds, i2);
                                parts[fid] = std::move(cand2);
                                builtCounts[fid] = bc2;
                                liveFolds = i2;
                            }
                        }
                    }
                    if (liveFolds > 0) {
                        // A single warped Coons quad should not replace the
                        // entire otherwise-clean grid with a dense fallback
                        // web. Try both diagonals and a surface-anchored
                        // centre fan locally, then keep the
                        // strictly cleaner exact-border candidate. This is
                        // the common pointed B-spline-tip case (MP9 face
                        // 914): the clean cells remain quads and only the
                        // saddle becomes a small triangle fan instead of
                        // hundreds of fallback polygons.
                        if (plan.kind == MesherKind::CoonsGrid ||
                            plan.kind == MesherKind::QuadFill) {
                            std::vector<size_t> foldedPolys;
                            invertedCells(parts[fid], &foldedPolys);
                            const PolyMesh foldBase = parts[fid];
                            for (int mode = 0; mode < 3; ++mode) {
                                PolyMesh cand = foldBase;
                                bool splitAny = false;
                                for (size_t pi : foldedPolys) {
                                    if (pi >= cand.polygons.size() ||
                                        cand.polygons[pi].size() != 4) {
                                        continue;
                                    }
                                    const auto q = cand.polygons[pi];
                                    const int pf =
                                        pi < cand.polygonFaceId.size()
                                            ? cand.polygonFaceId[pi]
                                            : fid;
                                    if (mode == 0) {
                                        cand.polygons[pi] = {q[0], q[1], q[2]};
                                        cand.polygons.push_back(
                                            {q[0], q[2], q[3]});
                                        cand.polygonFaceId.push_back(pf);
                                    } else if (mode == 1) {
                                        cand.polygons[pi] = {q[0], q[1], q[3]};
                                        cand.polygons.push_back(
                                            {q[1], q[2], q[3]});
                                        cand.polygonFaceId.push_back(pf);
                                    } else {
                                        double cu = 0.0, cv = 0.0;
                                        int anchored = 0;
                                        for (uint32_t qi : q) {
                                            if (qi >= cand.anchors.size() ||
                                                cand.anchors[qi].faceId != fid) {
                                                continue;
                                            }
                                            cu += cand.anchors[qi].u;
                                            cv += cand.anchors[qi].v;
                                            ++anchored;
                                        }
                                        if (anchored != 4) continue;
                                        cu /= 4.0;
                                        cv /= 4.0;
                                        const gp_Pnt cp = S->Value(cu, cv);
                                        const uint32_t ci =
                                            uint32_t(cand.vertices.size());
                                        cand.vertices.push_back(
                                            {cp.X(), cp.Y(), cp.Z()});
                                        cand.anchors.push_back({fid, cu, cv});
                                        cand.polygons[pi] = {q[0], q[1], ci};
                                        cand.polygons.push_back(
                                            {q[1], q[2], ci});
                                        cand.polygons.push_back(
                                            {q[2], q[3], ci});
                                        cand.polygons.push_back(
                                            {q[3], q[0], ci});
                                        cand.polygonFaceId.push_back(pf);
                                        cand.polygonFaceId.push_back(pf);
                                        cand.polygonFaceId.push_back(pf);
                                    }
                                    if (pi < cand.polygonFaceId.size()) {
                                        cand.polygonFaceId[pi] = pf;
                                    }
                                    splitAny = true;
                                }
                                if (!splitAny ||
                                    borderContractViolation(fid, cand) != 0) {
                                    continue;
                                }
                                const auto [t2, i2] = invertedCells(cand);
                                (void)t2;
                                if (i2 < liveFolds) {
                                    dbg("mesh face %d: self-heal - %d folded "
                                        "-> %d by local fold repair",
                                        fid, liveFolds, i2);
                                    parts[fid] = std::move(cand);
                                    liveFolds = i2;
                                }
                            }
                        }
                    }
                    if (liveFolds > 0) {
                        FaceMeshSettings fsT = s;
                        const double dscT =
                            std::clamp(settings.densityScale, 0.05, 20.0);
                        if (dscT != 1.0) {
                            fsT.chordTolerance /= dscT * dscT;
                            fsT.angleToleranceDeg = std::clamp(
                                fsT.angleToleranceDeg / dscT, 1.0, 60.0);
                        }
                        PolyMesh cand;
                        MeshBuilder cb(cand);
                        const bool built = meshContractFallback(
                            face, model, fid, solvedEdge, s.radial, cb,
                            &fsT, angleSplit, &pinnedEdge);
                        if (built &&
                            borderContractViolation(fid, cand) == 0) {
                            const auto [ctested, cinverted] =
                                invertedCells(cand);
                            (void)ctested;
                            if (cinverted < liveFolds) {
                                dbg("mesh face %d: self-heal — %d folded "
                                    "cell(s) on %s, floor folds %d, floor "
                                    "kept",
                                    fid, liveFolds,
                                    mesherKindName(plan.kind), cinverted);
                                parts[fid] = std::move(cand);
                                fellBack[fid] = 2;  // exact borders
                            }
                        }
                    }
                }
            }
        }
    };

    std::vector<int> workFaces;
    workFaces.reserve(cacheMisses);
    for (int fid = 1; fid <= faceN; ++fid) {
        if (!cached[fid]) workFaces.push_back(fid);
    }
    unsigned threads = std::min<unsigned>(
        std::max(1u, std::thread::hardware_concurrency()),
        std::max(1u, static_cast<unsigned>(workFaces.size())));
    if (!settings.parallelMeshing) threads = 1;
    if (threads > 1) {
        // OCCT computes pcurves and UV bounds lazily and caches them on
        // the SHARED TShape — workers racing through
        // BRepTools::AddUVBounds / BRep_Tool::CurveOnSurface segfault on
        // large assemblies (observed inside meshCoonsGrid on an 8k-face
        // model). Warm only the cache-miss faces and their edges
        // single-threaded first; the parallel pass then only reads.
        std::vector<char> warmEdge(model.edgeCount() + 1, 0);
        for (int fid : workFaces) {
            for (TopExp_Explorer ex(model.faces(fid), TopAbs_EDGE); ex.More();
                 ex.Next()) {
                const int eid = model.edges.FindIndex(ex.Current());
                if (eid > 0) warmEdge[eid] = 1;
            }
            try {
                Bnd_Box2d warm;
                BRepTools::AddUVBounds(TopoDS::Face(model.faces(fid)), warm);
                (void)BRep_Tool::Surface(TopoDS::Face(model.faces(fid)));
            } catch (...) {
                // A face too broken to bound fails later, visibly.
            }
        }
        for (int eid = 1; eid <= model.edgeCount(); ++eid) {
            if (!warmEdge[eid]) continue;
            try {
                double f = 0, l = 0;
                (void)BRep_Tool::Curve(TopoDS::Edge(model.edges(eid)), f, l);
            } catch (...) {
            }
        }
    }
    dbg("generate: meshing on %u thread(s), %d cached", threads, cacheHits);
    // Isolate a face whose mesher THROWS (a degenerate manual count, an OCCT
    // assertion) so it can't take the whole model's mesh down with it — the
    // app then keeps every other face and the user can adjust or undo the one
    // edit that broke this face, instead of losing the entire result. The
    // throwing face's partial output is cleared and it falls back to the plain
    // contract/OCCT triangulation, which always builds; if even that throws,
    // the face is left empty (a local hole) rather than aborting the run.
    auto meshFaceGuarded = [&](int fid) {
        try {
            meshFace(fid);
            return;
        } catch (const std::exception& e) {
            dbg("mesh face %d: mesher threw (%s) -> isolate + fallback", fid,
                e.what());
        } catch (...) {
            dbg("mesh face %d: mesher threw -> isolate + fallback", fid);
        }
        parts[fid] = PolyMesh();
        try {
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            BRepAdaptor_Surface surf(face);
            FaceMeshSettings s = settings.forFace(fid);
            s.cellCap = faceCellCap[fid];
            demote(fid, face, surf, s, "mesher threw");
        } catch (...) {
            parts[fid] = PolyMesh();  // fallback threw too: leave it empty
            dbg("mesh face %d: fallback threw too -> left empty", fid);
        }
    };
    auto meshFaceNeeded = [&](int fid) {
        meshFaceGuarded(fid);
        if (settings.progressFaces) {
            settings.progressFaces->fetch_add(1,
                                               std::memory_order_relaxed);
        }
    };
    if (threads <= 1) {
        for (int fid : workFaces) meshFaceNeeded(fid);
    } else {
        std::atomic<size_t> nextFace{0};
        std::exception_ptr firstError;
        std::mutex errorMutex;
        std::vector<std::thread> pool;
        for (unsigned t = 0; t < threads; ++t) {
            pool.emplace_back([&] {
                try {
                    for (size_t wi = nextFace.fetch_add(1);
                         wi < workFaces.size(); wi = nextFace.fetch_add(1)) {
                        meshFaceNeeded(workFaces[wi]);
                    }
                } catch (...) {
                    std::lock_guard<std::mutex> lock(errorMutex);
                    if (!firstError) firstError = std::current_exception();
                }
            });
        }
        for (std::thread& th : pool) th.join();
        if (firstError) std::rethrow_exception(firstError);
    }

    if (cache) {
        for (int fid = 1; fid <= faceN; ++fid) {
            if (!cached[fid]) {
                cache->faces[fid] = {cacheKey[fid], parts[fid],
                                     fellBack[fid], builtCounts[fid]};
            }
        }
    }
    timingCheckpoint("face meshing");

    // Report the counts the mesher ACTUALLY built, per face, against what was
    // requested — so a --radial a shared/feature-constrained rim couldn't take
    // (or a boundary/rail total) is visible instead of the sparse/zero
    // placeholder. Runs post-mesh so the built count (annulus dense rim,
    // boundary/rail totals) is settled. --debug only. Uses the planned kind
    // (before the demote loop below rewrites it to Fallback).
    for (int fid = 1; fid <= faceN; ++fid) {
        const FacePlan& plan = plans.at(fid);
        if (!plan.constrains) continue;
        const FaceMeshSettings& s = settings.forFace(fid);
        const int nu =
            builtCounts[fid][0] >= 0 ? builtCounts[fid][0] : counts[fid][0];
        const int nv =
            builtCounts[fid][1] >= 0 ? builtCounts[fid][1] : counts[fid][1];
        const bool isRev = plan.kind == MesherKind::RevolutionGrid ||
                           plan.kind == MesherKind::DiskCap ||
                           plan.kind == MesherKind::DomeCap ||
                           plan.kind == MesherKind::AnnulusRing;
        const int reqU = isRev ? s.radial : s.gridU;
        dbg("face %d %s: solved nu=%d nv=%d (requested %s=%d axial=%d)%s", fid,
            mesherKindName(plan.kind), nu, nv, isRev ? "radial" : "gridu",
            reqU, s.axial,
            (isRev && nu != std::max(3, reqU)) ? "  [rim not free]" : "");
    }

    // A part that fell back is a fallback for EVERY downstream stage:
    // conform must treat its borders as freeform movers, not as an
    // exact-border authority, and the report must tell the truth.
    for (int fid = 1; fid <= faceN; ++fid) {
        // Contract-floor parts (fellBack == 2) keep their plan: their
        // borders are exact at the solved counts, so they remain
        // conform AUTHORITIES — demoting them to Fallback would let
        // the conform pass kidnap verified border vertices and tear
        // web triangles open.
        if (fellBack[fid] != 1) continue;
        FacePlan& pl = plans.at(fid);
        if (pl.kind == MesherKind::Fallback) continue;
        dbg("mesh face %d: %s couldn't build, plan demoted to fallback",
            fid, mesherKindName(pl.kind));
        pl.kind = MesherKind::Fallback;
        pl.constrains = false;
        // A demoted face built the contract-floor tri soup, not the mesher
        // whose count was optimistically recorded — drop it so the report
        // shows the deviation-driven fallback ({0,0}), as before.
        builtCounts[fid] = {-1, -1};
        pl.coonsSides = {};
        pl.loops.clear();
    }

    PolyMesh mesh;
    std::vector<std::array<size_t, 2>> range(faceN + 1, {0, 0});
    for (int fid = 1; fid <= faceN; ++fid) {
        PolyMesh& part = parts[fid];
        uint32_t base = uint32_t(mesh.vertices.size());
        range[fid] = {size_t(base), size_t(base) + part.vertices.size()};
        mesh.vertices.insert(mesh.vertices.end(), part.vertices.begin(),
                             part.vertices.end());
        mesh.anchors.insert(mesh.anchors.end(), part.anchors.begin(),
                            part.anchors.end());
        for (auto& poly : part.polygons) {
            for (uint32_t& v : poly) v += base;
            mesh.polygons.push_back(std::move(poly));
        }
        mesh.polygonFaceId.insert(mesh.polygonFaceId.end(),
                                  part.polygonFaceId.begin(),
                                  part.polygonFaceId.end());
    }

    dbg("generate: merged (%zu verts, %zu polys)", mesh.vertexCount(),
        mesh.polygonCount());

    applyOrthogonalEdgeContracts(mesh,model,plans,solvedEdge,pinnedEdge);
    dbg("generate: orthogonal edge contracts applied");

    // Face-scoped topology tracing for full-model investigations. Keeping
    // this behind an environment variable makes it cheap enough to leave in
    // place, while avoiding misleading reduced STEP fixtures whose B-rep
    // topology can differ after round-tripping. Set WEFT_FACE_DEBUG to the
    // source face ID that should be traced.
    const char* faceDebugEnv = std::getenv("WEFT_FACE_DEBUG");
    const int faceDebugId = faceDebugEnv ? std::atoi(faceDebugEnv) : 0;
    auto traceFaceTopology = [&](const char* stage) {
        if (faceDebugId <= 0) return;
        size_t polys = 0, tris = 0, quads = 0, ngons = 0;
        std::set<uint32_t> verts;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (p >= mesh.polygonFaceId.size() ||
                mesh.polygonFaceId[p] != faceDebugId) {
                continue;
            }
            const auto& poly = mesh.polygons[p];
            ++polys;
            if (poly.size() == 3) ++tris;
            else if (poly.size() == 4) ++quads;
            else if (poly.size() > 4) ++ngons;
            verts.insert(poly.begin(), poly.end());
        }
        std::vector<uint32_t> geometric;
        const double positionTol = std::max(1e-12, settings.weldTolerance);
        const double positionTol2 = positionTol * positionTol;
        for (uint32_t v : verts) {
            if (v >= mesh.vertices.size()) continue;
            const auto& p = mesh.vertices[v];
            bool duplicate = false;
            for (uint32_t k : geometric) {
                const auto& q = mesh.vertices[k];
                const double dx = p[0] - q[0], dy = p[1] - q[1],
                             dz = p[2] - q[2];
                if (dx * dx + dy * dy + dz * dz <= positionTol2) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) geometric.push_back(v);
        }
        dbg("face %d topology after %s: %zu ids / %zu positions, %zu cells "
            "(%zu tri, %zu quad, %zu ngon)",
            faceDebugId, stage, verts.size(), geometric.size(), polys, tris,
            quads, ngons);
    };
    traceFaceTopology("merge");
    if (settings.finalizeMesh && settings.conformBorders) {
        conformFallbackBorders(mesh, model, plans, settings, range,
                               fellBack);
        dbg("generate: borders conformed");
        traceFaceTopology("border conform");
    }
    timingCheckpoint("merge + conform");

    if (report) {
        report->cacheHits = cacheHits;
        report->cacheMisses = cacheMisses;
        report->canonicalSharedEdges = canonicalSharedEdges;
        report->canonicalSharedSamples = canonicalSharedSamples;
        for (int fid = 1; fid <= faceN; ++fid) {
            (cached[fid] ? report->reusedFaces : report->remeshedFaces)
                .push_back(fid);
        }
        for (int fid = 1; fid <= faceN; ++fid) {
            const FaceMeshSettings& s = settings.forFace(fid);
            const FacePlan& plan = plans.at(fid);
            bool fallbackQuads = plan.forceFallbackQuads >= 0
                                     ? plan.forceFallbackQuads != 0
                                     : s.quadDominant;
            report->faceMesher[fid] =
                plan.kind == MesherKind::Fallback && fallbackQuads
                    ? MesherKind::QuadDominant
                    : plan.kind;
            if (!s.exclude) {
                report->faceBuild[fid] = parts[fid].polygons.empty()
                                             ? -1
                                             : int(fellBack[fid]);
            }
            if (plan.kind == MesherKind::RevolutionGrid &&
                plan.uEdges.size() == 2) {
                report->faceRims[fid] = {plan.uEdges[0], plan.uEdges[1]};
            }
            // Blend strips: which patch axis the fillet-loops knob
            // (across-the-blend) drives, so the UI can label knobs
            // semantically instead of leaking the wire-start-dependent
            // u/v orientation (mirror twins rotate their sides).
            if (plan.isFillet && (plan.kind == MesherKind::CoonsGrid ||
                                  plan.kind == MesherKind::PlanarGrid)) {
                report->faceAcross[fid] = plan.acrossIsU ? 1 : 2;
            }
            // The solved primary/secondary counts, so a UI can seed its
            // manual fields from what the face actually meshed at. Prefer the
            // count the mesher REPORTED building (annulus dense-rim columns,
            // boundary/rail totals) over the pre-mesh `counts` table, which
            // holds the sparse notch rim / zero placeholder for those kinds.
            report->faceCounts[fid] =
                builtCounts[fid][0] >= 0
                    ? builtCounts[fid]
                    : std::array<int, 2>{counts[fid][0], counts[fid][1]};
            if (plan.constrains) {
                for (int eid : plan.uEdges) {
                    report->edgeDivisions[eid] = density.countFor(eid, 0);
                }
                for (int eid : plan.vEdges) {
                    report->edgeDivisions[eid] = density.countFor(eid, 0);
                }
                if (plan.circleEdgeId > 0) {
                    report->edgeDivisions[plan.circleEdgeId] =
                        density.countFor(plan.circleEdgeId, 0);
                }
            }
        }
    }

    // Whole-mesh weld tolerance (the surfaced global knob), clamped to a
    // fraction of the model diagonal so an absurd user value can't fuse
    // the entire model into a point. 0.02 * diagonal is generous (a 1mm
    // gap on a 50mm part is 0.02 of a ~87mm diagonal) yet forbids a
    // model-scale collapse; the default 1e-6 is orders below the clamp, so
    // unionSeams stays bit-identical at default. (weldVertices below gets a
    // per-vertex, feature-clamped radius instead of this blunt scalar.)
    double weldGlobal = settings.weldTolerance;
    {
        double diag = cache ? cache->modelDiagonal : -1.0;
        if (diag < 0.0) {
            Bnd_Box wbb;
            BRepBndLib::Add(model.shape, wbb);
            if (!wbb.IsVoid()) diag = std::sqrt(wbb.SquareExtent());
            if (cache) cache->modelDiagonal = diag;
        }
        if (diag > 0.0) weldGlobal = std::min(weldGlobal, 0.02 * diag);
    }

    // Per-face weld tolerances (0 = inherit the global), indexed by FaceId.
    std::vector<double> faceWeld(faceN + 1, 0.0);
    bool anyPerFaceWeld = false;
    for (int fid = 1; fid <= faceN; ++fid) {
        double w = settings.forFace(fid).weldTolerance;
        if (w > 0.0) {
            faceWeld[fid] = w;
            anyPerFaceWeld = true;
        }
    }

    // Corner canonicalization: curve endpoints of DIFFERENT edges meeting
    // at one B-rep vertex disagree by the vertex tolerance (~1e-4 on real
    // exports), far above the weld tolerance — every face computes its
    // corner from its own edge, so corners never welded. Snap any mesh
    // vertex within a B-rep vertex's tolerance onto its exact point.
    auto finish = [&](PolyMesh& mesh) {
        std::shared_ptr<CornerRepairCache> repair;
        if (cache && cache->cornerRepair) {
            repair = std::static_pointer_cast<CornerRepairCache>(
                cache->cornerRepair);
        }
        if (!repair) {
            repair = std::make_shared<CornerRepairCache>();
        weft::ShapeMap vmap;
        TopExp::MapShapes(model.shape, TopAbs_VERTEX, vmap);

        // Micro-edge collapse: CAD booleans leave hairline edges (a few
        // microns to ~0.1mm on real parts) whose two vertices are
        // distinct, so sliver faces and unmatchable seams survive every
        // weld. Union the endpoints of any edge shorter than 5e-4 of the
        // model diagonal (a conventional stitch tolerance) — sliver
        // polygons then degenerate away in the weld and the flanking
        // faces zip directly.
        std::vector<int> root(vmap.Extent() + 1);
        std::iota(root.begin(), root.end(), 0);
        auto find = [&](int i) {
            while (root[i] != i) i = root[i] = root[root[i]];
            return i;
        };
        // Short edges at a structured revolution boundary are deliberate
        // slot-tip and seam topology, not disposable CAD noise. Protect the
        // vertices on those boundaries and every micro edge touching them;
        // otherwise a later union turns a six-corner cylindrical patch into
        // a misleading quad and silently transfers its B-rep ownership to a
        // neighbouring face.
        std::set<int> protectedMicroVertices;
        for (const auto& [fid, plan] : plans) {
            if (fid < 1 || fid > faceN ||
                !plan.orthogonalTrimGrid ||
                plan.kind != MesherKind::RevolutionGrid) {
                continue;
            }
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            for (TopExp_Explorer vx(face, TopAbs_VERTEX);
                 vx.More(); vx.Next()) {
                const int vi = vmap.FindIndex(vx.Current());
                if (vi > 0) protectedMicroVertices.insert(vi);
            }
        }
        const std::set<int> structuredBoundaryVertices =
            protectedMicroVertices;
        for (int fid = 1; fid <= faceN; ++fid) {
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            bool touchesStructuredBoundary = false;
            for (TopExp_Explorer vx(face, TopAbs_VERTEX);
                 vx.More(); vx.Next()) {
                const int vi = vmap.FindIndex(vx.Current());
                if (structuredBoundaryVertices.count(vi)) {
                    touchesStructuredBoundary = true;
                    break;
                }
            }
            if (!touchesStructuredBoundary) continue;
            for (TopExp_Explorer vx(face, TopAbs_VERTEX);
                 vx.More(); vx.Next()) {
                const int vi = vmap.FindIndex(vx.Current());
                if (vi > 0) protectedMicroVertices.insert(vi);
            }
        }
        Bnd_Box bb;
        BRepBndLib::Add(model.shape, bb);
        const double modelDiagonal = std::sqrt(bb.SquareExtent());
        const double microTol = 5e-4*modelDiagonal;
        // Structured boolean boundaries retain ordinary short CAD edges, but
        // sub-micron duplicates inside an imported tolerance envelope are not
        // useful topology. Keeping those creates a zero-area wedge beside the
        // exact edge and can make that edge three-manifold. This threshold is
        // 500x tighter than the legacy repair radius.
        const double structuredNoiseTol = 1e-6*modelDiagonal;
        repair->structuredNoiseTol = structuredNoiseTol;
        int microEdges = 0;
        // Preserve the established spatial repair radius for raw/fallback
        // meshers. Structured split rectangles use the face-parametric
        // contract below and never query this radius, so a collapsed
        // micro-edge cannot capture one of their legitimate lattice rows.
        std::vector<double> reach(vmap.Extent() + 1, 0.0);
        for (TopExp_Explorer ex(model.shape, TopAbs_EDGE); ex.More();
             ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(e, v1, v2);
            if (v1.IsNull() || v2.IsNull() || v1.IsSame(v2)) continue;
            const int i1 = vmap.FindIndex(v1);
            const int i2 = vmap.FindIndex(v2);
            const bool protectedStructured =
                protectedMicroVertices.count(i1) ||
                protectedMicroVertices.count(i2);
            const double collapseTolerance =
                protectedStructured ? structuredNoiseTol : microTol;
            if (BRep_Tool::Pnt(v1).Distance(BRep_Tool::Pnt(v2)) >=
                collapseTolerance) {
                continue;
            }
            BRepAdaptor_Curve c(e);
            const double len = GCPnts_AbscissaPoint::Length(c);
            if (len >= collapseTolerance) continue;
            const int a = find(i1);
            const int b = find(i2);
            if (a != b) {
                root[a] = b;
                reach[b] += reach[a] + len;
            } else {
                reach[b] += len;
            }
            ++microEdges;
        }

        // Snap-test against each vertex's own point/tolerance, but send
        // the mesh vertex to its GROUP representative's point.
        auto& corners = repair->corners;
        double& cornerCell = repair->cell;
        for (int i = 1; i <= vmap.Extent(); ++i) {
            const TopoDS_Vertex v = TopoDS::Vertex(vmap(i));
            const int r = find(i);
            const double tol =
                std::max({1e-7, 2.0 * BRep_Tool::Tolerance(v),
                          1.05 * reach[r]});
            corners.push_back({BRep_Tool::Pnt(v), tol,
                               BRep_Tool::Pnt(TopoDS::Vertex(vmap(r)))});
            cornerCell = std::max(cornerCell, tol);
        }

        // Build the face-parametric corner contract once. A mesh vertex with
        // an Anchor is not eligible for a 3D nearest-corner guess: on thin
        // parts an interior row can sit inside a perfectly valid CAD vertex
        // tolerance sphere. Instead, match it to an endpoint of an incident
        // pcurve on the same face. This is the same topological incidence
        // relation used by the face mesher and remains valid at periodic
        // seams because every coedge contributes its own UV endpoint.
        auto& faceCorners = repair->faceCorners;
        faceCorners.resize(faceN + 1);
        for (int fid = 1; fid <= faceN; ++fid) {
            // Cache geometry-only endpoint incidence for every face. The
            // cheap route mask below decides which current plans consume it,
            // so reusing GenerationCache across a mesher override cannot
            // produce a stale cached-vs-fresh result.
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            BRepAdaptor_Surface fs(face);
            const double uSpan = std::abs(fs.LastUParameter() -
                                          fs.FirstUParameter());
            const double vSpan = std::abs(fs.LastVParameter() -
                                          fs.FirstVParameter());
            // Mesh clipping uses a 1e-9 absolute parameter floor and scales
            // it for charts wider than one parameter unit. Allow a small
            // multiple of that reconstruction error, never a fraction of the
            // geometric vertex tolerance.
            const double uTol = 8e-9 * std::max(1.0, uSpan);
            const double vTol = 8e-9 * std::max(1.0, vSpan);
            auto addEndpoint = [&](const gp_Pnt2d& uv,
                                   const TopoDS_Vertex& a,
                                   const TopoDS_Vertex& b) {
                if (a.IsNull() && b.IsNull()) return;
                const gp_Pnt sheet = fs.Value(uv.X(), uv.Y());
                TopoDS_Vertex chosen;
                if (a.IsNull()) chosen = b;
                else if (b.IsNull()) chosen = a;
                else chosen = sheet.SquareDistance(BRep_Tool::Pnt(a)) <=
                                      sheet.SquareDistance(BRep_Tool::Pnt(b))
                                  ? a : b;
                const int vi = vmap.FindIndex(chosen);
                if (vi <= 0) return;
                // An explicit, non-degenerate B-rep edge is topology, even
                // when it is short. Structured/anchored meshes retain each
                // endpoint exactly; redirecting it through the micro-edge
                // union deletes real narrow rows. The union target remains
                // available to the unanchored spatial-repair path below.
                const gp_Pnt target = BRep_Tool::Pnt(chosen);
                auto& list = faceCorners[fid];
                for (const CachedUvCorner& c : list) {
                    if (std::abs(c.u - uv.X()) <= uTol &&
                        std::abs(c.v - uv.Y()) <= vTol &&
                        c.target.SquareDistance(target) <= 1e-24) {
                        return;
                    }
                }
                list.push_back({uv.X(), uv.Y(), uTol, vTol, target});
            };
            for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More();
                 wx.Next()) {
                for (BRepTools_WireExplorer we(
                         TopoDS::Wire(wx.Current()), face);
                     we.More(); we.Next()) {
                    const TopoDS_Edge edge = we.Current();
                    double first = 0.0, last = 0.0;
                    Handle(Geom2d_Curve) pc =
                        BRep_Tool::CurveOnSurface(edge, face, first, last);
                    if (pc.IsNull()) continue;
                    TopoDS_Vertex a, b;
                    TopExp::Vertices(edge, a, b);
                    addEndpoint(pc->Value(first), a, b);
                    addEndpoint(pc->Value(last), a, b);
                }
            }
        }

        // Spatially index CAD corners. The previous all-pairs scan compared
        // every mesh vertex with every B-rep vertex: MP9's ~41k mesh vertices
        // and thousands of CAD corners made a fully cached face edit spend
        // most of its time here. A cell is the largest capture radius, so any
        // matching corner must be in the query cell or one of its 26
        // neighbours. Keep the lowest corner index to preserve the old
        // first-match behaviour when tolerance spheres overlap.
        auto& cornerGrid = repair->grid;
        auto cornerCellOf = [&](const gp_Pnt& p) -> CornerCell {
            return {static_cast<long long>(std::floor(p.X() / cornerCell)),
                    static_cast<long long>(std::floor(p.Y() / cornerCell)),
                    static_cast<long long>(std::floor(p.Z() / cornerCell))};
        };
        for (size_t i = 0; i < corners.size(); ++i) {
            cornerGrid[cornerCellOf(corners[i].at)].push_back(i);
        }
        repair->microEdges = microEdges;
        if (cache) cache->cornerRepair = repair;
        }
        const auto& corners = repair->corners;
        const double cornerCell = repair->cell;
        const auto& cornerGrid = repair->grid;
        const auto& faceCorners = repair->faceCorners;
        const double structuredNoiseTol =
            repair->structuredNoiseTol;
        // Route selection can change while a GenerationCache survives.
        // Recompute this cheap mask from the current plans every run instead
        // of caching stale mesher state beside the geometry-only corner table.
        std::vector<bool> topologicalFaces(faceN + 1, false);
        for (const auto& [fid, plan] : plans) {
            if (fid < 1 || fid > faceN) continue;
            topologicalFaces[fid] =
                plan.orthogonalTrimGrid && !plan.orthogonalLocalComb &&
                plan.kind == MesherKind::CoonsGrid &&
                fid < static_cast<int>(faceCorners.size()) &&
                !faceCorners[fid].empty();
        }
        const int microEdges = repair->microEdges;
        auto cornerCellOf = [&](const gp_Pnt& p) -> CornerCell {
            return {static_cast<long long>(std::floor(p.X() / cornerCell)),
                    static_cast<long long>(std::floor(p.Y() / cornerCell)),
                    static_cast<long long>(std::floor(p.Z() / cornerCell))};
        };
        size_t snapped = 0;
        for (size_t mi = 0; mi < mesh.vertices.size(); ++mi) {
            auto& mv = mesh.vertices[mi];
            if (mi < mesh.anchors.size() && mesh.anchors[mi].faceId > 0 &&
                mesh.anchors[mi].faceId <
                    static_cast<int>(topologicalFaces.size()) &&
                topologicalFaces[mesh.anchors[mi].faceId]) {
                const Anchor& anchor = mesh.anchors[mi];
                if (anchor.faceId < static_cast<int>(faceCorners.size())) {
                    const auto& candidates = faceCorners[anchor.faceId];
                    const CachedUvCorner* bestUv = nullptr;
                    double bestScore = 1e300;
                    for (const CachedUvCorner& c : candidates) {
                        const double du = std::abs(anchor.u - c.u);
                        const double dv = std::abs(anchor.v - c.v);
                        if (du > c.uTol || dv > c.vTol) continue;
                        const double score = du / c.uTol + dv / c.vTol;
                        if (score < bestScore) {
                            bestScore = score;
                            bestUv = &c;
                        }
                    }
                    if (bestUv) {
                        mv = {bestUv->target.X(), bestUv->target.Y(),
                              bestUv->target.Z()};
                        ++snapped;
                    }
                }
                // A face anchor that did not match an endpoint is an
                // interior/edge sample by definition. Never fall through to
                // the spatial corner search.
                continue;
            }
            gp_Pnt p(mv[0], mv[1], mv[2]);
            const auto [cx, cy, cz] = cornerCellOf(p);
            size_t best = corners.size();
            size_t exact = corners.size();
            double exactDistance = 1e300;
            size_t nearExact = corners.size();
            double nearExactDistance =
                structuredNoiseTol*structuredNoiseTol;
            for (long long dx = -1; dx <= 1; ++dx) {
                for (long long dy = -1; dy <= 1; ++dy) {
                    for (long long dz = -1; dz <= 1; ++dz) {
                        auto it = cornerGrid.find(
                            {cx + dx, cy + dy, cz + dz});
                        if (it == cornerGrid.end()) continue;
                        for (size_t ci : it->second) {
                            const CachedCorner& c = corners[ci];
                            const double distance =
                                p.SquareDistance(c.at);
                            // An assembly edge contract places canonical
                            // endpoints exactly on their B-rep vertices.
                            // Imported tolerance spheres can overlap across a
                            // short but real edge; the historic lowest-index
                            // tie-break then moves both endpoints onto one
                            // corner and turns the edge into a point pinch.
                            // Preserve an exact incidence before applying the
                            // legacy tolerant first-match rule.
                            if (distance <= 1e-18 &&
                                distance < exactDistance) {
                                exact = ci;
                                exactDistance = distance;
                            } else if (distance <
                                           nearExactDistance) {
                                nearExact = ci;
                                nearExactDistance = distance;
                            } else if (exact == corners.size() &&
                                       ci < best &&
                                       distance < c.tol*c.tol) {
                                best = ci;
                            }
                        }
                    }
                }
            }
            if (exact != corners.size()) {
                best = exact;
            } else if (nearExact != corners.size()) {
                best = nearExact;
            }
            if (best != corners.size()) {
                const CachedCorner& c = corners[best];
                mv = {c.target.X(), c.target.Y(), c.target.Z()};
                ++snapped;
            }
        }
        dbg("generate: %zu corner verts canonicalized, %d micro edges "
            "collapsed",
            snapped, microEdges);
        traceFaceTopology("corner canonicalize");
        timingCheckpoint("corner canonicalize");

        // Solid-scoped weld: contacting bodies in a multi-body file have
        // coincident skins with opposing windings — a global weld fuses
        // them into non-manifold shared edges (every directed edge used
        // twice). Group vertices by owning solid so only same-body seams
        // merge.
        std::vector<int> weldGroup;
        {
            std::vector<int> faceSolid(faceN + 1, 0);
            int solidId = 0;
            auto assign = [&](const TopoDS_Shape& obj) {
                ++solidId;
                for (TopExp_Explorer fx(obj, TopAbs_FACE); fx.More();
                     fx.Next()) {
                    int fid = model.faces.FindIndex(fx.Current());
                    if (fid > 0 && faceSolid[fid] == 0) {
                        faceSolid[fid] = solidId;
                    }
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
            dbg("weld: %d body group(s)", solidId);
            if (solidId > 1) {
                weldGroup.assign(mesh.vertices.size(), 0);
                for (int fid = 1; fid <= faceN; ++fid) {
                    for (size_t v = range[fid][0]; v < range[fid][1]; ++v) {
                        weldGroup[v] = faceSolid[fid];
                    }
                }
            }
        }
        timingCheckpoint("weld groups");

        // Per-vertex weld radius: each vertex welds at the LOOSEST of the
        // global tolerance and every per-face override on a polygon that
        // touches it (max-wins, so raising one face closes its junctions).
        // The radius is clamped to HALF the shortest mesh edge incident to
        // the vertex — the local resolution — so a tolerance larger than
        // nearby detail only ever fuses genuine near-duplicates and never
        // swallows a distinct neighbouring vertex into non-manifold soup.
        // Never below 1e-6 (the historical floor), so the default global
        // 1e-6 with no override reproduces the old single-tolerance weld
        // bit for bit. Built only when a knob is actually off default;
        // otherwise the scalar path (weldGlobal) runs unchanged.
        std::vector<double> vertTol;
        double weldMax = weldGlobal;
        const bool perVertex =
            anyPerFaceWeld || settings.weldTolerance != 1e-6;
        if (perVertex) {
            constexpr double kFloor = 1e-6;
            const double g = std::max(settings.weldTolerance, 0.0);
            std::vector<double> loose(mesh.vertices.size(),
                                      std::max(g, kFloor));
            std::vector<double> shortEdge(mesh.vertices.size(), 1e300);
            for (size_t p = 0; p < mesh.polygons.size(); ++p) {
                const int fid = mesh.polygonFaceId[p];
                const double fw =
                    (fid >= 1 && fid <= faceN) ? faceWeld[fid] : 0.0;
                const auto& poly = mesh.polygons[p];
                const size_t n = poly.size();
                for (size_t i = 0; i < n; ++i) {
                    const uint32_t a = poly[i], b = poly[(i + 1) % n];
                    if (fw > loose[a]) loose[a] = fw;
                    if (fw > loose[b]) loose[b] = fw;
                    const auto& A = mesh.vertices[a];
                    const auto& B = mesh.vertices[b];
                    const double dx = A[0] - B[0], dy = A[1] - B[1],
                                 dz = A[2] - B[2];
                    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (len > 0.0) {
                        if (len < shortEdge[a]) shortEdge[a] = len;
                        if (len < shortEdge[b]) shortEdge[b] = len;
                    }
                }
            }
            vertTol.resize(mesh.vertices.size());
            weldMax = 0.0;
            for (size_t v = 0; v < mesh.vertices.size(); ++v) {
                const double clamp =
                    shortEdge[v] < 1e300 ? 0.5 * shortEdge[v] : 1e300;
                double t = std::min(loose[v], std::max(clamp, kFloor));
                if (t < kFloor) t = kFloor;
                vertTol[v] = t;
                if (t > weldMax) weldMax = t;
            }
            if (weldMax <= 0.0) weldMax = weldGlobal;
        }
        timingCheckpoint("weld tolerances");

        dbg("generate: welding%s%s", weldGroup.empty() ? "" : " (per solid)",
            perVertex ? " (per-vertex tol)" : "");
        weldVertices(mesh, weldMax,
                     weldGroup.empty() ? nullptr : &weldGroup,
                     perVertex ? &vertTol : nullptr);
        traceFaceTopology("vertex weld");
        timingCheckpoint("vertex weld");
    };

    finish(mesh);
    timingCheckpoint("corner repair + weld");
    if (!settings.finalizeMesh) {
        // Interactive editing still requires shared vertex IDs across face
        // seams. Stop after the cached corner repair and lightweight weld;
        // export-only border conformation, seam insertion/stitching, and
        // cleanup remain deferred.
        dbg("generate: connected preview done (%zu verts, %zu polys)",
            mesh.vertexCount(), mesh.polygonCount());
        timingCheckpoint("preview complete");
        return mesh;
    }
    if (settings.conformBorders) {
        // Post-weld: borders share ids now, so an open edge with an exact
        // complement path is a REAL T-junction, never a pre-weld ghost.
        unionSeams(mesh, model, weldGlobal);
    }
    traceFaceTopology("seam union");
    timingCheckpoint("union seams");
    const bool hasOrthogonalTrim = std::any_of(
        plans.begin(), plans.end(), [](const auto& kv) {
            return kv.second.orthogonalTrimGrid;
        });
    if ((settings.decoupleSeams || hasOrthogonalTrim) &&
        !std::getenv("WEFT_NO_STITCH")) {
        // The curve-guided stitcher: every 2-owner B-rep edge's two sides
        // merge onto one parameter-sorted vertex chain, closing the seams
        // the decoupled counts left open. Twin fusion first: two samples
        // of the same contract point (same edge, same param, a hair
        // apart) must become one vertex or the stitcher has nothing to
        // splice. WEFT_NO_STITCH / WEFT_NO_FUSE are the diagnosis
        // kill-switches.
        if (!std::getenv("WEFT_NO_FUSE")) {
            fuseSeamTwins(mesh, model, weldGlobal);
        }
        stitchSeams(mesh, model, weldGlobal, plans, settings, density);
    }
    traceFaceTopology("seam stitch");
    // Fold cleanup: a directed edge traversed twice WITHIN one face means
    // conform or decimation wrapped a flap of polygons over its
    // neighbours. The flap is the smaller overlapping polygon — drop it;
    // the tiny open it leaves beats a non-manifold fold.
    {
        std::map<std::pair<uint32_t, uint32_t>, std::vector<size_t>> dir;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            const auto& poly = mesh.polygons[p];
            for (size_t i = 0; i < poly.size(); ++i) {
                dir[{poly[i], poly[(i + 1) % poly.size()]}].push_back(p);
            }
        }
        auto polyArea = [&](size_t p) {
            const auto& poly = mesh.polygons[p];
            double nx = 0, ny = 0, nz = 0;
            for (size_t i = 0; i < poly.size(); ++i) {
                const auto& a = mesh.vertices[poly[i]];
                const auto& b = mesh.vertices[poly[(i + 1) % poly.size()]];
                nx += a[1] * b[2] - a[2] * b[1];
                ny += a[2] * b[0] - a[0] * b[2];
                nz += a[0] * b[1] - a[1] * b[0];
            }
            return 0.5 * std::sqrt(nx * nx + ny * ny + nz * nz);
        };
        std::set<size_t> drop;
        for (const auto& [e, ps] : dir) {
            if (ps.size() < 2) continue;
            bool sameFace = true;
            for (size_t p : ps) {
                sameFace &= mesh.polygonFaceId[p] ==
                            mesh.polygonFaceId[ps[0]];
            }
            if (!sameFace) continue;  // cross-face dup: not a local flap
            size_t keep = ps[0];
            double best = -1.0;
            for (size_t p : ps) {
                double a = polyArea(p);
                if (a > best) {
                    best = a;
                    keep = p;
                }
            }
            for (size_t p : ps) {
                if (p != keep) drop.insert(p);
            }
        }
        if (!drop.empty()) {
            std::vector<std::vector<uint32_t>> polys;
            std::vector<int> polyFace;
            polys.reserve(mesh.polygons.size() - drop.size());
            polyFace.reserve(polys.capacity());
            for (size_t p = 0; p < mesh.polygons.size(); ++p) {
                if (drop.count(p)) continue;
                polys.push_back(std::move(mesh.polygons[p]));
                polyFace.push_back(mesh.polygonFaceId[p]);
            }
            mesh.polygons = std::move(polys);
            mesh.polygonFaceId = std::move(polyFace);
            dbg("generate: %zu folded polygons dropped", drop.size());
        }
    }
    traceFaceTopology("fold cleanup");

    // De-slit: the micro-edge collapse zips a sliver strip's two rails
    // onto one welded segment, and a plate whose boundary WRAPS the
    // zero-width notch then walks that segment twice in opposite
    // directions inside one polygon. With the two flanking walls also on
    // the segment it counts 4 uses — non-manifold. Splitting the polygon
    // at the doubled segment into its two lobes removes both traversals
    // and leaves exactly the two wall uses: 2-manifold. Only that exact
    // situation is touched (two anti-parallel traversals in ONE polygon,
    // total use 4) — keyhole bridges (2 uses, both in the polygon) and
    // contact sandwiches (4 uses across different polygons) pass through
    // untouched.
    for (int pass = 0; pass < 4; ++pass) {
        std::map<std::pair<uint32_t, uint32_t>, int> use;
        for (const auto& poly : mesh.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                if (a > b) std::swap(a, b);
                ++use[{a, b}];
            }
        }
        size_t split = 0;
        // A segment is consumed by the first polygon split on it — the
        // use counts are stale within a pass.
        std::set<std::pair<uint32_t, uint32_t>> consumed;
        const size_t nPolys = mesh.polygons.size();
        for (size_t p = 0; p < nPolys; ++p) {
            const auto poly = mesh.polygons[p];  // copy: p may be replaced
            const size_t n = poly.size();
            if (n < 6) continue;
            // First doubled anti-parallel segment with 4 total uses.
            size_t i1 = n, i2 = n;
            for (size_t i = 0; i < n && i1 == n; ++i) {
                const uint32_t u = poly[i], v = poly[(i + 1) % n];
                if (u == v) continue;
                for (size_t j = i + 1; j < n; ++j) {
                    if (poly[j] == v && poly[(j + 1) % n] == u) {
                        uint32_t a = u, b = v;
                        if (a > b) std::swap(a, b);
                        if (use[{a, b}] == 4 && !consumed.count({a, b})) {
                            i1 = i;
                            i2 = j;
                            consumed.insert({a, b});
                        }
                        break;
                    }
                }
            }
            if (i1 == n) continue;
            // Lobe A: poly[i1+1 .. i2] walks v .. v — drop the closing
            // duplicate. Lobe B: poly[i2+1 .. i1] walks u .. u likewise.
            auto lobe = [&](size_t from, size_t to) {
                std::vector<uint32_t> out;
                for (size_t k = from; ; k = (k + 1) % n) {
                    out.push_back(poly[k]);
                    if (k == to) break;
                }
                while (out.size() > 1 && out.front() == out.back()) {
                    out.pop_back();
                }
                return out;
            };
            std::vector<uint32_t> lobeA = lobe((i1 + 1) % n, i2);
            std::vector<uint32_t> lobeB = lobe((i2 + 1) % n, i1);
            const int fid = mesh.polygonFaceId[p];
            bool first = true;
            for (auto& l : {lobeA, lobeB}) {
                if (l.size() < 3) continue;
                if (first) {
                    mesh.polygons[p] = l;
                    first = false;
                } else {
                    mesh.polygons.push_back(l);
                    mesh.polygonFaceId.push_back(fid);
                }
            }
            if (first) {
                // Both lobes degenerate: the polygon was pure slit.
                mesh.polygons[p].clear();
            }
            ++split;
        }
        if (!split) break;
        std::vector<std::vector<uint32_t>> polys;
        std::vector<int> polyFace;
        polys.reserve(mesh.polygons.size());
        polyFace.reserve(mesh.polygons.size());
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygons[p].size() < 3) continue;
            polys.push_back(std::move(mesh.polygons[p]));
            polyFace.push_back(mesh.polygonFaceId[p]);
        }
        mesh.polygons = std::move(polys);
        mesh.polygonFaceId = std::move(polyFace);
        dbg("generate: de-slit pass %d split %zu polygon(s)", pass, split);
    }
    traceFaceTopology("de-slit");

    // Non-manifold micro-segment collapse: twin border edges (two
    // near-coincident B-rep edges between the SAME two faces — a
    // hairline lens of imprint dirt) make both faces span the lens, so a
    // cross-lens rung where their samples coincide collects 4 polygons.
    // The rung is far below the model's feature size; fusing its two
    // vertices removes it with a sub-visible (half-rung) move and the
    // four quads become triangles around the fused vertex. Only edges
    // that are ALREADY non-manifold and shorter than the micro tolerance
    // are touched, so clean geometry is never altered.
    {
        Bnd_Box bb;
        BRepBndLib::Add(model.shape, bb);
        const double microTol = 5e-4 * std::sqrt(bb.SquareExtent());
        for (int pass = 0; pass < 4; ++pass) {
            std::map<std::pair<uint32_t, uint32_t>, int> use;
            for (const auto& poly : mesh.polygons) {
                for (size_t i = 0; i < poly.size(); ++i) {
                    uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                    if (a > b) std::swap(a, b);
                    ++use[{a, b}];
                }
            }
            std::map<uint32_t, uint32_t> fuse;
            for (const auto& [e, count] : use) {
                if (count <= 2) continue;
                if (fuse.count(e.first) || fuse.count(e.second)) continue;
                const auto& A = mesh.vertices[e.first];
                const auto& B = mesh.vertices[e.second];
                const double dx = A[0] - B[0], dy = A[1] - B[1],
                             dz = A[2] - B[2];
                if (dx * dx + dy * dy + dz * dz >= microTol * microTol) {
                    continue;
                }
                fuse[e.second] = e.first;
                mesh.vertices[e.first] = {0.5 * (A[0] + B[0]),
                                          0.5 * (A[1] + B[1]),
                                          0.5 * (A[2] + B[2])};
            }
            if (fuse.empty()) break;
            std::vector<std::vector<uint32_t>> polys;
            std::vector<int> polyFace;
            polys.reserve(mesh.polygons.size());
            polyFace.reserve(mesh.polygons.size());
            for (size_t p = 0; p < mesh.polygons.size(); ++p) {
                std::vector<uint32_t> mapped;
                mapped.reserve(mesh.polygons[p].size());
                for (uint32_t v : mesh.polygons[p]) {
                    auto it = fuse.find(v);
                    const uint32_t m = it == fuse.end() ? v : it->second;
                    if (mapped.empty() || mapped.back() != m) {
                        mapped.push_back(m);
                    }
                }
                while (mapped.size() > 1 && mapped.front() == mapped.back()) {
                    mapped.pop_back();
                }
                if (mapped.size() < 3) continue;
                polys.push_back(std::move(mapped));
                polyFace.push_back(mesh.polygonFaceId[p]);
            }
            mesh.polygons = std::move(polys);
            mesh.polygonFaceId = std::move(polyFace);
            dbg("generate: micro nm-segment pass %d fused %zu vertex "
                "pair(s)",
                pass, fuse.size());
        }
    }
    traceFaceTopology("non-manifold micro fuse");

    // The non-manifold micro-segment repair above can collapse one side of a
    // local-comb quad, leaving an honest triangle beside its unchanged cell.
    // Restore the route's grouped topology only when the two cells have one
    // opposite shared edge and their UV union is provably simple, additive,
    // in-face, CAD-normal aligned, and fold-free.  This is deliberately a
    // route-local certification pass; an unproven triangle is left alone.
    int mergedCertifiedComb = 0;
    for (;;) {
        struct EdgeUse { size_t poly; uint32_t a, b; };
        std::map<std::pair<uint32_t, uint32_t>, std::vector<EdgeUse>> owners;
        for (size_t pi = 0; pi < mesh.polygons.size(); ++pi) {
            const auto& poly = mesh.polygons[pi];
            for (size_t k = 0; k < poly.size(); ++k) {
                const uint32_t a = poly[k], b = poly[(k + 1) % poly.size()];
                owners[{std::min(a, b), std::max(a, b)}].push_back({pi, a, b});
            }
        }

        bool changed = false;
        for (size_t pi = 0; pi < mesh.polygons.size() && !changed; ++pi) {
            if (mesh.polygons[pi].size() != 3) continue;
            const int fid = mesh.polygonFaceId[pi];
            const auto planIt = plans.find(fid);
            if (planIt == plans.end() ||
                !planIt->second.orthogonalLocalComb)
                continue;
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface.IsNull()) continue;
            BRepAdaptor_Surface sa(face);
            const double uvSpan = std::max(
                {1.0, std::abs(sa.LastUParameter() - sa.FirstUParameter()),
                 std::abs(sa.LastVParameter() - sa.FirstVParameter())});
            const double uvTol = 1e-10 * uvSpan;
            const double uvAreaTol = uvTol * uvSpan;
            const double faceTol = BRep_Tool::Tolerance(face);
            const bool reversed = face.Orientation() == TopAbs_REVERSED;

            auto ringUv = [&](const std::vector<uint32_t>& ring,
                              std::vector<gp_Pnt2d>& uv) {
                uv.clear();
                uv.reserve(ring.size());
                for (uint32_t vi : ring) {
                    if (vi >= mesh.vertices.size()) return false;
                    if (vi < mesh.anchors.size() &&
                        mesh.anchors[vi].faceId == fid) {
                        const Anchor& a = mesh.anchors[vi];
                        uv.emplace_back(a.u, a.v);
                        continue;
                    }
                    const auto& v = mesh.vertices[vi];
                    GeomAPI_ProjectPointOnSurf project(
                        gp_Pnt(v[0], v[1], v[2]), surface);
                    if (!project.IsDone() || project.NbPoints() < 1)
                        return false;
                    double u = 0.0, vv = 0.0;
                    project.LowerDistanceParameters(u, vv);
                    uv.emplace_back(u, vv);
                }
                return true;
            };
            auto signedArea = [](const std::vector<gp_Pnt2d>& uv) {
                double area = 0.0;
                for (size_t k = 0; k < uv.size(); ++k) {
                    const gp_Pnt2d& a = uv[k];
                    const gp_Pnt2d& b = uv[(k + 1) % uv.size()];
                    area += a.X() * b.Y() - b.X() * a.Y();
                }
                return 0.5 * area;
            };
            auto uvCross = [](const gp_Pnt2d& a, const gp_Pnt2d& b,
                              const gp_Pnt2d& c) {
                return (b.X() - a.X()) * (c.Y() - a.Y()) -
                       (b.Y() - a.Y()) * (c.X() - a.X());
            };
            auto onSegment = [&](const gp_Pnt2d& p, const gp_Pnt2d& a,
                                 const gp_Pnt2d& b) {
                const double len = std::sqrt(a.SquareDistance(b));
                if (len <= uvTol) return p.SquareDistance(a) <= uvTol * uvTol;
                if (std::abs(uvCross(a, b, p)) > uvTol * len) return false;
                return (p.X() - a.X()) * (p.X() - b.X()) +
                           (p.Y() - a.Y()) * (p.Y() - b.Y()) <=
                       uvTol * uvTol;
            };
            auto simpleUv = [&](const std::vector<gp_Pnt2d>& uv) {
                for (size_t i = 0; i < uv.size(); ++i) {
                    const size_t i2 = (i + 1) % uv.size();
                    if (uv[i].SquareDistance(uv[i2]) <= uvTol * uvTol)
                        return false;
                    for (size_t j = i + 1; j < uv.size(); ++j) {
                        const size_t j2 = (j + 1) % uv.size();
                        if (i2 == j || j2 == i) continue;
                        const double abC = uvCross(uv[i], uv[i2], uv[j]);
                        const double abD = uvCross(uv[i], uv[i2], uv[j2]);
                        const double cdA = uvCross(uv[j], uv[j2], uv[i]);
                        const double cdB = uvCross(uv[j], uv[j2], uv[i2]);
                        if ((abC * abD < 0.0 && cdA * cdB < 0.0) ||
                            onSegment(uv[i], uv[j], uv[j2]) ||
                            onSegment(uv[i2], uv[j], uv[j2]) ||
                            onSegment(uv[j], uv[i], uv[i2]) ||
                            onSegment(uv[j2], uv[i], uv[i2]))
                            return false;
                    }
                }
                return true;
            };
            auto certify = [&](size_t qi, uint32_t a, uint32_t b,
                               std::vector<uint32_t>& united,
                               double& quality) {
                if (qi == pi || mesh.polygonFaceId[qi] != fid) return false;
                const auto& tri = mesh.polygons[pi];
                const auto& neighbour = mesh.polygons[qi];
                size_t ti = tri.size(), ni = neighbour.size();
                for (size_t k = 0; k < tri.size(); ++k) {
                    if (tri[k] == a && tri[(k + 1) % tri.size()] == b) {
                        ti = k;
                        break;
                    }
                }
                for (size_t k = 0; k < neighbour.size(); ++k) {
                    if (neighbour[k] == b &&
                        neighbour[(k + 1) % neighbour.size()] == a) {
                        ni = k;
                        break;
                    }
                }
                if (ti == tri.size() || ni == neighbour.size()) return false;
                united.clear();
                for (size_t k = 0; k < neighbour.size(); ++k)
                    united.push_back(neighbour[(ni + 1 + k) % neighbour.size()]);
                for (size_t k = 1; k + 1 < tri.size(); ++k)
                    united.push_back(tri[(ti + 1 + k) % tri.size()]);
                std::set<uint32_t> unique(united.begin(), united.end());
                if (united.size() < 4 || unique.size() != united.size())
                    return false;

                std::vector<gp_Pnt2d> uv, uvTri, uvNeighbour;
                if (!ringUv(united, uv) || !ringUv(tri, uvTri) ||
                    !ringUv(neighbour, uvNeighbour) || !simpleUv(uv))
                    return false;
                const double area = signedArea(uv);
                const double areaTri = signedArea(uvTri);
                const double areaNeighbour = signedArea(uvNeighbour);
                if (std::abs(area) <= uvAreaTol || area * areaTri <= 0.0 ||
                    area * areaNeighbour <= 0.0 ||
                    std::abs(area - areaTri - areaNeighbour) >
                        1e-7 * std::max(std::abs(area), uvAreaTol))
                    return false;

                double twiceArea = 0.0, cu = 0.0, cv = 0.0;
                for (size_t k = 0; k < uv.size(); ++k) {
                    const gp_Pnt2d& x = uv[k];
                    const gp_Pnt2d& y = uv[(k + 1) % uv.size()];
                    const double cr = x.X() * y.Y() - y.X() * x.Y();
                    twiceArea += cr;
                    cu += (x.X() + y.X()) * cr;
                    cv += (x.Y() + y.Y()) * cr;
                }
                if (std::abs(twiceArea) <= 2.0 * uvAreaTol) return false;
                cu /= 3.0 * twiceArea;
                cv /= 3.0 * twiceArea;
                BRepClass_FaceClassifier centerOwner(
                    const_cast<TopoDS_Face&>(face), gp_Pnt2d(cu, cv), faceTol);
                if (centerOwner.State() == TopAbs_OUT) return false;

                // The new boundary must not duplicate another polygon's
                // directed edge or create a third use of an undirected edge.
                for (size_t k = 0; k < united.size(); ++k) {
                    const uint32_t x = united[k];
                    const uint32_t y = united[(k + 1) % united.size()];
                    const auto own = owners.find(
                        {std::min(x, y), std::max(x, y)});
                    if (own == owners.end()) continue;
                    int outsideUses = 0;
                    for (const EdgeUse& use : own->second) {
                        if (use.poly == pi || use.poly == qi) continue;
                        ++outsideUses;
                        if (use.a == x && use.b == y) return false;
                    }
                    if (outsideUses > 1) return false;
                }

                gp_XYZ nw(0, 0, 0);
                for (size_t k = 0; k < united.size(); ++k) {
                    const auto& x = mesh.vertices[united[k]];
                    const auto& y = mesh.vertices[united[(k + 1) % united.size()]];
                    nw += gp_XYZ(x[1] * y[2] - x[2] * y[1],
                                 x[2] * y[0] - x[0] * y[2],
                                 x[0] * y[1] - x[1] * y[0]);
                }
                gp_Pnt cp; gp_Vec du, dv;
                sa.D1(cu, cv, cp, du, dv);
                gp_Vec cadN = du.Crossed(dv);
                if (reversed) cadN.Reverse();
                if (gp_Vec(nw).Magnitude() <= 1e-16 ||
                    cadN.Magnitude() <= 1e-16 || gp_Vec(nw).Dot(cadN) <= 0.0)
                    return false;

                const auto certified = triangulatePoly(mesh.vertices, united);
                if (certified.size() != united.size() - 2) return false;
                quality = 1.0;
                for (const auto& t : certified) {
                    const uint32_t ia = united[t[0]], ib = united[t[1]],
                                   ic = united[t[2]];
                    const auto& A = mesh.vertices[ia];
                    const auto& B = mesh.vertices[ib];
                    const auto& C = mesh.vertices[ic];
                    const gp_Vec ab(gp_Pnt(A[0], A[1], A[2]),
                                    gp_Pnt(B[0], B[1], B[2]));
                    const gp_Vec ac(gp_Pnt(A[0], A[1], A[2]),
                                    gp_Pnt(C[0], C[1], C[2]));
                    const gp_Vec tn = ab.Crossed(ac);
                    const gp_Pnt2d probe(
                        (uv[t[0]].X() + uv[t[1]].X() + uv[t[2]].X()) / 3.0,
                        (uv[t[0]].Y() + uv[t[1]].Y() + uv[t[2]].Y()) / 3.0);
                    BRepClass_FaceClassifier triOwner(
                        const_cast<TopoDS_Face&>(face), probe, faceTol);
                    if (triOwner.State() == TopAbs_OUT) return false;
                    gp_Pnt tp; gp_Vec tdu, tdv;
                    sa.D1(probe.X(), probe.Y(), tp, tdu, tdv);
                    gp_Vec tCad = tdu.Crossed(tdv);
                    if (reversed) tCad.Reverse();
                    if (tn.Magnitude() <= 1e-16 || tCad.Magnitude() <= 1e-16)
                        return false;
                    const double agree =
                        tn.Dot(tCad) / (tn.Magnitude() * tCad.Magnitude());
                    if (agree <= 0.0) return false;
                    quality = std::min(quality, agree);
                }
                return true;
            };

            size_t bestNeighbour = mesh.polygons.size();
            double bestQuality = -1.0;
            std::vector<uint32_t> bestRing;
            const auto tri = mesh.polygons[pi];
            for (size_t k = 0; k < tri.size(); ++k) {
                const uint32_t a = tri[k], b = tri[(k + 1) % tri.size()];
                const auto own = owners.find(
                    {std::min(a, b), std::max(a, b)});
                if (own == owners.end() || own->second.size() != 2) continue;
                for (const EdgeUse& use : own->second) {
                    if (use.poly == pi || use.a != b || use.b != a) continue;
                    std::vector<uint32_t> ring;
                    double quality = -1.0;
                    if (certify(use.poly, a, b, ring, quality) &&
                        (quality > bestQuality ||
                         (quality == bestQuality &&
                          use.poly < bestNeighbour))) {
                        bestNeighbour = use.poly;
                        bestQuality = quality;
                        bestRing = std::move(ring);
                    }
                }
            }
            if (bestNeighbour == mesh.polygons.size()) continue;
            mesh.polygons[bestNeighbour] = std::move(bestRing);
            mesh.polygons.erase(mesh.polygons.begin() + pi);
            mesh.polygonFaceId.erase(mesh.polygonFaceId.begin() + pi);
            ++mergedCertifiedComb;
            changed = true;
        }
        if (!changed) break;
    }
    if (mergedCertifiedComb) {
        mesh.polygonCornerAnchors.clear();
        refreshCertifiedTriangulations(mesh);
        dbg("generate: certified %d collapsed local-comb triangle merge(s)",
            mergedCertifiedComb);
    }
    traceFaceTopology("local comb recovery");

    dbg("generate: done (%zu verts, %zu polys)", mesh.vertexCount(),
        mesh.polygonCount());
    timingCheckpoint("cleanup");
    return mesh;
}

}  // namespace weft
