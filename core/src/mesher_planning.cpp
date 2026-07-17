#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// Detect a spherical / dome cap: a single-outer-wire face whose surface
// bulges from a base loop to a single pole, so it can mesh as a UV
// hemisphere instead of a spiralling Coons grid. The test is GEOMETRIC (the
// surface is usually a bspline, so the analytic type tells us nothing):
//
//  * exactly one outer wire, no interior holes (the base contract only
//    covers the outer rim; a hole would leak);
//  * the AZIMUTH parametric direction is periodic/closed (the base loop wraps
//    a full turn about the axis);
//  * exactly one of the two POLAR domain ends collapses to a point (the pole)
//    while the opposite end is the finite base loop;
//  * the surface is STAR-SHAPED about the base->apex axis: azimuth about the
//    axis is monotone along the azimuth param, and height along the axis is
//    monotone along the polar param — so constant-azimuth walks are straight
//    meridians and constant-polar walks are clean latitude rings.
//
// Any doubt returns false so the face keeps its existing (Coons/fallback)
// route and nothing else can regress.
bool planDomeCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const Model& model, FacePlan& plan) {
    // One outer wire only; interior wires (holes) disqualify.
    int wireCount = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) ++wireCount;
    if (wireCount != 1) return false;

    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double uspan = u1 - u0, vspan = v1 - v0;
    if (!(uspan > 1e-9) || !(vspan > 1e-9)) return false;

    // Face scale from its bounding box diagonal — the collapse/planarity
    // tolerances ride it so the test is scale-free.
    Bnd_Box bb;
    BRepBndLib::Add(face, bb);
    if (bb.IsVoid()) return false;
    double bx0, by0, bz0, bx1, by1, bz1;
    bb.Get(bx0, by0, bz0, bx1, by1, bz1);
    const double diag = gp_Pnt(bx0, by0, bz0).Distance(gp_Pnt(bx1, by1, bz1));
    if (!(diag > 1e-9)) return false;
    const double collapseTol = 0.01 * diag;

    // 3D extent of a domain iso-line (fixed param, swept along the other).
    auto isoExtent = [&](bool fixU, double fixed) {
        gp_Pnt lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
        for (int k = 0; k <= 16; ++k) {
            const double t = k / 16.0;
            const gp_Pnt p = fixU ? surf.Value(fixed, v0 + vspan * t)
                                  : surf.Value(u0 + uspan * t, fixed);
            lo.SetX(std::min(lo.X(), p.X())); hi.SetX(std::max(hi.X(), p.X()));
            lo.SetY(std::min(lo.Y(), p.Y())); hi.SetY(std::max(hi.Y(), p.Y()));
            lo.SetZ(std::min(lo.Z(), p.Z())); hi.SetZ(std::max(hi.Z(), p.Z()));
        }
        return lo.Distance(hi);
    };
    const double eU0 = isoExtent(true, u0);   // u=u0 iso
    const double eU1 = isoExtent(true, u1);   // u=u1 iso
    const double eV0 = isoExtent(false, v0);  // v=v0 iso
    const double eV1 = isoExtent(false, v1);  // v=v1 iso

    // The pole is the single collapsing polar end; its opposite is the base.
    // The perpendicular direction is the azimuth and must be periodic/closed
    // (the base loop wraps a full turn). Exactly one collapse is required.
    bool azimIsV;         // azimuth runs along V
    double polarBase, polarPole;
    const bool uPolar =                    // polar runs along U, azimuth V
        (eU0 < collapseTol) != (eU1 < collapseTol) &&
        eV0 > collapseTol && eV1 > collapseTol;
    const bool vPolar =                    // polar runs along V, azimuth U
        (eV0 < collapseTol) != (eV1 < collapseTol) &&
        eU0 > collapseTol && eU1 > collapseTol;
    if (uPolar && !vPolar) {
        if (!surf.IsVClosed()) return false;
        azimIsV = true;
        polarPole = eU0 < collapseTol ? u0 : u1;
        polarBase = eU0 < collapseTol ? u1 : u0;
    } else if (vPolar && !uPolar) {
        if (!surf.IsUClosed()) return false;
        azimIsV = false;
        polarPole = eV0 < collapseTol ? v0 : v1;
        polarBase = eV0 < collapseTol ? v1 : v0;
    } else {
        return false;  // zero, two, or ambiguous collapses: not a clean cap
    }

    // Apex = the collapsed pole point; base centroid + normal from a ring of
    // base samples. The axis runs base centroid -> apex.
    auto surfAt = [&](double polar, double azim) {
        return azimIsV ? surf.Value(polar, azim) : surf.Value(azim, polar);
    };
    const double aLo = azimIsV ? v0 : u0;
    const double aSpan = azimIsV ? vspan : uspan;
    const gp_Pnt apex = surfAt(polarPole, aLo);
    gp_Pnt baseC(0, 0, 0);
    const int NB = 24;
    std::vector<gp_Pnt> baseP(NB);
    for (int k = 0; k < NB; ++k) {
        baseP[k] = surfAt(polarBase, aLo + aSpan * k / double(NB));
        baseC.SetX(baseC.X() + baseP[k].X() / NB);
        baseC.SetY(baseC.Y() + baseP[k].Y() / NB);
        baseC.SetZ(baseC.Z() + baseP[k].Z() / NB);
    }
    gp_Vec axis(baseC, apex);
    const double axisLen = axis.Magnitude();
    if (axisLen < 0.05 * diag) return false;  // too flat to read as a dome
    axis /= axisLen;

    // Base must be roughly planar (a loop, not a bowl) and its mean radius a
    // real fraction of the face — so the axis and azimuth frame are stable.
    double baseR = 0, basePlanar = 0;
    for (const gp_Pnt& p : baseP) {
        gp_Vec r(baseC, p);
        basePlanar = std::max(basePlanar, std::abs(r.Dot(axis)));
        baseR += (r - r.Dot(axis) * axis).Magnitude() / NB;
    }
    if (baseR < 0.1 * diag) return false;
    if (basePlanar > 0.25 * baseR) return false;

    // Orthonormal azimuth frame in the base plane.
    gp_Vec eX = gp_Vec(baseC, baseP[0]) - gp_Vec(baseC, baseP[0]).Dot(axis) * axis;
    if (eX.Magnitude() < 1e-9) return false;
    eX.Normalize();
    gp_Vec eY = axis.Crossed(eX);
    if (eY.Magnitude() < 1e-9) return false;
    eY.Normalize();

    // STAR-SHAPED test: over a UV grid, azimuth about the axis must advance
    // monotonically along the azimuth param (one full turn, no backtracking)
    // and height along the axis must advance monotonically along the polar
    // param. Either failing means meridians would cross or the cap folds.
    const int GA = 24, GP = 8;  // azimuth / polar grid resolution
    auto azimuthAt = [&](const gp_Pnt& p) {
        gp_Vec r(baseC, p);
        return std::atan2(r.Dot(eY), r.Dot(eX));
    };
    auto heightAt = [&](const gp_Pnt& p) { return gp_Vec(baseC, p).Dot(axis); };
    for (int j = 1; j < GP; ++j) {  // interior polar rings only (poles skip)
        const double polar = polarBase + (polarPole - polarBase) * j / double(GP);
        double prev = 0, turn = 0;
        bool first = true;
        for (int k = 0; k <= GA; ++k) {
            const double a = azimuthAt(surfAt(polar, aLo + aSpan * k / double(GA)));
            if (!first) {
                double d = a - prev;
                while (d > M_PI) d -= 2 * M_PI;
                while (d < -M_PI) d += 2 * M_PI;
                if (d < -1e-3) return false;  // azimuth backtracks: spiral risk
                turn += d;
            }
            prev = a;
            first = false;
        }
        if (std::abs(turn) < 1.5 * M_PI) return false;  // not a full wrap
    }
    for (int k = 0; k < GA; ++k) {  // meridians: height monotone base->pole
        const double a = aLo + aSpan * (k + 0.5) / double(GA);
        double prevH = heightAt(surfAt(polarBase, a));
        for (int j = 1; j <= GP; ++j) {
            const double polar =
                polarBase + (polarPole - polarBase) * j / double(GP);
            const double h = heightAt(surfAt(polar, a));
            if (h < prevH - 0.02 * axisLen) return false;  // dips: not a dome
            prevH = h;
        }
    }

    // Collect the base rim edges (drive the meridian count) and the polar
    // boundary meridian edges (drive the latitude count). A base edge's
    // pcurve hugs the base polar value; a meridian spans the polar range at
    // fixed azimuth. The degenerate pole edge and anything else disqualifies.
    std::vector<int> baseEdges, meridianEdges;
    const double polarTol = 0.05 * std::abs(polarBase - polarPole);
    for (BRepTools_WireExplorer we(BRepTools::OuterWire(face), face); we.More();
         we.Next()) {
        const TopoDS_Edge e = we.Current();
        if (BRep_Tool::Degenerated(e)) continue;  // the pole apex edge
        const int eid = model.edges.FindIndex(e);
        if (eid < 1) return false;
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e, face, f, l);
        if (pc.IsNull()) return false;
        double pmin = 1e300, pmax = -1e300;
        for (int k = 0; k <= 8; ++k) {
            const gp_Pnt2d uv = pc->Value(f + (l - f) * k / 8.0);
            const double pv = azimIsV ? uv.X() : uv.Y();  // polar coordinate
            pmin = std::min(pmin, pv);
            pmax = std::max(pmax, pv);
        }
        const bool atBase = std::abs(pmin - polarBase) < polarTol &&
                            std::abs(pmax - polarBase) < polarTol;
        const bool spansPolar = pmax - pmin > 0.6 * std::abs(polarBase - polarPole);
        if (atBase) {
            if (std::find(baseEdges.begin(), baseEdges.end(), eid) ==
                baseEdges.end()) {
                baseEdges.push_back(eid);
            }
        } else if (spansPolar) {
            if (std::find(meridianEdges.begin(), meridianEdges.end(), eid) ==
                meridianEdges.end()) {
                meridianEdges.push_back(eid);
            }
        } else {
            return false;  // an edge that is neither base nor meridian
        }
    }
    if (baseEdges.empty()) return false;

    plan.kind = MesherKind::DomeCap;
    plan.constrains = true;
    plan.domeAzimIsV = azimIsV;
    plan.domePolarBase = polarBase;
    plan.domePolarPole = polarPole;
    plan.uEdges = baseEdges;       // azimuth ring -> radial (meridian count)
    plan.vEdges = meridianEdges;   // polar meridian -> axial (latitude count)
    return true;
}

FacePlan planFace(int fid, const Model& model, const Analysis& analysis,
                  const GenerationSettings& settings,
                  GenerationCache* cache) {
    const TopoDS_Face face = TopoDS::Face(model.faces(fid));
    const FaceMeshSettings& s = settings.forFace(fid);
    const FaceInfo& info = analysis.faces[fid - 1];
    BRepAdaptor_Surface surf(face);
    FacePlan plan;

    if (const char* uvFaces = std::getenv("WEFT_UV_FACES")) {
        std::stringstream requested(uvFaces);
        std::string token;
        bool trace = false;
        while (std::getline(requested, token, ',')) {
            if (std::atoi(token.c_str()) == fid) trace = true;
        }
        if (trace) {
            dbg("uvface %d: type=%d bounds u=[%.6g,%.6g] v=[%.6g,%.6g]",
                fid, int(surf.GetType()), surf.FirstUParameter(),
                surf.LastUParameter(), surf.FirstVParameter(),
                surf.LastVParameter());
            for (int eid : info.edgeIds) {
                const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
                double f = 0.0, l = 0.0;
                Handle(Geom2d_Curve) pc =
                    BRep_Tool::CurveOnSurface(edge, face, f, l);
                if (pc.IsNull()) {
                    dbg("uvface %d edge %d: no pcurve", fid, eid);
                    continue;
                }
                double eu0 = 1e300, eu1 = -1e300;
                double ev0 = 1e300, ev1 = -1e300;
                for (int k = 0; k <= 16; ++k) {
                    const gp_Pnt2d uv =
                        pc->Value(f + (l - f) * k / 16.0);
                    eu0 = std::min(eu0, uv.X());
                    eu1 = std::max(eu1, uv.X());
                    ev0 = std::min(ev0, uv.Y());
                    ev1 = std::max(ev1, uv.Y());
                }
                dbg("uvface %d edge %d: u=[%.6g,%.6g] v=[%.6g,%.6g] "
                    "closed=%d deg=%d",
                    fid, eid, eu0, eu1, ev0, ev1,
                    BRep_Tool::IsClosed(edge, face) ? 1 : 0,
                    BRep_Tool::Degenerated(edge) ? 1 : 0);
            }
        }
    }

    // Geometry-only probe results memoize in the cache (the classifier
    // calls dominate planning cost and never change for a model).
    auto revCovers = [&] {
        if (cache) {
            auto it = cache->revolutionCovers.find(fid);
            if (it != cache->revolutionCovers.end()) return it->second;
        }
        bool v = revolutionCovers(face);
        if (cache) cache->revolutionCovers[fid] = v;
        return v;
    };
    auto geomRev = [&] {
        // This probe exists for CAD writers that hide a revolve behind a
        // spline/offset surface. A surface of extrusion can also resemble
        // concentric closed samples over a small trimmed chart, but treating
        // it as a revolve discards its real boundary layout (MP9 face 424).
        // Typed analytic revolves use isClosedRevolution() instead.
        switch (surf.GetType()) {
            case GeomAbs_BSplineSurface:
            case GeomAbs_BezierSurface:
            case GeomAbs_OffsetSurface:
            case GeomAbs_OtherSurface: break;
            default: return false;
        }
        if (cache) {
            auto it = cache->geomRevolution.find(fid);
            if (it != cache->geomRevolution.end()) return it->second;
        }
        bool v = isGeometricClosedRevolution(surf);
        if (cache) cache->geomRevolution[fid] = v;
        return v;
    };
    bool coonsReflex = false;  // flat outline with a strong reflex bend
    int coonsEffectiveRotate = s.coonsRotate;
    auto coonsOk = [&](CoonsPatch& patch) {
        // Patch construction is cheap; a memoized NEGATIVE skips it (and
        // the probes); a positive still rebuilds the (cheap) patch data.
        // Validity is rotation-independent, so the memo stays keyed by
        // face alone.
        if (cache) {
            auto it = cache->coonsValid.find(fid);
            if (it != cache->coonsValid.end() && !it->second) return false;
        }
        const char* why = nullptr;
        bool reflex = false;
        bool v = makeCoonsPatch(face, model, patch, s.coonsRotate, &why,
                                &reflex);
        // Four-sided trims on analytic drums often cannot use the strict
        // open-band mesher (their side curves are not full-height isos), but
        // they still need the primitive's semantic axes. Wire start order is
        // arbitrary, so canonicalize the default Coons orientation: grid U
        // follows surface U/azimuth and grid V follows the axis/profile. This
        // also keeps the GPU UV proxy aligned with the released CPU mesh.
        const GeomAbs_SurfaceType st = surf.GetType();
        const bool analyticDrum =
            st == GeomAbs_Cylinder || st == GeomAbs_Cone ||
            st == GeomAbs_SurfaceOfRevolution ||
            st == GeomAbs_SurfaceOfExtrusion;
        if (v && analyticDrum && s.coonsRotate == 0 &&
            !patch.collapsedLast) {
            auto axisTravel = [&](int side) {
                double du = 0.0, dv = 0.0;
                gp_Pnt2d prev = patch.side(side, 0.0);
                for (int k = 1; k <= 8; ++k) {
                    const gp_Pnt2d cur = patch.side(side, k / 8.0);
                    du += std::abs(cur.X() - prev.X());
                    dv += std::abs(cur.Y() - prev.Y());
                    prev = cur;
                }
                return std::array<double, 2>{du, dv};
            };
            const auto a0 = axisTravel(0);
            const auto a1 = axisTravel(1);
            if (a0[1] > a0[0] && a1[0] > a1[1]) {
                CoonsPatch canonical;
                const char* canonicalWhy = nullptr;
                bool canonicalReflex = false;
                if (makeCoonsPatch(face, model, canonical, 1,
                                   &canonicalWhy, &canonicalReflex)) {
                    patch = std::move(canonical);
                    reflex = canonicalReflex;
                    coonsEffectiveRotate = 1;
                    dbg("plan face %d: canonical drum coons axes", fid);
                }
            }
        }
        if (!v && why) dbg("coons: face %d rejected: %s", fid, why);
        if (v && reflex) dbg("coons: face %d has a reflex flat outline", fid);
        coonsReflex = v && reflex;
        if (cache) {
            cache->coonsValid[fid] = v;
            cache->coonsReflex[fid] = coonsReflex;
        }
        return v;
    };
    auto coonsChainsCompatible = [&](CoonsPatch& patch) {
        if (!patch.chained()) return true;
        const GeomAbs_SurfaceType st = surf.GetType();
        const bool analyticDrum =
            st == GeomAbs_Cylinder || st == GeomAbs_Cone ||
            st == GeomAbs_SurfaceOfRevolution ||
            st == GeomAbs_SurfaceOfExtrusion;
        // A trimmed analytic drum keeps its canonical UV patch: its surface
        // directions are meaningful even when a T-junction splits one rim.
        if (analyticDrum) return true;

        // STEP edge-piece counts are bookkeeping, not patch topology. A
        // perfectly good shell can have one long rail split at several
        // assembly T-junctions while its opposite rail remains one edge.
        // Preflight the geometric station corridor that the Coons mesher
        // actually consumes instead: a single four-sided domain, monotone
        // arc-length chains, a dense in-face lattice, and one non-zero
        // Jacobian orientation throughout. Opposite sides may carry different
        // SOLVED station totals; meshCoonsGrid preserves both natural borders
        // and arc-resamples only the interior scaffold between them.
        struct CoonsProbe {
            bool ok = false;
            const char* why = "unknown";
            std::array<double, 4> sideLength{};
            double railRatio = 0.0;
            double jacobianMargin = 0.0;
        };
        auto preflight = [&](const CoonsPatch& candidate) {
            CoonsProbe result;
            if (candidate.collapsedLast || candidate.stubEdgeId > 0) {
                result.why = "not four complete sides";
                return result;
            }
            if (!candidate.holeWires.empty()) {
                result.why = "chained patch has holes";
                return result;
            }
            int wireCount = 0;
            for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
                ++wireCount;
            }
            if (wireCount != 1) {
                result.why = "not one outer wire";
                return result;
            }
            for (const auto& side : candidate.chain) {
                if (side.empty()) {
                    result.why = "empty side chain";
                    return result;
                }
            }

            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface.IsNull()) {
                result.why = "surface missing";
                return result;
            }

            // Sample by the chain's 3D arc-length parameter (CoonsPatch::side
            // already walks its pieces by length). Projection onto the end
            // chord must advance monotonically. This admits arched ends up to
            // a semicircle while rejecting loops/backtracking outlines whose
            // normalized stations cannot be paired without crossing.
            constexpr int kSideSamples = 32;
            std::array<std::array<gp_Pnt, kSideSamples + 1>, 4> sidePoints;
            for (int sd = 0; sd < 4; ++sd) {
                try {
                    for (int k = 0; k <= kSideSamples; ++k) {
                        const gp_Pnt2d uv =
                            candidate.side(sd, double(k) / kSideSamples);
                        sidePoints[sd][k] = surface->Value(uv.X(), uv.Y());
                    }
                } catch (const Standard_Failure&) {
                    result.why = "side sampling failed";
                    return result;
                }
                double arc = 0.0;
                for (int k = 1; k <= kSideSamples; ++k) {
                    arc += sidePoints[sd][k - 1].Distance(sidePoints[sd][k]);
                }
                const gp_Vec chord(sidePoints[sd][0],
                                   sidePoints[sd][kSideSamples]);
                const double chord2 = chord.SquareMagnitude();
                if (!std::isfinite(arc) || arc <= 1e-12 ||
                    chord2 <= 0.0144 * arc * arc) {  // chord < 12% of arc
                    result.why = "side chain loops back";
                    return result;
                }
                double furthest = 0.0;
                for (int k = 1; k <= kSideSamples; ++k) {
                    const double progress =
                        gp_Vec(sidePoints[sd][0], sidePoints[sd][k])
                            .Dot(chord) /
                        chord2;
                    // Pcurve/STEP noise may wobble a few percent, but a real
                    // reversal is not a viable station rail.
                    if (progress < -0.05 || progress > 1.05 ||
                        progress + 0.05 < furthest) {
                        result.why = "non-monotone side chain";
                        return result;
                    }
                    furthest = std::max(furthest, progress);
                }
                result.sideLength[sd] = arc;
            }

            // Opposite rails need comparable geometric extent, not matching
            // STEP piece counts. The transition-strip path can absorb a large
            // solved-count difference, but an 8:1 geometric mismatch is no
            // longer a four-sided station corridor in practice.
            for (int sd = 0; sd < 2; ++sd) {
                const double a = result.sideLength[sd];
                const double b = result.sideLength[sd + 2];
                if (std::max(a, b) > 8.0 * std::max(1e-12, std::min(a, b))) {
                    result.why = "opposite side extents diverge";
                    return result;
                }
            }

            // Dense UV probes prove that normalized opposite stations form a
            // single inside corridor and keep one orientation. This is the
            // same mapping the mesher seeds before its 3D blend/projection.
            constexpr int kGrid = 12;
            const double tol = BRep_Tool::Tolerance(face);
            const double uSpan =
                std::max(1e-12, candidate.outerBox[1] - candidate.outerBox[0]);
            const double vSpan =
                std::max(1e-12, candidate.outerBox[3] - candidate.outerBox[2]);
            const double areaScale = uSpan * vSpan;
            const double h = 0.24 / kGrid;
            double referenceJacobian = 0.0;
            double minAbsJacobian = 1e300;
            double maxAbsJacobian = 0.0;
            for (int j = 0; j < kGrid; ++j) {
                const double b = (j + 0.5) / kGrid;
                for (int i = 0; i < kGrid; ++i) {
                    const double a = (i + 0.5) / kGrid;
                    const gp_Pnt2d uv = candidate.uv(a, b);
                    BRepClass_FaceClassifier cls(
                        const_cast<TopoDS_Face&>(face), uv, tol);
                    if (cls.State() == TopAbs_OUT) {
                        result.why = "dense interior probe outside face";
                        return result;
                    }
                    const gp_Pnt2d am = candidate.uv(a - h, b);
                    const gp_Pnt2d ap = candidate.uv(a + h, b);
                    const gp_Pnt2d bm = candidate.uv(a, b - h);
                    const gp_Pnt2d bp = candidate.uv(a, b + h);
                    const double jac =
                        (ap.X() - am.X()) * (bp.Y() - bm.Y()) -
                        (ap.Y() - am.Y()) * (bp.X() - bm.X());
                    const double aj = std::abs(jac);
                    if (!std::isfinite(jac) ||
                        aj <= 1e-10 * areaScale / (kGrid * kGrid)) {
                        result.why = "degenerate Coons Jacobian";
                        return result;
                    }
                    if (referenceJacobian == 0.0) referenceJacobian = jac;
                    if (jac * referenceJacobian <= 0.0) {
                        result.why = "Coons Jacobian changes orientation";
                        return result;
                    }
                    minAbsJacobian = std::min(minAbsJacobian, aj);
                    maxAbsJacobian = std::max(maxAbsJacobian, aj);
                }
            }
            if (maxAbsJacobian <= 0.0 ||
                minAbsJacobian < 1e-7 * maxAbsJacobian) {
                result.why = "Coons Jacobian pinches";
                return result;
            }
            result.jacobianMargin = minAbsJacobian / maxAbsJacobian;
            result.railRatio =
                (result.sideLength[0] + result.sideLength[2]) /
                std::max(1e-12,
                         result.sideLength[1] + result.sideLength[3]);
            result.ok = true;
            result.why = "ok";
            return result;
        };

        CoonsProbe chosen = preflight(patch);
        int chosenRotate = coonsEffectiveRotate;
        // With the default rotation, canonicalize a shell so sides 0/2 are
        // its long rails and sides 1/3 are the arched ends. Grid columns then
        // cross the shell between rails instead of following an arbitrary STEP
        // wire start. An explicit rotate remains authoritative.
        const bool explicitRotate =
            settings.perFace.count(fid) > 0 &&
            s.coonsRotate != settings.defaults.coonsRotate;
        if (!explicitRotate) {
            CoonsPatch alternate;
            const int altRotate = (coonsEffectiveRotate + 1) % 4;
            if (makeCoonsPatch(face, model, alternate, altRotate)) {
                CoonsProbe alternateProbe = preflight(alternate);
                if (alternateProbe.ok &&
                    (!chosen.ok || alternateProbe.railRatio >
                                       chosen.railRatio * 1.05)) {
                    patch = std::move(alternate);
                    chosen = alternateProbe;
                    chosenRotate = altRotate;
                }
            }
        }
        if (!chosen.ok) {
            dbg("coons: face %d rejected by geometric preflight: %s "
                "chains=%zu/%zu/%zu/%zu lengths=%.3g/%.3g/%.3g/%.3g",
                fid, chosen.why, patch.chain[0].size(),
                patch.chain[1].size(), patch.chain[2].size(),
                patch.chain[3].size(), chosen.sideLength[0],
                chosen.sideLength[1], chosen.sideLength[2],
                chosen.sideLength[3]);
            return false;
        }
        coonsEffectiveRotate = chosenRotate;
        dbg("coons: face %d geometric preflight ok rotate=%d "
            "lengths=%.3g/%.3g/%.3g/%.3g rail-ratio=%.3g jac=%.3g",
            fid, chosenRotate, chosen.sideLength[0], chosen.sideLength[1],
            chosen.sideLength[2], chosen.sideLength[3], chosen.railRatio,
            chosen.jacobianMargin);
        return true;
    };

    auto finishRevolution = [&]() {
        plan.kind = MesherKind::RevolutionGrid;
        // Interior insert wires must never contribute rim candidates —
        // a slot border classified as a rim contaminates the emitted
        // rim row with interior points. Open-band side edges are the
        // row contract, not rims, and stay out for the same reason.
        std::set<int> nonRim(plan.bandSides.begin(), plan.bandSides.end());
        for (const auto& w : plan.insertWires) {
            nonRim.insert(w.begin(), w.end());
        }
        std::vector<int> rimCandidates;
        for (int eid : info.edgeIds) {
            if (!nonRim.count(eid)) rimCandidates.push_back(eid);
        }
        collectIsoEdges(face, model, rimCandidates, plan,
                        /*skipNonIso=*/true);
        // Rim membership by CONNECTIVITY (shared with the loftability
        // gate): wavy chains misfile under nearest-end tests. The
        // chains replace whatever iso classification put in uEdges;
        // they also drive the density SUM constraint (multi-edge rims
        // must total the opposite rim, not copy its per-edge count).
        if (rimChains(face, model, nonRim, plan.rimLow, plan.rimHigh)) {
            plan.uEdges = plan.rimLow;
            plan.uEdges.insert(plan.uEdges.end(), plan.rimHigh.begin(),
                               plan.rimHigh.end());
        } else {
            plan.rimLow.clear();
            plan.rimHigh.clear();
        }
        // The sides drive the row count exactly as a closed band's seam
        // does — never the rim-chain u-iso pieces collectIsoEdges saw.
        if (!plan.bandSides.empty()) plan.vEdges = plan.bandSides;
        if (plan.uEdges.empty()) plan.constrains = false;
        if (!s.linkRims && plan.uEdges.size() == 2 &&
            plan.bandSides.empty()) {
            plan.linkRims = false;  // the taper cannot span an open band
        }
        // A full-wrap rim cut through by a notch (one plain rim, one rim
        // whose walls drop to a floor): straight uniform lattice + rim-open
        // cut, driven by the plain rim. Closed bands only (open bands own
        // the partial-wrap version above).
        if (plan.bandSides.empty()) {
            const int pr = castellatedRimBand(face, surf, model, plan.rimLow,
                                              plan.rimHigh);
            if (pr) {
                plan.castellated = true;
                plan.plainRimEdge = pr;
                dbg("plan face %d: full-wrap castellated rim, plain edge %d",
                    fid, pr);
            }
        }
    };

    // Partial-wrap revolution band plan: two full-height u-iso sides
    // bound the band and carry the row contract the way a seam would;
    // the flat full-span rim drives the column count (bandDriver) and
    // the castellated chain is boolean-cut at mesh time.
    auto tryOpenBand = [&]() {
        std::vector<int> sides;
        std::vector<std::vector<int>> inserts;
        bool repeatedNotched = false;
        int repeatedFeatureCount = 0;
        if (!openBandSides(face, surf, model, sides, &repeatedNotched,
                           &repeatedFeatureCount) ||
            !edgesHugRimsOrInserts(face, surf, model, inserts, &sides,
                                   repeatedNotched)) {
            return false;
        }
        // Strictly-interior wires (a slot or hole through the wall) mesh
        // as boolean-cut inserts: straight full-height columns with the
        // covered cells deleted and the cutout webbed as a local collar,
        // instead of the coons cutout that fanned the slot ends.
        plan.insertWires = std::move(inserts);
        plan.bandSides = std::move(sides);
        finishRevolution();
        if (plan.rimLow.empty() || plan.rimHigh.empty()) {
            plan = FacePlan();  // chains failed: keep the default route
            return false;
        }
        // The boolean-cut insert path needs CLEAN flat rims: a stepped
        // or castellated rim would entangle its transition machinery
        // with the cutout rows, and the mesher would fail to the floor —
        // worse than the coons cutout those walls take today. Gate at
        // plan time so they keep their existing route.
        if (!plan.insertWires.empty()) {
            const double vspanG = std::max(
                1e-12, surf.LastVParameter() - surf.FirstVParameter());
            for (const std::vector<int>* rim :
                 {&plan.rimLow, &plan.rimHigh}) {
                for (int eid : *rim) {
                    const TopoDS_Edge e = TopoDS::Edge(model.edges(eid));
                    if (BRep_Tool::Degenerated(e)) continue;
                    double f, l;
                    Handle(Geom2d_Curve) pc =
                        BRep_Tool::CurveOnSurface(e, face, f, l);
                    if (pc.IsNull()) {
                        plan = FacePlan();
                        return false;
                    }
                    double ev0 = 1e300, ev1 = -1e300;
                    for (int k = 0; k <= 8; ++k) {
                        const double vv =
                            pc->Value(f + (l - f) * k / 8.0).Y();
                        ev0 = std::min(ev0, vv);
                        ev1 = std::max(ev1, vv);
                    }
                    if (ev1 - ev0 > 0.02 * vspanG) {
                        plan = FacePlan();  // rim not flat: keep coons
                        return false;
                    }
                }
            }
        }
        const double uspanB = std::max(
            1e-12, surf.LastUParameter() - surf.FirstUParameter());
        const double vspanB = std::max(
            1e-12, surf.LastVParameter() - surf.FirstVParameter());
        plan.bandWrapFrac = uspanB / (2.0 * M_PI);
        // Column driver: a lone rim edge that hugs its rim level across
        // the whole wrap (the plain circle of a castellated barrel).
        // Castellated chains never drive.
        auto chainDriver = [&](const std::vector<int>& chain) {
            if (chain.size() != 1) return 0;
            const TopoDS_Edge e = TopoDS::Edge(model.edges(chain[0]));
            if (BRep_Tool::Degenerated(e)) return 0;
            double f, l;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(e, face, f, l);
            if (pc.IsNull()) return 0;
            double eu0 = 1e300, eu1 = -1e300;
            double ev0 = 1e300, ev1 = -1e300;
            for (int k = 0; k <= 16; ++k) {
                gp_Pnt2d uv = pc->Value(f + (l - f) * k / 16.0);
                eu0 = std::min(eu0, uv.X());
                eu1 = std::max(eu1, uv.X());
                ev0 = std::min(ev0, uv.Y());
                ev1 = std::max(ev1, uv.Y());
            }
            if (ev1 - ev0 > 0.02 * vspanB) return 0;  // wavy
            if (eu1 - eu0 < 0.98 * uspanB) return 0;  // partial
            return chain[0];
        };
        plan.bandDriver = chainDriver(plan.rimHigh);
        if (!plan.bandDriver) plan.bandDriver = chainDriver(plan.rimLow);
        if (repeatedNotched) {
            // Deep repeated cone/chamfer seams are monotone trim combs, but
            // the open-band WAVE transition has to bridge the entire wavy
            // rim in one strip and can fold across a groove.  Retain the
            // open-band density contract (plain rim columns + two side row
            // contracts) while using the exact clipped primitive lattice;
            // its drum-comb localizer removes each endpoint station outside
            // the owning groove corridor.
            plan.orthogonalTrimGrid = true;
            plan.orthogonalDrumComb = true;
            plan.orthogonalDriverU = plan.bandDriver;
            plan.orthogonalDriverV = plan.bandSides.empty()
                                         ? 0 : plan.bandSides.front();
            plan.orthogonalEdges = info.edgeIds;
            plan.orthogonalFeatureCount = repeatedFeatureCount;
            dbg("plan face %d: repeated-notch cone -> localized revolution "
                "grid", fid);
        }
        dbg("plan face %d: open revolution band, sides %d/%d, driver %d "
            "wrap %.3f",
            fid, plan.bandSides[0], plan.bandSides[1], plan.bandDriver,
            plan.bandWrapFrac);
        return true;
    };

    // The user can force a strategy; if it can't build on this face the
    // plan degrades to plain triangulation so the choice is visible.
    if (s.forceMesher > 0) {
        MesherKind want = MesherKind(s.forceMesher - 1);
        switch (want) {
            case MesherKind::RevolutionGrid:
                // Forced: also accept u-closed freeform surfaces (revolved
                // bsplines and the like) that the auto path won't touch,
                // and partial wraps that route as open bands.
                if (isClosedRevolution(surf) || surf.IsUClosed() ||
                    geomRev()) {
                    finishRevolution();
                    return plan;
                }
                if (tryOpenBand()) return plan;
                break;
            case MesherKind::DiskCap: {
                int capEdgeId = 0;
                if (surf.GetType() == GeomAbs_Plane &&
                    boundingCircle(face, plan.circ, capEdgeId, model)) {
                    plan.kind = MesherKind::DiskCap;
                    plan.uEdges.push_back(capEdgeId);
                    plan.constrains = true;
                    return plan;
                }
                break;
            }
            case MesherKind::RingJunction:
                if (planRingJunction(face, model, plan)) return plan;
                break;
            case MesherKind::PlanarGrid:
                if (parametricGridFits(face, surf, std::max(1, s.gridU),
                                       std::max(1, s.gridV))) {
                    plan.kind = want;
                    collectIsoEdges(face, model, info.edgeIds, plan);
                    if (plan.uEdges.size() != 2 || plan.vEdges.size() != 2) {
                        plan.constrains = false;
                    }
                    return plan;
                }
                break;
            case MesherKind::MinimalNGon:
                // Any planar face can go minimal: single wire -> one exact
                // boundary n-gon, holes -> bridged web, zero interior verts.
                if (planMinimalPlanar(face, surf, model, plan)) return plan;
                break;
            case MesherKind::CoonsGrid: {
                CoonsPatch patch;
                if (coonsOk(patch)) {
                    plan.kind = MesherKind::CoonsGrid;
                    plan.constrains = true;
                    plan.insertWires = patch.holeWires;
                    insertCountFloors(face, patch, plan.insertMinU,
                                      plan.insertMinV);
                    if (patch.chained()) {
                        for (int i = 0; i < 4; ++i) {
                            for (const auto& pce : patch.chain[i]) {
                                plan.coonsSides[i].push_back(pce.edgeId);
                            }
                        }
                    } else {
                        plan.uEdges = {patch.edgeIds[0], patch.edgeIds[2]};
                        plan.vEdges = {patch.edgeIds[1], patch.edgeIds[3]};
                    }
                    return plan;
                }
                break;
            }
            case MesherKind::AnnulusRing:
                // Forced: no ring-shape gate — the user asked for the band.
                // The open (C-shaped) single-wire ring routes here too.
                if (planAnnulusCRing(face, model, plan)) return plan;
                if (planAnnulus(face, model, plan, /*requireRing=*/false)) {
                    return plan;
                }
                break;
            case MesherKind::PlateWeb:
                if (planPlateWeb(face, surf, model, plan,
                                 /*requireRoundHoles=*/false)) {
                    return plan;
                }
                break;
            case MesherKind::QuadFill:
                if (planQuadFill(face, surf, model, plan)) return plan;
                break;
            case MesherKind::RibbonSweep:
                // Forced: skip the auto ribbonDetect gate (the user asked for
                // it) but still build the quad-fill plan the ribbon mesher
                // reads its edges/density from. meshRibbonSweep falls straight
                // back to quad-fill if the rails don't resolve.
                if (planQuadFill(face, surf, model, plan)) {
                    plan.kind = MesherKind::RibbonSweep;
                    return plan;
                }
                break;
            case MesherKind::RailLadder:
                if (planRailLadder(face, model, plan)) return plan;
                break;
            case MesherKind::DomeCap:
                if (planDomeCap(face, surf, model, plan)) return plan;
                break;
            case MesherKind::QuadDominant:
                plan.kind = MesherKind::Fallback;
                plan.forceFallbackQuads = 1;
                return plan;
            case MesherKind::Fallback:
                plan.kind = MesherKind::Fallback;
                plan.forceFallbackQuads = 0;
                return plan;
        }
        plan.kind = MesherKind::Fallback;
        return plan;
    }

    // Sliver fillet guard: a fillet whose blend radius is below the chord
    // tolerance is a micro-round smaller than the deviation the mesh is
    // allowed to make anyway (the teleporter's 0.096 mm torus fillets). A
    // transfinite Coons grid across such a near-degenerate strip folds —
    // the untangler cannot separate rows thinner than the weld quantum —
    // and it carries no shape a single boundary web can't. Demote it to a
    // minimal n-gon: its border samples the neighbours' solved counts, so
    // it still welds, and a boundary web cannot invert. Generalizes to any
    // model's micro-fillets: catches the teleporter's 0.096 mm rounds and
    // the STEP board's 0.06-0.125 mm ones, while the 0.15 mm gate sits
    // safely below every genuine fillet (foam's smallest is 0.217 mm).
    if (info.isFillet && info.radius > 0.0 &&
        info.radius < 1.5 * std::max(1e-9, s.chordTolerance) &&
        collectPlanarLoops(face, surf, model, plan, /*requirePlane=*/false)) {
        plan.kind = MesherKind::MinimalNGon;
        dbg("plan face %d: sliver fillet r=%.4g (< %.4g) -> minimal n-gon",
            fid, info.radius, 1.5 * s.chordTolerance);
        return plan;
    }

    // Game topology (plan §1): a curved face carrying a boolean CUTOUT hole
    // ships as a single boundary n-gon with each cutout as a LOCAL bridged
    // hole (zero interior verts, no spanning support loops) instead of a
    // revolution/ladder grid whose columns are shattered into one span per
    // cutout rim edge. Only when minimal (game) mode is on AND the face
    // carries a real interior cutout wire (>1 wire) — a clean structural
    // wall (single wire) keeps its revolution/coons route and its curvature.
    // The minimal web is UV-2D and surface-type agnostic; if the hole-bridge
    // ear-clip can't build (e.g. a periodic seam self-crosses) it returns
    // false and the face falls through to its normal route untouched.
    if (s.minimal) {
        int wireCount = 0;
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            ++wireCount;
        }
        // A CYLINDER wall that actually wraps must NOT flatten to an n-gon
        // (the user's primitive-first order: a slotted barrel is a cylinder
        // with a local cutout, not a flat panel) — it keeps its curvature on
        // the revolution / coons path below. A nearly-flat cylinder ring, and
        // any shallow cone/bspline (foam's dished spray-can rings), still
        // collapse. Only a CURVED cylinder is excluded from the grab.
        const bool curvedCyl =
            surf.GetType() == GeomAbs_Cylinder &&
            !isGeometricallyFlat(face, surf, /*flatFrac=*/0.08);
        if (wireCount > 1 && !curvedCyl &&
            planMinimalPlanar(face, surf, model, plan, /*requirePlane=*/false)) {
            dbg("plan face %d: curved cutout -> minimal n-gon (%d wires, "
                "local holes)",
                fid, wireCount);
            return plan;
        }
    }

    // revCovers is NOT required: a pipe-saddle band legitimately fails
    // fixed-v coverage — edgesHugRimsOrInserts checks between-chain
    // coverage itself, so wavy-rim bands loft instead of falling to a
    // coons patch (which degenerates on a full-period chart).
    if (isClosedRevolution(surf) || geomRev()) {
        std::vector<std::vector<int>> inserts;
        if (edgesHugRimsOrInserts(face, surf, model, inserts)) {
            plan.insertWires = std::move(inserts);
            finishRevolution();
            return plan;
        }
    }

    int capEdgeId = 0;
    if (surf.GetType() == GeomAbs_Plane &&
        boundingCircle(face, plan.circ, capEdgeId, model)) {
        plan.kind = MesherKind::DiskCap;
        plan.uEdges.push_back(capEdgeId);  // ring count = edge subdivisions
        plan.constrains = true;
        return plan;
    }

    // Explicit specialized-control override: when the user typed a control
    // that NAMES a specialized planner — a per-face `radial` asks for radial
    // spokes on a flat ring, a per-face `rings` asks for concentric
    // ring-junction loops — route to that planner BEFORE the minimal-n-gon
    // grab below. Otherwise the per-face override falls into the minimal
    // branch and silently demotes the ring/junction to one flat n-gon,
    // dropping the control entirely. Gated on the field DIFFERING from the
    // model default (the same "explicit count" test solveDensity uses for
    // its pins), so a face without such an override keeps the unchanged
    // default route and default output stays byte-identical.
    if (settings.perFace.count(fid)) {
        const FaceMeshSettings& dfl = settings.defaults;
        // FLAT rings only: a radial override on a curved slotted wall
        // (the barrel fixture's cylinder, a sleeve with a slot) must not
        // yank the face out of its revolution/open-band route into
        // washer spokes — that meshed the wall as an inverted bowtie fan
        // the moment any non-default radial was set.
        if (surf.GetType() == GeomAbs_Plane && s.radial != dfl.radial) {
            // An explicit radial on a flat ring: mesh it as radial spokes
            // (open C-ring or full washer), not a boundary n-gon. Solve
            // pins the shared rim group to `radial`, so the neighbour rims
            // densify with it and the band stays watertight.
            if (planAnnulusCRing(face, model, plan)) return plan;
            if (planAnnulus(face, model, plan, /*requireRing=*/false)) {
                return plan;
            }
        }
        if (s.junctionRings != dfl.junctionRings &&
            planRingJunction(face, model, plan)) {
            return plan;
        }
    }

    // Game-topology minimal, explicit per-face override: the user asked
    // for THIS face's boundary shape, so it wins even over the junction
    // patterns below.
    if (s.minimal && settings.perFace.count(fid) &&
        planMinimalPlanar(face, surf, model, plan)) {
        return plan;
    }

    // A notch cut clean through a flat ring leaves an open (C-shaped)
    // annulus band: two concentric arc rails joined by two walls. It reads
    // as a flat n-gon, but the neighbour cylinder/bore rims carry column
    // azimuths (pinned) that this face's arcs share — so it meshes as
    // radial quad spokes aligned to those columns, not a flat boundary
    // n-gon. Wins over the default minimal grab (but not an explicit
    // per-face minimal override handled just above).
    if (planAnnulusCRing(face, model, plan)) return plan;

    // Minimal n-gon owns EVERY flat face it can express when the mode is
    // on (the topology policy: big flats are n-gons, quads go to curves;
    // triangulation is an export option). The junction patterns below
    // only see flat faces when minimal is off or can't build the face.
    if (s.minimal && planMinimalPlanar(face, surf, model, plan)) {
        if (getenv("WEFT_FLAT_DEBUG")) {
            dbg("plan face %d: minimal-ngon (surf type %d)", fid,
                (int)surf.GetType());
        }
        return plan;
    }

    if (planRingJunction(face, model, plan)) return plan;

    // Auto picks stay conservative: the annulus band only for actual
    // concentric rings, the plate web only for compact (bolt-style) holes.
    // Everything else keeps the fallback unless the user forces a mesher.
    if (planAnnulus(face, model, plan, /*requireRing=*/true)) return plan;

    if (planPlateWeb(face, surf, model, plan, /*requireRoundHoles=*/true)) {
        return plan;
    }

    // Curved surfaces skip the parametric grid on auto: its border rows
    // sample the SURFACE uniformly, which never lands vertex-for-vertex on
    // a neighbour's sampling of the shared edge. Four-sided curved faces
    // fall through to the Coons patch below, whose border rows evaluate on
    // the 3D edge curves (weld-exact); the rest conform as freeform.
    if (surf.GetType() == GeomAbs_Plane &&
        parametricGridFits(face, surf, std::max(1, s.gridU),
                           std::max(1, s.gridV))) {
        // Only a plain 2u+2v rectangle ties its grid to its edges. On auto
        // anything else must NOT grid: an unconstrained grid's coarse
        // border (two verts a side at 1x1) can never zip against a denser
        // neighbour — conform merges verts but cannot split edges. A
        // rectangle whose side is split into collinear edges (T-junction)
        // falls through to the fallback, whose shared-edge nodes come from
        // the model-wide triangulation and match the neighbour exactly.
        collectIsoEdges(face, model, info.edgeIds, plan);
        if (plan.uEdges.size() == 2 && plan.vEdges.size() == 2) {
            plan.kind = MesherKind::PlanarGrid;
            if (info.isFillet) {
                plan.isFillet = true;
                // The blend arc runs along u for a cylinder strip and along
                // the minor circle (v) for a toroidal corner patch.
                plan.acrossIsU = surf.GetType() == GeomAbs_Cylinder;
            }
            return plan;
        }
        plan.uEdges.clear();
        plan.vEdges.clear();
    }

    // Split rectangle / side-notched primitive band. This must precede
    // Coons: a many-piece side is CAD bookkeeping, not a request to pull
    // every piece toward a patch centre. One global UV lattice preserves
    // the primitive spans and clips only the cells outside the trim.
    {
        FacePlan orth;
        if (planOrthogonalTrimGrid(face, surf, model, orth)) {
            dbg("plan face %d: orthogonal trim grid u=%zu v=%zu", fid,
                orth.uEdges.size(), orth.vEdges.size());
            plan = std::move(orth);
            return plan;
        }
    }

    // Spherical / dome cap (a single-wire revolution-like bspline that bulges
    // from a base loop to a pole): mesh as a clean UV hemisphere — latitude
    // rings + straight meridians + a pole fan — instead of the Coons grid,
    // which spirals inward to a messy centre. Tightly gated (planDomeCap bails
    // on any doubt), so only genuine caps divert here; everything else keeps
    // its existing route byte-for-byte.
    if (planDomeCap(face, surf, model, plan)) {
        dbg("plan face %d: dome cap (base %zu edges, meridian %zu)", fid,
            plan.uEdges.size(), plan.vEdges.size());
        return plan;
    }

    // Primitive-priority (the artist's rule: cylinder > curves >
    // interior faces): a PARTIAL-WRAP revolution wall carrying interior
    // cutout wires (a slot or hole through the wall) is a primitive
    // first and a cutout second — route it to the open band's
    // boolean-cut insert path BEFORE coons can claim it as a cutout
    // patch, which fans the slot ends and lays full-width rows across
    // the primitive. Single-wire walls keep their existing order
    // (coons/ladder first) byte-for-byte.
    {
        int wireCount = 0;
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            ++wireCount;
        }
        if (wireCount > 1 && tryOpenBand()) return plan;
    }

    // Primitive-priority, single-wire drums: a partial-wrap cylinder /
    // cone / revolution wall subtending a REAL arc is a primitive
    // segment — booleans split drums at meridians all the time (foam's
    // body carries half- and quarter-drum pairs like u=[0,pi] +
    // u=[pi,2pi]). Coons can express a 4-sided segment, so these never
    // reached the open band below and meshed as UV patches whose
    // circumferential density ignores the radius — the artist's
    // patchwork. Columns at azimuths is the doctrine (cylinder >
    // curves); blend strips (fillets) keep their coons route, and
    // tryOpenBand's own gates bail cleanly back to coons on any rim it
    // cannot chain.
    // DEFAULT ON since the co-axial grouping round (kill-switch
    // WEFT_NO_DRUM_BANDS=1): the original blocker — the body face 183's
    // revolution grid folding under the rerouted counts — was the
    // stacked iso-azimuth rim samples of its top-rim bite, fixed in the
    // strip reconcile (drive-rim azimuth dedupe). Under COUPLED seams
    // the reroute additionally requires SINGLE-EDGE rims: the band's
    // rim then shares its density group with the lattice columns and
    // welds 1:1 by construction, while a multi-piece rim (a quarter
    // drum whose rim chains three edges — foam face 826) solves
    // per-edge counts the band's columns can't honor without the
    // post-weld stitcher, so those keep their coons route.
    if (!std::getenv("WEFT_NO_DRUM_BANDS")) {
        const GeomAbs_SurfaceType st = surf.GetType();
        // The blend detector flags anything tangentially joined as a
        // fillet — including foam's 48-tall half-drums. A real blend
        // STRIP is narrow relative to its radius (a quarter-round is
        // 1.57r across) OR subtends at most ~109 degrees of wrap (edge
        // rounds are quarter arcs plus tangent slack; a cylinder
        // strip's v is its AXIS, so a long box-edge round fails the
        // v-span test yet is still a blend — the fillet-loops knob must
        // keep driving it). Only genuinely wide wraps (foam's
        // half-drums, pi and up) leave the coons/fillet route.
        const double vSpan3D =
            surf.LastVParameter() - surf.FirstVParameter();
        const bool filletStrip =
            info.isFillet &&
            (info.radius <= 1e-9 || vSpan3D <= 1.8 * info.radius ||
             surf.LastUParameter() - surf.FirstUParameter() <= 1.9);
        if (!filletStrip &&
            (st == GeomAbs_Cylinder || st == GeomAbs_Cone ||
             st == GeomAbs_SurfaceOfRevolution) &&
            !surf.IsUClosed() &&
            surf.LastUParameter() - surf.FirstUParameter() >= 1.0) {
            if (tryOpenBand()) {
                if (!settings.decoupleSeams &&
                    (plan.rimLow.size() != 1 || plan.rimHigh.size() != 1)) {
                    dbg("plan face %d: drum multi-piece rims -> coons",
                        fid);
                    plan = FacePlan();
                } else {
                    dbg("plan face %d: partial drum -> open band", fid);
                    return plan;
                }
            } else {
                dbg("plan face %d: drum open-band bail (span %.2f)", fid,
                    surf.LastUParameter() - surf.FirstUParameter());
            }
        }
    }

    // Four-sided freeform/trimmed faces get a structured Coons grid; the
    // across-the-blend direction of a fillet strip is whichever side pair
    // is shorter in 3D.
    {
        CoonsPatch patch;
        if (coonsOk(patch) && coonsChainsCompatible(patch)) {
            // A flat chevron (reflex outline) folds under any transfinite
            // grid. In quad-dominant mode quad-fill's grid + CDT rim is
            // strictly better and samples the same solved counts. In
            // defaults there is no denser-safe replacement — minimal
            // n-gons starve shared rails against unconstrained fallback
            // neighbours (measured: dup flaps on sliver strips) — so the
            // patch proceeds and the untangler + fold overlay take over.
            if (coonsReflex && s.quadDominant) {
                plan = FacePlan();
                plan.kind = MesherKind::Fallback;
                plan.forceFallbackQuads = 1;
                return plan;
            }
            plan.kind = MesherKind::CoonsGrid;
            plan.coonsRotate = coonsEffectiveRotate;
            plan.constrains = true;
            plan.insertWires = patch.holeWires;
            if (patch.collapsedLast && patch.poleCurve) {
                auto surfaceUTravel = [&](std::initializer_list<int> sides) {
                    double travel = 0.0;
                    for (int sd : sides) {
                        gp_Pnt2d prev = patch.side(sd, 0.0);
                        for (int k = 1; k <= 12; ++k) {
                            const gp_Pnt2d cur =
                                patch.side(sd, double(k) / 12.0);
                            travel += std::abs(cur.X() - prev.X());
                            prev = cur;
                        }
                    }
                    return travel;
                };
                plan.coonsPolePatch = true;
                // Grid U runs along sides 0/2; grid V runs along side 1 and
                // the collapsed pole side 3. Ignore side 3's bookkeeping
                // pcurve when deciding the physical angular direction.
                plan.coonsPoleAroundIsU =
                    surfaceUTravel({0, 2}) >= surfaceUTravel({1});
                plan.coonsPoleTurnFraction = std::clamp(
                    (surf.LastUParameter() - surf.FirstUParameter()) /
                        (2.0 * M_PI),
                    0.0, 1.0);
            }
            insertCountFloors(face, patch, plan.insertMinU,
                              plan.insertMinV);
            if (patch.chained()) {
                for (int i = 0; i < 4; ++i) {
                    for (const auto& pce : patch.chain[i]) {
                        plan.coonsSides[i].push_back(pce.edgeId);
                    }
                }
                dbg("coons: face %d chained sides "
                    "[%zu:%d..][%zu:%d..][%zu:%d..][%zu:%d..]",
                    fid, patch.chain[0].size(),
                    patch.chain[0].empty() ? 0 : patch.chain[0][0].edgeId,
                    patch.chain[1].size(),
                    patch.chain[1].empty() ? 0 : patch.chain[1][0].edgeId,
                    patch.chain[2].size(),
                    patch.chain[2].empty() ? 0 : patch.chain[2][0].edgeId,
                    patch.chain[3].size(),
                    patch.chain[3].empty() ? 0 : patch.chain[3][0].edgeId);
            } else {
                if (plan.coonsPolePatch) {
                    // A genuine collapsed revolution pole is one half of a
                    // capsule chart: sides 0/2 are its longitudinal rails and
                    // side 1 is the semicircular base. They share one station
                    // count. Unite all three real borders so the largest
                    // neighbouring count wins; never lower the exterior
                    // capsule driver just to satisfy the cap.
                    plan.uEdges = {patch.edgeIds[0], patch.edgeIds[1],
                                   patch.edgeIds[2]};
                    plan.vEdges = plan.uEdges;
                } else {
                    plan.uEdges = {patch.edgeIds[0], patch.edgeIds[2]};
                    plan.vEdges = {patch.edgeIds[1], patch.edgeIds[3]};
                }
            }
            if (info.isFillet) {
                plan.isFillet = true;
                auto sideLen = [&](int i) {
                    BRepAdaptor_Curve c(
                        TopoDS::Edge(model.edges(patch.edgeIds[i])));
                    return GCPnts_AbscissaPoint::Length(c);
                };
                const double pairU = sideLen(0) + sideLen(2);
                const double pairV = sideLen(1) + sideLen(3);
                plan.acrossIsU = pairU < pairV;
                // Assignment monitor (artist request): near-equal side
                // pairs make the across pick a coin toss — flag it so a
                // wrong-axis loops knob is traceable; the report's
                // faceAcross makes the pick visible in the UI either
                // way.
                if (std::min(pairU, pairV) >
                    0.85 * std::max(pairU, pairV)) {
                    dbg("plan face %d: fillet across ambiguous "
                        "(side pairs %.4g / %.4g)",
                        fid, pairU, pairV);
                }
            }
            // A bent ribbon whose end notch coons just chained into one side:
            // the transfinite grid would fan/crowd toward the pocket and cover
            // it. Flag the plan so the dispatch first tries the rail sweep's
            // clean notch cut, keeping the Coons plan intact as the fallback
            // when the solve is too coarse for the cut to weld.
            if (!info.isFillet && plan.insertWires.empty() &&
                ribbonEndNotchDetect(face, model)) {
                plan.tryRibbonNotch = true;
                dbg("plan face %d: coons + end-notch ribbon -> try sweep cut",
                    fid);
            }
            // The rung-skew class: a long thin strip whose sides are
            // CHAINS. REVERTED TO OPT-IN (WEFT_CHAIN_SWEEP=1): the
            // sweep's arc-length re-pairing straightens the rungs but
            // BREAKS STATION CONTINUITY — the neighbouring fillet
            // strips' loops used to continue across the band through
            // the coons lattice's station-k-to-station-k rungs, and
            // under the sweep they dead-end into absorption triangles
            // (artist verdict: flow beats perpendicularity). The real
            // fix is upstream in the DENSITY SOLVE: align the two
            // rails' station arc-fractions (per-piece counts
            // distributed proportionally to arc within each chain, so
            // matching stations sit at matching fractions and rungs
            // are straight AND continuous) — the FilletBand
            // prerequisite.
            if (std::getenv("WEFT_CHAIN_SWEEP") &&
                !plan.tryRibbonNotch && plan.insertWires.empty() &&
                patch.chained()) {
                auto chainLen = [&](int i) {
                    double L = 0;
                    if (!patch.chain[i].empty()) {
                        for (const auto& pce : patch.chain[i]) {
                            BRepAdaptor_Curve c(
                                TopoDS::Edge(model.edges(pce.edgeId)));
                            L += GCPnts_AbscissaPoint::Length(c);
                        }
                    } else if (patch.edgeIds[i] > 0) {
                        BRepAdaptor_Curve c(
                            TopoDS::Edge(model.edges(patch.edgeIds[i])));
                        L += GCPnts_AbscissaPoint::Length(c);
                    }
                    return L;
                };
                const double a = chainLen(0) + chainLen(2);
                const double b = chainLen(1) + chainLen(3);
                const double lo2 = std::min(a, b), hi2 = std::max(a, b);
                if (lo2 > 1e-12 && hi2 / lo2 >= 3.0) {
                    plan.tryRibbonSweep = true;
                    dbg("plan face %d: chained strip (aspect %.1f) -> try "
                        "rail sweep",
                        fid, hi2 / lo2);
                }
            }
            return plan;
        }
    }

    // Two-tip bands (crescents, lunes, tangent strips): coons wants four
    // corners and the webs fan these — the rail ladder pairs the two
    // rails directly.
    if (planRailLadder(face, model, plan)) return plan;

    // PARTIAL-WRAP revolution band (a barrel wall trimmed short of the
    // full period, its rim castellated by notches): meshes as a
    // STRAIGHT lattice (columns at fixed azimuths) with the
    // castellation boolean-cut out and webbed. Faces a Coons patch or
    // the ladder can express never reach this point, so only the
    // long-chain outlines that used to lattice as quad-fill qualify.
    if (tryOpenBand()) return plan;

    // Quad Fill is not an automatic output strategy. For a genuine long,
    // thin freeform ribbon we still reuse its boundary analysis to seed the
    // specialized rail sweep; all other unclaimed faces take the local
    // exact-border fallback below.
    const bool planarHere = surf.GetType() == GeomAbs_Plane;
    if (!planarHere) {
        // A long thin bent ribbon (grip / trigger-guard rails) may sweep its
        // two rails into regular rows. Failure demotes to the contract floor,
        // never back to Quad Fill.
        const GeomAbs_SurfaceType st = surf.GetType();
        const bool analyticDrum =
            st == GeomAbs_Cylinder || st == GeomAbs_Cone ||
            st == GeomAbs_SurfaceOfRevolution;
        if (!analyticDrum && ribbonDetect(face, model) &&
            planQuadFill(face, surf, model, plan)) {
            plan.kind = MesherKind::RibbonSweep;
            return plan;
        }
    }

    // Broad, almost-flat freeform panels may legitimately be shallower than
    // the requested chord tolerance even though the relative flatness test
    // rejects them.  Prove the complete trimmed face against that absolute
    // tolerance before emitting one exact-boundary game n-gon.
    if (s.minimal && !info.isFillet &&
        planDeviationFlatPanel(face, surf, model, s, plan)) {
        dbg("plan face %d: deviation-flat exact-boundary panel", fid);
        return plan;
    }

    // A concave freeform trim which every primitive/Coons/ribbon planner has
    // already declined may still be a rigorously simple two-rail corridor.
    // Keep this late and independent: it cannot broaden the successful Coons
    // preflight, and its own dense in-face rung proof must pass before it can
    // replace the contract floor.
    if (!info.isFillet &&
        planTrimCorridor(face, surf, model, s, plan)) {
        dbg("plan face %d: trim corridor axis=%c rails %zu/%zu bands=%d",
            fid, plan.trimCorridorAxisU ? 'u' : 'v',
            plan.coonsSides[0].size(), plan.coonsSides[2].size(),
            plan.trimCorridorBands);
        return plan;
    }

    // Curved many-piece neck/transition trims: a separate UV-section strip,
    // never a relaxation of the corridor or Coons gates above.  It qualifies
    // only when two near-iso longitudinal boundary chains span the chart and
    // a broad central ruled interval stays inside with one orientation; its
    // bounded concave ends remain exact-boundary closure n-gons.
    if (!info.isFillet &&
        planSectionStrip(face, surf, model, s, plan)) {
        dbg("plan face %d: curved multi-section trim strip", fid);
        return plan;
    }

    // A shallow conical cap nothing else claimed would tri-fan; a single
    // boundary n-gon is the clean flat-panel result (foam spray-can disk
    // faces 325/366). Tightly gated (isShallowCapCone) so only genuine
    // round, barely-dished, hole-free caps qualify — never a cone wall.
    if (s.minimal && isShallowCapCone(face, surf) &&
        collectPlanarLoops(face, surf, model, plan, /*requirePlane=*/false)) {
        plan.kind = MesherKind::MinimalNGon;
        return plan;
    }
    plan = FacePlan();
    plan.kind = MesherKind::Fallback;
    if (plan.forceFallbackQuads < 0) {
        // Automatic fallback stays triangulated: pairing unrelated floor
        // triangles can recreate the very long poles Quad Fill was removed
        // for. Quad pairing remains an explicit --quads/user choice.
        plan.forceFallbackQuads =
            s.quadDominant && !s.pureTriFloor ? 1 : 0;
    }
    // Quad-dominant decimation moves border verts by up to the chord
    // tolerance; on a face with features SMALLER than that it wraps flaps
    // over the neighbours (folds the weld then has to amputate). Such
    // faces triangulate plainly instead.
    if (s.quadDominant) {
        for (int eid : info.edgeIds) {
            const double len = analysis.edges[eid - 1].length;
            if (len > 1e-12 && len < 3.0 * s.chordTolerance) {
                plan.forceFallbackQuads = 0;
                break;
            }
        }
    }
    return plan;
}


}  // namespace weft::mesher_impl
