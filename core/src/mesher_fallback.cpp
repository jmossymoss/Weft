#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// The demotion floor for ANY face with pcurves: every wire sampled at
// the solved counts (the border contract, same formula every mesher
// uses), holes bridged in UV, the region web-triangulated. Interior
// quality is modest, but the borders are exact by construction — a
// face that lands here cannot leak. The OCCT triangulation fallback
// remains only for faces this cannot express (null curves, degenerate
// UV rings).
// Single-loop CURVED patch webbed on its own surface: ring at solved
// counts (the contract) around one interior vertex at the UV centroid,
// evaluated ON the surface. A bore piercing a wall leaves such a patch
// on EACH side of the intersection loop; flat chord webs from the two
// sides pick the same ring diagonals and weld NON-MANIFOLD (mohne face
// 204 vs face 2's insert web) — a fan has no ring diagonals at all,
// and the private apex follows the surface instead of denting it.
bool meshSurfaceCapFan(const TopoDS_Face& face, const Model& model,
                       int faceId, const std::vector<int>& solvedEdge,
                       int radialDefault, MeshBuilder& out) {
    int wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        ++wires;
    }
    if (wires != 1) return false;
    BRepAdaptor_Surface surf(face);
    if (surf.GetType() == GeomAbs_Plane) return false;  // flat webs fine
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
        return false;
    }
    if (rings.size() != 1 || rings[0].uv.size() < 3) return false;
    const PlanarRing& r = rings[0];
    const size_t n = r.uv.size();
    double cu = 0, cv = 0;
    for (const gp_Pnt2d& q : r.uv) {
        cu += q.X();
        cv += q.Y();
    }
    cu /= double(n);
    cv /= double(n);
    // Every fan triangle must run the ring's way in UV (positive with
    // the normalized outer winding); a reflex loop that captures the
    // centroid outside bails to the flat floor.
    for (size_t i = 0; i < n; ++i) {
        const gp_Pnt2d& a = r.uv[i];
        const gp_Pnt2d& b = r.uv[(i + 1) % n];
        const double cross = (a.X() - cu) * (b.Y() - cv) -
                             (a.Y() - cv) * (b.X() - cu);
        if (cross <= 1e-14) return false;
    }
    gp_Pnt apex;
    try {
        apex = surf.Value(cu, cv);
    } catch (const Standard_Failure&) {
        return false;
    }
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    std::vector<uint32_t> ring(n);
    for (size_t i = 0; i < n; ++i) {
        ring[i] = out.addVertex(r.p[i],
                                {faceId, r.uv[i].X(), r.uv[i].Y()});
    }
    const uint32_t c = out.addVertex(apex, {faceId, cu, cv});
    for (size_t i = 0; i < n; ++i) {
        out.addPolygon({c, ring[i], ring[(i + 1) % n]}, faceId, flip);
    }
    dbg("surface cap fan %d: %zu ring verts", faceId, n);
    return true;
}


// The deflection budget a face's triangulation must honor — the same
// meaning meshFallback gives the deviation slider (relative mode scales
// by THIS face's extent).
double faceDeflection(const TopoDS_Face& face, const FaceMeshSettings& s) {
    double defl = std::max(1e-9, s.chordTolerance);
    if (s.relativeDeviation) {
        Bnd_Box bb;
        BRepBndLib::Add(face, bb);
        if (!bb.IsVoid()) {
            double x0, y0, z0, x1, y1, z1;
            bb.Get(x0, y0, z0, x1, y1, z1);
            const double diag =
                gp_Pnt(x0, y0, z0).Distance(gp_Pnt(x1, y1, z1));
            defl = std::max(1e-9, s.chordTolerance * 0.05 * diag);
        }
    }
    return defl;
}

// Interior curvature can exceed the curvature of a trim edge. A structured
// split rectangle therefore needs a face-based subdivision floor as well as
// its edge counts; otherwise two nearly straight rims can leave broad chords
// across a visibly bowed B-spline sheet. Find the smallest uniform axis count
// whose true-surface midpoint sag satisfies the same chord-deviation control
// used elsewhere in the mesher. Boundary edge counts already enforce the
// turn-angle contract; using raw D1 normal angles here is unreliable at
// legitimate B-spline parameter singularities where equivalent normals can
// reverse direction.
int orthogonalSurfaceDivisionFloor(const TopoDS_Face& face,
                                   const BRepAdaptor_Surface& surf,
                                   const FaceMeshSettings& s,
                                   bool alongU) {
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    if (surface.IsNull()) return 1;
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    if (std::abs(u1 - u0) <= 1e-15 || std::abs(v1 - v0) <= 1e-15) return 1;
    const double deflection = faceDeflection(face, s);
    const double axis0 = alongU ? u0 : v0;
    const double axis1 = alongU ? u1 : v1;
    const double other0 = alongU ? v0 : u0;
    const double other1 = alongU ? v1 : u1;

    struct ProbeSpan {
        double other = 0.0;
        double lo = 0.0;
        double hi = 0.0;
    };
    std::vector<ProbeSpan> spans;
    const double faceTol = BRep_Tool::Tolerance(face);
    constexpr int kScan = 32;
    for (int cross = 1; cross <= 9; cross += 2) {
        const double q = double(cross) / 10.0;
        const double other = other0 + (other1 - other0) * q;
        int first = -1, last = -1;
        for (int sample = 0; sample < kScan; ++sample) {
            const double t = (sample + 0.5) / kScan;
            const double axis = axis0 + (axis1 - axis0) * t;
            const gp_Pnt2d uv =
                alongU ? gp_Pnt2d(axis, other)
                       : gp_Pnt2d(other, axis);
            BRepClass_FaceClassifier owner(
                const_cast<TopoDS_Face&>(face), uv, faceTol);
            if (owner.State() == TopAbs_OUT) continue;
            if (first < 0) first = sample;
            last = sample;
        }
        if (first < 0 || last < first) continue;
        const double step = (axis1 - axis0) / kScan;
        const double lo = axis0 + first * step;
        const double hi = axis0 + (last + 1) * step;
        if (hi - lo > 1e-12) spans.push_back({other, lo, hi});
    }
    if (spans.empty()) return 4;

    auto meets = [&](int divisions) {
        double worstSag = 0.0;
        try {
            for (const ProbeSpan& span : spans) {
                for (int band = 0; band < divisions; ++band) {
                    const double t0 = double(band) / divisions;
                    const double t1 = double(band + 1) / divisions;
                    const double tm = 0.5 * (t0 + t1);
                    const double a0 = span.lo + (span.hi - span.lo) * t0;
                    const double a1 = span.lo + (span.hi - span.lo) * t1;
                    const double am = span.lo + (span.hi - span.lo) * tm;
                    gp_Pnt p0, p1, pm;
                    if (alongU) {
                        p0 = surface->Value(a0, span.other);
                        p1 = surface->Value(a1, span.other);
                        pm = surface->Value(am, span.other);
                    } else {
                        p0 = surface->Value(span.other, a0);
                        p1 = surface->Value(span.other, a1);
                        pm = surface->Value(span.other, am);
                    }
                    const gp_Pnt chord((p0.XYZ() + p1.XYZ()) * 0.5);
                    worstSag = std::max(worstSag, pm.Distance(chord));
                }
            }
        } catch (const Standard_Failure&) {
            return false;
        }
        return worstSag <= deflection;
    };

    int failed = 0;
    int passing = 1;
    while (passing < 32 && !meets(passing)) {
        failed = passing;
        passing *= 2;
    }
    if (!meets(passing)) return 32;
    while (failed + 1 < passing) {
        const int mid = failed + (passing - failed) / 2;
        if (meets(mid)) passing = mid;
        else failed = mid;
    }
    return passing;
}

// Whether the surface region stays within the deflection budget of a
// single flat sheet — the gate that lets a cap fan (no interior detail)
// stand in for a refined web.
bool faceWithinDeflection(const TopoDS_Face& face,
                          const FaceMeshSettings& s) {
    try {
        Handle(Geom_Surface) S = BRep_Tool::Surface(face);
        if (S.IsNull()) return true;
        double u0, u1, v0, v1;
        BRepTools::UVBounds(face, u0, u1, v0, v1);
        const double defl = faceDeflection(face, s);
        std::vector<gp_Pnt> pts;
        for (int i = 0; i <= 5; ++i) {
            for (int j = 0; j <= 5; ++j) {
                pts.push_back(S->Value(u0 + (u1 - u0) * i / 5.0,
                                       v0 + (v1 - v0) * j / 5.0));
            }
        }
        gp_XYZ c(0, 0, 0);
        for (const gp_Pnt& p : pts) c += p.XYZ();
        c /= double(pts.size());
        // Newell normal over the sample fan gives a stable plane.
        gp_XYZ n(0, 0, 0);
        for (size_t k = 0; k + 1 < pts.size(); ++k) {
            n += (pts[k].XYZ() - c).Crossed(pts[k + 1].XYZ() - c);
        }
        if (n.Modulus() < 1e-12) return true;
        n /= n.Modulus();
        double maxD = 0;
        for (const gp_Pnt& p : pts) {
            maxD = std::max(maxD, std::abs((p.XYZ() - c).Dot(n)));
        }
        return maxD <= defl;
    } catch (const Standard_Failure&) {
        return true;
    }
}

// Deviation-driven interior refinement for floor webs: split interior
// edges whose surface midpoint sags off the mesh by more than the
// deflection, flip diagonals toward Delaunay in UV, then (optionally)
// pair triangles into quads. Border edges are the weld contract and
// never split; every new vertex evaluates ON the surface and carries a
// face anchor. This is what makes deviation / min-size / quad-dominant
// LIVE on floored faces — a border-only web has no interior to respond.
void refineFloorWeb(PolyMesh& part, const TopoDS_Face& face, int faceId,
                    const FaceMeshSettings& s, double uScale,
                    std::vector<gp_Pnt2d> uvOf, bool angleSplit = false) {
    Handle(Geom_Surface) S = BRep_Tool::Surface(face);
    if (S.IsNull()) return;
    if (uvOf.size() != part.vertices.size()) return;
    std::vector<std::array<uint32_t, 3>> tris;
    tris.reserve(part.polygons.size());
    for (const auto& poly : part.polygons) {
        if (poly.size() != 3) return;  // floor webs are all-tri
        tris.push_back({poly[0], poly[1], poly[2]});
    }
    const double defl = faceDeflection(face, s);
    // Surface normal at a UV, for the facet-turn (angle) split criterion.
    auto surfNormal = [&](const gp_Pnt2d& uv) -> gp_Vec {
        gp_Pnt p;
        gp_Vec du, dv;
        S->D1(uv.X(), uv.Y(), p, du, dv);
        return du.Crossed(dv);
    };
    const double angTol = std::max(1.0, s.angleToleranceDeg);
    auto p3 = [&](uint32_t v) {
        return gp_Pnt(part.vertices[v][0], part.vertices[v][1],
                      part.vertices[v][2]);
    };
    auto ekey = [](uint32_t a, uint32_t b) {
        return (uint64_t(std::min(a, b)) << 32) | std::max(a, b);
    };

    for (int round = 0; round < 8 && tris.size() < 20000; ++round) {
        std::map<uint64_t, std::array<int, 2>> etri;
        for (size_t t = 0; t < tris.size(); ++t) {
            for (int i = 0; i < 3; ++i) {
                auto& e =
                    etri.emplace(ekey(tris[t][i], tris[t][(i + 1) % 3]),
                                 std::array<int, 2>{-1, -1})
                        .first->second;
                (e[0] < 0 ? e[0] : e[1]) = int(t);
            }
        }
        auto splittable = [&](uint32_t a, uint32_t b) {
            const auto it = etri.find(ekey(a, b));
            if (it == etri.end() || it->second[1] < 0) return false;
            const double len = p3(a).Distance(p3(b));
            if (s.minSize > 0 && len <= 2.0 * s.minSize) return false;
            const gp_Pnt2d um(0.5 * (uvOf[a].X() + uvOf[b].X()),
                              0.5 * (uvOf[a].Y() + uvOf[b].Y()));
            // Deviation (chord) criterion: the surface midpoint sags off the
            // straight span by more than the deflection. faceDeflection above
            // already folds in relativeDeviation, so `reldev` refines here too.
            gp_Pnt onSurf = S->Value(um.X(), um.Y());
            gp_XYZ lerp = (p3(a).XYZ() + p3(b).XYZ()) / 2.0;
            if (onSurf.Distance(gp_Pnt(lerp)) > defl) return true;
            // Angle criterion: split where the surface TURNS more than the
            // angle tolerance across the span (the facet-turn limit the border
            // sampler already applies via GCPnts_TangentialDeflection), so a
            // tightened `angle` finally densifies a floor whose sag stays
            // under the chord budget. Gated on an explicit angle override —
            // at the default tolerance this stays off and default output is
            // byte-identical.
            if (angleSplit) {
                const gp_Vec na = surfNormal(uvOf[a]);
                const gp_Vec nb = surfNormal(uvOf[b]);
                if (na.Magnitude() > 1e-12 && nb.Magnitude() > 1e-12 &&
                    na.Angle(nb) * 180.0 / M_PI > angTol) {
                    return true;
                }
            }
            return false;
        };
        std::set<uint64_t> marked;
        for (const auto& t : tris) {
            for (int i = 0; i < 3; ++i) {
                uint32_t a = t[i], b = t[(i + 1) % 3];
                if (marked.count(ekey(a, b))) continue;
                if (splittable(a, b)) marked.insert(ekey(a, b));
            }
        }
        if (marked.empty()) break;
        // One split per triangle per round keeps the children sane.
        std::vector<char> touched(tris.size(), 0);
        for (uint64_t key : marked) {
            const auto it = etri.find(key);
            if (it == etri.end()) continue;
            const int t1 = it->second[0], t2 = it->second[1];
            if (t1 < 0 || t2 < 0 || touched[t1] || touched[t2]) continue;
            const uint32_t a = uint32_t(key >> 32), b = uint32_t(key);
            const gp_Pnt2d um(0.5 * (uvOf[a].X() + uvOf[b].X()),
                              0.5 * (uvOf[a].Y() + uvOf[b].Y()));
            const gp_Pnt pw = S->Value(um.X(), um.Y());
            const uint32_t w = uint32_t(part.vertices.size());
            part.vertices.push_back({pw.X(), pw.Y(), pw.Z()});
            part.anchors.push_back({faceId, um.X(), um.Y()});
            uvOf.push_back(um);
            for (int t : {t1, t2}) {
                touched[t] = 1;
                std::array<uint32_t, 3> tri = tris[t];
                for (int i = 0; i < 3; ++i) {
                    uint32_t p = tri[i], q = tri[(i + 1) % 3];
                    if ((p == a && q == b) || (p == b && q == a)) {
                        const uint32_t r = tri[(i + 2) % 3];
                        tris[t] = {p, w, r};
                        tris.push_back({w, q, r});
                        touched.push_back(1);
                        break;
                    }
                }
            }
        }
        // Delaunay flips in (anisotropy-corrected) UV restore quality
        // after the splits.
        for (int sweep = 0; sweep < 2; ++sweep) {
            std::map<uint64_t, std::array<int, 2>> em;
            for (size_t t = 0; t < tris.size(); ++t) {
                for (int i = 0; i < 3; ++i) {
                    auto& e = em.emplace(
                                    ekey(tris[t][i], tris[t][(i + 1) % 3]),
                                    std::array<int, 2>{-1, -1})
                                  .first->second;
                    (e[0] < 0 ? e[0] : e[1]) = int(t);
                }
            }
            auto sc = [&](uint32_t v) {
                return gp_Pnt2d(uvOf[v].X() * uScale, uvOf[v].Y());
            };
            auto cross2 = [](const gp_Pnt2d& o, const gp_Pnt2d& a,
                             const gp_Pnt2d& b) {
                return (a.X() - o.X()) * (b.Y() - o.Y()) -
                       (a.Y() - o.Y()) * (b.X() - o.X());
            };
            bool flipped = false;
            for (auto& [key, e] : em) {
                if (e[1] < 0) continue;
                const uint32_t p = uint32_t(key >> 32),
                               q = uint32_t(key);
                int t1 = e[0], t2 = e[1];
                // The map goes stale as flips land: skip entries whose
                // triangles no longer carry this edge (a garbage third
                // vertex here once tore a face into internal opens).
                auto hasEdge = [&](int t) {
                    int hit = 0;
                    for (uint32_t v : tris[t]) {
                        if (v == p || v == q) ++hit;
                    }
                    return hit == 2;
                };
                if (!hasEdge(t1) || !hasEdge(t2)) continue;
                uint32_t c = 0, d = 0;
                for (uint32_t v : tris[t1]) {
                    if (v != p && v != q) c = v;
                }
                for (uint32_t v : tris[t2]) {
                    if (v != p && v != q) d = v;
                }
                if (c == d || em.count(ekey(c, d))) continue;
                // in-circle test on t1 (p,q,c) against d
                const gp_Pnt2d P = sc(p), Q = sc(q), C = sc(c), D = sc(d);
                const double ax = P.X() - D.X(), ay = P.Y() - D.Y();
                const double bx = Q.X() - D.X(), by = Q.Y() - D.Y();
                const double cx = C.X() - D.X(), cy = C.Y() - D.Y();
                const double det =
                    (ax * ax + ay * ay) * (bx * cy - by * cx) -
                    (bx * bx + by * by) * (ax * cy - ay * cx) +
                    (cx * cx + cy * cy) * (ax * by - ay * bx);
                const double orient = cross2(P, Q, C);
                if (orient == 0 || det * (orient > 0 ? 1 : -1) <= 0) {
                    continue;
                }
                // Flip only when both children stay non-degenerate.
                if (std::abs(cross2(C, P, D)) < 1e-16 ||
                    std::abs(cross2(D, Q, C)) < 1e-16) {
                    continue;
                }
                // Rebuild with the orientation pattern of the originals:
                // t1 walks p->q somewhere; the children keep that hand.
                bool fwd1 = false;
                for (int i = 0; i < 3; ++i) {
                    if (tris[t1][i] == p &&
                        tris[t1][(i + 1) % 3] == q) {
                        fwd1 = true;
                    }
                }
                if (fwd1) {
                    tris[t1] = {p, d, c};
                    tris[t2] = {d, q, c};
                } else {
                    tris[t1] = {q, d, c};
                    tris[t2] = {d, p, c};
                }
                // Register the new diagonal so a later stale entry can't
                // recreate it (duplicate edge = non-manifold).
                em[ekey(c, d)] = {t1, t2};
                flipped = true;
            }
            if (!flipped) break;
        }
    }

    // Rebuild the part's polygons; pair into quads by default (greedy by
    // corner-angle cost, exactly the quad-dominant fallback's move) —
    // the refined web's triangles are grid-shaped already, so merging
    // interior diagonals is free and borders never move.
    std::vector<std::vector<uint32_t>> polys;
    if (s.pureTriFloor) {
        for (const auto& t : tris) polys.push_back({t[0], t[1], t[2]});
    } else {
        std::map<uint64_t, std::array<int, 2>> em;
        for (size_t t = 0; t < tris.size(); ++t) {
            for (int i = 0; i < 3; ++i) {
                auto& e = em.emplace(
                                ekey(tris[t][i], tris[t][(i + 1) % 3]),
                                std::array<int, 2>{-1, -1})
                              .first->second;
                (e[0] < 0 ? e[0] : e[1]) = int(t);
            }
        }
        struct Cand {
            double cost;
            int t1, t2;
            std::array<uint32_t, 4> ring;
        };
        std::vector<Cand> cands;
        for (const auto& [key, e] : em) {
            if (e[1] < 0) continue;
            const uint32_t p = uint32_t(key >> 32), q = uint32_t(key);
            int t1 = e[0], t2 = e[1];
            // Orient by t1: it walks the shared edge in SOME direction;
            // the merged cycle keeps that hand.
            uint32_t P = p, Q = q;
            bool fwd = false;
            for (int i = 0; i < 3; ++i) {
                if (tris[t1][i] == p && tris[t1][(i + 1) % 3] == q) {
                    fwd = true;
                }
            }
            if (!fwd) std::swap(P, Q);
            uint32_t c = 0, d = 0;
            for (uint32_t v : tris[t1]) {
                if (v != p && v != q) c = v;
            }
            for (uint32_t v : tris[t2]) {
                if (v != p && v != q) d = v;
            }
            const std::array<uint32_t, 4> ring{P, d, Q, c};
            const double cost = quadAngleCost(
                {p3(ring[0]), p3(ring[1]), p3(ring[2]), p3(ring[3])});
            if (cost > 1e8) continue;
            cands.push_back({cost, t1, t2, ring});
        }
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b) {
                      return a.cost < b.cost;
                  });
        std::vector<char> used(tris.size(), 0);
        for (const Cand& cd : cands) {
            if (used[cd.t1] || used[cd.t2]) continue;
            used[cd.t1] = used[cd.t2] = 1;
            polys.push_back({cd.ring[0], cd.ring[1], cd.ring[2],
                             cd.ring[3]});
        }
        for (size_t t = 0; t < tris.size(); ++t) {
            if (!used[t]) {
                polys.push_back({tris[t][0], tris[t][1], tris[t][2]});
            }
        }
    }
    part.polygons = std::move(polys);
    part.polygonFaceId.assign(part.polygons.size(), faceId);
}

bool meshContractFallback(const TopoDS_Face& face, const Model& model,
                          int faceId, const std::vector<int>& solvedEdge,
                          int radialDefault, MeshBuilder& out,
                          const FaceMeshSettings* refine,
                          bool angleSplit,
                          const PinnedEdges* pins) {
    std::vector<PlanarRing> rings;
    // Pins matter: the contract checker (and every neighbour) samples a
    // pinned edge at its explicit pin positions, not uniform steps — a
    // floor that samples uniformly misses them by a hair and "violates"
    // its own contract (the notched fixture's radial-23/46/51/69 raw
    // demotions were exactly this).
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings,
                           pins)) {
        dbg("contract floor %d: ring sampling failed", faceId);
        return false;
    }
    if (rings.empty()) return false;
    const bool flip = face.Orientation() == TopAbs_REVERSED;
    // Curved UV charts are anisotropic (u in radians, v in model units):
    // scale u by the local relative stretch so the triangulator sees
    // true shapes. Positive scale keeps the normalized windings.
    double uScale = 1.0;
    try {
        BRepAdaptor_Surface surf(face);
        const double um =
            (surf.FirstUParameter() + surf.LastUParameter()) / 2;
        const double vm =
            (surf.FirstVParameter() + surf.LastVParameter()) / 2;
        const double su = std::max(
            1e-9,
            surf.Value(um, vm).Distance(surf.Value(um + 1e-3, vm)) / 1e-3);
        const double sv = std::max(
            1e-9,
            surf.Value(um, vm).Distance(surf.Value(um, vm + 1e-3)) / 1e-3);
        uScale = su / sv;
    } catch (const Standard_Failure&) {
    }
    std::vector<WebPoint> outer;
    std::vector<std::vector<WebPoint>> holes;
    std::vector<gp_Pnt2d> uvOf;  // per emitted vertex, UNSCALED uv
    for (PlanarRing& r : rings) {
        std::vector<WebPoint> ring;
        for (size_t i = 0; i < r.uv.size(); ++i) {
            ring.push_back(
                {gp_Pnt2d(r.uv[i].X() * uScale, r.uv[i].Y()),
                 out.addVertex(r.p[i], {})});
            uvOf.push_back(r.uv[i]);
        }
        if (r.isOuter) outer = std::move(ring);
        else holes.push_back(std::move(ring));
    }
    if (outer.size() < 3) return false;
    if (!triangulateWeb(std::move(outer), std::move(holes), faceId, flip,
                        out)) {
        dbg("contract floor %d: web triangulation failed", faceId);
        return false;
    }
    if (refine) {
        refineFloorWeb(out.mesh(), face, faceId, *refine, uScale,
                       std::move(uvOf), angleSplit);
    }
    return true;
}

bool planQuadFill(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, FacePlan& plan) {
    // Any trimmed surface patch works — the grid lives in UV and maps
    // through the surface. Only faces that wrap a FULL period need the
    // seam-aware revolution grids; a small patch trimmed from a closed
    // surface (fillet corners, wedges on cylinders) is a plain chart.
    //
    // A full-period BSPLINE band is the exception we DO take: it wraps a
    // closed freeform loop (a curved skirt/collar), yet has no analytic
    // revolution to fall back on — the revolution grids only fire for
    // classic surfaces of revolution / cylinders, and coons rejects the
    // seam. Meshing it as a plain UV chart keeps the seam ON the
    // v-boundary (the grid never crosses it), so the two seam columns
    // coincide in 3D and weld watertight, and the band gets clean quad
    // flow instead of a fallback tri fan. Analytic periodic surfaces
    // (cone caps, cylinders) keep rejecting so their dedicated
    // revolution / minimal-cap routes still own them byte-for-byte.
    {
        Handle(Geom_Surface) S = BRep_Tool::Surface(face);
        if (S.IsNull()) return false;
        double umin, umax, vmin, vmax;
        BRepTools::UVBounds(face, umin, umax, vmin, vmax);
        const bool isBSpline = surf.GetType() == GeomAbs_BSplineSurface;
        const bool uFull =
            S->IsUPeriodic() && umax - umin >= 0.999 * S->UPeriod();
        const bool vFull =
            S->IsVPeriodic() && vmax - vmin >= 0.999 * S->VPeriod();
        // A doubly-periodic full wrap (a whole torus / closed tube) has no
        // boundary to anchor a chart — never take it, even as a bspline.
        if ((uFull || vFull) && !(isBSpline && !(uFull && vFull))) {
            return false;
        }
    }
    FacePlan probe;
    if (!collectPlanarLoops(face, surf, model, probe,
                            /*requirePlane=*/false,
                            /*tolerateDegenerate=*/true)) {
        return false;
    }
    plan.loops = std::move(probe.loops);
    plan.uEdges = std::move(probe.uEdges);
    plan.constrains = true;
    plan.kind = MesherKind::QuadFill;
    return true;
}

// Quad-fill: a planar face of any outline gets an interior quad grid sized
// from its border density, and the gap between the grid and the exact
// boundary closes with the hole-bridged ear-clip web. Large clean quad
// flow on plates instead of fan triangulations.
// Zipper a simple band between an outer ring (CCW in UV) and a hole ring
// (CW): rotational alignment by total rail length, then a greedy walk that
// advances whichever side makes the shorter diagonal. Equal counts give a
// pure quad ring; the rim of a quad-fill face reads as flow, not ear soup.
// The zip is validated before anything is emitted: when the two rings are
// not actually a band (a long thin outline against a small localized
// frontier loop), the greedy walk fans across the void and folds — every
// candidate cell's UV winding must agree, or the caller ear-clips instead.
bool zipperRings(const std::vector<WebPoint>& outer,
                 const std::vector<WebPoint>& hole, int faceId, bool flip,
                 MeshBuilder& out) {
    const int n = int(outer.size());
    const int m = int(hole.size());
    // The hole winds opposite to the outer; reverse it so both progress
    // the same way around the band.
    std::vector<WebPoint> ring(hole.rbegin(), hole.rend());
    auto d2 = [](const WebPoint& a, const WebPoint& b) {
        return a.uv.SquareDistance(b.uv);
    };
    int bestOff = 0;
    double bestSum = 1e300;
    for (int off = 0; off < m; ++off) {
        double sum = 0;
        for (int i = 0; i < n; i += std::max(1, n / 64)) {
            sum += d2(outer[i], ring[(off + i * m / n) % m]);
        }
        if (sum < bestSum) {
            bestSum = sum;
            bestOff = off;
        }
    }
    std::vector<std::array<const WebPoint*, 4>> cells;  // [3] null = tri
    if (n == m) {  // pure quad ring
        for (int i = 0; i < n; ++i) {
            int j = (bestOff + i) % m;
            cells.push_back({&outer[i], &outer[(i + 1) % n],
                             &ring[(j + 1) % m], &ring[j]});
        }
    } else {
        int ia = 0, ib = 0;
        while (ia < n || ib < m) {
            const WebPoint& a = outer[ia % n];
            const WebPoint& a1 = outer[(ia + 1) % n];
            const WebPoint& b = ring[(bestOff + ib) % m];
            const WebPoint& b1 = ring[(bestOff + ib + 1) % m];
            bool stepA;
            if (ia >= n) stepA = false;
            else if (ib >= m) stepA = true;
            else stepA = d2(a1, b) <= d2(a, b1);
            if (stepA) {
                cells.push_back({&a, &a1, &b, nullptr});
                ++ia;
            } else {
                cells.push_back({&a, &b1, &b, nullptr});
                ++ib;
            }
        }
    }
    double total = 0;
    std::vector<double> areas;
    areas.reserve(cells.size());
    double meanAbs = 0;
    for (const auto& c : cells) {
        const int k = c[3] ? 4 : 3;
        double a = 0;
        for (int i = 0; i < k; ++i) {
            const gp_Pnt2d& p = c[i]->uv;
            const gp_Pnt2d& q = c[(i + 1) % k]->uv;
            a += p.X() * q.Y() - q.X() * p.Y();
        }
        areas.push_back(a);
        total += a;
        meanAbs += std::abs(a);
    }
    meanAbs /= double(std::max<size_t>(1, areas.size()));
    for (double a : areas) {
        if (a * total < 0 && std::abs(a) > 1e-3 * meanAbs) return false;
    }
    for (const auto& c : cells) {
        if (c[3]) {
            out.addPolygon({c[0]->vert, c[1]->vert, c[2]->vert, c[3]->vert},
                           faceId, flip);
        } else {
            out.addPolygon({c[0]->vert, c[1]->vert, c[2]->vert}, faceId,
                           flip);
        }
    }
    return true;
}

// A curved single-loop cap that coons rejected for want of four clear
// corners -- a dished disk, a two-tip lens, a rounded triangle -- meshes
// here as CONCENTRIC quad rings shrinking toward the UV centroid, the
// innermost closed as one n-gon: 0 tris, borders exact at their solved
// counts, every interior vertex evaluated ON the surface so the dish is
// followed (the Plasticity disk-cap pattern). The alternative, quad-fill's
// grid + CDT rim, tri-fans the pointed rim of exactly these shapes. Bails
// to false (so quad-fill takes over) for anything not star-shaped from its
// centroid, or with four+ genuine corners (a rectangle already grids clean).
bool meshDiskCap(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                 const Model& model, int faceId,
                 const std::vector<int>& solvedEdge, int radialDefault,
                 MeshBuilder& out) {
    int wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        if (++wires > 1) return false;  // a hole needs a web, not a fan cap
    }
    if (wires != 1) return false;
    if (surf.GetType() == GeomAbs_Plane) return false;  // flat: CDT/n-gon own it

    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
        return false;
    }
    if (rings.size() != 1) return false;
    const PlanarRing& r = rings[0];
    const size_t n = r.uv.size();
    if (n < 6) return false;

    // UV centroid, and the star-shape test: every boundary edge must turn
    // the same way about it (positive with the normalized outer winding),
    // or a homothety-shrunk ring would self-cross. samplePlanarRings has
    // already oriented the outer ring CCW in UV.
    double cu = 0, cv = 0;
    for (const gp_Pnt2d& q : r.uv) {
        cu += q.X();
        cv += q.Y();
    }
    cu /= double(n);
    cv /= double(n);
    // Tolerance relative to the strongest turn: a near-collinear rim vertex
    // (a slightly dished, elongated cap) dips a hair negative from rounding
    // and must not veto the cap; only a genuine reflex notch does.
    double maxCross = 0;
    for (size_t i = 0; i < n; ++i) {
        const gp_Pnt2d& a = r.uv[i];
        const gp_Pnt2d& b = r.uv[(i + 1) % n];
        maxCross = std::max(maxCross, std::abs((a.X() - cu) * (b.Y() - cv) -
                                               (a.Y() - cv) * (b.X() - cu)));
    }
    const double starTol = -1e-3 * maxCross;
    for (size_t i = 0; i < n; ++i) {
        const gp_Pnt2d& a = r.uv[i];
        const gp_Pnt2d& b = r.uv[(i + 1) % n];
        const double cross = (a.X() - cu) * (b.Y() - cv) -
                             (a.Y() - cv) * (b.X() - cu);
        if (cross <= starTol) return false;  // reflex / centroid outside
    }

    // Corner screen: a cap has at most a few genuine corners (disk 0, lens
    // 2, rounded triangle 3). Four or more is a panel the grid quads well
    // already -- don't hijack it.
    int corners = 0;
    for (size_t i = 0; i < n; ++i) {
        const gp_Pnt& a = r.p[(i + n - 1) % n];
        const gp_Pnt& b = r.p[i];
        const gp_Pnt& c = r.p[(i + 1) % n];
        gp_Vec u(a, b), v(b, c);
        if (u.Magnitude() < 1e-12 || v.Magnitude() < 1e-12) continue;
        if (u.Angle(v) > 50.0 * M_PI / 180.0) ++corners;
    }
    if (corners > 3) return false;

    // Radial resolution: divide the mean rim->apex distance into steps of
    // roughly the rim's own vertex spacing so ring cells stay near-square.
    // At least two layers => >=1 quad ring plus the central n-gon.
    gp_Pnt apex;
    try {
        apex = surf.Value(cu, cv);
    } catch (const Standard_Failure&) {
        return false;
    }
    double meanEdge = 0, meanR = 0;
    for (size_t i = 0; i < n; ++i) {
        meanEdge += r.p[i].Distance(r.p[(i + 1) % n]);
        meanR += r.p[i].Distance(apex);
    }
    meanEdge /= double(n);
    meanR /= double(n);
    if (meanEdge < 1e-12 || meanR < 1e-12) return false;
    int steps = int(std::lround(meanR / meanEdge));
    steps = std::clamp(steps, 2, 16);

    const bool flip = face.Orientation() == TopAbs_REVERSED;

    // Layer 0 = the rim (the contract, its own vertices). Inner layers
    // homothety-shrink the rim toward the UV centroid and re-evaluate on
    // the surface. The innermost layer is closed as a single n-gon.
    // Compute every layer's points FIRST (no mesh mutation yet). Layer 0 is
    // the rim; inner layers homothety-shrink it toward the UV centroid and
    // re-evaluate on the surface.
    std::vector<std::vector<gp_Pnt>> P(steps, std::vector<gp_Pnt>(n));
    std::vector<std::vector<gp_Pnt2d>> UV(steps, std::vector<gp_Pnt2d>(n));
    for (size_t i = 0; i < n; ++i) {
        P[0][i] = r.p[i];
        UV[0][i] = r.uv[i];
    }
    for (int k = 1; k < steps; ++k) {
        const double f = double(steps - k) / double(steps);
        for (size_t i = 0; i < n; ++i) {
            const double uu = cu + f * (r.uv[i].X() - cu);
            const double vv = cv + f * (r.uv[i].Y() - cv);
            try {
                P[k][i] = surf.Value(uu, vv);
            } catch (const Standard_Failure&) {
                return false;
            }
            UV[k][i] = gp_Pnt2d(uu, vv);
        }
    }
    // Fold guard: a strongly-dished cap can shrink a ring past a curvature
    // crease and flip a cell over the surface. Sign each ring cell by its
    // normal dotted with the analytic surface normal at the cell's UV
    // centroid; a clean cap is sign-consistent (the sign only encodes face
    // orientation). If any cell disagrees with the others, a cell folds --
    // bail so quad-fill owns the face instead of shipping the fold.
    int pos = 0, neg = 0;
    for (int k = 0; k + 1 < steps; ++k) {
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            const gp_Vec cn = gp_Vec(P[k][i], P[k][j])
                                  .Crossed(gp_Vec(P[k][i], P[k + 1][i]));
            const double mu = 0.25 * (UV[k][i].X() + UV[k][j].X() +
                                      UV[k + 1][j].X() + UV[k + 1][i].X());
            const double mv = 0.25 * (UV[k][i].Y() + UV[k][j].Y() +
                                      UV[k + 1][j].Y() + UV[k + 1][i].Y());
            gp_Pnt sp;
            gp_Vec sdu, sdv;
            try {
                surf.D1(mu, mv, sp, sdu, sdv);
            } catch (const Standard_Failure&) {
                return false;
            }
            const gp_Vec sn = sdu.Crossed(sdv);
            if (cn.Magnitude() > 1e-18 && sn.Magnitude() > 1e-18) {
                (cn.Dot(sn) >= 0 ? pos : neg)++;
            }
        }
    }
    if (pos > 0 && neg > 0) return false;  // a cell folds -> quad-fill owns it

    // Clean: commit the vertices and cells.
    std::vector<std::vector<uint32_t>> layer(steps);
    for (int k = 0; k < steps; ++k) {
        layer[k].resize(n);
        for (size_t i = 0; i < n; ++i) {
            layer[k][i] =
                out.addVertex(P[k][i], {faceId, UV[k][i].X(), UV[k][i].Y()});
        }
    }
    for (int k = 0; k + 1 < steps; ++k) {
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            out.addPolygon({layer[k][i], layer[k][j], layer[k + 1][j],
                            layer[k + 1][i]},
                           faceId, flip);
        }
    }
    out.addPolygon(layer[steps - 1], faceId, flip);
    dbg("disk cap %d: %zu rim verts, %d ring(s) + central %zu-gon", faceId,
        n, steps - 1, n);
    return true;
}

bool meshQuadFill(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  const Model& model, int faceId,
                  const std::vector<int>& solvedEdge, int radialDefault,
                  const FaceMeshSettings& fs, MeshBuilder& out,
                  int gridUOverride, int gridVOverride) {
    // A curved dished cap (coons rejected it for want of four corners)
    // gets clean concentric quad rings + a central n-gon instead of the
    // grid+CDT rim's tri fan. Tightly gated inside; falls through here on
    // any doubt so no other quad-fill face is disturbed.
    if (meshDiskCap(face, surf, model, faceId, solvedEdge, radialDefault,
                    out)) {
        return true;
    }
    const double minSize = fs.minSize;
    std::vector<PlanarRing> rings;
    if (!samplePlanarRings(face, model, solvedEdge, radialDefault, rings)) {
        return false;
    }
    if (rings.empty()) return false;
    const bool flip = face.Orientation() == TopAbs_REVERSED;

    // Boundary segments (for spacing, containment, and clearance tests).
    struct Seg {
        gp_Pnt2d a, b;
    };
    std::vector<Seg> segs;
    std::vector<double> lens;
    double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
    for (const PlanarRing& r : rings) {
        for (size_t i = 0; i < r.uv.size(); ++i) {
            const gp_Pnt2d& a = r.uv[i];
            const gp_Pnt2d& b = r.uv[(i + 1) % r.uv.size()];
            segs.push_back({a, b});
            lens.push_back(a.Distance(b));
            umin = std::min(umin, a.X());
            umax = std::max(umax, a.X());
            vmin = std::min(vmin, a.Y());
            vmax = std::max(vmax, a.Y());
        }
    }
    if (segs.size() < 3) return false;
    std::sort(lens.begin(), lens.end());
    // Slightly finer than the border spacing: the rim web needs a cell of
    // clearance, so a coarser grid would waste most of the face on rim.
    // NOTE lens are UV distances; on a curved surface the metric differs
    // per direction, so the cell size splits into hu/hv using the surface
    // derivatives at the patch centre (a curved patch's UV chart can be
    // arbitrarily anisotropic — a cylinder's u is an angle).
    double h3 = std::max(0.55 * lens[lens.size() / 2], minSize);
    if (h3 < 1e-12) return false;
    double su = 1.0, sv = 1.0;
    {
        gp_Pnt sp;
        gp_Vec du, dv;
        surf.D1((umin + umax) / 2, (vmin + vmax) / 2, sp, du, dv);
        su = std::max(1e-9, du.Magnitude());
        sv = std::max(1e-9, dv.Magnitude());
    }
    // lens were measured in UV; estimate the 3D border spacing and derive
    // per-direction UV cell sizes from it.
    const double uvToWorld = (su + sv) / 2;
    double hWorld = h3 * uvToWorld;
    double hu = hWorld / su, hv = hWorld / sv;
    const double huBorder = hu, hvBorder = hv;  // rim clearance scale
    // Per-DIRECTION geometry sizing: the border spacing says how fine the
    // rims are, not how much the surface bends. On a barrel chart the v
    // direction is ruled — stamping the rim's 1.7mm arc spacing onto a
    // 60mm straight axis buys a thousand cells that say nothing. Each
    // direction follows its own centre iso-curve under the face's
    // deviation/angle budget; a straight direction costs ONE row.
    {
        auto isoCount = [&](bool uDir) -> int {
            try {
                Handle(Geom_Surface) S = BRep_Tool::Surface(face);
                if (S.IsNull()) return 0;
                Handle(Geom_Curve) iso =
                    uDir ? S->VIso((vmin + vmax) / 2)
                         : S->UIso((umin + umax) / 2);
                if (iso.IsNull()) return 0;
                const double a = uDir ? umin : vmin;
                const double b = uDir ? umax : vmax;
                GeomAdaptor_Curve gc(iso, a, b);
                double chord = std::max(1e-9, fs.chordTolerance);
                if (fs.relativeDeviation) {
                    const gp_Pnt pf = gc.Value(a);
                    const gp_Pnt pl = gc.Value(b);
                    const gp_Pnt pm = gc.Value(0.5 * (a + b));
                    const double extent = std::max(
                        {pf.Distance(pl), pf.Distance(pm), 1e-6});
                    chord = std::max(chord * 0.2 * extent, 1e-9);
                }
                const double ang =
                    std::max(1.0, fs.angleToleranceDeg) * M_PI / 180.0;
                return std::clamp(stableDeflectionCount(gc, ang, chord), 1,
                                  256);
            } catch (const Standard_Failure&) {
                return 0;
            }
        };
        const int gu = isoCount(true);
        const int gv = isoCount(false);
        // Flat charts keep the border-driven grid (plates in quad-
        // dominant mode deliberately grid at border density).
        if (gu > 1 || gv > 1) {
            if (gu > 0) {
                hu = std::max((umax - umin) / std::max(1, gu), huBorder);
            }
            if (gv > 0) {
                hv = std::max((vmax - vmin) / std::max(1, gv), hvBorder);
            }
            // Hole wires still demand rows fine enough that each hole
            // spans whole cells in the straight direction too.
            for (size_t ri = 0; ri < rings.size(); ++ri) {
                if (rings[ri].isOuter) continue;
                double hu0 = 1e300, hu1 = -1e300;
                double hv0 = 1e300, hv1 = -1e300;
                for (const gp_Pnt2d& q : rings[ri].uv) {
                    hu0 = std::min(hu0, q.X());
                    hu1 = std::max(hu1, q.X());
                    hv0 = std::min(hv0, q.Y());
                    hv1 = std::max(hv1, q.Y());
                }
                const double needU = (hu1 - hu0) + 2.0 * huBorder;
                const double needV = (hv1 - hv0) + 2.0 * hvBorder;
                if (needU > 4.0 * huBorder) hu = std::min(hu, needU);
                if (needV > 4.0 * hvBorder) hv = std::min(hv, needV);
            }
        }
    }
    // Cap the grid size; a tiny median segment on a huge plate would
    // otherwise explode the cell count.
    while ((umax - umin) / hu * ((vmax - vmin) / hv) > 20000.0) {
        hu *= 1.5;
        hv *= 1.5;
    }

    auto insideDomain = [&](const gp_Pnt2d& p) {
        int crossings = 0;
        for (const Seg& s : segs) {
            if ((s.a.Y() > p.Y()) == (s.b.Y() > p.Y())) continue;
            double x = s.a.X() + (p.Y() - s.a.Y()) / (s.b.Y() - s.a.Y()) *
                                     (s.b.X() - s.a.X());
            if (x > p.X()) ++crossings;
        }
        return (crossings & 1) != 0;
    };

    // The grid lives in a box inset by one BORDER cell per side: the
    // rim web needs its clearance by construction — a 1-row grid has no
    // sacrificial rows for the culling to eat.
    const double iu0 = umin + huBorder, iu1 = umax - huBorder;
    const double iv0 = vmin + hvBorder, iv1 = vmax - hvBorder;
    // Explicit interior grid density: gridU/gridV name the interior column/
    // row counts directly, an alternative to the geometry/border-driven
    // spacing above (the `boundary` key sizes the RIM; these size the
    // interior grid). Only when the user typed them (differs from the model
    // default) — otherwise the spacing above stands and default output is
    // byte-identical. Sized on the inset box so nx/ny land on the request;
    // the coverage pass below may still add rows, never remove them.
    if (gridUOverride > 0) hu = std::max(1e-9, (iu1 - iu0) / gridUOverride);
    if (gridVOverride > 0) hv = std::max(1e-9, (iv1 - iv0) / gridVOverride);
    if (iu1 - iu0 < 0.5 * hu || iv1 - iv0 < 0.5 * hv) return false;
    // Face uv area (outer rings minus holes): the coverage check below
    // compares kept-cell area against it.
    double faceArea = 0;
    for (const PlanarRing& r : rings) {
        double a = 0;
        for (size_t i = 0; i < r.uv.size(); ++i) {
            const gp_Pnt2d& pa = r.uv[i];
            const gp_Pnt2d& pb = r.uv[(i + 1) % r.uv.size()];
            a += pa.X() * pb.Y() - pb.X() * pa.Y();
        }
        faceArea += (r.isOuter ? 1.0 : -1.0) * std::abs(a / 2);
    }
    faceArea = std::max(1e-12, faceArea);
    int nx = 1, ny = 1;
    double u0 = iu0, v0 = iv0;
    double marginU = 0, marginV = 0;
    std::vector<char> keep;
    // Geometry sizing gives the LEAN grid; a castellated or strongly
    // concave outline can cull most of it, leaving the rim web to fan
    // across the face — worse than a denser grid. Densify until the
    // kept cells actually cover the face (floored at border spacing;
    // when both directions are already there, this is one pass).
    // Build the grid at the current hu/hv: fill nx/ny/u0/v0/keep and
    // return the number of kept (emitted) cells.
    auto buildGrid = [&]() -> int {
        nx = std::max(1, int((iu1 - iu0) / hu));
        ny = std::max(1, int((iv1 - iv0) / hv));
        // Center the grid in the inset box so border cells get equal
        // clearance on both sides instead of flush against one edge.
        u0 = iu0 + 0.5 * ((iu1 - iu0) - nx * hu);
        v0 = iv0 + 0.5 * ((iv1 - iv0) - ny * hv);

        // Bin boundary segments by grid row so the per-cell clearance
        // test only looks at nearby geometry.
        std::vector<std::vector<int>> rowSegs(ny + 1);
        for (int si = 0; si < int(segs.size()); ++si) {
            double y0s = std::min(segs[si].a.Y(), segs[si].b.Y()) - hv;
            double y1s = std::max(segs[si].a.Y(), segs[si].b.Y()) + hv;
            int j0 = std::max(0, int(std::floor((y0s - v0) / hv)));
            int j1 = std::min(ny, int(std::floor((y1s - v0) / hv)) + 1);
            for (int j = j0; j <= j1; ++j) rowSegs[j].push_back(si);
        }

        // A cell is kept when its four corners are inside the domain and
        // no boundary segment comes near its (slightly inflated) box —
        // the rim web needs breathing room to stay well-shaped.
        // Clearance margins scale with the BORDER spacing, not the cell:
        // a full-height cell inflated by 30% of itself always overlaps
        // the rims and the whole grid self-culls to nothing.
        marginU = 0.30 * std::min(hu, huBorder);
        marginV = 0.30 * std::min(hv, hvBorder);
        keep.assign(size_t(nx) * ny, 0);
        int kept = 0;
        for (int j = 0; j < ny; ++j) {
            for (int i = 0; i < nx; ++i) {
                bool ok = true;
                for (int c = 0; c < 4 && ok; ++c) {
                    ok = insideDomain(
                        gp_Pnt2d(u0 + (i + (c & 1)) * hu,
                                 v0 + (j + (c >> 1)) * hv));
                }
                if (!ok) continue;
                double x0 = u0 + i * hu - marginU;
                double x1 = x0 + hu + 2 * marginU;
                double y0 = v0 + j * hv - marginV;
                double y1 = y0 + hv + 2 * marginV;
                for (int si : rowSegs[j]) {
                    const Seg& s = segs[si];
                    // Conservative: reject when the segment's box overlaps
                    // the inflated cell box.
                    if (std::max(s.a.X(), s.b.X()) < x0 ||
                        std::min(s.a.X(), s.b.X()) > x1 ||
                        std::max(s.a.Y(), s.b.Y()) < y0 ||
                        std::min(s.a.Y(), s.b.Y()) > y1) {
                        continue;
                    }
                    ok = false;
                    break;
                }
                if (ok) {
                    keep[size_t(j) * nx + i] = 1;
                    ++kept;
                }
            }
        }
        return kept;
    };
    int keptCells = 0;
    // An explicit gridU/gridV is the user's exact interior density; the
    // coverage heuristic must not silently refine it away, so skip the
    // densify pass when either was set (the cellCap guard below still bites
    // a genuine pathology).
    const bool gridPinned = gridUOverride > 0 || gridVOverride > 0;
    for (int attempt = 0; !gridPinned; ++attempt) {
        keptCells = buildGrid();
        const double coverage = keptCells * hu * hv / faceArea;
        const bool canShrink =
            hu > 1.05 * huBorder || hv > 1.05 * hvBorder;
        if (coverage >= 0.45 || !canShrink || attempt >= 4) break;
        hu = std::max(huBorder, hu / 1.7);
        hv = std::max(hvBorder, hv / 1.7);
    }
    if (gridPinned) keptCells = buildGrid();
    // Per-face pathology guard on ACTUAL emitted cells (fs.cellCap): an
    // offset surface whose fine border/curvature drove the interior far
    // past its area-share budget gets coarsened until it fits. A sane
    // face exits the loop above already under its budget and never
    // enters here, so its grid — and output — is byte-for-byte unchanged.
    if (fs.cellCap > 0) {
        for (int guard = 0;
             keptCells > fs.cellCap && (nx > 1 || ny > 1) && guard < 48;
             ++guard) {
            hu = std::min(iu1 - iu0, hu * 1.3);
            hv = std::min(iv1 - iv0, hv * 1.3);
            keptCells = buildGrid();
        }
    }
    auto cornerUV = [&](int i, int j) {
        return gp_Pnt2d(u0 + i * hu, v0 + j * hv);
    };
    auto keepAt = [&](int i, int j) -> char& {
        return keep[size_t(j) * nx + i];
    };

    // Diagonal pinches (two kept cells touching only at a corner) would
    // give that corner four frontier edges; drop one cell until clean.
    for (bool changed = true; changed;) {
        changed = false;
        for (int j = 0; j + 1 < ny; ++j) {
            for (int i = 0; i + 1 < nx; ++i) {
                char& a = keepAt(i, j);
                char& b = keepAt(i + 1, j + 1);
                char& c = keepAt(i + 1, j);
                char& d = keepAt(i, j + 1);
                if (a && b && !c && !d) { a = 0; changed = true; }
                else if (c && d && !a && !b) { c = 0; changed = true; }
            }
        }
    }

    auto kept = [&](int i, int j) {
        return i >= 0 && j >= 0 && i < nx && j < ny && keepAt(i, j);
    };

    // Interior quad grid. Corner vertices are created on demand and shared
    // with the frontier loops, so the rim web welds to the grid exactly.
    std::map<std::pair<int, int>, uint32_t> cornerVert;
    auto vertAt = [&](int i, int j) {
        auto it = cornerVert.find({i, j});
        if (it != cornerVert.end()) return it->second;
        gp_Pnt2d uv = cornerUV(i, j);
        uint32_t v = out.addVertex(surf.Value(uv.X(), uv.Y()),
                                   {faceId, uv.X(), uv.Y()});
        cornerVert[{i, j}] = v;
        return v;
    };
    bool anyCell = false;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (!keepAt(i, j)) continue;
            anyCell = true;
            out.addPolygon({vertAt(i, j), vertAt(i + 1, j),
                            vertAt(i + 1, j + 1), vertAt(i, j + 1)},
                           faceId, flip);
        }
    }

    // Ring -> web points, and boundary ring vertices (anchorless).
    auto ringWeb = [&](const PlanarRing& r) {
        std::vector<WebPoint> web;
        for (size_t i = 0; i < r.uv.size(); ++i) {
            web.push_back({r.uv[i], out.addVertex(r.p[i], {})});
        }
        return web;
    };
    std::vector<WebPoint> faceOuter;
    std::vector<std::vector<WebPoint>> faceHoles;
    for (const PlanarRing& r : rings) {
        if (r.isOuter) faceOuter = ringWeb(r);
        else faceHoles.push_back(ringWeb(r));
    }
    if (faceOuter.size() < 3) return false;

    if (!anyCell) {  // no room for a grid: plain web over the whole face
        return triangulateWeb(std::move(faceOuter), std::move(faceHoles),
                              faceId, flip, out);
    }

    // Frontier: kept-region boundary edges, traced into closed loops on
    // the grid corners (pinch removal guarantees two frontier edges per
    // frontier corner).
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> adj;
    auto frontierEdge = [&](int i0, int j0, int i1, int j1) {
        adj[{i0, j0}].push_back({i1, j1});
        adj[{i1, j1}].push_back({i0, j0});
    };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            if (!keepAt(i, j)) continue;
            if (!kept(i, j - 1)) frontierEdge(i, j, i + 1, j);
            if (!kept(i, j + 1)) frontierEdge(i, j + 1, i + 1, j + 1);
            if (!kept(i - 1, j)) frontierEdge(i, j, i, j + 1);
            if (!kept(i + 1, j)) frontierEdge(i + 1, j, i + 1, j + 1);
        }
    }
    std::vector<std::vector<std::pair<int, int>>> frontierLoops;
    std::set<std::pair<std::pair<int, int>, std::pair<int, int>>> used;
    for (const auto& [start, nbrs] : adj) {
        for (const auto& first : nbrs) {
            if (used.count({start, first})) continue;
            std::vector<std::pair<int, int>> loop{start};
            std::pair<int, int> prev = start, cur = first;
            used.insert({start, first});
            used.insert({first, start});  // one traversal per edge
            bool closed = false;
            for (size_t guard = 0; guard < adj.size() * 4 + 4; ++guard) {
                if (cur == start) { closed = true; break; }
                loop.push_back(cur);
                const auto& next = adj.at(cur);
                std::pair<int, int> step{-1, -1};
                for (const auto& n : next) {
                    if (n != prev && !used.count({cur, n})) {
                        step = n;
                        break;
                    }
                }
                if (step.first < 0) break;
                used.insert({cur, step});
                used.insert({step, cur});
                prev = cur;
                cur = step;
            }
            if (closed && loop.size() >= 4) {
                frontierLoops.push_back(std::move(loop));
            }
        }
    }
    if (frontierLoops.empty()) return false;  // shouldn't happen with cells

    // Every frontier loop either encloses a web POCKET (its inside is not
    // kept: it acts as that pocket's outer ring) or wraps a kept ISLAND
    // (it is a hole of the enclosing web region).
    struct Region {
        std::vector<WebPoint> outer;
        std::vector<std::vector<WebPoint>> holes;
        double area = 0;  // |signed| of the outer, for nesting
    };
    std::vector<Region> regions;
    regions.push_back({std::move(faceOuter), {}, 1e300});

    auto loopWeb = [&](const std::vector<std::pair<int, int>>& loop) {
        std::vector<WebPoint> web;
        for (const auto& [i, j] : loop) web.push_back({cornerUV(i, j),
                                                       vertAt(i, j)});
        return web;
    };
    auto signedAreaOf = [&](const std::vector<WebPoint>& web) {
        double a = 0;
        for (size_t i = 0; i < web.size(); ++i) {
            const gp_Pnt2d& p = web[i].uv;
            const gp_Pnt2d& q = web[(i + 1) % web.size()].uv;
            a += p.X() * q.Y() - q.X() * p.Y();
        }
        return a / 2;
    };
    std::vector<std::vector<WebPoint>> pendingHoles;
    for (const auto& loop : frontierLoops) {
        std::vector<WebPoint> web = loopWeb(loop);
        double area = signedAreaOf(web);
        // A point just inside the loop: offset from the first edge's
        // midpoint toward the interior; kept there => island (hole).
        gp_Pnt2d m((web[0].uv.X() + web[1].uv.X()) / 2,
                   (web[0].uv.Y() + web[1].uv.Y()) / 2);
        gp_Pnt2d dir(web[1].uv.X() - web[0].uv.X(),
                     web[1].uv.Y() - web[0].uv.Y());
        double side = area > 0 ? 1.0 : -1.0;  // interior is left of CCW
        // Perpendicular offset clamped to a fraction of the cell step:
        // on anisotropic grids a quarter of a LONG edge can jump past
        // the neighbouring row and misclassify an island as a pocket.
        double offX = -side * dir.Y() * 0.25;
        double offY = side * dir.X() * 0.25;
        if (std::abs(offX) > 0.4 * hu) offX *= 0.4 * hu / std::abs(offX);
        if (std::abs(offY) > 0.4 * hv) offY *= 0.4 * hv / std::abs(offY);
        gp_Pnt2d probe(m.X() + offX, m.Y() + offY);
        int pi = int(std::floor((probe.X() - u0) / hu));
        int pj = int(std::floor((probe.Y() - v0) / hv));
        if (kept(pi, pj)) {
            // Island: a hole of whichever region contains it.
            if (area > 0) {
                std::reverse(web.begin(), web.end());  // holes wind CW
            }
            pendingHoles.push_back(std::move(web));
        } else {
            // Pocket: its own web region, outer CCW.
            if (area < 0) std::reverse(web.begin(), web.end());
            regions.push_back({std::move(web), {}, std::abs(area)});
        }
    }
    for (auto& hole : faceHoles) pendingHoles.push_back(std::move(hole));

    // Assign each hole to the smallest region whose outer contains it.
    auto containsPoint = [&](const std::vector<WebPoint>& ring,
                             const gp_Pnt2d& p) {
        int crossings = 0;
        for (size_t i = 0; i < ring.size(); ++i) {
            const gp_Pnt2d& a = ring[i].uv;
            const gp_Pnt2d& b = ring[(i + 1) % ring.size()].uv;
            if ((a.Y() > p.Y()) == (b.Y() > p.Y())) continue;
            double x = a.X() + (p.Y() - a.Y()) / (b.Y() - a.Y()) *
                                   (b.X() - a.X());
            if (x > p.X()) ++crossings;
        }
        return (crossings & 1) != 0;
    };
    for (auto& hole : pendingHoles) {
        int best = 0;
        double bestArea = 1e300;
        for (size_t r = 0; r < regions.size(); ++r) {
            if (regions[r].area >= bestArea) continue;
            if (r == 0 || containsPoint(regions[r].outer, hole[0].uv)) {
                best = int(r);
                bestArea = regions[r].area;
            }
        }
        regions[best].holes.push_back(std::move(hole));
    }
    for (Region& r : regions) {
        // A simple band (one boundary ring around one frontier ring, or a
        // frontier pocket around one hole ring) zippers into flowing
        // quads/tris; anything more complex keeps the ear-clipped web.
        bool zipped = false;
        if (r.holes.size() == 1 && r.outer.size() >= 3 &&
            r.holes[0].size() >= 3) {
            zipped = zipperRings(r.outer, r.holes[0], faceId, flip, out);
        }
        if (!zipped &&
            !triangulateWeb(std::move(r.outer), std::move(r.holes), faceId,
                            flip, out)) {
            return false;
        }
    }
    return true;
}


}  // namespace weft::mesher_impl
