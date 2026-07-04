#include "weft/validate.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <map>
#include <set>
#include <utility>

namespace weft {

namespace {

gp_Pnt at(const PolyMesh& mesh, uint32_t i) {
    const auto& v = mesh.vertices[i];
    return gp_Pnt(v[0], v[1], v[2]);
}

// Sum of cross products over a fan: robust area for planar-ish polygons.
double polygonArea(const PolyMesh& mesh, const std::vector<uint32_t>& poly) {
    gp_Vec acc(0, 0, 0);
    gp_Pnt origin = at(mesh, poly[0]);
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        gp_Vec a(origin, at(mesh, poly[i]));
        gp_Vec b(origin, at(mesh, poly[i + 1]));
        acc += a.Crossed(b);
    }
    return 0.5 * acc.Magnitude();
}

double minCornerAngleDeg(const PolyMesh& mesh, const std::vector<uint32_t>& poly) {
    double best = 180.0;
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        gp_Pnt p = at(mesh, poly[i]);
        gp_Vec e1(p, at(mesh, poly[(i + 1) % n]));
        gp_Vec e2(p, at(mesh, poly[(i + n - 1) % n]));
        if (e1.Magnitude() < 1e-12 || e2.Magnitude() < 1e-12) return 0.0;
        best = std::min(best, e1.Angle(e2) * 180.0 / M_PI);
    }
    return best;
}

}  // namespace

ValidationReport validateMesh(const PolyMesh& mesh, const Model* model) {
    ValidationReport r;
    r.polygons = mesh.polygons.size();

    // Edge usage: count per undirected edge, and track directed traversal
    // to detect winding conflicts on interior edges.
    struct EdgeUse {
        int count = 0;
        int forward = 0;
        int firstPoly = -1;
    };
    std::map<std::pair<uint32_t, uint32_t>, EdgeUse> edges;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        const size_t n = poly.size();
        std::set<uint32_t> unique(poly.begin(), poly.end());
        if (unique.size() != n) ++r.degeneratePolygons;

        for (size_t i = 0; i < n; ++i) {
            uint32_t a = poly[i];
            uint32_t b = poly[(i + 1) % n];
            if (a == b) continue;
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            EdgeUse& use = edges[key];
            ++use.count;
            if (a < b) ++use.forward;
            if (use.firstPoly < 0) use.firstPoly = static_cast<int>(p);
        }
    }
    std::map<int, size_t> openPerFace;
    std::vector<std::pair<uint32_t, uint32_t>> openList;
    std::vector<int> openFace;
    for (const auto& [key, use] : edges) {
        if (use.count == 1) {
            ++r.openEdges;
            ++openPerFace[mesh.polygonFaceId[use.firstPoly]];
            openList.push_back(key);
            openFace.push_back(mesh.polygonFaceId[use.firstPoly]);
        } else if (use.count > 2) {
            ++r.nonManifoldEdges;
        } else if (use.forward != 1) {
            ++r.windingConflicts;  // 0 or 2: both polygons run the same way
        }
    }

    // Split the open edges into "along an input open-shell boundary"
    // (expected — the source geometry simply is not closed there) and
    // everything else (real cracks). Boundary curves are sampled into
    // polylines; an open mesh edge counts as expected when both its
    // endpoints sit within a mesh-edge-length of one.
    if (model && !openList.empty() && !mesh.vertices.empty()) {
        std::vector<std::array<gp_Pnt, 2>> boundarySegs;
        for (int eid = 1; eid <= model->edgeCount(); ++eid) {
            if (model->edgeToFaces.FindFromIndex(eid).Extent() >= 2) continue;
            const TopoDS_Edge edge = TopoDS::Edge(model->edges(eid));
            if (BRep_Tool::Degenerated(edge)) continue;
            double f = 0, l = 0;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, f, l);
            if (curve.IsNull()) continue;
            const int kSamples = 128;
            gp_Pnt prev = curve->Value(f);
            for (int i = 1; i <= kSamples; ++i) {
                gp_Pnt next = curve->Value(f + (l - f) * i / kSamples);
                boundarySegs.push_back({prev, next});
                prev = next;
            }
        }
        auto distToSeg = [](const gp_Pnt& p, const gp_Pnt& a, const gp_Pnt& b) {
            gp_Vec ab(a, b), ap(a, p);
            double len2 = ab.SquareMagnitude();
            double t = len2 < 1e-30 ? 0.0
                                    : std::min(1.0, std::max(0.0, ap.Dot(ab) / len2));
            gp_Pnt q(a.X() + t * ab.X(), a.Y() + t * ab.Y(), a.Z() + t * ab.Z());
            return p.Distance(q);
        };
        auto nearBoundary = [&](const gp_Pnt& p, double tol) {
            for (const auto& seg : boundarySegs) {
                if (distToSeg(p, seg[0], seg[1]) <= tol) return true;
            }
            return false;
        };
        for (size_t i = 0; i < openList.size(); ++i) {
            gp_Pnt a = at(mesh, openList[i].first);
            gp_Pnt b = at(mesh, openList[i].second);
            const double tol = std::max(2.0 * a.Distance(b), 1e-6);
            if (nearBoundary(a, tol) && nearBoundary(b, tol)) {
                ++r.openEdgesOnInputBoundary;
                auto it = openPerFace.find(openFace[i]);
                if (it != openPerFace.end() && it->second > 0) --it->second;
            }
        }
    }

    r.leakyFaces.assign(openPerFace.begin(), openPerFace.end());
    r.leakyFaces.erase(std::remove_if(r.leakyFaces.begin(), r.leakyFaces.end(),
                                      [](const auto& e) { return e.second == 0; }),
                       r.leakyFaces.end());
    std::sort(r.leakyFaces.begin(), r.leakyFaces.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    double areaScale = 0.0;
    for (const auto& poly : mesh.polygons) {
        areaScale = std::max(areaScale, polygonArea(mesh, poly));
    }
    const double areaEps = std::max(1e-30, areaScale * 1e-12);
    for (const auto& poly : mesh.polygons) {
        if (polygonArea(mesh, poly) < areaEps) {
            ++r.degeneratePolygons;
            continue;
        }
        if (minCornerAngleDeg(mesh, poly) < r.sliverAngleDeg) ++r.sliverPolygons;
    }

    // Chord deviation: for polygons whose corners all anchor to the polygon's
    // source face, compare the corner centroid with the surface evaluated at
    // the centroid of the corner UVs.
    if (model) {
        for (int eid = 1; eid <= model->edgeCount(); ++eid) {
            const int adj = model->edgeToFaces.FindFromIndex(eid).Extent();
            if (adj < 2) {
                // Degenerate edges (sphere poles, cone apexes) border one
                // face but are points — not real open boundary.
                if (!BRep_Tool::Degenerated(TopoDS::Edge(model->edges(eid)))) {
                    ++r.inputBoundaryEdges;
                }
            } else if (adj > 2) {
                ++r.inputNonManifoldEdges;
            }
        }
        double sum = 0.0;
        std::map<int, BRepAdaptor_Surface> surfCache;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            const auto& poly = mesh.polygons[p];
            const int fid = mesh.polygonFaceId[p];
            if (fid < 1 || fid > model->faceCount()) continue;

            double cu = 0, cv = 0, cx = 0, cy = 0, cz = 0;
            double uLo = 1e300, uHi = -1e300, vLo = 1e300, vHi = -1e300;
            bool anchored = true;
            for (uint32_t idx : poly) {
                const Anchor& a = mesh.anchors[idx];
                if (a.faceId != fid) { anchored = false; break; }
                cu += a.u;
                cv += a.v;
                uLo = std::min(uLo, a.u);
                uHi = std::max(uHi, a.u);
                vLo = std::min(vLo, a.v);
                vHi = std::max(vHi, a.v);
                cx += mesh.vertices[idx][0];
                cy += mesh.vertices[idx][1];
                cz += mesh.vertices[idx][2];
            }
            if (!anchored) continue;
            const double n = double(poly.size());
            const TopoDS_Face face = TopoDS::Face(model->faces(fid));
            auto cacheIt = surfCache.find(fid);
            if (cacheIt == surfCache.end()) {
                cacheIt = surfCache.emplace(fid, BRepAdaptor_Surface(face)).first;
            }
            BRepAdaptor_Surface& surf = cacheIt->second;
            // Polygons that wrap a periodic seam (a cylinder's closing
            // quads run from u ~ 2*pi back to u = 0) would average their
            // UVs across the whole period and report a bogus deviation.
            if (surf.IsUPeriodic() && uHi - uLo > 0.5 * surf.UPeriod()) continue;
            if (surf.IsVPeriodic() && vHi - vLo > 0.5 * surf.VPeriod()) continue;
            gp_Pnt centroid(cx / n, cy / n, cz / n);
            double d = surf.Value(cu / n, cv / n).Distance(centroid);
            // The averaged-UV estimate misreads polygons touching poles or
            // apexes (their collapsed vertex carries an arbitrary u). Sag
            // beyond a tenth of the polygon's own radius is rare on real
            // meshes, so above that measure the true distance instead —
            // the estimate then only ever errs on the high side of truth.
            double radius2 = 0.0;
            for (uint32_t idx : poly) {
                radius2 = std::max(radius2,
                                   centroid.SquareDistance(at(mesh, idx)));
            }
            if (d * d > 0.01 * radius2 && d > 1e-12) {
                TopLoc_Location loc;
                Handle(Geom_Surface) hs = BRep_Tool::Surface(face);
                if (!hs.IsNull()) {
                    GeomAPI_ProjectPointOnSurf proj(centroid, hs);
                    if (proj.NbPoints() > 0) {
                        d = std::min(d, proj.LowerDistance());
                    }
                }
            }
            r.maxDeviation = std::max(r.maxDeviation, d);
            sum += d;
            ++r.deviationSamples;
        }
        if (r.deviationSamples) r.meanDeviation = sum / r.deviationSamples;
    }
    return r;
}

std::string formatReport(const ValidationReport& r) {
    char buf[512];
    std::string out;
    std::snprintf(buf, sizeof buf, "  watertight:     %s (open edges: %zu, non-manifold: %zu)\n",
                  r.watertight() ? "yes" : "NO", r.openEdges, r.nonManifoldEdges);
    out += buf;
    if (r.inputBoundaryEdges > 0 && r.openEdges > 0) {
        std::snprintf(buf, sizeof buf,
                      "                  (%zu of the open edges run along the input's "
                      "%zu open-shell B-rep edges — the source isn't closed there; "
                      "%zu are unexplained cracks)\n",
                      r.openEdgesOnInputBoundary, r.inputBoundaryEdges,
                      r.openEdges - r.openEdgesOnInputBoundary);
        out += buf;
    }
    if (r.inputNonManifoldEdges > 0 && r.nonManifoldEdges > 0) {
        std::snprintf(buf, sizeof buf,
                      "                  (input B-rep has %zu non-manifold edges; "
                      "the welded mesh necessarily mirrors them)\n",
                      r.inputNonManifoldEdges);
        out += buf;
    }
    if (!r.leakyFaces.empty()) {
        std::snprintf(buf, sizeof buf, "  leakiest faces: ");
        out += buf;
        size_t shown = 0;
        for (const auto& [fid, count] : r.leakyFaces) {
            if (++shown > 8) break;
            std::snprintf(buf, sizeof buf, "#%d(%zu) ", fid, count);
            out += buf;
        }
        out += "\n";
    }
    std::snprintf(buf, sizeof buf, "  winding:        %s (%zu conflicting edge pairs)\n",
                  r.windingConflicts == 0 ? "consistent" : "INCONSISTENT",
                  r.windingConflicts);
    out += buf;
    std::snprintf(buf, sizeof buf, "  degenerate:     %zu polygons, %zu slivers (< %.1f deg)\n",
                  r.degeneratePolygons, r.sliverPolygons, r.sliverAngleDeg);
    out += buf;
    if (r.deviationSamples) {
        std::snprintf(buf, sizeof buf,
                      "  chord deviation: max %.6g, mean %.6g (%zu polygons sampled)\n",
                      r.maxDeviation, r.meanDeviation, r.deviationSamples);
        out += buf;
    }
    return out;
}

}  // namespace weft
