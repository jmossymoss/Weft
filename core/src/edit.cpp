#include "weft/edit.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Surface.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>

namespace weft {

namespace {

Handle(Geom_Surface) surfaceOf(const Model& model, int faceId) {
    if (faceId < 1 || faceId > model.faceCount()) {
        throw std::runtime_error("bad face id: " + std::to_string(faceId));
    }
    return BRep_Tool::Surface(TopoDS::Face(model.faces(faceId)));
}

using EdgeKey = std::pair<uint32_t, uint32_t>;

EdgeKey keyOf(uint32_t a, uint32_t b) {
    return a < b ? EdgeKey{a, b} : EdgeKey{b, a};
}

}  // namespace

Anchor snapToFace(const Model& model, int faceId, std::array<double, 3>& p) {
    GeomAPI_ProjectPointOnSurf proj(gp_Pnt(p[0], p[1], p[2]),
                                    surfaceOf(model, faceId));
    if (!proj.IsDone() || proj.NbPoints() < 1) {
        throw std::runtime_error("projection failed on face " +
                                 std::to_string(faceId));
    }
    gp_Pnt q = proj.NearestPoint();
    Anchor a{faceId, 0.0, 0.0};
    proj.LowerDistanceParameters(a.u, a.v);
    p = {q.X(), q.Y(), q.Z()};
    return a;
}

void moveVertex(PolyMesh& mesh, const Model& model, size_t vertIdx,
                const std::array<double, 3>& target) {
    Anchor& anchor = mesh.anchors.at(vertIdx);
    if (anchor.faceId == 0) {
        mesh.vertices[vertIdx] = target;  // unanchored: free move
        return;
    }
    std::array<double, 3> p = target;
    anchor = snapToFace(model, anchor.faceId, p);
    mesh.vertices[vertIdx] = p;
}

namespace {

// Create the loop vertex on edge (a -> b) at fraction f from a, exactly on
// the CAD surface: UV interpolation when both ends share a face, otherwise
// a 3D lerp re-projected onto one end's face.
uint32_t makeSplitVertex(PolyMesh& mesh, const Model& model, uint32_t a,
                         uint32_t b, double f) {
    const Anchor& aa = mesh.anchors[a];
    const Anchor& ab = mesh.anchors[b];
    const auto& pa = mesh.vertices[a];
    const auto& pb = mesh.vertices[b];

    if (aa.faceId != 0 && aa.faceId == ab.faceId) {
        double u = aa.u + f * (ab.u - aa.u);
        double v = aa.v + f * (ab.v - aa.v);
        gp_Pnt p = surfaceOf(model, aa.faceId)->Value(u, v);
        mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
        mesh.anchors.push_back({aa.faceId, u, v});
    } else {
        std::array<double, 3> p{pa[0] + f * (pb[0] - pa[0]),
                                pa[1] + f * (pb[1] - pa[1]),
                                pa[2] + f * (pb[2] - pa[2])};
        int faceId = aa.faceId != 0 ? aa.faceId : ab.faceId;
        Anchor anchor;
        if (faceId != 0) anchor = snapToFace(model, faceId, p);
        mesh.vertices.push_back(p);
        mesh.anchors.push_back(anchor);
    }
    return static_cast<uint32_t>(mesh.vertices.size() - 1);
}

struct StripWalk {
    // Undirected edges split by the loop, with the ordered direction and
    // fraction the split was measured in.
    std::map<EdgeKey, uint32_t> splits;
    // Quads the loop passes through: polygon index -> (entry, exit) keys.
    std::map<size_t, std::pair<EdgeKey, EdgeKey>> crossed;
};

// Walk the quad strip starting at polygon `poly`, entering through the
// ordered edge (a,b) whose split sits at fraction f from a. Splits every
// crossed edge and records the strip until it exits the mesh or returns to
// an already-split edge (closed loop).
void walkStrip(PolyMesh& mesh, const Model& model, StripWalk& walk,
               size_t poly, uint32_t a, uint32_t b, double f,
               const std::map<EdgeKey, std::vector<size_t>>& edgePolys) {
    size_t guard = mesh.polygons.size() + 2;
    while (guard-- > 0) {
        const std::vector<uint32_t>& quad = mesh.polygons[poly];
        if (quad.size() != 4) return;  // strip ends at a non-quad
        if (walk.crossed.count(poly)) return;

        // Locate the entry edge in this quad's winding.
        int k = -1;
        for (int i = 0; i < 4; ++i) {
            if (quad[i] == a && quad[(i + 1) % 4] == b) { k = i; break; }
        }
        if (k < 0) {  // entry appears reversed (mesh with flipped neighbour)
            for (int i = 0; i < 4; ++i) {
                if (quad[i] == b && quad[(i + 1) % 4] == a) {
                    k = i;
                    std::swap(a, b);
                    f = 1.0 - f;
                    break;
                }
            }
        }
        if (k < 0) return;

        uint32_t c = quad[(k + 2) % 4];
        uint32_t d = quad[(k + 3) % 4];

        if (!walk.splits.count(keyOf(a, b))) {
            walk.splits[keyOf(a, b)] = makeSplitVertex(mesh, model, a, b, f);
        }
        // The exit edge (c,d) pairs geometrically as b<->c, a<->d, so the
        // parallel loop crosses it at fraction (1-f) from c.
        bool exitKnown = walk.splits.count(keyOf(c, d)) != 0;
        if (!exitKnown) {
            walk.splits[keyOf(c, d)] =
                makeSplitVertex(mesh, model, c, d, 1.0 - f);
        }
        walk.crossed[poly] = {keyOf(a, b), keyOf(c, d)};
        if (exitKnown) return;  // closed back onto the loop

        // Continue into the neighbour across (c,d).
        auto it = edgePolys.find(keyOf(c, d));
        size_t next = poly;
        if (it != edgePolys.end()) {
            for (size_t p : it->second) {
                if (p != poly) { next = p; break; }
            }
        }
        if (next == poly) return;  // open boundary
        poly = next;
        // Entering the neighbour, the shared edge runs (d,c) in its winding;
        // the split at (1-f) from c sits at fraction f from d.
        a = d;
        b = c;
    }
}

}  // namespace

int insertLoop(PolyMesh& mesh, const Model& model, const ManualOp& op) {
    gp_Pnt target = surfaceOf(model, op.faceId)->Value(op.u, op.v);

    std::map<EdgeKey, std::vector<size_t>> edgePolys;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            edgePolys[keyOf(poly[i], poly[(i + 1) % poly.size()])].push_back(p);
        }
    }

    // The edge the loop will cross: nearest edge midpoint among quads
    // generated from the anchored B-rep face.
    size_t bestPoly = SIZE_MAX;
    uint32_t bestA = 0, bestB = 0;
    double bestDist = 1e300;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] != op.faceId) continue;
        const auto& poly = mesh.polygons[p];
        if (poly.size() != 4) continue;
        for (size_t i = 0; i < 4; ++i) {
            uint32_t a = poly[i];
            uint32_t b = poly[(i + 1) % 4];
            const auto& pa = mesh.vertices[a];
            const auto& pb = mesh.vertices[b];
            gp_Pnt mid(0.5 * (pa[0] + pb[0]), 0.5 * (pa[1] + pb[1]),
                       0.5 * (pa[2] + pb[2]));
            double dist = mid.Distance(target);
            if (dist < bestDist) {
                bestDist = dist;
                bestPoly = p;
                bestA = a;
                bestB = b;
            }
        }
    }
    if (bestPoly == SIZE_MAX) return 0;

    StripWalk walk;
    walkStrip(mesh, model, walk, bestPoly, bestA, bestB, op.t, edgePolys);
    // If the strip didn't close, extend it backwards through the other
    // polygon sharing the seed edge.
    if (!walk.splits.empty()) {
        for (size_t p : edgePolys[keyOf(bestA, bestB)]) {
            if (p != bestPoly && !walk.crossed.count(p)) {
                walkStrip(mesh, model, walk, p, bestB, bestA, 1.0 - op.t,
                          edgePolys);
            }
        }
    }
    if (walk.crossed.empty()) return 0;

    // Rebuild polygons: insert split vertices into every ring that carries a
    // split edge (keeps terminal n-gons watertight), then cut the crossed
    // quads in two along their split pair.
    std::vector<std::vector<uint32_t>> polys;
    std::vector<int> polyFace;
    polys.reserve(mesh.polygons.size() + walk.crossed.size());
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        std::vector<uint32_t> ring;
        ring.reserve(poly.size() + 2);
        for (size_t i = 0; i < poly.size(); ++i) {
            ring.push_back(poly[i]);
            auto it = walk.splits.find(keyOf(poly[i], poly[(i + 1) % poly.size()]));
            if (it != walk.splits.end()) ring.push_back(it->second);
        }

        auto crossedIt = walk.crossed.find(p);
        if (crossedIt == walk.crossed.end()) {
            polys.push_back(std::move(ring));
            polyFace.push_back(mesh.polygonFaceId[p]);
            continue;
        }
        uint32_t m1 = walk.splits[crossedIt->second.first];
        uint32_t m2 = walk.splits[crossedIt->second.second];
        size_t i1 = SIZE_MAX, i2 = SIZE_MAX;
        for (size_t i = 0; i < ring.size(); ++i) {
            if (ring[i] == m1) i1 = i;
            if (ring[i] == m2) i2 = i;
        }
        std::vector<uint32_t> half1, half2;
        for (size_t i = i1; ; i = (i + 1) % ring.size()) {
            half1.push_back(ring[i]);
            if (i == i2) break;
        }
        for (size_t i = i2; ; i = (i + 1) % ring.size()) {
            half2.push_back(ring[i]);
            if (i == i1) break;
        }
        polys.push_back(std::move(half1));
        polyFace.push_back(mesh.polygonFaceId[p]);
        polys.push_back(std::move(half2));
        polyFace.push_back(mesh.polygonFaceId[p]);
    }
    mesh.polygons = std::move(polys);
    mesh.polygonFaceId = std::move(polyFace);
    return static_cast<int>(walk.crossed.size());
}

std::vector<std::vector<uint32_t>> boundaryLoops(const PolyMesh& mesh) {
    // Directed edges used exactly once with no reverse partner are open
    // boundary; chain them into loops.
    std::map<EdgeKey, int> directed;  // (a,b) key with a<b: +1 fwd, -1 rev
    std::map<uint32_t, uint32_t> next;
    std::map<std::pair<uint32_t, uint32_t>, bool> seen;
    for (const auto& poly : mesh.polygons) {
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
            seen[{a, b}] = true;
        }
    }
    for (const auto& [e, _] : seen) {
        if (!seen.count({e.second, e.first})) {
            // Boundary rings must run OPPOSITE the existing polygon edge so
            // a closing strip cancels it; store the reversed direction.
            next[e.second] = e.first;
        }
    }
    std::vector<std::vector<uint32_t>> loops;
    std::map<uint32_t, bool> used;
    for (const auto& [start, _] : next) {
        if (used[start]) continue;
        std::vector<uint32_t> loop;
        uint32_t v = start;
        while (!used[v]) {
            used[v] = true;
            loop.push_back(v);
            auto it = next.find(v);
            if (it == next.end()) break;
            v = it->second;
        }
        if (loop.size() >= 3 && v == start) loops.push_back(std::move(loop));
    }
    return loops;
}

namespace {

// Mean distance from a vertex loop to a sampled B-rep edge curve — used to
// find which open boundary hugs which CAD edge.
double loopToEdgeDistance(const PolyMesh& mesh,
                          const std::vector<uint32_t>& loop,
                          const std::vector<gp_Pnt>& samples) {
    double sum = 0;
    for (uint32_t v : loop) {
        gp_Pnt p(mesh.vertices[v][0], mesh.vertices[v][1],
                 mesh.vertices[v][2]);
        double best = 1e300;
        for (const gp_Pnt& s : samples) best = std::min(best, p.Distance(s));
        sum += best;
    }
    return sum / double(loop.size());
}

std::vector<gp_Pnt> sampleEdgeCurve(const Model& model, int edgeId, int n) {
    std::vector<gp_Pnt> pts;
    if (edgeId < 1 || edgeId > model.edgeCount()) return pts;
    BRepAdaptor_Curve curve(TopoDS::Edge(model.edges(edgeId)));
    double f = curve.FirstParameter(), l = curve.LastParameter();
    for (int i = 0; i <= n; ++i) {
        pts.push_back(curve.Value(f + (l - f) * i / double(n)));
    }
    return pts;
}

double vdist(const PolyMesh& m, uint32_t a, uint32_t b) {
    const auto& p = m.vertices[a];
    const auto& q = m.vertices[b];
    return std::sqrt((p[0] - q[0]) * (p[0] - q[0]) +
                     (p[1] - q[1]) * (p[1] - q[1]) +
                     (p[2] - q[2]) * (p[2] - q[2]));
}

}  // namespace

int bridgeLoops(PolyMesh& mesh, const Model& model, const ManualOp& op) {
    std::vector<std::vector<uint32_t>> loops = boundaryLoops(mesh);
    if (loops.size() < 2) return 0;

    auto nearestLoop = [&](int edgeId, int excludeIdx) -> int {
        std::vector<gp_Pnt> samples = sampleEdgeCurve(model, edgeId, 32);
        if (samples.empty()) return -1;
        int best = -1;
        double bestDist = 1e300;
        for (int i = 0; i < int(loops.size()); ++i) {
            if (i == excludeIdx) continue;
            double d = loopToEdgeDistance(mesh, loops[i], samples);
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
        return best;
    };
    int ia = nearestLoop(op.edgeA, -1);
    int ib = nearestLoop(op.edgeB, ia);
    if (ia < 0 || ib < 0 || ia == ib) return 0;

    // boundaryLoops stores each loop REVERSED relative to its polygons'
    // windings, so a bridge polygon must traverse loop edges in loop order
    // to cancel the open edge. The two rims counter-rotate geometrically,
    // which pairs A's forward walk with a DECREASING index walk on B.
    const std::vector<uint32_t>& A = loops[ia];
    const std::vector<uint32_t>& B = loops[ib];
    const int n = int(A.size()), m = int(B.size());
    auto wrapB = [&](int k) { return ((k % m) + m) % m; };

    // Rotational alignment: pair A[i] with B[off - i]; pick the offset
    // minimizing total rail length.
    int bestOff = 0;
    double bestSum = 1e300;
    for (int off = 0; off < m; ++off) {
        double sum = 0;
        for (int i = 0; i < n; ++i) {
            sum += vdist(mesh, A[i], B[wrapB(off - i)]);
        }
        if (sum < bestSum) {
            bestSum = sum;
            bestOff = off;
        }
    }

    int added = 0;
    auto emit = [&](std::vector<uint32_t> poly) {
        mesh.polygons.push_back(std::move(poly));
        mesh.polygonFaceId.push_back(0);  // bridge strip: no source face
        ++added;
    };

    if (n == m) {
        // Equal counts: one clean quad ring.
        for (int i = 0; i < n; ++i) {
            int k = wrapB(bestOff - i);
            emit({A[i], A[(i + 1) % n], B[k], B[(k + 1) % m]});
        }
        return added;
    }

    // Unequal counts: greedy triangle zipper. i counts consumed A edges
    // (walking forward), t consumed B edges (index walking backward from
    // bestOff); advance whichever makes the shorter bridging diagonal.
    int i = 0, t = 0;
    while (i < n || t < m) {
        int ai = i % n;            // current A vertex
        int bp = wrapB(bestOff - t);  // current B vertex
        bool stepA;
        if (i >= n) stepA = false;
        else if (t >= m) stepA = true;
        else {
            stepA = vdist(mesh, A[(ai + 1) % n], B[bp]) <=
                    vdist(mesh, A[ai], B[wrapB(bp - 1)]);
        }
        if (stepA) {
            emit({A[ai], A[(ai + 1) % n], B[bp]});
            ++i;
        } else {
            emit({A[ai], B[wrapB(bp - 1)], B[bp]});
            ++t;
        }
    }
    return added;
}

void applyOps(PolyMesh& mesh, const Model& model,
              const std::vector<ManualOp>& ops) {
    for (const ManualOp& op : ops) {
        switch (op.kind) {
            case ManualOp::Kind::LoopInsert: insertLoop(mesh, model, op); break;
            case ManualOp::Kind::Bridge: bridgeLoops(mesh, model, op); break;
        }
    }
}

}  // namespace weft
