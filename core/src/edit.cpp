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
#include <set>
#include <algorithm>

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

namespace {

// Both bridge picks landed on ONE boundary loop — a deleted band whose two
// rims connect (through a shared corner, or a strip with open ends). Split
// the loop into a rail hugging each picked curve, zipper the rails, and
// close the two left-over spans (the band's ends) as n-gon caps.
int bridgeWithinLoop(PolyMesh& mesh, const std::vector<uint32_t>& L,
                     const std::vector<gp_Pnt>& samplesA,
                     const std::vector<gp_Pnt>& samplesB) {
    const int n = int(L.size());
    auto distTo = [&](uint32_t v, const std::vector<gp_Pnt>& s) {
        gp_Pnt p(mesh.vertices[v][0], mesh.vertices[v][1],
                 mesh.vertices[v][2]);
        double best = 1e300;
        for (const gp_Pnt& q : s) best = std::min(best, p.Distance(q));
        return best;
    };
    std::vector<char> nearA(n);
    for (int i = 0; i < n; ++i) {
        nearA[i] = distTo(L[i], samplesA) < distTo(L[i], samplesB);
    }
    // Longest contiguous cyclic run on each side = the two rails; whatever
    // sits between them (corner arcs, noise) belongs to the end caps.
    auto longestRun = [&](char want, int& start, int& len) {
        start = -1;
        len = 0;
        for (int s = 0; s < n; ++s) {
            if (nearA[s] != want || nearA[(s + n - 1) % n] == want) continue;
            int l = 0;
            while (l < n && nearA[(s + l) % n] == want) ++l;
            if (l > len) {
                len = l;
                start = s;
            }
        }
        if (start < 0 && nearA[0] == want) {  // the whole loop is one side
            start = 0;
            len = n;
        }
    };
    int sa, la, sb, lb;
    longestRun(1, sa, la);
    longestRun(0, sb, lb);
    if (sa < 0 || sb < 0 || la < 2 || lb < 2 || la >= n || lb >= n) return 0;

    auto at = [&](int k) { return L[((k % n) + n) % n]; };
    std::vector<uint32_t> ra, rb;
    for (int k = 0; k < la; ++k) ra.push_back(at(sa + k));
    for (int k = 0; k < lb; ++k) rb.push_back(at(sb + k));
    const int N = int(ra.size()) - 1;  // rail edge counts
    const int M = int(rb.size()) - 1;

    int added = 0;
    auto emit = [&](std::vector<uint32_t> poly) {
        mesh.polygons.push_back(std::move(poly));
        mesh.polygonFaceId.push_back(0);
        ++added;
    };

    // Zipper: ra walks forward in loop order, rb walks BACKWARD from its
    // far end (the two rails counter-rotate along the band). Every rail
    // edge is traversed in loop order inside its polygon, cancelling the
    // open boundary edge it covers. Advance by ARC FRACTION so the rungs
    // stay evenly matched — distance-greedy runs away on offset rails
    // and fans the leftover around one vertex.
    auto vd = [&](uint32_t a, uint32_t b) { return vdist(mesh, a, b); };
    std::vector<double> accA(N + 1, 0.0), accB(M + 1, 0.0);
    for (int k = 0; k < N; ++k) accA[k + 1] = accA[k] + vd(ra[k], ra[k + 1]);
    for (int k = 0; k < M; ++k) {
        accB[k + 1] = accB[k] + vd(rb[M - k], rb[M - k - 1]);
    }
    const double totA = std::max(accA[N], 1e-12);
    const double totB = std::max(accB[M], 1e-12);
    int i = 0, t = 0;
    while (i < N || t < M) {
        uint32_t bq = rb[M - t];
        bool stepA;
        if (i >= N) stepA = false;
        else if (t >= M) stepA = true;
        else stepA = accA[i + 1] * totB <= accB[t + 1] * totA;
        if (stepA) {
            emit({ra[i], ra[i + 1], bq});
            ++i;
        } else {
            emit({ra[i], rb[M - t - 1], bq});
            ++t;
        }
    }

    // End caps: the loop spans between the rails, closed by the zipper's
    // first/last rails. Emitted in loop order, so every remaining open
    // edge cancels and the strip is watertight.
    auto cap = [&](int from, int to) {  // loop indices, inclusive walk
        std::vector<uint32_t> poly;
        for (int k = from; ; ++k) {
            poly.push_back(at(k));
            if (((k % n) + n) % n == ((to % n) + n) % n) break;
            if (int(poly.size()) > n) return;  // safety
        }
        if (poly.size() >= 3) emit(std::move(poly));
    };
    cap(sa + la - 1, sb);           // ra end -> gap -> rb start
    cap(sb + lb - 1, sa);           // rb end -> gap -> ra start
    return added;
}

}  // namespace

int bridgeLoops(PolyMesh& mesh, const Model& model, const ManualOp& op) {
    std::vector<std::vector<uint32_t>> loops = boundaryLoops(mesh);
    if (loops.empty()) return 0;

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
    int ib = nearestLoop(op.edgeB, -1);
    if (ia < 0 || ib < 0) return 0;
    if (ia == ib) {
        // One loop, two sides: split it between the picked edges.
        if (op.edgeA == op.edgeB) return 0;
        return bridgeWithinLoop(mesh, loops[ia],
                                sampleEdgeCurve(model, op.edgeA, 32),
                                sampleEdgeCurve(model, op.edgeB, 32));
    }

    // boundaryLoops stores each loop REVERSED relative to its polygons'
    // windings, so a bridge polygon must traverse loop edges in loop order
    // to cancel the open edge. The two rims counter-rotate geometrically,
    // which pairs A's forward walk with a DECREASING index walk on B.
    const std::vector<uint32_t>& A = loops[ia];
    const std::vector<uint32_t>& B = loops[ib];
    const int n = int(A.size()), m = int(B.size());
    auto wrapB = [&](int k) { return ((k % m) + m) % m; };

    // Rotational alignment: pair A[i] with B[off - i]; pick the offset
    // minimizing total rail length. The user twists apply on top of the
    // automatic pick — rotating A before the search would just be undone
    // by it. A and B twist in opposite directions so the two rims can be
    // counter-rotated against a shared spiral.
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
    bestOff = wrapB(bestOff + op.twist - op.twistA);

    int added = 0;
    auto emit = [&](std::vector<uint32_t> poly) {
        mesh.polygons.push_back(std::move(poly));
        mesh.polygonFaceId.push_back(0);  // bridge strip: no source face
        ++added;
    };

    if (n == m) {
        // Equal counts: a clean quad ring — subdivided into `spans` rows
        // across the strip (each column's rail lerped between its ends).
        const int rows = std::max(1, op.spans);
        std::vector<std::vector<uint32_t>> R(rows + 1,
                                             std::vector<uint32_t>(n));
        for (int i = 0; i < n; ++i) {
            uint32_t top = A[i];
            uint32_t bot = B[wrapB(bestOff - i + 1)];
            R[0][i] = top;
            R[rows][i] = bot;
            for (int j = 1; j < rows; ++j) {
                double t = double(j) / rows;
                std::array<double, 3> p;
                for (int c = 0; c < 3; ++c) {
                    p[c] = (1.0 - t) * mesh.vertices[top][c] +
                           t * mesh.vertices[bot][c];
                }
                R[j][i] = uint32_t(mesh.vertices.size());
                mesh.vertices.push_back(p);
                mesh.anchors.push_back({});
            }
        }
        for (int j = 0; j < rows; ++j) {
            for (int i = 0; i < n; ++i) {
                emit({R[j][i], R[j][(i + 1) % n], R[j + 1][(i + 1) % n],
                      R[j + 1][i]});
            }
        }
        return added;
    }

    // Unequal counts: triangle zipper advanced by ARC FRACTION — the rail
    // that's proportionally behind steps next, so the strips distribute
    // evenly whatever the shapes. (Distance-greedy walks run away when
    // the loops are laterally offset — an angled socket — consuming one
    // rail whole and fanning the rest around a single vertex.)
    std::vector<double> accA(n + 1, 0.0), accB(m + 1, 0.0);
    for (int k = 0; k < n; ++k) {
        accA[k + 1] = accA[k] + vdist(mesh, A[k], A[(k + 1) % n]);
    }
    for (int k = 0; k < m; ++k) {
        accB[k + 1] = accB[k] + vdist(mesh, B[wrapB(bestOff - k)],
                                      B[wrapB(bestOff - k - 1)]);
    }
    const double totA = std::max(accA[n], 1e-12);
    const double totB = std::max(accB[m], 1e-12);
    int i = 0, t = 0;
    while (i < n || t < m) {
        int ai = i % n;            // current A vertex
        int bp = wrapB(bestOff - t);  // current B vertex
        bool stepA;
        if (i >= n) stepA = false;
        else if (t >= m) stepA = true;
        else stepA = accA[i + 1] * totB <= accB[t + 1] * totA;
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

int fillLoop(PolyMesh& mesh, const Model& model, const ManualOp& op) {
    std::vector<std::vector<uint32_t>> loops = boundaryLoops(mesh);
    if (loops.empty()) return 0;
    std::vector<gp_Pnt> samples = sampleEdgeCurve(model, op.edgeA, 32);
    if (samples.empty()) return 0;
    int best = -1;
    double bestDist = 1e300;
    for (int i = 0; i < int(loops.size()); ++i) {
        double d = loopToEdgeDistance(mesh, loops[i], samples);
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    if (best < 0) return 0;
    // The loop in loop order traverses every open edge once in the
    // direction that cancels it — one watertight n-gon cap. The engine
    // (or a later minimal pass) triangulates it however it likes.
    mesh.polygons.push_back(loops[best]);
    mesh.polygonFaceId.push_back(0);
    return 1;
}

int deletePoly(PolyMesh& mesh, const ManualOp& op) {
    if (mesh.polygons.empty()) return 0;
    size_t best = 0;
    double bestD = 1e300;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        double cx = 0, cy = 0, cz = 0;
        for (uint32_t v : mesh.polygons[p]) {
            cx += mesh.vertices[v][0];
            cy += mesh.vertices[v][1];
            cz += mesh.vertices[v][2];
        }
        double k = double(mesh.polygons[p].size());
        double d = (cx / k - op.u) * (cx / k - op.u) +
                   (cy / k - op.v) * (cy / k - op.v) +
                   (cz / k - op.t) * (cz / k - op.t);
        if (d < bestD) {
            bestD = d;
            best = p;
        }
    }
    mesh.polygons.erase(mesh.polygons.begin() + best);
    mesh.polygonFaceId.erase(mesh.polygonFaceId.begin() + best);
    return 1;
}

int nudgeVertex(PolyMesh& mesh, const Model& model, const ManualOp& op) {
    if (op.faceId < 1 || op.faceId > model.faceCount()) return 0;
    // The source is found in anchor space, not 3D: it's stable under the
    // very nudges being replayed (an earlier op moving a vertex must not
    // steal a later op's target).
    size_t best = mesh.vertexCount();
    double bestD = 1e300;
    for (size_t v = 0; v < mesh.vertexCount(); ++v) {
        const Anchor& a = mesh.anchors[v];
        if (a.faceId != op.faceId) continue;
        double d = (a.u - op.u) * (a.u - op.u) + (a.v - op.v) * (a.v - op.v);
        if (d < bestD) {
            bestD = d;
            best = v;
        }
    }
    if (best == mesh.vertexCount()) return 0;
    Handle(Geom_Surface) surf = surfaceOf(model, op.faceId);
    gp_Pnt p = surf->Value(op.u2, op.v2);
    mesh.vertices[best] = {p.X(), p.Y(), p.Z()};
    mesh.anchors[best] = {op.faceId, op.u2, op.v2};
    return 1;
}

void applyOps(PolyMesh& mesh, const Model& model,
              const std::vector<ManualOp>& ops) {
    for (const ManualOp& op : ops) {
        switch (op.kind) {
            case ManualOp::Kind::LoopInsert: insertLoop(mesh, model, op); break;
            case ManualOp::Kind::DissolveLoop:
                dissolveLoop(mesh, model, op);
                break;
            case ManualOp::Kind::Bridge: bridgeLoops(mesh, model, op); break;
            case ManualOp::Kind::NudgeVertex:
                nudgeVertex(mesh, model, op);
                break;
            case ManualOp::Kind::FillLoop: fillLoop(mesh, model, op); break;
            case ManualOp::Kind::DeletePoly: deletePoly(mesh, op); break;
        }
    }
}

std::vector<std::pair<uint32_t, uint32_t>> walkEdgeLoop(const PolyMesh& mesh,
                                                        uint32_t a,
                                                        uint32_t b) {
    std::map<EdgeKey, std::vector<size_t>> edgePolys;
    std::map<uint32_t, std::vector<uint32_t>> nbrs;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const auto& poly = mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t u = poly[i], w = poly[(i + 1) % poly.size()];
            edgePolys[keyOf(u, w)].push_back(p);
            auto& nu = nbrs[u];
            if (std::find(nu.begin(), nu.end(), w) == nu.end()) {
                nu.push_back(w);
            }
            auto& nw = nbrs[w];
            if (std::find(nw.begin(), nw.end(), u) == nw.end()) {
                nw.push_back(u);
            }
        }
    }
    // Blender's rule: at each vertex the loop continues with the edge
    // sharing NEITHER polygon of the incoming edge — unique exactly at
    // interior 4-valence verts; anything else ends the loop.
    auto next = [&](uint32_t from, uint32_t via) -> uint32_t {
        std::set<uint32_t> banned;
        for (size_t p : edgePolys[keyOf(from, via)]) {
            for (uint32_t v : mesh.polygons[p]) banned.insert(v);
        }
        uint32_t out = UINT32_MAX;
        int count = 0;
        for (uint32_t nb : nbrs[via]) {
            if (nb == from || banned.count(nb)) continue;
            ++count;
            out = nb;
        }
        return count == 1 ? out : UINT32_MAX;
    };
    std::vector<std::pair<uint32_t, uint32_t>> loop = {{a, b}};
    std::set<EdgeKey> seen = {keyOf(a, b)};
    for (int dir = 0; dir < 2; ++dir) {
        uint32_t from = dir == 0 ? a : b, via = dir == 0 ? b : a;
        while (true) {
            uint32_t n = next(from, via);
            if (n == UINT32_MAX || seen.count(keyOf(via, n))) break;
            seen.insert(keyOf(via, n));
            loop.push_back({via, n});
            from = via;
            via = n;
        }
    }
    return loop;
}

int dissolveLoop(PolyMesh& mesh, const Model& model, const ManualOp& op) {
    (void)model;
    // Seed: the mesh edge whose midpoint is nearest the recorded point.
    const std::array<double, 3> target = {op.u, op.v, op.t};
    uint32_t sa = 0, sb = 0;
    double best = 1e300;
    for (const auto& poly : mesh.polygons) {
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t u = poly[i], w = poly[(i + 1) % poly.size()];
            const auto& A = mesh.vertices[u];
            const auto& B = mesh.vertices[w];
            double dx = 0.5 * (A[0] + B[0]) - target[0];
            double dy = 0.5 * (A[1] + B[1]) - target[1];
            double dz = 0.5 * (A[2] + B[2]) - target[2];
            double d = dx * dx + dy * dy + dz * dz;
            if (d < best) {
                best = d;
                sa = u;
                sb = w;
            }
        }
    }
    if (sa == sb) return 0;
    auto loop = walkEdgeLoop(mesh, sa, sb);

    std::set<EdgeKey> loopEdges;
    std::map<uint32_t, int> vertCut;  // loop vert -> dissolved edge count
    for (const auto& [u, w] : loop) {
        loopEdges.insert(keyOf(u, w));
        ++vertCut[u];
        ++vertCut[w];
    }

    // Merge the two polygons across each loop edge. Directed-edge lookup
    // is rebuilt lazily since each merge invalidates it for its ring.
    std::vector<std::vector<uint32_t>> polys(mesh.polygons.begin(),
                                             mesh.polygons.end());
    std::vector<int> polyFace(mesh.polygonFaceId.begin(),
                              mesh.polygonFaceId.end());
    std::vector<bool> dead(polys.size(), false);
    int dissolved = 0;
    for (const auto& [ea, eb] : loop) {
        // Find the polygon traversing ea->eb and the one traversing
        // eb->ea (live rings only).
        auto findDirected = [&](uint32_t u, uint32_t w) -> long {
            for (size_t p = 0; p < polys.size(); ++p) {
                if (dead[p]) continue;
                const auto& poly = polys[p];
                for (size_t i = 0; i < poly.size(); ++i) {
                    if (poly[i] == u &&
                        poly[(i + 1) % poly.size()] == w) {
                        return long(p);
                    }
                }
            }
            return -1;
        };
        long p1 = findDirected(ea, eb), p2 = findDirected(eb, ea);
        if (p1 < 0 || p2 < 0 || p1 == p2) continue;
        const auto& r1 = polys[p1];
        const auto& r2 = polys[p2];
        size_t i1 = 0, i2 = 0;
        for (size_t i = 0; i < r1.size(); ++i) {
            if (r1[i] == ea && r1[(i + 1) % r1.size()] == eb) i1 = i;
        }
        for (size_t i = 0; i < r2.size(); ++i) {
            if (r2[i] == eb && r2[(i + 1) % r2.size()] == ea) i2 = i;
        }
        // merged = r1 starting after the edge (eb .. ea) + r2 starting
        // after the reversed edge (ea .. eb), dropping the duplicated
        // endpoints — the shared edge vanishes.
        std::vector<uint32_t> merged;
        merged.reserve(r1.size() + r2.size() - 2);
        for (size_t k = 1; k < r1.size(); ++k) {
            merged.push_back(r1[(i1 + k) % r1.size()]);
        }
        for (size_t k = 1; k < r2.size(); ++k) {
            merged.push_back(r2[(i2 + k) % r2.size()]);
        }
        polys[p1] = std::move(merged);
        dead[p2] = true;
        ++dissolved;
    }
    if (!dissolved) return 0;

    // Drop fully-dissolved loop verts (both their loop edges gone) from
    // every ring: they're collinear rail points now, Blender-style.
    std::set<uint32_t> drop;
    for (const auto& [v, n] : vertCut) {
        if (n >= 2) drop.insert(v);
    }
    std::vector<std::vector<uint32_t>> outPolys;
    std::vector<int> outFace;
    outPolys.reserve(polys.size());
    for (size_t p = 0; p < polys.size(); ++p) {
        if (dead[p]) continue;
        std::vector<uint32_t> ring;
        ring.reserve(polys[p].size());
        for (uint32_t v : polys[p]) {
            if (drop.count(v)) continue;
            if (ring.empty() || ring.back() != v) ring.push_back(v);
        }
        while (ring.size() > 1 && ring.front() == ring.back()) {
            ring.pop_back();
        }
        if (ring.size() < 3) continue;
        outPolys.push_back(std::move(ring));
        outFace.push_back(polyFace[p]);
    }
    mesh.polygons = std::move(outPolys);
    mesh.polygonFaceId = std::move(outFace);
    return dissolved;
}

}  // namespace weft
