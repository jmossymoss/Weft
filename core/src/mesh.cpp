#include "weft/mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <stdexcept>
#include <unordered_map>

namespace weft {

size_t PolyMesh::countQuads() const {
    return std::count_if(polygons.begin(), polygons.end(),
                         [](const auto& p) { return p.size() == 4; });
}

size_t PolyMesh::countTris() const {
    return std::count_if(polygons.begin(), polygons.end(),
                         [](const auto& p) { return p.size() == 3; });
}

size_t PolyMesh::countNgons() const {
    return std::count_if(polygons.begin(), polygons.end(),
                         [](const auto& p) { return p.size() > 4; });
}

namespace {

struct CellKey {
    int64_t x, y, z;
    bool operator==(const CellKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct CellKeyHash {
    size_t operator()(const CellKey& k) const {
        size_t h = std::hash<int64_t>()(k.x);
        h = h * 31 + std::hash<int64_t>()(k.y);
        h = h * 31 + std::hash<int64_t>()(k.z);
        return h;
    }
};

}  // namespace

void weldVertices(PolyMesh& mesh, double tolerance,
                  const std::vector<int>* group) {
    if (tolerance <= 0 || mesh.vertices.empty()) return;

    // Spatial hash with a true distance test over the 27 neighbouring
    // cells: two points inside tolerance can still straddle a cell
    // boundary, so a plain cell-identity weld leaves hairline seams.
    std::unordered_map<CellKey, std::vector<uint32_t>, CellKeyHash> cells;
    std::vector<uint32_t> remap(mesh.vertices.size());
    std::vector<std::array<double, 3>> kept;
    std::vector<Anchor> keptAnchors;
    std::vector<size_t> keptSource;  // kept index -> first source vertex
    kept.reserve(mesh.vertices.size());
    keptAnchors.reserve(mesh.vertices.size());
    keptSource.reserve(mesh.vertices.size());
    const double tol2 = tolerance * tolerance;

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const auto& v = mesh.vertices[i];
        const int64_t cx = int64_t(std::llround(v[0] / tolerance));
        const int64_t cy = int64_t(std::llround(v[1] / tolerance));
        const int64_t cz = int64_t(std::llround(v[2] / tolerance));
        uint32_t match = UINT32_MAX;
        for (int dz = -1; dz <= 1 && match == UINT32_MAX; ++dz) {
            for (int dy = -1; dy <= 1 && match == UINT32_MAX; ++dy) {
                for (int dx = -1; dx <= 1 && match == UINT32_MAX; ++dx) {
                    auto it = cells.find({cx + dx, cy + dy, cz + dz});
                    if (it == cells.end()) continue;
                    for (uint32_t k : it->second) {
                        if (group && (*group)[keptSource[k]] !=
                                         (*group)[i]) {
                            continue;  // different solids never fuse
                        }
                        const auto& q = kept[k];
                        double ddx = q[0] - v[0], ddy = q[1] - v[1],
                               ddz = q[2] - v[2];
                        if (ddx * ddx + ddy * ddy + ddz * ddz <= tol2) {
                            match = k;
                            break;
                        }
                    }
                }
            }
        }
        if (match == UINT32_MAX) {
            match = uint32_t(kept.size());
            kept.push_back(v);
            keptAnchors.push_back(i < mesh.anchors.size() ? mesh.anchors[i]
                                                          : Anchor{});
            keptSource.push_back(i);
            cells[{cx, cy, cz}].push_back(match);
        }
        remap[i] = match;
    }
    mesh.vertices = std::move(kept);
    mesh.anchors = std::move(keptAnchors);

    std::vector<std::vector<uint32_t>> polys;
    std::vector<int> polyFace;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        std::vector<uint32_t> mapped;
        mapped.reserve(mesh.polygons[p].size());
        for (uint32_t idx : mesh.polygons[p]) {
            uint32_t m = remap[idx];
            if (mapped.empty() || mapped.back() != m) mapped.push_back(m);
        }
        while (mapped.size() > 1 && mapped.front() == mapped.back()) {
            mapped.pop_back();
        }
        if (mapped.size() < 3) continue;  // collapsed by the weld
        polys.push_back(std::move(mapped));
        polyFace.push_back(mesh.polygonFaceId[p]);
    }
    mesh.polygons = std::move(polys);
    mesh.polygonFaceId = std::move(polyFace);
}

std::vector<std::array<uint32_t, 3>> triangulatePoly(
    const std::vector<std::array<double, 3>>& verts,
    const std::vector<uint32_t>& poly) {
    const size_t n = poly.size();
    std::vector<std::array<uint32_t, 3>> tris;
    if (n < 3) return tris;
    auto fan = [&] {
        for (size_t k = 1; k + 1 < n; ++k) {
            tris.push_back({0, uint32_t(k), uint32_t(k + 1)});
        }
        return tris;
    };
    if (n == 3) return fan();

    // Project onto the polygon's dominant plane (Newell normal basis).
    double nx = 0, ny = 0, nz = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto& a = verts[poly[i]];
        const auto& b = verts[poly[(i + 1) % n]];
        nx += (a[1] - b[1]) * (a[2] + b[2]);
        ny += (a[2] - b[2]) * (a[0] + b[0]);
        nz += (a[0] - b[0]) * (a[1] + b[1]);
    }
    const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (nlen < 1e-30) return fan();  // degenerate: nothing better to do
    nx /= nlen;
    ny /= nlen;
    nz /= nlen;
    // u = normalize(a x n) for an axis a not parallel to n; v = n x u.
    const double ax = std::abs(nx) < 0.9 ? 1.0 : 0.0;
    const double ay = 1.0 - ax;
    double ux = ay * nz, uy = -ax * nz, uz = ax * ny - ay * nx;
    const double ulen = std::sqrt(ux * ux + uy * uy + uz * uz);
    ux /= ulen;
    uy /= ulen;
    uz /= ulen;
    const double vx = ny * uz - nz * uy;
    const double vy = nz * ux - nx * uz;
    const double vz = nx * uy - ny * ux;
    std::vector<std::array<double, 2>> p2(n);
    double span = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto& p = verts[poly[i]];
        p2[i] = {p[0] * ux + p[1] * uy + p[2] * uz,
                 p[0] * vx + p[1] * vy + p[2] * vz};
        span = std::max({span, std::abs(p2[i][0]), std::abs(p2[i][1])});
    }
    const double eps = 1e-12 * std::max(1.0, span * span);
    auto cross = [&](size_t o, size_t a, size_t b) {
        return (p2[a][0] - p2[o][0]) * (p2[b][1] - p2[o][1]) -
               (p2[a][1] - p2[o][1]) * (p2[b][0] - p2[o][0]);
    };
    // Convex fast path (also catches every plain quad).
    bool convex = true;
    for (size_t i = 0; i < n && convex; ++i) {
        convex = cross(i, (i + 1) % n, (i + 2) % n) > -eps;
    }
    if (convex) return fan();

    // Ear clipping with keyhole support: coincident duplicates (bridge
    // twins from hole merging) never block an ear, and the best-shaped
    // valid ear clips each round so slivers don't pile onto one vertex.
    auto d2 = [&](size_t a, size_t b) {
        double dx = p2[a][0] - p2[b][0], dy = p2[a][1] - p2[b][1];
        return dx * dx + dy * dy;
    };
    auto insideTri = [&](size_t a, size_t b, size_t c, size_t p) {
        return cross(a, b, p) > eps && cross(b, c, p) > eps &&
               cross(c, a, p) > eps;
    };
    std::vector<size_t> idx(n);
    for (size_t i = 0; i < n; ++i) idx[i] = i;
    size_t guard = 3 * n * n + 16;
    while (idx.size() > 3 && guard-- > 0) {
        size_t bestK = idx.size();
        double bestQ = -1.0;
        for (size_t k = 0; k < idx.size(); ++k) {
            size_t ip = idx[(k + idx.size() - 1) % idx.size()];
            size_t ic = idx[k];
            size_t in = idx[(k + 1) % idx.size()];
            double a2 = cross(ip, ic, in);
            if (a2 <= eps) continue;  // reflex or collinear
            double s = d2(ip, ic) + d2(ic, in) + d2(in, ip);
            double q = s > 1e-300 ? a2 / s : 0.0;
            if (q <= bestQ) continue;
            bool blocked = false;
            for (size_t other : idx) {
                if (other == ip || other == ic || other == in) continue;
                if (d2(other, ip) < eps || d2(other, ic) < eps ||
                    d2(other, in) < eps) {
                    continue;  // bridge twin
                }
                if (insideTri(ip, ic, in, other)) {
                    blocked = true;
                    break;
                }
            }
            if (blocked) continue;
            bestK = k;
            bestQ = q;
        }
        if (bestK >= idx.size()) {
            // Numerical dead end: fan the remainder to stay connected.
            for (size_t k = 1; k + 1 < idx.size(); ++k) {
                tris.push_back({uint32_t(idx[0]), uint32_t(idx[k]),
                                uint32_t(idx[k + 1])});
            }
            return tris;
        }
        size_t ip = idx[(bestK + idx.size() - 1) % idx.size()];
        size_t ic = idx[bestK];
        size_t in = idx[(bestK + 1) % idx.size()];
        tris.push_back({uint32_t(ip), uint32_t(ic), uint32_t(in)});
        idx.erase(idx.begin() + bestK);
    }
    if (idx.size() == 3) {
        tris.push_back(
            {uint32_t(idx[0]), uint32_t(idx[1]), uint32_t(idx[2])});
    }
    return tris;
}

void writeObj(const PolyMesh& mesh, const std::string& path,
              const std::vector<std::vector<int>>* solidFaces,
              const ObjExportOptions* options) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error("cannot open for writing: " + path);

    const ObjExportOptions opts = options ? *options : ObjExportOptions{};
    std::fprintf(f, "# weft export\n");
    for (const auto& v : mesh.vertices) {
        double x = v[0] * opts.scale;
        double y = v[1] * opts.scale;
        double z = v[2] * opts.scale;
        if (opts.yUp) {
            // Z-up CAD -> Y-up engine: X stays, old Z becomes Y, old Y
            // flips into -Z (right-handed both sides).
            double ny = z, nz = -y;
            y = ny;
            z = nz;
        }
        std::fprintf(f, "v %.9g %.9g %.9g\n", x, y, z);
    }

    // Object structure: FaceId -> solid index, so each CAD body writes as
    // its own "o" block and importers keep bodies as separate meshes.
    std::map<int, int> faceSolid;
    if (solidFaces) {
        for (size_t si = 0; si < solidFaces->size(); ++si) {
            for (int fid : (*solidFaces)[si]) faceSolid[fid] = int(si);
        }
    }

    // Group polygons by source B-rep face so CAD face IDs survive into the
    // DCC. Polygons of a face are contiguous by construction; emitting in
    // solid order keeps each object's polygons contiguous too.
    int currentObject = -1, currentGroup = -1;
    auto writeFacePolys = [&](size_t p) {
        if (mesh.polygonFaceId[p] != currentGroup) {
            currentGroup = mesh.polygonFaceId[p];
            auto so = faceSolid.find(currentGroup);
            int object = so == faceSolid.end() ? 0 : so->second;
            if (solidFaces && object != currentObject) {
                currentObject = object;
                std::fprintf(f, "o object_%d\n", currentObject + 1);
            }
            std::fprintf(f, "g face_%d\n", currentGroup);
        }
        const auto& poly = mesh.polygons[p];
        if (opts.triangulate && poly.size() > 3) {
            for (const auto& t : triangulatePoly(mesh.vertices, poly)) {
                std::fprintf(f, "f %u %u %u\n", poly[t[0]] + 1,
                             poly[t[1]] + 1, poly[t[2]] + 1);
            }
            return;
        }
        std::fprintf(f, "f");
        for (uint32_t idx : poly) {
            std::fprintf(f, " %u", idx + 1);
        }
        std::fprintf(f, "\n");
    };
    if (solidFaces && !faceSolid.empty()) {
        // Emit polygons ordered by (solid, face): stable per-object blocks.
        std::vector<size_t> order(mesh.polygons.size());
        for (size_t p = 0; p < order.size(); ++p) order[p] = p;
        std::stable_sort(order.begin(), order.end(),
                         [&](size_t a, size_t b) {
                             auto sa = faceSolid.find(mesh.polygonFaceId[a]);
                             auto sb = faceSolid.find(mesh.polygonFaceId[b]);
                             int ia = sa == faceSolid.end() ? 0 : sa->second;
                             int ib = sb == faceSolid.end() ? 0 : sb->second;
                             return ia < ib;
                         });
        for (size_t p : order) writeFacePolys(p);
    } else {
        for (size_t p = 0; p < mesh.polygons.size(); ++p) writeFacePolys(p);
    }
    std::fclose(f);
}

}  // namespace weft
