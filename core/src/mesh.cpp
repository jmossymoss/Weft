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

void weldVertices(PolyMesh& mesh, double tolerance) {
    if (tolerance <= 0 || mesh.vertices.empty()) return;

    // Spatial hash with a true distance test over the 27 neighbouring
    // cells: two points inside tolerance can still straddle a cell
    // boundary, so a plain cell-identity weld leaves hairline seams.
    std::unordered_map<CellKey, std::vector<uint32_t>, CellKeyHash> cells;
    std::vector<uint32_t> remap(mesh.vertices.size());
    std::vector<std::array<double, 3>> kept;
    std::vector<Anchor> keptAnchors;
    kept.reserve(mesh.vertices.size());
    keptAnchors.reserve(mesh.vertices.size());
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

void writeObj(const PolyMesh& mesh, const std::string& path,
              const std::vector<std::vector<int>>* solidFaces) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error("cannot open for writing: " + path);

    std::fprintf(f, "# weft export\n");
    for (const auto& v : mesh.vertices) {
        std::fprintf(f, "v %.9g %.9g %.9g\n", v[0], v[1], v[2]);
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
        std::fprintf(f, "f");
        for (uint32_t idx : mesh.polygons[p]) {
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
