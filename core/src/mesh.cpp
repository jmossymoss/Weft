#include "weft/mesh.hpp"
#include "weft/model.hpp"
#include "normals.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Surface.hxx>
#include <BRep_Tool.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

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
    int64_t x, y, z, g;
    bool operator==(const CellKey& o) const {
        return x == o.x && y == o.y && z == o.z && g == o.g;
    }
};

struct CellKeyHash {
    size_t operator()(const CellKey& k) const {
        size_t h = std::hash<int64_t>()(k.x);
        h = h * 31 + std::hash<int64_t>()(k.y);
        h = h * 31 + std::hash<int64_t>()(k.z);
        h = h * 31 + std::hash<int64_t>()(k.g);
        return h;
    }
};

}  // namespace

void weldVertices(PolyMesh& mesh, double tolerance,
                  const std::vector<int>* groups) {
    if (tolerance <= 0 || mesh.vertices.empty()) return;

    std::unordered_map<CellKey, uint32_t, CellKeyHash> firstInCell;
    std::vector<uint32_t> remap(mesh.vertices.size());
    std::vector<std::array<double, 3>> kept;
    std::vector<Anchor> keptAnchors;
    kept.reserve(mesh.vertices.size());
    keptAnchors.reserve(mesh.vertices.size());

    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const auto& v = mesh.vertices[i];
        CellKey key{static_cast<int64_t>(std::llround(v[0] / tolerance)),
                    static_cast<int64_t>(std::llround(v[1] / tolerance)),
                    static_cast<int64_t>(std::llround(v[2] / tolerance)),
                    groups && i < groups->size() ? (*groups)[i] : 0};
        auto [it, inserted] =
            firstInCell.try_emplace(key, static_cast<uint32_t>(kept.size()));
        if (inserted) {
            kept.push_back(v);
            keptAnchors.push_back(i < mesh.anchors.size() ? mesh.anchors[i]
                                                          : Anchor{});
        }
        remap[i] = it->second;
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

using detail::cadNormal;

void writeObj(const PolyMesh& mesh, const std::string& path,
              const Model* model) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) throw std::runtime_error("cannot open for writing: " + path);

    std::fprintf(f, "# weft export\n");
    for (const auto& v : mesh.vertices) {
        std::fprintf(f, "v %.9g %.9g %.9g\n", v[0], v[1], v[2]);
    }

    // Exact CAD normals, one per used (vertex, face) pair. Corners whose
    // normal can't be evaluated (poles, projection misses) reuse index 0's
    // slot semantics by falling back to no-normal for the whole polygon.
    std::map<std::pair<uint32_t, int>, int> normalIndex;
    if (model) {
        std::map<int, BRepAdaptor_Surface> cache;
        std::vector<std::array<double, 3>> normals;
        std::vector<std::vector<int>> polyNormals(mesh.polygons.size());
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            const int fid = mesh.polygonFaceId[p];
            std::vector<int>& ni = polyNormals[p];
            ni.reserve(mesh.polygons[p].size());
            for (uint32_t idx : mesh.polygons[p]) {
                auto key = std::make_pair(idx, fid);
                auto it = normalIndex.find(key);
                if (it == normalIndex.end()) {
                    std::array<double, 3> n;
                    int slot = -1;
                    if (cadNormal(mesh, *model, idx, fid, cache, n)) {
                        normals.push_back(n);
                        slot = static_cast<int>(normals.size());  // 1-based
                    }
                    it = normalIndex.emplace(key, slot).first;
                }
                ni.push_back(it->second);
            }
        }
        for (const auto& n : normals) {
            std::fprintf(f, "vn %.6g %.6g %.6g\n", n[0], n[1], n[2]);
        }

        int currentGroup = -1;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            if (mesh.polygonFaceId[p] != currentGroup) {
                currentGroup = mesh.polygonFaceId[p];
                std::fprintf(f, "g face_%d\n", currentGroup);
            }
            bool full = true;
            for (int ni : polyNormals[p]) {
                if (ni < 1) full = false;
            }
            std::fprintf(f, "f");
            for (size_t c = 0; c < mesh.polygons[p].size(); ++c) {
                if (full) {
                    std::fprintf(f, " %u//%d", mesh.polygons[p][c] + 1,
                                 polyNormals[p][c]);
                } else {
                    std::fprintf(f, " %u", mesh.polygons[p][c] + 1);
                }
            }
            std::fprintf(f, "\n");
        }
        std::fclose(f);
        return;
    }

    // Group polygons by source B-rep face so CAD face IDs survive into the
    // DCC. Polygons of a face are contiguous by construction.
    int currentGroup = -1;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        if (mesh.polygonFaceId[p] != currentGroup) {
            currentGroup = mesh.polygonFaceId[p];
            std::fprintf(f, "g face_%d\n", currentGroup);
        }
        std::fprintf(f, "f");
        for (uint32_t idx : mesh.polygons[p]) {
            std::fprintf(f, " %u", idx + 1);
        }
        std::fprintf(f, "\n");
    }
    std::fclose(f);
}

}  // namespace weft
