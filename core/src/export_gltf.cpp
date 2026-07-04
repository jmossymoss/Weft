#include "weft/export_gltf.hpp"
#include "weft/model.hpp"
#include "normals.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace weft {

namespace {

// A glTF vertex: one per (mesh vertex, B-rep face) pair, so per-face
// exact normals split naturally at sharp edges.
struct SplitVertex {
    float px, py, pz;
    float nx, ny, nz;
    float faceId;
};

void append(std::vector<uint8_t>& buf, const void* data, size_t n) {
    const uint8_t* b = static_cast<const uint8_t*>(data);
    buf.insert(buf.end(), b, b + n);
}

void pad4(std::vector<uint8_t>& buf, uint8_t filler) {
    while (buf.size() % 4) buf.push_back(filler);
}

}  // namespace

void writeGlb(const PolyMesh& mesh, const std::string& path,
              const Model* model) {
    // One glTF node+mesh per part, so assemblies import as bodies.
    const bool hasParts = mesh.polygonPartId.size() == mesh.polygons.size();
    std::map<int, std::vector<size_t>> polysOfPart;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        polysOfPart[hasParts ? mesh.polygonPartId[p] : 1].push_back(p);
    }

    struct Part {
        int id;
        std::vector<SplitVertex> verts;
        std::vector<uint32_t> indices;
    };
    std::vector<Part> parts;
    std::map<int, BRepAdaptor_Surface> cache;

    for (const auto& [partId, polyIdxs] : polysOfPart) {
    // Split vertices per (vertex, face) and fan-triangulate the polygons.
    std::map<std::pair<uint32_t, int>, uint32_t> splitOf;
    std::vector<SplitVertex> verts;
    std::vector<uint32_t> indices;

    auto splitVertex = [&](uint32_t idx, int fid) {
        auto key = std::make_pair(idx, fid);
        auto it = splitOf.find(key);
        if (it != splitOf.end()) return it->second;
        SplitVertex v{};
        v.px = static_cast<float>(mesh.vertices[idx][0]);
        v.py = static_cast<float>(mesh.vertices[idx][1]);
        v.pz = static_cast<float>(mesh.vertices[idx][2]);
        std::array<double, 3> n{0.0, 0.0, 1.0};
        if (!model || !detail::cadNormal(mesh, *model, idx, fid, cache, n)) {
            n = {0.0, 0.0, 0.0};  // filled from polygon fan below
        }
        v.nx = static_cast<float>(n[0]);
        v.ny = static_cast<float>(n[1]);
        v.nz = static_cast<float>(n[2]);
        v.faceId = static_cast<float>(fid);
        uint32_t slot = static_cast<uint32_t>(verts.size());
        verts.push_back(v);
        splitOf.emplace(key, slot);
        return slot;
    };

    for (size_t p : polyIdxs) {
        const auto& poly = mesh.polygons[p];
        const int fid = mesh.polygonFaceId[p];
        if (poly.size() < 3) continue;
        std::vector<uint32_t> ring(poly.size());
        for (size_t i = 0; i < poly.size(); ++i) {
            ring[i] = splitVertex(poly[i], fid);
        }
        for (size_t i = 1; i + 1 < ring.size(); ++i) {
            indices.push_back(ring[0]);
            indices.push_back(ring[i]);
            indices.push_back(ring[i + 1]);
        }
        // Corners whose exact normal failed (poles/apexes) take the flat
        // polygon normal so the export never carries zero normals.
        double ax = mesh.vertices[poly[1]][0] - mesh.vertices[poly[0]][0];
        double ay = mesh.vertices[poly[1]][1] - mesh.vertices[poly[0]][1];
        double az = mesh.vertices[poly[1]][2] - mesh.vertices[poly[0]][2];
        double bx = mesh.vertices[poly[2]][0] - mesh.vertices[poly[0]][0];
        double by = mesh.vertices[poly[2]][1] - mesh.vertices[poly[0]][1];
        double bz = mesh.vertices[poly[2]][2] - mesh.vertices[poly[0]][2];
        double cx = ay * bz - az * by, cy = az * bx - ax * bz,
               cz = ax * by - ay * bx;
        double len = std::sqrt(cx * cx + cy * cy + cz * cz);
        if (len > 1e-30) {
            for (uint32_t s : ring) {
                SplitVertex& v = verts[s];
                if (v.nx == 0.0f && v.ny == 0.0f && v.nz == 0.0f) {
                    v.nx = static_cast<float>(cx / len);
                    v.ny = static_cast<float>(cy / len);
                    v.nz = static_cast<float>(cz / len);
                }
            }
        }
    }
    if (!verts.empty()) {
        parts.push_back({partId, std::move(verts), std::move(indices)});
    }
    }  // per part
    if (parts.empty()) throw std::runtime_error("writeGlb: empty mesh");

    // Binary chunk: per part, positions / normals / face ids / indices,
    // each block 4-aligned. One node + mesh + primitive per part.
    std::vector<uint8_t> bin;
    std::string views, accessors, meshes, nodes, roots;
    char buf[512];
    int acc = 0, view = 0;
    for (size_t pi = 0; pi < parts.size(); ++pi) {
        const Part& part = parts[pi];
        float pmin[3] = {std::numeric_limits<float>::max(),
                         std::numeric_limits<float>::max(),
                         std::numeric_limits<float>::max()};
        float pmax[3] = {-std::numeric_limits<float>::max(),
                         -std::numeric_limits<float>::max(),
                         -std::numeric_limits<float>::max()};
        const size_t posOff = bin.size();
        for (const SplitVertex& v : part.verts) {
            float p3[3] = {v.px, v.py, v.pz};
            for (int i = 0; i < 3; ++i) {
                pmin[i] = std::min(pmin[i], p3[i]);
                pmax[i] = std::max(pmax[i], p3[i]);
            }
            append(bin, p3, sizeof p3);
        }
        const size_t normOff = bin.size();
        for (const SplitVertex& v : part.verts) {
            float n3[3] = {v.nx, v.ny, v.nz};
            append(bin, n3, sizeof n3);
        }
        const size_t fidOff = bin.size();
        for (const SplitVertex& v : part.verts) {
            append(bin, &v.faceId, sizeof v.faceId);
        }
        const size_t idxOff = bin.size();
        append(bin, part.indices.data(), part.indices.size() * sizeof(uint32_t));
        pad4(bin, 0);

        auto addView = [&](size_t off, size_t len, int target) {
            std::snprintf(buf, sizeof buf,
                          "%s{\"buffer\":0,\"byteOffset\":%zu,"
                          "\"byteLength\":%zu,\"target\":%d}",
                          view ? "," : "", off, len, target);
            views += buf;
            return view++;
        };
        const int vPos = addView(posOff, normOff - posOff, 34962);
        const int vNorm = addView(normOff, fidOff - normOff, 34962);
        const int vFid = addView(fidOff, idxOff - fidOff, 34962);
        const int vIdx =
            addView(idxOff, part.indices.size() * sizeof(uint32_t), 34963);

        std::snprintf(buf, sizeof buf,
                      "%s{\"bufferView\":%d,\"componentType\":5126,"
                      "\"count\":%zu,\"type\":\"VEC3\","
                      "\"min\":[%.9g,%.9g,%.9g],\"max\":[%.9g,%.9g,%.9g]}",
                      acc ? "," : "", vPos, part.verts.size(), pmin[0], pmin[1],
                      pmin[2], pmax[0], pmax[1], pmax[2]);
        accessors += buf;
        const int aPos = acc++;
        std::snprintf(buf, sizeof buf,
                      ",{\"bufferView\":%d,\"componentType\":5126,"
                      "\"count\":%zu,\"type\":\"VEC3\"}",
                      vNorm, part.verts.size());
        accessors += buf;
        const int aNorm = acc++;
        std::snprintf(buf, sizeof buf,
                      ",{\"bufferView\":%d,\"componentType\":5126,"
                      "\"count\":%zu,\"type\":\"SCALAR\"}",
                      vFid, part.verts.size());
        accessors += buf;
        const int aFid = acc++;
        std::snprintf(buf, sizeof buf,
                      ",{\"bufferView\":%d,\"componentType\":5125,"
                      "\"count\":%zu,\"type\":\"SCALAR\"}",
                      vIdx, part.indices.size());
        accessors += buf;
        const int aIdx = acc++;

        std::snprintf(buf, sizeof buf,
                      "%s{\"name\":\"part_%d\",\"primitives\":[{"
                      "\"attributes\":{\"POSITION\":%d,\"NORMAL\":%d,"
                      "\"_WEFT_FACE_ID\":%d},\"indices\":%d,\"mode\":4}]}",
                      pi ? "," : "", part.id, aPos, aNorm, aFid, aIdx);
        meshes += buf;
        std::snprintf(buf, sizeof buf,
                      "%s{\"mesh\":%zu,\"name\":\"part_%d\"}",
                      pi ? "," : "", pi, part.id);
        nodes += buf;
        std::snprintf(buf, sizeof buf, "%s%zu", pi ? "," : "", pi);
        roots += buf;
    }

    std::string json = "{\"asset\":{\"version\":\"2.0\","
                       "\"generator\":\"weft\"},\"scene\":0,"
                       "\"scenes\":[{\"nodes\":[" + roots + "]}],"
                       "\"nodes\":[" + nodes + "],"
                       "\"meshes\":[" + meshes + "],"
                       "\"buffers\":[{\"byteLength\":" +
                       std::to_string(bin.size()) + "}],"
                       "\"bufferViews\":[" + views + "],"
                       "\"accessors\":[" + accessors + "]}";

    std::vector<uint8_t> jsonChunk(json.begin(), json.end());
    pad4(jsonChunk, ' ');

    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) throw std::runtime_error("cannot open for writing: " + path);
    auto u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    u32(0x46546C67);  // "glTF"
    u32(2);
    u32(static_cast<uint32_t>(12 + 8 + jsonChunk.size() + 8 + bin.size()));
    u32(static_cast<uint32_t>(jsonChunk.size()));
    u32(0x4E4F534A);  // "JSON"
    std::fwrite(jsonChunk.data(), 1, jsonChunk.size(), f);
    u32(static_cast<uint32_t>(bin.size()));
    u32(0x004E4942);  // "BIN"
    std::fwrite(bin.data(), 1, bin.size(), f);
    std::fclose(f);
}

}  // namespace weft
