#include "weft/export_gltf.hpp"
#include "weft/model.hpp"
#include "normals.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
              const Model* model,
              const std::vector<std::vector<int>>* solidFaces,
              const GltfExportOptions* options) {
    const GltfExportOptions opts = options ? *options : GltfExportOptions{};
    // One glTF node+mesh per body, mapped from the analysis' per-solid
    // face lists (faces outside any solid group under part 0).
    std::map<int, int> partOfFace;
    if (solidFaces) {
        for (size_t si = 0; si < solidFaces->size(); ++si) {
            for (int fid : (*solidFaces)[si]) {
                partOfFace.emplace(fid, (int)si + 1);
            }
        }
    }
    std::map<int, std::vector<size_t>> polysOfPart;
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        auto it = partOfFace.find(mesh.polygonFaceId[p]);
        polysOfPart[it == partOfFace.end() ? 0 : it->second].push_back(p);
    }

    // Distinct source colors become glTF materials; each face resolves to a
    // material (own face color, else its solid's color) or -1 (none). Built
    // lazily so only colors that actually appear in the mesh are emitted, and
    // so an uncolored model produces no materials[] at all (bytes unchanged).
    std::vector<std::array<float, 3>> materials;
    std::map<std::array<int, 3>, int> matIndexOf;
    auto colorToMat = [&](const std::array<float, 3>& c) {
        auto q8 = [](float x) {
            int v = (int)std::lround(x * 255.0f);
            return v < 0 ? 0 : (v > 255 ? 255 : v);
        };
        std::array<int, 3> key{q8(c[0]), q8(c[1]), q8(c[2])};
        auto it = matIndexOf.find(key);
        if (it != matIndexOf.end()) return it->second;
        int idx = (int)materials.size();
        materials.push_back(c);
        matIndexOf.emplace(key, idx);
        return idx;
    };
    auto matForFace = [&](int fid) -> int {
        if (!opts.embedColors || !model) return -1;
        int fi = fid - 1;
        if (fi >= 0 && fi < (int)model->faceHasColor.size() && model->faceHasColor[fi])
            return colorToMat(model->faceColors[fi]);
        auto it = partOfFace.find(fid);  // fall back to the owning solid's color
        if (it != partOfFace.end()) {
            int s = it->second - 1;
            if (s >= 0 && s < (int)model->solidHasColor.size() && model->solidHasColor[s])
                return colorToMat(model->solidColors[s]);
        }
        return -1;
    };

    // One primitive per (part, material): a part's mesh holds one primitive
    // per distinct material its faces use.
    struct Prim {
        int material;
        std::vector<SplitVertex> verts;
        std::vector<uint32_t> indices;
    };
    struct Part {
        int id;
        std::string name;
        std::vector<Prim> prims;
    };
    std::vector<Part> parts;
    std::map<int, BRepAdaptor_Surface> cache;

    for (const auto& [partId, polyIdxs] : polysOfPart) {
    // Bucket this part's polygons by material (deterministic ascending order).
    std::map<int, std::vector<size_t>> polysOfMat;
    for (size_t p : polyIdxs) polysOfMat[matForFace(mesh.polygonFaceId[p])].push_back(p);

    Part part{partId, {}, {}};
    for (const auto& [matId, matPolys] : polysOfMat) {
    // Split vertices per (vertex, face) and fan-triangulate the polygons.
    std::map<std::pair<uint32_t, int>, uint32_t> splitOf;
    std::vector<SplitVertex> verts;
    std::vector<uint32_t> indices;

    auto splitVertex = [&](uint32_t idx, int fid) {
        auto key = std::make_pair(idx, fid);
        auto it = splitOf.find(key);
        if (it != splitOf.end()) return it->second;
        SplitVertex v{};
        double px = mesh.vertices[idx][0] * opts.scale;
        double py = mesh.vertices[idx][1] * opts.scale;
        double pz = mesh.vertices[idx][2] * opts.scale;
        if (opts.yUp) {  // Z-up CAD -> Y-up engine (X stays, Z->Y, Y->-Z)
            double ny = pz, nz = -py;
            py = ny;
            pz = nz;
        }
        v.px = static_cast<float>(px);
        v.py = static_cast<float>(py);
        v.pz = static_cast<float>(pz);
        std::array<double, 3> n{0.0, 0.0, 1.0};
        if (!opts.emitNormals || !model ||
            !detail::cadNormal(mesh, *model, idx, fid, cache, n)) {
            n = {0.0, 0.0, 0.0};  // filled from polygon fan below
        }
        if (opts.yUp) {
            double ny = n[2], nz = -n[1];
            n[1] = ny;
            n[2] = nz;
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

    for (size_t p : matPolys) {
        const auto& poly = mesh.polygons[p];
        const int fid = mesh.polygonFaceId[p];
        if (poly.size() < 3) continue;
        std::vector<uint32_t> ring(poly.size());
        for (size_t i = 0; i < poly.size(); ++i) {
            ring[i] = splitVertex(poly[i], fid);
        }
        // Ear-clipped like the OBJ path, so concave/keyhole n-gons export
        // correctly instead of as overlapping fans.
        for (const auto& t : triangulatePoly(mesh.vertices, poly)) {
            indices.push_back(ring[t[0]]);
            indices.push_back(ring[t[1]]);
            indices.push_back(ring[t[2]]);
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
            double fnx = cx / len, fny = cy / len, fnz = cz / len;
            if (opts.yUp) {
                double ry = fnz, rz = -fny;
                fny = ry;
                fnz = rz;
            }
            for (uint32_t s : ring) {
                SplitVertex& v = verts[s];
                if (v.nx == 0.0f && v.ny == 0.0f && v.nz == 0.0f) {
                    v.nx = static_cast<float>(fnx);
                    v.ny = static_cast<float>(fny);
                    v.nz = static_cast<float>(fnz);
                }
            }
        }
    }
    if (!verts.empty())
        part.prims.push_back({matId, std::move(verts), std::move(indices)});
    }  // per material

    if (!part.prims.empty()) {
        part.name = "object_" + std::to_string(partId);
        if (model && partId >= 1 && partId <= (int)model->solidNames.size() &&
            !model->solidNames[partId - 1].empty()) {
            part.name.clear();
            for (char c : model->solidNames[partId - 1]) {  // JSON-safe
                if (c >= ' ' && c != '"' && c != '\\') part.name += c;
            }
            if (part.name.empty()) part.name = "object_" + std::to_string(partId);
        }
        parts.push_back(std::move(part));
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
        std::string prims;  // primitive entries for this part's mesh
        for (size_t gi = 0; gi < part.prims.size(); ++gi) {
            const Prim& prim = part.prims[gi];
            float pmin[3] = {std::numeric_limits<float>::max(),
                             std::numeric_limits<float>::max(),
                             std::numeric_limits<float>::max()};
            float pmax[3] = {-std::numeric_limits<float>::max(),
                             -std::numeric_limits<float>::max(),
                             -std::numeric_limits<float>::max()};
            const size_t posOff = bin.size();
            for (const SplitVertex& v : prim.verts) {
                float p3[3] = {v.px, v.py, v.pz};
                for (int i = 0; i < 3; ++i) {
                    pmin[i] = std::min(pmin[i], p3[i]);
                    pmax[i] = std::max(pmax[i], p3[i]);
                }
                append(bin, p3, sizeof p3);
            }
            const size_t normOff = bin.size();
            for (const SplitVertex& v : prim.verts) {
                float n3[3] = {v.nx, v.ny, v.nz};
                append(bin, n3, sizeof n3);
            }
            const size_t fidOff = bin.size();
            for (const SplitVertex& v : prim.verts) {
                append(bin, &v.faceId, sizeof v.faceId);
            }
            const size_t idxOff = bin.size();
            append(bin, prim.indices.data(), prim.indices.size() * sizeof(uint32_t));
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
                addView(idxOff, prim.indices.size() * sizeof(uint32_t), 34963);

            std::snprintf(buf, sizeof buf,
                          "%s{\"bufferView\":%d,\"componentType\":5126,"
                          "\"count\":%zu,\"type\":\"VEC3\","
                          "\"min\":[%.9g,%.9g,%.9g],\"max\":[%.9g,%.9g,%.9g]}",
                          acc ? "," : "", vPos, prim.verts.size(), pmin[0], pmin[1],
                          pmin[2], pmax[0], pmax[1], pmax[2]);
            accessors += buf;
            const int aPos = acc++;
            std::snprintf(buf, sizeof buf,
                          ",{\"bufferView\":%d,\"componentType\":5126,"
                          "\"count\":%zu,\"type\":\"VEC3\"}",
                          vNorm, prim.verts.size());
            accessors += buf;
            const int aNorm = acc++;
            std::snprintf(buf, sizeof buf,
                          ",{\"bufferView\":%d,\"componentType\":5126,"
                          "\"count\":%zu,\"type\":\"SCALAR\"}",
                          vFid, prim.verts.size());
            accessors += buf;
            const int aFid = acc++;
            std::snprintf(buf, sizeof buf,
                          ",{\"bufferView\":%d,\"componentType\":5125,"
                          "\"count\":%zu,\"type\":\"SCALAR\"}",
                          vIdx, prim.indices.size());
            accessors += buf;
            const int aIdx = acc++;

            // Primitive entry (append a "material" only when the face group
            // carries one, so uncolored output stays byte-for-byte identical).
            std::snprintf(buf, sizeof buf,
                          "%s{\"attributes\":{\"POSITION\":%d,\"NORMAL\":%d,"
                          "\"_WEFT_FACE_ID\":%d},\"indices\":%d,\"mode\":4",
                          gi ? "," : "", aPos, aNorm, aFid, aIdx);
            prims += buf;
            if (prim.material >= 0) {
                std::snprintf(buf, sizeof buf, ",\"material\":%d", prim.material);
                prims += buf;
            }
            prims += "}";
        }

        meshes += (pi ? "," : "") + std::string("{\"name\":\"") + part.name +
                  "\",\"primitives\":[" + prims + "]}";
    }

    // Node graph. When the source carried a real assembly (more than a single
    // degenerate root), mirror it: one glTF node per AssemblyNode, meshes
    // attached to the solid leaves, children preserved. Geometry is already
    // world-placed at import, so nodes carry names/structure only (no matrix).
    // Otherwise (flat / degenerate assembly) keep one node per body.
    std::map<int, size_t> meshOfSolid;  // solidId (== part.id) -> mesh index
    for (size_t pi = 0; pi < parts.size(); ++pi) meshOfSolid[parts[pi].id] = pi;

    auto jsonName = [](const std::string& s) {
        std::string o;
        for (char c : s)
            if (c >= ' ' && c != '"' && c != '\\') o += c;
        return o;
    };

    const bool useAssembly = model && model->assembly.size() > 1;
    if (useAssembly) {
        const auto& asm_ = model->assembly;
        std::vector<char> isChild(asm_.size(), 0);
        for (const auto& n : asm_)
            for (int ch : n.children)
                if (ch >= 0 && ch < (int)asm_.size()) isChild[ch] = 1;
        std::vector<char> solidPlaced(parts.size(), 0);
        for (size_t j = 0; j < asm_.size(); ++j) {
            const AssemblyNode& n = asm_[j];
            std::string entry = (j ? "," : "");
            entry += "{\"name\":\"" + jsonName(n.name) + "\"";
            auto it = meshOfSolid.find(n.solidId);
            if (n.solidId >= 1 && it != meshOfSolid.end()) {
                entry += ",\"mesh\":" + std::to_string(it->second);
                solidPlaced[it->second] = 1;
            }
            if (!n.children.empty()) {
                entry += ",\"children\":[";
                bool first = true;
                for (int ch : n.children) {
                    if (ch < 0 || ch >= (int)asm_.size()) continue;
                    entry += (first ? "" : ",") + std::to_string(ch);
                    first = false;
                }
                entry += "]";
            }
            entry += "}";
            nodes += entry;
            if (!isChild[j]) roots += (roots.empty() ? "" : ",") + std::to_string(j);
        }
        // Safety net: any body the assembly never referenced becomes a root
        // node of its own, so no geometry is dropped from the scene.
        size_t extra = asm_.size();
        for (size_t pi = 0; pi < parts.size(); ++pi) {
            if (solidPlaced[pi]) continue;
            nodes += ",{\"mesh\":" + std::to_string(pi) + ",\"name\":\"" +
                     jsonName(parts[pi].name) + "\"}";
            roots += (roots.empty() ? "" : ",") + std::to_string(extra++);
        }
    } else {
        for (size_t pi = 0; pi < parts.size(); ++pi) {
            std::snprintf(buf, sizeof buf, "%s{\"mesh\":%zu,\"name\":\"%s\"}",
                          pi ? "," : "", pi, parts[pi].name.c_str());
            nodes += buf;
            std::snprintf(buf, sizeof buf, "%s%zu", pi ? "," : "", pi);
            roots += buf;
        }
    }

    // Materials table (PBR baseColorFactor from the source color). Emitted
    // only when at least one face carried a color.
    std::string materialsJson;
    for (size_t mi = 0; mi < materials.size(); ++mi) {
        std::snprintf(buf, sizeof buf,
                      "%s{\"pbrMetallicRoughness\":{\"baseColorFactor\":"
                      "[%.6g,%.6g,%.6g,1],\"metallicFactor\":0,"
                      "\"roughnessFactor\":0.8}}",
                      mi ? "," : "", materials[mi][0], materials[mi][1], materials[mi][2]);
        materialsJson += buf;
    }

    // Insert the materials array only when present (keeps uncolored output
    // byte-identical to the pre-materials writer).
    std::string matSection =
        materialsJson.empty() ? "" : ("\"materials\":[" + materialsJson + "],");
    std::string json = "{\"asset\":{\"version\":\"2.0\","
                       "\"generator\":\"weft\"},\"scene\":0,"
                       "\"scenes\":[{\"nodes\":[" + roots + "]}],"
                       "\"nodes\":[" + nodes + "],"
                       "\"meshes\":[" + meshes + "]," + matSection +
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
