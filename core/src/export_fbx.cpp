#include "weft/export_fbx.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace weft {

namespace {

// Binary FBX is a tree of node records:
//   uint32 endOffset, uint32 numProps, uint32 propListLen,
//   uint8 nameLen, name, properties, children..., 13-byte null record.
// Properties are one type char + payload; arrays carry
//   uint32 length, uint32 encoding(0 = raw), uint32 byteLength, data.
// Offsets are absolute file positions, so the tree is built in memory
// and patched on the way out.
struct FbxNode {
    std::string name;
    std::string props;  // encoded property payloads, concatenated
    uint32_t numProps = 0;
    std::vector<FbxNode> children;
    bool forceNull = false;  // emit the null terminator even when empty

    void propI64(int64_t v) {
        props.push_back('L');
        props.append(reinterpret_cast<const char*>(&v), 8);
        ++numProps;
    }
    void propI32(int32_t v) {
        props.push_back('I');
        props.append(reinterpret_cast<const char*>(&v), 4);
        ++numProps;
    }
    void propStr(const std::string& s) {
        props.push_back('S');
        uint32_t n = uint32_t(s.size());
        props.append(reinterpret_cast<const char*>(&n), 4);
        props.append(s);
        ++numProps;
    }
    void propF64Array(const std::vector<double>& a) {
        props.push_back('d');
        uint32_t n = uint32_t(a.size());
        uint32_t enc = 0;
        uint32_t bytes = n * 8;
        props.append(reinterpret_cast<const char*>(&n), 4);
        props.append(reinterpret_cast<const char*>(&enc), 4);
        props.append(reinterpret_cast<const char*>(&bytes), 4);
        props.append(reinterpret_cast<const char*>(a.data()), bytes);
        ++numProps;
    }
    void propI32Array(const std::vector<int32_t>& a) {
        props.push_back('i');
        uint32_t n = uint32_t(a.size());
        uint32_t enc = 0;
        uint32_t bytes = n * 4;
        props.append(reinterpret_cast<const char*>(&n), 4);
        props.append(reinterpret_cast<const char*>(&enc), 4);
        props.append(reinterpret_cast<const char*>(&bytes), 4);
        props.append(reinterpret_cast<const char*>(a.data()), bytes);
        ++numProps;
    }
};

void writeNode(std::string& out, const FbxNode& n) {
    const size_t start = out.size();
    uint32_t zero = 0;
    out.append(reinterpret_cast<const char*>(&zero), 4);  // endOffset slot
    out.append(reinterpret_cast<const char*>(&n.numProps), 4);
    uint32_t plen = uint32_t(n.props.size());
    out.append(reinterpret_cast<const char*>(&plen), 4);
    out.push_back(char(uint8_t(n.name.size())));
    out.append(n.name);
    out.append(n.props);
    if (!n.children.empty() || n.forceNull) {
        for (const FbxNode& c : n.children) writeNode(out, c);
        out.append(13, '\0');  // null record closes the child list
    }
    uint32_t end = uint32_t(out.size());
    std::memcpy(&out[start], &end, 4);
}

}  // namespace

void writeFbx(const PolyMesh& mesh, const std::string& path,
              const FbxExportOptions& opts) {
    // Geometry payload: positions (transformed into the requested
    // engine space) and polygon indices with the FBX end-marker
    // convention (last index of each polygon is bitwise-negated).
    std::vector<double> verts;
    verts.reserve(mesh.vertices.size() * 3);
    for (const auto& p : mesh.vertices) {
        double x = p[0] * opts.scale;
        double y = p[1] * opts.scale;
        double z = p[2] * opts.scale;
        if (opts.yUp) {
            verts.push_back(x);
            verts.push_back(z);
            verts.push_back(-y);
        } else {
            verts.push_back(x);
            verts.push_back(y);
            verts.push_back(z);
        }
    }
    std::vector<int32_t> idx;
    for (const auto& poly : mesh.polygons) {
        if (poly.size() < 3) continue;
        if (opts.triangulate && poly.size() > 3) {
            for (size_t i = 1; i + 1 < poly.size(); ++i) {
                idx.push_back(int32_t(poly[0]));
                idx.push_back(int32_t(poly[i]));
                idx.push_back(~int32_t(poly[i + 1]));
            }
        } else {
            for (size_t i = 0; i < poly.size(); ++i) {
                int32_t v = int32_t(poly[i]);
                idx.push_back(i + 1 == poly.size() ? ~v : v);
            }
        }
    }

    const int64_t geomId = 1000001;
    const int64_t modelId = 1000002;
    const std::string sep("\x00\x01", 2);  // FBX "name\0\1Class" format

    std::vector<FbxNode> top;

    {
        FbxNode header;
        header.name = "FBXHeaderExtension";
        FbxNode hv;
        hv.name = "FBXHeaderVersion";
        hv.propI32(1003);
        FbxNode fv;
        fv.name = "FBXVersion";
        fv.propI32(7400);
        FbxNode cr;
        cr.name = "Creator";
        cr.propStr("weft b-rep retopology");
        header.children = {hv, fv, cr};
        top.push_back(std::move(header));
    }
    {
        FbxNode gs;
        gs.name = "GlobalSettings";
        FbxNode v;
        v.name = "Version";
        v.propI32(1000);
        FbxNode p70;
        p70.name = "Properties70";
        p70.forceNull = true;
        gs.children = {v, p70};
        top.push_back(std::move(gs));
    }
    {
        FbxNode defs;
        defs.name = "Definitions";
        FbxNode v;
        v.name = "Version";
        v.propI32(100);
        FbxNode count;
        count.name = "Count";
        count.propI32(2);
        auto objType = [&](const char* type) {
            FbxNode t;
            t.name = "ObjectType";
            t.propStr(type);
            FbxNode c;
            c.name = "Count";
            c.propI32(1);
            t.children = {c};
            return t;
        };
        defs.children = {v, count, objType("Geometry"), objType("Model")};
        top.push_back(std::move(defs));
    }
    {
        FbxNode objects;
        objects.name = "Objects";

        FbxNode geom;
        geom.name = "Geometry";
        geom.propI64(geomId);
        geom.propStr("weft" + sep + "Geometry");
        geom.propStr("Mesh");
        FbxNode vtx;
        vtx.name = "Vertices";
        vtx.propF64Array(verts);
        FbxNode pvi;
        pvi.name = "PolygonVertexIndex";
        pvi.propI32Array(idx);
        FbxNode gv;
        gv.name = "GeometryVersion";
        gv.propI32(124);
        geom.children = {vtx, pvi, gv};

        FbxNode model;
        model.name = "Model";
        model.propI64(modelId);
        model.propStr("weft" + sep + "Model");
        model.propStr("Mesh");
        FbxNode mv;
        mv.name = "Version";
        mv.propI32(232);
        FbxNode p70;
        p70.name = "Properties70";
        p70.forceNull = true;
        model.children = {mv, p70};

        objects.children = {std::move(geom), std::move(model)};
        top.push_back(std::move(objects));
    }
    {
        FbxNode conns;
        conns.name = "Connections";
        FbxNode c1;
        c1.name = "C";
        c1.propStr("OO");
        c1.propI64(geomId);
        c1.propI64(modelId);
        FbxNode c2;
        c2.name = "C";
        c2.propStr("OO");
        c2.propI64(modelId);
        c2.propI64(0);
        conns.children = {c1, c2};
        top.push_back(std::move(conns));
    }

    std::string out;
    out.append("Kaydara FBX Binary  ");
    out.push_back('\0');
    out.push_back('\x1a');
    out.push_back('\0');
    uint32_t version = 7400;
    out.append(reinterpret_cast<const char*>(&version), 4);
    for (const FbxNode& n : top) writeNode(out, n);
    out.append(13, '\0');  // end of top-level list

    // Footer: the 16-byte id importers expect, zero padding to a
    // 16-byte boundary, the version again, 120 zeros, and the magic.
    static const uint8_t footId[16] = {0xfa, 0xbc, 0xab, 0x09, 0xd0, 0xc8,
                                       0xd4, 0x66, 0xb1, 0x76, 0xfb, 0x83,
                                       0x1c, 0xf7, 0x26, 0x7e};
    out.append(reinterpret_cast<const char*>(footId), 16);
    while (out.size() % 16 != 0) out.push_back('\0');
    out.append(4, '\0');
    out.append(reinterpret_cast<const char*>(&version), 4);
    out.append(120, '\0');
    static const uint8_t footMagic[16] = {0xf8, 0x5a, 0x8c, 0x6a, 0xde, 0xf5,
                                          0xd9, 0x7e, 0xec, 0xe9, 0x0c, 0xe3,
                                          0x75, 0x8f, 0x29, 0x0b};
    out.append(reinterpret_cast<const char*>(footMagic), 16);

    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write " + path);
    f.write(out.data(), std::streamsize(out.size()));
    f.close();  // flush here, not in the destructor, so a failure is visible
    if (!f) {
        std::remove(path.c_str());  // never leave a truncated FBX behind
        throw std::runtime_error("failed to write (disk full or I/O error): " +
                                 path);
    }
}

}  // namespace weft
