// Mesh-format writers for the weft::io registry. Obj/Gltf/Fbx are thin
// adapters over Weft's hand-rolled exporters (which carry the generated
// topology, exact CAD normals and _WEFT_FACE_ID that the OCCT CAF writers
// would discard). Stl triangulates the PolyMesh and writes it directly.

#include "weft/io/writer.hpp"

#include "../out_file.hpp"

#include "weft/export_fbx.hpp"
#include "weft/export_gltf.hpp"
#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace weft::io {

namespace {

// --------------------------------------------------------------------- OBJ ---
class ObjWriter final : public Writer {
public:
    void applyParams(const ParamGroup& g) override {
        m_opts.triangulate = g.getBool("triangulate");
        m_opts.yUp = g.getBool("yUp");
        m_opts.scale = g.getDouble("scale");
        m_opts.emitNormals = g.getBool("normals");
        m_opts.emitColors = g.getBool("mtl");
    }
    bool transfer(const WriteInput& in) override {
        if (!in.mesh) return false;
        m_in = in;
        return true;
    }
    bool writeFile(const std::string& path) override {
        ObjExportOptions o = m_opts;
        if (m_in.model) {
            o.objectNames = &m_in.model->solidNames;
            o.model = m_in.model;
        }
        writeObj(*m_in.mesh, path, m_in.solidFaces, &o);
        return true;
    }

private:
    WriteInput m_in;
    ObjExportOptions m_opts;
};

// -------------------------------------------------------------------- glTF ---
class GltfWriter final : public Writer {
public:
    void applyParams(const ParamGroup& g) override {
        m_opts.scale = g.getDouble("scale");
        m_opts.yUp = g.getBool("yUp");
        m_opts.emitNormals = g.getBool("normals");
        m_opts.embedColors = g.getBool("embedColors");
    }
    bool transfer(const WriteInput& in) override {
        if (!in.mesh) return false;
        m_in = in;
        return true;
    }
    bool writeFile(const std::string& path) override {
        GltfExportOptions o = m_opts;
        // Unit convention: glTF is metres. If the caller left scale at the
        // metres default and the source declared a non-mm unit, fold it in.
        if (m_in.model && std::abs(o.scale - 0.001) < 1e-12 &&
            m_in.model->lengthUnitMm != 1.0) {
            o.scale = 0.001 * m_in.model->lengthUnitMm;
        }
        writeGlb(*m_in.mesh, path, m_in.model, m_in.solidFaces, &o);
        return true;
    }

private:
    WriteInput m_in;
    GltfExportOptions m_opts;
};

// --------------------------------------------------------------------- FBX ---
class FbxWriter final : public Writer {
public:
    void applyParams(const ParamGroup& g) override {
        m_opts.triangulate = g.getBool("triangulate");
        m_opts.yUp = g.getBool("yUp");
        m_opts.scale = g.getDouble("scale");
    }
    bool transfer(const WriteInput& in) override {
        if (!in.mesh) return false;
        m_in = in;
        return true;
    }
    bool writeFile(const std::string& path) override {
        writeFbx(*m_in.mesh, path, m_opts);
        return true;
    }

private:
    WriteInput m_in;
    FbxExportOptions m_opts;
};

// --------------------------------------------------------------------- STL ---
// STL is pure mesh: its only knob is ascii/binary; it carries no color or
// hierarchy. Triangulate the PolyMesh (ear-clipped, matching every other
// export path) and emit facet normals from the triangle geometry.
class StlWriter final : public Writer {
public:
    void applyParams(const ParamGroup& g) override { m_ascii = g.getBool("ascii"); }
    bool transfer(const WriteInput& in) override {
        if (!in.mesh) return false;
        m_in = in;
        return true;
    }
    bool writeFile(const std::string& path) override {
        return m_ascii ? writeAscii(path) : writeBinary(path);
    }

private:
    struct Tri {
        float n[3];
        float v[3][3];
    };

    std::vector<Tri> triangles() const {
        const PolyMesh& mesh = *m_in.mesh;
        std::vector<Tri> out;
        for (size_t p = 0; p < mesh.polygons.size(); ++p) {
            const auto& poly = mesh.polygons[p];
            if (poly.size() < 3) continue;
            for (const auto& t : triangulatePoly(mesh.vertices, poly)) {
                const auto& a = mesh.vertices[poly[t[0]]];
                const auto& b = mesh.vertices[poly[t[1]]];
                const auto& c = mesh.vertices[poly[t[2]]];
                double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
                double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
                double nx = uy * vz - uz * vy;
                double ny = uz * vx - ux * vz;
                double nz = ux * vy - uy * vx;
                double len = std::sqrt(nx * nx + ny * ny + nz * nz);
                if (len > 1e-30) {
                    nx /= len;
                    ny /= len;
                    nz /= len;
                } else {
                    nx = ny = nz = 0.0;
                }
                Tri tri;
                tri.n[0] = static_cast<float>(nx);
                tri.n[1] = static_cast<float>(ny);
                tri.n[2] = static_cast<float>(nz);
                const std::array<double, 3>* vs[3] = {&a, &b, &c};
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        tri.v[i][j] = static_cast<float>((*vs[i])[j]);
                out.push_back(tri);
            }
        }
        return out;
    }

    bool writeBinary(const std::string& path) const {
        std::vector<Tri> tris = triangles();
        detail::OutFile out(path, "wb");
        FILE* f = out.get();
        char header[80] = {0};
        std::snprintf(header, sizeof header, "weft binary STL");
        std::fwrite(header, 1, 80, f);
        uint32_t count = static_cast<uint32_t>(tris.size());
        std::fwrite(&count, 4, 1, f);
        for (const Tri& t : tris) {
            std::fwrite(t.n, 4, 3, f);
            std::fwrite(t.v, 4, 9, f);
            uint16_t attr = 0;
            std::fwrite(&attr, 2, 1, f);
        }
        out.finish();
        return true;
    }

    bool writeAscii(const std::string& path) const {
        std::vector<Tri> tris = triangles();
        detail::OutFile out(path, "w");
        FILE* f = out.get();
        std::fprintf(f, "solid weft\n");
        for (const Tri& t : tris) {
            std::fprintf(f, "  facet normal %.6e %.6e %.6e\n", t.n[0], t.n[1], t.n[2]);
            std::fprintf(f, "    outer loop\n");
            for (int i = 0; i < 3; ++i)
                std::fprintf(f, "      vertex %.6e %.6e %.6e\n", t.v[i][0], t.v[i][1], t.v[i][2]);
            std::fprintf(f, "    endloop\n");
            std::fprintf(f, "  endfacet\n");
        }
        std::fprintf(f, "endsolid weft\n");
        out.finish();
        return true;
    }

    WriteInput m_in;
    bool m_ascii = false;
};

// ------------------------------------------------------------------ factory ---
class MeshFactoryWriter final : public FactoryWriter {
public:
    std::span<const Format> formats() const override {
        static constexpr Format kF[] = {Format::Obj, Format::Gltf, Format::Stl, Format::Fbx};
        return kF;
    }

    std::unique_ptr<Writer> create(Format f) const override {
        switch (f) {
            case Format::Obj: return std::make_unique<ObjWriter>();
            case Format::Gltf: return std::make_unique<GltfWriter>();
            case Format::Stl: return std::make_unique<StlWriter>();
            case Format::Fbx: return std::make_unique<FbxWriter>();
            default: return nullptr;
        }
    }

    ParamGroup createParams(Format f) const override {
        ParamGroup g;
        switch (f) {
            case Format::Obj:
                g.specs = {
                    {"triangulate", ParamSpec::Bool, {}, "false", "ear-clip everything to tris"},
                    {"yUp", ParamSpec::Bool, {}, "false", "rotate Z-up CAD into Y-up"},
                    {"scale", ParamSpec::Double, {}, "1", "unit multiplier"},
                    {"normals", ParamSpec::Bool, {}, "true", "emit exact CAD normals"},
                    {"mtl", ParamSpec::Bool, {}, "true", "emit .mtl sidecar for source colors"},
                };
                break;
            case Format::Gltf:
                g.specs = {
                    {"binary", ParamSpec::Bool, {}, "true", "glb (binary) vs gltf (text)"},
                    {"yUp", ParamSpec::Bool, {}, "true", "rotate Z-up CAD into Y-up"},
                    {"scale", ParamSpec::Double, {}, "0.001", "unit multiplier (mm->m)"},
                    {"normals", ParamSpec::Bool, {}, "true", "emit exact CAD normals"},
                    {"embedColors", ParamSpec::Bool, {}, "true", "materials[] from source colors"},
                };
                break;
            case Format::Stl:
                g.specs = {{"ascii", ParamSpec::Bool, {}, "false", "ascii vs binary STL"}};
                break;
            case Format::Fbx:
                g.specs = {
                    {"triangulate", ParamSpec::Bool, {}, "false", "ear-clip everything to tris"},
                    {"yUp", ParamSpec::Bool, {}, "true", "rotate Z-up CAD into Y-up"},
                    {"scale", ParamSpec::Double, {}, "1", "unit multiplier"},
                };
                break;
            default: break;
        }
        g.resetToDefaults();
        return g;
    }
};

}  // namespace

std::unique_ptr<FactoryWriter> makeMeshFactoryWriter() {
    return std::make_unique<MeshFactoryWriter>();
}

}  // namespace weft::io
