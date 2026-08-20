// Unit tests for the delivery/IO layer: the glTF and FBX writers, the
// registry (formats, params, probes, mesh/B-rep writers), the IGES/BREP
// readers, tessellation, edge sampling for the viewport, and the mesher
// debug log. Same harness convention as test_pipeline.cpp — no framework;
// each CHECK prints and the process exits non-zero on failure.

#include "weft/analysis.hpp"
#include "weft/export_fbx.hpp"
#include "weft/export_gltf.hpp"
#include "weft/fixture.hpp"
#include "weft/io/system.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/viz.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++failures;                                                 \
        }                                                               \
    } while (0)

#define CHECK_EQ(a, b)                                                   \
    do {                                                                 \
        auto va = (a);                                                   \
        auto vb = (b);                                                   \
        if (!(va == vb)) {                                               \
            std::printf("FAIL %s:%d: %s == %s (%lld vs %lld)\n",         \
                        __FILE__, __LINE__, #a, #b, (long long)va,       \
                        (long long)vb);                                  \
            ++failures;                                                  \
        }                                                                \
    } while (0)

namespace {

std::string tmpPath(const std::string& name) {
    for (const char* var : {"TMPDIR", "TMP", "TEMP"}) {
        if (const char* dir = std::getenv(var); dir && *dir) {
            return std::string(dir) + "/" + name;
        }
    }
    return name;
}

std::vector<uint8_t> readBytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(f),
                                std::istreambuf_iterator<char>());
}

// True when `f` threw std::runtime_error (the writers' only failure signal).
bool throwsRuntime(const std::function<void()>& f) {
    try {
        f();
    } catch (const std::runtime_error&) {
        return true;
    }
    return false;
}

size_t countOccurrences(const std::string& hay, const std::string& needle) {
    size_t n = 0;
    for (size_t p = hay.find(needle); p != std::string::npos;
         p = hay.find(needle, p + needle.size())) {
        ++n;
    }
    return n;
}

// Every integer that follows `key` in document order (used to read the
// accessors' "count" values without pulling in a JSON parser).
std::vector<long> intsAfterKey(const std::string& json, const std::string& key) {
    std::vector<long> out;
    for (size_t p = json.find(key); p != std::string::npos;
         p = json.find(key, p + key.size())) {
        out.push_back(std::strtol(json.c_str() + p + key.size(), nullptr, 10));
    }
    return out;
}

uint32_t u32At(const std::vector<uint8_t>& b, size_t off) {
    uint32_t v = 0;
    std::memcpy(&v, b.data() + off, 4);
    return v;
}

float f32At(const std::vector<uint8_t>& b, size_t off) {
    float v = 0;
    std::memcpy(&v, b.data() + off, 4);
    return v;
}

struct Glb {
    std::string json;
    std::vector<uint8_t> bin;
};

// Validate the .glb container and split it into its two chunks.
Glb readGlb(const std::string& path) {
    const std::vector<uint8_t> bytes = readBytes(path);
    CHECK(bytes.size() > 20);
    CHECK_EQ(u32At(bytes, 0), 0x46546C67u);  // "glTF"
    CHECK_EQ(u32At(bytes, 4), 2u);
    CHECK_EQ(u32At(bytes, 8), (uint32_t)bytes.size());
    const uint32_t jsonLen = u32At(bytes, 12);
    CHECK_EQ(u32At(bytes, 16), 0x4E4F534Au);  // "JSON"
    CHECK_EQ(jsonLen % 4, 0u);
    Glb glb;
    glb.json.assign(bytes.begin() + 20, bytes.begin() + 20 + jsonLen);
    const size_t binHeader = 20 + jsonLen;
    const uint32_t binLen = u32At(bytes, binHeader);
    CHECK_EQ(u32At(bytes, binHeader + 4), 0x004E4942u);  // "BIN"
    CHECK_EQ(binHeader + 8 + binLen, bytes.size());
    glb.bin.assign(bytes.begin() + binHeader + 8, bytes.end());
    return glb;
}

struct Bounds {
    double lo[3] = {1e300, 1e300, 1e300};
    double hi[3] = {-1e300, -1e300, -1e300};
    void add(double x, double y, double z) {
        const double p[3] = {x, y, z};
        for (int i = 0; i < 3; ++i) {
            lo[i] = std::min(lo[i], p[i]);
            hi[i] = std::max(hi[i], p[i]);
        }
    }
};

Bounds meshBounds(const weft::PolyMesh& mesh) {
    Bounds b;
    for (const auto& v : mesh.vertices) b.add(v[0], v[1], v[2]);
    return b;
}

// Positions of a single-primitive glb: the first bufferView is POSITION at
// offset 0 (the writer lays out pos / normal / faceId / indices per prim).
Bounds glbPositionBounds(const Glb& glb, size_t vertexCount) {
    Bounds b;
    for (size_t i = 0; i < vertexCount; ++i) {
        b.add(f32At(glb.bin, i * 12), f32At(glb.bin, i * 12 + 4),
              f32At(glb.bin, i * 12 + 8));
    }
    return b;
}

bool nearly(double a, double b, double tol = 1e-3) {
    return std::abs(a - b) <= tol * std::max(1.0, std::abs(b));
}

// A meshed fixture plus everything the exporters consume.
struct Sample {
    weft::Model model;
    weft::Analysis analysis;
    weft::PolyMesh mesh;
};

Sample makeSample(const std::string& fixture, int radial = 8, int axial = 2) {
    Sample s;
    const std::string stepPath = tmpPath("weft_io_" + fixture + ".step");
    weft::writeStep(weft::makeFixture(fixture), stepPath);
    s.model = weft::loadStep(stepPath);
    s.analysis = weft::analyze(s.model);
    weft::GenerationSettings gs;
    gs.defaults.minimal = false;
    gs.defaults.radial = radial;
    gs.defaults.axial = axial;
    s.mesh = weft::generate(s.model, s.analysis, gs);
    CHECK(!s.mesh.polygons.empty());
    return s;
}

size_t triangleCount(const weft::PolyMesh& mesh) {
    size_t n = 0;
    for (const auto& poly : mesh.polygons) {
        if (poly.size() < 3) continue;
        n += weft::triangulatePoly(mesh.vertices, poly).size();
    }
    return n;
}

// ------------------------------------------------------------ format table ---

void testFormatMetadata() {
    std::printf("-- format metadata --\n");
    using weft::io::Format;
    const Format all[] = {Format::Step, Format::Iges, Format::Brep,
                          Format::Stl,  Format::Obj,  Format::Gltf,
                          Format::Ply,  Format::Fbx};
    for (Format f : all) {
        CHECK(weft::io::formatIdentifier(f) != "UNKNOWN");
        CHECK(weft::io::formatName(f) != "Unknown");
        CHECK(!weft::io::formatFileSuffixes(f).empty());
        // Every format is exactly one of B-rep or mesh.
        CHECK(weft::io::formatProvidesBRep(f) != weft::io::formatProvidesMesh(f));
    }
    CHECK(weft::io::formatIdentifier(Format::Step) == "STEP");
    CHECK(weft::io::formatName(Format::Obj) == "Wavefront OBJ");
    CHECK(weft::io::formatFileSuffixes(Format::Gltf).size() == 2);
    CHECK(weft::io::formatFileSuffixes(Format::Gltf)[1] == "glb");
    CHECK(weft::io::formatProvidesBRep(Format::Iges));
    CHECK(weft::io::formatProvidesMesh(Format::Fbx));

    // Unknown carries no metadata and belongs to neither class.
    CHECK(weft::io::formatIdentifier(Format::Unknown) == "UNKNOWN");
    CHECK(weft::io::formatName(Format::Unknown) == "Unknown");
    CHECK(weft::io::formatFileSuffixes(Format::Unknown).empty());
    CHECK(!weft::io::formatProvidesBRep(Format::Unknown));
    CHECK(!weft::io::formatProvidesMesh(Format::Unknown));
}

// ------------------------------------------------------------- param groups ---

void testParamGroup() {
    std::printf("-- io param groups --\n");
    using weft::io::ParamGroup;
    using weft::io::ParamSpec;

    ParamGroup g;
    g.specs = {
        {"flag", ParamSpec::Bool, {}, "true", "a bool"},
        {"count", ParamSpec::Int, {}, "7", "an int"},
        {"scale", ParamSpec::Double, {}, "0.5", "a double"},
        {"schema", ParamSpec::Enum, {"ap203", "ap214"}, "ap214", "an enum"},
        {"label", ParamSpec::String, {}, "weft", "a string"},
    };
    g.resetToDefaults();
    CHECK_EQ(g.values.size(), 5u);
    CHECK(g.getBool("flag"));
    CHECK_EQ(g.getInt("count"), 7);
    CHECK(nearly(g.getDouble("scale"), 0.5));
    CHECK(g.getEnum("schema") == "ap214");
    CHECK(g.find("label") != nullptr);
    CHECK(g.find("nope") == nullptr);

    // Every accepted spelling of true, and anything else is false.
    for (const char* yes : {"1", "true", "yes", "on"}) {
        g.set("flag", yes);
        CHECK(g.getBool("flag"));
    }
    for (const char* no : {"0", "false", "off", ""}) {
        g.set("flag", no);
        CHECK(!g.getBool("flag"));
    }

    g.set("count", "-3");
    CHECK_EQ(g.getInt("count"), -3);
    g.set("scale", "2.25");
    CHECK(nearly(g.getDouble("scale"), 2.25));
    g.set("schema", "ap203");
    CHECK(g.getEnum("schema") == "ap203");
    // A non-enum spec passes its raw value through getEnum unvalidated.
    CHECK(g.getEnum("label") == "weft");

    // Unknown keys and off-list enum values are hard errors, not silent
    // defaults — a typo in a recipe must not change the written file.
    CHECK(throwsRuntime([&] { g.set("nope", "1"); }));
    CHECK(throwsRuntime([&] { g.set("schema", "ap999"); }));
    CHECK(throwsRuntime([&] { (void)g.getBool("nope"); }));
    g.values["schema"] = "ap999";  // bypass set() to hit the read-side check
    CHECK(throwsRuntime([&] { (void)g.getEnum("schema"); }));

    // A missing value falls back to the spec default; resetToDefaults is
    // idempotent and restores it.
    g.values.erase("count");
    CHECK_EQ(g.getInt("count"), 7);
    g.resetToDefaults();
    CHECK(g.values["count"] == "7");
    CHECK(g.values["schema"] == "ap214");
}

// ----------------------------------------------------------- format probes ---

void writeText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    f.write(text.data(), (std::streamsize)text.size());
}

void testFormatProbes() {
    std::printf("-- format probes --\n");
    using weft::io::Format;
    weft::io::System sys;
    weft::io::bootstrapIo(sys);

    auto probe = [&](const std::string& name, const std::string& content) {
        const std::string path = tmpPath(name);
        writeText(path, content);
        return sys.probeFormat(path);
    };

    CHECK(probe("weft_probe.dat", "ISO-10303-21;\nHEADER;\n") == Format::Step);
    CHECK(probe("weft_probe_brep.dat", "DBRep_DrawableShape\n\nCASCADE Topology V1\n") ==
          Format::Brep);
    CHECK(probe("weft_probe_iges.dat",
                std::string(72, ' ') + "S      1\n") == Format::Iges);
    CHECK(probe("weft_probe_stl.dat", "solid weft\nfacet normal 0 0 1\n") == Format::Stl);
    CHECK(probe("weft_probe_ply.dat", "ply format ascii 1.0\nelement vertex 0\n") ==
          Format::Ply);
    CHECK(probe("weft_probe_obj.dat", "# weft\nv 1.0 2.0 3.0\nf 1 1 1\n") == Format::Obj);
    CHECK(probe("weft_probe_gltf.dat",
                "{\"asset\":{\"version\":\"2.0\"},\"scene\":0}") == Format::Gltf);
    CHECK(probe("weft_probe_none.dat", "\x01\x02 nothing recognizable here\n") ==
          Format::Unknown);

    // Binary STL is recognized by its exact size law: 84 + 50 * facets.
    {
        const std::string path = tmpPath("weft_probe_bin.dat");
        std::string content(84, '\0');
        content[80] = 2;  // two facets
        content.append(100, '\0');
        writeText(path, content);
        CHECK(sys.probeFormat(path) == Format::Stl);
        // Truncate one byte and the size law fails -> no longer STL.
        writeText(path, content.substr(0, content.size() - 1));
        CHECK(sys.probeFormat(path) != Format::Stl);
    }

    // Binary glb magic.
    {
        const std::string path = tmpPath("weft_probe_glb.dat");
        writeText(path, std::string("glTF") + std::string(16, '\0'));
        CHECK(sys.probeFormat(path) == Format::Gltf);
    }

    // Content wins over a lying extension; an unreadable/empty file falls
    // back to the extension; output paths are extension-only.
    {
        const std::string path = tmpPath("weft_probe_liar.obj");
        writeText(path, "ISO-10303-21;\nHEADER;\n");
        CHECK(sys.probeFormat(path) == Format::Step);
        const std::string empty = tmpPath("weft_probe_empty.STP");
        writeText(empty, "");
        CHECK(sys.probeFormat(empty) == Format::Step);  // uppercase extension
        CHECK(sys.probeFormat(tmpPath("weft_probe_missing.brep")) == Format::Brep);
        CHECK(sys.probeFormat(tmpPath("weft_probe_missing.zzz")) == Format::Unknown);
        CHECK(sys.probeFormat(tmpPath("weft_probe_noext")) == Format::Unknown);
    }
    CHECK(sys.probeFormatForOutput("out.glb") == Format::Gltf);
    CHECK(sys.probeFormatForOutput("out.fbx") == Format::Fbx);  // writer-only format
    CHECK(sys.probeFormatForOutput("out.unknown") == Format::Unknown);
}

// -------------------------------------------------------- writer registry ---

void testWriterRegistry() {
    std::printf("-- writer registry --\n");
    using weft::io::Format;
    weft::io::System sys;
    weft::io::bootstrapIo(sys);

    for (Format f : {Format::Step, Format::Iges, Format::Brep}) {
        CHECK(sys.findFactoryReader(f) != nullptr);
        CHECK(sys.createReader(f) != nullptr);
        CHECK(sys.createWriter(f) != nullptr);
    }
    for (Format f : {Format::Obj, Format::Gltf, Format::Stl, Format::Fbx}) {
        CHECK(sys.createWriter(f) != nullptr);
    }
    CHECK_EQ(sys.readerFormats().size(), 3u);
    CHECK_EQ(sys.writerFormats().size(), 7u);  // 4 mesh + 3 B-rep
    CHECK(sys.findFactoryReader(Format::Obj) == nullptr);  // mesh import: not yet
    CHECK(sys.createReader(Format::Obj) == nullptr);
    CHECK(sys.findFactoryWriter(Format::Ply) == nullptr);
    CHECK(sys.createWriter(Format::Ply) == nullptr);

    // Advertised writer params: every spec has a default in `values`.
    const auto checkParams = [&](Format f, size_t specCount) {
        const weft::io::FactoryWriter* factory = sys.findFactoryWriter(f);
        CHECK(factory != nullptr);
        if (!factory) return weft::io::ParamGroup{};
        weft::io::ParamGroup g = factory->createParams(f);
        CHECK_EQ(g.specs.size(), specCount);
        CHECK_EQ(g.values.size(), specCount);
        for (const auto& s : g.specs) CHECK(g.values.count(s.key) == 1);
        return g;
    };
    weft::io::ParamGroup obj = checkParams(Format::Obj, 5);
    CHECK(!obj.getBool("triangulate"));
    CHECK(!obj.getBool("yUp"));  // OBJ stays in CAD space by default
    CHECK(obj.getBool("normals"));
    weft::io::ParamGroup gltf = checkParams(Format::Gltf, 5);
    CHECK(gltf.getBool("yUp"));  // engine delivery is Y-up
    CHECK(nearly(gltf.getDouble("scale"), 0.001));
    weft::io::ParamGroup stl = checkParams(Format::Stl, 1);
    CHECK(!stl.getBool("ascii"));
    weft::io::ParamGroup fbx = checkParams(Format::Fbx, 3);
    CHECK(fbx.getBool("yUp"));
    weft::io::ParamGroup step = checkParams(Format::Step, 3);
    CHECK(step.getEnum("schema") == "ap214");
    CHECK(step.getEnum("unit") == "mm");
    weft::io::ParamGroup iges = checkParams(Format::Iges, 2);
    CHECK(iges.getEnum("brepMode") == "faces");
    CHECK(sys.findFactoryWriter(Format::Brep)->createParams(Format::Brep).specs.empty());

    // Pairing rules: mesh writers need a mesh, B-rep writers need a shape.
    const std::string out = tmpPath("weft_io_registry_out.stl");
    CHECK(throwsRuntime(
        [&] { weft::io::exportFile(sys, Format::Stl, weft::io::WriteInput{}, out); }));
    CHECK(throwsRuntime(
        [&] { weft::io::exportFile(sys, Format::Step, weft::io::WriteInput{}, out); }));
    weft::Model empty;
    weft::io::WriteInput brepIn;
    brepIn.model = &empty;  // null shape is still not a B-rep source
    CHECK(throwsRuntime(
        [&] { weft::io::exportFile(sys, Format::Step, brepIn, out); }));
    CHECK(throwsRuntime(
        [&] { weft::io::exportFile(sys, Format::Ply, brepIn, out); }));  // no writer
    CHECK(throwsRuntime([&] { (void)weft::io::importFile(sys, tmpPath("weft_io_no.zzz")); }));
    CHECK(throwsRuntime([&] {
        weft::io::convertFile(sys, tmpPath("weft_io_registry_out.stl"),
                              tmpPath("weft_io_registry_out.zzz"));
    }));
}

// ----------------------------------------------------------- glTF exporter ---

void testGlbExport() {
    std::printf("-- glb export --\n");
    Sample s = makeSample("cylinder");
    const std::string path = tmpPath("weft_io_cylinder.glb");

    // Defaults: CAD space, unit scale, exact normals, one node per body.
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    Glb glb = readGlb(path);
    CHECK(glb.json.find("\"generator\":\"weft\"") != std::string::npos);
    CHECK(glb.json.find("_WEFT_FACE_ID") != std::string::npos);
    CHECK(glb.json.find("\"materials\"") == std::string::npos);  // uncolored source
    CHECK_EQ(countOccurrences(glb.json, "\"mesh\":"), 1u);       // single body
    // The body carries the imported solid's name.
    CHECK(!s.model.solidNames.empty());
    CHECK(glb.json.find("\"name\":\"" + s.model.solidNames[0] + "\"") !=
          std::string::npos);

    // accessors[] counts: positions, normals, face ids (one per split
    // vertex) then the index count (3 per triangle).
    std::vector<long> counts = intsAfterKey(glb.json, "\"count\":");
    CHECK_EQ(counts.size(), 4u);
    const size_t verts = (size_t)counts[0];
    CHECK_EQ(counts[1], (long)verts);
    CHECK_EQ(counts[2], (long)verts);
    CHECK_EQ(counts[3], (long)(3 * triangleCount(s.mesh)));
    // Vertices split per (position, B-rep face), so there are at least as
    // many as the welded mesh has.
    CHECK(verts >= s.mesh.vertexCount());
    CHECK_EQ(glb.bin.size() % 4, 0u);

    // POSITION block spans the mesh bounding box exactly, and every normal
    // is unit length (exact CAD normals, flat fallback at poles/apexes).
    Bounds mb = meshBounds(s.mesh);
    Bounds gb = glbPositionBounds(glb, verts);
    for (int i = 0; i < 3; ++i) {
        CHECK(nearly(gb.lo[i], mb.lo[i]));
        CHECK(nearly(gb.hi[i], mb.hi[i]));
    }
    for (size_t i = 0; i < verts; ++i) {
        const size_t off = verts * 12 + i * 12;
        const double nx = f32At(glb.bin, off), ny = f32At(glb.bin, off + 4),
                     nz = f32At(glb.bin, off + 8);
        CHECK(nearly(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0, 1e-5));
    }
    // Face ids ride along as a float attribute in the model's id range.
    for (size_t i = 0; i < verts; ++i) {
        const float fid = f32At(glb.bin, verts * 24 + i * 4);
        CHECK(fid >= 1.0f && fid <= (float)s.model.faceCount());
    }

    // Engine space: scale multiplies positions, yUp maps (x, y, z) to
    // (x, z, -y).
    weft::GltfExportOptions opts;
    opts.scale = 2.0;
    opts.yUp = true;
    opts.emitNormals = false;  // flat polygon normals instead of CAD normals
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces, &opts);
    Glb engine = readGlb(path);
    const size_t engineVerts = (size_t)intsAfterKey(engine.json, "\"count\":")[0];
    Bounds eb = glbPositionBounds(engine, engineVerts);
    CHECK(nearly(eb.lo[0], 2.0 * mb.lo[0]));
    CHECK(nearly(eb.hi[0], 2.0 * mb.hi[0]));
    CHECK(nearly(eb.lo[1], 2.0 * mb.lo[2]));
    CHECK(nearly(eb.hi[1], 2.0 * mb.hi[2]));
    CHECK(nearly(eb.lo[2], -2.0 * mb.hi[1]));
    CHECK(nearly(eb.hi[2], -2.0 * mb.lo[1]));
    for (size_t i = 0; i < engineVerts; ++i) {
        const size_t off = engineVerts * 12 + i * 12;
        const double nx = f32At(engine.bin, off), ny = f32At(engine.bin, off + 4),
                     nz = f32At(engine.bin, off + 8);
        CHECK(nearly(std::sqrt(nx * nx + ny * ny + nz * nz), 1.0, 1e-5));
    }

    // Writing without a model still produces a valid file (flat normals).
    weft::writeGlb(s.mesh, path);
    CHECK(!readGlb(path).json.empty());

    // Nothing to export is an error, not an empty file.
    CHECK(throwsRuntime([&] { weft::writeGlb(weft::PolyMesh{}, path); }));
}

void testGlbColorsAndNames() {
    std::printf("-- glb materials and node names --\n");
    Sample s = makeSample("cylinder");
    const std::string path = tmpPath("weft_io_colored.glb");
    const int faces = s.model.faceCount();
    CHECK(faces >= 3);

    // Two distinct face colors become two materials, so the body's mesh
    // carries one primitive per material.
    s.model.faceColors.assign(faces, {0.25f, 0.5f, 0.75f});
    s.model.faceHasColor.assign(faces, 1);
    s.model.faceColors[0] = {1.0f, 0.0f, 0.0f};
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    Glb two = readGlb(path);
    CHECK_EQ(countOccurrences(two.json, "\"pbrMetallicRoughness\""), 2u);
    CHECK_EQ(countOccurrences(two.json, "\"material\":"), 2u);
    CHECK_EQ(countOccurrences(two.json, "\"mesh\":"), 1u);

    // Identical colors collapse to a single material.
    s.model.faceColors.assign(faces, {0.25f, 0.5f, 0.75f});
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    CHECK_EQ(countOccurrences(readGlb(path).json, "\"pbrMetallicRoughness\""), 1u);

    // embedColors off drops materials entirely.
    weft::GltfExportOptions plain;
    plain.embedColors = false;
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces, &plain);
    CHECK(readGlb(path).json.find("\"materials\"") == std::string::npos);

    // Uncolored faces fall back to the owning solid's color.
    s.model.faceHasColor.assign(faces, 0);
    s.model.solidColors.assign(1, {0.1f, 0.2f, 0.3f});
    s.model.solidHasColor.assign(1, 1);
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    CHECK_EQ(countOccurrences(readGlb(path).json, "\"pbrMetallicRoughness\""), 1u);

    // Body names come from the source product names, JSON-sanitized; a name
    // with nothing left after sanitizing falls back to object_<n>.
    s.model.solidNames.assign(1, "Body\"1\\x");
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    std::string json = readGlb(path).json;
    CHECK(json.find("\"name\":\"Body1x\"") != std::string::npos);
    s.model.solidNames[0] = "\"\\";
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    CHECK(readGlb(path).json.find("\"name\":\"object_1\"") != std::string::npos);
}

void testGlbAssemblyGraph() {
    std::printf("-- glb assembly graph --\n");
    Sample s = makeSample("cylinder");
    const std::string path = tmpPath("weft_io_assembly.glb");

    // A real assembly is mirrored as glTF nodes: the grouping root keeps
    // its children, the leaf carries the mesh, and out-of-range child
    // indices are dropped rather than emitted.
    weft::AssemblyNode root;
    root.name = "root";
    root.children = {1, 99};
    weft::AssemblyNode leaf;
    leaf.name = "body";
    leaf.solidId = 1;
    s.model.assembly = {root, leaf};
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    std::string json = readGlb(path).json;
    CHECK(json.find("\"scenes\":[{\"nodes\":[0]}]") != std::string::npos);
    CHECK(json.find("\"children\":[1]") != std::string::npos);
    CHECK(json.find("\"name\":\"root\"") != std::string::npos);
    CHECK_EQ(countOccurrences(json, "\"mesh\":"), 1u);

    // A body the assembly never references must still reach the scene: it
    // becomes an extra root node instead of being dropped.
    s.model.assembly[1].solidId = 7;  // no such body
    weft::writeGlb(s.mesh, path, &s.model, &s.analysis.solidFaces);
    json = readGlb(path).json;
    CHECK(json.find("\"scenes\":[{\"nodes\":[0,2]}]") != std::string::npos);
    CHECK_EQ(countOccurrences(json, "\"mesh\":"), 1u);
}

// ------------------------------------------------------------ FBX exporter ---

// Minimal binary-FBX record walker: collects (name -> array payload) for the
// nodes the writer emits.
struct FbxDoc {
    std::map<std::string, std::vector<double>> doubles;
    std::map<std::string, std::vector<int32_t>> ints;
    std::vector<std::string> names;
};

void fbxWalk(const std::vector<uint8_t>& b, size_t pos, size_t end, FbxDoc& doc) {
    while (pos + 13 <= end) {
        const uint32_t endOffset = u32At(b, pos);
        if (endOffset == 0) return;  // 13-byte null record closes the list
        const uint32_t numProps = u32At(b, pos + 4);
        const uint32_t propLen = u32At(b, pos + 8);
        const uint8_t nameLen = b[pos + 12];
        const std::string name((const char*)b.data() + pos + 13, nameLen);
        doc.names.push_back(name);
        size_t p = pos + 13 + nameLen;
        const size_t propsEnd = p + propLen;
        for (uint32_t i = 0; i < numProps && p < propsEnd; ++i) {
            const char type = (char)b[p++];
            if (type == 'L') {
                p += 8;
            } else if (type == 'I') {
                p += 4;
            } else if (type == 'S') {
                p += 4 + u32At(b, p);
            } else if (type == 'd' || type == 'i') {
                const uint32_t n = u32At(b, p);
                const uint32_t bytes = u32At(b, p + 8);
                const size_t data = p + 12;
                if (type == 'd') {
                    std::vector<double> vals(n);
                    if (n) std::memcpy(vals.data(), b.data() + data, n * 8);
                    doc.doubles[name] = std::move(vals);
                } else {
                    std::vector<int32_t> vals(n);
                    if (n) std::memcpy(vals.data(), b.data() + data, n * 4);
                    doc.ints[name] = std::move(vals);
                }
                p = data + bytes;
            } else {
                return;  // unexpected property type: stop rather than guess
            }
        }
        if (propsEnd < endOffset) fbxWalk(b, propsEnd, endOffset, doc);
        pos = endOffset;
    }
}

FbxDoc readFbx(const std::string& path, const weft::PolyMesh& mesh) {
    const std::vector<uint8_t> b = readBytes(path);
    CHECK(b.size() > 27 + 160);
    CHECK(std::memcmp(b.data(), "Kaydara FBX Binary  ", 20) == 0);
    CHECK_EQ(u32At(b, 23), 7400u);  // version, after the 3 magic bytes
    // Footer: the 16-byte magic closes the file and the version repeats.
    static const uint8_t footMagic[16] = {0xf8, 0x5a, 0x8c, 0x6a, 0xde, 0xf5,
                                          0xd9, 0x7e, 0xec, 0xe9, 0x0c, 0xe3,
                                          0x75, 0x8f, 0x29, 0x0b};
    CHECK(std::memcmp(b.data() + b.size() - 16, footMagic, 16) == 0);
    CHECK_EQ(u32At(b, b.size() - 140), 7400u);
    (void)mesh;
    FbxDoc doc;
    fbxWalk(b, 27, b.size(), doc);
    return doc;
}

void testFbxExport() {
    std::printf("-- fbx export --\n");
    Sample s = makeSample("box");
    const std::string path = tmpPath("weft_io_box.fbx");

    // Defaults are Y-up, unit scale, polygons preserved.
    weft::writeFbx(s.mesh, path);
    FbxDoc doc = readFbx(path, s.mesh);
    CHECK(std::find(doc.names.begin(), doc.names.end(), "FBXHeaderExtension") !=
          doc.names.end());
    CHECK(std::find(doc.names.begin(), doc.names.end(), "Objects") != doc.names.end());
    CHECK(std::find(doc.names.begin(), doc.names.end(), "Connections") != doc.names.end());
    CHECK(std::find(doc.names.begin(), doc.names.end(), "Properties70") != doc.names.end());
    const std::vector<double>& verts = doc.doubles["Vertices"];
    const std::vector<int32_t>& idx = doc.ints["PolygonVertexIndex"];
    CHECK_EQ(verts.size(), s.mesh.vertexCount() * 3);
    size_t corners = 0;
    for (const auto& poly : s.mesh.polygons) {
        if (poly.size() >= 3) corners += poly.size();
    }
    CHECK_EQ(idx.size(), corners);
    // Exactly one negated end marker per polygon, and it is the last index.
    size_t markers = 0;
    for (int32_t v : idx) {
        if (v < 0) ++markers;
    }
    CHECK_EQ(markers, s.mesh.polygonCount());
    CHECK(!idx.empty() && idx.back() < 0);
    for (size_t i = 0; i < idx.size(); ++i) {
        const int32_t v = idx[i] < 0 ? ~idx[i] : idx[i];
        CHECK(v >= 0 && v < (int32_t)s.mesh.vertexCount());
    }
    // Y-up default: (x, y, z) -> (x, z, -y).
    for (size_t i = 0; i < s.mesh.vertexCount(); ++i) {
        CHECK(nearly(verts[i * 3], s.mesh.vertices[i][0]));
        CHECK(nearly(verts[i * 3 + 1], s.mesh.vertices[i][2]));
        CHECK(nearly(verts[i * 3 + 2], -s.mesh.vertices[i][1]));
    }

    // Triangulated CAD space with a unit scale factor.
    weft::FbxExportOptions opts;
    opts.triangulate = true;
    opts.yUp = false;
    opts.scale = 0.001;
    weft::writeFbx(s.mesh, path, opts);
    FbxDoc tri = readFbx(path, s.mesh);
    const std::vector<int32_t>& triIdx = tri.ints["PolygonVertexIndex"];
    CHECK_EQ(triIdx.size(), 3 * triangleCount(s.mesh));
    for (size_t i = 0; i < triIdx.size(); ++i) {
        CHECK((triIdx[i] < 0) == (i % 3 == 2));  // every third index closes a tri
    }
    const std::vector<double>& scaled = tri.doubles["Vertices"];
    for (size_t i = 0; i < s.mesh.vertexCount(); ++i) {
        CHECK(nearly(scaled[i * 3], s.mesh.vertices[i][0] * 0.001));
        CHECK(nearly(scaled[i * 3 + 1], s.mesh.vertices[i][1] * 0.001));
        CHECK(nearly(scaled[i * 3 + 2], s.mesh.vertices[i][2] * 0.001));
    }

    // An empty mesh is a valid (empty) FBX, not a crash.
    weft::writeFbx(weft::PolyMesh{}, path);
    FbxDoc emptyDoc = readFbx(path, weft::PolyMesh{});
    CHECK(emptyDoc.doubles["Vertices"].empty());
}

// ------------------------------------------------------- mesh writer paths ---

void testStlWriter() {
    std::printf("-- stl writer --\n");
    using weft::io::Format;
    weft::io::System sys;
    weft::io::bootstrapIo(sys);
    Sample s = makeSample("cylinder");
    weft::io::WriteInput in;
    in.mesh = &s.mesh;
    in.model = &s.model;
    in.solidFaces = &s.analysis.solidFaces;
    const size_t tris = triangleCount(s.mesh);

    weft::io::ParamGroup params =
        sys.findFactoryWriter(Format::Stl)->createParams(Format::Stl);
    const std::string binPath = tmpPath("weft_io_cylinder.stl");
    weft::io::exportFile(sys, Format::Stl, in, binPath, &params);
    std::vector<uint8_t> bin = readBytes(binPath);
    CHECK_EQ(bin.size(), 84 + 50 * tris);
    CHECK_EQ(u32At(bin, 80), (uint32_t)tris);
    CHECK(std::memcmp(bin.data(), "weft binary STL", 15) == 0);
    // The written file must round-trip through the binary-STL probe.
    CHECK(sys.probeFormat(binPath) == Format::Stl);

    params.set("ascii", "true");
    const std::string asciiPath = tmpPath("weft_io_cylinder_ascii.stl");
    weft::io::exportFile(sys, Format::Stl, in, asciiPath, &params);
    std::ifstream ascii(asciiPath);
    CHECK(ascii.good());
    size_t facets = 0, loops = 0, vertices = 0;
    std::string first;
    for (std::string line; std::getline(ascii, line);) {
        if (first.empty()) first = line;
        if (line.find("facet normal") != std::string::npos) ++facets;
        if (line.find("outer loop") != std::string::npos) ++loops;
        if (line.find("vertex ") != std::string::npos) ++vertices;
    }
    CHECK(first == "solid weft");
    CHECK_EQ(facets, tris);
    CHECK_EQ(loops, tris);
    CHECK_EQ(vertices, 3 * tris);
    CHECK(sys.probeFormat(asciiPath) == Format::Stl);
}

void testMeshWritersThroughRegistry() {
    std::printf("-- mesh writers through the registry --\n");
    using weft::io::Format;
    weft::io::System sys;
    weft::io::bootstrapIo(sys);
    Sample s = makeSample("cylinder");
    s.model.lengthUnitMm = 25.4;  // an inch-unit source
    weft::io::WriteInput in;
    in.mesh = &s.mesh;
    in.model = &s.model;
    in.solidFaces = &s.analysis.solidFaces;

    // glTF is metres: leaving `scale` at its default folds in the source's
    // declared unit instead of writing raw millimetre numbers.
    weft::io::ParamGroup gltf =
        sys.findFactoryWriter(Format::Gltf)->createParams(Format::Gltf);
    const std::string glbPath = tmpPath("weft_io_registry.glb");
    weft::io::exportFile(sys, Format::Gltf, in, glbPath, &gltf);
    Glb glb = readGlb(glbPath);
    const size_t verts = (size_t)intsAfterKey(glb.json, "\"count\":")[0];
    Bounds mb = meshBounds(s.mesh);
    Bounds gb = glbPositionBounds(glb, verts);
    CHECK(nearly(gb.hi[0], mb.hi[0] * 0.001 * 25.4));
    // An explicit scale is the caller's number and is used as given.
    gltf.set("scale", "1");
    weft::io::exportFile(sys, Format::Gltf, in, glbPath, &gltf);
    Glb raw = readGlb(glbPath);
    CHECK(nearly(glbPositionBounds(raw, verts).hi[0], mb.hi[0]));

    // OBJ and FBX go through their own adapters.
    weft::io::ParamGroup objParams =
        sys.findFactoryWriter(Format::Obj)->createParams(Format::Obj);
    objParams.set("triangulate", "true");
    const std::string objPath = tmpPath("weft_io_registry.obj");
    weft::io::exportFile(sys, Format::Obj, in, objPath, &objParams);
    std::ifstream obj(objPath);
    CHECK(obj.good());
    size_t faceLines = 0, groups = 0;
    for (std::string line; std::getline(obj, line);) {
        if (line.rfind("f ", 0) == 0) ++faceLines;
        if (line.rfind("g face_", 0) == 0) ++groups;
    }
    CHECK_EQ(faceLines, triangleCount(s.mesh));
    CHECK_EQ(groups, (size_t)s.model.faceCount());

    weft::io::ParamGroup fbxParams =
        sys.findFactoryWriter(Format::Fbx)->createParams(Format::Fbx);
    const std::string fbxPath = tmpPath("weft_io_registry.fbx");
    weft::io::exportFile(sys, Format::Fbx, in, fbxPath, &fbxParams);
    CHECK_EQ(readFbx(fbxPath, s.mesh).doubles["Vertices"].size(),
             s.mesh.vertexCount() * 3);
}

// --------------------------------------------- B-rep writers and readers ---

void testBRepWriteReadRoundTrip() {
    std::printf("-- b-rep writers and readers --\n");
    using weft::io::Format;
    weft::io::System sys;
    weft::io::bootstrapIo(sys);
    const std::string stepIn = tmpPath("weft_io_roundtrip_in.step");
    weft::writeStep(weft::makeFixture("box"), stepIn);
    const weft::Model source = weft::loadStep(stepIn);
    CHECK_EQ(source.faceCount(), 6);
    weft::io::WriteInput in;
    in.model = &source;

    // BREP: geometry only, no metadata, so the shape must survive exactly.
    const std::string brepPath = tmpPath("weft_io_roundtrip.brep");
    weft::io::exportFile(sys, Format::Brep, in, brepPath);
    CHECK(sys.probeFormat(brepPath) == Format::Brep);
    const weft::Model brep = weft::io::importFile(sys, brepPath);
    CHECK_EQ(brep.faceCount(), 6);
    CHECK_EQ(brep.edgeCount(), source.edgeCount());

    // STEP: non-default schema and unit knobs still produce a readable file.
    weft::io::ParamGroup stepParams =
        sys.findFactoryWriter(Format::Step)->createParams(Format::Step);
    stepParams.set("schema", "ap242");
    stepParams.set("unit", "m");
    const std::string stepPath = tmpPath("weft_io_roundtrip_out.step");
    weft::io::exportFile(sys, Format::Step, in, stepPath, &stepParams);
    CHECK(sys.probeFormat(stepPath) == Format::Step);
    CHECK_EQ(weft::io::importFile(sys, stepPath).faceCount(), 6);

    // IGES: both B-rep modes, read back through the IGES reader.
    weft::io::ParamGroup igesParams =
        sys.findFactoryWriter(Format::Iges)->createParams(Format::Iges);
    igesParams.set("brepMode", "brep");
    const std::string igesPath = tmpPath("weft_io_roundtrip.iges");
    weft::io::exportFile(sys, Format::Iges, in, igesPath, &igesParams);
    CHECK(sys.probeFormat(igesPath) == Format::Iges);
    const weft::Model iges = weft::io::importFile(sys, igesPath);
    CHECK_EQ(iges.faceCount(), 6);

    // A reader handed a file of the wrong format fails at readFile rather
    // than producing a bogus model.
    std::unique_ptr<weft::io::Reader> brepReader = sys.createReader(Format::Brep);
    CHECK(brepReader != nullptr);
    CHECK(!brepReader->readFile(igesPath));
    CHECK(throwsRuntime([&] { (void)brepReader->transfer(); }));
    std::unique_ptr<weft::io::Reader> igesReader = sys.createReader(Format::Iges);
    CHECK(!igesReader->readFile(brepPath));
}

void testTessellateAndConvert() {
    std::printf("-- tessellate and convert --\n");
    using weft::io::Format;
    weft::io::System sys;
    weft::io::bootstrapIo(sys);
    const std::string stepPath = tmpPath("weft_io_convert.step");
    weft::writeStep(weft::makeFixture("box"), stepPath);
    const weft::Model model = weft::loadStep(stepPath);

    // Default tessellation: triangles only, every one attributed to a face.
    const weft::PolyMesh mesh = weft::io::tessellate(model);
    CHECK(!mesh.vertices.empty());
    CHECK(!mesh.polygons.empty());
    CHECK_EQ(mesh.polygonFaceId.size(), mesh.polygonCount());
    CHECK_EQ(mesh.anchors.size(), mesh.vertexCount());
    CHECK_EQ(mesh.countTris(), mesh.polygonCount());
    for (int fid : mesh.polygonFaceId) CHECK(fid >= 1 && fid <= model.faceCount());
    CHECK(weft::io::tessellate(weft::Model{}).polygons.empty());

    // convertFile drives import -> tessellate -> writer for mesh targets and
    // serializes the shape directly for B-rep targets.
    const std::string glbPath = tmpPath("weft_io_convert.glb");
    weft::io::convertFile(sys, stepPath, glbPath);
    CHECK(sys.probeFormat(glbPath) == Format::Gltf);
    CHECK_EQ(countOccurrences(readGlb(glbPath).json, "\"mesh\":"), 1u);

    const std::string stlPath = tmpPath("weft_io_convert.stl");
    weft::io::convertFile(sys, stepPath, stlPath);
    CHECK_EQ(u32At(readBytes(stlPath), 80), (uint32_t)mesh.polygonCount());

    const std::string brepPath = tmpPath("weft_io_convert.brep");
    weft::io::convertFile(sys, stepPath, brepPath);
    CHECK_EQ(weft::io::importFile(sys, brepPath).faceCount(), 6);

    // Explicit params override the writer's advertised defaults.
    weft::io::ParamGroup ascii =
        sys.findFactoryWriter(Format::Stl)->createParams(Format::Stl);
    ascii.set("ascii", "true");
    const std::string asciiPath = tmpPath("weft_io_convert_ascii.stl");
    weft::io::convertFile(sys, stepPath, asciiPath, &ascii);
    std::ifstream f(asciiPath);
    std::string firstLine;
    std::getline(f, firstLine);
    CHECK(firstLine == "solid weft");
}

// ------------------------------------------------- viewport edge sampling ---

void testEdgeSampling() {
    std::printf("-- viewport edge sampling --\n");
    const std::string stepPath = tmpPath("weft_io_viz.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    const weft::Model model = weft::loadStep(stepPath);

    std::vector<weft::EdgePolyline> lines = weft::sampleEdges(model, 8);
    CHECK(!lines.empty());
    CHECK(lines.size() <= (size_t)model.edgeCount());  // degenerate edges skipped
    for (const auto& line : lines) {
        CHECK(line.edgeId >= 1 && line.edgeId <= model.edgeCount());
        CHECK_EQ(line.points.size(), 9u);  // segments + 1
    }
    // Ends land on the curve, so the two circles' samples stay on radius 10.
    const weft::Analysis analysis = weft::analyze(model);
    for (const auto& line : lines) {
        for (const auto& p : line.points) {
            CHECK(std::sqrt(p[0] * p[0] + p[1] * p[1]) <= 10.0 + 1e-6);
        }
    }

    // A solved subdivision count overrides the global one per edge, so the
    // overlay's chords coincide with the generated mesh.
    const int eid = lines.front().edgeId;
    std::map<int, int> perEdge{{eid, 20}, {9999, 5}};
    std::vector<weft::EdgePolyline> pinned = weft::sampleEdges(model, 8, perEdge);
    CHECK_EQ(pinned.size(), lines.size());
    for (const auto& line : pinned) {
        CHECK_EQ(line.points.size(), line.edgeId == eid ? 21u : 9u);
    }
    // Nonsense counts degrade to a single segment instead of dividing by zero.
    for (const auto& line : weft::sampleEdges(model, 0)) {
        CHECK_EQ(line.points.size(), 2u);
    }
    std::map<int, int> zero{{eid, 0}};
    for (const auto& line : weft::sampleEdges(model, 3, zero)) {
        CHECK_EQ(line.points.size(), 4u);
    }
    CHECK(weft::sampleEdges(weft::Model{}, 4).empty());
    (void)analysis;
}

// ----------------------------------------------------- generation debug log ---

void testGenerateDebugLog() {
    std::printf("-- generation debug log --\n");
    const std::string logPath = tmpPath("weft_io_debug.log");
    const std::string stepPath = tmpPath("weft_io_debug.step");
    weft::writeStep(weft::makeFixture("cylinder"), stepPath);
    const weft::Model model = weft::loadStep(stepPath);
    const weft::Analysis analysis = weft::analyze(model);
    weft::GenerationSettings gs;
    gs.defaults.radial = 8;
    gs.defaults.axial = 1;

    std::FILE* log = std::fopen(logPath.c_str(), "w");
    CHECK(log != nullptr);
    weft::setGenerateDebugLog(log);
    weft::PolyMesh traced = weft::generate(model, analysis, gs);
    weft::setGenerateDebugLog(nullptr);
    std::fclose(log);
    CHECK(!traced.polygons.empty());

    std::ifstream logFile(logPath);
    std::string body((std::istreambuf_iterator<char>(logFile)),
                     std::istreambuf_iterator<char>());
    CHECK(!body.empty());
    CHECK(body.find("[core]") != std::string::npos);

    // With the log detached, generation must not append anything more.
    const uintmax_t size = std::filesystem::file_size(logPath);
    weft::PolyMesh silent = weft::generate(model, analysis, gs);
    CHECK_EQ(silent.polygonCount(), traced.polygonCount());
    CHECK_EQ(std::filesystem::file_size(logPath), size);
}

}  // namespace

int main() {
#define RUN(fn)   \
    do {          \
        fn();     \
    } while (0)
    RUN(testFormatMetadata);
    RUN(testParamGroup);
    RUN(testFormatProbes);
    RUN(testWriterRegistry);
    RUN(testGlbExport);
    RUN(testGlbColorsAndNames);
    RUN(testGlbAssemblyGraph);
    RUN(testFbxExport);
    RUN(testStlWriter);
    RUN(testMeshWritersThroughRegistry);
    RUN(testBRepWriteReadRoundTrip);
    RUN(testTessellateAndConvert);
    RUN(testEdgeSampling);
    RUN(testGenerateDebugLog);
    if (failures) {
        std::printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("\nall checks passed\n");
    return 0;
}
