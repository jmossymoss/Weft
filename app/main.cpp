// weft_app — interactive shell over weft_core.
//
// Phase-2/6 viewport (plan §6): load STEP (or a built-in fixture), see the
// B-rep with feature colouring, click faces to select, drag density controls
// and watch the topology regenerate live. Orbit/pan/zoom like Blender.
//
//   weft_app [model.step] [--fixture demo] [--screenshot out.png]

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"
#include "weft/viz.hpp"

#include <GLFW/glfw3.h>
#include <GL/glext.h>

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Minimal GL 3.3 loader: core entry points fetched through GLFW.

#define WEFT_GL_FUNCS(X)                                    \
    X(PFNGLCREATESHADERPROC, glCreateShader)                \
    X(PFNGLSHADERSOURCEPROC, glShaderSource)                \
    X(PFNGLCOMPILESHADERPROC, glCompileShader)              \
    X(PFNGLGETSHADERIVPROC, glGetShaderiv)                  \
    X(PFNGLGETSHADERINFOLOGPROC, glGetShaderInfoLog)        \
    X(PFNGLCREATEPROGRAMPROC, glCreateProgram)              \
    X(PFNGLATTACHSHADERPROC, glAttachShader)                \
    X(PFNGLLINKPROGRAMPROC, glLinkProgram)                  \
    X(PFNGLGETPROGRAMIVPROC, glGetProgramiv)                \
    X(PFNGLGETPROGRAMINFOLOGPROC, glGetProgramInfoLog)      \
    X(PFNGLDELETESHADERPROC, glDeleteShader)                \
    X(PFNGLUSEPROGRAMPROC, glUseProgram)                    \
    X(PFNGLGETUNIFORMLOCATIONPROC, glGetUniformLocation)    \
    X(PFNGLUNIFORMMATRIX4FVPROC, glUniformMatrix4fv)        \
    X(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays)          \
    X(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray)          \
    X(PFNGLGENBUFFERSPROC, glGenBuffers)                    \
    X(PFNGLBINDBUFFERPROC, glBindBuffer)                    \
    X(PFNGLBUFFERDATAPROC, glBufferData)                    \
    X(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer)  \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray)

#define WEFT_GL_DECLARE(type, name) static type name = nullptr;
WEFT_GL_FUNCS(WEFT_GL_DECLARE)

static void loadGl() {
#define WEFT_GL_LOAD(type, name) \
    name = reinterpret_cast<type>(glfwGetProcAddress(#name));
    WEFT_GL_FUNCS(WEFT_GL_LOAD)
}

// ---------------------------------------------------------------------------
// Tiny column-major mat4.

struct Mat4 {
    float m[16];
};

static Mat4 matMul(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c) {
        for (int row = 0; row < 4; ++row) {
            float s = 0;
            for (int k = 0; k < 4; ++k) s += a.m[k * 4 + row] * b.m[c * 4 + k];
            r.m[c * 4 + row] = s;
        }
    }
    return r;
}

static Mat4 matPerspective(float fovyDeg, float aspect, float zn, float zf) {
    float t = std::tan(fovyDeg * float(M_PI) / 360.0f);
    Mat4 r{};
    r.m[0] = 1.0f / (aspect * t);
    r.m[5] = 1.0f / t;
    r.m[10] = -(zf + zn) / (zf - zn);
    r.m[11] = -1.0f;
    r.m[14] = -(2 * zf * zn) / (zf - zn);
    return r;
}

struct Vec3 {
    float x, y, z;
};
static Vec3 sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
static Vec3 norm(Vec3 v) {
    float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return l > 0 ? Vec3{v.x / l, v.y / l, v.z / l} : v;
}

static Mat4 matLookAt(Vec3 eye, Vec3 at, Vec3 up) {
    Vec3 f = norm(sub(at, eye));
    Vec3 s = norm(cross(f, up));
    Vec3 u = cross(s, f);
    Mat4 r{};
    r.m[0] = s.x; r.m[4] = s.y; r.m[8] = s.z;
    r.m[1] = u.x; r.m[5] = u.y; r.m[9] = u.z;
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
    r.m[12] = -(s.x * eye.x + s.y * eye.y + s.z * eye.z);
    r.m[13] = -(u.x * eye.x + u.y * eye.y + u.z * eye.z);
    r.m[14] = f.x * eye.x + f.y * eye.y + f.z * eye.z;
    r.m[15] = 1.0f;
    return r;
}

// ---------------------------------------------------------------------------
// Shaders.

static GLuint compile(GLenum type, const char* src) {
    GLuint sh = glCreateShader(type);
    glShaderSource(sh, 1, &src, nullptr);
    glCompileShader(sh);
    GLint ok = 0;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(sh, sizeof log, nullptr, log);
        std::fprintf(stderr, "shader error: %s\n", log);
    }
    return sh;
}

static GLuint makeProgram(const char* vs, const char* fs) {
    GLuint p = glCreateProgram();
    GLuint v = compile(GL_VERTEX_SHADER, vs);
    GLuint f = compile(GL_FRAGMENT_SHADER, fs);
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

static const char* kLitVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uMVP;
uniform mat4 uMV;
out vec3 vPosVS;
out vec3 vColor;
void main() {
    vPosVS = (uMV * vec4(aPos, 1.0)).xyz;
    vColor = aColor;
    gl_Position = uMVP * vec4(aPos, 1.0);
})";

static const char* kLitFS = R"(#version 330 core
in vec3 vPosVS;
in vec3 vColor;
out vec4 frag;
void main() {
    vec3 n = normalize(cross(dFdx(vPosVS), dFdy(vPosVS)));
    vec3 l = normalize(-vPosVS);
    float diff = abs(dot(n, l));
    float rim = pow(1.0 - diff, 2.0) * 0.12;
    frag = vec4(vColor * (0.28 + 0.68 * diff) + vec3(rim), 1.0);
})";

static const char* kFlatVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aColor;
uniform mat4 uMVP;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uMVP * vec4(aPos, 1.0);
})";

static const char* kFlatFS = R"(#version 330 core
in vec3 vColor;
out vec4 frag;
void main() { frag = vec4(vColor, 1.0); })";

// ---------------------------------------------------------------------------
// GPU vertex buffers (pos3 + col3).

struct Buffer {
    GLuint vao = 0, vbo = 0;
    int count = 0;

    void upload(const std::vector<float>& data) {
        if (!vao) {
            glGenVertexArrays(1, &vao);
            glGenBuffers(1, &vbo);
        }
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), data.data(),
                     GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glBindVertexArray(0);
        count = static_cast<int>(data.size() / 6);
    }
};

// ---------------------------------------------------------------------------
// Application state.

struct Camera {
    Vec3 target{0, 0, 0};
    float yaw = 0.9f, pitch = 0.5f, dist = 80.0f;

    Vec3 eye() const {
        return {target.x + dist * std::cos(pitch) * std::cos(yaw),
                target.y + dist * std::cos(pitch) * std::sin(yaw),
                target.z + dist * std::sin(pitch)};
    }
};

enum class Mode { Idle, LoopCut };

struct App {
    // Document.
    std::string sourcePath;
    std::string status = "load a STEP file or a fixture";
    bool hasModel = false;
    weft::Model model;
    weft::Analysis analysis;
    weft::Recipe recipe;
    weft::PolyMesh mesh;
    weft::GenerationReport report;
    std::vector<weft::EdgePolyline> brepEdges;

    int selectedFace = 0;
    bool dirty = false;  // regenerate this frame

    // Keyboard-centric editing state.
    Mode mode = Mode::Idle;
    std::string numberEntry;   // typed digits, Enter applies to density
    weft::ManualOp hoverOp;    // loop-cut candidate under the cursor
    bool hoverValid = false;

    // Display toggles.
    bool showFill = true;
    bool showWire = true;
    bool showBrepEdges = true;

    // GPU.
    Buffer fill, wire, brep, pick, preview;
    Camera cam;

    // UI buffers.
    char pathBuf[512] = "";
    char recipeBuf[512] = "session.recipe";
};

static std::array<float, 3> faceColor(const weft::FaceInfo& f, bool selected) {
    std::array<float, 3> c{0.58f, 0.60f, 0.66f};  // plane / default
    switch (f.type) {
        case weft::SurfaceType::Cylinder: c = {0.34f, 0.58f, 0.68f}; break;
        case weft::SurfaceType::Cone: c = {0.38f, 0.54f, 0.72f}; break;
        case weft::SurfaceType::Sphere: c = {0.52f, 0.47f, 0.72f}; break;
        case weft::SurfaceType::Torus: c = {0.42f, 0.62f, 0.52f}; break;
        case weft::SurfaceType::BSpline:
        case weft::SurfaceType::Bezier: c = {0.62f, 0.55f, 0.45f}; break;
        default: break;
    }
    if (f.isFillet) c = {0.80f, 0.52f, 0.28f};
    if (f.isHole) c = {0.58f, 0.38f, 0.70f};
    if (selected) {
        c[0] = 0.35f * c[0] + 0.65f * 0.98f;
        c[1] = 0.35f * c[1] + 0.65f * 0.80f;
        c[2] = 0.35f * c[2] + 0.65f * 0.25f;
    }
    return c;
}

static void rebuildBuffers(App& app) {
    const weft::PolyMesh& m = app.mesh;

    std::vector<float> fill, pick, wire;
    fill.reserve(m.polygons.size() * 18);
    auto push = [](std::vector<float>& v, const std::array<double, 3>& p,
                   const std::array<float, 3>& c) {
        v.push_back(float(p[0]));
        v.push_back(float(p[1]));
        v.push_back(float(p[2]));
        v.push_back(c[0]);
        v.push_back(c[1]);
        v.push_back(c[2]);
    };

    for (size_t i = 0; i < m.polygons.size(); ++i) {
        int fid = m.polygonFaceId[i];
        const weft::FaceInfo& info = app.analysis.faces[fid - 1];
        std::array<float, 3> col = faceColor(info, fid == app.selectedFace);
        std::array<float, 3> id{float(fid & 255) / 255.0f,
                                float((fid >> 8) & 255) / 255.0f,
                                170.0f / 255.0f};
        const auto& poly = m.polygons[i];
        for (size_t k = 1; k + 1 < poly.size(); ++k) {  // fan triangulation
            push(fill, m.vertices[poly[0]], col);
            push(fill, m.vertices[poly[k]], col);
            push(fill, m.vertices[poly[k + 1]], col);
            push(pick, m.vertices[poly[0]], id);
            push(pick, m.vertices[poly[k]], id);
            push(pick, m.vertices[poly[k + 1]], id);
        }
        std::array<float, 3> wc{0.10f, 0.11f, 0.13f};
        for (size_t k = 0; k < poly.size(); ++k) {
            push(wire, m.vertices[poly[k]], wc);
            push(wire, m.vertices[poly[(k + 1) % poly.size()]], wc);
        }
    }
    app.fill.upload(fill);
    app.pick.upload(pick);
    app.wire.upload(wire);

    std::vector<float> brep;
    for (const weft::EdgePolyline& e : app.brepEdges) {
        const weft::EdgeInfo& info = app.analysis.edges[e.edgeId - 1];
        std::array<float, 3> c{0.55f, 0.55f, 0.55f};  // boundary/seam
        switch (info.convexity) {
            case weft::EdgeConvexity::Convex: c = {0.95f, 0.62f, 0.18f}; break;
            case weft::EdgeConvexity::Concave: c = {0.25f, 0.55f, 0.95f}; break;
            case weft::EdgeConvexity::Smooth: c = {0.30f, 0.78f, 0.42f}; break;
            default: break;
        }
        for (size_t i = 0; i + 1 < e.points.size(); ++i) {
            push(brep, e.points[i], c);
            push(brep, e.points[i + 1], c);
        }
    }
    app.brep.upload(brep);
}

static void regenerate(App& app) {
    if (!app.hasModel) return;
    app.report = {};
    app.mesh = weft::generate(app.model, app.analysis, app.recipe.settings,
                              &app.report);
    weft::applyOps(app.mesh, app.model, app.recipe.ops);
    rebuildBuffers(app);
    app.dirty = false;
}

static void frameModel(App& app) {
    if (app.mesh.vertices.empty()) return;
    std::array<double, 3> lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    for (const auto& v : app.mesh.vertices) {
        for (int i = 0; i < 3; ++i) {
            lo[i] = std::min(lo[i], v[i]);
            hi[i] = std::max(hi[i], v[i]);
        }
    }
    app.cam.target = {float(lo[0] + hi[0]) * 0.5f, float(lo[1] + hi[1]) * 0.5f,
                      float(lo[2] + hi[2]) * 0.5f};
    double dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
    app.cam.dist = 1.9f * float(std::sqrt(dx * dx + dy * dy + dz * dz) + 1.0);
}

static void loadModel(App& app, const std::string& path) {
    try {
        app.model = weft::loadStep(path);
        app.analysis = weft::analyze(app.model);
        app.brepEdges = weft::sampleEdges(app.model, 28);
        app.sourcePath = path;
        app.hasModel = true;
        app.selectedFace = 0;
        app.recipe = {};
        regenerate(app);
        frameModel(app);
        app.status = path + ": " + std::to_string(app.model.faceCount()) +
                     " faces, " + std::to_string(app.model.edgeCount()) +
                     " edges";
    } catch (const std::exception& e) {
        app.status = std::string("load failed: ") + e.what();
    }
}

static void loadFixture(App& app, const std::string& name) {
    const char* tmp = std::getenv("TMPDIR");
    std::string path = std::string(tmp ? tmp : "/tmp") + "/weft_fixture_" +
                       name + ".step";
    try {
        weft::writeStep(weft::makeFixture(name), path);
        loadModel(app, path);
        app.status = "fixture: " + name;
    } catch (const std::exception& e) {
        app.status = std::string("fixture failed: ") + e.what();
    }
}

// ---------------------------------------------------------------------------
// Keyboard editing helpers.

// The density field the bracket keys / typed numbers drive, chosen by what
// kind of face is selected (radial for revolved faces, grid for planar...).
static int* primaryDensity(App& app, weft::FaceMeshSettings& s, bool secondary) {
    weft::MesherKind kind = weft::MesherKind::RevolutionGrid;
    if (app.selectedFace > 0) {
        auto it = app.report.faceMesher.find(app.selectedFace);
        if (it != app.report.faceMesher.end()) kind = it->second;
    }
    switch (kind) {
        case weft::MesherKind::RevolutionGrid:
        case weft::MesherKind::DiskCap:
            return secondary ? &s.axial : &s.radial;
        case weft::MesherKind::RingJunction:
            return secondary ? &s.junctionRings : &s.gridU;
        default:
            return secondary ? &s.gridV : &s.gridU;
    }
}

// Settings block keyboard edits act on: the selected face's override
// (created on first use), or the defaults when nothing is selected.
static weft::FaceMeshSettings& editTarget(App& app) {
    if (app.selectedFace > 0) {
        auto it = app.recipe.settings.perFace.find(app.selectedFace);
        if (it == app.recipe.settings.perFace.end()) {
            it = app.recipe.settings.perFace
                     .emplace(app.selectedFace, app.recipe.settings.defaults)
                     .first;
        }
        return it->second;
    }
    return app.recipe.settings.defaults;
}

static void projectPoint(const Mat4& mvp, const std::array<double, 3>& p,
                         int fbw, int fbh, float out[3]) {
    float x = float(p[0]), y = float(p[1]), z = float(p[2]);
    float cx = mvp.m[0] * x + mvp.m[4] * y + mvp.m[8] * z + mvp.m[12];
    float cy = mvp.m[1] * x + mvp.m[5] * y + mvp.m[9] * z + mvp.m[13];
    float cw = mvp.m[3] * x + mvp.m[7] * y + mvp.m[11] * z + mvp.m[15];
    out[2] = cw;
    if (cw <= 1e-6f) return;
    out[0] = (cx / cw * 0.5f + 0.5f) * fbw;
    out[1] = (1.0f - (cy / cw * 0.5f + 0.5f)) * fbh;
}

// Loop-cut hover: nearest same-face mesh edge under the cursor, split
// fraction from the cursor's position along it. The preview is computed by
// actually running the op on a scratch copy — what you see IS the replay.
static void updateLoopCutHover(App& app, const Mat4& mvp, double mx, double my,
                               int fbw, int fbh) {
    app.hoverValid = false;
    app.preview.count = 0;
    if (!app.hasModel) return;

    const weft::PolyMesh& m = app.mesh;
    double bestDist = 26.0;  // px
    uint32_t bestA = 0, bestB = 0;
    double bestT = 0.5;
    for (size_t p = 0; p < m.polygons.size(); ++p) {
        const auto& poly = m.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
            if (a > b) continue;  // undirected once
            const weft::Anchor& aa = m.anchors[a];
            const weft::Anchor& ab = m.anchors[b];
            if (aa.faceId == 0 || aa.faceId != ab.faceId) continue;
            float pa[3] = {0, 0, -1}, pb[3] = {0, 0, -1};
            projectPoint(mvp, m.vertices[a], fbw, fbh, pa);
            projectPoint(mvp, m.vertices[b], fbw, fbh, pb);
            if (pa[2] <= 0 || pb[2] <= 0) continue;
            float ex = pb[0] - pa[0], ey = pb[1] - pa[1];
            float len2 = ex * ex + ey * ey;
            if (len2 < 1e-6f) continue;
            float t = (float(mx) - pa[0]) * ex + (float(my) - pa[1]) * ey;
            t = std::clamp(t / len2, 0.0f, 1.0f);
            float dx = float(mx) - (pa[0] + t * ex);
            float dy = float(my) - (pa[1] + t * ey);
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < bestDist) {
                bestDist = d;
                bestA = a;
                bestB = b;
                bestT = std::clamp(double(t), 0.05, 0.95);
            }
        }
    }
    if (bestA == bestB) return;

    const weft::Anchor& aa = m.anchors[bestA];
    const weft::Anchor& ab = m.anchors[bestB];
    weft::ManualOp op;
    op.faceId = aa.faceId;
    op.u = 0.5 * (aa.u + ab.u);
    op.v = 0.5 * (aa.v + ab.v);
    op.t = bestT;

    // Probe on a copy; if the split lands on the far side of the edge from
    // the cursor (the walker picked the reversed ordering), mirror t.
    auto previewSegments = [&](const weft::ManualOp& probe,
                               double* splitScreenDist) -> std::vector<float> {
        weft::PolyMesh copy = m;
        if (weft::insertLoop(copy, app.model, probe) == 0) return {};
        size_t firstNew = m.vertexCount();
        if (splitScreenDist) {
            *splitScreenDist = 1e30;
            for (size_t v = firstNew; v < copy.vertexCount(); ++v) {
                float s[3] = {0, 0, -1};
                projectPoint(mvp, copy.vertices[v], fbw, fbh, s);
                if (s[2] <= 0) continue;
                double d = std::hypot(s[0] - mx, s[1] - my);
                *splitScreenDist = std::min(*splitScreenDist, d);
            }
        }
        std::vector<float> lines;
        for (const auto& poly : copy.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                uint32_t v0 = poly[i], v1 = poly[(i + 1) % poly.size()];
                if (v0 < firstNew || v1 < firstNew || v0 > v1) continue;
                for (uint32_t v : {v0, v1}) {
                    lines.push_back(float(copy.vertices[v][0]));
                    lines.push_back(float(copy.vertices[v][1]));
                    lines.push_back(float(copy.vertices[v][2]));
                    lines.push_back(1.0f);
                    lines.push_back(0.85f);
                    lines.push_back(0.25f);
                }
            }
        }
        return lines;
    };

    double dist = 0, distFlipped = 0;
    std::vector<float> lines = previewSegments(op, &dist);
    weft::ManualOp flipped = op;
    flipped.t = 1.0 - op.t;
    std::vector<float> linesFlipped = previewSegments(flipped, &distFlipped);
    if (!linesFlipped.empty() &&
        (lines.empty() || distFlipped + 1.0 < dist)) {
        op = flipped;
        lines = std::move(linesFlipped);
    }
    if (lines.empty()) return;

    app.preview.upload(lines);
    app.hoverOp = op;
    app.hoverValid = true;
}

// ---------------------------------------------------------------------------
// Face picking: render IDs into the back buffer, read one pixel.

static int pickFace(App& app, GLuint flatProg, const Mat4& mvp, int px, int py,
                    int fbw, int fbh) {
    if (!app.hasModel || app.pick.count == 0) return 0;
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(flatProg);
    glUniformMatrix4fv(glGetUniformLocation(flatProg, "uMVP"), 1, GL_FALSE,
                       mvp.m);
    glBindVertexArray(app.pick.vao);
    glDrawArrays(GL_TRIANGLES, 0, app.pick.count);
    glBindVertexArray(0);
    glFinish();
    unsigned char rgba[4] = {0, 0, 0, 0};
    glReadPixels(px, fbh - 1 - py, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
    (void)fbw;
    if (rgba[2] != 170) return 0;
    return int(rgba[0]) + (int(rgba[1]) << 8);
}

// ---------------------------------------------------------------------------
// UI.

static void styleUi() {
    ImGui::StyleColorsDark();
    ImGuiStyle& s = ImGui::GetStyle();
    s.WindowRounding = 6.0f;
    s.FrameRounding = 4.0f;
    s.GrabRounding = 4.0f;
    s.WindowPadding = {10, 10};
    s.FramePadding = {8, 4};
    s.ItemSpacing = {8, 6};
    ImVec4* c = s.Colors;
    c[ImGuiCol_WindowBg] = {0.106f, 0.113f, 0.133f, 0.97f};
    c[ImGuiCol_TitleBgActive] = {0.14f, 0.15f, 0.18f, 1.0f};
    c[ImGuiCol_Header] = {0.20f, 0.22f, 0.27f, 1.0f};
    c[ImGuiCol_HeaderHovered] = {0.26f, 0.29f, 0.35f, 1.0f};
    c[ImGuiCol_FrameBg] = {0.16f, 0.17f, 0.20f, 1.0f};
    c[ImGuiCol_FrameBgHovered] = {0.22f, 0.24f, 0.28f, 1.0f};
    c[ImGuiCol_Button] = {0.20f, 0.22f, 0.27f, 1.0f};
    c[ImGuiCol_ButtonHovered] = {0.95f, 0.62f, 0.18f, 0.55f};
    c[ImGuiCol_ButtonActive] = {0.95f, 0.62f, 0.18f, 0.85f};
    c[ImGuiCol_SliderGrab] = {0.95f, 0.62f, 0.18f, 0.9f};
    c[ImGuiCol_CheckMark] = {0.95f, 0.62f, 0.18f, 1.0f};
}

static bool settingsEditor(weft::FaceMeshSettings& s) {
    bool ch = false;
    ch |= ImGui::DragInt("radial", &s.radial, 0.2f, 3, 256);
    ch |= ImGui::DragInt("axial", &s.axial, 0.2f, 1, 256);
    ch |= ImGui::DragInt("grid u", &s.gridU, 0.2f, 1, 256);
    ch |= ImGui::DragInt("grid v", &s.gridV, 0.2f, 1, 256);
    ch |= ImGui::DragInt("fillet loops", &s.filletLoops, 0.2f, 1, 64);
    float hold = float(s.filletHold);
    if (ImGui::SliderFloat("hold", &hold, 0.0f, 0.95f)) {
        s.filletHold = hold;
        ch = true;
    }
    ch |= ImGui::DragInt("junction rings", &s.junctionRings, 0.2f, 1, 32);
    int cap = s.cap == weft::CapStyle::Fan ? 1 : 0;
    if (ImGui::Combo("cap style", &cap, "ngon\0fan\0")) {
        s.cap = cap ? weft::CapStyle::Fan : weft::CapStyle::NGon;
        ch = true;
    }
    ch |= ImGui::Checkbox("quad-dominant fallback", &s.quadDominant);
    ch |= ImGui::Checkbox("minimal n-gon (flat panels)", &s.minimal);
    return ch;
}

// Mode indicator + typed-number + hotkey reference, floating over the
// viewport so the keyboard flow never needs the side panel.
static void drawOverlay(App& app) {
    ImGui::SetNextWindowPos({12, 12});
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("##overlay", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    if (app.mode == Mode::LoopCut) {
        ImGui::TextColored({1.0f, 0.85f, 0.25f, 1.0f}, "LOOP CUT");
        ImGui::SameLine();
        ImGui::TextDisabled("hover an edge - click commits - R/esc exits");
    } else {
        ImGui::TextDisabled(app.selectedFace > 0 ? "face #%d selected"
                                                 : "no selection",
                            app.selectedFace);
    }
    if (!app.numberEntry.empty()) {
        ImGui::TextColored({1.0f, 0.85f, 0.25f, 1.0f}, "divisions: %s_",
                           app.numberEntry.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("enter applies (shift+enter: secondary)");
    }
    ImGui::End();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({12, vp->WorkSize.y - 12}, ImGuiCond_Always,
                            {0.0f, 1.0f});
    ImGui::SetNextWindowBgAlpha(0.45f);
    ImGui::Begin("##hotkeys", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    ImGui::TextDisabled(
        "R loop cut   12<enter> set divisions   [ ] nudge (+shift: 2nd axis)\n"
        "C cap ngon/fan   T tris ok   M minimal n-gon   ctrl+Z undo op\n"
        "W wire   B feature edges   F frame   esc deselect/cancel");
    ImGui::End();
}

static void drawUi(App& app) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float width = 330.0f;
    ImGui::SetNextWindowPos({vp->WorkPos.x + vp->WorkSize.x - width,
                             vp->WorkPos.y});
    ImGui::SetNextWindowSize({width, vp->WorkSize.y});
    ImGui::Begin("weft", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar);

    ImGui::TextColored({0.95f, 0.62f, 0.18f, 1.0f}, "WEFT");
    ImGui::SameLine();
    ImGui::TextDisabled("b-rep retopology");
    ImGui::Separator();

    if (ImGui::CollapsingHeader("Model", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputTextWithHint("##path", "path/to/model.step", app.pathBuf,
                                 sizeof app.pathBuf);
        ImGui::SameLine();
        if (ImGui::Button("Load")) loadModel(app, app.pathBuf);
        ImGui::TextDisabled("fixtures:");
        const char* fixtures[] = {"cylinder", "box",  "cone", "sphere",
                                  "torus",    "fillet", "hole", "boss", "demo"};
        for (int i = 0; i < 9; ++i) {
            if (i % 3) ImGui::SameLine();
            if (ImGui::Button(fixtures[i], {96, 0})) loadFixture(app, fixtures[i]);
        }
        ImGui::TextWrapped("%s", app.status.c_str());
    }

    if (app.hasModel &&
        ImGui::CollapsingHeader("Topology", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("%zu verts   %zu polys", app.mesh.vertexCount(),
                    app.mesh.polygonCount());
        ImGui::Text("%zu quads  %zu tris  %zu n-gons", app.mesh.countQuads(),
                    app.mesh.countTris(), app.mesh.countNgons());
        ImGui::Separator();
        ImGui::TextDisabled("defaults (live)");
        if (settingsEditor(app.recipe.settings.defaults)) app.dirty = true;
    }

    if (app.hasModel &&
        ImGui::CollapsingHeader("Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (app.selectedFace <= 0) {
            ImGui::TextDisabled("click a face in the viewport");
        } else {
            const weft::FaceInfo& f = app.analysis.faces[app.selectedFace - 1];
            ImGui::Text("face #%d  %s%s%s", f.id, weft::surfaceTypeName(f.type),
                        f.isFillet ? "  [fillet]" : "",
                        f.isHole ? "  [hole]" : "");
            if (f.radius > 0) ImGui::Text("radius %.3f", f.radius);
            auto it = app.report.faceMesher.find(f.id);
            if (it != app.report.faceMesher.end()) {
                ImGui::Text("mesher: %s", weft::mesherKindName(it->second));
            }
            bool overridden =
                app.recipe.settings.perFace.count(app.selectedFace) > 0;
            if (ImGui::Checkbox("override this face", &overridden)) {
                if (overridden) {
                    app.recipe.settings.perFace[app.selectedFace] =
                        app.recipe.settings.defaults;
                } else {
                    app.recipe.settings.perFace.erase(app.selectedFace);
                }
                app.dirty = true;
            }
            if (overridden) {
                ImGui::PushID("perface");
                if (settingsEditor(
                        app.recipe.settings.perFace[app.selectedFace])) {
                    app.dirty = true;
                }
                ImGui::PopID();
            }
        }
    }

    if (app.hasModel && ImGui::CollapsingHeader("Recipe")) {
        ImGui::InputText("##recipe", app.recipeBuf, sizeof app.recipeBuf);
        if (ImGui::Button("Save recipe")) {
            try {
                weft::saveRecipe(app.recipe, app.recipeBuf);
                app.status = std::string("saved ") + app.recipeBuf;
            } catch (const std::exception& e) {
                app.status = e.what();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Load recipe")) {
            try {
                app.recipe = weft::loadRecipe(app.recipeBuf);
                app.dirty = true;
                app.status = std::string("loaded ") + app.recipeBuf;
            } catch (const std::exception& e) {
                app.status = e.what();
            }
        }
        ImGui::Text("%zu manual op(s) recorded", app.recipe.ops.size());
    }

    if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("fill", &app.showFill);
        ImGui::SameLine();
        ImGui::Checkbox("wire", &app.showWire);
        ImGui::SameLine();
        ImGui::Checkbox("feature edges", &app.showBrepEdges);
        ImGui::TextDisabled("orange convex / blue concave / green smooth");
        ImGui::TextDisabled("LMB select · RMB orbit · shift pan · wheel zoom");
        ImGui::Text("%zu manual op(s)", app.recipe.ops.size());
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------

static float gScroll = 0.0f;
static void scrollCb(GLFWwindow*, double, double dy) {
    gScroll += float(dy);
}

int main(int argc, char** argv) {
    std::string screenshotPath, startModel, startFixture = "demo";
    int startSelect = 0;
    float startYaw = 0.9f, startPitch = 0.5f;
    bool demoLoopCut = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--screenshot" && i + 1 < argc) screenshotPath = argv[++i];
        else if (a == "--fixture" && i + 1 < argc) startFixture = argv[++i];
        else if (a == "--select" && i + 1 < argc) startSelect = std::stoi(argv[++i]);
        else if (a == "--yaw" && i + 1 < argc) startYaw = std::stof(argv[++i]);
        else if (a == "--pitch" && i + 1 < argc) startPitch = std::stof(argv[++i]);
        else if (a == "--loopcut") demoLoopCut = true;  // screenshot testing
        else startModel = a;
    }

    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* window =
        glfwCreateWindow(1600, 950, "weft — b-rep retopology", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "failed to create window/GL context\n");
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    loadGl();

    glfwSetScrollCallback(window, scrollCb);  // ImGui chains it

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    styleUi();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    GLuint litProg = makeProgram(kLitVS, kLitFS);
    GLuint flatProg = makeProgram(kFlatVS, kFlatFS);

    App app;
    if (!startModel.empty()) loadModel(app, startModel);
    else loadFixture(app, startFixture);
    app.cam.yaw = startYaw;
    app.cam.pitch = startPitch;
    if (startSelect > 0 && startSelect <= app.model.faceCount()) {
        app.selectedFace = startSelect;
        rebuildBuffers(app);
    }

    double lastX = 0, lastY = 0;
    bool rotating = false, panning = false;
    double downX = 0, downY = 0;
    bool prevLmb = false;
    int frame = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGuiIO& io = ImGui::GetIO();

        int fbw, fbh;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        if (demoLoopCut) {  // scripted screenshots: cursor at viewport center
            app.mode = Mode::LoopCut;
            mx = fbw * 0.42;
            my = fbh * 0.5;
        }

        // Camera controls (Blender-ish): RMB/MMB orbit, +shift pan, wheel zoom.
        if (!io.WantCaptureMouse) {
            bool orbitBtn =
                glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ||
                glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
            if (orbitBtn && !rotating && !panning) {
                rotating = !shift;
                panning = shift;
                lastX = mx;
                lastY = my;
            }
            if (!orbitBtn) rotating = panning = false;
            if (rotating) {
                app.cam.yaw -= float(mx - lastX) * 0.008f;
                app.cam.pitch += float(my - lastY) * 0.008f;
                app.cam.pitch = std::clamp(app.cam.pitch, -1.55f, 1.55f);
            }
            if (panning) {
                float k = app.cam.dist * 0.0016f;
                Vec3 eye = app.cam.eye();
                Vec3 f = norm(sub(app.cam.target, eye));
                Vec3 r = norm(cross(f, {0, 0, 1}));
                Vec3 u = cross(r, f);
                app.cam.target.x -= (float(mx - lastX) * r.x - float(my - lastY) * u.x) * k;
                app.cam.target.y -= (float(mx - lastX) * r.y - float(my - lastY) * u.y) * k;
                app.cam.target.z -= (float(mx - lastX) * r.z - float(my - lastY) * u.z) * k;
            }
            lastX = mx;
            lastY = my;
            if (gScroll != 0.0f) {
                app.cam.dist *= std::pow(0.92f, gScroll);
                app.cam.dist = std::clamp(app.cam.dist, 0.5f, 10000.0f);
            }
        }

        // Keyboard-centric editing (plan §6: modal verbs, typed precision).
        if (!io.WantCaptureKeyboard) {
            bool shift = io.KeyShift;
            if (ImGui::IsKeyPressed(ImGuiKey_F, false)) frameModel(app);
            if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
                app.mode = app.mode == Mode::LoopCut ? Mode::Idle : Mode::LoopCut;
                app.hoverValid = false;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                if (app.mode != Mode::Idle) app.mode = Mode::Idle;
                else if (!app.numberEntry.empty()) app.numberEntry.clear();
                else if (app.selectedFace) {
                    app.selectedFace = 0;
                    rebuildBuffers(app);
                }
            }
            for (int d = 0; d <= 9; ++d) {
                if (ImGui::IsKeyPressed(ImGuiKey(ImGuiKey_0 + d), false) ||
                    ImGui::IsKeyPressed(ImGuiKey(ImGuiKey_Keypad0 + d), false)) {
                    if (app.numberEntry.size() < 4) {
                        app.numberEntry += char('0' + d);
                    }
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Backspace, false) &&
                !app.numberEntry.empty()) {
                app.numberEntry.pop_back();
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Enter, false) &&
                !app.numberEntry.empty() && app.hasModel) {
                int value = std::atoi(app.numberEntry.c_str());
                weft::FaceMeshSettings& s = editTarget(app);
                *primaryDensity(app, s, shift) = std::max(1, value);
                app.numberEntry.clear();
                app.dirty = true;
            }
            bool dec = ImGui::IsKeyPressed(ImGuiKey_LeftBracket);
            bool inc = ImGui::IsKeyPressed(ImGuiKey_RightBracket);
            if ((dec || inc) && app.hasModel) {
                weft::FaceMeshSettings& s = editTarget(app);
                int* v = primaryDensity(app, s, shift);
                *v = std::max(1, *v + (inc ? 1 : -1));
                app.dirty = true;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_C, false) && app.hasModel) {
                weft::FaceMeshSettings& s = editTarget(app);
                s.cap = s.cap == weft::CapStyle::NGon ? weft::CapStyle::Fan
                                                      : weft::CapStyle::NGon;
                app.dirty = true;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_T, false) && app.hasModel) {
                weft::FaceMeshSettings& s = editTarget(app);
                s.quadDominant = !s.quadDominant;
                app.dirty = true;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_M, false) && app.hasModel) {
                weft::FaceMeshSettings& s = editTarget(app);
                s.minimal = !s.minimal;
                app.dirty = true;
            }
            if (ImGui::IsKeyPressed(ImGuiKey_W, false)) app.showWire = !app.showWire;
            if (ImGui::IsKeyPressed(ImGuiKey_B, false)) {
                app.showBrepEdges = !app.showBrepEdges;
            }
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z, false) &&
                !app.recipe.ops.empty()) {
                app.recipe.ops.pop_back();
                app.dirty = true;
                app.status = "undid last op";
            }
        }

        bool lmb = !io.WantCaptureMouse &&
                   glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) ==
                       GLFW_PRESS;
        if (lmb && !prevLmb) { downX = mx; downY = my; }
        bool clicked = prevLmb && !lmb && std::abs(mx - downX) < 4 &&
                       std::abs(my - downY) < 4;
        prevLmb = lmb;
        gScroll = 0.0f;

        if (app.dirty) regenerate(app);

        Mat4 proj = matPerspective(42.0f, fbh > 0 ? float(fbw) / fbh : 1.6f,
                                   app.cam.dist * 0.01f, app.cam.dist * 40.0f);
        Mat4 view = matLookAt(app.cam.eye(), app.cam.target, {0, 0, 1});
        Mat4 mvp = matMul(proj, view);

        // Loop-cut hover preview follows the cursor; click commits the op
        // into the recipe (regeneration replays it — fully non-destructive).
        if (app.mode == Mode::LoopCut && !io.WantCaptureMouse) {
            updateLoopCutHover(app, mvp, mx, my, fbw, fbh);
            if (clicked && app.hoverValid) {
                app.recipe.ops.push_back(app.hoverOp);
                app.dirty = true;
                app.status = "loop cut committed (ctrl+z undoes)";
            }
        } else if (clicked && app.hasModel) {
            // Click-select: pick pass + pixel read.
            glViewport(0, 0, fbw, fbh);
            int hit = pickFace(app, flatProg, mvp, int(mx), int(my), fbw, fbh);
            app.selectedFace = hit == app.selectedFace ? 0 : hit;
            rebuildBuffers(app);
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawUi(app);
        drawOverlay(app);
        ImGui::Render();

        glViewport(0, 0, fbw, fbh);
        glClearColor(0.117f, 0.125f, 0.145f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        if (app.hasModel && app.showFill && app.fill.count) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 1.0f);
            glUseProgram(litProg);
            glUniformMatrix4fv(glGetUniformLocation(litProg, "uMVP"), 1,
                               GL_FALSE, mvp.m);
            glUniformMatrix4fv(glGetUniformLocation(litProg, "uMV"), 1,
                               GL_FALSE, view.m);
            glBindVertexArray(app.fill.vao);
            glDrawArrays(GL_TRIANGLES, 0, app.fill.count);
            glDisable(GL_POLYGON_OFFSET_FILL);
        }
        glUseProgram(flatProg);
        glUniformMatrix4fv(glGetUniformLocation(flatProg, "uMVP"), 1, GL_FALSE,
                           mvp.m);
        if (app.hasModel && app.showWire && app.wire.count) {
            glBindVertexArray(app.wire.vao);
            glDrawArrays(GL_LINES, 0, app.wire.count);
        }
        if (app.hasModel && app.showBrepEdges && app.brep.count) {
            glLineWidth(2.0f);
            glBindVertexArray(app.brep.vao);
            glDrawArrays(GL_LINES, 0, app.brep.count);
            glLineWidth(1.0f);
        }
        if (app.mode == Mode::LoopCut && app.hoverValid && app.preview.count) {
            glDisable(GL_DEPTH_TEST);
            glLineWidth(3.0f);
            glBindVertexArray(app.preview.vao);
            glDrawArrays(GL_LINES, 0, app.preview.count);
            glLineWidth(1.0f);
            glEnable(GL_DEPTH_TEST);
        }
        glBindVertexArray(0);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);

        if (!screenshotPath.empty() && ++frame >= 4) {
            std::vector<unsigned char> px(size_t(fbw) * fbh * 3);
            glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            stbi_flip_vertically_on_write(1);
            stbi_write_png(screenshotPath.c_str(), fbw, fbh, 3, px.data(),
                           fbw * 3);
            std::printf("wrote %s\n", screenshotPath.c_str());
            break;
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
