// weft_app — interactive shell over weft_core.
//
// Phase-2/6 viewport (plan §6): load STEP (or a built-in fixture), see the
// B-rep with feature colouring, click faces to select, drag density controls
// and watch the topology regenerate live. Orbit/pan/zoom like Blender.
//
//   weft_app [model.step] [--fixture demo] [--screenshot out.png]

// windows.h + commdlg.h must come FIRST: OCCT's headers include windows.h
// themselves with slimmed-down defines, and a later re-include is a no-op
// (header guard), leaving commdlg.h without the dialog types it needs.
// Full windows.h here wins the race and satisfies everyone.
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#include <dbghelp.h>
#endif

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "weft/fixture.hpp"
#include "weft/mesh.hpp"
#include "weft/meshers.hpp"
#include "weft/model.hpp"
#include "weft/recipe.hpp"
#include "weft/viz.hpp"

#include <GLFW/glfw3.h>
#include "gl_compat.hpp"

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Debug log: weft_debug.log next to the working directory, one flushed line
// per action/stage, so after a crash its tail names the culprit. A crash
// handler appends the exception/signal before the process dies.

static FILE* gDebugLog = nullptr;
static std::string gDataDir;   // per-user app data (log, imgui.ini)
static std::string gLogPath;

// %LOCALAPPDATA%\Weft on Windows, ~/.local/state/weft elsewhere — keeps
// the app's own files (debug log, UI layout) out of whatever directory
// it was launched from.
static std::string userDataDir() {
#ifdef _WIN32
    const char* base = std::getenv("LOCALAPPDATA");
    std::string dir = std::string(base && *base ? base : ".") + "\\Weft";
#else
    std::string dir;
    if (const char* x = std::getenv("XDG_STATE_HOME"); x && *x) {
        dir = std::string(x) + "/weft";
    } else {
        const char* home = std::getenv("HOME");
        dir = std::string(home && *home ? home : ".") + "/.local/state/weft";
    }
#endif
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return dir;
}

// UI scale from the monitor's content scale (Windows DPI setting). Fonts
// and style metrics rebuild when it changes (e.g. dragging the window to
// a monitor with a different scale).
static float gUiScale = 1.0f;
static float gPendingUiScale = 0.0f;

static void logLine(const char* fmt, ...) {
    if (!gDebugLog) return;
    va_list args;
    va_start(args, fmt);
    std::fprintf(gDebugLog, "[app ] ");
    std::vfprintf(gDebugLog, fmt, args);
    std::fputc('\n', gDebugLog);
    std::fflush(gDebugLog);
    va_end(args);
}

#ifdef _WIN32
static LONG WINAPI crashFilter(EXCEPTION_POINTERS* info) {
    logLine("FATAL: unhandled exception 0x%08lX at %p",
            info->ExceptionRecord->ExceptionCode,
            info->ExceptionRecord->ExceptionAddress);
    // Symbolized backtrace (needs the .pdb next to the exe). For a stack
    // overflow this runs on the reserved guarantee area set in
    // installCrashHandler.
    void* frames[48];
    USHORT n = CaptureStackBackTrace(0, 48, frames, nullptr);
    HANDLE proc = GetCurrentProcess();
    SymSetOptions(SYMOPT_DEFERRED_LOADS | SYMOPT_UNDNAME);
    SymInitialize(proc, nullptr, TRUE);
    char buf[sizeof(SYMBOL_INFO) + 256];
    for (USHORT i = 0; i < n; ++i) {
        auto* sym = reinterpret_cast<SYMBOL_INFO*>(buf);
        sym->SizeOfStruct = sizeof(SYMBOL_INFO);
        sym->MaxNameLen = 255;
        DWORD64 disp = 0;
        if (SymFromAddr(proc, DWORD64(frames[i]), &disp, sym)) {
            logLine("  #%02d %s +0x%llx", i, sym->Name,
                    (unsigned long long)disp);
        } else {
            logLine("  #%02d %p", i, frames[i]);
        }
    }
    return EXCEPTION_EXECUTE_HANDLER;
}
#else
#include <csignal>
static void crashSignal(int sig) {
    logLine("FATAL: signal %d", sig);
    std::signal(sig, SIG_DFL);
    std::raise(sig);
}
#endif

static void installCrashHandler() {
#ifdef _WIN32
    // Reserve stack for the crash filter so it can run (and symbolize)
    // even when the crash IS a stack overflow.
    ULONG guarantee = 64 * 1024;
    SetThreadStackGuarantee(&guarantee);
    SetUnhandledExceptionFilter(crashFilter);
#else
    std::signal(SIGSEGV, crashSignal);
    std::signal(SIGABRT, crashSignal);
    std::signal(SIGFPE, crashSignal);
#endif
}

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
    X(PFNGLUNIFORM1FPROC, glUniform1f)                     \
    X(PFNGLUNIFORM3FVPROC, glUniform3fv)                   \
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
uniform float uAmbient;
uniform float uDiffuse;
uniform float uRim;
void main() {
    vec3 n = normalize(cross(dFdx(vPosVS), dFdy(vPosVS)));
    vec3 l = normalize(-vPosVS);
    float diff = abs(dot(n, l));
    float rim = pow(1.0 - diff, 2.0) * uRim;
    frag = vec4(vColor * (uAmbient + uDiffuse * diff) + vec3(rim), 1.0);
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
uniform float uMix;    // 0 = per-vertex colour, 1 = uColor override
uniform vec3 uColor;
void main() { frag = vec4(mix(vColor, uColor, uMix), 1.0); })";

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

enum class Mode { Idle, LoopCut, Bridge };
enum class SelectMode { Face, Edge };

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
    weft::GenerationCache genCache;  // per-face reuse across regenerates
    std::vector<weft::EdgePolyline> brepEdges;

    // Selection: Blender-style modes. Face mode selects B-rep faces, edge
    // mode selects B-rep edges (for per-edge density pins and bridging).
    // shift+click extends; activeFace is the last-clicked face (drives the
    // panels and which mesher kind modal edits key off).
    SelectMode selectMode = SelectMode::Face;
    std::set<int> selFaces;
    std::set<int> selEdges;
    int activeFace = 0;
    bool dirty = false;  // regenerate this frame
    std::set<int> hiddenFaces;
    bool openFacePopup = false;  // context popup requested at the cursor

    // Undo: recipe snapshots, one per edit gesture (drags coalesce).
    std::vector<weft::Recipe> undoStack;
    weft::Recipe preFrame;
    bool mutatedThisFrame = false;
    bool changedLastFrame = false;

    // Keyboard-centric editing state.
    Mode mode = Mode::Idle;
    std::string numberEntry;   // typed digits, Enter applies to density
    weft::ManualOp hoverOp;    // loop-cut candidate under the cursor
    bool hoverValid = false;

    // Bridge tool: open boundary loops of the current mesh, each mapped to
    // its nearest B-rep edge (the stable id recorded in the op).
    std::vector<std::vector<uint32_t>> bLoops;
    std::vector<int> bLoopEdge;
    int hoverLoop = -1;
    int bridgeFirstEdge = 0;  // first clicked loop's edge id (0 = none yet)

    // Display / viewport preferences.
    int shadingMode = 0;  // 0 shaded+wire, 1 shaded, 2 wireframe, 3 flat+wire
    bool showFill = true;
    bool showWire = true;
    bool showBrepEdges = true;
    bool showVerts = true;  // vertices of the selected faces
    float bgColor[3] = {0.117f, 0.125f, 0.145f};
    float wireColor[3] = {0.10f, 0.11f, 0.13f};
    float vertColor[3] = {1.0f, 0.72f, 0.25f};
    float lightAmbient = 0.28f;
    float lightDiffuse = 0.68f;
    float lightRim = 0.12f;

    // Floating value HUD for modal wheel edits.
    char hudText[64] = "";
    double hudUntil = 0.0;

    // Undo gesture: coalesce a whole drag / wheel burst into one step.
    bool gestureActive = false;
    double lastMutationTime = 0.0;

    // GPU.
    Buffer fill, wire, brep, pick, preview, verts;
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

    static const weft::FaceInfo kBridgeInfo{};  // bridge strips: faceId 0
    for (size_t i = 0; i < m.polygons.size(); ++i) {
        int fid = m.polygonFaceId[i];
        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
        const weft::FaceInfo& info =
            fid > 0 ? app.analysis.faces[fid - 1] : kBridgeInfo;
        std::array<float, 3> col =
            faceColor(info, fid > 0 && app.selFaces.count(fid) > 0);
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

    // Vertices of the selected faces (drawn as points; colour comes from
    // the uniform tint, so the values here are placeholders).
    std::vector<float> verts;
    if (!app.selFaces.empty()) {
        std::set<uint32_t> seen;
        for (size_t i = 0; i < m.polygons.size(); ++i) {
            int fid = m.polygonFaceId[i];
            if (fid <= 0 || !app.selFaces.count(fid)) continue;
            if (app.hiddenFaces.count(fid)) continue;
            for (uint32_t v : m.polygons[i]) {
                if (!seen.insert(v).second) continue;
                push(verts, m.vertices[v], {1, 1, 1});
            }
        }
    }
    app.verts.upload(verts);

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
        if (app.selEdges.count(e.edgeId)) c = {1.0f, 1.0f, 1.0f};  // selected
        for (size_t i = 0; i + 1 < e.points.size(); ++i) {
            push(brep, e.points[i], c);
            push(brep, e.points[i + 1], c);
        }
    }
    app.brep.upload(brep);
}

static void regenerate(App& app) {
    if (!app.hasModel) return;
    // Never let a geometry failure take the app down: keep the previous
    // mesh, surface the error, and let the user undo the change.
    logLine("regenerate: begin (%zu overrides, %zu edge pins, %zu ops)",
            app.recipe.settings.perFace.size(),
            app.recipe.settings.perEdge.size(), app.recipe.ops.size());
    try {
        weft::GenerationReport report;
        double t0 = glfwGetTime();
        weft::PolyMesh mesh =
            weft::generate(app.model, app.analysis, app.recipe.settings,
                           &report, &app.genCache);
        logLine("regenerate: generate took %.1f ms",
                (glfwGetTime() - t0) * 1000.0);
        logLine("regenerate: generate ok, applying %zu op(s)",
                app.recipe.ops.size());
        weft::applyOps(mesh, app.model, app.recipe.ops);
        app.mesh = std::move(mesh);
        app.report = std::move(report);
    } catch (const std::exception& e) {
        logLine("regenerate: FAILED: %s", e.what());
        app.status = std::string("regenerate failed (ctrl+Z): ") + e.what();
        app.dirty = false;
        return;
    } catch (...) {
        logLine("regenerate: FAILED (unknown exception)");
        app.status = "regenerate failed (ctrl+Z to revert)";
        app.dirty = false;
        return;
    }
    logLine("regenerate: ops applied, rebuilding buffers");

    // Resample the B-rep edge overlay at the solved divisions so its
    // chords coincide with the mesh instead of ghosting past it. The edge
    // curve's parameter origin can be rotated against the surface's, so
    // snap each sample onto the nearest generated vertex too.
    app.brepEdges = weft::sampleEdges(app.model, 28,
                                      app.report.edgeDivisions);
    for (weft::EdgePolyline& e : app.brepEdges) {
        if (!app.report.edgeDivisions.count(e.edgeId)) continue;
        if (e.points.size() < 2) continue;
        double cl2 = 0;  // squared chord length as the snap radius
        {
            double dx = e.points[1][0] - e.points[0][0];
            double dy = e.points[1][1] - e.points[0][1];
            double dz = e.points[1][2] - e.points[0][2];
            cl2 = (dx * dx + dy * dy + dz * dz) * 0.36;  // (0.6*chord)^2
        }
        for (auto& p : e.points) {
            double best = cl2;
            const std::array<double, 3>* hit = nullptr;
            for (const auto& v : app.mesh.vertices) {
                double dx = v[0] - p[0], dy = v[1] - p[1], dz = v[2] - p[2];
                double d = dx * dx + dy * dy + dz * dz;
                if (d < best) {
                    best = d;
                    hit = &v;
                }
            }
            if (hit) p = *hit;
        }
    }

    // Open boundary loops (deleted faces leave them) for the bridge tool,
    // each mapped to its nearest sampled B-rep edge for a stable op id.
    app.bLoops = weft::boundaryLoops(app.mesh);
    app.bLoopEdge.assign(app.bLoops.size(), 0);
    app.hoverLoop = -1;
    for (size_t li = 0; li < app.bLoops.size(); ++li) {
        double bestDist = 1e300;
        for (const weft::EdgePolyline& e : app.brepEdges) {
            double sum = 0;
            for (uint32_t v : app.bLoops[li]) {
                const auto& p = app.mesh.vertices[v];
                double dmin = 1e300;
                for (const auto& q : e.points) {
                    double dx = p[0] - q[0], dy = p[1] - q[1],
                           dz = p[2] - q[2];
                    dmin = std::min(dmin, dx * dx + dy * dy + dz * dz);
                }
                sum += dmin;
            }
            if (sum < bestDist) {
                bestDist = sum;
                app.bLoopEdge[li] = e.edgeId;
            }
        }
    }

    rebuildBuffers(app);
    app.dirty = false;
    logLine("regenerate: done (%zu verts, %zu polys)",
            app.mesh.vertexCount(), app.mesh.polygonCount());
}

// Frame the selection if there is one, else the whole model (F).
static void frameModel(App& app) {
    if (app.mesh.vertices.empty()) return;
    std::array<double, 3> lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    bool any = false;
    if (!app.selFaces.empty()) {
        for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
            if (!app.selFaces.count(app.mesh.polygonFaceId[p])) continue;
            for (uint32_t vi : app.mesh.polygons[p]) {
                const auto& v = app.mesh.vertices[vi];
                for (int i = 0; i < 3; ++i) {
                    lo[i] = std::min(lo[i], v[i]);
                    hi[i] = std::max(hi[i], v[i]);
                }
                any = true;
            }
        }
    }
    if (!any) {
        for (const auto& v : app.mesh.vertices) {
            for (int i = 0; i < 3; ++i) {
                lo[i] = std::min(lo[i], v[i]);
                hi[i] = std::max(hi[i], v[i]);
            }
        }
    }
    app.cam.target = {float(lo[0] + hi[0]) * 0.5f, float(lo[1] + hi[1]) * 0.5f,
                      float(lo[2] + hi[2]) * 0.5f};
    double dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
    app.cam.dist = 1.9f * float(std::sqrt(dx * dx + dy * dy + dz * dz) + 1.0);
}

static void loadModel(App& app, const std::string& path) {
    logLine("load: %s", path.c_str());
    try {
        app.model = weft::loadStep(path);
        app.analysis = weft::analyze(app.model);
        app.brepEdges = weft::sampleEdges(app.model, 28);
        app.sourcePath = path;
        app.hasModel = true;
        app.selFaces.clear();
        app.selEdges.clear();
        app.activeFace = 0;
        app.undoStack.clear();
        app.genCache.clear();
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

// Native "open file" dialog: comdlg32 on Windows, zenity on Linux (falls
// back to the text field in the panel when neither is available).
static std::string openFileDialog() {
#ifdef _WIN32
    char file[1024] = "";
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof ofn;
    ofn.lpstrFilter = "STEP files (*.step;*.stp)\0*.step;*.stp\0"
                      "All files\0*.*\0";
    ofn.lpstrFile = file;
    ofn.nMaxFile = sizeof file;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn)) return file;
    return "";
#else
    FILE* p = popen(
        "zenity --file-selection --title='Open STEP' "
        "--file-filter='STEP | *.step *.stp *.STEP *.STP' 2>/dev/null",
        "r");
    if (!p) return "";
    char buf[1024] = "";
    std::string r;
    if (fgets(buf, sizeof buf, p)) {
        r = buf;
        while (!r.empty() && (r.back() == '\n' || r.back() == '\r')) {
            r.pop_back();
        }
    }
    pclose(p);
    return r;
#endif
}

// Native "save file" dialog for exports.
static std::string saveFileDialog(const char* defaultName) {
#ifdef _WIN32
    char file[1024];
    std::snprintf(file, sizeof file, "%s", defaultName);
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof ofn;
    ofn.lpstrFilter = "Wavefront OBJ (*.obj)\0*.obj\0All files\0*.*\0";
    ofn.lpstrFile = file;
    ofn.nMaxFile = sizeof file;
    ofn.lpstrDefExt = "obj";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return file;
    return "";
#else
    std::string cmd =
        "zenity --file-selection --save --title='Export OBJ' "
        "--filename='" + std::string(defaultName) + "' 2>/dev/null";
    FILE* p = popen(cmd.c_str(), "r");
    if (!p) return "";
    char buf[1024] = "";
    std::string r;
    if (fgets(buf, sizeof buf, p)) {
        r = buf;
        while (!r.empty() && (r.back() == '\n' || r.back() == '\r')) {
            r.pop_back();
        }
    }
    pclose(p);
    return r;
#endif
}

static std::string tempDir() {
    for (const char* var : {"TMPDIR", "TMP", "TEMP"}) {
        if (const char* d = std::getenv(var); d && *d) return d;
    }
    return ".";
}

static void loadFixture(App& app, const std::string& name) {
    std::string path = tempDir() + "/weft_fixture_" + name + ".step";
    try {
        weft::writeStep(weft::makeFixture(name), path);
        loadModel(app, path);
        app.status = "fixture: " + name;
    } catch (const std::exception& e) {
        app.status = std::string("fixture failed: ") + e.what();
    }
}

// ---------------------------------------------------------------------------
// Modal editing helpers.

// Every recipe mutation goes through this so the undo stack can snapshot
// the pre-edit state once per gesture (see the frame bookkeeping in main).
static void markDirty(App& app) {
    app.dirty = true;
    app.mutatedThisFrame = true;
}

// The density field the modal keys / wheel drive, chosen by what kind of
// face is active (radial for revolved faces, grid for planar...).
static int* primaryDensity(App& app, weft::FaceMeshSettings& s, bool secondary) {
    weft::MesherKind kind = weft::MesherKind::RevolutionGrid;
    if (app.activeFace > 0) {
        auto it = app.report.faceMesher.find(app.activeFace);
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

// Apply an edit to every selected face, auto-creating overrides — clicking
// a face and changing a value IS overriding it, no checkbox first. With
// no selection, edits go to the defaults.
template <typename F>
static void editSelected(App& app, F&& fn) {
    logLine("edit: %zu selected face(s), active %d", app.selFaces.size(),
            app.activeFace);
    if (app.selFaces.empty()) {
        fn(app.recipe.settings.defaults);
    } else {
        for (int fid : app.selFaces) {
            auto it = app.recipe.settings.perFace.find(fid);
            if (it == app.recipe.settings.perFace.end()) {
                it = app.recipe.settings.perFace
                         .emplace(fid, app.recipe.settings.defaults)
                         .first;
            }
            fn(it->second);
        }
    }
    markDirty(app);
}

// Read-only view of the active selection's current settings.
static const weft::FaceMeshSettings& activeSettings(const App& app) {
    return app.recipe.settings.forFace(app.activeFace);
}

// Adjust a per-edge division pin for every selected edge (edge mode).
static void adjustSelectedEdges(App& app, int typedValue, int delta) {
    for (int eid : app.selEdges) {
        int cur = typedValue;
        if (typedValue <= 0) {
            auto pin = app.recipe.settings.perEdge.find(eid);
            if (pin != app.recipe.settings.perEdge.end()) cur = pin->second;
            else {
                auto it = app.report.edgeDivisions.find(eid);
                cur = it != app.report.edgeDivisions.end() ? it->second : 8;
            }
            cur += delta;
        }
        app.recipe.settings.perEdge[eid] = std::max(1, cur);
    }
    if (!app.selEdges.empty()) markDirty(app);
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
    double bestDist = 26.0 * gUiScale;  // px
    uint32_t bestA = 0, bestB = 0;
    double bestT = 0.5;
    for (size_t p = 0; p < m.polygons.size(); ++p) {
        if (app.hiddenFaces.count(m.polygonFaceId[p])) continue;
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

// Bridge hover: nearest open boundary loop under the cursor. The hovered
// loop previews yellow; the first-clicked loop stays orange until the
// second click commits the bridge op.
static void updateBridgeHover(App& app, const Mat4& mvp, double mx, double my,
                              int fbw, int fbh) {
    app.hoverLoop = -1;
    app.preview.count = 0;
    if (!app.hasModel || app.bLoops.empty()) return;

    double bestDist = 30.0 * gUiScale;  // px
    for (size_t li = 0; li < app.bLoops.size(); ++li) {
        const auto& loop = app.bLoops[li];
        for (size_t i = 0; i < loop.size(); ++i) {
            float pa[3] = {0, 0, -1}, pb[3] = {0, 0, -1};
            projectPoint(mvp, app.mesh.vertices[loop[i]], fbw, fbh, pa);
            projectPoint(mvp, app.mesh.vertices[loop[(i + 1) % loop.size()]],
                         fbw, fbh, pb);
            if (pa[2] <= 0 || pb[2] <= 0) continue;
            float ex = pb[0] - pa[0], ey = pb[1] - pa[1];
            float len2 = ex * ex + ey * ey;
            float t = len2 < 1e-6f
                          ? 0.0f
                          : std::clamp(((float(mx) - pa[0]) * ex +
                                        (float(my) - pa[1]) * ey) / len2,
                                       0.0f, 1.0f);
            double d = std::hypot(double(mx) - (pa[0] + t * ex),
                                  double(my) - (pa[1] + t * ey));
            if (d < bestDist) {
                bestDist = d;
                app.hoverLoop = int(li);
            }
        }
    }

    std::vector<float> lines;
    auto pushLoop = [&](size_t li, float r, float g, float b) {
        const auto& loop = app.bLoops[li];
        for (size_t i = 0; i < loop.size(); ++i) {
            for (uint32_t v : {loop[i], loop[(i + 1) % loop.size()]}) {
                lines.push_back(float(app.mesh.vertices[v][0]));
                lines.push_back(float(app.mesh.vertices[v][1]));
                lines.push_back(float(app.mesh.vertices[v][2]));
                lines.push_back(r);
                lines.push_back(g);
                lines.push_back(b);
            }
        }
    };
    for (size_t li = 0; li < app.bLoops.size(); ++li) {
        if (app.bridgeFirstEdge && app.bLoopEdge[li] == app.bridgeFirstEdge) {
            pushLoop(li, 1.0f, 0.55f, 0.15f);  // committed first pick
        } else if (int(li) == app.hoverLoop) {
            pushLoop(li, 1.0f, 0.85f, 0.25f);  // hovered
        } else {
            pushLoop(li, 0.35f, 0.75f, 0.95f);  // available boundary
        }
    }
    if (!lines.empty()) app.preview.upload(lines);
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

static void buildUiScale(float scale) {
    gUiScale = scale;
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    ImFontConfig cfg;
    cfg.SizePixels = std::floor(13.0f * scale);
    io.Fonts->AddFontDefault(&cfg);
}

static void styleUi() {
    ImGui::GetStyle() = ImGuiStyle();  // reset before rescaling
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
    s.ScaleAllSizes(gUiScale);
}

// Density controls. With a mesher kind, only the settings that actually
// drive that face are shown — everything visible has a visible effect.
// Without one (the defaults), everything is shown, grouped by what it
// applies to; "freeform" leads because on an imported model most faces
// are freeform and deviation/angle are the real global density knobs.
static bool settingsEditor(weft::FaceMeshSettings& s,
                           const weft::MesherKind* kind = nullptr,
                           bool isFillet = false) {
    using MK = weft::MesherKind;
    const bool all = kind == nullptr;
    const MK k = kind ? *kind : MK::Fallback;
    const bool revolved = all || k == MK::RevolutionGrid || k == MK::DiskCap;
    const bool grid = all || k == MK::PlanarGrid || k == MK::MinimalNGon ||
                      k == MK::RingJunction || k == MK::CoonsGrid;
    const bool freeform = all || k == MK::QuadDominant || k == MK::Fallback;
    bool ch = false;

    if (freeform) {
        if (all) ImGui::TextDisabled("freeform / imported surfaces");
        float dev = float(s.chordTolerance);
        if (ImGui::DragFloat("deviation", &dev, 0.01f, 0.0005f, 100.0f,
                             "%.4f", ImGuiSliderFlags_Logarithmic)) {
            s.chordTolerance = dev;
            ch = true;
        }
        float ang = float(s.angleToleranceDeg);
        if (ImGui::DragFloat("angle", &ang, 0.25f, 1.0f, 60.0f, "%.1f deg")) {
            s.angleToleranceDeg = ang;
            ch = true;
        }
        ch |= ImGui::Checkbox("quad-dominant fallback", &s.quadDominant);
        float ms = float(s.minSize);
        if (ImGui::DragFloat("min size", &ms, 0.01f, 0.0f, 100.0f, "%.3f")) {
            s.minSize = ms;
            ch = true;
        }
        ch |= ImGui::Checkbox("relative deviation", &s.relativeDeviation);
    }
    if (revolved) {
        if (all) ImGui::TextDisabled("revolved surfaces");
        ch |= ImGui::DragInt("radial", &s.radial, 0.2f, 3, 256);
        if (all || k == MK::RevolutionGrid) {
            ch |= ImGui::DragInt("axial", &s.axial, 0.2f, 1, 256);
        }
        if (all || k == MK::DiskCap) {
            int cap = s.cap == weft::CapStyle::Fan ? 1 : 0;
            if (ImGui::Combo("cap style", &cap, "ngon\0fan\0")) {
                s.cap = cap ? weft::CapStyle::Fan : weft::CapStyle::NGon;
                ch = true;
            }
        }
    }
    if (grid) {
        if (all) ImGui::TextDisabled("planar / parametric grids");
        ch |= ImGui::DragInt("grid u", &s.gridU, 0.2f, 1, 256);
        ch |= ImGui::DragInt("grid v", &s.gridV, 0.2f, 1, 256);
        if (all || k == MK::RingJunction) {
            ch |= ImGui::DragInt("junction rings", &s.junctionRings, 0.2f, 1,
                                 32);
        }
        if (all || k == MK::PlanarGrid || k == MK::MinimalNGon) {
            ch |= ImGui::Checkbox("minimal n-gon (flat panels)", &s.minimal);
        }
    }
    if (all || isFillet) {
        if (all) ImGui::TextDisabled("fillets / blends");
        ch |= ImGui::DragInt("fillet loops", &s.filletLoops, 0.2f, 1, 64);
        float hold = float(s.filletHold);
        if (ImGui::SliderFloat("hold", &hold, 0.0f, 0.95f)) {
            s.filletHold = hold;
            ch = true;
        }
    }
    if (kind) {  // per-face contexts only
        // Manual mesher choice: auto picks per geometry; forcing one that
        // can't build on the face falls back to triangulation.
        static const char* kMesherItems =
            "auto\0revolution-grid\0disk-cap\0parametric-grid\0"
            "coons-grid\0ring-junction\0quad-dominant\0minimal-ngon\0"
            "fallback-tri\0annulus-ring\0";
        int mesher = s.forceMesher;
        if (ImGui::Combo("mesher", &mesher, kMesherItems)) {
            s.forceMesher = mesher;
            ch = true;
        }
        ch |= ImGui::Checkbox("delete face (bridge with J)", &s.exclude);
    }
    return ch;
}

// Rim controls for revolution bands: linked (one radial count, quad band)
// or unlinked (each rim pinned individually, triangulated taper between).
static void rimControls(App& app, int fid) {
    auto rims = app.report.faceRims.find(fid);
    if (rims == app.report.faceRims.end()) return;
    const weft::FaceMeshSettings& cur = app.recipe.settings.forFace(fid);
    bool linked = cur.linkRims;
    if (ImGui::Checkbox("link rims", &linked)) {
        editSelected(app,
                     [&](weft::FaceMeshSettings& s) { s.linkRims = linked; });
    }
    if (linked) return;
    ImGui::SameLine();
    ImGui::TextDisabled("(taper: tris between rims)");
    for (int r = 0; r < 2; ++r) {
        int eid = (*rims).second[r];
        int count = 8;
        auto pin = app.recipe.settings.perEdge.find(eid);
        if (pin != app.recipe.settings.perEdge.end()) count = pin->second;
        else {
            auto it = app.report.edgeDivisions.find(eid);
            if (it != app.report.edgeDivisions.end()) count = it->second;
        }
        ImGui::PushID(r);
        char label[32];
        std::snprintf(label, sizeof label, "rim %c (edge %d)", 'A' + r, eid);
        if (ImGui::DragInt(label, &count, 0.2f, 3, 256)) {
            app.recipe.settings.perEdge[eid] = std::max(3, count);
            markDirty(app);
        }
        ImGui::PopID();
    }
}

// Mode indicator + typed-number + hotkey reference, floating over the
// viewport so the keyboard flow never needs the side panel.
static void drawOverlay(App& app) {
    ImGui::SetNextWindowPos({12 * gUiScale, 12 * gUiScale});
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
    } else if (app.mode == Mode::Bridge) {
        ImGui::TextColored({1.0f, 0.85f, 0.25f, 1.0f}, "BRIDGE");
        ImGui::SameLine();
        ImGui::TextDisabled(app.bridgeFirstEdge
                                ? "pick the second loop - esc restarts"
                                : "pick two loops - [ ] twists last bridge"
                                  " - J/esc exits");
        if (app.hoverLoop >= 0) {
            ImGui::Text("loop: edge #%d, %zu verts",
                        app.bLoopEdge[app.hoverLoop],
                        app.bLoops[app.hoverLoop].size());
            ImGui::SameLine();
            ImGui::TextDisabled("( [ ] or 12<enter> pins the count )");
        }
        if (app.bLoops.empty()) {
            ImGui::TextDisabled(
                "no open boundaries - delete a face first (popup or outliner)");
        }
    } else if (app.selectMode == SelectMode::Edge) {
        ImGui::TextColored({0.6f, 0.8f, 1.0f, 1.0f}, "EDGE MODE");
        ImGui::SameLine();
        if (app.selEdges.empty()) {
            ImGui::TextDisabled("click edges (shift extends) - tab: faces");
        } else {
            ImGui::TextDisabled("%zu edge(s) - wheel/[ ]/number pins verts"
                                " - J bridges 2",
                                app.selEdges.size());
        }
    } else if (!app.selFaces.empty()) {
        if (app.selFaces.size() == 1) {
            ImGui::TextDisabled("face #%d selected", app.activeFace);
        } else {
            ImGui::TextDisabled("%zu faces selected (active #%d)",
                                app.selFaces.size(), app.activeFace);
        }
        ImGui::SameLine();
        ImGui::TextDisabled(" shift+wheel density, ctrl+wheel 2nd axis");
    } else {
        ImGui::TextDisabled("no selection - click a face, tab for edges");
    }
    if (!app.numberEntry.empty()) {
        ImGui::TextColored({1.0f, 0.85f, 0.25f, 1.0f}, "divisions: %s_",
                           app.numberEntry.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("enter applies (shift+enter: secondary)");
    }
    ImGui::End();

    // Floating value readout at the cursor while wheel-editing density.
    if (glfwGetTime() < app.hudUntil && app.hudText[0]) {
        ImVec2 mp = ImGui::GetMousePos();
        ImGui::SetNextWindowPos({mp.x + 20 * gUiScale, mp.y + 16 * gUiScale});
        ImGui::SetNextWindowBgAlpha(0.75f);
        ImGui::Begin("##wheelhud", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoInputs |
                         ImGuiWindowFlags_NoFocusOnAppearing |
                         ImGuiWindowFlags_NoNav);
        ImGui::SetWindowFontScale(1.5f);
        ImGui::TextColored({1.0f, 0.85f, 0.25f, 1.0f}, "%s", app.hudText);
        ImGui::End();
    }

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({12 * gUiScale, vp->WorkSize.y - 12 * gUiScale},
                            ImGuiCond_Always, {0.0f, 1.0f});
    ImGui::SetNextWindowBgAlpha(0.45f);
    ImGui::Begin("##hotkeys", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    ImGui::TextDisabled(
        "tab face/edge mode   shift+click multi-select   ctrl+Z undo\n"
        "shift+wheel density   ctrl+wheel 2nd axis   ctrl+shift+wheel loops\n"
        "12<enter> divisions   [ ] nudge\n"
        "X delete face   H hide (shift+H show all)   R loop cut   J bridge\n"
        "C cap   T tris   M minimal   W wire   B edges   F focus   esc");
    ImGui::End();
}

// Context popup on RIGHT-click over a face (Blender-style): the active
// face's live controls (edits apply to the whole selection and override
// automatically) plus visibility actions. Left-click just selects; the
// modal keys/wheel are the primary editing path.
static void drawFacePopup(App& app) {
    if (app.openFacePopup) {
        if (app.activeFace > 0) ImGui::OpenPopup("##facectx");
        app.openFacePopup = false;
    }
    if (!ImGui::BeginPopup("##facectx")) return;
    if (app.activeFace <= 0) {
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
    }
    const weft::FaceInfo& f = app.analysis.faces[app.activeFace - 1];
    if (app.selFaces.size() > 1) {
        ImGui::Text("%zu faces (active #%d %s)", app.selFaces.size(), f.id,
                    weft::surfaceTypeName(f.type));
    } else {
        ImGui::Text("face #%d  %s%s%s", f.id, weft::surfaceTypeName(f.type),
                    f.isFillet ? "  [fillet]" : "", f.isHole ? "  [hole]" : "");
    }
    weft::MesherKind kind = weft::MesherKind::Fallback;
    auto it = app.report.faceMesher.find(f.id);
    if (it != app.report.faceMesher.end()) kind = it->second;
    ImGui::TextDisabled("mesher: %s", weft::mesherKindName(kind));

    // Editing auto-overrides: changes land on every selected face.
    weft::FaceMeshSettings edited = activeSettings(app);
    // When a mesher is forced, show ITS controls (so it can be tuned
    // before/despite building) and flag when it couldn't build here.
    if (edited.forceMesher > 0) {
        weft::MesherKind forced = weft::MesherKind(edited.forceMesher - 1);
        if (forced != kind) {
            ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                               "forced %s couldn't build here",
                               weft::mesherKindName(forced));
        }
        kind = forced;
    }
    ImGui::Separator();
    ImGui::PushID("ctx");
    ImGui::PushItemWidth(150 * gUiScale);
    bool changed = settingsEditor(edited, &kind, f.isFillet);
    if (changed) {
        editSelected(app, [&](weft::FaceMeshSettings& s) { s = edited; });
    }
    if (kind == weft::MesherKind::RevolutionGrid) {
        rimControls(app, app.activeFace);
    }
    ImGui::PopItemWidth();
    ImGui::PopID();
    if (app.recipe.settings.perFace.count(app.activeFace)) {
        if (ImGui::SmallButton("clear override(s)")) {
            for (int fid : app.selFaces) {
                app.recipe.settings.perFace.erase(fid);
            }
            markDirty(app);
        }
    }

    ImGui::Separator();
    if (ImGui::MenuItem("delete face(s)", "X")) {
        editSelected(app, [](weft::FaceMeshSettings& s) { s.exclude = true; });
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::MenuItem("hide face(s)", "H")) {
        for (int fid : app.selFaces) app.hiddenFaces.insert(fid);
        app.selFaces.clear();
        app.activeFace = 0;
        rebuildBuffers(app);
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::MenuItem("isolate selection")) {
        app.hiddenFaces.clear();
        for (const auto& fi : app.analysis.faces) {
            if (!app.selFaces.count(fi.id)) app.hiddenFaces.insert(fi.id);
        }
        rebuildBuffers(app);
    }
    if (!app.hiddenFaces.empty() && ImGui::MenuItem("show all", "shift+H")) {
        app.hiddenFaces.clear();
        rebuildBuffers(app);
    }
    if (ImGui::MenuItem("loop cut mode", "R")) app.mode = Mode::LoopCut;
    ImGui::EndPopup();
}

static void drawUi(App& app) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float width = 330.0f * gUiScale;
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
        if (ImGui::Button("Open STEP...", {-1, 0})) {
            std::string p = openFileDialog();
            if (!p.empty()) {
                std::snprintf(app.pathBuf, sizeof app.pathBuf, "%s",
                              p.c_str());
                loadModel(app, p);
            }
        }
        ImGui::InputTextWithHint("##path", "or type a path...", app.pathBuf,
                                 sizeof app.pathBuf);
        ImGui::SameLine();
        if (ImGui::Button("Load")) loadModel(app, app.pathBuf);
        if (app.hasModel && ImGui::Button("Export OBJ...", {-1, 0})) {
            // Default name: the source file with .obj — one group per
            // B-rep face, so CAD face IDs survive into Blender.
            std::string base = app.sourcePath;
            size_t slash = base.find_last_of("/\\");
            if (slash != std::string::npos) base = base.substr(slash + 1);
            size_t dot = base.find_last_of('.');
            if (dot != std::string::npos) base = base.substr(0, dot);
            if (base.empty()) base = "weft";
            std::string out = saveFileDialog((base + ".obj").c_str());
            if (!out.empty()) {
                try {
                    weft::writeObj(app.mesh, out);
                    app.status = "exported " + out;
                    logLine("export: %s (%zu verts, %zu polys)", out.c_str(),
                            app.mesh.vertexCount(), app.mesh.polygonCount());
                } catch (const std::exception& e) {
                    app.status = std::string("export failed: ") + e.what();
                }
            }
        }
        ImGui::TextWrapped("%s", app.status.c_str());
    }

    if (app.hasModel &&
        ImGui::CollapsingHeader("Outliner", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::SmallButton("show all")) {
            app.hiddenFaces.clear();
            rebuildBuffers(app);
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%zu object(s), %zu hidden face(s)",
                            app.analysis.solidFaces.size(),
                            app.hiddenFaces.size());
        ImGui::BeginChild("##outliner", {0, 200 * gUiScale}, true);
        for (size_t si = 0; si < app.analysis.solidFaces.size(); ++si) {
            const std::vector<int>& fids = app.analysis.solidFaces[si];
            ImGui::PushID(int(si));
            // Object row: visibility eye + expandable face list.
            bool anyVisible = false;
            for (int fid : fids) {
                if (!app.hiddenFaces.count(fid)) anyVisible = true;
            }
            bool vis = anyVisible;
            if (ImGui::Checkbox("##ovis", &vis)) {
                for (int fid : fids) {
                    if (vis) app.hiddenFaces.erase(fid);
                    else app.hiddenFaces.insert(fid);
                }
                rebuildBuffers(app);
            }
            ImGui::SameLine();
            char objLabel[64];
            std::snprintf(objLabel, sizeof objLabel, "object %zu (%zu faces)",
                          si + 1, fids.size());
            bool open = ImGui::TreeNodeEx(
                objLabel, ImGuiTreeNodeFlags_OpenOnArrow |
                              ImGuiTreeNodeFlags_SpanAvailWidth);
            if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) {
                // Select the whole object's faces (shift extends).
                if (!ImGui::GetIO().KeyShift) app.selFaces.clear();
                for (int fid : fids) app.selFaces.insert(fid);
                if (!fids.empty()) app.activeFace = fids[0];
                rebuildBuffers(app);
            }
            if (open) {
                for (int fid : fids) {
                    const weft::FaceInfo& f = app.analysis.faces[fid - 1];
                    ImGui::PushID(fid);
                    bool fvis = !app.hiddenFaces.count(fid);
                    if (ImGui::Checkbox("##vis", &fvis)) {
                        if (fvis) app.hiddenFaces.erase(fid);
                        else app.hiddenFaces.insert(fid);
                        rebuildBuffers(app);
                    }
                    ImGui::SameLine();
                    auto ov = app.recipe.settings.perFace.find(fid);
                    bool deleted = ov != app.recipe.settings.perFace.end() &&
                                   ov->second.exclude;
                    char label[112];
                    std::snprintf(label, sizeof label, "face %-4d %s%s%s%s",
                                  fid, weft::surfaceTypeName(f.type),
                                  f.isFillet ? " [fillet]" : "",
                                  f.isHole ? " [hole]" : "",
                                  deleted ? " [deleted]" : "");
                    if (ImGui::Selectable(label,
                                          app.selFaces.count(fid) > 0)) {
                        if (!ImGui::GetIO().KeyShift) app.selFaces.clear();
                        if (app.selFaces.count(fid) &&
                            ImGui::GetIO().KeyShift) {
                            app.selFaces.erase(fid);
                        } else {
                            app.selFaces.insert(fid);
                            app.activeFace = fid;
                        }
                        rebuildBuffers(app);
                    }
                    ImGui::PopID();
                }
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        ImGui::EndChild();
    }

    if (app.hasModel &&
        ImGui::CollapsingHeader("Topology", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("%zu verts   %zu polys", app.mesh.vertexCount(),
                    app.mesh.polygonCount());
        ImGui::Text("%zu quads  %zu tris  %zu n-gons", app.mesh.countQuads(),
                    app.mesh.countTris(), app.mesh.countNgons());
        if (!app.bLoops.empty()) {
            ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                               "%zu open border loop(s)",
                               app.bLoops.size());
            ImGui::SameLine();
            ImGui::TextDisabled("(J bridges, deleted faces expected)");
        }
        ImGui::Separator();
        ImGui::TextDisabled("defaults (live)");
        if (settingsEditor(app.recipe.settings.defaults)) markDirty(app);
    }

    if (app.hasModel &&
        ImGui::CollapsingHeader("Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (app.selectMode == SelectMode::Edge) {
            if (app.selEdges.empty()) {
                ImGui::TextDisabled("click edges (shift extends)");
            } else {
                ImGui::Text("%zu edge(s) selected", app.selEdges.size());
                for (int eid : app.selEdges) {
                    auto pin = app.recipe.settings.perEdge.find(eid);
                    auto cur = app.report.edgeDivisions.find(eid);
                    ImGui::TextDisabled(
                        "edge #%d: %d divisions%s", eid,
                        pin != app.recipe.settings.perEdge.end()
                            ? pin->second
                            : (cur != app.report.edgeDivisions.end()
                                   ? cur->second
                                   : 0),
                        pin != app.recipe.settings.perEdge.end()
                            ? " (pinned)"
                            : "");
                }
                ImGui::TextDisabled("wheel / [ ] / 12<enter> pins verts");
                bool anyPinned = false;
                for (int eid : app.selEdges) {
                    anyPinned |= app.recipe.settings.perEdge.count(eid) > 0;
                }
                if (anyPinned && ImGui::SmallButton("clear pin(s)")) {
                    for (int eid : app.selEdges) {
                        app.recipe.settings.perEdge.erase(eid);
                    }
                    markDirty(app);
                }
            }
        } else if (app.activeFace <= 0) {
            ImGui::TextDisabled("click a face in the viewport");
        } else {
            const weft::FaceInfo& f = app.analysis.faces[app.activeFace - 1];
            if (app.selFaces.size() > 1) {
                ImGui::Text("%zu faces (active #%d)", app.selFaces.size(),
                            f.id);
            }
            ImGui::Text("face #%d  %s%s%s", f.id, weft::surfaceTypeName(f.type),
                        f.isFillet ? "  [fillet]" : "",
                        f.isHole ? "  [hole]" : "");
            if (f.radius > 0) ImGui::Text("radius %.3f", f.radius);
            weft::MesherKind kind = weft::MesherKind::Fallback;
            auto it = app.report.faceMesher.find(f.id);
            if (it != app.report.faceMesher.end()) {
                kind = it->second;
                ImGui::Text("mesher: %s", weft::mesherKindName(kind));
            }
            // Editing auto-overrides every selected face.
            weft::FaceMeshSettings edited = activeSettings(app);
            if (edited.forceMesher > 0) {
                weft::MesherKind forced =
                    weft::MesherKind(edited.forceMesher - 1);
                if (forced != kind) {
                    ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                                       "forced %s couldn't build here",
                                       weft::mesherKindName(forced));
                }
                kind = forced;
            }
            ImGui::PushID("perface");
            bool changed = settingsEditor(edited, &kind, f.isFillet);
            ImGui::PopID();
            if (changed) {
                editSelected(app,
                             [&](weft::FaceMeshSettings& s) { s = edited; });
            }
            if (kind == weft::MesherKind::RevolutionGrid) {
                ImGui::PushID("rims");
                rimControls(app, app.activeFace);
                ImGui::PopID();
            }
            if (app.recipe.settings.perFace.count(app.activeFace)) {
                ImGui::TextDisabled("overridden");
                ImGui::SameLine();
                if (ImGui::SmallButton("clear")) {
                    for (int fid : app.selFaces) {
                        app.recipe.settings.perFace.erase(fid);
                    }
                    markDirty(app);
                }
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
                markDirty(app);
                app.status = std::string("loaded ") + app.recipeBuf;
            } catch (const std::exception& e) {
                app.status = e.what();
            }
        }
        ImGui::Text("%zu manual op(s) recorded", app.recipe.ops.size());
    }

    if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Combo("shading", &app.shadingMode,
                         "shaded + wire\0shaded\0wireframe\0"
                         "flat + wire\0")) {
            app.showFill = app.shadingMode != 2;
            app.showWire = app.shadingMode != 1;
        }
        ImGui::Checkbox("feature edges", &app.showBrepEdges);
        ImGui::SameLine();
        ImGui::Checkbox("selection verts", &app.showVerts);
        ImGui::ColorEdit3("background", app.bgColor,
                          ImGuiColorEditFlags_NoInputs);
        ImGui::SameLine();
        ImGui::ColorEdit3("wireframe", app.wireColor,
                          ImGuiColorEditFlags_NoInputs);
        ImGui::SameLine();
        ImGui::ColorEdit3("verts", app.vertColor,
                          ImGuiColorEditFlags_NoInputs);
        ImGui::TextDisabled("lighting (image-based HDRI planned)");
        ImGui::SliderFloat("ambient", &app.lightAmbient, 0.0f, 1.0f);
        ImGui::SliderFloat("diffuse", &app.lightDiffuse, 0.0f, 1.5f);
        ImGui::SliderFloat("rim light", &app.lightRim, 0.0f, 0.5f);
        ImGui::TextDisabled("theme:");
        ImGui::SameLine();
        auto theme = [&](const char* name, float br, float bg2, float bb,
                         float wr, float wg, float wb) {
            if (ImGui::SmallButton(name)) {
                app.bgColor[0] = br; app.bgColor[1] = bg2; app.bgColor[2] = bb;
                app.wireColor[0] = wr; app.wireColor[1] = wg;
                app.wireColor[2] = wb;
            }
            ImGui::SameLine();
        };
        theme("dark", 0.117f, 0.125f, 0.145f, 0.10f, 0.11f, 0.13f);
        theme("light", 0.86f, 0.87f, 0.89f, 0.28f, 0.29f, 0.32f);
        theme("slate", 0.16f, 0.19f, 0.24f, 0.09f, 0.11f, 0.15f);
        ImGui::NewLine();
        ImGui::TextDisabled("orange convex / blue concave / green smooth");
        ImGui::TextDisabled("LMB select · MMB orbit · shift+MMB pan");
        ImGui::TextDisabled("ctrl+MMB zoom · alt+MMB axis view · wheel");
        ImGui::Text("%zu manual op(s)", app.recipe.ops.size());
    }

    if (ImGui::CollapsingHeader("Debug")) {
        ImGui::TextDisabled("bisect switches — try these if it crashes");
        bool single = !app.recipe.settings.parallelMeshing;
        if (ImGui::Checkbox("single-threaded meshing", &single)) {
            app.recipe.settings.parallelMeshing = !single;
            markDirty(app);
        }
        bool conform = app.recipe.settings.conformBorders;
        if (ImGui::Checkbox("border conformity pass", &conform)) {
            app.recipe.settings.conformBorders = conform;
            markDirty(app);
        }
        static bool coreTrace = true;
        if (ImGui::Checkbox("core trace in log", &coreTrace)) {
            weft::setGenerateDebugLog(coreTrace ? gDebugLog : nullptr);
        }
        if (ImGui::Button("force regenerate")) app.dirty = true;
        ImGui::TextDisabled("log (flushed per line — after a crash its");
        ImGui::TextDisabled("tail names the face/stage that died):");
        ImGui::TextWrapped("%s", gLogPath.c_str());
    }

    if (ImGui::CollapsingHeader("Dev fixtures")) {
        ImGui::TextDisabled("built-in test shapes");
        const char* fixtures[] = {"cylinder", "box",  "cone", "sphere",
                                  "torus",    "fillet", "hole", "boss", "demo"};
        for (int i = 0; i < 9; ++i) {
            if (i % 3) ImGui::SameLine();
            if (ImGui::Button(fixtures[i], {96 * gUiScale, 0})) {
                loadFixture(app, fixtures[i]);
            }
        }
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------

static float gScroll = 0.0f;
static void scrollCb(GLFWwindow*, double, double dy) {
    gScroll += float(dy);
}

int main(int argc, char** argv) {
    gDataDir = userDataDir();
    gLogPath = gDataDir + "/weft_debug.log";
    gDebugLog = std::fopen(gLogPath.c_str(), "w");
    installCrashHandler();
    weft::setGenerateDebugLog(gDebugLog);
    logLine("weft_app start (built %s %s)", __DATE__, __TIME__);

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
    // Windows DPI: size the window by the monitor's content scale and
    // track scale changes when it moves between monitors.
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
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
    static std::string iniPath = gDataDir + "/imgui.ini";
    ImGui::GetIO().IniFilename = iniPath.c_str();
    {
        float sx = 1.0f, sy = 1.0f;
        glfwGetWindowContentScale(window, &sx, &sy);
        buildUiScale(sx > 0 ? sx : 1.0f);
        logLine("ui scale: %.2f", gUiScale);
    }
    styleUi();
    glfwSetWindowContentScaleCallback(
        window, [](GLFWwindow*, float sx, float) { gPendingUiScale = sx; });
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
        app.selFaces = {startSelect};
        app.activeFace = startSelect;
        rebuildBuffers(app);
    }

    double lastX = 0, lastY = 0;
    bool navOrbit = false, navPan = false, navZoom = false, navSnap = false;
    double downX = 0, downY = 0, downRX = 0, downRY = 0;
    bool prevLmb = false, prevRmb = false;
    int frame = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGuiIO& io = ImGui::GetIO();

        // DPI changed (window moved to a different-scale monitor):
        // rebuild fonts and style metrics at the new scale.
        if (gPendingUiScale > 0 &&
            std::abs(gPendingUiScale - gUiScale) > 0.01f) {
            buildUiScale(gPendingUiScale);
            styleUi();
            ImGui_ImplOpenGL3_DestroyDeviceObjects();  // re-uploads fonts
            logLine("ui scale changed: %.2f", gUiScale);
        }
        gPendingUiScale = 0.0f;

        // Undo bookkeeping: snapshot the recipe before this frame's edits;
        // if a gesture STARTS this frame (markDirty after a quiet frame),
        // that snapshot becomes the undo point. Drags coalesce into one.
        app.preFrame = app.recipe;
        app.mutatedThisFrame = false;

        int fbw, fbh;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        if (demoLoopCut) {  // scripted screenshots: cursor at viewport center
            app.mode = Mode::LoopCut;
            mx = fbw * 0.42;
            my = fbh * 0.5;
        }

        // Blender-standard navigation: MMB orbit, shift+MMB pan, ctrl+MMB
        // drag-zoom, wheel zoom; alt+MMB orbits and snaps to the nearest
        // axis-aligned view on release.
        if (!io.WantCaptureMouse) {
            bool mmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) ==
                       GLFW_PRESS;
            bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                         glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            bool ctrl = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
            bool alt = glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
            if (mmb && !navOrbit && !navPan && !navZoom) {
                navPan = shift;
                navZoom = ctrl && !shift;
                navOrbit = !navPan && !navZoom;
                navSnap = alt;
                lastX = mx;
                lastY = my;
            }
            if (!mmb) {
                if (navOrbit && navSnap) {  // alt+MMB: nearest axis view
                    const float halfPi = 1.5707964f;
                    app.cam.yaw = std::round(app.cam.yaw / halfPi) * halfPi;
                    app.cam.pitch =
                        std::round(app.cam.pitch / halfPi) * halfPi;
                    app.cam.pitch = std::clamp(app.cam.pitch, -1.55f, 1.55f);
                }
                navOrbit = navPan = navZoom = navSnap = false;
            }
            if (navOrbit) {
                app.cam.yaw -= float(mx - lastX) * 0.008f;
                app.cam.pitch += float(my - lastY) * 0.008f;
                app.cam.pitch = std::clamp(app.cam.pitch, -1.55f, 1.55f);
            }
            if (navPan) {
                float k = app.cam.dist * 0.0016f;
                Vec3 eye = app.cam.eye();
                Vec3 f = norm(sub(app.cam.target, eye));
                Vec3 r = norm(cross(f, {0, 0, 1}));
                Vec3 u = cross(r, f);
                app.cam.target.x -= (float(mx - lastX) * r.x - float(my - lastY) * u.x) * k;
                app.cam.target.y -= (float(mx - lastX) * r.y - float(my - lastY) * u.y) * k;
                app.cam.target.z -= (float(mx - lastX) * r.z - float(my - lastY) * u.z) * k;
            }
            if (navZoom) {
                app.cam.dist *= std::pow(1.006f, float(my - lastY));
                app.cam.dist = std::clamp(app.cam.dist, 0.5f, 10000.0f);
            }
            lastX = mx;
            lastY = my;
            if (gScroll != 0.0f) {
                // Modal density: shift+wheel drives the primary axis
                // (radial/grid-u), ctrl+wheel the secondary (axial/grid-v),
                // on the whole selection — Blender-style. Plain wheel zooms.
                int steps = int(gScroll > 0 ? std::ceil(gScroll)
                                            : std::floor(gScroll));
                if (app.hasModel && (shift || ctrl) &&
                    app.selectMode == SelectMode::Edge &&
                    !app.selEdges.empty()) {
                    adjustSelectedEdges(app, 0, steps);
                    int eid = *app.selEdges.begin();
                    std::snprintf(app.hudText, sizeof app.hudText,
                                  "edge verts: %d",
                                  app.recipe.settings.perEdge[eid]);
                    app.hudUntil = glfwGetTime() + 0.9;
                } else if (app.hasModel && ctrl && shift &&
                           !app.selFaces.empty()) {
                    // ctrl+shift+wheel: fillet support loops.
                    editSelected(app, [&](weft::FaceMeshSettings& s) {
                        s.filletLoops = std::max(1, s.filletLoops + steps);
                    });
                    std::snprintf(app.hudText, sizeof app.hudText,
                                  "fillet loops: %d",
                                  app.recipe.settings.forFace(app.activeFace)
                                      .filletLoops);
                    app.hudUntil = glfwGetTime() + 0.9;
                } else if (app.hasModel && (shift || ctrl) &&
                           (!app.selFaces.empty())) {
                    bool secondary = ctrl;
                    editSelected(app, [&](weft::FaceMeshSettings& s) {
                        int* v = primaryDensity(app, s, secondary);
                        *v = std::max(1, *v + steps);
                    });
                    weft::FaceMeshSettings cur =
                        app.recipe.settings.forFace(app.activeFace);
                    std::snprintf(app.hudText, sizeof app.hudText, "%s: %d",
                                  secondary ? "secondary" : "primary",
                                  *primaryDensity(app, cur, secondary));
                    app.hudUntil = glfwGetTime() + 0.9;
                } else {
                    app.cam.dist *= std::pow(0.92f, gScroll);
                    app.cam.dist = std::clamp(app.cam.dist, 0.5f, 10000.0f);
                }
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
            if (ImGui::IsKeyPressed(ImGuiKey_J, false)) {
                if (app.selectMode == SelectMode::Edge &&
                    app.selEdges.size() == 2) {
                    // Direct bridge between the two selected edges.
                    auto it = app.selEdges.begin();
                    weft::ManualOp op;
                    op.kind = weft::ManualOp::Kind::Bridge;
                    op.edgeA = *it++;
                    op.edgeB = *it;
                    app.recipe.ops.push_back(op);
                    markDirty(app);
                    app.status = app.bLoops.empty()
                                     ? "bridge recorded (needs open "
                                       "boundaries - X deletes faces)"
                                     : "bridge committed (ctrl+Z undoes)";
                } else {
                    app.mode =
                        app.mode == Mode::Bridge ? Mode::Idle : Mode::Bridge;
                    app.bridgeFirstEdge = 0;
                    app.hoverLoop = -1;
                    if (app.mode == Mode::Bridge && app.bLoops.empty()) {
                        app.status =
                            "bridge: no open boundaries (delete a face first)";
                    }
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                if (app.mode == Mode::Bridge && app.bridgeFirstEdge) {
                    app.bridgeFirstEdge = 0;
                } else if (app.mode != Mode::Idle) app.mode = Mode::Idle;
                else if (!app.numberEntry.empty()) app.numberEntry.clear();
                else if (!app.selFaces.empty() || !app.selEdges.empty()) {
                    app.selFaces.clear();
                    app.selEdges.clear();
                    app.activeFace = 0;
                    rebuildBuffers(app);
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Tab, false)) {
                app.selectMode = app.selectMode == SelectMode::Face
                                     ? SelectMode::Edge
                                     : SelectMode::Face;
                app.status = app.selectMode == SelectMode::Edge
                                 ? "edge select mode"
                                 : "face select mode";
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
                if (app.mode == Mode::Bridge && app.hoverLoop >= 0) {
                    // Pin the hovered boundary loop's vertex count: equal
                    // counts bridge as quads, unequal as triangles.
                    int eid = app.bLoopEdge[app.hoverLoop];
                    app.recipe.settings.perEdge[eid] = std::max(3, value);
                    markDirty(app);
                } else if (app.selectMode == SelectMode::Edge) {
                    adjustSelectedEdges(app, value, 0);
                } else {
                    editSelected(app, [&](weft::FaceMeshSettings& s) {
                        *primaryDensity(app, s, shift) = std::max(1, value);
                    });
                }
                app.numberEntry.clear();
            }
            bool dec = ImGui::IsKeyPressed(ImGuiKey_LeftBracket);
            bool inc = ImGui::IsKeyPressed(ImGuiKey_RightBracket);
            if ((dec || inc) && app.hasModel) {
                int delta = inc ? 1 : -1;
                if (app.mode == Mode::Bridge && app.hoverLoop >= 0) {
                    int eid = app.bLoopEdge[app.hoverLoop];
                    int cur = int(app.bLoops[app.hoverLoop].size());
                    auto it = app.recipe.settings.perEdge.find(eid);
                    if (it != app.recipe.settings.perEdge.end()) {
                        cur = it->second;
                    }
                    app.recipe.settings.perEdge[eid] = std::max(3, cur + delta);
                    markDirty(app);
                } else if (app.mode == Mode::Bridge) {
                    // Nothing hovered: twist the most recent bridge so its
                    // rails stop spiralling.
                    for (auto op = app.recipe.ops.rbegin();
                         op != app.recipe.ops.rend(); ++op) {
                        if (op->kind != weft::ManualOp::Kind::Bridge) continue;
                        op->twist += delta;
                        markDirty(app);
                        std::snprintf(app.hudText, sizeof app.hudText,
                                      "bridge twist: %+d", op->twist);
                        app.hudUntil = glfwGetTime() + 0.9;
                        app.status = "bridge twist adjusted";
                        break;
                    }
                } else if (app.selectMode == SelectMode::Edge) {
                    adjustSelectedEdges(app, 0, delta);
                } else {
                    editSelected(app, [&](weft::FaceMeshSettings& s) {
                        int* v = primaryDensity(app, s, shift);
                        *v = std::max(1, *v + delta);
                    });
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_C, false) && app.hasModel) {
                weft::CapStyle next = activeSettings(app).cap ==
                                              weft::CapStyle::NGon
                                          ? weft::CapStyle::Fan
                                          : weft::CapStyle::NGon;
                editSelected(app,
                             [&](weft::FaceMeshSettings& s) { s.cap = next; });
            }
            if (ImGui::IsKeyPressed(ImGuiKey_T, false) && app.hasModel) {
                bool next = !activeSettings(app).quadDominant;
                editSelected(app, [&](weft::FaceMeshSettings& s) {
                    s.quadDominant = next;
                });
            }
            if (ImGui::IsKeyPressed(ImGuiKey_M, false) && app.hasModel) {
                bool next = !activeSettings(app).minimal;
                editSelected(app,
                             [&](weft::FaceMeshSettings& s) { s.minimal = next; });
            }
            if (ImGui::IsKeyPressed(ImGuiKey_X, false) && app.hasModel &&
                !app.selFaces.empty()) {
                editSelected(app,
                             [](weft::FaceMeshSettings& s) { s.exclude = true; });
                app.status = "face(s) deleted (ctrl+Z undoes, J bridges rims)";
            }
            if (ImGui::IsKeyPressed(ImGuiKey_H, false) && app.hasModel) {
                if (shift) {
                    app.hiddenFaces.clear();
                    rebuildBuffers(app);
                    app.status = "all faces shown";
                } else if (!app.selFaces.empty()) {
                    for (int fid : app.selFaces) app.hiddenFaces.insert(fid);
                    app.selFaces.clear();
                    app.activeFace = 0;
                    rebuildBuffers(app);
                    app.status = "face(s) hidden (shift+H shows all)";
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_W, false)) app.showWire = !app.showWire;
            if (ImGui::IsKeyPressed(ImGuiKey_B, false)) {
                app.showBrepEdges = !app.showBrepEdges;
            }
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
                if (!app.undoStack.empty()) {
                    app.recipe = app.undoStack.back();
                    app.undoStack.pop_back();
                    app.dirty = true;  // not a mutation: no undo push
                    app.status = "undo";
                } else {
                    app.status = "nothing to undo";
                }
            }
        }

        bool lmb = !io.WantCaptureMouse &&
                   glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) ==
                       GLFW_PRESS;
        if (lmb && !prevLmb) { downX = mx; downY = my; }
        bool clicked = prevLmb && !lmb && std::abs(mx - downX) < 4 &&
                       std::abs(my - downY) < 4;
        prevLmb = lmb;
        bool rmb = !io.WantCaptureMouse &&
                   glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) ==
                       GLFW_PRESS;
        if (rmb && !prevRmb) { downRX = mx; downRY = my; }
        bool rClicked = prevRmb && !rmb && std::abs(mx - downRX) < 4 &&
                        std::abs(my - downRY) < 4;
        prevRmb = rmb;
        gScroll = 0.0f;

        if (app.mutatedThisFrame) logLine("frame: input handled, dirty");
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
                markDirty(app);
                app.status = "loop cut committed (ctrl+z undoes)";
            }
        } else if (app.mode == Mode::Bridge && !io.WantCaptureMouse) {
            updateBridgeHover(app, mvp, mx, my, fbw, fbh);
            if (clicked && app.hoverLoop >= 0) {
                int eid = app.bLoopEdge[app.hoverLoop];
                if (app.bridgeFirstEdge == 0) {
                    app.bridgeFirstEdge = eid;
                    app.status = "bridge: pick the second boundary loop";
                } else if (eid != app.bridgeFirstEdge) {
                    weft::ManualOp op;
                    op.kind = weft::ManualOp::Kind::Bridge;
                    op.edgeA = app.bridgeFirstEdge;
                    op.edgeB = eid;
                    app.recipe.ops.push_back(op);
                    app.bridgeFirstEdge = 0;
                    markDirty(app);
                    app.status = "bridge committed (ctrl+z undoes)";
                }
            }
        } else if ((clicked || rClicked) && app.hasModel) {
            bool shift = io.KeyShift;
            if (app.selectMode == SelectMode::Edge && clicked) {
                // Edge picking: nearest projected B-rep edge polyline.
                int hit = 0;
                double best = 14.0 * gUiScale;  // px
                for (const weft::EdgePolyline& e : app.brepEdges) {
                    for (size_t i = 0; i + 1 < e.points.size(); ++i) {
                        float pa[3] = {0, 0, -1}, pb[3] = {0, 0, -1};
                        projectPoint(mvp, e.points[i], fbw, fbh, pa);
                        projectPoint(mvp, e.points[i + 1], fbw, fbh, pb);
                        if (pa[2] <= 0 || pb[2] <= 0) continue;
                        float ex = pb[0] - pa[0], ey = pb[1] - pa[1];
                        float len2 = ex * ex + ey * ey;
                        float t = len2 < 1e-6f
                                      ? 0.0f
                                      : std::clamp(((float(mx) - pa[0]) * ex +
                                                    (float(my) - pa[1]) * ey) /
                                                       len2,
                                                   0.0f, 1.0f);
                        double d = std::hypot(double(mx) - (pa[0] + t * ex),
                                              double(my) - (pa[1] + t * ey));
                        if (d < best) {
                            best = d;
                            hit = e.edgeId;
                        }
                    }
                }
                if (!shift) app.selEdges.clear();
                if (hit > 0) {
                    if (shift && app.selEdges.count(hit)) {
                        app.selEdges.erase(hit);
                    } else {
                        app.selEdges.insert(hit);
                    }
                }
                rebuildBuffers(app);
            } else {
                // Face picking: pick pass + pixel read. Plain click
                // replaces the selection, shift+click extends/toggles;
                // right-click opens the context popup for the hit face.
                glViewport(0, 0, fbw, fbh);
                int hit =
                    pickFace(app, flatProg, mvp, int(mx), int(my), fbw, fbh);
                if (rClicked) {
                    if (hit > 0) {
                        if (!app.selFaces.count(hit)) {
                            app.selFaces = {hit};
                        }
                        app.activeFace = hit;
                        app.openFacePopup = true;
                    }
                } else if (shift) {
                    if (hit > 0) {
                        if (app.selFaces.count(hit)) {
                            app.selFaces.erase(hit);
                            if (app.activeFace == hit) {
                                app.activeFace = app.selFaces.empty()
                                                     ? 0
                                                     : *app.selFaces.begin();
                            }
                        } else {
                            app.selFaces.insert(hit);
                            app.activeFace = hit;
                        }
                    }
                } else {
                    if (hit > 0 && !(app.selFaces.size() == 1 &&
                                     app.selFaces.count(hit))) {
                        app.selFaces = {hit};
                        app.activeFace = hit;
                    } else {
                        app.selFaces.clear();
                        app.activeFace = 0;
                    }
                }
                rebuildBuffers(app);
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        drawUi(app);
        drawOverlay(app);
        drawFacePopup(app);
        ImGui::Render();

        glViewport(0, 0, fbw, fbh);
        glClearColor(app.bgColor[0], app.bgColor[1], app.bgColor[2], 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        const bool wantFill = app.showFill && app.shadingMode != 2;
        const bool wantWire = app.showWire && app.shadingMode != 1;
        const bool flatFill = app.shadingMode == 3;
        if (app.hasModel && wantFill && app.fill.count) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 1.0f);
            GLuint prog = flatFill ? flatProg : litProg;
            glUseProgram(prog);
            glUniformMatrix4fv(glGetUniformLocation(prog, "uMVP"), 1,
                               GL_FALSE, mvp.m);
            if (flatFill) {
                glUniform1f(glGetUniformLocation(prog, "uMix"), 0.0f);
            } else {
                glUniformMatrix4fv(glGetUniformLocation(prog, "uMV"), 1,
                                   GL_FALSE, view.m);
                glUniform1f(glGetUniformLocation(prog, "uAmbient"),
                            app.lightAmbient);
                glUniform1f(glGetUniformLocation(prog, "uDiffuse"),
                            app.lightDiffuse);
                glUniform1f(glGetUniformLocation(prog, "uRim"), app.lightRim);
            }
            glBindVertexArray(app.fill.vao);
            glDrawArrays(GL_TRIANGLES, 0, app.fill.count);
            glDisable(GL_POLYGON_OFFSET_FILL);
        }
        glUseProgram(flatProg);
        glUniformMatrix4fv(glGetUniformLocation(flatProg, "uMVP"), 1, GL_FALSE,
                           mvp.m);
        GLint uMix = glGetUniformLocation(flatProg, "uMix");
        GLint uColor = glGetUniformLocation(flatProg, "uColor");
        if (app.hasModel && wantWire && app.wire.count) {
            glUniform1f(uMix, 1.0f);  // live wireframe colour, no rebuild
            glUniform3fv(uColor, 1, app.wireColor);
            glBindVertexArray(app.wire.vao);
            glDrawArrays(GL_LINES, 0, app.wire.count);
        }
        glUniform1f(uMix, 0.0f);
        if (app.hasModel && app.showBrepEdges && app.brep.count) {
            glLineWidth(2.0f);
            glBindVertexArray(app.brep.vao);
            glDrawArrays(GL_LINES, 0, app.brep.count);
            glLineWidth(1.0f);
        }
        if (app.hasModel && app.showVerts && app.verts.count) {
            glDisable(GL_DEPTH_TEST);
            glPointSize(5.0f * gUiScale);
            glUniform1f(uMix, 1.0f);
            glUniform3fv(uColor, 1, app.vertColor);
            glBindVertexArray(app.verts.vao);
            glDrawArrays(GL_POINTS, 0, app.verts.count);
            glUniform1f(uMix, 0.0f);
            glEnable(GL_DEPTH_TEST);
        }
        if (((app.mode == Mode::LoopCut && app.hoverValid) ||
             app.mode == Mode::Bridge) &&
            app.preview.count) {
            glDisable(GL_DEPTH_TEST);
            glLineWidth(3.0f);
            glBindVertexArray(app.preview.vao);
            glDrawArrays(GL_LINES, 0, app.preview.count);
            glLineWidth(1.0f);
            glEnable(GL_DEPTH_TEST);
        }
        glBindVertexArray(0);

        if (app.mutatedThisFrame) logLine("frame: ui/render done");
        // One undo step per GESTURE: a slider drag or a burst of wheel
        // steps coalesces (active widget, or mutations within 0.35s).
        {
            double now = glfwGetTime();
            if (app.mutatedThisFrame && !app.gestureActive) {
                app.undoStack.push_back(app.preFrame);
                if (app.undoStack.size() > 100) {
                    app.undoStack.erase(app.undoStack.begin());
                }
                logLine("frame: undo snapshot pushed (%zu)",
                        app.undoStack.size());
            }
            if (app.mutatedThisFrame || ImGui::IsAnyItemActive()) {
                app.lastMutationTime = now;
            }
            app.gestureActive = app.mutatedThisFrame ||
                                ImGui::IsAnyItemActive() ||
                                (now - app.lastMutationTime < 0.35);
        }

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
