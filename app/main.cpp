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
#include "weft/remap.hpp"
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

static Mat4 matOrtho(float halfH, float aspect, float zn, float zf) {
    Mat4 m{};
    const float halfW = halfH * aspect;
    m.m[0] = 1.0f / halfW;
    m.m[5] = 1.0f / halfH;
    m.m[10] = -2.0f / (zf - zn);
    m.m[14] = -(zf + zn) / (zf - zn);
    m.m[15] = 1.0f;
    return m;
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

// World-space ray through a screen pixel (must match matPerspective's
// 42-degree vertical fov and matLookAt's z-up view used for the mvp).
static Vec3 mouseRay(const Camera& cam, double mx, double my, int fbw,
                     int fbh) {
    Vec3 eye = cam.eye();
    Vec3 f = norm(sub(cam.target, eye));
    Vec3 s = norm(cross(f, {0, 0, 1}));
    Vec3 u = cross(s, f);
    float th = std::tan(42.0f * float(M_PI) / 360.0f);
    float aspect = fbh > 0 ? float(fbw) / fbh : 1.6f;
    float px = (2.0f * float(mx) / std::max(1, fbw) - 1.0f) * th * aspect;
    float py = (1.0f - 2.0f * float(my) / std::max(1, fbh)) * th;
    return norm({f.x + s.x * px + u.x * py, f.y + s.y * px + u.y * py,
                 f.z + s.z * px + u.z * py});
}

enum class Mode { Idle, LoopCut, Bridge, Grab };
// Selection modes on 1-6 (Blender-style): mesh verts, mesh edges, mesh
// faces (polygons), B-rep feature edges, B-rep elements (faces), whole
// objects (solids).
enum class SelectMode { Vert, MeshEdge, Poly, Edge, Face, Object };

struct App {
    // Document.
    std::string sourcePath;
    std::string recipePath;  // <model>.recipe next to the source
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
    // Polygon mode: indices into mesh.polygons (cleared on regenerate —
    // indices are only meaningful for the current mesh). Selects ANY
    // polygon, including bridge/fill strips that carry no B-rep face.
    std::set<size_t> selPolys;
    // Vert / mesh-edge modes: direct mesh element selection (cleared on
    // regenerate with selPolys — indices belong to the current mesh).
    std::set<uint32_t> selVerts;
    int64_t hoverVert = -1;
    std::set<uint64_t> selMeshEdges;  // (min vert << 32) | max vert
    uint64_t hoverMeshEdge = UINT64_MAX;
    // Vert -> owning B-rep faces, for occlusion tests against the pick
    // buffer (a vert/edge is selectable only where ITS face is what the
    // camera actually sees — no selecting through the mesh).
    std::vector<std::vector<int>> vertFaces;
    int activeFace = 0;
    int hoverFace = 0;  // pre-click feedback (face mode, idle)
    // Contiguous fill-buffer runs per face, for tint-only highlight draws.
    std::vector<std::array<int, 3>> fillSegs;  // {fid, firstVert, count}
    // Per-polygon fill-buffer run {firstVert, count} ({-1,0} if hidden).
    std::vector<std::array<int, 2>> polyFillRange;
    bool dirty = false;  // regenerate this frame
    std::set<int> hiddenFaces;
    bool openFacePopup = false;  // context popup requested at the cursor

    // Undo: recipe snapshots, one per edit gesture (drags coalesce).
    std::vector<weft::Recipe> undoStack;
    std::vector<weft::Recipe> redoStack;
    weft::Recipe preFrame;
    bool mutatedThisFrame = false;
    bool changedLastFrame = false;

    // Keyboard-centric editing state.
    Mode mode = Mode::Idle;
    // Pie menus (Blender-style): Tab = select modes, Q = tools. Opened at
    // the cursor; the sector under the mouse commits on click.
    int pieKind = -1;  // -1 closed, 0 = modes, 1 = tools
    float pieCenter[2] = {0, 0};
    // Blender-style hold-to-choose: while the opening key is held the
    // pie is live and RELEASING over a sector commits; a quick tap
    // leaves it open for a click instead.
    bool pieHold = false;
    weft::ManualOp hoverOp;    // loop-cut candidate under the cursor
    bool hoverValid = false;

    // Direct manipulation. Loop slide: dragging right after a cut keeps
    // re-parameterizing its t along the split edge (screen projection).
    int slideOp = -1;  // index into recipe.ops, -1 = not sliding
    float slideA[2] = {0, 0}, slideB[2] = {0, 0};  // split edge on screen
    bool slideFlipped = false;  // hover chose the mirrored ordering
    // Vertex grab (G): the nudge op being dragged; the drag plane faces
    // the camera through the vertex's start position, and every hit
    // re-projects exactly onto the CAD face (snapToFace).
    int grabOp = -1;
    std::array<double, 3> grabStart{};

    // Bridge tool: open boundary loops of the current mesh, each mapped to
    // its nearest B-rep edge (the stable id recorded in the op).
    std::vector<std::vector<uint32_t>> bLoops;
    std::vector<int> bLoopEdge;
    int hoverLoop = -1;
    int bridgeFirstEdge = 0;  // first clicked loop's edge id (0 = none yet)

    // STEP hot-reload: watch the source file and re-import when the CAD
    // app re-exports over it. The in-memory recipe is remapped onto the
    // new B-rep geometrically (see weft/remap.hpp), so overrides follow
    // their features even when face/edge ids shuffle. Debounced: reload
    // only once the mtime stops moving (the exporter finished writing).
    bool watchSource = true;
    std::filesystem::file_time_type sourceMtime{};
    bool reloadPending = false;
    std::filesystem::file_time_type pendingMtime{};
    double pendingSince = 0.0;

    // Blender live link: every regenerate mirrors the mesh to this OBJ
    // (atomic tmp+rename); blender/weft_link.py watches it and reimports,
    // keeping CAD face ids as the "weft_face" face attribute.
    bool liveLink = false;
    std::string livePath;  // <data dir>/weft_live.obj, set at startup

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
    // Problem overlay: open edges (red) and non-manifold edges (magenta)
    // rebuilt after every regenerate — the trust meter for game export.
    Buffer problems;
    int openEdgeCount = 0, multiEdgeCount = 0;
    bool showProblems = true;
    // Quality heatmap: tint polys by worst corner angle vs the regular
    // polygon's — pinches and slivers glow before they reach the DCC.
    bool qualityView = false;
    // Fit-to-budget: target polygon count for the density-scale solver.
    int budgetTarget = 5000;
    // Export shaping for game engines.
    bool exportTriangulate = false;
    bool exportYUp = false;
    float exportScale = 1.0f;
    // Orthographic projection (numpad 5 toggles; 1/3/7 set views).
    bool orthoView = false;
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

// Worst corner angle of a polygon relative to its regular ideal: 1.0 is
// perfectly regular, 0 collapses. Slivers and pinches score low.
static double polyQuality(const weft::PolyMesh& m, size_t i) {
    const auto& poly = m.polygons[i];
    const size_t n = poly.size();
    if (n < 3) return 0.0;
    const double ideal = M_PI * (1.0 - 2.0 / double(n));
    double worst = 1.0;
    for (size_t k = 0; k < n; ++k) {
        const auto& a = m.vertices[poly[(k + n - 1) % n]];
        const auto& b = m.vertices[poly[k]];
        const auto& c = m.vertices[poly[(k + 1) % n]];
        double ux = a[0] - b[0], uy = a[1] - b[1], uz = a[2] - b[2];
        double vx = c[0] - b[0], vy = c[1] - b[1], vz = c[2] - b[2];
        double lu = std::sqrt(ux * ux + uy * uy + uz * uz);
        double lv = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (lu < 1e-12 || lv < 1e-12) return 0.0;
        double cosA = std::clamp(
            (ux * vx + uy * vy + uz * vz) / (lu * lv), -1.0, 1.0);
        worst = std::min(worst, std::acos(cosA) / ideal);
    }
    return std::clamp(worst, 0.0, 1.0);
}

static std::array<float, 3> heatColor(double q) {
    // neutral green-grey (good) -> orange (mediocre) -> red (sliver)
    const std::array<float, 3> good{0.55f, 0.68f, 0.55f};
    const std::array<float, 3> mid{0.95f, 0.62f, 0.18f};
    const std::array<float, 3> bad{0.92f, 0.18f, 0.14f};
    auto lerp3 = [](const std::array<float, 3>& x,
                    const std::array<float, 3>& y, float t) {
        return std::array<float, 3>{x[0] + (y[0] - x[0]) * t,
                                    x[1] + (y[1] - x[1]) * t,
                                    x[2] + (y[2] - x[2]) * t};
    };
    if (q >= 0.55) {
        return lerp3(mid, good,
                     float(std::min(1.0, (q - 0.55) / 0.35)));
    }
    return lerp3(bad, mid, float(std::max(0.0, (q - 0.15) / 0.4)));
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
    app.fillSegs.clear();
    app.polyFillRange.assign(m.polygons.size(), {-1, 0});
    for (size_t i = 0; i < m.polygons.size(); ++i) {
        int fid = m.polygonFaceId[i];
        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
        int segStart = int(fill.size() / 6);
        app.polyFillRange[i] = {segStart, 0};
        if (!app.fillSegs.empty() && app.fillSegs.back()[0] == fid &&
            app.fillSegs.back()[1] + app.fillSegs.back()[2] == segStart) {
            // extended below
        } else {
            app.fillSegs.push_back({fid, segStart, 0});
        }
        const weft::FaceInfo& info =
            fid > 0 ? app.analysis.faces[fid - 1] : kBridgeInfo;
        std::array<float, 3> col =
            app.qualityView
                ? heatColor(polyQuality(m, i))
                : faceColor(info, fid > 0 && app.selFaces.count(fid) > 0);
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
        app.fillSegs.back()[2] = int(fill.size() / 6) - app.fillSegs.back()[1];
        app.polyFillRange[i][1] = int(fill.size() / 6) - app.polyFillRange[i][0];
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
        // Hidden objects hide their feature edges too: skip edges whose
        // owner faces are all hidden.
        if (!info.faceIds.empty()) {
            bool allHidden = true;
            for (int fid : info.faceIds) {
                if (!app.hiddenFaces.count(fid)) {
                    allHidden = false;
                    break;
                }
            }
            if (allHidden) continue;
        }
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

// Scan the final mesh for open and non-manifold (multiply-used) directed
// edges and rebuild the red/magenta overlay lines. This is the same test
// the export pipeline cares about: zero of both = watertight.
static void updateProblems(App& app) {
    app.openEdgeCount = 0;
    app.multiEdgeCount = 0;
    std::map<std::pair<uint32_t, uint32_t>, int> dir;
    for (const auto& poly : app.mesh.polygons) {
        for (size_t i = 0; i < poly.size(); ++i) {
            ++dir[{poly[i], poly[(i + 1) % poly.size()]}];
        }
    }
    std::vector<float> lines;
    auto pushEdge = [&](uint32_t a, uint32_t b, float r, float g,
                        float bl) {
        for (uint32_t v : {a, b}) {
            lines.push_back(float(app.mesh.vertices[v][0]));
            lines.push_back(float(app.mesh.vertices[v][1]));
            lines.push_back(float(app.mesh.vertices[v][2]));
            lines.push_back(r);
            lines.push_back(g);
            lines.push_back(bl);
        }
    };
    for (const auto& [e, count] : dir) {
        if (count > 1) {
            pushEdge(e.first, e.second, 1.0f, 0.2f, 0.9f);  // magenta
            app.multiEdgeCount += count - 1;
        } else if (!dir.count({e.second, e.first})) {
            pushEdge(e.first, e.second, 1.0f, 0.25f, 0.15f);  // red
            ++app.openEdgeCount;
        }
    }
    app.problems.upload(lines);
    if (lines.empty()) app.problems.count = 0;
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
        app.selPolys.clear();  // mesh indices died with the old mesh
        app.selVerts.clear();
        app.selMeshEdges.clear();
        app.hoverVert = -1;
        app.hoverMeshEdge = UINT64_MAX;
        app.vertFaces.assign(app.mesh.vertexCount(), {});
        for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
            int fid = app.mesh.polygonFaceId[p];
            if (fid <= 0) continue;
            for (uint32_t v : app.mesh.polygons[p]) {
                auto& vf = app.vertFaces[v];
                if (std::find(vf.begin(), vf.end(), fid) == vf.end()) {
                    vf.push_back(fid);
                }
            }
        }
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
    updateProblems(app);

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

    // Blender live link: mirror every result to the watched OBJ. Written
    // to a temp file and renamed into place, so the addon's mtime poll
    // never reads a half-written export.
    if (app.liveLink && !app.livePath.empty()) {
        try {
            std::string tmp = app.livePath + ".tmp";
            weft::writeObj(app.mesh, tmp, &app.analysis.solidFaces);
            std::filesystem::rename(tmp, app.livePath);
        } catch (const std::exception& e) {
            app.status = std::string("live link write failed: ") + e.what();
        }
    }
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
        std::error_code ec;
        app.sourceMtime = std::filesystem::last_write_time(path, ec);
        app.reloadPending = false;
        app.hasModel = true;
        app.selFaces.clear();
        app.selEdges.clear();
        app.activeFace = 0;
        app.undoStack.clear();
        app.genCache.clear();
        app.recipe = {};
        // New sessions solve curvature adaptively (deviation/angle drive
        // each edge's count); saved recipes bring their own flag back.
        app.recipe.settings.defaults.adaptive = true;
        regenerate(app);
        frameModel(app);
        app.status = path + ": " + std::to_string(app.model.faceCount()) +
                     " faces, " + std::to_string(app.model.edgeCount()) +
                     " edges";
        // Session persistence: a recipe next to the model auto-loads, and
        // ctrl+S writes back to the same file.
        size_t dot = path.find_last_of('.');
        app.recipePath = (dot == std::string::npos ? path
                                                   : path.substr(0, dot)) +
                         ".recipe";
        std::snprintf(app.recipeBuf, sizeof app.recipeBuf, "%s",
                      app.recipePath.c_str());
        if (std::filesystem::exists(app.recipePath)) {
            try {
                app.recipe = weft::loadRecipe(app.recipePath);
                regenerate(app);
                app.status += "  (recipe loaded)";
                logLine("load: applied %s", app.recipePath.c_str());
            } catch (const std::exception& e) {
                app.status = std::string("recipe load failed: ") + e.what();
            }
        }
    } catch (const std::exception& e) {
        app.status = std::string("load failed: ") + e.what();
    }
}

// Hot-reload: the source STEP changed on disk (the CAD app re-exported
// over it). Re-import, remap the LIVE recipe onto the new B-rep by
// geometric signature — the recipe file on disk is not re-read, so
// unsaved edits survive — and regenerate; the Blender live link then
// pushes the result downstream. Selection and hidden faces follow the
// same mapping. The undo stack refers to old ids, so it resets.
static void reloadModel(App& app) {
    logLine("hot-reload: %s", app.sourcePath.c_str());
    try {
        weft::Model fresh = weft::loadStep(app.sourcePath);
        weft::Analysis freshAnalysis = weft::analyze(fresh);
        weft::RemapReport rep;
        weft::Recipe remapped =
            weft::remapRecipe(app.recipe, app.model, app.analysis, fresh,
                              freshAnalysis, &rep);
        app.model = std::move(fresh);
        app.analysis = std::move(freshAnalysis);
        app.recipe = std::move(remapped);

        auto mapSet = [](std::set<int>& ids, const std::map<int, int>& m) {
            std::set<int> out;
            for (int id : ids) {
                auto it = m.find(id);
                if (it != m.end() && it->second > 0) out.insert(it->second);
            }
            ids = std::move(out);
        };
        mapSet(app.selFaces, rep.faceMap);
        mapSet(app.hiddenFaces, rep.faceMap);
        mapSet(app.selEdges, rep.edgeMap);
        auto af = rep.faceMap.find(app.activeFace);
        app.activeFace = af != rep.faceMap.end() ? af->second : 0;
        if (app.activeFace == 0 && !app.selFaces.empty()) {
            app.activeFace = *app.selFaces.begin();
        }

        app.genCache.clear();
        app.undoStack.clear();
        app.mode = Mode::Idle;
        app.slideOp = -1;
        app.grabOp = -1;
        app.bridgeFirstEdge = 0;
        app.hoverFace = 0;
        app.hoverValid = false;
        regenerate(app);

        int dropped = rep.facesDropped + rep.edgesDropped + rep.opsDropped;
        char msg[256];
        if (dropped == 0) {
            std::snprintf(msg, sizeof msg,
                          "reloaded from disk: %d faces, recipe followed "
                          "(%zu overrides, %zu pins, %zu ops)",
                          app.model.faceCount(),
                          app.recipe.settings.perFace.size(),
                          app.recipe.settings.perEdge.size(),
                          app.recipe.ops.size());
        } else {
            std::snprintf(msg, sizeof msg,
                          "reloaded from disk: %d faces — %d recipe "
                          "reference(s) matched nothing and were dropped",
                          app.model.faceCount(), dropped);
        }
        app.status = msg;
        logLine("hot-reload: done (%d dropped refs)", dropped);
    } catch (const std::exception& e) {
        app.status = std::string("hot-reload failed: ") + e.what();
        logLine("hot-reload: FAILED: %s", e.what());
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

// Kind-aware density nudge: EVERY mesher answers the wheel / [ ] with the
// field that actually drives its density — counts for structured grids,
// boundary totals for plate-web/quad-fill/minimal, deviation scaling for
// the freeform triangulators. Returns the HUD line describing the change.
static std::string adjustFaceDensityOne(App& app, weft::FaceMeshSettings& s,
                                        bool secondary, int steps) {
    weft::MesherKind kind = weft::MesherKind::RevolutionGrid;
    if (app.activeFace > 0) {
        auto it = app.report.faceMesher.find(app.activeFace);
        if (it != app.report.faceMesher.end()) kind = it->second;
    }
    char hud[64] = "";
    auto count = [&](int& v, int lo, const char* name) {
        v = std::max(lo, v + steps);
        s.adaptive = false;  // explicit count = manual
        std::snprintf(hud, sizeof hud, "%s: %d", name, v);
    };
    // Pinned totals coexist with adaptive density — don't clear it.
    auto total = [&](int& v, const char* name) {
        if (v <= 0) v = 16;  // 0 = auto; seed a sensible total first
        v = std::max(4, v + steps);
        std::snprintf(hud, sizeof hud, "%s: %d", name, v);
    };
    // Deviation-style knobs: wheel up = denser = smaller tolerance.
    auto scale = [&](double& v, double lo, double hi, const char* name) {
        v = std::clamp(v * std::pow(0.82, double(steps)), lo, hi);
        std::snprintf(hud, sizeof hud, "%s: %.4g", name, v);
    };
    using MK = weft::MesherKind;
    switch (kind) {
        case MK::RevolutionGrid:
        case MK::DiskCap:
            if (secondary) count(s.axial, 1, "axial");
            else count(s.radial, 3, "radial");
            break;
        case MK::RingJunction:
            if (secondary) count(s.junctionRings, 1, "junction rings");
            else count(s.gridU, 1, "grid u");
            break;
        case MK::AnnulusRing:
            count(s.radial, 3, "loop verts");
            break;
        case MK::PlateWeb:
            if (secondary) count(s.junctionRings, 1, "collar rings");
            else total(s.boundary, "boundary verts");
            break;
        case MK::QuadFill:
            if (secondary) scale(s.chordTolerance, 5e-4, 100.0, "deviation");
            else total(s.boundary, "boundary verts");
            break;
        case MK::MinimalNGon:
            total(s.boundary, "boundary verts");
            break;
        case MK::QuadDominant:
        case MK::Fallback:
            if (secondary) {
                scale(s.angleToleranceDeg, 1.0, 60.0, "angle");
            } else {
                scale(s.chordTolerance, 5e-4, 100.0, "deviation");
            }
            break;
        default:  // PlanarGrid, CoonsGrid
            if (secondary) count(s.gridV, 1, "grid v");
            else count(s.gridU, 1, "grid u");
            break;
    }
    return hud;
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

// Density nudge over the whole selection + HUD readout.
static void adjustFaceDensity(App& app, bool secondary, int steps) {
    std::string hud;
    editSelected(app, [&](weft::FaceMeshSettings& s) {
        hud = adjustFaceDensityOne(app, s, secondary, steps);
    });
    if (!hud.empty()) {
        std::snprintf(app.hudText, sizeof app.hudText, "%s", hud.c_str());
        app.hudUntil = glfwGetTime() + 0.9;
    }
}

// Shared verbs (key handlers + pie menus call the same code).
static void setSelectMode(App& app, SelectMode next) {
    if (next == app.selectMode) return;
    app.selectMode = next;
    app.selFaces.clear();
    app.selEdges.clear();
    app.selPolys.clear();
    app.selVerts.clear();
    app.selMeshEdges.clear();
    app.activeFace = 0;
    app.hoverFace = 0;
    app.hoverVert = -1;
    app.hoverMeshEdge = UINT64_MAX;
    rebuildBuffers(app);
    app.status = next == SelectMode::Vert       ? "vert mode"
                 : next == SelectMode::MeshEdge ? "edge mode"
                 : next == SelectMode::Poly     ? "face mode"
                 : next == SelectMode::Edge     ? "feature edge mode"
                 : next == SelectMode::Face     ? "element mode"
                                                : "object mode";
}

// Object mode: a click selects the whole solid the face belongs to, so
// every face-scoped tool (density, hide, delete) acts per body.
static void selectSolidOf(App& app, int fid, bool extend) {
    if (!extend) app.selFaces.clear();
    for (const auto& fids : app.analysis.solidFaces) {
        if (std::find(fids.begin(), fids.end(), fid) != fids.end()) {
            for (int f : fids) app.selFaces.insert(f);
        }
    }
    app.activeFace = fid;
}

static uint64_t meshEdgeKey(uint32_t a, uint32_t b) {
    return (uint64_t(std::min(a, b)) << 32) | std::max(a, b);
}

static void deleteSelection(App& app) {
    // Polygon surgery: recorded ops anchored to the poly's world
    // centroid, replayed by nearest-centroid match. Deleting verts or
    // mesh edges deletes their adjacent polygons, Blender-style.
    auto deletePoly = [&](size_t p) {
        double c[3] = {0, 0, 0};
        for (uint32_t v : app.mesh.polygons[p]) {
            c[0] += app.mesh.vertices[v][0];
            c[1] += app.mesh.vertices[v][1];
            c[2] += app.mesh.vertices[v][2];
        }
        double k = double(app.mesh.polygons[p].size());
        weft::ManualOp op;
        op.kind = weft::ManualOp::Kind::DeletePoly;
        op.u = c[0] / k;
        op.v = c[1] / k;
        op.t = c[2] / k;
        app.recipe.ops.push_back(op);
    };
    if (app.selectMode == SelectMode::Poly && !app.selPolys.empty()) {
        for (size_t p : app.selPolys) {
            if (p < app.mesh.polygons.size()) deletePoly(p);
        }
        app.selPolys.clear();
        markDirty(app);
        app.status = "polygon(s) deleted (ctrl+Z undoes, F/J refills)";
    } else if (app.selectMode == SelectMode::Vert && !app.selVerts.empty()) {
        size_t n = 0;
        for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
            for (uint32_t v : app.mesh.polygons[p]) {
                if (app.selVerts.count(v)) {
                    deletePoly(p);
                    ++n;
                    break;
                }
            }
        }
        app.selVerts.clear();
        markDirty(app);
        app.status = std::to_string(n) +
                     " polygon(s) around verts deleted (ctrl+Z undoes)";
    } else if (app.selectMode == SelectMode::MeshEdge &&
               !app.selMeshEdges.empty()) {
        size_t n = 0;
        for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
            const auto& poly = app.mesh.polygons[p];
            bool hit = false;
            for (size_t i = 0; i < poly.size() && !hit; ++i) {
                hit = app.selMeshEdges.count(meshEdgeKey(
                    poly[i], poly[(i + 1) % poly.size()])) != 0;
            }
            if (hit) {
                deletePoly(p);
                ++n;
            }
        }
        app.selMeshEdges.clear();
        markDirty(app);
        app.status = std::to_string(n) +
                     " polygon(s) around edges deleted (ctrl+Z undoes)";
    } else if (!app.selFaces.empty()) {
        editSelected(app,
                     [](weft::FaceMeshSettings& s) { s.exclude = true; });
        app.status = "face(s) deleted (ctrl+Z undoes, J bridges rims)";
    }
}

static void hideSelection(App& app, bool showAll) {
    if (showAll) {
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

static void toggleBridgeMode(App& app) {
    app.mode = app.mode == Mode::Bridge ? Mode::Idle : Mode::Bridge;
    app.bridgeFirstEdge = 0;
    app.hoverLoop = -1;
    if (app.mode == Mode::Bridge && app.bLoops.empty()) {
        app.status = "bridge: no open boundaries (delete a face first)";
    }
}

static void toggleLoopCutMode(App& app) {
    app.mode = app.mode == Mode::LoopCut ? Mode::Idle : Mode::LoopCut;
    app.hoverValid = false;
    app.slideOp = -1;
}

// Adjust a per-edge division pin for every selected edge (edge mode).
// Multi-edge density edit: the selection is treated as ONE chain/loop —
// a circle split into arcs by seams behaves like the closed loop it is.
// The typed value (or the nudged total) is the COMBINED vertex count,
// distributed across the selected edges proportionally to arc length.
// Returns the resulting total for the HUD.
static int adjustSelectedEdges(App& app, int typedValue, int delta) {
    if (app.selEdges.empty()) return 0;
    const int count = int(app.selEdges.size());
    auto currentOf = [&](int eid) {
        auto pin = app.recipe.settings.perEdge.find(eid);
        if (pin != app.recipe.settings.perEdge.end()) return pin->second;
        auto it = app.report.edgeDivisions.find(eid);
        return it != app.report.edgeDivisions.end() ? it->second : 8;
    };
    int total = 0;
    double lengthSum = 0;
    std::vector<std::pair<int, double>> edges;  // eid, length
    for (int eid : app.selEdges) {
        total += currentOf(eid);
        double len = eid >= 1 && eid <= int(app.analysis.edges.size())
                         ? app.analysis.edges[eid - 1].length
                         : 0.0;
        if (len <= 0) len = 1.0;
        edges.push_back({eid, len});
        lengthSum += len;
    }
    int target = typedValue > 0 ? typedValue : total + delta;
    target = std::max(count, target);  // at least one span per edge

    // Largest-remainder split: shares sum EXACTLY to the target.
    std::vector<int> share(edges.size(), 1);
    std::vector<std::pair<double, size_t>> remainder;
    int assigned = 0;
    for (size_t i = 0; i < edges.size(); ++i) {
        double exact = target * edges[i].second / lengthSum;
        share[i] = std::max(1, int(exact));
        assigned += share[i];
        remainder.push_back({exact - int(exact), i});
    }
    std::sort(remainder.rbegin(), remainder.rend());
    for (size_t k = 0; assigned < target; ++k) {
        ++share[remainder[k % remainder.size()].second];
        ++assigned;
    }
    for (size_t k = 0; assigned > target; ++k) {
        size_t i = remainder[remainder.size() - 1 - (k % remainder.size())]
                       .second;
        if (share[i] > 1) {
            --share[i];
            --assigned;
        }
    }
    for (size_t i = 0; i < edges.size(); ++i) {
        app.recipe.settings.perEdge[edges[i].first] = share[i];
    }
    markDirty(app);
    return target;
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
    weft::ManualOp op;
    op.faceId = aa.faceId;
    // Anchor the op at the hovered edge's TRUE midpoint (projected onto
    // the face). Averaging the two endpoint anchors instead wraps to the
    // far side on periodic surfaces (a cylinder's u seam), and the replay
    // then seeds from some other edge with mixed orientation — the
    // hovered edge is the orientation the cut must cross.
    {
        const auto& pa = m.vertices[bestA];
        const auto& pb = m.vertices[bestB];
        std::array<double, 3> mid = {0.5 * (pa[0] + pb[0]),
                                     0.5 * (pa[1] + pb[1]),
                                     0.5 * (pa[2] + pb[2])};
        try {
            weft::Anchor snapped = weft::snapToFace(app.model, aa.faceId, mid);
            op.u = snapped.u;
            op.v = snapped.v;
        } catch (...) {
            const weft::Anchor& ab = m.anchors[bestB];
            op.u = 0.5 * (aa.u + ab.u);
            op.v = 0.5 * (aa.v + ab.v);
        }
    }
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
    bool usedFlipped = false;
    if (!linesFlipped.empty() &&
        (lines.empty() || distFlipped + 1.0 < dist)) {
        op = flipped;
        lines = std::move(linesFlipped);
        usedFlipped = true;
    }
    if (lines.empty()) return;

    // Remember the split edge on screen: a press-drag right after the cut
    // slides t by re-projecting the cursor onto this segment.
    float pa[3] = {0, 0, -1}, pb[3] = {0, 0, -1};
    projectPoint(mvp, m.vertices[bestA], fbw, fbh, pa);
    projectPoint(mvp, m.vertices[bestB], fbw, fbh, pb);
    app.slideA[0] = pa[0];
    app.slideA[1] = pa[1];
    app.slideB[0] = pb[0];
    app.slideB[1] = pb[1];
    app.slideFlipped = usedFlipped;

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

// Nearest projected B-rep edge to the cursor (same picking as edge-select
// mode). The bridge tool records THIS edge, not the loop's canonical one,
// so two picks on the same boundary loop can name its two sides.
static int nearestBrepEdge(App& app, const Mat4& mvp, double mx, double my,
                           int fbw, int fbh, double maxPx) {
    int hit = 0;
    double best = maxPx;
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
                                        (float(my) - pa[1]) * ey) / len2,
                                       0.0f, 1.0f);
            double d = std::hypot(double(mx) - (pa[0] + t * ex),
                                  double(my) - (pa[1] + t * ey));
            if (d < best) {
                best = d;
                hit = e.edgeId;
            }
        }
    }
    return hit;
}

// Polygon picking (poly select mode): the front-most polygon whose screen
// projection contains the cursor. Runs over the raw polygons — bridge and
// fill strips (no B-rep face) are selectable like any other.
static int pickPolygon(App& app, const Mat4& mvp, double mx, double my,
                       int fbw, int fbh) {
    const weft::PolyMesh& m = app.mesh;
    int best = -1;
    double bestDepth = 1e30;
    std::vector<std::array<float, 3>> proj;
    for (size_t p = 0; p < m.polygons.size(); ++p) {
        int fid = m.polygonFaceId[p];
        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
        const auto& poly = m.polygons[p];
        proj.assign(poly.size(), {0, 0, -1});
        bool behind = false;
        for (size_t i = 0; i < poly.size(); ++i) {
            projectPoint(mvp, m.vertices[poly[i]], fbw, fbh,
                         proj[i].data());
            if (proj[i][2] <= 0) { behind = true; break; }
        }
        if (behind) continue;
        auto inTri = [&](const std::array<float, 3>& a,
                         const std::array<float, 3>& b,
                         const std::array<float, 3>& c) {
            auto cross = [&](const std::array<float, 3>& o,
                             const std::array<float, 3>& q) {
                return (q[0] - o[0]) * (float(my) - o[1]) -
                       (q[1] - o[1]) * (float(mx) - o[0]);
            };
            float d1 = cross(a, b), d2 = cross(b, c), d3 = cross(c, a);
            bool neg = d1 < 0 || d2 < 0 || d3 < 0;
            bool pos = d1 > 0 || d2 > 0 || d3 > 0;
            return !(neg && pos);
        };
        for (size_t k = 1; k + 1 < poly.size(); ++k) {
            if (!inTri(proj[0], proj[k], proj[k + 1])) continue;
            double depth = (proj[0][2] + proj[k][2] + proj[k + 1][2]) / 3;
            if (depth < bestDepth) {
                bestDepth = depth;
                best = int(p);
            }
            break;
        }
    }
    return best;
}

// Face ids the camera actually sees over a screen rect, read from one
// render of the pick buffer. This is the occlusion oracle for every
// selection mode: a vert/edge is only pickable where one of ITS faces is
// the front-most pixel — nothing selects through the mesh.
struct PickRect {
    int x0 = 0, y0 = 0, w = 0, h = 0;
    std::vector<int> fid;  // row-major from (x0,y0); 0 = background
    int at(double x, double y) const {
        int ix = int(std::lround(x)) - x0;
        int iy = int(std::lround(y)) - y0;
        if (ix < 0 || iy < 0 || ix >= w || iy >= h) return 0;
        return fid[size_t(iy) * w + ix];
    }
};

static PickRect readPickRect(App& app, GLuint flatProg, const Mat4& mvp,
                             int x0, int y0, int x1, int y1, int fbw,
                             int fbh) {
    PickRect pr;
    if (!app.hasModel || app.pick.count == 0) return pr;
    glViewport(0, 0, fbw, fbh);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glUseProgram(flatProg);
    glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.0f);
    glUniformMatrix4fv(glGetUniformLocation(flatProg, "uMVP"), 1, GL_FALSE,
                       mvp.m);
    glBindVertexArray(app.pick.vao);
    glDrawArrays(GL_TRIANGLES, 0, app.pick.count);
    glBindVertexArray(0);
    glFinish();
    if (x0 > x1) std::swap(x0, x1);
    if (y0 > y1) std::swap(y0, y1);
    x0 = std::clamp(x0, 0, fbw - 1);
    x1 = std::clamp(x1, 0, fbw - 1);
    y0 = std::clamp(y0, 0, fbh - 1);
    y1 = std::clamp(y1, 0, fbh - 1);
    pr.x0 = x0;
    pr.y0 = y0;
    pr.w = x1 - x0 + 1;
    pr.h = y1 - y0 + 1;
    std::vector<unsigned char> px(size_t(pr.w) * pr.h * 4);
    glReadPixels(x0, fbh - 1 - y1, pr.w, pr.h, GL_RGBA, GL_UNSIGNED_BYTE,
                 px.data());
    pr.fid.assign(size_t(pr.w) * pr.h, 0);
    // glReadPixels rows run bottom-up; PickRect::at expects top-down.
    for (int iy = 0; iy < pr.h; ++iy) {
        for (int ix = 0; ix < pr.w; ++ix) {
            size_t src = (size_t(pr.h - 1 - iy) * pr.w + ix) * 4;
            if (px[src + 2] != 170) continue;
            pr.fid[size_t(iy) * pr.w + ix] =
                int(px[src]) + (int(px[src + 1]) << 8);
        }
    }
    return pr;
}

static bool vertVisibleAt(const App& app, uint32_t v, int fidAtPixel) {
    if (fidAtPixel <= 0 || v >= app.vertFaces.size()) return false;
    const auto& vf = app.vertFaces[v];
    return std::find(vf.begin(), vf.end(), fidAtPixel) != vf.end();
}

// Nearest VISIBLE mesh vertex / polygon edge to the cursor, for the vert
// and mesh-edge selection modes: hidden faces are skipped and the pick
// buffer must show one of the element's own faces at its pixel.
static int64_t pickMeshVert(App& app, const Mat4& mvp, double mx, double my,
                            int fbw, int fbh, const PickRect& pr) {
    const weft::PolyMesh& m = app.mesh;
    double best = 12.0 * gUiScale;  // px
    int64_t bestV = -1;
    std::vector<bool> seen(m.vertexCount(), false);
    for (size_t p = 0; p < m.polygons.size(); ++p) {
        int fid = m.polygonFaceId[p];
        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
        for (uint32_t v : m.polygons[p]) {
            if (seen[v]) continue;
            seen[v] = true;
            float sp[3] = {0, 0, -1};
            projectPoint(mvp, m.vertices[v], fbw, fbh, sp);
            if (sp[2] <= 0) continue;
            double d = std::hypot(sp[0] - mx, sp[1] - my);
            if (d < best && vertVisibleAt(app, v, pr.at(sp[0], sp[1]))) {
                best = d;
                bestV = int64_t(v);
            }
        }
    }
    return bestV;
}

static uint64_t pickMeshEdge(App& app, const Mat4& mvp, double mx, double my,
                             int fbw, int fbh, const PickRect& pr) {
    const weft::PolyMesh& m = app.mesh;
    double best = 10.0 * gUiScale;  // px
    uint64_t bestE = UINT64_MAX;
    for (size_t p = 0; p < m.polygons.size(); ++p) {
        int fid = m.polygonFaceId[p];
        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
        const auto& poly = m.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
            float pa[3] = {0, 0, -1}, pb[3] = {0, 0, -1};
            projectPoint(mvp, m.vertices[a], fbw, fbh, pa);
            projectPoint(mvp, m.vertices[b], fbw, fbh, pb);
            if (pa[2] <= 0 || pb[2] <= 0) continue;
            float ex = pb[0] - pa[0], ey = pb[1] - pa[1];
            float len2 = ex * ex + ey * ey;
            float t = len2 < 1e-6f
                          ? 0.0f
                          : std::clamp(((float(mx) - pa[0]) * ex +
                                        (float(my) - pa[1]) * ey) /
                                           len2,
                                       0.0f, 1.0f);
            double px = pa[0] + t * ex, py = pa[1] + t * ey;
            double d = std::hypot(double(mx) - px, double(my) - py);
            int atPx = pr.at(px, py);
            if (d < best && vertVisibleAt(app, a, atPx) &&
                vertVisibleAt(app, b, atPx)) {
                best = d;
                bestE = meshEdgeKey(a, b);
            }
        }
    }
    return bestE;
}

// Vertex grab (G): pick the interior vertex nearest the cursor and start a
// NudgeVertex op. Only face-anchored vertices qualify — border vertices
// belong to shared B-rep edges and are owned by density + conformity.
static void startVertexGrab(App& app, const Mat4& mvp, double mx, double my,
                            int fbw, int fbh) {
    const weft::PolyMesh& m = app.mesh;
    double best = 30.0 * gUiScale;  // px
    size_t bestV = m.vertexCount();
    for (size_t v = 0; v < m.vertexCount(); ++v) {
        const weft::Anchor& a = m.anchors[v];
        if (a.faceId == 0 || app.hiddenFaces.count(a.faceId)) continue;
        float s[3] = {0, 0, -1};
        projectPoint(mvp, m.vertices[v], fbw, fbh, s);
        if (s[2] <= 0) continue;
        double d = std::hypot(s[0] - mx, s[1] - my);
        if (d < best) {
            best = d;
            bestV = v;
        }
    }
    if (bestV == m.vertexCount()) {
        app.status = "grab: no interior vertex under the cursor (border "
                     "verts are density-driven)";
        return;
    }
    const weft::Anchor& a = m.anchors[bestV];
    weft::ManualOp op;
    op.kind = weft::ManualOp::Kind::NudgeVertex;
    op.faceId = a.faceId;
    op.u = a.u;
    op.v = a.v;
    op.u2 = a.u;
    op.v2 = a.v;
    app.recipe.ops.push_back(op);
    app.grabOp = int(app.recipe.ops.size()) - 1;
    app.grabStart = m.vertices[bestV];
    app.mode = Mode::Grab;
    app.status = "grab: drag on the surface - click commits, esc cancels";
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
    glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.0f);
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

// All face ids whose pick pixels appear inside the given screen rect.
static std::set<int> pickFacesInRect(App& app, GLuint flatProg,
                                     const Mat4& mvp, int x0, int y0, int x1,
                                     int y1, int fbw, int fbh) {
    std::set<int> hits;
    PickRect pr = readPickRect(app, flatProg, mvp, x0, y0, x1, y1, fbw, fbh);
    for (int fid : pr.fid) {
        if (fid > 0) hits.insert(fid);
    }
    return hits;
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
    // Annulus loops and plate-web/quad-fill borders take the radial
    // default too (each closed loop, or each hole circle, proposes it).
    const bool revolved = all || k == MK::RevolutionGrid ||
                          k == MK::DiskCap || k == MK::AnnulusRing ||
                          k == MK::PlateWeb || k == MK::QuadFill;
    const bool grid = all || k == MK::PlanarGrid || k == MK::MinimalNGon ||
                      k == MK::RingJunction || k == MK::CoonsGrid;
    const bool freeform = all || k == MK::QuadDominant || k == MK::Fallback;
    bool ch = false;

    // Curvature-adaptive density: deviation/angle size every curved edge;
    // the manual counts below become floors. Nudging a count via the
    // wheel flips the face back to manual.
    ch |= ImGui::Checkbox("adaptive density (curvature)", &s.adaptive);
    if (freeform || s.adaptive) {
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
    }
    if (freeform) {
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
        if (!all && k == MK::PlateWeb) {
            // Concentric collar rings around each hole ("all" shows this
            // under the grid section already).
            ch |= ImGui::DragInt("junction rings", &s.junctionRings, 0.2f,
                                 1, 32);
            ch |= ImGui::Checkbox("square collars", &s.squareCollar);
        }
        if (!all && (k == MK::PlateWeb || k == MK::QuadFill ||
                     k == MK::MinimalNGon)) {
            // Total verts around the outer loop, length-distributed and
            // pinned (drives the neighbouring walls' shared edges).
            ch |= ImGui::DragInt("boundary verts (0=auto)", &s.boundary,
                                 0.2f, 0, 512);
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

        if (!all && k == MK::CoonsGrid) {
            // Which corner anchors the grid; on triangular patches this
            // moves the corner the fan terminates in.
            int rot = s.coonsRotate;
            if (ImGui::SliderInt("rotate patch", &rot, 0, 3)) {
                s.coonsRotate = rot;
                ch = true;
            }
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
    if (all || kind) {
        // Flat geometry (plane OR flat bspline) collapses to one exact
        // boundary n-gon, holes bridged in — available everywhere since
        // any mesher's face can turn out flat.
        ch |= ImGui::Checkbox("minimal n-gon (flat panels)", &s.minimal);
    }
    if (kind) {  // per-face contexts only
        // Manual mesher choice: auto picks per geometry; forcing one that
        // can't build on the face falls back to triangulation.
        static const char* kMesherItems =
            "auto\0revolution-grid\0disk-cap\0parametric-grid\0"
            "coons-grid\0ring-junction\0quad-dominant\0minimal-ngon\0"
            "fallback-tri\0annulus-ring\0plate-web\0quad-fill\0";
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

// Mode indicator + hotkey reference, floating over the
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
        ImGui::TextDisabled(
            "hover an edge - press cuts, drag slides - R/esc exits");
    } else if (app.mode == Mode::Grab) {
        ImGui::TextColored({1.0f, 0.85f, 0.25f, 1.0f}, "GRAB");
        ImGui::SameLine();
        ImGui::TextDisabled(
            "vertex slides on its CAD face - click commits, esc cancels");
    } else if (app.mode == Mode::Bridge) {
        ImGui::TextColored({1.0f, 0.85f, 0.25f, 1.0f}, "BRIDGE");
        ImGui::SameLine();
        ImGui::TextDisabled(app.bridgeFirstEdge
                                ? "pick the second side - esc restarts"
                                : "pick two loops (or two sides of one) - "
                                  "F fills hovered - [ ] twists (shift+"
                                  "wheel flips side) - J/esc exits");
        if (app.hoverLoop >= 0) {
            ImGui::Text("loop: edge #%d, %zu verts",
                        app.bLoopEdge[app.hoverLoop],
                        app.bLoops[app.hoverLoop].size());
            ImGui::SameLine();
            ImGui::TextDisabled("( [ ] pins the count )");
        }
        if (app.bLoops.empty()) {
            ImGui::TextDisabled(
                "no open boundaries - delete a face first (popup or outliner)");
        }
    } else if (app.selectMode == SelectMode::Vert) {
        ImGui::TextColored({0.55f, 1.0f, 0.7f, 1.0f}, "VERT MODE");
        ImGui::SameLine();
        if (app.selVerts.empty()) {
            ImGui::TextDisabled(
                "click verts (shift extends) - G grabs interior verts");
        } else {
            ImGui::TextDisabled("%zu vert(s) - G grabs", app.selVerts.size());
        }
    } else if (app.selectMode == SelectMode::MeshEdge) {
        ImGui::TextColored({0.55f, 0.9f, 1.0f, 1.0f}, "EDGE MODE");
        ImGui::SameLine();
        if (app.selMeshEdges.empty()) {
            ImGui::TextDisabled("click mesh edges (shift extends)");
        } else {
            ImGui::TextDisabled("%zu edge(s) selected",
                                app.selMeshEdges.size());
        }
    } else if (app.selectMode == SelectMode::Edge) {
        ImGui::TextColored({0.6f, 0.8f, 1.0f, 1.0f}, "FEATURE EDGES");
        ImGui::SameLine();
        if (app.selEdges.empty()) {
            ImGui::TextDisabled("click feature edges (shift extends)");
        } else {
            ImGui::TextDisabled("%zu edge(s) - wheel/[ ] sets loop"
                                " total - J bridges - F fills",
                                app.selEdges.size());
        }
    } else if (app.selectMode == SelectMode::Poly) {
        ImGui::TextColored({0.85f, 0.7f, 1.0f, 1.0f}, "FACE MODE");
        ImGui::SameLine();
        if (app.selPolys.empty()) {
            ImGui::TextDisabled(
                "click polygons (bridge strips too) - X deletes - G grabs");
        } else {
            ImGui::TextDisabled("%zu polygon(s) - X deletes",
                                app.selPolys.size());
        }
    } else if (app.selectMode == SelectMode::Object) {
        ImGui::TextColored({1.0f, 0.75f, 0.45f, 1.0f}, "OBJECT MODE");
        ImGui::SameLine();
        if (app.selFaces.empty()) {
            ImGui::TextDisabled("click a body (shift extends) - X deletes"
                                " - H hides");
        } else {
            ImGui::TextDisabled("%zu face(s) across selected object(s)",
                                app.selFaces.size());
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
        ImGui::TextDisabled("no selection - 1 verts, 2 edges, 3 faces, "
                            "4 feature edges, 5 elements, 6 objects");
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
        "1-6 verts/edges/faces/feature edges/elements/objects\n"
        "Tab mode pie   Q tools pie   (hold + release picks)\n"
        "drag box-select (shift extends)\n"
        "shift+click multi-select   ctrl+Z undo\n"
        "shift+wheel density   ctrl+wheel 2nd axis   ctrl+shift+wheel loops\n"
        "[ ] nudge counts\n"
        "X delete   H/ctrl+H hide   alt+H show all   R loop cut   J bridge\n"
        "G grab vertex   C cap   T tris   M minimal   W wire   B edges\n"
        "numpad 1/3/7 views (ctrl flips)   numpad 5 ortho\n"
        "ctrl+I invert   ctrl+shift+Z redo   F focus   esc");
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
        if (app.hasModel) {
            // Hot-reload: re-export from the CAD app over the same STEP
            // and Weft re-imports it, carrying the recipe across.
            if (ImGui::Checkbox("watch file (reload + remap recipe)",
                                &app.watchSource) &&
                app.watchSource) {
                app.status = "watching " + app.sourcePath;
            }
            // Live link: mirror every regenerate into the watched OBJ that
            // the bundled Blender addon (blender/weft_link.py) reimports.
            if (ImGui::Checkbox("live link (Blender)", &app.liveLink) &&
                app.liveLink) {
                app.dirty = true;  // push the current mesh out right away
                                   // (not a recipe mutation: no undo entry)
                app.status = "live link on - install blender/weft_link.py "
                             "and enable watching";
            }
            if (app.liveLink) {
                ImGui::SameLine();
                ImGui::TextDisabled("(?)");
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("writes %s on every change;\n"
                                      "the addon polls it and swaps the "
                                      "'Weft' object's mesh in place",
                                      app.livePath.c_str());
                }
            }
        }
        if (app.hasModel) {
            ImGui::Checkbox("triangulate", &app.exportTriangulate);
            ImGui::SameLine();
            ImGui::Checkbox("Y up", &app.exportYUp);
            ImGui::SetNextItemWidth(90.0f * gUiScale);
            ImGui::DragFloat("unit scale", &app.exportScale, 0.001f, 0.0001f,
                             1000.0f, "%.4g");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("0.001 = mm to metres (Unity/Blender)\n"
                                  "0.1 = mm to cm (Unreal)");
            }
        }
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
                    weft::ObjExportOptions opts;
                    opts.triangulate = app.exportTriangulate;
                    opts.yUp = app.exportYUp;
                    opts.scale = double(app.exportScale);
                    weft::writeObj(app.mesh, out, &app.analysis.solidFaces,
                                   &opts);
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
        if (app.openEdgeCount == 0 && app.multiEdgeCount == 0) {
            ImGui::TextColored({0.4f, 0.9f, 0.45f, 1.0f}, "watertight");
        } else {
            ImGui::TextColored({1.0f, 0.45f, 0.3f, 1.0f},
                               "%d open edge(s), %d non-manifold",
                               app.openEdgeCount, app.multiEdgeCount);
            ImGui::SameLine();
            ImGui::Checkbox("show##problems", &app.showProblems);
        }
        if (!app.bLoops.empty()) {
            ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                               "%zu open border loop(s)",
                               app.bLoops.size());
            ImGui::SameLine();
            ImGui::TextDisabled("(J bridges, deleted faces expected)");
        }
        ImGui::Separator();
        // One knob for the whole budget: scales every density proposal.
        float ds = float(app.recipe.settings.densityScale);
        if (ImGui::SliderFloat("density scale", &ds, 0.25f, 4.0f, "%.2fx",
                               ImGuiSliderFlags_Logarithmic)) {
            app.recipe.settings.densityScale = ds;
            markDirty(app);
        }
        // Fit-to-budget: secant search on the scale knob (poly count is
        // roughly quadratic in linear density) until within 5%.
        ImGui::SetNextItemWidth(110.0f * gUiScale);
        ImGui::InputInt("target##budget", &app.budgetTarget, 0, 0);
        ImGui::SameLine();
        if (ImGui::Button("fit polys") && app.budgetTarget > 100) {
            weft::Recipe preFit = app.recipe;
            double target = double(app.budgetTarget);
            for (int it = 0; it < 5; ++it) {
                double P = double(app.mesh.polygonCount());
                if (P < 1) break;
                double err = std::abs(P - target) / target;
                if (err < 0.05) break;
                double next = app.recipe.settings.densityScale *
                              std::sqrt(target / P);
                next = std::clamp(next, 0.25, 4.0);
                if (std::abs(next - app.recipe.settings.densityScale) <
                    1e-3) {
                    break;
                }
                app.recipe.settings.densityScale = next;
                regenerate(app);
            }
            rebuildBuffers(app);
            app.dirty = false;
            app.undoStack.push_back(preFit);
            app.redoStack.clear();
            char buf[96];
            std::snprintf(buf, sizeof buf,
                          "fit: %zu polys at %.2fx (target %d)",
                          app.mesh.polygonCount(),
                          app.recipe.settings.densityScale,
                          app.budgetTarget);
            app.status = buf;
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("solve density scale toward the target\n"
                              "polygon count (a few regenerations)");
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
                ImGui::TextDisabled("wheel / [ ] pins verts");
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
        if (ImGui::Checkbox("quality heatmap", &app.qualityView)) {
            rebuildBuffers(app);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("tint polys by worst corner angle:\n"
                              "green ok, orange skewed, red sliver");
        }
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
    bool startQuality = false;
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
        else if (a == "--quality") startQuality = true;
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
    app.livePath = gDataDir + "/weft_live.obj";
    if (!startModel.empty()) loadModel(app, startModel);
    else loadFixture(app, startFixture);
    app.cam.yaw = startYaw;
    app.cam.pitch = startPitch;
    if (startQuality) {
        app.qualityView = true;
        if (app.hasModel) rebuildBuffers(app);
    }
    if (startSelect > 0 && startSelect <= app.model.faceCount()) {
        app.selFaces = {startSelect};
        app.activeFace = startSelect;
        rebuildBuffers(app);
    }

    double lastX = 0, lastY = 0;
    bool navOrbit = false, navPan = false, navZoom = false, navSnap = false;
    double downX = 0, downY = 0, downRX = 0, downRY = 0;
    bool prevLmb = false, prevRmb = false;
    double lastClickTime = 0;  // double-click select-similar
    int lastClickFace = 0;
    double hoverX = -1, hoverY = -1;  // last hover-picked cursor position
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
                if (app.hasModel && shift && app.mode == Mode::Bridge &&
                    !app.recipe.ops.empty()) {
                    // shift+wheel while bridging: flip WHICH loop the
                    // twist rotates (A or B) on the most recent bridge.
                    for (auto op = app.recipe.ops.rbegin();
                         op != app.recipe.ops.rend(); ++op) {
                        if (op->kind != weft::ManualOp::Kind::Bridge) {
                            continue;
                        }
                        op->twistSide = op->twistSide ? 0 : 1;
                        std::snprintf(app.hudText, sizeof app.hudText,
                                      "twist side: %s",
                                      op->twistSide ? "A" : "B");
                        app.hudUntil = glfwGetTime() + 0.9;
                        markDirty(app);
                        break;
                    }
                } else if (app.hasModel && (shift || ctrl) &&
                    app.selectMode == SelectMode::Edge &&
                    !app.selEdges.empty()) {
                    int total = adjustSelectedEdges(app, 0, steps);
                    std::snprintf(app.hudText, sizeof app.hudText,
                                  app.selEdges.size() > 1
                                      ? "loop verts: %d (%zu edges)"
                                      : "edge verts: %d",
                                  total, app.selEdges.size());
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
                    adjustFaceDensity(app, /*secondary=*/ctrl, steps);
                } else {
                    app.cam.dist *= std::pow(0.92f, gScroll);
                    app.cam.dist = std::clamp(app.cam.dist, 0.5f, 10000.0f);
                }
            }
        }

        // Keyboard-centric editing (plan §6: modal verbs, typed precision).
        if (!io.WantCaptureKeyboard) {
            bool shift = io.KeyShift;
            if (ImGui::IsKeyPressed(ImGuiKey_F, false)) {
                if (app.mode == Mode::Bridge && app.hoverLoop >= 0) {
                    // Fill tool: cap the hovered open boundary with one
                    // n-gon (recorded, replayable, remappable).
                    weft::ManualOp op;
                    op.kind = weft::ManualOp::Kind::FillLoop;
                    op.edgeA = app.bLoopEdge[app.hoverLoop];
                    app.recipe.ops.push_back(op);
                    markDirty(app);
                    app.status = "boundary filled (ctrl+Z undoes)";
                } else if (app.selectMode == SelectMode::Edge &&
                           !app.selEdges.empty() && !app.bLoops.empty()) {
                    // Fill from edge mode: cap the open loop nearest each
                    // selected edge (filling an already-closed loop is a
                    // harmless no-op on replay).
                    for (int eid : app.selEdges) {
                        weft::ManualOp op;
                        op.kind = weft::ManualOp::Kind::FillLoop;
                        op.edgeA = eid;
                        app.recipe.ops.push_back(op);
                    }
                    markDirty(app);
                    app.status = "boundary filled near selected edge(s) "
                                 "(ctrl+Z undoes)";
                } else {
                    frameModel(app);
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_R, false)) {
                toggleLoopCutMode(app);
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
                    toggleBridgeMode(app);
                }
            }
            if (ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                if (app.pieKind >= 0) {
                    app.pieKind = -1;
                } else if (app.mode == Mode::Grab) {
                    // Cancel: drop the nudge op and restore the mesh.
                    if (app.grabOp >= 0 &&
                        app.grabOp < int(app.recipe.ops.size())) {
                        app.recipe.ops.erase(app.recipe.ops.begin() +
                                             app.grabOp);
                    }
                    app.grabOp = -1;
                    app.mode = Mode::Idle;
                    markDirty(app);
                    app.status = "grab cancelled";
                } else if (app.mode == Mode::Bridge && app.bridgeFirstEdge) {
                    app.bridgeFirstEdge = 0;
                } else if (app.mode != Mode::Idle) app.mode = Mode::Idle;
                else if (!app.selFaces.empty() || !app.selEdges.empty() ||
                         !app.selVerts.empty() || !app.selMeshEdges.empty()) {
                    app.selFaces.clear();
                    app.selEdges.clear();
                    app.selVerts.clear();
                    app.selMeshEdges.clear();
                    app.activeFace = 0;
                    rebuildBuffers(app);
                }
            }
            // Selection modes on 1-5: verts, mesh edges, mesh faces,
            // feature edges (B-rep), elements (B-rep faces). Counts are
            // adjusted with the wheel and [ ] — digits never type values.
            if (!io.KeyCtrl) {
                if (ImGui::IsKeyPressed(ImGuiKey_1, false)) {
                    setSelectMode(app, SelectMode::Vert);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_2, false)) {
                    setSelectMode(app, SelectMode::MeshEdge);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_3, false)) {
                    setSelectMode(app, SelectMode::Poly);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_4, false)) {
                    setSelectMode(app, SelectMode::Edge);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_5, false)) {
                    setSelectMode(app, SelectMode::Face);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_6, false)) {
                    setSelectMode(app, SelectMode::Object);
                }
            }
            // Blender numpad views: 1 front, 3 right, 7 top (ctrl for
            // the opposite), 5 toggles orthographic.
            {
                const float hp = 1.55f;
                auto view = [&](float yaw, float pitch) {
                    app.cam.yaw = yaw;
                    app.cam.pitch = pitch;
                };
                if (ImGui::IsKeyPressed(ImGuiKey_Keypad1, false)) {
                    view(io.KeyCtrl ? 1.5708f : -1.5708f, 0.0f);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_Keypad3, false)) {
                    view(io.KeyCtrl ? 3.1416f : 0.0f, 0.0f);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_Keypad7, false)) {
                    view(app.cam.yaw, io.KeyCtrl ? -hp : hp);
                }
                if (ImGui::IsKeyPressed(ImGuiKey_Keypad5, false)) {
                    app.orthoView = !app.orthoView;
                    app.status = app.orthoView ? "orthographic"
                                               : "perspective";
                }
            }
            // ctrl+I: invert the selection in element / face mode.
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_I, false) &&
                app.hasModel) {
                if (app.selectMode == SelectMode::Face ||
                    app.selectMode == SelectMode::Object) {
                    std::set<int> inv;
                    for (int f = 1; f <= app.model.faceCount(); ++f) {
                        if (!app.selFaces.count(f) &&
                            !app.hiddenFaces.count(f)) {
                            inv.insert(f);
                        }
                    }
                    app.selFaces = std::move(inv);
                    app.activeFace =
                        app.selFaces.empty() ? 0 : *app.selFaces.begin();
                    rebuildBuffers(app);
                } else if (app.selectMode == SelectMode::Poly) {
                    std::set<size_t> inv;
                    for (size_t pp = 0; pp < app.mesh.polygons.size();
                         ++pp) {
                        int fid = app.mesh.polygonFaceId[pp];
                        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
                        if (!app.selPolys.count(pp)) inv.insert(pp);
                    }
                    app.selPolys = std::move(inv);
                }
            }
            // Pie menus at the cursor: Tab = select modes, Q = tools.
            // Blender-style: HOLD the key and release over a sector to
            // commit; a quick tap leaves the pie open for a click.
            if (app.hasModel &&
                (ImGui::IsKeyPressed(ImGuiKey_Tab, false) ||
                 ImGui::IsKeyPressed(ImGuiKey_Q, false))) {
                int want = ImGui::IsKeyPressed(ImGuiKey_Tab, false) ? 0 : 1;
                if (app.pieKind == want) {
                    app.pieKind = -1;
                } else {
                    app.pieKind = want;
                    app.pieHold = true;
                    app.pieCenter[0] = float(mx);
                    app.pieCenter[1] = float(my);
                }
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
                } else if (app.mode == Mode::Bridge ||
                           (app.selectMode == SelectMode::Edge && shift &&
                            !app.recipe.ops.empty() &&
                            app.recipe.ops.back().kind ==
                                weft::ManualOp::Kind::Bridge)) {
                    // Adjust the most recent bridge: [ ] twists the rail
                    // pairing, shift+[ ] changes the rows across (spans).
                    for (auto op = app.recipe.ops.rbegin();
                         op != app.recipe.ops.rend(); ++op) {
                        if (op->kind != weft::ManualOp::Kind::Bridge) continue;
                        if (shift) {
                            op->spans = std::max(1, op->spans + delta);
                            std::snprintf(app.hudText, sizeof app.hudText,
                                          "bridge spans: %d", op->spans);
                        } else {
                            op->twist += delta;
                            std::snprintf(app.hudText, sizeof app.hudText,
                                          "bridge twist: %+d", op->twist);
                        }
                        markDirty(app);
                        app.hudUntil = glfwGetTime() + 0.9;
                        break;
                    }
                } else if (app.selectMode == SelectMode::Edge) {
                    adjustSelectedEdges(app, 0, delta);
                } else {
                    adjustFaceDensity(app, /*secondary=*/shift, delta);
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
            if (ImGui::IsKeyPressed(ImGuiKey_X, false) && app.hasModel) {
                if (io.KeyCtrl && app.selectMode == SelectMode::MeshEdge &&
                    !app.selMeshEdges.empty()) {
                    // ctrl+X: dissolve the selected loops, faces survive.
                    // One recorded op per connected loop.
                    std::set<uint64_t> left = app.selMeshEdges;
                    int nLoops = 0;
                    while (!left.empty()) {
                        uint64_t e = *left.begin();
                        uint32_t a = uint32_t(e >> 32);
                        uint32_t b = uint32_t(e & 0xffffffffu);
                        for (const auto& [u, w] :
                             weft::walkEdgeLoop(app.mesh, a, b)) {
                            left.erase(meshEdgeKey(u, w));
                        }
                        const auto& A = app.mesh.vertices[a];
                        const auto& B = app.mesh.vertices[b];
                        weft::ManualOp op;
                        op.kind = weft::ManualOp::Kind::DissolveLoop;
                        op.u = 0.5 * (A[0] + B[0]);
                        op.v = 0.5 * (A[1] + B[1]);
                        op.t = 0.5 * (A[2] + B[2]);
                        app.recipe.ops.push_back(op);
                        ++nLoops;
                    }
                    app.selMeshEdges.clear();
                    markDirty(app);
                    app.status = std::to_string(nLoops) +
                                 " loop(s) dissolved (ctrl+Z undoes)";
                } else if (!io.KeyCtrl) {
                    deleteSelection(app);
                }
            }
            // H / ctrl+H hide the selection, alt+H / shift+H show all.
            if (ImGui::IsKeyPressed(ImGuiKey_H, false) && app.hasModel) {
                hideSelection(app, shift || io.KeyAlt);
            }
            if (ImGui::IsKeyPressed(ImGuiKey_W, false)) app.showWire = !app.showWire;
            if (ImGui::IsKeyPressed(ImGuiKey_B, false) &&
                app.selectMode == SelectMode::Edge &&
                app.selEdges.size() == 2 && app.hasModel) {
                auto it = app.selEdges.begin();
                weft::ManualOp op;
                op.kind = weft::ManualOp::Kind::Bridge;
                op.edgeA = *it++;
                op.edgeB = *it;
                app.recipe.ops.push_back(op);
                markDirty(app);
                app.status = "bridged ([ ] twist, shift+[ ] spans, "
                             "ctrl+Z undoes)";
            }
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S, false) &&
                app.hasModel && !app.recipePath.empty()) {
                try {
                    weft::saveRecipe(app.recipe, app.recipePath);
                    app.status = "saved " + app.recipePath;
                    logLine("saved recipe %s", app.recipePath.c_str());
                } catch (const std::exception& e) {
                    app.status = std::string("save failed: ") + e.what();
                }
            }
            if (io.KeyCtrl && !shift &&
                ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
                if (!app.undoStack.empty()) {
                    app.redoStack.push_back(app.recipe);
                    app.recipe = app.undoStack.back();
                    app.undoStack.pop_back();
                    app.dirty = true;  // not a mutation: no undo push
                    app.status = "undo";
                } else {
                    app.status = "nothing to undo";
                }
            }
            if (io.KeyCtrl && shift &&
                ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
                if (!app.redoStack.empty()) {
                    app.undoStack.push_back(app.recipe);
                    app.recipe = app.redoStack.back();
                    app.redoStack.pop_back();
                    app.dirty = true;
                    app.status = "redo";
                } else {
                    app.status = "nothing to redo";
                }
            }
        }

        bool lmb = !io.WantCaptureMouse &&
                   glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) ==
                       GLFW_PRESS;
        bool lmbPressed = lmb && !prevLmb;
        if (lmbPressed) { downX = mx; downY = my; }
        bool clicked = prevLmb && !lmb && std::abs(mx - downX) < 4 &&
                       std::abs(my - downY) < 4;
        // Box select: an LMB drag in idle rubber-bands in EVERY mode.
        bool boxDrag = lmb && app.mode == Mode::Idle && app.pieKind < 0 &&
                       (std::abs(mx - downX) > 6 || std::abs(my - downY) > 6);
        bool boxReleased = prevLmb && !lmb && !clicked &&
                           app.mode == Mode::Idle && app.pieKind < 0;
        prevLmb = lmb;
        bool rmb = !io.WantCaptureMouse &&
                   glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) ==
                       GLFW_PRESS;
        if (rmb && !prevRmb) { downRX = mx; downRY = my; }
        bool rClicked = prevRmb && !rmb && std::abs(mx - downRX) < 4 &&
                        std::abs(my - downRY) < 4;
        prevRmb = rmb;
        gScroll = 0.0f;

        // STEP hot-reload: poll the source file's mtime (cheap) and reload
        // once it changes AND stops changing — exporters write in bursts,
        // and a half-written STEP must never be parsed.
        if (app.hasModel && app.watchSource && !app.sourcePath.empty()) {
            double now = glfwGetTime();
            static double nextPoll = 0.0;
            if (now >= nextPoll) {
                nextPoll = now + 0.5;
                std::error_code ec;
                auto m = std::filesystem::last_write_time(app.sourcePath, ec);
                if (!ec && m != app.sourceMtime) {
                    if (!app.reloadPending || m != app.pendingMtime) {
                        app.reloadPending = true;   // (re)start the debounce
                        app.pendingMtime = m;
                        app.pendingSince = now;
                    } else if (now - app.pendingSince > 0.4) {
                        app.sourceMtime = m;
                        app.reloadPending = false;
                        reloadModel(app);
                    }
                } else if (!ec) {
                    app.reloadPending = false;
                }
            }
        }

        if (app.mutatedThisFrame) logLine("frame: input handled, dirty");
        if (app.dirty) regenerate(app);

        const float vpAspect = fbh > 0 ? float(fbw) / fbh : 1.6f;
        Mat4 proj =
            app.orthoView
                ? matOrtho(app.cam.dist * std::tan(21.0f * float(M_PI) /
                                                   180.0f),
                           vpAspect, app.cam.dist * 0.01f,
                           app.cam.dist * 40.0f)
                : matPerspective(42.0f, vpAspect, app.cam.dist * 0.01f,
                                 app.cam.dist * 40.0f);
        Mat4 view = matLookAt(app.cam.eye(), app.cam.target, {0, 0, 1});
        Mat4 mvp = matMul(proj, view);

        // Vertex grab starts from idle: G picks the interior vertex under
        // the cursor and drags it constrained to its CAD surface.
        if (app.hasModel && app.mode == Mode::Idle &&
            !io.WantCaptureKeyboard && !io.WantCaptureMouse &&
            ImGui::IsKeyPressed(ImGuiKey_G, false)) {
            startVertexGrab(app, mvp, mx, my, fbw, fbh);
        }

        // Loop-cut hover preview follows the cursor; pressing commits the
        // op into the recipe and dragging before release slides its t along
        // the strip (regeneration replays it — fully non-destructive).
        if (app.mode == Mode::LoopCut && !io.WantCaptureMouse) {
            if (app.slideOp >= 0) {
                if (!lmb) {
                    app.slideOp = -1;
                    app.status = "loop cut committed (ctrl+z undoes)";
                } else if (app.slideOp < int(app.recipe.ops.size())) {
                    float ex = app.slideB[0] - app.slideA[0];
                    float ey = app.slideB[1] - app.slideA[1];
                    float len2 = ex * ex + ey * ey;
                    if (len2 > 1e-6f) {
                        float t = ((float(mx) - app.slideA[0]) * ex +
                                   (float(my) - app.slideA[1]) * ey) /
                                  len2;
                        double nt = std::clamp(double(t), 0.05, 0.95);
                        if (app.slideFlipped) nt = 1.0 - nt;
                        weft::ManualOp& op = app.recipe.ops[app.slideOp];
                        if (std::abs(nt - op.t) > 1e-4) {
                            op.t = nt;
                            markDirty(app);
                        }
                        std::snprintf(app.hudText, sizeof app.hudText,
                                      "loop slide: %.2f", op.t);
                        app.hudUntil = glfwGetTime() + 0.5;
                    }
                }
            } else {
                updateLoopCutHover(app, mvp, mx, my, fbw, fbh);
                if (lmbPressed && app.hoverValid) {
                    app.recipe.ops.push_back(app.hoverOp);
                    app.slideOp = int(app.recipe.ops.size()) - 1;
                    markDirty(app);
                    app.status = "loop cut: drag slides, release commits";
                }
            }
        } else if (app.mode == Mode::Grab && app.hasModel) {
            if (app.grabOp >= 0 && app.grabOp < int(app.recipe.ops.size())) {
                // Drag plane: camera-facing through the grab point; the
                // hit re-projects exactly onto the vertex's CAD face.
                weft::ManualOp& op = app.recipe.ops[app.grabOp];
                Vec3 eye = app.cam.eye();
                Vec3 dir = mouseRay(app.cam, mx, my, fbw, fbh);
                Vec3 f = norm(sub(app.cam.target, eye));
                float relDot = (float(app.grabStart[0]) - eye.x) * f.x +
                               (float(app.grabStart[1]) - eye.y) * f.y +
                               (float(app.grabStart[2]) - eye.z) * f.z;
                float denom = dir.x * f.x + dir.y * f.y + dir.z * f.z;
                if (std::abs(denom) > 1e-6f) {
                    float tt = relDot / denom;
                    std::array<double, 3> p = {double(eye.x + dir.x * tt),
                                               double(eye.y + dir.y * tt),
                                               double(eye.z + dir.z * tt)};
                    try {
                        weft::Anchor a =
                            weft::snapToFace(app.model, op.faceId, p);
                        if (std::abs(a.u - op.u2) > 1e-12 ||
                            std::abs(a.v - op.v2) > 1e-12) {
                            op.u2 = a.u;
                            op.v2 = a.v;
                            markDirty(app);
                        }
                    } catch (const std::exception&) {
                        // Projection can fail while the cursor is far off
                        // the surface; keep the last good position.
                    }
                }
                if (clicked) {
                    app.mode = Mode::Idle;
                    app.grabOp = -1;
                    app.status = "vertex nudged (ctrl+z undoes)";
                } else if (rClicked) {
                    app.recipe.ops.erase(app.recipe.ops.begin() + app.grabOp);
                    app.grabOp = -1;
                    app.mode = Mode::Idle;
                    markDirty(app);
                    app.status = "grab cancelled";
                }
            } else {
                app.mode = Mode::Idle;
                app.grabOp = -1;
            }
        } else if (app.mode == Mode::Bridge && !io.WantCaptureMouse) {
            updateBridgeHover(app, mvp, mx, my, fbw, fbh);
            if (clicked && app.hoverLoop >= 0) {
                // The edge under the CURSOR, not the loop's canonical one:
                // two picks on the same loop then name its two sides, and
                // the core splits the loop between them.
                int eid = nearestBrepEdge(app, mvp, mx, my, fbw, fbh,
                                          30.0 * gUiScale);
                if (eid == 0) eid = app.bLoopEdge[app.hoverLoop];
                if (app.bridgeFirstEdge == 0) {
                    app.bridgeFirstEdge = eid;
                    app.status = "bridge: pick the second loop or the "
                                 "other side of this one";
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
        } else if (boxReleased && app.hasModel) {
            // Box select in every mode; the pick buffer is the occlusion
            // oracle, so only what the camera sees gets selected.
            PickRect pr = readPickRect(app, flatProg, mvp, int(downX),
                                       int(downY), int(mx), int(my), fbw,
                                       fbh);
            const bool extend = io.KeyShift;
            auto inRect = [&](double x, double y) {
                return x >= std::min(downX, mx) && x <= std::max(downX, mx) &&
                       y >= std::min(downY, my) && y <= std::max(downY, my);
            };
            if (app.selectMode == SelectMode::Vert) {
                if (!extend) app.selVerts.clear();
                std::vector<bool> seen(app.mesh.vertexCount(), false);
                for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
                    int fid = app.mesh.polygonFaceId[p];
                    if (fid > 0 && app.hiddenFaces.count(fid)) continue;
                    for (uint32_t v : app.mesh.polygons[p]) {
                        if (seen[v]) continue;
                        seen[v] = true;
                        float sp[3] = {0, 0, -1};
                        projectPoint(mvp, app.mesh.vertices[v], fbw, fbh, sp);
                        if (sp[2] <= 0 || !inRect(sp[0], sp[1])) continue;
                        if (vertVisibleAt(app, v, pr.at(sp[0], sp[1]))) {
                            app.selVerts.insert(v);
                        }
                    }
                }
            } else if (app.selectMode == SelectMode::MeshEdge) {
                if (!extend) app.selMeshEdges.clear();
                for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
                    int fid = app.mesh.polygonFaceId[p];
                    if (fid > 0 && app.hiddenFaces.count(fid)) continue;
                    const auto& poly = app.mesh.polygons[p];
                    for (size_t i = 0; i < poly.size(); ++i) {
                        uint32_t a = poly[i], b = poly[(i + 1) % poly.size()];
                        float pa[3] = {0, 0, -1}, pb[3] = {0, 0, -1};
                        projectPoint(mvp, app.mesh.vertices[a], fbw, fbh, pa);
                        projectPoint(mvp, app.mesh.vertices[b], fbw, fbh, pb);
                        if (pa[2] <= 0 || pb[2] <= 0) continue;
                        if (!inRect(pa[0], pa[1]) || !inRect(pb[0], pb[1])) {
                            continue;
                        }
                        int atPx = pr.at((pa[0] + pb[0]) / 2,
                                         (pa[1] + pb[1]) / 2);
                        if (vertVisibleAt(app, a, atPx) &&
                            vertVisibleAt(app, b, atPx)) {
                            app.selMeshEdges.insert(meshEdgeKey(a, b));
                        }
                    }
                }
            } else if (app.selectMode == SelectMode::Poly) {
                if (!extend) app.selPolys.clear();
                for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
                    int fid = app.mesh.polygonFaceId[p];
                    if (fid <= 0 || app.hiddenFaces.count(fid)) continue;
                    double cx = 0, cy = 0;
                    bool ok = true;
                    for (uint32_t v : app.mesh.polygons[p]) {
                        float sp[3] = {0, 0, -1};
                        projectPoint(mvp, app.mesh.vertices[v], fbw, fbh, sp);
                        if (sp[2] <= 0) {
                            ok = false;
                            break;
                        }
                        cx += sp[0];
                        cy += sp[1];
                    }
                    if (!ok) continue;
                    cx /= double(app.mesh.polygons[p].size());
                    cy /= double(app.mesh.polygons[p].size());
                    if (inRect(cx, cy) && pr.at(cx, cy) == fid) {
                        app.selPolys.insert(p);
                    }
                }
            } else if (app.selectMode == SelectMode::Edge) {
                if (!extend) app.selEdges.clear();
                for (const weft::EdgePolyline& e : app.brepEdges) {
                    if (e.edgeId < 1 ||
                        size_t(e.edgeId) > app.analysis.edges.size()) {
                        continue;
                    }
                    const auto& adj =
                        app.analysis.edges[e.edgeId - 1].faceIds;
                    for (const auto& pt : e.points) {
                        float sp[3] = {0, 0, -1};
                        projectPoint(mvp, pt, fbw, fbh, sp);
                        if (sp[2] <= 0 || !inRect(sp[0], sp[1])) continue;
                        int atPx = pr.at(sp[0], sp[1]);
                        if (atPx > 0 && std::find(adj.begin(), adj.end(),
                                                  atPx) != adj.end()) {
                            app.selEdges.insert(e.edgeId);
                            break;
                        }
                    }
                }
                rebuildBuffers(app);
            } else {
                // Element / object mode: visible faces in the rect
                // (objects expand to their whole solid).
                std::set<int> hits;
                for (int fid : pr.fid) {
                    if (fid > 0) hits.insert(fid);
                }
                if (!extend) app.selFaces.clear();
                for (int fid : hits) {
                    if (app.selectMode == SelectMode::Object) {
                        selectSolidOf(app, fid, /*extend=*/true);
                    } else {
                        app.selFaces.insert(fid);
                    }
                }
                if (!hits.empty()) app.activeFace = *hits.begin();
                else if (!extend) app.activeFace = 0;
                rebuildBuffers(app);
            }
        } else if ((clicked || rClicked) && app.hasModel &&
                   app.pieKind < 0) {
            bool shift = io.KeyShift;
            const double pickR = 16.0 * gUiScale;
            auto pickRectAtCursor = [&]() {
                return readPickRect(app, flatProg, mvp, int(mx - pickR),
                                    int(my - pickR), int(mx + pickR),
                                    int(my + pickR), fbw, fbh);
            };
            if (app.selectMode == SelectMode::Vert && clicked) {
                PickRect pr = pickRectAtCursor();
                int64_t hit = pickMeshVert(app, mvp, mx, my, fbw, fbh, pr);
                if (!shift) app.selVerts.clear();
                if (hit >= 0) {
                    uint32_t h = uint32_t(hit);
                    if (shift && app.selVerts.count(h)) {
                        app.selVerts.erase(h);
                    } else {
                        app.selVerts.insert(h);
                    }
                }
            } else if (app.selectMode == SelectMode::MeshEdge && clicked) {
                PickRect pr = pickRectAtCursor();
                uint64_t hit = pickMeshEdge(app, mvp, mx, my, fbw, fbh, pr);
                if (!shift) app.selMeshEdges.clear();
                if (hit != UINT64_MAX) {
                    if (io.KeyAlt) {
                        // Blender-style: alt+click selects the edge LOOP.
                        auto loop = weft::walkEdgeLoop(
                            app.mesh, uint32_t(hit >> 32),
                            uint32_t(hit & 0xffffffffu));
                        for (const auto& [a, b] : loop) {
                            app.selMeshEdges.insert(meshEdgeKey(a, b));
                        }
                        app.status = "loop selected (" +
                                     std::to_string(loop.size()) +
                                     " edges) - ctrl+X dissolves";
                    } else if (shift && app.selMeshEdges.count(hit)) {
                        app.selMeshEdges.erase(hit);
                    } else {
                        app.selMeshEdges.insert(hit);
                    }
                }
            } else if (app.selectMode == SelectMode::Poly) {
                // Polygon picking: front-most poly under the cursor (works
                // on bridge/fill strips too). Right-click does nothing —
                // the context popup is face-scoped.
                if (clicked) {
                    int hit = pickPolygon(app, mvp, mx, my, fbw, fbh);
                    if (!shift) app.selPolys.clear();
                    if (hit >= 0) {
                        size_t h = size_t(hit);
                        if (shift && app.selPolys.count(h)) {
                            app.selPolys.erase(h);
                        } else {
                            app.selPolys.insert(h);
                        }
                    }
                }
            } else if (app.selectMode == SelectMode::Edge && clicked) {
                // Edge picking: nearest projected B-rep edge polyline that
                // is actually VISIBLE (the pick pixel at the closest point
                // shows one of the edge's own faces).
                PickRect pr = pickRectAtCursor();
                int hit = 0;
                double best = 14.0 * gUiScale;  // px
                for (const weft::EdgePolyline& e : app.brepEdges) {
                    if (e.edgeId < 1 ||
                        size_t(e.edgeId) > app.analysis.edges.size()) {
                        continue;
                    }
                    const auto& adj =
                        app.analysis.edges[e.edgeId - 1].faceIds;
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
                        double px = pa[0] + t * ex, py = pa[1] + t * ey;
                        double d = std::hypot(double(mx) - px,
                                              double(my) - py);
                        if (d >= best) continue;
                        int atPx = pr.at(px, py);
                        if (atPx <= 0 || std::find(adj.begin(), adj.end(),
                                                   atPx) == adj.end()) {
                            continue;
                        }
                        best = d;
                        hit = e.edgeId;
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
                } else if (app.selectMode == SelectMode::Object) {
                    // Click selects the whole body; shift extends.
                    if (hit > 0) {
                        selectSolidOf(app, hit, shift);
                    } else if (!shift) {
                        app.selFaces.clear();
                        app.activeFace = 0;
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
                    double now = glfwGetTime();
                    if (hit > 0 && hit == lastClickFace &&
                        now - lastClickTime < 0.35) {
                        // Double-click: select everything similar (same
                        // surface type and fillet/hole tags).
                        const weft::FaceInfo& ref =
                            app.analysis.faces[hit - 1];
                        app.selFaces.clear();
                        for (const auto& fi : app.analysis.faces) {
                            if (fi.type == ref.type &&
                                fi.isFillet == ref.isFillet &&
                                fi.isHole == ref.isHole) {
                                app.selFaces.insert(fi.id);
                            }
                        }
                        app.activeFace = hit;
                        app.status = "selected similar (" +
                                     std::to_string(app.selFaces.size()) +
                                     " faces)";
                    } else if (hit > 0 && !(app.selFaces.size() == 1 &&
                                            app.selFaces.count(hit))) {
                        app.selFaces = {hit};
                        app.activeFace = hit;
                    } else {
                        app.selFaces.clear();
                        app.activeFace = 0;
                    }
                    lastClickTime = now;
                    lastClickFace = hit;
                }
                rebuildBuffers(app);
            }
        }

        // Hover pre-highlight: pick under the cursor when it moved (idle
        // only; the overlay draws below need no buffer rebuild).
        if (app.hasModel && app.mode == Mode::Idle && !io.WantCaptureMouse &&
            !lmb && !rmb &&
            (std::abs(mx - hoverX) > 1 || std::abs(my - hoverY) > 1)) {
            if (app.selectMode == SelectMode::Face ||
                app.selectMode == SelectMode::Object) {
                glViewport(0, 0, fbw, fbh);
                app.hoverFace = pickFace(app, flatProg, mvp, int(mx),
                                         int(my), fbw, fbh);
            } else if (app.selectMode == SelectMode::Vert ||
                       app.selectMode == SelectMode::MeshEdge) {
                const double r = 16.0 * gUiScale;
                PickRect pr =
                    readPickRect(app, flatProg, mvp, int(mx - r),
                                 int(my - r), int(mx + r), int(my + r),
                                 fbw, fbh);
                if (app.selectMode == SelectMode::Vert) {
                    app.hoverVert =
                        pickMeshVert(app, mvp, mx, my, fbw, fbh, pr);
                } else {
                    app.hoverMeshEdge =
                        pickMeshEdge(app, mvp, mx, my, fbw, fbh, pr);
                }
            }
            hoverX = mx;
            hoverY = my;
        }
        if (io.WantCaptureMouse) {
            app.hoverFace = 0;
            app.hoverVert = -1;
            app.hoverMeshEdge = UINT64_MAX;
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        if (boxDrag) {
            ImGui::GetForegroundDrawList()->AddRect(
                {float(downX), float(downY)}, {float(mx), float(my)},
                IM_COL32(255, 200, 80, 200), 0.0f, 0, 1.5f);
        }
        // Pie menus: sectors around the opening point, nearest-direction
        // hover, click commits, esc/right-click cancels.
        if (app.pieKind >= 0) {
            struct PieItem {
                const char* label;
                const char* hint;
            };
            static const PieItem kModes[] = {
                {"vert", "1"},         {"edge", "2"},    {"face", "3"},
                {"feature edge", "4"}, {"element", "5"}, {"object", "6"},
            };
            static const PieItem kTools[] = {
                {"loop cut", "R"}, {"bridge", "J"},   {"grab", "G"},
                {"delete", "X"},   {"hide", "H"},     {"show all", "sh+H"},
                {"frame", "F"},    {"wire", "W"},
            };
            const PieItem* items = app.pieKind == 0 ? kModes : kTools;
            const int n = app.pieKind == 0 ? 6 : 8;
            const float cx = app.pieCenter[0], cy = app.pieCenter[1];
            const float radius = 92.0f * gUiScale;
            float dx = float(mx) - cx, dy = float(my) - cy;
            int hover = -1;
            if (std::hypot(dx, dy) > 18.0f * gUiScale) {
                float ang = std::atan2(dy, dx);
                float bestDot = -2.0f;
                for (int i = 0; i < n; ++i) {
                    float ia = float(-M_PI / 2 + i * 2.0 * M_PI / n);
                    float d = std::cos(ia) * std::cos(ang) +
                              std::sin(ia) * std::sin(ang);
                    if (d > bestDot) {
                        bestDot = d;
                        hover = i;
                    }
                }
            }
            ImDrawList* dl = ImGui::GetForegroundDrawList();
            dl->AddCircleFilled({cx, cy}, 5.0f * gUiScale,
                                IM_COL32(255, 196, 64, 255));
            if (hover >= 0) {
                dl->AddLine({cx, cy}, {float(mx), float(my)},
                            IM_COL32(255, 196, 64, 140), 2.0f * gUiScale);
            }
            for (int i = 0; i < n; ++i) {
                float ia = float(-M_PI / 2 + i * 2.0 * M_PI / n);
                ImVec2 at{cx + std::cos(ia) * radius,
                          cy + std::sin(ia) * radius};
                char text[48];
                std::snprintf(text, sizeof text, "%s  %s", items[i].label,
                              items[i].hint);
                ImVec2 sz = ImGui::CalcTextSize(text);
                ImVec2 pad{8.0f * gUiScale, 5.0f * gUiScale};
                ImVec2 a{at.x - sz.x / 2 - pad.x, at.y - sz.y / 2 - pad.y};
                ImVec2 b{at.x + sz.x / 2 + pad.x, at.y + sz.y / 2 + pad.y};
                bool hot = i == hover;
                dl->AddRectFilled(a, b,
                                  hot ? IM_COL32(240, 158, 46, 255)
                                      : IM_COL32(28, 30, 36, 235),
                                  6.0f * gUiScale);
                dl->AddRect(a, b, IM_COL32(255, 196, 64, hot ? 255 : 90),
                            6.0f * gUiScale, 0, 1.0f);
                dl->AddText({at.x - sz.x / 2, at.y - sz.y / 2},
                            hot ? IM_COL32(20, 20, 20, 255)
                                : IM_COL32(220, 224, 232, 255),
                            text);
            }
            // Commit paths: click, or releasing the held key over a
            // sector. Releasing with nothing hovered keeps the pie open
            // (that was a tap).
            bool commit = clicked;
            if (app.pieHold &&
                ImGui::IsKeyReleased(app.pieKind == 0 ? ImGuiKey_Tab
                                                      : ImGuiKey_Q)) {
                app.pieHold = false;
                if (hover >= 0) commit = true;
            }
            if (rClicked) {
                app.pieKind = -1;
            } else if (commit) {
                int kind = app.pieKind;
                app.pieKind = -1;
                if (hover >= 0 && kind == 0) {
                    static const SelectMode kOrder[] = {
                        SelectMode::Vert, SelectMode::MeshEdge,
                        SelectMode::Poly, SelectMode::Edge,
                        SelectMode::Face, SelectMode::Object};
                    setSelectMode(app, kOrder[hover]);
                } else if (hover >= 0) {
                    switch (hover) {
                        case 0: toggleLoopCutMode(app); break;
                        case 1: toggleBridgeMode(app); break;
                        case 2:
                            if (app.mode == Mode::Idle) {
                                startVertexGrab(app, mvp, cx, cy, fbw, fbh);
                            }
                            break;
                        case 3: deleteSelection(app); break;
                        case 4: hideSelection(app, false); break;
                        case 5: hideSelection(app, true); break;
                        case 6: frameModel(app); break;
                        case 7: app.showWire = !app.showWire; break;
                    }
                }
            }
        }
        // Vert / mesh-edge mode overlays: screen-space markers for the
        // hover candidate and the selection (under the UI panels).
        if (app.hasModel && (app.selectMode == SelectMode::Vert ||
                             app.selectMode == SelectMode::MeshEdge)) {
            ImDrawList* dl = ImGui::GetBackgroundDrawList();
            auto projV = [&](uint32_t v, float* sp) {
                projectPoint(mvp, app.mesh.vertices[v], fbw, fbh, sp);
                return sp[2] > 0;
            };
            const float r = 3.5f * gUiScale;
            if (app.selectMode == SelectMode::Vert) {
                for (uint32_t v : app.selVerts) {
                    if (v >= app.mesh.vertexCount()) continue;
                    float sp[3];
                    if (projV(v, sp)) {
                        dl->AddRectFilled({sp[0] - r, sp[1] - r},
                                          {sp[0] + r, sp[1] + r},
                                          IM_COL32(255, 196, 64, 255));
                    }
                }
                if (app.hoverVert >= 0 &&
                    app.hoverVert < int64_t(app.mesh.vertexCount())) {
                    float sp[3];
                    if (projV(uint32_t(app.hoverVert), sp)) {
                        dl->AddCircle({sp[0], sp[1]}, 6.0f * gUiScale,
                                      IM_COL32(255, 255, 255, 220), 0,
                                      1.5f * gUiScale);
                    }
                }
            } else {
                auto edgeLine = [&](uint64_t key, ImU32 col, float w) {
                    uint32_t a = uint32_t(key >> 32);
                    uint32_t b = uint32_t(key & 0xffffffffu);
                    if (a >= app.mesh.vertexCount() ||
                        b >= app.mesh.vertexCount()) {
                        return;
                    }
                    float sa[3], sb[3];
                    if (projV(a, sa) && projV(b, sb)) {
                        dl->AddLine({sa[0], sa[1]}, {sb[0], sb[1]}, col, w);
                    }
                };
                for (uint64_t e : app.selMeshEdges) {
                    edgeLine(e, IM_COL32(255, 196, 64, 255),
                             3.0f * gUiScale);
                }
                if (app.hoverMeshEdge != UINT64_MAX) {
                    edgeLine(app.hoverMeshEdge,
                             IM_COL32(255, 255, 255, 200), 2.0f * gUiScale);
                }
            }
        }
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
            // Hover highlight: re-draw just that face's runs, tinted.
            if (app.hoverFace > 0 && !app.selFaces.count(app.hoverFace)) {
                glDepthFunc(GL_LEQUAL);
                glUseProgram(flatProg);
                glUniformMatrix4fv(glGetUniformLocation(flatProg, "uMVP"), 1,
                                   GL_FALSE, mvp.m);
                glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.30f);
                float hl[3] = {1.0f, 0.95f, 0.75f};
                glUniform3fv(glGetUniformLocation(flatProg, "uColor"), 1, hl);
                glBindVertexArray(app.fill.vao);
                for (const auto& seg : app.fillSegs) {
                    if (seg[0] == app.hoverFace) {
                        glDrawArrays(GL_TRIANGLES, seg[1], seg[2]);
                    }
                }
                glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.0f);
                glDepthFunc(GL_LESS);
            }
            // Polygon-mode selection: tint the selected polys' runs.
            if (app.selectMode == SelectMode::Poly && !app.selPolys.empty()) {
                glDepthFunc(GL_LEQUAL);
                glUseProgram(flatProg);
                glUniformMatrix4fv(glGetUniformLocation(flatProg, "uMVP"), 1,
                                   GL_FALSE, mvp.m);
                glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.65f);
                float sl[3] = {0.98f, 0.80f, 0.25f};
                glUniform3fv(glGetUniformLocation(flatProg, "uColor"), 1, sl);
                glBindVertexArray(app.fill.vao);
                for (size_t p : app.selPolys) {
                    if (p >= app.polyFillRange.size()) continue;
                    const auto& r = app.polyFillRange[p];
                    if (r[0] >= 0 && r[1] > 0) {
                        glDrawArrays(GL_TRIANGLES, r[0], r[1]);
                    }
                }
                glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.0f);
                glDepthFunc(GL_LESS);
            }
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
        if (app.hasModel && app.showProblems && app.problems.count) {
            // Open (red) / non-manifold (magenta) edges, drawn through
            // the mesh so nothing hides a leak.
            glDisable(GL_DEPTH_TEST);
            glLineWidth(3.0f);
            glUniform1f(uMix, 1.0f);
            glBindVertexArray(app.problems.vao);
            glDrawArrays(GL_LINES, 0, app.problems.count);
            glUniform1f(uMix, 0.0f);
            glLineWidth(1.0f);
            glEnable(GL_DEPTH_TEST);
        }
        if (app.hasModel &&
            (app.showVerts || app.selectMode == SelectMode::Vert) &&
            app.verts.count) {
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
                app.redoStack.clear();  // a fresh edit forks history
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
