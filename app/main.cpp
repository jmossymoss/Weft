// weft_app — interactive shell over weft_core.
//
// Phase-2/6 viewport (plan §6): load STEP (or a built-in fixture), see the
// B-rep with feature colouring, click faces to select, drag density controls
// and watch the topology regenerate live. Orbit/pan/zoom like Blender.
//
//   weft_app [model.step] [--fixture demo] [--screenshot out.png] [--finalize]
//   weft_app model.step --finalize --screenshot-objects <dir>

// windows.h + commdlg.h must come FIRST: OCCT's headers include windows.h
// themselves with slimmed-down defines, and a later re-include is a no-op
// (header guard), leaving commdlg.h without the dialog types it needs.
// Full windows.h here wins the race and satisfies everyone.
#ifdef _WIN32
// OCCT >= 8.0's CMake config injects NOGDI/NOMINMAX into every consumer;
// NOGDI strips the GDI/user types (LOGFONT, NMHDR, DLGTEMPLATE) that
// commdlg.h/prsht.h (file dialogs) require. This TU never touches OCCT
// visualization, so restoring GDI is safe.
#undef NOGDI
#undef NOUSER
#undef NOCTLMGR
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#include <dbghelp.h>
#endif

#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Vec.hxx>

#include "weft/analysis.hpp"
#include "weft/edit.hpp"
#include "bake_queue.hpp"
#include "weft/export_fbx.hpp"
#include "weft/export_gltf.hpp"
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
#ifdef IMGUI_HAS_DOCK
#include <imgui_internal.h>
#endif
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

#include "weft_logo_data.h"   // embedded RGBA window-icon + Settings badge

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <thread>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <numeric>
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

// Control scheme. Desktop = Blender-style MMB navigation. Laptop = the
// same scheme with alt+LMB standing in for MMB (Blender's "emulate
// 3-button mouse") so a bare trackpad can orbit/pan/zoom. Persisted per
// user next to imgui.ini.
static bool gLaptopControls = false;
static std::string gControlsPath;
static void loadControls() {
    if (FILE* f = std::fopen(gControlsPath.c_str(), "r")) {
        int v = 0;
        if (std::fscanf(f, "laptop=%d", &v) == 1) gLaptopControls = v != 0;
        std::fclose(f);
    }
}
static void saveControls() {
    if (FILE* f = std::fopen(gControlsPath.c_str(), "w")) {
        std::fprintf(f, "laptop=%d\n", gLaptopControls ? 1 : 0);
        std::fclose(f);
    }
}

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

// UI scale from the monitor's content scale (HiDPI / fractional scaling).
// Fonts and style metrics rebuild when it changes (e.g. dragging the
// window to a monitor with a different scale).
static float gUiScale = 1.0f;
static GLuint gLogoBadgeTex = 0;   // Settings-panel logo badge (0 until GL up)
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
    X(PFNGLUNIFORM1IPROC, glUniform1i)                     \
    X(PFNGLUNIFORM3FVPROC, glUniform3fv)                   \
    X(PFNGLGENVERTEXARRAYSPROC, glGenVertexArrays)          \
    X(PFNGLBINDVERTEXARRAYPROC, glBindVertexArray)          \
    X(PFNGLGENBUFFERSPROC, glGenBuffers)                    \
    X(PFNGLBINDBUFFERPROC, glBindBuffer)                    \
    X(PFNGLBUFFERDATAPROC, glBufferData)                    \
    X(PFNGLVERTEXATTRIBPOINTERPROC, glVertexAttribPointer)  \
    X(PFNGLENABLEVERTEXATTRIBARRAYPROC, glEnableVertexAttribArray) \
    X(PFNGLDISABLEVERTEXATTRIBARRAYPROC, glDisableVertexAttribArray)

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

// Read-only nearest-vertex index used when the generated mesh is adopted.
// The old edge-overlay snap tested every sampled B-rep point against every
// mesh vertex (O(samples * vertices)); on an assembly like MP9 that repeated
// hundreds of millions of distance checks after even a one-face cache hit.
class VertexKdTree {
public:
    explicit VertexKdTree(
        const std::vector<std::array<double, 3>>& points)
        : points_(points) {
        order_.resize(points.size());
        std::iota(order_.begin(), order_.end(), uint32_t{0});
        nodes_.reserve(points.size());
        root_ = build(0, order_.size(), 0);
    }

    bool nearestWithin(const std::array<double, 3>& point, double maxDist2,
                       uint32_t& hit) const {
        bool found = false;
        search(root_, point, maxDist2, hit, found);
        return found;
    }

private:
    struct Node {
        uint32_t point = 0;
        int left = -1;
        int right = -1;
        unsigned char axis = 0;
    };

    int build(size_t begin, size_t end, int depth) {
        if (begin >= end) return -1;
        const unsigned char axis = static_cast<unsigned char>(depth % 3);
        const size_t mid = begin + (end - begin) / 2;
        std::nth_element(order_.begin() + begin, order_.begin() + mid,
                         order_.begin() + end, [&](uint32_t a, uint32_t b) {
                             return points_[a][axis] < points_[b][axis];
                         });
        const int node = static_cast<int>(nodes_.size());
        nodes_.push_back({order_[mid], -1, -1, axis});
        const int left = build(begin, mid, depth + 1);
        const int right = build(mid + 1, end, depth + 1);
        nodes_[node].left = left;
        nodes_[node].right = right;
        return node;
    }

    void search(int nodeIndex, const std::array<double, 3>& point,
                double& bestDist2, uint32_t& hit, bool& found) const {
        if (nodeIndex < 0) return;
        const Node& node = nodes_[nodeIndex];
        const auto& candidate = points_[node.point];
        const double dx = candidate[0] - point[0];
        const double dy = candidate[1] - point[1];
        const double dz = candidate[2] - point[2];
        const double dist2 = dx * dx + dy * dy + dz * dz;
        if (dist2 < bestDist2) {
            bestDist2 = dist2;
            hit = node.point;
            found = true;
        }
        const double delta = point[node.axis] - candidate[node.axis];
        const int nearNode = delta < 0.0 ? node.left : node.right;
        const int farNode = delta < 0.0 ? node.right : node.left;
        search(nearNode, point, bestDist2, hit, found);
        if (delta * delta < bestDist2) {
            search(farNode, point, bestDist2, hit, found);
        }
    }

    const std::vector<std::array<double, 3>>& points_;
    std::vector<uint32_t> order_;
    std::vector<Node> nodes_;
    int root_ = -1;
};

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
layout(location=2) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat4 uMV;
out vec3 vPosVS;
out vec3 vColor;
out vec3 vNrmVS;
void main() {
    vPosVS = (uMV * vec4(aPos, 1.0)).xyz;
    vColor = aColor;
    vNrmVS = mat3(uMV) * aNormal;
    gl_Position = uMVP * vec4(aPos, 1.0);
})";

static const char* kLitFS = R"(#version 330 core
in vec3 vPosVS;
in vec3 vColor;
in vec3 vNrmVS;
out vec4 frag;
uniform float uAmbient;
uniform float uDiffuse;
uniform float uRim;
uniform int uMode;    // 0 = studio lighting, 1 = procedural matcap
uniform int uSmooth;  // 1 = per-vertex smoothing-angle normals
void main() {
    vec3 n = (uSmooth == 1 && dot(vNrmVS, vNrmVS) > 1e-8)
                 ? normalize(vNrmVS)
                 : normalize(cross(dFdx(vPosVS), dFdy(vPosVS)));
    if (uMode == 1) {
        // Procedural studio matcap: shading depends only on the view-space
        // normal, so surface flow, dents and folds read the same from any
        // camera angle. Key + fill lobes, a rim, and two specular hits.
        vec3 nn = n.z < 0.0 ? -n : n;
        float key  = clamp(dot(nn, normalize(vec3(-0.45, 0.55, 0.70))), 0.0, 1.0);
        float fil  = clamp(dot(nn, normalize(vec3( 0.65,-0.20, 0.74))), 0.0, 1.0);
        float rimL = pow(1.0 - clamp(nn.z, 0.0, 1.0), 2.5);
        vec3 col = vec3(0.20, 0.205, 0.22)
                 + vec3(0.60, 0.58, 0.55) * pow(key, 1.4)
                 + vec3(0.17, 0.18, 0.21) * pow(fil, 2.0)
                 + vec3(0.14, 0.15, 0.18) * rimL
                 + vec3(0.80) * pow(key, 24.0)
                 + vec3(0.22) * pow(fil, 18.0);
        // Keep selection / heatmap tints readable through the matcap
        // without letting face-type colours swallow the studio shading.
        vec3 tint = mix(vec3(1.0), clamp(vColor * 1.55, 0.0, 1.5), 0.28);
        frag = vec4(col * tint, 1.0);
        return;
    }
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

// GPU-only topology feedback. The current CAD surface triangles carry
// normalized face UVs; the fragment shader draws the requested grid at frame
// rate while the exact CPU mesher settles in the background.
static const char* kProxyVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aUv;
uniform mat4 uMVP;
out vec2 vUv;
void main() {
    vUv = aUv;
    gl_Position = uMVP * vec4(aPos, 1.0);
})";

static const char* kProxyFS = R"(#version 330 core
in vec2 vUv;
out vec4 frag;
uniform vec3 uGrid;  // requested U count, V count, opacity
void main() {
    vec2 cell = vUv * max(uGrid.xy, vec2(1.0));
    vec2 phase = fract(cell);
    vec2 edge = min(phase, 1.0 - phase) / max(fwidth(cell), vec2(1e-5));
    float line = 1.0 - smoothstep(0.25, 1.15, min(edge.x, edge.y));
    if (line < 0.02) discard;
    frag = vec4(0.12, 0.92, 1.0, line * uGrid.z);
})";

// ---------------------------------------------------------------------------
// GPU vertex buffers (pos3 + col3).

struct Buffer {
    GLuint vao = 0, vbo = 0;
    int count = 0;

    // Layout: pos3 + col3, plus nrm3 when floatsPerVert == 9 (the solid
    // fill buffer carries smooth-shading normals; everything else stays
    // 6-float).
    void upload(const std::vector<float>& data, int floatsPerVert = 6) {
        if (!vao) {
            glGenVertexArrays(1, &vao);
            glGenBuffers(1, &vbo);
        }
        const GLsizei stride = floatsPerVert * sizeof(float);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), data.data(),
                     GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                              (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        if (floatsPerVert >= 9) {
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride,
                                  (void*)(6 * sizeof(float)));
            glEnableVertexAttribArray(2);
        } else {
            glDisableVertexAttribArray(2);
        }
        glBindVertexArray(0);
        count = static_cast<int>(data.size()) / floatsPerVert;
    }

    void uploadUv(const std::vector<float>& data) {
        if (!vao) {
            glGenVertexArrays(1, &vao);
            glGenBuffers(1, &vbo);
        }
        const GLsizei stride = 5 * sizeof(float);
        glBindVertexArray(vao);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, data.size() * sizeof(float), data.data(),
                     GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride,
                              (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
        glBindVertexArray(0);
        count = static_cast<int>(data.size()) / 5;
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
    // Per-face bake queue (Phase 1): single worker, latest-wins dedupe,
    // full-mesh adopt. Always calls weft::generate() (AD-1).
    weft_app::FaceBakeQueue bakeQueue;
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
    // Pick ORDER of selVerts — weld-to-last/first need it (a set forgets).
    std::vector<uint32_t> selVertOrder;
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
    bool openFacePopup = false;
    bool openWeldPopup = false;  // context popup requested at the cursor

    // Undo: recipe snapshots, one per edit gesture (drags coalesce).
    std::vector<weft::Recipe> undoStack;
    std::vector<weft::Recipe> redoStack;
    weft::Recipe preFrame;
    bool mutatedThisFrame = false;

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
    int lightStyle = 0;  // 0 studio-lit, 1 matcap, 2 flat colour
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
    // Smoothing-angle vertex normals (Blender auto-smooth style): faces
    // meeting under the angle shade smooth, harder creases stay sharp.
    bool smoothShade = true;
    float smoothAngleDeg = 30.0f;
    // Viewer shading with exact CAD surface normals per (vertex, face) —
    // the MoI lesson: shading stops depending on tessellation, so coarse
    // or count-mismatched cylinders never band in the viewport. Verts
    // without a usable normal (poles, manual-op geometry) fall back to
    // the smoothing-angle average. Cache cleared per regenerate.
    bool exactNormals = true;
    std::map<uint64_t, std::array<float, 3>> exactNormalCache;
    // Keybinds help panel (collapsed to a bottom-left prompt by default).
    bool showKeybinds = false;
    // Import and regenerate both run away from the UI thread. OCCT STEP
    // translation/healing can take tens of seconds on a large assembly before
    // meshing even starts, so import needs the same responsiveness guarantee.
    std::thread loadThread;
    std::atomic<bool> loadBusy{false};
    std::atomic<bool> loadReady{false};
    double loadStartTime = 0.0;
    bool loadGenerateAfter = true;
    std::string loadPath;
    std::string loadError;
    weft::Model loadedModel;
    weft::Analysis loadedAnalysis;
    std::vector<weft::EdgePolyline> loadedBrepEdges;

    // Async regenerate: the mesh builds on a worker thread; a centred
    // progress overlay reports faces meshed.
    std::thread genThread;
    std::atomic<bool> genBusy{false};
    std::atomic<bool> genReady{false};
    std::atomic<int> genProgress{0};
    std::atomic<int> genTotal{0};
    double genStartTime = 0.0;
    weft::GenerationSettings genSettings;
    std::vector<weft::ManualOp> genOps;  // worker's frozen ops snapshot
    weft::PolyMesh genMesh;
    weft::GenerationReport genReport;
    int genOpsApplied = 0;
    int genOpsFailed = 0;
    std::string genError;
    // A defaults control being hovered highlights the faces it drives —
    // the live "which parts does this knob change" map.
    std::set<int> highlightMeshers;  // weft::MesherKind values

    // Floating value HUD for modal wheel edits.
    char hudText[64] = "";
    double hudUntil = 0.0;

    // Undo gesture: coalesce a whole drag / wheel burst into one step.
    bool gestureActive = false;
    double lastMutationTime = 0.0;

    // GPU.
    Buffer fill, wire, brep, pick, preview, verts, gpuProxy;
    Buffer allVerts;  // every visible vert, drawn small+black in vert mode
    // Problem overlay: open edges (red) and non-manifold edges (magenta)
    // rebuilt after every regenerate — the trust meter for game export.
    Buffer problems;
    int openEdgeCount = 0, multiEdgeCount = 0;
    int foldedPolyCount = 0;
    bool meshFinalized = false;
    // Screenshot / visual-QA path: force the production finalize pass so
    // headless captures match CLI export (preview leaves seams open).
    bool forceFinalize = false;
    bool gpuProxyPending = false;
    int gpuProxyFace = 0;
    bool showProblems = true;
    // Quality heatmap: tint polys by worst corner angle vs the regular
    // polygon's — pinches and slivers glow before they reach the DCC.
    bool qualityView = false;
    // Fit-to-budget: target polygon count for the density-scale solver.
    int budgetTarget = 5000;
    // Export shaping for game engines.
    int exportFormat = 0;  // 0 = OBJ, 1 = glTF (.glb), 2 = FBX
    bool openExportPopup = false;
    float vertSizeActive = 5.0f;    // px at 1x ui scale
    float vertSizeInactive = 3.0f;
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

// Amber tint while a face is queued/baking so the artist sees which
// regions the background worker is still resolving.
static std::array<float, 3> faceColorWithFidelity(const weft::FaceInfo& f,
                                                  bool selected,
                                                  weft_app::FaceFidelity fid) {
    auto c = faceColor(f, selected);
    if (fid == weft_app::FaceFidelity::Baking) {
        c[0] = 0.45f * c[0] + 0.55f * 0.95f;
        c[1] = 0.45f * c[1] + 0.55f * 0.62f;
        c[2] = 0.45f * c[2] + 0.55f * 0.18f;
    } else if (fid == weft_app::FaceFidelity::Queued ||
               fid == weft_app::FaceFidelity::LowPolyProxy) {
        c[0] = 0.65f * c[0] + 0.35f * 0.85f;
        c[1] = 0.65f * c[1] + 0.35f * 0.70f;
        c[2] = 0.65f * c[2] + 0.35f * 0.35f;
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

    std::vector<float> fill, pick, wire, proxy;
    fill.reserve(m.polygons.size() * 27);
    auto push = [](std::vector<float>& v, const std::array<double, 3>& p,
                   const std::array<float, 3>& c) {
        v.push_back(float(p[0]));
        v.push_back(float(p[1]));
        v.push_back(float(p[2]));
        v.push_back(c[0]);
        v.push_back(c[1]);
        v.push_back(c[2]);
    };

    // Smoothing-angle vertex normals: per-polygon Newell normals,
    // averaged per WELDED vertex over the adjacent polygons within the
    // angle threshold — creases sharper than the angle keep both sides'
    // own normals (Blender auto-smooth). Computed only when smooth
    // shading is on; the fill layout always carries the slot.
    std::vector<std::array<float, 3>> polyN;
    std::vector<std::vector<uint32_t>> vertPolys;
    if (app.smoothShade) {
        polyN.assign(m.polygons.size(), {0, 0, 0});
        vertPolys.assign(m.vertices.size(), {});
        for (size_t i = 0; i < m.polygons.size(); ++i) {
            const auto& poly = m.polygons[i];
            double nx = 0, ny = 0, nz = 0;
            for (size_t k = 0; k < poly.size(); ++k) {
                const auto& a = m.vertices[poly[k]];
                const auto& b = m.vertices[poly[(k + 1) % poly.size()]];
                nx += (a[1] - b[1]) * (a[2] + b[2]);
                ny += (a[2] - b[2]) * (a[0] + b[0]);
                nz += (a[0] - b[0]) * (a[1] + b[1]);
            }
            // Keep the MAGNITUDE: area weighting makes big neighbours
            // dominate slivers in the average, like every DCC.
            polyN[i] = {float(nx), float(ny), float(nz)};
            for (uint32_t v : poly) {
                if (v < vertPolys.size()) vertPolys[v].push_back(uint32_t(i));
            }
        }
    }
    const float cosSmooth =
        std::cos(app.smoothAngleDeg * float(M_PI) / 180.0f);
    auto cornerNormal = [&](size_t polyIdx,
                            uint32_t vert) -> std::array<float, 3> {
        const auto& pn = polyN[polyIdx];
        const float pl =
            std::sqrt(pn[0] * pn[0] + pn[1] * pn[1] + pn[2] * pn[2]);
        if (pl < 1e-20f) return {0, 0, 0};
        float sx = 0, sy = 0, sz = 0;
        for (uint32_t j : vertPolys[vert]) {
            const auto& qn = polyN[j];
            const float ql =
                std::sqrt(qn[0] * qn[0] + qn[1] * qn[1] + qn[2] * qn[2]);
            if (ql < 1e-20f) continue;
            const float d =
                (pn[0] * qn[0] + pn[1] * qn[1] + pn[2] * qn[2]) / (pl * ql);
            if (d < cosSmooth) continue;
            sx += qn[0];
            sy += qn[1];
            sz += qn[2];
        }
        const float sl = std::sqrt(sx * sx + sy * sy + sz * sz);
        if (sl < 1e-20f) return {pn[0] / pl, pn[1] / pl, pn[2] / pl};
        return {sx / sl, sy / sl, sz / sl};
    };
    // Exact CAD corner normals: the true surface normal of the polygon's
    // face at each vertex — anchors give (u,v) for free; border verts
    // anchored to the neighbouring face project once and cache. Shading
    // then reads from the SURFACE, not the tessellation, so a coarse or
    // count-mismatched cylinder stack cannot band in the viewport (the
    // exporters already write these; this is viewer parity).
    std::map<int, BRepAdaptor_Surface> surfCache;
    auto exactCorner = [&](uint32_t vert, int fid,
                           std::array<float, 3>& out) -> bool {
        if (!app.exactNormals || !app.hasModel || fid < 1 ||
            fid > app.model.faceCount() || vert >= m.anchors.size()) {
            return false;
        }
        const uint64_t key = (uint64_t(vert) << 32) | uint32_t(fid);
        auto it = app.exactNormalCache.find(key);
        if (it == app.exactNormalCache.end()) {
            std::array<float, 3> n{0, 0, 0};  // zero = no unique normal
            try {
                const TopoDS_Face face =
                    TopoDS::Face(app.model.faces(fid));
                auto sit = surfCache.find(fid);
                if (sit == surfCache.end()) {
                    sit = surfCache.emplace(fid, BRepAdaptor_Surface(face))
                              .first;
                }
                double u = 0, v = 0;
                bool have = false;
                const weft::Anchor& a = m.anchors[vert];
                if (a.faceId == fid) {
                    u = a.u;
                    v = a.v;
                    have = true;
                } else {
                    Handle(Geom_Surface) hs = BRep_Tool::Surface(face);
                    if (!hs.IsNull()) {
                        gp_Pnt p(m.vertices[vert][0], m.vertices[vert][1],
                                 m.vertices[vert][2]);
                        GeomAPI_ProjectPointOnSurf proj(p, hs);
                        if (proj.NbPoints() >= 1) {
                            proj.LowerDistanceParameters(u, v);
                            have = true;
                        }
                    }
                }
                if (have) {
                    gp_Pnt p;
                    gp_Vec du, dv;
                    sit->second.D1(u, v, p, du, dv);
                    gp_Vec nn = du.Crossed(dv);
                    if (nn.Magnitude() > 1e-14) {
                        nn.Normalize();
                        if (face.Orientation() == TopAbs_REVERSED) {
                            nn.Reverse();
                        }
                        n = {float(nn.X()), float(nn.Y()), float(nn.Z())};
                    }
                }
            } catch (const Standard_Failure&) {
            }
            it = app.exactNormalCache.emplace(key, n).first;
        }
        out = it->second;
        return out[0] != 0.0f || out[1] != 0.0f || out[2] != 0.0f;
    };
    auto pushN = [&](std::vector<float>& v, size_t polyIdx, uint32_t vert) {
        if (!app.smoothShade) {
            v.insert(v.end(), {0.0f, 0.0f, 0.0f});
            return;
        }
        std::array<float, 3> en;
        if (polyIdx < m.polygonFaceId.size() &&
            exactCorner(vert, m.polygonFaceId[polyIdx], en)) {
            v.insert(v.end(), {en[0], en[1], en[2]});
            return;
        }
        const auto n = cornerNormal(polyIdx, vert);
        v.insert(v.end(), {n[0], n[1], n[2]});
    };

    static const weft::FaceInfo kBridgeInfo{};  // bridge strips: faceId 0
    app.fillSegs.clear();
    app.polyFillRange.assign(m.polygons.size(), {-1, 0});
    for (size_t i = 0; i < m.polygons.size(); ++i) {
        int fid = m.polygonFaceId[i];
        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
        int segStart = int(fill.size() / 9);
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
                : faceColorWithFidelity(
                      info, fid > 0 && app.selFaces.count(fid) > 0,
                      fid > 0 ? app.bakeQueue.fidelity(uint32_t(fid))
                              : weft_app::FaceFidelity::HighFidelity);
        std::array<float, 3> id{float(fid & 255) / 255.0f,
                                float((fid >> 8) & 255) / 255.0f,
                                170.0f / 255.0f};
        const auto& poly = m.polygons[i];
        if (poly.size() <= 4) {
            for (size_t k = 1; k + 1 < poly.size(); ++k) {  // fan
                for (size_t c : {size_t(0), k, k + 1}) {
                    push(fill, m.vertices[poly[c]], col);
                    pushN(fill, i, poly[c]);
                    push(pick, m.vertices[poly[c]], id);
                }
            }
        } else {
            // N-gons ear-clip: a fan across a concave or keyhole ring
            // (minimal n-gon with bridged holes) would paint over the
            // holes — the mesh is right, the fan isn't.
            for (const auto& t : weft::triangulatePoly(m.vertices, poly)) {
                for (int c = 0; c < 3; ++c) {
                    push(fill, m.vertices[poly[t[c]]], col);
                    pushN(fill, i, poly[t[c]]);
                    push(pick, m.vertices[poly[t[c]]], id);
                }
            }
        }
        std::array<float, 3> wc{0.10f, 0.11f, 0.13f};
        for (size_t k = 0; k < poly.size(); ++k) {
            push(wire, m.vertices[poly[k]], wc);
            push(wire, m.vertices[poly[(k + 1) % poly.size()]], wc);
        }
        app.fillSegs.back()[2] = int(fill.size() / 9) - app.fillSegs.back()[1];
        app.polyFillRange[i][1] = int(fill.size() / 9) - app.polyFillRange[i][0];
    }

    // Build UV-bearing triangles only for the active face. This buffer is
    // stable for the life of the current exact mesh; density edits change two
    // shader uniforms, never CPU geometry or GPU uploads.
    if (app.activeFace >= 1 && app.activeFace <= app.model.faceCount()) {
        try {
            const int fid = app.activeFace;
            const TopoDS_Face face = TopoDS::Face(app.model.faces(fid));
            BRepAdaptor_Surface surf(face);
            const double u0 = surf.FirstUParameter();
            const double u1 = surf.LastUParameter();
            const double v0 = surf.FirstVParameter();
            const double v1 = surf.LastVParameter();
            const double du = u1 - u0, dv = v1 - v0;
            if (std::isfinite(du) && std::isfinite(dv) &&
                std::abs(du) > 1e-12 && std::abs(dv) > 1e-12) {
                std::map<uint32_t, std::array<double, 2>> uvCache;
                auto uvOf = [&](uint32_t vi,
                                std::array<double, 2>& uv) -> bool {
                    auto hit = uvCache.find(vi);
                    if (hit != uvCache.end()) {
                        uv = hit->second;
                        return std::isfinite(uv[0]) && std::isfinite(uv[1]);
                    }
                    weft::Anchor a;
                    if (vi < m.anchors.size()) a = m.anchors[vi];
                    if (a.faceId != fid) {
                        std::array<double, 3> point = m.vertices[vi];
                        a = weft::snapToFace(app.model, fid, point);
                    }
                    uv = {(a.u - u0) / du, (a.v - v0) / dv};
                    uvCache[vi] = uv;
                    return a.faceId == fid && std::isfinite(uv[0]) &&
                           std::isfinite(uv[1]);
                };
                auto emitTri = [&](uint32_t a, uint32_t b, uint32_t c) {
                    std::array<uint32_t, 3> vi{a, b, c};
                    std::array<std::array<double, 2>, 3> uv;
                    for (int k = 0; k < 3; ++k) {
                        if (!uvOf(vi[k], uv[k])) return;
                    }
                    // Keep periodic seam triangles local in UV space so the
                    // procedural grid does not streak across the full chart.
                    for (int axis = 0; axis < 2; ++axis) {
                        const bool periodic = axis == 0 ? surf.IsUPeriodic()
                                                        : surf.IsVPeriodic();
                        if (!periodic) continue;
                        double lo = uv[0][axis], hi = lo;
                        for (int k = 1; k < 3; ++k) {
                            lo = std::min(lo, uv[k][axis]);
                            hi = std::max(hi, uv[k][axis]);
                        }
                        if (hi - lo > 0.5) {
                            for (int k = 0; k < 3; ++k) {
                                if (uv[k][axis] < 0.5) uv[k][axis] += 1.0;
                            }
                        }
                    }
                    for (int k = 0; k < 3; ++k) {
                        const auto& p = m.vertices[vi[k]];
                        proxy.insert(proxy.end(), {float(p[0]), float(p[1]),
                                                   float(p[2]),
                                                   float(uv[k][0]),
                                                   float(uv[k][1])});
                    }
                };
                for (size_t pi = 0; pi < m.polygons.size(); ++pi) {
                    if (m.polygonFaceId[pi] != fid) continue;
                    const auto& poly = m.polygons[pi];
                    if (poly.size() <= 4) {
                        for (size_t k = 1; k + 1 < poly.size(); ++k) {
                            emitTri(poly[0], poly[k], poly[k + 1]);
                        }
                    } else {
                        for (const auto& t :
                             weft::triangulatePoly(m.vertices, poly)) {
                            emitTri(poly[t[0]], poly[t[1]], poly[t[2]]);
                        }
                    }
                }
            }
        } catch (const Standard_Failure&) {
            proxy.clear();
        }
    }
    app.fill.upload(fill, 9);
    app.pick.upload(pick);
    app.wire.upload(wire);
    app.gpuProxy.uploadUv(proxy);
    app.gpuProxyFace = proxy.empty() ? 0 : app.activeFace;

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

    // Every visible vertex, for vert mode: unselected verts read as small
    // dark points (Blender-style); selection/hover markers draw on top.
    std::vector<float> allV;
    {
        std::set<uint32_t> seen;
        for (size_t i = 0; i < m.polygons.size(); ++i) {
            int fid = m.polygonFaceId[i];
            if (fid > 0 && app.hiddenFaces.count(fid)) continue;
            for (uint32_t v : m.polygons[i]) {
                if (!seen.insert(v).second) continue;
                push(allV, m.vertices[v], {1, 1, 1});
            }
        }
    }
    app.allVerts.upload(allV);

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
// the export pipeline cares about: zero of both = watertight. Folded
// cells (winding against the surface normal) get orange outlines: they
// keep the mesh manifold — a region doubled back over its neighbour pairs
// every directed edge — so the edge scan alone would never show them.
static void updateProblems(App& app) {
    app.openEdgeCount = 0;
    app.multiEdgeCount = 0;
    app.foldedPolyCount = 0;
    std::map<std::pair<uint32_t, uint32_t>, int> dir;
    if (app.meshFinalized) {
        for (const auto& poly : app.mesh.polygons) {
            for (size_t i = 0; i < poly.size(); ++i) {
                ++dir[{poly[i], poly[(i + 1) % poly.size()]}];
            }
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
    if (app.meshFinalized) {
        for (const auto& [e, count] : dir) {
            if (count > 1) {
                pushEdge(e.first, e.second, 1.0f, 0.2f, 0.9f);  // magenta
                app.multiEdgeCount += count - 1;
            } else if (!dir.count({e.second, e.first})) {
                pushEdge(e.first, e.second, 1.0f, 0.25f, 0.15f);  // red
                ++app.openEdgeCount;
            }
        }
    }
    const std::vector<uint8_t> folded =
        weft::foldedPolys(app.model, app.mesh);
    for (size_t p = 0; p < folded.size(); ++p) {
        if (!folded[p]) continue;
        ++app.foldedPolyCount;
        const auto& poly = app.mesh.polygons[p];
        for (size_t i = 0; i < poly.size(); ++i) {
            pushEdge(poly[i], poly[(i + 1) % poly.size()], 1.0f, 0.12f,
                     0.12f);  // red outline
        }
    }
    app.problems.upload(lines);
    if (lines.empty()) app.problems.count = 0;
}

static void finishGenerate(App& app);

// Snapshot recipe → bake queue. faceId 0 = model-wide (load / defaults).
static void enqueueBake(App& app, uint32_t faceId) {
    if (!app.hasModel) return;
    weft::GenerationSettings s = app.recipe.settings;
    s.finalizeMesh = app.forceFinalize || app.liveLink;
    app.bakeQueue.enqueue(faceId, s, app.recipe.ops);
    app.dirty = true;
    app.genBusy = true;
    app.genStartTime = glfwGetTime();
    app.genProgress = 0;
    app.genTotal = -1;
}

// Kick a bake: UI thread never blocks on meshing. Settings are snapshotted
// inside the queue; further edits coalesce via latest-wins pending overwrite.
static void startGenerate(App& app) {
    if (!app.hasModel) return;
    logLine("regenerate: begin (%zu overrides, %zu edge pins, %zu ops)",
            app.recipe.settings.perFace.size(),
            app.recipe.settings.perEdge.size(), app.recipe.ops.size());
    app.genError.clear();
    app.dirty = false;
    enqueueBake(app, 0);  // model-wide trigger; pending faces still coalesce
}

// Worker finished: adopt its mesh on the UI thread and rebuild all the
// GL-side derived state. Failures keep the previous mesh (ctrl+Z path).
static void frameModel(App& app);

static void adoptBakeResult(App& app, weft_app::FaceBakeResult& result) {
    app.genBusy = app.bakeQueue.busy();
    app.genReady = false;
    const bool firstMesh = app.mesh.vertices.empty();
    if (!result.error.empty()) {
        logLine("regenerate: FAILED: %s", result.error.c_str());
        app.status = "regenerate failed (ctrl+Z): " + result.error;
        return;
    }
    logLine("regenerate: generate took %.1f ms (triggers=%zu gen=%llu)",
            (glfwGetTime() - app.genStartTime) * 1000.0,
            result.triggerFaces.size(),
            (unsigned long long)result.generation);
    {
        app.mesh = std::move(result.mesh);
        app.meshFinalized = result.finalizeMesh;
        app.gpuProxyPending = app.dirty && app.activeFace > 0;
        app.report = std::move(result.report);
        app.exactNormalCache.clear();
        app.selPolys.clear();
        app.selVerts.clear();
        app.selVertOrder.clear();
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
    }
    logLine("regenerate: ops applied (%d ok, %d failed), rebuilding buffers",
            result.opsApplied, result.opsFailed);
    if (result.opsFailed > 0) {
        app.status = "regen: " + std::to_string(result.opsFailed) +
                     " correction(s) did not apply — check recipe / undo";
    }
    updateProblems(app);

    app.brepEdges = weft::sampleEdges(app.model, 28,
                                      app.report.edgeDivisions);
    const VertexKdTree meshVertices(app.mesh.vertices);
    for (weft::EdgePolyline& e : app.brepEdges) {
        if (!app.report.edgeDivisions.count(e.edgeId)) continue;
        if (e.points.size() < 2) continue;
        double cl2 = 0;
        {
            double dx = e.points[1][0] - e.points[0][0];
            double dy = e.points[1][1] - e.points[0][1];
            double dz = e.points[1][2] - e.points[0][2];
            cl2 = (dx * dx + dy * dy + dz * dz) * 0.36;
        }
        for (auto& p : e.points) {
            double best = cl2;
            uint32_t hit = 0;
            if (meshVertices.nearestWithin(p, best, hit)) {
                p = app.mesh.vertices[hit];
            }
        }
    }

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
    if (firstMesh) frameModel(app);
    logLine("regenerate: done (%zu verts, %zu polys; %d remeshed, %d reused)",
            app.mesh.vertexCount(), app.mesh.polygonCount(),
            app.report.cacheMisses, app.report.cacheHits);
    if (!firstMesh && app.report.cacheHits > 0) {
        app.status = "updated " + std::to_string(app.report.cacheMisses) +
                     " face(s), reused " +
                     std::to_string(app.report.cacheHits);
    }

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

static void finishGenerate(App& app) {
    // Legacy entry: drain bake-queue completions (genThread path retired).
    for (auto& r : app.bakeQueue.pollCompleted()) {
        adoptBakeResult(app, r);
    }
    app.genBusy = app.bakeQueue.busy();
}

// Synchronous regenerate for flows that need the fresh mesh in hand
// (budget fitting, recipe remap, load-with-recipe): drain the bake queue
// and wait for one settled run.
static void regenerate(App& app) {
    if (!app.hasModel) return;
    auto drain = [&] {
        while (app.bakeQueue.busy() || app.bakeQueue.hasPending()) {
            finishGenerate(app);
            if (app.bakeQueue.busy() || app.bakeQueue.hasPending()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        finishGenerate(app);
    };
    drain();
    startGenerate(app);
    drain();
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
        // Prefer visible (non-hidden) faces so isolate+frame focuses the
        // remaining object instead of the full assembly bbox.
        for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
            const int fid = app.mesh.polygonFaceId[p];
            if (fid > 0 && app.hiddenFaces.count(fid)) continue;
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

// Outliner isolate: hide every face not owned by solidIndex (0-based), select
// that solid's faces, and frame the selection.
static bool isolateSolidObject(App& app, size_t solidIndex) {
    if (solidIndex >= app.analysis.solidFaces.size()) return false;
    const std::vector<int>& fids = app.analysis.solidFaces[solidIndex];
    app.hiddenFaces.clear();
    app.selFaces.clear();
    std::set<int> inSolid(fids.begin(), fids.end());
    for (const auto& fi : app.analysis.faces) {
        if (!inSolid.count(fi.id)) app.hiddenFaces.insert(fi.id);
    }
    // Frame while selected (bbox of solid), then fully deselect and rebuild
    // so no orange selection fill/verts remain — wireframe must be readable.
    for (int fid : fids) app.selFaces.insert(fid);
    app.activeFace = fids.empty() ? 0 : fids.front();
    rebuildBuffers(app);
    frameModel(app);
    app.selFaces.clear();
    app.selEdges.clear();
    app.activeFace = 0;
    rebuildBuffers(app);
    return true;
}

static std::string solidObjectFileStem(const App& app, size_t solidIndex) {
    char buf[96];
    std::string nm;
    if (solidIndex < app.model.solidNames.size())
        nm = app.model.solidNames[solidIndex];
    for (char& c : nm) {
        if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-' ||
              c == '_')) {
            c = '_';
        }
    }
    if (!nm.empty()) {
        std::snprintf(buf, sizeof buf, "object_%03zu_%s", solidIndex + 1,
                      nm.c_str());
    } else {
        std::snprintf(buf, sizeof buf, "object_%03zu", solidIndex + 1);
    }
    return buf;
}

static void finishLoadModel(App& app) {
    if (app.loadThread.joinable()) app.loadThread.join();
    app.loadBusy = false;
    app.loadReady = false;
    if (!app.loadError.empty()) {
        logLine("load: FAILED after %.1f ms: %s",
                (glfwGetTime() - app.loadStartTime) * 1000.0,
                app.loadError.c_str());
        app.status = "load failed: " + app.loadError;
        return;
    }
    const std::string path = app.loadPath;
    // The main loop only adopts this result after the old model's generation
    // worker has drained, so moving the document cannot race the mesher.
    try {
        app.model = std::move(app.loadedModel);
        app.analysis = std::move(app.loadedAnalysis);
        app.brepEdges = std::move(app.loadedBrepEdges);
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
        // The old model's report must not outlive it: its face ids and
        // build-health flags would render (and be clickable) against the
        // new model until the first async run lands.
        app.report = weft::GenerationReport();
        app.recipe = {};
        // New sessions solve curvature adaptively (deviation/angle drive
        // each edge's count); saved recipes bring their own flag back.
        app.recipe.settings.defaults.adaptive = true;
        // Deviation relative to feature size: a 500mm bore and a 5mm bore
        // carry the same ring topology, the angle criterion drives counts.
        app.recipe.settings.defaults.relativeDeviation = true;
        // Readable cylinders: never below 24 circumferential spans.
        app.recipe.settings.defaults.minCurvedSegments = 24;
        app.bakeQueue.clearPending();
        app.bakeQueue.bind(&app.model, &app.analysis, &app.genCache);
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
                app.status += "  (recipe loaded)";
                logLine("load: applied %s", app.recipePath.c_str());
            } catch (const std::exception& e) {
                app.status = std::string("recipe load failed: ") + e.what();
            }
        }
        // Start once, after a sidecar recipe has replaced the defaults. This
        // avoids generating the same large model twice during open.
        if (app.loadGenerateAfter) startGenerate(app);
        logLine("load: ready in %.1f ms (%d faces, %d edges)",
                (glfwGetTime() - app.loadStartTime) * 1000.0,
                app.model.faceCount(), app.model.edgeCount());
    } catch (const std::exception& e) {
        app.status = std::string("load failed: ") + e.what();
    }
}

static void loadModel(App& app, const std::string& path,
                      bool generateAfter = true) {
    if (path.empty()) return;
    if (app.loadBusy) {
        app.status = "already loading " + app.loadPath;
        return;
    }
    // Drain bake worker before the load thread prepares a replacement model.
    while (app.bakeQueue.busy() || app.bakeQueue.hasPending()) {
        finishGenerate(app);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    finishGenerate(app);
    app.bakeQueue.clearPending();
    logLine("load: %s", path.c_str());
    if (app.loadThread.joinable()) app.loadThread.join();
    app.loadPath = path;
    app.loadError.clear();
    app.loadedModel = {};
    app.loadedAnalysis = {};
    app.loadedBrepEdges.clear();
    app.loadGenerateAfter = generateAfter;
    app.loadStartTime = glfwGetTime();
    app.loadBusy = true;
    app.loadReady = false;
    app.status = "loading " + path;
    App* a = &app;
    app.loadThread = std::thread([a] {
        try {
            weft::Model model = weft::loadStep(a->loadPath);
            weft::Analysis analysis = weft::analyze(model);
            std::vector<weft::EdgePolyline> edges =
                weft::sampleEdges(model, 28);
            a->loadedModel = std::move(model);
            a->loadedAnalysis = std::move(analysis);
            a->loadedBrepEdges = std::move(edges);
        } catch (const std::exception& e) {
            a->loadError = e.what();
        } catch (...) {
            a->loadError = "unknown exception";
        }
        a->loadReady = true;
    });
}

// Hot-reload: the source STEP changed on disk (the CAD app re-exported
// over it). Re-import, remap the LIVE recipe onto the new B-rep by
// geometric signature — the recipe file on disk is not re-read, so
// unsaved edits survive — and regenerate; the Blender live link then
// pushes the result downstream. Selection and hidden faces follow the
// same mapping. The undo stack refers to old ids, so it resets.
static void reloadModel(App& app) {
    logLine("hot-reload: %s", app.sourcePath.c_str());
    // Never swap the model out from under a running bake worker.
    while (app.bakeQueue.busy() || app.bakeQueue.hasPending()) {
        finishGenerate(app);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    finishGenerate(app);
    app.bakeQueue.clearPending();
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
        app.bakeQueue.bind(&app.model, &app.analysis, &app.genCache);

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

#ifndef _WIN32
// Whether the last dialog attempt found NO dialog tool at all — the
// panel then tells the user to type a path instead of doing nothing.
static bool gNoDialogTool = false;
static std::string runDialog(const char* zenityCmd, const char* kdialogCmd) {
    gNoDialogTool = false;
    for (const char* cmd : {zenityCmd, kdialogCmd}) {
        FILE* p = popen(cmd, "r");
        if (!p) continue;
        char buf[1024] = "";
        std::string r;
        if (fgets(buf, sizeof buf, p)) {
            r = buf;
            while (!r.empty() && (r.back() == '\n' || r.back() == '\r')) {
                r.pop_back();
            }
        }
        // 0 = picked, 1 = cancelled; 127/126 = tool missing, try next.
        const int rc = pclose(p);
        const int code = WIFEXITED(rc) ? WEXITSTATUS(rc) : 127;
        if (code == 0 && !r.empty()) return r;
        if (code <= 1) return "";  // real dialog, user cancelled
    }
    gNoDialogTool = true;
    return "";
}
#endif

// Native "open file" dialog: comdlg32 on Windows, zenity/kdialog on
// Linux (the panel's text field is the fallback when neither exists).
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
    return runDialog(
        "zenity --file-selection --title='Open STEP' "
        "--file-filter='STEP | *.step *.stp *.STEP *.STP' 2>/dev/null",
        "kdialog --getopenfilename . "
        "'STEP files (*.step *.stp *.STEP *.STP)' 2>/dev/null");
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
    return runDialog(
        ("zenity --file-selection --save --title='Export OBJ' "
         "--filename='" + std::string(defaultName) + "' 2>/dev/null")
            .c_str(),
        ("kdialog --getsavefilename '" + std::string(defaultName) +
         "' 2>/dev/null")
            .c_str());
#endif
}

// Built-in fallback file browser: an ImGui modal used whenever no
// native dialog exists (Linux without zenity/kdialog). Zero external
// dependencies, so Open/Export work on every system out of the box.
struct FileBrowser {
    bool open = false;
    bool saveMode = false;
    std::string dir;
    char nameBuf[512] = "";
    std::string picked;  // consumed by the panel code once non-empty
    std::vector<std::string> dirs, files;

    void start(bool save, const char* defaultName) {
        open = true;
        saveMode = save;
        std::snprintf(nameBuf, sizeof nameBuf, "%s",
                      defaultName ? defaultName : "");
        if (dir.empty()) dir = homeDir();
        refresh();
    }
    static std::string homeDir() {
        const char* h = std::getenv("HOME");
#ifdef _WIN32
        if (!h || !*h) h = std::getenv("USERPROFILE");
#endif
        return h && *h ? h : ".";
    }
    void refresh() {
        dirs.clear();
        files.clear();
        std::error_code ec;
        for (const auto& e :
             std::filesystem::directory_iterator(dir, ec)) {
            std::string n = e.path().filename().string();
            if (n.empty() || n[0] == '.') continue;
            std::error_code ec2;
            if (e.is_directory(ec2)) {
                dirs.push_back(n);
                continue;
            }
            std::string low = n;
            for (char& c : low) c = char(std::tolower((unsigned char)c));
            const bool stepish =
                low.size() > 4 && (low.rfind(".step") == low.size() - 5 ||
                                   low.rfind(".stp") == low.size() - 4);
            if (saveMode || stepish) files.push_back(n);
        }
        std::sort(dirs.begin(), dirs.end());
        std::sort(files.begin(), files.end());
    }
    void draw() {
        const char* title = saveMode ? "Export##fb" : "Open STEP##fb";
        if (open && !ImGui::IsPopupOpen(title)) ImGui::OpenPopup(title);
        ImGui::SetNextWindowSize({560, 440}, ImGuiCond_Appearing);
        if (!ImGui::BeginPopupModal(title, &open)) return;
        ImGui::TextWrapped("%s", dir.c_str());
        if (ImGui::SmallButton("up")) {
            std::filesystem::path p(dir);
            if (p.has_parent_path() && p.parent_path() != p) {
                dir = p.parent_path().string();
                refresh();
            }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("home")) {
            dir = homeDir();
            refresh();
        }
        const float foot = 2.2f * ImGui::GetFrameHeightWithSpacing();
        if (ImGui::BeginChild("##fbList", {0, -foot}, true)) {
            for (size_t i = 0; i < dirs.size(); ++i) {
                if (ImGui::Selectable((dirs[i] + "/").c_str())) {
                    dir = (std::filesystem::path(dir) / dirs[i]).string();
                    refresh();
                    break;
                }
            }
            for (const std::string& f : files) {
                if (ImGui::Selectable(f.c_str())) {
                    if (saveMode) {
                        std::snprintf(nameBuf, sizeof nameBuf, "%s",
                                      f.c_str());
                    } else {
                        picked =
                            (std::filesystem::path(dir) / f).string();
                        open = false;
                        ImGui::CloseCurrentPopup();
                    }
                }
            }
        }
        ImGui::EndChild();
        if (saveMode) {
            ImGui::SetNextItemWidth(-110);
            ImGui::InputText("##fbName", nameBuf, sizeof nameBuf);
            ImGui::SameLine();
            if (ImGui::Button("Save", {-1, 0}) && nameBuf[0]) {
                picked = (std::filesystem::path(dir) / nameBuf).string();
                open = false;
                ImGui::CloseCurrentPopup();
            }
        } else {
            ImGui::TextDisabled("pick a .step / .stp file");
        }
        ImGui::EndPopup();
    }
};
static FileBrowser gBrowser;
#ifdef _WIN32
static const bool gNoDialogTool = false;  // comdlg32 always exists
#endif

static std::string tempDir() {
    for (const char* var : {"TMPDIR", "TMP", "TEMP"}) {
        if (const char* d = std::getenv(var); d && *d) return d;
    }
#ifndef _WIN32
    // Linux desktops rarely set TMPDIR; "." is often read-only (app
    // launched from a file manager). /tmp is the convention.
    return "/tmp";
#else
    return gDataDir.empty() ? "." : gDataDir;
#endif
}

static weft::PolyMesh finalizedMeshForExport(App& app) {
    // Drain the bake queue before a sync finalize generate (shared cache).
    while (app.bakeQueue.busy() || app.bakeQueue.hasPending()) {
        finishGenerate(app);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    finishGenerate(app);

    weft::GenerationSettings settings = app.recipe.settings;
    settings.finalizeMesh = true;
    settings.progressFaces = nullptr;
    settings.progressTotal = nullptr;
    weft::GenerationReport report;
    weft::PolyMesh mesh =
        weft::generate(app.model, app.analysis, settings, &report,
                       &app.genCache);
    weft::applyOps(mesh, app.model, app.recipe.ops);
    return mesh;
}

static void exportObjTo(App& app, const std::string& out,
                        const weft::PolyMesh& mesh);

// Format dispatch by extension: the dialog seeds the right one, and a
// hand-typed path still lands with the exporter it names.
static void exportMeshTo(App& app, const std::string& out) {
    std::string ext;
    size_t dot = out.find_last_of('.');
    if (dot != std::string::npos) {
        ext = out.substr(dot + 1);
        for (char& c : ext) c = char(std::tolower(c));
    }
    try {
        app.status = "finalizing mesh for export...";
        const double finalizeStart = glfwGetTime();
        weft::PolyMesh exportMesh = finalizedMeshForExport(app);
        logLine("export: finalization took %.1f ms",
                (glfwGetTime() - finalizeStart) * 1000.0);
        if (ext == "glb" || ext == "gltf") {
            // writeGlb bakes exact CAD normals; engine-space knobs
            // apply to a transformed copy (glTF is Y-up by spec).
            weft::PolyMesh copy = exportMesh;
            for (auto& v : copy.vertices) {
                double x = v[0] * app.exportScale;
                double y = v[1] * app.exportScale;
                double z = v[2] * app.exportScale;
                if (app.exportYUp) {
                    v = {x, z, -y};
                } else {
                    v = {x, y, z};
                }
            }
            weft::writeGlb(copy, out,
                           app.exportYUp || app.exportScale != 1.0f
                               ? nullptr  // moved verts: recompute
                               : &app.model,
                           &app.analysis.solidFaces);
            app.status = "exported " + out;
        } else if (ext == "fbx") {
            weft::FbxExportOptions fo;
            fo.triangulate = app.exportTriangulate;
            fo.yUp = app.exportYUp;
            fo.scale = app.exportScale;
            weft::writeFbx(exportMesh, out, fo);
            app.status = "exported " + out;
        } else {
            exportObjTo(app, out, exportMesh);
        }
    } catch (const std::exception& e) {
        app.status = std::string("export failed: ") + e.what();
    }
}

static void exportObjTo(App& app, const std::string& out,
                        const weft::PolyMesh& mesh) {
    weft::ObjExportOptions opts;
    opts.triangulate = app.exportTriangulate;
    opts.yUp = app.exportYUp;
    opts.scale = double(app.exportScale);
    // CAD part names ride through to the export's o-blocks.
    opts.objectNames = &app.model.solidNames;
    weft::writeObj(mesh, out, &app.analysis.solidFaces, &opts);
    app.status = "exported " + out;
    logLine("export: %s (%zu verts, %zu polys)", out.c_str(),
            mesh.vertexCount(), mesh.polygonCount());
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
// Per-face fidelity flips to Queued immediately; the debounced main-loop
// flush enqueues a latest-wins bake so camera stays free.
static void markDirty(App& app) {
    app.dirty = true;
    app.mutatedThisFrame = true;
    app.lastMutationTime = glfwGetTime();
    app.gpuProxyPending = app.activeFace > 0;
    if (app.activeFace > 0) {
        app.bakeQueue.noteQueued(uint32_t(app.activeFace));
    }
}

// The solved subdivision total around a face's OUTER loop — the honest
// seed when a pinned boundary total switches on (seeding from 0 or a
// tiny constant collapses the whole neighbourhood to 1-per-edge pins).
static int outerLoopSolvedTotal(App& app, int faceId) {
    if (!app.hasModel || faceId < 1 || faceId > app.model.faceCount()) {
        return 0;
    }
    try {
        TopoDS_Wire w = BRepTools::OuterWire(
            TopoDS::Face(app.model.faces(faceId)));
        int total = 0;
        for (TopExp_Explorer ex(w, TopAbs_EDGE); ex.More(); ex.Next()) {
            int eid = app.model.edges.FindIndex(ex.Current());
            auto it = app.report.edgeDivisions.find(eid);
            if (eid > 0 && it != app.report.edgeDivisions.end()) {
                total += it->second;
            }
        }
        return total;
    } catch (const std::exception&) {
        return 0;
    }
}

// The solved primary/secondary counts the mesher used for a face (nu/nv:
// radial/axial, gridU/gridV, ...). {0,0} when unavailable. Leaving
// adaptive seeds the manual fields from these so the value the face was
// already meshed at appears in the box — no dead zone before a manual
// count climbs past the adaptive floor.
static std::array<int, 2> faceSolvedCounts(App& app, int faceId) {
    auto it = app.report.faceCounts.find(faceId);
    if (it == app.report.faceCounts.end()) return {0, 0};
    return it->second;
}

// The mesher kind that drives a face RIGHT NOW. A forced choice wins —
// the wheel and the panels must edit the fields the forced mesher reads,
// even before it has rebuilt — otherwise the kind the last generate
// actually used. Faces the report hasn't seen yet key off RevolutionGrid
// (the count-style nudge default). Every UI surface that lists or edits
// kind-specific parameters resolves through this one helper so they can
// never disagree with each other.
static weft::MesherKind effectiveKind(const App& app, int fid) {
    if (app.hasModel && fid > 0) {
        const weft::FaceMeshSettings& s = app.recipe.settings.forFace(fid);
        if (s.forceMesher > 0) return weft::MesherKind(s.forceMesher - 1);
        auto it = app.report.faceMesher.find(fid);
        if (it != app.report.faceMesher.end()) return it->second;
    }
    return weft::MesherKind::RevolutionGrid;
}

// One-line face identity for multi-select rosters and topology debugging:
// "#1446  bspline  [coons-grid]  contract-floor".
static std::string faceDebugLabel(const App& app, int fid) {
    char buf[256];
    if (fid < 1 || fid > int(app.analysis.faces.size())) {
        std::snprintf(buf, sizeof buf, "#%d  (stale id)", fid);
        return buf;
    }
    const weft::FaceInfo& f = app.analysis.faces[fid - 1];
    const char* build = "";
    auto bit = app.report.faceBuild.find(fid);
    if (bit != app.report.faceBuild.end()) {
        if (bit->second == -1) build = "  EMPTY";
        else if (bit->second == 1) build = "  raw-fallback";
        else if (bit->second == 2) build = "  contract-floor";
    }
    std::snprintf(buf, sizeof buf, "#%d  %s%s%s  %s/%s  [%s]%s", f.id,
                  weft::surfaceTypeName(f.type),
                  f.isFillet ? " [fillet]" : "", f.isHole ? " [hole]" : "",
                  weft::featureClassName(f.featureClass),
                  weft::chartKindName(f.chartKind),
                  weft::mesherKindName(effectiveKind(app, fid)), build);
    return buf;
}

// Scrollable list of every selected B-rep face. Click a row to make it
// active (knobs below edit that face) without shrinking the selection —
// shift-clicks in the viewport still extend the set.
static void drawSelectedFacesRoster(App& app) {
    if (app.selFaces.size() <= 1) return;
    ImGui::TextDisabled("%zu selected — click a row to edit that face",
                        app.selFaces.size());
    const float rowH = ImGui::GetTextLineHeightWithSpacing();
    const float h = std::min(rowH * 8.5f, rowH * float(app.selFaces.size() + 1));
    if (!ImGui::BeginChild("##selfaces", ImVec2(0.0f, h), true)) {
        ImGui::EndChild();
        return;
    }
    for (int fid : app.selFaces) {
        const bool active = fid == app.activeFace;
        ImGui::PushID(fid);
        if (ImGui::Selectable(faceDebugLabel(app, fid).c_str(), active)) {
            app.activeFace = fid;
        }
        if (active && ImGui::IsWindowAppearing()) {
            ImGui::SetScrollHereY(0.25f);
        }
        ImGui::PopID();
    }
    ImGui::EndChild();
}

static std::array<float, 2> gpuProxyCounts(App& app) {
    const int fid = app.activeFace;
    if (fid < 1) return {1.0f, 1.0f};
    const weft::FaceMeshSettings& s = app.recipe.settings.forFace(fid);
    const weft::MesherKind kind = effectiveKind(app, fid);
    std::array<int, 2> n = faceSolvedCounts(app, fid);
    auto manual = [&](int u, int v) {
        n = {std::max(1, u), std::max(1, v)};
    };
    // Blend strips: semantic knobs → parametric U/V for the isoline
    // overlay. faceAcross 1 = loops ride U; 2 = loops ride V. Without
    // this remap the proxy densifies the wrong GPU axis when the artist
    // scrubs along / fillet-loops (adaptive-off path).
    const bool filletFace =
        fid <= int(app.analysis.faces.size()) &&
        app.analysis.faces[fid - 1].isFillet;
    const auto axIt = app.report.faceAcross.find(fid);
    const int stripAcross =
        filletFace && axIt != app.report.faceAcross.end() ? axIt->second
                                                          : 0;
    using MK = weft::MesherKind;
    switch (kind) {
        case MK::RevolutionGrid:
        case MK::DomeCap:
            if (stripAcross && kind == MK::RevolutionGrid) {
                if (stripAcross == 1) {
                    manual(s.filletLoops, s.radial);
                } else {
                    manual(s.radial, s.filletLoops);
                }
            } else {
                manual(s.radial, s.axial);
            }
            break;
        case MK::DiskCap:
        case MK::AnnulusRing: manual(s.radial, 1); break;
        case MK::RibbonSweep:
        case MK::RailLadder: manual(s.radial, std::max(1, s.filletLoops)); break;
        case MK::PlateWeb:
            manual(s.boundary > 0 ? s.boundary : s.radial,
                   std::max(1, s.junctionRings));
            break;
        case MK::MinimalNGon:
            manual(s.boundary > 0 ? s.boundary : 1, 1);
            break;
        default:
            if (stripAcross) {
                if (stripAcross == 1) {
                    manual(s.filletLoops, s.gridU);
                } else {
                    manual(s.gridU, s.filletLoops);
                }
            } else {
                manual(s.gridU, s.gridV);
            }
            break;
    }
    if (s.adaptive) {
        const std::array<int, 2> live = faceSolvedCounts(app, fid);
        if (live[0] > 0) n[0] = live[0];
        if (live[1] > 0) n[1] = live[1];
        const weft::FaceMeshSettings& old = app.genSettings.forFace(fid);
        const double chordScale = std::sqrt(std::clamp(
            old.chordTolerance / std::max(1e-9, s.chordTolerance),
            0.0625, 16.0));
        const double angleScale = std::clamp(
            old.angleToleranceDeg /
                std::max(1.0, s.angleToleranceDeg),
            0.25, 4.0);
        const double scale = std::max(chordScale, angleScale);
        n[0] = std::max(1, int(std::lround(n[0] * scale)));
        n[1] = std::max(1, int(std::lround(n[1] * scale)));
    }
    return {float(std::clamp(n[0], 1, 256)),
            float(std::clamp(n[1], 1, 256))};
}

// Copy only the fields that CHANGED this frame onto a target. Panel and
// popup edits go through an edited copy of the ACTIVE face's settings;
// assigning that whole struct to every selected face stomped the other
// faces' unrelated overrides (their radial, their forced mesher...) with
// the active face's values. Diffing before/after keeps a multi-select
// edit scoped to the knob that actually moved.
static void applyChangedFields(const weft::FaceMeshSettings& before,
                               const weft::FaceMeshSettings& after,
                               weft::FaceMeshSettings& t) {
    if (after.radial != before.radial) t.radial = after.radial;
    if (after.axial != before.axial) t.axial = after.axial;
    if (after.gridU != before.gridU) t.gridU = after.gridU;
    if (after.gridV != before.gridV) t.gridV = after.gridV;
    if (after.cap != before.cap) t.cap = after.cap;
    if (after.chordTolerance != before.chordTolerance) {
        t.chordTolerance = after.chordTolerance;
    }
    if (after.angleToleranceDeg != before.angleToleranceDeg) {
        t.angleToleranceDeg = after.angleToleranceDeg;
    }
    if (after.filletLoops != before.filletLoops) {
        t.filletLoops = after.filletLoops;
    }
    if (after.filletHold != before.filletHold) {
        t.filletHold = after.filletHold;
    }
    if (after.junctionRings != before.junctionRings) {
        t.junctionRings = after.junctionRings;
    }
    if (after.quadDominant != before.quadDominant) {
        t.quadDominant = after.quadDominant;
    }
    if (after.pureTriFloor != before.pureTriFloor) {
        t.pureTriFloor = after.pureTriFloor;
    }
    if (after.minimal != before.minimal) t.minimal = after.minimal;
    if (after.exclude != before.exclude) t.exclude = after.exclude;
    if (after.forceMesher != before.forceMesher) {
        t.forceMesher = after.forceMesher;
    }
    if (after.linkRims != before.linkRims) t.linkRims = after.linkRims;
    if (after.minSize != before.minSize) t.minSize = after.minSize;
    if (after.relativeDeviation != before.relativeDeviation) {
        t.relativeDeviation = after.relativeDeviation;
    }
    if (after.weldTolerance != before.weldTolerance) {
        t.weldTolerance = after.weldTolerance;
    }
    if (after.squareCollar != before.squareCollar) {
        t.squareCollar = after.squareCollar;
    }
    if (after.coonsRotate != before.coonsRotate) {
        t.coonsRotate = after.coonsRotate;
    }
    if (after.boundary != before.boundary) t.boundary = after.boundary;
    if (after.adaptive != before.adaptive) t.adaptive = after.adaptive;
    if (after.minCurvedSegments != before.minCurvedSegments) {
        t.minCurvedSegments = after.minCurvedSegments;
    }
    if (after.cellCap != before.cellCap) t.cellCap = after.cellCap;
}

// Which mesher-combo options can plausibly build on a face, as a bitmask
// over the combo indices (bit i set = combo item i is offered; bit 0
// "auto" is always set). CONSERVATIVE: only the structured meshers with
// strict geometric prerequisites are greyed out (a plane can't be a
// revolution grid; a disk-cap needs a single circular loop; an annulus
// needs two loops or two co-axial radii). The flexible meshers
// (parametric/coons/quad-fill/minimal/quad-dominant/fallback) stay
// offered everywhere and fail gracefully if forced where they can't
// build — better than hiding a choice that would have worked. Combo
// order: 0 auto, 1 revolution-grid, 2 disk-cap, 3 parametric-grid,
// 4 coons-grid, 5 ring-junction, 6 quad-dominant, 7 minimal-ngon,
// 8 fallback-tri, 9 annulus-ring, 10 plate-web, 11 quad-fill.
static uint32_t buildableMesherMask(App& app, int faceId) {
    if (!app.hasModel || faceId < 1 || faceId > app.model.faceCount()) {
        return ~0u;
    }
    uint32_t mask = ~0u;
    auto off = [&](int i) { mask &= ~(1u << i); };
    try {
        const TopoDS_Face face = TopoDS::Face(app.model.faces(faceId));
        const bool isPlane =
            BRepAdaptor_Surface(face).GetType() == GeomAbs_Plane;
        int nLoops = 0, nSingleCircleLoops = 0;
        std::vector<double> radii;
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            ++nLoops;
            int nEdges = 0;
            bool circle = false;
            for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More();
                 ex.Next()) {
                ++nEdges;
                double f, l;
                Handle(Geom_Curve) c =
                    BRep_Tool::Curve(TopoDS::Edge(ex.Current()), f, l);
                if (!c.IsNull() && c->IsKind(STANDARD_TYPE(Geom_Circle))) {
                    circle = true;
                    radii.push_back(
                        Handle(Geom_Circle)::DownCast(c)->Radius());
                }
            }
            if (nEdges == 1 && circle) ++nSingleCircleLoops;
        }
        std::sort(radii.begin(), radii.end());
        int distinctRadii = 0;
        for (size_t i = 0; i < radii.size(); ++i) {
            if (i == 0 || radii[i] - radii[i - 1] > 1e-4) ++distinctRadii;
        }
        if (isPlane) off(1);                        // revolution-grid
        if (!isPlane || nLoops != 1 || nSingleCircleLoops != 1) off(2);  // disk
        if (!isPlane || nLoops < 2) off(5);         // ring-junction
        if (!isPlane) off(10);                      // plate-web
        if (nLoops < 2 && distinctRadii < 2) off(9);  // annulus-ring
    } catch (const Standard_Failure&) {
        return ~0u;
    }
    return mask;
}

// Kind-aware density nudge: EVERY mesher answers the wheel / [ ] with the
// field that actually drives its density — counts for structured grids,
// boundary totals for plate-web/quad-fill/minimal, deviation scaling for
// the freeform triangulators. Keys off THIS face's effective kind (forced
// choice first), not the active face's — a mixed selection nudges each
// face's own driving field. Returns the HUD line describing the change.
static std::string adjustFaceDensityOne(App& app, int fid,
                                        weft::FaceMeshSettings& s,
                                        bool secondary, int steps) {
    const weft::MesherKind kind = effectiveKind(app, fid);
    char hud[64] = "";
    // The counts the face was actually meshed at — the honest starting
    // point when the wheel leaves adaptive (nu -> radial/grid u, nv ->
    // axial/grid v).
    const std::array<int, 2> live = faceSolvedCounts(app, fid);
    auto count = [&](int& v, int lo, const char* name, int liveSeed) {
        // Scrolling a count IS choosing manual density — always leave
        // adaptive. A PER-FACE adapt-off is safe (the shared borders are
        // held by the adaptive neighbours, so a coons face doesn't
        // collapse — only a model-wide adapt-off does). Seed from the
        // live solved value first so the wheel starts at the number on
        // screen, not a stale default below the adaptive floor.
        if (s.adaptive) {
            if (liveSeed > 0) v = std::max(v, liveSeed);
            s.adaptive = false;
        }
        v = std::max(lo, v + steps);
        std::snprintf(hud, sizeof hud, "%s: %d", name, v);
    };
    // Pinned totals coexist with adaptive density — don't clear it.
    auto total = [&](int& v, const char* name) {
        if (v <= 0) {
            // 0 = auto: seed from the CURRENT solved total so the pin
            // starts where the mesh already is, not at a collapse.
            const int live = outerLoopSolvedTotal(app, fid);
            v = live > 0 ? live : 16;
        }
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
        case MK::DomeCap: {
            // Analytic fillet strips: primary = along (radial), secondary =
            // fillet loops across — not raw axial (which was the wrong
            // parametric axis on cylinder fillets with acrossIsU).
            const bool strip = fid <= int(app.analysis.faces.size()) &&
                               app.analysis.faces[fid - 1].isFillet;
            const auto ax = app.report.faceAcross.find(fid);
            if (kind == MK::RevolutionGrid && strip &&
                ax != app.report.faceAcross.end()) {
                if (secondary) {
                    const int liveAcross =
                        ax->second == 1 ? live[0] : live[1];
                    if (s.adaptive) {
                        if (liveAcross > 0) {
                            s.filletLoops =
                                std::max(s.filletLoops, liveAcross);
                        }
                        s.adaptive = false;
                    }
                    s.filletLoops = std::max(1, s.filletLoops + steps);
                    std::snprintf(hud, sizeof hud,
                                  "fillet loops (across): %d",
                                  s.filletLoops);
                } else {
                    count(s.radial, 3, "along the blend",
                          ax->second == 1 ? live[1] : live[0]);
                }
            } else if (secondary) {
                count(s.axial, 1, "axial", live[1]);
            } else {
                count(s.radial, 3, "radial", live[0]);
            }
            break;
        }
        case MK::DiskCap:
            // DiskCap density is rim-only (radial). Axial is unused.
            if (secondary) {
                std::snprintf(hud, sizeof hud, "disk cap: radial only");
            } else {
                count(s.radial, 3, "radial", live[0]);
            }
            break;
        case MK::RibbonSweep:
        case MK::RailLadder:
            // Rail density along the sweep — radial seeds the rail sample
            // count; there is no independent secondary knob.
            count(s.radial, 3, "rail density", live[0]);
            break;
        case MK::RingJunction:
            // Rect sides size the outer and set hole samples n=2*(nu+nv).
            // Concentric rings stay on the panel (junction rings).
            if (secondary) count(s.gridV, 1, "rect v", live[1]);
            else count(s.gridU, 1, "rect u", live[0]);
            break;
        case MK::AnnulusRing:
            count(s.radial, 3, "loop verts", live[0]);
            break;
        case MK::PlateWeb:
            if (secondary) count(s.junctionRings, 0, "collar rings", 0);
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
        default: {  // PlanarGrid, CoonsGrid
            // Blend strips scrub SEMANTIC axes: primary = along the
            // blend (always gridU — the solve remaps it to whichever
            // patch axis runs along, so mirror twins agree), secondary
            // = the across loop count (gridV is inert on strips; a raw
            // grid-v scrub would be a dead knob).
            const bool strip = fid <= int(app.analysis.faces.size()) &&
                               app.analysis.faces[fid - 1].isFillet;
            const auto ax = app.report.faceAcross.find(fid);
            if (strip && ax != app.report.faceAcross.end()) {
                if (secondary) {
                    s.filletLoops = std::max(1, s.filletLoops + steps);
                    std::snprintf(hud, sizeof hud,
                                  "fillet loops (across): %d",
                                  s.filletLoops);
                } else {
                    count(s.gridU, 1, "along the blend",
                          ax->second == 1 ? live[1] : live[0]);
                }
            } else if (secondary) {
                count(s.gridV, 1, "grid v", live[1]);
            } else {
                count(s.gridU, 1, "grid u", live[0]);
            }
            break;
        }
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

// Density nudge over the whole selection + HUD readout. Each face is
// nudged through ITS OWN effective mesher kind (editSelected can't carry
// the face id into the edit, so the override plumbing is inlined here) —
// the HUD shows the active face's change and how many faces moved.
static void adjustFaceDensity(App& app, bool secondary, int steps) {
    logLine("edit: %zu selected face(s), active %d", app.selFaces.size(),
            app.activeFace);
    std::string hud;
    if (app.selFaces.empty()) {
        // No selection: do not touch global density scale — wheel+modifier
        // over empty space used to fight face density keybinds and silently
        // rescale the whole model. Scale stays on the panel slider only.
        std::snprintf(app.hudText, sizeof app.hudText,
                      "select a face to edit density");
        app.hudUntil = glfwGetTime() + 0.9;
        return;
    } else {
        for (int fid : app.selFaces) {
            auto it = app.recipe.settings.perFace.find(fid);
            if (it == app.recipe.settings.perFace.end()) {
                it = app.recipe.settings.perFace
                         .emplace(fid, app.recipe.settings.defaults)
                         .first;
            }
            std::string h =
                adjustFaceDensityOne(app, fid, it->second, secondary, steps);
            if (hud.empty() || fid == app.activeFace) hud = h;
        }
        if (app.selFaces.size() > 1) {
            hud += " (x" + std::to_string(app.selFaces.size()) + " faces)";
        }
    }
    markDirty(app);
    if (!hud.empty()) {
        std::snprintf(app.hudText, sizeof app.hudText, "%s", hud.c_str());
        app.hudUntil = glfwGetTime() + 0.9;
    }
}

// Fluid hover editing: with nothing selected, modifier+wheel edits the
// face UNDER THE CURSOR directly (auto-creating its override). Over empty
// space it does NOT change global density scale (that fought face density
// keybinds) — only angle / fillet-loop defaults:
//   shift+wheel        face primary density   | (idle — select a face)
//   ctrl+wheel         face secondary density | global angle tolerance
//   ctrl+shift+wheel   face fillet loops      | global fillet loops
static void adjustHovered(App& app, bool ctrl, bool shift, int steps) {
    if (app.hoverFace > 0) {
        const int fid = app.hoverFace;
        auto it = app.recipe.settings.perFace.find(fid);
        if (it == app.recipe.settings.perFace.end()) {
            it = app.recipe.settings.perFace
                     .emplace(fid, app.recipe.settings.defaults)
                     .first;
        }
        std::string hud;
        if (ctrl && shift) {
            it->second.filletLoops = std::max(1, it->second.filletLoops + steps);
            hud = "fillet loops: " + std::to_string(it->second.filletLoops);
            // Honest HUD: fillet loops feed coons/planar/rev-grid fillet
            // meshers — flag the nudge when this face ignores it.
            const weft::MesherKind k = effectiveKind(app, fid);
            const bool used =
                fid <= int(app.analysis.faces.size()) &&
                app.analysis.faces[fid - 1].isFillet &&
                (k == weft::MesherKind::CoonsGrid ||
                 k == weft::MesherKind::PlanarGrid ||
                 k == weft::MesherKind::RevolutionGrid);
            if (!used) hud += " (no effect here)";
        } else {
            hud = adjustFaceDensityOne(app, fid, it->second, ctrl, steps);
        }
        std::snprintf(app.hudText, sizeof app.hudText, "face %d  %s", fid,
                      hud.c_str());
        app.hudUntil = glfwGetTime() + 0.9;
        markDirty(app);
        return;
    }
    // Background: angle / fillet defaults only — never density scale.
    weft::FaceMeshSettings& d = app.recipe.settings.defaults;
    if (ctrl && shift) {
        d.filletLoops = std::max(1, d.filletLoops + steps);
        std::snprintf(app.hudText, sizeof app.hudText, "fillet loops: %d",
                      d.filletLoops);
    } else if (ctrl) {
        d.angleToleranceDeg =
            std::clamp(d.angleToleranceDeg * std::pow(0.86, double(steps)),
                       1.0, 60.0);
        std::snprintf(app.hudText, sizeof app.hudText, "angle: %.1f deg",
                      d.angleToleranceDeg);
    } else {
        std::snprintf(app.hudText, sizeof app.hudText,
                      "select a face to edit density");
    }
    app.hudUntil = glfwGetTime() + 0.9;
    if (ctrl) markDirty(app);
}

// Shared verbs (key handlers + pie menus call the same code).
static void setSelectMode(App& app, SelectMode next) {
    if (next == app.selectMode) return;
    app.selectMode = next;
    app.selFaces.clear();
    app.selEdges.clear();
    app.selPolys.clear();
    app.selVerts.clear();
    app.selVertOrder.clear();
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

// Replace the face selection with every id matching `keep`, switch to
// element (B-rep face) mode, and make the lowest id active so the
// Selection roster + knobs open on a real face.
static void selectFacesMatching(App& app,
                                const std::function<bool(int)>& keep) {
    setSelectMode(app, SelectMode::Face);  // clears prior selection
    app.selFaces.clear();
    for (const auto& fi : app.analysis.faces) {
        if (fi.id >= 1 && fi.id <= app.model.faceCount() && keep(fi.id)) {
            app.selFaces.insert(fi.id);
        }
    }
    app.activeFace = app.selFaces.empty() ? 0 : *app.selFaces.begin();
    rebuildBuffers(app);
    if (!app.selFaces.empty()) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "selected %zu face(s)",
                      app.selFaces.size());
        app.status = buf;
    }
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
        app.selVertOrder.clear();
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
        if (poly.size() <= 4) {
            for (size_t k = 1; k + 1 < poly.size(); ++k) {
                if (!inTri(proj[0], proj[k], proj[k + 1])) continue;
                double depth =
                    (proj[0][2] + proj[k][2] + proj[k + 1][2]) / 3;
                if (depth < bestDepth) {
                    bestDepth = depth;
                    best = int(p);
                }
                break;
            }
        } else {
            // Keyhole n-gons: a fan containment test would hover-hit the
            // holes; test the real tessellation instead.
            for (const auto& t : weft::triangulatePoly(m.vertices, poly)) {
                if (!inTri(proj[t[0]], proj[t[1]], proj[t[2]])) continue;
                double depth =
                    (proj[t[0]][2] + proj[t[1]][2] + proj[t[2]][2]) / 3;
                if (depth < bestDepth) {
                    bestDepth = depth;
                    best = int(p);
                }
                break;
            }
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
    // Only vertices some SURVIVING polygon still uses: DeletePoly
    // leaves orphan verts behind (valid anchors, invisible) and a
    // ghost stealing the pick reads as "grab does nothing".
    std::vector<bool> live(m.vertexCount(), false);
    for (size_t p = 0; p < m.polygons.size(); ++p) {
        int fid = m.polygonFaceId[p];
        if (fid > 0 && app.hiddenFaces.count(fid)) continue;
        for (uint32_t v : m.polygons[p]) live[v] = true;
    }
    double best = 30.0 * gUiScale;  // px
    size_t bestV = m.vertexCount();
    bool sawAnchorless = false;
    for (size_t v = 0; v < m.vertexCount(); ++v) {
        if (!live[v]) continue;
        const weft::Anchor& a = m.anchors[v];
        float s[3] = {0, 0, -1};
        projectPoint(mvp, m.vertices[v], fbw, fbh, s);
        if (s[2] <= 0) continue;
        double d = std::hypot(s[0] - mx, s[1] - my);
        if (d >= best) continue;
        if (a.faceId == 0) {
            sawAnchorless = true;  // bridge strip / weld vert: no surface
            continue;
        }
        if (app.hiddenFaces.count(a.faceId)) continue;
        best = d;
        bestV = v;
    }
    if (bestV == m.vertexCount()) {
        app.status = sawAnchorless
                         ? "grab: that vertex has no CAD anchor (bridge/"
                           "weld verts can't be surface-grabbed)"
                         : "grab: no interior vertex under the cursor "
                           "(border verts are density-driven)";
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

// Per-face density controls: only the settings that actually drive the
// face's effective mesher kind are shown — everything visible has a
// visible effect. (The global defaults get their own tabbed panel,
// drawMesherDefaultTabs, grouped by mesher family.)
// adaptiveOffViaCheckbox reports that THIS frame's edit was the adaptive
// checkbox turning off — the caller re-seeds the other selected faces'
// counts from their own live values instead of the active face's.
static bool settingsEditor(App& app, weft::FaceMeshSettings& s,
                           weft::MesherKind k, bool isFillet,
                           bool* adaptiveOffViaCheckbox = nullptr) {
    using MK = weft::MesherKind;
    // The knob-to-parts map: hovering a control tints the faces that
    // control actually drives (cyan overlay), so "which segment relates
    // to which parts" is answered by pointing, not guessing. -2 marks
    // "fillet faces" (a face property, not a mesher kind).
    auto hover = [&](std::initializer_list<int> kinds) {
        if (!ImGui::IsItemHovered()) return;
        for (int kk : kinds) app.highlightMeshers.insert(kk);
    };
    constexpr int kAllKinds = -1, kFilletFaces = -2;
    // Annulus loops and plate-web/quad-fill borders take the radial
    // default too (each closed loop, or each hole circle, proposes it).
    const bool revolved = k == MK::RevolutionGrid || k == MK::DiskCap ||
                          k == MK::AnnulusRing || k == MK::PlateWeb ||
                          k == MK::QuadFill || k == MK::RibbonSweep ||
                          k == MK::RailLadder || k == MK::DomeCap;
    // MinimalNGon is NOT a grid: it emits one boundary n-gon, so grid
    // u/v would be dead knobs — its density lives in "boundary verts".
    const bool grid = k == MK::PlanarGrid || k == MK::RingJunction ||
                      k == MK::CoonsGrid;
    const bool freeform = k == MK::QuadDominant || k == MK::Fallback;
    bool ch = false;

    // Curvature-adaptive density: deviation/angle size every curved edge;
    // the manual counts below become floors. Nudging a count via the
    // wheel flips the face back to manual.
    const bool prevAdaptive = s.adaptive;
    ch |= ImGui::Checkbox("adaptive density (curvature)", &s.adaptive);
    hover({kAllKinds});
    if (adaptiveOffViaCheckbox) {
        *adaptiveOffViaCheckbox = prevAdaptive && !s.adaptive;
    }
    // Leaving adaptive: seed the manual count fields from what the face
    // was actually meshed at, so the boxes show the live value the user
    // sees on screen — not a stale default that needs cranking past the
    // adaptive floor before anything moves. nu seeds radial/grid u, nv
    // seeds axial/grid v (only the field the panel shows for this kind is
    // used; seeding both is harmless).
    if (prevAdaptive && !s.adaptive) {
        const std::array<int, 2> live =
            faceSolvedCounts(app, app.activeFace);
        if (live[0] > 0) {
            s.radial = std::max(s.radial, live[0]);
            s.gridU = std::max(s.gridU, live[0]);
        }
        if (live[1] > 0) {
            s.axial = std::max(s.axial, live[1]);
            s.gridV = std::max(s.gridV, live[1]);
        }
    }
    if (s.adaptive) {
        if (ImGui::DragInt("min curved segments", &s.minCurvedSegments, 0.2f,
                           1, 256)) {
            s.minCurvedSegments = std::clamp(s.minCurvedSegments, 1, 256);
            ch = true;
        }
        hover({kAllKinds});
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Lower floor for closed curved rings (cylinders, spheres,\n"
                "fillets). Adaptive never resolves those below this count.");
        }
    }
    // Per-face weld tolerance (mm, 0 = inherit the global). Governs how
    // loosely this face's boundary welds onto its neighbours; a shared
    // edge welds at the looser of the two faces (and the global), so
    // raising it on ONE side closes that junction. Shown for every kind:
    // an analytic face's value still loosens its freeform neighbour's
    // border onto it.
    {
        float wt = float(s.weldTolerance);
        if (ImGui::DragFloat("weld tol (0=global)", &wt, 0.0005f, 0.0f, 1.0f,
                             "%.5f", ImGuiSliderFlags_Logarithmic)) {
            s.weldTolerance = std::max(0.0, double(wt));
            ch = true;
        }
        hover({kAllKinds});
    }
    // Freeform always; adaptive faces use these as curvature sizing;
    // QuadFill reads deviation/angle for interior iso spacing even when
    // adaptive is off (matches the wheel secondary).
    if (freeform || s.adaptive || k == MK::QuadFill) {
        float dev = float(s.chordTolerance);
        if (ImGui::DragFloat("deviation", &dev, 0.01f, 0.0005f, 100.0f,
                             "%.4f", ImGuiSliderFlags_Logarithmic)) {
            s.chordTolerance = dev;
            ch = true;
        }
        hover({kAllKinds});
        float ang = float(s.angleToleranceDeg);
        if (ImGui::DragFloat("angle", &ang, 0.25f, 1.0f, 60.0f, "%.1f deg")) {
            s.angleToleranceDeg = ang;
            ch = true;
        }
        hover({kAllKinds});
    }
    if (freeform) {
        ch |= ImGui::Checkbox("quad-dominant fallback", &s.quadDominant);
        hover({int(MK::QuadDominant), int(MK::Fallback)});
        ch |= ImGui::Checkbox("triangulate fallback", &s.pureTriFloor);
        hover({int(MK::QuadDominant), int(MK::Fallback)});
        float ms = float(s.minSize);
        if (ImGui::DragFloat("min size", &ms, 0.01f, 0.0f, 100.0f, "%.3f")) {
            s.minSize = ms;
            ch = true;
        }
        hover({int(MK::QuadDominant), int(MK::Fallback)});
        ch |= ImGui::Checkbox("relative deviation", &s.relativeDeviation);
        hover({kAllKinds});
    }
    // While adaptive drives, the count boxes show the LIVE solved values
    // (what the mesh on screen actually uses), not the stale manual
    // numbers underneath — and a drag starts FROM the live value, flips
    // to manual, and keeps it as the starting point.
    const std::array<int, 2> liveN =
        s.adaptive ? faceSolvedCounts(app, app.activeFace)
                   : std::array<int, 2>{0, 0};
    // Blend strips get SEMANTIC axis knobs: the raw u/v (or radial/axial)
    // exposure leaks patch orientation. faceAcross says which axis the
    // fillet-loops knob drives; along is gridU (Coons/Planar) or radial
    // (RevolutionGrid analytic fillets).
    int stripAcross = 0;  // 1 = loops ride the patch u axis, 2 = v
    if (isFillet && (k == MK::CoonsGrid || k == MK::PlanarGrid ||
                     k == MK::RevolutionGrid)) {
        auto ax = app.report.faceAcross.find(app.activeFace);
        if (ax != app.report.faceAcross.end()) stripAcross = ax->second;
    }
    if (revolved) {
        // Typing a count IS choosing manual density for this face —
        // same rule as the wheel — otherwise the number displays while
        // adaptive keeps driving and they never match.
        // Labels match the wheel HUD so selected-face settings name the
        // same semantic axes the artist already scrolled.
        if (k == MK::RevolutionGrid && stripAcross) {
            const int alongLive =
                stripAcross == 1 ? liveN[1] : liveN[0];
            int alongShown =
                s.adaptive && alongLive > 0 ? alongLive : s.radial;
            if (ImGui::DragInt("along the blend", &alongShown, 0.2f, 3,
                               256)) {
                s.radial = alongShown;
                ch = true;
                s.adaptive = false;
            }
            hover({int(MK::RevolutionGrid)});
            ImGui::TextDisabled("across = fillet loops (patch %s)",
                                stripAcross == 1 ? "u" : "v");
        } else {
            const char* radialLabel = "radial";
            if (k == MK::AnnulusRing) radialLabel = "loop verts";
            else if (k == MK::RibbonSweep || k == MK::RailLadder) {
                radialLabel = "rail density";
            } else if (k == MK::PlateWeb || k == MK::QuadFill) {
                // Outer density is boundary verts when pinned; radial only
                // seeds shares on loops the pin does not cover (holes).
                radialLabel =
                    s.boundary > 0 ? "hole share seed" : "loop share seed";
            }
            int radialShown =
                s.adaptive && liveN[0] > 0 ? liveN[0] : s.radial;
            if (ImGui::DragInt(radialLabel, &radialShown, 0.2f, 3, 256)) {
                s.radial = radialShown;
                ch = true;
                // Manual only where radial IS the density; on plate-web /
                // quad-fill it merely seeds loop shares and killing
                // adaptive collapses the borders to flat pins.
                if (k == MK::RevolutionGrid || k == MK::DiskCap ||
                    k == MK::AnnulusRing || k == MK::RibbonSweep ||
                    k == MK::RailLadder || k == MK::DomeCap) {
                    s.adaptive = false;
                }
            }
            hover({int(MK::RevolutionGrid), int(MK::DiskCap),
                   int(MK::AnnulusRing), int(MK::PlateWeb), int(MK::QuadFill),
                   int(MK::RibbonSweep), int(MK::RailLadder),
                   int(MK::DomeCap)});
            if (k == MK::RevolutionGrid || k == MK::DomeCap) {
                int axialShown =
                    s.adaptive && liveN[1] > 0 ? liveN[1] : s.axial;
                if (ImGui::DragInt("axial", &axialShown, 0.2f, 1, 256)) {
                    s.axial = axialShown;
                    ch = true;
                    s.adaptive = false;
                }
                hover({int(MK::RevolutionGrid), int(MK::DomeCap)});
            }
        }
        if (k == MK::DiskCap) {
            int cap = s.cap == weft::CapStyle::Fan ? 1 : 0;
            if (ImGui::Combo("cap style", &cap, "ngon\0fan\0")) {
                s.cap = cap ? weft::CapStyle::Fan : weft::CapStyle::NGon;
                ch = true;
            }
            hover({int(MK::DiskCap)});
        }
        if (k == MK::PlateWeb) {
            // Hole-plate collars: off by default (0); raise rings or tick
            // the checkbox to turn the rim on.
            {
                bool collars = s.junctionRings > 0;
                if (ImGui::Checkbox("hole collars", &collars)) {
                    s.junctionRings = collars ? std::max(1, s.junctionRings)
                                              : 0;
                    ch = true;
                }
                hover({int(MK::PlateWeb)});
            }
            if (s.junctionRings > 0) {
                ch |= ImGui::DragInt("collar rings", &s.junctionRings, 0.2f,
                                     1, 32);
                hover({int(MK::RingJunction), int(MK::PlateWeb)});
                ch |= ImGui::Checkbox("square collars", &s.squareCollar);
                hover({int(MK::PlateWeb)});
            }
        }
    }
    // Boundary totals stand alone: MinimalNGon isn't in the revolved set
    // (nesting this inside it made the knob unreachable for exactly the
    // mesher whose ONLY density control it is).
    if (k == MK::PlateWeb || k == MK::QuadFill || k == MK::MinimalNGon) {
        // Total verts around the outer loop, length-distributed and
        // pinned (drives the neighbouring walls' shared edges).
        const int prevBoundary = s.boundary;
        if (ImGui::DragInt("boundary verts (0=auto)", &s.boundary,
                           0.2f, 0, 512)) {
            if (prevBoundary == 0 && s.boundary > 0) {
                const int live =
                    outerLoopSolvedTotal(app, app.activeFace);
                if (live > 0) s.boundary = std::max(s.boundary, live);
            }
            ch = true;
        }
        hover({int(MK::PlateWeb), int(MK::QuadFill), int(MK::MinimalNGon)});
    }
    // Blend strips (Coons / Planar): remapped along via gridU. Revolution
    // analytic fillets already showed along via radial above.
    if (grid && stripAcross) {
        const int alongLive = stripAcross == 1 ? liveN[1] : liveN[0];
        int alongShown =
            s.adaptive && alongLive > 0 ? alongLive : s.gridU;
        if (ImGui::DragInt("along the blend", &alongShown, 0.2f, 1,
                           256)) {
            s.gridU = alongShown;
            ch = true;
        }
        hover({int(MK::PlanarGrid), int(MK::CoonsGrid)});
        ImGui::TextDisabled("across = fillet loops (patch %s)",
                            stripAcross == 1 ? "u" : "v");
        if (k == MK::CoonsGrid) {
            int rot = s.coonsRotate;
            if (ImGui::SliderInt("rotate patch", &rot, 0, 3)) {
                s.coonsRotate = rot;
                ch = true;
            }
        }
    } else if (grid) {
        // RingJunction is a planar rectangle with a circular trim — both
        // rect sides drive outer samples and hole angular count.
        const char* uLabel = k == MK::RingJunction ? "rect u" : "grid u";
        const char* vLabel = k == MK::RingJunction ? "rect v" : "grid v";
        int gridUShown = s.adaptive && liveN[0] > 0 ? liveN[0] : s.gridU;
        if (ImGui::DragInt(uLabel, &gridUShown, 0.2f, 1, 256)) {
            s.gridU = gridUShown;
            ch = true;
            // Coons floors coexist with adaptive borders — no flip.
            if (k == MK::PlanarGrid || k == MK::RingJunction) {
                s.adaptive = false;
            }
        }
        hover({int(MK::PlanarGrid), int(MK::CoonsGrid),
               int(MK::RingJunction)});
        int gridVShown = s.adaptive && liveN[1] > 0 ? liveN[1] : s.gridV;
        if (ImGui::DragInt(vLabel, &gridVShown, 0.2f, 1, 256)) {
            s.gridV = gridVShown;
            ch = true;
            if (k == MK::PlanarGrid || k == MK::RingJunction) {
                s.adaptive = false;
            }
        }
        hover({int(MK::PlanarGrid), int(MK::CoonsGrid),
               int(MK::RingJunction)});
        if (k == MK::RingJunction) {
            ch |= ImGui::DragInt("junction rings", &s.junctionRings, 0.2f, 1,
                                 32);
            hover({int(MK::RingJunction)});
        }
        if (k == MK::CoonsGrid) {
            // Which corner anchors the grid; on triangular patches this
            // moves the corner the fan terminates in.
            int rot = s.coonsRotate;
            if (ImGui::SliderInt("rotate patch", &rot, 0, 3)) {
                s.coonsRotate = rot;
                ch = true;
            }
        }
    }
    // Fillet loops / hold feed Coons / Planar / RevolutionGrid fillet
    // meshers (across-the-blend count and crease clustering). Only
    // surface them where they actually do something — not on every face
    // the classifier merely tagged [fillet].
    if (isFillet && (k == MK::CoonsGrid || k == MK::PlanarGrid ||
                     k == MK::RevolutionGrid)) {
        ch |= ImGui::DragInt(stripAcross ? "fillet loops (across)"
                                         : "fillet loops",
                             &s.filletLoops, 0.2f, 1, 64);
        hover({kFilletFaces});
        float hold = float(s.filletHold);
        if (ImGui::SliderFloat("hold", &hold, 0.0f, 0.95f)) {
            s.filletHold = hold;
            ch = true;
        }
        hover({kFilletFaces});
    }
    // Flat geometry (plane OR flat bspline) collapses to one exact
    // boundary n-gon, holes bridged in — available everywhere since
    // any mesher's face can turn out flat.
    ch |= ImGui::Checkbox("minimal n-gon (flat panels)", &s.minimal);
    hover({int(MK::MinimalNGon)});
    {
        // Manual mesher choice: auto picks per geometry; forcing one that
        // can't build on the face falls back to triangulation. Options
        // whose geometric prerequisites the selected face can't meet are
        // greyed out so the user doesn't force a "couldn't build here".
        static const char* kMesherNames[] = {
            "auto",          "revolution-grid", "disk-cap",
            "parametric-grid", "coons-grid",    "ring-junction",
            "quad-dominant", "minimal-ngon",    "fallback-tri",
            "annulus-ring",  "plate-web",       "quad-fill",
            "rail-ladder",   "ribbon-sweep",    "dome-cap"};
        const int nMesher = int(IM_ARRAYSIZE(kMesherNames));
        const uint32_t bmask = buildableMesherMask(app, app.activeFace);
        int mesher = std::clamp(s.forceMesher, 0, nMesher - 1);
        if (ImGui::BeginCombo("mesher", kMesherNames[mesher])) {
            for (int i = 0; i < nMesher; ++i) {
                // Quad Fill remains loadable for legacy recipes, but is no
                // longer offered or selected by the geometry policy.
                if (i == 1 + int(MK::QuadFill)) continue;
                const bool ok = i == 0 || (bmask & (1u << i));
                ImGui::BeginDisabled(!ok);
                if (ImGui::Selectable(kMesherNames[i], mesher == i)) {
                    s.forceMesher = i;
                    ch = true;
                }
                ImGui::EndDisabled();
            }
            ImGui::EndCombo();
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

// The active face's settings block, shared by the Selection panel and the
// right-click popup so the two can never drift apart: the effective kind
// line (with forced / still-updating annotations), build health from the
// report, the kind-gated editor, rim controls, and the override reset.
static void drawActiveFaceSettings(App& app) {
    // Guard the range, not just positivity: a stale id (report entries
    // from a previous model) must never index analysis.faces.
    if (app.activeFace <= 0 ||
        app.activeFace > int(app.analysis.faces.size())) {
        return;
    }
    const weft::FaceInfo& f = app.analysis.faces[app.activeFace - 1];
    const weft::MesherKind kind = effectiveKind(app, app.activeFace);
    const bool forced =
        app.recipe.settings.forFace(app.activeFace).forceMesher > 0;
    // The report lags the recipe until the pending regenerate lands —
    // annotate instead of listing last run's kind as if it were current.
    const bool pending =
        app.dirty || app.bakeQueue.busy() || app.bakeQueue.hasPending();
    ImGui::Text("mesher: %s%s%s", weft::mesherKindName(kind),
                forced ? " (forced)" : "", pending ? "  updating..." : "");
    if (!pending) {
        if (forced) {
            auto rit = app.report.faceMesher.find(app.activeFace);
            if (rit != app.report.faceMesher.end() && rit->second != kind) {
                ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                                   "forced %s couldn't build here (built "
                                   "as %s)",
                                   weft::mesherKindName(kind),
                                   weft::mesherKindName(rit->second));
            }
        }
        // Build health: -1 = the face emitted nothing (a hole in the
        // output — the one state the viewport can't even show, since
        // there is nothing to click), 1 = raw triangle soup, 2 = the
        // contract floor. 0/absent = the planned mesher built.
        auto bit = app.report.faceBuild.find(app.activeFace);
        if (bit != app.report.faceBuild.end()) {
            if (bit->second == -1) {
                ImGui::TextColored({1.0f, 0.35f, 0.3f, 1.0f},
                                   "face emitted nothing - clear the "
                                   "override or ctrl+Z");
            } else if (bit->second == 1) {
                ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                                   "raw triangulation fallback");
            } else if (bit->second == 2) {
                ImGui::TextDisabled("contract floor (exact borders)");
            }
        }
        auto cit = app.report.faceBuildCause.find(app.activeFace);
        if (cit != app.report.faceBuildCause.end() && !cit->second.empty()) {
            ImGui::TextWrapped("cause: %s", cit->second.c_str());
        }
    }
    // Editing auto-overrides: the editor works on a copy of the ACTIVE
    // face's settings; only the fields that changed land on the other
    // selected faces (their unrelated overrides survive).
    weft::FaceMeshSettings edited = activeSettings(app);
    const weft::FaceMeshSettings before = edited;
    bool adaptiveOffCheckbox = false;
    const bool changed =
        settingsEditor(app, edited, kind, f.isFillet, &adaptiveOffCheckbox);
    if (changed) {
        // The edit lands on the selection PLUS the displayed face: after
        // an outliner deselect the panel still shows the active face, and
        // an edit made under its heading must reach it — never fall
        // through to the global defaults from a face-titled panel.
        std::set<int> targets = app.selFaces;
        targets.insert(app.activeFace);
        // Boundary pin flipped on this frame: the editor seeded `edited`
        // from the ACTIVE face's outer loop; other faces pin at their own
        // solved totals below.
        const bool boundaryPinned =
            before.boundary == 0 && edited.boundary > 0;
        for (int fid : targets) {
            auto it = app.recipe.settings.perFace.find(fid);
            if (it == app.recipe.settings.perFace.end()) {
                it = app.recipe.settings.perFace
                         .emplace(fid, app.recipe.settings.defaults)
                         .first;
            }
            weft::FaceMeshSettings& s = it->second;
            const bool wasAdaptive = s.adaptive;
            weft::FaceMeshSettings target = edited;
            if (fid != app.activeFace) {
                if (adaptiveOffCheckbox) {
                    // The count seeds baked into `edited` came from the
                    // ACTIVE face's live mesh; this face re-seeds from
                    // its OWN solved counts below.
                    target.radial = before.radial;
                    target.gridU = before.gridU;
                    target.axial = before.axial;
                    target.gridV = before.gridV;
                }
                if (boundaryPinned) target.boundary = before.boundary;
            }
            applyChangedFields(before, target, s);
            if (fid != app.activeFace) {
                // Re-seed only faces that were actually adaptive — a
                // face already on manual counts keeps them (the report
                // may lag fresh edits; don't resurrect old values).
                if (adaptiveOffCheckbox && wasAdaptive) {
                    const std::array<int, 2> live =
                        faceSolvedCounts(app, fid);
                    if (live[0] > 0) {
                        s.radial = std::max(s.radial, live[0]);
                        s.gridU = std::max(s.gridU, live[0]);
                    }
                    if (live[1] > 0) {
                        s.axial = std::max(s.axial, live[1]);
                        s.gridV = std::max(s.gridV, live[1]);
                    }
                }
                if (boundaryPinned) {
                    const int live = outerLoopSolvedTotal(app, fid);
                    s.boundary = live > 0 ? live : edited.boundary;
                }
            }
        }
        markDirty(app);
    }
    if (kind == weft::MesherKind::RevolutionGrid) {
        ImGui::PushID("rims");
        rimControls(app, app.activeFace);
        ImGui::PopID();
    }
    if (app.recipe.settings.perFace.count(app.activeFace)) {
        ImGui::TextDisabled("overridden");
        ImGui::SameLine();
        if (ImGui::SmallButton("clear override(s)")) {
            app.recipe.settings.perFace.erase(app.activeFace);
            for (int fid : app.selFaces) {
                app.recipe.settings.perFace.erase(fid);
            }
            markDirty(app);
        }
    }
}

// Global mesher defaults, split into per-family tabs (cylinders /
// fillets / ribbons / rings / flat faces / freeform) so the left panel
// lists each mesher's values under its own name instead of one flat
// wall of knobs. Hovering a tab label or any knob tints the faces it
// drives (cyan) — the live "which parts does this change" map. All
// controls edit recipe.settings.defaults; per-face overrides still win.
static void drawMesherDefaultTabs(App& app) {
    using MK = weft::MesherKind;
    weft::FaceMeshSettings& d = app.recipe.settings.defaults;
    bool ch = false;
    auto hover = [&](std::initializer_list<int> kinds) {
        if (!ImGui::IsItemHovered()) return;
        for (int kk : kinds) app.highlightMeshers.insert(kk);
    };
    constexpr int kAllKinds = -1, kFilletFaces = -2;
    // General knobs that feed every family stay above the tabs.
    const bool prevAdaptive = d.adaptive;
    ch |= ImGui::Checkbox("adaptive density (curvature)", &d.adaptive);
    hover({kAllKinds});
    // Leaving adaptive globally: seed the manual counts from the active
    // face's live solve (same rule as the per-face editor) — otherwise
    // the whole model collapses to the stale flat defaults.
    if (prevAdaptive && !d.adaptive) {
        const std::array<int, 2> live =
            faceSolvedCounts(app, app.activeFace);
        if (live[0] > 0) {
            d.radial = std::max(d.radial, live[0]);
            d.gridU = std::max(d.gridU, live[0]);
        }
        if (live[1] > 0) {
            d.axial = std::max(d.axial, live[1]);
            d.gridV = std::max(d.gridV, live[1]);
        }
    }
    if (d.adaptive) {
        if (ImGui::DragInt("min curved segments", &d.minCurvedSegments, 0.2f,
                           1, 256)) {
            d.minCurvedSegments = std::clamp(d.minCurvedSegments, 1, 256);
            ch = true;
        }
        hover({kAllKinds});
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip(
                "Lower floor for closed curved rings (cylinders, spheres,\n"
                "fillets). Adaptive never resolves those below this count.");
        }
    }
    if (!ImGui::BeginTabBar("##mesherdefaults",
                            ImGuiTabBarFlags_FittingPolicyScroll)) {
        if (ch) markDirty(app);
        return;
    }
    // One tab per family; hovering the LABEL highlights the family.
    auto tab = [&](const char* label, std::initializer_list<int> kinds) {
        const bool open = ImGui::BeginTabItem(label);
        hover(kinds);
        return open;
    };
    if (tab("freeform", {int(MK::QuadDominant), int(MK::Fallback)})) {
        ImGui::TextDisabled("imported / trimmed surfaces");
        if (d.adaptive) {
            ImGui::TextDisabled("adaptive: these size every family");
        }
        float dev = float(d.chordTolerance);
        if (ImGui::DragFloat("deviation", &dev, 0.01f, 0.0005f, 100.0f,
                             "%.4f", ImGuiSliderFlags_Logarithmic)) {
            d.chordTolerance = dev;
            ch = true;
        }
        hover({d.adaptive ? kAllKinds : int(MK::QuadDominant),
               int(MK::Fallback)});
        float ang = float(d.angleToleranceDeg);
        if (ImGui::DragFloat("angle", &ang, 0.25f, 1.0f, 60.0f,
                             "%.1f deg")) {
            d.angleToleranceDeg = ang;
            ch = true;
        }
        hover({d.adaptive ? kAllKinds : int(MK::QuadDominant),
               int(MK::Fallback)});
        ch |= ImGui::Checkbox("quad-dominant fallback", &d.quadDominant);
        hover({int(MK::QuadDominant), int(MK::Fallback)});
        ch |= ImGui::Checkbox("triangulate fallback", &d.pureTriFloor);
        hover({int(MK::QuadDominant), int(MK::Fallback)});
        float ms = float(d.minSize);
        if (ImGui::DragFloat("min size", &ms, 0.01f, 0.0f, 100.0f,
                             "%.3f")) {
            d.minSize = ms;
            ch = true;
        }
        hover({int(MK::QuadDominant), int(MK::Fallback)});
        ch |= ImGui::Checkbox("relative deviation", &d.relativeDeviation);
        hover({kAllKinds});
        ImGui::EndTabItem();
    }
    if (tab("cylinders", {int(MK::RevolutionGrid), int(MK::DiskCap),
                          int(MK::DomeCap)})) {
        ImGui::TextDisabled("revolves, domes, disk caps");
        if (ImGui::DragInt("radial", &d.radial, 0.2f, 3, 256)) ch = true;
        hover({int(MK::RevolutionGrid), int(MK::DiskCap),
               int(MK::AnnulusRing), int(MK::PlateWeb),
               int(MK::RibbonSweep), int(MK::RailLadder), int(MK::DomeCap)});
        if (ImGui::DragInt("axial", &d.axial, 0.2f, 1, 256)) ch = true;
        hover({int(MK::RevolutionGrid), int(MK::DomeCap)});
        int cap = d.cap == weft::CapStyle::Fan ? 1 : 0;
        if (ImGui::Combo("cap style", &cap, "ngon\0fan\0")) {
            d.cap = cap ? weft::CapStyle::Fan : weft::CapStyle::NGon;
            ch = true;
        }
        hover({int(MK::DiskCap)});
        ImGui::EndTabItem();
    }
    if (tab("coons grids", {int(MK::CoonsGrid)})) {
        ImGui::TextDisabled("four-sided curved patches");
        if (ImGui::DragInt("grid u", &d.gridU, 0.2f, 1, 256)) ch = true;
        hover({int(MK::CoonsGrid)});
        if (ImGui::DragInt("grid v", &d.gridV, 0.2f, 1, 256)) ch = true;
        hover({int(MK::CoonsGrid)});
        if (d.adaptive) {
            ImGui::TextDisabled("adaptive ON: solved counts floor these");
        }
        ImGui::EndTabItem();
    }
    if (tab("fillets", {kFilletFaces})) {
        ImGui::TextDisabled("blend chains (coons / planar fillets)");
        ch |= ImGui::DragInt("fillet loops", &d.filletLoops, 0.2f, 1, 64);
        hover({kFilletFaces});
        float hold = float(d.filletHold);
        if (ImGui::SliderFloat("hold", &hold, 0.0f, 0.95f)) {
            d.filletHold = hold;
            ch = true;
        }
        hover({kFilletFaces});
        ImGui::EndTabItem();
    }
    if (tab("ribbons", {int(MK::RibbonSweep), int(MK::RailLadder)})) {
        ImGui::TextDisabled("grip / rail strips");
        if (ImGui::DragInt("rail density", &d.radial, 0.2f, 3, 256)) {
            ch = true;
        }
        hover({int(MK::RibbonSweep), int(MK::RailLadder)});
        ImGui::TextDisabled("(shares the revolve radial default)");
        ImGui::EndTabItem();
    }
    if (tab("rings", {int(MK::RingJunction), int(MK::AnnulusRing),
                      int(MK::PlateWeb)})) {
        ImGui::TextDisabled("hole plates default to no collar rim");
        {
            bool collars = d.junctionRings > 0;
            if (ImGui::Checkbox("hole collars", &collars)) {
                d.junctionRings =
                    collars ? std::max(1, d.junctionRings) : 0;
                ch = true;
            }
            hover({int(MK::PlateWeb)});
        }
        if (d.junctionRings > 0) {
            ch |= ImGui::DragInt("collar rings", &d.junctionRings, 0.2f, 1,
                                 32);
            hover({int(MK::RingJunction), int(MK::PlateWeb)});
            ch |= ImGui::Checkbox("square collars", &d.squareCollar);
            hover({int(MK::PlateWeb)});
        }
        ImGui::TextDisabled("ring-junction uses max(1, collar rings)");
        ImGui::EndTabItem();
    }
    if (tab("flat faces", {int(MK::MinimalNGon), int(MK::PlanarGrid),
                           int(MK::PlateWeb)})) {
        ImGui::TextDisabled("planar panels and grids");
        ch |= ImGui::Checkbox("minimal n-gon (flat panels)", &d.minimal);
        hover({int(MK::MinimalNGon)});
        if (ImGui::DragInt("grid u", &d.gridU, 0.2f, 1, 256)) ch = true;
        hover({int(MK::PlanarGrid), int(MK::CoonsGrid),
               int(MK::RingJunction)});
        if (ImGui::DragInt("grid v", &d.gridV, 0.2f, 1, 256)) ch = true;
        hover({int(MK::PlanarGrid), int(MK::CoonsGrid),
               int(MK::RingJunction)});
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    if (ch) markDirty(app);
}

// Mode indicator + hotkey reference, floating over the
// viewport so the keyboard flow never needs the side panel.
// The 3D viewport rectangle (the dockspace's central node): overlays
// anchor to it so they never sit under the docked panels.
static ImVec2 gViewMin{0, 0}, gViewMax{0, 0};

// While the bake-queue worker meshes, a centred card shows a spinning
// hourglass and per-face progress — the app never just hangs.
static void drawGenProgress(App& app) {
    const bool loading = app.loadBusy.load(std::memory_order_relaxed);
    const bool baking =
        app.bakeQueue.busy() || app.bakeQueue.hasPending();
    if (!loading && !baking) return;
    const double started = loading ? app.loadStartTime : app.genStartTime;
    if (glfwGetTime() - started < 0.2) return;  // no flicker
    ImGui::SetNextWindowPos({(gViewMin.x + gViewMax.x) * 0.5f,
                             (gViewMin.y + gViewMax.y) * 0.5f},
                            ImGuiCond_Always, {0.5f, 0.5f});
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::Begin("##genprogress", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoInputs);
    const float w = 260.0f * gUiScale;
    const float r = 15.0f * gUiScale;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImVec2 cur = ImGui::GetCursorScreenPos();
    ImVec2 ctr{cur.x + w * 0.5f, cur.y + r + 6.0f * gUiScale};
    const float spin = float(glfwGetTime()) * 3.0f;
    auto rot = [&](float x, float y) {
        const float cs = std::cos(spin), sn = std::sin(spin);
        return ImVec2{ctr.x + x * cs - y * sn, ctr.y + x * sn + y * cs};
    };
    const float h = r * 0.62f;
    const ImU32 amber = IM_COL32(242, 158, 46, 255);
    dl->AddTriangleFilled(rot(-h * 0.8f, -h), rot(h * 0.8f, -h), rot(0, 0),
                          amber);
    dl->AddTriangleFilled(rot(-h * 0.8f, h), rot(h * 0.8f, h), rot(0, 0),
                          amber);
    dl->PathArcTo(ctr, r + 4.0f * gUiScale, spin * 0.7f,
                  spin * 0.7f + 4.6f, 32);
    dl->PathStroke(IM_COL32(242, 158, 46, 160), 0, 2.5f * gUiScale);
    ImGui::Dummy({w, (r + 8.0f * gUiScale) * 2.0f});
    if (loading) {
        ImGui::Text("reading + repairing STEP...");
        ImGui::TextDisabled("%.1f s elapsed; the app remains responsive",
                            glfwGetTime() - app.loadStartTime);
    } else {
        const int done =
            app.bakeQueue.progressFaces.load(std::memory_order_relaxed);
        const int total =
            app.bakeQueue.progressTotal.load(std::memory_order_relaxed);
        char label[96];
        if (total < 0) {
            std::snprintf(label, sizeof label,
                          "planning affected faces...");
        } else if (total == 0) {
            std::snprintf(label, sizeof label,
                          "all faces reused; updating seams...");
        } else if (done < total) {
            std::snprintf(label, sizeof label, "meshing %d / %d faces", done,
                          total);
        } else {
            std::snprintf(label, sizeof label, "assembling preview...");
        }
        const float fraction =
            total <= 0 ? (total == 0 ? 1.0f : 0.0f)
                       : std::min(1.0f, float(done) / float(total));
        ImGui::ProgressBar(fraction, {w, 0}, label);
        const int pending = app.bakeQueue.pendingDepth();
        const auto bakingFaces =
            app.bakeQueue.facesInState(weft_app::FaceFidelity::Baking);
        if (!bakingFaces.empty()) {
            ImGui::TextDisabled("baking %zu face region(s)...",
                                bakingFaces.size());
        }
        if (pending > 0 || app.dirty) {
            ImGui::TextDisabled(
                "newer edits queued (%d pending) — latest params win",
                std::max(pending, app.dirty ? 1 : 0));
        }
    }
    ImGui::End();
}

static void drawOverlay(App& app) {
    ImGui::SetNextWindowPos(
        {gViewMin.x + 12 * gUiScale, gViewMin.y + 12 * gUiScale});
    ImGui::SetNextWindowBgAlpha(0.55f);
    ImGui::Begin("##overlay", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    if (app.gpuProxyPending) {
        const std::array<float, 2> n = gpuProxyCounts(app);
        ImGui::TextColored({0.20f, 0.90f, 1.0f, 1.0f},
                           "GPU PREVIEW  %.0f x %.0f", n[0], n[1]);
        ImGui::TextDisabled("exact topology settling...");
        ImGui::Separator();
    }
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
                                  "F fills hovered - [ ] twists the active "
                                  "side (shift+wheel flips A/B, each keeps "
                                  "its twist) - J/esc exits");
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
                "click verts (shift extends) - G grabs interior verts - "
                "M welds");
        } else {
            ImGui::TextDisabled("%zu vert(s) - G grabs - M welds",
                                app.selVerts.size());
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

    // Keybinds help: a minimised prompt bottom-left; click to open the
    // formatted panel, the "–" in its corner collapses it again.
    ImGui::SetNextWindowPos(
        {gViewMin.x + 12 * gUiScale, gViewMax.y - 12 * gUiScale},
        ImGuiCond_Always, {0.0f, 1.0f});
    ImGui::SetNextWindowBgAlpha(app.showKeybinds ? 0.80f : 0.45f);
    ImGui::Begin("##hotkeys", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoFocusOnAppearing |
                     ImGuiWindowFlags_NoNav);
    const ImVec4 kKeyBlue{0.42f, 0.68f, 1.0f, 1.0f};
    if (!app.showKeybinds) {
        ImGui::TextColored(kKeyBlue, "Keybinds");
        ImGui::SameLine();
        ImGui::TextDisabled("help");
        if (ImGui::IsWindowHovered() &&
            ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            app.showKeybinds = true;
        }
    } else {
        ImGui::TextColored(kKeyBlue, "Keybinds");
        ImGui::SameLine(0.0f, 12.0f * gUiScale);
        if (ImGui::SmallButton("-")) app.showKeybinds = false;
        ImGui::Spacing();
        auto section = [&](const char* title) {
            ImGui::Spacing();
            ImGui::TextDisabled("%s", title);
        };
        auto bind = [&](const char* key, const char* what) {
            ImGui::TextColored(kKeyBlue, "%-18s", key);
            ImGui::SameLine(140.0f * gUiScale);
            ImGui::TextUnformatted(what);
        };
        section("select");
        bind("1 - 6", "verts / edges / faces / feature edges / elements / objects");
        bind("click", "select    shift+click extends");
        bind("drag", "box-select (shift extends)");
        bind("Tab / Q", "mode pie / tools pie (hold, release picks)");
        bind("ctrl+I", "invert selection");
        bind("esc", "clear");
        section("edit");
        bind("X", "delete face");
        bind("H / ctrl+H", "hide / hide others    alt+H show all");
        bind("R", "loop cut");
        bind("J", "bridge");
        bind("G", "grab vertex");
        bind("M", "weld selected verts (vert mode)");
        bind("ctrl+E", "export dialog");
        bind("C / T / M", "cap / tris / minimal");
        bind("W / B", "wire / edges");
        bind("[ ]", "nudge counts");
        bind("ctrl+Z", "undo    ctrl+shift+Z redo");
        section("density (hover a face, or empty space for globals)");
        bind("shift+wheel", "density");
        bind("ctrl+wheel", "second axis");
        bind("ctrl+shift+wheel", "fillet loops");
        section("view");
        bind("numpad 1/3/7", "axis views (ctrl flips)    numpad 5 ortho");
        bind("F", "focus selection");
        bind("wheel", "zoom");
    }
    ImGui::End();
}

// Context popup on RIGHT-click over a face (Blender-style): the active
// face's live controls (edits apply to the whole selection and override
// automatically) plus visibility actions. Left-click just selects; the
// modal keys/wheel are the primary editing path.

// Weld menu (Blender's M merge): the selected verts collapse into one,
// recorded as a replayable op keyed to their world positions. Drawn
// INSIDE the ImGui frame: popup calls in the pre-NewFrame input section
// dereference a null current window the moment any other popup is open
// (the right-click / shading-button crashes).

// Export dialog: format + engine-space settings in one place, opened by
// the Export button or ctrl+E (deferred: the key handler runs before
// NewFrame where popup calls are illegal).
static void drawExportPopup(App& app) {
    if (app.openExportPopup) {
        ImGui::OpenPopup("Export settings");
        app.openExportPopup = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(),
                            ImGuiCond_Appearing, {0.5f, 0.5f});
    if (ImGui::BeginPopupModal("Export settings", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
        const char* formats[] = {"OBJ (.obj)", "glTF binary (.glb)",
                                 "FBX (.fbx)"};
        ImGui::SetNextItemWidth(200.0f * gUiScale);
        ImGui::Combo("format", &app.exportFormat, formats, 3);
        ImGui::Checkbox("triangulate", &app.exportTriangulate);
        if (app.exportFormat == 1 && ImGui::IsItemHovered()) {
            ImGui::SetTooltip("glTF always triangulates");
        }
        ImGui::SameLine();
        ImGui::Checkbox("Y up", &app.exportYUp);
        ImGui::SetNextItemWidth(120.0f * gUiScale);
        ImGui::DragFloat("unit scale", &app.exportScale, 0.001f, 0.0001f,
                         1000.0f, "%.4g");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("0.001 = mm to metres (Unity/Blender)\n"
                              "0.1 = mm to cm (Unreal)");
        }
        ImGui::Separator();
        if (ImGui::Button("Export", {120.0f * gUiScale, 0})) {
            const char* exts[] = {".obj", ".glb", ".fbx"};
            std::string base = app.sourcePath;
            size_t slash = base.find_last_of("/\\");
            if (slash != std::string::npos) base = base.substr(slash + 1);
            size_t dot = base.find_last_of('.');
            if (dot != std::string::npos) base = base.substr(0, dot);
            if (base.empty()) base = "weft";
            std::string name = base + exts[app.exportFormat];
            ImGui::CloseCurrentPopup();
            std::string out = saveFileDialog(name.c_str());
            if (!out.empty()) {
                exportMeshTo(app, out);
            } else if (gNoDialogTool) {
                gBrowser.start(true, name.c_str());
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", {120.0f * gUiScale, 0})) {
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
}

static void drawWeldPopup(App& app) {
    if (app.openWeldPopup) {
        ImGui::OpenPopup("weld verts");
        app.openWeldPopup = false;
    }
    if (ImGui::BeginPopup("weld verts")) {
        auto doWeld = [&](int mode) {
            weft::ManualOp op;
            op.kind = weft::ManualOp::Kind::WeldVerts;
            op.weldMode = mode;
            for (uint32_t v : app.selVertOrder) {
                const auto& q = app.mesh.vertices[v];
                op.weldPoints.push_back({q[0], q[1], q[2]});
            }
            size_t n = op.weldPoints.size();
            weft::PolyMesh probe = app.mesh;
            if (weft::weldVerts(probe, op) < 2) {
                app.status = "weld failed: could not merge those verts";
                ImGui::CloseCurrentPopup();
                return;
            }
            app.recipe.ops.push_back(std::move(op));
            app.selVerts.clear();
            app.selVertOrder.clear();
            markDirty(app);
            app.status = "welded " + std::to_string(n) +
                         " vert(s) (ctrl+Z undoes)";
        };
        ImGui::TextDisabled("weld %zu verts", app.selVerts.size());
        ImGui::Separator();
        if (ImGui::MenuItem("at center")) doWeld(0);
        if (ImGui::MenuItem("at last pick")) doWeld(1);
        if (ImGui::MenuItem("at first pick")) doWeld(2);
        ImGui::EndPopup();
    }
}

static void drawFacePopup(App& app) {
    if (app.openFacePopup) {
        if (app.activeFace > 0) ImGui::OpenPopup("##facectx");
        app.openFacePopup = false;
    }
    if (!ImGui::BeginPopup("##facectx")) return;
    if (app.activeFace <= 0 ||
        app.activeFace > int(app.analysis.faces.size())) {
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
    }
    const weft::FaceInfo& f = app.analysis.faces[app.activeFace - 1];
    if (app.selFaces.size() > 1) {
        ImGui::Text("%zu faces (active #%d)", app.selFaces.size(), f.id);
        drawSelectedFacesRoster(app);
        ImGui::TextDisabled("editing active:");
    }
    ImGui::Text("%s", faceDebugLabel(app, f.id).c_str());
    ImGui::Separator();
    ImGui::PushID("ctx");
    ImGui::PushItemWidth(150 * gUiScale);
    drawActiveFaceSettings(app);
    ImGui::PopItemWidth();
    ImGui::PopID();

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


// Orientation gizmo under the shading bar: the three world axes drawn
// from the camera's own basis. Clicking an axis ball snaps the view to
// look down that axis; clicking it again flips to the opposite side.
static void drawAxisGizmo(App& app) {
    const float sz = 92.0f * gUiScale;
    // Anchor to the CENTRAL 3D area (gViewMin/gViewMax), not the OS
    // viewport — the Outliner docks at the right edge.
    ImGui::SetNextWindowPos({gViewMax.x - 10.0f * gUiScale,
                             gViewMin.y + 44.0f * gUiScale},
                            ImGuiCond_Always, {1.0f, 0.0f});
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::Begin("##axisgizmo", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing);
    ImGui::InvisibleButton("##gizmoarea", {sz, sz});
    const ImVec2 mn = ImGui::GetItemRectMin();
    const ImVec2 c{mn.x + sz * 0.5f, mn.y + sz * 0.5f};
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Camera basis (z-up view, same as matLookAt/mouseRay).
    Vec3 eye = app.cam.eye();
    Vec3 f = norm(sub(app.cam.target, eye));
    Vec3 s = norm(cross(f, {0, 0, 1}));
    Vec3 u = cross(s, f);
    struct Axis {
        Vec3 dir;
        ImU32 col;
        const char* label;
        float yaw, pitch;  // view that looks down this axis
    };
    const float hp = 1.55f;
    const Axis axes[3] = {
        {{1, 0, 0}, IM_COL32(235, 66, 78, 255), "X", 0.0f, 0.0f},
        {{0, 1, 0}, IM_COL32(108, 202, 77, 255), "Y", 1.5708f, 0.0f},
        {{0, 0, 1}, IM_COL32(72, 135, 240, 255), "Z", app.cam.yaw, hp},
    };
    const float R = sz * 0.36f;
    struct Ball {
        ImVec2 pos;
        float depth;
        ImU32 col;
        const char* label;  // null on the negative end
        float yaw, pitch;
    };
    std::vector<Ball> balls;
    for (const Axis& a : axes) {
        float sx = a.dir.x * s.x + a.dir.y * s.y + a.dir.z * s.z;
        float sy = a.dir.x * u.x + a.dir.y * u.y + a.dir.z * u.z;
        float dz = a.dir.x * f.x + a.dir.y * f.y + a.dir.z * f.z;
        balls.push_back({{c.x + sx * R, c.y - sy * R}, dz, a.col,
                         a.label, a.yaw, a.pitch});
        balls.push_back({{c.x - sx * R, c.y + sy * R}, -dz, a.col,
                         nullptr,
                         a.label[0] == 'Z' ? a.yaw : a.yaw + 3.1416f,
                         a.label[0] == 'Z' ? -hp : -a.pitch});
    }
    // Far side first so the near balls draw on top.
    std::sort(balls.begin(), balls.end(),
              [](const Ball& a, const Ball& b) { return a.depth > b.depth; });
    for (const Ball& b : balls) {
        if (b.label) {
            dl->AddLine(c, b.pos, (b.col & 0x00ffffff) | 0xB0000000,
                        1.6f * gUiScale);
        }
    }
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const Ball* hit = nullptr;
    for (const Ball& b : balls) {
        const float r = (b.label ? 7.0f : 5.0f) * gUiScale;
        const bool hov = ImGui::IsItemHovered() &&
                         std::hypot(mouse.x - b.pos.x, mouse.y - b.pos.y) <
                             r + 2.0f * gUiScale;
        ImU32 col = b.depth <= 0.0f ? b.col
                                    : (b.col & 0x00ffffff) | 0x66000000;
        if (b.label) {
            dl->AddCircleFilled(b.pos, r, col, 20);
            dl->AddText({b.pos.x - 3.5f * gUiScale,
                         b.pos.y - 6.5f * gUiScale},
                        IM_COL32(15, 15, 18, 255), b.label);
        } else {
            dl->AddCircleFilled(b.pos, r,
                                (b.col & 0x00ffffff) | 0x30000000, 16);
            dl->AddCircle(b.pos, r, col, 16, 1.4f * gUiScale);
        }
        if (hov) {
            dl->AddCircle(b.pos, r + 2.0f * gUiScale,
                          IM_COL32(255, 255, 255, 180), 20,
                          1.5f * gUiScale);
            if (ImGui::IsMouseClicked(0)) hit = &b;
        }
    }
    if (hit) {
        // Clicking the axis you are already on flips to the far side.
        const bool same = std::abs(app.cam.yaw - hit->yaw) < 0.05f &&
                          std::abs(app.cam.pitch - hit->pitch) < 0.05f;
        if (same) {
            app.cam.yaw = hit->pitch == 0.0f ? hit->yaw + 3.1416f
                                             : hit->yaw;
            app.cam.pitch = -hit->pitch;
        } else {
            app.cam.yaw = hit->yaw;
            app.cam.pitch = hit->pitch;
        }
    }
    ImGui::End();
}

// Blender-style viewport shading controls: a compact button row pinned to
// the viewport's top-right corner (wire / solid / matcap / flat) plus a
// popover with the full lighting, overlay, and background settings.
static void drawShadingBar(App& app) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float panelW = 330.0f * gUiScale;
    ImGui::SetNextWindowPos({vp->WorkPos.x + vp->WorkSize.x - panelW -
                                 10.0f * gUiScale,
                             vp->WorkPos.y + 10.0f * gUiScale},
                            ImGuiCond_Always, {1.0f, 0.0f});
    ImGui::SetNextWindowBgAlpha(0.85f);
    ImGui::Begin("##shadingbar", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing);
    auto modeBtn = [&](const char* label, bool active, const char* tip) {
        if (active) {
            ImGui::PushStyleColor(ImGuiCol_Button,
                                  {0.94f, 0.62f, 0.18f, 1.0f});
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                                  {0.98f, 0.70f, 0.26f, 1.0f});
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  {0.09f, 0.09f, 0.10f, 1.0f});
        }
        bool hit = ImGui::SmallButton(label);
        if (active) ImGui::PopStyleColor(3);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
        ImGui::SameLine(0.0f, 3.0f * gUiScale);
        return hit;
    };
    if (modeBtn("wire", !app.showFill, "wireframe only")) {
        app.showFill = false;
        app.showWire = true;
    }
    if (modeBtn("solid", app.showFill && app.lightStyle == 0,
                "studio-lit solid")) {
        app.showFill = true;
        app.lightStyle = 0;
    }
    if (modeBtn("matcap", app.showFill && app.lightStyle == 1,
                "matcap: shading follows the view, reads curvature")) {
        app.showFill = true;
        app.lightStyle = 1;
    }
    if (modeBtn("flat", app.showFill && app.lightStyle == 2,
                "flat face colours")) {
        app.showFill = true;
        app.lightStyle = 2;
    }
    if (ImGui::SmallButton("v##shadepop")) {
        ImGui::OpenPopup("viewport shading");
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("viewport shading");

    if (ImGui::BeginPopup("viewport shading")) {
        ImGui::TextDisabled("Viewport Shading");
        ImGui::Separator();
        ImGui::TextDisabled("lighting");
        if (ImGui::RadioButton("studio", app.lightStyle == 0)) {
            app.lightStyle = 0;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("matcap", app.lightStyle == 1)) {
            app.lightStyle = 1;
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("flat", app.lightStyle == 2)) {
            app.lightStyle = 2;
        }
        if (app.lightStyle == 0) {
            ImGui::SliderFloat("ambient", &app.lightAmbient, 0.0f, 1.0f);
            ImGui::SliderFloat("diffuse", &app.lightDiffuse, 0.0f, 1.5f);
            ImGui::SliderFloat("rim light", &app.lightRim, 0.0f, 0.5f);
        } else if (app.lightStyle == 1) {
            // Preview sphere of the procedural studio matcap.
            ImDrawList* dl = ImGui::GetWindowDrawList();
            const float R = 30.0f * gUiScale;
            ImVec2 p = ImGui::GetCursorScreenPos();
            float w = ImGui::GetContentRegionAvail().x;
            ImVec2 c{p.x + w * 0.5f, p.y + R + 6.0f * gUiScale};
            dl->AddRectFilled({p.x, p.y},
                              {p.x + w, p.y + 2 * R + 12.0f * gUiScale},
                              IM_COL32(22, 23, 27, 255), 4.0f * gUiScale);
            dl->AddCircleFilled(c, R, IM_COL32(56, 57, 62, 255), 48);
            for (int i = 1; i <= 12; ++i) {
                float t = i / 12.0f;
                int g = int(150 * t * t);
                dl->AddCircleFilled({c.x - 0.30f * R * t,
                                     c.y - 0.40f * R * t},
                                    R * (1.0f - 0.55f * t),
                                    IM_COL32(56 + g, 57 + g, 60 + g, 26),
                                    48);
            }
            dl->AddCircleFilled({c.x - 0.36f * R, c.y - 0.48f * R},
                                R * 0.12f, IM_COL32(246, 246, 242, 220),
                                24);
            dl->AddCircle(c, R, IM_COL32(12, 12, 14, 255), 48,
                          1.5f * gUiScale);
            ImGui::Dummy({w, 2 * R + 14.0f * gUiScale});
        }
        ImGui::Separator();
        ImGui::TextDisabled("overlays");
        ImGui::Checkbox("wireframe", &app.showWire);
        ImGui::SameLine();
        ImGui::ColorEdit3("##wirecol", app.wireColor,
                          ImGuiColorEditFlags_NoInputs);
        ImGui::SameLine();
        ImGui::Checkbox("feature edges", &app.showBrepEdges);
        ImGui::Checkbox("selection verts", &app.showVerts);
        ImGui::SameLine();
        ImGui::ColorEdit3("##vertcol", app.vertColor,
                          ImGuiColorEditFlags_NoInputs);
        if (ImGui::Checkbox("quality heatmap", &app.qualityView)) {
            rebuildBuffers(app);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("tint polys by worst corner angle:\n"
                              "green ok, orange skewed, red sliver");
        }
        ImGui::SameLine();
        ImGui::Checkbox("problems", &app.showProblems);
        ImGui::Separator();
        ImGui::TextDisabled("background");
        ImGui::ColorEdit3("##bgcol", app.bgColor,
                          ImGuiColorEditFlags_NoInputs);
        ImGui::SameLine();
        auto theme = [&](const char* name, float br, float bg2, float bb,
                         float wr, float wg, float wb) {
            if (ImGui::SmallButton(name)) {
                app.bgColor[0] = br;
                app.bgColor[1] = bg2;
                app.bgColor[2] = bb;
                app.wireColor[0] = wr;
                app.wireColor[1] = wg;
                app.wireColor[2] = wb;
            }
            ImGui::SameLine();
        };
        theme("dark", 0.117f, 0.125f, 0.145f, 0.10f, 0.11f, 0.13f);
        theme("light", 0.86f, 0.87f, 0.89f, 0.28f, 0.29f, 0.32f);
        theme("slate", 0.16f, 0.19f, 0.24f, 0.09f, 0.11f, 0.15f);
        ImGui::NewLine();
        ImGui::EndPopup();
    }
    ImGui::End();
}

// The Outliner lives in its own dockable window (right side by
// default): object visibility, per-face lists, CAD part names.
static void drawOutliner(App& app) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({vp->WorkPos.x + vp->WorkSize.x -
                                 300.0f * gUiScale,
                             vp->WorkPos.y + 40.0f * gUiScale},
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({280.0f * gUiScale, 420.0f * gUiScale},
                             ImGuiCond_FirstUseEver);
    ImGui::Begin("Outliner");
    if (app.hasModel) {
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
            // CAD part names carry all the way through: the outliner
            // shows what the source software called the body.
            char objLabel[96];
            const std::string nm = si < app.model.solidNames.size()
                                       ? app.model.solidNames[si]
                                       : std::string();
            if (!nm.empty()) {
                std::snprintf(objLabel, sizeof objLabel, "%s (%zu faces)",
                              nm.c_str(), fids.size());
            } else {
                std::snprintf(objLabel, sizeof objLabel,
                              "object %zu (%zu faces)", si + 1, fids.size());
            }
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
                    // A face that emitted nothing has no polygons to click
                    // in the viewport — the outliner is its way back.
                    auto bld = app.report.faceBuild.find(fid);
                    bool empty = bld != app.report.faceBuild.end() &&
                                 bld->second == -1 && !deleted;
                    char label[120];
                    std::snprintf(label, sizeof label, "face %-4d %s%s%s%s%s",
                                  fid, weft::surfaceTypeName(f.type),
                                  f.isFillet ? " [fillet]" : "",
                                  f.isHole ? " [hole]" : "",
                                  deleted ? " [deleted]" : "",
                                  empty ? " [empty!]" : "");
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
    ImGui::End();
}

static void drawUi(App& app) {
    // Rebuilt every frame from whichever control is hovered right now.
    app.highlightMeshers.clear();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float width = 330.0f * gUiScale;
    ImGui::SetNextWindowPos({vp->WorkPos.x, vp->WorkPos.y},
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize({width, vp->WorkSize.y},
                             ImGuiCond_FirstUseEver);
    ImGui::Begin("Settings", nullptr, ImGuiWindowFlags_NoCollapse);

    if (gLogoBadgeTex) {
        const float h = 30.0f * gUiScale;
        ImGui::Image((ImTextureID)(intptr_t)gLogoBadgeTex, {h, h});
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::TextColored({0.95f, 0.62f, 0.18f, 1.0f}, "WEFT");
        ImGui::TextDisabled("b-rep retopology");
        ImGui::EndGroup();
    } else {
        ImGui::TextColored({0.95f, 0.62f, 0.18f, 1.0f}, "WEFT");
        ImGui::SameLine();
        ImGui::TextDisabled("b-rep retopology");
    }
    ImGui::Separator();
    drawShadingBar(app);
    drawAxisGizmo(app);

    if (ImGui::CollapsingHeader("Model", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Open STEP...", {-1, 0})) {
            std::string p = openFileDialog();
            if (!p.empty()) {
                std::snprintf(app.pathBuf, sizeof app.pathBuf, "%s",
                              p.c_str());
                loadModel(app, p);
            }
            else if (gNoDialogTool) {
                gBrowser.start(false, nullptr);
            }
        }
        // Built-in browser results (native-dialog-less systems).
        gBrowser.draw();
        if (!gBrowser.picked.empty() && !gBrowser.saveMode) {
            std::string p = std::move(gBrowser.picked);
            gBrowser.picked.clear();
            std::snprintf(app.pathBuf, sizeof app.pathBuf, "%s", p.c_str());
            loadModel(app, p);
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
        if (app.hasModel && ImGui::Button("Export...  (ctrl+E)", {-1, 0})) {
            app.openExportPopup = true;
        }
        if (!gBrowser.picked.empty() && gBrowser.saveMode) {
            std::string out = std::move(gBrowser.picked);
            gBrowser.picked.clear();
            exportMeshTo(app, out);
        }
        ImGui::TextWrapped("%s", app.status.c_str());
    }


    if (app.hasModel &&
        ImGui::CollapsingHeader("Topology", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("%zu verts   %zu polys", app.mesh.vertexCount(),
                    app.mesh.polygonCount());
        ImGui::Text("%zu quads  %zu tris  %zu n-gons", app.mesh.countQuads(),
                    app.mesh.countTris(), app.mesh.countNgons());
        if (!app.meshFinalized) {
            ImGui::TextDisabled(
                "interactive preview - seam repair runs on export");
        } else if (app.openEdgeCount == 0 && app.multiEdgeCount == 0) {
            ImGui::TextColored({0.4f, 0.9f, 0.45f, 1.0f}, "watertight");
        } else {
            ImGui::TextColored({1.0f, 0.45f, 0.3f, 1.0f},
                               "%d open edge(s), %d non-manifold",
                               app.openEdgeCount, app.multiEdgeCount);
            ImGui::SameLine();
            ImGui::Checkbox("show##problems", &app.showProblems);
        }
        if (app.foldedPolyCount > 0) {
            ImGui::TextColored({1.0f, 0.25f, 0.2f, 1.0f},
                               "%d folded cell(s)", app.foldedPolyCount);
            if (app.openEdgeCount == 0 && app.multiEdgeCount == 0) {
                ImGui::SameLine();
                ImGui::Checkbox("show##problems", &app.showProblems);
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("select##foldedfaces")) {
                const std::vector<uint8_t> folded =
                    weft::foldedPolys(app.model, app.mesh);
                std::set<int> owners;
                for (size_t p = 0; p < folded.size(); ++p) {
                    if (!folded[p] || p >= app.mesh.polygonFaceId.size()) {
                        continue;
                    }
                    const int fid = app.mesh.polygonFaceId[p];
                    if (fid >= 1 && fid <= app.model.faceCount()) {
                        owners.insert(fid);
                    }
                }
                selectFacesMatching(app, [&](int fid) {
                    return owners.count(fid) > 0;
                });
            }
        }
        if (!app.bLoops.empty()) {
            ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                               "%zu open border loop(s)",
                               app.bLoops.size());
            ImGui::SameLine();
            ImGui::TextDisabled("(J bridges, deleted faces expected)");
            ImGui::SameLine();
            if (ImGui::SmallButton("select##openloopfaces")) {
                // Faces that own any polygon touching a boundary-loop
                // vertex — the usual owners of unexplained open seams.
                std::vector<uint8_t> onLoop(app.mesh.vertexCount(), 0);
                for (const auto& loop : app.bLoops) {
                    for (uint32_t v : loop) {
                        if (v < onLoop.size()) onLoop[v] = 1;
                    }
                }
                std::set<int> owners;
                for (size_t p = 0; p < app.mesh.polygons.size(); ++p) {
                    if (p >= app.mesh.polygonFaceId.size()) continue;
                    bool hit = false;
                    for (uint32_t v : app.mesh.polygons[p]) {
                        if (v < onLoop.size() && onLoop[v]) {
                            hit = true;
                            break;
                        }
                    }
                    if (!hit) continue;
                    const int fid = app.mesh.polygonFaceId[p];
                    if (fid >= 1 && fid <= app.model.faceCount()) {
                        owners.insert(fid);
                    }
                }
                selectFacesMatching(app, [&](int fid) {
                    return owners.count(fid) > 0;
                });
            }
        }
        // Build health from the report: a face that emitted nothing is a
        // hole in the output with nothing to click in the viewport — the
        // select button routes it back into the Selection panel where its
        // override can be cleared (and ctrl+Z now rebuilds reliably).
        {
            int emptyFaces = 0, rawFaces = 0, floorFaces = 0;
            for (const auto& [fid, b] : app.report.faceBuild) {
                if (b == -1) ++emptyFaces;
                else if (b == 1) ++rawFaces;
                else if (b == 2) ++floorFaces;
            }
            if (emptyFaces > 0) {
                ImGui::TextColored({1.0f, 0.35f, 0.3f, 1.0f},
                                   "%d face(s) emitted nothing", emptyFaces);
                ImGui::SameLine();
                if (ImGui::SmallButton("select##emptyfaces")) {
                    selectFacesMatching(app, [&](int fid) {
                        auto it = app.report.faceBuild.find(fid);
                        return it != app.report.faceBuild.end() &&
                               it->second == -1;
                    });
                }
            }
            if (rawFaces > 0) {
                ImGui::TextColored({1.0f, 0.6f, 0.3f, 1.0f},
                                   "%d face(s) on raw fallback", rawFaces);
                ImGui::SameLine();
                if (ImGui::SmallButton("select##rawfaces")) {
                    selectFacesMatching(app, [&](int fid) {
                        auto it = app.report.faceBuild.find(fid);
                        return it != app.report.faceBuild.end() &&
                               it->second == 1;
                    });
                }
            }
            if (floorFaces > 0) {
                ImGui::TextColored({1.0f, 0.7f, 0.35f, 1.0f},
                                   "%d face(s) on contract floor",
                                   floorFaces);
                ImGui::SameLine();
                if (ImGui::SmallButton("select##floorfaces")) {
                    selectFacesMatching(app, [&](int fid) {
                        auto it = app.report.faceBuild.find(fid);
                        return it != app.report.faceBuild.end() &&
                               it->second == 2;
                    });
                }
            }
        }
        ImGui::Separator();
        // One knob for the whole budget: scales every density proposal.
        float ds = float(app.recipe.settings.densityScale);
        if (ImGui::SliderFloat("density scale", &ds, 0.25f, 4.0f, "%.2fx",
                               ImGuiSliderFlags_Logarithmic)) {
            // The slider covers the everyday range; typed entry
            // (ctrl+click or double-click) reaches the full one.
            app.recipe.settings.densityScale =
                std::clamp(double(ds), 0.05, 20.0);
            markDirty(app);
        }
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
            ImGui::OpenPopup("density value");
        }
        if (ImGui::BeginPopup("density value")) {
            static float typed = 1.0f;
            if (ImGui::IsWindowAppearing()) {
                typed = float(app.recipe.settings.densityScale);
                ImGui::SetKeyboardFocusHere();
            }
            ImGui::SetNextItemWidth(90.0f * gUiScale);
            if (ImGui::InputFloat("##densityexact", &typed, 0, 0,
                                  "%.3f",
                                  ImGuiInputTextFlags_EnterReturnsTrue)) {
                app.recipe.settings.densityScale =
                    std::clamp(double(typed), 0.05, 20.0);
                markDirty(app);
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            ImGui::TextDisabled("0.05 .. 20");
            ImGui::EndPopup();
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
                next = std::clamp(next, 0.05, 20.0);
                if (std::abs(next - app.recipe.settings.densityScale) <
                    1e-3) {
                    break;
                }
                app.recipe.settings.densityScale = next;
                regenerate(app);
            }
            rebuildBuffers(app);
            // dirty is left alone: the sync regenerates cleared it when
            // they snapshotted, and if the loop broke before running at
            // all, a queued coalesced edit may still be pending — the
            // same silent drop finishGenerate used to cause.
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
        // Global weld tolerance (mm): how far apart coincident border
        // verts may sit and still fuse. The everyday floor is 1e-6;
        // raising it closes seams on sloppy CAD / off-curve fallback
        // borders. Per-face overrides (in the face panel) win when looser.
        float wt = float(app.recipe.settings.weldTolerance);
        if (ImGui::SliderFloat("weld tol (mm)", &wt, 1e-6f, 1.0f, "%.5f",
                               ImGuiSliderFlags_Logarithmic)) {
            app.recipe.settings.weldTolerance =
                std::clamp(double(wt), 1e-6, 1.0);
            markDirty(app);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("global weld tolerance: fuse border verts\n"
                              "within this distance (mm). Clamped to the\n"
                              "local feature size so it can't collapse\n"
                              "real geometry.");
        }
    }

    // Global defaults live in their own left-panel section, one tab per
    // mesher family — cylinders, fillets, ribbons, rings, flat faces —
    // instead of a flat wall of every knob at once.
    if (app.hasModel &&
        ImGui::CollapsingHeader("Mesher defaults",
                                ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushID("defaults");
        drawMesherDefaultTabs(app);
        ImGui::PopID();
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
        } else if (app.activeFace <= 0 ||
                   app.activeFace > int(app.analysis.faces.size())) {
            ImGui::TextDisabled("click a face in the viewport");
        } else {
            const weft::FaceInfo& f = app.analysis.faces[app.activeFace - 1];
            if (app.selFaces.size() > 1) {
                ImGui::Text("%zu faces (active #%d)", app.selFaces.size(),
                            f.id);
                drawSelectedFacesRoster(app);
                ImGui::Separator();
                ImGui::TextDisabled("editing active face:");
            }
            ImGui::Text("%s", faceDebugLabel(app, f.id).c_str());
            if (f.radius > 0) ImGui::Text("radius %.3f", f.radius);
            ImGui::PushID("perface");
            drawActiveFaceSettings(app);
            ImGui::PopID();
        }
    }

    if (ImGui::CollapsingHeader("Display")) {
        // The viewport display toggles live here too (the shading
        // popover keeps its copies for quick access).
        ImGui::Checkbox("wireframe", &app.showWire);
        ImGui::SameLine();
        ImGui::Checkbox("feature edges", &app.showBrepEdges);
        ImGui::Checkbox("show folded cells", &app.showProblems);
        ImGui::SetNextItemWidth(110.0f * gUiScale);
        ImGui::SliderFloat("vert size (active)", &app.vertSizeActive,
                           1.0f, 12.0f, "%.0f px");
        ImGui::SetNextItemWidth(110.0f * gUiScale);
        ImGui::SliderFloat("vert size (inactive)",
                           &app.vertSizeInactive, 1.0f, 12.0f,
                           "%.0f px");
        if (ImGui::Checkbox("smooth shading", &app.smoothShade)) {
            rebuildBuffers(app);
        }
        if (app.smoothShade) {
            if (ImGui::SliderFloat("smooth angle", &app.smoothAngleDeg, 0.0f,
                                   180.0f, "%.0f deg")) {
                rebuildBuffers(app);
            }
            if (ImGui::Checkbox("CAD-exact normals", &app.exactNormals)) {
                rebuildBuffers(app);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "shade with the true surface normal at each corner\n"
                    "(coarse cylinders stop banding; matches the OBJ/glTF\n"
                    "export). Off = smoothing-angle averages only.");
            }
        }
        ImGui::TextDisabled("overlays: viewport corner popover");
        ImGui::TextDisabled("orange convex / blue concave / green smooth");
        int scheme = gLaptopControls ? 1 : 0;
        const char* schemes[] = {"desktop (3-button mouse)",
                                 "laptop (trackpad, no MMB)"};
        if (ImGui::Combo("controls", &scheme, schemes, 2)) {
            gLaptopControls = scheme == 1;
            saveControls();
        }
        if (gLaptopControls) {
            ImGui::TextDisabled("LMB select · alt+LMB orbit");
            ImGui::TextDisabled("shift+alt+LMB pan · ctrl+alt+LMB zoom");
            ImGui::TextDisabled("two-finger scroll zooms");
        } else {
            ImGui::TextDisabled("LMB select · MMB orbit · shift+MMB pan");
            ImGui::TextDisabled("ctrl+MMB zoom · alt+MMB axis view · wheel");
        }
        ImGui::TextDisabled("hover a face, no selection needed:");
        ImGui::TextDisabled("shift+wheel density · ctrl+wheel 2nd axis");
        ImGui::TextDisabled("ctrl+shift+wheel loops · empty space = global");
    }

    if (ImGui::CollapsingHeader("Advanced")) {
        if (app.hasModel && ImGui::TreeNode("Recipe")) {
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
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("Debug")) {
            ImGui::TextDisabled("bisect switches — try these if it crashes");
            bool single = !app.recipe.settings.parallelMeshing;
            if (ImGui::Checkbox("single-threaded meshing", &single)) {
                app.recipe.settings.parallelMeshing = !single;
                markDirty(app);
            }
            bool conform = app.recipe.settings.conformBorders;
            if (ImGui::Checkbox("export border conformity", &conform)) {
                app.recipe.settings.conformBorders = conform;
                markDirty(app);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "Deferred during interactive preview; applied by the\n"
                    "authoritative mesh pass when exporting.");
            }
            bool stitch = app.recipe.settings.decoupleSeams;
            if (ImGui::Checkbox("decoupled seams (stitch)", &stitch)) {
                app.recipe.settings.decoupleSeams = stitch;
                markDirty(app);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(
                    "EXPERIMENT: skip global count equalization; the\n"
                    "post-weld splice reconciles mismatched seams with\n"
                    "n-gons instead of forced counts / absorber strips.");
            }
            static bool coreTrace = true;
            if (ImGui::Checkbox("core trace in log", &coreTrace)) {
                weft::setGenerateDebugLog(coreTrace ? gDebugLog : nullptr);
            }
            if (ImGui::Button("force regenerate")) app.dirty = true;
            ImGui::TextDisabled("log (flushed per line — after a crash its");
            ImGui::TextDisabled("tail names the face/stage that died):");
            ImGui::TextWrapped("%s", gLogPath.c_str());
            ImGui::TreePop();
        }
        if (ImGui::TreeNode("Dev fixtures")) {
            ImGui::TextDisabled("built-in test shapes");
            const char* fixtures[] = {
                "cylinder", "box",     "cone",   "sphere",
                "torus",    "fillet",  "hole",   "boss",
                "demo",     "notched", "slotted", "barrel",
                "bossfillet", "ribbon", "ribbonnotch"};
            const int nFixtures = int(sizeof fixtures / sizeof *fixtures);
            for (int i = 0; i < nFixtures; ++i) {
                if (i % 3) ImGui::SameLine();
                if (ImGui::Button(fixtures[i], {96 * gUiScale, 0})) {
                    loadFixture(app, fixtures[i]);
                }
            }
            ImGui::TreePop();
        }
    }

    ImGui::End();
}

// ---------------------------------------------------------------------------

static float gScroll = 0.0f;
// The mvp actually on screen this frame: wheel edits re-pick against it
// so they always hit the face under the cursor RIGHT NOW.
static Mat4 gScreenMvp;
static bool gScreenMvpValid = false;
static void scrollCb(GLFWwindow*, double, double dy) {
    gScroll += float(dy);
}

int main(int argc, char** argv) {
    gDataDir = userDataDir();
    gControlsPath = gDataDir + "/controls.ini";
    loadControls();
    gLogPath = gDataDir + "/weft_debug.log";
    gDebugLog = std::fopen(gLogPath.c_str(), "w");
    installCrashHandler();
    weft::setGenerateDebugLog(gDebugLog);
    logLine("weft_app start (built %s %s)", __DATE__, __TIME__);

    std::string screenshotPath, screenshotObjectsDir, startModel,
        startFixture = "demo";
    int screenshotObjectOnly = 0;  // 1-based; 0 = all solids
    int startSelect = 0, startMode = 0;
    bool startQuality = false, startMatcap = false, startSmooth = false;
    bool startProxy = false;
    float startYaw = 0.9f, startPitch = 0.5f;
    bool demoLoopCut = false, startStitch = false, startFinalize = false;
    std::vector<std::pair<int, std::string>> startFaceOverrides;  // FID:spec
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--screenshot" && i + 1 < argc) screenshotPath = argv[++i];
        else if (a == "--screenshot-objects" && i + 1 < argc)
            screenshotObjectsDir = argv[++i];
        else if (a == "--screenshot-object" && i + 1 < argc)
            screenshotObjectOnly = std::stoi(argv[++i]);
        else if (a == "--fixture" && i + 1 < argc) startFixture = argv[++i];
        else if (a == "--select" && i + 1 < argc) startSelect = std::stoi(argv[++i]);
        else if (a == "--yaw" && i + 1 < argc) startYaw = std::stof(argv[++i]);
        else if (a == "--pitch" && i + 1 < argc) startPitch = std::stof(argv[++i]);
        else if (a == "--loopcut") demoLoopCut = true;  // screenshot testing
        else if (a == "--stitch") startStitch = true;   // screenshot testing
        else if (a == "--finalize") startFinalize = true;  // visual QA = export mesh
        else if (a == "--quality") startQuality = true;
        else if (a == "--matcap") startMatcap = true;
        else if (a == "--smooth") startSmooth = true;
        else if (a == "--show-proxy") startProxy = true;
        else if (a == "--mode" && i + 1 < argc) startMode = std::stoi(argv[++i]);
        else if (a == "--faceradial" && i + 1 < argc) {  // FID:spec, screenshot testing
            std::string spec = argv[++i];
            size_t c = spec.find(':');
            if (c != std::string::npos)
                startFaceOverrides.emplace_back(std::stoi(spec.substr(0, c)),
                                                spec.substr(c + 1));
        }
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
    // Window/taskbar icon (embedded RGBA at several sizes; GLFW picks best).
    {
        GLFWimage icons[4] = {
            {16, 16, const_cast<unsigned char*>(weft_icon_16)},
            {32, 32, const_cast<unsigned char*>(weft_icon_32)},
            {48, 48, const_cast<unsigned char*>(weft_icon_48)},
            {64, 64, const_cast<unsigned char*>(weft_icon_64)},
        };
        glfwSetWindowIcon(window, 4, icons);
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    loadGl();

    // Settings-panel logo badge: upload the embedded RGBA as a GL texture.
    glGenTextures(1, &gLogoBadgeTex);
    glBindTexture(GL_TEXTURE_2D, gLogoBadgeTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, weft_badge_w, weft_badge_h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, weft_badge);
    glBindTexture(GL_TEXTURE_2D, 0);

    glfwSetScrollCallback(window, scrollCb);  // ImGui chains it

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // IMGUI_CHECKVERSION is compiled out in Release: a build tree
    // mixing stale non-docking ImGui objects with docking headers
    // crashes deep in the dock code on the first undock. Fail loud.
    if (!ImGui::DebugCheckVersionAndDataLayout(
            IMGUI_VERSION, sizeof(ImGuiIO), sizeof(ImGuiStyle),
            sizeof(ImVec2), sizeof(ImVec4), sizeof(ImDrawVert),
            sizeof(ImDrawIdx))) {
        std::fprintf(stderr,
                     "fatal: Dear ImGui header/library mismatch - "
                     "delete the build _deps directory and rebuild\n");
        return 1;
    }
    static std::string iniPath = gDataDir + "/imgui.ini";
    ImGui::GetIO().IniFilename = iniPath.c_str();
#ifdef IMGUI_HAS_DOCK
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    // First run (or first run since the panels split): lay Settings out
    // on the left and the Outliner on the right; afterwards the user's
    // own docking arrangement persists in imgui.ini.
    bool needDockLayout = true;
    {
        std::ifstream ini(iniPath);
        std::string text((std::istreambuf_iterator<char>(ini)),
                         std::istreambuf_iterator<char>());
        if (text.find("[Window][Settings]") != std::string::npos) {
            needDockLayout = false;
        }
    }
#endif
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
    GLuint proxyProg = makeProgram(kProxyVS, kProxyFS);

    App app;
    app.bakeQueue.start();
    // Model/analysis rebound on each successful load.
    app.livePath = gDataDir + "/weft_live.obj";
    bool startupLoadPending = !startModel.empty();
    // Fixture loads are async too: face overrides / select / proxy must
    // wait until finishLoadModel lands, same as a path argument.
    bool startupApplyPending =
        !startFaceOverrides.empty() || startSelect > 0 || startProxy;
    if (startupLoadPending) {
        // Skip auto-generate: overrides apply first, then startGenerate.
        loadModel(app, startModel, false);
    } else if (startupApplyPending) {
        // Same for fixtures that need select / density overrides / proxy.
        std::string path =
            tempDir() + "/weft_fixture_" + startFixture + ".step";
        try {
            weft::writeStep(weft::makeFixture(startFixture), path);
            loadModel(app, path, false);
            app.status = "fixture: " + startFixture;
        } catch (const std::exception& e) {
            app.status = std::string("fixture failed: ") + e.what();
        }
    } else {
        loadFixture(app, startFixture);
    }
    if (startFinalize || !screenshotObjectsDir.empty())
        app.forceFinalize = true;
    if ((startStitch || startFinalize || !screenshotObjectsDir.empty()) &&
        app.hasModel) {
        // After the load (which resets the recipe): apply screenshot
        // experiment flags and rebuild synchronously so the capture shows
        // the intended mesh (finalize = production export path).
        if (startStitch) app.recipe.settings.decoupleSeams = true;
        regenerate(app);
    }
    app.cam.yaw = startYaw;
    app.cam.pitch = startPitch;
    if (startQuality) {
        app.qualityView = true;
        if (app.hasModel) rebuildBuffers(app);
    }
    if (startMatcap) app.lightStyle = 1;
    if (startSmooth) {
        app.smoothShade = true;
        if (app.hasModel) rebuildBuffers(app);  // normals into the fill
    }
    if (startMode >= 1 && startMode <= 6) {
        setSelectMode(app, SelectMode(startMode - 1));
    }
    // Face overrides / select for an already-resident model (rare). The
    // common async-load path applies them in the main loop once hasModel.
    if (!startupApplyPending) {
        // nothing
    } else if (app.hasModel && !app.loadBusy) {
        for (const auto& [fid, spec] : startFaceOverrides) {
            weft::FaceMeshSettings s = app.recipe.settings.defaults;
            weft::applySettingsList(s, spec);
            app.recipe.settings.perFace[fid] = s;
        }
        if (startSelect > 0 && startSelect <= app.model.faceCount()) {
            app.selFaces = {startSelect};
            app.activeFace = startSelect;
            frameModel(app);
        }
        if (startProxy && app.activeFace > 0) app.gpuProxyPending = true;
        regenerate(app);
        rebuildBuffers(app);
        startupApplyPending = false;
    }

    double lastX = 0, lastY = 0;  // window-space cursor for orbit/pan feel
    bool navOrbit = false, navPan = false, navZoom = false, navSnap = false;
    bool navFromLmb = false;  // laptop scheme: this nav drag rode alt+LMB
    double downX = 0, downY = 0, downRX = 0, downRY = 0;      // framebuffer
    double downXWin = 0, downYWin = 0, downRXWin = 0, downRYWin = 0;  // window
    bool prevLmb = false, prevRmb = false;
    double lastClickTime = 0;  // double-click select-similar
    int lastClickFace = 0;
    double hoverX = -1, hoverY = -1;  // last hover-picked cursor (framebuffer)
    int frame = 0;
    size_t objectShotIndex =
        screenshotObjectOnly > 0 ? size_t(screenshotObjectOnly - 1) : 0;
    const size_t objectShotEnd =
        screenshotObjectOnly > 0 ? objectShotIndex + 1 : size_t(-1);
    int objectShotView = 0;
    int objectShotSettle = 0;
    bool objectShotArmed = !screenshotObjectsDir.empty();
    const float objectShotViews[2][2] = {{0.9f, 0.5f}, {2.4f, 0.35f}};
    if (objectShotArmed) {
        std::error_code ec;
        std::filesystem::create_directories(screenshotObjectsDir, ec);
        // Solid + mesh wire, fully deselected (no orange overlay / verts).
        app.showFill = true;
        app.showWire = true;
        app.showVerts = false;
        app.showBrepEdges = false;
        app.showProblems = false;
        app.lightStyle = 0;  // lit solid
        // Dark wires read on shaded fill (same as UI default on dark theme).
        app.wireColor[0] = 0.10f;
        app.wireColor[1] = 0.11f;
        app.wireColor[2] = 0.13f;
    }

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

        int fbw, fbh, winW, winH;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        glfwGetWindowSize(window, &winW, &winH);
        // GLFW cursor position is in window coordinates; glReadPixels /
        // projectPoint / the pick buffer use framebuffer pixels. On HiDPI
        // Linux (and macOS) these differ by the content scale — mixing them
        // offsets selection from the cursor. Keep both spaces: mx/my for
        // 3D picking, mxWin/myWin for ImGui overlays and orbit feel.
        double mxWin = 0, myWin = 0;
        glfwGetCursorPos(window, &mxWin, &myWin);
        const double fbSX = winW > 0 ? double(fbw) / double(winW) : 1.0;
        const double fbSY = winH > 0 ? double(fbh) / double(winH) : 1.0;
        double mx = mxWin * fbSX;
        double my = myWin * fbSY;
        if (demoLoopCut) {  // scripted screenshots: cursor at viewport center
            app.mode = Mode::LoopCut;
            mx = fbw * 0.42;
            my = fbh * 0.5;
            mxWin = fbSX > 0 ? mx / fbSX : mx;
            myWin = fbSY > 0 ? my / fbSY : my;
        }

        // Blender-standard navigation: MMB orbit, shift+MMB pan, ctrl+MMB
        // drag-zoom, wheel zoom; alt+MMB orbits and snaps to the nearest
        // axis-aligned view on release. Laptop scheme: alt+LMB emulates
        // MMB; once such a drag starts it stays navigation until the
        // button lifts, even if alt lifts first — otherwise the tail of
        // an orbit would turn into a box select.
        if (!io.WantCaptureMouse) {
            bool realMmb =
                glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) ==
                GLFW_PRESS;
            bool lmbBtn = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) ==
                          GLFW_PRESS;
            bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                         glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
            bool ctrl = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                        glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
            bool alt = glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
            bool navActive = navOrbit || navPan || navZoom;
            bool mmb = realMmb ||
                       (gLaptopControls &&
                        ((alt && lmbBtn && !navActive) ||
                         (navFromLmb && lmbBtn)));
            if (mmb && !navActive) {
                navPan = shift;
                navZoom = ctrl && !shift;
                navOrbit = !navPan && !navZoom;
                navSnap = alt && realMmb;  // axis snap needs a real MMB
                navFromLmb = !realMmb;
                lastX = mxWin;
                lastY = myWin;
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
                navFromLmb = false;
            }
            if (navOrbit) {
                app.cam.yaw -= float(mxWin - lastX) * 0.008f;
                app.cam.pitch += float(myWin - lastY) * 0.008f;
                app.cam.pitch = std::clamp(app.cam.pitch, -1.55f, 1.55f);
            }
            if (navPan) {
                float k = app.cam.dist * 0.0016f;
                Vec3 eye = app.cam.eye();
                Vec3 f = norm(sub(app.cam.target, eye));
                Vec3 r = norm(cross(f, {0, 0, 1}));
                Vec3 u = cross(r, f);
                app.cam.target.x -=
                    (float(mxWin - lastX) * r.x - float(myWin - lastY) * u.x) *
                    k;
                app.cam.target.y -=
                    (float(mxWin - lastX) * r.y - float(myWin - lastY) * u.y) *
                    k;
                app.cam.target.z -=
                    (float(mxWin - lastX) * r.z - float(myWin - lastY) * u.z) *
                    k;
            }
            if (navZoom) {
                app.cam.dist *= std::pow(1.006f, float(myWin - lastY));
                app.cam.dist = std::clamp(app.cam.dist, 0.5f, 10000.0f);
            }
            lastX = mxWin;
            lastY = myWin;
            if (gScroll != 0.0f) {
                // Modal density: shift+wheel drives the primary axis
                // (radial/grid-u), ctrl+wheel the secondary (axial/grid-v),
                // on the whole selection — Blender-style. Plain wheel zooms.
                int steps = int(gScroll > 0 ? std::ceil(gScroll)
                                            : std::floor(gScroll));
                if (app.hasModel && shift && app.mode == Mode::Bridge &&
                    !app.recipe.ops.empty()) {
                    // shift+wheel while bridging: flip WHICH side [ ]
                    // edits on the most recent bridge. Each side keeps
                    // its own twist (they counter-rotate), so flipping
                    // changes nothing until the wheel turns again.
                    for (auto op = app.recipe.ops.rbegin();
                         op != app.recipe.ops.rend(); ++op) {
                        if (op->kind != weft::ManualOp::Kind::Bridge) {
                            continue;
                        }
                        op->twistSide = op->twistSide ? 0 : 1;
                        std::snprintf(app.hudText, sizeof app.hudText,
                                      "twist side: %s (%+d)",
                                      op->twistSide ? "A" : "B",
                                      op->twistSide ? op->twistA
                                                    : op->twist);
                        app.hudUntil = glfwGetTime() + 0.9;
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
                    // ctrl+shift+wheel: fillet support loops. Flag the
                    // nudge when the active face's mesher ignores them
                    // (coons/planar/rev-grid fillet meshers read the value).
                    editSelected(app, [&](weft::FaceMeshSettings& s) {
                        s.filletLoops = std::max(1, s.filletLoops + steps);
                    });
                    const weft::MesherKind k =
                        effectiveKind(app, app.activeFace);
                    const bool used =
                        app.activeFace > 0 &&
                        app.activeFace <= int(app.analysis.faces.size()) &&
                        app.analysis.faces[app.activeFace - 1].isFillet &&
                        (k == weft::MesherKind::CoonsGrid ||
                         k == weft::MesherKind::PlanarGrid ||
                         k == weft::MesherKind::RevolutionGrid);
                    std::snprintf(app.hudText, sizeof app.hudText,
                                  "fillet loops: %d%s",
                                  app.recipe.settings.forFace(app.activeFace)
                                      .filletLoops,
                                  used ? "" : " (no effect here)");
                    app.hudUntil = glfwGetTime() + 0.9;
                } else if (app.hasModel && (shift || ctrl) &&
                           (!app.selFaces.empty())) {
                    adjustFaceDensity(app, /*secondary=*/ctrl, steps);
                } else if (app.hasModel && (shift || ctrl)) {
                    // No selection: edit the hovered face (or, over
                    // empty space, the globals) — scroll IS the editor.
                    // Re-pick at scroll time: the idle-frame hover can
                    // lag a mouse-off by a beat, and a stale target
                    // kept adjusting the OLD face.
                    if (gScreenMvpValid &&
                        (app.selectMode == SelectMode::Face ||
                         app.selectMode == SelectMode::Object)) {
                        glViewport(0, 0, fbw, fbh);
                        app.hoverFace =
                            pickFace(app, flatProg, gScreenMvp, int(mx),
                                     int(my), fbw, fbh);
                    }
                    adjustHovered(app, ctrl, shift, steps);
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
                    weft::PolyMesh probe = app.mesh;
                    if (weft::fillLoop(probe, app.model, op) == 0) {
                        app.status = "fill failed: no open boundary there";
                    } else {
                        app.recipe.ops.push_back(op);
                        markDirty(app);
                        app.status = "boundary filled (ctrl+Z undoes)";
                    }
                } else if (app.selectMode == SelectMode::Edge &&
                           !app.selEdges.empty() && !app.bLoops.empty()) {
                    // Fill from edge mode: cap the open loop nearest each
                    // selected edge. Probe first so unsupported fills do
                    // not corrupt the recipe.
                    int filled = 0;
                    for (int eid : app.selEdges) {
                        weft::ManualOp op;
                        op.kind = weft::ManualOp::Kind::FillLoop;
                        op.edgeA = eid;
                        weft::PolyMesh probe = app.mesh;
                        if (weft::fillLoop(probe, app.model, op) == 0) {
                            continue;
                        }
                        app.recipe.ops.push_back(op);
                        ++filled;
                    }
                    if (filled == 0) {
                        app.status = "fill failed: no open boundary near "
                                     "selected edge(s)";
                    } else {
                        markDirty(app);
                        app.status = "boundary filled near selected edge(s) "
                                     "(ctrl+Z undoes)";
                    }
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
                    if (app.bLoops.empty()) {
                        app.status = "bridge needs open boundaries "
                                     "(X deletes faces first)";
                    } else {
                        weft::PolyMesh probe = app.mesh;
                        if (weft::bridgeLoops(probe, app.model, op) == 0) {
                            app.status = "bridge failed: loops not "
                                         "bridgeable";
                        } else {
                            app.recipe.ops.push_back(op);
                            markDirty(app);
                            app.status = "bridge committed (ctrl+Z undoes)";
                        }
                    }
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
                    app.selVertOrder.clear();
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
                    app.pieCenter[0] = float(mxWin);
                    app.pieCenter[1] = float(myWin);
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
                            int& tw = op->twistSide ? op->twistA : op->twist;
                            tw += delta;
                            std::snprintf(app.hudText, sizeof app.hudText,
                                          "bridge twist %s: %+d",
                                          op->twistSide ? "A" : "B", tw);
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
                if (app.selectMode == SelectMode::Vert) {
                    if (app.selVerts.size() >= 2) {
                        // Deferred: this input block runs BEFORE
                        // NewFrame, where popup calls dereference a
                        // null current window (the right-click and
                        // shading-button crashes rode on that).
                        app.openWeldPopup = true;
                    } else {
                        app.status = "weld: select 2+ verts first (M opens "
                                     "the merge menu)";
                    }
                } else {
                    bool next = !activeSettings(app).minimal;
                    editSelected(app, [&](weft::FaceMeshSettings& s) {
                        s.minimal = next;
                    });
                }
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
                        weft::PolyMesh probe = app.mesh;
                        if (weft::dissolveLoop(probe, app.model, op) == 0) {
                            continue;
                        }
                        app.recipe.ops.push_back(op);
                        ++nLoops;
                    }
                    app.selMeshEdges.clear();
                    if (nLoops == 0) {
                        app.status = "dissolve failed: no edge loop there";
                    } else {
                        markDirty(app);
                        app.status = std::to_string(nLoops) +
                                     " loop(s) dissolved (ctrl+Z undoes)";
                    }
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
                weft::PolyMesh probe = app.mesh;
                if (weft::bridgeLoops(probe, app.model, op) == 0) {
                    app.status = "bridge failed: loops not bridgeable";
                } else {
                    app.recipe.ops.push_back(op);
                    markDirty(app);
                    app.status = "bridged ([ ] twist, shift+[ ] spans, "
                                 "ctrl+Z undoes)";
                }
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
            if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_E, false) &&
                app.hasModel) {
                app.openExportPopup = true;  // dialog drawn in-frame
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

        // Laptop scheme: alt+LMB belongs to navigation, and a nav drag
        // that started on LMB keeps owning the button until release.
        bool navChord =
            gLaptopControls &&
            (navFromLmb ||
             glfwGetKey(window, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
             glfwGetKey(window, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS);
        bool lmb = !io.WantCaptureMouse && !navChord &&
                   glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) ==
                       GLFW_PRESS;
        bool lmbPressed = lmb && !prevLmb;
        // Gesture thresholds stay in window pixels so click-vs-drag feel is
        // stable across DPI; downX/Y stay in framebuffer pixels for picking.
        if (lmbPressed) {
            downX = mx;
            downY = my;
            downXWin = mxWin;
            downYWin = myWin;
        }
        bool clicked = prevLmb && !lmb && std::abs(mxWin - downXWin) < 4 &&
                       std::abs(myWin - downYWin) < 4;
        // Box select: an LMB drag in idle rubber-bands in EVERY mode.
        bool boxDrag = lmb && app.mode == Mode::Idle && app.pieKind < 0 &&
                       (std::abs(mxWin - downXWin) > 6 ||
                        std::abs(myWin - downYWin) > 6);
        bool boxReleased = prevLmb && !lmb && !clicked &&
                           app.mode == Mode::Idle && app.pieKind < 0;
        prevLmb = lmb;
        bool rmb = !io.WantCaptureMouse &&
                   glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) ==
                       GLFW_PRESS;
        if (rmb && !prevRmb) {
            downRX = mx;
            downRY = my;
            downRXWin = mxWin;
            downRYWin = myWin;
        }
        bool rClicked = prevRmb && !rmb && std::abs(mxWin - downRXWin) < 4 &&
                        std::abs(myWin - downRYWin) < 4;
        prevRmb = rmb;
        gScroll = 0.0f;

        // STEP hot-reload: poll the source file's mtime (cheap) and reload
        // once it changes AND stops changing — exporters write in bursts,
        // and a half-written STEP must never be parsed.
        if (app.hasModel && !app.loadBusy && app.watchSource &&
            !app.sourcePath.empty()) {
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
        // Poll completed bakes every frame — camera never joins the worker.
        {
            auto done = app.bakeQueue.pollCompleted();
            for (auto& r : done) adoptBakeResult(app, r);
            app.genBusy = app.bakeQueue.busy();
            if (!done.empty() && startProxy && app.activeFace > 0) {
                app.gpuProxyPending = true;
            }
        }
        if (app.loadReady && !app.bakeQueue.busy() &&
            !app.bakeQueue.hasPending()) {
            finishLoadModel(app);
        }
        if ((startupLoadPending || startupApplyPending) && app.hasModel &&
            !app.loadBusy) {
            if (startStitch) app.recipe.settings.decoupleSeams = true;
            for (const auto& [fid, spec] : startFaceOverrides) {
                weft::FaceMeshSettings s = app.recipe.settings.defaults;
                weft::applySettingsList(s, spec);
                app.recipe.settings.perFace[fid] = s;
            }
            if (startSelect > 0 && startSelect <= app.model.faceCount()) {
                app.selFaces = {startSelect};
                app.activeFace = startSelect;
                frameModel(app);
            }
            if (startProxy && app.activeFace > 0) {
                app.gpuProxyPending = true;
            }
            startupLoadPending = false;
            startupApplyPending = false;
            startGenerate(app);
        }
        // Debounce continuous controls, then enqueue one latest-wins bake
        // for the active face (0 = model-wide defaults).
        const bool editingTopology =
            app.mutatedThisFrame || ImGui::IsAnyItemActive() ||
            (glfwGetTime() - app.lastMutationTime < 0.08);
        if (app.dirty && !app.bakeQueue.busy() && !app.loadBusy &&
            !app.loadReady && !editingTopology) {
            const uint32_t fid =
                app.activeFace > 0 ? uint32_t(app.activeFace) : 0u;
            app.dirty = false;
            enqueueBake(app, fid);
        }
        // If the worker is busy but newer edits arrived, keep them pending
        // via noteQueued; when the worker finishes, dirty flush above runs.
        if (app.dirty && app.bakeQueue.busy() && !editingTopology) {
            const uint32_t fid =
                app.activeFace > 0 ? uint32_t(app.activeFace) : 0u;
            enqueueBake(app, fid);
            app.dirty = false;
        }

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
        gScreenMvp = mvp;  // what the user is pointing at (scroll re-pick)
        gScreenMvpValid = true;

        // Vertex grab: G picks the interior vertex under the cursor and
        // drags it constrained to its CAD surface. Works from idle AND
        // from bridge/loop-cut (the natural next step after bridging is
        // fixing a vertex — G exits that mode instead of going dead).
        if (app.hasModel && app.mode != Mode::Grab && app.slideOp < 0 &&
            !io.WantCaptureKeyboard && !io.WantCaptureMouse &&
            ImGui::IsKeyPressed(ImGuiKey_G, false)) {
            app.bridgeFirstEdge = 0;
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
                    weft::PolyMesh probe = app.mesh;
                    if (weft::bridgeLoops(probe, app.model, op) == 0) {
                        app.status = "bridge failed: loops not bridgeable";
                    } else {
                        app.recipe.ops.push_back(op);
                        app.bridgeFirstEdge = 0;
                        markDirty(app);
                        app.status = "bridge committed (ctrl+z undoes)";
                    }
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
                if (!extend) app.selVertOrder.clear();
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
                            if (app.selVerts.insert(v).second) {
                                app.selVertOrder.push_back(v);
                            }
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
                if (!shift) app.selVertOrder.clear();
                if (hit >= 0) {
                    uint32_t h = uint32_t(hit);
                    if (shift && app.selVerts.count(h)) {
                        app.selVerts.erase(h);
                        app.selVertOrder.erase(
                            std::remove(app.selVertOrder.begin(),
                                        app.selVertOrder.end(), h),
                            app.selVertOrder.end());
                    } else {
                        if (app.selVerts.insert(h).second) {
                            app.selVertOrder.push_back(h);
                        }
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
#ifdef IMGUI_HAS_DOCK
        // Dockable panels: the viewport hosts a passthru dockspace so
        // Settings/Outliner dock to the sides (or float free) and the
        // 3D view stays interactive through the middle.
        {
            ImGuiID dockspace = ImGui::DockSpaceOverViewport(
                0, ImGui::GetMainViewport(),
                ImGuiDockNodeFlags_PassthruCentralNode);
            const ImGuiViewport* mainVp = ImGui::GetMainViewport();
            gViewMin = mainVp->WorkPos;
            gViewMax = {mainVp->WorkPos.x + mainVp->WorkSize.x,
                        mainVp->WorkPos.y + mainVp->WorkSize.y};
            if (ImGuiDockNode* central =
                    ImGui::DockBuilderGetCentralNode(dockspace)) {
                gViewMin = central->Pos;
                gViewMax = {central->Pos.x + central->Size.x,
                            central->Pos.y + central->Size.y};
            }
            if (needDockLayout) {
                needDockLayout = false;
                ImGui::DockBuilderRemoveNode(dockspace);
                ImGui::DockBuilderAddNode(
                    dockspace, ImGuiDockNodeFlags_PassthruCentralNode |
                                   ImGuiDockNodeFlags_DockSpace);
                ImGui::DockBuilderSetNodeSize(
                    dockspace, ImGui::GetMainViewport()->WorkSize);
                ImGuiID center = dockspace;
                ImGuiID right = ImGui::DockBuilderSplitNode(
                    center, ImGuiDir_Right, 0.20f, nullptr, &center);
                ImGuiID left = ImGui::DockBuilderSplitNode(
                    center, ImGuiDir_Left, 0.24f, nullptr, &center);
                ImGui::DockBuilderDockWindow("Settings", left);
                ImGui::DockBuilderDockWindow("Outliner", right);
                ImGui::DockBuilderFinish(dockspace);
            }
        }
#endif
        if (boxDrag) {
            ImGui::GetForegroundDrawList()->AddRect(
                {float(downXWin), float(downYWin)},
                {float(mxWin), float(myWin)},
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
            float dx = float(mxWin) - cx, dy = float(myWin) - cy;
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
                dl->AddLine({cx, cy}, {float(mxWin), float(myWin)},
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
            // projectPoint is in framebuffer pixels; ImGui draw lists use
            // window coordinates — convert before drawing on HiDPI.
            auto toUi = [&](float x, float y) -> ImVec2 {
                return {float(x / fbSX), float(y / fbSY)};
            };
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
                        ImVec2 p = toUi(sp[0], sp[1]);
                        dl->AddRectFilled({p.x - r, p.y - r},
                                          {p.x + r, p.y + r},
                                          IM_COL32(255, 196, 64, 255));
                    }
                }
                if (app.hoverVert >= 0 &&
                    app.hoverVert < int64_t(app.mesh.vertexCount())) {
                    float sp[3];
                    if (projV(uint32_t(app.hoverVert), sp)) {
                        ImVec2 p = toUi(sp[0], sp[1]);
                        dl->AddCircle(p, 6.0f * gUiScale,
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
                        dl->AddLine(toUi(sa[0], sa[1]), toUi(sb[0], sb[1]),
                                    col, w);
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
        drawOutliner(app);
        drawOverlay(app);
        drawGenProgress(app);
        drawFacePopup(app);
        drawWeldPopup(app);
        drawExportPopup(app);
        ImGui::Render();

        glViewport(0, 0, fbw, fbh);
        glClearColor(app.bgColor[0], app.bgColor[1], app.bgColor[2], 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);

        const bool wantFill = app.showFill;
        const bool wantWire = app.showWire;
        const bool flatFill = app.lightStyle == 2;
        const bool matcapFill = app.lightStyle == 1;
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
                glUniform1i(glGetUniformLocation(prog, "uMode"),
                            matcapFill ? 1 : 0);
                glUniform1i(glGetUniformLocation(prog, "uSmooth"),
                            app.smoothShade ? 1 : 0);
            }
            glBindVertexArray(app.fill.vao);
            glDrawArrays(GL_TRIANGLES, 0, app.fill.count);
            // Knob-to-parts map: hovering a defaults control tints every
            // face whose mesher that control drives.
            if (!app.highlightMeshers.empty()) {
                glDepthFunc(GL_LEQUAL);
                glUseProgram(flatProg);
                glUniformMatrix4fv(glGetUniformLocation(flatProg, "uMVP"), 1,
                                   GL_FALSE, mvp.m);
                glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.45f);
                float hk[3] = {0.35f, 0.85f, 1.0f};
                glUniform3fv(glGetUniformLocation(flatProg, "uColor"), 1, hk);
                glBindVertexArray(app.fill.vao);
                for (const auto& seg : app.fillSegs) {
                    if (seg[0] <= 0) continue;
                    bool hit = app.highlightMeshers.count(-1) > 0;
                    if (!hit && app.highlightMeshers.count(-2) &&
                        seg[0] <= int(app.analysis.faces.size())) {
                        hit = app.analysis.faces[seg[0] - 1].isFillet;
                    }
                    if (!hit) {
                        auto it = app.report.faceMesher.find(seg[0]);
                        hit = it != app.report.faceMesher.end() &&
                              app.highlightMeshers.count(int(it->second));
                    }
                    if (hit) glDrawArrays(GL_TRIANGLES, seg[1], seg[2]);
                }
                glUniform1f(glGetUniformLocation(flatProg, "uMix"), 0.0f);
                glDepthFunc(GL_LESS);
                glUseProgram(prog);
            }
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
        if (app.hasModel && app.gpuProxyPending && app.gpuProxy.count > 0 &&
            app.gpuProxyFace == app.activeFace) {
            const std::array<float, 2> count = gpuProxyCounts(app);
            const float grid[3] = {count[0], count[1], 0.88f};
            glUseProgram(proxyProg);
            glUniformMatrix4fv(glGetUniformLocation(proxyProg, "uMVP"), 1,
                               GL_FALSE, mvp.m);
            glUniform3fv(glGetUniformLocation(proxyProg, "uGrid"), 1, grid);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            glDepthFunc(GL_LEQUAL);
            glBindVertexArray(app.gpuProxy.vao);
            glDrawArrays(GL_TRIANGLES, 0, app.gpuProxy.count);
            glDepthFunc(GL_LESS);
            glDepthMask(GL_TRUE);
            glDisable(GL_BLEND);
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
        if (app.hasModel && app.selectMode == SelectMode::Vert &&
            app.allVerts.count) {
            // The whole cage: small dark points, depth-tested so only
            // front-facing verts show (the fill's polygon offset keeps
            // them from z-fighting their own surface).
            glPointSize(app.vertSizeInactive * gUiScale);
            glUniform1f(uMix, 1.0f);
            const float dark[3] = {0.03f, 0.03f, 0.04f};
            glUniform3fv(uColor, 1, dark);
            glBindVertexArray(app.allVerts.vao);
            glDrawArrays(GL_POINTS, 0, app.allVerts.count);
            glUniform1f(uMix, 0.0f);
        }
        if (app.hasModel &&
            (app.showVerts || app.selectMode == SelectMode::Vert) &&
            app.verts.count) {
            glDisable(GL_DEPTH_TEST);
            glPointSize(app.vertSizeActive * gUiScale);
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

        // Screenshots wait for the async mesh: an empty viewport is not
        // the model (automated visual checks depend on this). Read the
        // freshly-rendered BACK buffer before swapping: reading GL_BACK after
        // glfwSwapBuffers captures the previous frame, which was commonly the
        // "welding + conforming" progress card rather than the finished mesh.
                const bool meshReady = !app.loadBusy && !app.loadReady &&
            !(app.hasModel &&
              (app.bakeQueue.busy() || app.bakeQueue.hasPending() ||
               app.dirty));
        if (objectShotArmed && meshReady && app.hasModel &&
            !app.analysis.solidFaces.empty()) {
            if (objectShotSettle == 0) {
                if (objectShotIndex >= app.analysis.solidFaces.size() ||
                    objectShotIndex >= objectShotEnd) {
                    std::printf("screenshot-objects: done (%zu solids)\n",
                                app.analysis.solidFaces.size());
                    break;
                }
                if (!isolateSolidObject(app, objectShotIndex)) {
                    std::printf("screenshot-objects: skip %zu\n",
                                objectShotIndex + 1);
                    ++objectShotIndex;
                    objectShotView = 0;
                    continue;
                }
                app.cam.yaw = objectShotViews[objectShotView][0];
                app.cam.pitch = objectShotViews[objectShotView][1];
                frameModel(app);
                objectShotSettle = 1;
            } else if (++objectShotSettle >= 4) {
                if (!app.selFaces.empty() || app.activeFace != 0) {
                    app.selFaces.clear();
                    app.selEdges.clear();
                    app.activeFace = 0;
                    rebuildBuffers(app);
                }
                std::vector<unsigned char> px(size_t(fbw) * fbh * 3);
                glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE,
                             px.data());
                stbi_flip_vertically_on_write(1);
                const std::string stem =
                    solidObjectFileStem(app, objectShotIndex);
                const std::filesystem::path out =
                    std::filesystem::path(screenshotObjectsDir) /
                    (stem + "_v" + std::to_string(objectShotView) + ".png");
                stbi_write_png(out.string().c_str(), fbw, fbh, 3, px.data(),
                               fbw * 3);
                std::printf("wrote %s\n", out.string().c_str());
                objectShotSettle = 0;
                if (++objectShotView >= 2) {
                    objectShotView = 0;
                    ++objectShotIndex;
                }
            }
        } else if (!screenshotPath.empty() && ++frame >= 4 && meshReady) {
            std::vector<unsigned char> px(size_t(fbw) * fbh * 3);
            glReadPixels(0, 0, fbw, fbh, GL_RGB, GL_UNSIGNED_BYTE, px.data());
            stbi_flip_vertically_on_write(1);
            stbi_write_png(screenshotPath.c_str(), fbw, fbh, 3, px.data(),
                           fbw * 3);
            std::printf("wrote %s\n", screenshotPath.c_str());
            break;
        }
glfwSwapBuffers(window);
    }

    // A window close may arrive while either worker is active. Drain both so
    // their App pointer and OCCT objects remain alive through completion.
    if (app.loadThread.joinable()) app.loadThread.join();
    app.bakeQueue.stop();
    if (app.genThread.joinable()) app.genThread.join();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
