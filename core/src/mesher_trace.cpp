#include "mesher_trace.hpp"

#include "weft/meshers.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <mutex>

namespace weft {
namespace {

std::atomic<std::FILE*> gDebugLog{nullptr};
std::mutex gDebugMutex;

// Per-face trace capture. Bounded on both axes so a 900-face model with a
// chatty mesher cannot grow without limit.
constexpr size_t kMaxLinesPerFace = 24;
constexpr size_t kMaxTracedFaces = 4096;

std::atomic<bool> gCaptureEnabled{false};
std::mutex gTraceMutex;
std::map<int, std::vector<std::string>> gFaceTrace;
thread_local int tCurrentFace = 0;

}  // namespace

void setGenerateDebugLog(std::FILE* file) {
    gDebugLog.store(file, std::memory_order_release);
}

namespace mesher_detail {

void setFaceTraceEnabled(bool on) {
    gCaptureEnabled.store(on, std::memory_order_release);
}

bool faceTraceEnabled() {
    return gCaptureEnabled.load(std::memory_order_acquire);
}

void beginFaceTrace(int faceId) { tCurrentFace = faceId; }

void endFaceTrace() { tCurrentFace = 0; }

std::vector<std::string> takeFaceTrace(int faceId) {
    std::lock_guard<std::mutex> lock(gTraceMutex);
    auto it = gFaceTrace.find(faceId);
    if (it == gFaceTrace.end()) return {};
    std::vector<std::string> out = std::move(it->second);
    gFaceTrace.erase(it);
    return out;
}

void clearFaceTraces() {
    std::lock_guard<std::mutex> lock(gTraceMutex);
    gFaceTrace.clear();
}

void dbg(const char* format, ...) {
    std::FILE* log = gDebugLog.load(std::memory_order_acquire);
    const bool capture =
        gCaptureEnabled.load(std::memory_order_acquire) && tCurrentFace != 0;
    if (!log && !capture) return;

    char line[1024];
    {
        va_list args;
        va_start(args, format);
        std::vsnprintf(line, sizeof(line), format, args);
        va_end(args);
    }

    if (log) {
        std::lock_guard<std::mutex> lock(gDebugMutex);
        std::FILE* live = gDebugLog.load(std::memory_order_acquire);
        if (live) {
            std::fprintf(live, "[core] %s\n", line);
            std::fflush(live);
        }
    }
    if (capture) {
        std::lock_guard<std::mutex> lock(gTraceMutex);
        auto it = gFaceTrace.find(tCurrentFace);
        if (it == gFaceTrace.end()) {
            if (gFaceTrace.size() >= kMaxTracedFaces) return;
            it = gFaceTrace.emplace(tCurrentFace, std::vector<std::string>{})
                     .first;
        }
        if (it->second.size() < kMaxLinesPerFace) {
            it->second.emplace_back(line);
        }
    }
}

}  // namespace mesher_detail
}  // namespace weft
