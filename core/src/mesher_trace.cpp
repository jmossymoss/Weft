#include "mesher_trace.hpp"

#include "weft/meshers.hpp"

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace weft {
namespace {

std::atomic<std::FILE*> gDebugLog{nullptr};
std::mutex gDebugMutex;

}  // namespace

void setGenerateDebugLog(std::FILE* file) {
    gDebugLog.store(file, std::memory_order_release);
}

namespace mesher_detail {

void dbg(const char* format, ...) {
    if (!gDebugLog.load(std::memory_order_acquire)) return;
    std::lock_guard<std::mutex> lock(gDebugMutex);
    std::FILE* log = gDebugLog.load(std::memory_order_acquire);
    if (!log) return;

    va_list args;
    va_start(args, format);
    std::fprintf(log, "[core] ");
    std::vfprintf(log, format, args);
    std::fputc('\n', log);
    std::fflush(log);
    va_end(args);
}

}  // namespace mesher_detail
}  // namespace weft
