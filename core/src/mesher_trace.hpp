#pragma once

#include <string>
#include <vector>

namespace weft::mesher_detail {

// Internal stage-by-stage trace used throughout the mesher implementation.
// The public setGenerateDebugLog() function owns destination selection.
void dbg(const char* format, ...);

// Per-face capture of the same trace lines.
//
// A mesher that gives up returns false through one of ~420 bail points and the
// report keeps only a short cause string, so "why did THIS face bail?" used to
// mean adding temporary dbg() calls, rebuilding the core, re-running, reading
// the log, and stripping the instrumentation again — the single most repeated
// cost in this codebase. With capture on, every dbg() line emitted while a
// face context is active is also kept in a bounded per-face buffer that
// generate() attaches to the report, so a demotion explains itself with no
// rebuild.
//
// The current face is thread-local so concurrent face meshing stays correct;
// storage is shared and mutex-guarded.
void setFaceTraceEnabled(bool on);
bool faceTraceEnabled();
void beginFaceTrace(int faceId);
void endFaceTrace();
std::vector<std::string> takeFaceTrace(int faceId);
void clearFaceTraces();

// Scoped helper: begins on construction, ends on destruction.
class FaceTraceScope {
public:
    explicit FaceTraceScope(int faceId) { beginFaceTrace(faceId); }
    ~FaceTraceScope() { endFaceTrace(); }
    FaceTraceScope(const FaceTraceScope&) = delete;
    FaceTraceScope& operator=(const FaceTraceScope&) = delete;
};

}  // namespace weft::mesher_detail
