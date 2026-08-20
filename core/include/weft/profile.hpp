// Lightweight scoped execution profiler for generate() hot paths.
//
// Enable with WEFT_TIMINGS=1 (or WEFT_PROFILE=1). Prints elapsed ms to stderr
// when the scope exits. Nested scopes indent; total is wall time since the
// outermost GenerateProfileSession started.
//
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace weft {

using ProfileClock = std::chrono::high_resolution_clock;

inline bool profileTimingsEnabled() {
    static const bool on = [] {
        const char* a = std::getenv("WEFT_TIMINGS");
        const char* b = std::getenv("WEFT_PROFILE");
        return (a && a[0] && a[0] != '0') || (b && b[0] && b[0] != '0');
    }();
    return on;
}

// One session per generate() call — nested ScopedTimers report both
// self-elapsed and session-total.
class GenerateProfileSession {
public:
    GenerateProfileSession()
        : enabled_(profileTimingsEnabled()),
          begin_(ProfileClock::now()),
          depth_(0) {
        if (enabled_) active_ = this;
    }
    ~GenerateProfileSession() {
        if (enabled_ && active_ == this) active_ = nullptr;
    }

    GenerateProfileSession(const GenerateProfileSession&) = delete;
    GenerateProfileSession& operator=(const GenerateProfileSession&) = delete;

    bool enabled() const { return enabled_; }
    int& depth() { return depth_; }
    ProfileClock::time_point sessionStart() const { return begin_; }

    static GenerateProfileSession* active() { return active_; }

private:
    bool enabled_;
    ProfileClock::time_point begin_;
    int depth_;
    static inline GenerateProfileSession* active_ = nullptr;
};

// RAII block timer. Near-zero cost when WEFT_TIMINGS/WEFT_PROFILE is unset.
class ScopedTimer {
public:
    explicit ScopedTimer(const char* label)
        : label_(label),
          enabled_(profileTimingsEnabled()),
          session_(GenerateProfileSession::active()),
          start_(enabled_ ? ProfileClock::now()
                          : ProfileClock::time_point{}) {
        if (enabled_ && session_) ++session_->depth();
    }

    ~ScopedTimer() {
        if (!enabled_) return;
        const auto end = ProfileClock::now();
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            end - start_)
                            .count();
        long long totalMs = ms;
        int indent = 0;
        if (session_) {
            totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end - session_->sessionStart())
                          .count();
            indent = (std::max)(0, session_->depth() - 1);
            --session_->depth();
        }
        char pad[32];
        const int n = indent < 16 ? indent : 16;
        std::memset(pad, ' ', size_t(n));
        pad[n] = '\0';
        std::fprintf(stderr, "profile: %s%-28s %7lld ms  (session %7lld ms)\n",
                     pad, label_ ? label_ : "?",
                     static_cast<long long>(ms), totalMs);
        std::fflush(stderr);
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    const char* label_;
    bool enabled_;
    GenerateProfileSession* session_;
    ProfileClock::time_point start_;
};

}  // namespace weft

#define WEFT_PROFILE_CONCAT2(a, b) a##b
#define WEFT_PROFILE_CONCAT(a, b) WEFT_PROFILE_CONCAT2(a, b)
#define WEFT_PROFILE_SCOPE(label)                                            \
    ::weft::ScopedTimer WEFT_PROFILE_CONCAT(_weftProfileTimer_, __LINE__)(   \
        label)
