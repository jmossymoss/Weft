#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace weft::test {

inline std::uint64_t processId() noexcept {
#if defined(_WIN32)
    return static_cast<std::uint64_t>(_getpid());
#else
    return static_cast<std::uint64_t>(getpid());
#endif
}

inline std::filesystem::path uniqueTempPath(
    std::string_view stem, std::string_view extension = {}) {
    const auto nonce = std::chrono::high_resolution_clock::now()
                           .time_since_epoch()
                           .count();
    std::string filename(stem);
    filename += "_" + std::to_string(processId());
    filename += "_" + std::to_string(nonce);
    filename += extension;
    return std::filesystem::temp_directory_path() / filename;
}

}  // namespace weft::test
