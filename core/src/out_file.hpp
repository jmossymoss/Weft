#pragma once

// Write-checked stdio output for the exporters. stdio only reports a failed
// write through the stream's sticky error flag (and through fclose, which is
// where buffered bytes actually reach the disk), so an exporter that ignores
// both silently produces a truncated file and reports success. finish()
// converts either failure into an exception, and the destructor removes a
// partial file so a half-written export cannot be mistaken for a good one.

#include <cstdio>
#include <stdexcept>
#include <string>

namespace weft::detail {

class OutFile {
public:
    OutFile(const std::string& path, const char* mode)
        : m_path(path), m_file(std::fopen(path.c_str(), mode)) {
        if (!m_file) throw std::runtime_error("cannot open for writing: " + path);
    }
    OutFile(const OutFile&) = delete;
    OutFile& operator=(const OutFile&) = delete;

    // Unwinding out of a write (or a caller that forgot finish()) leaves a
    // partial file behind: drop it rather than publish it.
    ~OutFile() {
        if (!m_file) return;
        std::fclose(m_file);
        std::remove(m_path.c_str());
    }

    FILE* get() const { return m_file; }

    void finish() {
        if (!m_file) return;
        FILE* f = m_file;
        m_file = nullptr;
        const bool writeFailed = std::ferror(f) != 0;
        const bool closeFailed = std::fclose(f) != 0;
        if (writeFailed || closeFailed) {
            std::remove(m_path.c_str());
            throw std::runtime_error("failed to write (disk full or I/O error): " +
                                     m_path);
        }
    }

private:
    std::string m_path;
    FILE* m_file = nullptr;
};

}  // namespace weft::detail
