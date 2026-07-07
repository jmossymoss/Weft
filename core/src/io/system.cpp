#include "weft/io/system.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace weft::io {

static uint64_t filepathFileSize(const std::string& path) {
    std::error_code ec;
    auto sz = std::filesystem::file_size(path, ec);
    return ec ? 0u : static_cast<uint64_t>(sz);
}

void System::addFormatProbe(FormatProbe probe) { m_probes.push_back(std::move(probe)); }

void System::addFactoryReader(std::unique_ptr<FactoryReader> f) {
    if (!f) return;
    for (Format fmt : f->formats())
        if (std::find(m_readerFormats.begin(), m_readerFormats.end(), fmt) == m_readerFormats.end())
            m_readerFormats.push_back(fmt);
    m_factoryReaders.push_back(std::move(f));
}

void System::addFactoryWriter(std::unique_ptr<FactoryWriter> f) {
    if (!f) return;
    for (Format fmt : f->formats())
        if (std::find(m_writerFormats.begin(), m_writerFormats.end(), fmt) == m_writerFormats.end())
            m_writerFormats.push_back(fmt);
    m_factoryWriters.push_back(std::move(f));
}

const FactoryReader* System::findFactoryReader(Format fmt) const {
    for (const auto& f : m_factoryReaders)
        for (Format x : f->formats())
            if (x == fmt) return f.get();
    return nullptr;
}

const FactoryWriter* System::findFactoryWriter(Format fmt) const {
    for (const auto& f : m_factoryWriters)
        for (Format x : f->formats())
            if (x == fmt) return f.get();
    return nullptr;
}

std::unique_ptr<Reader> System::createReader(Format fmt) const {
    const FactoryReader* f = findFactoryReader(fmt);
    return f ? f->create(fmt) : nullptr;
}

std::unique_ptr<Writer> System::createWriter(Format fmt) const {
    const FactoryWriter* f = findFactoryWriter(fmt);
    return f ? f->create(fmt) : nullptr;
}

Format System::probeByExtension(const std::string& path) const {
    std::string ext = std::filesystem::path(path).extension().string();
    if (!ext.empty() && ext.front() == '.') ext.erase(0, 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext.empty()) return Format::Unknown;
    auto scan = [&](std::span<const Format> formats) -> Format {
        for (Format f : formats)
            for (std::string_view s : formatFileSuffixes(f))
                if (s == ext) return f;
        return Format::Unknown;
    };
    if (Format f = scan(m_readerFormats); f != Format::Unknown) return f;
    return scan(m_writerFormats);
}

Format System::probeFormat(const std::string& path) const {
    std::array<char, 2048> buf{};
    std::ifstream file(path, std::ios::binary);  // BINARY: don't corrupt STL magic
    file.read(buf.data(), buf.size());
    const std::streamsize n = file.gcount();
    FormatProbeInput in{path,
                        std::string_view(buf.data(), static_cast<size_t>(n > 0 ? n : 0)),
                        filepathFileSize(path)};
    for (const FormatProbe& p : m_probes)  // registration order; first non-Unknown wins
        if (Format f = p(in); f != Format::Unknown) return f;
    return probeByExtension(path);  // content failed: fall back to the extension
}

Format System::probeFormatForOutput(const std::string& path) const {
    return probeByExtension(path);
}

Model importFile(const System& sys, const std::string& path) {
    Format f = sys.probeFormat(path);
    if (f == Format::Unknown) throw std::runtime_error("unrecognized file format: " + path);
    std::unique_ptr<Reader> r = sys.createReader(f);
    if (!r) throw std::runtime_error("no reader registered for " + path);
    if (!r->readFile(path)) throw std::runtime_error("failed to read file: " + path);
    return r->transfer();
}

void exportFile(const System& sys, Format fmt, const WriteInput& in, const std::string& path,
                const ParamGroup* params) {
    std::unique_ptr<Writer> w = sys.createWriter(fmt);
    if (!w) throw std::runtime_error("no writer registered for output: " + path);
    if (params) w->applyParams(*params);
    if (!w->transfer(in)) throw std::runtime_error("writer transfer failed: " + path);
    if (!w->writeFile(path)) throw std::runtime_error("failed to write file: " + path);
}

}  // namespace weft::io
