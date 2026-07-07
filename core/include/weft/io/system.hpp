#pragma once

#include "weft/io/format.hpp"
#include "weft/io/reader.hpp"
#include "weft/io/writer.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace weft::io {

// The IO registry — ported from Mayo::IO::System. No globals, no macros; a
// bootstrap (bootstrapIo) builds the factories and hands them over.
class System {
public:
    struct FormatProbeInput {
        std::string filepath;
        std::string_view contentsBegin;  // first N bytes actually read
        uint64_t hintFullSize;           // true file size (binary-STL needs it)
    };
    using FormatProbe = std::function<Format(const FormatProbeInput&)>;

    void addFormatProbe(FormatProbe probe);             // appended in registration order
    Format probeFormat(const std::string& path) const;  // content first, extension fallback
    // Extension-only classification for OUTPUT paths (the content of a file we
    // are about to write does not exist yet).
    Format probeFormatForOutput(const std::string& path) const;

    void addFactoryReader(std::unique_ptr<FactoryReader>);  // unions formats() into readerFormats
    void addFactoryWriter(std::unique_ptr<FactoryWriter>);
    const FactoryReader* findFactoryReader(Format) const;  // first factory advertising it
    const FactoryWriter* findFactoryWriter(Format) const;
    std::unique_ptr<Reader> createReader(Format) const;  // find->create(); {} if none
    std::unique_ptr<Writer> createWriter(Format) const;

    std::span<const Format> readerFormats() const { return m_readerFormats; }
    std::span<const Format> writerFormats() const { return m_writerFormats; }

private:
    Format probeByExtension(const std::string& path) const;

    std::vector<FormatProbe> m_probes;
    std::vector<Format> m_readerFormats, m_writerFormats;
    std::vector<std::unique_ptr<FactoryReader>> m_factoryReaders;
    std::vector<std::unique_ptr<FactoryWriter>> m_factoryWriters;
};

// Registration bootstrap (bootstrap.cpp): registers the OCC B-rep reader
// factory, then addPredefinedFormatProbes(sys). Writers are added in Stage B.
void bootstrapIo(System& sys);
void addPredefinedFormatProbes(System& sys);

// Convenience one-shots used by the CLI/app call sites.
Model importFile(const System&, const std::string& path);  // probe->create->read->transfer
void exportFile(const System&, Format, const WriteInput&, const std::string& path,
                const ParamGroup* = nullptr);

// Default BRepMesh tessellation of Model::shape (for B-rep -> mesh convert,
// which has no retopo step). polygonFaceId carries the model's FaceId.
PolyMesh tessellate(const Model&);

}  // namespace weft::io
