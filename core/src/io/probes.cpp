#include "weft/io/system.hpp"

#include <cstdint>
#include <regex>
#include <string_view>

namespace weft::io {

namespace {

bool rsearch(std::string_view sv, const std::regex& re) {
    if (sv.empty()) return false;
    return std::regex_search(sv.data(), sv.data() + sv.size(), re);
}

// 1) STEP — ISO 10303-21 header, anchored at buffer start.
Format probeFormat_STEP(const System::FormatProbeInput& in) {
    static const std::regex re(R"(^\s*ISO-10303-21\s*;\s*HEADER)");
    return rsearch(in.contentsBegin, re) ? Format::Step : Format::Unknown;
}

// 2) IGES — the 73rd fixed column carries the 'S' (Start) section letter.
//    Needs the wide read window to reach column 73.
Format probeFormat_IGES(const System::FormatProbeInput& in) {
    static const std::regex re(R"(^.{72}S\s*[0-9]+\s*[\n\r\f])");
    return rsearch(in.contentsBegin, re) ? Format::Iges : Format::Unknown;
}

// 3) BREP — OpenCASCADE ASCII BRep dump.
Format probeFormat_BREP(const System::FormatProbeInput& in) {
    static const std::regex re(R"(^\s*DBRep_DrawableShape)");
    return rsearch(in.contentsBegin, re) ? Format::Brep : Format::Unknown;
}

// 4) STL — binary (facet count at byte 80 must size the file exactly) OR ascii.
Format probeFormat_STL(const System::FormatProbeInput& in) {
    std::string_view b = in.contentsBegin;
    if (b.size() >= 84 && in.hintFullSize >= 84) {
        const auto* p = reinterpret_cast<const unsigned char*>(b.data()) + 80;
        const uint32_t count = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                               (static_cast<uint32_t>(p[2]) << 16) |
                               (static_cast<uint32_t>(p[3]) << 24);
        if (static_cast<uint64_t>(50) * count + 84 == in.hintFullSize) return Format::Stl;
    }
    static const std::regex re(R"(^\s*solid\s+)");
    return rsearch(b, re) ? Format::Stl : Format::Unknown;
}

// 5) glTF — .glb magic, or a JSON document with an asset/version block.
Format probeFormat_GLTF(const System::FormatProbeInput& in) {
    std::string_view b = in.contentsBegin;
    if (b.size() >= 4 && b.substr(0, 4) == "glTF") return Format::Gltf;
    if (b.find('{') != std::string_view::npos && b.find("asset") != std::string_view::npos &&
        b.find("\"version\"") != std::string_view::npos)
        return Format::Gltf;
    return Format::Unknown;
}

// 6) PLY — the format directive right after the ply magic.
Format probeFormat_PLY(const System::FormatProbeInput& in) {
    static const std::regex re(
        R"(^\s*ply\s+format\s+(ascii|binary_little_endian|binary_big_endian)\s+)");
    return rsearch(in.contentsBegin, re) ? Format::Ply : Format::Unknown;
}

// 7) OBJ — a vertex/face directive anywhere; deliberately registered last
//    because the pattern is permissive.
Format probeFormat_OBJ(const System::FormatProbeInput& in) {
    static const std::regex re(R"([^\n]\s*(v|vt|vn|vp|f)\s+[-\+]?[0-9\.]+\s)");
    return rsearch(in.contentsBegin, re) ? Format::Obj : Format::Unknown;
}

}  // namespace

void addPredefinedFormatProbes(System& sys) {
    // Order matters: stricter/anchored recognizers first, permissive OBJ last.
    sys.addFormatProbe(&probeFormat_STEP);
    sys.addFormatProbe(&probeFormat_IGES);
    sys.addFormatProbe(&probeFormat_BREP);
    sys.addFormatProbe(&probeFormat_STL);
    sys.addFormatProbe(&probeFormat_GLTF);
    sys.addFormatProbe(&probeFormat_PLY);
    sys.addFormatProbe(&probeFormat_OBJ);
}

}  // namespace weft::io
