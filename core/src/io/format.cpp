#include "weft/io/format.hpp"

namespace weft::io {

std::string_view formatIdentifier(Format f) {
    switch (f) {
        case Format::Step: return "STEP";
        case Format::Iges: return "IGES";
        case Format::Brep: return "BREP";
        case Format::Stl: return "STL";
        case Format::Obj: return "OBJ";
        case Format::Gltf: return "GLTF";
        case Format::Ply: return "PLY";
        case Format::Unknown: break;
    }
    return "UNKNOWN";
}

std::string_view formatName(Format f) {
    switch (f) {
        case Format::Step: return "STEP (ISO 10303)";
        case Format::Iges: return "IGES (ANSI/US PRO)";
        case Format::Brep: return "OpenCASCADE BREP";
        case Format::Stl: return "STL (stereolithography)";
        case Format::Obj: return "Wavefront OBJ";
        case Format::Gltf: return "glTF (Khronos)";
        case Format::Ply: return "PLY (Stanford)";
        case Format::Unknown: break;
    }
    return "Unknown";
}

std::span<const std::string_view> formatFileSuffixes(Format f) {
    // Function-static tables: the returned span must outlive the call.
    static constexpr std::string_view kStep[] = {"step", "stp"};
    static constexpr std::string_view kIges[] = {"iges", "igs"};
    static constexpr std::string_view kBrep[] = {"brep", "brp"};
    static constexpr std::string_view kStl[] = {"stl"};
    static constexpr std::string_view kObj[] = {"obj"};
    static constexpr std::string_view kGltf[] = {"gltf", "glb"};
    static constexpr std::string_view kPly[] = {"ply"};
    switch (f) {
        case Format::Step: return kStep;
        case Format::Iges: return kIges;
        case Format::Brep: return kBrep;
        case Format::Stl: return kStl;
        case Format::Obj: return kObj;
        case Format::Gltf: return kGltf;
        case Format::Ply: return kPly;
        case Format::Unknown: break;
    }
    return {};
}

bool formatProvidesBRep(Format f) {
    return f == Format::Step || f == Format::Iges || f == Format::Brep;
}

bool formatProvidesMesh(Format f) {
    return f != Format::Unknown && !formatProvidesBRep(f);
}

}  // namespace weft::io
