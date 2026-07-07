#pragma once

#include <span>
#include <string_view>

namespace weft::io {

// The file formats the IO subsystem understands. This enum is the single
// source of truth; identity/name/suffix/classification are free functions
// that switch on it (Mayo's corrected model — Format is an enum, not a
// struct). Keep Unknown == 0 so a default-constructed Format is "unknown".
enum class Format {
    Unknown = 0,
    Step,
    Iges,
    Brep,  // B-rep formats  (formatProvidesBRep == true)
    Stl,
    Obj,
    Gltf,
    Ply  // mesh formats   (formatProvidesMesh == true)
};

// Metadata — all switch on the enum. Suffix arrays are function-static so the
// returned span outlives the call (never span a local temporary).
std::string_view formatIdentifier(Format);                 // "STEP"
std::string_view formatName(Format);                       // "STEP (ISO 10303)"
std::span<const std::string_view> formatFileSuffixes(Format);  // Step -> {step,stp}
bool formatProvidesBRep(Format);                           // Step|Iges|Brep
bool formatProvidesMesh(Format);                           // everything else != Unknown

}  // namespace weft::io
