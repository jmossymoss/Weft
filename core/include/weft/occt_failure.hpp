#pragma once

#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>

#include <string>

namespace weft {

// Prefer GetMessageString() across OCCT 7.6–7.9. Some 7.9 builds temporarily
// lack std::exception::what() on Standard_Failure; Debian/Ubuntu 7.6 also
// exposes only GetMessageString(). OCCT 8 restores what() while deprecating
// GetMessageString(), so use that path there.
inline std::string occtFailureMessage(const Standard_Failure& failure) {
#if OCC_VERSION_HEX >= 0x080000
    const char* const message = failure.what();
#else
    const char* const message = failure.GetMessageString();
#endif
    return message == nullptr || message[0] == '\0'
        ? "OCCT raised an unnamed failure"
        : std::string(message);
}

}  // namespace weft
