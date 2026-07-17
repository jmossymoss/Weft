#pragma once

#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>

#include <string>

namespace weft {

// OCCT 7.9 temporarily exposes Standard_Failure through its historical
// message API without std::exception::what(). OCCT 8 and older supported
// releases expose what(), where GetMessageString() is deprecated. Keep that
// version seam in one place so geometry diagnostics are portable and never
// lose a named fallback.
inline std::string occtFailureMessage(const Standard_Failure& failure) {
#if OCC_VERSION_MAJOR == 7 && OCC_VERSION_MINOR >= 9
    const char* const message = failure.GetMessageString();
#else
    const char* const message = failure.what();
#endif
    return message == nullptr || message[0] == '\0'
        ? "OCCT raised an unnamed failure"
        : std::string(message);
}

}  // namespace weft
