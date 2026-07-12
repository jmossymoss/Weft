#pragma once

namespace weft::mesher_detail {

// Internal stage-by-stage trace used throughout the mesher implementation.
// The public setGenerateDebugLog() function owns destination selection.
void dbg(const char* format, ...);

}  // namespace weft::mesher_detail
