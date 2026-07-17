#pragma once

#include <Standard_Handle.hxx>
#include <Standard_Version.hxx>

// OCCT 8 exposes handles and free down-casts through `occ`. OCCT 7.9 keeps
// the same intrusive handle implementation under `opencascade` and exposes
// down-casting as a static member. Keep the fixture source version-neutral so
// its geometry and evidence contracts remain identical in both CI lanes.
#if OCC_VERSION_HEX < 0x080000
namespace occ {

template <typename T>
using handle = opencascade::handle<T>;

template <typename Target, typename Source>
handle<Target> down_cast(const handle<Source>& source) {
  return handle<Target>::DownCast(source);
}

} // namespace occ
#endif
