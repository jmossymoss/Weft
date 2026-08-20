# OCCT Windows runtime DLLs (vendored)

These DLLs are the OpenCASCADE third-party runtimes that `TK*.dll`
imports at process start. They are copied next to `weft.exe` /
`weft_app.exe` by `build.bat` and CMake post-build — no discovery from
the OCCT install is required.

## Source

Extracted from the official OCCT 8.0.1 Windows 3rd-party package:

- https://github.com/Open-Cascade-SAS/OCCT/releases/download/V8.0.1/3rdparty-vc14-64.zip  
  (same release family as `opencascade-with-debug-pch.zip`)

Only the runtime `.dll` files Weft needs are kept here (not headers/libs).

## Contents

| DLL | Upstream product |
|-----|------------------|
| `tbb12.dll`, `tbb12_debug.dll`, `tbbmalloc*.dll` | oneTBB 2021.13.0 |
| `jemalloc.dll`, `jemalloc_debug.dll` | jemalloc (VC14) |
| `FreeImage.dll` | FreeImage 3.18.0 |
| `openvr_api.dll` | OpenVR 1.14.15 |
| `freetype.dll` | FreeType 2.13.3 |
| `zlib1.dll`, `zlib.dll` | zlib |
| `avcodec-57.dll`, `avformat-57.dll`, `avutil-55.dll`, … | FFmpeg 3.3.4 |

`jemalloc_debug.dll` is the debug build of jemalloc renamed from the
package’s `debug/bin/jemalloc.dll` so both variants can sit in one folder.

## Licenses

Each product keeps its own license (Apache-2.0 for oneTBB, BSD-style for
jemalloc/OpenVR, FreeImage license, FreeType FTL/GPL, LGPL/GPL for
FFmpeg, zlib license). Redistributed here solely so Weft Windows builds
run out of the box against a relocated OCCT install.