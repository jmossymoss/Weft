# Building Weft on Windows

## Quick Start

1. **Run the batch script** (requires Administrator):
   ```cmd
   build.bat
   ```
   
   This will automatically:
   - Check for and install CMake (via Chocolatey)
   - Check for and install Git (via Chocolatey)
   - Check for and install Visual Studio Build Tools (via Chocolatey)
   - Locate OpenCASCADE (with installation instructions if missing)
   - Configure CMake and build the project
   - Run the test suite

2. **Install OpenCASCADE** (if the script prompts you):
   - Download from: https://www.opencascade.com/content/latest-release
   - Extract to `C:\OCCT` (or another location)
   - Set `OCCT_DIR` environment variable or tell CMake where it is
   - Re-run the script

3. **Try the demo** (this fork's production path is `--independent`):
   ```cmd
   independent.bat
   independent.bat cli mesh demo.step -o demo.obj --validate
   ```
   Or the raw CLI:
   ```cmd
   build\bin\Release\weft.exe fixture demo.step --shape demo
   build\bin\Release\weft.exe inspect demo.step
   build\bin\Release\weft.exe mesh demo.step -o demo.obj --independent --validate
   ```

## Fast iteration (`dev.bat`)

`build.bat` re-checks dependencies, reconfigures CMake, redeploys DLLs, and
runs the full test suite every time — thorough, but slow for a one-line
change. Once `build.bat` has succeeded at least once, use `dev.bat` instead:

```cmd
dev.bat            REM build + run weft_app (incremental, no other checks)
dev.bat tests      REM build + run weft_tests
dev.bat cli mesh demo.step -o demo.obj --radial 12
independent.bat    REM GUI on independent tessellation
independent.bat cli mesh demo.step -o demo.obj --validate
dev.bat build      REM compile everything, run nothing
dev.bat -d app     REM Debug config: compiles faster, runs slower
```

Each of these is a plain `cmake --build --target <x>` under the hood, so
only what changed gets recompiled — normally a few seconds, not minutes.

The fastest loop of all is opening `build\weft.sln` directly in Visual
Studio: set `weft_app` as the startup project and hit F5/Ctrl+F5 for
incremental builds with breakpoints and edit-and-continue.

## Manual Build (if batch fails)

If `build.bat` encounters issues, build manually:

```cmd
REM Configure (adjust OCCT_DIR as needed)
cmake -B build -DOCCT_SEARCH_PATH=C:\OCCT

REM Build
cmake --build build --config Release -j

REM Independent-mesh tests (this fork's Windows gate)
ctest --test-dir build -C Release --output-on-failure -R "independent_mesh|bake_queue|topology_cache|geometry_pool"
```

Full `ctest -R pipeline` still exercises legacy `weft::generate()`. Those
asserts fail on Windows vcpkg OCCT the same way they fail on `main`; they
are not this fork's merge bar.

## Prerequisites

### Required
- **CMake** ≥ 3.20: https://cmake.org/download/
- **C++17 Compiler**: Visual Studio Build Tools or Visual Studio Community
- **OpenCASCADE dev**: https://www.opencascade.com/content/latest-release

### GUI app dependencies — automatic
GLFW and Dear ImGui are downloaded and built from source by CMake
automatically when they aren't installed (the normal case on Windows), so
`weft_app.exe` builds with no extra setup. The first configure needs
internet access for the two small source downloads.

## Setting up OpenCASCADE

### Option A: Pre-built Binaries (Recommended)
1. Download the Windows release from: https://www.opencascade.com/content/latest-release
   - Look for files like `opencascade-7.8.0-vc14-64.zip` (or later version)
2. Extract to a location (e.g., `C:\OCCT`)
3. Set environment variable: `OCCT_DIR = C:\OCCT`
4. Add `C:\OCCT\bin` to your PATH (optional, for runtime DLLs)

### Option B: Build from Source
1. Clone the repo: https://github.com/Open-Cascade-SAS/OCCT
2. Build using CMake and Visual Studio
3. Install to a known location
4. Set `OCCT_DIR` as above

## Troubleshooting

### CMake can't find OpenCASCADE
Pass the path explicitly during configure:
```cmd
cmake -B build -DOCCT_SEARCH_PATH=C:\path\to\OCCT
```

### Chocolatey not installed
Install from: https://chocolatey.org/install

Or manually install dependencies:
- CMake: https://cmake.org/download/
- Visual Studio Build Tools: https://visualstudio.microsoft.com/visual-cpp-build-tools/

### Permission denied errors
Run the batch script as Administrator (right-click → Run as Administrator).

### GUI app (weft_app.exe) missing from build\bin\Release
GLFW and ImGui are vendored under `third_party/glfw` and
`third_party/imgui`. If those folders are missing from your checkout,
pull again. Configure does not download them from the network.

### Configure hangs on `Selecting Windows SDK version...`
That line is CMake finishing `project()` / the first compiler probe. It
normally takes 30–90 seconds the first time (longer with Windows Defender).

After that you should see `Looking for OpenCASCADE...` then
`OCCT third-party runtimes: N vendored DLL(s)...`. Older revisions could
hang for many minutes here because CMake recursively scanned all of
`C:\OpenCASCADE` for third-party DLLs — that walk is gone; runtimes come
from `third_party\occt-win-runtime\` in the repo.

If nothing new appears for more than ~3 minutes:

```cmd
Ctrl+C
rmdir /s /q build
build.bat
```

Also:
- Confirm `third_party\occt-win-runtime\tbb12.dll` exists after `git pull`
- Exclude the repo folder from real-time antivirus scanning
- Close other Visual Studio instances that might lock the Windows SDK

### weft_app.exe starts then says a third-party DLL was not found
OCCT’s `TK*.dll`s import TBB, jemalloc, FreeImage, OpenVR, FreeType,
zlib, and (often) FFmpeg. Those runtimes are **vendored in git** under
`third_party/occt-win-runtime/` (from the OCCT 8.0.1
`3rdparty-vc14-64.zip` release asset).

`build.bat` and CMake post-build copy that folder into
`build\bin\Release` next to the exes. If a DLL is still missing:

1. `git pull` and confirm `third_party\occt-win-runtime\tbb12.dll` exists.
2. Re-run `build.bat` — it fails loudly if the vendored set is incomplete.
3. Launch `build\bin\Release\weft_app.exe` (or `dev.bat app`), not a
   copy of the exe sitting somewhere without those DLLs beside it.

## Output

After a successful build, everything lives in `build\bin\Release`,
**including the OpenCASCADE DLLs** — the script copies them next to the
executables, so they run from any prompt or double-click with no PATH setup:
- **CLI**: `build\bin\Release\weft.exe`
- **App**: `build\bin\Release\weft_app.exe` (if GUI dependencies were found)
- **Tests**: Run via `ctest --test-dir build -C Release`

## Next Steps

- Generate a demo model:
  `build\bin\Release\weft.exe fixture demo.step --shape demo`
- Inspect the B-rep: `build\bin\Release\weft.exe inspect demo.step`
- Mesh with independent tessellation (this fork's production path):
  `independent.bat` (GUI) or
  `independent.bat cli mesh demo.step -o demo.obj --validate`
- Raw CLI:
  `build\bin\Release\weft.exe mesh demo.step -o demo.obj --independent --validate`
- Default `weft mesh` still uses the legacy solver:
  `build\bin\Release\weft.exe mesh demo.step -o demo.obj --radial 12 --axial 3`
- Independent-mesh tests:
  `ctest --test-dir build -C Release --output-on-failure -R "independent_mesh|bake_queue|topology_cache|geometry_pool"`
- Open the OBJ in Blender; each B-rep face arrives as a named group

See [README.md](README.md) for product usage and
[docs/EXECUTION_PLAN.md](docs/EXECUTION_PLAN.md) for the authoritative roadmap
and completion gates.
