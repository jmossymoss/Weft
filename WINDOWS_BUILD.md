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

3. **Try the demo**:
   ```cmd
   build\bin\Release\weft.exe fixture demo.step --shape demo
   build\bin\Release\weft.exe inspect demo.step
   build\bin\Release\weft.exe mesh demo.step -o demo.obj --radial 12 --axial 3
   ```

## Fast iteration (`dev.bat`)

`build.bat` re-checks dependencies, reconfigures CMake, redeploys DLLs, and
runs the full test suite every time — thorough, but slow for a one-line
change. Once `build.bat` has succeeded at least once, use `dev.bat` instead:

```cmd
dev.bat            REM build + run weft_app (incremental, no other checks)
dev.bat tests      REM build + run weft_tests
dev.bat cli mesh demo.step -o demo.obj --radial 12
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

REM Test
ctest --test-dir build --output-on-failure
```

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
CMake fetches GLFW/ImGui sources during configure; if that download failed
(no internet at configure time), the app target is skipped and the CLI
still builds. Re-run `build.bat` with internet access, or delete
`build\CMakeCache.txt` first to force a fresh configure.

## Output

After a successful build, everything lives in `build\bin\Release`,
**including the OpenCASCADE DLLs** — the script copies them next to the
executables, so they run from any prompt or double-click with no PATH setup:
- **CLI**: `build\bin\Release\weft.exe`
- **App**: `build\bin\Release\weft_app.exe` (if GUI dependencies were found)
- **Tests**: Run via `ctest --test-dir build -C Release`

## Next Steps

- Generate a demo model: `weft fixture demo.step --shape demo`
- Inspect the B-rep: `weft inspect demo.step`
- Generate topology: `weft mesh demo.step -o demo.obj --radial 12 --axial 3`
- Open the OBJ in Blender; each B-rep face arrives as a named group

See [README.md](README.md), the
[secure-core architecture](docs/SECURE_CORE_REWRITE_PLAN.md), and the
[agent execution playbook](docs/governance/agent-execution-playbook.md).
