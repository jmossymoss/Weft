@echo off
REM Weft Build Script for Windows
REM Installs missing dependencies, configures, builds, and tests.
REM All output is also written to build_log.txt next to this script.
REM The window ALWAYS pauses before closing so errors stay readable.

setlocal enabledelayedexpansion
cd /d "%~dp0"
set "LOG=%~dp0build_log.txt"
echo Weft build started %date% %time% > "%LOG%"

echo.
echo ========================================
echo  Weft Builder for Windows
echo  (log: build_log.txt)
echo ========================================
echo.

REM ---------------------------------------------------------------
REM [1/5] Visual Studio / Build Tools
REM Do NOT look for cl.exe on PATH -- it only exists inside a
REM "Developer Command Prompt". Use vswhere, which ships with any
REM VS 2017+ install, to find a C++ toolset properly.
REM ---------------------------------------------------------------
echo [1/5] Checking Visual Studio C++ toolset...
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
set "VS_PATH="
if exist "%VSWHERE%" (
    for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%i"
)

if defined VS_PATH (
    echo   Found: !VS_PATH!
    echo Found VS at !VS_PATH! >> "%LOG%"
) else (
    echo   No C++ toolset found.
    call :ensure_choco || goto :fail
    echo   Installing Visual Studio 2022 Build Tools -- this is a LARGE
    echo   download and can take 10-30 minutes. Please wait...
    choco install visualstudio2022buildtools -y --no-progress ^
        --package-parameters "--add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --passive" >> "%LOG%" 2>&1
    if !errorLevel! neq 0 (
        echo   Chocolatey install failed ^(exit code !errorLevel!^).
        echo   Check build_log.txt for details, or install manually:
        echo     https://visualstudio.microsoft.com/visual-cpp-build-tools/
        echo   ^(select the "Desktop development with C++" workload^)
        goto :fail
    )
    REM Re-detect after install
    if exist "%VSWHERE%" (
        for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_PATH=%%i"
    )
    if not defined VS_PATH (
        echo   Build Tools installed but the C++ toolset was not detected.
        echo   You may need to reboot, or re-run this script from a fresh
        echo   command prompt. If it persists, open the Visual Studio
        echo   Installer and confirm "Desktop development with C++" is ticked.
        goto :fail
    )
    echo   Installed and detected at !VS_PATH!
)

REM ---------------------------------------------------------------
REM [2/5] CMake
REM ---------------------------------------------------------------
echo [2/5] Checking CMake...
where cmake >nul 2>&1
if %errorLevel% neq 0 (
    call :ensure_choco || goto :fail
    echo   Installing CMake...
    choco install cmake -y --no-progress --installargs "ADD_CMAKE_TO_PATH=System" >> "%LOG%" 2>&1
    if !errorLevel! neq 0 (
        echo   Failed. Install manually from https://cmake.org/download/
        goto :fail
    )
    REM Pick up the new PATH without needing a new console
    set "PATH=%PATH%;%ProgramFiles%\CMake\bin"
    where cmake >nul 2>&1 || (
        echo   CMake installed but not on PATH yet. Open a NEW command
        echo   prompt and re-run this script.
        goto :fail
    )
)
echo   Found

REM ---------------------------------------------------------------
REM [3/5] Git (only needed if you want to pull updates; not fatal)
REM ---------------------------------------------------------------
echo [3/5] Checking Git...
where git >nul 2>&1
if %errorLevel% neq 0 (
    echo   Not found ^(not required to build^) -- skipping.
) else (
    echo   Found
)

REM ---------------------------------------------------------------
REM [4/5] OpenCASCADE
REM ---------------------------------------------------------------
echo [4/5] Checking OpenCASCADE...
set "OCCT_DIR="
if defined CASROOT if exist "%CASROOT%" set "OCCT_DIR=%CASROOT%"
if not defined OCCT_DIR if exist "C:\OCCT" set "OCCT_DIR=C:\OCCT"
if not defined OCCT_DIR (
    for /d %%d in ("C:\OpenCASCADE*") do (
        if exist "%%d\cmake" set "OCCT_DIR=%%d"
        for /d %%s in ("%%d\opencascade-*") do set "OCCT_DIR=%%s"
    )
)

if not defined OCCT_DIR (
    echo.
    echo   OpenCASCADE was not found. It has no reliable Chocolatey
    echo   package, so it needs a one-time manual install:
    echo.
    echo     1. Download the Windows installer from
    echo        https://dev.opencascade.org/release
    echo        ^(e.g. opencascade-7.8.x-vc14-64.exe^)
    echo     2. Run it ^(default location is C:\OpenCASCADE\...^)
    echo     3. Re-run this script -- it auto-detects C:\OCCT,
    echo        C:\OpenCASCADE*, or the CASROOT environment variable.
    echo.
    goto :fail
)
echo   Found at !OCCT_DIR!
echo Using OCCT at !OCCT_DIR! >> "%LOG%"

REM ---------------------------------------------------------------
REM [5/5] Optional GUI deps (GLFW/ImGui/stb) -- app target skips
REM itself when absent, so this is informational only.
REM ---------------------------------------------------------------
echo [5/5] GUI dependencies...
echo   GLFW and ImGui are vendored under third_party\ ^(no network fetch^).

echo.
echo ========================================
echo  Configuring
echo ========================================
echo.
echo   After "Selecting Windows SDK version..." CMake compiles a short
echo   compiler probe ^(often 30-90s the first time, Windows Defender can
echo   stretch it^). Then it should print "Looking for OpenCASCADE...".
echo   If NOTHING new appears for more than ~3 minutes:
echo     1. Ctrl+C
echo     2. rmdir /s /q build
echo     3. Re-run build.bat
echo     Also exclude the repo folder from real-time antivirus scanning.
echo.

REM Stale half-configured trees are a common hang source — wipe if the
REM previous configure never finished (no CMakeCache.txt yet).
if exist "build\CMakeCache.txt" (
    echo   Reusing existing build\CMakeCache.txt
) else if exist "build" (
    echo   Clearing incomplete build\ from a previous interrupted configure...
    rmdir /s /q build 2>nul
)

REM Use the VS generator: CMake locates the compiler through the VS
REM installation itself, so we never need cl.exe on PATH or vcvars.
echo   Running: cmake -B build -G "Visual Studio 17 2022" -A x64 ...
echo. >> "%LOG%"
echo === cmake configure %date% %time% === >> "%LOG%"
REM Live console + append to build_log.txt. STATUS lines so the SDK probe
REM is not the last thing you see for minutes.
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
  -DCMAKE_PREFIX_PATH="!OCCT_DIR!" ^
  -DOCCT_SEARCH_PATH="!OCCT_DIR!" ^
  --log-level=STATUS
set "CFG_ERR=!errorLevel!"
>> "%LOG%" echo cmake configure exit=!CFG_ERR!
if !CFG_ERR! neq 0 (
    echo.
    echo   CMake configuration failed ^(exit !CFG_ERR!^) -- see output above.
    echo   If configure hung previously, delete the build folder and retry:
    echo     rmdir /s /q build
    goto :fail
)
if not exist "build\CMakeCache.txt" (
    echo   Configure did not produce build\CMakeCache.txt — treating as failure.
    goto :fail
)
echo   Configure finished.

echo.
echo ========================================
echo  Compiling
echo ========================================
echo.
cmake --build build --config Release -j
if %errorLevel% neq 0 (
    echo   Build failed -- scroll up for the first error, or see build_log.txt.
    goto :fail
)

echo.
echo ========================================
echo  Running tests
echo ========================================
echo.
REM The executables need OCCT's DLLs at runtime. Deploy ONLY what Weft
REM actually loads next to the exes: the TK*.dlls plus OCCT's bundled
REM third-party runtimes -- NOT the whole install (tcl/tk, Qt, ffmpeg
REM and friends stay out of the bin folder). xcopy /D keeps re-runs
REM incremental; stale DLLs from older builds are cleared first.
set "BINDIR=%~dp0build\bin\Release"
if not exist "%BINDIR%" mkdir "%BINDIR%"
echo   Deploying OCCT runtime DLLs to build\bin\Release...
if exist "%BINDIR%\tcl86.dll" (
    echo   ^(clearing DLLs from a previous full-copy deploy^)
    del /q "%BINDIR%\*.dll" >nul 2>nul
)
set "DLLDIRS=;"
for /f "delims=" %%f in ('dir /s /b "!OCCT_DIR!\TKernel.dll" 2^>nul') do (
    if "!DLLDIRS:%%~dpf;=!"=="!DLLDIRS!" (
        set "DLLDIRS=!DLLDIRS!%%~dpf;"
        xcopy "%%~dpfTK*.dll" "%BINDIR%" /D /Y >nul
    )
)
REM Third-party runtimes are VENDORED in the repo under
REM third_party\occt-win-runtime\ (from OCCT 8.0.1 3rdparty-vc14-64.zip).
REM Copy them next to the exes -- no discovery from the OCCT install.
set "TPRUNTIME=%~dp0third_party\occt-win-runtime"
if not exist "%TPRUNTIME%\tbb12.dll" (
    echo.
    echo   *** FATAL: vendored OCCT runtime DLLs missing:
    echo   ***   %TPRUNTIME%
    echo   *** Pull the latest branch ^(third_party/occt-win-runtime^) or
    echo   *** restore that folder from git. A build without these DLLs
    echo   *** cannot launch weft_app.exe.
    echo.
    goto :fail
)
echo   Deploying vendored OCCT third-party DLLs from third_party\occt-win-runtime...
xcopy "%TPRUNTIME%\*.dll" "%BINDIR%" /D /Y >nul
if %errorLevel% neq 0 (
    echo   Failed to copy vendored runtime DLLs.
    goto :fail
)
REM Required set that OCCT 8.x TK dlls import at process start.
set "MISSING_REQ="
for %%n in (tbb12.dll tbb12_debug.dll jemalloc.dll FreeImage.dll openvr_api.dll freetype.dll) do (
    if not exist "%BINDIR%\%%n" set "MISSING_REQ=!MISSING_REQ! %%n"
)
if defined MISSING_REQ (
    echo.
    echo   *** FATAL: required runtime DLL^(s^) not in %BINDIR%:!MISSING_REQ!
    echo   *** The vendored folder is incomplete. Restore third_party\occt-win-runtime from git.
    echo.
    goto :fail
)
for /f %%c in ('dir /b "%BINDIR%\*.dll" 2^>nul ^| find /c ".dll"') do (
    echo   %%c runtime DLL^(s^) in place ^(TK + vendored third-party^)
)
set "TPDIRS=%TPRUNTIME%;"

REM Also put the OCCT runtime folders on the user PATH: the copy above
REM covers build\bin\Release, but a PATH entry covers exes run from
REM anywhere and any DLL the copy list missed. User scope -- no admin,
REM no setx (setx truncates PATH at 1024 chars). Current session gets
REM it immediately; other terminals after a restart.
set "OCCTPATHS="
if not "!DLLDIRS!"==";" set "OCCTPATHS=!DLLDIRS:~1!"
if not "!TPDIRS!"==";" set "OCCTPATHS=!OCCTPATHS!!TPDIRS:~1!"
if defined OCCTPATHS (
    set "PATH=!PATH!;!OCCTPATHS!"
    echo   Ensuring OCCT runtime folders are on your user PATH...
    powershell -NoProfile -Command ^
        "$add = '!OCCTPATHS!'.TrimEnd(';').Split(';');" ^
        "$cur = [Environment]::GetEnvironmentVariable('Path','User');" ^
        "if ($null -eq $cur) { $cur = '' };" ^
        "$parts = $cur.Split(';') | ForEach-Object { $_.TrimEnd('\') };" ^
        "$new = $cur;" ^
        "foreach ($d in $add) { $t = $d.TrimEnd('\'); if ($t -and ($parts -notcontains $t)) { $new = ($new.TrimEnd(';') + ';' + $t) } };" ^
        "if ($new -ne $cur) { [Environment]::SetEnvironmentVariable('Path',$new,'User'); Write-Output '  user PATH updated' } else { Write-Output '  already on PATH' }" 2>>"%LOG%"
)
REM Verify the DLL closure BEFORE running anything: dump the direct
REM imports of the exes and every deployed TK dll (VS ships dumpbin),
REM and name any import that is neither in the bin dir, nor a Windows
REM system DLL, nor findable on PATH. A missing one is exactly the
REM 0xc0000135 ctest failure -- but with the DLL's name visible.
set "DUMPBIN="
for /f "delims=" %%f in ('dir /s /b "!VS_PATH!\dumpbin.exe" 2^>nul ^| findstr /i "Hostx64\\x64"') do (
    if not defined DUMPBIN set "DUMPBIN=%%f"
)
if defined DUMPBIN (
    echo   Verifying every imported DLL resolves...
    set "MISSING=;"
    for %%e in ("%BINDIR%\weft.exe" "%BINDIR%\weft_tests.exe" "%BINDIR%\weft_app.exe" "%BINDIR%\TK*.dll") do (
        if exist "%%~e" (
            for /f %%i in ('"!DUMPBIN!" /nologo /dependents "%%~e" 2^>nul') do (
                if /i "%%~xi"==".dll" (
                    if not exist "%BINDIR%\%%i" if not exist "%SystemRoot%\System32\%%i" (
                        where "%%i" >nul 2>&1 || (
                            if "!MISSING:;%%i;=!"=="!MISSING!" set "MISSING=!MISSING!%%i;"
                        )
                    )
                )
            )
        )
    )
    if not "!MISSING!"==";" (
        echo.
        echo   *** MISSING RUNTIME DLL^(s^): !MISSING:~1,-1!
        echo   *** OCCT imports these but they were not found in the OCCT
        echo   *** folder, its 3rdparty siblings, or on PATH. Find them:
        echo   ***     dir /s /b C:\!MISSING:~1,-1! ^(per name^)
        echo   *** then copy them into build\bin\Release or add their
        echo   *** folder to PATH and re-run.
        echo MISSING DLLS:!MISSING:~1! >> "%LOG%"
        echo.
    ) else (
        echo   All imports resolve.
    )
)
echo   Running independent-mesh tests ^(this fork's Windows gate^)...
ctest --test-dir build -C Release --output-on-failure -R "independent_mesh|bake_queue|topology_cache|geometry_pool"
if %errorLevel% neq 0 (
    echo   Independent-mesh tests failed ^(build itself succeeded^).
    goto :fail
)
echo   Legacy generate^(^) pipeline tests ^(non-blocking on Windows^)...
ctest --test-dir build -C Release -R pipeline --output-on-failure
if %errorLevel% neq 0 (
    echo   Pipeline tests failed on Windows ^(known; same as main vcpkg OCCT^).
    echo   Independent mesh gate passed; continuing.
)

echo.
echo ========================================
echo  Build complete!
echo ========================================
echo.
echo   Everything is in build\bin\Release ^(DLLs included -- the exes
echo   run from anywhere, no PATH setup needed^):
echo.
echo   CLI:  build\bin\Release\weft.exe
echo   App:  build\bin\Release\weft_app.exe   ^(if GUI deps were found^)
echo.
echo   Start the GUI on independent tessellation:
echo     independent.bat
echo     independent.bat path\to\model.step
echo   Or CLI:
echo     independent.bat cli mesh demo.step -o demo.obj --validate
echo.
echo   Try it:
echo     build\bin\Release\weft.exe fixture demo.step --shape demo
echo     build\bin\Release\weft.exe mesh demo.step -o demo.obj --independent --validate
echo.
pause
exit /b 0

REM ---------------------------------------------------------------
:ensure_choco
where choco >nul 2>&1 && exit /b 0
echo   Chocolatey is needed to auto-install this dependency.
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo   ERROR: Installing dependencies requires an ADMINISTRATOR prompt.
    echo   Right-click build.bat and choose "Run as administrator".
    exit /b 1
)
echo   Installing Chocolatey...
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "[System.Net.ServicePointManager]::SecurityProtocol = 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))" >> "%LOG%" 2>&1
set "PATH=%PATH%;%ProgramData%\chocolatey\bin"
where choco >nul 2>&1 && exit /b 0
echo   Chocolatey install failed -- see build_log.txt.
exit /b 1

REM ---------------------------------------------------------------
:fail
echo.
echo ======== BUILD SCRIPT STOPPED ^(details above / build_log.txt^) ========
echo.
pause
exit /b 1
