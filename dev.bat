@echo off
REM Fast incremental build for iterating on Weft during development.
REM Skips everything build.bat does except the actual compile: no VS/
REM CMake/OCCT re-checks, no CMake reconfigure, no DLL redeploy, no test
REM suite. Run build.bat once first to get a configured build\ directory
REM and the runtime DLLs in place; after that, this is what you want.
REM
REM   dev.bat                build + run weft_app (the GUI)
REM   dev.bat app            same as above
REM   dev.bat tests          build + run weft_tests
REM   dev.bat cli [args...]  build weft.exe, then run it with args
REM   dev.bat build          build everything, don't run anything
REM
REM Independent tessellation (this fork's production path):
REM   independent.bat                 GUI with --independent
REM   independent.bat cli mesh ...    CLI mesh/validate with --independent
REM
REM Add -d for a Debug build (faster to compile, slower to run):
REM   dev.bat -d app
REM   dev.bat -d cli inspect demo.step

setlocal enabledelayedexpansion
cd /d "%~dp0"

if not exist build\CMakeCache.txt (
    echo No configured build found. Run build.bat first.
    exit /b 1
)

set "CONFIG=Release"
if /I "%~1"=="-d" (
    set "CONFIG=Debug"
    shift
)

set "TARGET=%~1"
if "%TARGET%"=="" set "TARGET=app"
shift

REM Collect anything after the target token to forward to the exe
REM (dev.bat can't rely on %* here -- shift doesn't update it).
set "ARGS="
:collect
if "%~1"=="" goto collected
set "ARGS=!ARGS! "%~1""
shift
goto collect
:collected

set "BINDIR=%~dp0build\bin\%CONFIG%"

REM Debug lands in build\bin\Debug, but build.bat only ever deployed the
REM OCCT runtime DLLs into build\bin\Release. Mirror them over once so
REM Debug exes don't hit "DLL not found" -- xcopy /D keeps it cheap on
REM repeat runs (only copies what's missing or newer).
if /I "%CONFIG%"=="Debug" if not "%TARGET%"=="build" (
    if exist "%~dp0build\bin\Release\TKernel.dll" (
        if not exist "%BINDIR%" mkdir "%BINDIR%" >nul 2>nul
        xcopy "%~dp0build\bin\Release\*.dll" "%BINDIR%" /D /Y >nul
    )
)

REM Ensure vendored OCCT third-party runtimes sit next to the exe.
REM (build.bat does a full deploy; this keeps `dev.bat` usable after a
REM clean bin folder without re-running the whole suite.)
set "TPRUNTIME=%~dp0third_party\occt-win-runtime"
if exist "%TPRUNTIME%\tbb12.dll" (
    if not exist "%BINDIR%" mkdir "%BINDIR%" >nul 2>nul
    xcopy "%TPRUNTIME%\*.dll" "%BINDIR%" /D /Y >nul
)
if not "%TARGET%"=="build" (
    if not exist "%BINDIR%\tbb12.dll" (
        echo.
        echo   FATAL: tbb12.dll not next to the exe and vendored runtimes
        echo   missing. Pull third_party\occt-win-runtime from git, then
        echo   re-run. See WINDOWS_BUILD.md.
        echo.
        exit /b 1
    )
)

if /I "%TARGET%"=="app" (
    cmake --build build --config %CONFIG% --target weft_app -j || exit /b 1
    "%BINDIR%\weft_app.exe" !ARGS!
    exit /b 0
)
if /I "%TARGET%"=="tests" (
    cmake --build build --config %CONFIG% --target weft_tests -j || exit /b 1
    "%BINDIR%\weft_tests.exe"
    exit /b 0
)
if /I "%TARGET%"=="cli" (
    cmake --build build --config %CONFIG% --target weft -j || exit /b 1
    "%BINDIR%\weft.exe" !ARGS!
    exit /b 0
)
if /I "%TARGET%"=="build" (
    cmake --build build --config %CONFIG% -j
    exit /b !errorLevel!
)

echo Unknown target: %TARGET%
echo Usage: dev.bat [-d] [app^|tests^|cli^|build] [args...]
exit /b 1
