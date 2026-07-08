@echo off
REM Build Weft and run the DECOUPLED-core mesher on a model (Windows).
REM The Linux/macOS counterpart is build_decoupled.sh.
REM
REM The decoupled mesher (core\src\decoupled.cpp) is always COMPILED by the normal
REM build; it is just OPT-IN at runtime behind `weft mesh --decoupled` (plain
REM `weft mesh` and the app still use the production generate() path). This script
REM is the convenient way to build + exercise the decoupled path in one step.
REM
REM Usage:
REM   build_decoupled.bat <in.step|in.stp> [out.obj] [extra weft mesh args...]
REM   build_decoupled.bat --shape <name>   [out.obj] [extra weft mesh args...]
REM   build_decoupled.bat --no-build ...   skip the compile, just run (fast re-runs)
REM
REM Examples:
REM   build_decoupled.bat model.step                    -^> model.obj
REM   build_decoupled.bat model.step out.glb --radial 24
REM   build_decoupled.bat --shape notched               built-in fixture -^> notched.obj
REM   build_decoupled.bat --shape ribbon ribbon.obj     freeform Coons demo
REM
REM It compiles the `weft` CLI, then runs
REM   weft.exe mesh <input> --decoupled --validate -o <output> [extra args]
REM Run build.bat ONCE first -- it sets up Visual Studio, OpenCASCADE and deploys
REM the OCCT runtime DLLs. This script only does the incremental build + run.

setlocal enabledelayedexpansion
set "ORIG=%CD%"
cd /d "%~dp0"
set "REPO=%CD%"

set "DO_BUILD=1"
if /i "%~1"=="--no-build" set "DO_BUILD=0"
if /i "%~1"=="--no-build" shift

if "%~1"=="" goto :usage

REM ---------------------------------------------------------------
REM Incremental build of the CLI (the decoupled core is part of it).
REM ---------------------------------------------------------------
if "%DO_BUILD%"=="1" (
    if not exist "build\CMakeCache.txt" goto :notconfigured
    echo == building weft ^(decoupled core compiled in^) ==
    cmake --build build --config Release --target weft -j
    if !errorLevel! neq 0 goto :buildfail
)

REM ---------------------------------------------------------------
REM Locate weft.exe (build.bat gathers exes into build\bin\Release).
REM ---------------------------------------------------------------
set "WEFT="
if exist "%REPO%\build\bin\Release\weft.exe" set "WEFT=%REPO%\build\bin\Release\weft.exe"
if not defined WEFT if exist "%REPO%\build\cli\Release\weft.exe" set "WEFT=%REPO%\build\cli\Release\weft.exe"
if not defined WEFT if exist "%REPO%\build\cli\weft.exe" set "WEFT=%REPO%\build\cli\weft.exe"
if not defined WEFT goto :noexe

REM Resolve input/output relative to the caller's directory, not the repo.
cd /d "%ORIG%"

REM ---------------------------------------------------------------
REM Resolve the input model. --shape NAME generates a built-in fixture first.
REM ---------------------------------------------------------------
if /i "%~1"=="--shape" goto :shape
set "INPUT=%~1"
if not exist "!INPUT!" goto :noinput
set "DEFOUT=%~n1.obj"
shift
goto :afterinput

:shape
if "%~2"=="" goto :noshapename
set "SHAPE=%~2"
set "INPUT=%TEMP%\weft_!SHAPE!.step"
echo == generating fixture '!SHAPE!' ==
"!WEFT!" fixture "!INPUT!" --shape "!SHAPE!"
if !errorLevel! neq 0 goto :fixturefail
set "DEFOUT=!SHAPE!.obj"
shift
shift

:afterinput
REM Optional output path: a bare (non-flag) next argument, else the default.
set "OUTPUT=!DEFOUT!"
set "NEXT=%~1"
if not defined NEXT goto :collect
set "NEXT0=!NEXT:~0,1!"
if not "!NEXT0!"=="-" set "OUTPUT=%~1"
if not "!NEXT0!"=="-" shift

REM Collect any remaining passthrough args (--radial, --profile cad, --weld, ...).
:collect
set "EXTRA="
:collectloop
if "%~1"=="" goto :run
set "EXTRA=!EXTRA! %~1"
shift
goto :collectloop

:run
echo == running the DECOUPLED mesher ==
echo    "!WEFT!" mesh "!INPUT!" --decoupled --validate -o "!OUTPUT!"!EXTRA!
"!WEFT!" mesh "!INPUT!" --decoupled --validate -o "!OUTPUT!"!EXTRA!
if !errorLevel! neq 0 goto :meshfail
echo.
echo == done -^> !OUTPUT! ==
pause
exit /b 0

:usage
echo Usage:
echo   build_decoupled.bat ^<in.step^> [out.obj] [extra weft mesh args...]
echo   build_decoupled.bat --shape ^<name^> [out.obj] [extra args...]
echo   build_decoupled.bat --no-build ...   ^(skip compile, just run^)
pause
exit /b 1

:notconfigured
echo build\ is not configured yet. Run build.bat ONCE first -- it installs
echo Visual Studio / OpenCASCADE and deploys the OCCT runtime DLLs. Then re-run.
pause
exit /b 1

:noexe
echo weft.exe not found under build\. Run build.bat first.
pause
exit /b 1

:noinput
echo error: input not found: !INPUT!
pause
exit /b 1

:noshapename
echo error: --shape needs a fixture name ^(e.g. notched, ribbon, cylinder^).
pause
exit /b 1

:buildfail
echo build failed -- see the errors above.
pause
exit /b 1

:fixturefail
echo fixture generation failed.
pause
exit /b 1

:meshfail
echo mesh failed -- see the errors above.
pause
exit /b 1
