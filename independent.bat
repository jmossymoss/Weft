@echo off
REM Start Weft on this fork's independent tessellation path.
REM
REM   independent.bat                 build + GUI (demo fixture)
REM   independent.bat model.step      GUI, load that STEP
REM   independent.bat cli mesh in.step -o out.obj --validate
REM   independent.bat -d              Debug GUI
REM
REM Requires a configured build\ from build.bat. This is a thin wrapper
REM around dev.bat that always passes --independent.

setlocal enabledelayedexpansion
cd /d "%~dp0"

if not exist "%~dp0dev.bat" (
    echo independent.bat must live next to dev.bat
    exit /b 1
)

set "CONFIG_FLAG="
if /I "%~1"=="-d" (
    set "CONFIG_FLAG=-d"
    shift
)

if /I "%~1"=="cli" goto do_cli
goto do_app

:do_cli
shift
set "INJECT="
if /I "%~1"=="mesh" set "INJECT=--independent"
if /I "%~1"=="validate" set "INJECT=--independent"
set "ARGS="
:cli_collect
if "%~1"=="" goto cli_run
set "ARGS=!ARGS! "%~1""
shift
goto cli_collect
:cli_run
call "%~dp0dev.bat" %CONFIG_FLAG% cli !ARGS! %INJECT%
exit /b !errorLevel!

:do_app
set "ARGS="
:app_collect
if "%~1"=="" goto app_run
set "ARGS=!ARGS! "%~1""
shift
goto app_collect
:app_run
call "%~dp0dev.bat" %CONFIG_FLAG% app --independent !ARGS!
exit /b !errorLevel!
