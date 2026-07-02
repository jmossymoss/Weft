@echo off
REM Weft Build Script for Windows
REM Automatically installs dependencies and builds the project

setlocal enabledelayedexpansion

echo.
echo ========================================
echo Weft Builder for Windows
echo ========================================
echo.

REM Check for admin privileges (needed for Chocolatey install)
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo Warning: Administrator privileges not detected.
    echo Some dependency installations may fail.
    echo.
)

REM Check for CMake
echo [1/5] Checking CMake...
cmake --version >nul 2>&1
if %errorLevel% neq 0 (
    echo  Not found. Installing CMake via Chocolatey...
    choco install cmake -y
    if !errorLevel! neq 0 (
        echo Error: Failed to install CMake. Please install manually from https://cmake.org/download/
        exit /b 1
    )
    REM Refresh PATH
    call "%ProgramData%\chocolatey\tools\refreshEnv.cmd"
) else (
    echo  Found
)

REM Check for Git
echo [2/5] Checking Git...
git --version >nul 2>&1
if %errorLevel% neq 0 (
    echo  Not found. Installing Git via Chocolatey...
    choco install git -y
    if !errorLevel! neq 0 (
        echo Error: Failed to install Git. Please install manually from https://git-scm.com/
        exit /b 1
    )
    call "%ProgramData%\chocolatey\tools\refreshEnv.cmd"
) else (
    echo  Found
)

REM Check for Visual Studio Build Tools or MSVC
echo [3/5] Checking C++ compiler...
where cl.exe >nul 2>&1
if %errorLevel% neq 0 (
    echo  Not found. Attempting to install Visual Studio Build Tools...
    choco install visualstudio2022buildtools -y --package-parameters "--includeRecommended --add Microsoft.VisualStudio.Workload.NativeDesktop"
    if !errorLevel! neq 0 (
        echo Error: Failed to install MSVC. Please install Visual Studio Build Tools manually.
        echo Download from: https://visualstudio.microsoft.com/visual-cpp-build-tools/
        exit /b 1
    )
    call "%ProgramData%\chocolatey\tools\refreshEnv.cmd"
) else (
    echo  Found
)

REM Check for OpenCASCADE
echo [4/5] Checking OpenCASCADE...
set OCCT_DIR=
if exist "C:\OCCT" (
    set OCCT_DIR=C:\OCCT
) else if exist "%ProgramFiles%\OCCT" (
    set OCCT_DIR=%ProgramFiles%\OCCT
) else if exist "%ProgramFiles(x86)%\OCCT" (
    set OCCT_DIR=%ProgramFiles(x86)%\OCCT
)

if "!OCCT_DIR!"=="" (
    echo  Not found. OpenCASCADE is required but must be installed separately.
    echo.
    echo Please download and install OpenCASCADE from:
    echo   https://www.opencascade.com/content/latest-release
    echo.
    echo Installation instructions:
    echo   1. Download the Windows release (e.g., opencascade-7.8.0-vc14-64.zip)
    echo   2. Extract to a folder (e.g., C:\OCCT)
    echo   3. Set OCCT_DIR environment variable to that folder
    echo   4. Optionally, add %%OCCT_DIR%%\bin to your PATH
    echo.
    echo Alternatively, if CMake cannot find OpenCASCADE automatically,
    echo you can pass -DOCCT_SEARCH_PATH=C:\path\to\OCCT during cmake configure.
    exit /b 1
) else (
    echo  Found at !OCCT_DIR!
)

REM Check for optional dependencies (GLFW, ImGui, STB)
echo [5/5] Checking optional GUI dependencies...
set MISSING_OPTIONAL=0

if not exist "%ProgramFiles%\GLFW\include\GLFW" (
    if not exist "%ProgramFiles(x86)%\GLFW\include\GLFW" (
        echo  GLFW not found. GUI app will be skipped.
        set MISSING_OPTIONAL=1
    )
)

if !MISSING_OPTIONAL! equ 1 (
    echo  (GUI building is optional; headless CLI will still build)
)

echo.
echo ========================================
echo Building Weft
echo ========================================
echo.

REM Create build directory
if not exist build (
    mkdir build
)

REM Run CMake configure
if defined OCCT_DIR (
    echo Running CMake with OCCT_DIR=!OCCT_DIR!
    cmake -B build -DOCCT_SEARCH_PATH="!OCCT_DIR!"
) else (
    echo Running CMake...
    cmake -B build
)

if %errorLevel% neq 0 (
    echo Error: CMake configuration failed.
    exit /b 1
)

REM Build
echo.
echo Compiling...
cmake --build build --config Release -j
if %errorLevel% neq 0 (
    echo Error: Build failed.
    exit /b 1
)

REM Run tests
echo.
echo ========================================
echo Running Tests
echo ========================================
echo.

ctest --test-dir build --output-on-failure
if %errorLevel% neq 0 (
    echo Warning: Some tests failed.
    exit /b 1
)

echo.
echo ========================================
echo Build Complete!
echo ========================================
echo.
echo Executables:
echo   CLI:  build\cli\Release\weft.exe
echo   App:  build\app\Release\weft_app.exe (if GUI dependencies found)
echo.
echo Try the demo:
echo   build\cli\Release\weft.exe fixture demo.step --shape demo
echo   build\cli\Release\weft.exe inspect demo.step
echo   build\cli\Release\weft.exe mesh demo.step -o demo.obj --radial 12 --axial 3
echo.

exit /b 0
