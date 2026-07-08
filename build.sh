#!/usr/bin/env bash
# Weft build script for Linux — installs missing dependencies (apt-based
# distros), configures, builds, and tests. The Windows counterpart is
# build.bat. All output is echoed AND written to build_log.txt.
#
#   ./build.sh            release build into build/ + run tests
#   ./build.sh --no-deps  skip the dependency check (offline / non-apt)
#   ./build.sh --debug    RelWithDebInfo instead of Release

set -o pipefail
cd "$(dirname "$0")"
LOG="$PWD/build_log.txt"
: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }

BUILD_TYPE=Release
CHECK_DEPS=1
DECOUPLED=""   # ""=ask/env, ON/OFF=explicit; bakes the app's default mesher
for a in "$@"; do
    case "$a" in
        --no-deps)      CHECK_DEPS=0 ;;
        --debug)        BUILD_TYPE=RelWithDebInfo ;;
        --decoupled)    DECOUPLED=ON ;;
        --no-decoupled) DECOUPLED=OFF ;;
        *) say "unknown option: $a (expected --no-deps / --debug / --decoupled / --no-decoupled)"; exit 1 ;;
    esac
done

# Which mesher should the built app start on? ON = the decoupled core (rim-notch /
# subdivided-rim walls + freeform Coons quad grids); the Topology toggle still
# switches at runtime either way. Resolve from the flag, then WEFT_DECOUPLED, then
# an interactive prompt; non-interactive builds (CI) default to OFF.
if [ -z "$DECOUPLED" ]; then
    case "${WEFT_DECOUPLED:-}" in
        1|on|ON|yes|YES|y|Y|true)   DECOUPLED=ON ;;
        0|off|OFF|no|NO|n|N|false)  DECOUPLED=OFF ;;
        *)
            if [ -t 0 ]; then
                printf '\nUse the DECOUPLED mesher as the default in this build? [y/N] '
                read -r _ans || _ans=""
                case "$_ans" in [yY]*) DECOUPLED=ON ;; *) DECOUPLED=OFF ;; esac
            else
                DECOUPLED=OFF
            fi
            ;;
    esac
fi
say "  Mesher default for this build: $([ "$DECOUPLED" = ON ] && echo 'DECOUPLED core' || echo 'production generate()')"

say "========================================"
say " Weft builder for Linux ($BUILD_TYPE)"
say " (log: build_log.txt)"
say "========================================"

# ---------------------------------------------------------------
# Dependencies. OpenCASCADE, a C++17 compiler and CMake are required;
# GLFW + OpenGL dev headers are only needed for the interactive app
# (weft_app skips itself when they're absent). Debian/Ubuntu package
# names below; on Fedora use opencascade-devel glfw-devel mesa-libGL-devel,
# on Arch opencascade glfw — then run with --no-deps.
# ---------------------------------------------------------------
if [ "$CHECK_DEPS" = 1 ]; then
    say "[1/3] Checking dependencies..."
    MISSING=""
    command -v g++  >/dev/null || MISSING="$MISSING build-essential"
    command -v cmake >/dev/null || MISSING="$MISSING cmake"
    for p in libocct-foundation-dev libocct-modeling-data-dev \
             libocct-modeling-algorithms-dev libocct-data-exchange-dev \
             libocct-ocaf-dev libocct-visualization-dev \
             libglfw3-dev libgl1-mesa-dev; do
        dpkg -s "$p" >/dev/null 2>&1 || MISSING="$MISSING $p"
    done
    if [ -n "$MISSING" ]; then
        if ! command -v apt-get >/dev/null; then
            say "  Missing:$MISSING"
            say "  This system has no apt-get; install the equivalents for"
            say "  your distro and re-run with --no-deps."
            exit 1
        fi
        say "  Installing:$MISSING"
        SUDO=""
        [ "$(id -u)" != 0 ] && SUDO="sudo"
        $SUDO apt-get update >>"$LOG" 2>&1
        # shellcheck disable=SC2086
        $SUDO apt-get install -y $MISSING >>"$LOG" 2>&1 || {
            say "  apt-get install failed — see build_log.txt."
            exit 1
        }
    fi
    say "  OK"
fi

# ---------------------------------------------------------------
# Configure + compile. System OCCT lands in the default prefix, so no
# hint paths are needed; a custom build can be pointed at with
# CMAKE_PREFIX_PATH=/opt/occt ./build.sh
# ---------------------------------------------------------------
say "[2/3] Configuring + compiling..."
cmake -B build -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DWEFT_DECOUPLED_DEFAULT="$DECOUPLED" \
      ${CMAKE_PREFIX_PATH:+-DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"} \
      2>&1 | tee -a "$LOG" | grep -E "OCCT|weft_app|decoupled|error" || true
JOBS=$(nproc 2>/dev/null || echo 4)
if ! cmake --build build -j"$JOBS" 2>&1 | tee -a "$LOG" | grep -E "error|Built target"; then
    say "  Build failed — see build_log.txt for the first error."
    exit 1
fi

# ---------------------------------------------------------------
# Tests.
# ---------------------------------------------------------------
say "[3/3] Running tests..."
if ! ctest --test-dir build --output-on-failure 2>&1 | tee -a "$LOG" | tail -3; then
    say "  Some tests failed (build itself succeeded)."
    exit 1
fi

say ""
say "========================================"
say " Build complete!"
say "   CLI:  build/cli/weft"
say "   App:  build/app/weft_app   (if GLFW/OpenGL were found)"
say ""
say " Try it:"
say "   build/cli/weft fixture demo.step --shape demo"
say "   build/cli/weft mesh demo.step -o demo.obj --profile cad --validate"
say "========================================"
