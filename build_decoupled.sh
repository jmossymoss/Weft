#!/usr/bin/env bash
# Build Weft and run the DECOUPLED-core mesher on a model (Linux/macOS).
# The Windows counterpart is build_decoupled.bat.
#
# The decoupled mesher (core/src/decoupled.cpp) is always COMPILED by the normal
# build; it is just OPT-IN at runtime behind `weft mesh --decoupled` (plain
# `weft mesh` and the app still use the production generate() path). This script is
# the convenient way to build + exercise the decoupled path in one step.
#
# Usage:
#   ./build_decoupled.sh <in.step|in.stp> [out.obj] [extra weft mesh args...]
#   ./build_decoupled.sh --shape <name>   [out.obj] [extra weft mesh args...]
#
# Examples:
#   ./build_decoupled.sh model.step                    # -> model.obj
#   ./build_decoupled.sh model.step out.glb --radial 24
#   ./build_decoupled.sh --shape notched               # built-in fixture -> notched.obj
#   ./build_decoupled.sh --shape ribbon ribbon.obj     # freeform Coons demo
#
# It compiles the `weft` CLI (configuring build/ on first run), then runs
#   weft mesh <input> --decoupled --validate -o <output> [extra args]
# --validate prints the watertight / non-manifold / fold report so you can see the
# decoupled result quality immediately.
#
#   ./build_decoupled.sh --no-build ...   skip the build, just run (fast re-runs)

set -euo pipefail
ORIG_PWD="$PWD"          # input/output paths resolve against the caller's dir
cd "$(dirname "$0")"
SCRIPT_DIR="$PWD"        # absolute repo dir (where we build)

BUILD_TYPE="${WEFT_BUILD_TYPE:-Release}"
DO_BUILD=1
if [ "${1:-}" = "--no-build" ]; then DO_BUILD=0; shift; fi

if [ $# -lt 1 ]; then
    sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
fi

# ---------------------------------------------------------------
# Build the CLI. The decoupled core is part of libweft_core, so a normal compile
# includes it. First run configures; later runs are incremental. If configure
# fails (missing OpenCASCADE etc.), fall back to the full build.sh which installs
# dependencies.
# ---------------------------------------------------------------
if [ "$DO_BUILD" = 1 ]; then
    JOBS=$(nproc 2>/dev/null || echo 4)
    if [ ! -f build/CMakeCache.txt ]; then
        echo "== configuring build/ (first run, $BUILD_TYPE) =="
        if ! cmake -B build -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
                   ${CMAKE_PREFIX_PATH:+-DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH"}; then
            echo "== configure failed; running full ./build.sh to install deps =="
            ./build.sh
        fi
    fi
    echo "== building weft (decoupled core compiled in) =="
    cmake --build build --target weft -j"$JOBS"
fi

WEFT="$SCRIPT_DIR/build/cli/weft"
if [ ! -x "$WEFT" ]; then
    WEFT=$(find "$SCRIPT_DIR/build" -type f -name weft -perm -u+x 2>/dev/null | head -1 || true)
fi
if [ -z "${WEFT:-}" ] || [ ! -x "$WEFT" ]; then
    echo "error: weft binary not found under build/ -- run ./build.sh first." >&2
    exit 1
fi

# Resolve input/output relative to the caller's directory, not the repo.
cd "$ORIG_PWD"

# ---------------------------------------------------------------
# Resolve the input model. `--shape NAME` generates a built-in OCCT fixture to a
# temp STEP first (handy for the decoupled-only fixtures: notched, ribbon, ...).
# ---------------------------------------------------------------
if [ "$1" = "--shape" ]; then
    [ $# -ge 2 ] || { echo "error: --shape needs a fixture name." >&2; exit 1; }
    SHAPE="$2"; shift 2
    INPUT="$(mktemp -d)/${SHAPE}.step"
    echo "== generating fixture '$SHAPE' =="
    "$WEFT" fixture "$INPUT" --shape "$SHAPE"
    DEFAULT_OUT="${SHAPE}.obj"
else
    INPUT="$1"; shift
    [ -f "$INPUT" ] || { echo "error: input not found: $INPUT" >&2; exit 1; }
    DEFAULT_OUT="$(basename "${INPUT%.*}").obj"
fi

# Optional output path (a bare non-flag first remaining arg); else default.
OUTPUT="$DEFAULT_OUT"
if [ $# -ge 1 ] && [ "${1#-}" = "$1" ]; then OUTPUT="$1"; shift; fi

echo "== running the DECOUPLED mesher =="
echo "   $WEFT mesh \"$INPUT\" --decoupled --validate -o \"$OUTPUT\" $*"
"$WEFT" mesh "$INPUT" --decoupled --validate -o "$OUTPUT" "$@"
echo "== done -> $OUTPUT =="
