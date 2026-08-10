#!/usr/bin/env bash
# Build a portable Linux tarball for weft CLI + app.
# Bundles non-system shared libraries next to the binaries and sets
# RPATH=$ORIGIN/lib so testers do not need OCCT/GLFW installed.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${WEFT_BUILD_DIR:-$ROOT/build}"
OUT="${1:-$ROOT/dist}"
NAME="${WEFT_PACKAGE_NAME:-weft-linux-x64}"
STAGE="$OUT/$NAME"

if [[ ! -x "$BUILD/cli/weft" || ! -x "$BUILD/app/weft_app" ]]; then
  echo "missing binaries under $BUILD — configure and build Release first" >&2
  exit 1
fi

rm -rf "$STAGE"
mkdir -p "$STAGE/bin" "$STAGE/lib" "$STAGE/blender"

cp -f "$BUILD/cli/weft" "$STAGE/bin/weft"
cp -f "$BUILD/app/weft_app" "$STAGE/bin/weft_app"
cp -f "$ROOT/blender/weft_link.py" "$STAGE/blender/weft_link.py"
cp -f "$ROOT/README.md" "$STAGE/README.md"

# Collect direct + transitive shared deps that are not glibc/OpenGL/X11
# system libraries. Those stay on the host.
is_system_lib() {
  case "$1" in
    linux-vdso.so.*|ld-linux-*.so.*|libc.so.*|libm.so.*|libdl.so.*| \
    libpthread.so.*|librt.so.*|libresolv.so.*|libgcc_s.so.*| \
    libstdc++.so.*|libGL.so.*|libGLX.so.*|libGLdispatch.so.*| \
    libOpenGL.so.*|libX11.so.*|libXext.so.*|libXrandr.so.*| \
    libXinerama.so.*|libXcursor.so.*|libXi.so.*|libXxf86vm.so.*| \
    libxcb.so.*|libXau.so.*|libXdmcp.so.*)
      return 0
      ;;
  esac
  return 1
}

declare -A SEEN=()
queue=("$STAGE/bin/weft" "$STAGE/bin/weft_app")
while ((${#queue[@]})); do
  bin="${queue[0]}"
  queue=("${queue[@]:1}")
  while read -r line; do
    # "libFoo.so.1 => /path/libFoo.so.1 (0x...)" or "libFoo.so.1 => not found"
    name="${line%% =>*}"
    name="$(echo "$name" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')"
    [[ -z "$name" ]] && continue
    is_system_lib "$name" && continue
    path="$(echo "$line" | sed -n 's/^.*=>[[:space:]]*\([^ ]\+\)[[:space:]].*/\1/p')"
    [[ -z "$path" || "$path" == "not" || ! -f "$path" ]] && continue
    base="$(basename "$path")"
    [[ -n "${SEEN[$base]+x}" ]] && continue
    SEEN[$base]=1
    cp -fL "$path" "$STAGE/lib/$base"
    queue+=("$STAGE/lib/$base")
  done < <(ldd "$bin" 2>/dev/null || true)
done

# Point binaries at ./lib relative to the executable.
if command -v patchelf >/dev/null 2>&1; then
  for exe in "$STAGE/bin/weft" "$STAGE/bin/weft_app"; do
    patchelf --set-rpath '$ORIGIN/../lib' "$exe"
  done
  for so in "$STAGE/lib"/lib*.so*; do
    [[ -f "$so" ]] || continue
    patchelf --set-rpath '$ORIGIN' "$so" 2>/dev/null || true
  done
else
  # Fallback launcher when patchelf is unavailable.
  cat > "$STAGE/weft" <<'EOF'
#!/usr/bin/env bash
ROOT="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$ROOT/bin/weft" "$@"
EOF
  cat > "$STAGE/weft_app" <<'EOF'
#!/usr/bin/env bash
ROOT="$(cd "$(dirname "$0")" && pwd)"
export LD_LIBRARY_PATH="$ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$ROOT/bin/weft_app" "$@"
EOF
  chmod +x "$STAGE/weft" "$STAGE/weft_app"
fi

cat > "$STAGE/PRERELEASE.txt" <<EOF
Weft Linux pre-release package
==============================

Contents:
  bin/weft       CLI
  bin/weft_app   interactive app
  lib/           bundled OpenCASCADE / GLFW / TBB shared libraries
  blender/weft_link.py

Quick start:
  ./bin/weft_app
  ./bin/weft fixture demo.step --shape demo
  ./bin/weft mesh demo.step -o demo.obj --radial 12 --axial 3 --validate

Install blender/weft_link.py as a Blender add-on for the live link.

This is an engineering pre-release for testing, not an MVP package.
Needs a working OpenGL stack (typical Linux desktop / laptop GPU drivers).
EOF

mkdir -p "$OUT"
TAR="$OUT/${NAME}.tar.gz"
rm -f "$TAR"
tar -C "$OUT" -czf "$TAR" "$NAME"
echo "wrote $TAR"
du -h "$TAR"
ls -la "$STAGE/bin" "$STAGE/lib" | head -80
