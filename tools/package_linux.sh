#!/usr/bin/env bash
# WP-163: assemble a Linux release candidate tree with deterministic digests.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-${ROOT}/dist/weft-linux}"
BUILD="${WEFT_BUILD_DIR:-${ROOT}/build/linux-gcc}"

rm -rf "${OUT}"
mkdir -p "${OUT}/bin" "${OUT}/docs" "${OUT}/licenses"

install -m 755 "${BUILD}/cli/weft" "${OUT}/bin/weft"
install -m 755 "${BUILD}/app/weft_app" "${OUT}/bin/weft_app"

cp "${ROOT}/README.md" "${OUT}/docs/README.md"
cp "${ROOT}/docs/governance/blocked-routes.md" "${OUT}/docs/blocked-routes.md"
cp "${ROOT}/docs/governance/milestones.md" "${OUT}/docs/milestones.md"

# License notes (dependency inventory from WP-160).
cat >"${OUT}/licenses/NOTICE.txt" <<'EOF'
Weft Linux package — third-party notices
=======================================

Open CASCADE Technology (OCCT)
  License: LGPL-2.1-only with exception (system package / vendor install)
  Role: CAD kernel linked by weft_core

GLFW
  License: Zlib
  Role: window/input for weft_app (system libglfw3 or FetchContent 3.4)

Dear ImGui (docking branch, FetchContent pin v1.90.9-docking)
  License: MIT
  Role: GUI for weft_app

OpenGL / Mesa
  License: various (system)
  Role: rendering for weft_app

CGAL
  Not included. BR-006 remains: do not ship CGAL in distribution builds.
EOF

(
  cd "${OUT}"
  find . -type f ! -name SHA256SUMS | sort | while read -r f; do
    sha256sum "${f}"
  done
) >"${OUT}/SHA256SUMS"

echo "packaged ${OUT}"
wc -l "${OUT}/SHA256SUMS"
