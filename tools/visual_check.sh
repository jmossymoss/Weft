#!/usr/bin/env bash
# Visual QA sweep: mesh every STEP file in a folder, validate it, and
# render it from several angles so topology defects that watertightness
# can't catch (fans, shears, spirals, folds) are visible at a glance.
#
#   tools/visual_check.sh <step-folder> [out-folder]
#
# Produces per-model PNGs (app render, wireframe overlay, 3 angles) and
# report.html — open it and scroll. Needs the app built (build/app/
# weft_app) and, on a headless box, xvfb-run.

set -o pipefail
SRC="${1:?usage: visual_check.sh <step-folder> [out-folder]}"
OUT="${2:-visual_check}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APP="$ROOT/build/app/weft_app"
CLI="$ROOT/build/cli/weft"
[ -x "$APP" ] || APP="$ROOT/build-int/app/weft_app"
[ -x "$CLI" ] || CLI="$ROOT/build-int/cli/weft"
[ -x "$APP" ] || { echo "build the app first (./build.sh)"; exit 1; }
RUNNER=""
[ -z "$DISPLAY" ] && command -v xvfb-run >/dev/null && RUNNER="xvfb-run -a"

mkdir -p "$OUT"
HTML="$OUT/report.html"
cat > "$HTML" <<'EOF'
<!doctype html><meta charset="utf-8"><title>weft visual check</title>
<style>body{font-family:monospace;background:#151515;color:#ddd}
img{width:32%;image-rendering:auto;border:1px solid #333}
h2{margin:24px 0 4px}pre{color:#9c9}</style>
EOF

shopt -s nullglob nocaseglob
for f in "$SRC"/*.step "$SRC"/*.stp; do
    name=$(basename "$f"); base="${name%.*}"
    echo "== $name"
    v=$($CLI mesh "$f" -o "$OUT/$base.obj" --profile cad --validate 2>&1 \
        | grep -iE "polygons \(|watertight|folded|degenerate" || true)
    echo "$v"
    i=0
    for view in "0.9 0.5" "2.4 0.4" "4.0 -0.6"; do
        set -- $view
        $RUNNER "$APP" "$f" --yaw "$1" --pitch "$2" \
            --screenshot "$OUT/${base}_v$i.png" >/dev/null 2>&1
        i=$((i+1))
    done
    {
        echo "<h2>$name</h2><pre>$v</pre>"
        for k in 0 1 2; do echo "<img src='${base}_v$k.png'>"; done
    } >> "$HTML"
done
echo "report: $HTML"
