#!/usr/bin/env bash
# Strict release gate — CAD_CORPUS.tsv release-tier cases only.
# Never consumes tests/KNOWN_RED.tsv. Expected red until WP3.

set -u
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=corpus_common.sh
source "$SCRIPT_DIR/corpus_common.sh"
ROOT="$(corpus_root)"
cd "$ROOT"

WEFT=${WEFT:-build/cli/weft}
if [[ ! -x "$WEFT" && -x build/bin/Release/weft.exe ]]; then
    WEFT=build/bin/Release/weft.exe
fi
if [[ ! -x "$WEFT" && -x build/vs2022/bin/Release/weft.exe ]]; then
    WEFT=build/vs2022/bin/Release/weft.exe
fi

MANIFEST=tests/CAD_CORPUS.tsv
OUT=${OUT:-$(mktemp -d)}
mkdir -p "$OUT"
FAIL=0

run_one() {
    local name="$1" file="$2" args="$3" tag="$4"
    local log="$OUT/$name-$tag.log"
    "$WEFT" mesh "$file" -o "$OUT/$name-$tag.obj" $args --validate > "$log" 2>&1
    local rc=$?
    if [[ "$rc" != 0 ]]; then
        echo "FAIL $name [$tag]: exit $rc"
        FAIL=1
        return
    fi
    local demo
    demo=$(grep "demoted:" "$log" || true)
    if [[ -n "$demo" ]] && ! echo "$demo" | grep -q "0 to raw triangulation, 0 emitted nothing"; then
        echo "FAIL $name [$tag]: $demo"
        FAIL=1
    fi
    if grep -Eq 'watertight:[[:space:]]*NO' "$log"; then
        echo "FAIL $name [$tag]: watertightness"
        FAIL=1
    fi
}

echo "strict release gate (KNOWN_RED ignored)"
while IFS= read -r row; do
    name=$(corpus_field "$row" 1)
    tier=$(corpus_field "$row" 2)
    rel=$(corpus_field "$row" 3)
    [[ "$tier" == "release" ]] || continue
    step="$ROOT/tests/$rel"
    if [[ "$rel" == fixtures/* || "$rel" == fixtures\\* ]]; then
        ensure_fixture_step "$WEFT" "$step" "$name" "fixture" || true
    fi
    if [[ ! -f "$step" ]]; then
        echo "FAIL $name: missing tests/$rel"
        FAIL=1
        continue
    fi
    run_one "$name" "$step" "--profile cad" cad
    run_one "$name" "$step" "" default
done < <(corpus_rows "$MANIFEST")

if [[ "$FAIL" == 0 ]]; then
    echo "release gate: PASS"
else
    echo "release gate: FAIL (expected until WP3 clears release blockers)"
fi
exit "$FAIL"
