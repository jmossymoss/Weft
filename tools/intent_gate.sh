#!/usr/bin/env bash
# Artist-intent gate — density edits must never cost a face its planned
# topology (EXECUTION_PLAN §3.2 "a local edit does not silently demote an
# adjacent face").
#
#   tools/intent_gate.sh              fast tier (sampled faces/counts)
#   tools/intent_gate.sh --full       every editable face, every count
#   tools/intent_gate.sh --update     bank the current numbers as the ratchet
#   tools/intent_gate.sh --model NAME just one manifest case
#
# Cases come from tests/CAD_CORPUS.tsv only (no hardcoded model list). The
# ledger tools/golden_intent.txt is a RATCHET: self-lost, neighbour-lost,
# leaks and throws may fall but never rise. It starts non-zero on purpose —
# the debt is real and this is the number that has to walk to zero.

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

MANIFEST=tests/CAD_CORPUS.tsv
LEDGER=tools/golden_intent.txt
OUT=${OUT:-$(mktemp -d)}
mkdir -p "$OUT"

RANGE=${RANGE:-6..28}
STRIDE=3
FACE_STRIDE=8
UPDATE=0
ONLY=""
for arg in "$@"; do
    case "$arg" in
        --full) STRIDE=1; FACE_STRIDE=1 ;;
        --update) UPDATE=1 ;;
        --model=*) ONLY="${arg#--model=}" ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

FAIL=0
: > "$OUT/intent.txt"
SWEPT_PATHS=" "

while IFS= read -r row; do
    name=$(corpus_field "$row" 1)
    tier=$(corpus_field "$row" 2)
    rel=$(corpus_field "$row" 3)
    fast=$(corpus_field "$row" 4)
    wt=$(corpus_field "$row" 7)

    [[ -n "$ONLY" && "$name" != "$ONLY" ]] && continue
    # Density editing only makes sense where the base mesh is a closed solid
    # the artist would actually work on; open-shell diagnostics are excluded.
    [[ "$fast" == "1" ]] || continue
    [[ "$wt" == "1" ]] || continue
    case "$tier" in
        release|fixture|regression) ;;
        *) continue ;;
    esac

    step="$ROOT/tests/$rel"
    ensure_fixture_step "$WEFT" "$step" "$name" "$tier" || true
    [[ -f "$step" ]] || { echo "skip $name (missing tests/$rel)"; continue; }
    # Several manifest rows are class aliases of one file (foam_wt_open_nm is
    # foam). Sweeping a 846-face model twice buys nothing and dominates the
    # gate's runtime, so sweep each path once.
    if [[ "$SWEPT_PATHS" == *" $rel "* ]]; then
        echo "skip $name (same file as an earlier case: tests/$rel)"
        continue
    fi
    SWEPT_PATHS="$SWEPT_PATHS$rel "

    log="$OUT/$name-intent.log"
    "$WEFT" intent-sweep "$step" --profile cad --range "$RANGE" \
        --stride "$STRIDE" --face-stride "$FACE_STRIDE" > "$log" 2>&1
    line=$(grep -Eo 'faces=[0-9]+ runs=[0-9]+ self-lost=[0-9]+ neighbour-lost=[0-9]+ leaks=[0-9]+ winding=[0-9]+ throws=[0-9]+' "$log" | head -1 || true)
    if [[ -z "$line" ]]; then
        echo "FAIL $name: intent-sweep produced no summary (see $log)"
        FAIL=1
        continue
    fi
    echo "$name cad $line" >> "$OUT/intent.txt"
    causes=$(grep '^intent-causes:' "$log" || true)
    [[ -n "$causes" ]] && echo "  $name: $causes"
done < <(corpus_rows "$MANIFEST")

if [[ "$UPDATE" == 1 ]]; then
    if [[ -f "$LEDGER" ]]; then
        corpus_merge_counts "$LEDGER" "$OUT/intent.txt" "$OUT/ledger.new"
        cp "$OUT/ledger.new" "$LEDGER"
    else
        cp "$OUT/intent.txt" "$LEDGER"
    fi
    echo "intent ledger updated: $LEDGER"
    exit 0
fi

if [[ -f "$LEDGER" ]]; then
    if ! corpus_ratchet_intent "$LEDGER" "$OUT/intent.txt" > "$OUT/intent.diff"; then
        echo "FAIL artist-intent regressed:"
        cat "$OUT/intent.diff"
        FAIL=1
    elif [[ -s "$OUT/intent.diff" ]]; then
        echo "intent ratchet notes (run --update to bank improvements):"
        cat "$OUT/intent.diff"
    fi
else
    echo "note: no intent ledger yet — run with --update to create it"
fi

if [[ "$FAIL" == 0 ]]; then
    echo "intent gate: PASS"
else
    echo "intent gate: FAIL"
fi
exit "$FAIL"
