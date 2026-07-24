#!/usr/bin/env bash
# Corpus gate — manifest-driven release invariants and topology regression.
#
# Selects cases from tests/CAD_CORPUS.tsv only (no hardcoded fixture or STEP
# lists). Generates missing fixture-tier STEP files via `weft fixture`.
#
#   tools/corpus_gate.sh              run the gate
#   tools/corpus_gate.sh --update     rewrite golden counts from this run
#   tools/corpus_gate.sh --no-golden  invariants only (cross-platform CI)
#   tools/corpus_gate.sh --fast       only rows with fast=1
#
# Known-red allowances: when tests/KNOWN_RED.tsv lists an exact
# (name,profile,metric,ceiling) row, a matching failure is recorded but does
# not fail this gate. The strict release gate never reads that file.
#
# Exit: 0 clean (after known-red), 1 any unexpected invariant or count drift.

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
KNOWN_RED=tests/KNOWN_RED.tsv
GOLDEN=tools/golden_counts.txt
STRUCTURE=tools/golden_structure.txt
OUT=${OUT:-$(mktemp -d)}
mkdir -p "$OUT"
UPDATE=0
CHECK_GOLDEN=1
FAST_ONLY=0
for arg in "$@"; do
    case "$arg" in
        --update) UPDATE=1 ;;
        --no-golden) CHECK_GOLDEN=0 ;;
        --fast) FAST_ONLY=1 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

FAIL=0
: > "$OUT/counts.txt"
: > "$OUT/structure.txt"
: > "$OUT/failures.txt"

known_red_allows() {
    # $1=name $2=profile $3=metric $4=observed_int
    local name="$1" profile="$2" metric="$3" observed="$4"
    [[ -f "$KNOWN_RED" ]] || return 1
    awk -F'\t' -v n="$name" -v p="$profile" -v m="$metric" -v o="$observed" '
        NR == 1 { next }
        $1 == n && $2 == p && $3 == m {
            if (o + 0 <= $4 + 0) { found = 1 }
        }
        END { exit found ? 0 : 1 }
    ' "$KNOWN_RED"
}

record_fail() {
    local name="$1" profile="$2" metric="$3" observed="$4" detail="$5"
    if known_red_allows "$name" "$profile" "$metric" "$observed"; then
        echo "KNOWN_RED $name [$profile] $metric=$observed — $detail"
        return 0
    fi
    echo "FAIL $name [$profile]: $detail"
    echo "$name	$profile	$metric	$observed	$detail" >> "$OUT/failures.txt"
    FAIL=1
}

run_one() {
    local name="$1" file="$2" args="$3" tag="$4" wt="$5" max_raw="$6" max_empty="$7"
    local log="$OUT/$name-$tag.log"
    local timeout_bin=timeout
    command -v timeout >/dev/null 2>&1 || timeout_bin=""
    if [[ -n "$timeout_bin" ]]; then
        timeout 1200 "$WEFT" mesh "$file" -o "$OUT/$name-$tag.obj" \
            $args --validate > "$log" 2>&1
    else
        "$WEFT" mesh "$file" -o "$OUT/$name-$tag.obj" \
            $args --validate > "$log" 2>&1
    fi
    local rc=$?
    if [[ "$rc" != 0 ]]; then
        if [[ "$wt" == "1" ]]; then
            record_fail "$name" "$tag" "exit_code" "$rc" "exit $rc (see $log)"
        else
            echo "note $name [$tag]: exit $rc allowed (require_watertight=0)"
        fi
    fi
    local stats
    stats=$(grep -Eo '[0-9]+ quads, [0-9]+ tris, [0-9]+ n-gons' "$log" | head -1 || true)
    echo "$name $tag ${stats:-0 quads, 0 tris, 0 n-gons}" >> "$OUT/counts.txt"

    # Structure retention (weft::formatStructure): did faces keep the topology
    # their plan chose? Watertightness cannot see this, so it is ratcheted
    # separately from the pinned polygon counts.
    local structure
    structure=$(grep -Eo 'faces=[0-9]+ structured=[0-9]+ planned-floor=[0-9]+ failed-floor=[0-9]+ raw=[0-9]+ empty=[0-9]+' "$log" | head -1 || true)
    if [[ -n "$structure" ]]; then
        echo "$name $tag $structure" >> "$OUT/structure.txt"
    fi

    local demo
    demo=$(grep "demoted:" "$log" || true)
    if [[ -n "$demo" ]] && ! echo "$demo" | grep -q "0 to raw triangulation, 0 emitted nothing"; then
        local raw_n empty_n
        raw_n=$(echo "$demo" | grep -Eo '[0-9]+ to raw' | head -1 | grep -Eo '[0-9]+' || echo 999)
        empty_n=$(echo "$demo" | grep -Eo '[0-9]+ emitted' | head -1 | grep -Eo '[0-9]+' || echo 999)
        if [[ "$raw_n" -gt "$max_raw" ]]; then
            record_fail "$name" "$tag" "max_raw" "$raw_n" "$demo"
        fi
        if [[ "$empty_n" -gt "$max_empty" ]]; then
            record_fail "$name" "$tag" "max_empty" "$empty_n" "$demo"
        fi
    fi
}

if [[ ! -f "$MANIFEST" ]]; then
    echo "FAIL: missing $MANIFEST" >&2
    exit 1
fi
if [[ ! -x "$WEFT" && ! -f "$WEFT" ]]; then
    echo "FAIL: weft binary not found at $WEFT" >&2
    exit 1
fi

while IFS= read -r row; do
    name=$(corpus_field "$row" 1)
    tier=$(corpus_field "$row" 2)
    rel=$(corpus_field "$row" 3)
    fast=$(corpus_field "$row" 4)
    max_raw=$(corpus_field "$row" 5)
    max_empty=$(corpus_field "$row" 6)
    wt=$(corpus_field "$row" 7)

    if [[ "$FAST_ONLY" == 1 && "$fast" != "1" ]]; then
        continue
    fi
    # Default gate runs fixture + release + fast stress; slow stress needs --all
    # via release/stress scripts. Skip fast=0 here unless explicitly wanted.
    if [[ "$fast" != "1" ]]; then
        continue
    fi

    step="$ROOT/tests/$rel"
    if ! ensure_fixture_step "$WEFT" "$step" "$name" "$tier"; then
        record_fail "$name" "setup" "fixture" 1 "could not generate $step"
        continue
    fi
    if [[ ! -f "$step" ]]; then
        record_fail "$name" "setup" "missing_path" 1 "path missing: tests/$rel"
        continue
    fi

    run_one "$name" "$step" "--profile cad" cad "$wt" "$max_raw" "$max_empty"
    run_one "$name" "$step" "" default "$wt" "$max_raw" "$max_empty"
done < <(corpus_rows "$MANIFEST")

if [[ "${UPDATE:-0}" == 1 ]]; then
    if [[ -f "$GOLDEN" ]]; then
        corpus_merge_counts "$GOLDEN" "$OUT/counts.txt" "$OUT/golden.new"
        cp "$OUT/golden.new" "$GOLDEN"
    else
        cp "$OUT/counts.txt" "$GOLDEN"
    fi
    echo "golden counts updated: $GOLDEN"
    if [[ -f "$STRUCTURE" ]]; then
        corpus_merge_counts "$STRUCTURE" "$OUT/structure.txt" "$OUT/structure.new"
        cp "$OUT/structure.new" "$STRUCTURE"
    else
        cp "$OUT/structure.txt" "$STRUCTURE"
    fi
    echo "structure ratchet updated: $STRUCTURE"
elif [[ "$CHECK_GOLDEN" == 0 ]]; then
    echo "golden count diff skipped (invariants-only run)"
elif [[ -f "$GOLDEN" ]]; then
    if ! corpus_diff_counts "$GOLDEN" "$OUT/counts.txt" > "$OUT/counts.diff"; then
        echo "FAIL golden counts moved (compared by case key):"
        cat "$OUT/counts.diff"
        FAIL=1
    fi
else
    echo "note: no golden table yet — run with --update to create it"
fi

# Structure ratchet runs even on --no-golden: it is an invariant (planned
# topology must not be lost), not a platform-dependent count.
if [[ "${UPDATE:-0}" != 1 && -f "$STRUCTURE" && -s "$OUT/structure.txt" ]]; then
    if ! corpus_ratchet_structure "$STRUCTURE" "$OUT/structure.txt" \
            > "$OUT/structure.diff"; then
        echo "FAIL structure retention regressed:"
        cat "$OUT/structure.diff"
        FAIL=1
    elif [[ -s "$OUT/structure.diff" ]]; then
        echo "structure ratchet notes (run --update to bank improvements):"
        cat "$OUT/structure.diff"
    fi
fi

if [[ "$FAIL" == 0 ]]; then
    echo "corpus gate: PASS"
else
    echo "corpus gate: FAIL"
    if [[ -s "$OUT/failures.txt" ]]; then
        echo "unexpected failures:"
        cat "$OUT/failures.txt"
    fi
fi
exit "$FAIL"
