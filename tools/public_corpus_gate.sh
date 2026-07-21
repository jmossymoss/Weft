#!/usr/bin/env bash
# Optional public-corpus gate — ABC Dataset broad-nightly first.
#
# Opt-in only:
#   WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh
#
# Skips cleanly (exit 0) when WEFT_RUN_PUBLIC is unset/0, or when listed
# STEP files are missing from tests/public_corpus/_cache/. Does not modify
# CAD_CORPUS.tsv or the strict release/corpus gates.
#
#   tools/public_corpus_gate.sh           # ABC nightly (default)
#   tools/public_corpus_gate.sh abc       # same
#   tools/public_corpus_gate.sh nist      # NIST/CAx-IF interop smoke
#   tools/public_corpus_gate.sh mambo     # MAMBO blocking/stress smoke
#   tools/public_corpus_gate.sh all       # ABC, then NIST, then MAMBO
#
# Validity-aware: watertight / demotion checks only when validity=closed_solid.
# research_stress (MAMBO Medium) may fail meshing without failing the gate.

set -u
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=corpus_common.sh
source "$SCRIPT_DIR/corpus_common.sh"
ROOT="$(corpus_root)"
cd "$ROOT"

CACHE="${WEFT_PUBLIC_CORPUS_ROOT:-$ROOT/tests/public_corpus/_cache}"
WEFT=${WEFT:-build/cli/weft}
if [[ ! -x "$WEFT" && -x build/bin/Release/weft.exe ]]; then
    WEFT=build/bin/Release/weft.exe
fi
if [[ ! -x "$WEFT" && -x build/vs2022/bin/Release/weft.exe ]]; then
    WEFT=build/vs2022/bin/Release/weft.exe
fi

SCOPE=${1:-abc}
case "$SCOPE" in
    abc|abc-nightly|nightly)
        MANIFESTS=(tests/public_corpus/abc_nightly.tsv)
        ;;
    nist|nist-interop|cax-if)
        MANIFESTS=(tests/public_corpus/nist_interop.tsv)
        ;;
    mambo|mambo-stress)
        MANIFESTS=(tests/public_corpus/mambo_stress.tsv)
        ;;
    all)
        # Public authority: ABC (diversity), NIST (interop), MAMBO (stress).
        MANIFESTS=(
            tests/public_corpus/abc_nightly.tsv
            tests/public_corpus/nist_interop.tsv
            tests/public_corpus/mambo_stress.tsv
        )
        ;;
    *)
        echo "usage: $0 [abc|nist|mambo|all]" >&2
        exit 2
        ;;
esac

if [[ "${WEFT_RUN_PUBLIC:-0}" != "1" ]]; then
    echo "public corpus gate: skipped (set WEFT_RUN_PUBLIC=1 to enable)"
    exit 0
fi

if [[ ! -x "$WEFT" && ! -f "$WEFT" ]]; then
    echo "public corpus gate: weft binary not found at $WEFT" >&2
    exit 1
fi

OUT=${OUT:-$(mktemp -d)}
mkdir -p "$OUT"
FAIL=0
RAN=0
MISSING=0

# Parse a public manifest row. Schemas differ by layer:
#   ABC:    id  face_band  local_path  validity  notes
#   Fusion: id  ops_band   face_band   local_path  validity  notes
#   NIST/MAMBO: id  local_path  validity  notes
# Find local_path (*.step/*.stp) and validity token; never hardcode model names.
parse_public_row() {
    local row="$1"
    local -a fields
    IFS=$'\t' read -r -a fields <<< "$row"
    name="${fields[0]:-}"
    rel=""
    validity=""
    notes=""
    local f
    for f in "${fields[@]}"; do
        case "$f" in
            *.step|*.stp|*.STEP|*.STP|*.brep|*.brp|*.BREP|*.BRP)
                rel=$f
                ;;
            closed_solid|open|invalid|research|research_stress)
                validity=$f
                ;;
        esac
    done
    # notes = last field when present
    if [[ ${#fields[@]} -ge 2 ]]; then
        notes="${fields[$((${#fields[@]} - 1))]}"
    fi
}

run_public_row() {
    local name="$1" rel="$2" validity="$3"
    local step="$CACHE/$rel"
    if [[ ! -f "$step" ]]; then
        echo "missing $name -> $step"
        MISSING=$((MISSING + 1))
        return 0
    fi
    local tag="public"
    local log="$OUT/$name-$tag.log"
    local wt=1
    if [[ "$validity" != "closed_solid" ]]; then
        wt=0
    fi
    local timeout_bin=timeout
    command -v timeout >/dev/null 2>&1 || timeout_bin=""
    local rc=0
    if [[ -n "$timeout_bin" ]]; then
        timeout 1200 "$WEFT" mesh "$step" -o "$OUT/$name-$tag.obj" \
            --profile cad --validate > "$log" 2>&1 || rc=$?
    else
        "$WEFT" mesh "$step" -o "$OUT/$name-$tag.obj" \
            --profile cad --validate > "$log" 2>&1 || rc=$?
    fi
    RAN=$((RAN + 1))
    if [[ "$rc" != 0 ]]; then
        if [[ "$wt" == "1" ]]; then
            echo "FAIL $name: exit $rc (see $log)"
            FAIL=1
        else
            echo "note $name: exit $rc allowed (validity=$validity)"
        fi
        return 0
    fi
    local demo
    demo=$(grep "demoted:" "$log" || true)
    if [[ "$wt" == "1" ]] && [[ -n "$demo" ]] && ! echo "$demo" | grep -q "0 to raw triangulation, 0 emitted nothing"; then
        echo "FAIL $name: $demo"
        FAIL=1
        return 0
    fi
    # Watertight expectation only for closed_solid sources.
    if [[ "$wt" == "1" ]]; then
        if grep -Eqi 'open edges: *[1-9]|non-manifold: *[1-9]|open_edges=[1-9]|non_manifold=[1-9]' "$log"; then
            echo "FAIL $name: open/non-manifold edges (see $log)"
            FAIL=1
            return 0
        fi
    fi
    local stats
    stats=$(grep -Eo '[0-9]+ quads, [0-9]+ tris, [0-9]+ n-gons' "$log" | head -1 || true)
    echo "ok $name ${stats:-}"
}

any_present=0
for manifest in "${MANIFESTS[@]}"; do
    if [[ ! -f "$manifest" ]]; then
        echo "note: missing manifest $manifest"
        continue
    fi
    echo "== $manifest =="
    while IFS= read -r row; do
        parse_public_row "$row"
        if [[ -z "$name" || -z "$rel" ]]; then
            echo "skip (unparsed row)"
            continue
        fi
        # Skip explicit pending/ placeholders only (expected_slot rows are tried).
        if [[ "$rel" == *"/pending/"* ]]; then
            echo "skip $name (pending placeholder)"
            continue
        fi
        if [[ -f "$CACHE/$rel" ]]; then
            any_present=1
        fi
        run_public_row "$name" "$rel" "${validity:-closed_solid}"
    done < <(awk -F'\t' 'NR > 1 && $1 != "" && $1 !~ /^#/ { print }' "$manifest")
done

if [[ "$any_present" == 0 ]]; then
    echo "public corpus gate: no cached CAD files for selected manifests"
    echo "  ABC:  export WEFT_ABC_ROOT=/path/to/abc && tools/fetch_public_corpus.sh abc-nightly"
    echo "  NIST: tools/fetch_nist_corpus.sh"
    echo "  MAMBO: tools/fetch_mambo_corpus.sh"
    exit 0
fi

if [[ "$MISSING" -gt 0 ]]; then
    echo "note: $MISSING listed paths missing under $CACHE (ran $RAN)"
fi

if [[ "$FAIL" == 0 ]]; then
    echo "public corpus gate: PASS ($RAN cases)"
else
    echo "public corpus gate: FAIL"
fi
exit "$FAIL"
