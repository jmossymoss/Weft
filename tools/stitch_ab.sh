#!/usr/bin/env bash
# AD-2 stitch A/B across the release set (CAD_CORPUS.tsv tier=release).
# Meshes each release model with and without --stitch at default and cad
# profiles; captures open / non-manifold / raw / empty / quad / tri / ngon
# from validate output. Does not alter corpus_gate or promote stitch.
#
# Usage:
#   tools/stitch_ab.sh
#   WEFT=build/cli/weft OUT=docs/evidence/stitch-ab-raw tools/stitch_ab.sh
#
# Emits:
#   $OUT/stitch_ab.tsv   machine-readable rows
#   $OUT/<model>-<profile>-{baseline,stitch}.log
# And prints a markdown-friendly summary table to stdout.

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
if [[ ! -x "$WEFT" ]]; then
    echo "error: weft binary not found (set WEFT=...)" >&2
    exit 2
fi

MANIFEST=tests/CAD_CORPUS.tsv
OUT=${OUT:-$(mktemp -d /tmp/weft-stitch-ab.XXXXXX)}
mkdir -p "$OUT"
TSV="$OUT/stitch_ab.tsv"

# Parse one validate log into metric fields.
# Prints: open nm raw empty quads tris ngons verts polys exit
# Prefer bash [[ =~ ]] — sed '.*\([0-9]+\)' keeps only the last digit of
# multi-digit counts (greedy .* eats the leading digits).
parse_log() {
    local log="$1" rc="$2"
    local open=NA nm=NA raw=NA empty=NA quads=NA tris=NA ngons=NA verts=NA polys=NA
    if [[ -f "$log" ]]; then
        local line
        line=$(grep -E 'watertight:' "$log" | head -1 || true)
        if [[ "$line" =~ open\ edges:\ ([0-9]+),\ non-manifold:\ ([0-9]+) ]]; then
            open="${BASH_REMATCH[1]}"
            nm="${BASH_REMATCH[2]}"
        fi
        line=$(grep -E 'demoted:' "$log" | head -1 || true)
        if [[ "$line" =~ ([0-9]+)\ to\ raw\ triangulation,\ ([0-9]+)\ emitted\ nothing ]]; then
            raw="${BASH_REMATCH[1]}"
            empty="${BASH_REMATCH[2]}"
        else
            raw=0
            empty=0
        fi
        line=$(grep -E 'vertices, .* polygons' "$log" | head -1 || true)
        if [[ "$line" =~ ([0-9]+)\ vertices,\ ([0-9]+)\ polygons\ \(([0-9]+)\ quads,\ ([0-9]+)\ tris,\ ([0-9]+)\ n-gons\) ]]; then
            verts="${BASH_REMATCH[1]}"
            polys="${BASH_REMATCH[2]}"
            quads="${BASH_REMATCH[3]}"
            tris="${BASH_REMATCH[4]}"
            ngons="${BASH_REMATCH[5]}"
        fi
    fi
    printf '%s %s %s %s %s %s %s %s %s %s\n' \
        "$open" "$nm" "$raw" "$empty" "$quads" "$tris" "$ngons" "$verts" "$polys" "$rc"
}

run_one() {
    local name="$1" step="$2" profile="$3" mode="$4"
    local tag="${name}-${profile}-${mode}"
    local log="$OUT/${tag}.log"
    local args=()
    if [[ "$profile" == "cad" ]]; then
        args+=(--profile cad)
    fi
    if [[ "$mode" == "stitch" ]]; then
        args+=(--stitch)
    fi
    # validate exits 1 when not watertight; still parse metrics
    local rc=0
    "$WEFT" validate "$step" "${args[@]}" >"$log" 2>&1 || rc=$?
    parse_log "$log" "$rc"
}

delta() {
    local a="$1" b="$2"
    if [[ "$a" == "NA" || "$b" == "NA" ]]; then
        printf 'NA'
        return
    fi
    printf '%+d' $((b - a))
}

echo "stitch A/B (AD-2) — release tier from $MANIFEST"
echo "WEFT=$WEFT  OUT=$OUT"
echo

printf '%s\n' \
    "model	profile	mode	open	nm	raw	empty	quads	tris	ngons	verts	polys	exit" \
    >"$TSV"

# Collect rows for the markdown table printed at the end.
declare -a SUMMARY_ROWS=()

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
        echo "SKIP $name: missing tests/$rel" >&2
        continue
    fi

    for profile in default cad; do
        echo "=== $name / $profile ==="
        read -r b_open b_nm b_raw b_empty b_quads b_tris b_ngons b_verts b_polys b_rc \
            < <(run_one "$name" "$step" "$profile" baseline)
        printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
            "$name" "$profile" "baseline" \
            "$b_open" "$b_nm" "$b_raw" "$b_empty" \
            "$b_quads" "$b_tris" "$b_ngons" "$b_verts" "$b_polys" "$b_rc" \
            >>"$TSV"
        echo "  baseline: open=$b_open nm=$b_nm raw=$b_raw empty=$b_empty q/t/n=$b_quads/$b_tris/$b_ngons exit=$b_rc"

        read -r s_open s_nm s_raw s_empty s_quads s_tris s_ngons s_verts s_polys s_rc \
            < <(run_one "$name" "$step" "$profile" stitch)
        printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
            "$name" "$profile" "stitch" \
            "$s_open" "$s_nm" "$s_raw" "$s_empty" \
            "$s_quads" "$s_tris" "$s_ngons" "$s_verts" "$s_polys" "$s_rc" \
            >>"$TSV"
        echo "  stitch:   open=$s_open nm=$s_nm raw=$s_raw empty=$s_empty q/t/n=$s_quads/$s_tris/$s_ngons exit=$s_rc"

        d_open=$(delta "$b_open" "$s_open")
        d_nm=$(delta "$b_nm" "$s_nm")
        d_raw=$(delta "$b_raw" "$s_raw")
        d_empty=$(delta "$b_empty" "$s_empty")
        d_quads=$(delta "$b_quads" "$s_quads")
        d_tris=$(delta "$b_tris" "$s_tris")
        d_ngons=$(delta "$b_ngons" "$s_ngons")

        # Per-row recommendation hint (final AD-2 call is in the evidence doc).
        # Correctness failure = new/worse open, nm, raw, or empty vs baseline.
        hint="keep_quarantined"
        if [[ "$d_open" != "NA" && "$d_nm" != "NA" && "$d_raw" != "NA" && "$d_empty" != "NA" ]]; then
            worse=0
            better=0
            # numeric compare without leading +
            [[ "${d_open#+}" -gt 0 ]] && worse=1
            [[ "${d_nm#+}" -gt 0 ]] && worse=1
            [[ "${d_raw#+}" -gt 0 ]] && worse=1
            [[ "${d_empty#+}" -gt 0 ]] && worse=1
            [[ "${d_open#+}" -lt 0 ]] && better=1
            [[ "${d_nm#+}" -lt 0 ]] && better=1
            if [[ "$worse" -eq 1 ]]; then
                hint="regress_correctness"
            elif [[ "$better" -eq 1 ]]; then
                hint="correctness_improved"
            else
                hint="neutral_correctness"
            fi
        fi

        SUMMARY_ROWS+=("$(printf '%s|%s|o=%s nm=%s raw=%s empty=%s q/t/n=%s/%s/%s|o=%s nm=%s raw=%s empty=%s q/t/n=%s/%s/%s|Δo=%s Δnm=%s Δraw=%s Δempty=%s Δq/t/n=%s/%s/%s|%s' \
            "$name" "$profile" \
            "$b_open" "$b_nm" "$b_raw" "$b_empty" "$b_quads" "$b_tris" "$b_ngons" \
            "$s_open" "$s_nm" "$s_raw" "$s_empty" "$s_quads" "$s_tris" "$s_ngons" \
            "$d_open" "$d_nm" "$d_raw" "$d_empty" "$d_quads" "$d_tris" "$d_ngons" \
            "$hint")")
    done
done < <(corpus_rows "$MANIFEST")

echo
echo "## Summary table"
echo
echo "| model | profile | baseline | stitch | delta (stitch−baseline) | row hint |"
echo "| --- | --- | --- | --- | --- | --- |"
for r in "${SUMMARY_ROWS[@]}"; do
    IFS='|' read -r model profile baseline stitch delta hint <<<"$r"
    echo "| $model | $profile | $baseline | $stitch | $delta | $hint |"
done

echo
echo "TSV: $TSV"
echo "Logs: $OUT/"
echo
echo "AD-2: promote only with consistent advantage AND no new correctness"
echo "failures on the release set. Row hints are per-cell; overall call"
echo "belongs in the evidence document."
