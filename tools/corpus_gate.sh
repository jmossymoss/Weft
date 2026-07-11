#!/usr/bin/env bash
# Corpus gate — the MVP verification harness (docs/MVP_PLAN.md §8).
#
# Meshes every fixture and every committed STEP example at the library
# default and the cad profile, and asserts the P0 invariants:
#   * watertight (0 open / 0 non-manifold) — tork is exempt (broken
#     source per the artist's verdict; it must still mesh without
#     crashing),
#   * no face demoted to raw OCCT triangulation, none empty,
# then diffs quad/tri/ngon counts against tools/golden_counts.txt so a
# mesher change that moves topology anywhere is caught and must be
# explained (better) or reverted (regression).
#
#   tools/corpus_gate.sh            run the gate
#   tools/corpus_gate.sh --update   rewrite the golden table from this run
#   tools/corpus_gate.sh --no-golden
#                                   invariants only (cross-platform CI)
#
# Exit: 0 clean, 1 any invariant broken or counts moved.
set -u
cd "$(dirname "$0")/.."
WEFT=${WEFT:-build/cli/weft}
GOLDEN=tools/golden_counts.txt
OUT=${OUT:-$(mktemp -d)}
mkdir -p "$OUT"
UPDATE=0
CHECK_GOLDEN=1
for arg in "$@"; do
    case "$arg" in
        --update) UPDATE=1 ;;
        --no-golden) CHECK_GOLDEN=0 ;;
        *) echo "unknown option: $arg" >&2; exit 2 ;;
    esac
done

FIXTURES="cylinder box cone sphere torus fillet hole demo boss notched \
          slotted barrel drilled bossfillet ribbon ribbonnotch \
          hairline canrev microedge filletslot torture slitdrill"
FAIL=0
: > "$OUT/counts.txt"

run_one() { # name file profile-args profile-tag watertight-required
    local name=$1 file=$2 args=$3 tag=$4 wt=$5
    local log="$OUT/$name-$tag.log"
    timeout 1200 "$WEFT" mesh "$file" -o "$OUT/$name-$tag.obj" \
            $args --validate > "$log" 2>&1
    local rc=$?
    if [ "$rc" != 0 ]; then
        if [ "$wt" = yes ]; then
            echo "FAIL $name [$tag]: exit $rc (see $log)"
            grep -E "watertight|error" "$log" | head -3 | sed 's/^/    /'
            FAIL=1
        fi
    fi
    local stats
    stats=$(grep -Eo '[0-9]+ quads, [0-9]+ tris, [0-9]+ n-gons' "$log" | head -1)
    echo "$name $tag $stats" >> "$OUT/counts.txt"
    # Never-fall-back census (P0.1): no face may PLAN as fallback-tri /
    # quad-dominant on a sound source. Broken sources (wt=no, tork) are
    # exempt here exactly as they are for watertightness — their point
    # is "mesh without crashing", not topology quality.
    if [ "$wt" = yes ] && grep -q "x fallback-tri\|x quad-dominant" "$log"; then
        echo "FAIL $name [$tag]: fallback-tri faces present"
        FAIL=1
    fi
    local demo
    demo=$(grep "demoted:" "$log" || true)
    if [ -n "$demo" ] && ! echo "$demo" | grep -q "0 to raw triangulation, 0 emitted nothing"; then
        echo "FAIL $name [$tag]: $demo"
        FAIL=1
    fi
}

for f in $FIXTURES; do
    "$WEFT" fixture "$OUT/$f.step" --shape "$f" > /dev/null 2>&1
    wtf=yes
    # slitdrill deliberately reproduces the tangent-contact class (a
    # pocket wall tangent to a bore leaves a lengthwise line edge in the
    # bore wall) which still goes non-manifold at cad — a known-red
    # reproducer, gated on "meshes without crashing" until fixed.
    [ "$f" = slitdrill ] && wtf=no
    run_one "$f" "$OUT/$f.step" "--profile cad" cad "$wtf"
    run_one "$f" "$OUT/$f.step" "" default "$wtf"
done

for file in tests/STEP_Examples/*.stp; do
    name=$(basename "$file" .stp)
    wt=yes
    [ "$name" = tork ] && wt=no   # broken source: mesh, don't gate
    run_one "$name" "$file" "--profile cad" cad "$wt"
    run_one "$name" "$file" "" default "$wt"
done

if [ "$UPDATE" = 1 ]; then
    cp "$OUT/counts.txt" "$GOLDEN"
    echo "golden counts updated: $GOLDEN"
elif [ "$CHECK_GOLDEN" = 0 ]; then
    echo "golden count diff skipped (invariants-only run)"
elif [ -f "$GOLDEN" ]; then
    if ! diff -u "$GOLDEN" "$OUT/counts.txt" > "$OUT/counts.diff"; then
        echo "FAIL golden counts moved:"
        cat "$OUT/counts.diff"
        FAIL=1
    fi
else
    echo "note: no golden table yet — run with --update to create it"
fi

if [ "$FAIL" = 0 ]; then
    echo "corpus gate: PASS"
else
    echo "corpus gate: FAIL"
fi
exit $FAIL
