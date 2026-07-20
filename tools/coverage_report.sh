#!/usr/bin/env bash
# Report geometry-zoo / interaction / dirty coverage from CAD_CORPUS.tsv
# against tests/COVERAGE_MATRIX.tsv. Exit 1 if any required tag is missing.

set -u
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=corpus_common.sh
source "$SCRIPT_DIR/corpus_common.sh"
ROOT="$(corpus_root)"
cd "$ROOT"

MANIFEST=tests/CAD_CORPUS.tsv
MATRIX=tests/COVERAGE_MATRIX.tsv
FAIL=0

# Collect tag tokens from surfaces/curves/features columns (10-12, 1-based).
covered=""
while IFS= read -r row; do
    for idx in 11 12 13; do
        field=$(corpus_field "$row" "$idx")
        IFS=',' read -ra toks <<< "$field"
        for t in "${toks[@]}"; do
            t=$(echo "$t" | tr -d ' ')
            [[ -n "$t" && "$t" != "-" ]] || continue
            covered="$covered|$t|"
        done
    done
done < <(corpus_rows "$MANIFEST")

echo "coverage report (CAD_CORPUS vs COVERAGE_MATRIX)"
while IFS= read -r line; do
    [[ "$line" == tag$'\t'* || "$line" == tag* ]] && continue
    [[ -z "$line" || "$line" == \#* ]] && continue
    tag=$(printf '%s\n' "$line" | awk -F'\t' '{print $1}')
    kind=$(printf '%s\n' "$line" | awk -F'\t' '{print $2}')
    key="$tag"
    if [[ "$covered" == *"|$key|"* ]]; then
        echo "  OK  $kind/$tag"
    else
        echo "  MISSING  $kind/$tag"
        FAIL=1
    fi
done < "$MATRIX"

if [[ "$FAIL" == 0 ]]; then
    echo "coverage report: PASS"
else
    echo "coverage report: FAIL"
fi
exit "$FAIL"
