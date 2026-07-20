#!/usr/bin/env bash
# Shared CAD_CORPUS.tsv helpers for corpus and release gates.
# Sole case inventory: tests/CAD_CORPUS.tsv (see docs/EXECUTION_PLAN.md).

corpus_root() {
    cd "$(dirname "$0")/.."
    pwd
}

# Emit TSV data rows (no header). Fields are tab-separated.
corpus_rows() {
    local manifest="$1"
    awk -F'\t' 'NR > 1 && $1 != "" && $1 !~ /^#/ { print }' "$manifest"
}

# field indices (1-based): name tier path fast max_raw max_empty require_watertight visual notes
corpus_field() {
    local row="$1" idx="$2"
    printf '%s\n' "$row" | awk -F'\t' -v i="$idx" '{ print $i }'
}

ensure_fixture_step() {
    # $1 = weft binary, $2 = absolute step path, $3 = fixture name, $4 = tier
    local weft="$1" step="$2" name="$3" tier="$4"
    case "$tier" in
        fixture|dirty) ;;
        *) return 0 ;;
    esac
    if [[ -f "$step" ]]; then
        return 0
    fi
    mkdir -p "$(dirname "$step")"
    "$weft" fixture "$step" --shape "$name" >/dev/null
}
