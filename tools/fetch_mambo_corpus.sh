#!/usr/bin/env bash
# Fetch MAMBO stress STEP subset into tests/public_corpus/_cache/mambo/.
# Does not commit downloaded data. See tests/public_corpus/MAMBO.md.
#
# Usage:
#   tools/fetch_mambo_corpus.sh
#   tools/fetch_public_corpus.sh mambo   # thin wrapper
#
# Env:
#   WEFT_PUBLIC_CORPUS_ROOT — cache root (default tests/public_corpus/_cache)
#   WEFT_MAMBO_ROOT         — existing MAMBO checkout; skips clone when set
#   WEFT_MAMBO_REPO         — override clone URL
#   WEFT_MAMBO_KEEP_SRC=1   — keep full sparse clone under mambo/src (default: keep)

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CACHE="${WEFT_PUBLIC_CORPUS_ROOT:-$ROOT/tests/public_corpus/_cache}"
DEST="$CACHE/mambo"
MANIFEST="$ROOT/tests/public_corpus/mambo_stress.tsv"
REPO_URL="${WEFT_MAMBO_REPO:-https://gitlab.com/franck.ledoux/mambo.git}"
SRC_DIR="$DEST/src"

mkdir -p "$DEST"

resolve_src() {
    if [[ -n "${WEFT_MAMBO_ROOT:-}" ]]; then
        if [[ ! -d "$WEFT_MAMBO_ROOT" ]]; then
            echo "WEFT_MAMBO_ROOT is not a directory: $WEFT_MAMBO_ROOT" >&2
            exit 1
        fi
        echo "Using WEFT_MAMBO_ROOT=$WEFT_MAMBO_ROOT" >&2
        printf '%s\n' "$WEFT_MAMBO_ROOT"
        return
    fi

    if [[ -d "$SRC_DIR/.git" ]]; then
        echo "Reusing sparse clone: $SRC_DIR" >&2
        # Refresh sparse paths in case an older clone lacked Medium/.
        git -C "$SRC_DIR" sparse-checkout set Basic Simple Medium 2>/dev/null || true
        printf '%s\n' "$SRC_DIR"
        return
    fi

    echo "Shallow sparse-clone MAMBO -> $SRC_DIR" >&2
    echo "  $REPO_URL" >&2
    rm -rf "$SRC_DIR"
    mkdir -p "$DEST"
    # Blobless sparse clone: only Basic/Simple/Medium (skip Misc/Evolutive bulk).
    if git clone --depth 1 --filter=blob:none --sparse "$REPO_URL" "$SRC_DIR"; then
        git -C "$SRC_DIR" sparse-checkout set Basic Simple Medium
    else
        echo "sparse clone failed; falling back to full shallow clone" >&2
        rm -rf "$SRC_DIR"
        git clone --depth 1 "$REPO_URL" "$SRC_DIR"
    fi
    printf '%s\n' "$SRC_DIR"
}

upstream_rel_from_cache_path() {
    # Manifest paths are cache-relative: mambo/Basic/B0.step -> Basic/B0.step
    local rel=$1
    if [[ "$rel" == mambo/* ]]; then
        printf '%s\n' "${rel#mambo/}"
    else
        printf '%s\n' "$rel"
    fi
}

copy_manifest_cases() {
    local src=$1
    local copied=0
    local missing=0

    if [[ ! -f "$MANIFEST" ]]; then
        echo "missing manifest: $MANIFEST" >&2
        exit 1
    fi

    while IFS=$'\t' read -r id local_path validity notes; do
        [[ -z "${id:-}" || "$id" == \#* ]] && continue
        # Skip header
        [[ "$id" == "id" ]] && continue
        [[ -z "${local_path:-}" ]] && continue
        if [[ "$local_path" == *"/pending/"* ]] || [[ "${notes:-}" == *placeholder* ]]; then
            echo "skip $id (placeholder path)"
            continue
        fi

        local urel
        urel=$(upstream_rel_from_cache_path "$local_path")
        local from="$src/$urel"
        local to="$CACHE/$local_path"

        if [[ ! -f "$from" ]]; then
            echo "missing upstream file for $id: $from" >&2
            missing=$((missing + 1))
            continue
        fi
        mkdir -p "$(dirname "$to")"
        cp -f "$from" "$to"
        echo "ok $id -> $local_path (${validity:-?})"
        copied=$((copied + 1))
    done < <(awk -F'\t' 'NF >= 2 && $1 !~ /^#/ { print }' "$MANIFEST")

    echo "MAMBO fetch: copied $copied manifest cases into $DEST/"
    if [[ "$missing" -gt 0 ]]; then
        echo "MAMBO fetch: $missing upstream files missing" >&2
        exit 1
    fi
    if [[ "$copied" -eq 0 ]]; then
        echo "MAMBO fetch: no cases copied; check $MANIFEST" >&2
        exit 1
    fi
}

main() {
    echo "cache root: $CACHE"
    echo "manifest:   $MANIFEST"
    src=$(resolve_src)
    copy_manifest_cases "$src"
    echo "gate: WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh mambo"
}

main "$@"
