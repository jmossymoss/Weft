#!/usr/bin/env bash
# Fetch or link public corpus subsets into tests/public_corpus/_cache.
# Does not commit downloaded data. See tests/public_corpus/README.md.

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CACHE="${WEFT_PUBLIC_CORPUS_ROOT:-$ROOT/tests/public_corpus/_cache}"
mkdir -p "$CACHE"

usage() {
    echo "usage: $0 {fusion360-smoke|abc-nightly|nist|mambo|status}" >&2
    exit 2
}

[[ $# -ge 1 ]] || usage
cmd=$1

case "$cmd" in
status)
    echo "cache root: $CACHE"
    for d in fusion360 abc nist mambo; do
        if [[ -d "$CACHE/$d" ]]; then
            n=$(find "$CACHE/$d" -type f \( -name '*.step' -o -name '*.stp' -o -name '*.STEP' \) 2>/dev/null | wc -l | tr -d ' ')
            echo "  $d: $n STEP files"
        else
            echo "  $d: (missing)"
        fi
    done
    ;;
fusion360-smoke)
    dest="$CACHE/fusion360"
    mkdir -p "$dest/pending" "$dest/archives"
    echo "Fusion 360 Gallery Extended STEP (s2.0.1_extended_step, ~483 MB):"
    echo "  https://fusion-360-gallery-dataset.s3.us-west-2.amazonaws.com/segmentation/s2.0.1/s2.0.1_extended_step.zip"
    echo "Download manually into $dest/archives/ then unpack and sample into"
    echo "paths listed by tests/public_corpus/fusion360_smoke.tsv."
    echo "Docs: https://github.com/AutodeskAILab/Fusion360GalleryDataset"
    if [[ "${WEFT_FETCH_FUSION_ZIP:-}" == "1" ]]; then
        zip="$dest/archives/s2.0.1_extended_step.zip"
        if [[ ! -f "$zip" ]]; then
            curl -L -o "$zip" \
              "https://fusion-360-gallery-dataset.s3.us-west-2.amazonaws.com/segmentation/s2.0.1/s2.0.1_extended_step.zip"
        fi
        echo "archive ready: $zip (sample stratified IDs before enabling WEFT_RUN_PUBLIC)"
    fi
    ;;
abc-nightly)
    mkdir -p "$CACHE/abc/pending"
    if [[ -z "${WEFT_ABC_ROOT:-}" ]]; then
        echo "Set WEFT_ABC_ROOT to a local ABC STEP tree, then re-run."
        echo "Upstream: https://deep-geometry.github.io/abc-dataset/"
        exit 1
    fi
    echo "ABC root: $WEFT_ABC_ROOT"
    echo "Copy or symlink a stratified nightly sample into $CACHE/abc/ matching"
    echo "tests/public_corpus/abc_nightly.tsv."
    ;;
nist)
    # Dedicated fetcher avoids merge conflicts with other corpus layers.
    exec "$ROOT/tools/fetch_nist_corpus.sh"
    ;;
mambo)
    mkdir -p "$CACHE/mambo/pending"
    echo "Clone MAMBO and copy selected STEP files into $CACHE/mambo/"
    echo "  git clone https://gitlab.com/franck.ledoux/mambo.git"
    ;;
*)
    usage
    ;;
esac
