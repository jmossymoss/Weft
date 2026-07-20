#!/usr/bin/env bash
# Fetch or link public corpus subsets into tests/public_corpus/_cache.
# Does not commit downloaded data. See tests/public_corpus/README.md.
#
# Broad-nightly / geometric-diversity public corpus: ABC Dataset (not Fusion,
# not local STEP_Examples). Fusion remains an optional mechanical-feature smoke.

set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CACHE="${WEFT_PUBLIC_CORPUS_ROOT:-$ROOT/tests/public_corpus/_cache}"
mkdir -p "$CACHE"

FUSION_ZIP_URL="https://fusion-360-gallery-dataset.s3.us-west-2.amazonaws.com/segmentation/s2.0.1/s2.0.1_extended_step.zip"
FUSION_ZIP_NAME="s2.0.1_extended_step.zip"

ABC_UPSTREAM="https://deep-geometry.github.io/abc-dataset/"
ABC_STEP_CHUNKS="https://deep-geometry.github.io/abc-dataset/data/step_v00.txt"
ABC_STAT_CHUNKS="https://deep-geometry.github.io/abc-dataset/data/stat_v00.txt"
ABC_META_CHUNKS="https://deep-geometry.github.io/abc-dataset/data/meta_v00.txt"
ABC_SIZE_YAML="https://deep-geometry.github.io/abc-dataset/data/size.yml"

usage() {
    echo "usage: $0 {abc-nightly|fusion360-smoke|nist|mambo|status}" >&2
    exit 2
}

[[ $# -ge 1 ]] || usage
cmd=$1

case "$cmd" in
status)
    echo "cache root: $CACHE"
    for d in abc fusion360 nist mambo; do
        if [[ -d "$CACHE/$d" ]]; then
            n=$(find "$CACHE/$d" -type f \( \
                -name '*.step' -o -name '*.stp' -o -name '*.STEP' -o -name '*.STP' \
                -o -name '*.brep' -o -name '*.brp' \) 2>/dev/null \
                | grep -v '/src/' | wc -l | tr -d ' ')
            echo "  $d: $n CAD files (excl. src clone)"
        else
            echo "  $d: (missing)"
        fi
    done
    if [[ -f "$CACHE/abc/nightly_status.json" ]]; then
        echo "  abc nightly_status.json:"
        cat "$CACHE/abc/nightly_status.json"
    fi
    if [[ -f "$CACHE/fusion360/smoke_status.json" ]]; then
        echo "  fusion360 smoke_status.json:"
        cat "$CACHE/fusion360/smoke_status.json"
    fi
    ;;
abc-nightly)
    dest="$CACHE/abc"
    mkdir -p "$dest/archives" "$dest/nightly"

    cat <<EOF
ABC Dataset — broad-nightly / geometric-diversity public corpus
  Upstream:     $ABC_UPSTREAM
  STEP chunks:  $ABC_STEP_CHUNKS
  Stats chunks: $ABC_STAT_CHUNKS   (~1–2 MB each; optional face/surf metadata)
  Meta chunks:  $ABC_META_CHUNKS   (~0.5–1 MB each)
  Sizes:        $ABC_SIZE_YAML

How to obtain ABC STEP (not committed; multi-GB per chunk):
  1. Agree to the ABC / Onshape terms on the upstream page.
  2. Download chunk URL lists, e.g.:
       curl -fsSL -o step_v00.txt '$ABC_STEP_CHUNKS'
  3. Fetch one or more STEP chunks (~0.8–1.6 GB each, 7z):
       # first chunk:
       sed '1q;d' step_v00.txt | xargs -n 2 sh -c 'curl -fL -o "\$1" "\$0"'
       # or parallel (see upstream docs):
       #   cat step_v00.txt | xargs -n 2 -P 4 sh -c 'curl --insecure -o step/\$1 \$0'
  4. Unpack into a local tree, e.g.:
       mkdir -p "\$WEFT_ABC_ROOT"
       7z x abc_0000_step_v00.7z -o"\$WEFT_ABC_ROOT"
     Layout is per-model folders (00000002/…_step_001.step). Optionally unpack
     matching stat_v00 chunks alongside for exact #surfs stratification.
  5. Point WEFT_ABC_ROOT at that tree and sample:

EOF

    if [[ -z "${WEFT_ABC_ROOT:-}" ]]; then
        echo "WEFT_ABC_ROOT is unset — writing expected-slot smoke list only."
        echo "  After unpacking ABC STEP locally:"
        echo "    export WEFT_ABC_ROOT=/path/to/abc/step"
        echo "    $0 abc-nightly"
        echo "  Or: tools/sample_abc_nightly.py"
        python3 "$ROOT/tools/sample_abc_nightly.py" --skeleton-only \
            --manifest "$ROOT/tests/public_corpus/abc_nightly.tsv"
        echo
        echo "Note: full STEP chunks are too large for a tiny CI download"
        echo "(~1.5 GB for abc_0000_step_v00.7z). No ABC STEP blobs are fetched here."
        echo "gate: WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh"
        exit 0
    fi

    if [[ ! -d "$WEFT_ABC_ROOT" ]]; then
        echo "WEFT_ABC_ROOT is not a directory: $WEFT_ABC_ROOT" >&2
        exit 1
    fi

    echo "ABC root: $WEFT_ABC_ROOT"
    echo "sampling stratified nightly subset into $dest/nightly/ ..."
    if ! python3 "$ROOT/tools/sample_abc_nightly.py" --root "$WEFT_ABC_ROOT" --cache "$CACHE"; then
        echo "sampler failed" >&2
        exit 1
    fi
    echo "manifest: tests/public_corpus/abc_nightly.tsv"
    echo "gate:     WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh"
    ;;
fusion360-smoke)
    dest="$CACHE/fusion360"
    mkdir -p "$dest/archives"
    zip="$dest/archives/$FUSION_ZIP_NAME"
    echo "Fusion 360 Gallery Extended STEP (optional mechanical-feature smoke)"
    echo "  $FUSION_ZIP_URL"
    echo "  Docs: https://github.com/AutodeskAILab/Fusion360GalleryDataset"
    echo "  Note: WP1 broad-nightly public corpus is ABC, not Fusion."

    if [[ "${WEFT_FETCH_FUSION_ZIP:-}" == "1" ]]; then
        if [[ ! -f "$zip" ]]; then
            echo "downloading -> $zip"
            if ! curl -fL --retry 3 --retry-delay 2 -o "$zip.partial" "$FUSION_ZIP_URL"; then
                rm -f "$zip.partial"
                echo "download failed; place $FUSION_ZIP_NAME under $dest/archives/ and re-run." >&2
                exit 1
            fi
            mv "$zip.partial" "$zip"
        else
            echo "archive already present: $zip"
        fi
    elif [[ ! -f "$zip" ]]; then
        echo "archive not present: $zip"
        echo "  Download with: WEFT_FETCH_FUSION_ZIP=1 $0 fusion360-smoke"
        echo "  Or curl -L -o $zip '$FUSION_ZIP_URL'"
        exit 1
    else
        echo "archive ready: $zip"
    fi

    if [[ ! -f "$zip" ]]; then
        echo "cannot sample without archive" >&2
        exit 1
    fi

    echo "sampling stratified smoke subset into $dest/smoke/ ..."
    if ! python3 "$ROOT/tools/sample_fusion360_smoke.py" --archive "$zip" --cache "$CACHE"; then
        echo "sampler failed — archive may be incomplete; re-download or unpack manually." >&2
        exit 1
    fi
    echo "manifest: tests/public_corpus/fusion360_smoke.tsv"
    echo "gate:     WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh fusion"
    ;;
nist)
    # Dedicated fetcher avoids merge conflicts with other corpus layers.
    exec "$ROOT/tools/fetch_nist_corpus.sh"
    ;;
mambo)
    # Dedicated fetcher avoids merge conflicts with other corpus layers.
    exec "$ROOT/tools/fetch_mambo_corpus.sh"
    ;;
*)
    usage
    ;;
esac
