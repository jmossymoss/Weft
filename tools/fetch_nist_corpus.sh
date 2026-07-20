#!/usr/bin/env bash
# Fetch NIST / CAx-IF interoperability STEP samples into
# tests/public_corpus/_cache/nist/. Does not commit downloaded data.
# See tests/public_corpus/NIST.md.
#
# Usage:
#   tools/fetch_nist_corpus.sh
#   tools/fetch_public_corpus.sh nist   # thin wrapper
#
# Env:
#   WEFT_PUBLIC_CORPUS_ROOT — cache root (default tests/public_corpus/_cache)
#   WEFT_NIST_FORCE=1       — re-download archives even when present

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CACHE="${WEFT_PUBLIC_CORPUS_ROOT:-$ROOT/tests/public_corpus/_cache}"
DEST="$CACHE/nist"
ARCH="$DEST/archives"
MANIFEST="$ROOT/tests/public_corpus/nist_interop.tsv"

# Official NIST download page:
# https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0
PMI_URL="https://www.nist.gov/system/files/documents/noindex/2024/06/19/NIST-PMI-STEP-Files.zip"
PMI_ZIP="NIST-PMI-STEP-Files.zip"
PMI_SHA256="8fa78429e6d8d9b0d7681d223b6aa9ec98c3772185c55b1a0e3679b21c181911"

# Design-to-Manufacturing-and-Inspection models (linked from the same NIST page).
D2MI_URL="https://www.nist.gov/system/files/documents/el/msid/infotest/NIST-D2MI-Models.zip"
D2MI_ZIP="NIST-D2MI-Models.zip"
D2MI_SHA256="f20e36fb68633129dfedadf209ba4836bb0e42e18f12e7a2c28140ecdfb26cf3"

mkdir -p "$ARCH" "$DEST/ap203" "$DEST/ap242" "$DEST/d2mi"

download_checked() {
    local url=$1
    local dest=$2
    local expect_sha=$3
    local force=${WEFT_NIST_FORCE:-0}

    if [[ -f "$dest" && "$force" != "1" ]]; then
        echo "archive present: $dest"
    else
        echo "downloading -> $dest"
        echo "  $url"
        if ! curl -fL --retry 3 --retry-delay 2 -o "$dest.partial" "$url"; then
            rm -f "$dest.partial"
            echo "download failed: $url" >&2
            exit 1
        fi
        mv "$dest.partial" "$dest"
    fi

    local got
    got=$(sha256sum "$dest" | awk '{print $1}')
    if [[ "$got" != "$expect_sha" ]]; then
        echo "SHA-256 mismatch for $dest" >&2
        echo "  expected: $expect_sha" >&2
        echo "  got:      $got" >&2
        echo "Re-run with WEFT_NIST_FORCE=1 after removing the bad archive." >&2
        exit 1
    fi
    printf '%s  %s\n' "$got" "$(basename "$dest")" >"$dest.sha256"
    echo "checksum ok: $got"
}

extract_member() {
    local zip=$1
    local member=$2
    local out_dir=$3
    local out_name=$4
    local out="$out_dir/$out_name"

    mkdir -p "$out_dir"
    if [[ -f "$out" && "${WEFT_NIST_FORCE:-0}" != "1" ]]; then
        echo "  keep $out"
        return
    fi
    # unzip -j drops paths; rename into a stable cache-relative name.
    local tmp
    tmp=$(mktemp -d)
    if ! unzip -joqq "$zip" "$member" -d "$tmp"; then
        rm -rf "$tmp"
        echo "extract failed: $member from $zip" >&2
        exit 1
    fi
    local extracted
    extracted=$(find "$tmp" -type f | head -n 1)
    if [[ -z "$extracted" ]]; then
        rm -rf "$tmp"
        echo "extract produced no file: $member" >&2
        exit 1
    fi
    mv -f "$extracted" "$out"
    rm -rf "$tmp"
    echo "  wrote $out"
}

echo "NIST / CAx-IF interoperability corpus"
echo "  index: https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0"
echo "  cache: $DEST"

download_checked "$PMI_URL" "$ARCH/$PMI_ZIP" "$PMI_SHA256"
download_checked "$D2MI_URL" "$ARCH/$D2MI_ZIP" "$D2MI_SHA256"

echo "extracting curated STEP subset ..."
extract_member "$ARCH/$PMI_ZIP" \
    "NIST-PMI-STEP-Files/AP203 geometry only/nist_ctc_01_asme1_rd.stp" \
    "$DEST/ap203" "nist_ctc_01_asme1_rd.stp"
extract_member "$ARCH/$PMI_ZIP" \
    "NIST-PMI-STEP-Files/AP203 geometry only/nist_ftc_07_asme1_rd.stp" \
    "$DEST/ap203" "nist_ftc_07_asme1_rd.stp"
extract_member "$ARCH/$PMI_ZIP" \
    "NIST-PMI-STEP-Files/AP203 geometry only/nist_ftc_08_asme1_rc.stp" \
    "$DEST/ap203" "nist_ftc_08_asme1_rc.stp"
extract_member "$ARCH/$PMI_ZIP" \
    "NIST-PMI-STEP-Files/nist_ctc_01_asme1_ap242-e1.stp" \
    "$DEST/ap242" "nist_ctc_01_asme1_ap242-e1.stp"
extract_member "$ARCH/$PMI_ZIP" \
    "NIST-PMI-STEP-Files/nist_stc_06_asme1_ap242-e3.stp" \
    "$DEST/ap242" "nist_stc_06_asme1_ap242-e3.stp"
extract_member "$ARCH/$D2MI_ZIP" \
    "NIST-D2MI-Models/827-9999-905.stp" \
    "$DEST/d2mi" "827-9999-905.stp"

missing=0
if [[ -f "$MANIFEST" ]]; then
    while IFS=$'\t' read -r id local_path validity notes; do
        [[ -z "${id:-}" || "$id" == \#* ]] && continue
        f="$CACHE/$local_path"
        if [[ ! -f "$f" ]]; then
            echo "missing manifest path: $local_path" >&2
            missing=$((missing + 1))
        fi
    done <"$MANIFEST"
fi

n=$(find "$DEST" -type f \( -name '*.stp' -o -name '*.step' -o -name '*.STEP' \) | wc -l | tr -d ' ')
echo "nist STEP files in cache: $n"
echo "manifest: tests/public_corpus/nist_interop.tsv"
if [[ "$missing" -ne 0 ]]; then
    echo "$missing manifest path(s) missing after fetch" >&2
    exit 1
fi
