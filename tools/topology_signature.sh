#!/usr/bin/env bash
# Cross-platform topology signature (EXECUTION_PLAN §3.2 / WP2).
#
# Emit a machine-comparable artifact from `weft mesh … --validate` (via
# `--signature`), or compare two signatures under the policy (exit 0 if
# policy-equal, 1 if not). Byte-identical OBJ floats are not required.
#
# Usage:
#   tools/topology_signature.sh emit <in.step> -o <out.sig> [-- mesh options…]
#   tools/topology_signature.sh compare <a.sig> <b.sig>
#   tools/topology_signature.sh self-check   # cylinder/box/torture twice
#   tools/topology_signature.sh fixture-set -o <dir>   # small CI fixture set
#   tools/topology_signature.sh release-set -o <dir>   # five release models
#
# Env:
#   WEFT   path to weft binary (default: build/cli/weft)

set -euo pipefail

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

usage() {
    sed -n '2,16p' "$0" | sed 's/^# \?//'
    exit 2
}

cmd_emit() {
    if [[ $# -lt 3 ]]; then usage; fi
    local step="$1"
    shift
    local out=""
    local mesh_args=()
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -o|--output)
                out="$2"
                shift 2
                ;;
            --)
                shift
                mesh_args+=("$@")
                break
                ;;
            *)
                mesh_args+=("$1")
                shift
                ;;
        esac
    done
    if [[ -z "$out" ]]; then
        echo "emit needs -o <out.sig>" >&2
        exit 2
    fi
    if [[ ! -x "$WEFT" ]]; then
        echo "weft binary not found (WEFT=$WEFT)" >&2
        exit 2
    fi
    mkdir -p "$(dirname "$out")"
    # Signature path implies validate inside the CLI; still pass --validate
    # so logs stay consistent with corpus_gate.
    "$WEFT" mesh "$step" --validate --signature "$out" "${mesh_args[@]+"${mesh_args[@]}"}"
}

cmd_compare() {
    if [[ $# -ne 2 ]]; then usage; fi
    if [[ ! -x "$WEFT" ]]; then
        echo "weft binary not found (WEFT=$WEFT)" >&2
        exit 2
    fi
    "$WEFT" signature-compare "$1" "$2"
}

# Fast fixtures for same-platform determinism (not a golden corpus).
FIXTURE_SET=(cylinder box torture)

# Release models (EXECUTION_PLAN §4.3). Paths relative to repo root.
RELEASE_SET=(
    "flaregun:tests/STEP_Examples/flaregun.stp"
    "foam:tests/STEP_Examples/foam.stp"
    "teleporter:tests/STEP_Examples/teleporter.stp"
    "iso14649-demo:tests/STEP_Examples/iso14649-demo.stp"
    "torture:tests/fixtures/torture.step"
)

ensure_steps() {
    local name step
    for name in "${FIXTURE_SET[@]}"; do
        case "$name" in
            torture)
                step="tests/fixtures/torture.step"
                if [[ ! -f "$step" ]]; then
                    mkdir -p "$(dirname "$step")"
                    "$WEFT" fixture "$step" --shape torture >/dev/null
                fi
                ;;
            *)
                step="tests/fixtures/generated/${name}.step"
                ensure_fixture_step "$WEFT" "$ROOT/$step" "$name" fixture
                ;;
        esac
    done
}

cmd_self_check() {
    if [[ ! -x "$WEFT" ]]; then
        echo "weft binary not found (WEFT=$WEFT)" >&2
        exit 2
    fi
    ensure_steps
    local tmp
    tmp=$(mktemp -d)
    local fail=0
    local name step a b
    for name in "${FIXTURE_SET[@]}"; do
        if [[ "$name" == "torture" ]]; then
            step="tests/fixtures/torture.step"
        else
            step="tests/fixtures/generated/${name}.step"
        fi
        a="$tmp/${name}-a.sig"
        b="$tmp/${name}-b.sig"
        echo "== self-check $name =="
        "$WEFT" mesh "$step" --validate --signature "$a" >/dev/null
        "$WEFT" mesh "$step" --validate --signature "$b" >/dev/null
        if ! "$WEFT" signature-compare "$a" "$b"; then
            echo "FAIL $name: repeated signatures not policy-equal" >&2
            fail=1
        else
            echo "ok $name"
        fi
    done
    rm -rf "$tmp"
    if [[ "$fail" -ne 0 ]]; then
        echo "topology signature self-check FAILED" >&2
        exit 1
    fi
    echo "topology signature self-check passed"
}

cmd_fixture_set() {
    local out=""
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -o|--output) out="$2"; shift 2 ;;
            *) usage ;;
        esac
    done
    if [[ -z "$out" ]]; then
        echo "fixture-set needs -o <dir>" >&2
        exit 2
    fi
    if [[ ! -x "$WEFT" ]]; then
        echo "weft binary not found (WEFT=$WEFT)" >&2
        exit 2
    fi
    ensure_steps
    mkdir -p "$out"
    local name step
    for name in "${FIXTURE_SET[@]}"; do
        if [[ "$name" == "torture" ]]; then
            step="tests/fixtures/torture.step"
        else
            step="tests/fixtures/generated/${name}.step"
        fi
        echo "== fixture-set $name =="
        "$WEFT" mesh "$step" --validate --signature "$out/${name}.sig"
    done
    echo "wrote signatures under $out"
}

cmd_release_set() {
    local out=""
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -o|--output) out="$2"; shift 2 ;;
            *) usage ;;
        esac
    done
    if [[ -z "$out" ]]; then
        echo "release-set needs -o <dir>" >&2
        exit 2
    fi
    if [[ ! -x "$WEFT" ]]; then
        echo "weft binary not found (WEFT=$WEFT)" >&2
        exit 2
    fi
    mkdir -p "$out"
    # Ensure generated torture fixture exists.
    if [[ ! -f tests/fixtures/torture.step ]]; then
        mkdir -p tests/fixtures
        "$WEFT" fixture tests/fixtures/torture.step --shape torture >/dev/null
    fi
    local entry name step
    for entry in "${RELEASE_SET[@]}"; do
        name="${entry%%:*}"
        step="${entry#*:}"
        if [[ ! -f "$step" ]]; then
            echo "FAIL missing release STEP $step" >&2
            exit 1
        fi
        echo "== release-set $name (default) =="
        "$WEFT" mesh "$step" --validate --signature "$out/${name}-default.sig"
        echo "== release-set $name (cad) =="
        "$WEFT" mesh "$step" --profile cad --validate \
            --signature "$out/${name}-cad.sig"
    done
    echo "wrote release signatures under $out"
}

if [[ $# -lt 1 ]]; then usage; fi
cmd="$1"
shift
case "$cmd" in
    emit) cmd_emit "$@" ;;
    compare) cmd_compare "$@" ;;
    self-check) cmd_self_check "$@" ;;
    fixture-set) cmd_fixture_set "$@" ;;
    release-set) cmd_release_set "$@" ;;
    -h|--help|help) usage ;;
    *) echo "unknown command: $cmd" >&2; usage ;;
esac
