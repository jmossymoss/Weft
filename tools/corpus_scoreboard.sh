#!/usr/bin/env bash
# Emit a machine-readable scoreboard for fast CAD_CORPUS.tsv rows.
# Columns align with docs/EXECUTION_PLAN.md section 5 (subset for WP0 triage).

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
if [[ ! -x "$WEFT" && -x build/vs2022/bin/Release/weft.exe ]]; then
    WEFT=build/vs2022/bin/Release/weft.exe
fi

MANIFEST=tests/CAD_CORPUS.tsv
OUT=${OUT:-$(mktemp -d)}
mkdir -p "$OUT"

echo -e "name\ttier\tprofile\tload_ok\tverts\tpolys\tquads\ttris\tngons\topen\tnonmanifold\traw\tempty\twatertight"

while IFS= read -r row; do
    name=$(corpus_field "$row" 1)
    tier=$(corpus_field "$row" 2)
    rel=$(corpus_field "$row" 3)
    fast=$(corpus_field "$row" 4)
    [[ "$fast" == "1" ]] || continue
    step="$ROOT/tests/$rel"
    ensure_fixture_step "$WEFT" "$step" "$name" "$tier" || true
    if [[ ! -f "$step" ]]; then
        echo -e "$name\t$tier\tdefault\t0\t\t\t\t\t\t\t\t\t\t"
        continue
    fi
    for tag_args in "default|" "cad|--profile cad"; do
        tag=${tag_args%%|*}
        args=${tag_args#*|}
        log="$OUT/$name-$tag.log"
        # shellcheck disable=SC2086
        "$WEFT" mesh "$step" -o "$OUT/$name-$tag.obj" $args --validate > "$log" 2>&1
        load_ok=1
        [[ $? -eq 0 ]] || load_ok=0
        stats=$(grep -Eo '[0-9]+ vertices, [0-9]+ polygons \([0-9]+ quads, [0-9]+ tris, [0-9]+ n-gons\)' "$log" | head -1 || true)
        verts=$(echo "$stats" | grep -Eo '^[0-9]+' || echo 0)
        polys=$(echo "$stats" | grep -Eo '[0-9]+ polygons' | grep -Eo '[0-9]+' || echo 0)
        quads=$(echo "$stats" | grep -Eo '[0-9]+ quads' | grep -Eo '[0-9]+' || echo 0)
        tris=$(echo "$stats" | grep -Eo '[0-9]+ tris' | grep -Eo '[0-9]+' || echo 0)
        ngons=$(echo "$stats" | grep -Eo '[0-9]+ n-gons' | grep -Eo '[0-9]+' || echo 0)
        open=$(grep -Eo 'open edges: [0-9]+' "$log" | head -1 | grep -Eo '[0-9]+' || echo "")
        nm=$(grep -Eo 'non-manifold: [0-9]+' "$log" | head -1 | grep -Eo '[0-9]+' || echo "")
        raw=$(grep -Eo '[0-9]+ to raw triangulation' "$log" | head -1 | grep -Eo '^[0-9]+' || echo 0)
        empty=$(grep -Eo '[0-9]+ emitted nothing' "$log" | head -1 | grep -Eo '^[0-9]+' || echo 0)
        wt=0
        grep -Eq 'watertight:[[:space:]]*yes' "$log" && wt=1
        echo -e "$name\t$tier\t$tag\t$load_ok\t$verts\t$polys\t$quads\t$tris\t$ngons\t$open\t$nm\t$raw\t$empty\t$wt"
    done
done < <(corpus_rows "$MANIFEST")
