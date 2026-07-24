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

# Compare a golden count table against a fresh one BY KEY (name + profile).
# A positional `diff -u` aligns unrelated rows when several drift at once and
# then prints golden values for the wrong case — that misattribution sent a
# real investigation after a nonexistent "foam_wt_open_nm" regression whose
# numbers actually belonged to another branch. Keys never lie.
# $1 = golden table, $2 = actual table. Prints drift; returns 1 if any.
corpus_diff_counts() {
    local golden="$1" actual="$2"
    awk '
        function key(line,   a) { split(line, a, " "); return a[1] " " a[2] }
        function val(line,   a, i, s) {
            split(line, a, " ")
            s = ""
            for (i = 3; i <= length(a); ++i) s = s (i > 3 ? " " : "") a[i]
            return s
        }
        NR == FNR {
            if ($0 == "") next
            g[key($0)] = val($0)
            gorder[++gn] = key($0)
            next
        }
        {
            if ($0 == "") next
            k = key($0)
            seen[k] = 1
            if (!(k in g)) {
                printf "  NEW    %s: %s\n", k, val($0)
                bad = 1
                next
            }
            if (g[k] != val($0)) {
                printf "  MOVED  %s\n           golden: %s\n           actual: %s\n",
                       k, g[k], val($0)
                bad = 1
            }
        }
        END {
            for (i = 1; i <= gn; ++i) {
                if (!(gorder[i] in seen)) {
                    printf "  MISSING %s: %s (not produced by this run)\n",
                           gorder[i], g[gorder[i]]
                    bad = 1
                }
            }
            exit bad ? 1 : 0
        }
    ' "$golden" "$actual"
}

# Merge a fresh count table into the golden table by key: rows this run did
# not produce are preserved rather than silently dropped, so a partial run
# can never truncate the table.
corpus_merge_counts() {
    local golden="$1" actual="$2" out="$3"
    awk '
        function key(line,   a) { split(line, a, " "); return a[1] " " a[2] }
        NR == FNR { if ($0 != "") { fresh[key($0)] = $0 }; next }
        {
            if ($0 == "") next
            k = key($0)
            if (k in fresh) { print fresh[k]; done[k] = 1 }
            else { print }
        }
        END { for (k in fresh) if (!(k in done)) print fresh[k] }
    ' "$actual" "$golden" > "$out"
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
