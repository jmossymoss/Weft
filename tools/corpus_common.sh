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

# Ratchet a structure-retention table (see weft::formatStructure): faces may
# only move toward keeping their planned topology. Counts are NOT pinned like
# golden polygon counts — an improvement must pass, a regression must not.
# $1 = golden table, $2 = actual table. Prints verdicts; returns 1 on any
# regression.
corpus_ratchet_structure() {
    local golden="$1" actual="$2"
    awk '
        function key(line,   a) { split(line, a, " "); return a[1] " " a[2] }
        function field(line, name,   n, i, kv) {
            n = split(line, a, " ")
            for (i = 3; i <= n; ++i) {
                split(a[i], kv, "=")
                if (kv[1] == name) return kv[2] + 0
            }
            return -1
        }
        NR == FNR { if ($0 != "") { g[key($0)] = $0; gorder[++gn] = key($0) }; next }
        {
            if ($0 == "") next
            k = key($0)
            seen[k] = 1
            if (!(k in g)) {
                printf "  NEW      %s: %s\n", k, $0
                improved = 1
                next
            }
            split("", worse)
            nworse = 0
            # Debt metrics may only fall.
            for (m = 1; m <= 4; ++m) {
                name = (m == 1 ? "failed-floor" : (m == 2 ? "raw" : \
                       (m == 3 ? "empty" : "winding")))
                gv = field(g[k], name); av = field($0, name)
                if (av > gv) { worse[++nworse] = sprintf("%s %d -> %d", name, gv, av) }
                if (av < gv) { better = better sprintf("  BETTER   %s: %s %d -> %d\n", k, name, gv, av) }
            }
            # Structured faces may only rise, unless the face total changed
            # (a different model revision) — then only debt metrics rule.
            gt = field(g[k], "faces"); at = field($0, "faces")
            gs = field(g[k], "structured"); as = field($0, "structured")
            if (gt == at && as < gs) {
                worse[++nworse] = sprintf("structured %d -> %d", gs, as)
            }
            if (gt == at && as > gs) {
                better = better sprintf("  BETTER   %s: structured %d -> %d\n", k, gs, as)
            }
            if (nworse > 0) {
                printf "  REGRESSED %s:", k
                for (i = 1; i <= nworse; ++i) printf " %s;", worse[i]
                printf "\n"
                bad = 1
            }
        }
        END {
            for (i = 1; i <= gn; ++i) {
                if (!(gorder[i] in seen)) {
                    printf "  MISSING  %s (not produced by this run)\n", gorder[i]
                }
            }
            printf "%s", better
            exit bad ? 1 : 0
        }
    ' "$golden" "$actual"
}

# Ratchet the artist-intent ledger: every metric is debt, so all of them may
# only fall. `runs` is context, not a verdict (sampling settings change it).
corpus_ratchet_intent() {
    local golden="$1" actual="$2"
    awk '
        function key(line,   a) { split(line, a, " "); return a[1] " " a[2] }
        function field(line, name,   n, i, a, kv) {
            n = split(line, a, " ")
            for (i = 3; i <= n; ++i) {
                split(a[i], kv, "=")
                if (kv[1] == name) return kv[2] + 0
            }
            return -1
        }
        NR == FNR { if ($0 != "") { g[key($0)] = $0 }; next }
        {
            if ($0 == "") next
            k = key($0)
            if (!(k in g)) { printf "  NEW      %s: %s\n", k, $0; next }
            # Sampling must match or the comparison is meaningless.
            if (field(g[k], "runs") != field($0, "runs")) {
                printf "  SKIPPED  %s: runs %d -> %d (different sampling)\n",
                       k, field(g[k], "runs"), field($0, "runs")
                next
            }
            nworse = 0
            split("", worse)
            for (m = 1; m <= 5; ++m) {
                name = (m == 1 ? "self-lost" : \
                       (m == 2 ? "neighbour-lost" : \
                       (m == 3 ? "leaks" : (m == 4 ? "winding" : "throws"))))
                gv = field(g[k], name); av = field($0, name)
                if (av > gv) worse[++nworse] = sprintf("%s %d -> %d", name, gv, av)
                else if (av < gv) better = better sprintf("  BETTER   %s: %s %d -> %d\n", k, name, gv, av)
            }
            if (nworse > 0) {
                printf "  REGRESSED %s:", k
                for (i = 1; i <= nworse; ++i) printf " %s;", worse[i]
                printf "\n"
                bad = 1
            }
        }
        END { printf "%s", better; exit bad ? 1 : 0 }
    ' "$golden" "$actual"
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
