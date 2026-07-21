# ABC Dataset nightly sample

Role: **broad-nightly / geometric-diversity** public corpus for WP1. Not
CI-blocking geometry coverage. Do not treat local `tests/STEP_Examples` (MP9,
flaregun, foam, …) as this layer.

Upstream: [ABC Dataset](https://deep-geometry.github.io/abc-dataset/)

ABC STEP chunks are large (~0.8–1.6 GB each, 7z of 10k models) and licensed for
research use; blobs are **not** committed. Point `WEFT_ABC_ROOT` at a local STEP
tree, then sample into the gitignored cache.

## Obtain ABC

1. Read terms on the upstream page.
2. Chunk URL lists (from the site’s Download → Chunks section):
   - STEP: https://deep-geometry.github.io/abc-dataset/data/step_v00.txt
   - Stats (optional, small): https://deep-geometry.github.io/abc-dataset/data/stat_v00.txt
   - Meta (optional, small): https://deep-geometry.github.io/abc-dataset/data/meta_v00.txt
   - Sizes: https://deep-geometry.github.io/abc-dataset/data/size.yml
3. Download one STEP chunk, e.g. first line of `step_v00.txt`:

```sh
curl -fsSL -o step_v00.txt https://deep-geometry.github.io/abc-dataset/data/step_v00.txt
sed '1q;d' step_v00.txt | xargs -n 2 sh -c 'curl -fL -o "$1" "$0"'
7z x abc_0000_step_v00.7z -o"$WEFT_ABC_ROOT"
```

4. Optionally unpack the matching `stat_v00` chunk nearby so the sampler can use
   `#surfs` (B-rep surface count) instead of STEP entity heuristics.

A full STEP chunk is too large for a “tiny CI download”; the fetch helper does
not pull STEP archives by default.

## Sample by face-count bands

```sh
export WEFT_ABC_ROOT=/path/to/abc/step   # directory tree of .step / .stp
tools/fetch_public_corpus.sh abc-nightly
# equivalent:
tools/sample_abc_nightly.py
```

| Band | B-rep face / `#surfs` proxy |
| --- | --- |
| `1-100` | 1 ≤ n ≤ 100 |
| `100-500` | 101 ≤ n ≤ 500 |
| `500-2000` | 501 ≤ n ≤ 2000 |

Classification order: sibling ABC stats `#surfs` → `ADVANCED_FACE` count in
STEP → file-size bands. Copied files land under
`tests/public_corpus/_cache/abc/nightly/` and `abc_nightly.tsv` is rewritten.

Without `WEFT_ABC_ROOT`, `fetch_public_corpus.sh abc-nightly` only refreshes the
expected-slot smoke list in `abc_nightly.tsv`.

## Gate

```sh
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh          # ABC first (default)
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh all      # ABC then other public layers
```

Missing cache files are noted and skipped (ABC is optional on developer
machines). Watertight mesh is required only when `validity=closed_solid`.
