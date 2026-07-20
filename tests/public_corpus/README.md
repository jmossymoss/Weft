# Public real-world corpus

External datasets are **not** committed. Manifests here select a reproducible
subset; fetch into `tests/public_corpus/_cache/` (gitignored) before running.

| Layer | Role | Upstream |
| --- | --- | --- |
| `abc_nightly.tsv` | **Broad nightly / geometric diversity** (WP1 public) | [ABC Dataset](https://deep-geometry.github.io/abc-dataset/) |
| `fusion360_smoke.tsv` | Optional mechanical-feature smoke (stratified) | [Fusion 360 Gallery Extended STEP](https://github.com/AutodeskAILab/Fusion360GalleryDataset) |
| `nist_interop.tsv` | STEP import / assemblies / units | [NIST MBE PMI / CAx-IF](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0) |
| `mambo_stress.tsv` | Difficult meshing topologies | [MAMBO](https://gitlab.com/franck.ledoux/mambo) |

Local `tests/STEP_Examples` assets (MP9, flaregun, foam, …) stay out of this
layer. Plasticity targets remain in `CAD_CORPUS.tsv` as `tier=performance` /
`layer=target-assets` — they are **not** the public corpus.

## ABC nightly (primary public / broad-nightly)

ABC STEP chunks are multi-GB; the fetch helper documents chunk URLs and samples
from a local tree. Details: [`ABC.md`](ABC.md).

```sh
# Print obtain steps + refresh expected-slot paths (no multi-GB download)
tools/fetch_public_corpus.sh abc-nightly

# After unpacking ABC STEP locally:
export WEFT_ABC_ROOT=/path/to/abc/step
tools/fetch_public_corpus.sh abc-nightly
# equivalent sampler:
tools/sample_abc_nightly.py

# Status
tools/fetch_public_corpus.sh status

# Opt-in gate (ABC first; skips unless WEFT_RUN_PUBLIC=1 and files exist)
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh
```

Manual obtain (same URLs the fetch helper prints):

```sh
curl -fsSL -o step_v00.txt \
  https://deep-geometry.github.io/abc-dataset/data/step_v00.txt
# ~0.8–1.6 GB per chunk — download only what you need:
sed '1q;d' step_v00.txt | xargs -n 2 sh -c 'curl -fL -o "$1" "$0"'
7z x abc_0000_step_v00.7z -o"$WEFT_ABC_ROOT"
tools/sample_abc_nightly.py
```

Sampled STEP files stay in the gitignored cache. Do not commit ABC archives or
extracted models.

## Other subsets

```sh
tools/fetch_mambo_corpus.sh                # preferred MAMBO stress fetch
tools/fetch_public_corpus.sh mambo         # thin wrapper → fetch_mambo_corpus.sh
tools/fetch_public_corpus.sh nist
WEFT_FETCH_FUSION_ZIP=1 tools/fetch_public_corpus.sh fusion360-smoke

# MAMBO opt-in gate (require_watertight only for validity=closed_solid)
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh mambo
```

See [`MAMBO.md`](MAMBO.md): shallow sparse-clone of Basic/Simple/Medium, ~13
smoke STEP files under `_cache/mambo/`. Cache is gitignored.

## Environment

- `WEFT_PUBLIC_CORPUS_ROOT` — optional override for cache root
  (default `tests/public_corpus/_cache`)
- `WEFT_ABC_ROOT` — local ABC STEP tree for nightly selection
- `WEFT_ABC_SEED` — sampler seed (default `42`)
- `WEFT_MAMBO_ROOT` — optional existing MAMBO checkout for `fetch_mambo_corpus.sh`
- `WEFT_RUN_PUBLIC=1` — enable `tools/public_corpus_gate.sh` (ABC default;
  use `mambo` / `all` for other layers)
- `WEFT_FETCH_FUSION_ZIP=1` — allow Fusion zip download (optional layer)

Public cases are intentionally kept out of `CAD_CORPUS.tsv` so missing cache
paths cannot break the default corpus gate.

## Stratification (ABC)

Select by B-rep surface / face-count bands (`1-100`, `100-500`, `500-2000`)
using ABC stats `#surfs` when present, else STEP `ADVANCED_FACE` counts, else
file-size proxy. Expand breadth only after failures classify automatically.

## Validity

Public models are asserted only after source validity is recorded. Require
watertight mesh output only when the source is a valid closed solid. Dirty or
non-manifold inputs need an explicit heal / reject / research outcome in the
manifest row.
