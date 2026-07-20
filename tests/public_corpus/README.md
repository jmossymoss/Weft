# Public real-world corpus

External datasets are **not** committed. Manifests here select a reproducible
subset; fetch into `tests/public_corpus/_cache/` (gitignored) before running.

Public corpus authority (roles are distinct; none is a geometry-coverage
oracle — that remains the deterministic zoo + `COVERAGE_MATRIX.tsv`):

| Layer | Role | Upstream |
| --- | --- | --- |
| `abc_nightly.tsv` | Broad diversity / nightly robustness | [ABC Dataset](https://deep-geometry.github.io/abc-dataset/) |
| `nist_interop.tsv` | STEP import interoperability | [NIST MBE PMI / CAx-IF](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0) |
| `mambo_stress.tsv` | Meshing-topology stress | [MAMBO](https://gitlab.com/franck.ledoux/mambo) |

Optional supplemental smoke (not required, not primary):

| Layer | Role | Upstream |
| --- | --- | --- |
| `fusion360_smoke.tsv` | Optional stratified mechanical-feature sample | [Fusion 360 Gallery Extended STEP](https://github.com/AutodeskAILab/Fusion360GalleryDataset) (`s2.0.1_extended_step`) |

See also [`ABC.md`](ABC.md), [`NIST.md`](NIST.md), and [`MAMBO.md`](MAMBO.md).

Local `STEP_Examples` release/stress paths and MP9 live in `CAD_CORPUS.tsv`.
MP9 is `tier=performance` / `layer=target-assets` only. Neither MP9 nor other
Plasticity target assets are public-corpus geometry-coverage benchmarks.

## Fetch

```sh
tools/fetch_public_corpus.sh abc-nightly   # requires WEFT_ABC_ROOT
tools/fetch_public_corpus.sh nist
tools/fetch_public_corpus.sh mambo

# Optional Fusion sample (not part of public corpus authority):
tools/fetch_public_corpus.sh fusion360-smoke

tools/fetch_public_corpus.sh status

# Optional gate (skips unless WEFT_RUN_PUBLIC=1 and files exist)
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh
```

## Environment

- `WEFT_PUBLIC_CORPUS_ROOT` — optional override for cache root
  (default `tests/public_corpus/_cache`)
- `WEFT_ABC_ROOT` — local ABC STEP tree for nightly selection
- `WEFT_MAMBO_ROOT` — optional existing MAMBO checkout
- `WEFT_FETCH_FUSION_ZIP=1` — allow downloading the optional Fusion archive
- `WEFT_RUN_PUBLIC=1` — enable `tools/public_corpus_gate.sh`

Public cases are intentionally kept out of `CAD_CORPUS.tsv` so missing cache
paths cannot break the default corpus gate.

## Selection notes

- **ABC**: select by surface/curve mix and face-count bands rather than an
  unclassified random sample. Expand breadth only after failures classify
  automatically.
- **NIST / CAx-IF**: import, units, assemblies, and kernel-produced STEP
  (AP203/AP242-style). Prefer small reviewable smoke rows before large suites.
- **MAMBO**: difficult blocking / meshing configurations for stress, not
  breadth or interop coverage.
- **Fusion (optional)**: if used, prefer Extended STEP segmentation labels and
  face-count bands so a tiny sample stays reviewable. It does not define public
  corpus authority.

## Validity

Public models are asserted only after source validity is recorded. Require
watertight mesh output only when the source is a valid closed solid. Dirty or
non-manifold inputs need an explicit heal / reject / research outcome in the
manifest row.
