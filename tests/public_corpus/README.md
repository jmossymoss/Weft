# Public real-world corpus

External datasets are **not** committed. Manifests here select a reproducible
subset; fetch into `tests/public_corpus/_cache/` (gitignored) before running.

| Layer | Role | Upstream |
| --- | --- | --- |
| `fusion360_smoke.tsv` | Primary mechanical-feature smoke (stratified) | [Fusion 360 Gallery Extended STEP](https://github.com/AutodeskAILab/Fusion360GalleryDataset) (`s2.0.1_extended_step`, 42,912 STEP) |
| `abc_nightly.tsv` | Broad nightly robustness sample | [ABC Dataset](https://deep-geometry.github.io/abc-dataset/) |
| `nist_interop.tsv` | STEP import / assemblies / units | [NIST MBE PMI / CAx-IF](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0) |
| `mambo_stress.tsv` | Difficult meshing topologies | [MAMBO](https://gitlab.com/franck.ledoux/mambo) |

MP9 and other Plasticity target assets stay in `CAD_CORPUS.tsv` as
`tier=performance` / `layer=target-assets`. They are **not** geometry-coverage
benchmarks.

## Fetch

```sh
tools/fetch_public_corpus.sh fusion360-smoke   # small stratified list only
tools/fetch_public_corpus.sh abc-nightly       # requires ABC root env
tools/fetch_public_corpus.sh nist              # download linked NIST samples
tools/fetch_public_corpus.sh mambo
```

Environment:

- `WEFT_PUBLIC_CORPUS_ROOT` — optional override for cache root
  (default `tests/public_corpus/_cache`)
- `WEFT_ABC_ROOT` — local ABC STEP tree for nightly selection
- `WEFT_RUN_PUBLIC=1` — enable public rows in `tools/corpus_gate.sh`

## Stratification (Fusion)

Prefer the Extended STEP segmentation set. Select by modeling-operation labels
(extrude, cut, fillet, chamfer, revolution) and face-count bands so the smoke
subset exercises mechanical features without becoming a download-the-universe
gate. Expand breadth only after failures classify automatically.

## Validity

Public models are asserted only after source validity is recorded. Require
watertight mesh output only when the source is a valid closed solid. Dirty or
non-manifold inputs need an explicit heal / reject / research outcome in the
manifest row.
