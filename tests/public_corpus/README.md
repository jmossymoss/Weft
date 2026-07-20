# Public real-world corpus

External datasets are **not** committed. Manifests select a reproducible subset;
fetch into `tests/public_corpus/_cache/` (gitignored) before running.

| Layer | Role | Upstream |
| --- | --- | --- |
| `abc_nightly.tsv` | Broad geometric diversity / nightly robustness | [ABC Dataset](https://deep-geometry.github.io/abc-dataset/) |
| `nist_interop.tsv` | STEP import, assemblies, units, metadata | [NIST MBE PMI / CAx-IF](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0) |
| `mambo_stress.tsv` | Difficult meshing / blocking stress | [MAMBO](https://gitlab.com/franck.ledoux/mambo) |

These three are the public corpus authority for WP1. Local `tests/STEP_Examples`
(MP9, flaregun, foam, …) are not this layer — Plasticity targets stay in
`CAD_CORPUS.tsv` as performance/release cases only.

## Fetch

```sh
tools/fetch_public_corpus.sh status

# ABC — sample from a local ABC STEP tree (chunks are multi-GB)
export WEFT_ABC_ROOT=/path/to/abc/step
tools/fetch_public_corpus.sh abc-nightly

# NIST / CAx-IF — small official archives
tools/fetch_nist_corpus.sh

# MAMBO — shallow clone + smoke selection
tools/fetch_mambo_corpus.sh
```

Details: [`ABC.md`](ABC.md), [`NIST.md`](NIST.md), [`MAMBO.md`](MAMBO.md).

## Gate

```sh
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh          # ABC
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh nist
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh mambo
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh all      # ABC → NIST → MAMBO
```

Watertight mesh is required only when the manifest row’s `validity` is
`closed_solid`. Dirty / research / stress rows may fail meshing without failing
the gate when that is the declared outcome.

## Validity

Record source validity before asserting output. Valid closed solids can require
watertight meshes; open or invalid inputs need an explicit heal / reject /
bounded research outcome.
