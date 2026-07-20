# CAD regression corpus

This document explains corpus mechanics. Product scope, release models, public
datasets, and completion gates are defined only in
[`docs/EXECUTION_PLAN.md`](../docs/EXECUTION_PLAN.md).

## Corpus layers

| Layer | Purpose | Inventory |
| --- | --- | --- |
| `geometry-zoo` | Atomic OCCT surface/curve coverage | generated fixtures in `CAD_CORPUS.tsv` |
| `interaction-zoo` | Holes, fillets, seams, multi-body | generated / committed fixtures |
| `dirty-step` | Gaps, open shells, invalid trims | `tier=dirty` fixtures with explicit validity |
| `public-real` | Stratified Fusion 360 Gallery Extended | `tests/public_corpus/fusion360_smoke.tsv` |
| `broad-nightly` | ABC breadth | `tests/public_corpus/abc_nightly.tsv` |
| `release` | MVP artist gate | `tier=release` in `CAD_CORPUS.tsv` |
| `target-assets` | Plasticity workloads (incl. MP9) | `tier=performance` / `stress` — **not** geometry coverage |

Primary real-world benchmark: Fusion 360 Gallery Extended STEP. ABC is nightly
breadth. NIST/CAx-IF covers import interoperability. MAMBO is a small meshing
stress supplement. See [`public_corpus/README.md`](public_corpus/README.md).

MP9 is a performance / integration workload only. It must not drive geometry
coverage or face-ID assertions.

`CAD_CORPUS.tsv` is the sole machine-readable case inventory for CI. Public
manifests select external files; they do not replace the zoo.

## Manifest columns

`name tier path fast max_raw max_empty require_watertight visual validity layer surfaces curves features notes`

- `validity`: `closed_solid` | `open` | `invalid` | `research`
- Watertight mesh is required only when `validity=closed_solid` and
  `require_watertight=1`
- `surfaces` / `curves` / `features`: comma-separated coverage tags checked by
  `tests/COVERAGE_MATRIX.tsv`

## Validation policy

- Validate source topology first.
- A valid closed solid in the release gate must eventually produce 0 open edges,
  0 non-manifold edges, no folds, no raw demotions, and no empty faces.
- Open or dirty inputs declare heal / reject / bounded research outcomes.
- Do not raise failure limits merely to pass.

`tests/KNOWN_RED.tsv` holds temporary release/regression allowances. The strict
release gate never consumes them.

## Runners

```sh
ctest --test-dir build --output-on-failure
tools/coverage_report.sh
tools/corpus_gate.sh --no-golden
tools/release_gate.sh          # expected red until WP3
tools/corpus_scoreboard.sh > build/scoreboard.tsv
tools/fetch_public_corpus.sh status
```

Public rows run only when `WEFT_RUN_PUBLIC=1` and cache files exist.

## Adding a generated fixture

1. Add the constructor to `weft::makeFixture`.
2. Tag surfaces / curves / features / validity / layer in `CAD_CORPUS.tsv`.
3. Ensure `COVERAGE_MATRIX.tsv` tags are covered (or extend the matrix).
4. Generate during test; do not depend on an untracked local STEP file.
5. Add visual evidence when polygon flow matters.

## Reducing a real-world failure

Prefer a Fusion (or release) failure class, extract a minimal neighborhood, and
add a deterministic fixture. Do not special-case filenames or face IDs.
