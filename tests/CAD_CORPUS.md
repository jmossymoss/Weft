# CAD regression corpus

This document explains corpus mechanics. Product scope, release models, public
datasets, and completion gates are defined only in
[`docs/EXECUTION_PLAN.md`](../docs/EXECUTION_PLAN.md). Corpus runners mesh
through CLI `weft mesh` → `weft::generate()`; see
[`docs/PRODUCTION_PATH.md`](../docs/PRODUCTION_PATH.md) for the shared
production path and stitch quarantine.

## Corpus layers

| Layer | Purpose | Inventory |
| --- | --- | --- |
| `geometry-zoo` | Atomic OCCT surface/curve coverage | generated fixtures in `CAD_CORPUS.tsv` |
| `interaction-zoo` | Holes, fillets, seams, multi-body | generated / committed fixtures |
| `dirty-step` | Gaps, open shells, invalid trims | `tier=dirty` fixtures with explicit validity |
| `broad-nightly` | ABC broad diversity | `tests/public_corpus/abc_nightly.tsv` |
| `interop` | NIST / CAx-IF STEP import | `tests/public_corpus/nist_interop.tsv` |
| `meshing-stress` | MAMBO difficult topologies | `tests/public_corpus/mambo_stress.tsv` |
| `public-optional` | Optional Fusion 360 Gallery sample | `tests/public_corpus/fusion360_smoke.tsv` |
| `release` | MVP artist gate | `tier=release` in `CAD_CORPUS.tsv` |
| `target-assets` | Plasticity workloads (incl. MP9) | `tier=performance` / `stress` — **not** geometry coverage |

Public corpus authority is ABC (broad diversity), NIST/CAx-IF (interop), and
MAMBO (meshing stress). Fusion 360 Gallery is optional supplemental smoke, not
required or primary. See [`public_corpus/README.md`](public_corpus/README.md)
and [`public_corpus/ABC.md`](public_corpus/ABC.md).

Geometry coverage comes from the deterministic zoo and `COVERAGE_MATRIX.tsv`.
Local `STEP_Examples` release models are the artist gate, not a public-corpus
or geometry-coverage oracle. MP9 is a performance / integration workload only;
it must not drive geometry coverage or face-ID assertions.

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

## Ledgers: pinned counts vs ratchets

Three tables record different kinds of truth, and they fail differently.

| Table | Semantics | Failure |
| --- | --- | --- |
| `tools/golden_counts.txt` | pinned polygon arity per case/profile | any change, up or down |
| `tools/golden_structure.txt` | structure retention + winding per case | debt may only fall |
| `tools/golden_intent.txt` | what a typed density edit costs per case | debt may only fall |

Counts are compared BY CASE KEY, never positionally: a positional `diff -u`
aligns unrelated rows once several drift and then prints golden values for the
wrong case, which has already misdirected one investigation.

Structure retention (`weft::formatStructure`, printed by `weft mesh`) counts
faces that kept the topology their plan chose. It separates a deliberate
`planned-floor` from `failed-floor` debt, because a face can sit on the
contract floor while every watertightness check passes — that is what an artist
reports as "it still triangulates". `winding` is the §3.1 consistent-winding
requirement, which the validator has always measured and no gate read.

The intent ledger is the artist-facing one: for every density-editable face it
turns that family's SEMANTIC knob (radial / fillet loops / grid u) across a
count range with adaptive OFF — the wheel and panel path — and records how
often the edited face or a NEIGHBOUR loses its planned topology. It starts
non-zero by design; the numbers are the debt, and they may only fall.

Bank an improvement with `--update` on the owning gate, and explain any
intentional count movement (root cause plus rendered before/after, see
`tools/render_obj_wireframe.py`) in `docs/evidence/`.

## Runners

```sh
ctest --test-dir build --output-on-failure
tools/coverage_report.sh
tools/corpus_gate.sh --no-golden
tools/intent_gate.sh                  # artist-intent ratchet (fast sampling)
tools/intent_gate.sh --full           # every editable face, every count
tools/intent_gate.sh --model=flaregun # one case
build/cli/weft mesh M.step -o m.obj --profile cad --why   # why faces demoted
tools/topology_signature.sh self-check   # §3.2 cross-platform signature smoke
tools/release_gate.sh          # expected red until WP3
tools/corpus_scoreboard.sh > build/scoreboard.tsv
tools/fetch_public_corpus.sh status
tools/fetch_mambo_corpus.sh
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh         # ABC first; opt-in
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh mambo   # MAMBO stress smoke
```

Topology signatures (`tools/topology_signature.sh`) are the §3.2 machine-
comparable artifact for face routing, raw/empty status, polygon arity, and
connectivity — not golden OBJ bytes. See
[`docs/PRODUCTION_PATH.md`](../docs/PRODUCTION_PATH.md).

Public rows run only when `WEFT_RUN_PUBLIC=1` and cache files exist
(`tools/public_corpus_gate.sh`; default manifest is `abc_nightly.tsv`).

## Adding a generated fixture

1. Add the constructor to `weft::makeFixture`.
2. Tag surfaces / curves / features / validity / layer in `CAD_CORPUS.tsv`.
3. Ensure `COVERAGE_MATRIX.tsv` tags are covered (or extend the matrix).
4. Generate during test; do not depend on an untracked local STEP file.
5. Add visual evidence when polygon flow matters.

## Reducing a real-world failure

Prefer an ABC, NIST/CAx-IF, MAMBO, or release failure class, extract a minimal
neighborhood, and add a deterministic fixture. Do not special-case filenames or
face IDs.
