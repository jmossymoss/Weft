# CAD regression corpus

This document explains corpus mechanics. Product scope, release models, public
datasets, and completion gates are defined only in
[`docs/EXECUTION_PLAN.md`](../docs/EXECUTION_PLAN.md).

## Corpus layers

The intended corpus is layered by purpose:

- Generated geometry fixtures isolate one surface, trim, feature, or topology
  interaction. `weft fixture` creates these deterministically for tests.
- Adversarial generated fixtures exercise invalid or near-degenerate input with
  an explicit expected healing or diagnostic outcome.
- Reduced regressions preserve the smallest adjacency context needed to
  reproduce a real model failure.
- Committed STEP examples exercise real mechanical parts and assemblies.
- Release models gate the artist-usable MVP.
- Stress and performance models remain visible without silently expanding the
  release definition.
- External public datasets are selected through reproducible manifests and are
  not committed wholesale.
- `visual_baselines` stores reviewed viewport evidence. It is not a
  pixel-perfect golden-image gate.

`CAD_CORPUS.tsv` is the sole machine-readable case inventory. During work
package 0, its fixture and regression paths must be reconciled with files
generated or committed by the tests, and every runner must select from it. A
manifest row is not evidence that its referenced file currently exists.

The current `tools/corpus_gate.sh` hardcodes fixtures and automatically includes
all committed STEP examples. That is known bootstrap debt, not an alternate
inventory. Work package 0 replaces those lists with manifest tiers.

## Validation policy

Every case records source validity before asserting output validity:

- A valid closed solid must produce 0 open edges, 0 non-manifold edges, no
  folds, no raw triangulation demotions, and no empty faces when it is in the
  release gate.
- An intentionally open extracted neighborhood may have boundary edges that are
  identified as expected extraction boundaries.
- Invalid or deliberately dirty input must declare whether Weft should heal it,
  reject it with a diagnostic, or retain a bounded research-only defect.

Failure limits in the manifest are temporary known-red ceilings, not acceptance
targets. Do not raise them or change `require_watertight` merely to make a test
pass.

Reproducible release blockers belong in `KNOWN_RED.tsv` during stabilization.
Regression gates may consume those exact allowances; the strict release gate
must not. Stress/research ceilings may remain at release when they describe
their source validity and cannot increase.

## Current runners

The standard pipeline tests are:

```sh
ctest --test-dir build --output-on-failure
```

The bootstrap corpus gate generates its fixture inputs in a temporary directory
and tests committed STEP examples at default and CAD profiles:

```sh
tools/corpus_gate.sh
```

Use `--no-golden` only for cross-platform invariant checks. Use `--update` only
after an intentional topology change has been explained and visually reviewed.

For one model:

```sh
build/cli/weft mesh model.step -o out.obj --profile cad --validate
build/cli/weft sweep model.step
```

## Adding a generated fixture

1. Add the geometry constructor to the fixture registry.
2. Give it one primary surface/topology purpose.
3. Record source validity and expected behavior.
4. Generate it during the test; do not depend on an untracked local STEP file.
5. Add focused assertions for import, routing, topology, and diagnostics.
6. Add visual evidence when polygon flow matters.
7. Add it to `CAD_CORPUS.tsv` only when the manifest path and generation
   lifecycle are implemented.

## Reducing a real-world failure

```sh
build/cli/weft extract model.step \
  --faces 42 --rings 1 \
  -o tests/regressions/model_face_42.step
```

Keep the failing face and the smallest adjacency radius that reproduces the
failure. Then:

1. Confirm the reduced model has the same failure class.
2. Record whether the reduced source is closed and valid.
3. Add the file and expectations to `CAD_CORPUS.tsv`.
4. Add an automated assertion that fails for the observed reason.
5. Save one useful viewport artifact when the defect is visual.
6. Fix the topology class rather than the source filename or face ID.

## Visual review

Review wireframe and diagnostic overlays in both the app and Blender when a
mesher change affects topology. Check:

- straight and coherent primitive columns;
- understandable fillet flow;
- local collars around holes and slots;
- deliberate poles, triangles, and n-gons;
- folds, overlaps, spirals, slivers, and seam artifacts;
- hard-surface shading and editability.

Numeric counts alone cannot approve a topology change.
