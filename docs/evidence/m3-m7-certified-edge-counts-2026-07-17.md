# M3/M7 certified edge-count evidence - 2026-07-17

## Proven increment

Source-referenced recipe-v2 edge counts now resolve to exact working-edge
interval constraints. The exact solver reconciles them with canonical-boundary
equality, certified minimum, parity, and cap requirements before any face is
meshed. CLI, desktop regeneration, hot reload, and reporting consume that same
result; there is no face-local resampling or authoritative weld.

## Automated proof

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release --parallel 2
cmake --build --preset vs2022-static-analysis --config Release --parallel 2
ctest --test-dir build/vs2022 -C Release -E "^pipeline$" --output-on-failure
ctest --test-dir build/vs2022-static-analysis -C Release -E "^pipeline$" --output-on-failure
```

Both complete-product builds pass. All 14 secure/corpus tests pass in each
lane. The focused proof establishes:

- exact count propagation through an equality class;
- named incompatible-exact, below-minimum, and parity refusals;
- a four-interval box edge represented by five canonical sample IDs, each
  consumed by both owning faces;
- deterministic repeated topology fingerprints;
- a pinned 64-interval cylinder rim reconciled to 64 samples on both rims;
- generation-report counts equal to the solved canonical counts;
- named missing-edge and zero-count refusals;
- named refusal of a cylinder axial-edge pin that would require uncertified
  interior provenance; and
- successful migration, resolution, and application validation for an
  edge-only recipe.

## Executable CLI replay

A fresh box STEP was generated twice with the Release CLI: once with
`--edge 1:4 --save-recipe`, and once by loading the saved recipe v2. Both runs
reported 11 vertices and 18 triangles. The recipe stores source edge 1 with
its complete geometric fingerprint and exact count 4.

- direct/reloaded OBJ SHA-256:
  `2D1C7B031B8A2C5555FDD314FA38D306FCCFE35D3183ADF16221D72C519AFAB9`;
- direct/reloaded validation-report SHA-256:
  `12FA9C3FFBAD33C5FCF9712377A7772E0CF41AE686F8698C9AAF3DB908E6211B`.

The report includes
`secure_pipeline.exact_edge_interval_constraints expected=1 checked=1
skipped=0 failed=0`.

## Executable app proof

The Release screenshot harness loaded the same STEP and source-referenced v2
sidecar in feature-edge mode. It displayed `recipe v2 ready`, 11 vertices,
18 triangles, `watertight`, and the active certified feature-edge constraint
panel. Face and model controls remained visibly inspect-only. Artifact:
`build/secure-edge-recipe-final-20260717/app-edge-v2-active.png`.

## Status boundary

This proves one exact per-edge interval consumer for the existing plane/full-
cylinder certified floor. It does not prove per-face settings, manual edits,
legacy compiler edge constraints, cylinder axial interior rings, partial
cylinders, general periodic faces, Linux determinism, or the full M3/M7 gates.
Both milestones remain `IN_PROGRESS`.
