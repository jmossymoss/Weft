# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded).

## Phase 2+3 — DONE

At CAD defaults, `tests/STEP_Examples/mp9_Edited.stp`:

| metric | baseline | tip |
|--------|--------:|----:|
| openEdges | 54 | 0 |
| nonManifoldEdges | 39 | 0 |
| windingConflicts | 3 | 0 |
| failed-floor | 1 | 0 |
| raw | 2 | 0 |
| planned-floor | 31 | 0 |
| foldedPolygons | 23 | 0 |
| retention | 0.9889 | 1.0000 |

Locked by `testMp9EditedWatertight` (opens/NM/winding/folds/floors/raw) and
`CAD_CORPUS.tsv` row `mp9_edited` with `require_watertight=1`.

## Classes landed

### Validity
1. Openband side × orthogonal pin
2. Rail-ladder digon / incomplete-wire n-gon + winding neighbour flip
3. Freeform Coons opposite-chain (≤14, ribbon-aware) + sparse-fold protect
4. Orphan seam absorb (structured pairs, partner near-miss)
5. unionSeams 35% sagitta + post-fold re-run
6. Open flap triangle drop
7. Iterative digon-spur cleanup

### Structure
1. Pole-tolerant MinimalNGon (`tolerateDegenerate`)
2. Late-retry drum insetVertical ≤ 4
3. Fresh-FacePlan freeform MinimalNGon rescue (budget 128)

### Folds
1. Cone rim sum equalization
2. Digon rail-ladder → single n-gon
3. Tiny freeform (3 edges) → MinimalNGon
4. Tip-fold ≤2 MinimalNGon (Coons; ribbon ≤14 edges; revgrid drums)
5. Surface-evaluated Newell for `foldedPolys` (weld-drift false folds)
6. Freeform tiny-revolve (3–5 edges) → MinimalNGon before RevolutionGrid

## Original MP9.stp

Opens measured 7 after the same class fixes (was ~247).
`tests/KNOWN_RED.tsv` ceiling tightened 300→20.

## Release spot-check

flaregun / foam / teleporter / iso14649-demo watertight at CAD defaults.

## Visual inspection

App screenshots (`app_*.png`) from `weft_app` with CAD defaults
`--finalize`: watertight badge, structured quads on drums/fillets/grips,
exploded assembly reads as an MP9 (suppressor, optic, magazine spring).
Matplotlib `mp9_edited_*.png` are coarse overview plots only.

## Phase 4 — suite / corpus harden (2026-08-06)

- `ctest` green (pipeline + topology_signature).
- IsoBand open-band prefer gated to `edgeIds >= 64` with `bandDriver`, so ABC
  notched drums take open-band while mp9 muzzle two-full-height keeps orth
  column cells.
- Corpus goldens refreshed after intentional topology moves on foam /
  teleporter / flaregun / demo / torture / canrev / ellipse_plate (structured
  retention held; counts drift from IsoBand / tip-fold / floor-clear classes
  banked this campaign).
- `tools/corpus_gate.sh` PASS after `--update`.
