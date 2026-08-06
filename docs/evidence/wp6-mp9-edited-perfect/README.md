# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Phase 2 validity — DONE

At CAD defaults, `tests/STEP_Examples/mp9_Edited.stp` is watertight:

| metric | baseline | tip |
|--------|--------:|----:|
| openEdges | 54 | 0 |
| nonManifoldEdges | 39 | 0 |
| windingConflicts | 3 | 0 |
| failed-floor | 1 | 0 |
| raw | 2 | 0 |
| planned-floor | 31 | 0 |
| foldedPolygons | 23 | 31 |
| retention | 0.9889 | 1.0000 |

Locked by `testMp9EditedWatertight` and `CAD_CORPUS.tsv` row `mp9_edited`
with `require_watertight=1`.

## Validity classes landed

1. Openband side × orthogonal pin
2. Rail-ladder digon / incomplete-wire n-gon + winding neighbour flip
3. Freeform Coons opposite-chain (≤14, ribbon-aware) + sparse-fold protect
4. Orphan seam absorb (structured pairs, partner near-miss, open-endpoint weld)
5. unionSeams 35% sagitta + post-fold re-run + deepest-t walk
6. Open flap triangle drop
7. Iterative digon-spur cleanup (NM → 0)

## Phase 3 structure — DONE

planned-floor = 0, failed-floor = 0, raw = 0, retention = 1.0.

### Structure classes landed

1. Pole-tolerant MinimalNGon rescue (`tolerateDegenerate`) for digons / drivers
2. Late-retry drum admit with insetVertical ≤ 4 (foam #514 at 5 stays gated)
3. Fresh-FacePlan freeform MinimalNGon rescue (edge budget 128) so dirty orth
   leftovers do not poison `#722`/`#728`/`#134`

## Phase 3 remaining

- 31 folded polygons (led by `#375` cone drum; geoheal discard-reclip drops
  them but cannot re-fill cone holes)

Release spot-check: flaregun / foam / teleporter watertight at CAD defaults;
teleporter retention now 1.0.
