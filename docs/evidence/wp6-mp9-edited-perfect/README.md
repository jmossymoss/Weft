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
| planned-floor | 31 | 18 |
| foldedPolygons | 23 | 31 |
| retention | 0.9889 | 0.9941 |

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

## Phase 3 remaining

- 31 folded polygons (led by `#375` cone drum; geoheal discard-reclip drops
  them but cannot re-fill cone holes)
- 18 planned-floor (drum inset / freeform step / poles / closed chart)

Release spot-check: flaregun / foam / teleporter watertight at CAD defaults.
