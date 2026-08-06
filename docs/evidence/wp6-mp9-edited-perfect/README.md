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
| foldedPolygons | 23 | 3 |
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

### Cone rim sum equalization

Widen split-rim density repair for `GeomAbs_Cone` (caps 12/12, deficit≤8)
so notched lead-in rims equalize instead of strip-reconciling folded
azimuth quads. Folds 31→26; #375 7→2.

### Digon rail-ladder → single n-gon

Thin extrusion digons (#1073) emit one border-exact n-gon instead of
ladder rungs that fold under the UV census. Folds 26→21.

### Tiny freeform → MinimalNGon

Freeform faces with ≤3 edges claim MinimalNGon early (#1828 class).
Folds 21→16.

### Freeform Coons tip → MinimalNGon

When sparse-fold protect would keep freeform Coons tip folds, remesh as
MinimalNGon if fold census improves (#47/#8). Folds 16→11.

### Tip-fold ≤2 MinimalNGon (ribbon ≤14 edges + revgrid drums)

Rescue freeform Coons/ribbons with ≤2 tip folds and ≤14 edges, plus
RevolutionGrid drums with ≤2 residual folds, to MinimalNGon when the
census improves. Flaregun earclip (16 edges) and dense-fold straps stay
RibbonSweep. Folds 11→3.

## Phase 3 remaining

- 31 folded polygons (led by `#375` cone drum; geoheal discard-reclip drops
  them but cannot re-fill cone holes)

Release spot-check: flaregun / foam / teleporter watertight at CAD defaults;
teleporter retention now 1.0.
