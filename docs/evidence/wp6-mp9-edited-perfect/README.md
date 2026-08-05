# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| openband side-pin | 51 | 39 | 23 | 3 | 0 | 2 | 31 | 0.9892 |
| rail-ladder digon | 37 | 39 | 28 | 0 | 0 | 1 | 31 | 0.9895 |

## Classes landed

### Openband side × orthogonal pin
Reducer: `tests/regressions/mp9/openband_border_contract_fillet.step`

### Rail-ladder digon fold → raw
Reducer: `tests/regressions/mp9/rail_ladder_digon_fold.step`

## Still open
- raw `#2005` (rail-ladder border contract on edge 5775; extract alone does not repro — full-model pin/density context)
- ~37 unexplained opens / 39 NM
- ~28 folds (led by `#375` cone drum)
- 31 planned-floor (freeform interior step dominant)

Release spot-check after tip: flaregun/foam/teleporter still watertight at CAD defaults.
