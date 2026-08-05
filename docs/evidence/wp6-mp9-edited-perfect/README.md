# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| openband side-pin fix | 51 | 39 | 23 | 3 | 0 | 2 | 31 | 0.9892 |
| rail-ladder digon | 37 | 39 | 28 | 0 | 0 | 1 | 31 | 0.9895 |

## Classes landed

### Openband side × orthogonal pin
Reducer: `tests/regressions/mp9/openband_border_contract_fillet.step`
Skip orthogonal station pins on `bandSides`; sample sides via
`edgeSampleFractions`.

### Rail-ladder digon fold → raw
Reducer: `tests/regressions/mp9/rail_ladder_digon_fold.step`
Two-edge extrusion digons use the B-rep edges as rails (not angle tips),
carry UV anchors, equalize rail counts, majority fold demote + sparse
fold protect so the ladder is not traded for a floor/raw path.

## Still open
- raw face `#2005` (border contract failed)
- ~37 unexplained opens / 39 NM
- ~28 folds (led by `#375` cone drum)
- 31 planned-floor (freeform interior step dominant)
