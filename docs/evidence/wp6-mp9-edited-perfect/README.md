# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| openband side-pin | 51 | 39 | 23 | 3 | 0 | 2 | 31 | 0.9892 |
| rail-ladder digon + pin skip | 37 | 39 | 28 | 0 | 0 | 1 | 31 | 0.9895 |

## Classes landed

### Openband side × orthogonal pin
Reducer: `tests/regressions/mp9/openband_border_contract_fillet.step`

### Rail-ladder digon fold → raw
Reducer: `tests/regressions/mp9/rail_ladder_digon_fold.step`

### Rail-ladder outline pin skip
Skip orthogonal station pins on edges belonging to any RailLadder plan
(same doctrine as open-band sides). Minimal n-gon rescue attempted when a
rail-ladder still misses its border contract; `#2005` still goes raw on the
full model (extract alone does not repro).

## Still open
- raw `#2005` (rail-ladder border contract on edge 5775; full-model context)
- ~37 unexplained opens / 39 NM (leakiest `#1892` with 9)
- ~28 folds (led by `#375` cone drum; digons contribute residual folds)
- 31 planned-floor (freeform interior step / non-monotone dominant)

Release spot-check: flaregun / foam / teleporter remain watertight at CAD defaults.
