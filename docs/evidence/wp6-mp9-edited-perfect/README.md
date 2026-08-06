# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| validity clears | 37 | 39 | 28 | 0 | 0 | 0 | 31 | 0.9898 |
| freeform Coons chains | 40 | 39 | 32 | 0 | 0 | 0 | 26 | 0.9915 |

## Classes landed

1. Openband side × orthogonal pin — `openband_border_contract_fillet.step`
2. Rail-ladder digon fold→raw — `rail_ladder_digon_fold.step`
3. Incomplete-wire rail-ladder n-gon + neighbour winding flip — `#2005`
4. Freeform B-spline Coons chain compatibility (≤10 edges) + Freeform sparse-fold
   protect (≤4) — reducer `freeform_coons_chain_panel.step`

## Still open
- 40 unexplained opens / 39 NM (leakiest `#1892` orth×coons rim)
- 32 folds (led by `#375`)
- 26 planned-floor (freeform interior step / no fourth corner / drum inset `#65`)

## Parked experiments
- Orth shared-rim UV snap skip (opens↓, failed-floor↑)
- Shared rev×coons pin overwrite (release failed-floor)
