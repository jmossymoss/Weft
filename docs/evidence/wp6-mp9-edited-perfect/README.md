# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| validity clears | 37 | 39 | 28 | 0 | 0 | 0 | 31 | 0.9898 |
| freeform Coons chains | 40 | 39 | 33 | 0 | 0 | 0 | 24 | 0.9921 |

## Classes landed

1. Openband side × orthogonal pin — `openband_border_contract_fillet.step`
2. Rail-ladder digon fold→raw — `rail_ladder_digon_fold.step`
3. Incomplete-wire rail-ladder n-gon + neighbour winding flip — clears `#2005` raw/winding
4. Freeform B-spline Coons chain compatibility up to 10 edges + sparse fold protect (4) for Freeform Coons — planned-floor 31→24

## Still open
- 40 unexplained opens / 39 NM (leakiest `#1892` orth×coons rim)
- 33 folds (led by `#375`)
- 24 planned-floor

Release spot-check: flaregun / foam / teleporter watertight at CAD defaults.
