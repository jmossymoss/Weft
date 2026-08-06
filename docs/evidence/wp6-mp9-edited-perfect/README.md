# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| validity clears | 37 | 39 | 28 | 0 | 0 | 0 | 31 | 0.9898 |
| freeform Coons chains | 40 | 39 | 32 | 0 | 0 | 0 | 26 | 0.9915 |
| orphan seam absorb | 36 | 39 | 32 | 0 | 0 | 0 | 26 | 0.9915 |
| freeform Coons ≤14 edges | 36 | 39 | 33 | 0 | 0 | 0 | 25 | 0.9918 |

## Classes landed

1. Openband side × orthogonal pin — `openband_border_contract_fillet.step`
2. Rail-ladder digon fold→raw — `rail_ladder_digon_fold.step`
3. Incomplete-wire rail-ladder n-gon + neighbour winding flip — `#2005`
4. Freeform B-spline Coons chain compatibility (≤14 edges) + Freeform sparse-fold
   protect (≤4) — reducer `freeform_coons_chain_panel.step`

## Still open
- 40 unexplained opens / 39 NM (leakiest `#1892` orth×coons rim)
- 32 folds (led by `#375`)
- 26 planned-floor (freeform interior step / no fourth corner / drum inset `#65`)

## Parked experiments
- Orth shared-rim UV snap skip (opens↓, failed-floor↑)
- Shared rev×coons pin overwrite (release failed-floor)

## Open-edge diagnosis (#1891/#1892)

Shared B-rep edge 5482: stitch sees sides 27/31 (coons/orth), not equal.
Coons candidate border segments on the curve: 26, all already shared with
orth (`open=0`). Orth has 4 open segments. Orth emits exclusive rim verts
that do not fall between any Coons border chord on this curve — Coons is
not covering the full pin param range on the shared rim. Next: force
Coons border sampling to emit every pin fraction on 2-owner edges shared
with an orthogonal RevolutionGrid.

### Orphan seam absorb (landed)

`absorbOrphanSeamStations` after stitch collapses open orth×coons
exclusive stations onto the partner within 0.12. Opens 40→36;
#1892 opens 9→5 (residual: 1×#1886 long T-junction + 4×#1891).
Coons chained sides now keep every pin station (`includeLast`).
