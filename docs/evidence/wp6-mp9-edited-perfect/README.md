# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| current tip | 37 | 39 | 28 | 0 | 0 | 0 | 31 | 0.9898 |

## Classes landed

1. Openband side × orthogonal pin — `openband_border_contract_fillet.step`
2. Rail-ladder digon fold→raw — `rail_ladder_digon_fold.step`
3. Incomplete-wire rail-ladder n-gon — clears raw `#2005`
4. Post-weld neighbour flip — clears `#2005/#2006` winding

## Open residual (research notes)

### Opens (37) / NM (39)
Leakiest `#1892` (orth RevolutionGrid) vs `#1891` (Coons): ~0.05 same-curve
near-misses with pins already present. Orth UV station-snap on shared rims
is implicated; skipping snap cleared near-misses (opens→13) but demoted
many orth faces (failed-floor→20). Need a snap policy that keeps shared
rim weld identity without breaking the row builder.

### Folds (28)
Led by `#375` cone drum (7).

### Planned-floor (31)
Dominated by freeform interior step / non-monotone orthogonal rejects.

## Parked experiments (regress releases)
- Broaden `samplePlanarRings` to face TopExp
- Overwrite shared rev×coons pins with contract fractions
- Skip all UV snap on shared orth samples
