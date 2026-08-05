# WP6 — mp9_Edited perfect-topology campaign

Structured Weft only (accuracy-tessellator excluded). Tip continues from
`cursor/axial1-straight-columns-fff5`.

## Metrics (CAD default)

| revision | opens | NM | folds | winding | failed-floor | raw | planned-floor | retention |
|----------|------:|---:|------:|--------:|-------------:|----:|--------------:|----------:|
| baseline `4a77e4f` | 54 | 39 | 23 | 3 | 1 | 2 | 31 | 0.9889 |
| openband + digon | 37 | 39 | 28 | 0 | 0 | 1 | 31 | 0.9895 |
| incomplete-wire n-gon | 37 | 39 | 28 | 8 | 0 | 0 | 31 | 0.9898 |
| n-gon neighbour flip | 37 | 39 | 28 | 0 | 0 | 0 | 31 | 0.9898 |

## Classes landed

### Openband side × orthogonal pin
Reducer: `tests/regressions/mp9/openband_border_contract_fillet.step`

### Rail-ladder digon fold → raw
Reducer: `tests/regressions/mp9/rail_ladder_digon_fold.step`

### Incomplete-wire rail-ladder → contract n-gon + neighbour winding flip
`#2005`: WireExplorer reports 2 edges but the face owns 6. Emit a
contract-sampled boundary n-gon from every face edge. Post-weld, if that
n-gon conflicts with a small neighbour (`#2006`), flip the neighbour when
it reduces global winding conflicts.

## Still open
- ~37 unexplained opens / 39 NM (leakiest `#1892` with 9)
- ~28 folds (led by `#375` cone drum)
- 31 planned-floor (freeform interior step / non-monotone dominant)

Release spot-check: flaregun / foam / teleporter watertight at CAD defaults.
