# WP6 — mp9_Edited perfect-topology campaign

Source tip: structured Weft (accuracy-tessellator excluded).

## Phase 1 baseline (`4a77e4f`)

```sh
build/cli/weft mesh tests/STEP_Examples/mp9_Edited.stp \
  -o /tmp/mp9e.obj --profile cad --validate --why
```

| metric | baseline |
|--------|------:|
| open edges (unexplained) | 54 |
| non-manifold | 39 |
| folded polygons | 23 |
| winding conflicts | 3 |
| planned-floor | 31 |
| failed-floor | 1 |
| raw | 2 |
| retention | 0.9889 |

## Class: openband side × orthogonal pin

Reducer: `tests/regressions/mp9/openband_border_contract_fillet.step`

Root cause: `pinOrthogonalTrimGrids` pinned open-band *side* meridians with
station crossings, so the pin set outgrew the band's `nv`. Open-band
`sampleSide` emitted uniform `nv` samples while the border contract demanded
pin stations → `border contract failed` / failed-floor.

Fix: skip orthogonal pins on edges that are `bandSides` of a RevolutionGrid
open band; sample sides via `edgeSampleFractions` (shared contract).

### After fix

| metric | mp9_Edited |
|--------|------:|
| open edges | 51 |
| non-manifold | 39 |
| folded | 23 |
| failed-floor | 0 |
| raw | 2 |
| planned-floor | 31 |
| retention | 0.9892 |

Reducer: structured=9, failed-floor=0, retention=1.0.
