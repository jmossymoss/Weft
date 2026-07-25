# WP6 — ABC multi-tooth open-band drum (2026-07-23)

## Class

Near-full analytic cylinders (`Drum × IsoBand`) with a multi-tooth
castellated rim and two full-height U-gap sides (ABC `00008536` face 57).
Prior path: open-band loft gates rejected gear-tooth plunges → Coons
over budget → contract-floor needle soup (~235+ of ~359 model slivers).

No filename / face-id specials. No new `MesherKind`.

## Fix

1. `edgesHugRimsOrInserts` with `bandSides`: skip plunge-limit and
   between-chain OUT coverage (open-band boolean-cuts teeth).
2. Drum open-band plan: allow multi-piece rims when `bandDriver` is set.
3. `meshRevolutionOpenBand`:
   - rim assignment by surface-bound touch (not mean V);
   - raise `nu` for ≥3 notches; raise side `nv` for castellated cut rims;
   - clamp side-crowding regions; tighten region merge / pad;
   - enforce disjoint column spans; skip inverted strip gaps.
4. Sparse-fold protect: open-band drums keep structured lattice up to
   `max(8, n/8)` folds instead of losing to a zero-fold floor web.

## Metrics (CAD profile)

| Asset | Before | After |
| --- | --- | --- |
| `notched_drum_iso_band_r0` | contract-floor, ~248 slivers | revolution-grid, 12 slivers |
| ABC `00008536` whole | 359 slivers, face 57 floor | 136 slivers, face 57 revolution-grid |
| ABC `00008536` face 57 | `contract-floor` / planned | `revolution-grid` build=0 |

## Tests

- `testNotchedDrumOpenBand` — `tests/regressions/abc/notched_drum_iso_band_r0.step`
