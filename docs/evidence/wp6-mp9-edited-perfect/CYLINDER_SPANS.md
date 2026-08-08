# CAD cylinder span floor (min 24)

## Policy

- `minCurvedSegments` default / CAD profile / app session: **24**
- All Drum faces (plain and notched/boolean) take the 24-span floor on
  closed circumferential rings via `drumRingFloor`
- Curvature adaptive can still raise above 24 when the model-diagonal
  chord demands it (`n = max(24, nCurv)`)
- Non-drum closed rings (fillet circles) keep the legacy floor of 6
- Notched drums with a plain rim at 24 vs a shorter notch chain use
  annulus-body with `nvBody ≥ 2` so the reduction band does not
  double-cover (mp9 #1611/#1613)
- IsoBand wrap floor (`ceil(24×wrap)`) still deferred — shared-rim raises
  reopen seams (mp9 f374/f375)

## Object 5

| Face | nu | Notes |
|------|---:|-------|
| 364 / 366 / 368 | 24 | simple cylinders at floor |
| 362 | ≥24 | notched cone |
| 374 | ~11 | IsoBand — wrap floor deferred |
| 375 | ~20+ | complex / MinimalNGon |

## Locks

- `testMp9EditedWatertight` at `minCurvedSegments=24`: structured drums
  with ≤8 edges have `nu ≥ 24`; heavier notches ≥3 with structured build
- Screenshots: `objects/cyl24/`
