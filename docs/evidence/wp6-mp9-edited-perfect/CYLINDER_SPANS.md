# CAD cylinder span floor (min 24)

## Policy

- `minCurvedSegments` default / CAD profile / app session: **24**
- Simple full-period cylinders (≤4 edges): circumferential spans ≥ 24
- Curvature adaptive still raises above 24 when the model-diagonal chord
  demands it (`n = max(24, nCurv)`)
- Boolean/notched multi-edge drums keep the legacy floor of 6 (forcing 24
  self-checks — mp9 #1611/#1613)
- IsoBand wrap floor (`ceil(24×wrap)`) deferred — shared-rim raises break
  FullPeriod neighbours (mp9 f374/f375)

## Object 5 (after)

| Face | nu | Notes |
|------|---:|-------|
| 364 / 368 | 24 | simple hole/wall floors |
| 366 | 24 | small bore floor |
| 362 | ~20 | complex/notched cone |
| 374 | 6 | IsoBand — wrap floor deferred |
| 375 | ~20 | MinimalNGon / complex |

## Locks

- `testMp9EditedWatertight` uses `minCurvedSegments=24` and asserts simple
  full-period RevolutionGrid drums have `nu ≥ 24`
- Screenshots: `objects/cyl24/`
