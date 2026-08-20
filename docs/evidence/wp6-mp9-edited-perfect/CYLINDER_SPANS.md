# CAD cylinder span floor (min 24) + density raise/lower

## Policy

- `minCurvedSegments` = **24** (defaults / `--profile cad` / app)
- All Drum faces (plain and notched) take that floor on closed rings
- IsoBand walls with wrap ≥ 0.75 and ≥16 edges raise column seams to
  `ceil(24×wrap)`, and to 24 when shared with a FullPeriod neighbour
- Adapt-off radial pins below 24 win (density down works)
- Annulus-body skipped when dense/notch ratio > 1.2 or notch height >
  45% of band (avoids self-check on density edits)
- Non-drum closed rings stay at legacy floor 6

## Object 5 (CAD defaults)

| Face | nu | Notes |
|------|---:|-------|
| 364/366/368 | 24 | simple cylinders |
| 362 | 28 | notched |
| 374 | 24 | IsoBand wrap floor |
| 375 | 25 | FullPeriod neighbour |

## Density raise/lower (demo.step kind harness)

| Kind | Down | Up |
|------|------|-----|
| revolution-grid | ok (f7/f8) | ok |
| coons-grid (loops) | ok | ok |
| annulus-ring | ok | ok |
| plate-web | ok | ok |

mp9: watertight, failed-floor=0, folded=0, structured 3052/3052.

## Locks

- `testMp9EditedWatertight` at minCurved=24
- Screenshots: `objects/cyl24/`
