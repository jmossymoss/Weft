# Capsule slots lose their round to a trim-grid pin (2026-07-25)

Artist report: the MP9 muzzle's capsule slots "are supposed to be capsules, but
the hole doesn't have that at all" — the ends read as flat faces.

## Root cause (measured, not inferred)

The counts are already correct. Probing the four sides of a capsule end wall
(muzzle extract face 6, a quarter-cylinder) gives:

```
sideprobe face 6: nu=11 nv=1 sides e38=11 e35=1 e37=11 e36=1 natB=2 ...
```

11 on both arcs, 1 on both straight rulings — exactly right for a round wall
one division deep. Nothing is mismatched.

What flattens it is a PIN. The drum plans as an orthogonal trim grid, and
`pinOrthogonalTrimGrids` pins every shared boundary edge to its endpoints plus
its crossings with the grid's station lines. A capsule end arc crosses no
station line, so it is pinned to exactly two fractions, 0 and 1. A pin is the
authoritative sampling for an edge wherever it is read, so:

- the drum draws that arc as one straight chord (chamfered slot end), and
- the end wall's side collapses to 2 points, leaving no lattice, so it ships as
  a single crescent n-gon.

Both symptoms, one cause.

## The change

Seed the pin with the edge's OWN solved samples (curved edges only — interior
samples on a line carry no shape) in addition to the station crossings. The
curve's count is what makes it round; the stations only add the crossings the
clip needs.

Result on the muzzle extract:

| | before | after |
| --- | --- | --- |
| capsule end wall (face 6) | 1 n-gon | 10 quads + 1 pentagon |
| drum slot cells (face 21) | 9 n-gons | 21 pentagons + 3 hexagons |
| MP9 coons/plane extract cracks | 52 | 26 |

![capsule ends](capsule_ends_before_after.png)

## Why this is NOT merged

It regresses release invariants, so it is parked here rather than landed:

- `foam` (both profiles): 2 winding conflicts, 3 open edges
- `teleporter` (cad): 2 winding conflicts
- `testMp9CoonsPlaneSeamCanonicalize`: `floors <= 1` exceeded
- `testSparseFoldKeepsStructuredCharts` fails

The winding conflicts sit on foam faces 508 (cone, drum/iso-band) and 525
(cylinder, drum/iso-band), which share edges. Each face is internally oriented
against its own surface normal — the trim grid already normalizes cell winding
that way — so this is a CROSS-FACE disagreement that the denser shared sampling
exposes, not a missing sign check. It needs its own investigation.

Narrowing the seeding to curved edges, and then further to RevolutionGrid
plans only, did not clear the fallout.

## Reproducer

```sh
weft extract tests/STEP_Examples/MP9.stp --faces 3432 --rings 1 -o muzzle.step
weft mesh muzzle.step -o muzzle.obj --profile cad     # ~1s
python3 tools/render_obj_wireframe.py muzzle.obj ends.png --faces 6,7,10,11
```
