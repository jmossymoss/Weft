# WP6 — freeformComb shared-edge border snap (2026-07-23)

## Class

FreeformComb orthogonal-trim lattices emit border verts via `surf.Value` at
station-snapped UV. On shared seams those 3D points drift ~0.03–0.85 mm off
the B-rep edge while partner Coons / plane samples sit on-curve → unexplained
opens (#1805 family on `coons_plane_1805_r0`, and the same class across MP9).

Tried and rejected on this class (folds / NM / MP9 regressions):

- bare Extrema onto every face edge (notch collapse);
- UV-gated emission-time curve Value / Extrema without shared-only filter;
- widening fuse/stitch on-curve bands to 1.0 mm globally (MP9 opens rose).

## Fix

In `meshOrthogonalTrimGrid` freeformComb post-pass:

1. Keep exact-contract snap within 0.25 mm (partner identity).
2. Else project mesh-boundary verts onto a dense polyline of **2-owner**
   face edges only, within 1.0 mm (shared-seam drift repair; skips
   open-shell notch rails).

No filename / face-id specials. No new `MesherKind`.

## Metrics

| Probe | Before | After |
| --- | --- | --- |
| `coons_plane_1805_r0` unexplained | 73 | 52 |
| same, folds / NM | 0 / 0 | 0 / 0 |
| MP9 CAD open edges | ~378–383 | **247** |
| MP9 leakiest | #1805 family led | #1805 off top; #3434/#1904 lead |
| foam / teleporter CAD | watertight | watertight |

## Tests / inventory

- `testMp9CoonsPlaneSeamCanonicalize` — unexplained `< 55` (was `< 75`)
- `tests/KNOWN_RED.tsv` MP9 `max_open_edges` ceiling 450 → 300

## Residual

Shared-seam T-junctions remain on the reducer (edges 4/5/41) and on MP9
(~247 opens, some NM). Next leverage is stitch insert success on count-
mismatched freeformComb↔partner seams without widening contamination bands.
