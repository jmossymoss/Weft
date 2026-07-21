# WP5 — public smoke + visual status (2026-07-21)

Active package: WP5 — validate real work.

## Numeric smoke (necessary, not sufficient)

| Layer | Result |
| --- | --- |
| NIST | PASS 6/6 |
| MAMBO closed_solid + research_stress policy | PASS 13 |
| ABC nightly (resampled) | PASS 6/6 |
| `tools/release_gate.sh` | PASS |

## Visual status

**Fail.** See `docs/evidence/wp5-visual-rubric-2026-07-21.md`.

Watertight closed solids still show hair-thin triangles, messy freeform
panels, and bad hole/flat topology under CAD. A routing fix improved bored
planar collars; it does not clear section 7.

## Routing fix (this revision)

- Planar multi-wire faces are no longer grabbed by the early CAD “curved
  cutout → MinimalNGon” path.
- Ring-junction / annulus / plate-web run before residual MinimalNGon.
- Plate-web failure under `minimal` rescues to MinimalNGon (not raw).

## Remaining for WP5 exit

- Clear section 7 on release + fresh target assets (visual, not only WT).
- Fresh Plasticity exports + Blender side-by-side.
- Zoo reducers for freeform-sliver and residual fan-n-gon classes.
