# WP5 — plate-web residual sliver class (2026-07-21)

## Class

Multi-hole planar plates under CAD leave a border-only CDT residual between
hole collars. Those triangles often sit under the chord budget while reading
as hair-thin soup (ABC `00008536` face 27; dense bolt plates).

Notched cylindrical drums that fall to the contract floor (ABC face 57) are
a separate class — still open.

## Fix

In `refineFloorWeb`:

- Preserve non-triangle polygons (hole collars) while refining residual tris.
- When a planar residual web is already mostly needles (`≥35%` of tris with
  min corner angle `< 5°`) and the face is not `pureTriFloor`, split the
  longest interior edge of each sliver and Steiner-refine on the surface.
- Pair residual tris into quads afterward (unchanged default for non-pure-tri).

`meshPlateWeb` now attaches UV anchors on border verts and runs that refine
after a successful build. Curved / demoted pure-tri floors do not take the
aggressive Steiner path (measured: it densified needles on drums).

## Numeric evidence (CAD profile)

| Asset | Setting | Before (slivers) | After (slivers) | Notes |
| --- | --- | --- | --- | --- |
| ABC `00008536` face 27 | `--density 0.35` | 371 | 0 | plate-web residual cleared |
| ABC `00008536` whole | `--density 0.35` | 809 | 438 | remainder ≈ face 57 drum floor |
| ABC `00008536` face 27 | default density | (soup) | 0 | collars + refine |
| ABC `00008536` whole | default density | 359 | 359 | face 57 still ~235 needles |
| teleporter | `--density 0.35` | 166 | 164 | no regression |
| torture | CAD default-ish | 22 | 21 | no regression |

Watertightness unchanged (`yes`) on closed solids. `tools/release_gate.sh` PASS.
`testPlateWebSliverRefine` covers a 9-hole CAD plate (`plate-web` slivers `< 5%`).

## Visual

Screenshots refreshed under `docs/evidence/wp5-visual/` (ABC + teleporter).
Outer plate flow on ABC is cleaner; the notched inner cylindrical band (face 57
contract-floor) still reads as tri soup — next class.

## Remaining (still blocks WP5 visual exit)

1. Notched / multi-feature revolution floors (ABC face 57) — needs structured
   drum routing or a curved-safe floor, not planar Steiner refine.
2. Teleporter demoted rail-ladder / ribbon floors (border-contract failures).
3. Fresh Plasticity Blender side-by-side.
