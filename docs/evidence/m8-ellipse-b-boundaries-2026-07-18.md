# Evidence — WP-171 ellipse curve boundaries (2026-07-18)

## Change

- `ellipse` curve family promoted to first-template supported.
- Canonical critical segmentation admits ellipse (closed quarter-turn lattice;
  open arcs domain/contact only).
- Interval demand via `ellipticalArcSegmentCount` (major-radius circle bound).
- Cylinder rim resolution accepts ellipse rims (planar section of a cylinder).

## Proof

- Fixture `ellipse_hole` (planar face + exact elliptical inner wire):
  `weft mesh` → 30 tris.
- `WEFT_ELLIPSE_F` in `secure_meshing` test.
- MP9 extract `tests/fixtures/mp9_extracts/cylinder_ellipse.step` still named-
  refuses `cylinder_rims_unresolved` when only one rim is an ellipse (other
  side is not a second open circular/elliptical arc) — correct fail-closed.

## Exit gate

Ellipse edges build canonical boundaries on supported faces; unsupported
composites refuse by name.
