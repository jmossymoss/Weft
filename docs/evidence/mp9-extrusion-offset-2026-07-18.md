# Evidence — WP-174 extrusion/offset policy (2026-07-18)

## Policy

1. Four-sided non-periodic extrusion/offset → `mapped.four_sided_candidate`
   with GeomAPI surface projection when p-curves are missing.
2. Periodic extrusion/offset bands → named
   `extrusion.periodic_band_deferred` / `offset.periodic_band_deferred`
   (UV rectangles self-intersect in 3D).
3. Non-four-sided → `extrusion.non_four_sided_deferred` /
   `offset.non_four_sided_deferred`.

## Proof

- MP9 face 250 (`offset_quad.step`): 4-edge offset → 2048 tris.
- MP9 face 1932: full-periodic extrusion → `extrusion.periodic_band_deferred`.
- MP9 face 464: 11-edge extrusion → `mapped.non_four_sided_deferred` +
  `extrusion.non_four_sided_deferred`.

## Exit gate

No silent extrusion/offset path; certify four-sided non-periodic or stable
named refusal.
