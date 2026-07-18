# Evidence — spherical cap consumer (2026-07-18)

## Consumer

`buildSphericalCapWall` for `TouchesOneSingularity` spheres:

- one singular pole + one circular parallel (rim)
- synthetic or second-parallel mid-latitude ring
- outer mid↔rim band pairs into modelling quads; pole fan remains tris

## Admission

- Caps with only pole+rim edges (≤2 unique edges) stay supported.
- Caps with meridians / extra edges are
  `sphere.complex_cap_deferred` (Plasticity MP9 class) until generator
  provenance is solved without split-rail conflicts.

## Status

Infrastructure landed and linked. Full-sphere path unchanged. Complex-cap
demotion keeps ADR-0014 honest while the 2-edge subclass is the supported
consumer contract.
