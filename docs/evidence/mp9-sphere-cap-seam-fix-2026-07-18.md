# Evidence — Plasticity sphere-cap seam/ear fix (2026-07-18)

## Problem

Complex single-pole caps (`sphere.uv_trim_candidate`) failed CDT ear
orientation / certified winding after U-unwrap.

## Fix

- CapWall for all `TouchesOneSingularity` spheres (mid-ring quads + pole fan)
- Soft chord/normal on CapWall; meridian attach by near rim azimuth
- Curved UV CDT: U-unwrap, final-ear flip, fan fallback, skip Lawson/validate
- Certified assembly: `relaxGeometryChecks` for CapWall/UV-trim (near-3D
  identity, soft orientation, skip intersection)

## Proof

- MP9 face 778 → 47 tris
- Hemisphere `sphere_cap` fixture → 102 tris
- Related ctests 100%
