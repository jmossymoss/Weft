# G3: mapped hard seam refuse → UV-trim — 2026-07-19

## Change
- `mapped.seam_sample_unmatched` is fail-closed (no freeform soft-skip).
- Caller UV-trim fallback forces `allowCurvedUv` and reports trim/CDT codes.
- Annulus: sole-outer holes assigned without winding; curved UV nearest hole bridge when exact cones fail.
- Freeform mapped lattice still uses scoped `relaxGeometryChecks` for orientation.

## Proof
- `bspline_139.step` (annulus): meshes via UV-trim after mapped refuse
- MP9: EXIT 0 with `omitDeferred=false`
