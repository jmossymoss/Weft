# G3: freeform / mapped / extrusion / offset hard certify — 2026-07-20

## Package

WP-177 track G3 (surface consumers). No omit/soft seam skip.

## Changes

- `mapped_template.cpp`: hard seam match remains refuse
  (`mapped.seam_sample_unmatched`); removed freeform
  `relaxGeometryChecks` soft after lattice certify.
- `secure_meshing.cpp` (mapped/freeform/extrusion/offset branches):
  - interval consumption under-consumption soft removed (fail by name);
  - mapped refuse → UV-trim without forcing a second
    `relaxGeometryChecks` soft;
  - extrusion/offset densify by `revolutionRadialSegments` LOD;
  - n-gon UV-trim failures refuse with face id (no silent fall-through).
- `secure_core.cpp` (import coordination): single-face identity-invalid
  Plasticity extracts may enter meshing
  (`import.working.invalid_identity_mesh_allowed`) so G3 fixtures are not
  stuck on `import_not_meshable` while BRepCheck fails open shells.
- `tests/test_mapped_template.cpp`: G3 extract battery.

## Planar_* coordination (not edited)

`planar_cdt.cpp` still sets `relaxGeometryChecks` when `allowCurvedUv`
(period-unwrapped curved UV). G3 wants `validateMesh` on when ears succeed;
that flag lives in shared planar CDT — leave to G1/G5 with this note.

## Proof (vs2022 Release, OCCT on PATH)

```
weft_mapped_template_tests.exe
WEFT_G3_EXTRACT freeform_pent.step tris=47 independent=1 quads=22
WEFT_G3_EXTRACT freeform_mapped_fallback.step tris=7 independent=1 quads=2
WEFT_G3_EXTRACT bspline_139.step tris=7 independent=1 quads=2
WEFT_G3_EXTRACT offset_quad.step tris=2048 independent=1 quads=992
WEFT_G3_EXTRACT freeform_hex.step tris=64 independent=1 quads=31 uv_trim=1
mapped template checks passed
```

## Remaining codes / follow-ups

| Code / issue | Owner |
|---|---|
| `relaxGeometryChecks` from curved UV CDT | planar_cdt (G1/G5) — run validateMesh when ears succeed |
| Coverage zeroing at assemble | G5 |
| Full MP9 body without omit | WP-177 after G1–G4 |

## Exit

G3 extract acceptance met: Independent modelling quads on freeform/mapped/
offset witnesses; seam unmatched is refuse→UV-trim, not soft-skip.
