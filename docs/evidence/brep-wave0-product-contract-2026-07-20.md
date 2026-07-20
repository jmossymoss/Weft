# Wave 0 — product UV-trim / soft-arm contract — 2026-07-20

## Goal

Cross-cutting product contract (`omitDeferredResiduals=false`):

1. Every UV-trim success path exits with `relaxGeometryChecks=false` and
   `windingsMatchOrientedFaceNormal=true`, else named refuse (no
   `windingsMatch=0` residual admission).
2. `hardOrientUvTrimMesh` never drops minority triangles; refuse instead.
3. `previewFast` / discrepancy widen / assembly tol bump arm only via
   `--allow-partial-body` (`omitDeferredResiduals`) or explicit preview —
   never from `faceCount > 500` alone.
4. Shared curved CDT may set `relaxGeometryChecks`; product consumers clear
   via hardOrient or refuse before assemble
   (`secure_pipeline.relaxed_geometry_on_product_path`).

Wave A plane matrix and Wave F subclass refuse codes are preserved
(out of scope to revert).

## Revision

Working tree on `cursor/open-tasks-processing-238d` atop
`750a2abadf0bc6fae76a1bd2a3286aa5bca6684a`.

## Audit summary (finish, not rewrite)

| Contract | Status |
|---|---|
| UV-trim → hardOrient or named refuse (cyl/cone/sphere/torus/freeform/mapped) | DONE |
| `hardOrientUvTrimMesh`: majority global flip only; `uvAgainst!=0` → refuse | DONE |
| CapWall: refuse `sphere.cap_orientation_unresolved` (no minority drop); UV-trim fallback hard-orients | DONE |
| `previewFast` + discrepancy widen gated on `omitDeferredResiduals` + coverage `preview_fast_boundaries` | DONE |
| Assemble tol bump only under omit; no `faceCount>500` product soft arm | DONE |
| Pre-assemble product refuse if any face retains `relaxGeometryChecks` | DONE |
| Wave F `unsupported_curve_family.<family>` / `unsupported_surface_family.kernel_specific` | preserved |
| Wave A `WEFT_PLANE_MATRIX` / planar_* | preserved |

Markers observed: `windingsMatch=1 relax=0` on success; `windingsMatch=0 (refuse)`
on hardOrient failure (invert-retry or named refuse — never residual admit).
`WEFT_WAVE0 product contract: omit=0 previewFast=0; omit=1 previewFast=1; no faceCount>500 soft arm`.

## Commands

```
cmake --build --preset vs2022 --target weft_secure_meshing_tests `
  weft_mapped_template_tests weft_cylinder_template_tests `
  weft_sphere_template_tests weft_planar_cdt_tests weft_certified_mesh_tests
ctest --preset vs2022 -R "^(secure_meshing|mapped_template|cylinder_template|sphere_template|planar_cdt|certified_mesh)$" --output-on-failure
```

Result: 6/6 PASS (real ≈ 37s).

## Primary files

- `core/src/secure_meshing.cpp` — hardOrient; omit-only preview/tol; product relax gate
- `core/src/certified_mesh.cpp` — windingsMatch grazing vote aligned with hardOrient
- `core/src/sphere_template.cpp` — CapWall refuse-not-drop + grazing skip
- `core/include/weft/planar_cdt.hpp` — Wave 0 relax comment
- `tests/test_secure_meshing.cpp` — `testG0FailClosedDefaults` / `WEFT_WAVE0`

## Out of scope

Full MP9 body mesh; Wave B–E matrix batteries; commits / plan edits.
