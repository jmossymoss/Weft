# Wave B — cylinder consumer matrix lock — 2026-07-20

## Goal

Lock cylinder subclass extract battery fail-closed under `WEFT_CYLINDER_MATRIX`.
Hard UV-trim (`relax=0` via Wave-0 hardOrient). No full-body MP9.

## Battery (`testCylinderMatrix` in `cylinder_template`)

| Extract | Subclass | tris | Outcome |
|---|---|---|---|
| `cylinder_band.step` | full periodic band | 64 | HARD |
| `cylinder_complex.step` | complex / partial UV-trim | 66 | HARD |
| `cylinder_ellipse.step` | ellipse rim | 18 | HARD |
| `cylinder_ellipse_band.step` | ellipse band | 18 | HARD |
| `cyl_24.step` | split open 4-semicircle rims | 128 | HARD |
| `cyl_24_from_mp9.step` | split-rim (MP9-sourced) | 128 | HARD |
| `cyl_filleted_slot_bore.step` | multi-bore / filleted-slot bore | 64 | HARD |

Contract: `omitDeferredResiduals=false`; non-vacuous
`certified.triangle_intersection` (no soft/vacuous intersection from leftover
`relaxGeometryChecks`). Individual marker `WEFT_G2_CYL24` remains.

## Commands

```
cmake --build --preset vs2022 --target weft_cylinder_template_tests `
  weft_brep_consumer_matrix_tests
ctest --preset vs2022 -R "brep_consumer_matrix|cylinder_template" --output-on-failure
```

Result: 2/2 PASS. Marker line:
`WEFT_CYLINDER_MATRIX locked=7/7 fail_closed=1 relax=0`.

Authority: `docs/governance/brep-consumer-matrix.md` Wave B rows all HARD.
Scaffold `brep_consumer_matrix` presence checks still green.

## Out of scope

Full MP9 body; Wave C–E; `secure_meshing.cpp` / Wave 0 hardOrient / Wave F
refuse-code edits.
