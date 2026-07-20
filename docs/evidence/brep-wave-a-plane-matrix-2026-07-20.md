# Wave A — plane consumer matrix lock — 2026-07-20

## Goal

Lock plane subclass extract battery fail-closed under `WEFT_PLANE_MATRIX`.
No `allowCurvedUv` on planes. No full-body MP9.

## Battery (`testPlaneMatrix` in `planar_trim_assembly`)

| Extract | Subclass | tris | Outcome |
|---|---|---|---|
| `plane_multi.step` | simple / multi-outer | 2 | HARD |
| `plane_2732.step` | digon UV-strip recovery | 6 | HARD |
| `plane_3605.step` | perforated / filleted-slot residual | 151 | HARD |
| `plane_3821.step` | ellipse densify ≥48 | 98 | HARD |
| `plane_1793.step` | self-intersect candidate recovery | 18 | HARD |

Contract: `omitDeferredResiduals=false`; hole-fixture assemblies assert
`allowCurvedUv=0`. Individual markers `WEFT_G1_PLANE2732` / `3605` / `3821`
remain.

## Commands

```
cmake --build --preset vs2022 --target weft_planar_trim_assembly_tests `
  weft_brep_consumer_matrix_tests weft_planar_cdt_tests `
  weft_planar_trim_validation_tests
ctest --preset vs2022 -R "brep_consumer_matrix|planar_trim_assembly|planar_cdt|planar_trim_validation" --output-on-failure
```

Result: 4/4 PASS. Marker line:
`WEFT_PLANE_MATRIX locked=5/5 fail_closed=1 allowCurvedUv=0`.

Authority: `docs/governance/brep-consumer-matrix.md` Wave A rows all HARD.
Scaffold `brep_consumer_matrix` presence checks still green.

## Out of scope

Full MP9 body; Wave B–F; `secure_meshing.cpp` edits (ellipse densify already
landed for 3821).
