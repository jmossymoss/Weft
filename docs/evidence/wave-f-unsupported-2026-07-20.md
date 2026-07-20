# Wave F — Honest unsupported families (2026-07-20)

## Goal

Named refuse codes (not generic-only) for hyperbola/parabola/offset curves and
`kernel_specific` surfaces, with minimal committed fixtures and
`WEFT_UNSUPPORTED_MATRIX` assertions.

## Commands

```powershell
cmake --build --preset vs2022 --target weft_core weft weft_secure_meshing_tests weft_brep_consumer_matrix_tests -j 8
D:\Weft\build\vs2022\bin\Release\weft.exe fixture tests\fixtures\unsupported\curve_hyperbola.step --shape curve_hyperbola
D:\Weft\build\vs2022\bin\Release\weft.exe fixture tests\fixtures\unsupported\curve_parabola.step --shape curve_parabola
D:\Weft\build\vs2022\bin\Release\weft.exe fixture tests\fixtures\unsupported\curve_offset.brep --shape curve_offset
D:\Weft\build\vs2022\bin\Release\weft_secure_meshing_tests.exe
D:\Weft\build\vs2022\bin\Release\weft_brep_consumer_matrix_tests.exe
```

## Results

| Subclass | Fixture | Refuse code | Result |
|---|---|---|---|
| hyperbola | `tests/fixtures/unsupported/curve_hyperbola.step` | `secure_pipeline.unsupported_curve_family.hyperbola` | PASS |
| parabola | `tests/fixtures/unsupported/curve_parabola.step` | `secure_pipeline.unsupported_curve_family.parabola` | PASS |
| offset curve | `tests/fixtures/unsupported/curve_offset.brep` | `secure_pipeline.unsupported_curve_family.offset` | PASS |
| kernel_specific surface | `surface_kernel_specific.construction` (in-test GeomPlate) | `secure_pipeline.unsupported_surface_family.kernel_specific` | PASS |

Notes:

- Offset curves and GeomPlate/`OtherSurface` do not survive STEP; offset is
  committed as native BREP. GeomPlate cannot be persisted by OCCT ASCII BREP
  (`UNKNOWN SURFACE TYPE`), so kernel_specific is rebuilt in-process from the
  committed construction sidecar and imported via `buildImportedModel`.
- Deferred surface refuses now append family or preferred `*_deferred` subclass
  (`secure_pipeline.unsupported_surface_family.<name>`); never silent omit.
- Wave 0 `hardOrientUvTrimMesh` paths were not modified.

## Authority

Matrix: `docs/governance/brep-consumer-matrix.md` Wave F rows.
Marker: `WEFT_UNSUPPORTED_MATRIX` / `testUnsupportedFamilyMatrix`.
