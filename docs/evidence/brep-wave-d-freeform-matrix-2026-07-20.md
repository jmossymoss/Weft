# Wave D — freeform / mapped / extrusion / offset / revolution — 2026-07-20

## Goal

Close the revolution n≠4 UV-promote gap; lock general mapped/UV orientation
(no face-id special cases) under `WEFT_MAPPED_MATRIX` /
`WEFT_FREEFORM_MATRIX` on existing extracts plus extrusion/revolution fixtures.

## Product changes

- `secure_reconnaissance.cpp`: n≠4 `revolution` gets
  `freeform.uv_trim_candidate` + `revolution.uv_trim_attempted` (same promote
  as extrusion/offset). Support flips to SupportedAnalyticTemplate when UV-ready.
- `planar_trim_assembly.cpp`: allow `revolution` in the UV-surface gate
  (was extrusion/offset-only; without this, promote hit
  `trim_assembly.face_not_uv_surface`).
- `secure_meshing.cpp` (mapped/freeform/revolution/extrusion/offset only):
  n-gon UV fallback includes `revolution`; mapped densify LOD includes
  `revolution`. Wave 0 `hardOrient*` and Wave F refuse helpers untouched.
- Fixtures: `extrusion_quad` (MakePrism bspline edge), `revolution_ngon`
  (five-sided UV trim on SurfaceOfRevolution).

## Battery (`testMappedFreeformMatrix` in `mapped_template`)

| Marker | Subject | Outcome |
|---|---|---|
| `WEFT_MAPPED_MATRIX` | `face_33` / `103` / `107` orient extracts | HARD |
| `WEFT_MAPPED_MATRIX` | fixture `extrusion_quad` | HARD |
| `WEFT_MAPPED_MATRIX` | `offset_quad.step` | HARD |
| `WEFT_FREEFORM_MATRIX` | `freeform_pent/hex`, `crash_face_136_r0`, `freeform_135` | HARD |
| `WEFT_FREEFORM_MATRIX` | `freeform_mapped_fallback`, `bspline_139` | named refuse `mapped.uv_trim_orientation_unresolved` |
| `WEFT_FREEFORM_MATRIX` | fixture `revolution_ngon` | HARD + `uv_promote=1` |

Individual `WEFT_G3_*` markers remain.

## Commands

```
cmake --build --preset vs2022 --target weft_mapped_template_tests `
  weft_brep_consumer_matrix_tests weft_planar_trim_assembly_tests
ctest --preset vs2022 -R "mapped_template|brep_consumer_matrix|planar_trim_assembly" --output-on-failure
```

## Out of scope

Full MP9 body; Wave E assembly; commits.
