# Evidence — MP9 mesher Phase 1 (2026-07-18)

## Results

- `meshable=1`, `supported_faces=3969` / ~4270 (was blocked at import gates).
- Extracts:
  - `tests/fixtures/mp9_extracts/cylinder_band.step` (face 10) → 16 tris
  - `tests/fixtures/mp9_extracts/freeform_pent.step` (face 28) → 512 tris
- Analytic UV derivation via `projectPointToSurface` (ElSLib) when STEP omits
  p-curves.
- Reflection-aware cylinder rim registration.
- Face probes (sample): many planes/cylinders/freeforms OK; residuals are
  ellipse rim composites, curved-rail bands, mapped seam unmatched, cone
  non-apex, torus seam conflict, extrusion/offset.

## Next

WP-173+ freeform/mapped residuals; composite cylinder rims; extrusion/offset
policy; full-body ADR-0014 gate.
