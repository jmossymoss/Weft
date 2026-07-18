# Evidence — UV-trim consumers for complex bands / freeform n-gons (2026-07-18)

## Consumer

Curved faces that are not a clean structured band/grid assemble a UV trim
domain (`assemblePlanarTrimDomain` with `allowCurvedUv`) and triangulate with
the Lawson reference CDT (skipping planar nesting proofs).

## Admitted classes

| Class | Tag / route | Witness |
|---|---|---|
| Complex cylinder band (>4 edges, 2 rims) | UV-trim CDT | MP9 f13 → 6 tris |
| Freeform n-gon (≥6 edges) | `freeform.uv_trim_candidate` | MP9 f29 → 64 tris |
| Simple sphere cap (≤2 edges) | `buildSphericalCapWall` | reserved |
| Complex sphere cap | `sphere.complex_cap_deferred` | still residual |

## Fixtures

- `tests/fixtures/mp9_extracts/cylinder_complex.step`
- `tests/fixtures/mp9_extracts/freeform_hex.step`

## Quad note

UV-trim CDT emits triangles (certified floor). Modelling may pair some;
structured bands remain preferred for Independent all-quad lattices.
