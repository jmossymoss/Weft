# Evidence — WP-173 freeform subclass expansion (2026-07-18)

## Policy

Rectangular UV-grid freeform is certified only for ≤5 outer edges (proven on
MP9 face 28 / `freeform_pent` and the `freeform_patch` fixture).

Faces with ≥6 edges on an otherwise UV-grid-eligible trim are tagged
`freeform.high_edge_count_deferred` and stay `DeferredResidualSurface`.

## Proof

| Extract | Edges | Result |
|---|---|---|
| `freeform_pent.step` (f28) | 5 | meshes (512 tris) |
| `freeform_hex.step` (f29) | 6 | deferred `freeform.high_edge_count_deferred` |
| f35/f36 | 4 | mesh |
| f49/f52 | 8–9 | would have been opaque `seam_sample_unmatched`; now deferred by edge count |

`WEFT_FREE_HIGH_EDGE` in `secure_meshing` tests.

## Exit gate

Top freeform residual bucket named; rectangular ≤5-edge subclass remains
supported.
