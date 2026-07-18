# Evidence — WP-175 MP9 analytic residuals (2026-07-18)

## Policy

Demote analytic subclasses the certified templates do not consume:

| Tag | Meaning |
|---|---|
| `cone.non_apex_deferred` | cone trim ≠ `TouchesOneSingularity` |
| `sphere.partial_deferred` | sphere trim ≠ `TouchesTwoSingularities` |
| `cylinder.complex_boundary_deferred` | partial band with >4 edges |

`generateSecureMesh` fails closed on the first non-supported surface with
`secure_pipeline.unsupported_surface_family` (includes deferred tag) before
interval/boundary work (ADR-0014).

## Proof

- MP9 f29 (6-edge freeform): refuse
  `unsupported_surface_family (... freeform.high_edge_count_deferred)`
- MP9 f13 (6-edge cylinder band): refuse
  `unsupported_surface_family (... cylinder.complex_boundary_deferred)`
- Inventory prints `cone.*` / `sphere.*` / `cylinder.complex_*` condition
  histograms.

## Exit gate

Residual analytic refusals are named at recon time; late opaque template
refusals are no longer the primary signal for these subclasses.
