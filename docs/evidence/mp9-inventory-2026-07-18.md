# MP9 inventory evidence - 2026-07-18

## Source

Local Windows run (OCCT 8.0) committed under `weft logs/`:

- `mp9-20260718-181534.out.log`
- `mp9-20260718-181534.err.log`
- `mp9-gaps-20260718-181534.tsv`

## Headline

| Metric | Value |
|---|---|
| Import | ~31–33 s (after repair O(n×model) fix) |
| Recon subjects | 15269 |
| Faces | 4270 |
| `meshable` | **0** (body-level) |
| Supported faces | 1523 (planes only) |
| Unsupported subjects | 2958 |
| Body mesh probe | `secure_pipeline.import_not_meshable` |

This is why the app shows B-rep lines only: regenerate refuses before any face template runs.

## Face families

| Family | Count | Support |
|---|---|---|
| plane | 1523 | supported_analytic_template |
| bspline | 1233 | deferred_residual_surface |
| cylinder | 1004 | deferred_residual_surface |
| torus | 221 | deferred_residual_surface |
| cone | 184 | deferred_residual_surface |
| sphere | 70 | deferred_residual_surface |
| extrusion | 18 | deferred_residual_surface |
| offset | 17 | deferred_residual_surface |

## Curve blockers

| Family | Count |
|---|---|
| ellipse | 211 |

## Notable condition tags

| Code | Count |
|---|---|
| cutout.multi_bore_cylinder_deferred | 1004 |
| cutout.filleted_slot_deferred | 2527 |
| cutout.cylindrical_bore | 852 |
| freeform.uv_grid_candidate | 986 |
| mapped.four_sided_candidate | 701 |
| mapped.non_four_sided_deferred | 550 |
| freeform.general_deferred | 247 |

BSpline faces often carry `freeform.uv_grid_candidate` / `mapped.four_sided_candidate` but remain deferred — representation readiness (stored p-curve mappings) fails, so templates are not promoted.

## Plane trim mix (even among “supported”)

| Trim | Count |
|---|---|
| multiple_disconnected_domains | 882 |
| convex_simple_region | 387 |
| concave_simple_region | 138 |
| self_intersecting_or_invalid | 55 |
| annulus | 37 |
| simple_disk | 21 |
| multiply_perforated_disk | 3 |

## Ordered work to get faces (not just B-rep)

1. **Meshable gate / SameParameter** — Plasticity STEP leaves many edges without p-curves or unproven SameParameter; whole-model `meshable=0` blocks everything. Softened: no-pcurve edges warn only (see follow-up commit).
2. **P-curve / representation readiness** — until curved faces have evaluable p-curves (or a certified alternative), cylinder/cone/sphere/torus/bspline stay deferred.
3. **Ellipse curves** (211) — body-level unsupported_curve_family once meshable.
4. **Multi-bore / filleted cut-graphs** — all cylinders tagged multi_bore deferred.
5. **Freeform/mapped subclasses** — 986 uv-grid candidates + 701 four-sided once representation-ready.
6. **Partial sphere/torus/cone** — open_or_gapped_loop trims.
7. **Offset / extrusion** — small counts, named policy.
8. **Plane multidomain / invalid trims** — 882 + 55 may still refuse planar CDT after support.

## Probes

Single-face plane extracts mesh (`ok tris=2`). Cylinder/cone/sphere/torus/bspline face extracts refused `import_not_meshable` under the pre-softening gate.

## Follow-up (same day)

App certificate on conservative MP9 shows `source valid: no`, `working valid: no`,
`identity: yes`, `meshable: no`. Compatibility no longer crashes; it refuses
with `compatibility_scale_refused` (>2000 faces).

Commit allowing `meshable=true` for identity-invalid pairs (with warning
`import.working.invalid_identity_mesh_allowed`) so regenerate can enter the
pipeline; unsupported faces still refuse individually.
