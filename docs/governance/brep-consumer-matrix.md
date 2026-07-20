# B-rep consumer matrix

Living authority for family × subclass → product consumer → extract fixture →
HARD / REFUSE. Fail-closed MP9 is an integration gate only after matrix rows
are green—not a face-id chase driver.

Status vocabulary for Outcome: `HARD` (fail-closed certify), `REFUSE` (stable
named refuse + extract), `OPEN` (row not locked), `GAP` (known hole vs
industrial models).

Product path means `omitDeferredResiduals=false` (no `--allow-partial-body`).
Default **floor policy** is `HardSurfaceFloor` (see below). `--strict-all-faces`
restores Wave 0 full hardOrient. No omit / OCCT soup / weld / silent heal on
the hard floor.

Related: [milestones.md](milestones.md), [ADR-0014](../adr/ADR-0014-atomic-secure-meshing-orchestration.md),
`tests/fixtures/mp9_extracts/`, Wave agents fill `WEFT_*_MATRIX` assertions.

---

## Product default: HardSurfaceFloor + soft residuals

Games care about Plasticity weak spots — cylinder spans, Coons/4-sided fills,
fillets/blends, and boolean-trimmed planes. Everything else can be lax/fast
(Plasticity-class quality is fine).

| Tier | Families / subclasses | Product behaviour |
|---|---|---|
| **HARD floor** | `plane` (boolean cuts/holes), `cylinder`, `cone`, `torus` / fillet-blend tags, 4-sided Coons (`mapped.four_sided` / 4-edge UV-grid) | Prefer hardOrient / certified templates. Under HardSurfaceFloor only, labeled soft last resorts may admit residual plane trim (`softPlaneFallback` / `plane.trim_soft`), cone/torus UV/wall orient, and mapped UV-trim when hardOrient fails. StrictAllFaces refuses those paths. |
| **SOFT residual** | sphere, n-gon freeform, offset/extrusion/revolution residuals | Soft-admit on orient/CDT soft path; assemble may relax proofs for those faces |

MP9 must mesh under HardSurfaceFloor (EXIT 0). Full certify of every freeform
face is **not** the product gate — use `--strict-all-faces` for research.
Plane soft residual activates only when `softResidualsAllowed` (HardSurfaceFloor);
Wave A plane matrix / StrictAllFaces stay fail-closed.

---

## Obsolete `*_deferred` picker vs recon UV-promote

ADR-0014 early refuse in `generateSecureMesh` still prefers condition codes
containing `_deferred` when explaining `secure_pipeline.unsupported_surface_family`
(`core/src/secure_meshing.cpp`, preferred-code loop over `record.conditionCodes`).

Recon (`core/src/secure_reconnaissance.cpp`) now **UV-promotes** many former
deferred subclasses to `SupportedAnalyticTemplate` via tags such as
`freeform.uv_trim_candidate`, `sphere.uv_trim_candidate`,
`mapped.four_sided_candidate`, plus `*.uv_trim_attempted`. Those faces never hit
the ADR-0014 residual picker because support is already analytic.

Do not treat leftover `*_deferred` inventory tags as the product refuse
authority when a UV-promote tag also applies:

| Obsolete / inventory tag | Current recon intent | Matrix wave |
|---|---|---|
| `mapped.non_four_sided_deferred` | Still tagged on n≠4 mapped families; extrusion/offset/**revolution** also get `freeform.uv_trim_candidate` and promote (Wave D). | D |
| `freeform.high_edge_count_deferred` (historical) | Superseded by `freeform.uv_trim_candidate` for n-gon UV-trim | D |
| `cone.complex_boundary_deferred` / apex-only demotes (historical) | Non-apex/non-frustum → UV-trim attempt tags; stay supported | C |
| `sphere.complex_cap_deferred` (historical) | Multi-edge CapWall → `sphere.uv_trim_candidate` | C |
| `cylinder.complex*` demotes (historical) | Complex bands stay supported; mesher UV-trim | B |
| `cutout.multi_bore_cylinder_deferred` / `filleted_slot_deferred` | Inventory/cut-graph tags; cylinder consumer must HARD or named REFUSE | B |
| ADR-0014 preferred prefixes (`freeform.high_edge`, `cone.non_apex`, `sphere.partial`, `cylinder.complex`, `extrusion.`, `offset.`) | Prefer those codes only when support is still Deferred; ignore as refuse reason once UV-promoted | matrix-doc note |

Wave agents: when locking a row, assert the **consumer outcome** (HARD/REFUSE
code), not the presence of a historical `*_deferred` tag.

---

## Cross-cutting product contract (Wave 0)

Applies to every UV-trim / mapped success path on the product route:

| Contract | Expected | Outcome |
|---|---|---|
| UV-trim exit | `relaxGeometryChecks=false` and `windingsMatchOrientedFaceNormal=true`, else named refuse | HARD (Wave 0) |
| No minority-tri drop in `hardOrientUvTrimMesh` | Refuse instead of silent drop | HARD (Wave 0) |
| `previewFast` / discrepancy widen / assembly tol bump | Not armed from `faceCount > 500` alone; only `--allow-partial-body` / explicit preview | HARD (Wave 0) |
| Shared CDT `relaxGeometryChecks` | Allowed only for curved UV inside CDT; product clears via hardOrient or refuse before assemble | HARD (Wave 0) |

ctest hooks: `WEFT_G0` / `WEFT_WAVE0` in `secure_meshing` (`testG0FailClosedDefaults`);
evidence `docs/evidence/brep-wave0-product-contract-2026-07-20.md`.

---

## Surface family matrix

### Wave A — Plane

Consumer: `planar_trim_assembly` / `planar_cdt` / plane branch in `secure_meshing`.
Constraint: no `allowCurvedUv` on planes.
Lock battery: `testPlaneMatrix` / `WEFT_PLANE_MATRIX` in `planar_trim_assembly`
(meshes all five extracts fail-closed; asserts `allowCurvedUv=false` on plane
assemblies).

| Subclass | Consumer | Extract | Outcome | Evidence / existing WEFT_* |
|---|---|---|---|---|
| Simple disk / quad | planar CDT | `plane_multi.step` | HARD | `WEFT_PLANE_MATRIX` |
| Digon / collapsed-loop recovery | planar UV-strip recovery | `plane_2732.step` | HARD | `WEFT_G1_PLANE2732`; `WEFT_PLANE_MATRIX` |
| Multiply perforated / filleted-slot residual | walk-landing bridges | `plane_3605.step` | HARD | `WEFT_G1_PLANE3605`; `WEFT_PLANE_MATRIX` |
| Ellipse-dense self-intersect recovery | ellipse densify ≥48 | `plane_3821.step` | HARD | `WEFT_G1_PLANE3821`; `WEFT_PLANE_MATRIX` |
| Self-intersect candidate (body residual) | densify / certified recovery | `plane_1793.step` | HARD | `WEFT_PLANE_MATRIX` |
| Multi-outer / annulus / multi-hole | planar CDT | `plane_multi.step` (+ corpus hole fixtures) | HARD | `WEFT_PLANE_MATRIX`; `testPerforatedPlanarFace` |
| Filleted-slot residual planes | plane path via 3605 bridges | `plane_3605.step` | HARD | `WEFT_PLANE_MATRIX`; G1 perforated recovery |

### Wave B — Cylinder

Consumer: `cylinder_template` + cylinder UV-trim in `secure_meshing`.
Constraint: UV-trim success clears `relaxGeometryChecks` via Wave-0 hardOrient
(asserted by non-vacuous `certified.triangle_intersection`).
Lock battery: `testCylinderMatrix` / `WEFT_CYLINDER_MATRIX` in
`cylinder_template` (meshes all eight extracts fail-closed).

| Subclass | Consumer | Extract | Outcome | Evidence / existing WEFT_* |
|---|---|---|---|---|
| Full periodic band | structured wall | `cylinder_band.step` | HARD | `WEFT_CYLINDER_MATRIX` |
| Complex / partial UV-trim | UV-trim + hardOrient | `cylinder_complex.step` | HARD | `WEFT_CYLINDER_MATRIX`; G2 no-relax |
| Ellipse rims | UV-trim / structured | `cylinder_ellipse.step`, `cylinder_ellipse_band.step` | HARD | `WEFT_CYLINDER_MATRIX` |
| Split open 4-semicircle rims | split-rim structured wall | `cyl_24.step`, `cyl_24_from_mp9.step` | HARD | `WEFT_G2_CYL24`; `WEFT_CYLINDER_MATRIX` |
| Multi-bore / filleted slot bore | cylinder template / UV-trim | `cyl_filleted_slot_bore.step` | HARD | `WEFT_CYLINDER_MATRIX` |
| Multi-rim ellipse-cut full-period UV-trim | UV-trim invert → period-folded iso-lattice hardOrient | `cyl_441.step` | HARD | `WEFT_CYLINDER_MATRIX`; MP9 face 441 |
| Analytic fixture body | template | generated `cylinder` | HARD | `WEFT_CYL_*` / cylinder_template |

### Wave C — Cone / sphere / torus

Lock batteries: `testConeMatrix` / `WEFT_CONE_MATRIX` (`cone_template`),
`testSphereMatrix` / `WEFT_SPHERE_MATRIX` (`sphere_template`),
`testTorusMatrix` / `WEFT_TORUS_MATRIX` (`torus_template`). Product path
`omitDeferredResiduals=false`; CapWall refuses orientation minorities
(refuse-not-drop) then hard UV-trim.

| Subclass | Consumer | Extract / fixture | Outcome | Evidence / existing WEFT_* |
|---|---|---|---|---|
| Cone apex | `cone_template` / UV-trim hardOrient | `cone_apex.step` | HARD | `WEFT_CONE_MATRIX` |
| Cone frustum band | revolved band | `cone_frustum.step` / `truncated_cone` fixture | HARD | `WEFT_CONE_MATRIX`; `WEFT_CONE_FRUSTUM` |
| Cone residual → UV-trim | hard UV-trim | `cone_apex.step` (apex extract uses UV-trim hardOrient) | HARD | `WEFT_CONE_MATRIX` apex_uvtrim_hardOrient=1 |
| Sphere full wall | `sphere_template` | generated `sphere` | HARD | `WEFT_SPHERE_MATRIX`; `WEFT_SPHERE_*` |
| Sphere CapWall / band | CapWall refuse → UV-trim hardOrient | `sphere_cap_778.step`, `sphere_cap_complex.step` | HARD | `WEFT_SPHERE_MATRIX`; `WEFT_SPHERE_CAP_HARD` |
| Torus structured wall | `torus_template` | generated `torus` | HARD | `WEFT_TORUS_MATRIX`; `WEFT_TORUS_*` |
| Torus densify + hard UV-trim | wall densify / UV-trim hardOrient | `face_116_torus.step` | HARD | `WEFT_TORUS_MATRIX` (matrix ctest, not fixture-only LOD) |
| Torus split-rail orient densify | wall orient densify → hardOrient / UV-trim fallback | `face_115_torus.step` | HARD | `WEFT_TORUS_MATRIX` |
| Torus UV-degen wall densify | densify on `torus.triangle_uv_degenerate` | `face_143_torus.step` | HARD | `WEFT_TORUS_MATRIX`; evidence `brep-wave-c-torus-143-2026-07-20.md` |

### Wave D — Freeform / mapped / extrusion / offset / revolution

Consumer: mapped four-sided → densify → UV-trim; n-gon / `freeform.uv_trim_candidate` → UV-trim (`secure_meshing` ~2155+). Revolution n≠4 promotes like extrusion/offset.

| Subclass | Consumer | Extract / fixture | Outcome | Evidence / existing WEFT_* |
|---|---|---|---|---|
| BSpline/Bezier ≤5 UV-grid | mapped / UV-grid | `freeform_pent.step`, `bspline_139.step` | HARD / named refuse | `WEFT_FREEFORM_MATRIX`, `WEFT_G3_*` |
| BSpline n-gon UV-trim | UV-trim hardOrient | `freeform_hex.step` | HARD | `WEFT_FREEFORM_MATRIX` |
| Mapped four-sided + densify | mapped → UV-trim | `freeform_mapped_fallback.step`, `face_103_orient.step` | HARD / named refuse | `WEFT_MAPPED_MATRIX` |
| Mapped / UV-trim orientation | hardOrient windingsMatch (general) | `face_33_orient.step`, `face_107_orient.step` | HARD | `WEFT_MAPPED_MATRIX` |
| Crash-locus curved UV CDT | UV-trim (no AV) | `crash_face_136_r0.step` (+ r1/r2 context) | HARD / REFUSE | `WEFT_FREEFORM_MATRIX` |
| Periodic band pent + general_attempted | UV-trim hardOrient (interval cap + coarsen retry) | `freeform_135.step` | HARD | `WEFT_FREEFORM_MATRIX`; evidence `brep-wave-d-freeform-135-2026-07-20.md` |
| Periodic band hex + period≠2π | UV-trim hardOrient (authoritative `curvedUvUPeriod` unwrap) | `freeform_137.step` | HARD | `WEFT_FREEFORM_MATRIX`; evidence `brep-wave-d-freeform-137-2026-07-20.md` |
| Periodic band pent + period≠2π wide ears | UV-trim hardOrient (corner vote when U-span > 0.35·period) | `freeform_138.step` | HARD | `WEFT_FREEFORM_MATRIX`; evidence `brep-wave-d-freeform-138-2026-07-20.md` |
| V-periodic band + tiny OCCT period | UV-trim hardOrient (V unwrap + dual-image seam open) | `freeform_186.step` | HARD | `WEFT_FREEFORM_MATRIX`; evidence `mp9-gate-blocker-freeform-186-2026-07-20.md` |
| Shared-seam U-periodic + circular caps | UV-trim iso-parametric band hardOrient (half-open U wrap) | `freeform_399.step` | HARD | `WEFT_FREEFORM_MATRIX`; evidence `mp9-gate-blocker-freeform-399-2026-07-20.md`; MP9 face 399 |
| Convex simple-region n-gon (non-periodic) | UV-trim invert/coarsen → iso-lattice hardOrient | `freeform_553.step` | HARD | `WEFT_FREEFORM_MATRIX`; evidence `mp9-gate-blocker-freeform-553-2026-07-20.md`; MP9 face 553 |
| Offset UV-trim | UV-trim / mapped densify | `offset_quad.step` | HARD | `WEFT_MAPPED_MATRIX` |
| Extrusion four-sided | mapped / UV-trim | fixture `extrusion_quad` | HARD | `WEFT_MAPPED_MATRIX` |
| Revolution four-sided | mapped | MAP path when 4-edge | HARD when 4-edge | MAP path |
| Revolution n≠4 | UV-trim promote | fixture `revolution_ngon` | HARD / named refuse | `WEFT_FREEFORM_MATRIX` |
| True unmeshable mapped | named refuse | annulus extracts when unorientable | REFUSE | `mapped.orientation_unresolved` / `mapped.uv_trim_orientation_unresolved` |

### Wave E — Assembly certificate

Consumer: `assembleCertifiedBoundaryMesh` / `certified_mesh`.

| Subclass | Consumer | Fixture | Outcome | Evidence / existing WEFT_* |
|---|---|---|---|---|
| Closed solid manifold | per-solid closed-manifold when shell closed | box / connected fixtures | HARD | `WEFT_G5`; `WEFT_ASSEMBLY_MATRIX` |
| Incidence / orientation / intersection non-vacuous | certificate | G5 path | HARD | `WEFT_ASSEMBLY_MATRIX`; G5 evidence |
| Adversaries | named refuse | existing certified_mesh adversaries | REFUSE | certified_mesh tests (orientation/position tamper) |
| Multi-solid inter-solid vs leak | Track M (light) | multi-component closed mesh + app per-solid open counts | HARD | `WEFT_ASSEMBLY_MATRIX` multi_solid; Track M UI |

### Wave F — Honest unsupported

| Subclass | Consumer | Fixture | Outcome | Evidence / existing WEFT_* |
|---|---|---|---|---|
| Curve hyperbola | refuse `secure_pipeline.unsupported_curve_family.hyperbola` | `tests/fixtures/unsupported/curve_hyperbola.step` | REFUSE | `WEFT_UNSUPPORTED_MATRIX` |
| Curve parabola | refuse `secure_pipeline.unsupported_curve_family.parabola` | `tests/fixtures/unsupported/curve_parabola.step` | REFUSE | `WEFT_UNSUPPORTED_MATRIX` |
| Curve offset | refuse `secure_pipeline.unsupported_curve_family.offset` | `tests/fixtures/unsupported/curve_offset.brep` (native; STEP flattens) | REFUSE | `WEFT_UNSUPPORTED_MATRIX` |
| Surface `kernel_specific` | refuse `secure_pipeline.unsupported_surface_family.kernel_specific` | `tests/fixtures/unsupported/surface_kernel_specific.construction` (in-test GeomPlate; BREP cannot persist OtherSurface) | REFUSE | `WEFT_UNSUPPORTED_MATRIX` |
| Deferred-only subclass | named `unsupported_surface_family.<tag\|family>` + extract | per row | REFUSE | never silent omit |

---

## WEFT_* matrix ctest scaffolding

Planned consolidate markers (Wave agents implement assertions; scaffold binary
only checks extract presence today):

| Wave | Marker | Host binary (today) | Existing entry points to extend |
|---|---|---|---|
| 0 | `WEFT_G0` (+ Wave-0 guards) | `secure_meshing` | `testG0FailClosedDefaults` |
| A | `WEFT_PLANE_MATRIX` | `planar_trim_assembly` | `testPlaneMatrix` (+ `testG1Plane2732*` / `3605*` / `3821*`) |
| B | `WEFT_CYLINDER_MATRIX` | `cylinder_template` | `testCylinderMatrix` (+ `testCyl24SplitRimExtract`) |
| C | `WEFT_CONE_MATRIX` / `WEFT_SPHERE_MATRIX` / `WEFT_TORUS_MATRIX` | `cone_template`, `sphere_template`, `torus_template` | `testConeMatrix`, `testSphereMatrix`, `testTorusMatrix` (+ CapWall / face_116) |
| D | `WEFT_MAPPED_MATRIX` / `WEFT_FREEFORM_MATRIX` | `mapped_template` | `testMappedFreeformMatrix` + `testG3*`; revolution n≠4 UV-promote |
| E | `WEFT_ASSEMBLY_MATRIX` | `secure_meshing` / `certified_mesh` | `testG5HardCertifiedAssembly` + `testAssemblyMatrixMultiComponent`; adversaries in `certified_mesh` |
| F | `WEFT_UNSUPPORTED_MATRIX` | `secure_meshing` | `testUnsupportedFamilyMatrix`; hyperbola/parabola STEP + offset BREP + GeomPlate kernel_specific construction |
| Gate | fail-closed MP9 CLI | `weft mesh` (not ctest matrix) | WP-177 / integration only |

Scaffold harness: `tests/test_brep_consumer_matrix.cpp` → ctest name
`brep_consumer_matrix` (fixture presence + prints markers; no consumer mesh).

Narrow run examples (Windows):

```powershell
ctest --preset vs2022 -R "brep_consumer_matrix|planar_trim_assembly|cylinder_template|mapped_template|sphere_template|torus_template|cone_template|secure_meshing" --output-on-failure
```

---

## Integration gate

After Waves 0 + A–D matrix markers pass on extracts: fail-closed
`weft mesh tests/STEP_Examples/MP9.stp` → EXIT 0, non-empty OBJ, modelling
quads > 0, certificate complete, `omitDeferredResiduals=false`. Any miss adds a
matrix subclass + extract—not a face-id special case.
