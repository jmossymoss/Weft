# G2: cyl_24 split-rim certify — 2026-07-20

## Goal

After G3 single-face `identity_mesh_allowed`, `cyl_24.step` cleared import but
refused at `boundary.curve_evaluation_failed` `subjects=[9:3]`. Prefer certified
mesh over advisory-only residual.

## Root cause

MP9 face 24 is a full-period cylinder with:

- one axial `SEAM_CURVE` (dual p-curves at U=0 and U=2π);
- four open semicircle rim arcs (two axial levels × two arcs), including a
  second-period rim p-curve with U ∈ [2π, 3π].

Periodic-target inverse solves on those p-curves landed a few ULPs outside the
exact 3D trim. `appendCriticalEvent` admitted them within epsilon, but
`evaluateCurve` uses a strict `[first,last]` domain check →
`boundary.curve_evaluation_failed`.

Separately, structured-wall routing required two *closed* rims, so even after
boundaries built the face fell through to coarse UV-trim.

## Fix

1. `canonical_boundary.cpp`: clamp near-domain critical parameters into the
   exact trim before sampling; snap endpoint ULPs at evaluate time; surface the
   underlying geometry failure code in the refusal message.
2. `secure_meshing.cpp` (cylinder branch): treat `fullPeriodic && openCircles==4`
   as structured two-rim topology.
3. `cylinder_template.cpp`: accept open circular/elliptical arcs on full-period
   walls; merge arcs that share an axial V level into two rims ordered by U.

Do not revert G3 `secure_core` identity-mesh allowlisting.

## Proof

Host: Windows MSVC, `build/bin/Release/weft.exe`, OCCT on PATH.

```
weft mesh tests/fixtures/mp9_extracts/cyl_24.step -o build/_c24_struct.obj
→ EXIT 0; 128 vertices, 64 quads, 0 tris (no folded-polygon warning)

weft_cylinder_template_tests
→ WEFT_G2_CYL24 tris=128; certified cylinder template checks passed
```

Sibling extracts still EXIT 0: `cylinder_band`, `cylinder_complex`,
`cylinder_ellipse`, `cylinder_ellipse_band`, `cyl_filleted_slot_bore`.

## Remaining

- Filleted-slot *plane* residuals (`cutout.filleted_slot_residual` ×22) stay G1
  plane CDT ownership; bore cylinders (e.g. MP9 f279 extract) already certify.
- Ellipse single-rim extracts still UV-trim (only one circular/elliptical rim).
