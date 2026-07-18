# M8 CONE-A reconnaissance evidence - 2026-07-18

## Proven increment

Cone subjects are classified exactly once by `reconnoitre` without promoting
the lateral face to a certified automatic template. Sphere/torus remain
deferred residuals. Secure meshing still refuses the apex-cone fixture by a
stable named code before any weld or legacy fallback.

### Fixture topology (`makeFixture("cone")`)

- 2 faces: conical lateral face + planar base
- 3 edges: base circle, linear seam generator, degenerate apex circle
- Cone trim class: `touches_one_singularity` with `u_periodic`
- Cone support: `deferred_residual_surface` /
  `reason.deferred_residual_surface` (`ProvenAnalytic`)
- Base plane: `supported_analytic_template` / `strategy.surface.plane`
- Degenerate apex circle: `invalid_imported_geometry` (condition `degenerate`)
- Base circle + seam line: supported analytic curve templates

### Mesh refusal (deferred until CONE-C)

```text
WEFT_CONE_A mesh_refusal=secure_pipeline.circle_domain_invalid
WEFT_CONE_A sphere_refusal=secure_pipeline.unsupported_curve_family
```

The apex-cone path currently fails while assigning circle interval demands for
the degenerate apex edge. That named refusal is accepted for CONE-A; CONE-B/C
must teach critical segmentation / interval assignment to skip or specially
account for degenerate apex circles before the cone wall template can run.

Probe-level family checks also lock cone/sphere/torus as analytic deferred
residuals (`probeSurfaceFamily`).

## Environment

- Host: Cursor Cloud Linux / g++ 13.3 / OCCT 7.6.3
- Presets: `linux-gcc`, `linux-gcc-static-analysis`
- Revision: see commit closing WP-070

## Commands

```bash
cmake --build --preset linux-gcc --target weft_secure_core_tests -j"$(nproc)"
ctest --preset linux-gcc -R '^secure_core$' --output-on-failure
cmake --build --preset linux-gcc-static-analysis --target weft_secure_core_tests -j"$(nproc)"
ctest --preset linux-gcc-static-analysis -R '^secure_core$' --output-on-failure
build/linux-gcc/cli/weft fixture /tmp/cone.step --shape cone
build/linux-gcc/cli/weft inspect /tmp/cone.step
```

## Outcomes

- Both Linux lanes: `secure_core` passed.
- `testConeReconnaissanceDeferred` accounts every cone face/edge once and
  prints the `WEFT_CONE_A` classification / refusal lines above.
- OCCT 7.6 note: the inverted-shell product STEP orientation witness
  normalizes on write/import; repair assertions remain enforced on OCCT 7.8+.

## Status boundary

WP-070 (CONE-A) is closed. Cone is not mesh-supported. CONE-B must extend
critical segmentation and interval handling for the apex singularity before
CONE-C can promote `SupportedAnalyticTemplate`.
