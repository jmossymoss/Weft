# M8 CONE-C certified floor evidence - 2026-07-18

## Proven increment

Apex-cone solids now certify through `generateSecureMesh`:

- `cone` is a `SupportedAnalyticTemplate` family when exact non-degenerate
  coedge mappings exist (degenerate apex edges are skipped for readiness).
- New `buildApexConeWall` fans triangles from one singular apex station to a
  closed circular base rim, consumes every face/coedge use, and certifies
  rim-chord and normal bounds.
- Planar base faces still use the existing planar CDT path.
- Body assembly passes intersection/incidence/Euler with no weld.
- CLI `weft mesh` exports a certified OBJ for `makeFixture("cone")`.
- Sphere remains a named refusal before generation.

### Fixture metrics (default secure test sampling)

```text
WEFT_CONE_C tris=38 verts=21 fingerprint=4206cd65287f33d3
WEFT_CONE_C adversary=interval.count_exceeds_maximum
WEFT_CONE_A mesh_ok tris=38 verts=21
```

CLI smoke (fresh `weft` binary): 27 vertices / 50 tris OBJ for the same fixture
under CLI density defaults.

## Environment

- Host: Cursor Cloud Linux / g++ 13.3 / OCCT 7.6.3
- Presets: `linux-gcc`, `linux-gcc-static-analysis`

## Commands

```bash
cmake --preset linux-gcc
cmake --build --preset linux-gcc --target \
  weft_cone_template_tests weft_secure_meshing_tests weft_secure_core_tests \
  weft_canonical_boundary_tests weft -j"$(nproc)"
ctest --preset linux-gcc \
  -R 'cone_template|secure_meshing|secure_core|canonical_boundary' \
  --output-on-failure
cmake --preset linux-gcc-static-analysis
cmake --build --preset linux-gcc-static-analysis --target \
  weft_cone_template_tests weft_secure_meshing_tests weft_secure_core_tests \
  weft_canonical_boundary_tests -j"$(nproc)"
ctest --preset linux-gcc-static-analysis \
  -R 'cone_template|secure_meshing|secure_core|canonical_boundary' \
  --output-on-failure
build/linux-gcc/cli/weft fixture /tmp/cone_c.step --shape cone
build/linux-gcc/cli/weft mesh /tmp/cone_c.step -o /tmp/cone_c.obj
```

## Outcomes

- Both Linux lanes: the four tests passed.
- CLI mesh wrote `/tmp/cone_c.obj` with non-zero certified triangles.

## Status boundary

WP-072 (CONE-C) is closed. CONE-D covers modelling provenance/quads; CONE-E
product screenshots; CONE-F density/corpus/determinism proof.
