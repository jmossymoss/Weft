# M7 admission path audit evidence - 2026-07-18

## Proven increment

Product mesh consumers now pass through `admitCertifiedMeshingResult` before
`makeCertifiedPolyMeshAdapter` use:

| Route | Location | Gate |
|---|---|---|
| CLI `mesh` / export | `cli/main.cpp` regenerate lambda | admit + throw on refusal |
| App async regenerate | `app/main.cpp` gen worker | admit + throw on refusal |
| App export path | `app/main.cpp` secure export helper | admit + throw on refusal |
| Adapter fail-closed | `makeCertifiedPolyMeshAdapter` | empty mesh + refusal code |

Admission requires complete validation certificate, non-empty certified mesh /
fingerprint, matching generation epoch when provided, and valid modelling
provenance. Tests cover happy path, incomplete certificate, stale epoch, and
unexplained alias.

## Enumerated product mesh call sites

- `cli/main.cpp`: secure regenerate → adapter → writers
- `app/main.cpp`: async regenerate → viewport mesh
- `app/main.cpp`: export helper → OBJ/GLB/FBX writers
- Sweep/convert CLI paths already consume `generateSecureMesh` results under
  the same secure-only routing (BR-001)

No legacy generator selector or OCCT triangulation entry point remains on these
routes.

## Environment / commands

```bash
ctest --preset linux-gcc -R 'certified_mesh|secure_meshing' --output-on-failure
ctest --preset linux-gcc-static-analysis -R 'certified_mesh|secure_meshing' \
  --output-on-failure
build/linux-gcc/cli/weft fixture /tmp/box.step --shape box
build/linux-gcc/cli/weft mesh /tmp/box.step -o /tmp/box.obj
```

Linux GCC 13.3 / OCCT 7.6.3. Windows MSVC strict lane not available in this
environment; deferred to CI.

## Outcomes

- Both Linux lanes passed.
- CLI mesh of the box fixture produced a certified OBJ (8 verts / 12 tris at
  the default secure counts used by that command path).

## Status boundary

WP-040 is closed on Linux for the enumerated product routes. M7 remains
`IN_PROGRESS` (per-face recipe, cache invalidation, editing/overlays remain).
