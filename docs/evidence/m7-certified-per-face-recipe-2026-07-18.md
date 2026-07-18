# M7 certified per-face recipe evidence - 2026-07-18

## Proven increment

One certified per-face recipe consumer is now admitted: an axial-only override
on a single face maps to `SecureMeshingConfiguration::cylinderAxialIntervals`.

- `validateSecureRecipeApplication` allows exactly that narrow case and still
  refuses broader per-face/manual settings (BR-010).
- `certifiedPerFaceCylinderAxial` exposes the resolved axial count.
- App `secureConfiguration` and CLI mesh configuration apply the override.
- Save/reload + double `generateSecureMesh` replay keep identical topology
  fingerprints.

## Environment / commands

```bash
ctest --preset linux-gcc -R 'secure_recipe|secure_meshing' --output-on-failure
ctest --preset linux-gcc-static-analysis -R 'secure_recipe|secure_meshing' \
  --output-on-failure
```

Linux GCC 13.3 / OCCT 7.6.3.

## Outcomes

- Both lanes passed.
- Cylinder fixture with per-face axial=2 resolves, persists, and remeshes
  deterministically.

## Status boundary

WP-041 is closed on Linux for the axial-only cylinder consumer. Other per-face
fields remain named refusals.
