# M4/M6 certified full-cylinder evidence - 2026-07-17

## Proven increment

A full periodic cylinder wall now consumes canonical rims and seam endpoint
uses, registers equal-count rings, carries per-triangle periodic UV lifts, and
checks non-vacuous chord and normal error. Together with its two exact planar
CDT caps it forms one closed no-weld `CertifiedMesh`.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release
cmake --build --preset vs2022-static-analysis --config Release
ctest --preset vs2022 -E "^pipeline$" --output-on-failure
ctest --preset vs2022-static-analysis -E "^pipeline$" --output-on-failure
```

Both full builds completed and all 12 secure/corpus tests passed in each lane.
The cylinder fixture proves:

- 64 samples on each canonical rim;
- 128 globally shared vertices;
- 128 wall triangles plus two 62-triangle CDT caps;
- complete boundary, registration, exact UV orientation, chord, and normal
  evidence;
- closed incidence and opposite winding without a weld;
- identical topology fingerprint after reversing face-product input order;
- explicit safe-floor modelling alias while modelling quads remain unproven.

Named refusal fixtures cover a reflected/twisted rim, mismatched rim counts,
extra axial seam samples, chord and normal limits, missing p-curve provenance,
a wrong periodic corner lift, and a non-cylindrical face.

## Status boundary

M4, M5, and M6 remain in progress. This is one full-cylinder family proof, not
partial-cylinder support, interior axial refinement, a general curved CDT,
adaptive surface-patch bounds, 3D self-intersection, modelling-quad acceptance,
workflow routing, Linux determinism, or a complete corpus gate.
