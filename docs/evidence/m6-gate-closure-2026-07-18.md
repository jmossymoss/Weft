# M6 planar and cylinder templates gate closure - 2026-07-18

## Gate

Box, full/partial cylinder, mismatched-frame cylinder, and connected
plane/cylinder fixtures pass end to end.

## Fixture matrix

| Fixture / case | Proof |
|---|---|
| `box` | Certified + Independent modelling (secure_meshing) |
| full `cylinder` | Certified + chord/normal coverage; deterministic fingerprint |
| `partial_cylinder` | Certified CLI OBJ (42v/80t); unequal-arc refuses |
| mismatched-frame | Azimuth registration unit + template adversaries |
| connected `hole` | Certified plane+cylinder body |
| Independent modelling | Box/fillet/cone paths carry Independent or truthful alias |

## Verification

```bash
ctest --preset linux-gcc -R 'secure_meshing|cylinder_template|canonical_boundary|certified_mesh' --output-on-failure
build/linux-gcc/cli/weft mesh box|cylinder|partial_cylinder.step -o out.obj
```

## Outcomes

- Listed tests passed on Linux gcc.
- CLI exports real OBJs for box/cylinder/partial_cylinder.
- Prior M7 app/Blender evidence covers box/full-cylinder product routes.

## Status

WP-117 closed. M6 marked `PASSED`.
