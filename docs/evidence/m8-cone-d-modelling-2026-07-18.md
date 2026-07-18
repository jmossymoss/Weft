# M8 CONE-D modelling topology evidence - 2026-07-18

## Proven increment

Apex-cone secure results carry truthful modelling provenance:

- Default cone fixture yields `ModelingProvenanceKind::Independent` with paired
  modelling polygons above the certified triangle floor
  (`WEFT_CONE_D modeling=Independent polys=69`).
- Tampered independent claims without a non-vacuous certificate refuse
  `modeling.provenance_independent_uncertified`.
- Floor-alias remains an allowed truthful fallback when pairing cannot prove
  quads (existing provenance validator).

## Commands

```bash
ctest --preset linux-gcc -R '^secure_meshing$' --output-on-failure
ctest --preset linux-gcc-static-analysis -R '^secure_meshing$' --output-on-failure
```

## Outcomes

Both Linux lanes passed.

## Status boundary

WP-073 (CONE-D) is closed. CONE-E covers app/CLI product admission smoke.
