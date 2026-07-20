# WP3 — foam plane_fillet_bspline_junction stitch seal (2026-07-20)

## Class

`plane_fillet_bspline_junction` (foam).

## Root cause (two layers)

1. Pad/plane `MinimalNGon` verts inserted into successful `CoonsGrid`
   fillet/bspline borders (`stitchSeams`), folding the strip and opening
   the junction (default + CAD).
2. CAD residual on coons↔revolution fillet seams: short rim chords failed
   the 35% midpoint sagitta gate while the neighbor still spliced,
   leaving a one-sided T-junction.

## Fixes (class-level)

In `stitchSeams` (`core/src/meshers.cpp`):

1. Skip insertion into successful `CoonsGrid` when the other B-rep edge
   owner is successful `MinimalNGon` (keeps coons↔revolution stitches).
2. On open curves, if the geometric midpoint fails the 35% test but the
   union chain already has an intervening on-curve sample between the
   segment endpoints' params, accept the param span and splice.

No filename, model-name, or face-ID special cases. Builds on
`docs/evidence/wp3-planar-ngon-stitch-protect-2026-07-20.md`.

## Metrics

| model | profile | before | after |
| --- | --- | --- | --- |
| foam | default | open 6, NM 2 | watertight |
| foam | cad | open 12 → 6 after (1); then 6 → 0 after (2) | watertight |
| teleporter | cad | watertight | watertight |
| teleporter | default | open 14, NM 2 | unchanged (secondary class) |
| flaregun | cad | watertight | watertight |
