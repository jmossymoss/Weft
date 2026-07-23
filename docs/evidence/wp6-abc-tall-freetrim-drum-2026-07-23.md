# WP6: ABC tall FreeTrim drums off MinimalNGon needles (2026-07-23)

## Class

ABC nightly `00002324` (and the shared topology class): tall skinny
`Drum × FreeTrim` cylinder walls (U-span ≪ 1 rad, height ≫ U-chord) that
CAD/minimal collapsed to a single stretched boundary n-gon.

Reducer: `tests/regressions/abc/tall_free_trim_drum_r0.step`
(extract of source face 35, rings=0).
Test: `testTallFreeTrimDrum`.

## Root causes

1. Residual CAD MinimalNGon grab treated false-flat drums as panels.
   `isGeometricallyFlat` compared dish to the long diagonal, so sagitta ≪
   height passed as flat. Drums are now excluded from that grab, and analytic
   drums measure dish against the short in-plane span.
2. Early open-band routing required `uspan >= 1.0`, which is exactly the
   IsoBand threshold — FreeTrim never reached open-band before Coons claimed
   the 4-sided outline as a 1×1 patch.
3. Open-band with `nu = max(2, rim1)` and `passPlain=false` failed the plain
   strip when the low rail had only the single interior column. Side tops are
   now included on that rail.
4. CAD `axial=1` + adaptive-on-straights left one cell of full height. Narrow
   wrap (`bandWrapFrac < 0.25`) tall bands now take an aspect axial floor
   (clamped to 24).

## Results

| Asset | Before | After |
| --- | --- | --- |
| r0 reducer | minimal-ngon, 4 verts, aspect ~400 | revolution-grid, 48 quads, 0 slivers, build=0 |
| ABC `00002324` | 16 FreeTrim drums → minimal-ngon; 224 slivers | 16 → revolution-grid build=0; WT yes; 264 slivers |
| ABC nightly gate | PASS 6/6 | PASS 6/6 |

Golden count lifts (foam / flaregun): residual-flat no longer swallows
false-flat drums, so those faces take Coons/open-band instead of one n-gon.
Invariants (WT, raw=0) unchanged. IsoBand / wide open bands keep prior axial.

## Visual

`docs/evidence/wp6-abc-visual/` — CLI `--profile cad` + app screenshots.
`00002324` source shape is tall towers; drum walls now carry axial rows
rather than single-span n-gon needles. `00008536` notched drum remains
RevolutionGrid with local tooth folds. `00006051` still shows 2 contract
floors (next class).
