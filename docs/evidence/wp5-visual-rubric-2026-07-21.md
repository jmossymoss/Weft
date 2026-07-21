# WP5 — visual rubric (honest, 2026-07-21)

Numeric watertightness alone is not a pass. Section 7 review of release,
MAMBO, and ABC smoke screenshots.

## Verdict

WP5 visual exit is **not met**. Many shapes remain incorrectly meshed for
artist editability even when `watertight: yes`.

## What was wrong with the earlier “PASS”

Public gates and release gate scored closed solids as green. Screenshots show:

- hair-thin / fan triangles on flats and freeform panels;
- density cliffs and seam artifacts at junctions;
- heavy triangulation where structured quads are expected;
- bored plates previously collapsing to one MinimalNGon fan under CAD.

## Class fix landed this revision

CAD/`minimal` routing no longer swallows **planar** multi-wire faces before
ring-junction / annulus / plate-web. Failed plate-web under minimal rescues to
MinimalNGon instead of raw OCCT.

| Before (CAD) | After (CAD) |
| --- | --- |
| hole fixture bored flats → MinimalNGon | → RingJunction (may contract-floor) |
| iso14649 many MinimalNGon flats | → AnnulusRing / PlateWeb on round holes |
| slitdrill plate-web fail → raw=2 | → MinimalNGon rescue, raw=0 |

Tests + `tools/release_gate.sh` still PASS.

## Visual review (after fix)

Screenshots: `docs/evidence/wp5-visual/` (release), `…/mambo/`, `…/abc/`.

| Model | Watertight | Slivers | Section 7 notes |
| --- | --- | --- | --- |
| iso14649-demo | yes | 54 | Better hole collars; top still irregular / density cliffs |
| flaregun | yes | 40 | Multi-solid; seams/shading still noisy |
| foam | yes | 208 | Junction + freeform top still chaotic |
| teleporter | yes | 230 | Freeform panels: long slivers, bad flow |
| MAMBO S5 | yes | 74 | Some plate-web/annulus; large flat still sliver-tris |
| MAMBO S34 | yes | 13 | Coons/freeform tris on vaulted top |
| MAMBO B40 | yes | 8 | Mostly MinimalNGon fans + tris |
| ABC 00008536 | yes | 359 | Outer quads OK; notched inner ring sliver soup |
| ABC 00002324 | yes | 224 | Sparse n-gon flats; high sliver count |

## Remaining failure classes (block WP5 exit)

1. **Freeform / Coons sliver panels** — teleporter, foam, ABC notched rings.
2. **Complex non-round planar cutouts** — still MinimalNGon fans when plate-web
   rejects elongated holes.
3. **Density transition artifacts** — adaptive CAD cliffs between features.
4. **Fresh Plasticity Blender compare** — still needs artist Plasticity meshes.

## Next work

Do not treat public-gate green as visual green. Next highest leverage:

- freeform/Coons sliver reduction (or earlier structured routing for near-analytic
  trimmed panels);
- MinimalNGon ear-clip quality when it remains the residual flat;
- keep reducing shareable ABC/MAMBO defects into the deterministic zoo.
