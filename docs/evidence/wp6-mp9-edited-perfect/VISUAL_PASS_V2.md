# Visual pass v2 (2026-08-09) — fresh gallery, eyes-first

Gallery: `/opt/cursor/artifacts/mp9_visual_pass_v2/` and
`docs/evidence/wp6-mp9-edited-perfect/objects/visual_pass_v2/`.

Numeric gate (secondary): watertight, folds=0, empty=0, structured 3052/3052.
That gate is NOT sufficient — grades below are from screenshots.

Rubric: solid+wire deselected; cylinder columns readable; no mid-span
support rings; no melted blobs; clean notch/fillet transitions; no
shredded/clipped/overlapping wire clumps; density consistent across
adjacent bands.

## Priority objects (hand-inspected)

| Obj | Grade | Visual findings |
|----:|:-----:|-----------------|
| 1 | WEAK | Hole fans / dense poles at small recesses; otherwise readable |
| 2 | WEAK | Endcap no longer one 169-gon (QD rescue) but still irregular seam + radial fan feel |
| 3 | WEAK | Clean body columns; cyan end-ring density jump vs orange body |
| 4 | WEAK | Corner fan / dense cluster at lower step |
| 5 | WEAK→OK | Columns look structured; recess junctions still busy |
| 6 | OK | Clean short drum; large flat n-gon OK for CAD |
| 7 | OK | Clean |
| 8 | WEAK | Rail body mostly OK; conical indent / orange patches dense |
| 9 | BAD | Grip bands over-dense (horizontal soup); rear orange chaos |
| 10 | WEAK | Bore collar structured; large flat face sparse n-gon |
| 11 | BAD | Tan hole transitions still spoke/fan dense despite collar rings |
| 12 | WEAK | Internal cavity dense; outer I-beam readable |
| 14 | BAD | Shredded dense nest at cylinder→base groove; primer starburst |
| 19 | OK | Spring structured Coons, not melted blob |
| 20 | OK | Slotted pin clean columns + slot |
| 37 | WEAK | Fillet straps pinchy but simple L-bracket readable |
| 40 | IMPROVED | Strap nu capped (31→13); upright tops readable. `after_40b/`. |
| 41 | WEAK→improving | Planar strap panels restored (micro-edge protect). Screenshots `after_41d/`. Still need cleaner torus/sphere density. |
| 42 | WEAK→improving | Same class as 41; panels restored. `after_41d/`. |
| 45 | WEAK | Hex bore interior coarse; outer columns OK |
| 50 | WEAK | Orange hinge fillets dense vs grey bars |
| 55 | IMPROVED | Hook bands look regular after density clamp. `after_40b/`. |
| 65 | BAD | Teal/purple inner straps look shredded / intersecting |

## Sampled mid/tail objects

| Obj | Grade | Notes |
|----:|:-----:|-------|
| 15–18, 21–24, 25–29 | OK/WEAK | Mostly clean small hardware; spot density jumps |
| 30–36, 38–39 | WEAK | Occasional fillet density clumps |
| 43–44, 46–49 | WEAK | Plate/ring parts; some n-gon flats |
| 51–54, 56–64 | WEAK/OK | Mix; no worse than 55/65 class unless noted |

## Fix order (visual-driven)

1. Objects 41/42 shredded green straps (class: tiny torus/sphere + planar panels)
2. Object 14 cylinder→base shredded nest
3. Objects 40/55 over-dense fillet/strap bands (shared density class)
4. Object 65 shredded inner ring straps
5. Object 11 hole-transition residual n-gon / tan fans
6. Object 9 grip over-density
7. Object 2 endcap seam cleanup
8. Broader WEAK cohort

## Session notes

- Validity remains green while many objects still fail visual rubric.
- Do not claim "fixed" without retake of the same object after the change.

## Validity note (post-gallery)

CAD default assembly was opening 261 edges on object 22: multi-edge
iso-band fillet #1948 took orth-coons (3×42 stations) against neighbour
drum columns. Routed iso-band fillets with >24 edges to MinimalNGon
under CAD/minimal → opens 261→16. Remaining leaks: obj 9 (f742/f1130
large freeform) and a few others. Continue next.

## Validity (latest)

`testMp9EditedWatertight` green: watertight, structured 3052/3052, folds=0,
empty=0. Gallery retake: `objects/visual_pass_v3/`.
