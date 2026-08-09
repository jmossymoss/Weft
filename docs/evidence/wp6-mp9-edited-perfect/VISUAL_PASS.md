# Visual pass (2026-08-08) — eyes on screenshots

Gallery: `/opt/cursor/artifacts/mp9_visual_pass/` and
`docs/evidence/wp6-mp9-edited-perfect/objects/visual_pass/`.

Numeric gate (secondary): watertight, folds=0, failed-floor=0,
structured 3052/3052. That gate is NOT sufficient — see grades below.

Rubric: solid+wire deselected; cylinder columns readable; no mid-span
support rings; no melted blobs; clean notch/fillet transitions; no
shredded/clipped/overlapping wire clumps.

## Priority objects (hand-inspected)

| Obj | Grade | Visual findings |
|----:|:-----:|-----------------|
| 1 | WEAK | Dense fans around holes; long thin polys at cylinder/base junctions |
| 2 | WEAK→improving | f134 was a 169-gon MinimalNGon; now quad-dominant (241 quads, maxN=4). Screenshots `after_02a/`. Notch/tooth slivers may remain. |
| 5 | WEAK | Outer wall columns OK-ish; recess/flute junctions messy; endcap radial n-gons; groove ends pole-like |
| 11 | BAD→PARTIAL | CAD default now plants 1 collar ring on plate-webs (f1612: 3→57 polys). Tan fillet straps around holes still dense/spoke-like. `after_11a/`. |
| 14 | BAD | Chaotic dense nest at cylinder→base groove; primer face starburst |
| 19 | WEAK→OK | Spring reads as structured Coons (no melted blob); some density stretch on helix |
| 37 | WEAK | Fillet straps pinchy at elbow; otherwise simple |
| 41 | BAD | Dense parallel edge clump / shredded hatching on one face vs empty n-gon flats |
| 42 | BAD | Same class as 41 — catastrophic dense clump in recess |

## Sampled objects (vision review)

| Obj | Grade | Notes |
|----:|:-----:|-------|
| 3 | BAD | Zipper seam; huge endcap n-gons |
| 4 | WEAK | Corner fan slivers |
| 6 | OK | Clean |
| 7 | OK | Clean |
| 8 | BAD | Star/fan at conical indent |
| 9 | BAD | Long sliver tris; density jump |
| 10 | WEAK | Flat n-gon spokes |
| 12 | BAD | Clipping through walls |
| 15 | OK | Clean |
| 20 | BAD | Slot-corner pinch/overlap |
| 25 | OK | Clean |
| 30 | WEAK | Over-stripped wall |
| 35 | BAD | Diagonal slivers from holes |
| 40 | BAD | Ultra-dense orange strips vs n-gon walls |
| 45 | BAD | Broken hex-bore interior |
| 50 | BAD | Messy hole fans; density mismatch |
| 55 | BAD | Banding + T-junctions |
| 60 | WEAK | Hex-hole spokes |
| 65 | BAD | Self-intersecting inner ring look |

## Verdict vs prior “fixed” claims

- V1 (obj 5 cylinders): PARTIAL — columns denser, but junctions/endcaps still WEAK
- V2 (obj 19 spring): MOSTLY OK visually (structured, not blob)
- V3 (obj 41/42 tris): FAILED visual — n-gon swap removed tris but left shredded density clumps
- V4 (obj 11 drums): FAILED visual — hole transitions still fan soup
- V5 (obj 2 endcap): Still BAD as expected

## Next fix order (visual-driven)

1. Objects 41/42 shredded density clump (V3 visual failure) — IN PROGRESS:
   reverted torus/sphere→MinimalNGon (that made hatching worse); restored
   watertight empty=0. Screenshots `after_41c/`. Still need structured
   tiny-strap meshing that neither fans tris nor melts to dense n-gons.
2. Object 2 endcap giant n-gon (V5)
3. Object 11 hole-transition fans
4. Object 5 recess/groove junction cleanup
5. Broader BAD cohort (3,8,9,12,14,20,…)

## Session note
Attempted post-stitch restore of fusion-collapsed MinimalNGon panels
opened 100+ seams; abandoned. Prefer preventing collapse / reconstructing
from neighbour stations without private verts.
