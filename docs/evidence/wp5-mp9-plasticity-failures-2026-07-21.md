# WP5 — MP9 Plasticity visual failures (2026-07-21)

Artist-supplied screenshots against `tests/STEP_Examples/MP9.stp` (Plasticity
export; no separate STL in-repo — the STEP is the source). MP9 remains
`tier=performance` in `CAD_CORPUS.tsv`; these notes classify failures for
reduction into the zoo, not as a silent release-oracle expansion.

## Batch CAD profile snapshot (pre-sphere fix)

| Metric | Value |
| --- | --- |
| Faces / edges | 3748 / 10155 |
| Polygons | ~40k (CAD adaptive) |
| Watertight | NO — 1917 open edges, 15 non-manifold |
| Slivers | 2143 |
| Demotions | 141 → contract floor, 5 raw, 1 empty |
| Leakiest faces | #1827/#1805/#1904/#1806 (~200 opens each, bspline coons) |

## Screenshot classes → code attribution

| Artist note | Example IDs (session) | Class | Status |
| --- | --- | --- | --- |
| Open holes on foregrip | #2334, #2374 bspline | Junction / open-shell + large coons leaks (#1805 family dominates opens) | Open |
| Fillets over corners; mismatched counts | torus fillets, coons vs contract-floor | Fillet coons corner / demotion | Open |
| Foregrip fillet fanning | coons-grid torus | Coons rail fanning at tips | Open |
| Outer ≠ inner boolean counts; “5 vs floor 12” | sphere + plane annulus | Adaptive density + wrong sphere floor | Partial — sphere routing fixed |
| B-spline / freeform on contract-floor | #1446-class | Wrong Fallback routing / demotion | Open |
| Grip / optic / fluted body: Plasticity vs Weft vs Expected | freeform panels | Quad flow vs sliver fans (expected = structured quads) | Open |

Face IDs in the UI can drift across import settings; reducers below pin geometry.

## Fix landed this revision (sphere dimple class)

Trimmed `GeomAbs_Sphere` patches often report `IsUClosed()=false`, so they
skipped `RevolutionGrid` and became planned contract-floor needle soup. After
a fold-check demote the floor was worse than a boundary n-gon.

Changes in `core/src/meshers.cpp`:

1. `isClosedRevolution`: spheres always count as U-closed (surface type).
2. Auto plan: if rim-hug fails on a sphere, still `finishRevolution()`.
3. Fold-check demote: under CAD/`minimal`, rescue folding sphere revolution
   grids to `meshMinimalPlanar` instead of the contract floor.

Reducer: `tests/regressions/mp9/sphere_dimple_annulus.step` (MP9 #1221+#1224).

| | Before | After |
| --- | --- | --- |
| Sphere face mesher | contract-floor (472 tris) | minimal-ngon (1 n-gon) |
| Slivers on reducer | 3 | 0 |
| Watertight (annulus+sphere+wall) | yes | yes |

Test: `testSphereDimpleNotContractFloor`.

## Still open on MP9 (block visual / Plasticity compare exit)

1. **Watertightness** — ~1.9k opens; leakiest are large bspline coons panels.
2. **Fillet coons** — corner mismatch, fanning, demote-to-floor on torus blends.
3. **Freeform expected quad flow** — grip/optic/fluted compares still sliver fans.
4. **Adaptive radial floors** — verify cylinder “≥12” against boolean neighbors.
5. **Full Plasticity side-by-side** — artist sign-off on expected meshes.

## Next leverage order

1. Coons open-edge / weld class on multi-edge bspline panels (#1805 family).
2. Torus fillet corner continuity (shared count + non-fanning tips).
3. Keep extracting zoo reducers from each closed class (no filename specials).
