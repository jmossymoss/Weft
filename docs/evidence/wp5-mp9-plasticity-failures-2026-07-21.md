# WP5 — MP9 Plasticity visual failures (2026-07-21)

Artist-supplied screenshots against `tests/STEP_Examples/MP9.stp` (Plasticity
export; no separate STL in-repo — the STEP is the source). MP9 remains
`tier=performance` in `CAD_CORPUS.tsv`; these notes classify failures for
reduction into the zoo, not as a silent release-oracle expansion.

## Batch CAD profile snapshot

| Metric | Pre-sphere | After sphere + ortho seam |
| --- | --- | --- |
| Faces / edges | 3748 / 10155 | same |
| Polygons | ~40k (CAD adaptive) | ~39k |
| Watertight | NO — 1917 open edges | NO — 1086 open edges |
| Slivers | 2143 | ~2123 |
| Demotions | 141 → contract floor | 122 → contract floor |
| Leakiest faces | #1827/#1805/#1904/#1806 (~200 each) | same IDs (~93–97 each) |

## Screenshot classes → code attribution

| Artist note | Example IDs (session) | Class | Status |
| --- | --- | --- | --- |
| Open holes on foregrip | #2334, #2374 bspline | Junction / open-shell + large coons leaks (#1805 family dominates opens) | Partial — ortho border canonicalize |
| Fillets over corners; mismatched counts | torus fillets, coons vs contract-floor | Fillet coons corner / demotion | Open |
| Foregrip fillet fanning | coons-grid torus | Coons rail fanning at tips | Open |
| Outer ≠ inner boolean counts; “5 vs floor 12” | sphere + plane annulus | Adaptive density + wrong sphere floor | Fixed — sphere routing |
| B-spline / freeform on contract-floor | #1446-class | Wrong Fallback routing / demotion | Open |
| Grip / optic / fluted body: Plasticity vs Weft vs Expected | freeform panels | Quad flow vs sliver fans (expected = structured quads) | Open |

Face IDs in the UI can drift across import settings; reducers below pin geometry.

## Fix: sphere dimple class

Trimmed `GeomAbs_Sphere` patches often report `IsUClosed()=false`, so they
skipped `RevolutionGrid` and became planned contract-floor needle soup. After
a fold-check demote the floor was worse than a boundary n-gon.

Changes in `core/src/meshers.cpp`:

1. `isClosedRevolution`: spheres always count as U-closed (surface type).
2. Auto plan: if rim-hug fails on a sphere, still `finishRevolution()`.
3. Fold-check demote: under CAD/`minimal`, rescue folding sphere revolution
   grids to `meshMinimalPlanar` instead of the contract floor.

Reducer: `tests/regressions/mp9/sphere_dimple_annulus.step`.
Test: `testSphereDimpleNotContractFloor`.

## Fix: freeformComb ortho ↔ plane seam class (#1805 family)

Large multi-edge bspline panels match `planOrthogonalTrimGrid`'s freeformComb
gate and mesh as a UV-clipped lattice (`meshOrthogonalTrimGrid`, reported as
`coons-grid`). Clip/pcurve drift left lattice border verts ~0.03–0.12 mm off
the shared 3D edge while planar `minimal-ngon` neighbours sampled the exact
curve — unexplained opens concentrated on #1805/#1806/#1827/#1904.

Change in `meshOrthogonalTrimGrid` (`core/src/meshers.cpp`): widen on-curve
canonicalize radius for cell verts near exact border samples from 0.02 mm to
0.15 mm (class-level; no filename specials).

Reducer: `tests/regressions/mp9/coons_plane_1805_r0.step`
(MP9 #1804/#1805/#1806/#1826/#1904/#1910/#1918/#1957).

| | Before | After |
| --- | --- | --- |
| Full MP9 open edges | 1917 | 1086 |
| Leakiest #1805-family opens | ~194–198 each | ~92–97 each |
| Extract unexplained cracks | 131 | 83 |
| Structured coons on big panels | yes | yes (not demoted) |

Test: `testMp9CoonsPlaneSeamCanonicalize`.

Tried and rejected on this class: raising `solvedEdge` to pin counts (opens
exploded); skipping freeform `pinOrthogonalTrimGrids` (opens got worse);
disabling freeformComb (zeros extract cracks but demotes panels to
contract-floor needle soup — worse visually); wide 3D curve projection of
comb borders (folded cells by yanking notch verts onto the wrong edge);
refusing fold self-heal demote on freeformComb (raised extract cracks and
non-manifold edges).

### FreeformComb stitch deadlock repair (2026-07-22)

Dominant extract opens are face `#2`↔`#5` (shared edge `#41`), not only
plane seams. A fold self-heal demotes `#5` to the contract floor; stitch
then refused to rewrite *both* the floor and the Coons partner → deadlock.
Also Coons↔MinimalNGon skipped both sides.

Changes in `core/src/meshers.cpp`:

1. `FacePlan::orthogonalFreeformComb` from `planOrthogonalTrimGrid`.
2. Border sample snap (0.25 mm) after freeformComb lattice emission.
3. `stitchSeams`: allow freeformComb samples into MinimalNGon and into a
   partner contract floor; allow freeformComb Coons to accept floor
   samples; fold guard reverts bad authority inserts without collapsing
   the comb partner.
4. `fuseSeamTwins`: absolute near-duplicate band on freeformComb seams
   (lattice micro-edges otherwise refuse real twins).

| | Before | After |
| --- | --- | --- |
| Extract unexplained cracks | 83 | 75 |
| Folds / non-manifold | 2 / 0 | 0 / 0 |
| Structured coons on big panels | yes | yes (one sibling may floor) |
| foam / teleporter CAD | watertight | watertight |

## Still open on MP9 (block visual / Plasticity compare exit)

1. **Residual opens** — ~1.1k remain; #1805 family still leads (~75 on the
   extract, still f2↔f5-led after stitch deadlock repair).
2. **Fillet coons** — corner mismatch, fanning, demote-to-floor on torus blends.
3. **Freeform expected quad flow** — grip/optic/fluted compares still sliver fans.
4. **Grip / capsule fillet spans** — uneven coons, open borders vs boolean.
5. **Bullet body / transition (#3728)** — still coons with twisted spans.
6. **Full Plasticity side-by-side** — artist sign-off on expected meshes.

## Fix: bullet tip geometric sphere cap (#3729)

MP9 tip `#3729` is a `GeomAbs_Sphere` with a circular rim that is **not** a
UV pole chart (full model: 3 non-degenerate edges; no collapsing polar iso).
Unconditional sphere→`RevolutionGrid` lofted a fake U-wrap lattice (~half
inverted); fold self-heal then swapped to contract-floor tip soup. Blanket
skip of sphere floor heals broke foam/teleporter watertightness.

Changes in `core/src/meshers.cpp`:

1. `sphereHasUvPoleChart` — revolution only when deg poles / full U period /
   exactly one collapsing polar iso.
2. Closed pcurves → `EdgeIso::Neither` (stops rim in both u/v edge lists).
3. Non-chart spheres plan via a clean `planQuadFill` probe **before** coons
   (writing into a fresh `FacePlan`, then move — leftover plan state was
   enough for the tip to miss the route on full MP9).
4. `sphereCapPole` requires geometric V-iso collapse.

| Probe | Result |
| --- | --- |
| `bullet_tip_3728.step` | sphere → `quad-fill`, build=0, 0 tip tris, watertight |
| Full MP9 `#3729` | `quad-fill`, build=0, 120 quads / 0 tris |
| `sphere_dimple_annulus.step` | still `revolution-grid` |
| foam / teleporter CAD | watertight (0 open edges) |

Reducer: `tests/regressions/mp9/bullet_tip_3728.step`.
Test: `testBulletTipNotContractFloor`. Debug: `WEFT_SPHERE_CHART=1`.

## CAD plate-web residual (2026-07-21)

Under CAD/`minimal`, `meshPlateWeb` still built hole collars (wanted) but
filled the residual with CDT + Steiner refine — the "tonnes of unnecessary
geometry" on demo/torture bored plate walls (artist face often labeled #44;
stable id on `torture.step` is **#5**).

Fix: when `minimal`, residual uses `splitIntoSimplePolys` (bridged n-gons)
and skips `refineFloorWeb`. Collars stay. Demo CAD verts 851→772; plate-web
tris → 0.

Tests: `testPlateWebSliverRefine` (asserts 0 plate-web tris under minimal),
`testTorturePlateWebMinimalResidual`.

## Adaptive curved-ring floor (2026-07-21)

`FaceMeshSettings::minCurvedSegments` (default 6) is a closed-RING total
(cylinders, spheres, torus/fillet circles, annulus bores).

Artist report: UI at 12 still showed hexagonal holes (radial=6). Causes:

1. Ring-junction derived `circle = 2*(nu+nv)` from a sparse CAD plate,
   crushing bore proposals; the global 60° curvature floor then raised the
   circle to 6 and desynced the junction (demote → contract floor).
2. Plasticity-split co-circular open arcs skipped the closed-edge floor.

Fixes in `solveDensity` / `generate()`:

- Ring-junction grows plate `nu`/`nv` so `2*(nu+nv) ≥ max(mincurve,
  adaptive bore)` under adaptive CAD; historic non-adaptive "plate drives
  boss" unchanged.
- Co-circular full rings share `mincurve` by span (two semicircles at 12
  → 6+6).
- Global curvature floor skips `ringDerivedRoots` (already grown) and
  applies `minCurvedSegments` on true closed curves.
- Sphere fold rescue no longer flattens to a planar n-gon when the rim
  already meets `minCurvedSegments` (keeps revolution-grid).

| Surface | Default |
| --- | --- |
| Recipe | `mincurve=N` |
| CLI | `--min-curve N` |
| App | "min curved segments" under adaptive (per-face + global) |

Regression: `testAdaptiveDensity` asserts CAD hole + `minCurvedSegments=12`
keeps rim ≥ 12 and ring-junctions off the contract floor; closed cylinder
+ `densityScale=0.5` still ≥ 12.

## Selection / topology debug aids (2026-07-21)

Artist multi-select was collapsing to "N faces (active #id)" plus knobs for
the last-clicked face only — hard to inventory a broken cluster. App now:

- Lists every selected face as `#id  type [mesher]  build-health` (scrollable;
  click a row to retarget the editor without shrinking the set).
- Shows `faceBuildCause` under the active face when demoted/empty/raw.
- Topology panel `select` buttons for folded-cell owners, open-border-loop
  neighbours, raw-fallback faces, and contract-floor faces (plus the
  existing empty-face select).

Use these to dump the exact failing face set when reporting residual MP9
classes (sphere dimple, fillet #2219/#1644, grip coons, cylinder #3405).

## Next leverage order

1. Remaining #1805-class seam drift (border emission without surf.Value).
2. Torus fillet corner continuity (shared count + non-fanning tips).
3. Keep extracting zoo reducers from each closed class (no filename specials).
