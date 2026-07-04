# Handoff — real-CAD topology campaign

_Last updated: 2026-07-04, branch `claude/plasticity-parity-topology-apieu6`._

This documents where the "watertight topology on real CAD" campaign
stands, how to reproduce the numbers, and what the next contributor (human
or agent) should pick up. It replaces tribal knowledge from earlier
sessions — if a prompt references handoff steps that aren't in this file,
treat this file as the source of truth.

## Test models

Local fixtures (`weft fixture --shape ...`) cover every mesher, including
`notch` — the canonical parametric-grid-meets-triangulated-borders case.

Real-world models used for the baseline (NOT committed; fetch into a
scratch directory):

- `as1_pe_203.stp`, `as1-oc-214.stp`, `RC_Buggy_2_front_suspension.stp`
  from `tpaviot/pythonocc-demos` (`assets/models/`, via
  raw.githubusercontent.com) — two clean AP203/AP214 assemblies and one
  15 MB dirty suspension assembly.
- The user's own set lives in Google Drive (`plasticity.stp`,
  `HDD_SATA_3.5.stp`, `flaregun.STEP`). Note for remote sessions: the
  cloud environment's network policy blocks `drive.google.com` (CONNECT
  403), and the Drive MCP tool only returns base64 into model context —
  unusable for multi-MB binaries. Run those locally, or add an allowed
  mirror.

Never commit test models to the repo.

## State (measured, `weft validate`)

| model | open edges before | after | notes |
|---|---|---|---|
| all 10 fixtures | 0 (analytic only) | 0, `clean()` | in CTest |
| as1_pe_203 | 10,884 | **0** | winding consistent |
| as1-oc-214 | 4,748 | **0** | winding consistent |
| RC_Buggy front susp. | 320,316 (+3,495 non-manifold) | 10,128 (+2,387) | dirty CAD, see below |

Meshing the buggy takes ~45 s (was ~33 s at the old, broken counts; the
whole-shape triangulation now runs parallel — most of the time is OCCT).

## What was built (this branch)

1. **Whole-shape triangulation** — one `BRepMesh_IncrementalMesh` over the
   shape (parallel), so OCCT discretizes each B-rep edge once and adjacent
   trimmed faces share border polylines. Per-face chord/angle overrides
   re-mesh just that face, after capturing its edges' shared polylines.
2. **Plan demotions** — `PlanarGrid`/`RevolutionGrid` plans whose boundary
   is not exactly their full UV iso-rectangle (split rims, slanted trims,
   holes through walls) demote to conformal triangulation. Before this,
   they meshed their UV bounding rectangle at private densities — the
   single biggest crack source on real CAD, and it meshed over trim holes.
3. **Canonical edge polylines + boundary surgery** (`meshers.cpp`:
   `EdgePolyline`, `collectEmittedPolyline`, `conformChain`) — edges where
   a parametric face meets a triangulated face carry the exact vertices
   the parametric side emitted (collected geometrically by projecting its
   emitted vertices onto the edge curve). The triangulated side inserts
   missing canonical points (triangle splits) and collapses leftovers,
   then `flipToDelaunay` (Lawson, UV-convexity-guarded) repairs the sliver
   fans surgery leaves behind.
4. **Conformal quad subdivision** — midpoints of boundary segments are
   evaluated on the B-rep edge curve at the parameter midpoint (bitwise
   identical from both sides); parametric-authored segments never split
   (adjacent cells become n-gons); faces bounded entirely by parametric
   neighbours skip subdivision (plain guided pairing beats n-gonizing the
   border ring).
5. **Weld groups** — one per solid/shell; touching assembly parts no
   longer fuse into non-manifold contact shells.
6. **`weft validate` / `mesh --validate`** (`core/validate.{hpp,cpp}`) —
   open/non-manifold edges, winding consistency, degenerate + sliver
   polygons, chord deviation vs. the live B-rep (periodic-seam aware,
   projection-verified outliers). Non-zero exit when the mesh leaks.
7. **Exact CAD normals in OBJ** (`writeObj(mesh, path, &model)`) — per
   corner `v//n` from the polygon's own face; sharp edges split, tangent
   joins shade smooth, no angle heuristics. `--no-normals` opts out.
8. **Curvature-adaptive default density** — non-overridden curved strips
   floor their divisions at `ceil(angularSpan / angleTolerance)`.

## Known gaps / next steps (in priority order)

1. **Buggy residuals** — 10k open edges + 2.4k non-manifold on the 15 MB
   assembly. Suspected causes: closed edges whose curve parametrization
   phase doesn't match the surface u-origin (canonical collection bails —
   see `collectEmittedPolyline`'s closed-edge handling), faces whose
   `PolygonOnTriangulation` is missing, and genuinely self-touching
   solids. Diagnose with `weft validate` + a per-face open-edge breakdown
   (worth adding to the report).
2. **Grid conformity to arbitrary trims** (plan §7.1 proper) — demoted
   faces currently triangulate. The next level is a boundary-conforming
   quad layout so an angle-trimmed cylinder keeps exact radial control.
3. **Interior refinement for surgered faces** — faces with dense pinned
   borders and sparse interiors pair at ~40% quads. Steiner-point
   insertion (UV grid seeded, Delaunay-flipped) before pairing would
   raise quad share substantially.
4. **glTF (.glb) export** — engine-ready delivery next to OBJ: triangulate
   n-gons, reuse the exact-normal machinery, carry face IDs as a custom
   `_WEFT_FACE_ID` attribute. OBJ carries the full story today.
5. **Per-solid OBJ objects** (`o part_N`) and normals in the app viewport
   (it currently shades flat from polygon normals).
6. **App**: expose `validate` results as an overlay (open-edge highlighter
   is the plan §4.2 diagnostic); the panel already regenerates live.

## Reproduce

```sh
cmake -B build && cmake --build build -j && ctest --test-dir build
build/cli/weft validate path/to/model.step            # bake-ready checks
build/cli/weft mesh model.step -o out.obj --validate  # export + checks
```
