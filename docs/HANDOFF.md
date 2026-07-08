# START HERE — next session (short handoff, 2026-07-08)

WHERE WE ARE. The decoupled-core rewrite is a real OCCT mesher: `weft mesh <f>
--decoupled`, all in `core/src/decoupled.cpp` (~2150 lines), additive/opt-in
(production `generate()` untouched). The structured meshers are SOUND — the
border contract holds everywhere. flaregun is watertight (0 opens); ALL 15
fixtures now fully watertight — `notched` was closed this session by the
rim-notch wall mesher (increment 9), quad-dominant + 0 folds. That mesher also
turned out to be a big corpus win: the rim-open-notch (a cyl/bore whose rim is
cut by a slot) is a very common mechanical primitive, so opens/non-manifold
dropped across the whole STEP corpus with NO regressions (default weld:
4pinplug 12/7->0/0, 2827056 154/100->4/0, weldment 258/71->76/17, mohne
80/29->4/2, unterlaf 21/13->11/1, 1797609in 20/21->0/4, teleporter 630/77->539/18,
foam 271/30->257/27; flaregun/iso/nasty/angle/as1 unchanged). foam/teleporter's
remaining opens are still SUB-TOLERANCE INPUT GAPS (~0.007mm imprint artifacts in
the sloppy CAD) closed by `--weld 0.01` (now foam 94, teleporter 27). Full
increment log is in "ACTIVE DIRECTION" and the increment sections below.

STANDING CONSTRAINTS. Commit as `Jordan Moss <jordan.moss@live.co.uk>`, NO AI
attribution anywhere (no Co-Authored-By/Claude trailers, no PR footer). User's
STEP models live in `tests/STEP_Examples/` (foam, teleporter, flaregun are
committed). Build: `./build.sh` (full+ctest) or `cmake --build build --target
weft weft_tests -j4`. (Headless build note: the app needs GLFW's X11/wayland dev
packages + libtbb-dev; `-DGLFW_BUILD_WAYLAND=OFF` avoids wayland-scanner.)

VERIFY / DIAGNOSE. `weft mesh <f> --decoupled --validate` (opens/nm/winding/
folds). `WEFT_DC_CONTRACT=1` = border-contract verifier (which mesher pairs
fail to weld; +`WEFT_DC_CONTRACT2=1` for per-edge detail). `WEFT_FACE_KINDS=1` =
per-face mesher dump. ctest = `tests/test_pipeline.cpp::testDecoupled`.

NEXT (in priority order):
1. FREEFORM UV-COONS interior (quality, not opens): grid 4-sided bspline patches as
   quad grids instead of the tri floor — the floor path is meshFloorAuto (surface
   UV, pcurve-based, seam-unwrapped) in decoupled.cpp. This is THE dominant
   remaining quality gap: bspline faces on the tri floor DOMINATE (flaregun 82,
   foam 128, teleporter 215), and ~most are 4-EDGE curved quad patches (flaregun
   65/82, teleporter 151/215) — perfect Coons/transfinite targets. (Increment 10
   already cleared the cyl/cone floor walls.)
2. Decide whether to raise the default decoupled weld for imports (production
   keeps 1e-6 + relies on --weld, so leaving it is consistent).
3. Then: port the app/CLI to prefer the decoupled path, and A/B the two meshers.

---

# ACTIVE DIRECTION (2026-07-08): decoupled-core rewrite

Branch `rewrite/decoupled-core` (PR #25). This supersedes the incremental
meshers.cpp campaign below for new work. The bet, de-risked by two standalone
spikes (`core/spike/{seam_bridge,decoupled_face}.cpp`, 12/12 + 7/7 green):
faces couple ONLY through per-edge sample counts. Every shared B-rep edge is
sampled ONCE at a count that is a pure input (per-edge pin / per-face
radial+axial / geometry default); both incident faces read the same 3D points,
so they weld with NO global density solve and NO ripple. Each face meshes its
interior at its own count; the loop bridge absorbs interior↔border mismatches.

## Increment 1 (LANDED this session) — real OCCT core scaffold
New module `core/src/decoupled.cpp` (+ `core/include/weft/decoupled.hpp`),
`weft::meshDecoupled(model, analysis, settings, report)`, wired to the CLI as
`weft mesh --decoupled` (reuses GenerationSettings: radial/axial → revolution
interior, gridU/gridV → planar, perEdge → pins). Reuses `weldVertices` +
`triangulatePoly` + `validateMesh`. Additive and fully opt-in — `generate()` is
untouched. Test `testDecoupled` in test_pipeline.cpp; ctest green.

Pipeline: (1) solveEdgeCounts — per-edge count from geometry (full circle→
radial, arc→proportional, line→1, freeform→curvature) + revolution rim/side
proposals + perEdge pins. (2) sampleEdge — each edge's OWN 3D curve, arc-length
for freeform / phase-anchored ring for closed circles (coaxial rings share
column angles), N segments→N+1 pts. (3) per-face interior: meshRevolutionWall
(full cylinder/cone — azimuth-ALIGNED rim rings lerped into a grid, cone apex
collapse), meshFullPeriodic (full sphere/torus UV grid, v≥3 rings for a torus
tube), planar single-loop → boundary n-gon, non-planar 2-loop → bridgeLoops,
else keyhole floor. (4) weld per solid. (5) orientMeshConsistent — global
flood-fill winding pass (per-face orient isn't a global guarantee).

VERIFIED CLEAN (watertight + consistent winding + 0 fold): cylinder, box, cone,
sphere, torus, fillet — and demo/barrel/ribbon/ribbonnotch. Cylinder = 16 wall
quads + 2 n-gon caps (the Plasticity-parity target).

## Increment 2 (LANDED) — plate-web (planar plate-with-hole)
`meshPlanarAnnulus`: a flat face with one hole is cut by TWO non-crossing,
visibility-checked bridges into two SIMPLE n-gons (each emitted whole; the
engine triangulates n-gons — no ear-clip-over-keyhole to fold). Robust for any
count ratio (coarse rectangle outer vs fine circular hole). hole/boss now
watertight + consistent winding + 0 fold + quad-dominant; both promoted into the
strict `testDecoupled` list. Non-planar 2-loop bands still take `bridgeLoops`;
3+-loop plates still take the keyhole floor (multi-hole decomposition TODO).

## Increment 3 (LANDED) — partial-revolution walls (open-u cyl/cone bands)
`meshPartialRevolutionWall`: a partial-wrap cylinder/cone wall (< 360 deg) is
classified into two rim ARCS + two straight SIDE lines by curve type; the arcs
are azimuth-oriented (index 0->1 CCW so both share azimuth per column) and the
interior is lerped between them (rulings lie exactly on a cyl/cone), with side
columns taken from the side edges' shared samples so all four borders weld. Only
a clean 2-arc/2-line boundary with matched counts qualifies; anything else bails
to the floor. barrel 118p/112t -> 66p/28t; a 270-deg wedge solid is a watertight
pure-quad wall (new `testDecoupled` case). Hero models gained quads / lost tris
(flaregun 2769t->2293t, foam +450 quads) with no new leaks.

## Increment 4a (LANDED) — freeform floor triangulates in surface UV
`meshFloor` is parameterized by a coordinate array; `meshFloorAuto` feeds it the
per-vertex anchors' (u,v) for freeform faces (bspline/bezier/revolution/
extrusion/offset/other) so a curved patch conforms in its own parametrization
instead of collapsing under a 3D-plane projection. Guards bail to the 3D verts
on a full-period seam wrap (u-span > 1.9pi) or degenerate anchors. Analytic +
planar faces keep the 3D plane. No new opens; non-manifold edges dropped
(flaregun 26->16, foam 66->46, teleporter 52->32). The remaining OPENS are the
analytic-periodic / planar-multi-hole floor (below), the genuine trimmed-surface
problem — a proper constrained triangulation or the specific hard meshers.

## Increment 4b (LANDED) — multi-hole plate decomposition
`meshPlanarMultiHole` generalizes the single-hole two-bridge to K holes: each
hole splits its containing polygon with two non-crossing bridges (clear of the
polygon, that hole, AND every other hole), remaining holes are redistributed by
point-in-polygon containment, so a K-holed flat face becomes K+1 simple n-gons
with real shared edges. All-or-nothing: bails to the floor if a hole can't be
bridged. Planar 2+-loop faces route here (kind PlateWeb); hole/boss use the same
path. Quality win on holed plates: foam tris 4117->2789 & nm 46->16; flaregun
tris 2293->1905 & nm 16->12 (holed planar faces were tri floors, now clean
n-gons). New 2-bore-plate testDecoupled case.

## Increment 5 (LANDED) — bridge unequal revolution rims
meshRevolutionWall bridges the two closed rims (CCW-oriented) instead of bailing
when their counts differ (cone/frustum with a pinned rim).

## Increment 6 (LANDED) — revolution wall with interior bore holes
meshRevolutionWallInsert: main rims = the 2 extreme-height closed CIRCLE edges;
holes = the inner wires (bore rims are intersection curves, not circles). Grid
u-wrapped rings with v-rows bracketing each hole, punch the covered cells, bridge
the staircase to each hole's exact rim samples. Runs before meshRevolutionWall.
Also meshRevolutionBandLoops for cyl/cone/torus/sphere bands whose rims are
multi-edge loops. slotted opens 99 -> 0 (residual: boolean-cut bore walls whose
rims are split by the seam still floor -> 9 nm).

## Increment 7 (LANDED) — weld shared-border periodic patches (BIGGEST win)
meshFullPeriodic gridded any full-u sphere/torus from surf.Value, ignoring shared
edges — so a torus FILLET BAND leaked both rims. Now it bails when the face has
any shared edge (a patch, not a complete surface) and routes to
meshRevolutionBandLoops (bridges the two shared rim loops from the cache).
Hero-model opens collapsed: flaregun 515->64, teleporter 1455->673, foam
959->620; bossfillet 64->7.

## CURRENT STATE (2026-07-08, --decoupled): structured meshers SOUND
flaregun 0 open (was 515, WATERTIGHT). Fixtures: 14/15 fully watertight -- only
notched (7o/3nm, rim-notch) remains; slotted + bossfillet now CLEAN via the seam
band. foam 271 open / teleporter 630 at the DEFAULT weld (1e-6) -- but those are
SUB-TOLERANCE INPUT GAPS, not mesher bugs: adjacent bspline patches on the
sloppy-CAD hero models meet at nearly-coincident edges ~0.007mm apart (verified:
two distinct verts 0.0069mm apart at a "crack"). `--weld 0.01` closes them ->
foam 271->108 (108 ~= the 74 legit input open-shell edges), teleporter 630->114.
So the decoupled mesher is watertight on clean geometry; sloppy imports need the
weld knob exactly like the production path. The border contract holds
everywhere (WEFT_DC_CONTRACT).

REMAINING (small): notched rim-notch (a rim-open-notch mesher); and whether to
raise the default decoupled weld for imports (production `generate()` keeps
1e-6 and relies on --weld too, so leaving it is consistent).

## Increment 8 (LANDED) — seam band closes encircling annuli
meshSeamBand: a periodic face (torus fillet ring, boolean-cut cylinder bore
wall) whose boundary is ONE wire encircling the seam is an annular band, not a
disk. Reconstruct the two rims from the EDGE structure -- a rim edge's pcurve
runs along the encircling axis A, a seam edge runs across it; group u-varying
edges by their B-midpoint into two rims -- sample each rim from the shared cache
keeping ON-CURVE order (so every ring edge is a real B-rep segment), order the
edges by azimuth (uv[A] mod period), and bridgeLoops closed. Gated three ways so
it is a PURE improvement: (1) a rim must wind >0.75*period (segments stay on the
floor), (2) a COVERAGE check -- every shared boundary edge's samples must land on
a rim, else a misclassified face would drop a shared edge, (3) a WATERTIGHT
self-check -- the band cells must be 2-manifold with exactly the two rims as
boundary, else roll back. Runs after meshRevolutionBandLoops for cyl/cone/
sphere/torus. flaregun 64->0 opens; slotted 9nm->0; bossfillet 7o->0; foam/
teleporter improved (their remaining encirclers are non-clean and correctly
rejected to the floor). testDecoupled now asserts bossfillet + slotted watertight.

## Increment 9 (LANDED) — rim-open notch walls (biggest corpus win yet)
meshRimNotchWall + bridgeByAzimuth. A full-wrap cyl/cone WALL with ONE full
closed-circle rim and an OPPOSITE rim cut by a NOTCH open to that rim (the
flaregun face-81 class / any slot cut through a rim). In UV it is the rectangle
[0,2pi]x[hLo,hHi] MINUS [u0,u1]x[vNotch,hHi]; it ENCIRCLES the axis so the
seam-unwrap floor bails and its 3D projection self-overlaps (notched: 7o/3nm/4f).
Fix: walk the outer wire -> the un-notched rim (single closed circle) is the `lo`
ring; the shared, non-closed chain edges (top arcs + side drops + notch floor),
concatenated in wire order, form the `hi` ring (its two ends meet at the seam so
it closes in 3D and dips into the notch). Bridge lo->hi paired by AZIMUTH (not
index fraction, which folds at the notch dip) via bridgeByAzimuth: unwrap both to
monotone azimuth, merge-stitch (quads where they advance together, triangles
across a dip). Side drops (consecutive hi samples at one azimuth) are absorbed as
fans; an UP drop (exiting a notch) advances lo past the mouth FIRST so its corner
triangle fans from the full-height side (else it slivers inward -- the one fold we
chased down). Made a PURE improvement by four gates that roll back to the floor:
(1) exactly one outer wire + one closed-circle rim + a shared non-closed chain;
(2) naked-edge guard -- a non-shared edge is skipped only if BRep_Tool::IsClosed
(the true periodic seam), else bail (don't swallow a gap and chord across it);
(3) splice/closure coincidence within max(weld,1e-6) at every chain join (no
phantom chord over the cut-away); (4) span guard in the bridge (a non-monotone hi
ring -- dovetail slot, or seam inside the mouth -- unwraps past one period ->
bail) + a topological 2-manifold self-check + an azimuth-extent fold gate (a cell
spanning >=pi is a fold the normal-vote misses). NB a per-cell normal-vote fold
gate was tried and REVERTED: seam-ambiguous anchor projection gave valid bands
spurious mixed signs and rolled clean walls back to the leaky floor (unterlaf
folds 246->862). Results (default weld, decoupled --validate): notched
7o/3nm/4f->0/0/0 (quad-dominant); corpus-wide with NO regressions -- 4pinplug
12/7->0/0, 2827056 154/100->4/0, weldment 258/71->76/17, mohne 80/29->4/2,
unterlaf 21/13->11/1, 1797609in 20/21->0/4, teleporter 630/77->539/18, foam
271/30->257/27 (only foam gains 46 folds, out of a corpus fold drop of thousands:
weldment 3009->766, unterlaf 862->246, 2827056 72->2). testDecoupled asserts
notched watertight + fold-free. The design was adversarially reviewed before
coding; the review's split-at-vNotch quad grid and subdivided-base-rim
reconstruction are the queued follow-ups (see START HERE next steps).

## Increment 10 (LANDED) — subdivided rims + down-run hardening
Generalized meshRimNotchWall from "one closed-circle rim + a notch chain" to any
full-wrap cyl/cone wall whose boundary is TWO encircling rings joined by the
periodic seam. Instead of picking a single closed circle, it SPLITS the outer wire
at the seam edge(s) (the true periodic seam, BRep_Tool::IsClosed) into two runs of
shared edges; each run builds into a ring (a lone closed circle is a ring; an open
arc-chain closes at the seam), gated by the same splice/closure coincidence checks.
The ring with the larger axial spread is the notched one (hi), so bridgeByAzimuth's
side-drop handling applies; the flatter one is the base rim (lo). This catches
boolean-cut walls whose rims are arcs, not circles -- a very common class (35-121
such faces per hero model were landing on the floor). Also added the symmetric
DOWN-run branch in bridgeByAzimuth (the increment-9 verifier's recommended
hardening): a notch-entry drop is fanned from the current lo column BEFORE the quad
tie-break, so a quad can't pair lo past the drop azimuth and leave a backward-wound
sliver. Results vs increment 9 (no open/nm regressions anywhere): nasty_cheese
139nm/1298f -> 2nm/329f (!), 1797609in 4nm/8f -> 0nm/5f, mohne 2nm/26f -> 1nm/19f,
weldment 76o/17nm -> 73o/16nm; fixtures unchanged. Same rollback gates keep it a
pure improvement -- a non-band face fails the sweep/splice/self-check and takes the
floor.

DEFINITIVE DIAGNOSIS (via the new WEFT_DC_CONTRACT verifier, which checks that
every shared-edge sample lands on a welded vertex used by >=2 faces):
- The STRUCTURED meshers are SOUND. flaregun has ZERO border-contract
  violations; foam/teleporter violations are dominated by fallback-tri. Don't
  keep re-checking the grid meshers -- the border contract holds.
- EVERY remaining open is the FLOOR (fallback-tri) triangulating a face whose
  UV/3D projection SELF-INTERSECTS. Confirmed root case: a torus fillet SEGMENT
  whose u-range wraps the seam (flaregun face 104) falls back to a 3D projection
  that self-overlaps -> 6 opens + 1 nm. Exact pcurve UV (landed) does NOT fix it
  because the seam-wrapping loop is non-simple in the flat UV rectangle.

SEAM-AWARE FLOOR (landed the foundation): the floor now (a) reads exact pcurve
(u,v) and (b) UNWRAPS the periodic seam so a seam-CROSSING patch stays a simple
loop. This correctly handles non-encircling seam faces. It splits the remaining
leaks cleanly in two:
- NON-encircling seam faces: fixed (unwrap -> simple UV -> clean triangulation).
- ENCIRCLING faces (a SINGLE loop that winds a full period in u or v): these are
  the actual remaining leakers -- and they are NOT disks, they are ANNULAR BANDS
  (a full-ring fillet whose two rims + connecting seam are ONE wire). foam's 18 /
  teleporter's 44 / flaregun's 9 encircling faces account for ~ALL remaining
  opens (each leaks ~2*radial). They fall back to the 3D floor and self-overlap.

THE ONE REMAINING PIECE (biggest opens win): a reliable ENCIRCLING-BAND mesher.
Concept: extract the two rims (classify loop points by the non-encircling coord
into the two B-extremes; the in-between points are the seam edges), order each by
azimuth (uv[A] mod period -- NB the two rims are the SAME circle but pcurve gives
them DIFFERENT u-offsets, e.g. face 104 rimA u in [1.05,6.94], rimB in [1.05,
-4.84], so you MUST pair by u-mod-period, not raw u), then bridgeLoops closed.
TWO attempts this session (crossing-split; classify+sort+bridge) both made opens
WORSE -- the rims don't cleanly close over the seam gap and not every encircling
face is a clean 2-rim annulus, so a naive `return true` replaces a bad floor with
a worse band. Do it with a WATERTIGHT SELF-CHECK: mesh the band into a temp,
validate its own edges are 2-manifold, and only keep it if clean (else fall to
3D) -- that makes it a pure improvement. This is meshRevolutionBandLoops applied
to 1-wire faces; that function already handles the 2-wire case.
Everything else is bounded:
- RIM-OPEN NOTCH (notched 7o): full/partial wall with a rim-open notch.
- BOOLEAN-CUT BORE WALLS (slotted 9nm): a full cylinder whose rims are arcs
  split by the seam -- reassemble the 2 rim rings from the arcs (the band mesher
  needs 2 wires; these bores are 1 wire).
- FREEFORM UV-coons interior (quality, not opens): grid bspline patches as
  quads instead of the tri floor.

DIAGNOSTICS: `WEFT_FACE_KINDS=1` dumps the per-face mesher; `WEFT_DC_CONTRACT=1`
reports border-contract violations by mesher pair; add `WEFT_DC_CONTRACT2=1` for
per-edge detail. The floor path is meshFloorAuto -> meshFloor / meshPlanarMultiHole
in core/src/decoupled.cpp.

Everything below is the PRIOR campaign (meshers.cpp `generate()` path), still the
production mesher and still valid history. The decoupled core will grow to
replace it; until then both coexist.

---

# Session handoff — Plasticity-parity topology campaign

Read this first in a fresh session, then continue the plan below.

## Standing constraints (never violate)
- Commit as `git -c user.name="Jordan Moss" -c user.email="jordan.moss@live.co.uk" commit` — NO AI attribution lines anywhere.
- Branch: `claude/git-reset-scratch-ykbkcv`, push with `git push -u origin <branch>`.
- User's STEP/OBJ models are NEVER committed. Fetch from Google Drive into the
  session scratchpad: VentGasket.stp, NoHands_v2/v3/v3.2.stp, SawGuide.stp,
  DualHose3.stp, ExtendedClipHolder.stp, flaregun.STEP, and the reference
  `test export.obj` (Plasticity's flaregun export, ~1.9MB).
- User builds on Windows (build.bat). Keep `app/gl_compat.hpp` in sync when
  adding GL entry points (Windows has no glext.h).
- Probes live in `tools/probes/` — compile with `relink.sh probeNN` (edit its
  path first: it links `build/core/libweft_core.a`). ALWAYS relink after
  rebuilding core; stale probes have produced false diagnoses twice.

## Mission (task: Plasticity-parity topology)
User verdict: "I need clean good topology, nice rails, good alignment on
fillets and none of this triangle soup when a brep can't be solved."
Target = Plasticity's flaregun export: 9105 polys, 81% quads, 1.8% tris,
1603 n-gons, 0 open / 0 non-manifold. Its measured recipe:
1. Ruled surfaces sweep as full-length strips (37% of polys aspect >5);
   never subdivided along the straight direction.
2. Count mismatches absorbed as 5-gons with one tiny edge (1114 of them,
   median short/long edge ratio 0.08) — NOT triangle zips, NOT global
   count syncing.
3. Planar faces = single n-gons (71/84 big n-gons exactly planar).

## Measured experiments (do NOT redo — all reverted)
- Coons cap 8→16: minimal-profile tris 951→249 (96% quads) BUT chain
  fixpoint syncs rails → defaults 37,808 polys, multi 189. Cap raise needs
  count decoupling first.
- Universal pre-fallback quad-fill: watertight everywhere (its solved-count
  borders are sounder than fallback), BUT grid density follows dense
  borders → SawGuide defaults 15×, minimal profile heavier + more tris.
- Weft `defaults.minimal=1` on flaregun: 8012 polys / 6973 q / 951 t —
  87% of tris from SIX fallback faces: 114 (bspline,16e), 50/55
  (cylinders,14e), 207 (bspline,14e), 211 (bspline,10e), 197 (sphere,3e).

## Done (committed)
- `unionSeams` in core/src/meshers.cpp (runs post-weld in generate()):
  topological T-junction absorber. Open edge (u,v) + exact complement path
  v→w→u + w on segment + no duplicate directed edges created (count map
  updated live) → splice w into (u,v)'s polygon. Close-only by
  construction. Result: SawGuide quads 3 opens → 0 (first fully
  watertight), all else identical.

## Integrated from the parallel main line (2026-07-04)
This branch became main again with the other campaign's deliverables
ported on top: `weft validate` + `mesh --validate` (bake-ready checks,
leakiest-face attribution), binary glTF export (`-o out.glb`, one node
per body, `_WEFT_FACE_ID`), exact CAD normals in OBJ/glb (per-corner,
sharp edges split), STEP body names on `o`/node blocks (transfer-map
walk; Plasticity writes them; dedup suffixes), `--lods` density tiers,
`--yup/--scale/--triangulate` kept from this line, OCCT 8.0 header-
deprecation gating (weft::ShapeMap aliases), Windows fixes (NOGDI /U,
TColStd includes moot via auto). Stability: parallel meshing pre-warms
OCCT's lazy caches (UVBounds/Surface/Curve races segfaulted on an
8k-face model), and solved counts clamp at 256 in countFor — an
observed ~983k cascaded count OOM'd/crashed the Coons mesher. The
clamp is a tourniquet: count decoupling (step 1 below) is the cure.

## Revolution inserts (landed)
Full revolution bands with interior slot/hole wires (the flaregun barrel
case) now stay revolution grids: edgesHugRimsOrInserts collects strictly
interior closed wires, meshRevolutionInsert drops the covered cells and
webs the staircase to the wires' exact border sampling (3D edge curves
at solved counts — the wall faces' contract), multi-hole keyhole
ear-clip per staircase loop. Fixture `slotted` reproduces it; watertight
in defaults (guard falls back when a wire can't form a ring) and with
explicit counts + adapt=1. KNOWN GAP: --adaptive with AUTO radial can
leak at slot WALLS — the wall coons fails and demotes at mesh time,
which breaks the border contract (the pre-existing demotion crack class,
see doctrine). Plan-time routing for that case is part of step 1/2.

## Count decoupling round 1 (landed) + round 2 contract fix
Step 1 is implemented: the solver's chained-coons fixpoint (grow lighter
side until totals match) is gone; unionSeams extended to MULTI-VERTEX
complement paths (walk up to 8 open edges v->w1..wk->u, monotonic
on-segment t, bail on ambiguity, all non-manifold guards kept); coons
cap raised 8 -> 16. Profile flag: `--profile cad` = minimal + adaptive.

ROUND 1'S MISTAKE (fixed in round 2, keep it fixed): round 1 made
meshCoonsGrid arc-fraction-RESAMPLE the deficit rail and emit it — a
re-spaced shared border, the exact doctrine violation the code's own
comments warn about. Field result: folded cells + open border loops on
the user's flaregun/HDD/9mm-case tests; buggy opens 9,068 -> 11,007.
Round 2: the deficit rail's NATURAL solved points stay the emitted
border; the resampled rail is only interior blending scaffold (never
emitted); a monotone TRANSITION STRIP of quads (5-gon wherever the
dense line contributes an extra point — the Plasticity absorption
pattern) bridges the natural rail to the first interior grid line.
Corner stubs re-route into the strip's first polygon. Result: buggy
opens back to 9,087 (pre-decoupling baseline 9,068), polys 150,843,
all fixtures + as1 watertight in defaults and cad profile.
Flaregun board still needs the user's machine (probe27, both modes,
quad% targets). Absorber knobs if it regresses: walk depth (8),
on-segment slack (8%), pass count (4).

## The contract architecture (2026-07-05 — the load-bearing invariant)
THE BORDER CONTRACT: every vertex on a shared B-rep edge comes from
sampling that edge's 3D curve at exactly solvedEdge[eid] uniform
curve-parameter steps, honouring face-local edge orientation. Every
mesher follows it; nothing else may emit a border. Enforcement now has
three layers (all in core/src/meshers.cpp):
1. meshContractFallback — the demotion floor: wires sampled at solved
   counts, holes keyhole-bridged in UV (anisotropy normalized),
   triangulateWeb region fill. Exact borders by construction; any face
   with pcurves can land here and never leak.
2. borderContractViolation — a postcondition run on every planned
   mesher's part: each border edge's solved samples must appear as
   polygon edges or the face demotes through demote() (contract floor
   first, verified; raw OCCT triangulation only when even the floor is
   unavailable). All demotion sites route through demote().
3. Meshers that can genuinely need mismatched counts absorb them
   INSIDE the face: coons transition strips (natural rail bridged to
   the first interior line, quads + 5-gons), revolution closed strips
   (exact rim ring bridged to the uniform interior ring), insert bands
   (grid rows placed exactly at each slot band's v-extents so the
   staircase closes by construction, everything validated before
   emission).
Fixed this round (all were silent leaks): adaptive nv=1 insert bands
deleting rim-to-rim (the 168-open slotted case, `--profile cad` on the
15MB assembly was 130k opens), insert webs skipped when staircase
chains failed to close, insert-wire iso edges contaminating rim rows,
rim sampling ignoring face-local orientation, annulus emitting nothing
on thin rings, seam edges walked twice by planAnnulus, per-edge pins of
0, torus --axial 1 emitting nothing, absorber chord test dropping
curved seams (now 25% sagitta allowance + detour bound, walk 24,
passes 8). CLI gained --density F (global budget dial, composes with
--lods). Gates: ALL fixtures watertight in defaults/--adaptive/
--profile cad at densities 0.2–5; both as1 assemblies watertight.

## Field reports 2026-07-05 (user's HDD + flaregun screenshots, face IDs)
FIXED this round: HDD #245/#248 washer/dome rings (bspline revolves now
detected by isClosedRevolution via IsUClosed; annulus accepts 24-arc
outlines; curved two-loop faces no longer zipper flat), HDD #484/#240
fillet band crumples + zig-zag (ruled-chart interiors; arc-fraction
strip maps; arc-uniform coons scaffolds), HDD #169 quad-dominant vertex
spray (fallback now pairs only — NO midpoint subdivision, per user:
"un-triangulate, don't add edges"), bossfillet fixture reproduces the
boss-rim ring class (routes revolution-grid, watertight).
STILL OPEN (the strip-mesher gap, next big step): flaregun #49/#55/#56
jacket segments (partial cylinder wraps around the side slot — chained
coons shears rungs; needs iso-aligned ruled strips), #50 (slot touches
the border, falls back as fan spray), #112/#145/#146/#172 + HDD #314
(crescent/lune sliver bands and notched cone bands — two long rails
converging at tips; need a rail-ladder strip mesher: arc-fraction
monotone pairing rail-to-rail, quads + absorbed 5-gons, ends collapse
to the tips; emitStrip already implements the pairing — promote it to
a full mesher with plan-time routing for 2-rail faces).

## Audit backlog (13-dimension fleet, 47 agents, adversarially verified)
LANDED: pole-to-pole nv floor (full sphere at library-default axial=1
emitted ZERO polygons silently), v-closed surfaces refuse the insert
path (row alignment impossible -> contract floor), seam-cell centers
unwrap the period before box tests, insert branch dispatches before the
unlinked-rim taper (a taper never cuts slots), cache keys cover insert
wires + chained coons sides + all coons border counts (stale-part
reuse after density edits on slot borders was the CRITICAL find).
REMAINING (all in conformFallbackBorders + friends; consider narrowing
or retiring conform as contract coverage grows): solid-blind neighbour
pick splices the OTHER body's vertex ids on 3+-face contact edges;
pinned-resample targets land in weld group 0 (cross-body fusing);
1.2x-chord mover capture kidnaps other edges' verts on faces narrower
than the radius (corner folds); anti-wrap check skipped for counts <=3;
excluded faces can squat nfid; fellBack should be tri-state (floor vs
raw OCCT) so conform stops treating exact floor borders as decimated;
microTol is model-relative and eats small features on large assemblies;
quad-fill collar containment is vertex-only (can fold across thin
features of another loop). Full details: the workflow result JSON in
the session task file (14 confirmed / 22 refuted).

## Bracket reproducer (user upload, 2026-07-05)
User-supplied STEP bracket exposed the routing gap: with minimal ON,
the auto chain tried RingJunction/Annulus/PlateWeb BEFORE default
minimal, so the main plate meshed as an ear-clip triangle web. Fixed:
minimal now owns every flat face it can express (junction patterns
only see flats when minimal is off or fails); pattern tests opt out
via defaults.minimal=false. Bracket: 2318 -> 1685 polys, tris 1137 ->
788, plate faces are keyhole n-gons. REMAINING on this model: faces
5/8/37 still PLAN fallback-tri (flats planMinimalPlanar refused —
diagnose the gate), face 16 is minimal-ngon yet emits 143 tris
(investigate which path), 4 folded polys, 11 degenerate. The model
lives in the user's uploads only — never commit it.

## Visual review of tests/STEP_Examples (11 models, fleet-reviewed renders)
Verdicts: as1_pe/4pinplug/2827056 minor; bracket(1797609in)/angle1/
iso14649/mohne/nasty_cheese/unterlaf/weldment bad; tork = broken source
per user, IGNORE. tools/visual_check.sh runs the sweep (mesh + validate
+ 3 renders + report.html); visual review is part of the gate now.
FIX CLASSES, in priority order:
1. KEYHOLE N-GONS ARE NOT PRODUCTION POLYGONS. Doubled-bridge keyhole
   rings render/import as hole membranes (angle1's sealed bores) and
   show slit edges on every holed panel (as1_pe, 2827056, iso14649,
   mohne). Replace with SIMPLE decomposition: two non-crossing bridges
   per hole, splitting the panel into k+1 simple n-gons with real
   shared edges (what Plasticity exports). Implement in
   meshMinimalPlanar's rings>1 path (and quad-fill webs' outer rings);
   remove the keyhole splice from display/export paths afterwards.
2. Annulus/disc zippers emit WWWW triangle zigzags when ring counts
   differ (unterlaf perimeter, weldment flange fan, mohne cap sliver
   band). Replace meshAnnulusRing's alternating-triangle walk with the
   arc-fraction grouped bridging already used by the closed strips
   (quads + isolated 5-gons). Disc caps of large radius likewise.
3. iso14649-demo: bores render as staggered brick tris (69% tris
   overall) and a pocket has a folded membrane — run --debug, find why
   its prismatic/bore faces demote or plan fallback, fix the gate.
4. nasty_cheese: 243 folds + 57 non-manifold + hole barrels collapsing
   into dense baskets — likely tiny-feature density explosion plus
   fold-prone coons; diagnose per-face with --debug + isolate.
5. weldment: giant chord tris on round end plates (disc caps at huge
   radius rendered as coarse chords?) + 55 folds; 2 open border loops.
6. bracket pocket: counterbore pocket floor fans + countersink cone
   giant tri (cone face routed wrong at tiny radius, cf. face 37
   radius 0.313 planning fallback).
App/display: sheet models need two-sided shading (tork showed
backfaces as black); fold-highlight magenta overlay works well.

## One-shot batch results (2026-07-05 late)
LANDED (commit e628dcb): simple n-gon decomposition (keyhole slits and
hole membranes gone — holed panels = k+1 simple n-gons, two real
bridges per hole), annulus arc-bridging (WWWW sliver bands gone),
plain-coons sides sample solved counts, endpoint-tolerant border check
(lying corner tolerances were demoting good quad grids: iso14649 went
69% TRIS -> 98% QUADS, 22,598 -> 12,245 polys, zero demotions/folds),
degenerate-skip + manual wire chaining in the ring sampler, and
solid-scoped conform neighbours.
TOP OPEN ITEM — CONTACT-FACE DEDUP: pre-imprinted assemblies (as1_pe,
as1-oc-214) carry coincident internal contact plates (2-3 faces over
the same area, e.g. as1_pe faces 20/21/22). When duplicates mesh
IDENTICALLY (defaults minimal), the weld fuses them into non-manifold
sandwiches (82/56 nm, defaults only; cad profile unaffected because
adaptive counts differ per face). Detect coincident face pairs at
analysis (same edge set / same sampled AABB + centroid), mesh ONE,
skip or link the twin — the PixYZ-parity dedup feature. Until then
DEFAULTS on such assemblies reports the sandwich honestly.
Remaining from visual review: nasty_cheese folds/baskets, weldment
disc chords + flange fan (disk-cap/plate routing at huge radius),
bracket pocket floor fans + micro-radius cones (face 37 r=0.313),
crescent/lune strip mesher (the queued rail-ladder feature).

## Queue state after the curvature-floor round (2026-07-05, latest)
LANDED: solver curvature floor (no curved edge below ~60deg/segment —
killed the as1 'sandwich' which was really collapsed bend arcs fusing;
board-wide wins: bracket 187->31 tris, weldment 1743->571, unterlaf
4650->3209, mohne 458->295); CLI fold attribution (worst faces named).
Board: ALL ten kept examples watertight in cad profile; as1 defaults
0/0 again; fixtures green all modes/densities.
NEXT, in order:
1. RAIL-LADDER STRIP MESHER (the big one): crescents/lunes, notched
   bands, freeform pocket walls — arc-fraction monotone rail pairing,
   quads + absorbed 5-gons, tips collapse; plan-time routing for
   2-rail faces. Fixes bracket pocket fans, flaregun jackets 49/55/56,
   HDD cone 314, mohne strip webs, iso pocket walls at quad quality
   (they sit on the tri floor today).
2. nasty_cheese basket eruptions: drills tangent to faces create
   warped sliver patches (faces 46/321, 116 folds each) and 57 nm in
   defaults; needs per-face isolation (app --select renders) and
   probably plan-time sliver detection routing to the floor.
3. weldment disc chords: round end plates render as giant chord tris;
   check disk-cap vs minimal routing at large radius + fold overlay.
4. Contact-face dedup: still worth having for true imprinted
   assemblies (the PixYZ-parity feature), no longer urgent for as1.

## Rail-ladder + floor-first round (latest)
LANDED: MesherKind::RailLadder (two-tip bands: crescents/lunes/tangent
strips — wire-tangent detection >45deg, arc-fraction rail pairing,
quads + grouped 5-gons, tips collapse; engages e.g. 2 faces on
nasty_cheese, 4 on weldment). Planned Fallback/QuadDominant faces try
meshContractFallback FIRST (exact borders; OCCT only when the floor
can't express the face) — weldment 23->7 opens, 62->22 folds.
Weldment residual: 7 scattered opens (#2/#4 x2, #86/91/92 x1) against
2 non-manifold input edges — likely true input dirt; verify with
--debug + app isolate next session. nasty_cheese basket faces remain
(46/321 warped tangent-drill patches). The ladder currently targets
EXACTLY-2-sharp-corner outlines; extending to notched bands (2 tips +
smooth notches) and 4-corner elongated fold-prone strips is the next
quality lever, along with fillet-loop rows inside ladder rungs.

## Fold-postcondition round (latest)
LANDED (3a00abc): per-part fold vote (Newell vs projected CAD normal,
>25% inverted of >=8 tested -> demote to floor; bounded to <=2000-poly
parts). nasty_cheese 243 -> 10 folds (watertight); unterlaf 3209 ->
1323 tris via rerouting; all fixtures + as1 green everywhere.
RESIDUALS on the examples board (cad profile): weldment 7 scattered
opens (2 nm edges in the INPUT B-rep — verify input dirt with the app
isolate next session); mohne ONE non-manifold edge (new, marginal —
attribute it: probably an absorber splice or strip corner; 0 opens);
2827056 3 folds, unterlaf 12, weldment 22, nasty_cheese 10 (all small
warped patches below the demotion threshold). Everything else clean:
bracket 31t, angle1 0t, as1_pe 0t, iso 136t/98% quads, 4pinplug.

## Relative-deviation round (latest — bores as light rings)
User report: bores still hyper-dense brick halves (iso14649 face 118,
r=508mm arcs solved at 80/half). Cause: ABSOLUTE chord deviation
(0.1mm) on huge parts makes the sagitta criterion dominate the angle
criterion. LANDED: adaptiveCount honors s.relativeDeviation — chord
scales by the edge's own extent (max of endpoint distance and
first-to-mid distance, so closed circles use their diameter and don't
collapse to the 1e-6 floor -> n=256). Cache key gains the flag. cad
profile + app new-session defaults now set relativeDeviation = true.
Results: iso bores 80 -> 7/half (angle-driven), board no regressions,
unterlaf folds 12 -> 0 (now fully clean), weldment folds 22 -> 2,
slotted cad-profile folds 36 -> 6 (A/B vs old profile confirmed
improvement; slotted --adaptive absolute mode still 36 on face #1 —
pre-existing). Prior round (adfb61e) already unites co-circular arc
groups so half-bores share one density group; together the two fixes
make holes read as single revolution rings.

## Revolve-unify round (bores become ONE face + wavy-rim lofts)
User: "still solving holes as halfs / faces 124+118 should be a single
closed circle / feature edges should resolve as one circle". Landed:
1. model.cpp: ShapeUpgrade_UnifySameDomain after ShapeFix — pass 1
   merges faces on the same PERIODIC surface (KeepShape on everything
   else), pass 2 unscoped edge-unify (ConcatBSplines) so co-circular
   rim arcs merge into single closed circles. iso: 164->143 faces,
   410->347 edges; bores are single periodic faces, rims single
   circles (visually verified).
2. rimChains(): rim membership by CONNECTIVITY (union-find over shared
   vertices of non-seam/non-insert border edges, chains named by mean
   v; single chain keeps its own side). Nearest-end tests misfile deep
   saddles. Used by the loft gate, finishRevolution (fills
   plan.rimLow/rimHigh, replaces uEdges), and meshRevolutionGrid
   (rimLowOpt param).
3. Wavy-rim loftability gate inside edgesHugRimsOrInserts (replaces
   revCovers in the auto path): 64 u-bins, low chain must stay below
   high chain per bin, each chain must be a FUNCTION of u (per-bin
   v-spread < 30% — rejects gear teeth/unterlaf), between-chain
   classifier coverage (skips insert boxes).
4. meshRevolutionGrid v-LOFT: RimPt carries v; chained rows lerp v per
   column between the rims' own v (missing rim -> band bound vFar).
   Pipe saddles (weldment faces 2/4/253) mesh as single lofted rings.
5. Density rim-SUM constraint (after curvature floor): closed band
   rims equalize by TOTAL, not per edge. linkRims unite now only for
   1-edge-vs-1-edge rims; a lone closed rim is raised THROUGH its
   group to the opposite chain's sum. Never spread a deficit across a
   multi-edge chain (shared saddle edges pump forever). Irreconcilable
   multi-multi rims: meshRevolutionGrid returns false -> contract
   floor (lune strips on thin fillet tori fold; floor is honest).
6. TRI-STATE fellBack (audit backlog item, LANDED): 2 = verified
   contract floor -> plan NOT demoted, conform treats it as authority
   (isFreeform false). Boolean fellBack let conform kidnap verified
   floor borders and tear web triangles open (nasty_cheese leaked 10
   opens through demoted bores). Cache stores the char.
7. counts[] resolution: max(solvedEdge, countFor) — solvedEdge carries
   floors and rim raises the group solve can't see (disk caps went
   stale-count and violated); countFor keeps seam fallbacks (sphere
   axial default).
8. Fold check reads polygon anchor UVs (period-unwrapped mean) instead
   of projecting the 3D centroid — on a 0.4mm-minor fillet torus the
   centroid projects onto the FAR side of the tube and false-flags.
   (The weldment tori strips were REAL folds though: lune-shaped
   cells; fixed by 5/6.)
BOARD (cad profile, after the origin-flatness fix below): NINE OF
TEN examples watertight — 2827056 folds 3->0 CLEAN, weldment
WATERTIGHT (was 7 opens/2 nm pre-session), iso 737 polys with light
single-ring bores, unterlaf/nasty at baseline, angle1 69q/12n.
Only mohne keeps 1 pre-existing nm edge (0 opens, 4 folds). as1
pair + bracket + all 11 fixtures x 3 modes green.
WEFT_EDGE_DEBUG=ids dumps solvedEdge.

RESOLVED (was: weldment micro-corner 19 opens): isGeometricallyFlat
seeded `diag` with p.XYZ().Modulus() — the distance from the WORLD
ORIGIN — so flatness was origin-dependent: a curved 1mm sliver 87mm
out measured against an 87mm yardstick and became an n-gon whose
chords tore off the neighbouring pipe walls. diag is now the face's
own diameter only. WELDMENT IS WATERTIGHT (0 opens, 0 nm, 4 folds vs
7/2/22 at session start). WEFT_FLAT_DEBUG=1 dumps flat decisions +
minimal admissions. Residual watch item: micro edges are still
exempt from the border contract check (unexercised now, but scale
the tolerance rather than exempt when it next bites).

## Twist + app round (latest)
1. PHASE ANCHOR (829c32a): closed circular edges sample from a world-
   anchored angle -> coaxial rings share column angles, grooved-shaft
   flats stop twisting. Guards: only circles whose seam vertex is free
   (revolve seams ignored; tangent-bore junction vertices pin). The
   contract check expects the phase; modelVertexEdges caches vertex
   adjacency per model (mutex, bounded).
2. APP (e096777): smoothing-angle vertex normals (Display toggle +
   angle, --smooth for screenshots); hover-scroll editing (no
   selection: shift/ctrl/ctrl+shift wheel edit the hovered face's
   override, empty space edits globals); knob-to-parts hover highlight
   (defaults controls tint the faces they drive); Model auto-collapse,
   Advanced groups Recipe/Debug/Dev fixtures.
QUEUE: mohne last nm edge — face 204 (one-closed-edge cylinder patch,
coons rejects "under 3 real edges" -> OCCT fallback) overlaps face 2's
insert web at (-26.4,-29.4,8.9)-(-26.8,-29.4,10.5); a one-loop-on-
curved-chart mesher (ring + web on surface) or floor-with-uv-rings
would fix it. Weldment residual folds: #4(2) deep-saddle loft cells,
#72/#311 singles.

## Exact-counts round + the notched-band gap (latest)
LANDED: "16 radial segments means exactly 16" — four silent rewrites
of typed numbers removed: (1) densityScale no longer rescales
overridden proposals; (2) nor STRAIGHT-line groups (cylinder axial
stays the user's number under the budget slider); (3) the curvature
floor skips face-pinned groups (DensitySolution.pinnedRoots, filled
from facePinned + perEdge pins); (4) the rim-sum raise skips pinned
rings (mismatch stays visible via strip/floor instead of silently
raising the user's count). App: typing radial/axial/grid u/v in a
per-face context flips that face's adaptive OFF (same rule as the
wheel) so the typed number is always the live number. Verified:
cylinder --face 1:radial=16 --density 2 -> exactly 16 wall quads;
default cylinder at 2x -> 32 radial x 1 axial.

OPEN (user priority): flaregun face 81 class — a cylinder band with a
SLOT CUT THROUGH A RIM (notch open to the border). Not u-coverable:
revolution rejects (notch walls plunge >30% of vspan in the loft
bins), coons rejects (seam walked twice / >16 edges), so it lands on
fallback-tri fans. User explicitly wants the two-step design:
revolution grid as the BASIS, cut regions handled after — i.e. extend
meshRevolutionInsert to RIM-OPEN notches: (a) plan: classify a
contiguous u-range where a rim chain plunges as plan.notch
{u0,u1,vBottom,chain edges}; (b) mesh: place a vRow at the notch
bottom, delete grid cells inside the notch box, web the OPEN
staircase to the notch chain sampled at solved counts (the web loop
closes over the deleted rim segment, so the rim row must stop at the
notch mouth). Repro fixture to add: solid cylinder minus a box
channel cut from the top rim to mid-height ("notched"). All the
validation machinery (contract check, fold check, floor) already
guards the attempt.

## Next steps, in order
1. COUNT DECOUPLING: extend the absorber to multi-vertex gaps (complement
   path v→w1→…→wk→u along a shared B-rep edge), then let 9–16-edge chained
   coons faces mesh WITHOUT forcing opposite-side/neighbour equality —
   sample sides at natural adaptive counts, absorber closes the seams.
   Then re-raise the coons cap (`more than 8 edges` reject in
   makeCoonsPatch) to 16. Gate: full board (tools/probes/probe27 on all 8
   models, both modes) must not regress multi/open; flaregun minimal
   profile should approach 96% quads without the poly explosion.
2. RULED STRIPS: cylinders/cones (and ruled bsplines) get 1 cell along the
   ruling. Slots/holes must be handled by CDT rim inserts, NOT by dropping
   full-height cells (a dropped cell kills the whole strip).
3. "CAD n-gon" PROFILE: preset = minimal + adaptive + strips; expose as a
   profile choice next to the density controls.
4. Minimal-profile seam debt: flaregun minimal was 5 multi / 18 open.
5. Leftovers: sphere face 197 coons rejection; ECH 50 opens include truly
   naked B-rep edges (edge 233 borders one face — model defect).

## Baseline board (defaults + quads, multi/open) — protect this
flaregun 3/14 & 5/18 · VentGasket 0/0 & 0/0 · NoHands_v2 0/0 & 0/0 ·
NoHands_v3 0/0 & 0/0 · NoHands_v3.2 0/0 & 0/0 · SawGuide 0/0 & 0/0 ·
DualHose3 0/0 & 0/0 · ExtendedClipHolder 0/50 & 0/50 (naked edges).

## Hard-won doctrine (violating these cost days)
- Border positions on shared edges are a contract; never re-space one side.
- Mesh-time demotion to fallback trades folds for seam leaks — plan-time
  routing only.
- The weld runs LATE in generate(); any topological seam pass must run
  post-weld or every border looks open.
- Sliver bands (flaregun 2mm trim band, ECH strips) defeat any
  proximity/tolerance matching — use exact topology instead.

## Session ledger — cutouts, bridge, weld, live floors
Landed (each pushed to main, in order):
- **Coons cutout** (3626f91): coons faces with strictly-interior trim
  wires (angled bore through a curved bspline top — hole_angled face 10)
  mesh whole, delete cells whose uv RECT overlaps a hole box (centroid
  tests keep ring-crossed cells → web self-intersects), and web the
  staircase to the hole's exact solved-count ring. All-or-nothing local
  assembly; planar faces with holes stay with quad-fill/plate-web.
- **Anisotropic scaffold + honest freeform edges** (5686fa1): scaffold
  lines are per-direction — natural fractions (shared with the border
  samples, so those columns run border to border) + snug hole BRACKETS;
  a flat direction never sprouts rows (user's yellow/red annotation).
  Relative deviation gates freeform curves at 0.5% of extent
  (lines/circles/ellipses keep 2% → rings stay angle-driven): a 780mm
  edge with 27mm sagitta and 16° total turn was shipping as ONE span.
  Cost: weldment folds 6→18 (same chained-strip class, denser rails).
- **Bridge** (81ec535): arc-fraction zipper (distance-greedy ran away on
  offset loops and fanned the remainder around one vertex — the user's
  hex-socket collapse); per-side twist (twist=B, twistA=A, counter-
  rotating, each side KEEPS its value across shift+wheel side flips).
- **Weld + grab** (581ccc6): M in vert mode → merge at center/last/first
  (WeldVerts op, world-point keyed, recipe round-trips); selVertOrder
  tracks pick order. G now works from bridge/loop-cut modes (it was
  Idle-gated — dead right after bridging) and explains anchorless verts.
- **Live floors** (8a6dbdb): contract-floor webs refine their interiors
  to the deflection budget (split interior edges whose surface midpoint
  sags > defl; Delaunay flips in uScale-corrected UV; minSize floors
  edge length; quadDominant pairs by quadAngleCost). Cap fan only when
  the face fits the budget as one sheet. Demote path honors the
  density-scaled tolerances. CLI gains --quads (docs claimed quad
  pairing was default; it never was).

Known residuals:
- quadDominant=true GLOBALLY reroutes planning (planQuadFill) and mohne
  face 204 quad-fill leaks 2 opens + nm edges at chord 0.05 —
  pre-existing, reachable from the app checkbox too. Fix quad-fill's
  rim web seam before defaulting --quads on.
- Fold class (chained strips): mohne 4 / nasty 10 / weldment 18 under
  profile cad. Angle tolerance still only affects OCCT-fallback faces,
  not floor refinement (deviation/minSize/quads do).

## Session ledger — crash batch, collar round 2, exports
- Crashes (49857ab): weld/right-click/shading-v were ONE bug — the weld
  popup's OpenPopup/BeginPopup ran in the pre-NewFrame input section
  (null CurrentWindow once any popup is open; gdb-verified). Popups now
  latch flags and draw in-frame (drawWeldPopup/drawExportPopup pattern —
  NEVER call popup APIs from the input section). Worker snapshots
  recipe.ops (genOps) like settings — live reads were a use-after-free.
  Ghost verts (DeletePoly leaves orphans) stole grabs: live-vertex masks
  in startVertexGrab + nudgeVertex. Undock: sources fuzz clean under
  ASan; the Windows crash is a stale mixed build/_deps —
  DebugCheckVersionAndDataLayout now fails loud; user must delete
  build\_deps once and rebuild.
- Collar round 2 (594f2ea): insert-coons plans propose hole-sized count
  floors (PHYSICAL mid-isoline measure, not uv — bspline params
  compress); junctionRings quad collar between bore ring and staircase;
  brackets only when a pin starves a direction.
- Barrel fixture (2f9008b): ¾-wrap wall + capsule slot = flaregun
  81/87 class → coons cutout, watertight. Dev fixtures panel lists all
  complex shapes. Flaregun face #206 NOT reproducible here (no model in
  the repo corpus) — needs the user's file or a screenshot-matched
  fixture next session.
- Features (8e08448): export dialog (OBJ/glb/FBX + triangulate/Y-up/
  scale, ctrl+E), NEW core binary FBX 7.4 writer (assimp-validated),
  CLI .fbx by extension, XYZ view gizmo (gViewMin/gViewMax-anchored),
  smooth shading default, red fold outlines, Model open at launch,
  vert size sliders, density scale typed entry to [0.05, 20].

## Session ledger — Plasticity-parity round 1
Reference bar extracted from the user's Plasticity export (probe scripts
in scratchpad): ~11°/segment circumferential pitch at ANY radius; ZERO
interior rows on straight ruled sections (47:1 aspect quads shipped);
rows only at feature z-extents / blend curvature; features absorb into
ONE adjacent column as n-gons (≤2% tris); columns end ON rim verts
(rim count == column count); planar faces = single giant n-gons.
Landed:
- Override collapse fix (c8e6ed7): kind-aware adaptive flip + boundary
  totals seeded from live solved outer-loop totals.
- Insert web folds (2f62dbe): hole boxes bound the web's own solved-
  count/phase samples + 4% v-pad → strict annulus webs. slotted 0
  folds, mohne 5→0, weldment 16→7.
- Quad-fill anisotropy (91fe2a3): per-direction iso-curve sizing under
  the face budget; ruled direction = 1 row; margins capped at border
  spacing; inset grid box; probe clamp. Flaregun 8699→5778 polys,
  barrel walls 1101→69 / 1059→77, zero folds.
In flight: OPEN revolution bands (partial-wrap cylinders with
castellated rims — flaregun barrel class) for true columns-to-rims
topology; worktree agent implementing plan.bandSides routing +
meshRevolutionGrid open-u mode. MERGE CAUTION: meshRevolutionInsert
was edited on main after the worktree branched (insert-box sampling).

## OPEN REVOLUTION BANDS (landed)
Partial-wrap cylinders/cones/revolves with two full-height u-iso side
edges plan as open revolution bands (FacePlan.bandSides): nu+1 columns
sampled in wire order at solved counts, first/last columns ON the side
edges' 3D curves, no phase/bestOff/seam-wrap, sides carry the row
contract via plan.vEdges, rim SUM equalization raises the plain rim to
the castellated chain's total. Flaregun barrel walls: 1101/1059 polys
-> 57/65 (58/66 cols x 1 row), columns end ON rim verts, watertight,
zero folds. Board's best-ever folds: mohne 0 / weldment 7 / nasty 10.
Quad-fill (per-direction sizing + coverage guard) is the safety tier
for curved faces the band/coons/cutout routes reject.
Known: --density 2 grazes 2 Newell-heuristic slivers per barrel wall
at tall notch walls (watertight, parametrically simple); column shear
near feature-clustered rim chains is inherent to uniform top-rim
contract sampling.

## STRAIGHT-LATTICE OPEN BANDS (landed d921c95 + 27d40f4)
The open band no longer lofts columns between index-paired rim samples
(feature-clustered castellated chains sheared every column diagonally
and the rim-SUM equalization silently overrode radial/axial). It is a
straight revolution lattice FOLLOWED by a boolean castellation cut
(meshRevolutionOpenBand): columns at fixed azimuths driven by the flat
full-span rim (FacePlan.bandDriver — its solved count IS nu, so
radial/adaptive/density/per-face pins flow through the border
contract; count mismatch takes a transition strip, never re-spacing);
notch runs delete lattice cells and web to the chains' exact contract
samples; wavy rims (unterlaf gear flanks) absorb into the strip row
when the wave fits <=35% of the band, else contract floor; everything
runs in w = |v - vCut| space so top-castellated bands mirror. Side
bands anchor on the columns' REAL keys with shared boundary anchors
(27d40f4) — boundary-column suppression is comb-length-gated, fixing
interior holes at coarse radial x mid axial.
Verified: flaregun 43/50 = 77/85 polys, max column-edge |dAz| = 0 on
all lattice quads/strip n-gons (web tris carry the only diagonals, by
design); 4x4x2 radial/axial matrix + axial 100 stress: watertight, 0
folds, straight; density 2x scales columns 13->25; board mohne 0 /
nasty 10 / weldment 7; 42/42 fixtures; ctest green.
NEXT (user, screenshot 27fb3b0a): columns must extend THROUGH fillets
(face 49 torus under 43's base) — blend strips need rail-count + arc-
phase propagation so barrel columns continue into the lower section;
diagnosis workflow ran (face graph / solver rails / coons pairing /
Plasticity reference).

## FULL-WRAP CASTELLATED RIMS (landed a44535c)
Straight-lattice generalized from partial-wrap open bands to the CLOSED
u-wrap case (the notched fixture / flaregun face-81 class: a full 360
cylinder/cone whose top rim is castellated by a rim-open notch). The
old meshRevolutionGrid chained loft index-paired the plain bottom rim
against the notched top and u-smoothed across columns -> 27deg shear
(default, after rim-SUM equalization forced 21/21); when a deviation
override left the rims irreconcilable (13/21) the grid bailed to the
contract floor -> triangle soup. New meshRevolutionRimNotch lays
straight uniform columns anchored on the plain rim's own sample
azimuths (exact rulings + exact neighbour weld), boolean-cuts the notch,
and webs the walls/floor to their solved-count samples via structured
strips (vertical wall ladders v-matched so every tall edge is a ruling,
plus a horizontal floor strip) - never an ear-clip fan. Wiring:
castellatedRimBand detector marks plan.castellated; rim-SUM
equalization (8779 region), counts/dispatch/cache-key all branch on it;
plain rim drives nu (natural count, no equalize). Detection gated to
one plain full-circle rim + one mostly-flat rim with a single localized
dip whose walls reach a real v-range - wavy pipe-saddle weld rims
(mohne's 12) and plain matching rims keep the chained loft.
Verified: notched face_1/face_7 max|dAz| 0.468/0.425 -> 0.000; override
sweep face{1,7} x chord{0.05,0.1151,0.3} x radial{8,16,32} x angle{14,28}
all watertight/0-fold/straight, no demote; knobs live (radial 8/32 ->
nu 8/34, axial adds rows); flaregun2 OBJ byte-identical (open bands
untouched, 0 castellated faces); board mohne 0/nasty 10/weldment 7 all
watertight; 42/42 fixtures; ctest green.
NEXT still open: fillet flow-through (#24, columns through blend chains
barrel->fillet->lower with matched counts+azimuths; diagnosis in
workflow wcfz9arp2, not yet implemented).

## NOTCHED REFINEMENT — uniform lattice + clean cut (landed 2acb541)
User feedback on the rim-notch result (red/green annotation): remove the
inset ring, align columns top-to-bottom, no wall ladders, no extra
columns, no right-angle corner breaks. North star (user): "the end
geometry is a perfect cylinder with a notch taken out — look at the
cylinder like it never had the notch cut for its samples." New pinned-
sample facility (PinnedEdges + edgeSampleFractions near phasedT): an
edge carries explicit forward-curve fractions that BOTH sharing faces
emit (watertight while samples move off uniform phase). pinCastellatedRims
(post density solve) pins each castellated rim's base arcs + notch floor
to the plain rim's column azimuths and the single-span walls to their
endpoints (corner guard 0.02*pitch, guards global wall azimuths so seam-
split arcs keep near-seam columns). Consumers: samplePlanarRings
(neighbour annulus/floor/wall) + the border-contract postcondition adopt
the pins. meshRevolutionRimNotch pinned branch: uniform full-cylinder
lattice at the plain rim's natural count, columns straight plain->cut
rim, notch boolean-cut (notch-range columns stop at floor), each corner
one cap n-gon. Result face_1/face_7: z-levels {0, 22(floor,notch only),
40} — NO z=38 inset; top az == bottom az (uniform) + 2 notch corners;
11 quads + 2 hex caps; max|dAz|=0.000; watertight. Override sweep 54/54
clean; board mohne 0/nasty 10/weldment 7; 42/42 fixtures; ctest green;
flaregun untouched (facility opt-in). Follow-up: the top C-shaped annulus
is a boundary n-gon on the column azimuths (columns meet rim 1:1) but has
no interior radial spokes (not annulus-ring-able as a single C-loop);
axial no longer adds interior wall rows (radial drives columns).
Phase 2 (fillet flow-through) resumed separately.

## FILLET FLOW-THROUGH (landed 2b0e8a9 + 2431abb)
Columns now continue through blend chains (barrel -> torus fillet ->
torus fillet -> lower cyl) instead of dying in the open band's bottom
transition strip. Two parts:
- Pentagon fix (2b0e8a9): a fillet coons strip's ACROSS side proposed
  filletLoops per PIECE, so a k-edge across chain summed to k*loops and
  mismatched the single-edge opposite side (3-vs-6 -> bridged with
  pentagons). Now the loop count is distributed over the across chain
  as a TOTAL. flaregun sectors 49/57/59/61: {4:12,5:3} -> {4:9} pure
  quads.
- Pinning core (2431abb): pinFilletChains walks the coaxial coons blend
  strips from each open band's fillet-side rim, rail to rail, to the
  next revolution face, pinning every cross-rail arc to the band's
  column azimuths (3D azimuth about the rev axis; arc endpoints stay the
  castellation corners). COUNT-MATCHED to each rail's solved count so
  nothing cascades; rails whose count diverges stay uniform (frame-cut/
  slot/wrap-gap correctly skipped). Consumers: coons sampleSide (single
  + chained) and open-band samplePieces emit the pinned fractions via
  the FORWARD-param mapping (no rev flip -- rev only reorders; the coons
  uniform path's 1-t flip would break asymmetric pins -- key subtlety).
Result (ring_check.py on flaregun2 OBJ): 3 junction rings each 8 clean
matched columns; TOP->MID 7/8 <=0.4deg (worst 0.47), MID->BOT 8/8
<=0.4; every ring edge 2-shared; watertight, 0 fold at 1x. The 0.47
residual is one column where bands 43/50 place a different interior
count in a shared near-endpoint arc (geometric, not solved-count --
rail-unite tried, zero effect, reverted). Density-2 flow deferred (count-
match guard skips diverging rails; watertight preserved, pre-existing
leaks on 21/29/312/315/317/318 only). Gates: ctest green; board mohne 0/
nasty 10/weldment 7; 42/42 fixtures; notched (Phase 1) unaffected.

## NOTCHED TOP RING — open annulus (landed 19ed8f0)
The notched top ring (face 3, a C-shaped planar annulus: the notch cuts
through the ring between outer wall and inner bore) meshed as minimal-
ngon (auto) or a fallback-tri FAN (forced annulus-ring "couldn't build"
- it needs two closed loops, this is one C-wire). New open-annulus path:
planAnnulusCRing detects a single planar wire whose circle edges fall on
two co-axial radii (inner+outer rail) joined by exactly two non-arc
walls, each rail spanning > pi (a ring cut by a notch, not a thin
sector - the span>pi gate also fixed a weldment misdetection: 6 sector
faces -> 2 folds, reverted to 7). meshAnnulusCRing samples each rail at
its PINNED column azimuths (the same edges faces 1/7 pin via 2acb541),
pairs the rails by arc fraction into radial quads, walls carry the
across-ring row count. FacePlan gained cRing + cWalls; wired into forced
AnnulusRing and the auto path (ahead of the minimal grab). Result face 3
(radial=13): 11 quads + 1 five-gon, 0 tris, every column of faces 1/7
meets a spoke 1:1 (outer 14 = 12 cols + 2 corners, inner 13 = 11 + 2),
watertight. Radial knob -> across-ring rows. Closed 2-loop annuli
(barrel/barrel2/bossfillet) unregressed. Gates: ctest green; board mohne
0/nasty 10/weldment 7; 42/42 fixtures.
NEXT: grey out mesher-dropdown options that can't build per face (needs
core per-face buildable-mesher exposure; must reflect this C-ring
capability). Deviation dead-zone on genuine fallback faces (OCCT
quantization plateaus) still open/lower-priority.

## WELD TOLERANCE — global + per-face (landed 1818e79/ea8d56a/42fca0d/0796df4)
User-facing weld tolerance, global (UI slider + recipe `weld <v>` line + CLI
`--weld MM`) and per-face (FaceMeshSettings::weldTolerance, 0=inherit; UI
field + recipe `weld=` key + `--face ID:weld=`). Implemented as a PER-VERTEX
weldVertices tolerance, NOT via the conform pass: folding it into conform's
capture radius (tolTarget/tolMoverPre) kidnaps verts from adjacent edges and
folds borders onto the wrong curve (the tight tolerances at ~9910 guard
exactly that) — reverted. weldVertices takes an optional per-vertex radius;
a pair merges within the LOOSER of the two (max-wins). generate() derives each
vertex's radius from the loosest per-face override on any incident polygon
(via polygonFaceId) + global, CLAMPED to half the shortest incident mesh edge
(local resolution, so only genuine near-dups merge). The border-contract
oracle (~10598) is decoupled from the knob (it plans faces; must not shift on
a weld-only change). Bit-identical at default (0/22 board OBJs differ; the
per-vertex path is skipped when global==1e-6 and no override). ctest (+new
testWeldTolerance) green; board mohne 0/nasty 10/weldment 7; 42/42 fixtures.
Max-wins proven by unit test (0.01mm-gap seam closes when EITHER face
loosened). Honest limits: the repo board is already ~0-weld clean (border
contract welds by construction) so global reductions are small; the real
payoff is sloppy imports. An absurd value (e.g. --weld 0.5 on a small part)
still leaves a few non-manifold edges (graceful degradation bounded by the
clamp, not soup) — opt-in, default-safe.

## MISMATCHED REVOLUTION RIMS -> transition strip (landed ee67b7d)
meshRevolutionGrid bailed to the contract floor (tri soup) whenever a closed
band's two rims solved to different totals (the irreconcilable check fired
before the existing non-chained interior + emitClosedStrip path). Now an
analytic revolution surface (cyl/cone/torus) with usable-but-mismatched rims
routes into the strip: interior count = the DENSER rim's count (it welds 1:1;
only the sparser rim takes a strip); interior azimuths at the dense rim's own
samples for a clustered monotone rim (saddle) or uniform rulings for a near-
uniform scalloped rim (detected by azimuth backsteps); interior v lofts
between the two rims' v(u) profiles (no constant-v ring crosses a wavy rim).
emitClosedStrip rewritten to match by CUMULATIVE ANGLE along each ring's own
order (not raw wrapped-u) so a wire-chain-ordered rim pairs correctly — strict
generalization, identical for sorted input. Thinness+flat guard keeps the bail
where it mattered: reconcile only if bandH >= 0.35*reach AND the sparser rim
is flat (sparseVr <= 0.02*bandH) -> two wavy rims stay irreconcilable
(nasty_cheese drilled bores keep the floor, no regression). foam --profile cad:
face 4 (cone) 376{158q,218t} -> 534{512q,22n} 96% quad; 183 91%; 184 85%; 0
folds. foam's 82 open / 2 non-manifold are PRE-EXISTING open-shell B-rep (source
isn't a closed solid there) — unchanged before/after, the demoted faces never
contributed crack-opens; the win is quality. ctest green; board mohne 0/nasty
10/weldment 7; 42/42 fixtures; flaregun barrels + notched byte-identical/unaffected.
Foam dome-cap (bspline petals) + remaining coons rejects still open.

## RIBBON-SWEEP MESHER (landed 46499c5 + 37c9720)
New mesher for long CURVED bspline "ribbon" strips (flaregun grip/guard) that
coons rejects (non-convex bent domains fold) and quad-fill only reaches ~60%
quad. ribbonDetect (planFace branch, only on faces quad-fill would take):
findRibbonRails brute-forces every 4-corner wire split, picks the two long
ANTI-PARALLEL constant-width rails (tangent alignment >0.5, width ratio <2.6,
non-pinching caps, aspect >3.5) — finds rails where RailLadder's sharpest-
corner pick fails (93's 90deg bends, 131's weak 22deg notch corners).
meshRibbonSweep: both rails sampled at solved edge counts (border contract),
arc-length-ZIPPED station-by-station into an even quad ladder (advance the
lagging rail so a Z-crease never twists), winding from the 3D surface normal
(freeform UV area unreliable), end caps a quad rung or locally webbed
(triangulateWeb) when notched — never a global fan. Tight gating + safe
fallback: any doubt (unequal rails, web-heavy cap tris>quads, failed web)
hands the face back to quad-fill+pairing (watertight). New fixtures: `ribbon`
(bent bspline strip, 4-sided -> coons owns it 100% quad, proves gating) and
`ribbonnotch` (slotted end -> routes to ribbon-sweep). face 93: 60% -> 100%
quad (26-poly even ladder); face 131 byte-identical safe fallback (its deep
notched U-cap ~224mm rivals the 237mm rails -> no even rail-to-rail sweep, one
end wants 7 across-cells the other 56). Gates: ctest; board mohne 0/nasty
10/weldment 7; 48/48 fixtures (16 shapes x3); flaregun 1/3/7/43/50 + all non-
93/131 faces byte-identical; watertight, 0 fold. KNOWN: 1 zero-area collinear
quad on face 93 at the 90deg Z-crease (cross-rung has no width there; watertight,
consistent winding, 1 of 5721; force-split would make 2 zero-area tris) — a
crease singularity, flagged by validate as 1 degenerate. Candidate follow-up:
skip the zero-width rung and merge the crease into one cell.

## RIBBON CAP/CREASE ZERO-AREA FIX (landed 65365ff)
Both the ribbonnotch notch-end FAN and face 93's 1 degenerate quad were the
SAME bug: findRibbonRails swallowed short ~90deg cap-corner edges into the
rails, leaving a collinear zero-area cell (4 points at one y,z; edge parallel
to the rung) that no split/triangulation rescues. Fix: (1) findRibbonRails
rejects any 4-corner split whose cap-abutting cells collapse to ~zero area
(forces the honest split, corner stays in the cap); (2) where the strip edge
genuinely runs parallel to the rung, the dead cell is spliced into the
previous polygon across their shared rung = one valid n-gon, not a zero-area
quad or a fan. flaregun face 93: degenerate 1->0, 25 polys (24q+1hex) 96%
quad; flaregun overall 0 degenerate. ribbonnotch: notch end clean quads
around the slot (no fan/pinch/zero-area), watertight, 0 degenerate all 3
modes. ctest; board mohne 0/nasty 10/weldment 7; 48/48 fixtures; face 131 +
all non-93 flaregun faces byte-identical.

## FOAM BODY HARDENING (landed c24b7ec + bc40ab8)
Two foam robustness fixes.
- Radial-override degradation (c24b7ec): body face 183 (flat bottom rim,
  WAVY top rim ~46u v-jump near the head "mess") was 90% quad at default but
  61%/57% under radial=20/32,adapt=0. The ee67b7d reconciliation (a) bailed to
  the floor when the sparser-by-count rim wandered in v (under a manual radial
  the flat rim out-counts the ~32-sample wavy rim), and (b) always drove the
  interior from the DENSER rim, so the strip landed on the wavy rim and frayed
  into folds. Fix: the WAVIER rim drives interior azimuths (welds 1:1 over its
  v-jumps, no strip -> no folds), the FLATTER rim strips; reconcile only when a
  genuinely flat rim exists (min rim v-range <= tol) so both-wavy bands
  (nasty_cheese saddles) still floor; when the drive rim has a steep local
  v-jump (>0.25*bandH) a sparse manual radial can't spread, give the tall band
  height-proportional interior rows (aspect ~2:1, cap 8) gated on the JUMP not
  the count ratio (face 4's 278-sample gradual rim + default body untouched).
  face 183: radial 12/16/20/32 -> 92/100/86/85% quad (was .../61/57), straight
  columns, 0 fold.
- Shallow conical caps -> n-gon (bc40ab8): spray-can disk faces 325/366 are
  shallow cones (3% dish, r~5) that isGeometricallyFlat's 0.1% tol rejects, so
  they tri-fanned (~150 poly, ~50% tri). New isShallowCapCone gate (cone, dish
  <10% width, single round hole-free boundary, elongation <1.6) routes them to
  the boundary minimal n-gon -> 1 n-gon each. face 512 (concave cylinder fillet
  needing interior topology) correctly LEFT on the floor (not n-gon-able).
Gates: ctest; board mohne 0/nasty 10/weldment 7 BYTE-IDENTICAL (isShallowCapCone
catches 0 board faces, revolution fix touches nothing there); 48/48 fixtures;
foam default byte-identical except 325/366; flaregun byte-identical. foam opens
82->82 at default (pre-existing source open-shells; 0 new cracks). Remaining
foam: 512 concave fillet + a few cylinder quad-fill/coons-reject faces (49-60%
tri, watertight) still open.
