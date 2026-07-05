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
