# Weft — MVP Plan

> **RADIUS-SCALED DENSITY LAW (artist request 2026-07-11, landed).**
> The artist reported the demo reading INVERTED: the r=14 barrel at 13
> radial segments while smaller booleaned struts sat at 29-32, plus
> coons fillets carrying across-band divisions they shouldn't have —
> and asked that size-scaling carry to fillets too. Root cause was the
> relative-deviation basis in solveDensity's `adaptiveCount`
> (meshers.cpp): chord budget = fraction of the EDGE'S OWN extent
> (0.2 analytic / 0.05 freeform), which (a) made every ring
> angle-driven at 13 regardless of radius BY DESIGN, and (b) pushed
> fillet-owned bspline rims through the 4x tighter freeform gate, then
> dragged neighbouring primitive rings up through density groups (a
> plain r=5 cap circle solved at 32). THE NEW LAW: under
> relativeDeviation the chord budget is relative to the MODEL —
> `chord = chordTolerance * 0.01 * bboxDiag` (0.1 default = sag <=
> 0.1% of the diagonal) — so counts grow as sqrt(radius); the
> 28-degree default angle no longer floors rings at 13 (relaxed to a
> 60-degree kink guard; an angle the user TIGHTENS below the default
> is honored). Closed curved loops take a closed-form count on the
> UNROLLED radius len/2pi rather than per-interval sampling, so a
> tilted strut's blend rim (downhill side bends 2-3x tighter) does not
> get driven as dense as a barrel twice its size. The old
> primitiveDriven boolean-cut classifier died with the extent basis.
> Demo hierarchy now: barrel r14=22 > channel r11=19 > strut
> collars=18-19 > r6=14-15 > r5 bores=13 > micro=floor; fillet across
> counts: skirt seam 9->5, 30-deg seam 5->2, micro-edge 4->2, small
> rounds floor at filletLoops(3). Whole corpus moved at cad (leaner
> nearly everywhere: flaregun 9295q/176t -> 5821q/59t, teleporter
> halved, tork 998t -> 447t; weldment n-gons +365, the sloppy
> assembly absorbing coarser rims); default profile BYTE-IDENTICAL
> everywhere (non-relative path untouched). Gate PASS after --update,
> ctest PASS, visually verified on demo (hierarchy + skirt lattices +
> plate rounds), flaregun (grip/trigger flow), foam (patchwork barrel
> is the KNOWN co-axial task, unchanged). NOTE: quad-fill's interior
> isoCount (meshers.cpp ~6968) still uses the old 0.2*extent basis —
> align it with the model-relative law when quad-fill interiors next
> get attention. BRepBndLib box query is lazy (first relative-mode
> proposal) because warming OCCT's triangulation cache changes
> fallback output.

> **Stitch campaign update (2026-07-10, late session).** The decoupled
> seams path is nearly watertight corpus-wide. Scoreboard at cad
> `--stitch`: **11/14 watertight 0/0** (flaregun, foam, mohne,
> 1797609in, unterlaf, iso14649, 2827056, 4pinplug, angle1, as1_pe +
> fixtures); nasty_cheese 11o/0nm, teleporter 20o/0nm, weldment
> 37o/3nm; tork exempt. Started from flaregun 6/10, foam 43/16,
> nasty 1615, teleporter 132. Second-round fixes: pure-lattice bails
> on slit/notch rim chains (iso-azimuth runs are not rim material),
> full-edge chords bypass the midpoint test (a minimal plate's
> half-circle chord has 50% sagitta; terminal-to-terminal is
> unambiguous), conform under stitch is COMPLETENESS-gated (tight
> targets vs a loose pitch-scaled recount — mohne's 11-vert hole ring
> was collapsing onto 2 corner targets; 1797609in's legitimate
> freeform-onto-analytic decimation still runs), stitch vertex band
> 25% -> 35% of pitch (coons rails drift past the sagitta model).
> Remaining classes: micro-edge corner rings (nasty edge 699: a
> 0.1-long chamfer edge sampled 3 segments by one side, 0 by the
> other), and borders emitted 0.2-1.1 off-curve where the OPEN seam
> is on the vert's home curve, not the misattributed nearest one
> (teleporter faces 300/301/304/308/568, nasty face 125 vs edge 622
> — probe86's nearest-edge column misleads there; trace the home
> curve's own edge id instead). Default path byte-stable throughout
> (corpus gate PASS on every commit). Landed, each root-caused on a
> reproduced defect (3 commits):
>
> - **stitchSeams hardening**: pitch-scaled on-curve tolerances (25% of
>   the vertex's own boundary pitch, absolute cap 1% of model diagonal
>   on vertex acceptance only), HOME attribution (a vertex may only
>   join a seam chain for (nearly) the nearest of its face's own
>   curves), two topological guards (never splice into a segment both
>   faces traverse; never insert a vertex a face already has), closed-
>   curve wrap fix (the chord midpoint's param picks the true arc),
>   end-gated corner slack (the pitch floor only near an open curve's
>   ends), insertion bookkeeping (spliced verts join the side set).
> - **fuseSeamTwins** (new pass, runs before stitchSeams): two faces
>   sampling a shared edge at the SAME params can emit distinct verts a
>   few % of a pitch apart — past the weld, nothing to splice. Twins
>   (cross-side, mutually nearest in param, within 15% pitch in param /
>   25% in 3D) merge by union-find. Pitch scale = SHORTEST incident
>   segment; home-gated like the stitcher (both learned from hairline
>   strips whose rails otherwise fuse).
> - **Pure uniform lattices** (artist demand #1, full-wrap bands):
>   under stitch, meshRevolutionGrid with mismatched rims no longer
>   bails or builds transition strips — it emits a pure nu x nv lattice
>   at its own solved count; rim rows = rim chains resampled at the
>   column azimuths PLUS every B-rep edge junction (corners are
>   contract points; they also keep every rim segment within ONE edge,
>   per-edge stitchable). Non-column corners ride boundary cells as
>   extra vertices.
> - **Failure floors**: meshRevolutionGrid's return is now checked
>   (was silently ignored — irreconcilable bands shipped as HOLES),
>   and under stitch an empty part still demotes to the contract floor
>   (nasty shipped 45 drill walls as holes = 1615 opens).
>
> **Remaining residual classes (diagnosed, not yet fixed):**
> 1. Borders genuinely 0.2–1.5 off their B-rep curves — decimated /
>    conform-moved borders from the emitting meshers (nasty faces
>    6/8/44 against e.g. edge 46: endpoint 1.53 off a 6.2 curve;
>    teleporter faces 300/301/304 up to 1.1 off). Fix the emitters,
>    not the stitcher tolerances.
> 2. weldment 67o/3nm — assembly; partially the same class, needs the
>    per-face breakdown (probe86).
> 3. Multi-edge-spanning chords on coarse borders (flaregun ribbon
>    face 55 class was fixed for revolution rims via corner emission;
>    ribbons/coons rails may still span corners elsewhere).
>
> **Tooling for the next session:** WEFT_NO_STITCH / WEFT_NO_FUSE
> kill-switches; WEFT_STITCH_DEBUG=1 logs every insertion;
> WEFT_STITCH_EID=<eid> traces per-vertex accept/reject with reasons
> for one edge; probes 86 (defect classifier: every open/nm edge with
> owners, nearest curve, distances), 87 (face/edge context), 88
> (per-face build state), 89 (build census stitch vs default);
> tools/probes/relink86.sh — RELINK AFTER EVERY CORE REBUILD, static
> probes lie. Visual check in the loop: `xvfb-run -a
> ./build/app/weft_app <f> --stitch --screenshot out.png` (app binary
> also needs rebuilding — it statically links core).
>
> **Still open for the stitch flip:** the open revolution band's
> transition strips / rim hug rows under stitch (partial wraps);
> ribbon/rail meshers at their own counts (verify none still demote
> over rail disagreements); then the remaining residuals above, full
> corpus + sweep + golden/visual passes, and flip stitch to default.

> **Fixture + detection round (2026-07-11).** Six isolated reproducers
> now live in `weft fixture` AND as committed STEP under
> `tests/fixtures/`: hairline (twin-rail), canrev (offset-of-revolution
> can — demand #2 acceptance), slitdrill (tangent-contact slit, a
> KNOWN-RED reproducer: 2 nm at cad on the default path, gate-exempt
> like tork), microedge (0.12 chamfer edge), filletslot (demand #1b
> acceptance), torture (the demo scene — all classes in one solid;
> watertight on both default profiles, 60 opens under --stitch = the
> standing stitch testbed). All are in the corpus gate.
> Demand #2 first half LANDED: isGeometricClosedRevolution (probe-grid
> radius/height-about-fitted-axis test, memoized as
> GenerationCache::geomRevolution) routes offset/bspline closed
> revolves to revolution grids — canrev's offset wall is a pure
> 264-quad lattice now (was coons). The fold census learned the same
> lesson (offset surfaces lie about IsUPeriodic; the plan's u-range is
> the period). SECOND HALF STILL OPEN: foam's body is ~130 PARTIAL
> [0,1]x[0,1] bspline patches — segments of one revolution; per-face
> closure can never fire. They need co-axial patch GROUPING into one
> shared column flow (detect co-axial revolution segments at analysis,
> solve one shared column set across the group). probe92 dumps the
> per-face adaptor closure/type census.

> **Co-axial drums round (2026-07-11).** The REAL foam patchwork was
> found and measured: probe94 ranks coons faces by area, and foam's top
> offenders are typed-CYLINDER half/quarter drums (512/513 =
> u=[pi/2,pi]+[pi,3pi/2] of one drum, 456/458 = two halves of another)
> — co-axial segments split at meridians by booleans. They never reach
> the open band because (a) coons runs first and can express any
> 4-sided patch, and (b) the blend detector flags them isFillet (they
> join tangentially). The fix — open-band-first for partial-wrap
> revolution walls subtending >= 1 rad, with fillet-STRIP narrowness
> (vSpan <= 1.8*radius) replacing the raw isFillet flag — is LANDED
> BEHIND `WEFT_DRUM_BANDS=1` (default off): it reroutes 111 foam faces
> to columns (13737 -> 10530 polys, watertight) but perturbs shared
> counts so body face 183's revolution grid folds ONE cell and
> self-heals to the floor web — a visible regression on the biggest
> face. NEXT: root-cause that fold (RECDIAG on face 183, compare rim
> chains with/without the env), then flip the gate on with full
> corpus + visual passes. Cross-patch column alignment mostly comes
> free from the border contract (shared rims sample identically).
> NOTE the stale premise: today's foam has NO offset surfaces — the
> body is typed cylinders (254 of them) + 130 genuinely-freeform
> spray-head bsplines (probe93: ring-test deviations 5%, not
> revolution segments). isGeometricClosedRevolution still covers
> re-imports that DO arrive as offsets (canrev).

> **DONE (2026-07-11, same day): the composition below LANDED** —
> meshRevolutionRimNotch's pinned path takes optional interior row
> levels, meshRevolutionInsert builds its castellated base grid
> through it, and torture faces 34/45 are columned lattices (35 polys
> each, notch caps + slot collar, verified visually). Only torture's
> goldens moved; corpus byte-identical, gate PASS. Level margin must
> stay at the insert's 1% rim-clearance (2% silently dropped a slot
> row 1.5% above the rim and re-opened the carve's staircase).
> Historical context follows:
>
> **TOP PRIORITY (artist-flagged on the torture demo, 2026-07-11):
> notched cylinders must be revolved cylinders.** torture faces 34
> (muzzle outer wall) and 45 (bore wall) carry BOTH a rim-open notch
> (the channel through the top rim) and an interior capsule-slot wire.
> Each mechanism works alone; together they GUARANTEE the floor web —
> the patchwork visible on the demo:
> - the castellated route (meshRevolutionRimNotch) cannot emit interior
>   wires -> border contract fails on the slot's side lines (e91/e106);
> - the insert route (meshRevolutionInsert, now tried first for
>   castellated+insert walls — dispatch reorder landed, neutral since
>   both paths floored) builds its base grid through the notched-rim
>   reconcile (13 plain vs 29 notched samples -> transition strip) and
>   then fails: face 34's staircase bails, face 45 breaks contract on a
>   top-rim arc (e96).
> THE FIX (design): compose the two proven mechanisms — build the
> RIM-NOTCH lattice first (pinned columns, notch boolean-cut, the
> artist-approved topology), then run the insert carve on that lattice
> (rows at each slot band's v-extents, covered cells deleted, cavity
> laddered to the wire's contract samples — the meshRevolutionInsert
> core, generalized to take an existing lattice instead of building
> its own). Acceptance: torture faces 34/45 as clean columned walls,
> notch as local rim n-gons, slot as a local collar; flaregun
> unchanged; corpus gate + visual passes.

> **Chained-strip skew: the REAL design (artist verdict 2026-07-11).**
> The rail-sweep takeover for chained coons strips is REVERTED to
> opt-in (WEFT_CHAIN_SWEEP=1): arc-length re-pairing straightens rungs
> but breaks STATION CONTINUITY — the neighbouring fillet strips'
> loops used to continue across the band through the coons lattice's
> station-k-to-station-k rungs, and under the sweep they dead-end into
> absorption triangles. Flow beats perpendicularity. The real fix is
> in the DENSITY SOLVE, and it is the FilletBand prerequisite:
> **align the two rails' station arc-fractions** — distribute each
> chained rail's per-piece counts proportionally to arc length (and
> nudge piece boundaries into agreement across the strip) so matching
> stations sit at matching fractions; rungs are then straight AND
> continuous with zero mesher changes. Constraints: per-edge counts
> live in density groups (neighbours follow — fillet strips' across
> pairing keeps them consistent), integer rounding means alignment is
> approximate (largest-remainder), and the redistribution must be
> gated to strip-confined groups so it cannot ripple a whole model.

> **Demo skirt "dissolving" (artist report 2026-07-11): FIXED — the
> ring lattice landed.** Diagnosis first: the two tilted-strut blend
> skirts are v-CLOSED bspline rings (probe92/probe98: face 6
> u=[0,36.4] vClosed=1, face 18 u=[0,40.7] vClosed=1; both charts
> wildly non-arclength — u range 36-41 params over a 3-8.4 profile).
> The 30-deg skirt's coons built; the 40-deg skirt's coons FOLDED 14
> cells and the self-heal correctly kept the contract floor — the
> 'dissolving' collar WAS the floor web. Rotation ruled out
> (probe97: rotate 0-3 all floor). The fix is `meshRingLattice`
> (meshers.cpp, after meshParametricGrid): for a CoonsGrid face whose
> chart is closed in v only, bounded by exactly one seam (used twice)
> + two closed single-edge rims at EQUAL solved counts (the density
> solve's rail alignment delivers this), emit profile rows x ring
> stations evaluated DIRECTLY on the surface — rim rows are the rims'
> exact contract fractions (edgeSampleFractions + closedEdgePhase +
> pins), interior stations lerp v wrap-shortest between least-twist
> ROTATION-paired rim stations (min total wrapped v distance —
> per-station, what coonsRotate can't express), row spacing from the
> seam's arc-even solved samples so the non-arclength chart can't
> cluster rows. The ring emits CLOSED (no seam column twins; seams
> are contract-exempt). Transactional at the CoonsGrid dispatch:
> kept only when borderContractViolation==0 AND a periodic-unwrap
> Newell-vs-CAD fold census is 0, else the historic coons runs
> byte-identically. Result: demo/torture cad 887q/116t ->
> 1040q/76t/36n (the floor fans became quads), watertight both
> profiles and under --stitch, BOTH skirts verified visually as
> clean concentric-loop collars (probe98 build: face 18 2 -> 0).
> Only demo/torture goldens moved; foam face 495 also takes the
> lattice with identical counts; corpus gate PASS, ctest PASS.
> This IS the FilletBand closed-ring case (artist demand 1b, first
> half); the open-band FilletBand (filletslot fixture) remains. A
> u-closed transpose (blend rings OCCT charted the other way) is a
> cheap follow-up if a model surfaces one.
> NOTE for the stitch campaign (task 2): foam at cad --stitch now
> shows 4 open edges (faces 514/558/628 near edges 1464/1551) — A/B
> confirmed PRE-EXISTING before the ring lattice (rail-alignment-era
> drift; it was 0/0 in the 2026-07-10 scoreboard); nasty 12o/2nm and
> teleporter 14o likewise identical pre/post. Re-diagnose with
> probe86 when the campaign resumes.

> **NEXT SESSION — the artist's two standing demands (2026-07-10):**
>
> 1. **NO ABSORBER BANDS, ANYWHERE.** Stated five times. The decoupled-
>    seams experiment (`--stitch`, `decoupleSeams`) killed the global
>    equalizers, but the PER-FACE absorber machinery still exists: the
>    open band's transition strips / rim hug rows, and meshers that
>    demote when rails disagree (ribbon-sweep fell to the floor under
>    stitch). The work: under stitch, every revolution band emits a PURE
>    uniform lattice (rim chains sampled at the lattice's own column
>    positions), strips/hug rows deleted, border contract relaxed for
>    those rims, and unionSeams closes the seams. Ribbon/rail meshers
>    likewise build at their own counts instead of demoting. Then flip
>    stitch to default with full corpus + sweep + golden/visual passes.
> 1b. **Dedicated fillet mesher (artist request, with screenshot).**
>    Simple constant-radius fillets (slot-end quarter-rounds) mesh as
>    coons grids with adaptive-driven loop spam — the strip pitch floor
>    plus adaptive rail counts put 2-3x more rungs on a fillet than the
>    wall columns it welds into (turning adaptive off removes them:
>    both mechanisms are adaptive-gated). Build a FilletBand mesher:
>    across = filletLoops profile arcs, along = INHERIT the shared
>    rail's solved count so rungs land vertex-for-vertex on the
>    neighbour walls by construction. Route constant-radius blend strips
>    (analysis.isFillet + cylinder/torus surface) there before coons;
>    also exempt fillet strips from the pitch floor (their pitch is the
>    neighbour's, not width-derived).
> 2. **foam's can body must mesh as a revolution cylinder.** The body is
>    an OFFSET_SURFACE bspline — geometrically a surface of revolution,
>    typed freeform, so it takes coons patchwork (411 coons faces on
>    foam) instead of columns. Detect bspline surfaces of revolution
>    (constant radius about a fitted axis, probe-grid test, memoized in
>    GenerationCache like revolutionCovers) and route them through the
>    revolution machinery. Primary acceptance models: flaregun and foam,
>    judged in Blender, not the viewer.
>
> **Stitch progress this session:** the per-face absorber strips are
> GONE under `--stitch` (coons deficit rails emit the lattice's own
> resampled border, no transition strips; border-contract postcondition
> relaxed so nothing demotes over a seam mismatch), and a curve-guided
> stitcher (`stitchSeams`) closes seams by parameter-sorted vertex
> insertion on every 2-owner B-rep edge. Residual at cad: flaregun 6
> open / 10 non-manifold (was 742/1400 with the naive splice), foam
> 43/16. NEXT: diagnose the residuals (suspects: 3+-owner edges are
> skipped, freeform off-curve borders beyond the 0.4%-of-curve
> tolerance, seams whose faces share BOTH endpoints only), then the
> foam revolution detection. Default path byte-stable throughout.
>
> Stitch experiment numbers (cad, watertight, 0 folds): flaregun
> 7995q/50t/182n vs default 8530q/176t/154n — the absorber tris are the
> difference; foam/teleporter/nasty trade n-gons at seams; fixture count
> sweeps clean with equalizers OFF. Toggles: CLI `--stitch`, app Debug >
> decoupled seams, app `--stitch` for screenshots.

> **Handoff (2026-07-10, self-heal session).** Banding, the ChatGPT
> hardening pass, and "meshes can't fix themselves":
>
> - **ChatGPT pass verified** (`87d45bc`): the transactional
>   chained-coons repair is sound (fold claims reproduce on Linux) and
>   is kept. Two fixes on top (`0b46b8c`): writeStep's ShapeProcess pin
>   is OCCT>=7.8-only — version-guarded, the Linux 7.6 build was broken;
>   the gate's never-fall-back census was dropped rather than exempted —
>   restored with the tork (wt=no) exemption.
> - **Self-heal tournament** (`78a8745`, `8a4e710`): any face whose
>   build has inverted cells now competes against a contract-floor
>   candidate at the SAME exact borders; the better part wins, so a swap
>   can never regress. Corpus folds at cad: nasty_cheese 11→0,
>   teleporter 1→0, weldment 14→1, tork 16→2, all else 0, everything
>   watertight. The census unwraps periodic u AND v (a torus's v-seam
>   read as folded and got needlessly healed — caught by ctest).
> - **Banding solved as shading** (`e889118`): the barrel's "banding"
>   that came and went with segment counts was tessellation-dependent
>   *viewer* shading. The viewport now shades every corner with the
>   exact CAD surface normal (anchors' u,v; per-mesh cache) — the MoI
>   lesson, and the same normals the exporters already write. Display >
>   CAD-exact normals toggles it.
> - **MoI research note:** MoI meshes the natural UV grid with n-gons at
>   trim boundaries and ships CAD vertex normals — architecturally what
>   Weft already does, minus Weft's cross-face density matching (MoI
>   accepts cracks; Weft welds). No architecture change warranted; the
>   normals lesson is applied.
> - **Goldens refreshed, gate PASS**, pipeline ctest green. Open items:
>   weldment's 1 fold + tork's 2 (below-floor candidates also fold
>   there); nasty_cheese trades 2763 tris for its healed folds — a
>   denser-candidate tournament (quad-fill / re-solved counts) is the
>   next quality lever; full `weft sweep` acceptance not re-run since
>   the tournament landed.

> **Handoff (2026-07-10, fillet-flow session).** In response to the
> artist's fillet-flow report (uneven/missing cross rungs along fillet
> strips, diagonal zigzag transitions, the trigger-slot strip reading as
> uncut slats):
>
> - **Landed (3 commits, `ec8b9f2`/`dd63460`):** a *strip pitch floor*
>   in the density solve — strip-planned faces (coons strips, ribbon
>   sweeps, rail ladders, aspect ≥ 3) floor every outline edge to one
>   station per 2 strip-widths (w = 2A/L), so rungs march evenly by arc
>   length across large regions; adaptive faces only, so the flat-count
>   `default` profile is byte-identical. Hardened by: fragile-rim skip
>   (open-band/castellated rims can't take raised straight edges), a
>   hairline gauge (strips < 0.1% of model diagonal are seam shims —
>   skipped), a 24-station cap per edge, and a **chained-coons SUM
>   repair** fixpoint — post-solve floors were breaking the chain-pass
>   sum equality and skewing patches into diagonal absorption + folds
>   (this also fixed 8 pre-existing teleporter folds; it is the
>   "rail-chain sum equalization" lever, landed).
> - **Verified:** flaregun cad watertight, 0 demotions, 0 folds (was 1);
>   guard ribbon/fillet chains/slot lip show even rungs (matches the
>   artist's blue-tick drawings); teleporter 9→1 folds; unterlaf 0
>   folds; visual passes on foam/teleporter/unterlaf/mohne renders.
>   Note: the reported face #326 "cutout" has NO interior wire — the
>   pinch was the old build's 2-station slats; routes to coons with even
>   rungs now.
> - **Loose ends for the next session:**
>   1. `tools/golden_counts.txt` NOT yet refreshed — cad rows moved on
>      ~11 models (defaults all byte-identical), so the corpus gate and
>      CI are red on the branch tip until `tools/corpus_gate.sh
>      --update` is run after the remaining checks.
>   2. Fold spot-checks outstanding: nasty_cheese 10→11 (+1) and tork
>      10→16 (+6, broken source, gate-exempt) at cad — diagnose or
>      accept before the golden refresh. Re-run the full fold table
>      (`for f in tests/STEP_Examples/*.stp; ... --validate | grep
>      folded`) against the chain-sum-repair build, since the +1/+6 were
>      measured BEFORE the repair landed and may have changed.
>   3. Full corpus gate + `weft sweep` acceptance not re-run since the
>      chain-sum repair; pipeline ctest suite not re-run this session.
>   4. Visual verify the remaining movers at cad (angle1, iso14649-demo,
>      4pinplug, 2827056, nasty_cheese) per the corpus rule.
>   5. Pitch constant is 2.0 widths (1.5 tripped a neighbour band's
>      contract); artist may want per-family tuning via the new app tabs.

> **Progress (2026-07-09 session).** Phase 0 is done and most of Phase 1:
>
> - **P0.2 ✅ Universal watertightness.** foam was leaking through 4
>   OFFSET_SURFACE faces OCCT drops at translation (881 declared, 877
>   transferred) — the importer now caps the resulting boundary loops of
>   nearly-closed shells with real B-rep faces and sews them in (authored
>   sheet bodies like tork are exempt by their open-edge fraction). Its
>   2 non-manifold edges were a zero-width slit (de-slit pass splits the
>   doubled traversal) and a twin-edge lens (micro nm-segment collapse).
>   Every corpus model is watertight 0/0 at both profiles; tork (broken
>   source, artist's verdict) still meshes without crashing.
> - **P0.3 ✅ Platform-stable counts.** All adaptive/floor counts now come
>   from `stableDeflectionCount` — a closed-form per-uniform-interval
>   scan (tangent-angle turn + midpoint sag) instead of OCCT's
>   iterative `GCPnts_TangentialDeflection`, whose data-dependent
>   branches flipped on MSVC libm ulps. Floors now apply through the
>   density groups (per-edge floors were silently breaking group
>   equality), and conform leaves already-welded seams alone.
> - **P1.1 ✅ Interior cutouts on curved walls.** A partial-wrap wall
>   with a slot/hole routes to the open revolution band and boolean-cuts
>   the wire out of its straight full-height columns: per-column rows at
>   the cut's v-extents (no full-width band), covered cells deleted, the
>   cavity laddered to the wire's exact contract samples by polar angle
>   — grouped quads/n-gons, no fans. barrel fixture: 86 tris -> 0,
>   watertight, 0 folds; new `drilled` fixture (round hole) clean from
>   day one. Stepped/castellated rims keep their coons route (barrel2).
> - **P1.3 ✅ Density-edit safety.** The sweep harness (below) found
>   and fixed: the annulus containment floor (a holed plate's outer
>   ring must out-resolve its clearance or no web can triangulate it)
>   and full per-face cache keys (a stale part against a re-meshed
>   neighbour leaked exactly the edited count). Definitive acceptance:
>   **7,950 per-face radial-sweep runs (radial 8..48, warm cache)
>   across all 13 corpus models, 0 failures.**
> - **Harness ✅.** `tools/corpus_gate.sh` (watertight + no-raw-demotion
>   + golden count diff over all fixtures and corpus models, both
>   profiles) and `weft sweep` (adversarial per-face radial sweep
>   through the generation cache). `GenerationReport::faceBuild` +
>   the CLI's `demoted:` line make every demotion visible (P0.1's
>   reporting half); raw-OCCT demotions are gate failures.
> - **P0.1 ✅:** 0 fallback-tri at default corpus-wide; the full-corpus
>   density sweep above is the never-fall-back evidence (no raw
>   demotions or empties above any model's base at any count). Known
>   cosmetic residue: flaregun's grip ribbon (face 131) absorbs its
>   rail-chain mismatch as transition tris at the *default* profile
>   (cad profile: clean quad ladder + one pentagon cap) — a quality
>   item, not a correctness one (future lever: rail-chain sum
>   equalization in the density solve).
> - **Interactivity overhaul (app).** One `effectiveKind` resolver links
>   every parameter surface to the face's real mesher (forced choice
>   wins); the async dropped-edit bug is fixed (only startGenerate
>   clears `dirty`, so edits and undos made during a run always coalesce
>   into a follow-up pass); build health is visible everywhere
>   (emitted-nothing faces: Topology callout + select button, outliner
>   tag, panel warnings); global defaults sit in per-family tabs
>   (freeform / cylinders / fillets / ribbons / rings / flat faces)
>   with hover-to-highlight.

## 1. What this software is for

Weft turns a **Plasticity CAD export (STEP / B-rep)** into a **clean, watertight,
quad-dominant polygon mesh** that a Blender artist can actually work on — output
that beats Plasticity's own Bridge / mesh-export, which ships triangle soup or
coarse, seam-broken quads.

Two design goals, in tension, both required:

- **Global auto-solve.** Load a model, get good topology with zero tweaks. The
  planner reads the B-rep, classifies every face, and meshes each with the right
  strategy so the whole model reads as one coherent quad flow.
- **Local artist control.** Where the auto-solve isn't what the artist wants,
  they select a face (or region) and adjust density / mesher, and the result
  improves **without ever breaking watertightness**.

Primitive-priority ordering (the artist's rule, honoured throughout):
**Cylinder > Sphere > Hemisphere > Box > Torus > Curves > interior faces.**
Primitives drive the topology; curves and interior cuts follow.

## 2. MVP acceptance criteria (the bar)

An MVP is "an artist can use this instead of Plasticity's export on real work."
Concretely, on the test corpus **and** a batch of fresh Plasticity exports:

1. **Loads & meshes** every model without crashing or hanging.
2. **Watertight + manifold, always.** 0 open edges, 0 non-manifold edges. No
   exceptions (foam currently fails this — see §4 P0).
3. **Quad-dominant, no tri-soup.** No face demotes to the OCCT triangulation
   fallback on ordinary geometry. Triangles appear only as deliberate, local
   transitions (pole collapses, n-gon fans the user opted into).
4. **Deterministic across Windows / macOS / Linux.** Same model → same topology.
   (5 pipeline checks currently differ on MSVC — see §4 P0.)
5. **Beats Plasticity visually** on the representative set: primitives, fillets,
   flat panels, through-holes, slots. Clean columns on cylinders, clean strips
   on fillets, local collars around holes.
6. **Round-trips to Blender** via OBJ export (and the live link) with the mesh
   intact.
7. **Per-face override is safe:** select a face → change mesher / density →
   result improves and stays watertight. Never demotes a neighbour to fallback.

## 3. Where we are today (honest state)

**Solid foundation already in place:**

- A per-face planner (`planFace` in `core/src/meshers.cpp`) that covers the
  primitive taxonomy: `RevolutionGrid` (cyl/cone/sphere/torus), `DiskCap`,
  `PlanarGrid`, `CoonsGrid`, `RingJunction`, `MinimalNGon`, `AnnulusRing`,
  `PlateWeb`, `QuadFill`, `RailLadder`, `RibbonSweep`, `DomeCap`, `Fallback`.
- A density solve (`solveDensity`) with union-find edge groups, curvature-
  adaptive counts, and per-face / per-edge overrides.
- A **border contract**: shared edges are sampled identically by both faces, so
  welds are watertight by construction.
- An app viewer (`app/main.cpp`) with a per-face settings editor, adaptive-
  density toggle, radial/axial/gridU/gridV controls, OBJ export, Blender live
  link.
- Most of the corpus meshes watertight and quad-dominant today.

**Shipped this session (branch `claude/handoff-continuation-zvaa0u`):**

- Barrel rim-grounding made **robust to density changes**: bumping a barrel's
  radial no longer brings back the bottom transition ring, and no longer demotes
  a neighbour to triangulation. Achieved via blend-group density propagation +
  cut-rim pinning + a notch-corner degeneracy guard. Verified clean at radial
  8–48, default corpus byte-identical.
- **MSVC build fix** (a missing OCCT include that broke the Windows build).

**Known gaps (evidence gathered this session):**

- Faces still **demote to OCCT triangulation** in some situations (density
  edits, certain geometry) — the artist sees tri-soup.
- **foam** is not watertight at default (73 open edges).
- **5 pipeline checks fail on Windows** (platform float → different tessellation
  counts).
- **Interior cutouts on curved faces** (a slot/hole through a cylinder wall)
  mesh as a coons cutout that **fans** the slot ends and lays full-width rows.
  The clean path (open revolution band + local collar) is blocked by a seam-
  welding vs slot-row reconciliation problem (see §4 P1).
- **Coons on a cylinder ignores `radial`** — its circumferential count comes
  from `gridU` (default → 2), not curvature/radial.
- Density edits can still mismatch seams on faces outside the barrel blend group
  (propagation is not yet universal).

## 4. Gap analysis → prioritized workstreams

### P0 — Correctness & robustness (MVP blockers)

**P0.1 — The "never fall back" invariant.**
A face with an assigned mesher must never silently become OCCT triangle soup.
Audit every `demote(...)` / "border contract failed" path. Each mesher must
degrade *within its own family* — a structured transition strip, a local n-gon,
or a documented graceful floor — not the OCCT fallback. Where a genuine
irreconcilable case exists, it must be rare, local, and logged.
*Done when:* 0 `fallback-tri` on the corpus at default density, and a density
sweep (per-face radial 8–48 on every band/curved face) never introduces one.

**P0.2 — Universal watertightness.**
Fix foam's open edges and any other non-watertight corpus model. Add a hard
validation gate to the pipeline that treats open/non-manifold edges as a failure
to be fixed, not a warning to ship.
*Done when:* every corpus model + a fresh Plasticity batch is 0/0.

**P0.3 — Cross-platform determinism.**
Diagnose the 5 Windows pipeline failures (need the `FAIL …:` lines). Likely OCCT
`GCPnts_TangentialDeflection` producing different point counts on MSVC. Either
derive adaptive counts from a platform-stable formula, or make the count-
assertion tests tolerance-based where a ±1 tessellation difference is genuinely
equivalent. Add CI that runs the suite on Windows + Linux.
*Done when:* the pipeline suite passes identically on Win + Linux.

### P1 — Core topology quality (the value proposition)

**P1.1 — Interior cutouts on curved faces (holes & slots).**
This is the single biggest remaining quality gap and the most common real
feature (bolt holes, slots, vents through walls). Two viable routes:
- *(a)* Teach the **open revolution band** to accept an interior insert: a
  straight-column grid with rows at the slot's v-extents, the covered cells
  deleted, the slot webbed as a local collar. Blocker (already pinned down): the
  seam edges are shared borders, so the slot-aligned rows and the seam's
  contracted sample count must be reconciled — either the density solve places
  the seam count at the slot extents, or the insert path drives the seam
  sampling.
- *(b)* Rework the **coons cutout** to seed its lattice with slot-v-extent rows
  so the hole is strictly interior on the first pass — no refine, no fans — then
  delete + web instead of adaptively refining.
*Recommendation:* (a) gives the cleaner columns and matches the primitive-
priority rule; (b) is more contained but keeps the face on coons. Prototype (a)
first; fall back to (b) if the seam reconciliation proves too invasive.
*Done when:* the barrel fixture (and a hole-through-cylinder + slot corpus) mesh
with straight full-height columns and a local collar, watertight, no fans.

**P1.2 — Curved-coons radial density.**
When a cylinder/cone face is meshed as coons (because it carries an interior
wire the band can't yet take), its circumferential direction must be curvature-
/radial-driven, not `gridU`. Largely **subsumed by P1.1** — if the wall routes
to the open band, it gets radial columns natively. Keep this as the fallback fix
if some curved faces must stay on coons. (Note: the earlier flat-`radial=16`-on-
every-coons attempt regressed the corpus; any fix must be curvature-proportional
and verified per-model.)

**P1.3 — Global density coherence.**
Generalize the blend-group propagation (currently barrel-specific) so that
changing any face's density drags its column-flow-connected neighbours to a
consistent count — no seam ever ends up mismatched by an edit. This is what
makes per-face control *safe*.
*Done when:* editing any face's radial on any corpus model keeps every seam
watertight and never demotes a neighbour.

### P2 — Artist control & UX

**P2.1 — Coherent per-face tweak model.**
Audit the settings editor: for each mesher kind, show only the knobs that do
something (radial/axial for revolution, gridU/gridV for coons/planar, fillet
loops/hold for blends — the last already gated this session). Live preview on
edit. Make "select a bad face, fix it" a smooth loop.

**P2.2 — Minimal-ngon toggle → local triangulation.**
With minimal-ngons ON, a flat face is one boundary n-gon. With it OFF, only the
n-gon *regions* triangulate — not the whole face re-meshed. (Requested; not yet
implemented.)

**P2.3 — Overrides can't break watertightness.**
Any per-face/per-edge override is clamped/propagated so it can never open a seam
or force a neighbour to fallback (depends on P1.3).

### P3 — Polish & breadth

- **Ring B** (the notch feature-rows the artist dislikes) — decide whether the
  topology can avoid them or whether they're accepted as a local n-gon.
- **Broader corpus**: run a batch of real Plasticity exports, not just the
  fixtures + 7 hero models; fix whatever demotes/opens.
- **Thin bands / degenerate faces / tiny fillets** edge cases.

## 5. Phased roadmap

**Phase 0 — Bulletproof the current output (P0).**
No new topology; make what exists never fail. Exit: corpus 100% watertight, 0
fallback-tri at default, a per-face density sweep introduces no fallback, tests
green on Win + Linux.

**Phase 1 — Curved cutouts & density coherence (P1).**
The core quality leap. Exit: holes/slots through curved walls mesh as straight
columns + local collar; density edits stay coherent across seams everywhere.

**Phase 2 — Artist control (P2).**
The tweak loop. Exit: an artist can fix any face by selecting + adjusting, and
the mesh stays watertight; minimal-ngon toggle behaves locally.

**Phase 3 — Ship readiness (P3).**
Blender export hardening, broad Plasticity-export testing, docs, packaging.

## 6. Key strategic decisions

- **Harden the per-face `generate()` path for MVP; treat the decoupled/global
  core as post-MVP.** The per-face planner is ~80% of the way to the bar. A
  ground-up global solver is a higher ceiling but a much longer, riskier road —
  not the fastest path to a usable MVP.
- **Watertightness is the non-negotiable invariant.** Enforce it with a
  validation gate that fails loudly, plus the "never OCCT soup" rule (P0.1).
- **Determinism is a feature, not an afterthought.** Counts that vary by
  platform make the tool unreliable; lock the adaptive-count derivation.
- **Every mesher change is verified the disciplined way:** default corpus stays
  byte-identical (or the change is visually inspected per affected model), and a
  density sweep confirms no new fallback/ring/open-edge. This is the process
  that kept the barrel work from regressing.

## 7. Risks & mitigations

| Risk | Mitigation |
|---|---|
| Watertight-critical mesher edits regress the corpus (felt all session) | Byte-identical-default discipline + per-model visual verify + adversarial density sweeps before every commit |
| Cross-platform float determinism needs rework, not a patch | Treat as a P0 workstream, not a test tweak; may need a platform-stable count formula |
| "Global solve" scope creep | Explicitly deferred to post-MVP; MVP hardens the per-face path |
| Interior-insert seam reconciliation proves invasive | Fall back to the contained coons-cutout rework (P1.1b) |

## 8. Verification harness (build this alongside)

- **Corpus sweep**: mesh every model + fixture at default and across a radial
  sweep; assert watertight 0/0, 0 fallback-tri, record quad/tri/ngon counts for
  regression diffing.
- **Golden counts**: a per-model expected-count table; a change that moves them
  must be explained (better) or reverted (regression).
- **Cross-platform CI**: run the pipeline suite on Windows + Linux on every push.
- **Visual diff**: headless renders (the `--screenshot` path) of hero faces
  before/after, for the changes counts can't capture (rings are quads).
