# Weft — MVP Plan

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
