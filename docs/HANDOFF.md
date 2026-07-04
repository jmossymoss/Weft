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
