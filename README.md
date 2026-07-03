# Weft

A dedicated bridge between CAD (STEP from Plasticity) and game-ready topology
in Blender. Instead of retopologizing a dead mesh, Weft ingests the B-rep,
keeps the surface math live, and generates topology from it — feature-aware,
per-face controllable, regenerable at any density without losing manual work.

Full plan and architecture: [docs/PLAN.md](docs/PLAN.md).

## Status: Phase 2/3 — surface-constrained editing + feature recognition

Headless C++ core + CLI covering the plan's Phases 0–3 essentials:

- STEP import via OpenCASCADE (with shape healing) and stable face/edge IDs
- B-rep analysis: surface classification (plane/cylinder/cone/sphere/torus/
  NURBS...), edge convexity (convex/concave/smooth) with dihedral angles,
  face-adjacency graph
- Per-surface topology generation with **named, per-face density controls**:
  - closed surfaces of revolution (cylinder/cone/sphere/torus) → exact
    radial × axial quad grids with wrap-around seams; cone apexes and
    sphere poles collapse to clean triangle fans
  - circular caps → n-gon or triangle-fan, ring-aligned with the side face
  - planar/parametric faces → quad grids (with trim-boundary containment
    check)
  - trimmed/freeform faces → **guided quad-dominant meshing** (plan §3.5
    seed): OCCT triangulation, greedy tri-pairing scored by quad quality
    and alignment with the surface's parametric directions, then one
    midpoint subdivision — pure quads, with every new vertex evaluated
    exactly on the B-rep. `--pure-tris` disables it. (A cross-field
    solver slots in here later; grid conformity to trim curves is the
    plan's §7.1 hard problem, still ahead.)
- **Density matching across shared edges** (plan §4.2): edge subdivision
  counts are solved as shared constraints over the adjacency graph —
  opposite sides of a grid and the rings of a revolution face are grouped,
  each face proposes its settings, groups resolve to the max proposal.
  Change one face's density and its neighbours follow; the solid stays
  watertight with no T-junctions. Edges can also be pinned exactly
  (`--edge 3:20`), the first slice of the plan's per_edge_settings.
- **Fillet recognition + support loops** (plan §3.3): cylindrical/toroidal
  strips joined to their neighbours by tangent-smooth edges are detected as
  blends (`inspect` tags them). They get `--loops N` divisions across the
  blend — density-matched into the rest of the model — and `--hold F`
  clusters those loops toward the creases for bake-friendly shading.
- **Hole/boss recognition + ring junctions** (plan §3.3's junction
  patterns, first entry): bores are detected and tagged, and a planar face
  carrying a circular hole or boss root meshes as concentric quad rings
  (`--rings N`) instead of triangle soup. The ring count is derived from
  the face's border and propagates through density matching to the
  boss/bore itself — a drilled plate comes out as 100% quads, watertight.
- **Game-topology output is the point** (plan §1/§4.1): tris, quads, and
  n-gons are all first-class. Caps can be n-gons or triangle fans; poles
  and cone apexes are fans; trimmed faces can stay triangle-based
  (`--pure-tris`); and a flat panel can collapse to a **single boundary
  n-gon** (`minimal=1` per face) while its border stays density-matched —
  watertight with zero interior topology. Not everything has to be quads.
- **Surface-constrained editing** (plan §3.4): every generated vertex
  carries a `(faceId,u,v)` anchor onto the live B-rep. Edge-loop insertion
  walks quad strips (closing on itself or absorbing into terminal n-gons,
  always watertight) and evaluates new vertices exactly on the CAD surface;
  vertex moves re-project exactly — true snapping, not shrinkwrap.
- **Recipes** (plan §5): save the full setup — density defaults, per-face
  overrides, per-edge pins, AND manual ops — keyed to stable CAD IDs
  (`--save-recipe` / `--recipe`). Manual edits anchor to `(faceId,u,v)`,
  so a loop inserted at one density re-applies itself after you change the
  density and regenerate. Decisions persist; the mesh is just a view.
- Vertex welding across B-rep face borders (watertight where divisions match)
- OBJ export with one group per B-rep face, so CAD face IDs survive into
  Blender

## The app

`weft_app` is the interactive shell over the same core (plan §6, v0):

- 3D viewport (GLFW + OpenGL): orbit (RMB/MMB), pan (shift), zoom (wheel),
  `F` to frame
- Load a STEP file or any built-in fixture with one click
- **Feature colouring**: fillets orange, holes purple, surface types
  tinted; B-rep edges drawn orange (convex) / blue (concave) / green
  (smooth tangent)
- **Click a face to select it** (GPU picking) — see its type, radius, and
  which mesher produced it
- **Live density**: drag radial/axial/grid/loops/hold/rings and the
  topology regenerates as you drag; per-face overrides on the selection
- **Keyboard-centric editing** (plan §6 — the panel is optional):
  - `R` — loop-cut mode: hover any edge, a live preview loop follows the
    cursor, click commits (recorded as a recipe op, `ctrl+Z` undoes)
  - type `12` then `Enter` — set the selection's primary divisions
    (radial for revolved faces, grid for planar; `shift+Enter` = 2nd axis)
  - `[` / `]` — nudge divisions (`shift` for the second axis)
  - `C` — cap n-gon/fan · `T` — allow tris (quad-dominant toggle) ·
    `M` — minimal n-gon
  - `W` wire · `B` feature edges · `F` frame · `esc` cancel/deselect
  - `G` — grab: the interior vertex under the cursor slides constrained
    to its CAD surface (exact re-projection); click commits, `esc` cancels
- Save/load the session recipe from the panel (`ctrl+S`; `<model>.recipe`
  auto-loads next to the source)
- **Blender live link**: tick "live link (Blender)" and every edit
  mirrors the mesh to a watched OBJ; install `blender/weft_link.py` in
  Blender and hit "Start watching" (sidebar > Weft) — the `Weft` object
  updates in place, materials and modifiers intact, with CAD face ids in
  the `weft_face` face attribute

Uses system `libglfw3-dev libgl1-mesa-dev libimgui-dev libstb-dev` when
present; otherwise CMake fetches and builds GLFW/ImGui from source (the
normal path on Windows), so the app builds everywhere OpenGL exists.
`weft_app --fixture boss --screenshot out.png` renders headlessly (e.g.
under `xvfb-run`) for CI/visual checks.

The core stays headless-first by design (plan §2.1); the app and the
Blender bridge are layers over the same recipe pipeline.

## Build

Requires CMake ≥ 3.20, a C++17 compiler, and OpenCASCADE dev packages
(`libocct-*-dev` on Debian/Ubuntu).

```sh
cmake -B build
cmake --build build -j
ctest --test-dir build
```

## Try the loop

```sh
# Generate demo CAD (a cylinder and a box) as STEP
build/cli/weft fixture demo.step --shape demo

# See what the B-rep contains
build/cli/weft inspect demo.step

# Generate topology: 12 divisions around every cylinder, 3 along the axis
build/cli/weft mesh demo.step -o demo.obj --radial 12 --axial 3

# Per-face override: face 1 gets 24 radial divisions, everything else 12
build/cli/weft mesh demo.step -o demo.obj --radial 12 --face 1:radial=24,axial=2
```

Open the OBJ in Blender: each B-rep face arrives as a named group
(`face_1`, `face_2`, ...).

## Layout

```
core/   weft_core — headless geometry/meshing library (the product)
cli/    weft — command-line client of the core
tests/  end-to-end pipeline tests (CTest)
docs/   PLAN.md — the full technical plan this implements
```
