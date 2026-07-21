# Weft

Weft is a B-rep-native bridge from CAD to editable game topology:

```text
Plasticity STEP -> Weft -> Blender
```

It retains the source B-rep, generates feature-aware topology, provides local
per-face controls and surface-constrained edits, and stores those decisions in
recipes so output can be regenerated after density or CAD changes.

## Status

Weft is alpha software under correctness and workflow stabilization. The core
loop exists, but the release corpus and cross-platform gates are not yet the
evidence required for an artist-usable MVP. Do not interpret the implemented
feature count as a release-readiness claim.

The sole roadmap, completion definition, corpus strategy, and work order are in
[docs/EXECUTION_PLAN.md](docs/EXECUTION_PLAN.md). The single production
generation path (`weft::generate()`) is summarized in
[docs/PRODUCTION_PATH.md](docs/PRODUCTION_PATH.md).

## Product boundary

The active goal is a reliable, manual-first retopology assistant with strong
automatic defaults. It is not currently attempting to solve every possible
B-rep with a universal zero-touch quad layout.

Deferred work such as cross-field meshing, UV packing, a comprehensive manual
editing suite, scripting, and additional export formats is listed in the
execution plan and must not displace stabilization work.

## Implemented foundation

- STEP, IGES, and BREP import through OpenCASCADE, including shape healing and
  B-rep face/edge maps.
- Surface classification, edge convexity, feature tags, and face adjacency.
- Specialized topology generation for analytic revolutions, caps, planar
  regions, fillet and ribbon strips, annuli, plates, holes, and fallback cases.
- Shared-edge density solving with global defaults, per-face overrides, and
  exact per-edge pins.
- Mixed triangle, quad, and n-gon output with face-attributed generation
  diagnostics.
- `(faceId, u, v)` anchors and exact surface re-projection for supported edits.
- Recipes containing generation settings and manual operations.
- Geometric recipe remapping after STEP re-export.
- OBJ, glTF, FBX, and STL delivery paths.
- A headless CLI, interactive app, validation tools, and Blender file-watch
  add-on.

Support varies by geometry class. Use `--validate` and the corpus gates rather
than assuming every imported solid will be watertight or quad-dominant.

## Interactive app

`weft_app` is a GLFW/OpenGL/ImGui client of the headless core. Current
capabilities include:

- STEP loading and built-in fixtures.
- Orbit, pan, zoom, framing, face selection, and feature coloring.
- Live global and per-face density controls.
- Wireframe, feature-edge, quality, and topology diagnostics.
- Supported loop insertion and constrained vertex grab.
- Recipe save/load and undo for recorded operations.
- STEP hot reload with geometric recipe remapping.
- OBJ-based Blender live link.

### Artist correction loop

Typical pass after a STEP loads with a strong automatic start:

1. Select the problem face (face mode). Only knobs that drive that face's
   mesher appear; scroll or drag primary/secondary density (`[` / `]`, or
   Shift for the secondary axis).
2. Insert a constrained loop (`R`, click, drag to slide) or grab a vertex
   (`G`) — both stay on the CAD surface and record into the recipe.
3. For open borders after a delete (`X`): bridge (`J` / `B`) or fill (`F` in
   bridge mode). Weld verts with `M` in vertex mode. Unsupported edits fail
   with a status line and do not append to the recipe.
4. Undo with `Ctrl+Z`. Save the recipe (`Ctrl+S`), bump density, and regenerate
   — surviving ops re-apply from anchors.
5. Export (`Ctrl+E`) for the finalized mesh. With Blender live-link enabled,
   regenerates also write the finalized OBJ (not the reduced preview).

Common controls:

- `R`: loop-cut mode.
- `G`: constrained grab.
- `X`: delete selected faces or polygons; `Ctrl+X` dissolves a mesh-edge loop.
- `J` / `B`: bridge selected/open boundary edges.
- `M`: weld selected vertices in vertex mode; toggle minimal n-gon in face
  mode.
- Type a number then `Enter`: set the selected face's primary divisions.
- `Shift+Enter`: set its secondary divisions.
- `[` / `]`: adjust divisions; hold Shift for the secondary axis.
- `C`: cap style.
- `T`: triangle/quad-dominant toggle.
- `W`: wireframe.
- `F`: frame selection (or fill the hovered open boundary in bridge mode).
- `Esc`: cancel or deselect.
- `Ctrl+S`: save the recipe.
- `Ctrl+E`: export (finalized).

Install `blender/weft_link.py` as a Blender add-on and use the Weft sidebar to
watch the app's live-link OBJ.

Interactive regeneration may defer expensive whole-model repairs for
responsiveness unless live-link is on. Export always regenerates with
finalization enabled and is the authoritative mesh.

Headless screenshots are available for visual checks:

```sh
weft_app --fixture boss --screenshot out.png
```

## Build and test

Requires CMake 3.20 or newer, a C++17 compiler, and OpenCASCADE development
packages. The core remains headless; the app additionally needs OpenGL and
GLFW. CMake can fetch the app dependencies when system packages are absent.

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
tools/corpus_gate.sh
```

See [WINDOWS_BUILD.md](WINDOWS_BUILD.md) for Windows setup.

## Try the core loop

```sh
# Generate demo CAD as STEP.
build/cli/weft fixture demo.step --shape demo

# Inspect its B-rep.
build/cli/weft inspect demo.step

# Generate and validate topology.
build/cli/weft mesh demo.step -o demo.obj \
  --radial 12 --axial 3 --validate

# Override one face.
build/cli/weft mesh demo.step -o demo.obj \
  --radial 12 --face 1:radial=24,axial=2 --validate
```

OBJ output uses one group per B-rep face (`face_1`, `face_2`, and so on), which
preserves CAD face identity for Blender workflows.

## Repository guide

```text
AGENTS.md                  autonomous-agent operating rules
docs/EXECUTION_PLAN.md     sole product roadmap and completion definition
docs/PRODUCTION_PATH.md    authoritative generate() path and stitch quarantine
core/                      headless geometry and meshing library
cli/                       command-line client
app/                       interactive client
blender/                   live-link add-on
tests/                     fixtures, corpus inventory, and pipeline tests
tests/CAD_CORPUS.md        corpus mechanics (zoo, release, public layers)
tests/public_corpus/       ABC, NIST/CAx-IF, MAMBO manifests (external cache)
tools/                     corpus, visual, and diagnostic tooling
build.sh / build.bat       platform build entry points
WINDOWS_BUILD.md           Windows setup and troubleshooting
```
