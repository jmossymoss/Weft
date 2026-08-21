# Weft

Weft is a hard-surface CAD mesher:

```text
Plasticity STEP -> Weft -> n-gon / quad mesh -> Blender
```

This branch is a fork that tests MOI / Plasticity-style tessellation (angle +
chord, planar n-gons) plus per-feature span knobs for cylinders, holes,
slots, and fillets. It does not run the old model-wide border-contract
solver on the product path.

Roadmap: [docs/EXECUTION_PLAN.md](docs/EXECUTION_PLAN.md).
Which function each command calls: [docs/PRODUCTION_PATH.md](docs/PRODUCTION_PATH.md).

## Status

Alpha. The independent mesher is under T0 (callable core + zoo tests). Legacy
`weft::generate()` remains for comparison and for corpus goldens. Do not treat
either path as a shipping MVP.

## Product boundary

Hard-surface CAD only. The mesh should keep cylinders, bores, slots, and
fillets readable, with n-gon flats for DCC modeling. Organic remesh and
zero-touch global quads are out of scope.

Span edits are local: changing a drum from 24 to 8 is not allowed to
re-solve the rest of the body.

## Try the independent path

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

build/cli/weft fixture /tmp/cyl.step --shape cylinder
build/cli/weft mesh /tmp/cyl.step -o /tmp/cyl.obj --independent --validate

# Local span on one face (T1):
build/cli/weft mesh /tmp/cyl.step -o /tmp/cyl.obj --independent \
  --face 1:radial=8 --validate
```

Without `--independent`, `weft mesh` still calls legacy `generate()`.

On Windows after `build.bat`, `independent.bat` starts the GUI on this path
(`weft_app --independent`). `independent.bat cli mesh …` injects the flag.

## Implemented foundation

- STEP / IGES / BREP import through OpenCASCADE.
- `analyze()` surface, convexity, and feature classes (drum, fillet-strip,
  hole-plate, planar-panel, …).
- `weft::meshIndependent()`: per-face angle/chord tessellation, planar
  n-gons, spatial weld, optional span requests.
- Legacy `weft::generate()`: structured meshers + shared-edge density solve
  (comparison only on this fork).
- Recipes, surface-constrained edits, OBJ / glTF / FBX / STL export.
- Headless CLI, interactive app (still on `generate()` until T3), Blender
  live-link add-on.

## Build and test

Requires CMake 3.20 or newer, a C++17 compiler, and OpenCASCADE development
packages. The app also needs OpenGL and GLFW.

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

See [WINDOWS_BUILD.md](WINDOWS_BUILD.md) for Windows setup.

Corpus gates (`tools/corpus_gate.sh`) still measure `generate()`. Independent
mesh tests are `independent_mesh` in ctest.

## Repository guide

```text
AGENTS.md                  autonomous-agent operating rules
docs/EXECUTION_PLAN.md     sole product roadmap for this fork
docs/PRODUCTION_PATH.md    which entry points call which mesher
core/                      headless geometry and meshing library
cli/                       command-line client
app/                       interactive client
blender/                   live-link add-on
tests/                     fixtures, corpus inventory, and tests
tests/CAD_CORPUS.md        corpus mechanics
tools/                     corpus, visual, and diagnostic tooling
```
