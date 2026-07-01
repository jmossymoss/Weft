# Weft

A dedicated bridge between CAD (STEP from Plasticity) and game-ready topology
in Blender. Instead of retopologizing a dead mesh, Weft ingests the B-rep,
keeps the surface math live, and generates topology from it — feature-aware,
per-face controllable, regenerable at any density without losing manual work.

Full plan and architecture: [docs/PLAN.md](docs/PLAN.md).

## Status: Phase 0 — prove the loop

Headless C++ core + CLI covering the plan's Phase 0 spike end to end:

- STEP import via OpenCASCADE (with shape healing) and stable face/edge IDs
- B-rep analysis: surface classification (plane/cylinder/cone/sphere/torus/
  NURBS...), edge convexity (convex/concave/smooth) with dihedral angles,
  face-adjacency graph
- Per-surface topology generation with **named, per-face density controls**:
  - full-revolution cylinders → exact radial × axial quad grids
  - circular caps → n-gon or triangle-fan, ring-aligned with the side face
  - planar/parametric faces → quad grids (with trim-boundary containment
    check; trimmed faces fall back to OCCT triangulation — conforming grids
    to trim curves is the Phase 1+ hard problem, plan §7.1)
- Vertex welding across B-rep face borders (watertight where divisions match)
- OBJ export with one group per B-rep face, so CAD face IDs survive into
  Blender

The viewport shell, manual editing layer, and Blender bridge come later; the
core is headless-first by design (plan §2.1).

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
