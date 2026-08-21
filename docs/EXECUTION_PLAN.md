# Weft execution plan

This is the sole product and engineering roadmap for this fork. Historical
plans, the border-contract `generate()` path, old pull requests, and session
handoffs do not override this document.

Active work package: T0 — independent tessellation core.

This fork tests a MOI / Plasticity-style mesher: angle + chord tessellation,
planar n-gons, and per-feature span knobs. It exists so that experiment does
not disturb the previous border-contract pipeline. `weft::generate()` remains
in the tree as a comparison baseline. It is not the product path on this fork.

Change the active package only when its exit criteria pass at one revision.

## 1. Mission

Weft is a hard-surface CAD mesher for game and DCC work:

```text
Plasticity STEP -> Weft -> n-gon / quad mesh -> Blender
```

The job is a fast, editable mesh that keeps cylinders, holes, slots, and
fillets readable, with decent n-gon topology on flats. It is not organic
remeshing and not a universal cross-field quadrangulator.

Artists must be able to set spans on those features (the thing MOI's global
tessellator does not offer) without a model-wide density solve.

### Product strategy

- Independent face tessellation from global angle and chord (MOI / Plasticity
  mesh export).
- Planar faces emit boundary n-gons. Holed plates keep inner loops as holes,
  not fans.
- Feature span knobs (cylinder around/along, fillet across, hole/slot rim)
  request counts on that feature's edges. Neighbors are not re-solved.
- Boolean shards of one feature share a span when they share an edge, and
  later when they share axis + radius (or equivalent).
- Spatial weld, not a global vertex-for-vertex border contract.
- `weft::generate()` is legacy comparison only on this fork.

### Durable foundations

Keep these while changing the mesher:

- Headless C++ core and OpenCASCADE B-rep import.
- Stable face and edge identity.
- `analyze()` feature classes (drum, fillet-strip, hole-plate, planar-panel).
- Per-vertex `(faceId, u, v)` anchors where the mesher can fill them.
- Recipes for span / angle / chord decisions.
- OBJ / glTF / FBX export and the Blender live-link add-on.
- Corpus inventory in `tests/CAD_CORPUS.tsv`.

## 2. Why this fork exists

The previous pipeline meshed every B-rep face through a shared-border density
solver so neighboring faces met vertex-for-vertex. That produced accurate
stacks, and it made load and density edits expensive (full-model plan, group
solve, remesh of constraint closures, copy of cached parts, weld).

CAD apps that feel instant do not do that on load. They tessellate each face
from angle and chord. Matching rims happen because the same curve plus the
same deflection law yields the same N, not because a global solver raised
seventeen neighbors.

This fork takes that default and adds Weft's missing piece: local span knobs
on recognized hard-surface features.

## 3. Authoritative path

```text
import STEP → analyze → weft::meshIndependent() → optional applyOps → export
```

`weft::meshIndependent()` owns this fork's mesh. CLI `weft mesh --independent`
and tests for this experiment call it. `weft::generate()` stays callable for
A/B timing and topology comparison. Corpus gates that do not pass
`--independent` still exercise the legacy path so this branch cannot silently
change golden contract counts.

Details: [PRODUCTION_PATH.md](PRODUCTION_PATH.md).

## 4. Meshing rules

### 4.1 Default (no span override)

- Every face meshes from `chordTolerance` and `angleToleranceDeg`.
- Straight edges take 1 segment.
- Closed curved edges take the deflection count (and the artist
  `minCurvedSegments` floor when set).
- A shared B-rep edge is sampled once. Both faces reuse that polyline.
- Planar single-wire faces become one n-gon (quads count as four-gons).
- Planar faces with inner wires tessellate the sheet and keep hole rims.
- Simple full-period drums and spheres use a UV lattice. Four-sided
  patches use a transfinite grid (zippered when opposite counts differ).
- Other curved faces ear-clip the shared-sample loop in unwrapped UV.
  OCCT incremental mesh is only the last resort.
- Vertices fuse with `weldTolerance` (spatial, not combinatorial).

### 4.2 Feature spans

A span override is a request on a feature, not a model-wide constraint.

| Feature class | Knobs | What they change |
| --- | --- | --- |
| Drum (cylinder / cone / revolution wall, including bore walls) | radial, axial | Circular edges and along-axis edges of that face |
| Fillet strip | filletLoops | Deflection across the blend (sagitta from radius and loop count) |
| Hole plate / boss junction | radial, boundary | Inner-wire rim counts when those wires are curved |
| Planar panel | (none beyond global angle/chord) | Stays an n-gon |

Changing a drum from 24 to 8 remeshes faces that own the affected edges.
Unrelated faces keep their last mesh. Caps that share those circular edges
pick up 8 because the edge was resampled, not because a stack solver ran.

Do not propagate circumferential counts across an entire body by default.
A later explicit "match around" op may copy a rim count. It is not the cost
of opening a file.

### 4.3 Booleans

Plasticity booleans split one cylinder into shards. Classification must still
call those shards drums (already `FeatureClass::Drum`, including iso-bands
and hole walls). Spans apply per face in T0/T1. T2 groups shards that share
axis + radius so one knob drives the cut cylinder.

Slots are elongated hole wires or paired half-drums plus flats. Do not send
them through an organic remesher. Keep the inner loop and the wall features.

### 4.4 What this is not

- Not `--stitch` / `decoupleSeams` (those skip contracts inside `generate()`).
- Not a second `MesherKind` family bolted onto the density solver.
- Not shrinkwrap / voxel remesh.
- Not GPU isoline preview standing in for polygons.

## 5. Quality bar

For recognized hard-surface features on valid closed solids:

- Cylinders read as round at the chosen around-count, not as random tessellation.
- Holes and slots keep a rim loop; no triangle fans filling the bore.
- Fillets keep a visible across-span.
- Flats are n-gons (or a small web when holed).
- Output is usable in a DCC: mixed n-gon / quad / tri is acceptable.
- Open edges at a span mismatch (32 drum against a default cap) are allowed
  in interactive edits. Uniform global angle/chord on an unmodified model
  should weld on the zoo fixtures.

Numeric watertightness of the legacy contract path is not the T0 gate.
DCC-usable plus feature fidelity plus time is.

## 6. Performance bar

Targets for the independent path, CAD defaults, Release build:

- Zoo fixtures (cylinder, box, hole, fillet): mesh in well under a second.
- MP9-class (~3k faces): first mesh much cheaper than legacy `generate()`
  cold (~30 s class). Record the measured ratio; do not invent a budget
  until T0 has numbers.
- Editing one face's radial remeshes that face (and faces sharing its
  edges), not the model.

## 7. Engineering discipline

- Never special-case a filename, model name, or face ID.
- Do not change legacy `generate()` behavior to make this experiment look
  better. Do not refresh `tools/golden_*.txt` for `--independent` output.
- Add a deterministic reproducer before changing non-trivial meshing.
- `tests/CAD_CORPUS.tsv` remains the case inventory.
- Do not append session diaries to this plan. Put measurements in
  `docs/evidence/` or machine-readable reports.

## 8. Work packages

Agents work the first incomplete package only.

### T0: independent tessellation core

Goal: a callable `weft::meshIndependent()` that meshes a model without the
density solver, with tests and a CLI flag.

Tasks:

- Per-face OCCT tessellation from chord / angle, isolated so the live B-rep
  triangulation cache is not mutated.
- Shared edge sampling so both sides of an unmodified edge use one polyline
  intent (counts from deflection).
- Planar single-wire n-gons.
- Spatial weld.
- `weft mesh --independent` and `weft validate --independent`.
- Maintained tests on box, cylinder, and hole fixtures.
- Timing print on the CLI path.

Exit:

- Box: one polygon per face, no interior triangulation on flats.
- Cylinder: caps are n-gons / quads; the wall is tessellated; uniform
  settings produce a DCC-usable mesh.
- Hole fixture: the plate is not a triangle fan filling the bore.
- `--independent` does not change default `weft mesh` (legacy generate).
- Focused ctest for the new binary passes.

### T1: feature span knobs

Goal: radial / axial / filletLoops change only the edited feature.

Exit:

- `--face ID:radial=N` on a drum changes that drum's circular density.
- A second run with a different face id does not remesh the first drum
  unless they share the edited edges.
- FilletLoops tightens blend deflection on fillet-strip faces.

### T2: boolean feature groups

Goal: shards of one cylinder / fillet / slot share a span knob.

Exit:

- Grouping uses geometry (axis + radius, rail + radius, inner-wire
  signature), never face ids in source.
- A notched or iso-band drum takes one around-count.

### T3: interactive app

Goal: the app loads and edits on `meshIndependent()`, with span knobs.

Exit:

- Open / density nudge does not call `generate()`.
- Export dumps the independent mesh (see-what-you-get).
- Optional comparison action still runs `generate()`.

### T4: evidence vs legacy

Goal: A/B time and feature visuals on the release set and MP9.

Exit:

- Written report under `docs/evidence/` with timings, open-edge counts,
  and visual notes for cylinders / holes / fillets.
- Promote or keep quarantined from that report. Do not promote on taste.

## 9. Corpus

`tests/CAD_CORPUS.md` owns mechanics. This plan owns product intent.

Independent-mesh tests use generated zoo fixtures first. Release and MP9
enter at T4. Do not treat MP9 as a geometry-coverage oracle.

## 10. Legacy generate()

`weft::generate()` and AD notes from the previous mainline (border contract,
stitch quarantine, MesherKind freeze) still describe that function. They are
not the architecture of this fork's product path. Do not delete `generate()`
while T4 still needs A/B. Do not route corpus goldens through
`--independent` until T4 says to.
