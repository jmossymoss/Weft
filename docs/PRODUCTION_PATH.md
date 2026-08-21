# Production generation path

This fork has two mesh entry points. They are not interchangeable.

```text
import STEP → analyze → weft::meshIndependent() → optional applyOps → export
```

That is the product path under test (execution plan T0–T4).

```text
import STEP → analyze → weft::generate() → optional applyOps → export
```

That is the legacy border-contract path. Keep it for A/B and for corpus
goldens that were measured against `generate()`.

Architecture: [EXECUTION_PLAN.md](EXECUTION_PLAN.md).

## Independent mesh (this fork)

`weft::meshIndependent(model, analysis, settings, report?)` in
`core/include/weft/independent_mesh.hpp`.

It tessellates each face from angle and chord, emits planar n-gons, welds
spatially, and honors per-face span knobs as local edge requests. It does
not run the density-group solver.

Shared B-rep edges are sampled once. Planar faces become n-gons (keyhole
when holed). Simple drums/spheres get a UV lattice. Four-sided patches
use a transfinite grid. Remaining faces ear-clip the sample loop in UV.
OCCT is last resort only.

| Surface | Calls `meshIndependent()`? | Notes |
| --- | --- | --- |
| CLI `mesh` / `validate` with `--independent` | Yes | Opt-in; default remains `generate()` |
| `tests/test_independent_mesh.cpp` | Yes | Zoo fixtures |
| App interactive / export | Not yet | T3 |
| Corpus / release gates | No | Still `weft mesh` → `generate()` |
| CLI `convert` | No | `weft::io::tessellate()` only (format conversion) |

`--independent` is not persisted in recipes.

## Legacy generate()

Unchanged from mainline. Face strategies (`MesherKind`) stay internal to
`generate()`. `--stitch` / `decoupleSeams` stay quarantined on that path and
are unrelated to independent tessellation.

| Surface | Calls `weft::generate()`? | Notes |
| --- | --- | --- |
| CLI `mesh` / `validate` (default) | Yes | |
| App (until T3) | Yes | |
| Corpus / release / public gates | Yes | |
| `tests/test_pipeline.cpp` | Yes | |
| CLI `sweep` / `cache-check` | Yes | Contract-path harnesses |

## Settings that matter on the independent path

| Setting | Role |
| --- | --- |
| `chordTolerance`, `angleToleranceDeg` | Default tessellation |
| `relativeDeviation` | Scale chord by face size |
| `minCurvedSegments` | Floor on closed curved edges |
| `radial`, `axial` | Drum span requests |
| `filletLoops` | Fillet-strip deflection |
| `minimal` | Planar single-wire n-gons when true |
| `weldTolerance` | Spatial weld |
| `perFace` / `perEdge` | Local span / exact edge count |
| `exclude` | Skip face |

`densityScale`, `decoupleSeams`, and the contract-path density solver are
ignored by `meshIndependent()`.

## What is not a second architecture on this fork

- `weft::io::tessellate()` — convert-only OCCT dump, no n-gons, no spans.
- `--stitch` — diagnostic inside `generate()`.
- GPU isoline overlay — viewport hint only, not a mesher.
