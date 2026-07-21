# Production generation path

Authoritative MVP generation is a single call:

```text
import STEP → analyze → weft::generate() → optional applyOps → export
```

Architecture decision: [AD-1](EXECUTION_PLAN.md#ad-1-production-generation-path)
in `docs/EXECUTION_PLAN.md`. Product roadmap and gates stay there; this note
only records which entry points share that path.

## Authoritative API

`weft::generate(model, analysis, settings, report?, cache?)` in
`core/include/weft/meshers.hpp` owns planning, border-count contracts, per-face
meshing, weld, and finalization. Face strategies (`MesherKind`) are internal
routing inside that call, not alternate product pipelines.

## Entry points (audit)

| Surface | Calls `weft::generate()`? | Notes |
| --- | --- | --- |
| App interactive regen | Yes | `finalizeMesh = false` for responsive preview |
| App export / finalized mesh | Yes | `finalizeMesh = true` (authoritative mesh) |
| CLI `mesh` / `validate` / `sweep` / `cache-check` | Yes | Default `finalizeMesh = true` |
| Corpus / release / public gates | Yes | Via `build/cli/weft mesh` (no `--stitch`) |
| `tests/test_pipeline.cpp` | Yes | Direct API; stitch A/B only in quarantined tests |
| Blender live-link add-on | N/A | Consumes exported OBJ; does not mesh |
| CLI `convert` | No | Non-retopo import→export; mesh outputs use OCCT tessellation only |

No second retopology architecture was found. Preview vs export differs only by
`GenerationSettings::finalizeMesh` on the same `generate()` path.

## Settings that matter on the default path

| Setting | Default | Role |
| --- | --- | --- |
| Density / profile (`radial`, `axial`, `--profile cad`, …) | product defaults | Border proposals and feature routing |
| `perFace` / `perEdge` | empty | Local overrides and exact edge pins |
| `densityScale` | `1.0` | Global budget multiplier (persisted in recipes) |
| `weldTolerance` | `1e-6` | Seam fusion tolerance (persisted) |
| `conformBorders` | `true` | Border conformation on finalized runs |
| `finalizeMesh` | `true` | Full weld/conform/cleanup; app preview sets `false` |
| `parallelMeshing` | `true` | Runtime parallelism; not a topology fork |
| `decoupleSeams` | **`false`** | Quarantined experiment (AD-2) |

Recipes persist density, weld, per-face/per-edge settings, and manual ops.
They do **not** persist `decoupleSeams`, `finalizeMesh`, or `conformBorders`.

## Stitch quarantine (AD-2)

`GenerationSettings::decoupleSeams` and CLI/app `--stitch` are off by default
and are not a second product architecture. They skip global count equalization
and rely on post-weld seam splicing for diagnosis.

Where the opt-in lives:

- Default `false` in `GenerationSettings` (`core/include/weft/meshers.hpp`)
- CLI: `--stitch` in `cli/main.cpp` (experiment flag on `mesh` / validate options)
- App: `--stitch` for screenshot runs; optional ImGui “decoupled seams (stitch)”
- Tests may force it for A/B; corpus and release gates do not.
  Former `tools/probes` stitch dumps are retired (see
  `docs/evidence/wp2-probe-retirement-2026-07-20.md`).

Promote or remove only after the AD-2 A/B evidence criteria in the execution
plan.

## What is not a second architecture

- **Per-face `MesherKind` backends** — strategies inside `generate()`, frozen
  under AD-3 (do not add kinds during stabilization).
- **`finalizeMesh` preview vs export** — same pipeline; export is authoritative.
- **CLI `convert` tessellation** — format conversion without retopo; not a
  competing mesher for MVP delivery.
- **`--stitch` / `decoupleSeams`** — quarantined experiment (AD-2), not a
  release pipeline.
- **Removed decoupled-core rewrite** — historical; do not restore (AD-1).

If a new entry point meshes without `weft::generate()`, treat that as a WP2
blocker and document it here before landing it.

## Cross-platform topology signature (§3.2)

Machine-comparable artifact for Linux/Windows determinism. It captures face
routing (`MesherKind` counts + per-face ids), raw/empty/floor status, polygon
arity, connectivity/validity metrics, and an optional quantized UV anchor
hash. Byte-identical OBJ floats are **not** required.

```sh
# Emit (runs generate + validate; writes policy + info lines)
tools/topology_signature.sh emit model.step -o model.sig
# or: build/cli/weft mesh model.step --validate --signature model.sig

# Compare (exit 0 if policy-equal, 1 if not; ignores info.* lines)
tools/topology_signature.sh compare a.sig b.sig

# Same-platform smoke (cylinder / box / torture, twice each)
tools/topology_signature.sh self-check

# Small CI fixture set for later Windows comparison
tools/topology_signature.sh fixture-set -o build/topology_signatures
```

Schema: `weft.topology_signature.v1` (`weft::formatTopologySignature`).
