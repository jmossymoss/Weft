# WP2 evidence — border sampling, density ownership, fallback reporting (2026-07-20)

## Goal

Consolidate how shared borders get sample counts, who wins density conflicts,
and how fallback / contract-floor outcomes are reported — so WP3 fixes are
predictable. No new `MesherKind`, no filename / face-ID product special cases,
no intentional topology change in this slice.

## Ownership rules (plain language)

### 1. Border sampling (the shared contract)

Every structured mesher samples a B-rep edge at the **solved subdivision
count** for that edge, using the same forward 3D curve:

| Step | Rule | Code |
| --- | --- | --- |
| Count | `solvedEdge[eid]` from the density solve (+ floors) | `solveDensity` → floor passes → flat `solvedEdge` table in `generate()` (`core/src/meshers.cpp`) |
| Stations | Even 3D arc-length fractions when the curve is freeform; uniform parameter for lines/circles | `evenArcFractions` / `edgeSampleFractions` in `core/src/meshers.cpp` |
| Pins | Explicit station pins override fractions but keep the same positions on both faces | `PinnedEdges` + `edgeIsPinned` |
| Weld | Both faces emit identical sample positions → weld is vertex-for-vertex | border contract comments above `evenArcFractions` |

Fallback / contract-floor faces still honour exact solved borders
(`meshContractFallback`); raw OCCT triangulation is only the last resort when
the floor cannot express the face.

### 2. Density grouping

Before meshing, edges that a plan requires to share a count are united
(`EdgeGroups` union-find):

- opposite / matched sides of grids, Coons, revolutions (with rim-link rules);
- co-circular near-equal arcs (half-cylinder rims);
- **not** independent plate-web / minimal-ngon loop edges (each edge proposes
  alone so a bore can drive its hole).

### 3. Who wins a density conflict

Proposals land on the group root; resolve order is:

1. **Per-edge pin** (`GenerationSettings::perEdge`) — wins the whole group.
2. **Per-face count override** (typed `gridU` / `radial` / … differing from
   defaults) — pins that face's groups; several overrides share a group by
   **max**.
3. **Max of face proposals** among defaulted (and adaptive) faces.
4. **Ring-derived** counts for ring-junction circles: `2*(nu+nv)` from the
   plate ("plate drives the boss").
5. **Post-solve floors** (still group-wide so both faces agree):
   - curvature floor (~60° turn);
   - wire floor (closed wire ≥ 3 samples, rail-ladder ≥ 4);
   - annulus sag floor (outer ring vs hole clearance).

`densityScale` rescales non-explicit curved proposals before the group max;
explicit typed counts and straight edges do not rescale.

Band radial overrides also propagate through blend groups
(`propagateBandRadialToBlendGroup`) so a denser barrel does not leave a
sibling fillet on a sparser rim.

### 4. Fallback / contract-floor reporting

Unchanged from the demotion-attribution slice; surfaces next to density:

| Signal | Meaning |
| --- | --- |
| `faceBuild == 2` + `faceBuildCause` | Contract floor (exact borders) |
| `faceBuild == 1` + cause | Raw OCCT triangulation |
| `faceBuild == -1` + cause | Empty face |
| `formatBuildDemotions` | CLI/validate listing |

## Report surface added this slice

| Field / helper | Role |
| --- | --- |
| `GenerationReport::edgeDivisions` | Solved counts (now filled from `solvedEdge`, including floors) |
| `GenerationReport::edgeDivisionOwner` | Per-edge tag: `sole-proposal`, `max-proposal`, `face-pin`, `edge-pin`, `ring-derived`, `curvature-floor`, `wire-floor`, `annulus-floor` |
| `GenerationReport::densityConflicts` | Groups where proposals disagreed or a pin/floor raised the count |
| `formatDensityOwnership(report)` | Shared CLI/validate formatter (counts + owners + conflicts) |

Attribution is recording-only: the solve and mesh path are unchanged aside from
publishing loop edges on plate/minimal plans into `edgeDivisions` and reading
`solvedEdge` for reported counts.

## Focused test

`testSharedBorderSampleCounts` in `tests/test_pipeline.cpp`:

- Fixture: `box` via `makeFixture` (no corpus / interop edits).
- Discovers a shared edge and its two face ids from `analyze()`.
- Overrides density on one discovered face only.
- Asserts watertightness, matching shared mesh-border segments
  (undirected edges used by both faces equals the solved division count),
  ownership tag present, and `formatDensityOwnership` lists the edge plus
  at least one conflict.

## Consolidation note

Proposal recording, owner tags, and conflict emission now live beside
`DensitySolution` / `solveDensity` rather than being inferred after the fact
from dbg logs. Border sampling helpers remain the single contract used by
structured meshers and the contract floor. A larger extract of sampling into
`mesher_sampling.cpp` was deferred — current helpers are already the one path;
moving them would be mechanical and is not required for attribution.

## Conclusion

Shared-border sample counts, density ownership winners, and fallback causes are
explicit in `GenerationReport` and the CLI validate path. Fixes that change who
owns an edge count can now be checked against conflict tags without guessing
from polygon totals alone.
