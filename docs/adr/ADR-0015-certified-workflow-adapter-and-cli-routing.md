# ADR-0015: Certified workflow adapter and CLI routing

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core and CLI

## Context

`generateSecureMesh` returned the authoritative certified result, but the
existing exporters and workflow validators still consume `PolyMesh`. Leaving
the CLI on the historical generators would preserve two production meanings
for the same command. Rebuilding or welding certified triangles inside an
adapter would also destroy the identity proof carried by the secure result.

Recipe v1, manual surface operations, face overrides, and edge overrides are
keyed to the old representation. They cannot be applied to the certified mesh
until correspondence-aware recipe v2 migration exists.

## Decision

`makeCertifiedPolyMeshAdapter` is a narrow workflow/export adapter. It copies:

- certified positions without spatial search or welding;
- each certified triangle without retriangulation;
- the working face identifier;
- every certified corner UV as a face anchor; and
- the triangle indices into `PolyMesh::certifiedTriangles`.

The `weft mesh` command now imports with `importStepSecure`, calls only
`generateSecureMesh`, adapts the successful certified result, and then uses the
existing OBJ, FBX, glTF/GLB writer registry. `--radial`, `--axial`, and
`--chord` feed the secure count configuration. `--repair` selects the audited
repair profile and `--mesh-report` writes hashes plus every validation coverage
record.

Old `--pipeline` values and modelling-only flags remain parseable migration
input, but emit a warning and do not select a generator. Recipe v1, manual
operations, face overrides, edge overrides, and recipe saving block export by
name until recipe v2 migration is implemented. Unproven axial interior samples
also refuse by name.

## Invariants

- `weft mesh` has no legacy or experimental generation branch;
- the adapter cannot weld, omit, add, or retriangulate geometry;
- a secure refusal produces no export and no report claiming completion;
- a report can state `complete 1` only from a complete combined certificate;
- source and working hashes and the selected repair profile are visible;
- unsupported migrated controls are never silently discarded;
- representation-sensitive recipe operations cannot mutate certified output.

## Alternatives rejected

- keeping `--pipeline legacy|compiler` as a runtime selector;
- applying recipe v1 indices directly to working topology;
- regenerating corner UVs in exporters;
- calling the old generator for an unsupported secure family;
- accepting `--axial > 1` before interior samples carry exact provenance;
- emitting one ambiguous report for several LOD outputs.

## Verification

Strict and static-analysis MSVC builds pass. All 13 secure/corpus tests pass in
both lanes. The adapter unit test proves vertex, triangle, corner-anchor, and
certified-triangle cardinality and index identity.

Executable smoke tests prove:

- planar box to OBJ: 8 vertices, 12 triangles, watertight;
- capped cylinder to GLB: 48 vertices, 92 triangles, watertight;
- connected through-hole to FBX: 48 vertices, 96 triangles, watertight;
- each result has an identity conservative-repair certificate and a complete
  non-vacuous mesh report;
- `--pipeline legacy` emits a warning but still uses the secure route;
- sphere, axial-interior, and recipe-migration cases fail by stable reason and
  leave no output file.

## Consequences

The primary CLI mesh/export path now exercises the secure core end to end for
the currently proven plane/full-cylinder family. The app, Blender live link,
interactive edits, recipe v2 migration, `convert`, `cache-check`, partial
cylinders, and multi-tier report format remain M0/M7 work. `PolyMesh` remains a
compatibility adapter rather than an authoritative geometry representation.
