# ADR-0018: Source-referenced recipe v2

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core, CLI, and desktop app

## Context

Recipe v1 stores transient face and edge integers from whichever B-rep happened
to be active. Repair, source re-export, or topology reordering can make those
integers identify different entities. Its world-space polygon operations also
cannot be replayed against exact source geometry without guessing. Continuing
to write v1 would therefore allow silent loss or misapplication during the
secure-core migration.

## Decision

- `weft-recipe 2` stores immutable source entity IDs plus normalized geometric
  fingerprints for every face, edge, and surface-anchored operation.
- Numeric settings use classic-locale parsing and 17-digit floating-point
  formatting, preserving round-trip values independently of machine locale.
- Fingerprints include exact OCCT family, normalized measure and centroid, and
  topology cardinality. They are strict relocation evidence, not fuzzy scores.
- An unchanged source resolves by hash, source ordinal, fingerprint, and the
  audited source-to-working map.
- A changed source may relocate a reference only when its fingerprint has one
  unique candidate. Missing and ambiguous candidates are conflicts.
- Only bijective identity or modified correspondence can resolve a reference.
  Split, merged, removed, introduced, and multiply mapped entities conflict.
- Recipe v1 remains readable. Its pipeline selector is an ignored warning;
  face, edge, and supported manual operations migrate through source evidence.
  World-space delete, dissolve, and weld operations refuse by name.
- Application edits are captured in the reverse direction, from working IDs to
  source references, through the correspondence map. No ordinal equality is
  assumed.
- The CLI and app write only recipe v2. An incomplete v1 migration cannot be
  saved as a partial v2. A resolved but currently inapplicable v2 remains
  inspectable and may be saved without dropping its source references;
  unresolved references block app recapture.
- CLI flags following `--recipe` retain their historical precedence. The
  resolved working recipe is updated, captured back to source references, and
  only then saved or validated. Flags before `--recipe` remain superseded by
  the loaded recipe.
- At this rollout gate, only radial, axial, chord, and normal-angle global
  controls may drive secure generation. Per-face, per-edge, manual, and legacy
  modelling controls persist but block generation/export by a stable code.

## Invariants

- no recipe reference is silently dropped, reassigned, or interpreted as a
  healed-shape ordinal;
- every persisted entity reference resolves to immutable source evidence;
- changed-source relocation is unique or it fails;
- complete resolution is distinct from permission to apply an operation;
- `checked == 0`-style vacuity is mirrored here: an empty partial migration
  cannot be mistaken for a successful recipe;
- generation and export remain atomic when recipe conflicts exist.

## Alternatives rejected

- retaining v1 until every modelling operation is implemented;
- mapping old IDs directly onto working-shape indices;
- nearest-centroid or best-score fuzzy relocation;
- silently discarding world-space mesh surgery;
- saving a partially migrated v2 so the rest of the workflow can continue;
- applying per-face settings to uncertified legacy generators.

## Verification

`secure_recipe` tests cover source fingerprints, v1 migration, working-to-source
capture, non-identity correspondence ordinals, persistence, changed-source
relocation, missing and duplicate targets, malformed files, world-space
refusal, full-precision numeric round trips, and the application gate. Strict
and MSVC static-analysis builds pass, and all 14 secure/corpus tests pass in
both lanes.

Executable CLI proof migrates a v1 global recipe, writes v2, reloads it, and
produces identical OBJ and report SHA-256 digests. A per-face edit writes its
source fingerprint but exits 1 with
`secure_recipe.application.face_settings_unimplemented` and emits no mesh.
Ordered-override proof records radial 18 when `--radial 30` precedes
`--recipe`, and radial 30 when the same flag follows it.
The desktop app auto-loads a v2 sidecar and completes certified generation.

## Consequences

Recipe persistence and safe global replay are restored without reopening a
legacy generator. Per-entity density, surface-anchored editing, undo/redo v2
state, and geometric-fingerprint evolution still require gated M7 work.
