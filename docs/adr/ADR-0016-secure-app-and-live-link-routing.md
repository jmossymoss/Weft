# ADR-0016: Secure app and live-link routing

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core and desktop app

## Context

The desktop app independently generated three meshes: asynchronous viewport
preview, synchronous export finalization, and Blender live-link output. All
three still selected between historical generator families and replayed recipe
v1 operations after generation. A certified CLI beside an uncertified app
would leave the product with two conflicting correctness contracts.

The secure core currently proves planes, planar holes, and full periodic
cylinders. Temporary loss of unsupported modelling controls is preferable to
silently mutating certified topology or falling back to an old mesher.

## Decision

The app now:

- imports STEP through `importStepSecure` on its existing worker thread;
- retains the audited `ImportedModel` alongside the display `Model`;
- calls only `generateSecureMesh` for async regeneration and export;
- adapts certified triangles without welding or retriangulation;
- sends that same adapter to the viewport, OBJ/FBX/glTF exporters, and Blender
  live link;
- defaults to the supported box fixture instead of an unsupported mixed demo;
- exposes conservative/compatibility repair selection and repair-certificate
  hashes, validity, identity, correspondence, and operation counts;
- exposes only global radial, axial, chord, and normal-angle density controls
  as active secure controls;
- treats per-face/per-edge/manual recipe v1 data and legacy modelling knobs as
  visible migration conflicts;
- blocks recipe saving until recipe v2 exists, so the secure app never writes a
  misleading v1 file;
- keeps hot reload transactional when correspondence is incomplete; and
- replaces the watched Blender OBJ atomically, including an existing target.

Legacy pipeline fields in an otherwise safe recipe v1 are ignored visibly.
Safe global density values migrate. Representation-sensitive recipe data is
not applied or silently dropped.

## Invariants

- no app generation, export, or live-link path calls a historical generator;
- a failed secure regeneration keeps the previous valid mesh;
- unsupported geometry produces a named visible refusal and an empty viewport,
  not substituted triangles;
- export regenerates from the audited import and current secure settings;
- live-link bytes come from the certified adapter;
- replacing an existing live-link target does not expose a half-written OBJ;
- UI operations cannot persist a representation-sensitive v1 edit;
- hot reload retains the old document if correspondence would drop a reference.

## Alternatives rejected

- secure export with a legacy interactive preview;
- applying old edit operations after certified generation;
- leaving the legacy/compiler selector enabled but undocumented;
- loading a recipe and discarding unmatched references;
- deleting the live-link destination before rename;
- keeping the mixed demo as the default and silently falling back.

## Verification

Warnings-as-errors and MSVC static-analysis builds pass, followed by all 13
secure/corpus tests in both lanes.

Automated app screenshots prove a certified watertight box and cylinder render
with the secure-only controls. The cylinder live-link OBJ was written twice to
the same destination; both files had SHA-256
`41FD38903B2353C11540EEADBDED0CE8DB1B93716EA9E89DF53B956887677EBD`,
and the second atomic replacement succeeded.
An unsupported sphere displayed
`secure_pipeline.unsupported_curve_family` and rendered no mesh.

## Consequences

The main app, export dialog, and Blender file bridge now share the certified
floor. Editing is intentionally inspect-only until recipe v2 migration.
Structured modelling output, source/working overlays, validation heatmaps,
incremental secure caching, proxy timing, and other analytic families remain
open. Historical generator code remains compiled for frozen tests and the
remaining CLI diagnostic routes, but is not reachable from the app.
