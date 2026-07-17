# ADR-0022: Separate XDE product structure from B-rep occurrences

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

An XDE STEP transfer carries two related structures: product definitions and
component occurrences with transforms, and the exact `TopoDS_Shape` graph used
for geometry. OCCT's aggregate `OneShape()` may insert compound containers for
assembly expansion. Counting those containers both as product occurrences and
as modelling topology duplicates structure, while indexing only unique OCCT
subshapes loses repeated and co-located occurrences. Binding components only
through solid IDs also loses legitimate free compounds, wires, edges, and
vertices.

The secure core needs one deterministic occurrence account without changing or
discarding the processing-disabled source shape. It must also refuse any
working copy whose product-to-topology correspondence cannot be proved.

## Decision

- Retain the complete processing-disabled `OneShape()` and every located XDE
  leaf use in `SourceBRep`.
- Represent XDE product definitions and expanded uses explicitly as
  `AssemblyRecord` and `InstanceRecord` objects with label identities,
  parentage, local/world transforms, and exact topology roots.
- Build the B-rep occurrence tree recursively through compound, compsolid,
  solid, shell, face, wire, edge, and vertex children. Give every occurrence a
  deterministic ID and retain its exact located shape, orientation, tolerance,
  transforms, parent/children, instance owner, and canonical underlying ID.
- Record one coedge for every edge use directly owned by a wire. A coedge keeps
  wire/edge/face/instance identity, order, orientation, and only the p-curve
  representations actually stored by OCCT.
- Bind an XDE leaf instance only through its exact located XDE shape use. Solid
  indices and geometric proximity are not provenance mechanisms. This rule
  applies equally to body and non-body leaves and to multiple free product
  roots.
- Keep the outer aggregate compound as the B-rep topology root. A non-root
  compound whose complete descendant body-occurrence set exactly matches a
  non-root XDE assembly subtree is treated as an XDE expansion container and
  represented by the assembly/instance graph instead of being counted twice.
  Leaf compounds and unmatched compounds remain ordinary B-rep occurrences.
- Use TShape identity within one transferred model and XDE definition/path
  identity across expanded instances to assign canonical underlying IDs. Hash
  tables are lookup accelerators only; traversal order remains the source of
  stable ordinals.
- Require independent topology-account validation and exact source/working
  occurrence correspondence before setting a repair certificate meshable.
  A deep or repaired working copy without that proof remains inspectable and
  receives a named refusal.

The compatibility `BRepSnapshot` maps remain available to application and
meshing migration code, but `TopologyAccount` is the authoritative occurrence
representation.

## Invariants

- the immutable source `TopoDS_Shape` is never rebuilt to create the account;
- every retained occurrence has one exact shape and one parent path, except the
  declared topology root;
- every wire edge use has exactly one ordered coedge record;
- repeated and co-located instances remain distinct occurrences;
- canonical underlying identity never replaces occurrence identity;
- every non-assembly XDE instance resolves to at least one exact topology root;
- assembly containers are separated only by exact XDE/body-occurrence evidence;
- no nearest-geometry match, body-only fallback, p-curve synthesis, welding, or
  partial-account success is permitted;
- a validator with expected evidence cannot pass without checking it;
- existing `StableIdKind` numeric values remain unchanged; new compound kinds
  are appended explicitly.

## Alternatives rejected

- using `TopExp::MapShapes` as the authoritative account, because it collapses
  repeated occurrences;
- treating every OCCT assembly expansion compound as modelling topology;
- removing all compounds, which would erase legitimate leaf/product topology;
- binding instances only through solids or shells;
- using centroid, tolerance, or other geometric-nearest matching;
- reconstructing missing p-curves while importing;
- allowing compatibility repair to mesh when exact occurrence correspondence
  is incomplete;
- retaining quadratic occurrence/instance scans for large production models.

## Verification

Commit `33ceff6` adds the account, validator, certificate integration, and
fixture-backed tests. The reviewed nested repeated-box STEP proves two assembly
definitions, eight expanded instances, 517 topology occurrences, 144 coedges,
and the expected 35 canonical entities. A generated fillet-junction STEP proves
that non-solid XDE leaves receive exact roots, and a dedicated XDE fixture
proves two independent free product roots.

Tampered accounts fail by name for empty evidence, duplicate IDs, missing exact
shapes, missing wire coedges, and missing instance roots. Conservative identity
imports pass exact source/working occurrence correspondence; the compatibility
deep copy refuses because no exact modified-occurrence certificate exists yet.
Strict MSVC and MSVC `/analyze` builds both compile the full product and pass all
14 registered tests.

## Consequences

M1 now has an authoritative exact identity account and deterministic refusal
floor, and M2 can classify against a total occurrence hierarchy. Bounded
conservative repairs still require history-backed occurrence mappings before
they can pass this contract. Selective XDE-container separation is observable
and reversible because the untouched source shape and label evidence remain in
the source snapshot.
