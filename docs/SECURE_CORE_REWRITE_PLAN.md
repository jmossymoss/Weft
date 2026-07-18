# Weft secure-core rewrite

Status authority: `docs/governance/milestones.md`
Execution order: `docs/governance/agent-execution-playbook.md`

This plan replaces the production geometry core while keeping Weft as the
product repository and preserving its artist-facing workflows. Research
references and the artist-facing acceptance measurements remain in
`MESHING_RESEARCH_PLAN.md`.

## Non-negotiable architecture

1. Preserve an immutable, processing-disabled source B-rep.
2. Mesh only a separately identified working B-rep.
3. Record every repair in a source-to-working correspondence certificate.
4. Give every topological edge one immutable sample sequence, including
   per-coedge UV uses and measured curve-on-surface discrepancies.
5. Produce a boundary-exact, intersection-checked triangle mesh as the
   authoritative safety floor.
6. Build analytic and quad-dominant modeling topology above that floor.
7. Never use welding, missing faces, silent healing, or OCCT triangle soup as
   evidence that a result is correct.
8. Fail by stable diagnostic code when an invariant cannot be proved.

`Secure` means geometric correctness, provenance, deterministic failure, and
protection from silent representation loss. It does not mean cybersecurity
hardening.

## Public direction

The secure core introduces:

- `ImportedModel`: immutable source B-rep, derived working B-rep,
  correspondence, repair certificate, and diagnostics;
- `GeometryEvaluator`: the only exact curve, p-curve, and surface evaluation
  interface;
- `CanonicalBoundary`: one edge-owned sample sequence with every owning
  coedge's UV mapping;
- `MeshingResult`: certified mesh, modeling mesh, generation report, and a
  non-vacuous validation certificate;
- recipe schema 2: source-referenced settings and manual operations with
  explicit migration conflicts.

The conservative repair profile is the default. It may normalize orientation,
reconcile already-present parameter representations within their evidence
envelope, sew only when one-to-one provenance remains provable, and apply
bounded tolerance changes. P-curve synthesis, splits, merges, unrestricted
`ShapeFix`, or dropped faces require an explicit compatibility profile and a
complete certificate.

## Milestone order

- **M0** baseline, governance, build matrix, and removal of legacy production
  routing;
- **M1** immutable source import, audited working copy, correspondence, and
  evaluator contracts;
- **M2** total topology/geometry reconnaissance and logical regions;
- **M3** robust predicates, interval solving, periodic lifting, and canonical
  boundaries;
- **M4** certified constrained-Delaunay safety floor and critical regions;
- **M5** independent, non-vacuous validation;
- **M6** first usable planar and cylinder templates;
- **M7** recipe, CLI, app, editing, exporter, hot-reload, and Blender workflow
  restoration;
- **M8** research-backed sizing, primitive, connector, cut-graph, and
  freeform-field expansion;
- **M9** dependency licensing, reproducible packaging, and release gate.

No later milestone can be accepted ahead of its prerequisites. Experimental
work may run in parallel but does not enter production routing early.

## Required gates

- zero unmatched shared-edge sample IDs;
- zero authoritative weld passes;
- zero unexplained open/non-manifold edges or T-junctions;
- zero trim-containment violations, foldovers, or triangle intersections;
- chord and normal error within the request;
- complete source and working provenance for every output element;
- validation reports with expected/checked/skipped/failed counts and no
  vacuous passes;
- deterministic report and topology digests on Windows and Linux;
- source defects reported separately from meshing defects.

Certified triangles are a valid result when structured topology cannot be
proved. A body whose safety floor cannot be proved remains inspectable but is
not exportable.

## Evidence and dependency policy

The committed `D:/weftocct` corpus and contracts are copied and adapted into
Weft; that repository is never a runtime dependency. Uncommitted files in
either repository remain user-owned and are not copied or staged.

CGAL 6.2 may be used behind narrow predicate/CDT/intersection interfaces during
local research. Prototype binaries are not distributable until a commercial
license is obtained or those components are replaced with permissive
implementations. Shewchuk's public-domain adaptive predicates are the intended
predicate replacement foundation.

## Working discipline

Work lands in small, fixture-backed increments. Every gate requires a complete
clean rerun and an adversarial self-review. Interrupted or partial verification
is recorded as partial evidence and never promoted to a pass.

Agents follow the execution playbook rather than creating additional roadmaps,
handoff files, or investigation diaries. Durable decisions belong in ADRs;
completed verification belongs in the evidence index.
