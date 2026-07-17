# ADR-0014: Atomic secure meshing orchestration

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

The secure import, canonical boundaries, planar CDT, cylinder wall, and body
assembler were individually proven but only composed inside tests. Product
routing cannot safely adopt the rewrite until one public operation owns stage
ordering and guarantees that a later failure cannot leak an earlier partial
face set.

## Decision

`generateSecureMesh` is the first atomic orchestration entry point. It:

1. accepts only an audited `ImportedModel`;
2. runs total reconnaissance;
3. derives line and circle interval demands from immutable evaluator data and
   the analytic chord/normal count solver;
4. adds equality constraints between both rims of every full cylinder;
5. constructs the complete canonical boundary set;
6. routes planar faces, including direct holes, through exact trim assembly and
   CDT;
7. routes proven full periodic cylinders through the registered certified wall;
8. refuses every other curve or surface family by stable code;
9. assembles the complete expected face set without welding;
10. aggregates repair, reconnaissance, count, boundary, trim, CDT, cylinder,
    and body evidence into one non-vacuous `ValidationCertificate`;
11. returns a safe-floor `MeshingResult` only after that combined certificate
    is complete.

There is no legacy mesher, OCCT triangle soup, partial-shell, or face-omission
fallback inside this operation.

## Invariants

- one failed stage returns no `MeshingResult`;
- every working face is expected exactly once at body assembly;
- automatic interval counts cannot be below the analytic curve demand;
- full-cylinder rim equality is solved before sampling;
- unsupported geometry remains inspectable but cannot be mislabeled meshable;
- the final validation certificate includes every component certificate used
  to justify the result;
- repeated generation over the same imported model has the same topology
  fingerprint.

## Alternatives rejected

- letting callers compose stages and forget a validator;
- returning successfully meshed faces beside a failed face;
- silently calling historical generators for unsupported families;
- deriving circular density from a fixed radial default;
- checking only the final edge incidence while discarding stage evidence;
- routing product workflows before the public secure operation exists.

## Verification

The end-to-end battery certifies a planar box, a full capped cylinder twice with
an identical fingerprint, and the connected planar/cylindrical through-hole
fixture. The through-hole path exercises perforated planar CDT, a registered
cylindrical bore, shared rim identities, and closed body incidence together.

A sphere returns no result through a named unsupported route. Invalid sampling
configuration returns the exact interval configuration failure rather than
continuing with defaults.

Both warnings-as-errors and MSVC static-analysis lanes cover the operation.

## Consequences

The core now has one production-shaped secure generation API for its proven
family subset. CLI, app, recipe, exporter, and Blender routing are still M0/M7
work; they must consume this operation rather than recreate its stages. Partial
cylinders, interior axial refinement, other analytic families, modelling
templates, Linux determinism, and 3D self-intersection remain open.
