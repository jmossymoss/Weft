# ADR-0013: Certified full-cylinder floor

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

The certified planar floor cannot complete a common capped cylinder because
the periodic wall needs shared rim registration, per-triangle UV lifts, and
curved-surface error evidence. Treating its two rims as unrelated face-local
sequences recreates the historical twisted-wall failure.

## Decision

`buildFullCylinderWall` accepts only a proven supported, full-periodic
cylindrical face with exactly two canonical circular rims. It:

1. requires equal non-trivial rim counts and registers them through
   `azimuthRegistration`;
2. consumes every face/coedge/p-curve use represented by the one-band result;
3. refuses additional axial seam samples until certified interior-vertex
   provenance exists;
4. carries one periodic covering-space UV triple per triangle;
5. uses exact orientation predicates for every triangle;
6. proves each segment's cylinder sagitta at the analytic midpoint against the
   requested chord bound;
7. checks the planar strip-facet normal against exact surface normals at all
   four segment corners;
8. returns the same boundary-exact face-product contract consumed by the
   no-weld body assembler.

The body assembler is exposed as `assembleCertifiedBoundaryMesh`; the original
planar name remains as a compatibility wrapper.

## Invariants

- both caps and the wall consume the same canonical rim indices;
- no seam or rim vertex is welded or spatially matched;
- a count mismatch, failed registration, unconsumed seam sample, chord excess,
  normal excess, or degenerate UV triangle returns no wall;
- face input order cannot change the certified body fingerprint;
- planar triangles may use vertex UVs, while curved triangles explicitly own
  their periodic corner lifts.

## Alternatives rejected

- zipping rims by transient edge or sample order without registration;
- duplicating the periodic seam;
- dropping extra seam samples;
- validating curved geometry by vertices alone;
- accepting a visual cylinder while chord or normal evidence is empty;
- routing through the historical revolution mesher.

## Verification

The generated capped-cylinder fixture uses 64 samples on each rim and one
axial interval. It produces 128 shared vertices and 252 certified triangles:
128 on the wall and 62 on each CDT cap. The closed body passes complete
provenance, exact surface re-evaluation, incidence, opposite winding, and
fingerprint checks. Reversing face-product order preserves the fingerprint.

Adversarial cases require named refusal for a reflected/twisted rim sequence,
32/64 mismatched rims, two axial seam intervals, insufficient chord and normal
tolerances, a missing p-curve representation in provenance, a wrong periodic
corner lift, and a planar face passed to the cylinder template.

Both warnings-as-errors and MSVC static-analysis suites cover the change.

## Consequences

M4 now has its first curved closed-body proof and M6 has its first cylinder
family increment. Partial cylinders, certified interior axial rings, modelling
quads/cap topology, mismatched reference frames beyond cyclic registration,
workflow routing, 3D self-intersection, and Linux determinism remain open.
