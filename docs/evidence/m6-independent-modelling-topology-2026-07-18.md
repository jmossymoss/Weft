# M6 independent modelling topology evidence - 2026-07-18

## Proven increment

`tryBuildIndependentModelingMesh` pairs certified triangles that share an edge
into modelling quads (residuals remain triangles), marks
`ModelingProvenanceKind::Independent`, and attaches a non-vacuous
`independentValidation` certificate. `generateSecureMesh` prefers this
independent modelling mesh when pairing succeeds while leaving the certified
triangle floor unchanged. Floor-alias remains the fallback when pairing cannot
prove any quads.

## Environment

- Host: Cursor Cloud Linux / g++ 13.3 / OCCT 7.6.3
- Presets: `linux-gcc`, `linux-gcc-static-analysis`

## Commands

```bash
ctest --preset linux-gcc -R 'certified_mesh|secure_meshing' --output-on-failure
ctest --preset linux-gcc-static-analysis -R 'certified_mesh|secure_meshing' \
  --output-on-failure
```

## Outcomes

- Both lanes passed.
- Box secure results use Independent modelling provenance with paired quads.
- Certified triangle counts/fingerprints remain authoritative for export
  adapters.

## Status boundary

WP-032 is closed on Linux for the planar/cylinder baselines exercised by the
secure suite. Density-sweep and tampered-ancestry expansion remain available as
follow-on hardening. M6 remains `IN_PROGRESS` until the full ledger gate is
reviewed.
