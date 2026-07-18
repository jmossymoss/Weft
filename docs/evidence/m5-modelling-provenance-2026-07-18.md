# M5 modelling provenance evidence - 2026-07-18

## Proven increment

`ModelingMesh` now carries an explicit `ModelingProvenanceKind`
(`Absent`, `CertifiedFloorAlias`, `Independent`). Floor-alias results require
`aliasesCertified` plus a non-empty reason and must mirror certified
triangles/vertices. Independent claims require polygons and a non-vacuous
`independentValidation` certificate. `validateModelingProvenance` is enforced
in `generateSecureMesh`, and `makeCertifiedPolyMeshAdapter` reports the selected
output in `PolyMesh::selectedOutput`.

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
- Alias / unexplained-alias / uncertified-independent / absent fixtures behave
  as specified.
- Secure pipeline floor-alias results include `modeling.provenance` coverage.

## Status boundary

WP-031 is closed on Linux. Independent modelling polygons remain WP-032.
