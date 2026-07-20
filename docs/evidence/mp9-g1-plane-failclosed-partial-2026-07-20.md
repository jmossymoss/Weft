# G1 plane hard-trim fail-closed (partial) — 2026-07-20

## Goal

Remove plane `allowCurvedUv` soften paths; multi-outer / self-intersect /
hole-bridge must certify or refuse with stable `plane.*` / `cdt.*` codes.

## Done in this session

1. Plane trim validation always runs with `allowCurvedUv=false`.
2. Self-intersecting plane UV loops refuse
   `plane.self_intersecting_unmeshable` (no curved-UV recovery).
3. Collapsed plane loops refuse `plane.loop_collapsed_unmeshable` (no pad-fan).
4. Plane CDT failures no longer retry via UV-fan + `relaxGeometryChecks`.
5. Multi-outer recursive CDT no longer forces `allowCurvedUv=true`.
6. Nesting proofs remain on for planes (`validatePlanarTrimDomain`).

## Local verification

| Case | Result |
|---|---|
| `plane_multi.step` / `plane_1793.step` | EXIT 0 |
| fixtures `hole`, `drilled`, `boss`, `slotted` | EXIT 0 |
| `planar_cdt` / `planar_trim_*` ctests | PASS |
| Full `secure_meshing` suite | FAIL only on pre-existing `testApexCone` (`import_not_meshable` under OCCT 8.x) |

## Blocker — MP9 body ACCESS_VIOLATION

```
weft mesh tests/STEP_Examples/MP9.stp -o … --progress
```

Process exits with Windows `0xC0000005` (ACCESS_VIOLATION) after progress
reaches approximately `face 136/4270` (bspline). No OBJ written. Isolated
`--faces` extracts around 134–145 do not reproduce the crash (most refuse
`import_not_meshable` as single-face transfers; a few mesh OK).

Exact next action: reproduce under a debugger on the full-body path at the
face after 136, capture the faulting stack, commit an extract of the
offending face+ring, then either certify or named-refuse without soft paths.

## Not yet proven (G1 acceptance remaining)

- All 75 historically deferred planes mesh or refuse with extract fixtures.
- `cdt.hole_bridge_not_found` root fix with denser consistent samples.
- MP9 EXIT 0 under `omitDeferredResiduals=false` without crash/soft omit.
