# G1: digon plane 2732 certified recovery — 2026-07-20

## Goal

MP9 body EXIT 0 with `omitDeferredResiduals=false` was blocked by
`plane.loop_collapsed_unmeshable` on face 2732. Pad-fan / `allowCurvedUv`
remain forbidden; implement certified digon recovery.

## Diagnosis (`plane_2732.step`)

- Topology: 1 plane, 2 line edges, shared vertices (digon / thin strip).
- P-curves: parallel `DIRECTION(0,-1)` at distinct U origins
  (`u≈0` vs `u≈-7.68e-3`) — a thin UV parallelogram.
- Product line sampling used `lineSegmentCount()==1` (endpoints only).
- Endpoint lifts often share one UV after 3D identity; without interiors the
  assembled loop collapses to &lt;3 stations → named refuse.

## Recovery (certified)

1. `secure_meshing.cpp`: edges of two-edge plane faces get **4** line
   intervals (interior samples) so each parallel p-curve contributes UV
   stations along the strip.
2. `planar_trim_assembly.cpp`: for plane wires with exactly two coedges,
   when the same canonical vertex carries distinct UV, **keep both UV
   corners** (do not merge / do not pad). Closing likewise retains the
   digon strip’s four corners when UV disagrees.

`allowCurvedUv` stays false; nesting/orientation proofs still run.

## Proof

```
weft mesh tests/fixtures/mp9_extracts/plane_2732.step -o %TEMP%\p2732.obj
→ EXIT 0; 8 vertices, 4 polygons (2 quads, 2 tris)

ctest --preset vs2022 -R "planar_cdt|planar_trim"
→ PASS; WEFT_G1_PLANE2732 tris=6
```

## Body

Fail-closed MP9 (`omitDeferredResiduals` default false), frozen binary
`build/weft_mp9_012338.exe`, log `build/mp9_iso_012338.err`:

```
WEFT_PROGRESS face 2732/4270 id=2732 family=plane ms=176540
WEFT_PROGRESS face 2733/4270 id=2733 family=plane ms=176545
… progressed …
WEFT_PROGRESS face 3605/4270 id=3605 family=plane ms=193097
error: secure meshing refused [cdt.ear_clipping_stalled]
  subjects=[6:3605]
```

Body advanced past 2732. Next fail-closed refuse is perforated plane 3605
(`cdt.ear_clipping_stalled`; extract `plane_3605.step` already locked).

## Preserved

- G3 `edgeTris` copy-before-iterate AV fix
- No-fan-on-perforated-planes (3605 still `cdt.ear_clipping_stalled`)
- G2 cylinder / G3 mapped / identity_mesh_allowed paths untouched in intent
  (only digon line-count policy added in `secure_meshing.cpp`)
