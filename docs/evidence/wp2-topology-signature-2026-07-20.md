# WP2 evidence — cross-platform topology signature (2026-07-20)

## Goal

Add the machine-comparable topology signature required by
`docs/EXECUTION_PLAN.md` §3.2 / WP2 so Linux and Windows can assert policy
equality without requiring byte-identical OBJ text or floating-point
formatting.

## What was added

| Piece | Role |
| --- | --- |
| `weft::formatTopologySignature` | Stable `weft.topology_signature.v1` text |
| `weft::topologySignaturesEqual` | Policy compare (ignores `info.*`) |
| CLI `--signature FILE` | Emit while running `mesh` / `validate` |
| CLI `signature-compare` | Exit 0/1 policy compare |
| `tools/topology_signature.sh` | emit / compare / self-check / fixture-set |
| `testTopologySignature` + ctest `topology_signature` | Same-platform determinism |
| Linux CI after corpus_gate | Writes `build/topology_signatures/` artifact |

No new `MesherKind`. No filename or face-ID product special cases.
`weft::generate()` remains the sole production mesh path.

## Signature contents (policy lines)

- Polygon arity: `vertices`, `polygons`, `quads`, `tris`, `ngons`
- Validity / connectivity: `open_edges`, `non_manifold_edges`,
  `winding_conflicts`, `degenerate_polygons`, `sliver_polygons`,
  `folded_polygons`, `watertight`
- Face routing: `kind.<mesher-name>=count`, `face.<id>.kind`,
  `face.<id>.build` (+ `.cause` when demoted)
- Raw / empty / floor: counts and sorted face-id lists
- Edge contracts: `edge.<id>.div`
- Anchors (optional summary, not raw XYZ): counts, per-face histogram,
  `anchors.uv_qhash` at `anchors.uv_tol=1e-6`

Informational (`info.*`, ignored by compare): input basename, deviation
floats, open-on-boundary breakdown, tolerance note.

## How to compare

```sh
tools/topology_signature.sh emit a.step -o a.sig
tools/topology_signature.sh emit b.step -o b.sig   # or Windows twin of a
tools/topology_signature.sh compare a.sig b.sig    # exit 0 = policy-equal
```

Cross-platform MVP evidence: Linux CI uploads the small fixture-set artifact;
Windows can download and `compare` later. Same-platform smoke:
`tools/topology_signature.sh self-check` (cylinder, box, torture ×2).

## Constraints honored

- Authoritative path remains `weft::generate()` via CLI `mesh`
- `--stitch` untouched / still quarantined
- `tests/fixtures/interop/` not modified
- Golden counts not refreshed; validity checks not weakened
