# WP6 known-red visual inspection (2026-07-22)

Viewport screenshots from `tools/visual_check.sh` (app `--finalize`, three
yaw/pitch views `_v0`/`_v1`/`_v2`). Reviewed in-agent against the WP6 visual
rubric classes and the cleared KNOWN_RED rows.

| Model | Rubric / gate | Verdict | Notes |
|---|---|---|---|
| `slitdrill` | watertight cylinder + slit | pass | Clean cylinder grids; slit is a shallow manifold pocket into the bore (fixture fix). App overlay: watertight, 0 open / 0 NM. |
| `tan_slit` | dirty tangent contact | pass (CLI CAD) | Intentional non-manifold contact. CLI CAD via visual_check: 0 open / 1 NM. App default path can show higher open/NM (e.g. 8/2) — do not treat that as a CAD-gate regression. |
| `fillet_capsule_iso_band` | FilletStrip / capsule | pass | Capsule fillets stay Coons-like bands (not RevolutionGrid). Orange perimeter = open-shell extract boundaries (CLI unexplained opens = 0). |
| `grip_freeform_panels` | Freeform grip ribbons | pass | Freeform faces keep Coons lattices under CAD/minimal (no residual planar n-gon grab). Boundary opens are extract artifacts. |
| `bullet_tip_3728` | tip / body rim | pass | Tip quad fill + body Coons share one rim; app and CLI both watertight. |
| `foam` | production editable | pass | Watertight CAD mesh; contract-floor faces present but editable. |
| `teleporter` | production editable | pass | Watertight CAD mesh; same floor tradeoff as foam. |
| `coons_plane_1805_r0` | freeformComb seam | residual | Local crack / open-edge family remains on seams (green/orange overlays). Class not fully closed; leads remaining MP9 opens. |
| `MP9` | assembly open edges | residual | App/CLI ~383 open, NM, and folded cells visible on receiver/rail junctions. Justifies remaining `KNOWN_RED` `max_open_edges` row. |

## Commands

```sh
mkdir -p /tmp/kr/visual_src
cp tests/fixtures/slitdrill.step tests/fixtures/tan_slit.step \
   tests/regressions/mp9/fillet_capsule_iso_band.step \
   tests/regressions/mp9/grip_freeform_panels.step \
   tests/STEP_Examples/bullet_tip_3728.step \
   tests/STEP_Examples/foam.step tests/STEP_Examples/teleporter.step \
   tests/regressions/mp9/coons_plane_1805_r0.step \
   tests/STEP_Examples/MP9.stp /tmp/kr/visual_src/
bash tools/visual_check.sh /tmp/kr/visual_src docs/evidence/wp6-visual
```

PNGs + `report.html` in this directory are the reviewed set. Full dump also under
`/opt/cursor/artifacts/wp6-known-red-visual/`.
