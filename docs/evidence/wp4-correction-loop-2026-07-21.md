# WP4 — artist correction loop (2026-07-21)

Active package closed on this revision train. Exit evidence for section 3.3
and the WP4 package gate.

## Exit checklist

| Criterion | Evidence |
| --- | --- |
| Load STEP → face control → regenerate → save/reload recipe → export | `testArtistCorrectionWorkflow` |
| Constrained edits survive density when anchors remain valid | `testConstrainedEditSurvivesDensityChange` |
| CAD remap reports dropped decisions; world-space welds survive | `testRemapDropsLostOpsKeepsWeld`, identity remap in workflow test |
| Export / live-link use finalized mesh | App: `finalizeMesh = forceFinalize \|\| liveLink`; export via `finalizedMeshForExport`; workflow test sets `finalizeMesh` |
| Documented controls for a new user | README “Artist correction loop” |
| Corrections cannot bypass release geometry gate | Workflow boss density override + CLI `weft mesh … --recipe` with loop op stays watertight |

## Changes (cumulative)

1. **WeldVerts remap** — world-space; no longer dropped when `faceId == 0`.
2. **`applyOps` report** — `{applied, failed}`; app status on failures.
3. **Fail-closed recording** — weld / dissolve / fill / bridge probe before
   appending to the recipe.
4. **Live-link finalization** — regen finalizes while live-link is on.
5. **Semantic selected-face knobs** — RingJunction `rect u` / `rect v`;
   DiskCap wheel no longer nudges inert axial; PlateWeb `collar rings` /
   hole share seed; QuadFill deviation always visible; annulus/rail labels
   match the wheel HUD.
6. **§3.3 e2e test** — `testArtistCorrectionWorkflow`.

## CLI spot check (this machine)

```text
weft mesh tests/fixtures/generated/cylinder.step --recipe <loop op> --validate
  → watertight yes; 36 quads after loop replay
weft mesh tests/fixtures/generated/boss.step --validate
  → watertight yes
./build/tests/weft_tests → all checks passed
```

## Active package

WP4 exit criteria pass on this revision. Active work package advanced to
WP5 — validate real work.
