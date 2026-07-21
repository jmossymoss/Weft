# WP4 — artist correction loop (2026-07-21)

Active package: WP4. This note records the first correction-loop closure
slice after WP3 merge (`c2b3584`).

## Changes

1. **WeldVerts remap** — world-space ops (`WeldVerts`, with `DeletePoly` /
   `DissolveLoop`) no longer fall through the face-id remap path. A
   `faceId == 0` weld is kept; face-anchored ops that lose their feature
   still increment `opsDropped`.
2. **`applyOps` report** — returns `{applied, failed}` so silent no-ops are
   visible. App regen logs failures and sets status when any op fails.
3. **Fail-closed recording** — weld / dissolve / fill / bridge probe the
   live mesh before appending to the recipe; unsupported edits do not
   corrupt the recipe.
4. **Live-link finalization** — while live-link is on, regen sets
   `finalizeMesh = true` so Blender receives the export mesh (§3.3).
5. **Semantic density labels** — RingJunction knobs expose "around ring" /
   "along axis" instead of raw grid u/v.

## Tests added

- `testConstrainedEditSurvivesDensityChange` — loop insert + nudge, bump
  radial, regenerate + `applyOps`, assert watertight / validate / anchors.
- `testRemapDropsLostOpsKeepsWeld` — lost face-anchored nudge reports
  `opsDropped >= 1`; WeldVerts survives remap.

## Follow-ups in this slice

- Selected-face density labels aligned with the wheel HUD (annulus loop
  verts, rail density, ring around/along, plate loop-share seed).
- README artist correction-loop walkthrough under Interactive app.

## Remaining for WP4 exit

- Broader selected-face settings audit for any remaining inert knobs.
- Confirm a full artist dry-run (import → correct → undo → save → remap →
  export → Blender) without engineer intervention.
