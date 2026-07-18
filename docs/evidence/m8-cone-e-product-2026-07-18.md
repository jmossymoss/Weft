# M8 CONE-E product integration evidence - 2026-07-18

## Proven increment

Product routes admit certified apex-cone results through the existing M7
admission gates and still refuse unsupported families by name:

- CLI `weft mesh` exports certified OBJ for `cone` (27 verts / 50 tris under
  CLI defaults).
- App headless screenshot for `--fixture cone` writes a non-empty PNG via the
  secure certified pipeline.
- App `--fixture sphere` still produces an empty/refusal viewport path
  (`boundary.critical_segmentation_unsupported` / deferred residual).
- Secure cache invalidation machinery from WP-042 remains unchanged and covers
  family/settings keys.

## Artifacts

- `/opt/cursor/artifacts/screenshots/m8-cone-e-cone.png`
- `/opt/cursor/artifacts/screenshots/m8-cone-e-sphere-refusal.png`

## Commands

```bash
cmake --build --preset linux-gcc --target weft_app weft -j"$(nproc)"
build/linux-gcc/cli/weft fixture /tmp/cone_c.step --shape cone
build/linux-gcc/cli/weft mesh /tmp/cone_c.step -o /tmp/cone_e.obj
xvfb-run -a -s "-screen 0 1280x1024x24" \
  build/linux-gcc/app/weft_app --fixture cone \
  --screenshot /opt/cursor/artifacts/screenshots/m8-cone-e-cone.png
xvfb-run -a -s "-screen 0 1280x1024x24" \
  build/linux-gcc/app/weft_app --fixture sphere \
  --screenshot /opt/cursor/artifacts/screenshots/m8-cone-e-sphere-refusal.png
```

## Outcomes

CLI and app smokes succeeded as above on Cursor Cloud Linux / g++ 13.3 /
OCCT 7.6.3.

## Status boundary

WP-074 (CONE-E) is closed. CONE-F completes the family proof matrix.
