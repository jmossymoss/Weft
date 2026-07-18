# M8 app family matrix evidence - 2026-07-18

## Proven increment

CLI mesh + headless app screenshots for every supported fixture:

| Fixture | CLI mesh | App screenshot |
|---|---|---|
| box | 8v/12t | app-matrix/box.png |
| cylinder | 52v/100t | app-matrix/cylinder.png |
| partial_cylinder | 42v/80t | app-matrix/partial_cylinder.png |
| cone | 27v/50t | app-matrix/cone.png |
| sphere | 255v/506t | app-matrix/sphere.png |
| torus | 2304v/4608t | app-matrix/torus.png |
| fillet | 16v/28t | app-matrix/fillet.png |
| hole | 48v/96t | app-matrix/hole.png |
| mapped_patch | 289v/512t | app-matrix/mapped_patch.png |
| freeform_patch | 289v/512t | app-matrix/freeform_patch.png |

Artifacts under `/opt/cursor/artifacts/screenshots/app-matrix/`.

## Commands

```bash
for f in box cylinder partial_cylinder cone sphere torus fillet hole mapped_patch freeform_patch; do
  build/linux-gcc/cli/weft fixture /tmp/app_$f.step --shape $f
  build/linux-gcc/cli/weft mesh /tmp/app_$f.step -o /tmp/app_$f.obj
  xvfb-run -a -s "-screen 0 1280x1024x24" build/linux-gcc/app/weft_app --fixture $f \
    --screenshot /opt/cursor/artifacts/screenshots/app-matrix/$f.png
done
```

## Status

WP-150 closed.
