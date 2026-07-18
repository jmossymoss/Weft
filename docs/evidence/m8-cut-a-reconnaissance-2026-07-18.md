# M8 CUT-A evidence - 2026-07-18

## Proven increment

Tagged cutout.planar_perforated (annulus planes) and cutout.cylindrical_bore; hole fixture WEFT_CUT_A planar_perforated=2 cylindrical_bore=1.

## Commands

```bash
ctest --preset linux-gcc -R 'secure_core|secure_meshing' --output-on-failure
build/linux-gcc/cli/weft mesh hole.step -o hole.obj
```

## Outcomes

Passed on Linux gcc/static-analysis.

## Status boundary

Closed.
