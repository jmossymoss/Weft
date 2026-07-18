# M8 CUT-E evidence - 2026-07-18

## Proven increment

CLI OBJ + app screenshot m8-cut-e.png for hole.

## Commands

```bash
ctest --preset linux-gcc -R 'secure_core|secure_meshing' --output-on-failure
build/linux-gcc/cli/weft mesh hole.step -o hole.obj
```

## Outcomes

Passed on Linux gcc/static-analysis.

## Status boundary

Closed.
