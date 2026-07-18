# M8 MAP-E evidence - 2026-07-18

## Proven increment

CLI OBJ 289v/512t; app screenshot m8-map-e.png for mapped_patch.

## Commands

```bash
ctest --preset linux-gcc -R mapped_template --output-on-failure
build/linux-gcc/cli/weft mesh mapped_patch.step -o out.obj
```

## Outcomes

Passed on Linux gcc.

## Status boundary

Closed.
