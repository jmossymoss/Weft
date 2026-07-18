# M8 CUT-C evidence - 2026-07-18

## Proven increment

WEFT_CUT_C tris=280 fingerprint=efc0e9c351c0d80e for hole; slotted/drilled/filletslot remain named refusals.

## Commands

```bash
ctest --preset linux-gcc -R 'secure_core|secure_meshing' --output-on-failure
build/linux-gcc/cli/weft mesh hole.step -o hole.obj
```

## Outcomes

Passed on Linux gcc/static-analysis.

## Status boundary

Closed.
