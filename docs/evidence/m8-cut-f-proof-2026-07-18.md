# M8 CUT-F evidence - 2026-07-18

## Proven increment

Circular through-hole cutout subclass supported; complex multi-bore/slot cut-graphs deferred.

## Commands

```bash
ctest --preset linux-gcc -R 'secure_core|secure_meshing' --output-on-failure
build/linux-gcc/cli/weft mesh hole.step -o hole.obj
```

## Outcomes

Passed on Linux gcc/static-analysis.

## Status boundary

Closed.
