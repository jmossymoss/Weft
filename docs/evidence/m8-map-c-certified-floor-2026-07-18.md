# M8 MAP-C evidence - 2026-07-18

## Proven increment

buildMappedFourSidedPatch UV grid; mapped_patch fixture WEFT_MAP_C tris=512 fingerprint=667cb25dd465ece4; single-face open shells relax closed-manifold assembly.

## Commands

```bash
ctest --preset linux-gcc -R mapped_template --output-on-failure
build/linux-gcc/cli/weft mesh mapped_patch.step -o out.obj
```

## Outcomes

Passed on Linux gcc.

## Status boundary

Closed.
