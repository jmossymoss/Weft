# M9 package contents - 2026-07-18

## Tooling

`tools/package_linux.sh` builds:

```
dist/weft-linux/
  bin/weft
  bin/weft_app
  docs/{README,blocked-routes,milestones}.md
  licenses/NOTICE.txt
  SHA256SUMS
```

## Digests (example run 2026-07-18)

```
fc453a66267ddaaae96123c88587ec8c958da52621c55c785afd5cf400d5d0b6  ./bin/weft
f7fa9df5bb20c2b0acdfe354be5bdffdd89bd4b57383b2576723edfc85dc0d3e  ./bin/weft_app
```

(Full tree listed in package `SHA256SUMS`. Digests are rebuild-dependent.)

## Smoke

Packaged `weft fixture box` + `weft mesh` → 8 vertices / 12 tris.
