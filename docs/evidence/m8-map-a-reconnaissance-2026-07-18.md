# M8 MAP-A reconnaissance evidence - 2026-07-18

## Proven increment

Reconnaissance tags non-plane mapped-family faces by boundary cardinality:

- `mapped.four_sided_candidate` — `bspline` / `bezier` / `extrusion` /
  `revolution` with exactly four edges (Coons-ready)
- `mapped.non_four_sided_deferred` — same families with other edge counts

Support remains `deferred_residual_surface` until MAP-C.

### Fixture outcomes

```text
WEFT_MAP_A four_sided_candidates=6 non_four_sided=0   # ribbon
WEFT_MAP_A ribbonnotch_non_four_sided_bspline=1       # 8-edge bspline faces
```

Ribbon: 4 extrusion + 2 bspline four-sided faces. Ribbonnotch: notched
bspline faces tagged non-four-sided.

## Commands

```bash
ctest --preset linux-gcc -R '^secure_core$' --output-on-failure
ctest --preset linux-gcc-static-analysis -R '^secure_core$' --output-on-failure
```

## Outcomes

Both Linux lanes passed.

## Status boundary

WP-120 closed. MAP-B must build four-rail boundaries for candidates.
