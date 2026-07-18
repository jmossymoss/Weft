# M8 MAP-B canonical boundaries evidence - 2026-07-18

## Proven increment

Canonical boundaries now accept bounded `bspline`/`bezier` edges and
four-sided mapped candidate faces (`mapped.four_sided_candidate`) while
support remains deferred:

- Curve segmentation: line/circle unchanged; bspline/bezier use domain +
  contact critical events (no UV-period refinement yet).
- Non-line/circle p-curves on mapped faces skip UV-period refinement instead
  of refusing.
- Interval assignment admits bspline/bezier edges with
  `max(4, minimumClosedCurveSegments)`.

### Fixture

```text
WEFT_MAP_B boundaries=12 samples=60   # ribbon
```

## Commands

```bash
ctest --preset linux-gcc -R 'canonical_boundary|secure_core' --output-on-failure
```

## Outcomes

Linux gcc lane passed.

## Status boundary

WP-121 closed. MAP-C must add a certified mapped/Coons floor consumer.
