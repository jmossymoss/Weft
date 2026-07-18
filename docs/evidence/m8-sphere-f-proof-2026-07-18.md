# M8 SPHERE-F family proof evidence - 2026-07-18

## Proven increment

Density sweep + repeat-run fingerprint determinism (hex addd54e3573dc6bb on OCCT 7.6.3 is evidence-only, not a cross-build golden); sphere is a supported automatic family. Torus/fillet remain.

## Commands

```bash
ctest --preset linux-gcc -R 'sphere_template|canonical_boundary|secure_meshing|secure_core' --output-on-failure
```

## Outcomes

Linux gcc lane passed for the sphere package suite.

## Status boundary

WP for this step is closed.
