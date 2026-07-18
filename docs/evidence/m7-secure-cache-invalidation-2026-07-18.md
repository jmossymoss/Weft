# M7 secure cache invalidation evidence - 2026-07-18

## Proven increment

- `SecureCacheKey` + `lookupSecureCache` require matching source/recipe/settings/
  implementation/certificate fingerprints and a still-admissible cached
  `MeshingResult`.
- Named refusals: `cache.key_mismatch`, `cache.implementation_mismatch`,
  `cache.certificate_mismatch`, `cache.entry_corrupt`.
- App async generation bumps `generationEpoch` and admits worker results only
  when the epoch still matches (`admission.stale_generation` otherwise).

## Commands

```bash
ctest --preset linux-gcc -R secure_meshing --output-on-failure
ctest --preset linux-gcc-static-analysis -R secure_meshing --output-on-failure
```

## Outcomes

Both Linux lanes passed after rebuilding the updated tests.

## Status boundary

WP-042 closed on Linux. Full face-closure dependency invalidation and proxy
GPU cache tying remain available as follow-on hardening inside M7.
