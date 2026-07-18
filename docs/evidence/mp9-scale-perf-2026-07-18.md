# Evidence — WP-176 MP9 scale/performance (2026-07-18)

## Measured wall times (Linux cloud, Release `linux-gcc`)

| Stage | Wall time |
|---|---|
| Import + recon inventory (MP9 ~4270 faces) | ~35–41 s |
| Face extract (single face) | ~35 s (full STEP transfer) |
| Mesh of extracted cylinder/freeform/offset | <1 s after extract |

Import progress (`WEFT_IMPORT_PROGRESS`) shows no multi-minute stalls after
the conservative tolerance-fallback and recon O(N²) fixes.

## Memory

Compatibility profile refuses >2000 faces before deep-copy
(`import.repair.compatibility_scale_refused`) — no OOM crash path.

## Exit gate

Bounded import/recon times documented; no pathological hang on inventory.
Full-body mesh time is gated on WP-177 (still refused under ADR-0014 while
~841 surfaces remain deferred).
