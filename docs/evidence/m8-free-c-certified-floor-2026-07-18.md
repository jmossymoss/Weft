# M8 FREE-C evidence - 2026-07-18 (five-edge)

Five-sided UV-border freeform_patch certifies through the UV-grid floor.

Root cause of the prior block: half-edges inherited full-side interval counts
(16), so samples landed at half-cell UV offsets and collapsed onto wrong grid
stations (split-rail corner identity / 3D disagree). Fix: UV-span-aligned
interval demands for mapped/freeform edges + same-corner multi-sample attach.

```
WEFT_FREE_C tris=512 fingerprint=c819649b2ff26230
WEFT_FREE_C wider_refusal=mapped.seam_sample_unmatched
```

mapped_patch fingerprint remains `c6d2c3643ca44049`.
