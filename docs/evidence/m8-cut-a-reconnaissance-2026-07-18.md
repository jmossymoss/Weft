# M8 CUT-A evidence - 2026-07-18 (reopened)

## Proven increment

- `cutout.planar_perforated`, `cutout.cylindrical_bore`, `cutout.planar_slotted`
- Deferred: `cutout.multi_bore_cylinder_deferred`, `cutout.filleted_slot_deferred`
- Fixture `plate_slot` (rectangular through-slot)

## WEFT_CUT_A

```
fixture=hole planar_perforated=2 cylindrical_bore=1 mesh_ok tris=96
fixture=plate_slot planar_perforated=2 planar_slotted=2 mesh_ok tris=32
fixture=slotted multi_bore_deferred=6 refusal=secure_pipeline.cylinder_rims_unresolved
fixture=drilled multi_bore_deferred=3 refusal=secure_pipeline.cylinder_rims_unresolved
fixture=filletslot multi_bore_deferred=4 filleted_slot_deferred=14 refusal=cylinder.rim_uv_missing
```

## Status

Closed after audit reopen.
