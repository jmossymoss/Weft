# Evidence — MP9 import timing after large-repair skips (2026-07-19)

## Before
- inventory.import.done ≈ 35258 ms

## After (skip validity + orientation + shape validity for faces>2000 / edges>5000)
- inventory.import.done ≈ 14741 ms
- inventory.reconnaissance.done ≈ 20453 ms total
- meshable=1 retained

## Remaining import hotspots (importProgress clock)
- identity copy + param loop: ~50 ms
- correspondence → evaluators: ~2 s
- build_imported_model.done: ~3.5 s
- Additional ~11 s outside importProgress (STEP read / inventory wrap)

## Next
- Faster STEP/XCAF read
- Parallel recon / mesh face pool
- Warm certificate cache for ms regenerate
