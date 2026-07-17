# Secure-core evidence index

Evidence files record commands, exact revisions, outcomes, failures, and
artifacts for milestone gates. Evidence is append-only in meaning: a later run
may supersede a result but must not rewrite what an earlier run proved.

- `m0-legacy-baseline-2026-07-17.md` - pre-rewrite Release test baseline.
- `m0-strict-build-2026-07-17.md` - MSVC warnings-as-errors and static-analysis build evidence.
- `m0-parallel-test-isolation-2026-07-17.md` - process-isolated temporary artifacts and concurrent strict/static test evidence.
- `m1-secure-import-2026-07-17.md` - processing-disabled source/working import contract evidence.
- `m2-reconnaissance-2026-07-17.md` - total face/edge classification and planar projection evidence.
- `m3-interval-solver-2026-07-17.md` - exact equality/minimum/parity count assignment evidence.
- `m3-canonical-boundaries-2026-07-17.md` - atomic shared-edge samples, UV uses, and azimuth registration evidence.
- `m3-exact-predicates-2026-07-17.md` - exact dyadic orientation, incircle, and segment-relation evidence.
- `m3-planar-trim-validation-2026-07-17.md` - exact pre-CDT loop, intersection, containment, orientation, and coverage evidence.
- `m1-m2-frozen-corpus-2026-07-17.md` - frozen 106-record catalogue, committed STEP outcomes, and 77-fixture import/accounting evidence.
