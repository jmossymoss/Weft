# Secure-core evidence index

Evidence files record commands, exact revisions, outcomes, failures, and
artifacts for milestone gates. Evidence is append-only in meaning: a later run
may supersede a result but must not rewrite what an earlier run proved.

- `m0-legacy-baseline-2026-07-17.md` - pre-rewrite Release test baseline.
- `m0-strict-build-2026-07-17.md` - MSVC warnings-as-errors and static-analysis build evidence.
- `m1-secure-import-2026-07-17.md` - processing-disabled source/working import contract evidence.
- `m2-reconnaissance-2026-07-17.md` - total face/edge classification and planar projection evidence.
- `m3-interval-solver-2026-07-17.md` - exact equality/minimum/parity count assignment evidence.
