# M4/M8 certified coverage baseline - 2026-07-18

## Proven increment

Every frozen committed STEP subject and the built-in certified fixtures now
terminate as one of:

- `certified` — complete `MeshingResult` certificate
- `named_refusal` — stable `import.*` or `secure_pipeline.*` code
- `inspectable_only` — imported but not meshable (source/working inspect path)
- `import_refusal` — secure import throws a named `SecureImportError`

No subject is left unclassified. Totals reconcile:

```text
COVERAGE_TOTALS import_success=11 import_refusal=3 mesh_certified=5
mesh_named_refusal=5 mesh_inspectable_only=1 case_total=9 fixture_extra=5
```

### Gap groups (no status change)

| Group | Subjects | Notes |
|---|---|---|
| Plane/cylinder/hole certified | solid.box + fixtures box/cylinder/partial_cylinder/hole | Current automatic families |
| Unsupported curve families | nist_ctc_01, plasticity26* , fixture:sphere | Named `secure_pipeline.unsupported_curve_family` — M8 candidates (sphere/ellipse/etc.) |
| Transform loss refusals | mirrored/scaled mapped_item | Import BR-008 path |
| Corrupt syntax | corrupt.step.syntax_failure | Named transfer failure |
| Inspectable-only assembly | freecad_as1_autocad2000_ap214 | Imported, not meshable |

## Commands

```bash
ctest --preset linux-gcc -R secure_committed_step_corpus --output-on-failure
ctest --preset linux-gcc-static-analysis -R secure_committed_step_corpus \
  --output-on-failure
```

## Outcomes

Both Linux lanes passed.

## Status boundary

WP-050 closed. This is the measured baseline before M8 family expansion;
sphere/analytic freeform remain named refusals by design.
