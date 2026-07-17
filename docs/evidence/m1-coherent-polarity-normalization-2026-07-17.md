# M1 coherent-shell polarity normalization evidence - 2026-07-17

## Proven increment

ADR-0027 closes the second orientation defect family from the
reconnaissance decision boundary: a parity-coherent single-shell solid that
is inside out as a whole. Detection is one
`BRepClass3d_SolidClassifier::PerformInfinitePoint` classification of the
solid as stored; only an `IN` verdict makes the solid a candidate, and
acceptance still runs the full independent ADR-0026 proof chain
(trial-materialized assignments, positive finite signed volume, outside
infinite point, native validity, minimal occurrence flips, certificate
re-audit, correspondence admission at certified subjects only).

The load-bearing discovery, now pinned by an explicit test assertion: OCCT
`BRepCheck_Analyzer` accepts an inside-out coherent solid as valid. Before
this increment such models imported as valid, identity, meshable working
copies with inward normals; native validity therefore can never gate this
defect, which is why detection and acceptance both use independent
geometric evidence.

## Witnesses

In-test witnesses authored from the reviewed box baseline (frozen snapshot
untouched):

- reversed shell occurrence: repaired by exactly one shell-occurrence flip,
  zero face flips, twelve of twelve manifold edges, recovered `+7680 mm^3`,
  infinite point outside — and the repaired working digest is
  byte-identical to the untouched baseline import's working digest, proving
  exact restoration;
- uniformly reversed face occurrences: the minimal realization is the same
  single shell reversal (zero face flips);
- reversed solid occurrence: detectable but out of bounded scope — refuses
  as `repair.orientation.solid_occurrence_not_forward`, stays an identity
  copy, non-meshable, with the named diagnostic.

## Corpus outcome

In the 77-fixture generated corpus,
`solid.orientation.inverted_shell_face.step` round-trips its reversed shell
through STEP faithfully and is now a certified `Modified` repair: one shell
reversal, zero face flips, `+3840 mm^3` (the 20x16x12 pathology box),
infinite point outside, complete correspondence, still meshable. The
meshable/inspectable split stays 59/18 because the fixture was previously
meshable inside out — the corpus contract now asserts this fixture's exact
repair record and asserts identity plus empty repair lists for the other
76. All 1,559 reconnaissance subjects classify unchanged.

## Verification runs

Linux container (dev vehicle only), OCCT 8.0.0 from source,
`WEFT_WARNINGS_AS_ERRORS=OFF`:

```text
cmake --build build/linux-dev -j4
LD_LIBRARY_PATH=/opt/occt8/lib ctest --test-dir build/linux-dev --output-on-failure
```

All 14 registered tests passed, including the extended secure-core battery
and the updated generated-corpus contract. The Windows strict MSVC and MSVC
`/analyze` product lanes were NOT run in this session and remain the active
development gate; this evidence is partial until they rerun clean.

## Status boundary

M1 remains `IN_PROGRESS`. Reversed solid occurrences, multi-shell solids,
and shared shell definitions remain named orientation refusals; bounded
tolerance reconciliation, provable sewing, copy-on-write representation
rules, product STEP repair witnesses, explicit native-unit resolution,
secure IGES import, and complete compatibility correspondence remain open.
