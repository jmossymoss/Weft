# M1 face-adjacency orientation repair evidence - 2026-07-17

## Proven increment

This increment implements the orientation repair contract fixed by the
reconnaissance decision boundary (see ADR-0026 and
`m1-orientation-reconnaissance-2026-07-17.md`). The conservative derivation
now repairs a solid whose single closed shell violates shared-edge parity by:

- collecting every composed face use and every non-degenerate shared-edge
  use of the shell;
- requiring a closed two-manifold: exactly two uses per edge, two-sided
  occurrence orientations only, no repeated face definitions, and a
  connected two-colourable adjacency graph;
- solving the parity system, unique up to one global sign;
- selecting the global polarity independently: each coherent assignment is
  materialized as a trial solid, and the accepted one must have a finite
  positive signed volume, classify the infinite point outside, and pass
  native `BRepCheck` validity;
- rebuilding only occurrence orientations with the fewest flips (face
  occurrences, or the shell occurrence plus the complement), leaving
  TShapes, child order, geometry, p-curves, tolerances, flags, locations,
  and topology cardinality untouched;
- recording a `ShellOrientationRepair` certificate entry with non-vacuous
  manifold-edge coverage and the numeric polarity evidence.

`buildImportedModel` re-audits every certified repair against the
authoritative snapshots: stored occurrence orientations must differ at
exactly the certified subjects, and the working solid must independently
re-prove positive volume and an outside infinite point. Occurrence
correspondence admits an exact FORWARD/REVERSED inversion only for certified
flip subjects. Every detected-but-unproved candidate refuses by a stable
named code, emits `import.repair.face_orientation_unproven`, and forces the
import non-meshable.

## Positive and refusal witnesses

The reviewed native witness `corrupt.orientation.inverted_shell_face.brep`
(source signed volume `-5120 mm^3`, `BRepCheck_BadOrientationOfSubshape`)
imports through the public `importBRepSecure` API and proves:

- source invalid and unchanged; working valid and meshable;
- exactly one shell orientation repair: the shell occurrence reversal plus
  one face occurrence flip — the exact inverse of the two reviewed
  topology-token reversals;
- twelve of twelve manifold edges in the parity proof, the full recovered
  baseline-box volume `+7680 mm^3` (the incoherent source integrates to
  `-5120 mm^3` because the inverted face contribution flips sign), and the
  infinite point outside;
- no tolerance, stored-p-curve, parameterization, or topology-cardinality
  changes; complete one-to-one `Modified` correspondence for every
  occurrence;
- the stored source shell/face orientations are bit-unchanged while the
  working copy differs at exactly the two certified occurrences.

Adversarial witnesses authored in-test from the reviewed box baseline (the
frozen fixture snapshot is untouched) prove:

- an open incoherent shell refuses as `repair.orientation.shell_open`,
  stays an identity copy, and is non-meshable with the named diagnostic;
- a wire with one reversed edge occurrence produces an unsolvable parity
  system and refuses as `repair.orientation.non_orientable` with no flips;
- a structurally applied repair whose certificate is tampered to
  `checkedManifoldEdges=0` fails `import.repair.validation_incomplete` and
  cannot be meshed despite a valid working shape.

Identity imports are unaffected: coherent shells are never candidates, the
new `repair.face_adjacency_orientation` evidence row is complete with zero
expected units, and the parity scan adds no geometric cost.

## Frozen-corpus platform defect repaired in passing

The frozen catalogue gate failed on a fresh Linux checkout before any source
change: `licences/freecad-lgpl-2.1.txt` was committed with LF bytes while
`manifest.json` pins the reviewed CRLF bytes
(`7ffe1954587c77dfba1cf8eb9b2ea743671fa6e63f9e7a2f258119d42e14eefe`), so the
gate only passed on Windows working copies that still carried CRLF. The
committed file was restored to the exact reviewed CRLF bytes; the manifest
was not touched, and a full digest sweep confirms this was the only bound
file whose bytes disagreed with the frozen record. `git diff --check` flags
only that restoration (the CR bytes are the pinned content); every other
changed file is clean.

## Verification runs

Development and verification for this increment ran in a Linux container
(the only toolchain available to the session), against a from-source OCCT
8.0.0 (`V8_0_0`, Draw/Visualization off) with `WEFT_WARNINGS_AS_ERRORS=OFF`:

```text
cmake -B build/linux-dev -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DWEFT_WARNINGS_AS_ERRORS=OFF -DOpenCASCADE_DIR=/opt/occt8/lib/cmake/opencascade
cmake --build build/linux-dev -j4
LD_LIBRARY_PATH=/opt/occt8/lib ctest --test-dir build/linux-dev --output-on-failure
```

Outcomes:

- the full product (core, CLI, desktop app, and every test target)
  compiled; the only warnings are the pre-existing
  `-Wmissing-field-initializers` set already recorded as the deferred
  Linux proof;
- all 14 registered tests passed, and a second complete clean rerun also
  passed 14/14;
- the direct generated-corpus rerun reported: 77 imported, 59 meshable, 18
  inspectable-only, 1,559 classified subjects, all identity-certified —
  unchanged by the new repair stage, proving coherent shells are never
  candidates;
- the direct secure-core rerun printed `secure-core contract checks
  passed`.

The Windows strict MSVC and MSVC `/analyze` product lanes were NOT run in
this session and remain the active development gate; this evidence is
partial until those lanes rerun clean on Windows. No Linux-lane source
change was made per the current Windows-first decision.

## Status boundary

This closes only the face-adjacency orientation slice of M1. M1 remains
`IN_PROGRESS`. Coherent-but-inverted polarity normalization (which needs a
signed-volume sweep over every imported solid), multi-shell and
shared-definition orientation scopes, free-standing shells, bounded
tolerance reconciliation, provable one-to-one sewing, copy-on-write
representation rules, product STEP repair witnesses, explicit native-unit
resolution, secure IGES import, and complete compatibility correspondence
are still open. This evidence does not mark M1 passed.
