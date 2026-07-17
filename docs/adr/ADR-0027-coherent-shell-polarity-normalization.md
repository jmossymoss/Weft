# ADR-0027: Coherent-shell polarity normalization

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

ADR-0026 repairs shells whose face adjacency violates shared-edge parity,
and deliberately deferred the complementary defect: a parity-coherent shell
that is inside out as a whole. That defect is dangerous precisely because
OCCT native validity does not catch it — `BRepCheck_Analyzer` accepts a
solid whose shell occurrence is reversed or whose faces are uniformly
inverted, so such a model previously imported as a valid, identity,
meshable working copy with inward normals. The generated corpus contained a
live example: `solid.orientation.inverted_shell_face` round-trips its
reversed shell through STEP faithfully and was silently meshable inside
out.

## Decision

- Extend the ADR-0026 candidacy: a single-shell solid whose parity scan is
  coherent becomes a repair candidate when one
  `BRepClass3d_SolidClassifier::PerformInfinitePoint` classification of the
  solid as stored answers `IN`. The classification is detection evidence
  only; acceptance still requires the independent trial-polarity proof
  (positive finite signed volume, outside infinite point, native validity
  on the materialized assignment). `ON`/`UNKNOWN` verdicts and
  classification failures leave the solid untouched.
- The repair path, minimal occurrence-flip realization, certificate
  entry, re-audit, correspondence admission, and named refusals are the
  unchanged ADR-0026 machinery. For a uniformly inverted shell the minimal
  realization is the single shell-occurrence reversal with zero face flips.
- A non-forward solid occurrence remains out of repair scope, but detection
  still runs for it so an inside-out solid behind a reversed solid
  occurrence refuses as
  `repair.orientation.solid_occurrence_not_forward` instead of being
  skipped silently.
- The detection cost is one infinite-point classification per in-scope
  parity-coherent solid on every conservative import. This is accepted as
  the price of never shipping an inside-out working copy; no volume
  integration runs during detection.

## Invariants

- native validity is never the polarity oracle: detection uses point
  classification, acceptance uses the independent trial measurements, and
  the certificate audit re-derives both on the working solid;
- a coherent outward shell is never a candidate and keeps its identity
  certificate and byte-identical digest;
- a repaired inverted shell restores the exact pristine stored
  orientations when the defect was a single reversed occurrence — the
  working digest equals the untouched baseline's working digest;
- every unproved candidate refuses by the established named codes and
  forces the import non-meshable.

## Alternatives rejected

- keying detection on `BRepCheck` invalidity (it accepts inside-out
  coherent solids, so the defect would stay invisible);
- computing signed volumes of every solid during detection (strictly more
  expensive than point classification with no added soundness, since
  acceptance re-proves volume anyway);
- normalizing reversed solid occurrences in the same increment (requires
  mutating compound child lists; deferred with a named refusal instead);
- treating `ON`/`UNKNOWN` classifications as candidates (would let an
  unstable classification drive a repair).

## Verification

In-test witnesses derived from the reviewed box baseline prove: a reversed
shell occurrence is repaired by exactly one shell-occurrence flip with
twelve of twelve manifold edges proved, recovered `+7680 mm^3`, and a
working digest byte-identical to the untouched baseline import; uniformly
reversed face occurrences reduce to the same single shell reversal; a
reversed solid occurrence refuses by name as an identity copy. The
`BRepCheck` acceptance of the inside-out source is asserted explicitly to
pin why native validity cannot gate this defect.

In the 77-fixture generated corpus, `solid.orientation.inverted_shell_face`
now imports as a certified `Modified` repair — one shell reversal, zero
face flips, `+3840 mm^3`, infinite point outside — and remains meshable,
with every other fixture keeping its exact identity certificate and the
59/18 meshable split unchanged.

## Consequences

Both working-copy orientation defect families fixed by the reconnaissance
decision boundary are now repairable with complete proof chains, and the
corpus no longer contains a silently inside-out meshable import. Multi-shell
solids, shared shell definitions, and reversed solid occurrences remain
named refusals for future increments. M1 remains in progress.
