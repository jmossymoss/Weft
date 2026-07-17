# ADR-0002: Planar support without stored source p-curves

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Processing-disabled OCCT STEP import does not retain stored p-curves for many
planar faces; an ordinary box exhibits this on every coedge. Treating OCCT's
lazy computed projection as source evidence would violate the immutable import
contract, while refusing the simplest exact planar model would prevent the first
usable template.

## Decision

An exactly classified plane is projection-eligible without stored p-curves. Its
boundary 3D curves will be projected into the exact plane's stable 2D basis by
the planar template, and that mapping will be identified as derived working
evidence rather than a source p-curve.

Every supported curved face still requires stored face-specific p-curves,
`SameParameter` and `SameRange`, successful independent curve/p-curve/surface
evaluation, and measured agreement. No other family inherits the planar
exception.

## Invariants

- the source snapshot continues to report the planar p-curve as missing;
- planar projection is a named template mapping, not repair-certificate source
  evidence;
- a curved face missing any required stored mapping is not supported;
- unknown exact families remain named and unsupported.

## Alternatives rejected

- invoking OCCT's lazy planar p-curve computation and labelling it source data;
- synthesising p-curves during conservative repair;
- allowing every surface family to inverse-project boundary positions.

## Verification

`weft_secure_core_tests` proves that the processing-disabled box has missing
stored p-curves while all six exactly classified planes remain supported. The
cylinder fixture proves two planar caps plus one curved wall; the wall is
supported only after every stored curve-on-surface mapping evaluates.

## Consequences

Canonical-boundary construction must carry a mapping-kind/provenance field so a
stored p-curve and a derived planar projection can never be confused. Curved
families without complete p-curve evidence fail closed.
