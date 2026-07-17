# M3 canonical endpoint identity evidence - 2026-07-17

## Proven increment

Canonical endpoint samples now use one immutable exact B-rep vertex coordinate
after proving each incident curve endpoint lies within the scaled source
edge-plus-vertex tolerance envelope. The source/working map and immutable
snapshot now include solids, wires, and vertices in addition to faces and
edges. Every face-side tolerance envelope at an endpoint includes the source
vertex tolerance.

## Verification

The secure import, canonical boundary, full-cylinder, frozen committed STEP,
and 77-generated-fixture lanes pass in both MSVC configurations. Canonical
tests require bit-identical positions for every repeated canonical index and
non-zero complete vertex/curve comparison counts.

## Status boundary

This proves endpoint position identity for mapped topology. It does not yet
complete assembly/instance or coedge-occurrence correspondence, authorize an
unbounded compatibility repair, or replace source tolerance with a mesh-space
weld.
