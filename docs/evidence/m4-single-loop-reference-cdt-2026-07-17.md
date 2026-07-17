# M4 single-loop reference CDT evidence - 2026-07-17

## Proven increment

The core now exposes `PlanarCdtBackend` and compiles a distribution-safe
single-loop reference implementation. It validates trims exactly, constructs a
boundary-preserving ear triangulation, applies deterministic exact-incircle
Lawson flips to unconstrained convex internal edges, and returns no mesh until
an independent certificate pass succeeds.

Vertices retain canonical sample/index plus source/working edge provenance.
Every triangle retains source/working face provenance. No geometric point is
introduced and no weld, OCCT triangulation, epsilon sign, or CGAL dependency is
used.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_planar_cdt_tests --config Release
ctest --preset vs2022 -R "planar_trim_validation|planar_cdt" --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_planar_cdt_tests --config Release
ctest --preset vs2022-static-analysis -R "planar_trim_validation|planar_cdt" --output-on-failure
```

Both lanes passed. The battery proves:

- exact triangle and cocircular-tie behaviour;
- replacement of a known illegal quadrilateral diagonal by an exact Lawson
  flip and a passing local-Delaunay certificate;
- concave-domain triangulation;
- deterministic success over 12 collinear boundary-density levels;
- two identical topology results for each of 64 star-shaped domains;
- a valid subnormal-coordinate square;
- complete provenance, `n-2` cardinality, orientation, boundary identity,
  manifold incidence, all mesh-edge relations, and local-Delaunay evidence;
- stable refusal of invalid trim input, a valid domain with a hole, and an
  unavailable exact predicate backend.

Both complete MSVC build presets also passed. A full strict CTest rerun passed
all nine secure/corpus tests and reproduced exactly the eight frozen legacy
`pipeline` assertions (five MP9, two flaregun, one foam), with no new failure.
The static-analysis tree then passed all nine secure/corpus tests together.

## Status boundary

M4 is now in progress, not passed. This backend accepts one simple outer loop
only and is not routed into production. Holes, constraint recovery/cut graphs,
face/coedge assembly, adaptive surface error proof, critical-region/BVH
protection, shared-index body assembly, full triangle/triangle checks, Linux
determinism, and corpus-wide certified meshes remain open.
