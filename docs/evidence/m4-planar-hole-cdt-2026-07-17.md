# M4 planar-hole CDT evidence - 2026-07-17

## Proven increment

The exact Lawson reference backend now triangulates directly contained planar
holes through exact-predicate visibility bridges. Bridge endpoints are repeated
only in the ear-cut walk; output vertices remain the unique canonical input
vertices. Original outer and hole edges remain immutable constraints, and the
artificial bridge may be improved by exact Lawson flips.

Domain validation now also rejects canonical vertex or boundary sample identity
reused across separate loops.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_geometric_predicate_tests weft_planar_trim_validation_tests weft_planar_cdt_tests weft_planar_trim_assembly_tests weft_certified_mesh_tests --config Release
ctest --preset vs2022 -R "geometric_predicates|planar_trim_validation|planar_trim_assembly|planar_cdt|certified_mesh" --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_geometric_predicate_tests weft_planar_trim_validation_tests weft_planar_cdt_tests weft_planar_trim_assembly_tests weft_certified_mesh_tests --config Release
ctest --preset vs2022-static-analysis -R "geometric_predicates|planar_trim_validation|planar_trim_assembly|planar_cdt|certified_mesh" --output-on-failure
cmake --build --preset vs2022 --config Release
cmake --build --preset vs2022-static-analysis --config Release
ctest --preset vs2022 -E "^pipeline$" --output-on-failure
ctest --preset vs2022-static-analysis -E "^pipeline$" --output-on-failure
```

The covered perforated domains are:

- a four-vertex outer plus four-vertex hole: eight unique vertices and eight
  certified triangles;
- two rectangular holes: 12 unique vertices and 14 certified triangles;
- the same two holes in reversed caller order with identical triangle ordering;
- one hole inside a concave eight-vertex U-shaped outer loop;
- a generated STEP through-hole face assembled from real outer/hole wires and
  canonical circular/linear samples.

Every successful result completes boundary identity, triangle orientation,
edge-pair intersection, incidence, and local-Delaunay evidence. The real
perforated face also passes certified source/working provenance and exact planar
surface re-evaluation when explicitly treated as an open face product; the same
partial product refuses closed-manifold certification.

Both full MSVC build lanes completed. All 11 secure-core and frozen-corpus
tests passed in both lanes. Bridge ranking uses exact dyadic squared-distance
comparison, including a subnormal witness whose ordinary double square
underflows to zero; no `long double` topology choice remains.

The complete strict CTest run also reproduced the frozen legacy baseline:
all 11 secure tests passed and the isolated historical `pipeline` test reported
the same eight pre-rewrite failures. Those failures are evidence, not a secure
gate acceptance.

## Status boundary

M4 remains in progress. This proves direct planar holes for the covered
topologies, not an all-input cut-graph completeness theorem. Nested islands,
touching constraints, curved/perforated surfaces, critical-region protection,
adaptive error bounds, 3D triangle intersections, complete bodies beyond the
planar box, Linux determinism, and production routing remain open.
