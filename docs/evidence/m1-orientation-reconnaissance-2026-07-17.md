# M1 orientation-repair reconnaissance - 2026-07-17

## Question

Could a conservative orientation repair simply reverse a solid whenever its
signed volume is negative or the infinite point classifies as inside?

## Reviewed witness and result

The reviewed native artifact
`corrupt.orientation.inverted_shell_face.brep` changes only the clean box
shell occurrence and one face occurrence. It retains every geometry and
tolerance byte. Its independent oracle records signed volume `-5120 mm^3` and
`BRepCheck_BadOrientationOfSubshape`.

An OCCT DRAW experiment reproduced:

```text
source root:          SOLID FORWARD
source signed volume: -5120
infinite point:       IN
source check:         BRepCheck_BadOrientationOfSubshape

after root reverse:
signed volume:        5120
infinite point:       OUT
shape check:          BRepCheck_BadOrientationOfSubshape
```

OCCT 8.0 `BRepLib::OrientClosedSolid` performs the infinite-point
classification and reverses only the solid occurrence when the infinite point
is inside. It does not solve inconsistent face orientation within the shell.
See the [OCCT BRepLib contract](https://dev.opencascade.org/doc/refman/html/class_b_rep_lib.html)
and [OCCT 8.0 implementation](https://github.com/Open-Cascade-SAS/OCCT/blob/V8_0_0/src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib.cxx#L2087-L2102).

## Decision boundary

Whole-solid reversal is blocked as a general orientation repair. Positive
signed volume and an outside infinite point are necessary polarity evidence
for a simple closed solid, but they do not prove face-adjacency coherence.

The next orientation increment must establish a two-manifold face-adjacency
parity solution, reject non-orientable/open/non-manifold shells, select global
polarity independently, rebuild only occurrence orientations, preserve exact
geometry/tolerances/topology cardinality, and pass native validity plus
source/working correspondence. No orientation code was changed in this
increment, and M1 remains `IN_PROGRESS`.
