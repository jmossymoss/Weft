# Independent mesh: zoo fixture visual pass (2026-08-21)

`weft mesh --independent --validate` on generated zoo fixtures.

## Classes fixed

1. Doubly-periodic torus used UV ear-clip and collapsed to a fan. It now
   takes a UV lattice; both closed-edge sample counts drive `nu`/`nv`.
2. Closed drums (cylinder, cone, elliptical tube, torus fillet ring) loft
   from shared rim samples so caps weld. Iso-band 4-sided fillets stay on
   exact-border transfinite.
3. Failed UV ear-clip no longer fans (that filled holes). OCCT is last resort.
4. Four-sided freeform interiors blend in 3D, not UV, so slabs do not fold.

## Closed zoo at this revision (CAD defaults)

Watertight, 0 winding: box, cylinder, cone, sphere, torus (1024 quads),
fillet, hole, plate_holes, ellipse_plate, boss, bossfillet, ribbon,
filletslot, bspline/bezier/offset slabs, extrusion, microedge, compound2.

Still open (trimmed drums / offset walls, not T0 exit): canrev, notched,
slitdrill, hairline (3), torture. Dirty-step opens unchanged (expected).

T0 unit tests: box, cylinder, hole, fillet, sphere, torus lattice,
ellipse tube, bezier-slab winding.
