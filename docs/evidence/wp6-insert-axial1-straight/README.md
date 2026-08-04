# Insert drums at axial=1 stay straight columns (2026-08-04)

Artist instruction (demo face #34 / torture notched+window drum):

> Default test, still has these small edges on cylinder-revolve.
> I WANT NONE. JUST STRAIGHT EDGES.

With `axial=1`, long rulings must run rim-to-rim. A window/slot may
absorb its own sill and lintel as local n-gon side verts under the cutout
columns; those levels must not stamp a full-band ring around the drum.

## What went wrong

`meshRevolutionInsert` used the insert wire's v-extents as full-drum rows
so the staircase carve could close. At axial=1 that stamped two horizontal
rings through every column — the "small edges" in the viewport even when
the axial knob said 1.

A first fix applied those levels only under the slot (`levelBoxesOpt`).
That removed the rings, but the banded/unbanded transition n-gons sat
with their centroids inside the carve box. Keeping those n-gons (so the
staircase would stay closed) left the collar web fighting geometry that
still covered the hole → open insert-wall borders (10 opens on demo).
Deleting them opened the staircase and demoted the face to the contract
floor.

## Change

Keep local levels for axial=1, but expand the level u-boxes by one
circumferential pitch when subdividing columns. The carve still uses the
tight insert box. Transition n-gons therefore land outside the hole,
every covered cell under the slot is a deletable quad band, and the
collar welds to the insert walls.

## Result on `demo.step` face #34 (adapt off, axial=1)

| radial | opens | build | mid full-band rows |
| --- | --- | --- | --- |
| 19 | 0 | structured | 0 |
| 21 | 0 | structured | 0 |
| 32 | 0 | structured | 0 |
| 48 | 0 | structured | 0 |

CAD defaults (adaptive on) also keep the face on `RevolutionGrid` with
no full-band mid rows.

## Still open (separate classes)

- Raising radial on MP9 iso-band drums (e.g. `#374`) can still demote via
  `radial override stress → contract floor` — not the insert axial=1 ring
  class.
- Co-circular CAD arc splits treated as separate segments remain a
  chart/planning concern; not addressed here.

Reducer: `testInsertDrumAxialOneNoFullBandRings`.
