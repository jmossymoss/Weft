# mp9_Edited outliner objects

Captured with:

```sh
xvfb-run -a build/app/weft_app tests/STEP_Examples/mp9_Edited.stp \
  --finalize --screenshot-objects <dir>
```

Total solids: **65**. PNGs: `objects/object_XXX_v{0,1}.png`.

| # | B-rep faces (outliner) | Mesh polys | quads | tris | n-gons |
|--:|-----------------------:|-----------:|------:|-----:|-------:|
| 1 | 130 | 1367 | 1189 | 101 | 77 |
| 2 | 159 | 599 | 487 | 16 | 96 |
| 3 | 10 | 220 | 212 | 4 | 4 |
| 4 | 61 | 130 | 100 | 2 | 28 |
| 5 | 33 | 313 | 269 | 4 | 40 |
| 6 | 12 | 42 | 26 | 2 | 14 |
| 7 | 12 | 42 | 26 | 2 | 14 |
| 8 | 304 | 1184 | 913 | 66 | 205 |
| 9 | 868 | 3594 | 2992 | 167 | 435 |
| 10–65 | see gallery | see `VISUAL_QA.md` + OBJ `o object_N` | | | |

Mesh arity from CAD-profile export of the full model (`o object_N` groups).
