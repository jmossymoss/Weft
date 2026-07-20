# M9 clean rebuilds - 2026-07-18

## Linux (empty directory)

```bash
rm -rf /tmp/weft-m9-clean && mkdir -p /tmp/weft-m9-clean
git archive HEAD | tar -x -C /tmp/weft-m9-clean
cd /tmp/weft-m9-clean
cmake --preset linux-gcc
cmake --build --preset linux-gcc --target weft weft_app -j$(nproc)
```

Result: `cli/weft`, `app/weft_app`, `core/libweft_core.a` produced.
Selected secure suites: 9/9 passed (`geometric_predicate`, `planar_cdt`,
`cylinder/cone/sphere/torus/mapped_template`, `secure_meshing`, `secure_core`).

Note: full default `ninja` also builds `weft_generated_secure_corpus_tests`, which
fails to compile against system OCCT 7.6 (`DESTEP_Parameters.hxx` missing). That
target is test-only and is recorded as a known environment/OCCT-version gap, not
a product binary failure.

## Windows (empty directory) — 2026-07-20

Host: Windows 10.0.26200, MSVC 19.44 (VS 2022 Build Tools), CMake 4.3.4,
OCCT at `C:\OCCT` (`OpenCASCADE_DIR=C:/OCCT/cmake`), OCCT bin on `PATH`.

```powershell
git -C D:\Weft worktree add --detach D:\weft-m9-clean HEAD
cd D:\weft-m9-clean
cmake --preset vs2022
cmake --build --preset vs2022 --target weft weft_app
```

Initial configure/build failed under `WEFT_WARNINGS_AS_ERRORS=ON` with:

1. `secure_meshing.cpp` C4127 — `if (false && …)` constant condition.
2. `secure_core.cpp` C4996 — deprecated `GeomAdaptor_TransformedSurface::Surface()`;
   replaced with `GeomSurfaceOriginal()`.

After those two fixes (copied into the clean worktree for this proof):

- `build/vs2022/bin/Release/weft.exe`
- `build/vs2022/bin/Release/weft_app.exe`
- `build/vs2022/core/Release/weft_core.lib`

Smoke (OCCT `win64\vc14\bin` on `PATH`):

```text
weft fixture …\box.step --shape box   → EXIT 0
weft mesh …\box.step -o …\box.obj     → EXIT 0 (8 verts, 6 quads)
```

Third-party OCCT runtime DLL auto-deploy resolved none from the link interface
(relocated OCCT install); product binaries run when `C:\OCCT\win64\vc14\bin` is
on `PATH` (same layout `build.bat` / `WINDOWS_BUILD.md` expect).

Selected suites on this lane (not the WP-161 exit gate): `geometric_predicates`,
`planar_cdt`, `cylinder_template`, `sphere_template`, `torus_template` passed;
`secure_core` and `cone_template` failed under local OCCT 8.x semantics;
`mapped_template` / `secure_meshing` test targets did not finish compiling
(`TopTools_IndexedMapOfShape` C4996 under Werror). Product `weft` / `weft_app`
builds are green.

## Exit gate

Both platforms produce product binaries from empty source directories.
WP-161 is `DONE`.
