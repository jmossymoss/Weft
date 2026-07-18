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

## Windows

BLOCKED in this environment: no Windows host/agent available.
Exact missing proof: empty-dir `cmake --preset windows-*` (or CI windows job)
producing `weft.exe` / `weft_app.exe` with OCCT DLL deploy from a fresh checkout.
