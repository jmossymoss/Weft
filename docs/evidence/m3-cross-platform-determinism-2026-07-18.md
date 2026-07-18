# M3 cross-platform determinism evidence - 2026-07-18

## Proven increment

Count, boundary, lift, and report digests for the planar box, capped cylinder,
and through-hole fixtures now match across repeated runs and across Windows
MSVC and Linux GCC. Digests are locale-independent FNV-1a over ordered
structural fields; topology fingerprints use the same hex encoding.

OCCT/libm circle evaluations that differed by one ULP between platforms were
collapsing planar CDT topology on cylinder caps. After discrepancy certificates
accept raw evaluations, canonical-boundary sample positions and UV lifts clear
the lowest three IEEE-754 mantissa bits so exact dyadic CDT inputs and mesh
fingerprints agree without changing certified envelopes.

Golden digests are locked in `tests/test_secure_meshing.cpp` and printed as
`WEFT_M3_DIGEST` lines.

| Fixture | counts | boundary | lifts | report |
|---|---|---|---|---|
| box | `4d4b56a97e4194a1` | `bfc6fa72b8fb0801` | `2d5083505e7bff41` | `1271fe5128c8b97e` |
| cylinder | `df476af694433848` | `8928e6e02ad2fa92` | `612aaa31fa6d5784` | `bf093b9b1ccb2bd0` |
| hole | `559a67e76d02c618` | `303cc08b7ef4e792` | `c362170936b054d8` | `fcd7679c64a6bca5` |

## Verification

Windows (`vs2022`, Release):

```text
cmake --build --preset vs2022 --target weft_secure_meshing_tests --config Release
ctest --preset vs2022 -C Release -R "secure_meshing|canonical_boundary|cylinder_template|planar_trim|interval_solver|geometric_predicate|planar_cdt|certified_mesh" --output-on-failure
```

9/9 passed. Digests matched the golden table.

Linux (`linux-gcc`):

```text
cmake --build --preset linux-gcc --target weft_secure_meshing_tests
ctest --test-dir build/linux-gcc -R "secure_meshing|canonical_boundary|cylinder_template|planar_trim|interval_solver|geometric_predicate|planar_cdt|certified_mesh" --output-on-failure
```

9/9 passed. Digests matched the same golden table bit-for-bit.

## Status boundary

WP-015 is closed. M3's ordered playbook packages WP-010–WP-015 are complete.
Additional coupled-sum classes remain behind named `interval.*` refusals
(BR-011) rather than an open M3 work package.
