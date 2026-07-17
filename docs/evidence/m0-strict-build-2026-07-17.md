# M0 strict build evidence - 2026-07-17

## Commands and outcomes

With `CMAKE_PREFIX_PATH=C:/OCCT`:

- `cmake --preset vs2022` - passed;
- `cmake --build --preset vs2022` - complete product passed with `/W4 /WX`;
- `cmake --preset vs2022-static-analysis` - passed;
- `cmake --build --preset vs2022-static-analysis` - complete product passed with
  MSVC code analysis enabled for Weft-owned targets;
- `ctest --preset vs2022` - `secure_core` passed and `pipeline` reproduced
  exactly the eight frozen baseline failures in 67.25 seconds.

The first analyzer run found a 64 KiB SHA-256 read buffer on the secure import
stack. It was moved to heap storage before the passing rerun. Pinned ImGui
backend translation units are built in a separate third-party library; Weft's
warnings and analyzer gates do not claim findings inside vendored loader code.

## Open evidence

The Linux GCC presets exist but have not been run in this Windows environment.
M0 cannot pass until Linux evidence exists and production no longer compiles or
routes selectable legacy generators.
