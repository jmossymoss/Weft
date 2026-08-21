#!/usr/bin/env bash
# Cloud Agent install for Weft: system dependencies + a Release build.
# Idempotent and non-interactive so it is safe to re-run and to bake into an
# environment snapshot. Mirrors the Linux CI dependency set in
# .github/workflows/ci.yml.
set -euo pipefail
cd "$(dirname "$0")/.."

# OpenCASCADE (core), GLFW/OpenGL + fontconfig (weft_app), TBB, and the C++
# toolchain. Same package list the Linux CI job installs.
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake ninja-build \
  libocct-foundation-dev libocct-modeling-data-dev \
  libocct-modeling-algorithms-dev libocct-data-exchange-dev \
  libocct-ocaf-dev libocct-visualization-dev \
  libglfw3-dev libgl1-mesa-dev libtbb-dev libfontconfig1-dev

# Configure + build (Release). Pin gcc/g++: the base image's default c++
# alternative is clang, whose driver looks for a libstdc++ dev tree that is not
# installed, so linking fails with "cannot find -lstdc++". gcc-13 ships a
# working libstdc++-13-dev, and gcc matches CI.
CC=gcc CXX=g++ cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"

echo "Weft install complete: CLI at build/cli/weft, app at build/app/weft_app"
