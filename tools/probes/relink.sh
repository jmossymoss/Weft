#!/bin/bash
# Relink all probes against the current core lib.
for p in "$@"; do
  g++ -O2 -std=c++17 -I/home/user/Weft/core/include -I/usr/include/opencascade \
    $p.cpp /home/user/Weft/build-int/core/libweft_core.a \
    -lTKernel -lTKMath -lTKBRep -lTKG2d -lTKG3d -lTKGeomBase -lTKTopAlgo \
    -lTKGeomAlgo -lTKPrim -lTKBO -lTKFillet -lTKShHealing -lTKMesh \
    -lTKXSBase -lTKSTEP -lTKSTEPBase -o $p 2>/dev/null && echo "ok $p" || echo "FAIL $p"
done
