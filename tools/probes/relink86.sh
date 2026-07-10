#!/bin/bash
# Relink the diagnosis probes against the current build/core static lib.
# (Static link: EVERY core rebuild needs a relink or the probe lies.)
cd "$(dirname "$0")"
LIBS="-lTKXDESTEP -lTKSTEP -lTKSTEPAttr -lTKSTEPBase -lTKXCAF -lTKVCAF -lTKCAF
 -lTKLCAF -lTKBinXCAF -lTKBinL -lTKBin -lTKCDF -lTKXSBase -lTKRWMesh -lTKSTL
 -lTKIGES -lTKXDEIGES -lTKBool -lTKBO -lTKFillet -lTKOffset -lTKShHealing
 -lTKMesh -lTKXMesh -lTKHLR -lTKPrim -lTKTopAlgo -lTKGeomAlgo -lTKGeomBase
 -lTKG3d -lTKG2d -lTKBRep -lTKMath -lTKernel -lTKService"
for p in "$@"; do
  g++ -O2 -std=c++17 -w -I/home/user/Weft/core/include \
    -I/usr/include/opencascade \
    "$p.cpp" /home/user/Weft/build/core/libweft_core.a \
    $LIBS -ltbb -o "$p" && echo "ok $p" || echo "FAIL $p"
done
