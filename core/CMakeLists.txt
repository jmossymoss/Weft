if(MSVC)
  # PDBs even in Release so crash traces resolve core frames too.
  add_compile_options(/Zi)
endif()

add_library(weft_core
  src/model.cpp
  src/analysis.cpp
  src/mesh.cpp
  src/meshers.cpp
  src/edit.cpp
  src/recipe.cpp
  src/remap.cpp
  src/validate.cpp
  src/export_gltf.cpp
  src/viz.cpp
  src/fixture.cpp
)

target_include_directories(weft_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCASCADE_INCLUDE_DIR}
)

# OCCT 7.8 merged TKSTEP/TKSTEPBase/TKSTEPAttr/TKSTEP209 into TKDESTEP.
if(TARGET TKDESTEP)
  set(WEFT_OCCT_STEP TKDESTEP)
else()
  # StepRepr (entity names) lives in TKSTEPBase on pre-7.8 splits.
  set(WEFT_OCCT_STEP TKSTEP TKSTEPBase)
endif()

target_link_libraries(weft_core PUBLIC
  TKernel TKMath
  TKBRep TKG2d TKG3d TKGeomBase
  TKTopAlgo TKGeomAlgo TKPrim TKBO TKFillet TKShHealing TKMesh
  TKXSBase ${WEFT_OCCT_STEP}
)
