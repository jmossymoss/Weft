add_library(weft_core
  src/model.cpp
  src/analysis.cpp
  src/mesh.cpp
  src/meshers.cpp
  src/edit.cpp
  src/recipe.cpp
  src/fixture.cpp
)

target_include_directories(weft_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(weft_core PUBLIC
  TKernel TKMath
  TKBRep TKG2d TKG3d TKGeomBase
  TKTopAlgo TKGeomAlgo TKPrim TKBO TKFillet TKShHealing TKMesh
  TKXSBase TKSTEP
)
