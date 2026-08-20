if(MSVC)
  # PDBs even in Release so crash traces resolve core frames too.
  add_compile_options(/Zi)
  # OCCT on Windows pulls windows.h; without NOMINMAX the min/max macros
  # shatter std::min<T>/std::max<T> (MSVC: "type 'char' unexpected").
  add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

add_library(weft_core
  src/model.cpp
  src/analysis.cpp
  src/topology_cache.cpp
  src/mesh.cpp
  src/meshers.cpp
  src/mesher_sampling.cpp
  src/mesher_trace.cpp
  src/mesher_utils.cpp
  src/mesh_diagnostics.cpp
  src/edit.cpp
  src/recipe.cpp
  src/remap.cpp
  src/validate.cpp
  src/topology_signature.cpp
  src/export_fbx.cpp
  src/export_gltf.cpp
  src/viz.cpp
  src/fixture.cpp
  # --- new io subsystem (Stage A: registry + probe + XCAF B-rep import) ---
  src/io/format.cpp
  src/io/system.cpp
  src/io/probes.cpp
  src/io/bootstrap.cpp
  src/io/params.cpp
  src/io/occ_rollback.cpp
  src/io/xcaf.cpp
  src/io/step_reader.cpp
  src/io/iges_reader.cpp
  src/io/brep_reader.cpp
  # --- Stage B: writer registry (mesh + B-rep writers) ---
  src/io/mesh_writers.cpp
  src/io/occ_writers.cpp
)

target_include_directories(weft_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PUBLIC ${OpenCASCADE_INCLUDE_DIR}
)

# The public io registry API uses std::span, so consumers that include those
# headers must compile as C++20 too. Propagate the requirement through the
# target instead of making each CLI/app/test target rediscover it manually.
target_compile_features(weft_core PUBLIC cxx_std_20)

# OCCT 7.8 merged TKSTEP/TKSTEPBase/TKSTEPAttr/TKSTEP209 into TKDESTEP.
if(TARGET TKDESTEP)
  set(WEFT_OCCT_STEP TKDESTEP)
else()
  # StepRepr (entity names) lives in TKSTEPBase on pre-7.8 splits.
  set(WEFT_OCCT_STEP TKSTEP TKSTEPBase)
endif()

# XCAF / XDE document stack for the metadata-preserving importer (7.6 split
# names; 7.8 merged into TKDE*). This tree is 7.6.3 -> the else() branch.
if(TARGET TKDESTEP)                 # OCCT >= 7.8
  set(WEFT_OCCT_XDE  TKDESTEP TKDEIGES TKDE)
  set(WEFT_OCCT_XCAF TKXCAF TKLCAF TKCAF TKCDF TKBinXCAF TKBinL TKBin)
else()                              # OCCT 7.5-7.7 (this tree is 7.6.3)
  set(WEFT_OCCT_XDE  TKXDESTEP TKXDEIGES TKIGES)
  set(WEFT_OCCT_XCAF TKXCAF TKLCAF TKCAF TKCDF TKBinXCAF TKBinL TKBin)
endif()

# Mesh / tessellation IO toolkits. OCCT 7.8's Data Exchange refactor renamed
# these (TKSTL -> TKDESTL; TKRWMesh split into TKXMesh + TKDEGLTF/TKDEOBJ/
# TKDEPLY), so the old names are absent on >= 7.8 and the new names are absent
# on <= 7.7. Link whichever toolkits the installed OCCT actually provides so
# both trees build (this tree is 7.6.3 -> TKRWMesh + TKSTL).
set(WEFT_OCCT_MESHIO "")
foreach(_tk TKRWMesh TKSTL TKXMesh TKDESTL TKDEGLTF TKDEOBJ TKDEPLY)
  if(TARGET ${_tk})
    list(APPEND WEFT_OCCT_MESHIO ${_tk})
  endif()
endforeach()

target_link_libraries(weft_core PUBLIC
  TKernel TKMath
  TKBRep TKG2d TKG3d TKGeomBase
  TKTopAlgo TKGeomAlgo TKPrim TKBO TKBool TKFillet TKOffset TKShHealing TKMesh
  TKXSBase ${WEFT_OCCT_STEP}
  # --- new: XCAF/XDE document stack + mesh toolkits ---
  ${WEFT_OCCT_XCAF} ${WEFT_OCCT_XDE}
  ${WEFT_OCCT_MESHIO}   # RWStl/StlAPI + RWMesh/RWGltf/RWObj (convert + tessellation)
)
