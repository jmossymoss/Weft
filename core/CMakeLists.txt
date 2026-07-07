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

# The io subsystem's registry API uses std::span (Mayo-faithful signatures);
# the repo otherwise defaults to C++17. Require C++20 for weft_core's own
# translation units. Kept PRIVATE: Stage A callers (cli/app/tests) do not
# include the io headers, so their standard is unchanged.
target_compile_features(weft_core PRIVATE cxx_std_20)

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

target_link_libraries(weft_core PUBLIC
  TKernel TKMath
  TKBRep TKG2d TKG3d TKGeomBase
  TKTopAlgo TKGeomAlgo TKPrim TKBO TKFillet TKShHealing TKMesh
  TKXSBase ${WEFT_OCCT_STEP}
  # --- new: XCAF/XDE document stack + mesh toolkits ---
  ${WEFT_OCCT_XCAF} ${WEFT_OCCT_XDE}
  TKRWMesh   # RWMesh/RWGltf/RWObj (used by convert/default tessellation in later stages)
  TKSTL      # RWStl + StlAPI
)
