cmake_minimum_required(VERSION 3.25)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(CADMesherFixtureGenerator LANGUAGES CXX)

  option(CAD_MESHER_WARNINGS_AS_ERRORS "Treat generator warnings as errors" ON)
  option(CAD_MESHER_ENABLE_MSVC_ANALYZE "Compile with MSVC /analyze" OFF)

  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../cmake")
  include(ProjectOptions)

  find_package(
    OpenCASCADE CONFIG REQUIRED
    COMPONENTS FoundationClasses ModelingData ModelingAlgorithms Visualization
               ApplicationFramework DataExchange)
endif()

include(OcctRuntime)

foreach(required_target IN ITEMS TKBRep TKTopAlgo TKPrim TKBO TKGeomAlgo TKFillet TKOffset TKDESTEP
                                 TKXCAF)
  if(NOT TARGET ${required_target})
    message(FATAL_ERROR "OCCT did not provide the required ${required_target} target")
  endif()
endforeach()

add_executable(
  cad_mesher_generate_fixtures
  main.cpp
  fixture_generator.cpp
  fixture_builders.cpp
  fixture_observer.cpp
  fixture_curve_recipes.cpp
  fixture_surface_recipes.cpp
  fixture_topology_recipes.cpp
  fixture_feature_recipes.cpp
  fixture_pathology_recipes.cpp
  fixture_trim_support_recipes.cpp)
target_include_directories(cad_mesher_generate_fixtures PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(cad_mesher_generate_fixtures PRIVATE TKBRep TKTopAlgo TKPrim TKBO TKGeomAlgo
                                                            TKFillet TKOffset TKDESTEP TKXCAF)
cad_mesher_apply_project_options(cad_mesher_generate_fixtures)
cad_mesher_stage_occt_runtime_dependencies(cad_mesher_generate_fixtures)
