add_executable(weft_tests test_pipeline.cpp)
target_link_libraries(weft_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_tests)
add_test(NAME pipeline COMMAND weft_tests)
weft_deploy_occt_dlls(weft_tests)

add_executable(weft_secure_core_tests test_secure_core.cpp)
target_link_libraries(weft_secure_core_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_secure_core_tests)
add_test(NAME secure_core COMMAND weft_secure_core_tests)
weft_deploy_occt_dlls(weft_secure_core_tests)

add_executable(weft_interval_solver_tests test_interval_solver.cpp)
target_link_libraries(weft_interval_solver_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_interval_solver_tests)
add_test(NAME interval_solver COMMAND weft_interval_solver_tests)
weft_deploy_occt_dlls(weft_interval_solver_tests)

add_executable(weft_canonical_boundary_tests test_canonical_boundary.cpp)
target_link_libraries(weft_canonical_boundary_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_canonical_boundary_tests)
add_test(NAME canonical_boundary COMMAND weft_canonical_boundary_tests)
weft_deploy_occt_dlls(weft_canonical_boundary_tests)

add_executable(weft_geometric_predicate_tests test_geometric_predicates.cpp)
target_link_libraries(weft_geometric_predicate_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_geometric_predicate_tests)
add_test(NAME geometric_predicates COMMAND weft_geometric_predicate_tests)
weft_deploy_occt_dlls(weft_geometric_predicate_tests)

add_executable(weft_planar_trim_validation_tests
               test_planar_trim_validation.cpp)
target_link_libraries(weft_planar_trim_validation_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_planar_trim_validation_tests)
add_test(NAME planar_trim_validation
         COMMAND weft_planar_trim_validation_tests)
weft_deploy_occt_dlls(weft_planar_trim_validation_tests)

add_executable(weft_planar_trim_assembly_tests
               test_planar_trim_assembly.cpp)
target_link_libraries(weft_planar_trim_assembly_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_planar_trim_assembly_tests)
add_test(NAME planar_trim_assembly
         COMMAND weft_planar_trim_assembly_tests)
weft_deploy_occt_dlls(weft_planar_trim_assembly_tests)

add_executable(weft_planar_cdt_tests test_planar_cdt.cpp)
target_link_libraries(weft_planar_cdt_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_planar_cdt_tests)
add_test(NAME planar_cdt COMMAND weft_planar_cdt_tests)
weft_deploy_occt_dlls(weft_planar_cdt_tests)

add_executable(weft_certified_mesh_tests test_certified_mesh.cpp)
target_link_libraries(weft_certified_mesh_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_certified_mesh_tests)
add_test(NAME certified_mesh COMMAND weft_certified_mesh_tests)
weft_deploy_occt_dlls(weft_certified_mesh_tests)

find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(
  NAME secure_fixture_catalog
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/test_secure_fixture_catalog.py
          ${CMAKE_CURRENT_SOURCE_DIR}/secure_core_fixtures)

add_executable(weft_secure_corpus_tests test_secure_corpus.cpp)
target_link_libraries(weft_secure_corpus_tests PRIVATE weft_core)
target_compile_definitions(
  weft_secure_corpus_tests
  PRIVATE WEFT_SECURE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/secure_core_fixtures")
weft_enable_strict_warnings(weft_secure_corpus_tests)
add_test(NAME secure_committed_step_corpus COMMAND weft_secure_corpus_tests)
weft_deploy_occt_dlls(weft_secure_corpus_tests)

set(WEFT_DONOR_GENERATOR_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/secure_core_fixtures/generator)
add_executable(
  weft_generated_secure_corpus_tests
  test_generated_secure_corpus.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_generator.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_builders.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_observer.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_curve_recipes.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_surface_recipes.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_topology_recipes.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_feature_recipes.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_pathology_recipes.cpp
  ${WEFT_DONOR_GENERATOR_DIR}/fixture_trim_support_recipes.cpp)
target_include_directories(weft_generated_secure_corpus_tests
                           PRIVATE ${WEFT_DONOR_GENERATOR_DIR})
target_link_libraries(weft_generated_secure_corpus_tests PRIVATE weft_core)
weft_enable_strict_warnings(weft_generated_secure_corpus_tests)
add_test(NAME secure_generated_step_corpus
         COMMAND weft_generated_secure_corpus_tests)
weft_deploy_occt_dlls(weft_generated_secure_corpus_tests)
