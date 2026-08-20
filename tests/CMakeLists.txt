add_executable(weft_tests test_pipeline.cpp)
target_link_libraries(weft_tests PRIVATE weft_core)
add_test(NAME pipeline COMMAND weft_tests)
weft_deploy_occt_dlls(weft_tests)

# Bake-queue pending dedupe (no OCCT generate — does not start the worker).
add_executable(bake_queue_tests
  test_bake_queue.cpp
  ${CMAKE_SOURCE_DIR}/app/bake_queue.cpp)
target_include_directories(bake_queue_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/app
  ${CMAKE_SOURCE_DIR}/core/include)
target_link_libraries(bake_queue_tests PRIVATE weft_core)
add_test(NAME bake_queue COMMAND bake_queue_tests)
weft_deploy_occt_dlls(bake_queue_tests)

add_executable(topology_cache_tests test_topology_cache.cpp)
target_link_libraries(topology_cache_tests PRIVATE weft_core)
add_test(NAME topology_cache COMMAND topology_cache_tests)
weft_deploy_occt_dlls(topology_cache_tests)

# Integration: emit signature twice via CLI and assert policy-equal.
add_test(
  NAME topology_signature
  COMMAND ${CMAKE_COMMAND} -E env "WEFT=$<TARGET_FILE:weft>"
          ${CMAKE_SOURCE_DIR}/tools/topology_signature.sh self-check
)
set_tests_properties(topology_signature PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
