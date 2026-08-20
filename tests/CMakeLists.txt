add_executable(weft_tests test_pipeline.cpp)
target_link_libraries(weft_tests PRIVATE weft_core)
add_test(NAME pipeline COMMAND weft_tests)
weft_deploy_occt_dlls(weft_tests)

# Delivery/IO unit suite: exporters, writer/reader registry, tessellation,
# edge sampling. Separate binary so it stays fast to iterate on.
add_executable(weft_io_tests test_io_export.cpp)
target_link_libraries(weft_io_tests PRIVATE weft_core)
add_test(NAME io_export COMMAND weft_io_tests)
weft_deploy_occt_dlls(weft_io_tests)

# Integration: emit signature twice via CLI and assert policy-equal.
add_test(
  NAME topology_signature
  COMMAND ${CMAKE_COMMAND} -E env "WEFT=$<TARGET_FILE:weft>"
          ${CMAKE_SOURCE_DIR}/tools/topology_signature.sh self-check
)
set_tests_properties(topology_signature PROPERTIES
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
