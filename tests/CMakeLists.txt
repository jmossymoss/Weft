add_executable(weft_tests test_pipeline.cpp)
target_link_libraries(weft_tests PRIVATE weft_core)
add_test(NAME pipeline COMMAND weft_tests)
weft_deploy_occt_dlls(weft_tests)
