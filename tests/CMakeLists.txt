# accuracy-tessellator branch: smoke tests only (structured pipeline parked).
add_executable(weft_tests test_accuracy_smoke.cpp)
target_link_libraries(weft_tests PRIVATE weft_core)
add_test(NAME accuracy_smoke COMMAND weft_tests)
weft_deploy_occt_dlls(weft_tests)
