add_executable(weft main.cpp)
target_link_libraries(weft PRIVATE weft_core)
weft_deploy_occt_dlls(weft)
