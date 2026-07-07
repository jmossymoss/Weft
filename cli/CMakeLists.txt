add_executable(weft main.cpp)
target_link_libraries(weft PRIVATE weft_core)
# main.cpp now includes the io registry headers (std::span, Mayo-faithful).
target_compile_features(weft PRIVATE cxx_std_20)
weft_deploy_occt_dlls(weft)
