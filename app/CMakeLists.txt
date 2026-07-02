# Interactive shell. Requires GLFW + OpenGL + Dear ImGui (Debian/Ubuntu:
# libglfw3-dev libgl1-mesa-dev libimgui-dev libstb-dev). Skipped when absent
# so the headless core/CLI always build.
find_package(glfw3 3.3 QUIET)
find_package(OpenGL QUIET)
find_library(IMGUI_LIB imgui)
find_library(STB_LIB stb)
find_path(IMGUI_INCLUDE imgui.h PATH_SUFFIXES imgui)
find_path(STB_INCLUDE stb/stb_image_write.h)

if(NOT glfw3_FOUND OR NOT OpenGL_FOUND OR NOT IMGUI_LIB OR NOT IMGUI_INCLUDE
   OR NOT STB_INCLUDE OR NOT STB_LIB)
  message(STATUS "weft_app skipped (need glfw3, OpenGL, imgui, stb)")
  return()
endif()

add_executable(weft_app
  main.cpp
  imgui_backends/imgui_impl_glfw.cpp
  imgui_backends/imgui_impl_opengl3.cpp
)

target_include_directories(weft_app PRIVATE
  ${IMGUI_INCLUDE}
  ${IMGUI_INCLUDE}/backends
  ${STB_INCLUDE}
)

target_link_libraries(weft_app PRIVATE
  weft_core
  ${IMGUI_LIB}
  ${STB_LIB}
  glfw
  OpenGL::GL
)
