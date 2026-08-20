# Interactive shell. Needs GLFW + OpenGL + Dear ImGui.
# ImGui (docking) is always taken from third_party/imgui (vendored — no
# FetchContent git clone that hangs Windows configure).
# GLFW prefers a system package when present; otherwise uses
# third_party/glfw. stb_image_write is also vendored.
# Only OpenGL itself is a hard system requirement.
find_package(OpenGL QUIET)
if(NOT OpenGL_FOUND)
  message(STATUS "weft_app skipped (no OpenGL)")
  return()
endif()

set(WEFT_IMGUI_DIR "${CMAKE_SOURCE_DIR}/third_party/imgui")
if(NOT EXISTS "${WEFT_IMGUI_DIR}/imgui.cpp")
  message(FATAL_ERROR
    "Vendored ImGui missing at ${WEFT_IMGUI_DIR}. "
    "Pull third_party/imgui from git (docking branch v1.90.9).")
endif()

option(WEFT_FETCH_APP_DEPS
       "Unused (kept for compatibility); GLFW/ImGui are vendored" OFF)

find_package(glfw3 3.3 QUIET)
if(glfw3_FOUND)
  message(STATUS "weft_app: system GLFW + vendored ImGui")
else()
  set(WEFT_GLFW_DIR "${CMAKE_SOURCE_DIR}/third_party/glfw")
  if(NOT EXISTS "${WEFT_GLFW_DIR}/CMakeLists.txt")
    message(FATAL_ERROR
      "Vendored GLFW missing at ${WEFT_GLFW_DIR} and no system glfw3. "
      "Pull third_party/glfw from git (3.4) or install libglfw3-dev.")
  endif()
  message(STATUS "weft_app: vendored GLFW + ImGui (no network fetch)")
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  # Avoid Wayland pkg-config hard-fail on Linux hosts without those deps;
  # X11 (or Win32/Cocoa) is enough for weft_app.
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
  add_subdirectory("${WEFT_GLFW_DIR}" "${CMAKE_BINARY_DIR}/_deps/glfw-build"
                   EXCLUDE_FROM_ALL)
endif()

add_library(weft_imgui STATIC
  ${WEFT_IMGUI_DIR}/imgui.cpp
  ${WEFT_IMGUI_DIR}/imgui_draw.cpp
  ${WEFT_IMGUI_DIR}/imgui_tables.cpp
  ${WEFT_IMGUI_DIR}/imgui_widgets.cpp
)
target_include_directories(weft_imgui PUBLIC ${WEFT_IMGUI_DIR})

set(WEFT_IMGUI_LIB weft_imgui)
set(WEFT_IMGUI_INC ${WEFT_IMGUI_DIR})
set(WEFT_BACKEND_SRC
  ${WEFT_IMGUI_DIR}/backends/imgui_impl_glfw.cpp
  ${WEFT_IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

add_executable(weft_app main.cpp bake_queue.cpp ${WEFT_BACKEND_SRC})

target_include_directories(weft_app PRIVATE
  ${WEFT_IMGUI_INC}
  ${WEFT_IMGUI_INC}/backends
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_link_libraries(weft_app PRIVATE
  weft_core
  ${WEFT_IMGUI_LIB}
  glfw
  OpenGL::GL
)

if(WIN32)
  # comdlg32: native open-file dialog. dbghelp: symbolized crash traces.
  target_link_libraries(weft_app PRIVATE comdlg32 dbghelp)
  # Taskbar/Explorer icon for the .exe (weft.ico is generated from
  # assets/weft_logo.png by tools/make_logo.py).
  target_sources(weft_app PRIVATE weft.rc)
endif()
if(MSVC)
  # PDBs even in Release so the crash handler can name functions, and an
  # 8MB stack reserve (also inherited by worker threads): OCCT meshing
  # recurses deeply and the 1MB Windows default can overflow (0xC00000FD).
  target_compile_options(weft_app PRIVATE /Zi)
  target_link_options(weft_app PRIVATE /DEBUG /STACK:8388608)
endif()
weft_deploy_occt_dlls(weft_app)
